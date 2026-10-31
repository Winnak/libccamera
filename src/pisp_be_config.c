/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Configuration equations and tiling conventions adapted from libpisp v1.5.0.
 * Copyright (C) 2021-2023 Raspberry Pi Ltd. See LICENSES/BSD-2-Clause.txt.
 * This C17 implementation covers Bayer16 to cropped/scaled/flipped RGB/YUV outputs.
 * It has no libpisp dependency and never writes userspace DMA addresses. */
#include <ccamera/pisp_be.h>
#include "pisp_format.h"
#include "pisp_processing.h"
#include "pisp_tiling.h"

#include <errno.h>
#include <limits.h>
#include <linux/videodev2.h>
#include <string.h>

static int output_processing(struct pisp_be_config *c, uint32_t branch, const cc_format *output,
                              const struct cc_pisp_output_layout *layout)
{
    uint32_t quantization = output->quantization;
    if (!quantization)
        quantization = V4L2_MAP_QUANTIZATION_DEFAULT(!layout->yuv, output->colorspace, output->ycbcr_enc);
    if (quantization != V4L2_QUANTIZATION_FULL_RANGE && quantization != V4L2_QUANTIZATION_LIM_RANGE)
        return -ENOTSUP;
    bool limited = quantization == V4L2_QUANTIZATION_LIM_RANGE;
    struct pisp_be_output_format_config *format = &c->output_format[branch];
    format->image = layout->native;
    format->lo = format->lo2 = limited ? 16U << 8 : 0;
    format->hi = limited ? 235U << 8 : UINT16_MAX;
    format->hi2 = (uint16_t)(limited ? (layout->yuv ? 240U : 235U) << 8 : UINT16_MAX);
    c->global.rgb_enables |= (uint32_t)PISP_BE_RGB_ENABLE_OUTPUT(branch);
    if (!layout->yuv && !limited && !layout->swap_rb) return 0;
    struct pisp_be_ccm_config *csc = &c->csc[branch];
    c->global.rgb_enables |= (uint32_t)PISP_BE_RGB_ENABLE_CSC(branch);
    if (layout->yuv) {
        uint32_t encoding = output->ycbcr_enc;
        if (!encoding) encoding = V4L2_MAP_YCBCR_ENC_DEFAULT(output->colorspace);
        /* U10 matrices from libpisp's BSD-licensed default configuration. */
        static const int16_t matrices[2][2][9] = {
            { {306, 601, 117, -173, -339, 512, 512, -429, -83},
              {263, 516, 100, -152, -298, 450, 450, -377, -73} },
            { {218, 732, 74, -117, -395, 512, 512, -465, -47},
              {187, 629, 63, -103, -347, 450, 450, -409, -41} }
        };
        if (encoding != V4L2_YCBCR_ENC_601 && encoding != V4L2_YCBCR_ENC_709) return -ENOTSUP;
        memcpy(csc->coeffs, matrices[encoding == V4L2_YCBCR_ENC_709][limited], sizeof(csc->coeffs));
        csc->offsets[0] = limited ? 16 << 18 : 0;
        csc->offsets[1] = csc->offsets[2] = 128 << 18;
        if (layout->swap_uv) {
            for (size_t i = 0; i < 3; ++i) {
                int16_t coefficient = csc->coeffs[3 + i];
                csc->coeffs[3 + i] = csc->coeffs[6 + i];
                csc->coeffs[6 + i] = coefficient;
            }
        }
    } else {
        /* RGB channel order is changed through CSC, not ORDER_SWAPPED, which
         * controls padding placement and packed YUV422 byte order. */
        int16_t gain = limited ? 879 : 1024;
        csc->coeffs[layout->swap_rb ? 2 : 0] = gain;
        csc->coeffs[4] = gain;
        csc->coeffs[layout->swap_rb ? 6 : 8] = gain;
        if (limited) csc->offsets[0] = csc->offsets[1] = csc->offsets[2] = 16 << 18;
    }
    return 0;
}

static int auxiliary_processing(struct pisp_be_config *c, const cc_pisp_be_configuration *configuration,
                                 const struct cc_pisp_raw_layout *input)
{
    const uint32_t enable[4] = {PISP_BE_BAYER_ENABLE_TDN_INPUT, PISP_BE_BAYER_ENABLE_TDN_OUTPUT,
        PISP_BE_BAYER_ENABLE_STITCH_INPUT, PISP_BE_BAYER_ENABLE_STITCH_OUTPUT};
    const uint32_t compress[4] = {PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS, PISP_BE_BAYER_ENABLE_TDN_COMPRESS,
        PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS, PISP_BE_BAYER_ENABLE_STITCH_COMPRESS};
    struct pisp_image_format_config *formats[4] = {&c->tdn_input_format,&c->tdn_output_format,
        &c->stitch_input_format,&c->stitch_output_format};
    bool tdn = (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN) != 0;
    bool stitch = (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH) != 0;
    if (tdn && (!configuration->auxiliary[1].format.fourcc ||
        (!c->tdn.reset && !configuration->auxiliary[0].format.fourcc))) return -EINVAL;
    if (stitch && !configuration->auxiliary[2].format.fourcc) return -EINVAL;
    for (uint32_t i = 0; i < 4; ++i) {
        c->dirty_flags_bayer |= enable[i] | compress[i];
        const cc_pisp_be_aux_configuration *a = &configuration->auxiliary[i];
        if (!a->format.fourcc) continue;
        struct cc_pisp_raw_layout raw;
        int ret = cc_pisp_raw_layout(&a->format, &raw);
        if (ret < 0) return ret;
        if (a->format.width != configuration->input.width || a->format.height != configuration->input.height ||
            raw.bayer_order != input->bayer_order || a->compression_offset > UINT16_MAX ||
            (!raw.compression && a->compression_offset)) return -EINVAL;
        /* Disabled readers are still validated as part of the configured memory
         * contract, but do not consume a buffer for this job. */
        if ((!i && (!tdn || c->tdn.reset)) || (i == 2 && !stitch)) continue;
        *formats[i] = raw.native;
        c->global.bayer_enables |= enable[i];
        if (raw.compression) c->global.bayer_enables |= compress[i];
        struct pisp_compress_config compression = {.offset=(uint16_t)a->compression_offset,.mode=(uint8_t)raw.compression};
        struct pisp_decompress_config decompression = {.offset=(uint16_t)a->compression_offset,.mode=(uint8_t)raw.compression};
        if (i == 0) c->tdn_decompress = decompression;
        else if (i == 1) c->tdn_compress = compression;
        else if (i == 2) c->stitch_decompress = decompression;
        else c->stitch_compress = compression;
    }
    return 0;
}

int cc_pisp_be_build_configuration(const cc_pisp_be_configuration *configuration,
                                   const struct pisp_be_config *tuning,
                                   struct pisp_be_tiles_config *parameters)
{
    if (!configuration || !parameters) return -EINVAL;
    if (configuration->hardware_version && configuration->hardware_version != CC_PISP_BE_VERSION_BCM2712_C0 &&
        configuration->hardware_version != CC_PISP_BE_VERSION_BCM2712_D0) return -ENOTSUP;
    const cc_format *input = &configuration->input;
    const cc_format *outputs[2] = { &configuration->output, &configuration->output1 };
    uint32_t branches = configuration->output1.fourcc ? 2U : 1U;
    struct cc_pisp_raw_layout raw;
    int ret = cc_pisp_raw_layout(input, &raw);
    if (ret < 0) return ret;
    if (configuration->compression_offset > UINT16_MAX ||
        (!raw.compression && configuration->compression_offset)) return -EINVAL;
    struct cc_pisp_output_layout layouts[2];
    cc_rectangle crops[2] = {0};
    for (uint32_t i = 0; i < branches; ++i) {
        ret = cc_pisp_crop(input, &configuration->crop[i], &crops[i]);
        if (ret < 0) return ret;
        if (configuration->transform[i] & ~3U) return -EINVAL;
        ret = cc_pisp_output_layout(outputs[i], &layouts[i]);
        if (ret < 0) return ret;
        /* The C0 silicon does not implement padded 32-bit RGB output.
         * Unknown revisions follow the conservative C0 feature set. */
        if ((layouts[i].native.format & PISP_IMAGE_FORMAT_BPP_32) &&
            configuration->hardware_version != CC_PISP_BE_VERSION_BCM2712_D0) return -ENOTSUP;
    }
    struct pisp_be_tiles_config p = {0};
    struct pisp_be_config *c = &p.config;
    ret = cc_pisp_processing(c, tuning, input->width, input->height, configuration->hardware_version,
                            raw.bayer_order == PISP_BAYER_ORDER_GREYSCALE);
    if (ret < 0)
        return ret;
    if (raw.bayer_order != PISP_BAYER_ORDER_GREYSCALE) c->global.bayer_order = (uint8_t)raw.bayer_order;
    c->input_format = raw.native;
    c->dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DECOMPRESS;
    if (raw.compression) {
        c->global.bayer_enables |= PISP_BE_BAYER_ENABLE_DECOMPRESS;
        c->decompress = (struct pisp_decompress_config) {
            .offset = (uint16_t)configuration->compression_offset, .mode = (uint8_t)raw.compression};
    }
    ret = auxiliary_processing(c, configuration, &raw);
    if (ret < 0) return ret;
    for (uint32_t i = 0; i < branches; ++i) {
        ret = output_processing(c, i, outputs[i], &layouts[i]);
        if (ret < 0) return ret;
        c->output_format[i].transform = (uint8_t)configuration->transform[i];
    }
    ret = cc_pisp_resize(c, crops, tuning, branches);
    if (ret < 0) return ret;
    c->dirty_flags_rgb |= PISP_BE_RGB_ENABLE_OUTPUT1 | PISP_BE_RGB_ENABLE_CSC1;
    ret = cc_pisp_tiles(&p, crops, layouts, branches);
    if (ret < 0) return ret;
    *parameters = p;
    return 0;
}

int cc_pisp_be_build_parameters(const cc_format *input, const cc_format *output,
                                const struct pisp_be_config *tuning,
                                struct pisp_be_tiles_config *parameters)
{
    if (!input || !output) return -EINVAL;
    cc_pisp_be_configuration configuration = { .input = *input, .output = *output };
    return cc_pisp_be_build_configuration(&configuration, tuning, parameters);
}
