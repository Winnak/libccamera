/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "internal.h"
#include "pisp_format.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    const uint32_t raw[] = {V4L2_PIX_FMT_SRGGB8, V4L2_PIX_FMT_SBGGR10P,
        V4L2_PIX_FMT_SGBRG12P, V4L2_PIX_FMT_SGRBG10, V4L2_PIX_FMT_GREY,
        V4L2_PIX_FMT_Y10, V4L2_PIX_FMT_Y12};
    uint32_t w = 64U + 2U * (fuzz_u16(&in) % 8161U);
    uint32_t h = 64U + 2U * (fuzz_u16(&in) % 8161U);
    cc_cfe_configuration c = {.sensor = {.fourcc = raw[fuzz_u8(&in) % 7], .width = w, .height = h}};
    c.image = (cc_format){.fourcc = cc_cfe_output_fourcc(c.sensor.fourcc),
        .width = w, .height = h, .num_planes = 1};
    c.image.planes[0].stride = (2U * w + 15U) & ~15U;
    c.image.planes[0].size = c.image.planes[0].stride * h;
    if (mode & 4U) {
        struct cc_pisp_raw_layout raw_layout;
        FUZZ_CHECK(cc_pisp_raw_format(c.image.fourcc,&raw_layout)==0);
        const char orders[]={'R','g','B','G'};
        char order=raw_layout.bayer_order==PISP_BAYER_ORDER_GREYSCALE ? 'M' : orders[raw_layout.bayer_order];
        c.image.fourcc=CC_FOURCC('P','C',mode & 32U ? '3' : mode & 8U ? '2' : '1',order);
        c.image.planes[0].stride=(w+15U)&~15U;
        c.image.planes[0].size=c.image.planes[0].stride*h;
        c.compression_offset=fuzz_u16(&in);
    }
    if (mode & 16U) {
        uint32_t cw=(w/2U)&~1U, ch=(h/2U)&~1U;
        c.crop[0]=(cc_rectangle){0,0,cw,ch}; c.crop[1]=c.crop[0];
        c.downscale[0]=(struct pisp_fe_downscale_config){.xin=2,.xout=1,.yin=2,.yout=1};
        c.downscale[1]=(struct pisp_fe_downscale_config){.xin=3,.xout=2,.yin=3,.yout=2};
        c.image1=c.image; c.compression_offset1=c.compression_offset;
        cc_format *images[2]={&c.image,&c.image1};
        for (uint32_t i=0;i<2;++i) {
            images[i]->width=2U*(cw/2U*c.downscale[i].xout/c.downscale[i].xin);
            images[i]->height=2U*(ch/2U*c.downscale[i].yout/c.downscale[i].yin);
        }
    }
    if (mode & 1U) fuzz_patch(&in, &c, sizeof(c));
    struct pisp_fe_config tuning = {0};
    if (mode & 2U) {
        tuning.global.enables = fuzz_u32(&in);
        fuzz_patch(&in, &tuning, sizeof(tuning));
    }
    struct pisp_fe_config p, before;
    memset(&p, 0xa5, sizeof(p)); memcpy(&before, &p, sizeof(p));
    int ret = cc_cfe_build_parameters(&c, mode & 2U ? &tuning : NULL, &p);
    FUZZ_CHECK(ret <= 0);
    if (ret) FUZZ_CHECK(!memcmp(&p, &before, sizeof(p)));
    else {
        FUZZ_CHECK(p.input.format.width == c.sensor.width && p.input.format.height == c.sensor.height);
        bool compressed=PISP_IMAGE_FORMAT_COMPRESSED(p.ch[0].output.format.format);
        FUZZ_CHECK(p.ch[0].output.format.stride == (int32_t)c.image.planes[0].stride);
        if (!compressed) FUZZ_CHECK((p.ch[0].output.format.format & ~((uint32_t)PISP_IMAGE_FORMAT_SHIFT_MASK)) == PISP_IMAGE_FORMAT_BPS_16);
        else {
            FUZZ_CHECK(p.input.format.format==PISP_IMAGE_FORMAT_BPS_16 && (p.global.enables & PISP_FE_ENABLE_COMPRESS0));
            FUZZ_CHECK(p.ch[0].compress.offset==c.compression_offset && !p.ch[0].compress.pad);
            FUZZ_CHECK(p.ch[0].compress.mode==((c.image.fourcc>>16)&255U)-'0');
        }
        FUZZ_CHECK((p.global.enables & (PISP_FE_ENABLE_INPUT | PISP_FE_ENABLE_OUTPUT0)) ==
                   (PISP_FE_ENABLE_INPUT | PISP_FE_ENABLE_OUTPUT0));
        FUZZ_CHECK(!p.input_buffer.addr_lo && !p.input_buffer.addr_hi);
        for (size_t i = 0; i < PISP_FE_NUM_OUTPUTS; ++i)
            FUZZ_CHECK(!p.output_buffer[i].addr_lo && !p.output_buffer[i].addr_hi);
        FUZZ_CHECK(!p.stats_buffer.addr_lo && !p.stats_buffer.addr_hi);
    }
    return 0;
}
