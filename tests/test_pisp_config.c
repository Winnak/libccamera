/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/pisp_be.h>
#include <ccamera/cfe.h>

#include <errno.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "pisp_reference_hash.h"
#include "pisp_reference_vectors.h"
#include "pisp_format_reference_vectors.h"
#include "pisp_compressed_reference_vectors.h"

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x, e) CHECK((x) == -(e))

static cc_format image(uint32_t width, uint32_t height, uint32_t fourcc, uint32_t bytes_per_pixel)
{
    uint32_t stride = (width * bytes_per_pixel + 15) & ~15U;
    return (cc_format) { .width = width, .height = height, .fourcc = fourcc,
        .num_planes = 1, .planes = { {stride, stride * height} } };
}

/* Area + disjointness + containment proves complete output coverage without
 * reproducing the partition algorithm or allocating a frame-sized bitmap. */
static void verify_tiles(const cc_format *input, const cc_format *output,
                          const struct pisp_be_tiles_config *p)
{
    CHECK(p->num_tiles && p->num_tiles <= PISP_BACK_END_NUM_TILES);
    CHECK(p->config.global.bayer_enables & PISP_BE_BAYER_ENABLE_DEMOSAIC);
    CHECK(p->config.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT0);
    CHECK(!(p->config.global.rgb_enables & (PISP_BE_RGB_ENABLE_INPUT | PISP_BE_RGB_ENABLE_OUTPUT1 |
                                          PISP_BE_RGB_ENABLE_RESAMPLE0 | PISP_BE_RGB_ENABLE_DOWNSCALE0)));
    uint64_t area = 0;
    for (uint32_t i = 0; i < p->num_tiles; ++i) {
        const struct pisp_tile *t = &p->tiles[i];
        uint32_t x = t->output_offset_x[0], y = t->output_offset_y[0];
        uint32_t w = t->output_width[0], h = t->output_height[0];
        CHECK(w && h >= 16 && x + w <= output->width && y + h <= output->height);
        CHECK(t->input_width >= 16 && t->input_width <= 640);
        CHECK(t->input_height >= 16 && t->input_height <= 3072);
        CHECK((uint32_t)t->input_offset_x + t->input_width <= input->width);
        CHECK((uint32_t)t->input_offset_y + t->input_height <= input->height);
        CHECK(!((t->input_offset_x | t->input_offset_y | t->input_width | t->input_height) & 1U));
        CHECK(t->input_offset_x + t->crop_x_start[0] == x);
        CHECK(t->input_offset_y + t->crop_y_start[0] == y);
        CHECK(w + t->crop_x_start[0] + t->crop_x_end[0] == t->input_width);
        CHECK(h + t->crop_y_start[0] + t->crop_y_end[0] == t->input_height);
        CHECK(t->resample_in_width[0] == w && t->resample_in_height[0] == h);
        CHECK(t->input_addr_offset == t->input_offset_y * input->planes[0].stride + 2U * t->input_offset_x);
        CHECK(t->output_addr_offset[0] == y * output->planes[0].stride + 3U * x);
        CHECK(t->input_addr_offset % 4 == 0 && t->output_addr_offset[0] % 16 == 0);
        CHECK((uint64_t)t->input_addr_offset + (t->input_height - 1U) * input->planes[0].stride +
              2U * t->input_width <= input->planes[0].size);
        CHECK((uint64_t)t->output_addr_offset[0] + (h - 1U) * output->planes[0].stride +
              3U * w <= output->planes[0].size);
        CHECK(!!(t->edge & PISP_LEFT_EDGE) == (x == 0));
        CHECK(!!(t->edge & PISP_TOP_EDGE) == (y == 0));
        CHECK(!!(t->edge & PISP_RIGHT_EDGE) == (x + w == output->width));
        CHECK(!!(t->edge & PISP_BOTTOM_EDGE) == (y + h == output->height));
        if (x) CHECK(t->crop_x_start[0] >= 16);
        if (y) CHECK(t->crop_y_start[0] >= 16);
        if (x + w < output->width) CHECK(t->crop_x_end[0] >= 16);
        if (y + h < output->height) CHECK(t->crop_y_end[0] >= 16);
        CHECK(!t->output_width[1] && !t->output_height[1]);
        for (uint32_t j = 0; j < i; ++j) {
            const struct pisp_tile *other = &p->tiles[j];
            CHECK(x >= (uint32_t)other->output_offset_x[0] + other->output_width[0] ||
                  x + w <= other->output_offset_x[0] ||
                  y >= (uint32_t)other->output_offset_y[0] + other->output_height[0] ||
                  y + h <= other->output_offset_y[0]);
        }
        area += (uint64_t)w * h;
    }
    CHECK(area == (uint64_t)output->width * output->height);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 2; ++j) {
            CHECK(!p->config.input_buffer.addr[i][j]);
            CHECK(!p->config.output_buffer[0].addr[i][j]);
        }
}

static void test_coverage(void)
{
    const uint32_t heights[] = {16, 18, 32, 1232, 3038, 3040, 3042, 3054, 3056, 3070, 3072, 3074, 6096};
    const uint32_t raw[] = { V4L2_PIX_FMT_SRGGB16, V4L2_PIX_FMT_SGBRG16, V4L2_PIX_FMT_SBGGR16, V4L2_PIX_FMT_SGRBG16 };
    for (uint32_t width = 16; width <= 1700; width += 2) {
        for (size_t i = 0; i < sizeof(heights) / sizeof(heights[0]); ++i) {
            cc_format input = image(width, heights[i], raw[i % 4], 2);
            cc_format output = image(width, heights[i], i % 2 ? V4L2_PIX_FMT_RGB24 : V4L2_PIX_FMT_BGR24, 3);
            struct pisp_be_tiles_config p;
            OK(cc_pisp_be_build_parameters(&input, &output, NULL, &p));
            CHECK(p.config.global.bayer_order == i % 4);
            CHECK(p.config.demosaic.sharper == 8 && p.config.demosaic.fc_mode == 1);
            verify_tiles(&input, &output, &p);
            if (i % 2) {
                CHECK(!(p.config.global.rgb_enables & PISP_BE_RGB_ENABLE_CSC0));
            } else {
                /* The native matrix maps an RGB red sample to the BGR red
                 * byte, green to green and blue to the BGR blue byte. */
                CHECK(p.config.global.rgb_enables & PISP_BE_RGB_ENABLE_CSC0);
                const int expected[] = {0, 0, 1024, 0, 1024, 0, 1024, 0, 0};
                for (size_t j = 0; j < 9; ++j) CHECK(p.config.csc[0].coeffs[j] == expected[j]);
            }
        }
    }
    cc_format input = image(4608, 2592, V4L2_PIX_FMT_SRGGB16, 2);
    cc_format output = image(4608, 2592, V4L2_PIX_FMT_RGB24, 3);
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_parameters(&input, &output, NULL, &p));
    verify_tiles(&input, &output, &p);
    input = image(12000, 12000, V4L2_PIX_FMT_SRGGB16, 2);
    output = image(12000, 12000, V4L2_PIX_FMT_RGB24, 3);
    CHECK(cc_pisp_be_build_parameters(&input, &output, NULL, &p) == -E2BIG);
    input = image(65534, 16, V4L2_PIX_FMT_SRGGB16, 2);
    output = image(65534, 16, V4L2_PIX_FMT_RGB24, 3);
    CHECK(cc_pisp_be_build_parameters(&input, &output, NULL, &p) == -E2BIG);
}

static void test_tuning(void)
{
    cc_format input = image(1640, 1232, V4L2_PIX_FMT_SRGGB16, 2);
    cc_format output = image(1640, 1232, V4L2_PIX_FMT_RGB24, 3);
    struct pisp_be_config tuning = {0};
    tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_LSC | PISP_BE_BAYER_ENABLE_WBG | PISP_BE_BAYER_ENABLE_BLC;
    tuning.global.rgb_enables = PISP_BE_RGB_ENABLE_CCM;
    tuning.wbg.gain_r = 8192; tuning.wbg.gain_g = 4096; tuning.wbg.gain_b = 6144;
    tuning.blc.black_level_r = tuning.blc.black_level_gr = tuning.blc.black_level_gb = tuning.blc.black_level_b = 4096;
    tuning.ccm.coeffs[0] = tuning.ccm.coeffs[4] = tuning.ccm.coeffs[8] = 1024;
    tuning.input_buffer.addr[0][0] = tuning.output_buffer[0].addr[0][0] = UINT32_MAX;
    memset(tuning.blc.pad, 0xff, sizeof(tuning.blc.pad));
    memset(tuning.wbg.pad, 0xff, sizeof(tuning.wbg.pad));
    memset(tuning.ccm.pad, 0xff, sizeof(tuning.ccm.pad));
    for (size_t y = 0; y < PISP_BE_LSC_LUT_SIZE; ++y)
        for (size_t x = 0; x < PISP_BE_LSC_LUT_SIZE; ++x)
            tuning.lsc.lut_packed[y][x] = 0x20400200;
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_parameters(&input, &output, &tuning, &p));
    verify_tiles(&input, &output, &p);
    CHECK(p.config.wbg.gain_r == 8192 && p.config.wbg.gain_b == 6144);
    CHECK(p.config.blc.black_level_gr == 4096 && p.config.ccm.coeffs[8] == 1024);
    CHECK(!p.config.blc.pad[0] && !p.config.wbg.pad[0] && !p.config.ccm.pad[0]);
    CHECK(p.config.lsc.grid_step_x == 5115 && p.config.lsc.grid_step_y == 6808);
    for (uint32_t i = 0; i < p.num_tiles; ++i) {
        const struct pisp_tile *t = &p.tiles[i];
        CHECK(t->lsc_grid_offset_x == t->input_offset_x * 5115U);
        CHECK(t->lsc_grid_offset_y == t->input_offset_y * 6808U);
    }
    tuning.lsc.grid_step_x = 5000; tuning.lsc.grid_step_y = 6000;
    tuning.lsc_extra.offset_x = 10; tuning.lsc_extra.offset_y = 20;
    OK(cc_pisp_be_build_parameters(&input, &output, &tuning, &p));
    CHECK(p.tiles[0].lsc_grid_offset_x == 50000 && p.tiles[0].lsc_grid_offset_y == 120000);
}

static void test_invalid(void)
{
    for (int invalid = 0; invalid < 17; ++invalid) {
        cc_format input = image(640, 480, V4L2_PIX_FMT_SRGGB16, 2);
        cc_format output = image(640, 480, V4L2_PIX_FMT_RGB24, 3);
        struct pisp_be_config tuning = {0};
        switch (invalid) {
        case 0: input.fourcc = V4L2_PIX_FMT_SRGGB10P; break;
        case 1: output.fourcc = V4L2_PIX_FMT_NV12; break;
        case 2: output.width = 0; break;
        case 3: input.width = output.width = 641; break;
        case 4: input.num_planes = 2; break;
        case 5: output.planes[0].stride = 1900; break;
        case 6: input.planes[0].size = 1; break;
        case 7: tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_TDN; break;
        case 8: tuning.global.rgb_enables = PISP_BE_RGB_ENABLE_SHARPEN; break;
        case 9: tuning.demosaic.fc_mode = 3; break;
        case 10: tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_LSC; tuning.lsc.grid_step_x = UINT16_MAX; break;
        case 11: tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_DPC; tuning.dpc.flags = 2; break;
        case 12: tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_GEQ; tuning.geq.slope_sharper = 1024; break;
        case 13: tuning.global.rgb_enables = PISP_BE_RGB_ENABLE_CCM; tuning.ccm.coeffs[0] = 8192; break;
        case 14: tuning.global.rgb_enables = PISP_BE_RGB_ENABLE_GAMMA; tuning.gamma.lut[0] = UINT32_MAX; break;
        case 15: input.width = output.width = 8; break;
        case 16: output.planes[0].size -= 1; break;
        }
        struct pisp_be_tiles_config p;
        memset(&p, 0xa5, sizeof(p));
        struct pisp_be_tiles_config original = p;
        CHECK(cc_pisp_be_build_parameters(&input, &output, &tuning, &p) < 0);
        CHECK(!memcmp(&p, &original, sizeof(p)));
    }
    CHECK(cc_pisp_be_build_parameters(NULL, NULL, NULL, NULL) == -EINVAL);
}

struct output_kind {
    uint32_t fourcc, planes, components, pixel_bytes, xsub, ysub;
    bool swap_uv, y_second;
};

static cc_format output_image(const struct output_kind *kind, uint32_t width, uint32_t height)
{
    cc_format f = { .width = width, .height = height, .fourcc = kind->fourcc, .num_planes = kind->planes };
    uint32_t stride = (width * kind->pixel_bytes + 127) & ~127U;
    f.planes[0] = (cc_plane_format) { stride, stride * height };
    if (kind->planes > 1) {
        /* Deliberately use chroma strides unrelated to the Y stride. */
        uint32_t chroma = ((width / kind->xsub + 63) & ~63U) + 64;
        for (uint32_t i = 1; i < kind->planes; ++i)
            f.planes[i] = (cc_plane_format) { chroma, chroma * (height / kind->ysub) };
    } else if (kind->components > 1) {
        f.planes[0].size += (kind->components - 1) * (stride / kind->xsub) * (height / kind->ysub);
    }
    return f;
}

static void verify_yuv_tiles(const cc_format *input, const cc_format *output, const struct output_kind *kind,
                             const struct pisp_be_tiles_config *p, uint32_t branch)
{
    const struct pisp_image_format_config *native = &p->config.output_format[branch].image;
    uint32_t chroma_stride = kind->planes > 1 ? output->planes[1].stride : output->planes[0].stride / kind->xsub;
    CHECK(p->config.global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_OUTPUT(branch));
    CHECK(((native->format & PISP_IMAGE_FORMAT_ORDER_SWAPPED) != 0) == kind->y_second);
    if (kind->components > 1) CHECK(native->stride2 == (int32_t)chroma_stride);
    uint64_t area = 0;
    for (uint32_t i = 0; i < p->num_tiles; ++i) {
        const struct pisp_tile *t = &p->tiles[i];
        uint32_t x = t->output_offset_x[branch], y = t->output_offset_y[branch];
        uint32_t w = t->output_width[branch], h = t->output_height[branch];
        CHECK(w && h >= 16 && x + w <= output->width && y + h <= output->height);
        CHECK(t->input_width <= 640 && t->input_height <= 3072);
        CHECK(t->input_offset_x + t->crop_x_start[branch] == x && t->input_offset_y + t->crop_y_start[branch] == y);
        CHECK(w + t->crop_x_start[branch] + t->crop_x_end[branch] == t->input_width);
        CHECK(h + t->crop_y_start[branch] + t->crop_y_end[branch] == t->input_height);
        CHECK((uint32_t)t->input_offset_x + t->input_width <= input->width);
        CHECK((uint32_t)t->input_offset_y + t->input_height <= input->height);
        if (x) CHECK(t->crop_x_start[branch] >= 16);
        if (y) CHECK(t->crop_y_start[branch] >= 16);
        if (x + w < output->width) CHECK(t->crop_x_end[branch] >= 16);
        if (y + h < output->height) CHECK(t->crop_y_end[branch] >= 16);
        uint64_t component_base = 0;
        for (uint32_t component = 0; component < kind->components; ++component) {
            uint32_t xs = component ? kind->xsub : 1, ys = component ? kind->ysub : 1;
            uint32_t bytes = component ? 1 : kind->pixel_bytes;
            uint32_t stride = component ? chroma_stride : output->planes[0].stride;
            uint32_t offset = component ? t->output_addr_offset2[branch] : t->output_addr_offset[branch];
            CHECK(offset == y / ys * stride + x / xs * bytes && offset % 16 == 0);
            CHECK(x % xs == 0 && y % ys == 0 && w % xs == 0 && h % ys == 0);
            uint32_t memory_plane = kind->planes > 1 ? component : 0;
            uint64_t end = offset + (uint64_t)(h / ys - 1U) * stride + w / xs * bytes;
            if (kind->planes == 1) end += component_base;
            CHECK(end <= output->planes[memory_plane].size);
            component_base += (uint64_t)stride * (output->height / ys);
        }
        for (uint32_t j = 0; j < i; ++j) {
            const struct pisp_tile *q = &p->tiles[j];
            CHECK(x + w <= q->output_offset_x[branch] || x >= (uint32_t)q->output_offset_x[branch] + q->output_width[branch] ||
                  y + h <= q->output_offset_y[branch] || y >= (uint32_t)q->output_offset_y[branch] + q->output_height[branch]);
        }
        area += (uint64_t)w * h;
    }
    CHECK(area == (uint64_t)output->width * output->height);
}

static int csc_byte(const struct pisp_be_config *c, uint32_t branch, uint32_t channel, int r, int g, int b)
{
    const struct pisp_be_ccm_config *matrix = &c->csc[branch];
    int64_t value = (int64_t)matrix->coeffs[3 * channel] * r + (int64_t)matrix->coeffs[3 * channel + 1] * g +
                    (int64_t)matrix->coeffs[3 * channel + 2] * b + matrix->offsets[channel];
    value /= 1024;
    int lo = channel ? c->output_format[branch].lo2 : c->output_format[branch].lo;
    int hi = channel ? c->output_format[branch].hi2 : c->output_format[branch].hi;
    if (value < lo) value = lo;
    if (value > hi) value = hi;
    return (int)(value / 256);
}

static void test_yuv_and_outputs(void)
{
    const struct output_kind kinds[] = {
        {V4L2_PIX_FMT_YUYV, 1, 1, 2, 1, 1, false, false},
        {V4L2_PIX_FMT_UYVY, 1, 1, 2, 1, 1, false, true},
        {V4L2_PIX_FMT_YVYU, 1, 1, 2, 1, 1, true, false},
        {V4L2_PIX_FMT_VYUY, 1, 1, 2, 1, 1, true, true},
        {V4L2_PIX_FMT_NV12, 1, 2, 1, 1, 2, false, false},
        {V4L2_PIX_FMT_NV21, 1, 2, 1, 1, 2, true, false},
        {V4L2_PIX_FMT_NV12M, 2, 2, 1, 1, 2, false, false},
        {V4L2_PIX_FMT_NV21M, 2, 2, 1, 1, 2, true, false},
        {V4L2_PIX_FMT_YUV420, 1, 3, 1, 2, 2, false, false},
        {V4L2_PIX_FMT_YVU420, 1, 3, 1, 2, 2, true, false},
        {V4L2_PIX_FMT_YUV420M, 3, 3, 1, 2, 2, false, false},
        {V4L2_PIX_FMT_YVU420M, 3, 3, 1, 2, 2, true, false},
        {V4L2_PIX_FMT_YUV422P, 1, 3, 1, 2, 1, false, false},
        {V4L2_PIX_FMT_YUV422M, 3, 3, 1, 2, 1, false, false},
        {V4L2_PIX_FMT_YVU422M, 3, 3, 1, 2, 1, true, false},
        {V4L2_PIX_FMT_YUV444M, 3, 3, 1, 1, 1, false, false},
        {V4L2_PIX_FMT_YVU444M, 3, 3, 1, 1, 1, true, false},
        {V4L2_PIX_FMT_NV16, 1, 2, 1, 1, 1, false, false},
        {V4L2_PIX_FMT_NV61, 1, 2, 1, 1, 1, true, false},
        {V4L2_PIX_FMT_NV16M, 2, 2, 1, 1, 1, false, false},
        {V4L2_PIX_FMT_NV61M, 2, 2, 1, 1, 1, true, false}
    };
    const struct output_kind bgr = {V4L2_PIX_FMT_BGR24, 1, 1, 3, 1, 1, false, false};
    for (size_t k = 0; k < sizeof(kinds) / sizeof(kinds[0]); ++k) {
        for (int mode = 0; mode < 4; ++mode) {
            const struct output_kind *auxiliary = mode ? &kinds[(k + 1) % (sizeof(kinds) / sizeof(kinds[0]))] : &bgr;
            cc_format input = image(1640, 3074, V4L2_PIX_FMT_SRGGB16, 2);
            cc_pisp_be_configuration configuration = { .input = input,
                .output = output_image(&kinds[k], 1640, 3074),
                .output1 = output_image(auxiliary, 1640, 3074) };
            configuration.output.colorspace = V4L2_COLORSPACE_SMPTE170M;
            configuration.output.ycbcr_enc = mode & 1 ? V4L2_YCBCR_ENC_709 : V4L2_YCBCR_ENC_601;
            configuration.output.quantization = mode & 2 ? V4L2_QUANTIZATION_LIM_RANGE : V4L2_QUANTIZATION_FULL_RANGE;
            configuration.output1.colorspace = V4L2_COLORSPACE_JPEG; /* Default 601/full, independently of output0. */
            struct pisp_be_tiles_config p;
            OK(cc_pisp_be_build_configuration(&configuration, NULL, &p));
            verify_yuv_tiles(&input, &configuration.output, &kinds[k], &p, 0);
            verify_yuv_tiles(&input, &configuration.output1, auxiliary, &p, 1);
            CHECK(csc_byte(&p.config, 0, 0, 0, 0, 0) == (mode & 2 ? 16 : 0));
            CHECK(csc_byte(&p.config, 0, 0, 65535, 65535, 65535) == (mode & 2 ? 235 : 255));
            CHECK(csc_byte(&p.config, 0, 1, 65535, 65535, 65535) == 128);
            CHECK(csc_byte(&p.config, 0, 2, 65535, 65535, 65535) == 128);
            int red_y[4] = {76, 54, 81, 62};
            CHECK(csc_byte(&p.config, 0, 0, 65535, 0, 0) == red_y[mode]);
            uint32_t u = kinds[k].swap_uv ? 2U : 1U, v = kinds[k].swap_uv ? 1U : 2U;
            CHECK(csc_byte(&p.config, 0, u, 65535, 0, 0) < 128);
            CHECK(csc_byte(&p.config, 0, v, 65535, 0, 0) == (mode & 2 ? 240 : 255));
            CHECK(csc_byte(&p.config, 1, 0, 0, 0, 0) == 0 && csc_byte(&p.config, 1, 0, 65535, 65535, 65535) == 255);
        }
    }
    const struct output_kind *planar = &kinds[10];
    cc_format input = image(640, 480, V4L2_PIX_FMT_SRGGB16, 2);
    cc_pisp_be_configuration c = { .input = input, .output = output_image(planar, 640, 480) };
    struct pisp_be_tiles_config p;
    cc_format limited_bgr = image(640, 480, V4L2_PIX_FMT_BGR24, 3);
    limited_bgr.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    OK(cc_pisp_be_build_parameters(&input, &limited_bgr, NULL, &p));
    CHECK(csc_byte(&p.config, 0, 0, 65535, 0, 0) == 16);
    CHECK(csc_byte(&p.config, 0, 2, 65535, 0, 0) == 235);
    for (int bad = 0; bad < 6; ++bad) {
        cc_pisp_be_configuration invalid = c;
        if (bad == 0) invalid.output.planes[2].stride += 16;
        if (bad == 1) invalid.output.planes[1].size = 1;
        if (bad == 2) invalid.output.ycbcr_enc = V4L2_YCBCR_ENC_BT2020;
        if (bad == 3) invalid.output.quantization = UINT32_MAX;
        if (bad == 4) invalid.output1 = image(8, 8, V4L2_PIX_FMT_RGB24, 3);
        if (bad == 5) invalid.output.num_planes = 1;
        memset(&p, 0xa5, sizeof(p));
        struct pisp_be_tiles_config original = p;
        CHECK(cc_pisp_be_build_configuration(&invalid, NULL, &p) < 0);
        CHECK(!memcmp(&p, &original, sizeof(p)));
    }
}

static void verify_scaled(const cc_pisp_be_configuration *configuration, const struct pisp_be_tiles_config *p,
                           uint32_t branch, const struct output_kind *kind)
{
    const cc_format *output = branch ? &configuration->output1 : &configuration->output;
    cc_rectangle crop = configuration->crop[branch];
    if (!crop.width) crop = (cc_rectangle) {0, 0, configuration->input.width, configuration->input.height};
    uint32_t transform = configuration->transform[branch];
    bool downscale = (p->config.global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_DOWNSCALE(branch)) != 0;
    bool resample = (p->config.global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_RESAMPLE(branch)) != 0;
    uint32_t chroma_stride = kind->planes > 1 ? output->planes[1].stride : output->planes[0].stride / kind->xsub;
    uint64_t area = 0;
    CHECK(p->num_tiles && p->num_tiles <= 64 && !(p->config.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE0));
    for (uint32_t i = 0; i < p->num_tiles; ++i) {
        const struct pisp_tile *t = &p->tiles[i];
        CHECK(t->input_width >= 16 && t->input_width <= 640 && t->input_height >= 16 && t->input_height <= 3072);
        CHECK(!((t->input_offset_x | t->input_offset_y | t->input_width | t->input_height) & 1U));
        CHECK((uint32_t)t->input_offset_x + t->input_width <= configuration->input.width);
        CHECK((uint32_t)t->input_offset_y + t->input_height <= configuration->input.height);
        uint32_t w = t->output_width[branch], h = t->output_height[branch];
        if (!w || !h) {
            CHECK(!w && !h && t->crop_x_start[branch] == t->input_width && t->crop_y_start[branch] == t->input_height);
            continue;
        }
        uint32_t x = t->output_offset_x[branch], last_y = t->output_offset_y[branch];
        CHECK(!(transform & PISP_BE_TRANSFORM_VFLIP) || last_y + 1 >= h);
        uint32_t top_y = transform & PISP_BE_TRANSFORM_VFLIP ? last_y + 1 - h : last_y;
        CHECK(x + w <= output->width && top_y + h <= output->height);
        uint32_t logical_x = transform & PISP_BE_TRANSFORM_HFLIP ? output->width - x - w : x;
        uint32_t logical_y = transform & PISP_BE_TRANSFORM_VFLIP ? output->height - last_y - 1U : last_y;
        for (uint32_t axis = 0; axis < 2; ++axis) {
            uint32_t start = axis ? logical_y : logical_x, size = axis ? h : w;
            uint64_t first = start, last = start + size - 1U;
            uint32_t rf = axis ? p->config.resample[branch].scale_factor_v : p->config.resample[branch].scale_factor_h;
            uint32_t df = axis ? p->config.downscale[branch].scale_factor_v : p->config.downscale[branch].scale_factor_h;
            if (resample) { first = first * rf / 4096U; last = last * rf / 4096U; }
            if (downscale) { first = first * df / 4096U; last = last * df / 4096U; }
            uint32_t crop_offset = (uint32_t)(axis ? crop.y : crop.x);
            first += crop_offset; last += crop_offset;
            uint32_t raw_start = (axis ? t->input_offset_y : t->input_offset_x) +
                                 (uint32_t)(axis ? t->crop_y_start[branch] : t->crop_x_start[branch]);
            uint32_t raw_end = (axis ? t->input_offset_y + t->input_height : t->input_offset_x + t->input_width) -
                               (uint32_t)(axis ? t->crop_y_end[branch] : t->crop_x_end[branch]);
            CHECK(first >= raw_start && last < raw_end);
            if (resample) {
                uint16_t expected = (uint16_t)((((uint64_t)start * 16U * rf / 4096U) % 16U) * 256U);
                for (uint32_t plane = 0; plane < 3; ++plane)
                    CHECK((axis ? t->resample_phase_y[plane * 2 + branch] : t->resample_phase_x[plane * 2 + branch]) == expected);
            }
        }
        uint64_t base = 0;
        for (uint32_t component = 0; component < kind->components; ++component) {
            uint32_t xs = component ? kind->xsub : 1, ys = component ? kind->ysub : 1;
            uint32_t bytes = component ? 1 : kind->pixel_bytes;
            uint32_t stride = component ? chroma_stride : output->planes[0].stride;
            uint32_t offset = component ? t->output_addr_offset2[branch] : t->output_addr_offset[branch];
            CHECK(offset == last_y / ys * stride + x / xs * bytes && offset % 16 == 0);
            CHECK(x % xs == 0 && top_y % ys == 0 && w % xs == 0 && h % ys == 0);
            uint64_t row_span = (uint64_t)(h / ys - 1U) * stride;
            if (transform & PISP_BE_TRANSFORM_VFLIP) CHECK(offset >= row_span);
            uint64_t end = offset + (transform & PISP_BE_TRANSFORM_VFLIP ? 0 : row_span) + w / xs * bytes;
            if (kind->planes == 1) end += base;
            CHECK(end <= output->planes[kind->planes > 1 ? component : 0].size);
            base += (uint64_t)stride * (output->height / ys);
        }
        for (uint32_t previous = 0; previous < i; ++previous) {
            const struct pisp_tile *q = &p->tiles[previous];
            if (!q->output_width[branch] || !q->output_height[branch]) continue;
            uint32_t qy = transform & PISP_BE_TRANSFORM_VFLIP ?
                q->output_offset_y[branch] + 1U - q->output_height[branch] : q->output_offset_y[branch];
            CHECK(x + w <= q->output_offset_x[branch] || x >= (uint32_t)q->output_offset_x[branch] + q->output_width[branch] ||
                  top_y + h <= qy || top_y >= qy + q->output_height[branch]);
        }
        area += (uint64_t)w * h;
    }
    CHECK(area == (uint64_t)output->width * output->height);
    if (resample)
        for (uint32_t phase = 0; phase < 16; ++phase) {
            int sum = 0;
            for (uint32_t tap = 0; tap < 6; ++tap) {
                int coefficient = p->config.resample[branch].coef[phase * 6 + tap];
                CHECK(coefficient >= -2048 && coefficient <= 2047);
                sum += coefficient;
            }
            CHECK(sum >= 1018 && sum <= 1025);
        }
}

static cc_format aligned_image(uint32_t width, uint32_t height, uint32_t fourcc, uint32_t bytes)
{
    cc_format f = image(width, height, fourcc, bytes);
    f.planes[0].stride = (width * bytes + 63U) & ~63U;
    f.planes[0].size = f.planes[0].stride * height;
    return f;
}

static void test_extended_output_formats(void)
{
    const struct output_kind kinds[] = {
        {V4L2_PIX_FMT_NV16,1,2,1,1,1,false,false},
        {V4L2_PIX_FMT_NV61,1,2,1,1,1,true,false},
        {V4L2_PIX_FMT_NV16M,2,2,1,1,1,false,false},
        {V4L2_PIX_FMT_NV61M,2,2,1,1,1,true,false},
        {V4L2_PIX_FMT_RGBX32,1,1,4,1,1,false,false},
        {V4L2_PIX_FMT_XRGB32,1,1,4,1,1,false,true},
        {V4L2_PIX_FMT_XBGR32,1,1,4,1,1,false,false},
        {V4L2_PIX_FMT_BGRX32,1,1,4,1,1,false,true},
        {CC_FOURCC('R','G','B','6'),1,1,6,1,1,false,false},
        {CC_FOURCC('B','G','R','6'),1,1,6,1,1,false,false}
    };
    const size_t num_kinds=sizeof(kinds)/sizeof(kinds[0]);
    for (size_t i=0;i<sizeof(cc_pisp_format_reference_cases)/sizeof(cc_pisp_format_reference_cases[0]);++i) {
        const struct cc_pisp_format_reference_case *v=&cc_pisp_format_reference_cases[i];
        cc_pisp_be_configuration c={.input=aligned_image(v->width,v->height,V4L2_PIX_FMT_SRGGB16,2),
            .hardware_version=CC_PISP_BE_VERSION_BCM2712_D0};
        memcpy(c.crop,v->crop,sizeof(c.crop)); memcpy(c.transform,v->transform,sizeof(c.transform));
        cc_format *outputs[]={&c.output,&c.output1};
        size_t kind[2]={0};
        for (size_t b=0;b<2;++b) {
            if (!v->output_size[b*2]) continue;
            while (kind[b]<num_kinds && kinds[kind[b]].fourcc!=v->fourcc[b]) ++kind[b];
            CHECK(kind[b]<num_kinds);
            const struct output_kind *layout=&kinds[kind[b]];
            *outputs[b]=aligned_image(v->output_size[b*2],v->output_size[b*2+1],v->fourcc[b],layout->pixel_bytes);
            outputs[b]->num_planes=layout->planes;
            if (layout->components==2) {
                if (layout->planes==1) outputs[b]->planes[0].size*=2;
                else outputs[b]->planes[1]=outputs[b]->planes[0];
            }
        }
        struct pisp_be_tiles_config p;
        OK(cc_pisp_be_build_configuration(&c,NULL,&p));
        CHECK(cc_pisp_tile_hash(p.tiles,p.num_tiles)==v->hash);
        verify_scaled(&c,&p,0,&kinds[kind[0]]);
        if (c.output1.fourcc) verify_scaled(&c,&p,1,&kinds[kind[1]]);
    }
    for (size_t k=0;k<num_kinds;++k) for (uint32_t flip=0;flip<4;++flip) {
        size_t other=(k+1) % (sizeof(kinds)/sizeof(kinds[0]));
        cc_pisp_be_configuration c={.input=image(1640,1232,V4L2_PIX_FMT_SRGGB16,2),
            .output=output_image(&kinds[k],1234,871),.output1=output_image(&kinds[other],334,223),
            .crop={{8,6,1600,1200},{16,10,1540,1190}},.transform={flip,3U-flip},
            .hardware_version=CC_PISP_BE_VERSION_BCM2712_D0};
        struct pisp_be_tiles_config p;
        OK(cc_pisp_be_build_configuration(&c,NULL,&p));
        verify_scaled(&c,&p,0,&kinds[k]); verify_scaled(&c,&p,1,&kinds[other]);
        const size_t indices[]={k,other};
        for (uint32_t b=0;b<2;++b) {
            size_t index=indices[b];
            uint32_t fmt=p.config.output_format[b].image.format;
            CHECK(((fmt & PISP_IMAGE_FORMAT_ORDER_SWAPPED)!=0)==kinds[index].y_second);
            CHECK(((fmt & PISP_IMAGE_FORMAT_BPP_32)!=0)==(index>=4 && index<8));
            CHECK((fmt & PISP_IMAGE_FORMAT_BPS_MASK)==(index>=8 ? PISP_IMAGE_FORMAT_BPS_16 : PISP_IMAGE_FORMAT_BPS_8));
            if (index<4) {
                CHECK((fmt & PISP_IMAGE_FORMAT_PLANARITY_MASK)==PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR);
                CHECK((fmt & PISP_IMAGE_FORMAT_SAMPLING_MASK)==PISP_IMAGE_FORMAT_SAMPLING_422);
            } else {
                /* Independent native colour words: the packing flag changes
                 * padding placement, while CSC exchanges R/B channels. */
                bool swapped=index==6 || index==7 || index==9;
                bool csc=(p.config.global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_CSC(b))!=0;
                CHECK(csc==swapped);
                if (swapped) {
                    CHECK(p.config.csc[b].coeffs[2]==1024 && p.config.csc[b].coeffs[4]==1024 &&
                          p.config.csc[b].coeffs[6]==1024);
                    CHECK(csc_byte(&p.config,b,0,0x1200,0x5600,0x9a00)==0x9a &&
                          csc_byte(&p.config,b,2,0x1200,0x5600,0x9a00)==0x12);
                }
            }
        }
        /* C0 and unknown hardware must reject RGB32 on either branch before
         * touching the caller's parameter buffer. RGB48 and NV16 remain valid. */
        c.hardware_version=flip & 1U ? CC_PISP_BE_VERSION_BCM2712_C0 : 0;
        struct pisp_be_tiles_config saved; memcpy(&saved,&p,sizeof(saved));
        int ret=cc_pisp_be_build_configuration(&c,NULL,&p);
        if ((k>=4 && k<8) || (other>=4 && other<8)) {
            ERROR(ret,ENOTSUP); CHECK(!memcmp(&saved,&p,sizeof(p)));
        } else OK(ret);
    }
    cc_pisp_be_configuration c={.input=image(640,482,V4L2_PIX_FMT_SRGGB16,2),
        .output=output_image(&kinds[2],640,481)};
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_configuration(&c,NULL,&p)); /* 422 permits odd heights. */
    c.output.planes[1].size=c.output.planes[1].stride*480;
    ERROR(cc_pisp_be_build_configuration(&c,NULL,&p),EINVAL);
    c.output=output_image(&kinds[2],639,480);
    ERROR(cc_pisp_be_build_configuration(&c,NULL,&p),EINVAL);
    c.output=image(640,480,CC_FOURCC('R','G','B','6'),6);
    c.output.planes[0].stride=640*3;
    ERROR(cc_pisp_be_build_configuration(&c,NULL,&p),EINVAL);
}

static void test_compressed_bayer(void)
{
    const uint32_t sensors[]={V4L2_PIX_FMT_SRGGB10P,V4L2_PIX_FMT_SGBRG10P,V4L2_PIX_FMT_SBGGR10P,V4L2_PIX_FMT_SGRBG10P};
    const char orders[]={'R','g','B','G'};
    const struct output_kind rgb={V4L2_PIX_FMT_RGB24,1,1,3,1,1,false,false};
    const struct output_kind nv12={V4L2_PIX_FMT_NV12M,2,2,1,1,2,false,false};
    for (size_t order=0;order<4;++order) for (uint32_t mode=1;mode<=3;++mode) {
        cc_format raw=image(1642,1232,CC_FOURCC('P','C','0'+mode,orders[order]),1);
        cc_cfe_configuration fe={.sensor={.width=raw.width,.height=raw.height,.fourcc=sensors[order]},
            .image=raw,.compression_offset=2048};
        struct pisp_fe_config fp;
        OK(cc_cfe_build_parameters(&fe,NULL,&fp));
        CHECK(fp.global.bayer_order==order && (fp.global.enables & PISP_FE_ENABLE_COMPRESS0));
        CHECK(fp.input.streaming && fp.input.format.format==PISP_IMAGE_FORMAT_BPS_16);
        CHECK(fp.ch[0].output.format.format==mode*PISP_IMAGE_FORMAT_COMPRESSION_MODE_1 &&
              fp.ch[0].output.format.stride==(int32_t)raw.planes[0].stride);
        CHECK(fp.ch[0].compress.mode==mode && fp.ch[0].compress.offset==2048 && !fp.ch[0].compress.pad);
        cc_pisp_be_configuration c={.input=raw,.output=output_image(&rgb,1233,871),
            .output1=output_image(&nv12,640,480),.crop={{6,8,1600,1200},{10,12,1600,1200}},
            .transform={(uint32_t)order,3U-(uint32_t)order},.compression_offset=2048};
        struct pisp_be_tiles_config p;
        OK(cc_pisp_be_build_configuration(&c,NULL,&p));
        CHECK(p.config.global.bayer_order==order && (p.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS));
        CHECK(p.config.input_format.format==mode*PISP_IMAGE_FORMAT_COMPRESSION_MODE_1 &&
              p.config.decompress.mode==mode && p.config.decompress.offset==2048 && !p.config.decompress.pad);
        CHECK(p.config.dirty_flags_bayer & PISP_BE_BAYER_ENABLE_DECOMPRESS);
        verify_scaled(&c,&p,0,&rgb); verify_scaled(&c,&p,1,&nv12);
        for (uint32_t i=0;i<p.num_tiles;++i) {
            const struct pisp_tile *t=&p.tiles[i];
            CHECK(t->input_offset_x%8==0);
            CHECK((t->input_offset_x+t->input_width)%8==0 || t->input_offset_x+t->input_width==raw.width);
            CHECK(t->input_addr_offset==t->input_offset_y*raw.planes[0].stride+t->input_offset_x);
            uint32_t padded_width=((uint32_t)t->input_width+7U)&~7U;
            CHECK((uint64_t)t->input_addr_offset+(uint64_t)(t->input_height-1U)*raw.planes[0].stride+
                  padded_width<=raw.planes[0].size);
        }
        /* Per-request I/O fields cannot override the format/offset contract. */
        struct pisp_fe_config ft={0}; ft.ch[0].compress.mode=99; ft.ch[0].compress.offset=99;
        OK(cc_cfe_build_parameters(&fe,&ft,&fp));
        CHECK(fp.ch[0].compress.mode==mode && fp.ch[0].compress.offset==2048);
        struct pisp_be_config bt={0}; bt.decompress.mode=99; bt.decompress.offset=99;
        OK(cc_pisp_be_build_configuration(&c,&bt,&p));
        CHECK(p.config.decompress.mode==mode && p.config.decompress.offset==2048);
        c.compression_offset=0; OK(cc_pisp_be_build_configuration(&c,NULL,&p)); CHECK(!p.config.decompress.offset);
        fe.compression_offset=0; OK(cc_cfe_build_parameters(&fe,NULL,&fp)); CHECK(!fp.ch[0].compress.offset);
        for (unsigned int fault=0;fault<5;++fault) {
            cc_pisp_be_configuration bad=c;
            cc_cfe_configuration bad_fe=fe;
            if (fault==0) bad.compression_offset=bad_fe.compression_offset=65536;
            if (fault==1) bad.input.planes[0].stride=bad_fe.image.planes[0].stride=raw.width;
            if (fault==2) bad.input.planes[0].size=bad_fe.image.planes[0].size=1;
            if (fault==3) bad.input.fourcc=bad_fe.image.fourcc=CC_FOURCC('P','C','4',orders[order]);
            if (fault==4) bad.input.num_planes=bad_fe.image.num_planes=2;
            struct pisp_be_tiles_config saved; memcpy(&saved,&p,sizeof(saved));
            struct pisp_fe_config fsaved; memcpy(&fsaved,&fp,sizeof(fsaved));
            CHECK(cc_pisp_be_build_configuration(&bad,NULL,&p)<0 && !memcmp(&saved,&p,sizeof(p)));
            CHECK(cc_cfe_build_parameters(&bad_fe,NULL,&fp)<0 && !memcmp(&fsaved,&fp,sizeof(fp)));
        }
    }
    cc_cfe_configuration mono={.sensor={.width=640,.height=480,.fourcc=V4L2_PIX_FMT_GREY},
        .image=image(640,480,CC_FOURCC('P','C','1','M'),1)};
    struct pisp_fe_config fp; OK(cc_cfe_build_parameters(&mono,NULL,&fp));
    CHECK(fp.global.bayer_order==PISP_BAYER_ORDER_GREYSCALE && fp.ch[0].compress.mode==1);
    mono.image.fourcc=CC_FOURCC('P','C','1','R');
    ERROR(cc_cfe_build_parameters(&mono,NULL,&fp),ENOTSUP);
}

static void test_compressed_reference_tiles(void)
{
    const struct output_kind rgb={V4L2_PIX_FMT_RGB24,1,1,3,1,1,false,false};
    const struct output_kind nv12={V4L2_PIX_FMT_NV12M,2,2,1,1,2,false,false};
    for (size_t i=0;i<sizeof(cc_pisp_compressed_reference_cases)/sizeof(cc_pisp_compressed_reference_cases[0]);++i) {
        const struct cc_pisp_compressed_reference_case *v=&cc_pisp_compressed_reference_cases[i];
        cc_pisp_be_configuration c={.input=aligned_image(v->width,v->height,CC_FOURCC('P','C','0'+v->mode,'R'),1),
            .output=aligned_image(v->output_size[0],v->output_size[1],V4L2_PIX_FMT_RGB24,3),.compression_offset=2048};
        memcpy(c.crop,v->crop,sizeof(c.crop)); memcpy(c.transform,v->transform,sizeof(c.transform));
        if (v->output_size[2]) {
            c.output1=aligned_image(v->output_size[2],v->output_size[3],V4L2_PIX_FMT_NV12M,1);
            c.output1.num_planes=2;
            c.output1.planes[1]=(cc_plane_format){c.output1.planes[0].stride,c.output1.planes[0].stride*c.output1.height/2};
        }
        struct pisp_be_tiles_config p;
        OK(cc_pisp_be_build_configuration(&c,NULL,&p));
        CHECK(cc_pisp_tile_hash(p.tiles,p.num_tiles)==v->hash);
        verify_scaled(&c,&p,0,&rgb);
        if (c.output1.fourcc) verify_scaled(&c,&p,1,&nv12);
    }
}

static void test_reference_tiles(void)
{
    const struct output_kind rgb = {V4L2_PIX_FMT_RGB24, 1, 1, 3, 1, 1, false, false};
    const struct output_kind nv12m = {V4L2_PIX_FMT_NV12M, 2, 2, 1, 1, 2, false, false};
    for (size_t i = 0; i < sizeof(cc_pisp_reference_cases) / sizeof(cc_pisp_reference_cases[0]); ++i) {
        const struct cc_pisp_reference_case *v = &cc_pisp_reference_cases[i];
        cc_pisp_be_configuration c = { .input = aligned_image(v->width, v->height, V4L2_PIX_FMT_SRGGB16, 2),
            .output = aligned_image(v->output_size[0], v->output_size[1], V4L2_PIX_FMT_RGB24, 3) };
        memcpy(c.crop, v->crop, sizeof(c.crop));
        memcpy(c.transform, v->transform, sizeof(c.transform));
        if (v->output_size[2]) {
            c.output1 = aligned_image(v->output_size[2], v->output_size[3], V4L2_PIX_FMT_NV12M, 1);
            c.output1.num_planes = 2;
            c.output1.planes[1] = (cc_plane_format) { c.output1.planes[0].stride,
                c.output1.planes[0].stride * c.output1.height / 2 };
        }
        struct pisp_be_tiles_config p;
        OK(cc_pisp_be_build_configuration(&c, NULL, &p));
        CHECK(cc_pisp_tile_hash(p.tiles, p.num_tiles) == v->hash);
        verify_scaled(&c, &p, 0, &rgb);
        if (c.output1.fourcc) verify_scaled(&c, &p, 1, &nv12m);
    }
}

static void test_inactive_branch_cursor(void)
{
    /* A heavily upscaled auxiliary crop advances while output0 is inactive
     * between tiles. Its next active tile must resume, rather than repeat. */
    const struct output_kind rgb = {V4L2_PIX_FMT_RGB24, 1, 1, 3, 1, 1, false, false};
    const struct output_kind nv12m = {V4L2_PIX_FMT_NV12M, 2, 2, 1, 1, 2, false, false};
    cc_pisp_be_configuration c = {
        .input = image(1640, 2254, V4L2_PIX_FMT_SBGGR16, 2),
        .output = output_image(&rgb, 52, 3760),
        .output1 = output_image(&nv12m, 8718, 528),
        .crop = {{1126, 1476, 514, 778}, {884, 1982, 756, 272}}
    };
    for (uint32_t flip = 0; flip < 4; ++flip) {
        c.transform[0] = flip;
        struct pisp_be_tiles_config p;
        OK(cc_pisp_be_build_configuration(&c, NULL, &p));
        verify_scaled(&c, &p, 0, &rgb);
        verify_scaled(&c, &p, 1, &nv12m);
    }
}

static void test_resize_controls(void)
{
    cc_pisp_be_configuration c = { .input = image(1640, 1232, V4L2_PIX_FMT_SRGGB16, 2),
        .output = image(1280, 720, V4L2_PIX_FMT_RGB24, 3), .crop = {{10, 100, 1600, 900}} };
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_configuration(&c, NULL, &p));
    CHECK(p.config.resample[0].scale_factor_h == 1599U * 4096U / 1279U);
    CHECK(p.config.resample[0].scale_factor_v == 899U * 4096U / 719U);
    c.output = image(1640, 1232, V4L2_PIX_FMT_RGB24, 3);
    c.crop[0] = (cc_rectangle) {0};
    struct pisp_be_config tuning = {0};
    tuning.global.rgb_enables = PISP_BE_RGB_ENABLE_RESAMPLE0;
    for (uint32_t phase = 0; phase < 16; ++phase) tuning.resample[0].coef[phase * 6 + 2] = 1024;
    OK(cc_pisp_be_build_configuration(&c, &tuning, &p));
    CHECK(p.config.resample[0].scale_factor_h == 4096 && p.config.resample[0].coef[2] == 1024);
    for (int bad = 0; bad < 9; ++bad) {
        cc_pisp_be_configuration invalid = c;
        struct pisp_be_config tune = tuning;
        if (bad == 0) invalid.crop[0] = (cc_rectangle) {-1, 0, 640, 480};
        if (bad == 1) invalid.crop[0] = (cc_rectangle) {0, 0, 640, 0};
        if (bad == 2) invalid.crop[0] = (cc_rectangle) {1640, 0, 640, 480};
        if (bad == 3) invalid.transform[0] = 4;
        if (bad == 4) tune.resample[0].coef[0] = 2048;
        if (bad == 5) tune.global.rgb_enables = PISP_BE_RGB_ENABLE_RESAMPLE1;
        if (bad == 6) invalid.output = image(16, 16, V4L2_PIX_FMT_RGB24, 3);
        if (bad == 7) { invalid.crop[0] = (cc_rectangle) {0, 0, 32, 32}; invalid.output = image(1024, 1024, V4L2_PIX_FMT_RGB24, 3); }
        if (bad == 8) invalid.crop[0] = (cc_rectangle) {0, 0, 15, 16};
        memset(&p, 0xa5, sizeof(p));
        struct pisp_be_tiles_config original = p;
        CHECK(cc_pisp_be_build_configuration(&invalid, &tune, &p) < 0);
        CHECK(!memcmp(&p, &original, sizeof(p)));
    }
}

static uint32_t resize_random(uint32_t *state)
{
    *state = *state * 1664525U + 1013904223U;
    return *state;
}

static void test_resize_coverage(void)
{
    const struct output_kind kinds[] = {
        {V4L2_PIX_FMT_RGB24, 1, 1, 3, 1, 1, false, false},
        {V4L2_PIX_FMT_NV12M, 2, 2, 1, 1, 2, false, false},
        {V4L2_PIX_FMT_YUV420, 1, 3, 1, 2, 2, false, false}
    };
    const uint32_t ratios[7][2] = {{1, 2}, {1, 1}, {3, 2}, {2, 1}, {3, 1}, {4, 1}, {8, 1}};
    uint32_t random = 0x43726f70, accepted = 0;
    for (uint32_t test = 0; test < 2000; ++test) {
        uint32_t width = 64U + 2U * (resize_random(&random) % 2000U);
        uint32_t height = 64U + 2U * (resize_random(&random) % 1600U);
        cc_pisp_be_configuration c = { .input = image(width, height, V4L2_PIX_FMT_SRGGB16, 2) };
        cc_format *outputs[2] = {&c.output, &c.output1};
        uint32_t kind[2];
        for (uint32_t branch = 0; branch < 2; ++branch) {
            cc_rectangle *crop = &c.crop[branch];
            crop->x = (int32_t)(resize_random(&random) % (width - 31U));
            crop->y = (int32_t)(resize_random(&random) % (height - 31U));
            crop->width = 32U + resize_random(&random) % (width - (uint32_t)crop->x - 31U);
            crop->height = 32U + resize_random(&random) % (height - (uint32_t)crop->y - 31U);
            uint32_t ratio = resize_random(&random) % 7U;
            uint32_t ow = crop->width * ratios[ratio][1] / ratios[ratio][0];
            uint32_t oh = crop->height * ratios[ratio][1] / ratios[ratio][0];
            if (ow < 16) ow = 16;
            if (oh < 16) oh = 16;
            kind[branch] = resize_random(&random) % 3U;
            if (kind[branch]) { ow &= ~1U; oh &= ~1U; }
            *outputs[branch] = output_image(&kinds[kind[branch]], ow, oh);
            c.transform[branch] = (test + branch) % 4U;
        }
        struct pisp_be_tiles_config p;
        int ret = cc_pisp_be_build_configuration(&c, NULL, &p);
        if (ret < 0) { CHECK(ret == -ERANGE || ret == -E2BIG); continue; }
        ++accepted;
        verify_scaled(&c, &p, 0, &kinds[kind[0]]);
        verify_scaled(&c, &p, 1, &kinds[kind[1]]);
    }
    CHECK(accepted > 1500);
    /* Near-limit enlargement with odd RGB sizes and both flip axes. */
    cc_pisp_be_configuration c = { .input = image(128, 96, V4L2_PIX_FMT_SRGGB16, 2),
        .output = image(1001, 601, V4L2_PIX_FMT_RGB24, 3),
        .crop = {{32, 20, 64, 40}}, .transform = {PISP_BE_TRANSFORM_ROT180, 0} };
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_configuration(&c, NULL, &p));
    verify_scaled(&c, &p, 0, &kinds[0]);
}

static void test_history_configuration(void)
{
    struct pisp_be_tiles_config p;
    const struct output_kind rgb={V4L2_PIX_FMT_RGB24,1,1,3,1,1,false,false};
    for (uint32_t mode=0;mode<4;++mode) {
        cc_pisp_be_configuration c={.input=image(1642,1232,V4L2_PIX_FMT_SRGGB16,2),
            .output=output_image(&rgb,320,240),.output1=output_image(&rgb,160,120),
            .crop={{800,600,640,480},{900,700,640,480}}};
        for (uint32_t i=0;i<4;++i) {
            uint32_t compression=mode ? (mode+i)%3U+1U : 0;
            c.auxiliary[i].format=image(1642,1232,compression ? CC_FOURCC('P','C','0'+compression,'R') :
                V4L2_PIX_FMT_SRGGB16,compression ? 1:2);
            c.auxiliary[i].compression_offset=compression ? 4095U:0;
        }
        struct pisp_be_config t={0};
        t.global.bayer_enables=PISP_BE_BAYER_ENABLE_TDN|PISP_BE_BAYER_ENABLE_STITCH;
        t.tdn.ratio=16384; t.tdn.threshold=32768;
        t.stitch.exposure_ratio=8192; t.stitch.threshold_diff_power=15; t.stitch.motion_threshold_256=1;
        OK(cc_pisp_be_build_configuration(&c,&t,&p));
        CHECK(p.config.stitch.motion_threshold_recip==255);
        CHECK(p.config.tdn_input_format.width==1642 && p.config.stitch_output_format.height==1232);
        CHECK(!p.config.tdn.pad && !p.config.stitch.pad);
        verify_scaled(&c,&p,0,&rgb); verify_scaled(&c,&p,1,&rgb);
        unsigned char *coverage=calloc((size_t)1642*1232,1); CHECK(coverage);
        for (uint32_t n=0;n<p.num_tiles;++n) {
            const struct pisp_tile *tile=&p.tiles[n];
            CHECK(tile->input_offset_x%(mode ? 16U:8U)==0);
            const uint32_t offsets[4]={tile->tdn_input_addr_offset,tile->tdn_output_addr_offset,
                tile->stitch_input_addr_offset,tile->stitch_output_addr_offset};
            for (uint32_t i=0;i<4;++i) {
                const cc_format *f=&c.auxiliary[i].format;
                uint32_t bytes=mode ? 1U:2U;
                CHECK(offsets[i]==tile->input_offset_y*f->planes[0].stride+tile->input_offset_x*bytes);
                CHECK((uint64_t)offsets[i]+(uint64_t)(tile->input_height-1U)*f->planes[0].stride+
                    (mode ? ((tile->input_width+7U)&~7U):2U*tile->input_width)<=f->planes[0].size);
                if (i&1U) CHECK(offsets[i]%16U==0);
            }
            for (uint32_t y=tile->input_offset_y;y<(uint32_t)tile->input_offset_y+tile->input_height;++y)
                memset(coverage+(size_t)y*1642+tile->input_offset_x,1,tile->input_width);
        }
        for (size_t n=0;n<(size_t)1642*1232;++n) CHECK(coverage[n]);
        free(coverage);
        t.tdn.reset=1;
        OK(cc_pisp_be_build_configuration(&c,&t,&p));
        CHECK(!(p.config.global.bayer_enables & (PISP_BE_BAYER_ENABLE_TDN_INPUT|PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS)));
        CHECK(!p.config.tdn_input_format.width);
        for (uint32_t fault=0;fault<7;++fault) {
            cc_pisp_be_configuration bad=c; struct pisp_be_config tune=t;
            if (fault==0) bad.auxiliary[1].format.fourcc=0;
            if (fault==1) bad.auxiliary[2].format.fourcc=0;
            if (fault==2) bad.auxiliary[3].format.width-=2;
            if (fault==3) bad.auxiliary[0].compression_offset=65536;
            if (fault==4) tune.tdn.reset=2;
            if (fault==5) tune.stitch.threshold_diff_power=16;
            if (fault==6) tune.stitch.motion_threshold_recip=255; /* threshold=16 permits <=16 */
            if (fault==6) tune.stitch.motion_threshold_256=16;
            struct pisp_be_tiles_config before=p;
            CHECK(cc_pisp_be_build_configuration(&bad,&tune,&p)<0 && !memcmp(&before,&p,sizeof(p)));
        }
    }
}

static void test_frontend_geometry(void)
{
    cc_cfe_configuration c = {.sensor={.width=1640,.height=1232,.fourcc=V4L2_PIX_FMT_SRGGB14P},
        .image=image(1280,960,V4L2_PIX_FMT_SRGGB16,2),
        .image1=image(1280,960,CC_FOURCC('P','C','3','R'),1),
        .crop={{2,4,1280,960},{8,10,1280,960}},.compression_offset1=4095};
    struct pisp_fe_config p;
    for (uint32_t in=1;in<=31;++in) for (uint32_t out=1;out<=in;++out) {
        if (in > 16U*out) continue;
        uint32_t width = 2U*(640U*out/in), height = 2U*(480U*out/in);
        c.image = image(width,height,V4L2_PIX_FMT_SRGGB16,2);
        c.image1 = image(width,height,CC_FOURCC('P','C','3','R'),1);
        c.downscale[0] = (struct pisp_fe_downscale_config) {
            .xin=(uint8_t)in,.xout=(uint8_t)out,.yin=(uint8_t)in,.yout=(uint8_t)out};
        c.downscale[1] = c.downscale[0]; c.downscale[1].flags = DOWNSCALE_BIN;
        OK(cc_cfe_build_parameters(&c,NULL,&p));
        CHECK(p.input.format.width==1640 && p.input.format.height==1232);
        CHECK(p.agc_stats.size_x==102 && p.agc_stats.size_y==76); /* Full sensor statistics. */
        for (uint32_t i=0;i<2;++i) {
            CHECK(p.ch[i].downscale.output_width==width && p.ch[i].downscale.output_height==height);
            CHECK(p.ch[i].output.format.width==width && p.ch[i].output.format.height==height);
            CHECK(p.global.enables & (uint32_t)PISP_FE_ENABLE_OUTPUT(i));
            CHECK(p.global.enables & (uint32_t)PISP_FE_ENABLE_CROP(i));
            CHECK(p.global.enables & (uint32_t)PISP_FE_ENABLE_DOWNSCALE(i));
        }
        CHECK(p.ch[0].downscale.flags==DOWNSCALE_BAYER);
        CHECK(p.ch[1].downscale.flags==(DOWNSCALE_BAYER|DOWNSCALE_BIN));
        CHECK(p.ch[1].compress.mode==3 && p.ch[1].compress.offset==4095);
    }
    for (uint32_t fault=0;fault<10;++fault) {
        cc_cfe_configuration bad=c;
        if (fault==0) bad.crop[0].x=-2;
        if (fault==1) bad.crop[1].width=1279;
        if (fault==2) bad.crop[1].y=1000;
        if (fault==3) bad.downscale[0].xin=32;
        if (fault==4) bad.downscale[1].xout=0;
        if (fault==5) bad.downscale[1].xout=32;
        if (fault==6) { bad.downscale[1].xin=31; bad.downscale[1].xout=1; }
        if (fault==7) bad.downscale[0].flags=4;
        if (fault==8) bad.image1.width+=2;
        if (fault==9) bad.compression_offset1=65536;
        struct pisp_fe_config before=p;
        CHECK(cc_cfe_build_parameters(&bad,NULL,&p)<0 && !memcmp(&p,&before,sizeof(p)));
    }
    c.sensor.fourcc=V4L2_PIX_FMT_Y16; c.image.fourcc=V4L2_PIX_FMT_Y16;
    c.image1.fourcc=CC_FOURCC('P','C','3','M'); c.downscale[1].flags=0;
    c.downscale[0].xin=c.downscale[1].xin=2;
    c.downscale[0].xout=c.downscale[1].xout=1;
    c.image.width=c.image1.width=640;
    OK(cc_cfe_build_parameters(&c,NULL,&p));
    CHECK(p.global.bayer_order==PISP_BAYER_ORDER_GREYSCALE && !p.ch[0].downscale.flags);
}

static void test_mono_and_lsb_inputs(void)
{
    const uint32_t raw[][4] = {
        {V4L2_PIX_FMT_SRGGB10,V4L2_PIX_FMT_SGBRG10,V4L2_PIX_FMT_SBGGR10,V4L2_PIX_FMT_SGRBG10},
        {V4L2_PIX_FMT_SRGGB12,V4L2_PIX_FMT_SGBRG12,V4L2_PIX_FMT_SBGGR12,V4L2_PIX_FMT_SGRBG12},
        {V4L2_PIX_FMT_SRGGB14,V4L2_PIX_FMT_SGBRG14,V4L2_PIX_FMT_SBGGR14,V4L2_PIX_FMT_SGRBG14}};
    struct pisp_be_tiles_config p;
    for (uint32_t depth=0;depth<3;++depth) for (uint32_t order=0;order<4;++order) {
        cc_format input=image(1640,1232,raw[depth][order],2), output=image(1640,1232,V4L2_PIX_FMT_RGB24,3);
        OK(cc_pisp_be_build_parameters(&input,&output,NULL,&p));
        CHECK(p.config.global.bayer_order==order);
        CHECK(p.config.input_format.format==(PISP_IMAGE_FORMAT_BPS_16|(6U-2U*depth)*PISP_IMAGE_FORMAT_SHIFT_1));
        verify_tiles(&input,&output,&p);
        cc_cfe_configuration fe = {.sensor=input,.image=input};
        struct pisp_fe_config fp;
        OK(cc_cfe_build_parameters(&fe,NULL,&fp));
        CHECK(fp.global.bayer_order==order && fp.ch[0].output.format.format==p.config.input_format.format);
        CHECK(fp.input.format.format==PISP_IMAGE_FORMAT_BPS_16);
    }
    const uint32_t mono[] = {V4L2_PIX_FMT_Y16,V4L2_PIX_FMT_Y10,V4L2_PIX_FMT_Y12,V4L2_PIX_FMT_Y14,
        CC_FOURCC('P','C','1','M'),CC_FOURCC('P','C','2','M'),CC_FOURCC('P','C','3','M')};
    const struct output_kind kinds[] = {{V4L2_PIX_FMT_GREY,1,1,1,1,1,false,false},
        {V4L2_PIX_FMT_Y16,1,1,2,1,1,false,false}, {V4L2_PIX_FMT_RGB24,1,1,3,1,1,false,false}};
    for (size_t i=0;i<sizeof(mono)/sizeof(*mono);++i) for (size_t k=0;k<3;++k) {
        cc_pisp_be_configuration c={.input=image(1640,1232,mono[i],i<4 ? 2:1),
            .output=output_image(&kinds[k],800,600), .crop={{8,10,1600,1200}}, .transform={3,0}};
        OK(cc_pisp_be_build_configuration(&c,NULL,&p));
        CHECK(!(p.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_DEMOSAIC));
        struct pisp_be_config tuning = {0}; tuning.demosaic.fc_mode = 255;
        OK(cc_pisp_be_build_configuration(&c,&tuning,&p)); /* Bypassed block is ignored. */
        CHECK(!p.config.demosaic.fc_mode);
        CHECK(p.config.global.bayer_order==0); /* BE has a two-bit order, no FE mono flag. */
        CHECK(p.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT);
        CHECK(!(p.config.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT));
        verify_scaled(&c,&p,0,&kinds[k]);
        if (k<2) {
            CHECK(!PISP_IMAGE_FORMAT_THREE_CHANNEL(p.config.output_format[0].image.format));
            CHECK(p.config.csc[0].coeffs[0]+p.config.csc[0].coeffs[1]+p.config.csc[0].coeffs[2]==879);
        }
    }
}

int main(void)
{
    test_history_configuration();
    test_frontend_geometry();
    test_mono_and_lsb_inputs();
    test_compressed_bayer();
    test_compressed_reference_tiles();
    test_extended_output_formats();
    test_coverage();
    test_tuning();
    test_invalid();
    test_yuv_and_outputs();
    test_reference_tiles();
    test_inactive_branch_cursor();
    test_resize_controls();
    test_resize_coverage();
    puts("PiSP C17 native configuration, reference tiles, crop/resize/flip coverage, DMA bounds and tuning passed");
    return 0;
}
