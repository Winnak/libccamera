/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "pisp_format.h"
#include <ccamera/pisp_be.h>
#include <linux/videodev2.h>

static const uint32_t outputs[] = {
    V4L2_PIX_FMT_RGB24, V4L2_PIX_FMT_BGR24,
    V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_YVYU, V4L2_PIX_FMT_VYUY,
    V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M,
    V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_YVU420, V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_YVU420M,
    V4L2_PIX_FMT_YUV422P, V4L2_PIX_FMT_YUV422M, V4L2_PIX_FMT_YVU422M,
    V4L2_PIX_FMT_YUV444M, V4L2_PIX_FMT_YVU444M,
    V4L2_PIX_FMT_NV16, V4L2_PIX_FMT_NV61, V4L2_PIX_FMT_NV16M, V4L2_PIX_FMT_NV61M,
    V4L2_PIX_FMT_RGBX32, V4L2_PIX_FMT_XRGB32, V4L2_PIX_FMT_XBGR32, V4L2_PIX_FMT_BGRX32,
    CC_FOURCC('R','G','B','6'), CC_FOURCC('B','G','R','6'), V4L2_PIX_FMT_GREY, V4L2_PIX_FMT_Y16
};

static void verify(const cc_pisp_be_configuration *c, const struct pisp_be_tiles_config *p)
{
    bool compressed=PISP_IMAGE_FORMAT_COMPRESSED(p->config.input_format.format);
    FUZZ_CHECK(p->num_tiles && p->num_tiles <= PISP_BACK_END_NUM_TILES);
    for (uint32_t i = 0; i < p->num_tiles; ++i) {
        const struct pisp_tile *t = &p->tiles[i];
        FUZZ_CHECK(t->input_width >= 16 && t->input_width <= 640);
        FUZZ_CHECK(t->input_height >= 16 && t->input_height <= 3072);
        FUZZ_CHECK((uint32_t)t->input_offset_x + t->input_width <= c->input.width);
        FUZZ_CHECK((uint32_t)t->input_offset_y + t->input_height <= c->input.height);
        uint32_t width=compressed ? ((uint32_t)t->input_width+7U)&~7U : 2U*t->input_width;
        FUZZ_CHECK(t->input_addr_offset==(uint32_t)t->input_offset_y*c->input.planes[0].stride+
            (uint32_t)t->input_offset_x*(compressed ? 1U : 2U));
        if (compressed) FUZZ_CHECK(t->input_offset_x%8==0);
        FUZZ_CHECK((uint64_t)t->input_addr_offset +
            (uint64_t)(t->input_height - 1U) * c->input.planes[0].stride +
            width <= c->input.planes[0].size);
    }
    const cc_format *formats[] = {&c->output, &c->output1};
    for (uint32_t b = 0; b < (c->output1.fourcc ? 2U : 1U); ++b) {
        const cc_format *f = formats[b];
        struct cc_pisp_output_layout l;
        FUZZ_CHECK(cc_pisp_output_layout(f, &l) == 0);
        bool flip = (c->transform[b] & PISP_BE_TRANSFORM_VFLIP) != 0;
        uint64_t area = 0;
        for (uint32_t i = 0; i < p->num_tiles; ++i) {
            const struct pisp_tile *t = &p->tiles[i];
            uint32_t x = t->output_offset_x[b], y = t->output_offset_y[b];
            uint32_t w = t->output_width[b], h = t->output_height[b];
            if (!w || !h) { FUZZ_CHECK(!w && !h); continue; }
            FUZZ_CHECK(x + w <= f->width);
            FUZZ_CHECK(flip ? y < f->height && y + 1U >= h : y + h <= f->height);
            uint32_t top = flip ? y + 1U - h : y;
            uint64_t base = 0;
            for (uint32_t component = 0; component < l.components; ++component) {
                uint32_t xs = component ? l.xsub : 1, ys = component ? l.ysub : 1;
                uint32_t stride = component ? (uint32_t)l.native.stride2 : (uint32_t)l.native.stride;
                uint32_t bytes = component ? 1 : l.pixel_bytes;
                FUZZ_CHECK(w % xs == 0 && h % ys == 0 && x % xs == 0 && top % ys == 0);
                uint64_t offset = component ? t->output_addr_offset2[b] : t->output_addr_offset[b];
                FUZZ_CHECK(offset == (uint64_t)(y / ys) * stride + (x / xs) * bytes);
                uint64_t end = (uint64_t)(top / ys + h / ys - 1U) * stride + (x / xs + w / xs) * bytes;
                uint32_t plane = l.planes > 1 ? component : 0;
                FUZZ_CHECK(end + (l.planes == 1 ? base : 0) <= f->planes[plane].size);
                base += (uint64_t)stride * (f->height / ys);
            }
            for (uint32_t j = 0; j < i; ++j) {
                const struct pisp_tile *q = &p->tiles[j];
                uint32_t qw = q->output_width[b], qh = q->output_height[b];
                if (!qw || !qh) continue;
                uint32_t qy = flip ? q->output_offset_y[b] + 1U - qh : q->output_offset_y[b];
                FUZZ_CHECK(x + w <= q->output_offset_x[b] || x >= q->output_offset_x[b] + qw ||
                           top + h <= qy || top >= qy + qh);
            }
            area += (uint64_t)w * h;
        }
        FUZZ_CHECK(area == (uint64_t)f->width * f->height);
        for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 2; ++j)
            FUZZ_CHECK(!p->config.output_buffer[b].addr[i][j]);
    }
    for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 2; ++j)
        FUZZ_CHECK(!p->config.input_buffer.addr[i][j]);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    uint32_t w = 16U + 2U * (fuzz_u16(&in) % 8193U);
    uint32_t h = 16U + 2U * (fuzz_u16(&in) % 8193U);
    const uint32_t raw[] = {V4L2_PIX_FMT_SRGGB16, V4L2_PIX_FMT_SGBRG16, V4L2_PIX_FMT_SBGGR16, V4L2_PIX_FMT_SGRBG16};
    uint32_t raw_index=fuzz_u8(&in);
    cc_pisp_be_configuration c = {.input = {.fourcc = raw[raw_index % 4],
        .width = w, .height = h, .num_planes = 1, .planes = {{w * 2U, 0}}}};
    c.input.planes[0].stride = (w * 2U + 15U) & ~15U;
    c.input.planes[0].size = c.input.planes[0].stride * h;
    if (!(mode & 128U)) {
        static const uint32_t lsb[3][4] = {
            {V4L2_PIX_FMT_SRGGB10,V4L2_PIX_FMT_SGBRG10,V4L2_PIX_FMT_SBGGR10,V4L2_PIX_FMT_SGRBG10},
            {V4L2_PIX_FMT_SRGGB12,V4L2_PIX_FMT_SGBRG12,V4L2_PIX_FMT_SBGGR12,V4L2_PIX_FMT_SGRBG12},
            {V4L2_PIX_FMT_SRGGB14,V4L2_PIX_FMT_SGBRG14,V4L2_PIX_FMT_SBGGR14,V4L2_PIX_FMT_SGRBG14}};
        const uint32_t mono[]={V4L2_PIX_FMT_Y16,V4L2_PIX_FMT_Y10,V4L2_PIX_FMT_Y12,V4L2_PIX_FMT_Y14};
        if (raw_index>=16) c.input.fourcc=mono[raw_index%4];
        else if (raw_index>=4) c.input.fourcc=lsb[raw_index/4-1][raw_index%4];
    }
    if (mode & 128U) {
        const char order[]={'R','g','B','G'};
        c.input.fourcc=CC_FOURCC('P','C','1'+((raw_index/4)%3),raw_index>=12 ? 'M' : order[raw_index%4]);
        c.input.planes[0].stride=(w+15U)&~15U;
        c.input.planes[0].size=c.input.planes[0].stride*h;
        c.compression_offset=fuzz_u16(&in);
    }
    c.hardware_version = mode & 64U ? CC_PISP_BE_VERSION_BCM2712_D0 : 0;
    cc_format *formats[] = {&c.output, &c.output1};
    for (uint32_t b = 0; b < (mode & 4U ? 2U : 1U); ++b) {
        cc_rectangle *crop = &c.crop[b];
        *crop = (cc_rectangle){0, 0, w, h};
        if (mode & 8U) {
            crop->x = (int32_t)(2U * (fuzz_u16(&in) % ((w - 16U) / 2U + 1U)));
            crop->y = (int32_t)(2U * (fuzz_u16(&in) % ((h - 16U) / 2U + 1U)));
            crop->width = w - (uint32_t)crop->x; crop->height = h - (uint32_t)crop->y;
        }
        cc_format *f = formats[b];
        f->fourcc = outputs[fuzz_u8(&in) % (sizeof(outputs) / sizeof(outputs[0]))];
        f->width = mode & 16U ? 16U + 2U * (fuzz_u16(&in) % 8193U) : crop->width;
        f->height = mode & 16U ? 16U + 2U * (fuzz_u16(&in) % 8193U) : crop->height;
        c.transform[b] = fuzz_u8(&in) & 3U;
        FUZZ_CHECK(cc_pisp_prepare_output(f) == 0);
    }
    if (mode & 32U)
        for (uint32_t i=0;i<4;++i) {
            c.auxiliary[i].format=c.input;
            c.auxiliary[i].compression_offset=c.compression_offset;
        }
    if (mode & 1U) fuzz_patch(&in, &c, sizeof(c));
    struct pisp_be_config tuning = {0};
    if (mode & 32U) {
        tuning.global.bayer_enables=PISP_BE_BAYER_ENABLE_TDN|PISP_BE_BAYER_ENABLE_STITCH;
        tuning.tdn.ratio=16384; tuning.stitch.exposure_ratio=8192;
        tuning.stitch.threshold_diff_power=8; tuning.stitch.motion_threshold_256=16;
    }
    if (mode & 2U) {
        tuning.global.bayer_enables = fuzz_u32(&in);
        tuning.global.rgb_enables = fuzz_u32(&in);
        fuzz_patch(&in, &tuning, sizeof(tuning));
    }
    struct pisp_be_tiles_config p, before;
    memset(&p, 0xa5, sizeof(p)); memcpy(&before, &p, sizeof(p));
    int ret = cc_pisp_be_build_configuration(&c, mode & (2U|32U) ? &tuning : NULL, &p);
    FUZZ_CHECK(ret <= 0);
    if (ret) FUZZ_CHECK(!memcmp(&p, &before, sizeof(p)));
    else verify(&c, &p);
    return 0;
}
