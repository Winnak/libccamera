/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "pisp_format.h"

#include <errno.h>
#include <limits.h>
#include <linux/videodev2.h>
#include <string.h>

int cc_pisp_raw_format(uint32_t fourcc, struct cc_pisp_raw_layout *layout)
{
    if (!layout) return -EINVAL;
    struct cc_pisp_raw_layout l = {.pixel_bytes = 2, .native.format = PISP_IMAGE_FORMAT_BPS_16};
    switch (fourcc) {
    case V4L2_PIX_FMT_SRGGB16: l.bayer_order = PISP_BAYER_ORDER_RGGB; break;
    case V4L2_PIX_FMT_SGRBG16: l.bayer_order = PISP_BAYER_ORDER_GRBG; break;
    case V4L2_PIX_FMT_SGBRG16: l.bayer_order = PISP_BAYER_ORDER_GBRG; break;
    case V4L2_PIX_FMT_SBGGR16: l.bayer_order = PISP_BAYER_ORDER_BGGR; break;
    case V4L2_PIX_FMT_Y16: l.bayer_order = PISP_BAYER_ORDER_GREYSCALE; break;
#define RAW_LSB(order, native_order) \
    case V4L2_PIX_FMT_S##order##10: l.bayer_order = native_order; l.native.format |= PISP_IMAGE_FORMAT_SHIFT_6; break; \
    case V4L2_PIX_FMT_S##order##12: l.bayer_order = native_order; l.native.format |= PISP_IMAGE_FORMAT_SHIFT_4; break; \
    case V4L2_PIX_FMT_S##order##14: l.bayer_order = native_order; l.native.format |= PISP_IMAGE_FORMAT_SHIFT_2; break
    RAW_LSB(RGGB, PISP_BAYER_ORDER_RGGB);
    RAW_LSB(GRBG, PISP_BAYER_ORDER_GRBG);
    RAW_LSB(GBRG, PISP_BAYER_ORDER_GBRG);
    RAW_LSB(BGGR, PISP_BAYER_ORDER_BGGR);
#undef RAW_LSB
    case V4L2_PIX_FMT_Y10: l.bayer_order = PISP_BAYER_ORDER_GREYSCALE; l.native.format |= PISP_IMAGE_FORMAT_SHIFT_6; break;
    case V4L2_PIX_FMT_Y12: l.bayer_order = PISP_BAYER_ORDER_GREYSCALE; l.native.format |= PISP_IMAGE_FORMAT_SHIFT_4; break;
    case V4L2_PIX_FMT_Y14: l.bayer_order = PISP_BAYER_ORDER_GREYSCALE; l.native.format |= PISP_IMAGE_FORMAT_SHIFT_2; break;
    default: {
        /* Use stable FOURCC values with older toolchain Linux headers. */
        if ((fourcc & UINT32_C(0xffff)) != CC_FOURCC('P', 'C', 0, 0)) return -ENOTSUP;
        uint32_t mode = (fourcc >> 16) & 255U;
        if (mode != '1' && mode != '2' && mode != '3') return -ENOTSUP;
        switch (fourcc >> 24) {
        case 'R': l.bayer_order = PISP_BAYER_ORDER_RGGB; break;
        case 'G': l.bayer_order = PISP_BAYER_ORDER_GRBG; break;
        case 'g': l.bayer_order = PISP_BAYER_ORDER_GBRG; break;
        case 'B': l.bayer_order = PISP_BAYER_ORDER_BGGR; break;
        case 'M': l.bayer_order = PISP_BAYER_ORDER_GREYSCALE; break;
        default: return -ENOTSUP;
        }
        l.compression = mode - '0'; l.pixel_bytes = 1;
        l.native.format = l.compression * PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
        break;
    }
    }
    *layout = l;
    return 0;
}

int cc_pisp_raw_layout(const cc_format *format, struct cc_pisp_raw_layout *layout)
{
    if (!format || !layout) return -EINVAL;
    struct cc_pisp_raw_layout l;
    int ret = cc_pisp_raw_format(format->fourcc, &l);
    if (ret < 0) return ret;
    uint32_t width = l.compression ? (format->width + 7U) & ~7U : format->width;
    if (format->width < 16 || format->height < 16 || format->width > UINT16_MAX || format->height > UINT16_MAX ||
        (format->width | format->height) & 1U || format->num_planes != 1 ||
        format->planes[0].stride > INT32_MAX || format->planes[0].stride % 16 ||
        (uint64_t)width * l.pixel_bytes > format->planes[0].stride ||
        (uint64_t)format->planes[0].stride * format->height > format->planes[0].size) return -EINVAL;
    l.native.width = (uint16_t)format->width; l.native.height = (uint16_t)format->height;
    l.native.stride = (int32_t)format->planes[0].stride;
    *layout = l;
    return 0;
}

static int describe(uint32_t fourcc, struct cc_pisp_output_layout *out)
{
    struct cc_pisp_output_layout l = { .planes = 1, .components = 1, .pixel_bytes = 3,
        .xsub = 1, .ysub = 1, .alignment = 64, .native.format = PISP_IMAGE_FORMAT_THREE_CHANNEL };
    switch (fourcc) {
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_Y16:
        l.yuv = true; l.pixel_bytes = fourcc == V4L2_PIX_FMT_Y16 ? 2U : 1U;
        l.alignment = 64U / l.pixel_bytes;
        l.native.format = fourcc == V4L2_PIX_FMT_Y16 ? PISP_IMAGE_FORMAT_BPS_16 : PISP_IMAGE_FORMAT_BPS_8;
        break;
    case V4L2_PIX_FMT_RGB24: break;
    case V4L2_PIX_FMT_BGR24: l.swap_rb = true; break;
    case V4L2_PIX_FMT_RGBX32: case V4L2_PIX_FMT_XRGB32:
    case V4L2_PIX_FMT_XBGR32: case V4L2_PIX_FMT_BGRX32:
        l.pixel_bytes = 4; l.alignment = 16;
        l.native.format |= PISP_IMAGE_FORMAT_BPP_32;
        l.swap_rb = fourcc == V4L2_PIX_FMT_XBGR32 || fourcc == V4L2_PIX_FMT_BGRX32;
        if (fourcc == V4L2_PIX_FMT_XRGB32 || fourcc == V4L2_PIX_FMT_BGRX32)
            l.native.format |= PISP_IMAGE_FORMAT_ORDER_SWAPPED;
        break;
    /* RGB48/BGR48 identifiers are newer than some supported Linux headers. */
    case CC_FOURCC('R', 'G', 'B', '6'): case CC_FOURCC('B', 'G', 'R', '6'):
        l.pixel_bytes = 6; l.alignment = 32;
        l.native.format |= PISP_IMAGE_FORMAT_BPS_16;
        l.swap_rb = fourcc == CC_FOURCC('B', 'G', 'R', '6');
        break;
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_YVYU:
    case V4L2_PIX_FMT_VYUY:
        l.yuv = true;
        l.pixel_bytes = 2;
        l.alignment = 32;
        l.native.format |= PISP_IMAGE_FORMAT_SAMPLING_422;
        if (fourcc == V4L2_PIX_FMT_UYVY || fourcc == V4L2_PIX_FMT_VYUY)
            l.native.format |= PISP_IMAGE_FORMAT_ORDER_SWAPPED;
        l.swap_uv = fourcc == V4L2_PIX_FMT_YVYU || fourcc == V4L2_PIX_FMT_VYUY;
        break;
    case V4L2_PIX_FMT_NV12M: case V4L2_PIX_FMT_NV21M: l.planes = 2; /* fall through */
    case V4L2_PIX_FMT_NV12: case V4L2_PIX_FMT_NV21:
        l.yuv = true; l.components = 2; l.pixel_bytes = 1; l.ysub = 2;
        l.native.format |= PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR | PISP_IMAGE_FORMAT_SAMPLING_420;
        l.swap_uv = fourcc == V4L2_PIX_FMT_NV21 || fourcc == V4L2_PIX_FMT_NV21M;
        break;
    case V4L2_PIX_FMT_NV16M: case V4L2_PIX_FMT_NV61M: l.planes = 2; /* fall through */
    case V4L2_PIX_FMT_NV16: case V4L2_PIX_FMT_NV61:
        l.yuv = true; l.components = 2; l.pixel_bytes = 1;
        l.native.format |= PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR | PISP_IMAGE_FORMAT_SAMPLING_422;
        l.swap_uv = fourcc == V4L2_PIX_FMT_NV61 || fourcc == V4L2_PIX_FMT_NV61M;
        break;
    case V4L2_PIX_FMT_YUV420M: case V4L2_PIX_FMT_YVU420M: l.planes = 3; /* fall through */
    case V4L2_PIX_FMT_YUV420: case V4L2_PIX_FMT_YVU420:
        l.yuv = true; l.components = 3; l.pixel_bytes = 1; l.xsub = l.ysub = 2; l.alignment = 128;
        l.native.format |= PISP_IMAGE_FORMAT_PLANARITY_PLANAR | PISP_IMAGE_FORMAT_SAMPLING_420;
        l.swap_uv = fourcc == V4L2_PIX_FMT_YVU420 || fourcc == V4L2_PIX_FMT_YVU420M;
        break;
    case V4L2_PIX_FMT_YUV422M: case V4L2_PIX_FMT_YVU422M: l.planes = 3; /* fall through */
    case V4L2_PIX_FMT_YUV422P:
        l.yuv = true; l.components = 3; l.pixel_bytes = 1; l.xsub = 2; l.alignment = 128;
        l.native.format |= PISP_IMAGE_FORMAT_PLANARITY_PLANAR | PISP_IMAGE_FORMAT_SAMPLING_422;
        l.swap_uv = fourcc == V4L2_PIX_FMT_YVU422M;
        break;
    case V4L2_PIX_FMT_YUV444M: case V4L2_PIX_FMT_YVU444M:
        l.yuv = true; l.planes = l.components = 3; l.pixel_bytes = 1;
        l.native.format |= PISP_IMAGE_FORMAT_PLANARITY_PLANAR;
        l.swap_uv = fourcc == V4L2_PIX_FMT_YVU444M;
        break;
    default: return -ENOTSUP;
    }
    *out = l;
    return 0;
}

static int capacities(const cc_format *f, const struct cc_pisp_output_layout *l,
                       uint64_t sizes[3], uint32_t *stride2)
{
    uint32_t s0 = f->planes[0].stride, s2 = 0;
    if (s0 > INT32_MAX || s0 % 16 || s0 < f->width * l->pixel_bytes)
        return -EINVAL;
    sizes[0] = (uint64_t)s0 * f->height;
    sizes[1] = sizes[2] = 0;
    if (l->components > 1) {
        s2 = l->planes > 1 ? f->planes[1].stride : s0 / l->xsub;
        if (s2 > INT32_MAX || s2 % 16 || s2 < f->width / l->xsub ||
            (l->planes == 1 && s0 % l->xsub) ||
            (l->planes == 3 && f->planes[2].stride != s2))
            return -EINVAL;
        uint64_t chroma = (uint64_t)s2 * (f->height / l->ysub);
        if (l->planes == 1) sizes[0] += (l->components - 1U) * chroma;
        else {
            sizes[1] = chroma;
            if (l->planes == 3) sizes[2] = chroma;
        }
    }
    *stride2 = s2;
    return 0;
}

int cc_pisp_output_layout(const cc_format *format, struct cc_pisp_output_layout *layout)
{
    if (!format || !layout) return -EINVAL;
    struct cc_pisp_output_layout l;
    int ret = describe(format->fourcc, &l);
    if (ret < 0) return ret;
    if (!format->width || !format->height || format->width > UINT16_MAX || format->height > UINT16_MAX ||
        (l.yuv && (l.native.format & PISP_IMAGE_FORMAT_SAMPLING_MASK) && (format->width & 1U)) ||
        (l.ysub == 2 && (format->height & 1U)) || format->num_planes != l.planes)
        return -EINVAL;
    uint64_t sizes[3];
    uint32_t stride2;
    ret = capacities(format, &l, sizes, &stride2);
    if (ret < 0) return ret;
    for (uint32_t i = 0; i < l.planes; ++i)
        if (sizes[i] > format->planes[i].size)
            return -EINVAL;
    l.native.width = (uint16_t)format->width;
    l.native.height = (uint16_t)format->height;
    l.native.stride = (int32_t)format->planes[0].stride;
    l.native.stride2 = (int32_t)stride2;
    *layout = l;
    return 0;
}

int cc_pisp_prepare_output(cc_format *format)
{
    if (!format) return -EINVAL;
    struct cc_pisp_output_layout l;
    int ret = describe(format->fourcc, &l);
    if (ret < 0) return ret;
    if (!format->width || !format->height || format->width > UINT16_MAX || format->height > UINT16_MAX ||
        (l.yuv && (l.native.format & PISP_IMAGE_FORMAT_SAMPLING_MASK) && (format->width & 1U)) ||
        (l.ysub == 2 && (format->height & 1U)))
        return -EINVAL;
    if (format->num_planes && format->num_planes != l.planes) return -EINVAL;
    format->num_planes = l.planes;
    for (uint32_t i = 0; i < l.planes; ++i) {
        uint32_t minimum = !i ? format->width * l.pixel_bytes : format->width / l.xsub;
        uint32_t alignment = format->planes[i].stride ? 16 : 64;
        if (!i && l.planes == 1 && l.components == 3 && l.xsub == 2) alignment *= 2;
        uint32_t stride = format->planes[i].stride;
        if (stride < minimum) stride = minimum;
        if (stride > INT32_MAX - alignment + 1U) return -ERANGE;
        format->planes[i].stride = (stride + alignment - 1U) / alignment * alignment;
    }
    if (l.planes == 3) {
        uint32_t stride = format->planes[1].stride;
        if (stride < format->planes[2].stride) stride = format->planes[2].stride;
        format->planes[1].stride = format->planes[2].stride = stride;
    }
    uint64_t sizes[3];
    uint32_t stride2;
    ret = capacities(format, &l, sizes, &stride2);
    if (ret < 0) return ret;
    for (uint32_t i = 0; i < l.planes; ++i) {
        if (sizes[i] > UINT32_MAX) return -ERANGE;
        if (format->planes[i].size < sizes[i]) format->planes[i].size = (uint32_t)sizes[i];
    }
    if (!format->field) format->field = V4L2_FIELD_NONE;
    if (!format->colorspace) format->colorspace = l.yuv ? V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_SRGB;
    if (!format->ycbcr_enc) format->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(format->colorspace);
    if (!format->quantization)
        format->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(!l.yuv, format->colorspace, format->ycbcr_enc);
    if (!format->xfer_func) format->xfer_func = V4L2_XFER_FUNC_NONE;
    return 0;
}
