/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Frontend finalization equations adapted from libpisp frontend.cpp v1.5.0.
 * Copyright (C) 2021-2023 Raspberry Pi Ltd. Original notices retained in
 * LICENSES/BSD-2-Clause.txt. This C17 port requires no libpisp library at
 * build/runtime; all invalid geometry is reported as an errno. */
#include "internal.h"
#include "pisp_format.h"

#include <errno.h>
#include <limits.h>
#include <linux/media-bus-format.h>
#include <string.h>

uint32_t cc_cfe_output_fourcc(uint32_t fourcc)
{
    switch (cc_bayer_mbus(fourcc)) {
#define ORDER(order) \
    case MEDIA_BUS_FMT_S##order##8_1X8: \
    case MEDIA_BUS_FMT_S##order##10_1X10: \
    case MEDIA_BUS_FMT_S##order##12_1X12: \
    case MEDIA_BUS_FMT_S##order##14_1X14: \
    case MEDIA_BUS_FMT_S##order##16_1X16: return V4L2_PIX_FMT_S##order##16
    ORDER(RGGB);
    ORDER(GRBG);
    ORDER(GBRG);
    ORDER(BGGR);
#undef ORDER
    case MEDIA_BUS_FMT_Y8_1X8:
    case MEDIA_BUS_FMT_Y10_1X10:
    case MEDIA_BUS_FMT_Y12_1X12:
    case MEDIA_BUS_FMT_Y14_1X14:
    case MEDIA_BUS_FMT_Y16_1X16: return V4L2_PIX_FMT_Y16;
    default: return 0;
    }
}

static bool region_valid(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t width, uint32_t height)
{
    return w && h && !((x | y | w | h) & 1U) && (uint64_t)x + w <= width && (uint64_t)y + h <= height;
}

static int grid(uint16_t *x, uint16_t *y, uint16_t *sx, uint16_t *sy,
                 uint32_t columns, uint32_t rows, uint32_t width, uint32_t height)
{
    if (((uint32_t)*x | *y) & 1U || (uint32_t)*x * 2 >= width || (uint32_t)*y * 2 >= height)
        return -ERANGE;
    if (!*sx) *sx = (uint16_t)(((width - 2U * *x) / columns) & ~1U);
    if (!*sy) *sy = (uint16_t)(((height - 2U * *y) / rows) & ~1U);
    if (!*sx || !*sy || ((uint32_t)*sx | *sy) & 1U ||
        (uint64_t)*x + columns * *sx > width || (uint64_t)*y + rows * *sy > height)
        return -ERANGE;
    return 0;
}

/* Packed UAPI members must not be passed by address to grid(). */
#define FINALIZE_GRID(block, columns, rows) do { \
    uint16_t x = c.block.offset_x, y = c.block.offset_y, sx = c.block.size_x, sy = c.block.size_y; \
    int ret = grid(&x, &y, &sx, &sy, columns, rows, width, height); \
    if (ret < 0) return ret; \
    c.block.offset_x = x; c.block.offset_y = y; c.block.size_x = sx; c.block.size_y = sy; \
} while (0)

static uint16_t decimated(uint16_t value) { return (uint16_t)(((uint32_t)value + 2U) / 4U * 2U); }

int cc_cfe_geometry(const cc_cfe_configuration *c, uint32_t output,
                    struct pisp_fe_output_branch_config *ch, uint32_t *enables)
{
    if (!c || output > 1 || !ch || !enables) return -EINVAL;
    uint32_t width = c->sensor.width, height = c->sensor.height;
    if (width < 64 || height < 64 || width > 16384 || height > 16384 || (width | height) & 1U)
        return -EINVAL;
    *ch = (struct pisp_fe_output_branch_config) {0};
    cc_rectangle r = c->crop[output];
    if (r.x || r.y || r.width || r.height) {
        if (r.x < 0 || r.y < 0 || !region_valid((uint32_t)r.x, (uint32_t)r.y,
                r.width, r.height, width, height)) return -ERANGE;
        ch->crop = (struct pisp_fe_crop_config) { (uint16_t)r.x, (uint16_t)r.y,
            (uint16_t)r.width, (uint16_t)r.height };
        width = r.width; height = r.height;
        *enables |= (uint32_t)PISP_FE_ENABLE_CROP(output);
    }
    const struct pisp_fe_downscale_config *d = &c->downscale[output];
    if (d->flags & ~(uint32_t)(DOWNSCALE_BAYER | DOWNSCALE_BIN)) return -EINVAL;
    if (d->xin || d->xout || d->yin || d->yout) {
        bool mono = cc_cfe_output_fourcc(c->sensor.fourcc) == V4L2_PIX_FMT_Y16;
        if (!d->xin || !d->xout || !d->yin || !d->yout ||
            d->xin > 31 || d->yin > 31 || d->xout > d->xin || d->yout > d->yin ||
            d->xin > 16U * d->xout || d->yin > 16U * d->yout ||
            (mono && (d->xout > 16 || (d->flags & DOWNSCALE_BIN))) ||
            width > (output ? 4096U : 6144U)) return -ERANGE;
        ch->downscale = (struct pisp_fe_downscale_config) {
            .xin = d->xin, .xout = d->xout, .yin = d->yin, .yout = d->yout,
            .flags = (uint8_t)(mono ? 0 : DOWNSCALE_BAYER | (d->flags & DOWNSCALE_BIN)) };
        width = (width / 2U * d->xout / d->xin) * 2U;
        height = (height / 2U * d->yout / d->yin) * 2U;
        ch->downscale.output_width = (uint16_t)width;
        ch->downscale.output_height = (uint16_t)height;
        *enables |= (uint32_t)PISP_FE_ENABLE_DOWNSCALE(output);
    } else if (d->flags) return -EINVAL;
    if (width < 16 || height < 16) return -ERANGE;
    ch->output.format.width = (uint16_t)width;
    ch->output.format.height = (uint16_t)height;
    return 0;
}

int cc_cfe_build_parameters(const cc_cfe_configuration *configuration,
                            const struct pisp_fe_config *tuning, struct pisp_fe_config *parameters)
{
    if (!configuration || !parameters)
        return -EINVAL;
    const cc_format *image = &configuration->image;
    const cc_format *sensor = &configuration->sensor;
    uint32_t output_fourcc = cc_cfe_output_fourcc(sensor->fourcc);
    struct cc_pisp_raw_layout raw, expected;
    if (!output_fourcc || cc_pisp_raw_format(output_fourcc, &expected) < 0 ||
        cc_pisp_raw_format(image->fourcc, &raw) < 0 || raw.bayer_order != expected.bayer_order)
        return -ENOTSUP;
    int layout_ret = cc_pisp_raw_layout(image, &raw);
    if (layout_ret < 0) return layout_ret;
    if (configuration->compression_offset > UINT16_MAX || (!raw.compression && configuration->compression_offset))
        return -EINVAL;
    const uint32_t processing = PISP_FE_ENABLE_DECOMPAND | PISP_FE_ENABLE_BLA | PISP_FE_ENABLE_DPC |
        PISP_FE_ENABLE_STATS_CROP | PISP_FE_ENABLE_BLC | PISP_FE_ENABLE_CDAF_STATS |
        PISP_FE_ENABLE_AWB_STATS | PISP_FE_ENABLE_RGBY | PISP_FE_ENABLE_LSC | PISP_FE_ENABLE_AGC_STATS;
    struct pisp_fe_config c = {0};
    if (tuning) {
        if (tuning->global.enables & ~processing)
            return -EINVAL;
        c.global.enables = tuning->global.enables;
        c.decompand = tuning->decompand;
        c.bla = tuning->bla;
        c.dpc = tuning->dpc;
        c.stats_crop = tuning->stats_crop;
        c.blc = tuning->blc;
        c.rgby = tuning->rgby;
        c.lsc = tuning->lsc;
        c.agc_stats = tuning->agc_stats;
        c.awb_stats = tuning->awb_stats;
        c.cdaf_stats = tuning->cdaf_stats;
        c.floating_stats = tuning->floating_stats;
    } else {
        c.global.enables = PISP_FE_ENABLE_AGC_STATS | PISP_FE_ENABLE_AWB_STATS |
                           PISP_FE_ENABLE_CDAF_STATS | PISP_FE_ENABLE_RGBY;
        /* U10 BT.601 luminance coefficients, summing to unity. */
        c.rgby = (struct pisp_fe_rgby_config) { .gain_r = 306, .gain_g = 601, .gain_b = 117 };
        memset(c.agc_stats.weights, 0x11, sizeof(c.agc_stats.weights));
        c.awb_stats.r_hi = c.awb_stats.g_hi = c.awb_stats.b_hi = UINT16_MAX;
        c.cdaf_stats.mode = 0x15;
        c.floating_stats.regions[0].size_x = (uint16_t)sensor->width;
        c.floating_stats.regions[0].size_y = (uint16_t)sensor->height;
    }
    c.decompand.pad = 0;
    memset(c.bla.pad, 0, sizeof(c.bla.pad));
    memset(c.blc.pad, 0, sizeof(c.blc.pad));
    c.rgby.pad = c.lsc.pad0 = 0;
    memset(c.agc_stats.pad1, 0, sizeof(c.agc_stats.pad1));
    memset(c.awb_stats.pad, 0, sizeof(c.awb_stats.pad));
    if ((c.global.enables & PISP_FE_ENABLE_DPC) && c.dpc.flags & ~3U)
        return -ERANGE;
    c.global.enables |= PISP_FE_ENABLE_INPUT;
    c.global.bayer_order = (uint8_t)raw.bayer_order;
    c.input.streaming = 1;
    c.input.format = (struct pisp_image_format_config) { .width = (uint16_t)sensor->width,
        .height = (uint16_t)sensor->height, .format = PISP_IMAGE_FORMAT_BPS_16,
        .stride = (int32_t)((sensor->width * 2U + 63U) & ~63U) };
    for (uint32_t i = 0; i < (configuration->image1.fourcc ? 2U : 1U); ++i) {
        const cc_format *f = i ? &configuration->image1 : image;
        uint32_t offset = i ? configuration->compression_offset1 : configuration->compression_offset;
        struct cc_pisp_raw_layout l;
        int ret = cc_pisp_raw_layout(f, &l);
        if (ret < 0) return ret;
        if (l.bayer_order != expected.bayer_order) return -ENOTSUP;
        if (offset > UINT16_MAX || (!l.compression && offset)) return -EINVAL;
        uint32_t enables = c.global.enables;
        ret = cc_cfe_geometry(configuration, i, &c.ch[i], &enables);
        if (ret < 0) return ret;
        if (f->width != c.ch[i].output.format.width || f->height != c.ch[i].output.format.height)
            return -EINVAL;
        c.global.enables = enables | (uint32_t)PISP_FE_ENABLE_OUTPUT(i);
        c.ch[i].output.format = l.native;
        if (l.compression) {
            c.global.enables |= (uint32_t)PISP_FE_ENABLE_COMPRESS(i);
            c.ch[i].compress = (struct pisp_compress_config) {
                .offset = (uint16_t)offset, .mode = (uint8_t)l.compression };
        }
    }
    c.output_axi = (struct pisp_fe_output_axi_config) { .maxlen_flags = 0xaf, .cache_prot = 0x32,
        .qos = 0x8410, .thresh = 0x140, .throttle = 0x4100 };
    uint32_t width = sensor->width, height = sensor->height;
    if (c.global.enables & PISP_FE_ENABLE_STATS_CROP) {
        if (!region_valid(c.stats_crop.offset_x, c.stats_crop.offset_y, c.stats_crop.width,
                           c.stats_crop.height, width, height))
            return -ERANGE;
        width = c.stats_crop.width;
        height = c.stats_crop.height;
    }
    if (c.global.enables & PISP_FE_ENABLE_AGC_STATS) {
        FINALIZE_GRID(agc_stats, PISP_AGC_STATS_SIZE, PISP_AGC_STATS_SIZE);
        if ((c.agc_stats.row_offset_x | c.agc_stats.row_offset_y) & 1U ||
            2U * c.agc_stats.row_offset_x >= width || 2U * c.agc_stats.row_offset_y >= height)
            return -ERANGE;
        if (!c.agc_stats.row_size_x) c.agc_stats.row_size_x = (uint16_t)((width - 2U * c.agc_stats.row_offset_x) & ~1U);
        if (!c.agc_stats.row_size_y) {
            uint32_t rows = ((height - 2U * c.agc_stats.row_offset_y) / PISP_AGC_STATS_NUM_ROW_SUMS) & ~1U;
            c.agc_stats.row_size_y = (uint16_t)(rows < 2 ? 2 : rows);
        }
        if (!region_valid(c.agc_stats.row_offset_x, c.agc_stats.row_offset_y,
                           c.agc_stats.row_size_x, c.agc_stats.row_size_y, width, height))
            return -ERANGE;
    }
    if (c.global.enables & PISP_FE_ENABLE_AWB_STATS) {
        FINALIZE_GRID(awb_stats, PISP_AWB_STATS_SIZE, PISP_AWB_STATS_SIZE);
        if (c.awb_stats.r_lo > c.awb_stats.r_hi || c.awb_stats.g_lo > c.awb_stats.g_hi || c.awb_stats.b_lo > c.awb_stats.b_hi)
            return -ERANGE;
    }
    if (c.global.enables & PISP_FE_ENABLE_CDAF_STATS) {
        FINALIZE_GRID(cdaf_stats, PISP_CDAF_STATS_SIZE, PISP_CDAF_STATS_SIZE);
    }
    if (c.global.enables & PISP_FE_ENABLE_RGBY)
        if (c.rgby.gain_r > 16383 || c.rgby.gain_g > 16383 || c.rgby.gain_b > 16383 || c.rgby.maxflag > 1)
            return -ERANGE;
    if (c.global.enables & PISP_FE_ENABLE_LSC) {
        if (!c.lsc.centre_x) c.lsc.centre_x = (uint16_t)(width / 2);
        if (!c.lsc.centre_y) c.lsc.centre_y = (uint16_t)(height / 2);
        if (c.lsc.centre_x > width || c.lsc.centre_y > height)
            return -ERANGE;
        uint32_t dx = width - c.lsc.centre_x, dy = height - c.lsc.centre_y;
        if (dx < c.lsc.centre_x) dx = c.lsc.centre_x;
        if (dy < c.lsc.centre_y) dy = c.lsc.centre_y;
        uint32_t radius = dx * dx + dy * dy;
        if (!c.lsc.scale) {
            c.lsc.shift = 0;
            while (radius >= 2U * ((PISP_FE_LSC_LUT_SIZE - 1U) << 6)) { radius >>= 1; ++c.lsc.shift; }
            uint32_t scale = ((1024U * ((PISP_FE_LSC_LUT_SIZE - 1U) << 6)) - 1U) / radius;
            c.lsc.scale = (uint16_t)(scale >= 1024 ? 1023 : scale);
        }
        if (c.lsc.scale >= 1024 || c.lsc.shift > 31)
            return -ERANGE;
    }
    for (size_t i = 0; i < PISP_FLOATING_STATS_NUM_ZONES; ++i) {
        struct pisp_fe_floating_stats_region *r = &c.floating_stats.regions[i];
        if ((r->size_x || r->size_y) && !region_valid(r->offset_x, r->offset_y, r->size_x, r->size_y, width, height))
            return -ERANGE;
    }
    if (width > 12288)
        return -ERANGE;
    if (width > 6144) {
        c.global.enables |= PISP_FE_ENABLE_DECIMATE;
        c.lsc.centre_x = decimated(c.lsc.centre_x);
        c.lsc.centre_y = decimated(c.lsc.centre_y);
#define DECIMATE_GRID(block) do { \
    c.block.offset_x = decimated(c.block.offset_x); c.block.offset_y = decimated(c.block.offset_y); \
    c.block.size_x = decimated(c.block.size_x); c.block.size_y = decimated(c.block.size_y); \
} while (0)
        DECIMATE_GRID(agc_stats);
        DECIMATE_GRID(awb_stats);
        DECIMATE_GRID(cdaf_stats);
#undef DECIMATE_GRID
        c.agc_stats.row_offset_x = decimated(c.agc_stats.row_offset_x);
        c.agc_stats.row_offset_y = decimated(c.agc_stats.row_offset_y);
        c.agc_stats.row_size_x = decimated(c.agc_stats.row_size_x);
        c.agc_stats.row_size_y = decimated(c.agc_stats.row_size_y);
        c.cdaf_stats.skip_x = decimated(c.cdaf_stats.skip_x);
        c.cdaf_stats.skip_y = decimated(c.cdaf_stats.skip_y);
        for (size_t i = 0; i < PISP_FLOATING_STATS_NUM_ZONES; ++i) {
            c.floating_stats.regions[i].offset_x = decimated(c.floating_stats.regions[i].offset_x);
            c.floating_stats.regions[i].offset_y = decimated(c.floating_stats.regions[i].offset_y);
            c.floating_stats.regions[i].size_x = decimated(c.floating_stats.regions[i].size_x);
            c.floating_stats.regions[i].size_y = decimated(c.floating_stats.regions[i].size_y);
        }
    }
    /* Complete snapshots make every submitted job independent of prior tuning. */
    c.dirty_flags = processing | PISP_FE_ENABLE_INPUT | PISP_FE_ENABLE_DECIMATE |
                    PISP_FE_ENABLE_OUTPUT0 | PISP_FE_ENABLE_OUTPUT1 |
                    PISP_FE_ENABLE_CROP0 | PISP_FE_ENABLE_CROP1 | PISP_FE_ENABLE_DOWNSCALE0 |
                    PISP_FE_ENABLE_DOWNSCALE1 | PISP_FE_ENABLE_COMPRESS0 | PISP_FE_ENABLE_COMPRESS1;
    c.dirty_flags_extra = PISP_FE_DIRTY_GLOBAL | PISP_FE_DIRTY_FLOATING | PISP_FE_DIRTY_OUTPUT_AXI;
    *parameters = c;
    return 0;
}
