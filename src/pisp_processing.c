/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Native block copying/padding and grid finalization adapted from libpisp
 * v1.5.0 backend.cpp/backend_prepare.cpp. Copyright (C) 2021-2023 Raspberry Pi Ltd.
 * See LICENSES/BSD-2-Clause.txt. Register limits follow the PiSP specification,
 * sections 7.5.13-7.5.35. This file does not infer sensor tuning or colour curves. */
#include "pisp_processing.h"

#include <errno.h>
#include <limits.h>
#include <string.h>

static int grid_step(uint32_t step, uint32_t size, uint16_t offset, uint32_t extent)
{
    if (!step) step = extent / size;
    if (!step || step > UINT16_MAX || (uint64_t)step * (size + offset - 1U) >= extent)
        return -ERANGE;
    return (int)step;
}

static int grids(struct pisp_be_config *c, uint32_t width, uint32_t height)
{
    if (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_LSC) {
        const uint32_t extent = PISP_BE_LSC_GRID_SIZE << PISP_BE_LSC_STEP_PRECISION;
        int x = grid_step(c->lsc.grid_step_x, width, c->lsc_extra.offset_x, extent);
        int y = grid_step(c->lsc.grid_step_y, height, c->lsc_extra.offset_y, extent);
        if (x < 0 || y < 0) return -ERANGE;
        c->lsc.grid_step_x = (uint16_t)x; c->lsc.grid_step_y = (uint16_t)y;
    }
    if (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC) {
        const uint32_t extent = PISP_BE_CAC_GRID_SIZE << PISP_BE_CAC_STEP_PRECISION;
        int x = grid_step(c->cac.grid_step_x, width, c->cac_extra.offset_x, extent);
        int y = grid_step(c->cac.grid_step_y, height, c->cac_extra.offset_y, extent);
        if (x < 0 || y < 0) return -ERANGE;
        c->cac.grid_step_x = (uint16_t)x; c->cac.grid_step_y = (uint16_t)y;
        for (size_t row = 0; row < PISP_BE_CAC_LUT_SIZE; ++row)
            for (size_t col = 0; col < PISP_BE_CAC_LUT_SIZE; ++col)
                for (size_t rb = 0; rb < 2; ++rb)
                    for (size_t xy = 0; xy < 2; ++xy)
                        if (c->cac.lut[row][col][rb][xy] < -64 || c->cac.lut[row][col][rb][xy] > 63)
                            return -ERANGE;
    }
    return 0;
}

static int matrix(const struct pisp_be_ccm_config *m, uint32_t hardware_version)
{
    /* C0 has 27-bit signed offsets. D0 extends these to 29 bits. Unknown/zero
     * revision uses the common C0 range; the builder rejects unknown nonzero. */
    int32_t limit = hardware_version == CC_PISP_BE_VERSION_BCM2712_D0 ? INT32_C(268435456) : INT32_C(67108864);
    for (size_t i = 0; i < 9; ++i)
        if (m->coeffs[i] < -8192 || m->coeffs[i] > 8191) return -ERANGE;
    for (size_t i = 0; i < 3; ++i)
        if (m->offsets[i] < -limit || m->offsets[i] >= limit) return -ERANGE;
    return 0;
}

static int sharpen(struct pisp_be_sharpen_config *s)
{
    const int8_t *kernels[] = {s->kernel0, s->kernel1, s->kernel2, s->kernel3, s->kernel4};
    const uint16_t slopes[] = {s->threshold_slope0, s->threshold_slope1, s->threshold_slope2,
                              s->threshold_slope3, s->threshold_slope4};
    const uint16_t scales[] = {s->scale0, s->scale1, s->scale2, s->scale3, s->scale4};
    if ((s->enables | s->white | s->black) & ~31U || s->positive_strength > 4095 || s->negative_strength > 4095)
        return -ERANGE;
    for (size_t i = 0; i < 5; ++i) {
        int positive = 0, negative = 0;
        if (slopes[i] > 4095 || scales[i] > 4095) return -ERANGE;
        for (size_t j = 0; j < PISP_BE_SHARPEN_SIZE * PISP_BE_SHARPEN_SIZE; ++j) {
            int coefficient = kernels[i][j];
            if (coefficient < -32 || coefficient > 31) return -ERANGE;
            if (coefficient > 0) positive += coefficient;
            else negative += coefficient;
        }
        if (positive > 31 || negative < -32) return -ERANGE;
    }
    return 0;
}

static void padding(struct pisp_be_config *c)
{
    c->dpc.pad = c->sdn.pad = c->sat_control.pad = c->sh_fc_combine.pad = 0;
    memset(c->blc.pad, 0, sizeof(c->blc.pad));
    memset(c->wbg.pad, 0, sizeof(c->wbg.pad));
    memset(c->debin.pad, 0, sizeof(c->debin.pad));
    memset(c->demosaic.pad, 0, sizeof(c->demosaic.pad));
    memset(c->ccm.pad, 0, sizeof(c->ccm.pad));
    memset(c->ycbcr.pad, 0, sizeof(c->ycbcr.pad));
    memset(c->ycbcr_inverse.pad, 0, sizeof(c->ycbcr_inverse.pad));
    memset(c->false_colour.pad, 0, sizeof(c->false_colour.pad));
    memset(c->sharpen.pad0, 0, sizeof(c->sharpen.pad0));
    memset(c->sharpen.pad1, 0, sizeof(c->sharpen.pad1));
    memset(c->sharpen.pad2, 0, sizeof(c->sharpen.pad2));
    memset(c->sharpen.pad3, 0, sizeof(c->sharpen.pad3));
    memset(c->sharpen.pad4, 0, sizeof(c->sharpen.pad4));
    c->sharpen.pad5 = c->sharpen.pad6 = c->sharpen.pad7 = c->sharpen.pad8 = c->sharpen.pad9 = 0;
}

int cc_pisp_processing(struct pisp_be_config *c, const struct pisp_be_config *tuning,
                        uint32_t width, uint32_t height, uint32_t hardware_version, bool mono)
{
    const uint32_t bayer = PISP_BE_BAYER_ENABLE_DPC | PISP_BE_BAYER_ENABLE_GEQ | PISP_BE_BAYER_ENABLE_SDN |
        PISP_BE_BAYER_ENABLE_BLC | PISP_BE_BAYER_ENABLE_WBG | PISP_BE_BAYER_ENABLE_CDN | PISP_BE_BAYER_ENABLE_LSC |
        PISP_BE_BAYER_ENABLE_TONEMAP | PISP_BE_BAYER_ENABLE_CAC | PISP_BE_BAYER_ENABLE_DEBIN |
        PISP_BE_BAYER_ENABLE_TDN | PISP_BE_BAYER_ENABLE_STITCH;
    const uint32_t rgb = PISP_BE_RGB_ENABLE_CCM | PISP_BE_RGB_ENABLE_SAT_CONTROL | PISP_BE_RGB_ENABLE_YCBCR |
        PISP_BE_RGB_ENABLE_FALSE_COLOUR | PISP_BE_RGB_ENABLE_SHARPEN | PISP_BE_RGB_ENABLE_YCBCR_INVERSE |
        PISP_BE_RGB_ENABLE_GAMMA | PISP_BE_RGB_ENABLE_RESAMPLE0 | PISP_BE_RGB_ENABLE_RESAMPLE1;
    const uint32_t ycbcr = PISP_BE_RGB_ENABLE_YCBCR | PISP_BE_RGB_ENABLE_YCBCR_INVERSE;
    if (tuning) {
        if (tuning->global.bayer_enables & ~bayer || tuning->global.rgb_enables & ~rgb) return -EINVAL;
        c->global.bayer_enables = tuning->global.bayer_enables;
        c->global.rgb_enables = tuning->global.rgb_enables;
        c->dpc = tuning->dpc; c->geq = tuning->geq; c->sdn = tuning->sdn;
        c->blc = tuning->blc; c->wbg = tuning->wbg; c->cdn = tuning->cdn;
        c->lsc = tuning->lsc; c->lsc_extra = tuning->lsc_extra;
        c->cac = tuning->cac; c->cac_extra = tuning->cac_extra;
        c->debin = tuning->debin; c->tonemap = tuning->tonemap;
        c->tdn = tuning->tdn; c->stitch = tuning->stitch;
        c->demosaic = tuning->demosaic; c->ccm = tuning->ccm;
        c->sat_control = tuning->sat_control;
        c->ycbcr = tuning->ycbcr; c->ycbcr_inverse = tuning->ycbcr_inverse;
        c->false_colour = tuning->false_colour; c->sharpen = tuning->sharpen;
        c->sh_fc_combine = tuning->sh_fc_combine; c->gamma = tuning->gamma;
    } else c->demosaic = (struct pisp_be_demosaic_config) { .sharper = 8, .fc_mode = 1 };
    padding(c);
    c->tdn.pad = c->stitch.pad = 0;
    uint32_t be = c->global.bayer_enables, re = c->global.rgb_enables;
    /* Sharpen/false-colour operate in YCbCr, then the common gamma/output path
     * expects RGB again. Matrices are caller-supplied, independent of output CSC. */
    if (((re & ycbcr) && (re & ycbcr) != ycbcr) ||
        ((re & (PISP_BE_RGB_ENABLE_FALSE_COLOUR | PISP_BE_RGB_ENABLE_SHARPEN)) && (re & ycbcr) != ycbcr))
        return -EINVAL;
    if (!mono && c->demosaic.fc_mode > 2) return -ERANGE;
    if ((be & PISP_BE_BAYER_ENABLE_TDN) && c->tdn.reset > 1) return -ERANGE;
    if (be & PISP_BE_BAYER_ENABLE_STITCH) {
        if (c->stitch.threshold_diff_power > 15 || !(c->stitch.exposure_ratio & 32767U)) return -ERANGE;
        uint32_t threshold = c->stitch.motion_threshold_256;
        if (!c->stitch.motion_threshold_recip) {
            uint32_t reciprocal = threshold ? (256U + threshold - 1U) / threshold : 255U;
            c->stitch.motion_threshold_recip = (uint8_t)(reciprocal > 255U ? 255U : reciprocal);
        }
        if (threshold && c->stitch.motion_threshold_recip > 255U / threshold + 1U) return -ERANGE;
    }
    if ((be & PISP_BE_BAYER_ENABLE_DPC) &&
        (c->dpc.coeff_level > 31 || c->dpc.coeff_range > 31 || c->dpc.flags & ~1U)) return -ERANGE;
    if ((be & PISP_BE_BAYER_ENABLE_GEQ) &&
        (c->geq.slope_sharper & ~(PISP_BE_GEQ_SLOPE | PISP_BE_GEQ_SHARPER) || c->geq.min > c->geq.max)) return -ERANGE;
    if ((be & PISP_BE_BAYER_ENABLE_DEBIN) &&
        ((c->debin.h_enable != 0 && c->debin.h_enable != 1) ||
         (c->debin.v_enable != 0 && c->debin.v_enable != 1))) return -ERANGE;
    if ((be & PISP_BE_BAYER_ENABLE_TONEMAP) &&
        (c->tonemap.iir_strength > 4095 || c->tonemap.strength > 4095)) return -ERANGE;
    int ret = grids(c, width, height);
    if (ret < 0) return ret;
    if ((re & PISP_BE_RGB_ENABLE_CCM) && matrix(&c->ccm, hardware_version)) return -ERANGE;
    if ((re & ycbcr) && (matrix(&c->ycbcr, hardware_version) || matrix(&c->ycbcr_inverse, hardware_version)))
        return -ERANGE;
    if ((re & PISP_BE_RGB_ENABLE_SAT_CONTROL) &&
        (c->sat_control.shift_r > 2 || c->sat_control.shift_g > 2 || c->sat_control.shift_b > 2)) return -ERANGE;
    if ((re & PISP_BE_RGB_ENABLE_FALSE_COLOUR) && c->false_colour.distance != 1 && c->false_colour.distance != 2)
        return -ERANGE;
    if ((re & PISP_BE_RGB_ENABLE_SHARPEN) && sharpen(&c->sharpen)) return -ERANGE;
    if (re & PISP_BE_RGB_ENABLE_GAMMA)
        for (size_t i = 0; i < PISP_BE_GAMMA_LUT_SIZE; ++i)
            if (c->gamma.lut[i] & UINT32_C(0xc0000000)) return -ERANGE;
    c->global.bayer_enables |= PISP_BE_BAYER_ENABLE_INPUT;
    if (!mono) c->global.bayer_enables |= PISP_BE_BAYER_ENABLE_DEMOSAIC;
    else c->demosaic = (struct pisp_be_demosaic_config) {0};
    c->global.rgb_enables |= PISP_BE_RGB_ENABLE_OUTPUT0;
    c->dirty_flags_bayer = bayer | PISP_BE_BAYER_ENABLE_INPUT | PISP_BE_BAYER_ENABLE_DEMOSAIC;
    c->dirty_flags_rgb = rgb | PISP_BE_RGB_ENABLE_OUTPUT0 | PISP_BE_RGB_ENABLE_CSC0;
    c->dirty_flags_extra = PISP_BE_DIRTY_GLOBAL | PISP_BE_DIRTY_SH_FC_COMBINE;
    return 0;
}
