/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Copyright (C) 2021-2023 Raspberry Pi Ltd. Resize equations adapted from
 * libpisp v1.5.0 backend_prepare.cpp. See LICENSES/BSD-2-Clause.txt. */
#include "pisp_tiling.h"
#include "pisp_filters.h"

#include <errno.h>
#include <string.h>

int cc_pisp_crop(const cc_format *input, const cc_rectangle *requested, cc_rectangle *crop)
{
    cc_rectangle r = *requested;
    if (!r.width && !r.height && !r.x && !r.y)
        r = (cc_rectangle) { 0, 0, input->width, input->height };
    if (r.x < 0 || r.y < 0 || r.width < 16 || r.height < 16 ||
        (uint64_t)(uint32_t)r.x + r.width > input->width ||
        (uint64_t)(uint32_t)r.y + r.height > input->height)
        return -EINVAL;
    *crop = r;
    return 0;
}

static uint32_t downscaled(uint32_t input, uint32_t output)
{
    if (2U * output >= input) return input;
    uint32_t size = 2U * output, minimum = (input + 7U) / 8U, maximum = input / 2U;
    if (size < minimum) size = minimum;
    if (size > maximum) size = maximum;
    return size;
}

static void filter(struct pisp_be_resample_config *resample, uint32_t iw, uint32_t ih, uint32_t ow, uint32_t oh)
{
    uint64_t nx = iw - 1U, dx = ow - 1U, ny = ih - 1U, dy = oh - 1U;
    if (nx * 10U > dx * 21U && nx * dy * 10U < ny * dx * 11U && ny * dx * 10U < nx * dy * 11U) {
        /* Polyphase trapezoidal filter. Rational arithmetic avoids libm and
         * floating-to-integer rounding differences at phase boundaries. */
        if (nx > 5U * dx) nx = 5U * dx;
        uint32_t taps = (uint32_t)((nx + dx - 1U) / dx);
        for (uint32_t phase = 0; phase < 16; ++phase) {
            resample->coef[phase * 6] = (int16_t)((1024U - phase * 64U) * dx / nx);
            uint64_t remaining = 16U * nx - dx * (16U - phase);
            for (uint32_t tap = 1; tap <= taps; ++tap) {
                uint64_t weight = remaining < 16U * dx ? remaining : 16U * dx;
                resample->coef[phase * 6 + tap] = (int16_t)(weight * 1024U / (16U * nx));
                remaining -= weight;
            }
        }
    } else {
        const int16_t *coefficients = nx * 2U <= dx ? cc_pisp_lanczos3 :
            nx <= dx * 2U ? cc_pisp_lanczos2 : cc_pisp_michel_netravali;
        memcpy(resample->coef, coefficients, sizeof(resample->coef));
    }
}

int cc_pisp_resize(struct pisp_be_config *c, const cc_rectangle crops[2],
                    const struct pisp_be_config *tuning, uint32_t branches)
{
    if (branches == 1 && (c->global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE1)) return -EINVAL;
    for (uint32_t i = 0; i < branches; ++i) {
        uint32_t iw = crops[i].width, ih = crops[i].height;
        uint32_t ow = c->output_format[i].image.width, oh = c->output_format[i].image.height;
        if (ow < 16 || oh < 16) return -EINVAL;
        /* BCM2712 C0/D0 have a trapezoidal downscaler on branch 1 only. */
        if (i == 1 && (2U * ow < iw || 2U * oh < ih)) {
            uint32_t w = downscaled(iw, ow), h = downscaled(ih, oh);
            uint32_t sx = iw * 4096U / w, sy = ih * 4096U / h;
            if ((sx != 4096 && (sx < 8192 || sx > 32768)) ||
                (sy != 4096 && (sy < 8192 || sy > 32768))) return -ERANGE;
            c->downscale[i] = (struct pisp_be_downscale_config) {
                .scale_factor_h = (uint16_t)sx, .scale_factor_v = (uint16_t)sy,
                .scale_recip_h = (uint16_t)(w * 4096U / iw), .scale_recip_v = (uint16_t)(h * 4096U / ih)
            };
            c->downscale_extra[i] = (struct pisp_be_downscale_extra) { (uint16_t)w, (uint16_t)h };
            c->global.rgb_enables |= (uint32_t)PISP_BE_RGB_ENABLE_DOWNSCALE(i);
            iw = w; ih = h;
        }
        bool custom = tuning && (tuning->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_RESAMPLE(i));
        if (custom || iw != ow || ih != oh) {
            uint32_t sx = (iw - 1U) * 4096U / (ow - 1U), sy = (ih - 1U) * 4096U / (oh - 1U);
            if (sx < 256 || sy < 256 || sx >= 65536 || sy >= 65536) return -ERANGE;
            struct pisp_be_resample_config *r = &c->resample[i];
            r->scale_factor_h = (uint16_t)sx; r->scale_factor_v = (uint16_t)sy;
            if (custom) {
                for (size_t n = 0; n < PISP_BE_RESAMPLE_FILTER_SIZE; ++n)
                    if (tuning->resample[i].coef[n] < -2048 || tuning->resample[i].coef[n] > 2047) return -ERANGE;
                memcpy(r->coef, tuning->resample[i].coef, sizeof(r->coef));
            } else filter(r, iw, ih, ow, oh);
            c->resample_extra[i].scaled_width = (uint16_t)ow;
            c->resample_extra[i].scaled_height = (uint16_t)oh;
            c->global.rgb_enables |= (uint32_t)PISP_BE_RGB_ENABLE_RESAMPLE(i);
        }
    }
    c->dirty_flags_rgb |= PISP_BE_RGB_ENABLE_DOWNSCALE0 | PISP_BE_RGB_ENABLE_DOWNSCALE1 |
                          PISP_BE_RGB_ENABLE_RESAMPLE0 | PISP_BE_RGB_ENABLE_RESAMPLE1;
    return 0;
}
