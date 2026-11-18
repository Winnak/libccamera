/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PISP_NATIVE_H
#define CCAMERA_PISP_NATIVE_H
#include "vc4_profile.h"
#include <ccamera/pisp_tuning.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
/* Native register policy. history_frames counts successfully completed frames
 * in the SAME exposure channel. No CPU access to image buffers. */
void cc_pisp_native_denoise(const struct cc_vc4_profile_data *profile, double gain, double mode_factor,
    double exposure_ratio, uint32_t history_frames, int hdr, struct pisp_be_config *backend);
void cc_pisp_native_hdr(const struct cc_vc4_profile_data *profile, double short_long_ratio,
    const struct pisp_fe_config *frontend, const struct pisp_statistics *statistics,
    struct pisp_be_config *backend);
#endif
