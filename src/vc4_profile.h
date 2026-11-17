/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_VC4_PROFILE_H
#define CCAMERA_VC4_PROFILE_H
#include <ccamera/vc4_tuning.h>
#include <ccamera/af.h>
struct cc_vc4_prior { double lux; const double *pwl; size_t count; };
struct cc_vc4_profile_data {
    cc_vc4_profile_info info;
    uint16_t black;
    const double *gamma;
    size_t num_gamma;
    const double (*ccm)[10];
    size_t num_ccm;
    const double *lsc_ct, *lsc_r, *lsc_b, *lsc_y;
    size_t num_lsc, lsc_width, lsc_height;
    double luminance_strength, noise_constant, noise_slope, geq_offset, geq_slope;
    double sdn_deviation, sdn_strength, sharpen_threshold, sharpen_strength, sharpen_limit;
    double cdn_deviation, cdn_strength;
    const double (*ct)[3];
    size_t num_ct;
    const struct cc_vc4_prior *priors;
    size_t num_priors;
    double sensitivity_r, sensitivity_b, transverse_pos, transverse_neg;
    uint32_t bayes;
    const double *shutter, *gain, *target, *weights;
    size_t num_exposure, num_target;
    double base_ev, speed, constraint_target;
    double lux_shutter, lux_gain, lux_y, lux_reference;
    const double *metering[3]; /* centre, spot, matrix, in native region order. */
    double wb_modes[7][2]; /* auto, incandescent, tungsten, fluorescent, indoor, daylight, cloudy. */
    const cc_af_tuning *af;
};
const struct cc_vc4_profile_data *cc_vc4_profile_data(enum cc_vc4_profile profile);
const struct cc_vc4_profile_data *cc_pisp_profile_data(enum cc_vc4_profile profile);
int cc_sensor_geometry_valid(const cc_sensor_geometry *geometry);
#endif
