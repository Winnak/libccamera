/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PHOTO_PROCESSING_H
#define CCAMERA_PHOTO_PROCESSING_H
#include <ccamera/ipa.h>
#include "vc4_profile.h"
cc_photo_controls cc_photo_defaults(const cc_ipa_vc4_options *options);
cc_photo_controls cc_photo_resolve(const cc_ipa_vc4_options *options, const cc_photo_controls *controls);
double cc_photo_ev_multiplier(double ev);
void cc_photo_apply_vc4(const cc_photo_controls *controls, cc_vc4_tuning *tuning);
void cc_photo_apply_pisp(const struct cc_vc4_profile_data *profile, const cc_photo_controls *controls,
    struct pisp_fe_config *frontend, struct pisp_be_config *backend);
#endif
