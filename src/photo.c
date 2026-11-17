/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/photo.h>
#include <errno.h>
int cc_photo_validate_controls(const cc_photo_controls *c)
{
    if (!c) return -EINVAL;
    if (c->flags & ~CC_PHOTO_ALL) return -ENOTSUP;
    if (((c->flags & CC_PHOTO_GAIN) && !(c->analogue_gain == 0 || (c->analogue_gain >= 1 && c->analogue_gain <= 64))) ||
        ((c->flags & CC_PHOTO_EV) && !(c->exposure_value >= -8 && c->exposure_value <= 8)) ||
        ((c->flags & CC_PHOTO_METERING) && (c->metering < CC_PHOTO_CENTRE_WEIGHTED || c->metering > CC_PHOTO_MATRIX)) ||
        ((c->flags & CC_PHOTO_WB) && (c->white_balance < CC_PHOTO_WB_AUTO || c->white_balance > CC_PHOTO_WB_MANUAL ||
            (c->white_balance == CC_PHOTO_WB_MANUAL && !(c->red_gain >= 0.5 && c->red_gain <= 4 && c->blue_gain >= 0.5 && c->blue_gain <= 4)))) ||
        ((c->flags & CC_PHOTO_BRIGHTNESS) && !(c->brightness >= -1 && c->brightness <= 1)) ||
        ((c->flags & CC_PHOTO_CONTRAST) && !(c->contrast >= 0 && c->contrast <= 4)) ||
        ((c->flags & CC_PHOTO_SATURATION) && !(c->saturation >= 0 && c->saturation <= 2)) ||
        ((c->flags & CC_PHOTO_SHARPNESS) && !(c->sharpness >= 0 && c->sharpness <= 4)) ||
        ((c->flags & CC_PHOTO_DENOISE) && (c->denoise < CC_PHOTO_DENOISE_OFF || c->denoise > CC_PHOTO_DENOISE_NORMAL)) ||
        ((c->flags & CC_PHOTO_FLICKER) && c->mains_hz && c->mains_hz != 50 && c->mains_hz != 60) ||
        ((c->flags & CC_PHOTO_FRAME_DURATION) && ((!c->min_frame_duration_ns != !c->max_frame_duration_ns) ||
            c->min_frame_duration_ns > c->max_frame_duration_ns))) return -EINVAL;
    return 0;
}
