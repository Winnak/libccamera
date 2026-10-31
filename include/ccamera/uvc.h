/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_UVC_H
#define CCAMERA_UVC_H
#include <ccamera/ccamera.h>
#ifdef __cplusplus
extern "C" {
#endif

enum cc_uvc_control_index {
    CC_UVC_CONTROL_BRIGHTNESS, CC_UVC_CONTROL_CONTRAST, CC_UVC_CONTROL_SATURATION,
    CC_UVC_CONTROL_EXPOSURE_MODE, CC_UVC_CONTROL_EXPOSURE_TIME, CC_UVC_CONTROL_ANALOGUE_GAIN,
    CC_UVC_CONTROL_GAMMA, CC_UVC_CONTROL_COUNT
};
enum cc_uvc_control_flags {
    CC_UVC_BRIGHTNESS=1U, CC_UVC_CONTRAST=2U, CC_UVC_SATURATION=4U,
    CC_UVC_EXPOSURE_MODE=8U, CC_UVC_EXPOSURE_TIME=16U, CC_UVC_ANALOGUE_GAIN=32U, CC_UVC_GAMMA=64U
};
enum cc_uvc_exposure_mode { CC_UVC_EXPOSURE_AUTO, CC_UVC_EXPOSURE_MANUAL };
typedef struct cc_uvc_controls {
    uint32_t flags;
    double brightness, contrast, saturation, analogue_gain, gamma;
    uint64_t exposure_us; /* Native UVC exposure units are 100 us. */
    enum cc_uvc_exposure_mode exposure_mode;
} cc_uvc_controls;
typedef struct cc_uvc_profile {
    uint32_t flags; /* Supported/writable controls, using cc_uvc_control_flags. */
    uint32_t exposure_menu; /* Bits for the native V4L2_EXPOSURE_* menu indices 0..3. */
    cc_control_info native[CC_UVC_CONTROL_COUNT]; /* Indexed by cc_uvc_control_index. */
} cc_uvc_profile;
typedef struct cc_uvc_range { double minimum, maximum, default_value; } cc_uvc_range;
struct cc_request;

/* Read the current native ranges/menu on an actual uvcvideo capture node.
 * Missing/disabled/read-only or unrepresentable controls are omitted. Inactive
 * manual exposure controls may be used after selecting manual exposure mode.
 * Output stays unchanged on error. No device writes or long-lived cache. */
CC_API int cc_camera_get_uvc_profile(cc_camera *camera,cc_uvc_profile *profile);
/* Normalized ranges: brightness default=0, contrast/gain/saturation default=1,
 * gamma in linear units, exposure in us. Mode range uses the enum above; the
 * native exposure_menu determines which choices exist. No I/O. */
CC_API int cc_uvc_get_control_range(const cc_uvc_profile *profile,enum cc_uvc_control_index control,cc_uvc_range *range);
/* Upstream UVC control mappings, with checked binary64 arithmetic and no libm.
 * Selected floating values must be finite. Values saturate to native bounds;
 * integer rounding is followed by downward min-based native step quantization.
 * Exposure truncates to 100 us units before native step quantization. Auto mode
 * prefers AUTO then APERTURE_PRIORITY; manual prefers SHUTTER_PRIORITY then
 * MANUAL, as in the reference. *count is capacity/required; NULL queries count.
 * No I/O; outputs unchanged on rejection except required count on -ENOSPC. */
CC_API int cc_uvc_controls_to_native(const cc_uvc_profile *profile,const cc_uvc_controls *settings,
                                    cc_control *controls,size_t *count);
/* Immediate write, before graph adoption. Returns acknowledged normalized
 * values for selected controls. No exposure-delay compensation is implied.
 * Hardware may change partially on an error; settings remains unchanged. */
CC_API int cc_camera_set_uvc_controls(cc_camera *camera,cc_uvc_controls *settings);
/* Copied commands applied once after acquire fences and before source queuing.
 * NULL clears. Results retain acknowledged normalized values for this request;
 * reuse repeats commands. Only uvcvideo camera graphs accept these commands. */
CC_API int cc_request_set_uvc_controls(struct cc_request *request,const cc_uvc_controls *settings);

#ifdef __cplusplus
}
#endif
#endif
