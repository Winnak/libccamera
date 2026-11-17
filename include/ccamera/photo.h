/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PHOTO_H
#define CCAMERA_PHOTO_H
#include <ccamera/sensor.h>
#include <ccamera/af.h>
#include <ccamera/vc4_tuning.h>
#ifdef __cplusplus
extern "C" {
#endif
enum cc_photo_control_flags {
    CC_PHOTO_EXPOSURE = 1U, CC_PHOTO_GAIN = 2U, CC_PHOTO_EV = 4U,
    CC_PHOTO_METERING = 8U, CC_PHOTO_WB = 16U, CC_PHOTO_BRIGHTNESS = 32U,
    CC_PHOTO_CONTRAST = 64U, CC_PHOTO_SATURATION = 128U, CC_PHOTO_SHARPNESS = 256U,
    CC_PHOTO_DENOISE = 512U, CC_PHOTO_FLICKER = 1024U, CC_PHOTO_FRAME_DURATION = 2048U
};
#define CC_PHOTO_ALL 4095U
enum cc_photo_metering { CC_PHOTO_CENTRE_WEIGHTED, CC_PHOTO_SPOT, CC_PHOTO_MATRIX };
/* Presets constrain automatic WB to the module's calibrated temperature range.
 * NoIR modules support AUTO (grey-world) and MANUAL only. */
enum cc_photo_white_balance {
    CC_PHOTO_WB_AUTO, CC_PHOTO_WB_INCANDESCENT, CC_PHOTO_WB_TUNGSTEN,
    CC_PHOTO_WB_FLUORESCENT, CC_PHOTO_WB_INDOOR, CC_PHOTO_WB_DAYLIGHT,
    CC_PHOTO_WB_CLOUDY, CC_PHOTO_WB_MANUAL
};
enum cc_photo_denoise { CC_PHOTO_DENOISE_OFF, CC_PHOTO_DENOISE_NORMAL };
/* Per-request values, copied by capture; only flagged fields are read.
 * Unspecified fields use controller defaults on EVERY request (no sticky state).
 * Exposure/gain independently use 0 for automatic, >0 for manual. Native sensor
 * overrides take precedence. Times/gains quantize down to live native steps;
 * metadata reports driver-adjusted expected values for that frame.
 * Image adjustments use native ISP blocks, with no software post-processing. */
typedef struct cc_photo_controls {
    uint32_t flags;
    uint64_t exposure_ns; /* Nanoseconds, 0 = automatic. */
    double analogue_gain; /* Multiplier, 0 = automatic. */
    double exposure_value; /* Stops [-8,8], 0 default; +1 doubles the AE target. */
    enum cc_photo_metering metering;
    enum cc_photo_white_balance white_balance;
    double red_gain, blue_gain; /* Multipliers [0.5,4], required for MANUAL WB. */
    double brightness; /* [-1,1], additive fraction of the native 16-bit range. */
    double contrast, saturation; /* [0,4], [0,2], respectively; 1 default. */
    double sharpness; /* [0,4], calibrated strength multiplier, 1 default. */
    enum cc_photo_denoise denoise; /* NORMAL = calibrated spatial/colour NR. */
    uint32_t mains_hz; /* 0 (off), 50 or 60; automatic shutter only. */
    uint64_t min_frame_duration_ns, max_frame_duration_ns; /* Both 0 = mode bounds. */
} cc_photo_controls;
typedef struct cc_photo_range { double minimum, maximum, default_value; } cc_photo_range;
enum cc_photo_capability_flags {
    CC_PHOTO_CAP_AF = 1U, CC_PHOTO_CAP_PDAF = 2U, CC_PHOTO_CAP_PROVISIONAL_DELAYS = 4U
};
typedef struct cc_photo_capabilities {
    uint32_t controls, flags, metering_modes, white_balance_modes, denoise_modes;
    enum cc_vc4_profile profile;
    cc_photo_controls defaults;
    uint64_t min_exposure_ns, max_exposure_ns;
    uint64_t min_auto_exposure_ns, max_auto_exposure_ns; /* Normal calibrated curve, intersected with timing. */
    uint64_t min_frame_duration_ns, max_frame_duration_ns;
    cc_photo_range analogue_gain, colour_gain, exposure_value;
    cc_photo_range automatic_gain; /* Automatic gain uses the normal calibrated curve. */
    cc_photo_range white_balance_temperature[7]; /* Kelvin limits for each automatic WB mode. */
    cc_photo_range brightness, contrast, saturation, sharpness;
    cc_control_info native_gain; /* Gain steps are nonuniform: use the sensor model. */
    cc_sensor_timing_limits timing; /* HBLANK held at its configured value. */
    cc_sensor_mode_policy policy; /* Installed automatically by calibrated capture IPAs. */
    uint32_t num_delays;
    cc_sensor_control_delay delays[CC_SENSOR_MAX_CONTROLS];
    cc_control_info lens;
    cc_af_tuning focus; /* CAP_AF: module calibration in dioptres/native codes. */
} cc_photo_capabilities;
enum cc_photo_status_flags {
    CC_PHOTO_EXPECTED_SENSOR = 1U, CC_PHOTO_STATS_TRUSTED = 2U,
    CC_PHOTO_AUTO_EXPOSURE = 4U, CC_PHOTO_AUTO_GAIN = 8U, CC_PHOTO_AUTO_WB = 16U,
    CC_PHOTO_NATIVE_OVERRIDE = 32U, CC_PHOTO_LIMITED = 64U
};
/* No I/O. Preparing also checks capabilities/live limits (-ENOTSUP/-ERANGE).
 * Unknown flags return -ENOTSUP; malformed enums/values return -EINVAL. */
CC_API int cc_photo_validate_controls(const cc_photo_controls *controls);
#ifdef __cplusplus
}
#endif
#endif
