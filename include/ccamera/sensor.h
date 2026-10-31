/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_SENSOR_H
#define CCAMERA_SENSOR_H
#include <ccamera/ccamera.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_SENSOR_MAX_CONTROLS 32U
#define CC_SENSOR_MAX_DELAY 64U
#define CC_SENSOR_CONTROL_HISTORY 128U
#define CC_SENSOR_CONTROL_PRIORITY 1U

typedef struct cc_sensor_control_delay {
    uint32_t id;
    uint32_t frames; /* Frames from a frame-start write until expected effect. */
    uint32_t flags; /* CC_SENSOR_CONTROL_PRIORITY: write separately, first. */
} cc_sensor_control_delay;

/* Model of controls expected to affect a captured frame, using configured
 * sensor delays and values returned by successful V4L2 writes. These are not
 * measurements from embedded sensor data. Native V4L2 IDs/units; scalar only.
 * count == 0 means delay modelling was disabled. All payload pointers are NULL. */
typedef struct cc_sensor_controls {
    uint32_t frame_sequence, count;
    uint64_t cookie; /* Latest scheduled context effective at this frame; initially 0. */
    cc_control controls[CC_SENSOR_MAX_CONTROLS];
} cc_sensor_controls;

typedef struct cc_sensor_model cc_sensor_model;
/* EMBEDDED/PDAF/HDR_STATISTICS advertise decoders in embedded.h. They do not
 * configure sensor streams or guarantee that the module/driver exposes them. */
enum cc_sensor_feature {
    CC_SENSOR_CELL_SIZE = 1U, CC_SENSOR_DELAYS = 2U, CC_SENSOR_GAIN = 4U,
    CC_SENSOR_EXPOSURE = 8U, CC_SENSOR_DRIVER_DEPENDENT_DELAYS = 16U,
    CC_SENSOR_EMBEDDED = 32U, CC_SENSOR_PDAF = 64U, CC_SENSOR_HDR_STATISTICS = 128U
};
typedef struct cc_sensor_properties {
    const char *model; /* Static canonical name; never free. */
    uint32_t features;
    uint32_t cell_width_nm, cell_height_nm;
    uint32_t frame_integration_margin; /* Lines, if EXPOSURE is present. */
    uint32_t num_delays;
    cc_sensor_control_delay delays[4]; /* Exposure, gain, VBLANK, HBLANK. */
} cc_sensor_properties;
typedef struct cc_sensor_timing {
    uint64_t pixel_rate; /* Sensor pixel clocks per second, not CSI bit rate. */
    uint32_t width, height, hblank, vblank;
} cc_sensor_timing;

typedef struct cc_sensor_timing_limits {
    uint64_t pixel_rate;
    uint32_t width, height;
    uint32_t min_hblank, max_hblank, hblank_step;
    uint32_t min_vblank, max_vblank, vblank_step;
    uint32_t min_exposure, exposure_step; /* Native logical line codes. */
} cc_sensor_timing_limits;
enum cc_sensor_plan_flags {
    CC_SENSOR_PLAN_EXPOSURE_ADJUSTED=1U, CC_SENSOR_PLAN_DURATION_ADJUSTED=2U,
    CC_SENSOR_PLAN_LONG_EXPOSURE=4U
};
typedef struct cc_sensor_exposure_plan {
    uint32_t flags, exposure_lines, hblank, vblank;
    uint32_t frame_length, long_exposure_shift;
    uint64_t exposure_ns, frame_duration_ns;
} cc_sensor_exposure_plan;
typedef struct cc_sensor_mode_description {
    uint32_t width, height, binning_x;
    double scale_x; /* Sensor pixels per output pixel, including binning. */
    uint64_t min_frame_duration_ns;
} cc_sensor_mode_description;
typedef struct cc_sensor_mode_policy {
    double sensitivity;
    uint32_t hide_startup, hide_mode_switch;
    uint32_t mistrust_startup, mistrust_mode_switch;
    uint32_t min_debin_factor;
} cc_sensor_mode_policy;

#define CC_SENSOR_STREAM_ACTIVE 1U
#define CC_SENSOR_STREAMS_EMBEDDED 1U
#define CC_SENSOR_STREAMS_MULTIPLEXED 2U
typedef struct cc_sensor_stream_info {
    uint32_t pad, stream, mbus_code, width, height;
    uint32_t flags; /* CC_SENSOR_STREAM_ACTIVE for the current sensor route. */
} cc_sensor_stream_info;
typedef struct cc_sensor_streams {
    uint32_t flags;
    cc_sensor_stream_info image, embedded;
} cc_sensor_streams;

/* Exact model token lookup; an optional space-separated media-entity suffix
 * is ignored. Explicit IMX708 module aliases are recognized, arbitrary suffixes
 * and substring matches are not. Handles/properties are static borrowed data.
 * Unknown models return -ENOENT; missing features return -ENOTSUP in helpers.
 * Properties/delays follow the upstream libcamera reference;
 * driver/mode overrides may be needed. OV5647 delays are marked DRIVER_DEPENDENT
 * as in the reference. */
CC_API int cc_sensor_lookup(const char *name, const cc_sensor_model **model);
CC_API const cc_sensor_properties *cc_sensor_get_properties(const cc_sensor_model *model);
/* Reads the actual media sensor entity, independent of caller display names.
 * Self-contained capture nodes without a known sensor return -ENOTSUP. */
CC_API int cc_camera_get_sensor(cc_camera *camera, const cc_sensor_model **model);
CC_API int cc_camera_get_sensor_timing(cc_camera *camera, cc_sensor_timing *timing);
/* Live ranges for the configured sensor mode. Read-only blanking is collapsed
 * to its current value. Exposure maximum is frame-dependent and is derived by
 * the known sensor model in the planner; writes still require driver acceptance. */
CC_API int cc_camera_get_sensor_timing_limits(cc_camera *camera, cc_sensor_timing_limits *limits);
CC_API int cc_sensor_timing_bounds(const cc_sensor_timing_limits *limits, uint64_t *minimum_ns, uint64_t *maximum_ns);
/* Native C exposure/blanking planning, following the supplied RPi helpers.
 * Zero duration limits select the mode's physical limits; requested bounds are
 * clamped to those limits. Timing is quantized down to native line/blanking
 * steps, so the actual duration may be slightly below its requested minimum.
 * Returned times round up to whole ns, as in the other timing helpers.
 * Exposure uses the model's shutter equation, margin and live minimum/step.
 * IMX477/519/708 logical frame lengths are rounded for shifts 0..7. Do not write
 * a shift register: submit full logical VBLANK/exposure codes to the driver.
 * No I/O occurs; outputs are unchanged on error. Hardware acceptance, current
 * dynamic exposure ranges and delayed priority writes remain the caller's job. */
CC_API int cc_sensor_plan_exposure(const cc_sensor_model *model, const cc_sensor_timing_limits *limits,
                                  uint64_t exposure_ns, uint64_t min_frame_duration_ns,
                                  uint64_t max_frame_duration_ns, cc_sensor_exposure_plan *plan);
/* Model startup/mode-switch discard and statistics-trust counts, sensitivity
 * and minimum debinning factor. Geometry/binning must come from the actual mode;
 * querying this policy does not itself hide frames or alter capture state. */
CC_API int cc_sensor_get_mode_policy(const cc_sensor_model *model, const cc_sensor_mode_description *mode,
                                    cc_sensor_mode_policy *policy);
/* Live sensor source-pad/stream discovery. Stream-aware sensors are classified
 * from bounded routing/format enumeration, including the internal embedded-data
 * format; image and embedded stream IDs are not assumed to be 0/1. Legacy
 * sensors use their media source pads. Ambiguous image/embedded routes fail with
 * -ENOTUNIQ; unsupported formats with -ENOTSUP. Outputs stay unchanged on error.
 * Embedded presence here does not configure its receiver or capture queue. */
CC_API int cc_camera_get_sensor_streams(cc_camera *camera, cc_sensor_streams *streams);

/* Native analogue-gain model. Finite gain >=1 is required. Encode rounds down
 * to a representable code and saturates at the model's mathematical maximum;
 * this does not replace live driver bounds. Decode rejects invalid codes.
 * No libm is needed, including logarithmic-register sensors. */
CC_API int cc_sensor_gain_code(const cc_sensor_model *model, double gain, uint32_t *code);
CC_API int cc_sensor_gain(const cc_sensor_model *model, uint32_t code, double *gain);
/* Intersect model and V4L2_CID_ANALOGUE_GAIN integer min/max/step, then quantize
 * downward (or clamp to the minimum). actual is the model gain at that code,
 * before any adjustment made by a later driver write. Outputs unchanged on error. */
CC_API int cc_sensor_quantize_gain(const cc_sensor_model *model, const cc_control_info *range,
                                  double gain, cc_control *control, double *actual);

/* Integer-nanosecond timing, with checked full-width arithmetic. Exposure
 * encoding floors to lines; decoding rounds time up to preserve re-encoding.
 * IMX296 uses its fixed shutter period/offset and permits NULL timing. Other
 * supported models use (width+HBLANK)/pixel_rate and require timing. These
 * conversions do not enforce live exposure limits or change frame blanking.
 * Current driver timing is not a measurement of delayed per-frame settings. */
CC_API int cc_sensor_exposure_lines(const cc_sensor_model *model, const cc_sensor_timing *timing,
                                   uint64_t exposure_ns, uint32_t *lines);
CC_API int cc_sensor_exposure_time(const cc_sensor_model *model, const cc_sensor_timing *timing,
                                  uint32_t lines, uint64_t *exposure_ns);
CC_API int cc_sensor_frame_duration(const cc_sensor_timing *timing, uint64_t *duration_ns);

#ifdef __cplusplus
}
#endif
#endif
