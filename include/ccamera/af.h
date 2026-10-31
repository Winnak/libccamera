/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_AF_H
#define CCAMERA_AF_H
#include <ccamera/ccamera.h>
#ifdef __cplusplus
extern "C" {
#endif

#define CC_AF_MAX_WINDOWS 10U
#define CC_AF_MAX_REGIONS 1024U
#define CC_AF_MAX_MAP 32U
typedef struct cc_af cc_af;
enum cc_af_mode { CC_AF_MANUAL, CC_AF_AUTO, CC_AF_CONTINUOUS };
enum cc_af_range { CC_AF_NORMAL, CC_AF_MACRO, CC_AF_FULL };
enum cc_af_speed { CC_AF_SPEED_NORMAL, CC_AF_SPEED_FAST };
enum cc_af_pause { CC_AF_PAUSE_IMMEDIATE, CC_AF_PAUSE_DEFERRED, CC_AF_RESUME };
enum cc_af_state { CC_AF_IDLE, CC_AF_SCANNING, CC_AF_FOCUSED, CC_AF_FAILED };
enum cc_af_pause_state { CC_AF_RUNNING, CC_AF_PAUSING, CC_AF_PAUSED };
typedef struct cc_af_range_tuning { double minimum, maximum, default_position; } cc_af_range_tuning;
typedef struct cc_af_speed_tuning {
    double step_coarse, step_fine, contrast_ratio, retrigger_ratio;
    uint32_t retrigger_delay;
    double pdaf_gain, pdaf_squelch, max_slew;
    uint32_t pdaf_frames, dropout_frames, step_frames;
} cc_af_speed_tuning;
typedef struct cc_af_map_point { double dioptres, code; } cc_af_map_point;
typedef struct cc_af_tuning {
    cc_af_range_tuning ranges[3];
    cc_af_speed_tuning speeds[2];
    uint32_t confidence_epsilon, confidence_threshold, confidence_clip, skip_frames, check_for_ir;
    uint32_t num_map_points;
    cc_af_map_point map[CC_AF_MAX_MAP];
} cc_af_tuning;
enum cc_af_control_flags {
    CC_AF_SET_MODE=1U, CC_AF_SET_RANGE=2U, CC_AF_SET_SPEED=4U, CC_AF_SET_METERING=8U,
    CC_AF_SET_WINDOWS=16U, CC_AF_SET_LENS=32U, CC_AF_TRIGGER=64U, CC_AF_CANCEL=128U, CC_AF_SET_PAUSE=256U
};
typedef struct cc_af_controls {
    uint32_t flags;
    enum cc_af_mode mode;
    enum cc_af_range range;
    enum cc_af_speed speed;
    enum cc_af_pause pause;
    uint32_t use_windows, num_windows;
    cc_rectangle windows[CC_AF_MAX_WINDOWS]; /* In stats_region coordinates. */
    double lens_position; /* Dioptres; applied only in manual mode. */
} cc_af_controls;
#define CC_AF_LENS_VALID 1U
typedef struct cc_af_status {
    uint32_t flags;
    enum cc_af_state state;
    enum cc_af_pause_state pause_state;
    int32_t lens_setting; /* Native, quantized driver code. Expected, not measured. */
    double lens_position; /* Calibrated dioptres for that code. */
} cc_af_status;
typedef struct cc_af_pdaf { uint16_t confidence; int16_t phase; } cc_af_pdaf; /* Signed Q4 phase. */
typedef struct cc_af_pdaf_grid { uint32_t rows, cols; const cc_af_pdaf *regions; } cc_af_pdaf_grid;
typedef struct cc_af_colour { uint64_t red, green, blue; uint32_t counted; } cc_af_colour;
typedef struct cc_af_statistics {
    uint32_t focus_rows, focus_cols, colour_rows, colour_cols;
    const uint64_t *focus;
    const cc_af_colour *colour;
} cc_af_statistics;

/* C17 port of the RPi hybrid PDAF/CDAF state machine. No device I/O, libm or
 * configuration-file dependency. Calls are serialized. Input arrays are borrowed
 * for each call. Tuning/map are copied. Coarse scans are bounded to 4096 samples.
 * The reference defaults assume the standard Raspberry Pi V3/IMX708 lens and
 * no more than 30 controller updates/s. Settling/slew tuning is update-relative;
 * explicitly select and calibrate them for the actual camera module. */
CC_API void cc_af_reference_tuning(cc_af_tuning *tuning);
CC_API int cc_af_create(const cc_af_tuning *tuning, cc_af **af);
CC_API void cc_af_destroy(cc_af *af);
/* Configure the native writable FOCUS_ABSOLUTE range and statistics region.
 * Reconfiguration resets to manual mode, unknown lens position. */
CC_API int cc_af_configure(cc_af *af, const cc_rectangle *stats_region, const cc_control_info *lens);
CC_API int cc_af_validate_controls(const cc_af_controls *controls);
CC_API int cc_af_set_controls(cc_af *af, const cc_af_controls *controls);
/* prepare consumes current PDAF (optional) and the previous process statistics;
 * process consumes contrast/colour from the completed frame. Empty grids are
 * allowed. Native BSTA/RPFS adapters use upstream focus scales and grid sizes.
 * Lens settling counts controller updates, so callers must not advance this
 * controller repeatedly without capturing/processing corresponding frames. */
CC_API int cc_af_prepare(cc_af *af, const cc_af_pdaf_grid *pdaf, cc_af_status *status);
CC_API int cc_af_process(cc_af *af, const cc_af_statistics *statistics);
CC_API int cc_af_process_native(cc_af *af, uint32_t fourcc, const void *data, size_t size);
CC_API int cc_af_lens_position(const cc_af *af, int32_t code, double *dioptres);
#ifdef __cplusplus
}
#endif
#endif
