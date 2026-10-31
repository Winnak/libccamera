/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_H
#define CCAMERA_IPA_H
#include <ccamera/cfe.h>
#include <ccamera/pisp_be.h>
#include <ccamera/sensor.h>
#include <ccamera/af.h>
#include <ccamera/embedded.h>
#include <ccamera/vc4_tuning.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IPA_ABI_VERSION 5U
#define CC_IPA_MAX_CONTROLS 32U
typedef struct cc_ipa cc_ipa;
enum cc_ipa_pipeline { CC_IPA_VC4 = 1, CC_IPA_CFE, CC_IPA_PISP };
enum cc_ipa_statistics_flags { CC_IPA_STATS_LUMA = 1U, CC_IPA_STATS_COLOUR = 2U };
typedef struct cc_ipa_statistics {
    uint32_t flags;
    double luminance; /* Linear histogram mean in [0,1]. */
    double red, green, blue; /* Mean accepted samples in [0,1], before WB. */
    uint64_t colour_samples;
} cc_ipa_statistics;
typedef struct cc_ipa_configuration {
    enum cc_ipa_pipeline pipeline;
    uint32_t statistics_fourcc;
    cc_format source;
    cc_format sensor_format; /* Before CFE conversion, or equal to source. */
    const cc_sensor_model *sensor; /* Optional static known-model handle. */
    cc_sensor_timing timing; /* Zero when the source has no sensor timing API. */
    uint32_t num_sensor_controls;
    cc_control_info sensor_controls[CC_IPA_MAX_CONTROLS];
    cc_control initial_controls[CC_IPA_MAX_CONTROLS];
    cc_control_info red_balance, blue_balance, digital_gain; /* VC4, id=0 if unavailable. */
    cc_control_info lens; /* Available only when CC_IPA_CAP_LENS is requested. */
    cc_embedded_configuration embedded; /* FOURCC=0 if embedded capture is disabled. */
    cc_sensor_timing_limits timing_limits; /* Required with SENSOR_TIMING capability. */
    cc_sensor_control_delay sensor_delays[CC_IPA_MAX_CONTROLS]; /* Same order as controls. */
    cc_sensor_geometry geometry; /* With SENSOR_GEOMETRY capability. */
} cc_ipa_configuration;
typedef struct cc_ipa_request {
    uint64_t sequence, cookie;
    cc_sensor_timing timing; /* Current driver-register timing; zero when unavailable. */
    const cc_control *sensor_controls; /* Caller overrides, borrowed for the call. */
    size_t num_sensor_controls;
    const cc_af_controls *af; /* Optional request commands, borrowed for this call. */
    cc_sensor_timing_limits timing_limits; /* Refreshed live limits with SENSOR_TIMING. */
} cc_ipa_request;
enum cc_ipa_settings_flags { CC_IPA_FRONTEND = 1U, CC_IPA_BACKEND = 2U, CC_IPA_LENS = 4U, CC_IPA_VC4_TUNING = 8U };
typedef struct cc_ipa_settings {
    uint32_t flags, num_sensor_controls, num_isp_controls;
    cc_control sensor_controls[CC_IPA_MAX_CONTROLS]; /* Scalar native V4L2 values. */
    cc_control isp_controls[CC_IPA_MAX_CONTROLS];
    struct pisp_fe_config frontend;
    struct pisp_be_config backend;
    cc_af_status af; /* Snapshot for this frame; LENS flag requests an actuator write. */
    cc_vc4_tuning vc4;
} cc_ipa_settings;
enum cc_ipa_metadata_flags {
    CC_IPA_METADATA_AE = 1U, CC_IPA_METADATA_AWB = 2U,
    CC_IPA_AE_CONVERGED = 4U, CC_IPA_AWB_CONVERGED = 8U, CC_IPA_METADATA_AF = 16U,
    CC_IPA_METADATA_SENSOR = 32U, CC_IPA_METADATA_PDAF = 64U, CC_IPA_METADATA_HDR = 128U, CC_IPA_METADATA_VC4 = 256U,
    CC_IPA_EXPOSURE_LIMIT = 512U
};
typedef struct cc_ipa_metadata {
    uint32_t flags;
    uint64_t exposure_ns;
    double analogue_gain, red_gain, blue_gain, luminance;
    cc_af_status af;
    cc_sensor_embedded_status sensor_status; /* Native unshifted register report, if SENSOR. */
    cc_sensor_hdr_statistics hdr; /* Decoded raw sensor histogram, if HDR. */
    double colour_temperature, lux;
    uint64_t frame_duration_ns;
} cc_ipa_metadata;
typedef struct cc_ipa_frame {
    uint64_t sequence, cookie;
    uint64_t timestamp_ns;
    cc_sensor_controls sensor; /* Expected driver-adjusted values, not sensor measurements. */
    cc_ipa_statistics statistics;
    uint32_t statistics_fourcc;
    const void *statistics_data; /* Borrowed only during process(). */
    size_t statistics_size;
    const cc_control *isp_controls;
    size_t num_isp_controls;
    const struct pisp_be_config *backend;
    cc_af_status af; /* Applied request snapshot, including driver-adjusted native code. */
    const void *embedded_data; /* Timestamp-matched sensor bytes, borrowed during process(). */
    size_t embedded_size; /* 0 when no packet matched; parsers must not reuse previous data. */
    const cc_control *request_sensor_controls, *settings_sensor_controls;
    size_t num_request_sensor_controls, num_settings_sensor_controls;
    const cc_vc4_tuning *vc4; /* Applied snapshot. */
    uint32_t source_sequence, untrusted;
} cc_ipa_frame;

/* A native C module ABI. The library copies this table; callback code/context
 * must remain alive until destroy. Calls are serialized and must not reenter
 * their capture graph. Inputs are borrowed for each call; copy configuration
 * data rather than retaining its pointer. Keep callbacks bounded so SOF events
 * can be serviced promptly. Callback errors are negative errno. No callbacks may
 * access device nodes directly. configure/start/prepare/process are required;
 * stop is required and destroy is optional (for caller-owned contexts).
 * prepare's ISP payload pointers must reference module-owned storage valid until
 * the next module call, not the passed settings object. The capture graph
 * deep-copies them immediately. Native blocks are inline.
 * This is not the upstream C++ IPA ABI. Loading/IPC/sandbox adapters are separate. */
enum cc_ipa_capabilities { CC_IPA_CAP_LENS=1U, CC_IPA_CAP_SENSOR_TIMING=2U, CC_IPA_CAP_SENSOR_GEOMETRY=4U };
typedef struct cc_ipa_ops {
    uint32_t abi_version;
    size_t size;
    int (*configure)(void *context, const cc_ipa_configuration *configuration);
    int (*start)(void *context);
    void (*stop)(void *context);
    int (*prepare)(void *context, const cc_ipa_request *request, cc_ipa_settings *settings);
    int (*process)(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata);
    void (*destroy)(void *context);
    uint32_t capabilities; /* Explicit opt-in to lens access or live timing limits. */
} cc_ipa_ops;
CC_API int cc_ipa_create(const cc_ipa_ops *ops, void *context, cc_ipa **ipa);
CC_API void cc_ipa_destroy(cc_ipa *ipa);
CC_API uint32_t cc_ipa_get_capabilities(const cc_ipa *ipa);
CC_API int cc_ipa_configure(cc_ipa *ipa, const cc_ipa_configuration *configuration);
CC_API int cc_ipa_start(cc_ipa *ipa);
CC_API void cc_ipa_stop(cc_ipa *ipa);
CC_API int cc_ipa_prepare(cc_ipa *ipa, const cc_ipa_request *request, cc_ipa_settings *settings);
CC_API int cc_ipa_process(cc_ipa *ipa, const cc_ipa_frame *frame, cc_ipa_metadata *metadata);
/* Safe for unaligned native BSTA/RPFS payloads. Empty statistics return flags=0;
 * malformed/truncated payloads fail without changing *statistics. */
CC_API int cc_ipa_parse_statistics(uint32_t fourcc, const void *data, size_t size, cc_ipa_statistics *statistics);

enum cc_ipa_reference_flags { CC_IPA_REFERENCE_AE = 1U, CC_IPA_REFERENCE_AWB = 2U, CC_IPA_REFERENCE_AF = 4U };
typedef struct cc_ipa_reference_tuning {
    uint32_t flags;
    double target_luminance; /* (0,1), linear. */
    double exposure_speed, white_balance_speed; /* (0,1]. */
    double min_colour_gain, max_colour_gain; /* [0.5,4], enclosing 1. */
    uint64_t max_exposure_ns; /* 0 uses live exposure/frame-length bounds. */
    /* Optional explicit gain/exposure model for self-contained raw sources
     * lacking a sensor identity. A conflicting detected model is rejected. */
    const cc_sensor_model *sensor;
    const cc_af_tuning *af; /* Required with AF; copied at create time. */
    /* Both zero preserve current-frame AE. Both nonzero enable exposure/blanking
     * planning against live mode limits. Requires AE and managed priority VBLANK
     * with delay >= EXPOSURE. Writable managed HBLANK may also vary if its delay
     * is >= EXPOSURE; otherwise horizontal blanking stays at the live value. */
    uint64_t min_frame_duration_ns, max_frame_duration_ns;
} cc_ipa_reference_tuning;
/* Proportional AE/AGC, damped grey-world AWB and optional RPi hybrid AF, with no runtime tuning
 * files or libm. Tuning is explicit; this is not a calibrated RPi image profile.
 * Exposure stays within the current frame unless duration planning is enabled.
 * AF requires explicit tuning and a native focus actuator. Configured embedded
 * capture supplies sensor reports/PDAF and available HDR histograms. Invalid or
 * absent sensor packets fall back to contrast AF and ordinary ISP statistics.
 * Bayesian AWB, flicker/HDR policies and calibrated image tuning are pending. */
CC_API int cc_ipa_reference_create(const cc_ipa_reference_tuning *tuning, cc_ipa **ipa);

/* Calibrated OV5647/VC4 still controller. Defaults: AE+AWB, 50 Hz,
 * official normal exposure curve. Manual sensor request overrides win.
 * Manual WB (both gains >0) disables AWB. mains_hz is 0, 50 or 60.
 * Options are copied. NULL selects defaults. */
typedef struct cc_ipa_vc4_options {
    uint32_t mains_hz;
    double red_gain, blue_gain;
} cc_ipa_vc4_options;
CC_API int cc_ipa_vc4_create(const cc_ipa_vc4_options *options, cc_ipa **ipa);

#ifdef __cplusplus
}
#endif
#endif
