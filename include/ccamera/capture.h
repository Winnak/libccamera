/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_CAPTURE_H
#define CCAMERA_CAPTURE_H
#include <ccamera/ccamera.h>
#include <ccamera/cfe.h>
#include <ccamera/pisp_be.h>
#include <ccamera/vc4_isp.h>
#include <ccamera/sensor.h>
#include <ccamera/ipa.h>
#include <ccamera/vimc.h>
#include <ccamera/uvc.h>
#include <poll.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_capture cc_capture;
typedef struct cc_request cc_request;

typedef struct cc_capture_devices {
    cc_camera *camera; /* Exactly one of camera/frontend. */
    cc_cfe *frontend;
    cc_vc4_isp *vc4;   /* Optional processor for camera. */
    cc_pisp_be *backend; /* Optional processor for camera/frontend. */
    uint32_t request_buffers; /* CC_CAPTURE_REQUEST_* image queues, allocated as DMABUF. */
} cc_capture_devices;

enum cc_request_status {
    CC_REQUEST_IDLE = 0, CC_REQUEST_PENDING, CC_REQUEST_COMPLETE,
    CC_REQUEST_CANCELLED, CC_REQUEST_FAILED
};
enum cc_capture_stream {
    CC_CAPTURE_RAW = 0, CC_CAPTURE_OUTPUT0, CC_CAPTURE_OUTPUT1, CC_CAPTURE_STATISTICS, CC_CAPTURE_EMBEDDED,
    CC_CAPTURE_FRONTEND_IMAGE1, CC_CAPTURE_TDN_OUTPUT, CC_CAPTURE_STITCH_OUTPUT
};
enum cc_capture_request_buffers {
    CC_CAPTURE_REQUEST_RAW = 1U, CC_CAPTURE_REQUEST_OUTPUT0 = 2U, CC_CAPTURE_REQUEST_OUTPUT1 = 4U
};
typedef struct cc_capture_result {
    enum cc_request_status status;
    int error; /* 0, -ECANCELED, or a negative errno; ERROR buffer flags give -EIO. */
    uint64_t cookie, sequence; /* Request sequence, independent of sensor sequence. */
    cc_frame raw; /* Source frame; already processed RGB on VIMC. */
    cc_frame output[2];
    cc_frame statistics;
    cc_frame frontend_parameters;
    cc_frame backend_parameters;
    cc_frame embedded; /* Optional timestamp-matched sensor packet. */
    cc_sensor_controls sensor; /* Expected effective controls, if enabled. */
    uint32_t untrusted; /* Startup statistics must not update controllers. */
    cc_ipa_metadata ipa; /* Optional algorithm metadata for this completed frame. */
    cc_vimc_controls vimc; /* Optional acknowledged image controls for this request. */
    cc_uvc_controls uvc; /* Optional acknowledged normalized UVC commands. */
    cc_frame frontend_image1; /* Optional independently cropped/scaled FE output. */
    cc_frame backend_auxiliary[4]; /* TDN input/output, STITCH input/output. Input data pointers are NULL. */
} cc_capture_result;

/* Adopt configured, allocated, idle devices. Success clears *devices and
 * transfers ownership, including buffers. Failure leaves ownership unchanged.
 * Processor input must exactly match source layout. At most one processor;
 * frontend+vc4 is unsupported. Optional FE image1 is returned with the request
 * and must use MMAP storage when adopted by a graph. All calls must be serialized.
 * No background threads or encoding is added. Sensor delays and IPA are opt-in.
 * count returns usable common slots (the minimum allocated queue count, <=256).
 * request_buffers opts image ports into per-request DMA-BUF storage. Each bit
 * requires a present DMABUF image queue; outputs/CFE imports require this mode.
 * Camera pre-bound imports remain supported with RAW's bit clear. New request
 * graphs canonicalize exports of internal image/metadata pools for alias checks.
 * Device handles must not be used separately after adoption. */
CC_API int cc_capture_create(cc_capture_devices *devices, cc_capture **capture, uint32_t *count);
/* Configure/adopt a C IPA module on an idle stopped graph with statistics,
 * or the existing VIMC graph with its mock IPA.
 * Success clears *ipa; failure leaves ownership unchanged. NULL detaches and
 * destroys the previous module. Modules run only through dispatch/dequeue;
 * configure/start repeats on restart using live sensor ranges and baselines.
 * Explicit request controls override matching algorithm controls; caller FE/BE
 * tuning overrides the whole corresponding generated tuning block. Generated
 * settings are private per-run snapshots and never alter reusable request data.
 * Sensor outputs require configured delays; explicit sequence plans cannot be
 * mixed with an attached module. Errors cancel the graph normally. No implicit
 * module selection, background threads, IPC bridge or encoding is added.
 * Explicitly loaded C modules (ipa_module.h) have the same adoption lifecycle. */
CC_API int cc_capture_set_ipa(cc_capture *capture, cc_ipa **ipa);
/* Native source/sensor controls, applied immediately. They are not associated
 * with request sequences and sensor delays are not compensated. Allocated
 * layout-changing controls remain blocked. Calls must be serialized with I/O. */
CC_API int cc_capture_query_control(cc_capture *capture, uint32_t id, cc_control_info *info);
CC_API int cc_capture_query_menu(cc_capture *capture,uint32_t id,uint32_t index,cc_control_menu *entry);
CC_API int cc_capture_query_lens(cc_capture *capture, cc_control_info *info);
CC_API int cc_capture_get_lens_position(cc_capture *capture, int32_t *position);
/* Native immediate focus is busy while a running IPA owns the lens. */
CC_API int cc_capture_set_lens_position(cc_capture *capture, int32_t *position);
CC_API int cc_capture_next_control(cc_capture *capture, uint32_t after_id, cc_control_info *info);
CC_API int cc_capture_get_controls(cc_capture *capture, cc_control *controls, size_t count, size_t *error_index);
CC_API int cc_capture_set_controls(cc_capture *capture, const cc_control *controls, size_t count, size_t *error_index);
CC_API int cc_capture_get_sensor(cc_capture *capture, const cc_sensor_model **model);
CC_API int cc_capture_get_properties(cc_capture *capture, cc_camera_properties *properties);
/* Borrowed source discovery/identity info; NULL for NULL or a dead graph. */
CC_API const cc_camera_info *cc_capture_get_info(const cc_capture *capture);
/* Orientation of the graph source. Downstream ISP output transforms, if
 * configured separately, are additional to this sensor orientation. */
CC_API int cc_capture_get_orientation(cc_capture *capture, enum cc_orientation *orientation);
CC_API int cc_capture_get_sensor_timing(cc_capture *capture, cc_sensor_timing *timing);
/* Native active interval, independent of delayed expected sensor metadata.
 * Configure intervals before allocating/adopting devices; graph timing changes
 * use the existing delayed blanking planner instead of these immediate ioctls. */
CC_API int cc_capture_get_frame_interval(cc_capture *capture,cc_fraction *interval);
CC_API int cc_capture_get_sensor_timing_limits(cc_capture *capture, cc_sensor_timing_limits *limits);
CC_API int cc_capture_get_sensor_streams(cc_capture *capture, cc_sensor_streams *streams);
CC_API int cc_capture_get_embedded_configuration(cc_capture *capture, cc_embedded_configuration *configuration);
/* Opt in to the known profile's default delays. Uses the actual media sensor
 * model, filters unavailable/read-only controls, then subscribes to SOF events.
 * Unknown profiles return -ENOENT; known profiles lacking delays return
 * -ENODATA. No fallback delays are guessed. Same idle/stopped preconditions as
 * configure_sensor_delays. Defaults can be inspected via sensor properties;
 * explicit mode/driver overrides remain available through that function. */
CC_API int cc_capture_configure_sensor_defaults(cc_capture *capture);
/* Opt-in startup/restart filtering; only hide/mistrust fields are consumed.
 * NULL clears. Idle, stopped graph only. Policy persists across stop. */
CC_API int cc_capture_configure_sensor_policy(cc_capture *capture, const cc_sensor_mode_policy *policy);
CC_API int cc_capture_get_sensor_geometry(cc_capture *capture, cc_sensor_geometry *geometry);
/* Bounded tracing: no file I/O in dispatch. Reading drains records; dropped
 * counts signal trace loss, independent of the scheduler's SOF checks. */
typedef struct cc_sensor_trace {
    uint32_t frame_sequence, target_sequence, id;
    uint64_t sof_ns, write_start_ns, write_end_ns;
    int64_t requested, acknowledged;
    int error;
} cc_sensor_trace;
CC_API int cc_capture_enable_sensor_trace(cc_capture *capture, int enable);
CC_API int cc_capture_read_sensor_trace(cc_capture *capture, cc_sensor_trace *records,
    size_t capacity, size_t *count, uint64_t *dropped);
/* Configure optional scalar sensor-control delays on an idle, stopped graph
 * with no owned requests. Reads baseline values and subscribes FRAME_SYNC on
 * the source node; unsupported events/controls cause failure. NULL/0 disables.
 * Delay values must come from the sensor mode/driver, never guessed globally.
 * Managed IDs cannot be changed through immediate set_controls while enabled.
 * Priority writes (typically VBLANK) precede the remaining batched writes.
 * Call dispatch/poll promptly for every frame even while holding results. */
CC_API int cc_capture_configure_sensor_delays(cc_capture *capture, const cc_sensor_control_delay *delays, size_t count);
/* Schedule controls for an explicit V4L2 sensor frame sequence, NOT a request
 * sequence. Each write is issued at target - its configured delay. Frames
 * start at zero after stream start; 32-bit wrap is handled. Targets must lie
 * in the next 128-frame window, with every write still in the future (or on
 * the next frame start). Repeated targets merge; repeated IDs are rejected.
 * Cookie tags the effective control context. Baseline context is zero.
 * Explicit schedules and request sensor controls cannot be mixed between stops;
 * attempting to mix them returns -EBUSY without changing queued work.
 * Stop cancels pending plans; restart reuses a fresh baseline. */
CC_API int cc_capture_schedule_sensor_controls(cc_capture *capture, uint32_t target,
                                              const cc_control *controls, size_t count, uint64_t cookie);
/* Close downstream import queues before source allocations, including when
 * cancellation fails. Outstanding requests become CANCELLED and detached;
 * they remain caller-owned and may then be destroyed or reused. */
CC_API void cc_capture_close(cc_capture *capture);

CC_API int cc_request_create(uint64_t cookie, cc_request **request);
/* Returns -EBUSY while owned by a capture or referenced as another request's history. */
CC_API int cc_request_destroy(cc_request *request);
CC_API int cc_request_set_cookie(cc_request *request, uint64_t cookie);
/* Bind a raw source to a request's TDN_INPUT or STITCH_INPUT. A source request
 * must hold a COMPLETE, delivered result from this capture graph. RAW,
 * FRONTEND_IMAGE1, TDN_OUTPUT and STITCH_OUTPUT are valid source streams.
 * source == request selects its current RAW/FRONTEND_IMAGE1 capture instead.
 * NULL source clears that binding. Layout, Bayer order and compression offset
 * must match the configured auxiliary reader. Setters are atomic on error and
 * may be used only before queueing the destination. Source release/destroy
 * returns -EBUSY while referenced. Bindings are consumed at completion or
 * cancellation, and cleared when an idle destination is destroyed. Stop/close
 * cancels readers before dropping references. No copies of image pixels are made. */
CC_API int cc_request_set_pisp_input(cc_request *request, enum cc_pisp_be_port input,
                                    cc_request *source, enum cc_capture_stream stream);
/* Assign caller-selected image storage to a request. RAW/OUTPUT0/OUTPUT1 only;
 * the graph must opt into the corresponding request_buffers bit, and that
 * image queue must use DMABUF allocation. All opted-in images are required on
 * every request. Other image queues continue to use their allocated buffers.
 * Planes are whole DMA-BUF objects at offset zero; count/layout/capacities are
 * validated at queue. The setter duplicates fds with CLOEXEC, so caller fds may
 * be closed immediately. NULL/0 clears; assignments persist on reuse and own
 * their fds until replaced/cleared/destroyed, including after graph close.
 * Setter failure leaves the previous binding intact. Aliases within a request
 * are rejected; queue compares DMA-BUF device/inode identities against other
 * owned requests and canonical graph pools, including held results. Distinct
 * exporter objects can alias physical memory: callers must avoid such aliases,
 * including exports created before graph adoption. Callers serialize access
 * across graphs and external users. Imported result planes have data == NULL.
 * CPU mapping/cache sync and external accesses remain the caller's responsibility;
 * optional acquire fences below gate the camera's use of these buffers. */
CC_API int cc_request_set_buffer(cc_request *request, enum cc_capture_stream stream,
                                  const cc_dmabuf_plane *planes, uint32_t num_planes);
/* One optional Linux sync_file acquire fence per RAW/OUTPUT0/OUTPUT1 image,
 * including images using default storage. The setter validates and duplicates
 * the fd with CLOEXEC; -1 clears. Atomic on error, idle requests only.
 * All image fences must signal before any request-specific controls or DMA.
 * Successfully signaled fences are consumed and are NOT repeated on reuse.
 * Failed/unsignaled fences remain owned by the request after cancellation;
 * take, replace or clear them before queueing it again (-EEXIST otherwise).
 * Fences do not provide CPU cache synchronization or release/out fences. */
CC_API int cc_request_set_acquire_fence(cc_request *request, enum cc_capture_stream stream, int fd);
/* Default 300 ms from queue acceptance; 0 disables the preparation timeout.
 * Dispatch services deadlines. A fence error/timeout cancels only this request
 * with its negative errno/-ETIMEDOUT and no frame views. Other jobs continue.
 * A shorter dequeue timeout retains pending work and fences as usual. */
CC_API int cc_request_set_fence_timeout(cc_request *request, int timeout_ms);
/* Transfer a retained fence fd to the caller, or -ENOENT when absent. Allowed
 * on idle/detached requests or a delivered completed/cancelled/failed request;
 * pending and undelivered owned requests return -EBUSY. Caller closes the fd. */
CC_API int cc_request_take_acquire_fence(cc_request *request, enum cc_capture_stream stream);
/* Deep snapshots. NULL clears the corresponding tuning. Only idle requests
 * may be edited; detached requests cancelled by close can be edited too.
 * FE/BE tuning is rejected on graphs without that device. */
CC_API int cc_request_set_tuning(cc_request *request, const struct pisp_fe_config *frontend,
                                const struct pisp_be_config *backend);
/* Native VC4 ISP controls, applied when the preceding ISP job has completed.
 * Payloads are copied; enabled native lens-shading DMA-BUF fds are duplicated
 * with CLOEXEC and retained until replacement or request destruction.
 * Layout-changing controls are rejected by the ISP.
 * These are ISP controls; sensor exposure-delay scheduling remains separate.
 * Unspecified controls retain the device's current values. NULL/0 clears. */
CC_API int cc_request_set_vc4_tuning(cc_request *request, const cc_vc4_tuning *tuning);
CC_API int cc_request_set_isp_controls(cc_request *request, const cc_control *controls, size_t count);
/* Copy native scalar sensor controls onto an idle request. NULL/0 clears them;
 * settings persist on reuse. The graph must have configured sensor delays and
 * manage every supplied ID. Queue validates settings before taking ownership.
 * Dispatch assigns the next reachable effective sensor frame, staggers writes
 * by their delays and recycles earlier source frames internally. Successful
 * frames are delivered/processed only at or after that target; buffer errors
 * fail the request immediately, retaining their views as usual. Omitted controls
 * retain previous values; result.sensor reports the expected driver-adjusted
 * values and this request's cookie, not embedded-data measurements. This may
 * introduce settling frames with the current one-job source controllers.
 * Explicit sequence schedules cannot be mixed with this mode until stop. */
CC_API int cc_request_set_sensor_controls(cc_request *request, const cc_control *controls, size_t count);
/* Copied, optional AF commands for the IPA. NULL clears them. Commands such as
 * TRIGGER apply on every reuse until cleared; mode/range/window values persist
 * inside the controller. Requires an IPA; reference modules require AF tuning. */
CC_API int cc_request_set_af_controls(cc_request *request, const cc_af_controls *controls);
CC_API int cc_request_get_status(const cc_request *request); /* enum or negative errno. */
/* Borrowed result after dequeue (or cancellation on capture_close), otherwise
 * NULL. Frame views last until release/stop/close. A failed frame can retain
 * views; graph cancellation clears every view, including delivered results. */
CC_API const cc_capture_result *cc_request_get_result(const cc_request *request);

/* Accepts up to 256 outstanding requests, including held results. Queue copies
 * no pixels and performs validation only; dispatch/dequeue drive the devices.
 * Requests beyond available slots wait in FIFO order. Capture and processing
 * overlap; each lower-level controller still has one job in flight.
 * A request may belong to only one graph at a time. */
CC_API int cc_capture_queue(cc_capture *capture, cc_request *request);
/* Nonblocking progress; hardware errors cancel the graph, leaving requests
 * drainable. Acquire-fence failures cancel only their request. pollfds returns
 * unfinished device ports and pending acquire fences (POLLIN), with native
 * POLLIN/POLLOUT directions for hardware.
 * It does not dispatch. *count is capacity on input and required size on output;
 * NULL fds queries the size. -ENOSPC leaves fds unchanged. Borrowed fds. */
CC_API int cc_capture_dispatch(cc_capture *capture);
CC_API int cc_capture_pollfds(const cc_capture *capture, struct pollfd *fds, size_t *count);
/* Remaining time until the earliest acquire-fence deadline, rounded up to ms; -1 in
 * *timeout_ms means none, 0 means dispatch is due. Combine this with the caller's
 * own timeout when polling pollfds, then dispatch on readiness OR expiration.
 * Query does not dispatch or consume fences; output is unchanged on error. */
CC_API int cc_capture_get_timeout(const cc_capture *capture, int *timeout_ms);
/* Returns a completed/cancelled/failed request, once, in completion order.
 * timeout 0/-1/>0 means nonblocking/unbounded/bounded over both stages.
 * Timeouts retain all work and leave *request unchanged. Errors that cancel
 * work are reported in returned requests first, then as a negative errno once
 * the completion queue is drained. Complete requests keep slots until release.
 * If held slots prevent progress and no hardware is active, returns -EAGAIN. */
CC_API int cc_capture_dequeue(cc_capture *capture, cc_request **request, int timeout_ms);
/* Release a delivered result, returning the request to IDLE. Cookie/tuning/
 * controls persist for reuse; all result views expire. No work is autoqueued. */
CC_API int cc_capture_release(cc_capture *capture, cc_request *request);
/* Returns a caller-owned CLOEXEC fd. Finish downstream DMA before release,
 * stop or close, and close exports before destroying/reallocating the graph. */
CC_API int cc_capture_export_plane(cc_capture *capture, const cc_request *request,
                                  enum cc_capture_stream stream, uint32_t plane);
/* Cancel all owned requests and invalidate views. Cancelled requests remain
 * drainable/releasable. A successful stop permits reuse/restart; a failed stop
 * closes the graph before releasing memory and requires a new graph. */
CC_API int cc_capture_stop(cc_capture *capture);

#ifdef __cplusplus
}
#endif
#endif
