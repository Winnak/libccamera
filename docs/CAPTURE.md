# Capture and request lifecycle

[Project overview](../README.md). Run example commands from the repository root.

- [Capture and ownership](#capture-and-ownership)
- [Frame intervals](#frame-intervals)
- [Capture requests](#capture-requests)
- [Raspberry Pi raw capture](#raspberry-pi-raw-capture)

## Capture and ownership

Media-backed pipelines prefer complete media-v2 topology snapshots, including
entities, interfaces, pads, data links and ancillary relationships. Device paths
come from interface major/minor numbers; interface and ancillary links stay out
of capture-route searches. Full 64-byte entity-name storage is retained instead
of truncating names to the legacy descriptor width. Entity/pad arrays may arrive
in any order. See the [kernel topology API](https://docs.kernel.org/userspace-api/media/mediactl/media-ioc-g-topology.html).

Each object array is bounded to 4096 entries. The reader retries a changed
topology version, changed counts or `ENOSPC` up to three times, then returns
`-EAGAIN`. Invalid object references, duplicate IDs/names, non-contiguous pad
indices and malformed replies fail explicitly. Linux media versions before 4.19,
which lack reliable v2 pad indices/flags, and drivers without the topology ioctl
use legacy enumeration. A corrupt or unstable supported v2 graph does not fall
back silently. Link writes and associated-lens acquisition check the stored
topology version and return `-ESTALE` after object changes. Discovery remains a
snapshot; notification-based hotplug and stable camera identities are pending.

UVC cameras are identified as `CC_PIPELINE_UVC`. Media discovery selects the
unique usable `MEDIA_ENT_FL_DEFAULT` capture entity and checks the `uvcvideo`
capture node; opening it retains both the media and video device locks. Direct
capture-node opening also detects the actual `uvcvideo` driver. Other
self-contained nodes keep the generic V4L2 backend. Capture/format negotiation,
MMAP, DMA-BUF ownership and request buffers use the common queue engine. Camera
identity, location/pixel-array properties and notification hotplug remain pending.

[ccamera/uvc.h](../include/ccamera/uvc.h) exposes live control profiles and normalized
brightness, contrast, saturation, exposure mode/time, gain and gamma. Brightness
zero maps to the native default, including asymmetric ranges. Contrast and gain
follow the upstream device-range heuristic with unity at the native default;
they are not a calibrated sensor gain model. Saturation zero maps to its native
minimum and unity to its default. Gamma uses native hundredths and exposure uses
microseconds, truncated to native 100-microsecond units. Values saturate, round
to integer controls and quantize down to the native step grid. Degenerate
unmappable saturation ranges are omitted; constant controls and defaults at the
contrast/gain bounds are handled without division by zero. No libm is needed.

`cc_camera_set_uvc_controls` applies immediate commands and returns acknowledged
normalized values. `cc_request_set_uvc_controls` copies commands for a UVC graph;
they are applied once after acquire fences, before queuing the source buffer.
`result.uvc` retains only that request's acknowledged commands until release.
Omitted controls persist in the device. These are write acknowledgements, not
measurements of exposure on that image; the UVC driver/device can have latency.
Requests explicitly select auto/manual mode when needed. Auto prefers native
AUTO then APERTURE_PRIORITY; manual prefers SHUTTER_PRIORITY then MANUAL.

Native menu querying is available through `cc_camera_query_menu`,
`cc_video_query_menu`, `cc_cfe_query_menu`, `cc_vc4_isp_query_menu` and
`cc_capture_query_menu`. Query the control's native minimum/maximum first and
probe its indices: `-ENOENT` can indicate a hole, so it does not mean later
indices are absent. String menu labels and signed 64-bit integer-menu values
are distinct; malformed replies leave the result unchanged. UVC mode selection
uses the actual sparse native exposure menu.

VIMC is discovered as one `CC_PIPELINE_VIMC` camera. Its processed RGB/YUV node
uses Sensor B → Debayer B → Scaler; the required Raw Capture 1 tap is also
configured, without allocating or streaming it. Mutable competing paths are
disabled while configuring and restored on close. Native `RGB3` is always
supported; `BGR3` requires media version 5.7 or newer. Before 5.16 the scaler
requires a fixed 3× enlargement and output dimensions divisible by six; newer
kernels use a 1920×1080 sensor and even output dimensions. TRY changes no ACTIVE
pads or links. Use the discovered camera with `ccamera-capture -c INDEX`.

[ccamera/vimc.h](../include/ccamera/vimc.h) provides normalized brightness `[-1,1]`
and contrast/saturation `[0,2]`, with finite checks, saturation and native scalar
rounding. `cc_request_set_vimc_controls` copies commands and applies them once
before queuing the request's source buffer. `result.vimc` retains its acknowledged
values until release; omitted controls persist in the driver and have no result
flag. The graph's `result.raw` source view contains the processed RGB frame.
No frame-sync event subscription is needed. The upstream dummy VIMC IPA's
mock-buffer/IPC test and normalized camera properties remain pending with the
common module-loading/IPC and properties work. Native behavior is tested with
injected drivers; capture on a loaded kernel VIMC device remains unverified.

The public API is [include/ccamera/ccamera.h](../include/ccamera/ccamera.h). The
caller discovers and opens a camera, negotiates its format, allocates and queues
buffers, starts capture, dequeues frames, consumes their bytes, and requeues the
buffers. The caller controls threading and can integrate the borrowed camera fd
with poll/epoll. Calls operating on the same camera must be serialized.

Each frame exposes the kernel's memory planes, stride and color information,
sequence number, timestamp and V4L2 flags. For each plane, the image payload starts
at `data + data_offset` and has `bytes_used - data_offset` bytes. Padding is retained.
Raw camera capture returns bytes unchanged. The optional VC4 processing path
configures hardware debayering, cropping and scaling, returning the processed
memory planes. Your library can feed either kind of output to any encoder,
store it, or process it directly. The library does not encode, decode, or add a
container. If a capture device already supplies compressed data, those bytes are
passed through unchanged as well.

Dequeuing transfers buffer ownership to the caller. Its mapping remains valid
until the buffer is requeued, freed, or the camera is closed. A previously saved
pointer must not be accessed while its buffer is queued. DMA-BUF export returns
a new fd owned by the caller; close all exported fds before freeing buffers. If
`free_buffers` fails with `-EBUSY` due to outstanding exports, mappings have
already been removed: close the exports and retry `free_buffers`.

For externally allocated DMA-BUFs, use `cc_camera_prepare_imports` instead of
`cc_camera_allocate_buffers`, then bind each slot with `cc_camera_import_buffer`.
The library duplicates the plane fds; caller fds can be closed after import
succeeds. Dequeued imported planes have `data == NULL` and expose their lengths
and payload metadata. CPU mapping and DMA-BUF CPU cache synchronization are the
caller's responsibility for these buffers.

Functions return zero or a negative errno, except exported fds and borrowed fds.
Enumeration ends with `-ENOENT`. Nonblocking dequeue returns `-EAGAIN`; a bounded
wait can return `-ETIMEDOUT`. Keep the V4L2 timestamp flags when interpreting the
timestamp's clock and source, and inspect `V4L2_BUF_FLAG_ERROR` before using a
frame. `-EPROTO` means malformed driver metadata; stop capture before proceeding.
The buffer layout follows the [Linux V4L2 buffer API](https://docs.kernel.org/userspace-api/media/v4l/buffer.html).

## Frame intervals

Frame intervals are exact `cc_fraction` values in seconds per frame. For example,
`{1001, 30000}` represents about 29.97 fps without floating-point conversion.
`cc_stream_enum_frame_interval` and `cc_camera_enum_frame_interval` enumerate a
capture/output node's intervals for a FOURCC and image size. Results distinguish
discrete, continuous and stepwise ranges; discrete values have equal minimum and
maximum and no step. For sensor MBUS codes and sensor geometry, use
`cc_camera_enum_sensor_frame_interval` or its CFE wrapper. Enumeration uses the
actual selected source pad and, for multiplexed sensors, the discovered image
stream ID. It does not select or reconfigure a sensor mode.

Configure the format first, then set the interval before allocating buffers:

```c
cc_fraction period = {1001, 30000};
int ret = cc_camera_set_frame_interval(camera, &period);
/* On success, period contains the driver's actual interval. */
```

`cc_stream_get_frame_interval` / `cc_stream_set_frame_interval` use native
[G/S_PARM](https://docs.kernel.org/userspace-api/media/v4l/vidioc-g-parm.html)
for image capture/output queues. Existing capture/output and extended modes are
preserved. Set requires native TIMEPERFRAME support and every queue sharing the
handle to have no allocated buffers. A video request `{0,0}` restores the driver
default; a partly zero fraction is invalid. Metadata queues do not use this API.

Camera get/set uses the sensor's active
[frame-interval ioctl](https://docs.kernel.org/userspace-api/media/v4l/vidioc-subdev-g-frame-interval.html)
when a sensor subdevice is present, otherwise the video-node API. Sensor requests
must be positive. CFE offers corresponding wrappers, and
`cc_capture_get_frame_interval` reads the graph source's current native interval.
Unsupported ioctls return `-ENOTSUP`; outputs remain unchanged on errors, though
a failed setter can already have changed hardware. No software pacing, frame
skipping or repetition is added by this library.

Register-list sensors can instead expose pixel-rate/blanking controls rather
than interval ioctls. The existing timing getters and exposure/blanking planner
remain the API for those sensors. There is no synthetic ioctl fallback or
implicit delayed-control write. During graph capture, use the optional reference
AE frame-duration policy or delayed native blanking controls. A native interval
read describes current device settings, not the expected timing of a previously
captured frame. Physical interval accuracy and mode-specific behavior remain
unverified on camera hardware.

## Capture requests

[include/ccamera/capture.h](../include/ccamera/capture.h) provides queued frame
requests above the device APIs. Configure and allocate the source and optional
processor first, then call `cc_capture_create` with exactly one `camera` or
`frontend` and at most one `vc4` or `backend`. The processor's input layout must
match the source. Devices must be idle, with no queued or held buffers. Creation
adopts the handles and buffers on success and clears the supplied device bundle;
failure leaves ownership with the caller. Do not use adopted handles separately.

Create a `cc_request` with a caller cookie, set optional native FE/BE tuning or
VC4 ISP controls or scalar sensor controls, and queue it. Tuning and control
payloads are copied, so the caller can reuse its parameter storage immediately.
Source/sensor controls remain
available through `cc_capture_query_control`/`next_control`/`get_controls`/
`set_controls`; these changes apply immediately, without request association.
Optional delayed and request-associated scheduling is described below.
Layout-changing controls stay blocked while allocated.
Queueing validates tuning and accepts the request without
starting hardware. `cc_capture_dispatch` or `cc_capture_dequeue` drives work.
There are no background threads or hidden event loops.

Up to 256 requests can be outstanding, including delivered results. Requests
wait in FIFO order for a common buffer slot. The usable slot count is the
minimum allocated queue count, capped at 256. Capture of a later request can
overlap ISP processing of an earlier one; each lower-level hardware controller
still runs one job at a time. Both command-line examples now maintain reusable
request pools instead of manually exporting/submitting each captured frame.

`cc_capture_dequeue` returns a request once, in completion order. Its result
contains status/error, caller cookie, request sequence, original raw frame,
processed outputs, statistics and native parameter views where present. Driver
buffer-error flags give `CC_REQUEST_FAILED/-EIO` with views retained. A graph
error cancels every affected request and clears views. Cancelled/failed requests
are drainable before dequeue reports the graph error. A successful stop permits
restart; failed cancellation closes the graph and requires creating it again.

Results keep their slots until `cc_capture_release`; held results apply
backpressure to capture. Release expires the views and returns the request to
IDLE, preserving its cookie/settings for reuse. Update the cookie or settings
before queueing it again. Destroying an owned request returns `-EBUSY`.
Stop/close invalidates all views, including previously delivered ones. Closing
the graph detaches requests but leaves their objects owned by the caller.

Requests can select external image storage independently for the source, main
output and auxiliary output. Allocate those image queues as `CC_MEMORY_DMABUF`:
use `cc_camera_prepare_imports`, `cc_cfe_allocate_buffers_with_memory`, or the
VC4/PiSP `allocate_buffers_with_memory` APIs. The latter take two output memory
types, so one output can use MMAP while the other imports caller buffers.
Statistics, parameters and embedded packets keep their mapped storage for IPA
access. Before adoption set `cc_capture_devices.request_buffers` to the desired
`CC_CAPTURE_REQUEST_RAW/OUTPUT0/OUTPUT1` bits. Every queued request must provide
each opted-in image through `cc_request_set_buffer`.

The setter duplicates all plane fds with CLOEXEC, checks their identities and
rejects aliases within the request. Caller descriptors can be closed immediately.
Plane counts and capacities are checked against the negotiated layout at queue;
each plane starts at offset zero. Queue also rejects identities shared with a
pending/held request or a canonical internal image/metadata pool. Internal MMAP
exports are retained and duplicated for these graphs because repeated kernel
exports can otherwise create different DMA-BUF objects for one allocation.
Distinct exporter objects can still alias physical pages, including exports
created before adoption; callers must avoid those aliases and serialize external
access. Imported planes start at offset zero; CPU mapping/cache synchronization
remains the caller's responsibility. See the [kernel DMA-BUF contract](https://docs.kernel.org/driver-api/dma-buf.html).

Imported image views have `data == NULL`; request plane exports duplicate the
selected storage. Native `frame.index` values remain internal queue slots and
do not identify the caller's allocation. Releasing a result unbinds its managed
images, while the request keeps its own fd copies for reuse until cleared,
replaced or destroyed. Stop/close cancels views and safely releases native
imports; detached request bindings remain reusable. Camera graphs with pre-bound
imports and RAW's request bit clear continue to use their existing slot bindings.

`cc_request_set_acquire_fence` associates an optional Linux
[sync-file fence](https://docs.kernel.org/driver-api/sync_file.html) with each
source/main/auxiliary image, including default storage. It validates and
duplicates the fd with CLOEXEC; the caller can close its copy immediately.
All acquire fences must signal before request-specific controls, IPA work or
image DMA starts. Signaled fences are consumed once and do not repeat on reuse.
Fence errors or the default 300 ms preparation timeout cancel only the affected
request, with no frame views and its error reason. Other hardware jobs and held
results continue. The preparation deadline starts at queue acceptance, even
while waiting behind other requests or held slots. Change it on an idle request
with `cc_request_set_fence_timeout`; zero disables it.

Waiting fences remain owned after timeout, stop, close or a preparation error.
Use `cc_request_take_acquire_fence` on an idle/detached request or delivered
result to recover a retained fd, or replace/clear it after release. Requeueing
with an unresolved fence returns `-EEXIST`; a successful prepare consumes its
fences automatically. A shorter dequeue timeout leaves pending work intact.
For an external event loop, combine `cc_capture_pollfds` (including all pending
fence fds) with `cc_capture_get_timeout`, which returns the remaining milliseconds
until the earliest preparation deadline, or -1 when none exists. Dispatch on readiness or
expiration and drain results. Dispatch checks current readiness before timing
out remaining active fences; callers must service their loop promptly. The
blocking dequeue combines that deadline with its caller-supplied timeout.
Acquire fences do not provide CPU cache synchronization or release/out fences.

Timeouts cover the entire capture/processing operation and retain pending work.
When no device can advance because slots are held, dequeue returns `-EAGAIN`.
For an external event loop, dispatch nonblocking work, query
`cc_capture_pollfds`, poll those borrowed descriptors, and dispatch again.
Descriptors include only unfinished ports and specify POLLIN/POLLOUT as needed.
The library's blocking dequeue uses the same poll set and handles interrupted
waits. Exported raw/output/statistics plane fds are caller-owned; finish
downstream DMA before release/stop/close, and close exports before destroying
the graph. Encoding remains the application's choice.

Arbitrary device graphs, concurrent jobs
within each controller, media requests, mode-specific sensor overrides and advanced automatic
image algorithms are still pending. Tests cover raw/CFE request capture,
camera-to-VC4/BE chains and combined CFE-to-BE automatic exposure/WB feedback.
The combined path includes independent delayed sensor/scene models, held buffer
backpressure, overlapping capture/processing, copied FE/BE overrides, restart
and cancellation after statistics/device failures. Physical capture, actual
pixel processing and image quality still need hardware validation.

## Raspberry Pi raw capture

Use camera discovery rather than opening a numbered video node directly, so the
sensor and media links are configured together. Discovery reads sysfs and the
media graph without libudev, and does not depend on fixed `/dev/videoN` numbers.
`-c` selects an index from the current discovery result:

```sh
./build/ccamera-capture -l
./build/ccamera-capture -c 0 -m
./build/ccamera-capture -c 0 -w 1640 -h 1232 -f pRAA -n 10 -o frames.raw
```

Select a FOURCC and sensor size reported by your hardware; `pRAA` above is an
example of packed RGGB10. Bayer order and supported bit depths depend on the
sensor and its configuration. The program prints the actual negotiated format,
plane strides and frame metadata to stderr. It concatenates plane payloads into
the output file without adding a container or encoding. Omit `-o` to inspect
capture without writing pixels.

Unicam capture configures the sensor, any mux/bridge along the media route, and
the Unicam image node. Pi 5 raw capture uses `rp1-cfe-csi2-ch0` and disconnects
competing CFE outputs while it owns the device. This uses the CFE's direct CSI
capture path described in the [kernel RP1 CFE documentation](https://www.kernel.org/doc/html/latest/admin-guide/media/raspberrypi-rp1-cfe.html).
Image configuration discovers the native sensor image stream ID. Optional
embedded capture adds its metadata stream and timestamp pairing; multi-exposure
HDR graphs and custom virtual-channel policies remain pending. Low-level controls
use native V4L2 sensor IDs and units, with immediate application. The owned
capture graph optionally adds [frame-delay scheduling](SENSORS.md#sensor-control-scheduling)
and [reference automatic controls](IPA.md#c-ipa-modules-and-automatic-controls) when a
front-end/ISP statistics port is present. Optional AF uses configured sensor PDAF
and contrast statistics, with contrast fallback when phase data is unavailable.

Camera access uses nonblocking exclusive advisory locks on the media, subdevice
and capture fds. Other applications must cooperate with these locks; kernel
driver ownership checks also apply. Closing a camera stops capture and attempts
to restore media links changed by the library. Sensor formats and controls retain
their last configured values. Discovery is a snapshot: hot removal is reported
through ordinary syscall errors, and the caller can enumerate again.
