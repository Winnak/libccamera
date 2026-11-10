# Capture and request lifecycle

[Project overview](../README.md). Run example commands from the repository root.

- [Capture and ownership](#capture-and-ownership)
- [Camera identities](#camera-identities)
- [Camera properties](#camera-properties)
- [Image orientation](#image-orientation)
- [Sensor mode selection](#sensor-mode-selection)
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
topology version and return `-ESTALE` after object changes. `cc_enumerate` returns
a snapshot; the [hotplug monitor](#camera-hotplug) tracks arrivals and removals.
See [camera identities](#camera-identities) for location-based selection.

UVC cameras are identified as `CC_PIPELINE_UVC`. Media discovery selects the
unique usable `MEDIA_ENT_FL_DEFAULT` capture entity and checks the `uvcvideo`
capture node; opening it retains both the media and video device locks. Direct
capture-node opening also detects the actual `uvcvideo` driver. Other
self-contained nodes keep the generic V4L2 backend. Capture/format negotiation,
MMAP, DMA-BUF ownership and request buffers use the common queue engine. See
[camera properties](#camera-properties) for model, location and geometry queries.
UVC identities follow the reference's firmware-controller and USB-port policy.
The hotplug monitor supports both media-associated and standalone UVC nodes.

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
mock-buffer/IPC test remains pending with the common IPC work. Explicit shared
C module loading is available separately.
Native behavior is tested with
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

## Camera identities

`cc_enumerate` returns an opaque `id` and an `id_type` in each `cc_camera_info`.
The same identity is available from `cc_camera_get_info`, `cc_cfe_get_info` and
`cc_capture_get_info` after opening or adopting a camera. Those getters return
borrowed pointers valid until their owning object is closed; copy the ID before
freeing a discovery list or closing a handle.

| Type | Identity policy |
| --- | --- |
| `CC_CAMERA_ID_FIRMWARE` | Sensor Device Tree node path, with `/sys/firmware/devicetree` stripped, or its ACPI namespace path |
| `CC_CAMERA_ID_PLATFORM` | Reference fallback for a sensor under `/sys/devices/platform/`: path relative to `/sys/devices/`, followed by its model; used by VIMC |
| `CC_CAMERA_ID_USB` | Nearest firmware-described USB ancestor, bus-number-free port/configuration/interface, and vendor/product IDs |
| `CC_CAMERA_ID_VIDEO` | Generic V4L2 extension: firmware/platform location plus the driver's video index |
| `CC_CAMERA_ID_NONE` | No usable identity metadata; the ID is empty |

Firmware sensor IDs survive I2C bus and device-node renumbering. UVC IDs survive
USB bus/device-node renumbering when the controller firmware description and
physical port/interface stay the same. A different port or interface changes the
ID. IDs describe a **location**, rather than a unit serial number: an identical
replacement at the same USB location can retain the same ID. Firmware changes
can change IDs. Platform fallbacks depend on the platform-instance/sysfs path,
which can include dynamic bus numbers; generic video identities also depend on
the driver's index remaining consistent. Treat IDs as opaque strings.

For example, the UVC reference policy can produce
`/base/axi/usb@2000-2.4:1.0-046d:0825`. It deliberately excludes the USB bus
number and includes the port chain `2.4` and configuration/interface `1.0`.
No ID is synthesized from `/dev/videoN`, `/dev/mediaN`, enumeration order,
camera display names or `bus_info`.

Open using fresh discovery so renumbered paths are resolved automatically:

```c
cc_camera *camera = NULL;
int ret = cc_camera_open_by_id(saved_id, &camera);
/* Or cc_cfe_open_by_id(saved_id, &frontend) for managed Pi 5 FE capture. */
```

`cc_camera_open` and `cc_cfe_open` also verify any nonempty `info.id` against
the hardware they actually opened. A different ID/type returns `-ESTALE` and
closes the partial handle. Missing verification fails explicitly; it does not
fall back to the cached device path. ID-based lookup returns `-ENOENT` when no
current camera has that ID. Discovery rejects duplicate nonempty IDs with
`-ENOTUNIQ`, leaving the returned list empty. These checks also cover a device
replacement between discovery and opening.

When identity metadata is unavailable, discovery still lists the accessible
camera with an empty ID and path-based opening remains available. Unexpected
sysfs failures and malformed replies propagate. IDs are limited to 1023 bytes
plus the terminator, canonical sysfs paths to 4095 bytes, and camera lists to
4096 entries. Overlong values fail without truncation.

The examples print IDs with `ccamera-capture -l` and accept `-I ID` in place of
`-c INDEX` (or `-d` for the raw capture example). Quote IDs, as platform names
can contain spaces. For example, replace the ID below with the one from your
listing:

```sh
./build/ccamera-capture -I '/base/axi/i2c@1000/imx219@10' -m
./build/ccamera-still -I '/base/axi/i2c@1000/ov5647@36' -o photo.ppm
```

The still tool also accepts `--camera-id ID`. Camera indices are positions in the
current snapshot. Physical reconnect validation remains pending.
`cc_camera_info` has grown: rebuild C consumers against the new
experimental headers. Fixture regeneration needs the external reference archive;
normal builds and runtime do not:

```sh
python3 tools/generate_camera_id_vectors.py /path/to/libcamera-0.7.2-reference-ce4d60131c5f.tar.xz
```

## Camera hotplug

[`ccamera/monitor.h`](../include/ccamera/monitor.h) provides a caller-driven camera
registry. `cc_camera_monitor_open` subscribes before taking its initial discovery
snapshot and queues `CC_CAMERA_ADDED` for each usable camera. There are no worker
threads or callbacks. Drive it by dequeuing events, or poll its borrowed descriptor
alongside capture descriptors. The descriptor belongs to the monitor: do not read
or close it. Serialize calls on each monitor.

```c
#include <ccamera/monitor.h>

cc_camera_monitor *monitor = NULL;
int ret = cc_camera_monitor_open(&monitor);
if (ret < 0)
    return ret;

cc_camera_event event;
ret = cc_camera_monitor_dequeue(monitor, &event, 1000);
if (!ret && event.type == CC_CAMERA_RESYNC) {
    cc_camera_info *cameras = NULL;
    size_t count = 0;
    uint64_t generation = 0;
    ret = cc_camera_monitor_snapshot(monitor, &cameras, &count, &generation);
    if (!ret) {
        /* Replace the application's registry using this complete snapshot. */
        cc_camera_list_free(cameras);
    }
}
/* Process ADDED, REMOVED or CHANGED using event.camera and event.previous. */
cc_camera_monitor_close(monitor);
```

Dequeue accepts `0` for a nonblocking call (`-EAGAIN` when empty), `-1` for an
indefinite wait, or a positive timeout in milliseconds (`-ETIMEDOUT`). Waiting
accounts for interrupted waits and notifications that produce no camera event.
For an external event loop, poll `cc_camera_monitor_fd()` for `POLLIN`, then
dequeue with timeout zero until `-EAGAIN`. Dequeue dispatches when its event queue
is empty; `cc_camera_monitor_dispatch()` explicitly drains a bounded batch without
waiting. Keep polling while the descriptor remains readable. `cc_camera_monitor_refresh()`
forces reconciliation after an application changes permissions or a media graph.

The registry uses raw kernel uevents, `/dev` inotify notifications and bounded
retry backoff. Media-associated cameras wait for all character-device children
of that media graph, including bridges and auxiliary FE endpoints. Standalone
V4L2/UVC nodes can still be discovered without a media controller. Temporary
discovery/open/identity failures retain known cameras whose tracked device nodes
still exist. Missing or replaced nodes remove registrations; an observed delete
followed by creation produces removal before addition even if its location ID
stays the same. Raw kernel removals request reconciliation; inotify deletion
edges and node identities prevent an early or delayed raw notice from revoking a
newly created node. Incomplete arrivals retry, initially after 20 ms and eventually
once per second, while the application continues to drive the monitor.

Events contain copied camera information and a publication generation.
`CC_CAMERA_CHANGED` includes the previous value, for example when an ID first
becomes available or its media association changes. Snapshot copies are owned by
the caller and freed with `cc_camera_list_free`; snapshot does not dispatch and
its generation can be newer than an event still queued. Registry order is not
stable. After replacing application state from a snapshot, discard queued events
whose generation is at or below that snapshot's generation. Use IDs to identify
cameras and revalidate with `cc_camera_open_by_id`
or `cc_cfe_open_by_id` when opening from an event.

The queue holds at most 1024 events. A notification loss or application queue
overflow replaces queued history with `CC_CAMERA_RESYNC` and
`CC_CAMERA_EVENTS_LOST`; queue overflow also sets
`CC_CAMERA_EVENT_QUEUE_OVERFLOW`. Take a fresh snapshot instead of relying on
missing edges. Failed reconciliation preserves the published snapshot and
queued events, and schedules another attempt. An entire disconnect/reconnect
can be unobservable if notifications are lost and the node identity is reused.

The monitor owns no camera or capture handles. Applications handle capture
errors, release their requests and close or reopen devices as appropriate;
closing the monitor does not stop capture. Subscription requires kernel uevent
socket access and an inotify watch on `/dev`. There is no libudev dependency.
Physical Pi/USB unplug/reconnect and udev timing still need hardware validation.

The complete polling example is [`examples/watch.c`](../examples/watch.c):

```sh
./build/ccamera-watch
```

## Camera properties

`cc_camera_get_properties` returns a read-only `cc_camera_properties` value before
configuration or while capturing. `cc_cfe_get_properties` and
`cc_capture_get_properties` provide the same query for a managed CFE or an
adopted capture graph. Check the flags before using optional fields:

```c
cc_camera_properties properties;
int ret = cc_camera_get_properties(camera, &properties);
if (!ret && (properties.flags & CC_CAMERA_PROPERTY_MODEL))
    printf("Model: %s\n", properties.model);
if (!ret && (properties.flags & CC_CAMERA_PROPERTY_ROTATION))
    printf("Mounting rotation: %d degrees\n", (int)properties.rotation);
```

| Property | Sensor / VIMC | UVC / self-contained video |
| --- | --- | --- |
| Model | Entity model; I2C address/function suffix removed when recognized | UVC media model, otherwise video card name |
| Location | Orientation control default: Front, Back or External; absent if unsupported | UVC: USB `removable=fixed` means Front, otherwise External; generic video: absent |
| Rotation | Native sensor rotation control default, or zero when absent | Absent |
| Pixel array | `CROP_BOUNDS` size; largest advertised sensor mode as fallback; VIMC always uses the mode fallback | Largest advertised video size |
| Active area | `CROP_DEFAULT`; full pixel array as fallback; VIMC always uses the fallback | Full inferred pixel array |
| Unit cell | Static sensor profile dimensions, in nanometres | Absent |
| Colour filter | Lowest native Bayer/mono MBUS code, for native 8/10/12/14/16-bit codes; absent for VIMC | Absent |

Model conversion removes non-ASCII bytes using the reference policy. The sensor
location and rotation are control **defaults**, independent of current user
control values. Unknown location defaults map to External. Rotation preserves
the native V4L2 value, including non-quarter-turn values. Use the
[orientation API](#image-orientation) to negotiate and apply sensor flips.

Colour-filter queries undo current layout-changing horizontal/vertical flips
before selecting the native Bayer order, including code-order changes caused
by those flips. Mono is unaffected. If the driver exposes layout-changing flips
but cannot report their current values, colour-filter availability is cleared.
No controls, formats, routing, links or buffers are written by the query.

Geometry uses each advertised range's maximum, comparing area then width with
64-bit arithmetic. It does not synthesize intermediate sizes. Sensor queries
use the discovered image pad and stream, including multiplexed sensors whose
image stream is not zero. Size inference is bounded to 256 codes/formats and
4096 ranges; excessive replies return `-E2BIG`. Unsupported optional ioctls
leave the corresponding fields absent or use the documented fallback. A USB
attribute that cannot be read defaults to External, as in the reference.
Malformed replies return `-EPROTO`; unexpected driver errors propagate. On any
error the entire caller result remains unchanged. Unavailable fields are zero.
Serialize the query with all other operations on the device.

Lens/flash properties and additional metadata remain pending.

## Image orientation

`cc_camera_configure_oriented` combines the reference's sensor-orientation policy,
native flip controls and format negotiation. Configure before allocating image
or embedded buffers:

```c
cc_format raw = {.fourcc = V4L2_PIX_FMT_SRGGB10P, .width = 1920, .height = 1080};
enum cc_orientation orientation = CC_ORIENTATION_ROTATE_0;
int ret = cc_camera_configure_oriented(camera, &raw, &orientation);
/* Success returns actual orientation, dimensions and Bayer order in raw.
 * Use that returned format when configuring a downstream ISP. */
```

Orientations use the archived libcamera names and EXIF tag 274 numeric values:

| Value | Orientation | Value | Orientation |
| --- | --- | --- | --- |
| 1 | Rotate0 | 5 | Rotate90Mirror |
| 2 | Rotate0Mirror | 6 | Rotate270 |
| 3 | Rotate180 | 7 | Rotate270Mirror |
| 4 | Rotate180Mirror | 8 | Rotate90 |

The mounting rotation comes from the sensor rotation control's default. Missing
or non-quarter-turn mounting values use Rotate0 for negotiation, while the
property query still preserves the original native rotation value. Signed
quarter turns are normalized modulo 360.

As in `CameraSensorLegacy::computeTransform` and `CameraSensorRaw::computeTransform`,
both writable horizontal and vertical flip controls are required. The requested
orientation is divided by mounting orientation to find the sensor transform.
If flips are unavailable, or that transform requires a transpose, the policy
selects identity and adjusts orientation to the native mounting orientation.
For example, a sensor mounted at 180 degrees can produce Rotate0 by applying
both flips; a sensor mounted at 90 degrees cannot produce Rotate0 using flips
alone and returns Rotate90. No software rotation or sensor transpose is added.

`cc_camera_try_orientation` performs this selection without any writes and
returns both adjusted orientation and transform. It can be called before
configuration or during capture. `cc_cfe_try_orientation` provides the same
query for a managed CFE. The legacy `cc_camera_configure` and `cc_cfe_configure`
retain native flip settings; use the oriented variants to request an orientation.

Oriented configuration sets the supported flip controls before configuring
sensor, bridges and capture queues. On fallback, independently writable flips
are reset to zero. Fixed read-only flips are retained and reflected in the
returned actual orientation. A driver-adjusted Bayer order is propagated through
the raw path without changing bit depth or memory packing. Mono formats keep
their original packing. A bit-depth or Bayer/mono change is rejected.

`cc_cfe_configure_oriented` also updates both FE output Bayer orders, preserving
their depth, containers/compression mode, offsets and crop/scale configuration.
FE crops refer to the sensor readout after these flips. Use the returned sensor
and image formats when configuring downstream processing. Allocation checks
cover both image outputs and metadata queues, as for normal CFE configuration.

`cc_camera_get_orientation`, `cc_cfe_get_orientation` and
`cc_capture_get_orientation` read the mounting orientation composed with current
sensor flips. The graph getter describes its source; separately configured ISP
output transforms are additional. Self-contained video/UVC reports Rotate0,
and VIMC uses its sensor capabilities (normally no flips).

All setters return caller outputs only after complete success. A partial flip
write, rejected acknowledgement, sensor/bridge/video format failure or final
readback failure can leave hardware changed. Reconfigure before capturing after
an error. Allocated devices reject orientation changes with `-EBUSY`. Serialize
all queries and configuration with other device operations.

Pure helpers provide the complete eight-element transform algebra:
`cc_orientation_from_rotation`, `cc_transform_compose`, `cc_transform_inverse`,
`cc_orientation_transform` and `cc_orientation_apply`. Transform bits encode
horizontal flip, vertical flip and transpose, in that order. Composition applies
the first transform followed by the second, matching the archived `operator*`.
Invalid arguments return `-EINVAL` and leave output unchanged. Regenerate the
reference fixtures, if needed, with:

```sh
python3 tools/generate_orientation_vectors.py /path/to/libcamera-0.7.2-reference-ce4d60131c5f.tar.xz
```

## Sensor mode selection

`cc_camera_select_sensor_mode` chooses a sensor MBUS code and output size from
live enumeration. Supply accepted codes in preference order and positive minimum
width/height; set both maximum dimensions to zero for no upper bound. The
generic libcamera policy ranks candidates by closest aspect ratio, then smallest
area, then code preference. Each advertised range contributes its maximum size.
It does not invent intermediate modes or accept a mode smaller than either
requested dimension. Ratios are compared exactly and areas use 64 bits.

```c
#include <ccamera/ccamera.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

const uint32_t codes[] = { MEDIA_BUS_FMT_SRGGB10_1X10 };
cc_sensor_mode_request request = {
    .mbus_codes = codes, .num_mbus_codes = 1, .width = 1280, .height = 720
};
cc_sensor_format sensor;
int ret = cc_camera_select_sensor_mode(camera, &request, &sensor);
if (!ret) {
    /* This accepted MBUS code can be stored as packed RGGB10. */
    cc_format raw = {
        .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = sensor.width, .height = sensor.height
    };
    ret = cc_camera_configure(camera, &raw);
}
```

Selection changes no formats, routes, links, controls or buffers. It can be
queried before configuration or during capture, with calls serialized as usual.
The result contains sensor geometry, not a negotiated buffer layout; configure
the raw camera or the CFE sensor input separately, and use the actual returned
format. `cc_cfe_select_sensor_mode` provides the same query on a managed CFE.
The OV5647 still example uses equal minimum and maximum dimensions to require
the full-resolution 2592×1944 mode.

No fitting mode returns `-ENOENT`; a camera without a sensor returns `-ENOTSUP`.
Malformed replies return `-EPROTO`, driver errors propagate, and the output stays
unchanged on every failure. Enumeration is bounded to 256 advertised codes and
4096 size ranges across accepted codes; requests accept at most 256 codes.
Exceeding these bounds returns `-E2BIG`. Duplicate advertised codes are rejected.

### Raspberry Pi selection policy

`cc_camera_select_rpi_sensor_mode` and `cc_cfe_select_rpi_sensor_mode` implement
the archived libcamera 0.7.2 Pi pipeline's weighted policy. Supply desired
dimensions and sample depth along with the native Bayer/mono MBUS codes accepted
by your capture path. Depths of 8, 10, 12, 14 and 16 bits are supported; 12 is the
upstream pipeline's usual default. Accepted code order has no effect. Mono
14-bit support applies the same scoring to this port's additional native format;
the archived upstream format table lacks that code.

```c
const uint32_t pi_codes[] = {
    MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SRGGB12_1X12
};
cc_rpi_sensor_mode_request pi_request = {
    .mbus_codes = pi_codes, .num_mbus_codes = 2,
    .width = 1920, .height = 1080, .bit_depth = 12
};
cc_sensor_format pi_sensor;
int pi_ret = cc_camera_select_rpi_sensor_mode(camera, &pi_request, &pi_sensor);
```

For each range's maximum size, the score is width error plus height error, plus
1500 times aspect-ratio error, plus 500 per bit of depth difference. Each error
is twice the absolute difference when the candidate value is smaller than the
desired value, or one quarter of the difference when larger. The lowest score
wins. Equal scores prefer the greater numeric MBUS code, then larger area, then
larger width, matching upstream's sorted traversal and replacement on equal
score. This permits a mode smaller than either desired dimension. Unlike the
generic policy's exact ratio comparisons, Pi scoring uses upstream's `double`
arithmetic.

This query has the same read-only behavior, range handling, enumeration bounds
and output-on-error guarantees as the generic selector. An accepted code outside
the supported native Bayer/mono formats returns `-ENOTSUP`; zero codes and
invalid dimensions/depth return `-EINVAL`. It does not check whether a selected
mode reaches a desired frame rate. The fixture generator can reproduce 80
upstream comparisons from the documented archive:

```sh
python3 tools/generate_rpi_sensor_modes.py /path/to/libcamera-0.7.2-reference-ce4d60131c5f.tar.xz
```

The [role configuration layer](#stream-roles) selects and applies modes to supplied
source/processor devices. Frame-rate-aware mode filtering remains pending. The
archived Pi selector also uses only dimensions and bit depth; frame-duration
controls are handled separately through sensor timing. VIMC uses its scaler's
output conversion; UVC and self-contained V4L2 nodes use capture-node formats.

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

## Read-only pipeline validation

Use `cc_camera_try_format` to negotiate a source format and the following APIs
to validate an entire native processing configuration before applying it:

| API | Validation |
| --- | --- |
| `cc_cfe_try_configuration` | Sensor/CSI/FE TRY formats and routing, both cropped/scaled image outputs, packing/compression, statistics and parameter capacities |
| `cc_vc4_isp_try_configuration` | Exact source input layout, both output layouts, statistics format and a computed/checked crop |
| `cc_pisp_be_try_configuration` | Hardware-revision builder limits, crops, transforms, packing/compression, both output layouts, colour metadata, all four optional TDN/STITCH layouts and parameter capacity |

They take the same configuration structures as the corresponding `configure`
functions. On success they return negotiated layouts; on error the caller's
entire configuration stays unchanged. They preserve active formats, links,
controls, buffer allocations, the saved configuration and in-flight jobs. They
may update the device handle's separate TRY format/routing state. Optional image
or history ports opened for validation are closed afterwards on success and
failure; already owned ports stay open. Serialize validation with other calls on
each device, and do not use individual devices after a capture graph adopts them.

For example, with opened `frontend` and `backend` handles:

```c
#include <ccamera/cfe.h>
#include <ccamera/pisp_be.h>
#include <time.h>
#include <linux/videodev2.h>

cc_cfe_configuration fe = {
    .sensor = { .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = 1920, .height = 1080 }
};
int ret = cc_cfe_try_configuration(frontend, &fe);
if (ret < 0)
    return ret;
cc_pisp_be_configuration be = {
    .input = fe.image,
    .output = { .fourcc = V4L2_PIX_FMT_RGB24, .width = 1280, .height = 720 }
};
ret = cc_pisp_be_try_configuration(backend, &be);
if (ret < 0)
    return ret;
/* Apply only after all requested stages have passed validation. */
ret = cc_cfe_configure(frontend, &fe);
if (ret < 0)
    return ret;
be.input = fe.image; /* Use the actual applied producer layout. */
ret = cc_pisp_be_configure(backend, &be);
```

An input layout must already describe the producer's negotiated planes, strides
and capacities exactly. Validation can be performed while buffers are allocated
or a job is pending; applying a new configuration still requires freeing buffers.
If a driver rejects TRY negotiation, the error propagates without an active-format
fallback. VC4 has no TRY crop-selection ioctl: its validation computes/checks the
crop locally, and `configure` still applies and checks the actual driver selection.

CFE's basic TRY call uses the current sensor flips. The oriented variants
`cc_camera_try_format_oriented` and `cc_cfe_try_configuration_oriented` predict
the flips that configuration would apply, including fixed controls and resulting
Bayer order. They write no orientation controls. After actual oriented
configuration, use its returned layout for downstream processing. TRY success is
advisory: application still revalidates hardware and may negotiate different
values or fail. The native `get_configuration` getters copy applied layouts,
including metadata, without requiring allocated buffers; they return `-EPIPE`
before configuration or after invalidation.

## Stream roles

[`ccamera/configuration.h`](../include/ccamera/configuration.h) generates,
validates and applies image-stream configurations over opened source and optional
processor devices. Use it before `cc_capture_create` adopts the devices. Exactly
one `camera` or `frontend` is required, with at most one `vc4` or `backend`.
Processed Pi streams require a supplied processor. Device discovery, processor
acquisition, buffer allocation and IPA/tuning selection remain explicit.

Pi defaults follow the archived reference:

| Role | Initial format and size | Recommended buffers |
| --- | --- | --- |
| `CC_ROLE_RAW` | Largest advertised sensor output, preferred 12-bit samples and CSI2 packing | 2 |
| `CC_ROLE_STILL_CAPTURE` | YUV420 at the largest sensor output, sYCC | 1 |
| `CC_ROLE_VIDEO_RECORDING` | 1920×1080 YUV420, Rec.709 | 4 |
| `CC_ROLE_VIEWFINDER` | 800×600 XRGB8888 in native byte order, sYCC | 4 |

Validation returns actual native FOURCCs, dimensions, planes and capacities.
UVC exposes one stream regardless of roles: its default is the first supported
reference pixel-format order and largest discrete/recommended size. Generic V4L2
nodes use the same policy. For ranges, the reference's 53 common sizes are
filtered by native steps; no arbitrary intermediate size is invented. VIMC
defaults to 1920×1080 native RGB24 and applies its version-dependent geometry
and pixel-format rules. These sources cap edited configurations to one stream
and use four buffers.

```c
#include <ccamera/configuration.h>
#include <time.h>
#include <linux/videodev2.h>

/* camera and isp are already opened and remain caller-owned. */
cc_capture_devices devices = { .camera = camera, .vc4 = isp };
const enum cc_stream_role roles[] = { CC_ROLE_RAW, CC_ROLE_STILL_CAPTURE };
cc_pipeline_configuration config;
int status = cc_pipeline_generate_configuration(&devices, roles, 2, &config);
if (status < 0)
    return status;
config.streams[1].format.fourcc = V4L2_PIX_FMT_RGB24;
status = cc_pipeline_validate_configuration(&devices, &config);
if (status < 0)
    return status;
/* Inspect/revise adjusted fields before applying. */
status = cc_pipeline_configure(&devices, &config);
if (status < 0)
    return status;
/* Allocate source/processor buffers, then adopt devices with cc_capture_create.
 * config.buffer_count recommends a common pool; drivers may adjust its size. */
```

Success returns `CC_CONFIGURATION_VALID` (zero) or
`CC_CONFIGURATION_ADJUSTED` (positive). Test for **negative** errors, rather than
nonzero results. Caller structures and ownership stay unchanged on error.
Zero roles generate an empty configuration; validating/applying one is invalid.
Configurations hold at most three streams. Pi permits one raw and two processed
streams, and validation preserves application array order. Each stream's derived
`stream` field identifies the corresponding `cc_capture_result` port. The largest
processed stream is assigned to output0; a second one is bounded to its dimensions
and assigned output1. A single PiSP processed stream uses output1 for its wider
downscale range. The current BE controller also configures an internal output0
image, which appears in request results even when no role refers to it. VC4
likewise retains its internal auxiliary output.

Pi sensor selection uses the existing weighted dimension/aspect/depth policy.
Raw streams drive mode selection when present; otherwise the largest processed
stream does. `sensor_bit_depth` retains the raw sensor preference when FE packing
describes 16-bit/compressed samples. Change it explicitly, or set it to zero to
infer from an uncompressed requested format (compressed defaults to 12).
An all-zero `sensor` field selects automatically; otherwise it requires exact
sensor size and sample depth, with Bayer order following orientation. Generation
uses advertised sensor-output sizes, which may be smaller than physical pixel
array bounds. Derived `sensor_format` and `source_format` describe the selected
sensor and actual capture layout.

Orientation defaults to upright and is negotiated using sensor mounting/flip
capabilities. Set it to zero to preserve live flips; `applied_orientation` is zero
if those values cannot be read. Bayer raw formats are classified from their
FOURCC; for monochrome formats the Raw role distinguishes a sensor output from
processed luma. CFE normalizes raw packing to native 16-bit or PC1/PC2/PC3 output;
14/16-bit sensors use uncompressed output. Pi processed streams share the first
valid requested JPEG/SMPTE170M/Rec.709 colour space (including the sRGB alias),
with full-range RGB.

PiSP validates aspect crops and native scaling limits. On C0, padded RGB32 is
adjusted to RGB24/BGR24 because software packing is not implemented; D0 retains
supported native RGB32 formats. Broader software format/downscale fixups remain
pending. These configurations coordinate layouts and colour labels/matrices;
applications still supply matching gamma, calibration and IPA/tuning through the
existing APIs. The layer does not install an algorithm module implicitly.

Validation uses TRY state and may run with allocated buffers or pending jobs.
Apply checks **every** participating device is unallocated before the first
active write. A later native error may have changed hardware: all participating
saved configurations are invalidated, and the complete plan must be reapplied.
Configure optional embedded data, history ports or other native extensions after
applying the role plan, then allocate and adopt. The OV5647 still example uses
this layer with its required full-resolution 10-bit sensor mode and calibrated
RGB output.

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
