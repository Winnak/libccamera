# Sensor controls and metadata

[Project overview](../README.md). Run example commands from the repository root.

- [Sensor models, gain and timing](#sensor-models-gain-and-timing)
- [Sensor control scheduling](#sensor-control-scheduling)
- [Sensor embedded data](#sensor-embedded-data)

## Sensor models, gain and timing

[include/ccamera/sensor.h](../include/ccamera/sensor.h) exposes static cell-size and
known-delay properties for 36 sensor models from the archived reference. Feature
flags distinguish missing information from a real zero delay. The implemented
gain and exposure helpers cover the 17 model names handled by the reference
Raspberry Pi helpers: IMX219, 283, 290, 296, 327, 415, 462, 477, 519, 678, 708;
OV5647, 64A40, 7251, 9281; and VD55G1/56G3. IMX708 wide/NoIR aliases share its
canonical profile. Other known models can expose properties while unsupported
gain/exposure operations return `-ENOTSUP`.

`cc_sensor_lookup` accepts an exact model token or a media entity name such as
`imx219 10-0010`. It does not match arbitrary substrings or invent properties for
unknown names. `cc_camera_get_sensor`, `cc_cfe_get_sensor` and
`cc_capture_get_sensor` identify the actual sensor entity, independently of the
camera's display name. Their returned handles and property pointers are static
borrowed data.

`cc_camera_get_sensor_streams`, `cc_cfe_get_sensor_streams` and
`cc_capture_get_sensor_streams` return live image/embedded source pad IDs, stream
IDs, media-bus codes, dimensions and sensor-route active flags. Stream-aware
sensors are classified through their routing and format enumeration, including
the internal embedded-data format; legacy sensors use their source pads.
Image and embedded IDs need not be 0/1. Ambiguous image or embedded streams
return `-ENOTUNIQ`; self-contained video nodes return `-ENOTSUP`. Output remains
unchanged after a failed query. These are native stream IDs, independent of CSI
virtual-channel numbers and of capture buffer indices.

Raw configuration and sensor-mode enumeration use the discovered image stream.
The first stream-aware bridge/CSI receiver maps that incoming ID onto output
stream 0, which is propagated along the remaining image path. Fixed-pad bridges
reject nonzero incoming IDs instead of silently dropping the selection. TRY
routing remains separate from ACTIVE routing. Current and older routing ioctl
layouts work with old installed Linux headers. Discovery is bounded to 64 sensor
routes and 256 media-bus codes per stream; malformed replies fail explicitly.
Image configuration activates its sensor route and disables the recognized
embedded route, preserving unrelated internal routes. Configure embedded capture
after the image format to add its receiver route and metadata queue.

On an idle, stopped capture graph, `cc_capture_configure_sensor_defaults` selects
the known profile delays, omits unavailable/read-only controls and enables the
same frame-start scheduler. Unknown models return `-ENOENT`; known models with
missing delay data return `-ENODATA`. There is no guessed fallback. For example,
IMX219 has a one-frame gain delay, while IMX477 has a three-frame blanking delay.
Explicit `cc_capture_configure_sensor_delays` overrides remain available for a
particular driver/mode. OV5647 defaults retain the reference's driver-dependent
qualification in `CC_SENSOR_DRIVER_DEPENDENT_DELAYS`; they are not a universal
mainline-driver guarantee.

The gain helpers convert between native analogue-gain codes and linear gain.
They support reciprocal, linear and logarithmic register models without libm.
Logarithmic values are checked-in binary64 tables, with monotone lookup for
encoding. Code round trips are stable, gain is rounded down, and mathematical
model bounds are checked. `cc_sensor_quantize_gain` additionally intersects the
driver's current `V4L2_CID_ANALOGUE_GAIN` min/max/step and returns a native control
plus its model gain. A later driver write can adjust it again; delayed metadata
continues to retain that acknowledged value.

`cc_camera_get_sensor_timing`, `cc_cfe_get_sensor_timing` and
`cc_capture_get_sensor_timing` read the current pixel rate, HBLANK and VBLANK
with the sensor's configured output size. The pure timing helpers use pixel
clocks and blanking, following the register-list sensor convention in the
reference; buffer stride and ISP output size are not timing inputs. See the
[kernel sensor timing interface](https://www.kernel.org/doc/html/latest/userspace-api/media/drivers/camera-sensor.html)
for the driver timing domain. Exposure conversion floors to lines; inverse
conversion rounds nanoseconds up so the same line code can be recovered. IMX296
uses its fixed shutter period and 14.26-microsecond offset. Frame duration uses
both horizontal and vertical blanking. Checked 128-bit multiply/divide is
implemented using standard C17 `uint64_t`, with no compiler extensions or extra
libraries. Overflow leaves output unchanged.

`cc_camera_get_sensor_timing_limits`, `cc_cfe_get_sensor_timing_limits` and
`cc_capture_get_sensor_timing_limits` read native blanking ranges, exposure
minimum/step and pixel rate for the active mode. Read-only blanking is fixed to
its current value. `cc_sensor_timing_bounds` returns the physical duration bounds.
`cc_sensor_plan_exposure` chooses native exposure, HBLANK and VBLANK using the
RPi helper margins and shutter equations. Zero duration bounds use the physical
limits; explicit bounds are clamped. Native steps round down, so a requested
minimum duration can be undershot slightly. Integer line periods and returned
nanoseconds are checked without libm. IMX477/519/708 frame lengths are rounded
for shifts 0..7; submit the full logical VBLANK/exposure codes, leaving register
shifts to the sensor driver. This pure planner performs no I/O.

`cc_sensor_get_mode_policy` reports sensitivity, startup/mode-switch discard and
statistics-mistrust counts, and minimum debinning factors for a supplied actual
mode. Querying policy does not automatically discard capture frames or suppress
statistics. Those runtime policies, test-pattern mappings and further generic
sensor helpers remain pending. Timing reads describe current driver settings;
delayed metadata may describe earlier values. Physical camera validation is
still required.

## Sensor control scheduling

The capture graph can optionally stagger native sensor-control writes so that
controls with different delays are expected to take effect on the same frame.
Call `cc_capture_configure_sensor_delays` on a stopped graph with no owned
requests. Supply each writable scalar control ID, its delay for the actual
sensor/mode, and any `CC_SENSOR_CONTROL_PRIORITY` flag. Blanking controls that
change the permitted exposure range can use priority writes; these are sent
separately before the remaining batch. No sensor delay is selected implicitly;
`cc_capture_configure_sensor_defaults` explicitly opts into known model defaults.

Configuration reads the baseline controls and subscribes to
[V4L2 FRAME_SYNC events](https://docs.kernel.org/userspace-api/media/v4l/vidioc-dqevent.html)
on the source capture node. Unsupported events, payload/volatile/read-only
controls, layout changes and invalid delay specifications are rejected. The
current limits are 32 managed controls, delays up to 64 frames, and a 128-frame
planning/history window. Managed IDs cannot also be changed through immediate
`cc_capture_set_controls`; disabling delay scheduling with NULL/0 restores
immediate access.

After configuring delays, attach native scalar values to individual requests
with `cc_request_set_sensor_controls`. The graph selects the next reachable
effective sensor frame, staggers the writes and internally recycles earlier
source frames. Successful results and downstream processing receive only frames
at or after the settings become effective. This works with camera and CFE
sources; the CFE image/statistics/configuration buffers retire together.
The current one-job source controllers can require settling frames for each
changed request; look-ahead scheduling remains future work.

For example, with the graph's sensor delays already configured, queue a native
exposure value chosen from the driver's live exposure range:

```c
#include <ccamera/capture.h>
#include <linux/videodev2.h>

int queue_exposure(cc_capture *capture, cc_request *request, int64_t exposure_lines)
{
    cc_control control = { .id = V4L2_CID_EXPOSURE, .value = exposure_lines };
    int ret = cc_request_set_sensor_controls(request, &control, 1);
    return ret < 0 ? ret : cc_capture_queue(capture, request);
}
```

The setter copies values and accepts only idle requests; NULL/0 clears them.
Queueing rejects unknown/unmanaged IDs, inappropriate scalar widths and missing
delay configuration before taking ownership. Driver adjustments appear in
`result.sensor`, with the request's cookie as its effective context. Request
settings persist on release/reuse. Requests without sensor settings capture the
next frame using the preceding control context. Settling-frame buffer errors
fail the affected request immediately rather than being hidden. Stop and graph
failures cancel pending settling work through the normal request lifecycle.

For explicit scheduling instead, use `cc_capture_schedule_sensor_controls` with
a **V4L2 sensor frame sequence**, a sparse list of native values, and a context cookie. This is
independent of the request sequence/cookie. Target frames must allow each write
to occur on a future frame start: a control with delay D is written at target-D.
Multiple schedules for one target merge; unspecified controls retain their
previous effective values. Scheduling rejects unknown/duplicate IDs, payloads,
late writes and targets outside the planning window without changing the plan.
32-bit sensor-sequence wrap is handled internally. Explicit sequences and
request-associated controls cannot be mixed between stops: the conflicting API
returns `-EBUSY`. A successful stop clears this choice along with pending plans.

The request result's `sensor` field snapshots the expected controls and context
for `raw.sequence`. The baseline context is zero. Returned values include any
adjustment acknowledged by `VIDIOC_S_EXT_CTRLS`; this avoids reporting a
requested value that the driver clamped. Metadata is a model based on supplied
delays, rather than a measurement from embedded sensor data. Each request keeps
its snapshot even when global history has been overwritten.

Dispatch services SOF events before frame completion, and `cc_capture_pollfds`
includes POLLPRI while scheduling is enabled and the source is running. Service
these events promptly, including while results are held. Backlogged events are
drained before choosing the current write deadline. A lost event, missed write,
new frame during a write, expired metadata or failed control update cancels
the graph and clears result views. A frame whose SOF notification is pending
waits for it with the existing timeout; its image is not dequeued twice.
Stop discards future plans, unsubscribes events and reads a fresh baseline for
restart. Failure to stop/unsubscribe closes the owned graph before releasing
buffers.

Look-ahead control scheduling across multiple queued hardware jobs, automatic
mode-specific overrides, and validation against embedded/physical sensor behavior
remain to be implemented. The low-level camera/CFE control APIs still apply
controls immediately. Delay scheduling itself adds no IPA, threads or encoding.

## Sensor embedded data

[include/ccamera/embedded.h](../include/ccamera/embedded.h) provides pure C decoders
for received sensor buffers. `cc_smia_parse` reads tagged registers in 8/10/12/14-bit
packing, with a fixed byte stride or line-start search and an optional line limit.
It checks every tag/dummy-byte boundary and rescans each frame, so a layout change
cannot reuse stale offsets. Up to 64 distinct requested addresses are supported;
repeated reports do not count as distinct registers. Truncated or malformed
packets and missing registers leave the caller's output unchanged.

`cc_sensor_parse_embedded_status` reports native exposure, gain and line/frame
length registers for IMX477/519/708, decoded linear gain and available temperature.
These are values from the supplied buffer, separate from the graph's modelled
delayed-control snapshots. Long-exposure shifts are absent from these reports;
callers must retain the modelled exposure/frame length when using that mode.
The typed decoder excludes IMX219, following the reference helper's restriction
on its unreliable embedded reports. The generic SMIA reader remains available.

For IMX708, `cc_sensor_parse_pdaf` decodes the third embedded scanline into a
12-by-16 confidence/phase grid. `cc_sensor_parse_hdr_statistics` decodes the fourth
scanline where present, checks histogram markers and linear/logarithmic overlap,
and returns a linear histogram and raw 16-bit-scale mean. It widens products
before accumulating large histogram counts. Both accept 10/12/14-bit packing.
Pass the actual metadata scanline start/length, accounting for capture data
offsets and byte stride; do not infer these from a processed image's dimensions.
For example, after obtaining an IMX708 phase scanline:

```c
#include <ccamera/embedded.h>

cc_sensor_pdaf pdaf;
int ret = cc_sensor_parse_pdaf(sensor, 10, phase_line, phase_line_bytes, &pdaf);
if (ret == 0) {
    cc_af_pdaf_grid grid = {pdaf.rows, pdaf.cols, pdaf.regions};
    ret = cc_af_prepare(af, &grid, &status);
}
```

Feature flags `CC_SENSOR_EMBEDDED`, `CC_SENSOR_PDAF` and
`CC_SENSOR_HDR_STATISTICS` indicate decoder support. These functions perform no
device I/O or stream configuration.

To capture packets, call `cc_camera_configure_embedded` or
`cc_cfe_configure_embedded` after image configuration and before allocating any
buffers. Both discover and own the associated `unicam-embedded` or
`rp1-cfe-csi2-ch1` node, configure sensor/receiver routes, and return actual payload
geometry and packing. Generic 8/10/12/14-bit metadata and legacy `SENS` buffers
are supported. Shared receiver image formats are retained, including CFE's
16-bit image transport. For example, before allocating CFE buffers:

```c
#include <ccamera/embedded.h>

cc_embedded_configuration embedded;
int ret = cc_cfe_configure_embedded(frontend, &embedded);
```

Passing NULL disables embedded capture while buffers are unallocated. Image
reconfiguration clears it. A partially failed native setup requires image
reconfiguration. Allocation reserves an extra metadata buffer and can reduce
the returned usable image-slot count. There are no additional dependencies.

Image dequeue pairs metadata by timestamp, recycling stale packets and retaining
a future packet. The future packet proves that the current image has no match;
an empty metadata queue retains pending work across timeouts. Different image
and metadata indices/sequence counters are allowed. With two zero timestamps,
pairing instead requires matching native sequence numbers and handles wrap.
Incompatible declared clocks or mixed timestamp availability fail explicitly.
The request graph includes only outstanding ports in its poll set. Applications
using low-level nonblocking camera I/O should poll both image and embedded fds.

The paired bytes are available in `cc_capture_result.embedded`,
`cc_cfe_result.embedded`, or `cc_camera_embedded_frame` using the completed image
slot. A missing packet has zero planes in grouped results and returns
`-ENODATA` through the camera getter. Views remain held with the image/request;
release/requeue/stop/free invalidates them. Export with `CC_CAPTURE_EMBEDDED`,
`CC_CFE_EMBEDDED` or `cc_camera_export_embedded_plane`; returned fds are owned by
the caller. Queue/actuator failures cancel the graph and invalidate every view.

IPA ABI 4 carries borrowed paired bytes and their configured geometry. The
reference module reports `CC_IPA_METADATA_SENSOR`, `CC_IPA_METADATA_PDAF` and
`CC_IPA_METADATA_HDR` for successfully decoded components. Native sensor reports
remain separate from `result.sensor`'s expected delayed-control snapshot; long
exposure shifts are not invented. Decoded PDAF feeds the next eligible AF update.
Valid sensor HDR histograms supply linear luma with the reference's fourfold
headroom factor for AE. Missing/malformed components fall back independently,
without retaining old phase/HDR packets. The capture examples still default to
contrast AF; embedded capture is an explicit native API configuration. Sensor
HDR exposure policies/fusion, additional packet formats and physical validation
remain outstanding.
