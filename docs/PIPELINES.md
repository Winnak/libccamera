# Pipeline queues and Raspberry Pi processing

[Project overview](../README.md). Run example commands from the repository root.

- [Pipeline queues](#pipeline-queues)
- [VC4 ISP processing](#vc4-isp-processing)
- [Pi 5 front-end capture](#pi-5-front-end-capture)
- [Pi 5 back-end processing](#pi-5-back-end-processing)

## Pipeline queues

[include/ccamera/stream.h](../include/ccamera/stream.h) provides the shared queue
layer used by camera capture and by further pipeline ports. It supports image
capture/output (single- and multi-plane), metadata capture/output, MMAP buffers,
and imported DMA-BUFs. These low-level operations do not choose an image
processor, configure media links, or invoke a codec. The Pi ISP processing
backends for VC4, the Pi 5 CFE front end and PiSP back end use this layer.

Open a `cc_video_device`, create the required `cc_stream` queue or queues, select
formats, and allocate buffers. Capture and output queues on a memory-to-memory
device share one open device handle, as required by the
[V4L2 M2M interface](https://docs.kernel.org/userspace-api/media/v4l/dev-mem2mem.html).
The device fd stays alive until its owner reference and all streams are closed.
Only one queue per direction can own an open handle, including the alternative
single/multi-plane or image/metadata interfaces.

Output buffers have writable mappings for raw input or ISP configuration data.
Fill the payload, set each plane's `bytes_used` (including `data_offset`), and
pass that `cc_frame` to `cc_stream_queue`. Capture queues take a NULL payload.
Metadata formats use one plane with `fourcc` as the metadata format and
`planes[0].size` as buffer capacity; dimensions and stride are zero unless the
driver exposes line-based metadata. Output timestamps pass through the kernel's
microsecond timestamp interface. `cc_stream_poll_events` selects POLLIN for
capture completion and POLLOUT for output completion.

To transfer a captured buffer to a processor, export its plane fds and bind them
to an output queue allocated with `CC_MEMORY_DMABUF`. The fds, not pixel bytes,
are passed to the importing driver, following the
[V4L2 DMA-BUF interface](https://docs.kernel.org/userspace-api/media/v4l/dmabuf.html).
Keep the producer buffer dequeued until that output queue completes or is
cancelled. Importing holds memory references; it does not automatically schedule
or enforce ownership across devices. In particular, an imported slot's fds stay
open after dequeue, until that slot is replaced or its buffers are freed. Release
the consumer's import queue before freeing the producer's MMAP buffers. Closing
duplicated fds with `cc_stream_unimport` releases the library's slot binding, but
the kernel can retain the previous DMA-BUF attachment until that slot is rebound
or the import queue is freed.

Stopping a stream returns all queued slots to the caller. Malformed completion
metadata makes the queue unusable until a successful stop. A failed release
leaves buffers inaccessible until `cc_stream_free_buffers` succeeds. If the
driver refuses cancellation/release during close, the shared device fd is
closed before memory is released; sibling streams then need to be closed too.
All calls sharing a device must be serialized by the caller.

Native controls on `cc_video_device` use the same scalar and compound-payload
implementation as camera controls. Layout-changing controls require all queues
on the device to release their buffers, and invalidate their configurations.
Image streams also expose the V4L2 crop/compose selection API. If a selection
changes the memory layout, buffers must be freed and the stream reconfigured.

## VC4 ISP processing

[include/ccamera/vc4_isp.h](../include/ccamera/vc4_isp.h) implements the bcm2835 ISP
path for Pi 4 and earlier. It discovers the input, main image, auxiliary image
and BSTA statistics ports from a locked media device. `cc_vc4_isp_open(NULL, ...)`
selects an accessible instance, or pass an explicit media path. The backend
follows the four-port interface in the
[Raspberry Pi bcm2835 ISP driver](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/staging/vc04_services/bcm2835-isp/bcm2835-v4l2-isp.c).

First configure the raw camera, then configure the ISP using that exact input
memory layout and a main output format. The backend rejects input adjustment
that would reinterpret producer bytes. It also configures the second image
output, defaulting to a smaller YUV420 stream, and the statistics stream. A zero
crop requests a centered crop matching the main output aspect ratio; an
explicit crop is expressed in raw input pixels. All negotiated layouts and the
actual crop are returned.

Allocate ISP buffer slots, dequeue a raw camera frame, export its plane fds, and
submit a processing job with a slot index and caller cookie. Submit duplicates
the fds; the caller can close its exports immediately. Dequeue returns both
image outputs, statistics, and the source frame metadata together. The raw
producer must keep the source buffer dequeued until that job completes or is
cancelled. A timed-out job stays pending and can be waited on again or stopped.
Result pointers remain valid until release/stop/free/close. Output plane fds can
also be exported for an external encoder or another processor.

The low-level ISP has one job in flight at a time, with additional slots for
held results. The request layer queues jobs and overlaps camera capture with
ISP processing while retaining each raw buffer through completion. A
driver error cancels the job on every port. If normal cancellation fails, the
backend closes all ISP handles before releasing its duplicated input fds.
Free/close the ISP before freeing raw producer buffers, because kernel imports
may retain attachments after processing completes.

The capture example can exercise the full camera-to-ISP path:

```sh
./build/ccamera-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -p -W 1280 -H 720 -F YU12 -n 10 -o frames.yuv -s statistics.bin
```

Select a raw sensor format/size supported by your camera. `-p` selects VC4 ISP
processing and `-i /dev/mediaN` can override discovery. Uppercase `-W`, `-H` and
`-F` specify processed output. The files contain concatenated payloads, without
an encoding or container. The example prints the actual output layout and crop
and releases downstream queues before the raw producer, including on timeout.

The low-level ISP API is manual; the owned capture graph optionally adds
[reference AE/AWB](IPA.md#c-ipa-modules-and-automatic-controls), enabled by `-g` in the example.
The [OV5647 still profile](PI4_OV5647.md) adds calibrated tuning. Broader
sensor tuning, sensor PDAF integration and mode-specific overrides remain pending. Native ISP
control values/payloads can be queried and set before submitting jobs. The
vendored [bcm2835 ISP UAPI header](../include/ccamera/linux/bcm2835-isp.h) supplies
compound control and statistics definitions even when distribution headers omit
the downstream interface. No image quality or physical capture claims are made
by the simulated-driver tests; both Pi generations still need hardware validation.

## Pi 5 front-end capture

[include/ccamera/cfe.h](../include/ccamera/cfe.h) owns the sensor, CSI routing,
front-end image0, optional image1, statistics and configuration queues on an RP1 CFE media device.
Open it using a discovered `CC_PIPELINE_RPI_CFE` camera, configure the sensor
mode, allocate buffer slots, then queue and dequeue jobs. One job is in flight
at a time; completed slots may be held while another runs. Release a slot after
consuming its image and statistics. A timeout retains the pending job. Errors
cancel every port, and closing restores modified media links.

The current path accepts 8/10/12/14/16-bit Bayer or monochrome sensor modes and
returns full-resolution 16-bit Bayer/mono by default. CSI expansion aligns the sample
to the most significant bits. Set `configuration.image.fourcc` to a matching
PiSP compressed mode 1/2/3 FOURCC (`PC1R`, `PC1G`, `PC1g`, `PC1B`, `PC1M`, or
their `PC2`/`PC3` equivalents) to use fixed-rate lossy raw compression. Mode 1 uses
delta coding; mode 2 uses companding; mode 3 combines both for HDR data with
up to 14 significant bits. Compressed rows have one byte per pixel,
with capacity for eight-pixel blocks and aligned stride. Modes 1/2 target up to
12 significant bits. None of the modes preserves every 16-bit sample exactly. PC3 uses the library's corresponding FOURCC convention.
These layouts follow the [kernel PiSP compressed-format definitions](https://docs.kernel.org/userspace-api/media/v4l/pixfmt-srggb8-pisp-comp.html).
The running driver must support the FOURCC; many Pi kernels expose mode 1 only.

`compression_offset` is an explicit `[0,65535]` black-level offset subtracted
before FE compression and added back by BE decompression. It defaults to zero;
use the same value in the producer and consumer configurations. Capture-graph
creation rejects mismatched offsets before adopting either device. Compression
enable/mode/offset derive from configuration, so per-request native tuning cannot
change this contract. Streaming CSI/FE links still carry uncompressed 16-bit
samples; compression applies to the FE memory writer and BE memory reader.

The sensor size is even and at least 64x64, with a statistics region no wider
than 12288 pixels. Each FE output can use its own even input-pixel `crop[]` and
`downscale[]` ratios. All-zero crop selects the full sensor; all-zero ratios
bypass scaling. Otherwise use `1 <= OUT <= IN <= 31` and `IN/OUT <= 16` on
each axis. The resulting dimension is `2*floor((crop_size/2)*OUT/IN)`, at least
16 pixels. Downscaler input width is limited to 6144 on branch 0 and 4096 on
branch 1; crop first for a wider sensor. Bayer phase correction is automatic;
`DOWNSCALE_BIN` optionally selects binning without that correction. Mono uses
no Bayer flag and limits horizontal OUT to 16. Statistics retain their own
sensor-space region, independently of image crops and scales.

Set `image1.fourcc` to enable the second FE image, using `compression_offset1`
for its independent compression offset. Results, polling, release and error
cancellation group both images and statistics. `CC_CFE_IMAGE1` selects its fd
and exported plane. `cc_cfe_allocate_outputs` and `cc_cfe_import_output` allow
independent MMAP/DMA-BUF storage. The convenience allocators use MMAP for image1.
The owned capture graph requires MMAP image1, returns `frontend_image1`, and
exports it through `CC_CAPTURE_FRONTEND_IMAGE1`.

The kernel video queues retain full-sensor dimensions for media link validation.
Native FE parameters specify the logical output dimensions returned in `image`
and `image1`; their stride and allocation capacity still cover the kernel
allocation envelope. Padding and unused trailing rows are unspecified. Keep the
returned stride/capacity when passing a logical output to the BE. Unpacked
10/12/14-bit output FOURCCs use LSB-aligned 16-bit containers and the FE output
shifter. Driver format support is required for every selected memory layout.

`cc_cfe_build_parameters` generates a complete native `pisp_fe_config` in C17,
including I/O geometry, AXI defaults, statistics grids, optional statistics crop,
and lens-shading/decimation finalization. NULL tuning enables uniform AGC, AWB and
focus statistics. Callers can supply the documented native processing blocks and
enable flags; DMA addresses and I/O fields are always generated internally. The
vendored [PiSP UAPI headers](../include/ccamera/linux/pisp/) define configuration and
`pisp_statistics` records. There is no libpisp link or build dependency. The
configuration equations are adapted from
[libpisp v1.5.0](https://github.com/raspberrypi/libpisp/blob/v1.5.0/src/libpisp/frontend/frontend.cpp),
and queue/routing behavior follows the
[RP1 CFE driver](https://github.com/raspberrypi/linux/tree/rpi-6.12.y/drivers/media/platform/raspberrypi/rp1_cfe).

The separate front-end capture example writes image and statistics payloads:

```sh
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -n 10 -o bayer16.raw -s statistics.bin
```

Use `ccamera-capture -l` and `-m` to choose a camera and sensor mode. The output
contains the negotiated stride/padding and no container or encoding. Exported
image/statistics DMA-BUFs are caller-owned; finish downstream DMA before releasing
a result, and free downstream import queues before freeing CFE buffers. Sensor
controls are immediate native V4L2 controls; the owned capture graph optionally
adds delayed scheduling and [reference AE](IPA.md#c-ipa-modules-and-automatic-controls).
Calibrated tuning and advanced algorithms are still pending. Simulated tests verify the
control and ownership paths; actual front-end capture remains unverified here.

## Pi 5 back-end processing

[include/ccamera/pisp_be.h](../include/ccamera/pisp_be.h) provides manual PiSP jobs
and a stateless C17 native configuration generator. `cc_pisp_be_open(NULL, ...)`
discovers an accessible instance by media entities; an explicit media path can
select one. Configure it with the exact producer memory layout and allocate
slots, then submit a frame's exported DMA-BUF plane. Input, output0 and RPBC
configuration completion are grouped under a caller cookie, including optional
`configuration.output1` when its FOURCC is nonzero. Only one job is in
flight; completed output slots can be held while another runs. Errors cancel
every port; a timeout retains the job. Free/close the back end before freeing
producer buffers because kernel DMA-BUF attachments can outlive completion.

The current generator supports Bayer and mono in uncompressed 16-bit containers
and PiSP compression modes 1/2/3, producing cropped, resized and flipped RGB/YUV
or GREY/Y16 luma, with C tiling within the Pi 5's 640-pixel tile width, 16-pixel filter
context and 64-tile limit. Bayer16/Y16 is MSB-aligned; unpacked 10/12/14-bit
FOURCCs are LSB-aligned and expanded with the native input shifter. Mono bypasses
demosaic and duplicates the sample into all RGB channels. GREY/Y16 outputs
apply the selected luma matrix and range. It programs Bayer demosaic defaults
and accepts native manual DPC, GEQ, spatial/colour denoise, black level, white balance gains, lens
shading, chromatic-aberration correction, debin, tone mapping, colour matrix,
saturation control, YCbCr conversion/inverse, false-colour suppression, sharpening
and gamma blocks. It generates I/O geometry and DMA offsets; caller-supplied
DMA addresses are ignored and left for the kernel driver. Native definitions
are vendored, with no libpisp dependency. Queue behavior follows the
[kernel PiSP BE interface](https://docs.kernel.org/admin-guide/media/raspberrypi-pisp-be.html)
and [RPBC configuration ABI](https://docs.kernel.org/userspace-api/media/v4l/metafmt-pisp-be.html).
The configuration/tiling conventions are based on
[libpisp v1.5.0](https://github.com/raspberrypi/libpisp/tree/v1.5.0/src/libpisp/backend).

Manual BE jobs also support temporal denoise (TDN) and HDR fusion (STITCH).
`configuration.auxiliary[port - CC_PISP_BE_TDN_INPUT]` describes the TDN input,
TDN output, STITCH input and STITCH output, respectively. Set a raw FOURCC to
enable a port; dimensions and Bayer/mono order match the main input. Packing,
stride and compression offset can differ per port. Managed configuration fills
zero dimensions/strides/capacities; explicit producer layouts must be preserved.
Inputs use DMA-BUF imports and raw outputs use MMAP storage.

Tuning enables `PISP_BE_BAYER_ENABLE_TDN` or `PISP_BE_BAYER_ENABLE_STITCH`;
all auxiliary I/O and compression flags derive from configuration. TDN requires
an output. Use `tdn.reset=1` without a history input to initialize its LTA frame,
then `reset=0` with a previous LTA frame. `tdn.ratio` is U2.14; black level,
noise constant/slope and threshold use native register units. STITCH requires
an input; its native U0.15 exposure ratio maps long-exposure pixels to the short
exposure's range, with the top bit selecting a long streaming input. Threshold
separation is `1 << threshold_diff_power` (0..15). A zero motion reciprocal is
finalized from its threshold. These are manual parameters, without automatic
HDR policy or calibrated noise tuning.

`cc_pisp_be_submit_with_auxiliary` takes TDN/STITCH source descriptors and frame
metadata. Enabled sources must stay dequeued until grouped completion or
cancellation. Inputs are duplicated; writable/source aliases return `-EEXIST`
before import. Keep a previous raw output slot held while another job reads its
exported fd. Results include all four `auxiliary[]` frames, with metadata-only
input views. `cc_pisp_be_export_port_plane` exports raw outputs. Configured raw
outputs are also written when TDN/STITCH processing is bypassed. Their tiling
covers the full source even when RGB outputs use smaller crops. Errors stop
all participating ports before releasing imported references. Owned capture
graphs currently reject auxiliary BE configurations; use the manual job API.

Each output has an independent input-pixel `crop[]`, target size and
`transform[]` (horizontal/vertical flip). An all-zero crop uses the full input;
zero requested output dimensions follow that crop. Their memory layouts,
colour encoding and quantization are independent; their common processing and
gamma path is shared. YUV conversion supports V4L2 601/709 encoding and
full/limited range, including UV/VU and packed-byte ordering. RGB limited range
is supported too. Default colour fields follow V4L2 conventions; the managed
path defaults transfer to linear until callers supply corresponding gamma tuning.
The public output metadata describes the processing submitted to PiSP, including
on kernels whose V4L2 node format fields are forced to unrelated defaults. No
gamma curve is inferred from a transfer-function label.

Manual tuning uses native register units from the
[PiSP specification](https://datasheets.raspberrypi.com/camera/raspberry-pi-image-signal-processor-specification.pdf).
SDN accepts both noise-estimation pairs (normally equal without temporal
denoise); CDN accepts its independent threshold and IIR controls. Debin corrects
the spatial alignment of binned samples without changing image dimensions.
Tone mapping accepts its 64-entry packed position/slope table and 12-bit IIR and
detail strengths. Saturation shifts are 0, 1 or 2.

LSC/CAC grid steps of zero are calculated from the full input size. Explicit
steps and `lsc_extra`/`cac_extra` pixel offsets support a subregion of a larger
calibration grid; coverage is checked over the whole input, including every
tile's filter context. CAC uses a 9x9 grid of signed 7-bit red/blue XY shifts,
in units of 1/32 pixel. The automatic CAC grid needs input dimensions above 128;
smaller images need explicit steps for larger cells covering the image subregion.

Sharpening and false-colour suppression require both `YCBCR` and
`YCBCR_INVERSE` enable flags and caller-supplied forward/inverse matrices. This
shared internal YCbCr path is independent of the per-output CSC. The native
`sh_fc_combine` block controls their combined response. Sharpen kernels have
coefficients in `[-32,31]`, a positive sum at most 31 and a negative sum at least
-32; native masks, slopes, scales and strengths are validated too. Padding is
cleared before submission. Rejected tuning leaves builder output unchanged and
does not import source buffers into a managed job.

Managed configuration reports the media node's `hardware_version` and uses it
for register checks. The standalone extended builder accepts
`CC_PISP_BE_VERSION_BCM2712_C0` or `CC_PISP_BE_VERSION_BCM2712_D0`; zero uses
common C0 limits. C0 colour-matrix offsets fit `[-2^26,2^26)`, while D0 permits
`[-2^28,2^28)`. Unknown nonzero revisions return `-ENOTSUP`. The single-output
convenience builder uses the common range. These checks prevent silently
truncated matrix offsets on C0 hardware.

Both branches have a 6-tap/16-phase resampler. Pi 5's output 1 additionally has
the trapezoidal downscaler used for larger reductions; output 0 uses the
resampler alone. Filter tables and integer trapezoidal-filter generation are
ported from libpisp, without JSON, floating-point math libraries or runtime
configuration files. Native resampler factors `(input-1)/(output-1)` must be
within `[1/16,16)`; the output-1 downscaler can add up to 8x reduction. Tile size,
overlap and minimum-size limits also apply: an unsupported geometry returns
`-ERANGE`, and exceeding 64 tiles returns `-E2BIG`. Input/crop dimensions must
be at least 16 pixels; YUV420/422 output width is even and YUV420 height is even.
RGB dimensions may be odd. Custom native S2.10 resampling coefficients can be
supplied through the tuning block using `RESAMPLE0/1` enable flags.

| Output layout | FOURCC | Memory planes |
| --- | --- | --- |
| GREY/Y16 luma | `GREY`, `Y16 ` | 1 |
| RGB24/BGR24 | `RGB3`, `BGR3` | 1 |
| Padded RGB32 (D0 only) | `XB24`, `BX24`, `XR24`, `RX24` | 1 |
| RGB48/BGR48 (16 bits per channel) | `RGB6`, `BGR6` | 1 |
| Packed YUV422 | `YUYV`, `UYVY`, `YVYU`, `VYUY` | 1 |
| Semi-planar YUV420 | `NV12`, `NV21` / `NM12`, `NM21` | 1 / 2 |
| Semi-planar YUV422 | `NV16`, `NV61` / `NM16`, `NM61` | 1 / 2 |
| Planar YUV420 | `YU12`, `YV12` / `YM12`, `YM21` | 1 / 3 |
| Planar YUV422 | `422P` / `YM16`, `YM61` | 1 / 3 |
| Planar YUV444 | `YM24`, `YM42` | 3 |

Padded RGB32 follows V4L2's byte-order names: RGBX32 (`XB24`) stores R,G,B,X;
XRGB32 (`BX24`) stores X,R,G,B; XBGR32 (`XR24`) stores B,G,R,X; BGRX32 (`RX24`)
stores X,B,G,R. X bytes have unspecified values. This mode requires D0 silicon;
the standalone extended builder needs an explicit D0 `hardware_version`, while
managed configuration uses the media device's reported revision. RGB48/BGR48
store each 16-bit channel little-endian on both C0 and D0. Formats must also be
accepted by the running kernel driver; some kernels expose only a subset.
These mappings follow the [V4L2 RGB definitions](https://docs.kernel.org/userspace-api/media/v4l/pixfmt-rgb.html)
and [libpisp variant limits](https://github.com/raspberrypi/libpisp/blob/v1.5.0/src/libpisp/variants/variant.cpp).

For contiguous planar 420/422, the Y stride must also keep each chroma stride
16-byte aligned. Separate chroma planes can use strides independent of the Y
stride, with equal U/V strides. The managed controller negotiates capacities
again if a driver widens chroma strides without growing the allocation.
`cc_pisp_be_build_configuration` generates parameters for both outputs;
`cc_pisp_be_build_parameters` remains the single-output convenience form.

The example connects sensor/CFE capture to PiSP through DMA-BUFs:

```sh
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -p -F RGB3 -n 10 -o linear.rgb -s statistics.bin
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -p -F NV12 -A YM12 -a auxiliary.yuv -n 10 -o main.nv12
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -p -r 0,154,1640,924 -W 1280 -H 720 -F NV12 \
    -A YM12 -U 640 -V 480 -a auxiliary.yuv -t 1 -o main.nv12
```

`-i /dev/mediaN` overrides BE discovery. `-A FOURCC` or `-a path` enables the
second output, defaulting to `YU12`; `-a` writes it. Each file concatenates its
memory-plane payloads, including padding. The example prints all plane layouts
and colour metadata. `-W/-H` choose the main size; `-U/-V` choose the auxiliary
size. `-r/-R` supply the respective crops; `-t/-T` choose flips (0 none, 1
horizontal, 2 vertical, 3 both). The default output derives from **linear sensor RGB**,
with no automatic black level, white balance, colour correction or gamma;
it is not a tuned sRGB photograph/video. Applications supplying manual processing
must match their colour metadata to it. RGB/YUV input, additional native output
layouts, owned-graph TDN/HDR history assignment, HOG output, richer tuning and
advanced automatic algorithms are pending. `-g` adds reference AE/AWB but
does not select calibrated image tuning or gamma. The tests simulate queue/DMA ownership
and inspect parameters; actual pixel processing and image quality require Pi 5
hardware validation.

To capture compressed raw memory or feed it through the BE, select an FE output
with the sensor's Bayer order. For example, `pRAA` is RGGB10 and pairs with `PC1R`:

```sh
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -b PC1R -k 2048 -n 10 -o frames.pc1
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA \
    -b PC1R -k 2048 -g -p -F NV12 -n 60 -o frames.nv12
```

`-b` selects the FE memory FOURCC and `-k` its offset (default zero). The example
copies the same offset into BE configuration. Raw `.pc1` files contain only
memory-plane payloads; consumers also need the dimensions, stride, Bayer order,
compression mode and offset. Mode/offset are not embedded in each frame. This
uses hardware raw-memory packing and introduces no encoder or codec dependency.
