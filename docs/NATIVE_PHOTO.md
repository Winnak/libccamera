# Automatic native PiSP photographs

Pi 5 CFE/PiSP graphs with a calibrated photo IPA can acquire and process a
temporal sequence, a short/long exposure bracket, or both. The library schedules
sensor exposures, selects history, generates calibrated TDN/STITCH/tonemap and
shadow-gain parameters, and retires intermediate buffers. Image pixels stay in
the native ISP/DMA path. Applications still own encoding and file writing.
This path is **not hardware validated**; Pi 4/OV5647 cannot validate these blocks.

The supported HDR capture mode uses successive exposures on the selected sensor
image route, as in the reference PiSP MultiExposure path. It works with the ten
PiSP colour/NoIR/wide profiles. It does not require a sensor with simultaneous
exposure channels. Simultaneous CSI multi-image routes, sensor-internal WDR,
Night/SingleExposure HDR effects and software image fusion are not selected by
this API. VC4, raw-only, mono and uncalibrated graphs return `-ENOTSUP`.

## Setup and capture

Configure a full-size primary CFE image and a BE with the same input layout,
then configure history ports before allocating buffers:

```c
uint32_t features = CC_NATIVE_PHOTO_TDN | CC_NATIVE_PHOTO_HDR;
cc_pisp_be_configuration be_config = {
    .input = cfe_config.image,
    .compression_offset = cfe_config.compression_offset,
    .output = output_format,
};
int ret = cc_pisp_be_configure_native_photo(backend, &be_config, features);
/* Check ret, allocate CFE/BE buffers and adopt them with cc_capture_create.
 * Configure sensor defaults and attach cc_ipa_pisp_create's controller, as in
 * the ordinary calibrated PiSP path. */
cc_native_photo_options options = {.features = features};
ret = cc_request_set_native_photo(request, &options);
if (!ret) ret = cc_capture_queue(capture, request);
if (!ret) ret = cc_capture_dequeue(capture, &completed, timeout_ms);
/* Check result->status/error; consume result->output and native_photo metadata.
 * Export, release and reuse the request through the ordinary capture APIs. */
```

The configure helper negotiates lossless MSB-aligned Bayer16 histories with
zero offsets. The main FE image may still use any supported raw packing or
compression mode. Advanced users can configure history layouts themselves;
each reader must exactly match its writer, including stride, capacity, packing
and compression offset. STITCH history is after black subtraction and must use
offset zero. Both image outputs, BE crop/scale/flip and ordinary photo controls
remain available. Caller FE/BE tuning and manual history bindings conflict with
automatic native processing and are rejected.

`cc_capture_get_photo_capabilities` advertises `CC_PHOTO_CAP_NATIVE_TDN` and
`CC_PHOTO_CAP_NATIVE_HDR` when the adopted graph has the corresponding ports,
calibration, delays and at least two common slots. Combined TDN/HDR needs **three**
slots; either feature alone needs two. Allocation may return fewer buffers than
requested: check the common count. A missing/mismatched port returns `-EPIPE`,
and insufficient slots return `-ENOSPC` before queue ownership changes.

The CFE example accepts `-D tdn`, `-D hdr` or `-D both`, which imply `-q auto`.
It serializes photographs and prints native sequence metadata. For example,
append `-D both` to an existing calibrated CFE capture command. This adds no
encoder; `-o` writes the final native RGB/YUV plane bytes as before.

## Exposure and history policy

`temporal_frames=0` selects four frames per exposure channel; explicit values
are 2..8. HDR alone captures two frames; combined defaults capture four short
frames followed by four long frames. Every photograph starts fresh history.
Startup/mode-switch frames are discarded until the sensor's trust policy permits
them to seed the sequence. Only one automatic transaction may be owned at a time;
queueing another request while it is pending or held returns `-EBUSY`.

The calibrated IPA chooses the long exposure and gain, with the ordinary
manual/native overrides, frame-duration limits and mains policy. The short
shutter is derived from that snapshot: `hdr_ratio=0` selects 4, or explicitly
choose a long/short target in [2,8]. Both channels use the same requested gain
and line/frame timing. The sensor-specific exposure planner quantizes the short
shutter, respecting live bounds. Automatic short shutters also follow the mains
policy. If live bounds cannot produce at least a 2x bracket, the transaction
fails with `-ERANGE`. Actual fusion ratios use the delayed, driver-adjusted
exposure **products**, not the requested ratio. The library waits for each
stage's controls before allowing its image into the ISP.

TDN operates independently in each exposure channel. Its U2.14 history ratio
uses actual exposure products. A new channel, exposure jump outside [1/4,4),
untrusted history, reversed/nonadvancing source sequence/time or expired history
resets TDN and its spatial/colour denoise ramp. The profile's normal/HDR noise
parameters scale with gain; successful history frames gradually back off SDN
and CDN toward their with-TDN calibration. Denoise OFF conflicts with requested
TDN (`-EINVAL`). HDR alone can use denoise OFF.

Freshness allows the configured maximum sensor-control delay plus two frames
per stage. Timestamp expiry uses the greater of that frame-duration budget and
one second, so long exposures retain valid history. HDR allows the complete
long-channel sequence within that budget; a stale short frame fails with
`-ESTALE` instead of fusing unrelated exposures. Missing/empty short statistics
fail with `-ENODATA`. The short histogram determines the native tone curve;
its AWB regions determine calibrated, diffused spatial shadow gains. STITCH
uses the profile's saturation and motion thresholds and streams the long frame.

## Results, timeouts and cancellation

Only the final frame is delivered. Its raw image, statistics, embedded data,
expected sensor controls and IPA metadata describe the final exposure (long for
HDR). `result->native_photo` reports features, accepted frame count, TDN resets,
first/last source sequence, short/long shutter, final gain and actual HDR ratio.
The final processed image combines the internally acquired exposures; the raw
image remains a single sensor exposure. Intermediate frames are not exported.

A dequeue timeout leaves the transaction live at its current stage. Continue
dispatch/poll/dequeue or explicitly stop; do not requeue it. Histories remain
reserved through all input/output/configuration DMA completion. The library
releases obsolete histories immediately after their last consumer completes.
Stop, source/ISP errors and malformed packets cancel the entire transaction;
failed cancellation closes device handles before releasing references. Final
results have ordinary held-buffer semantics and release immediately frees all
transaction state. Request options persist on reuse; passing NULL to the setter
restores ordinary single-frame capture. Restart never reuses old history.

## Calibration and software evidence

Compiled calibration comes from the vendored, unchanged `*-pisp.json` files and
the archived Raspberry Pi controller's `denoise.cpp`, `hdr.cpp` and `pisp.cpp`.
The C implementation uses bounded arithmetic without a libm runtime dependency.
Motion thresholds are converted to the native /256 units before quantization.
The exposure bracket is a bounded still-photo policy using the existing normal
AE curve, rather than the reference's continuous multi-channel AGC cadence.

Independent Python/JSON fixtures check gain-dependent temporal noise, SDN/CDN
ramps and native histogram tone LUTs across all 12 PiSP profiles. Driver simulations
check exposure delays, stages, history ownership, timeout continuation, restart,
native-port failures and grouped cancellation, with both local and isolated
controllers. They inspect native jobs and ownership; they do not establish
physical image quality or pixel-level silicon behavior.
