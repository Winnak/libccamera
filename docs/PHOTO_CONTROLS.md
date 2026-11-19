# Raspberry Pi photographic controls

The calibrated VC4 and CFE/PiSP IPAs expose the same per-request controls in
[`photo.h`](../include/ccamera/photo.h). They cover all ten supported module
profiles, in process and through isolated workers. These are software-validated
controls; physical image quality and control-delay acceptance remain pending.
The [profile matrix](TUNING.md) describes module selection and calibration limits.
Generic/reference IPAs, raw-only CFE, UVC and VIMC do not implement this Pi photo
contract; their existing native or platform controls remain available.

## Capabilities and requests

After configuring/attaching a calibrated IPA, call
`cc_ipa_get_photo_capabilities()` or `cc_capture_get_photo_capabilities()`.
The result describes the configured sensor mode: supported control/mode masks,
defaults, exposure and frame-duration bounds, manual and automatic gain bounds,
native gain range/step, timing limits, startup/trust policy and actual configured
delays. With AF enabled it also contains the module's focus calibration and live
actuator range. Queries perform no device writes and preserve the output on
error. Query again after reconfiguration; these are mode limits, not a promise
that every combination of shutter and frame-duration bounds is attainable.
Automatic AE uses the profile's normal curve; its limits can be narrower than
manual exposure/gain limits. HBLANK stays at its configured value.
Immediate HBLANK writes are busy while a photo IPA is attached. Detach/reconfigure
before changing the line clock; direct IPA prepare rejects a changed line clock
until configuration is refreshed.
The capture query reports its installed trust policy, including explicit overrides;
the direct IPA query reports the sensor's recommended policy.

```c
cc_photo_capabilities caps;
int ret = cc_capture_get_photo_capabilities(capture, &caps);
if (!ret) {
    cc_photo_controls controls = {
        .flags = CC_PHOTO_EXPOSURE | CC_PHOTO_GAIN | CC_PHOTO_EV |
                 CC_PHOTO_METERING | CC_PHOTO_WB,
        .exposure_ns = 20000000, /* 20 ms manual shutter */
        .analogue_gain = 0,      /* automatic gain, independently */
        .exposure_value = 1,     /* +1 stop */
        .metering = CC_PHOTO_CENTRE_WEIGHTED,
        .white_balance = CC_PHOTO_WB_DAYLIGHT
    };
    ret = cc_request_set_photo_controls(request, &controls);
}
```

Only flagged fields are consumed. Every prepare resolves unspecified fields
against the controller's creation defaults; commands do not persist to other
requests. The setter copies commands, rejects changes to owned requests with
`-EBUSY`, and preserves existing commands on failure. A released/reused request
keeps its own commands until replaced or cleared with NULL. Generated settings
remain private to that execution. Syntax validation is also available as
`cc_photo_validate_controls()`.

| Control | Units and range | Default/behavior |
| --- | --- | --- |
| Exposure | Nanoseconds; live mode bounds | 0 selects automatic shutter |
| Analogue gain | Multiplier; live model/driver bounds within 1..64 | 0 selects automatic gain |
| Exposure compensation | Stops, -8..8 | 0; +1 doubles the luminance and highlight-constraint targets, bounded at full scale |
| Metering | Centre weighted, spot, matrix | Centre weighted; module's native region weights |
| White balance | Auto, incandescent, tungsten, fluorescent, indoor, daylight, cloudy, manual | Auto, or creation-time manual gains; presets restrict the calibrated automatic CT search |
| Manual WB gains | Red/blue multipliers, 0.5..4 | Both required in manual WB mode |
| Brightness | Additive fraction of the 16-bit gamma range, -1..1 | 0 |
| Contrast | Gamma contrast multiplier, 0..4 | 1, pivot at native value 32768 |
| Saturation | Colour-matrix saturation multiplier, 0..2 | 1; 0 produces equal RGB rows |
| Sharpness | Calibrated native strength multiplier, 0..4 | 1; 0 disables native sharpening |
| Denoise | Off, normal | Normal enables calibrated spatial and colour NR; native photo transactions can add temporal history |
| Flicker | Lighting mains frequency: 0, 50, 60 Hz | Creation default (NULL options: 50 Hz); only automatic shutter is quantized to lighting half-cycles |
| Frame durations | Minimum/maximum nanoseconds, both positive or both zero | Both zero select the physical mode bounds; blanking/exposure use the sensor-specific planner |

Manual times and gains quantize down to representable sensor lines/codes and
live steps. Request scheduling uses the configured sensor delays. With a shorter
VBLANK delay, a frame-length increase is written before shutter on the same frame
start. A decrease is written after the shorter shutter, at the configured delay.
A fixed frame ceiling can shorten a manual shutter and is reported
as a limit; frame timing follows the planner's documented native rounding.
Nonintersecting frame bounds or manual values outside live capabilities return
`-ERANGE`. Unknown control flags return `-ENOTSUP`; malformed values/enums,
including nonfinite numbers, return `-EINVAL`. A NoIR module explicitly rejects
WB presets: it supports trimmed grey-world auto and manual gains, with a nominal
4500 K processing temperature. Its automatic temperature capability bounds are
zero because grey-world does not estimate temperature.

Image controls modify native gamma, CCM, sharpening and denoise blocks before
processing. Metadata describes the normalized values used to construct those
blocks; register coefficients and LUTs have native quantization and clipping.
These controls do not introduce a software image-processing stage.
[Native PiSP transactions](NATIVE_PHOTO.md) add automatic TDN/HDR through
`cc_request_set_native_photo`; graph capabilities advertise the available features.

A request carrying photo controls requires a photo-capable IPA; queueing without
one returns `-ENOTSUP`. Mode/live-limit checks happen at prepare, after acquire
fences signal. A rejected setting fails that request and cancels the graph's
remaining work through the usual error lifecycle. No setting is silently ignored.

## Applied metadata and convergence

`result.ipa.flags & CC_IPA_METADATA_PHOTO` identifies the common contract.
`result.ipa.photo.flags` identifies valid normalized fields. Exposure, analogue
gain and red/blue gains contain the values for this completed frame, rather than
values selected for a later request. The main IPA exposure/gain/WB fields contain
the same values. Frame duration is in `frame_duration_ns`; CT is in Kelvin, lux
is the profile's scene estimate, and luminance is linear 0..1.

`photo_status` distinguishes:

- `CC_PHOTO_EXPECTED_SENSOR`: driver-acknowledged sensor values projected through
  configured delays. They are expected values, not sensor measurements. Embedded
  reports have separate `CC_IPA_METADATA_SENSOR` validity and native line/code data.
- `CC_PHOTO_STATS_TRUSTED`: startup/mode-switch policy permits feedback from this
  frame. Untrusted frames still report applied values, with no AE/AWB convergence,
  no feedback update, and zero luminance/lux. Missing/empty statistics do not
  establish automatic convergence.
- `CC_PHOTO_AUTO_EXPOSURE`, `CC_PHOTO_AUTO_GAIN`, `CC_PHOTO_AUTO_WB`: the automatic
  components of this request. Manual shutter/gain can be selected independently.
- `CC_PHOTO_LIMITED`: frame/exposure constraints or automatic exposure/gain
  saturation limited the requested result. `CC_IPA_EXPOSURE_LIMIT` also identifies
  automatic AE saturation; reaching a limit does not establish convergence.
- `CC_PHOTO_NATIVE_OVERRIDE`: explicit native commands replaced normalized domains.
  Affected image-policy flags are cleared; actual sensor and WB values remain in
  the main IPA metadata. Native red/blue/digital controls invalidate the normalized
  WB policy and AWB convergence, while preserving unaffected image-control flags.

Explicit native sensor commands win over corresponding normalized values.
Explicit FE/BE or VC4 tuning replaces the corresponding generated native block;
known ISP controls invalidate their respective normalized domains. Unknown native
ISP controls conservatively invalidate all normalized image-adjustment domains.
VC4 reads back scalar WB controls before processing, so driver adjustment is
reflected in that frame's metadata. PiSP reports the quantized native WB ratios.

Automatic AE convergence requires trusted valid luminance, a settled target and
matching expected/applied sensor settings. Fully manual AE acknowledges matching
applied settings without requiring a luminance measurement, after startup trust.
Automatic WB requires sufficient accepted colour regions and settled gains;
manual WB requires matching applied gains within native quantization. None of
these flags certify physical timing, colour accuracy or focus.

## Startup, timing and focus calibration

Attaching a calibrated photo IPA automatically installs the supported sensor's
startup/mode-switch hide and mistrust counts. An explicit
`cc_capture_configure_sensor_policy()` call, including NULL to disable filtering,
takes precedence. Delays still require `cc_capture_configure_sensor_defaults()`
or explicit driver/mode overrides before IPA attachment. Capabilities report
those configured delays; OV5647's defaults carry
`CC_PHOTO_CAP_PROVISIONAL_DELAYS`. They are not measured hardware delays.

Set `cc_ipa_vc4_options.autofocus = 1` (also used by PiSP) to opt into the native
lens controller. Only the four IMX708 profiles have supplied lens calibration.
A missing actuator, incompatible actuator range or module without AF calibration
fails configuration. AF defaults to manual mode with no lens write; use
`cc_request_set_af_controls()` for manual dioptres, single-shot/continuous AF,
range/speed/windows and pause/trigger/cancel commands.

Standard V3 maps 0..15 dioptres to codes 445..925; wide V3 maps 0..35 to 420..920.
Their normal/macro/full ranges, coarse/fine steps, slew, settling, confidence and
PDAF gains come from each module's vendored JSON; NoIR variants enable the
reference IR policy. `caps.focus` exposes the entire selected calibration.
Native actuator quantization/readback is converted back to calibrated dioptres
in frame metadata. AF updates run at no more than 30 Hz with valid monotonic frame
timestamps; commands can force an immediate update. Capture-graph restart and controller reconfiguration clear
controller phase/history. Valid timestamp-paired embedded PDAF drives hybrid AF;
missing/malformed packets clear phase validity and fall back to native contrast
statistics. Configure embedded capture before graph adoption to enable PDAF;
`CC_PHOTO_CAP_PDAF` advertises that configured path.

The standard/wide and NoIR calibrations have software convergence, native-code,
restart, malformed/missing-packet, request ownership and worker acceptance tests.
They remain **not physically AF validated**. The available OV5647 is fixed focus
and explicitly cannot provide physical AF/PDAF acceptance. HQ/IMX477 requires
manual focus of its attached lens; this API does not invent an actuator calibration.

**IPA ABI 9 / native wire version 4.** Rebuild C modules, workers and consumers
together. Older binaries are rejected. See [building/testing](BUILDING.md) and
the separate [Pi 4/OV5647 physical acceptance guide](PI4_OV5647.md).
