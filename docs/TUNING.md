# Raspberry Pi still-photo tuning

The opt-in VC4 controller now selects camera-specific black level, gamma,
colour matrices, static lens shading, green equalisation, spatial/colour denoise
and sharpening. It also selects the profile's normal exposure curve, luminance
target, highlight constraint, lux calibration and white-balance policy. Tables
are compiled into the library; no runtime JSON reader, Python or libm is needed.

These are **software-tested profiles**. Physical image quality remains unverified.
The required hardware acceptance target is Pi 4 with standard-colour OV5647.
Pi 5 has a separate calibrated CFE/PiSP single-exposure path described below.
Neither controller accepts the other ISP pipeline. Both implement the
[common photographic controls and metadata contract](PHOTO_CONTROLS.md), including
module-specific opt-in V3 autofocus.

## Coverage and module selection

Sensor capture support and tuned-photo support are separate. The sensor helpers
recognize many more models than the four families with VC4 and PiSP profiles below.
An unknown photo sensor fails with `-ENOTSUP` rather than borrowing a calibration.

| Profile name | Module/lens | Automatic white balance | Limitations |
| --- | --- | --- | --- |
| `ov5647` | Standard Camera Module v1.3 | Calibrated Bayesian | Pi 4 hardware acceptance pending |
| `ov5647_noir` | NoIR v1.3 | Trimmed grey-world | No physical validation; IR illumination changes colour response |
| `imx219` | Standard Camera Module v2 | Calibrated Bayesian | No physical validation |
| `imx219_noir` | NoIR v2 | Trimmed grey-world | No physical validation; illumination-dependent colour |
| `imx477` | HQ, standard colour | Calibrated Bayesian | Generic reference shading; no calibration for a particular attached lens |
| `imx477_noir` | HQ with IR filter removed | Trimmed grey-world | Generic reference shading; no lens-specific or physical validation |
| `imx477_6mm` | Colour HQ with the reference 6 mm lens | Calibrated Bayesian | PiSP only. Lens-specific chromatic aberration correction (CAC), generic HQ shading |
| `imx477_16mm` | Colour HQ with the reference 16 mm lens | Calibrated Bayesian | PiSP only. Lens-specific CAC, generic HQ shading |
| `imx708` | Standard Camera Module v3 | Calibrated Bayesian | Opt-in calibrated AF; no physical validation |
| `imx708_noir` | Standard-lens NoIR v3 | Trimmed grey-world | No physical/AF validation; illumination-dependent colour |
| `imx708_wide` | Wide Camera Module v3 | Calibrated Bayesian | No physical/AF validation |
| `imx708_wide_noir` | Wide NoIR v3 | Trimmed grey-world | No physical/AF validation; illumination-dependent colour |

`CC_VC4_PROFILE_AUTO` assumes the standard-colour module for the detected sensor.
The sensor model does not identify an IR filter, replacement lens or wide module.
Select those variants explicitly. `cc_vc4_select_profile()` checks the override
against the actual sensor and leaves its output unchanged on failure. Enum IDs
select static data; an override cannot name an arbitrary file. Profile info
exposes the canonical sensor, NoIR/wide/generic-lens flags and a calibration ID
containing the source hash. PiSP also flags profiles with factory CAC.

```c
enum cc_vc4_profile profile;
int ret = cc_vc4_select_profile(sensor, CC_VC4_PROFILE_IMX708_WIDE, &profile);
if (!ret) {
    cc_ipa_vc4_options options = {.mains_hz = 50, .profile = profile};
    ret = cc_ipa_vc4_create(&options, &ipa);
}
```

Attach the controller to a VC4 capture graph with live sensor timing, geometry,
delayed controls and ISP white-balance controls, as in `examples/still.c`.
`cc_ipa_vc4_create_proxy()` preserves the same selection in an isolated worker.
The options are copied. A sensor/profile mismatch fails during configuration.
Zero-initialized options select AUTO and disable flicker quantization; NULL
options select AUTO and the 50 Hz default. Manual sensor request overrides and
manual colour gains retain their documented precedence.

Advanced callers can use `cc_vc4_build_tuning()` to build a concrete profile's
native block without device I/O. Geometry must describe the actual active area,
crop and flips. Cropping/resampling and normalization adapt the shading grid;
black levels use the ISP's 16-bit scale even for 10/12-bit sensor input.
`cc_vc4_ov5647_build_tuning()` and `cc_vc4_ov5647_profile_id()` remain wrappers
for the original standard-colour profile.

## Measured lens shading

Both controllers accept a measured flat-field gain grid through
`options.lens_shading`. Set `enabled = 1` and fill all three R/G/B arrays
with the 16×12 cell-centre gains in row-major order across the full unflipped
sensor active area. For each colour channel, divide the brightest cell's
black-subtracted flat-field level by each cell's black-subtracted level.
Use a uniform, unsaturated exposure from the actual camera and lens setup.
The accepted gain range is 1 to 7.99 on both ISPs. The controller copies the
grid, including when created through an isolated worker.

The measured grid replaces factory shading and follows the live sensor crop
and flips. Each channel is normalized to a minimum gain of one after
resampling. The selected module still determines black level, colour, gamma,
denoise, sharpening and any factory CAC. The reported calibration ID identifies
that module profile. Record the measured grid's provenance alongside it.
Applications building native blocks directly can pass the same grid to
`cc_vc4_build_tuning_with_shading()` or `cc_pisp_build_tuning_with_shading()`.

## Still tool

```sh
./build/ccamera-still --list-profiles
./build/ccamera-still -c 0 --profile imx708_wide -o photo.ppm --metadata photo.jsonl
```

The tool selects the active area's full-resolution Bayer mode, using 12 bits
for IMX477 and 10 bits for the other supported sensors. It applies sensor-specific
startup/trust policies and default control delays, with the existing explicit
delay overrides. Timing diagnostics remain limited to OV5647. JSONL records
the selected calibration ID. Native gain codes differ by sensor: code 0 is unity
on the Sony sensors, while OV5647 uses code 16. `--gain-code 0` is an explicit
manual setting. Check live control limits before choosing a native code.

The [Pi 4/OV5647 guide](PI4_OV5647.md) remains the hardware acceptance procedure.
For V3, the calibrated controllers now offer opt-in module-specific AF through
the [common photo-control API](PHOTO_CONTROLS.md). For HQ, choose and
focus the physical lens and assess its shading before relying on the photograph.

## Calibration and algorithm limits

Factory lens shading uses the reference's chroma tables and luminance strength.
Scene-adaptive ALSC remains outside this implementation. Noise and sharpening
use the reference's estimated binning factor from live crop and output sizes,
with at most twofold binning per axis. Individual modes have no physically
validated noise calibration.
Bayesian AWB uses a bounded synchronous CT/transverse search with each module's
curve, bounds, sensitivities and lux priors. NoIR uses a middle-half grey-world
average and a nominal 4500 K because grey-world cannot estimate temperature.
Neither policy guarantees colour accuracy under arbitrary IR spectra.

AE uses the profile's centre-weighted regions, normal exposure/gain curve,
piecewise luminance target, base EV, filtering speed and upper-histogram lower
constraint. This is a single-exposure still controller, with simpler convergence
and filtering than upstream. Dynamic contrast, adaptive sharpening,
HDR, preview-specific policies and individual-unit/lens calibration remain open.
Explicit native controls and request tuning remain available.

**IPA ABI is 9 and the native wire version is 4.** Rebuild C IPA modules,
workers and application consumers together. The wire codec carries the options'
module selection and measured shading.

## Provenance and software checks

The unchanged `data/*-vc4.json` files come from `src/ipa/rpi/vc4/data/` in the
archived libcamera 0.7.2 reference described in [PORTING.md](../PORTING.md#reference-source-and-recovery).
Each JSON has a BSD-2-Clause license sidecar. Generated C tables and independent
arithmetic fixtures include the complete source SHA-256 hashes. The original
OV5647 calibration ID is preserved.

Regenerate after deliberately updating calibration data:

```sh
python3 tools/generate_vc4_profiles.py
python3 tools/generate_vc4_tuning_vectors.py
```

The C tests compare 50 JSON-derived arithmetic fixtures across all ten profiles:
temperature endpoints/interpolation, gain scaling, gamma, signed colour matrices,
full-field/cropped shading and flips. They also check module selection, failure
atomicity, native gain models, manual overrides, automatic exposure feedback,
NoIR white balance, restart, and controller configuration through a real isolated
worker. Optional Python checks verify that compiled data and fixtures match the
vendored JSON. Installed consumers exercise profile selection and native tuning.
These checks validate parameter generation and control feedback, not physical
pixels, focus, DMA/cache behavior or sensor-control latency.

## Pi 5 CFE/PiSP photo path

`cc_ipa_pisp_create()` connects the 12 profiles above to PiSP-specific calibration,
native CFE statistics and the back-end photo pipeline. The underlying PiSP
calibration, gamma and 32×32 shading tables are separate. Use
`cc_pisp_lookup_profile()` to resolve a name, `cc_pisp_select_profile()` to check
the sensor/override, and
`cc_pisp_get_profile_info()` to obtain the PiSP calibration ID. Automatic selection
still assumes the standard-colour module, never a NoIR/wide variant.

Select `imx477_6mm` or `imx477_16mm` explicitly for the matching reference HQ
lens. Both use the archived HQ shading, with distinct factory CAC displacement
tables. PiSP also applies factory CAC on colour standard and wide V3. The
native tables use signed shifts in 1/32-pixel units. The port maps the full
active-area tables to the sensor crop, reflects the displacement vectors for
sensor flips and scales them to the sensor-mode pixel size.

```c
cc_ipa_pisp_options options = {.mains_hz = 50, .profile = CC_VC4_PROFILE_IMX708_WIDE};
int ret = cc_ipa_pisp_create(&options, &ipa);
if (!ret)
    ret = cc_capture_set_ipa(capture, &ipa);
```

The capture graph must already own configured, allocated CFE and PiSP BE devices,
with delayed sensor controls enabled. Missing geometry, incompatible profiles,
raw-only graphs and CFE crop/downscale are rejected explicitly. The calibrated
path requires the primary FE image to match the sensor image dimensions; sensor
mode cropping itself is supported through live geometry. BE crops, resizing,
flips and two outputs remain available. Request processing copies native blocks
and retains an applied colour-temperature snapshot through capture, processing
and IPC. Manual exposure/gain overrides retain precedence. `cc_ipa_pisp_create_proxy()`
provides the same controller in the isolated worker.

`cc_cfe_get_sensor_geometry()` and the capture wrapper expose the live active
area, crop and flips through owned/adopted CFE graphs. Advanced callers can use
`cc_pisp_build_tuning()` without I/O and submit the generated processing blocks
through the existing native APIs. The builders add I/O layouts, output CSC,
dirty flags and tiles. Choose JPEG/sRGB transfer metadata for these photo curves;
both outputs share the common colour/gamma path. The generic reference controller
and NULL native tuning retain their existing behavior.

The native path applies black subtraction separately to FE statistics and BE
images, WBG/RGBY, DPC, gain-dependent GEQ, spatial/colour denoise, calibrated
colour matrices, packed gamma and shading, and reference sharpening with its
YCbCr/inverse stages. Static shading follows the actual sensor crop/flips,
normalizes each channel and packs the 33×33 native corner grid. AWB compensates
pre-LSC CFE zones using the applied back-end shading snapshot. Metering maps the
profile's 15×15 weights into the 16×16 CFE grid with a zero-weight final row and
column. RPFS feedback uses its full 1024-bin/1024-zone, 16-bit statistics.

Ordinary single-exposure denoise uses each profile's **no-TDN** deviation and strength.
[Automatic native photo transactions](NATIVE_PHOTO.md) add calibrated temporal
sequences and short/long HDR brackets, including internal history ownership,
tone mapping and spatial shadow gains. Adaptive ALSC and contrast remain outside
this implementation. V3
autofocus is available through the common controls with module-specific
calibration; physical AF validation remains unavailable. No Pi 5 hardware validation is claimed.

The existing raw capture utility exposes this path with `-q auto` or `-q NAME`:

```sh
./build/ccamera-cfe-capture -c 0 -w 4608 -h 2592 -f pRAA \
    -q imx708_wide -n 30 -o frames.rgb -s frames.rpfs
```

Select a sensor mode reported by discovery; the example mode/FOURCC assumes
IMX708. `-q` implies BE processing and selects JPEG/sRGB output metadata; it
excludes `-g`/`-j`. This utility writes all frames and does not wait for still-photo
convergence. Native delays/startup policies are defaults requiring validation.

The unchanged `data/*-pisp.json` files come from the same archived reference.
`data/pisp-default.json` comes from the BSD-licensed
[libpisp v1.5.0 defaults](https://github.com/raspberrypi/libpisp/blob/v1.5.0/src/libpisp/backend/backend_default_config.json).
Regenerate the PiSP tables, native defaults and arithmetic fixtures with:

```sh
python3 tools/generate_vc4_profiles.py --pisp
python3 tools/generate_pisp_defaults.py
python3 tools/generate_pisp_tuning_vectors.py
```

Sixty independent JSON arithmetic fixtures check black level, CCM, gamma,
GEQ/noise and shading across every profile, temperature extremes, gains and crops.
Independent CAC fixtures check all signed shifts, crops, scaling and flips.
C0/D0 native builders check both outputs and packed grids. Measured shading
checks cover both controllers, including invalid data, crop/flip resampling and
isolated-worker delivery. Simulated controllers
check AE step response, shading-compensated Bayesian/NoIR WB with a statistics ROI, manual settings,
startup mistrust and restart. Driver graphs cover uncompressed/PC1/PC2/PC3 inputs,
parameter delivery, held snapshots, timeouts, cancellation and stop/start, both
in-process and through real isolated workers. These checks establish software
parameter/feedback/ownership behavior, not physical image quality or cache/DMA
behavior on Pi 5.
