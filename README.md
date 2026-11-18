# libccamera

A C17 rewrite of [libcamera](https://github.com/raspberrypi/libcamera/)'s 
core for Linux, built with CMake, mostly written by AI. Static linking
is the default, and libc is the only required runtime dependency by default. The library
uses Linux media/V4L2 APIs without depending on the upstream C++ library,
libudev, Python, graphics libraries, codecs, or an event-loop library.

**This is an in-progress port with an experimental C API.** It is not yet a
complete libcamera replacement. Physical camera behavior, sensor timing and
image quality still need hardware validation. The upstream source is an
external porting reference, not a build or runtime dependency.

## Current support

| Pipeline | Implemented capture path | Validation |
| --- | --- | --- |
| Raspberry Pi 4B + standard OV5647 v1.3 | Unicam raw, VC4 RGB/YUV with two image outputs and statistics, calibrated still controls | Simulated driver and calibration checks. Physical acceptance pending using the [Pi 4B guide](docs/PI4_OV5647.md). |
| Other Pi 4 or earlier Unicam/VC4 setups | Raw and processed capture, with calibrated still controls on selected modules | Software checks. Physical acceptance pending. See [profile coverage](docs/TUNING.md). |
| Raspberry Pi 5 CFE/PiSP | Raw/statistics, up to two processed outputs, calibrated still controls on selected modules, native HDR/TDN on supported graphs | Software checks. No hardware validation. See [PiSP tuning](docs/TUNING.md#pi-5-cfepisp-photo-path) and [native photographs](docs/NATIVE_PHOTO.md). |
| USB UVC | Native video formats and normalized controls reported by the selected device | Simulated driver checks. No USB camera hardware validation. |
| Standalone V4L2 | Native formats on self-contained streaming capture nodes | Simulated queue and capture checks. Other device-specific behavior is unverified. |
| VIMC | Sensor B through the kernel media graph's debayer and scaler | Software pipeline tests. VIMC is a virtual device. |

Disconnect/reconnect and concurrent physical cameras remain unverified.
Use the [capture contract](docs/CAPTURE.md#restart-and-recovery) for buffer
ownership, errors and restarting. Fixed-focus OV5647 cannot validate the
V3 autofocus path.

Shared features include queued capture requests, MMAP/DMA-BUF buffers, native
controls, firmware/USB location IDs with ID-based opening, hotplug notifications,
role-based configuration, read-only generic and Pi sensor-mode selection, sensor timing helpers
and normalized camera properties (model, location, mounting rotation and sensor
geometry), sensor orientation negotiation/flip application, plus optional C IPA
modules for exposure, white balance and autofocus, including explicit
[shared C module loading](docs/IPA.md#loading-shared-c-modules) and
[directory discovery/selection](docs/IPA.md#discovering-and-selecting-modules). An opt-in calibrated
[VC4 still-profile family](docs/TUNING.md) covers OV5647, IMX219, IMX477 and IMX708,
including explicit NoIR/wide variants. A separate calibrated
[PiSP single-exposure path](docs/TUNING.md#pi-5-cfepisp-photo-path) covers the same
modules on Pi 5 in software. Both share [photographic controls and metadata](docs/PHOTO_CONTROLS.md)
with capability queries, manual/automatic exposure and gain, EV/metering, WB,
native image adjustments and calibrated opt-in V3 AF/PDAF. PiSP also supports
[automatic native temporal denoise and HDR still transactions](docs/NATIVE_PHOTO.md),
including delayed exposure brackets and internal history ownership. Physical photo acceptance and advanced algorithms
remain pending.

Native [IPC transport](docs/IPC.md) provides reference-compatible datagram and
descriptor framing, namespace-isolated workers and C IPA callback proxies for
current VC4/CFE/PiSP graphs and the existing VIMC mock IPA. Optional
[libcrypto-backed signing](docs/BUILDING.md#optional-ipa-signing) verifies native C
modules and selects in-process or isolated execution automatically.

## Build and test

Requires Linux, a C17 compiler, CMake 3.16 or newer, and Linux media/V4L2 UAPI
headers. Python 3 is optional for analysis and calibration-data checks. On Ubuntu/Debian, install
`build-essential cmake linux-libc-dev python3`.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

The static library is `build/libccamera.a`. Set `-DBUILD_SHARED_LIBS=ON` for a
shared library. Tests use simulated kernel interfaces and require no camera.
GitHub Actions checks GCC/Clang builds, installed consumers, sanitizers, fuzz
smoke tests and AArch64 compilation. See the [build guide](docs/BUILDING.md) for
build options, cross-compilation and detailed test coverage.

## Get started

Discover cameras and inspect the modes of a selected camera:

```sh
./build/ccamera-capture -l
./build/ccamera-capture -c 0 -m
```

Choose the index reported by discovery. To capture a PPM from a camera that
supports RGB24:

```sh
./build/ccamera-photo -c 0 photo.ppm
./build/ccamera-photo -c 0 -W 1280 -H 720 small.ppm
```

The [still-capture API](docs/CAPTURE.md#still-photographs) selects the Raspberry
Pi processor and calibrated tuning, negotiates the image layout and allocates
buffers. Native video cameras use their device controls and formats. For Pi 4B
and Camera Module v1.3 hardware acceptance, use the
[OV5647 bring-up and validation guide](docs/PI4_OV5647.md).

The listing also reports camera IDs when available. Use `-I 'ID'` instead of
`-c INDEX` to select by identity across device-node renumbering. See
[camera identities](docs/CAPTURE.md#camera-identities) for their stability rules.
Run `./build/ccamera-watch` to watch arrivals, changes and removals. The
[hotplug monitor](docs/CAPTURE.md#camera-hotplug) integrates with caller-owned
poll loops and preserves the library's libc-only dependency.
Native CFE/VC4/PiSP configurations also support
[read-only TRY validation](docs/CAPTURE.md#read-only-pipeline-validation) before
applying formats or allocating buffers. The [stream-role layer](docs/CAPTURE.md#stream-roles)
generates defaults, negotiates sensor/output formats and applies a complete plan
to supplied devices. Select a NoIR, wide or supported HQ lens variant with
`ccamera-photo -p NAME`. See the [module profile list](docs/TUNING.md).
`ccamera-still` provides the VC4 diagnostics described in the OV5647 guide.

The public API starts at [ccamera/ccamera.h](include/ccamera/ccamera.h), with
complete examples in [examples/](examples/). To use an installed library:

```cmake
find_package(ccamera 0.1 CONFIG REQUIRED)
target_link_libraries(your_library PRIVATE ccamera::ccamera)
```

See [CMake integration](docs/BUILDING.md#link-from-another-library-or-application)
for installation, `CMAKE_PREFIX_PATH`, embedding the source and pkg-config.

## Documentation

| Guide | Contents |
| --- | --- |
| [Building and integration](docs/BUILDING.md) | Build options, cross-compilation, tests and linking |
| [Capture and requests](docs/CAPTURE.md) | Discovery, controls, frame intervals, buffer ownership and request lifecycle |
| [Sensor controls and metadata](docs/SENSORS.md) | Sensor profiles, gain/timing, delayed controls and embedded data |
| [Algorithms and autofocus](docs/IPA.md) | C IPA modules, reference AE/AWB and lens control |
| [Pi photo tuning](docs/TUNING.md) | VC4/PiSP module selection, calibration coverage and limitations |
| [Pipeline processing](docs/PIPELINES.md) | Low-level queues, VC4 ISP, Pi 5 CFE and PiSP back end |
| [Pi 4B / OV5647 stills](docs/PI4_OV5647.md) | Calibrated still capture, diagnostics and hardware validation |
| [Fuzz testing](tests/fuzz/README.md) | Fuzzer targets, longer campaigns and reproducer handling |

See [CONTRIBUTING.md](CONTRIBUTING.md) for development checks and contribution
guidelines.

## License

Library code is LGPL-2.1-or-later. See [LICENSE](LICENSE) and the
[license overview](LICENSES/README.md) for the additional BSD notices and Linux
UAPI syscall exception. Individual file notices are authoritative.
