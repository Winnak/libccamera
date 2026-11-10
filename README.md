# libccamera

A C17 rewrite of [libcamera](https://github.com/raspberrypi/libcamera/)'s 
core for Linux, built with CMake, mostly written by AI. Static linking
is the default, and libc is the only required runtime dependency. The library
uses Linux media/V4L2 APIs without depending on the upstream C++ library,
libudev, Python, graphics libraries, codecs, or an event-loop library.

**This is an in-progress port with an experimental C API.** It is not yet a
complete libcamera replacement. Physical camera behavior, sensor timing and
image quality still need hardware validation. The upstream source is an
external porting reference, not a build or runtime dependency.

## Current support

- **Pi 4 and earlier:** Unicam raw capture and bcm2835 VC4 ISP processing, with
  two image outputs and statistics.
- **Pi 5:** RP1 CFE raw/statistics capture and PiSP back-end processing, including
  crop, resize, flip and Bayer/mono conversion to RGB/YUV/luma.
- **Other cameras:** self-contained V4L2 capture nodes, USB UVC cameras and the
  VIMC Sensor B/debayer/scaler pipeline.

Shared features include queued capture requests, MMAP/DMA-BUF buffers, native
controls, read-only generic and Pi sensor-mode selection, sensor timing helpers
and optional C IPA modules for exposure, white balance and autofocus. An opt-in calibrated
OV5647/VC4 still profile is available; broader tuning and advanced algorithms
remain pending.

## Build and test

Requires Linux, a C17 compiler, CMake 3.16 or newer, and Linux media/V4L2 UAPI
headers. Python 3 is optional for one analysis test. On Ubuntu/Debian, install
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

Choose the index reported by discovery. The [capture guide](docs/CAPTURE.md)
covers capture, buffer ownership and queued requests. For still photographs
with a Pi 4B and Camera Module v1.3, follow the
[OV5647 bring-up and validation guide](docs/PI4_OV5647.md).

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
| [Pipeline processing](docs/PIPELINES.md) | Low-level queues, VC4 ISP, Pi 5 CFE and PiSP back end |
| [Pi 4B / OV5647 stills](docs/PI4_OV5647.md) | Calibrated still capture, diagnostics and hardware validation |
| [Fuzz testing](tests/fuzz/README.md) | Fuzzer targets, longer campaigns and reproducer handling |

See [CONTRIBUTING.md](CONTRIBUTING.md) for development checks and contribution
guidelines.

## License

Library code is LGPL-2.1-or-later. See [LICENSE](LICENSE) and the
[license overview](LICENSES/README.md) for the additional BSD notices and Linux
UAPI syscall exception. Individual file notices are authoritative.
