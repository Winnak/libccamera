# Contributing to libccamera

Open an issue to discuss larger changes to the API or pipeline architecture
before starting work. Please note that this project has mostly been written by
AI. For a more human written project check out
[libcamera](https://github.com/raspberrypi/libcamera/).

## Build and validate

Development requires Linux, a C17 compiler, CMake 3.16 or newer, and Linux
media/V4L2 UAPI headers. Ninja is optional. Python 3 enables the sensor timing
analysis test; it is not a library dependency. On Ubuntu/Debian:

```sh
sudo apt-get install build-essential cmake ninja-build linux-libc-dev python3 pkg-config
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS=-Werror
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

Tests use simulated kernel interfaces and need no camera or root privileges.
The installed-consumer test also installs into the build directory, builds a
separate CMake consumer, and runs it. To check a shared Release build:

```sh
cmake -S . -B build-shared -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON -DCMAKE_C_FLAGS=-Werror
cmake --build build-shared --parallel 2
ctest --test-dir build-shared --output-on-failure
```

For the sanitizer and fuzz configuration used by CI, install Clang and its
matching compiler-rt runtimes (`clang libclang-rt-dev` on Ubuntu), then run:

```sh
cmake -S . -B build-sanitizers -G Ninja \
    -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_FLAGS="-Werror -fsanitize=address,undefined -fno-omit-frame-pointer" \
    -DCCAMERA_BUILD_FUZZERS=ON
cmake --build build-sanitizers --parallel 2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
    ctest --test-dir build-sanitizers --output-on-failure
```

See [fuzz testing](tests/fuzz/README.md) for longer campaigns and reproducer
handling. Keep captures and generated fuzz artifacts in an ignored build directory.

## Hardware testing

Test reports for any board or camera are welcome. Build as described above, close
other camera applications and run the examples from the repository root. Record
the commit (`git rev-parse HEAD`), board revision, camera or sensor model and
lens variant, OS version, kernel release (`uname -sr`), architecture (`uname -m`),
compiler and any custom CMake options.

List cameras, then inspect modes for your camera's discovery index. Replace `0`
with the index reported for your camera.

```sh
mkdir -p build/hardware
./build/ccamera-capture -l
camera_index=0
./build/ccamera-capture -c "$camera_index" -m
```

Substitute a width, height and FOURCC advertised by the driver. The `640x480`
`YUYV` example applies to cameras offering that mode.

```sh
./build/ccamera-capture -c "$camera_index" -w 640 -h 480 -f YUYV -n 10 \
    -o build/hardware/frames.raw 2>build/hardware/capture.log
```

Check the negotiated size, format, frame count and timestamps in the log.
Repeat the command with separate output and log paths to check that capture
works after reopening, then try another advertised size if one is available.
The output file contains concatenated native frame payloads. Follow the guide
for the pipeline reported by discovery for more coverage.

For processed stills on a tuned Pi module or a native camera offering RGB24,
capture a neutral test scene and inspect the saved PPM.

```sh
./build/ccamera-photo -c "$camera_index" build/hardware/photo.ppm \
    2>build/hardware/photo.log
```

Select the matching `-p` profile for a NoIR, wide or other supported module or
lens variant. See the [profile list](docs/TUNING.md#coverage-and-module-selection).

- **Pi 4 and earlier (`rpi-unicam`).** Use an advertised Bayer format for raw
  capture. See the [OV5647 guide](docs/PI4_OV5647.md) for manual and automatic
  exposure, image comparisons, timing and repeatability checks on that module.
- **Pi 5 (`rpi-cfe`).** Use the same discovery and raw capture steps. Try
  `ccamera-cfe-capture` with the reported mode to test FE images and statistics,
  or `ccamera-photo` to test calibrated stills on a supported module. See the
  [Pi 5 pipeline examples](docs/PIPELINES.md#pi-5-front-end-capture).
- **USB UVC and standalone V4L2.** Use the native format and size reported by
  the device. `ccamera-photo` requires RGB24 output for PPM. Record any controls
  you exercised and their observed effect.

### Report hardware results

In the project repository on GitHub, open **Issues → New issue**. Use a title
such as `Hardware test: <board> / <camera>`. Reports of working setups,
partial results and discovery failures all help establish coverage. Include
the following information.

- Board revision, camera or sensor, module or lens variant, OS, kernel and
  architecture.
- Commit, compiler, build options and discovery output with the pipeline name.
- Exact commands, requested and negotiated resolution and FOURCC, and whether
  discovery, capture and reopen worked.
- Expected and observed behaviour for each failure, the relevant error or log
  excerpt, and whether it reproduces. Include the reference application and its
  result if you tested one on the same hardware.

Before sharing logs or captures, redact hostnames, usernames, home directory
paths, addresses, device serial numbers and unique camera IDs. Use a neutral
test scene for shared images and check their metadata for personal details.

## C API compatibility

Public headers and type layouts may change during the 0.x series. Rebuild
applications against matching headers when updating the library. The installed
consumer test checks the current build's headers and library together.

For a stable C API, designate the supported headers, types and behaviours.
Document a versioning and deprecation policy, then build and run existing
consumer source against the new release. A shared-library ABI promise also
requires checking exported symbols and public type layouts and running
previously built consumers against the new library. Give incompatible ABI
changes a new shared-library version.
The [IPA module ABI](docs/IPA.md#loading-shared-c-modules) has its own version.

## Changes and reports

Keep changes focused and follow the surrounding C style. Preserve the libc-only
runtime dependency and the distinction between public API and private test
interfaces. Add a regression test for a behavioral fix when practical, and
update the public documentation when behavior or API changes.

In a pull request, describe the problem, resulting behavior, and validation.
For hardware-related changes, include the board, sensor/camera, OS/kernel,
compiler, command line and relevant diagnostics. State whether validation used
simulated drivers, an emulator, or physical hardware. The
[Pi 4 / OV5647 guide](docs/PI4_OV5647.md) describes capture diagnostics.

Report bugs through GitHub Issues with a minimal reproducer and the expected
and observed behavior. Remove private images or other personal data before
attaching captures and logs.

## Licensing

Contributions use the license of the files they change. New library code,
examples and tests use `LGPL-2.1-or-later`; preserve existing copyright,
attribution and SPDX notices, including additional BSD terms on derived code.
Mark build helpers intended for CC0 with `SPDX-License-Identifier: CC0-1.0`.
See the [license overview](LICENSES/README.md) and [project license](LICENSE).
