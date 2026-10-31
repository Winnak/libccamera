# Contributing to libccamera

The C API is experimental. Open an issue to discuss larger changes to the API
or pipeline architecture before starting work. Please note that this project
has mostly been written by AI. For a more human written project check out
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
