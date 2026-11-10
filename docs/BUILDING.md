# Building, testing and integration

[Project overview](../README.md). Run example commands from the repository root.

- [Build and test](#build-and-test)
- [Link from another library or application](#link-from-another-library-or-application)

## Build and test

Building requires Linux, a C17 compiler, CMake 3.16 or newer, and installed Linux
media/V4L2 UAPI headers. Python 3 is optional for one analysis test. On
Ubuntu/Debian, install `build-essential cmake linux-libc-dev python3`.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

`build/libccamera.a` is the static library. Set `-DBUILD_SHARED_LIBS=ON` for a
shared library, or disable optional tools with `-DCCAMERA_BUILD_EXAMPLES=OFF`
and `-DCCAMERA_BUILD_TESTS=OFF`. Cross-compilation uses a normal CMake toolchain
file; no target binaries execute during the library build.

GitHub Actions runs GCC and Clang builds with static Debug and shared Release
libraries, the hardware-independent tests, installed CMake/pkg-config consumers,
and Clang ASan/UBSan plus bounded fuzz smoke tests. A separate AArch64 job
cross-compiles the library, examples and tests; it does not run target binaries.
See [contributing](../CONTRIBUTING.md) for commands to reproduce these checks.
Physical camera and image-quality validation remains separate.

Optional [fuzz testing](../tests/fuzz/README.md) uses Clang/libFuzzer with ASan and
UBSan to exercise PiSP/CFE configuration, sensor arithmetic, delayed controls
and malformed V4L2 metadata without hardware. Enable it in a fresh Clang build
with `-DCCAMERA_BUILD_FUZZERS=ON`; run the bounded smoke suite with
`ctest --test-dir build-fuzz -L fuzz --output-on-failure`.

An optional native fence test creates a temporary, initially signaled DRM
syncobj and exports a real Linux sync file. It submits no GPU work or display
changes and needs no libdrm library or headers. It checks validation, CLOEXEC,
ownership after closing the producer, polling and one-shot consumption:

```sh
cmake -S . -B build-native-fences -DCCAMERA_BUILD_TESTS=OFF \
    -DCCAMERA_BUILD_EXAMPLES=OFF -DCCAMERA_BUILD_DRM_FENCE_TEST=ON
cmake --build build-native-fences
ctest --test-dir build-native-fences -R native-sync-file --output-on-failure
```

`CCAMERA_DRM_RENDER_NODE` selects the device (default `/dev/dri/renderD128`).
The optional test skips when the node or syncobj feature is unavailable.
Ordinary tests and builds require no DRM device. Pending/error fences and
capture scheduling are covered by the simulated-driver tests.

For a 64-bit Raspberry Pi userspace, the Arm GNU Linux toolchain can use the
included file (put `aarch64-none-linux-gnu-gcc` on `PATH` first):

```sh
cmake -S . -B build-pi -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu.cmake \
    -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS=-Werror
cmake --build build-pi
```

This builds `libccamera.a`, both capture examples and the hardware-independent
test executables for AArch64. Run these executables on the Pi, or configure
`CMAKE_CROSSCOMPILING_EMULATOR` to run CTest through an emulator/remote launcher.
The installed-consumer test also forwards the toolchain/sysroot and uses that
launcher; without one it builds the consumer and reports that target execution
is still required. The library is statically linked into the default tools;
the tools themselves use the target's libc and Linux loader.

Tests exercise the actual library source against an injected kernel interface.
They cover both CSI pipelines, mux selection and restoration, old and new RP1
routing APIs, format negotiation, single- and multi-plane capture, DMA-BUF import/export,
scalar/64-bit/payload controls, interrupted and bounded waits, invalid metadata,
and cleanup after failures. Stream tests cover all six image/metadata queue
directions with both MMAP and DMA-BUF memory, shared M2M handles, output payloads,
atomic import replacement, and buffer lifetime across a simulated two-device
processing pipeline. The VC4 test uses the actual backend against a simulated
four-port driver, including crop and controls, different buffer counts per port,
partial completions, timeouts, fd and kernel attachment lifetime, and failure
recovery at every queue/start/dequeue step. CFE tests cover sensor/CSI/front-end
routing, C parameter generation, frame/statistics pairing, held slots, partial
completions, timeouts and failure cancellation on all three ports. A separate
PiSP back-end test checks native parameter delivery, one/two/three-plane image
layouts, both output streams, widened chroma-stride allocation repair, source
fd/attachment lifetime, partial completions and per-port failure cancellation.
Its C tiler is checked for complete, disjoint image
coverage, filter context, DMA alignment and capacity bounds across more than
10,000 frame sizes, plus YUV chroma-plane bounds, byte order, independent
601/709 conversion and full/limited clipping. Crop/resize/flip tests use 32
upstream-generated native tile fixtures and 2,000 deterministic geometry cases,
checking output coverage, scaler phases and DMA bounds. Driver tests check
resized jobs and colour metadata when V4L2 nodes force defaults. Processing tests
check native denoise, debin, tone mapping, saturation, false-colour and sharpen
blocks, CAC grid coverage and tile phases, invalid register values/kernel sums,
padding, matrix-pair dependencies and the differing C0/D0 matrix-offset ranges.
Simulated jobs verify revision discovery and reject bad tuning before importing
source buffers. Request tests cover ownership transfer, FIFO queues, held-slot
backpressure, tuning/control snapshots, overlapping capture and ISP processing,
partial completion polling, deadlines across both stages, malformed completions,
and cancellation/forced close before source memory is freed. Delayed-control
tests compare 2,000 frames against an independent model and check priority and
adjusted scalar/64-bit writes, sequence wrap/history expiry, SOF polling, held
metadata, missed events/write deadlines, partial failure and restart on both
Unicam and CFE sources. Sensor-helper tests check native gain-code round trips,
quantization against driver min/max/step, independent exponential equations,
200 arbitrary-precision timing vectors and live model/default-delay selection.
Sensor-mode selection checks generic ranking against 16 upstream-generated
fixtures, large integer dimensions, arbitrary image-stream IDs, read-only
queries and bounded/malformed driver enumeration.
An install test links and runs an external C17 consumer using
`find_package`, including the public ISP/CFE and native UAPI headers. No physical capture device
is needed for these tests. Real Pi captures remain unverified in this workspace.

## Link from another library or application

With both projects in the same build:

```cmake
set(CCAMERA_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CCAMERA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/libccamera)
target_link_libraries(your_library PRIVATE ccamera::ccamera)
```

Or install with `cmake --install build --prefix /your/prefix` and consume it:

```cmake
find_package(ccamera 0.1 CONFIG REQUIRED)
target_link_libraries(your_library PRIVATE ccamera::ccamera)
```

Pass `-DCMAKE_PREFIX_PATH=/your/prefix` when configuring the consumer. A
`ccamera.pc` file is also installed; pkg-config is not required to build libccamera.
