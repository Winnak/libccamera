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
Camera-property tests check sensor/UVC/VIMC defaults, I2C model extraction,
ASCII conversion, physical and inferred geometry, all 25 native Bayer/mono
codes, flip-invariant native CFA selection, arbitrary sensor image streams,
enumeration limits and atomic failures. CFE, camera and adopted capture-graph
queries are exercised alongside installed C17 consumers.
Orientation tests compare the complete transform algebra and 144 sensor-policy
fixtures with compiled, unmodified archived C++ methods. Native raw and CFE
tests cover all supported Bayer/mono packing families, dual cropped/scaled FE
outputs, compression modes 1/2/3, live graph queries, allocation guards, partial
flip writes, clamped acknowledgements and configuration failure recovery.
Identity tests use 18 fixtures generated by compiling unmodified archived
firmware, sensor-model and sensor/UVC ID methods with simulated sysfs. They
cover DT/ACPI/platform IDs, I2C/USB/device-node renumbering, generic video indices,
stale snapshots and replacements during open, duplicate IDs, malformed/overlong
metadata, native bounded sysfs reads and cleanup of failed discovery/open calls.
Raw, UVC, VIMC, managed CFE and adopted graphs expose their source identity.
Hotplug tests cover kernel/inotify packet validation, media child dependencies,
access failures, identity fallback, reconnects, renumbering, publication rollback,
queue loss/overflow and wait deadlines. Native Linux tests exercise subscriptions,
poll readiness, timer delivery, descriptor zero and cleanup after partial setup
fails. They need access to kernel uevent sockets and `/dev` inotify, but do not
modify device nodes or camera configuration. `ccamera-watch` demonstrates polling
and signal-driven shutdown. A separate fuzz target mutates notification packets.
Native configuration validation tests exercise CFE sensor/CSI/FE TRY routing,
dual crop/scale outputs and VC4/PiSP image, metadata and history layouts. They
reject active-write ioctls during validation, check temporary-port cleanup and
unchanged allocated/streaming state, inject failures at every negotiated port,
and verify that capture/processing continues with the original configuration.
Stream-role tests add 288 fixtures from compiled archived Size/StreamFormats
methods and the UVC validator's pre-I/O policy, plus 84 generated reference
V4L2/DRM sort keys. They cover native/UVC/VIMC defaults, source/processor
assignment on Unicam/VC4 and CFE/PiSP, Bayer/mono orientation prediction,
physical versus advertised sensor sizes, C0/D0 format adjustment, colour sharing,
preflight allocation guards, partial-apply invalidation/recovery and request
capture after graph adoption. The new configuration fuzz target checks bounded
enumeration, malformed driver replies and atomic publication without device I/O.
The checked-in role fixtures and format traits can be regenerated from the
SHA-verified libcamera reference archive:

```sh
python3 tools/generate_configuration_vectors.py /path/to/libcamera-reference.tar.xz
python3 tools/generate_configuration_formats.py /path/to/libcamera-reference.tar.xz
```

Regeneration requires Python 3 and C/C++17 compilers; normal builds use the
generated files and require neither the archive nor a C++ compiler.

Shared-IPA tests build real C modules and inspect them without running their
constructors. They cover ABI/name rejection, malformed and unaligned ELF tables,
signatures, missing/unresolved factories, `$ORIGIN` dependencies, changed files,
descriptor zero, factory cleanup and unloading after the last callback. Loaded
instances also run through VC4/CFE/PiSP graph adoption, completion, restart,
failed configuration and cancellation. A pure ELF fuzz target exercises raw and
structured malformed inputs without loading code. The system dynamic loader
is part of libc on current glibc/musl systems; CMake links system libdl when older
platforms require it and propagates it to static CMake/pkg-config consumers.
IPA discovery adds 60 fixtures from compiled, unmodified archived
`parseDir`/`addDir`/`module`/`match` methods, with text-only metadata stubs. Native
tests reproduce their filesystem trees using real C modules and cover priority,
recursion, aliases, version/name selection, skipped invalid modules, no fallback
after selected-module failure, environment/privilege rules, snapshot behavior,
descriptor zero, limits and complete descriptor cleanup. Selected modules also
run through VC4/CFE/PiSP graph adoption and teardown. A separate pure path-policy
fuzzer performs no filesystem access. The reference fixtures can be regenerated:

```sh
python3 tools/generate_ipa_manager_vectors.py /path/to/libcamera-reference.tar.xz
```

Runtime-location tests discover a real module two levels under the active build
image's `ipa/` directory, load it and retain its callbacks after manager closure.
Separate probes copy the executable and shared core (when enabled) to a temporary
tree, move that tree, launch through a symlink and change working directory after
a relative launch. They check marker validation, depth limits, environment
precedence and descriptor zero. Installed consumers verify that installed copies
do not search the development tree. Shared build dependencies use `$ORIGIN`;
the marker is generated beside the core build image and is not installed.
The probes also cover static builds without RPATH and a shared-library
multi-configuration CMake build.

IPC tests compare 155 fixtures from compiled archived send/message methods with
the native datagram framing. Real sockets check descriptor passing/CLOEXEC,
fd-only payloads, bidirectional and cross-process messages, ownership transfer,
descriptor zero, kernel backpressure and injected partial/fatal sends. Malformed
packets, ancillary truncation and unexpected pidfds are rejected with descriptor
and queued-kernel-reference cleanup. Pure header/ancillary fuzzing does no OS I/O.
Process tests cover exec failures and deadlines, exact fd inheritance, pidfd
signals/waits/reaping, parent death, spawning during background allocation, and
both close-range and proc-fd cleanup. Namespace tests verify different user/net
namespace inodes using the `/proc/self/ns/*` symlink targets, so AppArmor need
not allow following those links into nsfs. Unsupported kernel/policy
configurations report a skip locally; CI rejects skipped tests to ensure that
namespace isolation and the rest of the configured suite are actually exercised.
Worker/proxy tests exercise module and built-in factories, namespace separation,
deep callback data, descriptor-bearing controls, failures/timeouts, parent exit,
independent instances and reaping. Existing VC4/CFE/PiSP/VIMC graph suites also
run through actual workers. Runtime and installed-consumer probes exercise worker
lookup/packaging; see [IPC status](IPC.md).
The worker is built and installed even with examples/tests disabled. Its install
location follows `CMAKE_INSTALL_LIBEXECDIR`; the core and self-contained worker
need no C++ runtime. The process stress test uses the platform thread library;
the core and worker create no background thread.

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


## Optional IPA signing

`CCAMERA_WITH_IPA_CRYPTO` defaults to `OFF`. Enabled builds require CMake 3.18+,
OpenSSL 3+ development files for the target's `Crypto` component, and a **host**
`openssl` executable. They link `OpenSSL::Crypto`, never libssl. Public C headers
contain no OpenSSL types or includes. On Debian/Ubuntu add `libssl-dev openssl`.
For cross builds, configure target headers/libraries through the toolchain and
`OPENSSL_ROOT_DIR` as usual. `CCAMERA_HOST_OPENSSL` can select an absolute host
executable; its discovery bypasses CMake's target root. Signing never executes a
target verifier or target OpenSSL executable. When using cross pkg-config,
set `PKG_CONFIG_LIBDIR`/`PKG_CONFIG_SYSROOT_DIR` for the target metadata as usual;
this prevents host compression dependencies from entering target static links.
Compatibility helpers support OpenSSL 3 version checks and static dependencies
with CMake 3.18 as well as current CMake releases.

```sh
cmake -S . -B build-crypto -DCCAMERA_WITH_IPA_CRYPTO=ON \
    -DCCAMERA_IPA_PRIVATE_KEY=/secure/persistent-ipa-key.pem
cmake --build build-crypto
ctest --test-dir build-crypto --output-on-failure
```

`CCAMERA_IPA_PRIVATE_KEY` and `CCAMERA_IPA_PUBLIC_KEY` are mutually exclusive PEM
file options, rejected when crypto is disabled. Keys must be plain RSA keys;
EC, RSA-PSS and password-encrypted keys are rejected without prompting. With a
private key the build embeds only its public DER SubjectPublicKeyInfo. With
neither option the build generates an RSA-2048 development key in
`build-crypto/ipa-keys/development.pem`, mode 0600 in an owner-only directory.
It persists across builds and reconfiguration. Replacing the key updates the
embedded root on the next build. No private key is installed or exported.

Use a persistent, securely managed private key for distributed releases: fresh
build directories otherwise have distinct development roots. A verification-only
packaging build can use a public key without generating a production private key:

```sh
openssl pkey -in /secure/persistent-ipa-key.pem -pubout -out ipa-public.pem
cmake -S . -B build-public -DCCAMERA_WITH_IPA_CRYPTO=ON \
    -DCCAMERA_IPA_PUBLIC_KEY="$PWD/ipa-public.pem"
```

Crypto tests use an independent development trust root, so public-key-only builds
do not require the production private key. Runtime trust is exclusively the
embedded public key; supplying a different key beside a module has no effect.

For a module target in this library build, use the configured signing key:

```cmake
add_library(my_ipa MODULE my_ipa.c)
target_link_libraries(my_ipa PRIVATE ccamera::ccamera)
ccamera_sign_ipa(TARGET my_ipa INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/ccamera/ipa"
                COMPONENT ipa)
```

Installed consumers obtain the same helper from `find_package(ccamera CONFIG
REQUIRED)` and **must** provide `PRIVATE_KEY /secure/key.pem`. Public-key-only
builds also require an explicit private key when using the helper. The helper
supports `TARGET`, `PRIVATE_KEY`, `INSTALL_DESTINATION` and `COMPONENT` arguments.
It creates an ALL target named `<target>-ipa-sign`; building that target signs the
module, and a normal build includes it. It regenerates the binary `.so.sign`
sidecar after a module/key change even if the module did not relink. The helper
writes signatures atomically and fails on signing errors.

With `INSTALL_DESTINATION`, the helper installs the module and signs its final
installed bytes **after** stripping and RPATH changes. Do not also install that
module elsewhere through a later modifying rule. The helper supports `DESTDIR`,
`cmake --install --prefix`, components, absolute destinations and multiple
configurations. Private paths occur only in local build/install instructions,
never in the installed package exports. Any later packaging edits to module bytes
(including strip, debug splitting or RPATH rewriting) require re-signing:

```sh
openssl dgst -sha256 -sign /secure/persistent-ipa-key.pem -keyform PEM -passin pass: \
    -sigopt rsa_padding_mode:pkcs1 -out module.so.sign module.so
ccamera-ipa-verify module.so
```

`ccamera-ipa-verify` is installed only with crypto support and never loads code.
It exits 0 for valid, 1 for missing/invalid signatures, and 2 for unavailable
verification, usage errors or operational failures. Signing follows the
[OpenSSL dgst workflow](https://docs.openssl.org/4.0/man1/openssl-dgst/); runtime
verification uses the high-level
[EVP API](https://docs.openssl.org/4.0/man3/EVP_DigestVerifyInit/).

Static CMake exports find `OpenSSL::Crypto` and carry its link dependencies.
`ccamera.pc` declares `Requires.private: libcrypto >= 3`, so use
`pkg-config --static --libs ccamera` for static linking, including libcrypto's
transitive private dependencies. Shared consumers need no OpenSSL public headers.
