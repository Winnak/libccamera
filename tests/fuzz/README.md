# Fuzz testing

These Linux targets use Clang's libFuzzer, AddressSanitizer and
UndefinedBehaviorSanitizer. They require no camera, root access or upstream
libcamera source or build. Install Clang and its matching compiler-rt runtimes
first. The library sources are compiled into a separate instrumented static
archive; the normal library and installed CMake/pkg-config exports are unaffected.

```sh
cmake -S . -B build-fuzz -G Ninja \
    -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCCAMERA_BUILD_FUZZERS=ON \
    -DCCAMERA_BUILD_EXAMPLES=OFF -DCCAMERA_BUILD_TESTS=OFF
cmake --build build-fuzz
ctest --test-dir build-fuzz -L fuzz --output-on-failure
```

Fuzz smoke tests are registered even with ordinary tests disabled. Each runs
1,000 inputs with a fixed random seed, a 5-second per-input timeout, a 512 MiB
RSS limit, and a 60-second CTest timeout. With `CCAMERA_BUILD_TESTS=ON`, a normal
CTest invocation runs both suites. Use the same commands in CI.

The native IPA wire target allows 65536-byte inputs so complete register blocks
are mutated; the other smoke targets use 4096 bytes. Regenerate its checked-in
seeds with a normal test build and
`build/ccamera-ipa-wire-test --corpus tests/fuzz/corpus/ipa_wire`. The fuzzer uses
only the ordinary pure codec; native descriptor ownership is tested separately
with real handles in `ipa-wire`, `ipa-message` and `ipa-proxy`.

| Executable (`build-fuzz/fuzz/`) | Critical paths and checks |
| --- | --- |
| `ccamera-fuzz-pisp` | Bayer16 and compressed mode 1/2 input, RGB/YUV output layouts; two independent crops, resizers and flips; native tuning and register limits; complete, disjoint tile coverage, compressed block alignment and DMA capacity bounds; zero DMA addresses; unchanged output on rejection |
| `ccamera-fuzz-cfe` | Sensor-to-image formats including compressed mode 1/2, strides, black-level offsets, statistics grids, decimation and native tuning; generated I/O geometry, zero DMA addresses and unchanged output on rejection |
| `ccamera-fuzz-sensor` | Model lookup, gain tables/inversion and quantization, arbitrary binary64 gains including NaNs, exposure/frame timing and blanking planning checked against independent 128-bit arithmetic, native grids and long-exposure limits |
| `ccamera-fuzz-delayed` | Model-based explicit and automatically assigned frame scheduling with 1–32 controls, different delays, priority writes, adjusted acknowledgements, rejected commits, cookies, history expiry, reset and sequence wrap |
| `ccamera-fuzz-stream` | Six V4L2 image/metadata directions, 1–8 planes, malformed format/completion replies through the private ioctl seam, output payload validation, queue poisoning and stop/restart recovery; exact rational interval ranges, G/S_PARM replies and atomic rejection |
| `ccamera-fuzz-controls` | Native control types, payload capacities, driver errors, layout changes and malformed query replies; normalized VIMC controls with arbitrary binary64 values, saturation, finite checks, capacity and atomic rejection |
| `ccamera-fuzz-media` | Legacy and full media-v2 snapshots, interfaces and ancillary links, names/global IDs/pad indices, version/count races, stale guards and malformed acknowledgements; normalized routes checked against independent raw-graph reachability |
| `ccamera-fuzz-ipa` | Native BSTA/RPFS payloads, atomic rejection, reference controller binary64 inputs, exposure/gain bounds, optional frame-duration planning and independent gain equations |
| `ccamera-fuzz-ipa_module` | Pure ELF module inspection: raw and unaligned structured files, section/symbol/string extents, native headers, metadata ABI/names, truncation and unchanged outputs on error; never loads or executes a module |
| `ccamera-fuzz-ipa_manager` | Pure search-path policy: explicit/environment/build/installed priority, empty components, privileged environment suppression, build recursion, depth/count/flag limits, owned path cleanup and atomic errors; no filesystem access |
| `ccamera-fuzz-ipc` | Raw and unaligned native framing/ancillary headers, descriptor counts, overflow/truncation/record alignment, partial cleanup lists and atomic header rejection; descriptor values are opaque, with no sockets or handle operations |
| `ccamera-fuzz-ipa_wire` | All native C IPA object schemas, lengths/counts, sensor identities and optional fields, deep pointer reconstruction and canonical decode/encode stability; no OS or module I/O |
| `ccamera-fuzz-af` | Bounded hybrid AF scans, phase/contrast statistics, binary64 tuning/maps, commands, extreme windows, weighted-sum overflow and native payloads; native lens bounds and status validity |
| `ccamera-fuzz-embedded` | Packed SMIA tags/registers and line boundaries, Sony sensor reports, signed IMX708 phase/confidence and linear/logarithmic HDR histograms; malformed/truncated payloads, atomic rejection and independent 64-bit mean checks |
| `ccamera-fuzz-routing` | Live sensor route/format discovery, arbitrary source IDs, legacy/current ioctl layouts, bounded arrays, malformed pad/stream replies, reordered route acknowledgements and atomic query rejection |
| `ccamera-fuzz-buffers` | Request-owned plane descriptors, capacities and object identities; duplicate/identity failures, atomic replacement, overlap checked against an independent object model and exact handle cleanup |
| `ccamera-fuzz-fences` | Bounded sync-file header queries, arbitrary status/error replies, fd zero, duplicate failures, atomic replacement, one-shot consumption, take/clear and exact descriptor ownership |
| `ccamera-fuzz-uvc` | Live native range shapes, arbitrary binary64 normalized values, asymmetric/default mappings, exposure modes and native grids; bounded string/integer menu queries, extreme indices, malformed acknowledgements and atomic rejection |
| `ccamera-fuzz-monitor` | Raw kernel uevent and unaligned inotify packets, embedded NUL fields, inconsistent headers, device-node names, record lengths, loss markers, iteration progress and atomic rejection; no sockets or device access |
| `ccamera-fuzz-configuration` | Role generation/validation over a native-video model, arbitrary dimensions/formats/roles, bounded and malformed enumeration, output adjustment and atomic errors; active-write ioctls are forbidden |

Stream tests construct valid in-memory stream state and inject replies; they do
not fuzz the kernel or discover devices. The delayed-control target compares
valid schedules against an independent frame model and probes invalid
acknowledgements. It does not fuzz arbitrary pointer values. Geometry targets
mix structured inputs that reach tiling/finalization with bounded mutations of
integer-only native/configuration records. Checks remain enabled in Release
builds. Inputs are capped at 4 KiB during smoke runs; no target allocates image
buffers or performs blocking waits.

For a longer campaign, select any target name from `pisp`, `cfe`, `sensor`,
`delayed`, `stream`, `controls`, `media`, `ipa`, `ipa_module`, `ipa_manager`, `ipc`, `ipa_wire`, `af`, `embedded`, `routing`, `buffers`, `fences`, `uvc`, `monitor`, `configuration`:

```sh
name=pisp
mkdir -p "build-fuzz/corpus/$name" "build-fuzz/artifacts/$name"
"build-fuzz/fuzz/ccamera-fuzz-$name" \
    "build-fuzz/corpus/$name" "tests/fuzz/corpus/$name" \
    -max_total_time=3600 -max_len=4096 -timeout=5 -rss_limit_mb=512 \
    "-artifact_prefix=build-fuzz/artifacts/$name/"
```

The first corpus directory is writable; the second supplies checked-in seeds.
CTest uses its own writable corpus and artifacts in `build-fuzz/tests/fuzz/`.
Keep generated discoveries under the ignored build directory. Checked-in seeds
cover output layouts, hardware geometry boundaries, enable bits, sensor models,
arithmetic extremes, history wrap and malformed metadata. To regenerate these
small binary files, run `python3 tests/fuzz/generate_corpus.py` (optional; Python
is not needed for building or fuzzing). Integer fields are read little-endian,
missing bytes are zero-filled, and arbitrary tuning patches use native UAPI
byte offsets on the host.

Sanitizer findings, property failures and timeouts stop the process and write a
reproducer under the artifact directory. Replay it with the same executable:

```sh
build-fuzz/fuzz/ccamera-fuzz-pisp build-fuzz/artifacts/pisp/crash-HASH
```

Minimize a reproducible failure before adding it to the appropriate checked-in
corpus alongside the fix:

```sh
build-fuzz/fuzz/ccamera-fuzz-pisp -minimize_crash=1 \
    -exact_artifact_path=build-fuzz/minimized-pisp \
    build-fuzz/artifacts/pisp/crash-HASH
```
