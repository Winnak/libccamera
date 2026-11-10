# Image processing algorithms and autofocus

[Project overview](../README.md). Run example commands from the repository root.

- [C IPA modules and automatic controls](#c-ipa-modules-and-automatic-controls)
- [Loading shared C modules](#loading-shared-c-modules)
- [Discovering and selecting modules](#discovering-and-selecting-modules)
- [OV5647 VC4 still profile](#ov5647-vc4-still-profile)
- [Autofocus and lens control](#autofocus-and-lens-control)

## C IPA modules and automatic controls

[include/ccamera/ipa.h](../include/ccamera/ipa.h) defines a versioned C module ABI and
an explicit reference AE/AGC/AWB controller with optional RPi autofocus. `cc_capture_set_ipa` configures and
adopts a module on an idle stopped graph with a statistics port, or the existing
VIMC graph with its mock IPA. Success clears
the caller's module pointer; failure retains caller ownership. NULL detaches and
destroys the previous module. The graph calls configure/start on startup and
restart, prepare before capture, process after successful frame completion, and
stop/destroy during teardown. All callbacks run in the caller's dispatch thread.

The reference module uses proportional exposure/gain control and damped grey-world
white balance. Its tuning is a C struct; no JSON/YAML files or libm are needed.
AE requires a supported sensor gain/exposure model and configured exposure/gain
delays. It respects native control steps, live driver timing, frame-length margins
and an optional exposure ceiling. By default it keeps the current frame length.
Setting both `min_frame_duration_ns` and `max_frame_duration_ns` opts into native
exposure/blanking planning. This requires managed priority VBLANK with a delay
at least as long as EXPOSURE. Managed HBLANK can vary when its delay is sufficient;
otherwise it stays at its live value. This mode requests `CC_IPA_CAP_SENSOR_TIMING`
and receives live timing limits before each prepare call. The driver still checks
and acknowledges every write. Caller shutter/blanking overrides take precedence;
impossible manual combinations fail before submission. AWB programs
VC4 red/blue/digital gains or PiSP white-balance and CFE luminance weights. Raw CFE
can use AE alone; raw Unicam without an ISP has no statistics port.

For an already configured, idle capture graph:

```c
#include <ccamera/capture.h>

int enable_reference_controls(cc_capture *capture, uint32_t algorithms)
{
    int ret = 0;
    if (algorithms & CC_IPA_REFERENCE_AE) ret = cc_capture_configure_sensor_defaults(capture);
    if (ret < 0) return ret;
    cc_ipa_reference_tuning tuning = {
        .flags = algorithms, /* CC_IPA_REFERENCE_AE, optionally | CC_IPA_REFERENCE_AWB */
        .target_luminance = 0.18, .exposure_speed = 0.25, .white_balance_speed = 0.25,
        .min_colour_gain = 0.5, .max_colour_gain = 4
    };
    cc_ipa *ipa = NULL;
    ret = cc_ipa_reference_create(&tuning, &ipa);
    if (!ret) ret = cc_capture_set_ipa(capture, &ipa);
    cc_ipa_destroy(ipa); /* NULL after adoption; otherwise still caller-owned. */
    return ret;
}
```

Each run gets a private snapshot of module settings. Explicit request sensor/ISP
controls override matching generated IDs; explicit FE/BE tuning replaces the whole
corresponding generated block. Reusing a request does not retain generated values
as caller settings. `result.ipa` contains validity/convergence flags and the
expected applied exposure/gain/WB values. Sensor values include driver adjustments
from the delay model; these are not measurements from embedded sensor data.
Empty native statistics leave the controller unchanged. Malformed statistics or
module failures cancel the graph and invalidate views through the usual lifecycle.

Both examples accept `-g`: on `ccamera-capture` it enables VC4 processing and
reference AE/AWB; on `ccamera-cfe-capture` it adds AE, plus AWB when `-p` is used.
It explicitly opts into known model delay defaults and uses the tuning shown
above. The examples print exposure/gain, histogram mean and white-balance metadata.

```sh
./build/ccamera-capture -c 0 -w 1640 -h 1232 -f pRAA -g -n 60 -o frames.rgb
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA -g -p -F NV12 -n 60 -o frames.nv12
```

The exposure/white-balance controller is a basic reference, not a calibrated camera profile.
Bayesian AWB, flicker/HDR policies, long-exposure planning and sensor-specific
colour/black-level/lens-shading/gamma tuning remain pending. The C ABI supports
caller-registered and explicitly loaded shared C modules; upstream C++ IPA binaries
need a bridge. Directory discovery and pipeline/version selection are available;
the [IPC worker/proxy](IPC.md) supports isolated C modules and built-in controllers
on the current VC4/CFE/PiSP graphs and the existing VIMC mock. Signature
verification remains pending. Simulations prove
feedback and lifecycle behavior, not physical image quality.

## Loading shared C modules

[include/ccamera/ipa_module.h](../include/ccamera/ipa_module.h) provides explicit
ELF inspection and loading for the native C IPA ABI. `cc_ipa_module_open` reads
metadata without calling `dlopen` or running constructors. It checks the native
ELF class, byte order and architecture, bounded symbol/section extents, both API
versions and terminated printable names. Module files are limited to 64 MiB.
`cc_ipa_module_match` compares the exact pipeline name and an inclusive pipeline
version interval. Pipeline names and versions are contracts between the module
and its application; the factory/configure callback must also reject unsupported
configurations.

```c
#include <ccamera/ipa_module.h>
#include <ccamera/capture.h>
#include <errno.h>

int enable_shared_ipa(cc_capture *capture, const char *path)
{
    cc_ipa_module *module = NULL;
    cc_ipa *ipa = NULL;
    int ret = cc_ipa_module_open(path, &module);
    if (ret < 0) return ret;
    ret = cc_ipa_module_match(module, "rpi/vc4", 1, 1);
    if (ret == 0) ret = -ENOENT;
    else if (ret > 0) ret = cc_ipa_module_create(module, NULL, 0, &ipa);
    cc_ipa_module_close(module); /* An instantiated IPA holds its own reference. */
    if (!ret) ret = cc_capture_set_ipa(capture, &ipa);
    cc_ipa_destroy(ipa); /* NULL after adoption; otherwise still caller-owned. */
    return ret;
}
```

The first create loads the absolute module path using `RTLD_NOW | RTLD_LOCAL`.
Relative `$ORIGIN` dependencies retain normal dynamic-linker behavior. Subsequent
instances share the loaded object and have independent contexts. The loader
detects changes/replacement since inspection and checks the loaded descriptor
again. Linux `/proc/self/maps` checks the mapped inode, so an older image cached
by the dynamic linker cannot satisfy a newly inspected replacement. Do not modify
the module or its dependencies concurrently. The object remains loaded until
the caller closes its handle and every instance has finished
its stop/destroy callbacks, including instances adopted by capture graphs.
Factory failures and malformed callback tables publish no IPA handle; successful
factory contexts are destroyed when later validation fails.

A shared module exports two symbols with default visibility and C linkage:

- A const `cc_ipa_module_info` object named `cc_ipa_module_descriptor`. Set
  `module_api_version = CC_IPA_MODULE_API_VERSION`,
  `ipa_abi_version = CC_IPA_ABI_VERSION`, a pipeline version and both names.
- A function named `cc_ipa_module_create_v1` matching `cc_ipa_module_factory`.
  It checks the host ABI and table size, fills the supplied `cc_ipa_ops`, and
  returns its context. Module-defined tuning bytes are borrowed for this call.
  On every nonzero return the factory retains responsibility for cleanup.
  After success, the ordinary IPA callback and destroy rules apply.

These modules execute trusted code in the calling process. The optional
`.sign` sidecar beside the resolved module is copied as opaque bytes (up to 1 MiB),
available through `cc_ipa_module_get_signature`; it is not verified. Missing or unreadable signatures
are empty. Direct module loading does not search directories. No automatic fallback,
trust policy or sandbox is implied. C++ libcamera IPA binaries use a different interface. A module calling
other libccamera APIs must link those symbols normally; a statically linked host
does not automatically export its library symbols to plugins.

## Discovering and selecting modules

[include/ccamera/ipa_manager.h](../include/ccamera/ipa_manager.h) discovers a module
snapshot without loading code. `cc_ipa_manager_open(NULL, &manager)` searches
`CCAMERA_IPA_MODULE_PATH` (colon-separated), a detected uninstalled build's `ipa/`
directory, then the configured installation directory, normally
`${CMAKE_INSTALL_FULL_LIBDIR}/ccamera/ipa`. Empty environment
components are skipped, including leading/trailing colons. Environment paths are
ignored when Linux reports `AT_SECURE` or the real/effective user or group IDs
differ. Installation lookup can be set with CMake's `CCAMERA_IPA_MODULE_DIR`.

Explicit options place caller directories first, followed by environment,
build-tree and installation sources only when their flags are enabled. Build-tree
lookup uses `CC_IPA_MANAGER_BUILD_TREE` and recurses to depth 2. It identifies the
active library/executable image and the generated `.ccamera-build-tree` marker
beside it. CMake does not install the marker. Image-relative paths support moved
build trees, symlink launches and changes of working directory; Linux mapped-image
lookup avoids resolving relative loader names against a new working directory.
Programs statically linked outside this build layout can supply explicit
directories instead. Lookup does not require RPATH. Nested real directories are traversed
up to the requested depth; nested symlink directories are skipped. `.so` file symlinks are
accepted. Each directory's complete candidate list is sorted before metadata
inspection, preserving directory priority. Duplicate candidates/aliases retain
their ordering. Missing/inaccessible directories and invalid modules are skipped;
resource, traversal and limit failures abort construction with a NULL manager.

```c
#include <ccamera/ipa_manager.h>
#include <ccamera/capture.h>

cc_ipa_module_directory directories[] = {{ "/opt/camera/ipa", 0 }};
cc_ipa_manager_options options = {
    .directories = directories, .num_directories = 1,
    .flags = CC_IPA_MANAGER_INSTALLED, .max_modules = 32
};
cc_ipa_manager *manager = NULL;
int ret = cc_ipa_manager_open(&options, &manager);
if (ret < 0) return ret;
cc_ipa *ipa = NULL;
ret = cc_ipa_manager_create(manager, "rpi/vc4", 1, 1, NULL, NULL, 0, &ipa);
cc_ipa_manager_close(manager); /* The created IPA retains its module. */
if (!ret) ret = cc_capture_set_ipa(capture, &ipa);
cc_ipa_destroy(ipa);
```

Selection follows the archived reference: the **first** exact pipeline name and
inclusive version match wins. It does not choose the highest version. An optional
metadata name narrows selection; `cc_ipa_manager_select` returns the index for
inspection before loading, and indexed getters expose metadata, canonical paths
and opaque signature bytes. Factory/link failures from the selected module are
returned directly, without trying lower-priority modules. Reopen the manager to
discover filesystem changes; existing IPA instances keep their code after the
manager closes. Created IPAs use the same capture adoption/restart/cancellation
lifecycle as directly loaded modules. Creation loads trusted in-process code;
the manager does not verify signatures. Use `cc_ipa_manager_create_proxy` from
`ccamera/ipa_proxy.h` for a separate worker with user/network namespaces; see
[isolated C IPAs](IPC.md#isolated-c-ipas).

Discovery is bounded to 64 search directories, depth 8, 4096 candidates, 65536
directory entries and at most 256 valid modules (or a lower `max_modules`). The
environment text limit is 64 KiB. Exceeding search limits returns `-E2BIG` and
cleans up the entire partial snapshot. Explicit argument-range errors return
`-EINVAL`. No threads or background rescans are created.

## OV5647 VC4 still profile

An opt-in OV5647/VC4 calibrated still controller and `ccamera-still` example now
provide black level, gamma, colour matrices, calibration-based lens shading,
noise/sharpening controls, Bayesian AWB and single-channel AE with 50/60 Hz
anti-flicker. The tool waits for settled settings and saves a PPM with optional
matching raw/statistics and diagnostic metadata. Two-frame OV5647 delays remain
provisional. See the [Pi bring-up and validation guide](PI4_OV5647.md).
Physical timing and image quality still require your Pi captures. The generic
reference controller remains available. The C IPA ABI is now version 5.

## Autofocus and lens control

[include/ccamera/af.h](../include/ccamera/af.h) exposes a C17 port of the supplied
upstream RPi hybrid AF state machine: confidence-weighted phase feedback, coarse
and fine contrast scans, quadratic peak interpolation, continuous scene-change
retriggering, range/speed selection, ten weighted focus windows, manual focus,
trigger/cancel, and immediate/deferred pause and resume. Optional IR detection
suppresses unreliable phase feedback. It uses explicit C tuning and a copied
piecewise-linear dioptres-to-driver-code calibration map; no libm or config-file
parser is required. The reference profile assumes the standard V3/IMX708 lens.
Other modules and lenses need their own calibration and settling parameters.
The control sequence follows the [RPi autofocus controller](https://github.com/libcamera-org/libcamera/blob/v0.7.2/src/ipa/rpi/controller/rpi/af.cpp).

Use `cc_af_create/configure`, then alternate `cc_af_prepare` and `cc_af_process`
(or `cc_af_process_native`). Prepare accepts optional PDAF grids; process accepts
contrast/colour grids or native BSTA/RPFS payloads. Native contrast scales follow
upstream: VC4 uses filter `[1][1] / 1000`, PiSP uses `foms >> 20`. The controller
copies tuning and windows and borrows statistics only for each call. Default
tuning assumes at most 30 updates/s; its settling counts and slew limits are
relative to controller updates. Invalid tuning, oversized grids, overflowing
statistics and malformed native payloads return negative errno.

For capture integration, set `CC_IPA_REFERENCE_AF` and provide `tuning.af` when
creating the reference IPA. It copies that tuning and declares `CC_IPA_CAP_LENS`.
ABI version **4** carries AF, timestamps, optional embedded data and opt-in live
timing limits/delay descriptions; C modules must be
rebuilt against the current header. Lens access is opt-in, so AE/AWB-only modules
do not open or lock a lens. RPi lens discovery follows media-v2 ancillary links
from the selected sensor; ambiguous associations are rejected. Self-contained
V4L2 cameras use `FOCUS_ABSOLUTE` on their capture node. Native query/get/set lens
APIs are available on camera, CFE and capture objects; setters retain the driver's
acknowledged native code. The control's units are [driver-defined](https://docs.kernel.org/userspace-api/media/v4l/ext-ctrls-camera.html).

`cc_request_set_af_controls` copies optional commands. A block's mode change is
applied before its other commands.
For example, after attaching an AF-enabled IPA:

```c
cc_af_controls af = { .flags = CC_AF_SET_MODE | CC_AF_TRIGGER, .mode = CC_AF_AUTO };
int ret = cc_request_set_af_controls(request, &af);
```

After dequeue and release, clear the one-shot trigger with
`cc_request_set_af_controls(request, NULL)`.

Trigger/cancel commands apply on each reuse until cleared. Mode, speed, range
and windows persist inside the controller. Windows use source-image coordinates
for graph integration. `result.ipa.af` reports state, pause state and the expected
calibrated lens position/native code for that frame. These are commanded values,
not physical lens measurements. Held results keep their own snapshots. Actuator
errors cancel the graph; immediate native lens writes are busy while a running
IPA owns the lens. Restart reconfigures AF to manual/unknown position, so send
the desired mode again. Continuous AF stays paused across scene changes.

The reference IPA advances AF only with new completed statistics, limiting its
automatic update rate to about 30 Hz when frame timestamps advance. Otherwise,
the caller must provide an appropriate update cadence. Explicit command changes
are applied immediately. The graph supplies contrast/colour statistics and,
when embedded capture is configured before allocation, timestamp-paired IMX708
PDAF. Missing or invalid phase data falls back to contrast without reusing the
previous frame's grid. Standalone/custom modules can also feed normalized PDAF.
Physical timing, focus quality and module calibration still need Pi validation.

Both examples accept `-j` to opt into the standard V3/IMX708 continuous AF profile;
they reject other detected sensor models. `ccamera-capture` enables VC4 processing
for focus statistics; CFE can autofocus raw output without the BE. `-g` additionally
enables AE/AWB as before. For example:

```sh
./build/ccamera-cfe-capture -c 0 -w 1640 -h 1232 -f pRAA -g -j -p -n 120 -o frames.rgb
```
