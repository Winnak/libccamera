# IPA process communication

[Project overview](../README.md). This tracks IPC parity for the native C IPA
interface. The reference is the archived libcamera 0.7.2 source identified in

IPC parity is implemented for the native C IPA interface: transport, process
supervision, RPC, workers, callback serialization and current-pipeline integration.

| Layer | Required behavior | Status / evidence |
| --- | --- | --- |
| Unix transport | Ordered datagrams, bytes and owned descriptors, nonblocking progress and polling, bounded parsing and complete cleanup | Implemented in `ipc.c`; native socket/process tests and 155 compiled reference fixtures |
| Message framing | Native command/cookie prefix and descriptor-preserving payloads | Implemented; reference `IPCMessage::payload` fixtures and cross-process round trip |
| Process supervision | Exec and fd allowlist, pidfd exit/signal handling, user/network namespace isolation, startup failure, termination and reaping | Implemented; native exec/pidfd/namespace tests, forced proc-fd fallback and multithreaded spawning |
| IPC pipe / RPC | Synchronous calls with deadlines, asynchronous messages, cookie correlation, notifications and disconnection | Implemented; cross-process RPC, notifications/FD ownership, deadlines, backpressure and bounded queues |
| C IPA serialization | Configure/start/stop/prepare/process/destroy, capabilities, sensor identity, controls and payloads, AF/tuning, statistics/embedded data, safe buffer/fd lifetime; no cross-process pointers | Implemented; callback codec tests, large sealed snapshots and namespace-worker round trips |
| IPA worker/proxy | Worker lookup in build/install trees, module and built-in controller creation, independent instances, worker errors/timeouts and host-output validation | Implemented; actual module/built-in workers, build relocation and installed-worker checks |
| Capture integration | Current VC4/CFE/PiSP graphs, restart/cancellation/held results, existing VIMC mock IPC behavior | Implemented; proxy graph tests and VIMC mock maps, async requests/events and lifecycle traces |

The socket framing and command/cookie prefix are compatible with the archived
transport. Upstream C++ IPA binaries use a different interface from the C IPA
ABI; C++ binary/ABI compatibility is outside this port's scope.

## Byte and descriptor transport

[`ccamera/ipc.h`](../include/ccamera/ipc.h) creates an unnamed connected,
nonblocking `AF_UNIX/SOCK_DGRAM` pair. The local endpoint is a `cc_ipc` handle;
the caller owns the remote descriptor, which can be passed to another process
and bound there. Binding duplicates an already-connected nonblocking endpoint
without changing the caller's descriptor. Both library-owned and received
descriptors have CLOEXEC set. Calls on a channel must be serialized.

Each payload uses two datagrams, matching `IPCUnixSocket`: an 8-byte header with
a native-endian 32-bit byte count and an 8-bit descriptor count, followed by the
bytes with `SCM_RIGHTS`. Header padding is ignored on receive. The native bounds
are 65536 bytes and 253 descriptors; an fd-only payload is supported, while a
completely empty raw payload is invalid. `cc_ipc_send_message` prepends the
archived native-endian command/cookie pair; its 8 bytes count toward the limit.

`cc_ipc_send` snapshots bytes and duplicates all descriptors before writing the
header. Success means accepted, allowing caller data and descriptors to be
released immediately. A pending send occupies one slot; another send returns
`-EBUSY`. Call `cc_ipc_flush` when polling reports writable readiness. If the
header was sent and the payload is temporarily blocked, the saved payload and
descriptors remain owned until flush or close. An unrecoverable send error
fails the channel, avoiding a later payload being mistaken for that frame.

`cc_ipc_receive` retains a header until its payload arrives. `-EAGAIN` publishes
no packet. Success returns an owned packet: getters borrow its bytes/descriptor
array, `cc_ipc_packet_take_fd` transfers a selected descriptor, and destroying the
packet closes all untaken descriptors. A packet remains usable after closing
the channel. Framing errors, length/count mismatches, truncation and unsupported
ancillary records fail the channel and close received handles, including Linux
pidfds in unsupported ancillary records. Closing also drops queued kernel
references and any partially received frame.

`cc_ipc_fd` exposes a borrowed poll descriptor and `cc_ipc_events` reports
`POLLIN`, plus `POLLOUT` while a send is pending. These functions do not wait,
dispatch callbacks or create threads. A datagram socket does not provide the
same read-side EOF semantics as a byte stream: worker-exit monitoring and RPC
deadlines belong to the process/pipe layers above it.

## Process supervision

[`ccamera/process.h`](../include/ccamera/process.h) uses Linux `clone3` with
`CLONE_PIDFD` and the exact namespace flags requested by the caller. IPA workers
use `CC_PROCESS_NEW_USER | CC_PROCESS_NEW_NET`, matching the reference. Kernel or
namespace policy errors are returned without silently reducing isolation.
Spawning is asynchronous: a second pollable descriptor reports exec errors,
and `cc_process_wait_exec` supports a deadline while retaining ownership on
timeout. Paths and argument strings are copied; no shell is involved.

The child preserves only the explicit fd allowlist and stderr. Unused standard
descriptors refer to `/dev/null`; passed descriptor numbers survive exec without
changing the parent's CLOEXEC flags. Closing other fds uses `close_range` where
available, with a `/proc/self/fd` fallback that also finds descriptors above a
lowered resource limit. Child setup avoids allocation and inherited application
signal handlers after cloning, including when the parent is multithreaded.

The borrowed pidfd is pollable for exit. Signals use `pidfd_send_signal` and exit
collection uses `waitid(P_PIDFD)`, avoiding numeric PID reuse. Close kills and
reaps a running child; a signal/wait failure leaves the object owned for retry.
The optional parent-death flag uses Linux parent-thread-death semantics. No
SIGCHLD handler or background thread is installed. If the application has
already reaped the child or ignores SIGCHLD, exit is reported as `EXIT_UNKNOWN`.
All calls on one object must be serialized.

## RPC pipe

[`ccamera/ipc_pipe.h`](../include/ccamera/ipc_pipe.h) adopts a transport and can
monitor a duplicated peer pidfd. `cc_ipc_pipe_send` accepts asynchronous messages;
`cc_ipc_pipe_call` flushes earlier output, sends a request and matches its reply
by cookie. The reference call deadline is available as
`CC_IPC_PIPE_DEFAULT_TIMEOUT_MS` (2000 ms). Calls use one monotonic deadline
across flushing, sending and receiving. A valid final reply is collected before
reporting peer exit. Peer death without a reply fails promptly with `-EPIPE`.

Incoming messages with other cookies are queued in order. The application drains
them through `cc_ipc_pipe_receive`, including after a synchronous call; no
application callback runs inside the call. Messages retain their bytes and fds
independently of the pipe lifetime. Queues are bounded to 64 messages, 1 MiB and
256 fds; overflow fails the pipe and closes the rejected packet. Previously
queued messages remain readable before the failure is returned.

Timeout after a request is accepted fails the pipe and releases any retained
transmit snapshot. This avoids reusing uncertain remote state or matching a late
reply to a new call. A receive-only timeout leaves the pipe usable. A call that
times out while flushing earlier asynchronous output, before accepting its own
request, also leaves the pipe usable. Caller storage and descriptors can be
released immediately after an asynchronous send succeeds. All calls on a pipe
must be serialized; no thread or event dispatcher is created.

## Native IPA data

The private C IPA codec encodes configuration, requests, settings, frames,
metadata and built-in controller options. Sensor models cross as canonical names
and are looked up locally. Control arrays and their payloads, optional AF input,
native statistics/embedded bytes and optional backend/VC4 snapshots are deep
copied. Decoding owns every reconstructed pointer. Scalar fields use the native
byte order; pointer-free kernel register blocks preserve their native layout.
Protocol version, C IPA ABI and native block sizes must match. This is separate
from the archived C++ IPA serialization ABI.

Native VC4 lens-shading control payloads carry a DMA-BUF fd. The codec replaces
that field with a descriptor-table index and transfers the actual handle with
`SCM_RIGHTS`. Decoding owns CLOEXEC duplicates independently of packet lifetime;
prepare results retain them until the next callback. Request snapshots duplicate
them again for capture lifetime. Disabled shading carries no handle. Opaque
custom payload bytes must be process independent; arbitrary embedded pointers
or unknown handle schemas cannot be inferred. The descriptor table is bounded
to 192 handles, with unused or invalid references rejected.

Messages up to the socket payload limit carry inline bytes. Larger snapshots use
one CLOEXEC memfd sealed against writes, growth, shrinking and seal changes.
The receiver checks seals, exact file size and descriptor count before copying
the data. The total serialized data limit is 16 MiB; oversize data returns an
error. Caller storage can be released after acceptance. Received data and
descriptors remain valid after closing the channel, until their owner is freed.

## Isolated C IPAs

[`ccamera/ipa_proxy.h`](../include/ccamera/ipa_proxy.h) creates ordinary `cc_ipa`
handles backed by independent exec workers. Module, manager-selected, reference
AE/AWB/AF and calibrated VC4 constructors have proxy variants. Each instance uses
new user and network namespaces, with no downgrade on policy/kernel failure.
The parent inspects module metadata without loading module code. A retained file
descriptor, file stamp and metadata cross to the worker; import and loading
reject stale files. Loading uses the original canonical path, preserving module
`$ORIGIN` dependencies. Closing the parent's module or manager leaves the worker
instance valid.

Worker lookup accepts an explicit path, then `CCAMERA_IPA_PROXY_PATH` directories
(ignored when privileged), the active build image and the configured libexec
directory. The executable is self-contained in static/shared installations.
Each callback uses the caller's configured deadline (2000 ms by default).
Factory and callback errors propagate; protocol, timeout and worker failures
remain persistent for that instance. Host IPA validation still checks outputs.
Prepare payloads stay proxy-owned until the next callback, and inputs borrowing
those payloads are encoded before releasing them. Stop preserves the worker for
restart; destroy stops, waits for normal exit and kills/reaps on failure.
Between callbacks, workers monitor a parent pidfd and clean up when the host exits.

Module-defined factory data must be relocatable bytes. Arbitrary pointers in a
custom factory blob cannot be translated; the standard callback objects and
built-in tuning have explicit codecs. To adopt a proxy into an existing graph:

```c
cc_ipa *ipa = NULL;
int ret = cc_ipa_reference_create_proxy(&tuning, NULL, &ipa);
if (!ret)
    ret = cc_capture_set_ipa(capture, &ipa);
cc_ipa_destroy(ipa); /* NULL after successful adoption. */
```

## Existing VIMC mock

[`ccamera/ipa_vimc.h`](../include/ccamera/ipa_vimc.h) adapts the archived VIMC test
IPA to the C interface. Supply nonempty configuration bytes and an optional
nonblocking trace writer. Init returns Flag1 and traces the native uint32 Init
operation; start/stop trace their operations. The mock performs no image
algorithm. VIMC capture graphs may adopt it without a statistics node.

Start maps two mock 160x120 BGR parameter buffers into the worker; stop unmaps
them. Requests send asynchronous queue operations and validate asynchronous
parameters-computed notifications for mapped IDs. Manual diagnostics expose
map/unmap, queue, compute, dispatch and event dequeue. Multiple planes and
non-page-aligned offsets are supported, with 16 active buffers and 16 MiB of
mapped data as bounds. Mapping duplicates kernel references and rolls back
atomically on failure. Unknown unmap IDs are ignored; computing an unknown ID
emits no event. Notifications survive intervening synchronous calls in a bounded
queue. Destroy releases mappings, trace references and the worker.

Configuration is supplied as bytes instead of using the C++ tuning-file locator;
native callback and graph ownership follow the existing C API.

## Validation

`ipc-transport` checks byte compatibility against 155 fixtures generated by
compiling unmodified archived `IPCUnixSocket::send`, `sendData` and
`IPCMessage::payload` methods with recording syscalls. It also exercises actual
kernel descriptor transfer, ordering and CLOEXEC, fd-only messages, bidirectional
traffic, an echo across `fork`, descriptor zero, transfer of ownership, real
kernel backpressure, partial-send snapshots and injected send/receive failures.

`ipc-malformed` rejects bad framing, oversized declarations, mismatched payload
lengths/counts and unexpected descriptor-bearing headers. It verifies cleanup
after actual ancillary truncation and unsupported pidfd records, as well as pipe
EOF after discarding descriptors still queued in the kernel. Bind validation,
limits and failures before transmission preserve caller-owned descriptors.

`ccamera-fuzz-ipc` mutates raw and unaligned headers/ancillary records through pure
decoders. Descriptor values are opaque integers in that target: it opens no
socket and never uses them as OS handles.

`process-lifecycle` checks exec arguments/environment, missing/nonexecutable and
invalid executables, fd allowlisting/CLOEXEC, fd zero, timeout ownership, signals,
reaping and unrelated-child preservation. A stopped pre-exec child verifies
startup deadlines and cleanup; malloc activity on a second thread exercises
spawning without child allocation. The forced proc-fd fallback closes a hidden
fd above a lowered soft fd limit. Parent death and external/SIGCHLD-ignored
reaping are also exercised. `process-namespaces` verifies distinct user and
network namespace inodes with both fd-closing paths. These tests skip only when
the kernel or execution policy does not support the requested clone flags.

`ipc-rpc` exercises a real exec worker: cookie zero/wraparound, FD-preserving
replies, notifications during a call, optional discarded replies, final-reply
delivery and prompt peer-exit failure. Missing replies and blocked partial sends
verify deadlines and retained-resource cleanup. Injected backpressure covers
both framing stages and flushing an earlier asynchronous message. Local checks
cover queue wraparound, message/byte/fd limits, malformed messages and pipe EOF
after releasing queued descriptor references.

`ipa-wire` checks all object kinds, sensor/AF reconstruction, immutable input
arrays, deep copies, large payloads, every truncation, unaligned input, trailing
bytes and protocol/header mismatch. The pure `ipa_wire` fuzzer mutates those
schemas and checks canonical decode/encode stability without OS or module I/O.
`ipa-message` checks empty, boundary-size and 16 MiB snapshots, sender-storage
mutation, channel-independent ownership, fd zero and attempted writes/truncation
of received memfds. Missing/extra fds, incomplete seals, file-size mismatches and
malformed envelopes are rejected without leaking handles. Native descriptor-table
tests cover inline/large snapshots, up to 192 auxiliary handles, lens-shading
round trips in both directions, independent ownership and request retention.

`ipa-proxy` verifies module execution in a different PID and user/net namespaces,
callback data with large payloads, output ownership, independent instances,
restart and descriptor zero. Factory/callback errors, invalid outputs, worker
exit, hangs, first-match worker lookup and parent exit exercise cleanup/reaping.
Constructor/destructor traces and the parent's mapped images confirm that the
module executes only in the worker. The algorithm and calibrated VC4 test suites
also run through proxy constructors with their ordinary expected results.

Current graph tests adopt direct/manager-selected proxies after module/manager
closure and exercise raw CFE, VC4 and PiSP completion, AF/PDAF, embedded data,
timing, native settings, imported buffers, held results, restart and cancellation.
Runtime probes copy/move the worker with the active image and test symlink and
relative launches after changing directory. The installed consumer creates an
IPA using the installed worker binary.

`ipa-vimc-ipc` checks lifecycle traces/flags, multiple-plane maps, unaligned
offsets, full tables, atomic rollback, duplicate IDs, unknown IDs, fd zero,
asynchronous requests/notifications and repeated cleanup. Closed trace readers
remain best-effort errors. `ipa-vimc-graphs` exercises existing VIMC MMAP/imported
capture, controls, held results, restart, cancellation and injected driver errors
with the actual isolated mock.

Regenerate the checked-in fixtures with Python 3, C++17 and the reference archive:

```sh
python3 tools/generate_ipc_vectors.py /path/to/libcamera-reference.tar.xz
```

Ordinary C builds use the checked-in fixtures and do not require C++ or the
archive. Ordinary test runs need no physical camera; capture hardware remains
unverified in this workspace.
