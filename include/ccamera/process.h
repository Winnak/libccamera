/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PROCESS_H
#define CCAMERA_PROCESS_H
#include <ccamera/ccamera.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_PROCESS_MAX_ARGUMENTS 128U
#define CC_PROCESS_MAX_FDS 256U
typedef struct cc_process cc_process;
enum cc_process_flags {
    CC_PROCESS_NEW_USER = 1U, CC_PROCESS_NEW_NET = 2U,
    CC_PROCESS_PARENT_DEATH = 4U /* Linux parent-thread-death SIGKILL semantics. */
};
enum cc_process_status { CC_PROCESS_RUNNING = 0, CC_PROCESS_EXITED, CC_PROCESS_SIGNALLED, CC_PROCESS_EXIT_UNKNOWN };
typedef struct cc_process_result {
    int pid;
    enum cc_process_status status;
    int exit_code; /* -1 unless EXITED. */
    int signal; /* 0 unless SIGNALLED. */
    uint32_t core_dumped;
    uint32_t exec_complete;
    int exec_error; /* 0 or negative errno, valid when exec_complete is set. */
} cc_process_result;
typedef struct cc_process_options {
    const char *path;
    const char *const *arguments; /* Excludes argv[0]; strings are copied. */
    size_t num_arguments;
    const int *fds; /* Fd numbers preserved in child, CLOEXEC cleared there only. */
    size_t num_fds;
    uint32_t flags; /* Exact namespace policy; no silent downgrade. */
} cc_process_options;
/* clone3/CLONE_PIDFD, then exec with inherited environment. Requires kernel
 * support; namespace-policy errors are returned directly. The reference IPA
 * worker namespace flags are NEW_USER|NEW_NET. No background thread or SIGCHLD
 * handler is installed. Only requested fds and stderr survive; unused standard
 * input/output/error refer to /dev/null. Parent descriptors are unchanged.
 * Spawn is asynchronous; wait_exec reports exec completion/errors separately.
 * Max 128 arguments, 256 passed fds, and 1 MiB of path/argument text.
 * *process is NULL on error. Calls on an object must be serialized. */
CC_API int cc_process_spawn(const cc_process_options *options, cc_process **process);
/* Kill a still-running child through its pidfd, reap it, and release resources.
 * Does not signal a reused numeric pid. NULL is allowed.
 * A signal/wait failure leaves the object owned by the caller for retry. */
CC_API int cc_process_close(cc_process *process);
/* Original pid for diagnostics; fd is borrowed, CLOEXEC and pollable for exit.
 * After exit collection, fd returns -ENODATA. */
CC_API int cc_process_pid(const cc_process *process);
CC_API int cc_process_fd(const cc_process *process);
/* Borrowed readiness fd for exec completion; -ENODATA after collection. */
CC_API int cc_process_exec_fd(const cc_process *process);
/* Collect exec completion: 0 polls, -1 waits indefinitely. An exec error or
 * timeout does not relinquish process ownership. */
CC_API int cc_process_wait_exec(cc_process *process, int timeout_ms);
/* Nonblocking exit collection. -EAGAIN means still running. If the application
 * reaped the child or ignores SIGCHLD, an observed exit is EXIT_UNKNOWN. */
CC_API int cc_process_dispatch(cc_process *process);
/* Snapshot, including RUNNING. Does not wait or collect an exit. */
CC_API int cc_process_get_result(const cc_process *process, cc_process_result *result);
/* Wait and collect: 0 polls, -1 waits indefinitely. Timeout/error leaves result
 * unchanged and process ownership intact. Repeated waits on an exit succeed. */
CC_API int cc_process_wait(cc_process *process, cc_process_result *result, int timeout_ms);
/* Signal through pidfd_send_signal. signal=0 probes; -ESRCH after collection. */
CC_API int cc_process_signal(cc_process *process, int signal);

#ifdef __cplusplus
}
#endif
#endif
