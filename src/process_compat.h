/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PROCESS_COMPAT_H
#define CCAMERA_PROCESS_COMPAT_H
#include <stdint.h>
#include <linux/sched.h>
#include <sys/syscall.h>

#ifndef CLONE_PIDFD
#define CLONE_PIDFD 0x00001000
#endif
#ifndef CLONE_ARGS_SIZE_VER0
struct clone_args {
    uint64_t flags, pidfd, child_tid, parent_tid, exit_signal;
    uint64_t stack, stack_size, tls, set_tid, set_tid_size, cgroup;
};
#endif
/* The native x86-64/AArch64 syscall numbers are stable even with older UAPI
 * headers. Other architectures use their installed syscall definitions. */
#if (defined(__x86_64__) || defined(__aarch64__)) && !defined(__ILP32__)
#ifndef SYS_clone3
#define SYS_clone3 435
#endif
#ifndef SYS_pidfd_send_signal
#define SYS_pidfd_send_signal 424
#endif
#ifndef SYS_pidfd_open
#define SYS_pidfd_open 434
#endif
#endif
#endif
