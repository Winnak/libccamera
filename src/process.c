/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* clone3/pidfd namespace and fd-allowlist model follows libcamera/process.cpp.
 * Reference: Copyright (C) 2019 Google Inc. */
#define _GNU_SOURCE
#include "process.h"
#include "process_compat.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define PROCESS_TEXT_LIMIT (1024U * 1024U)
#define CHILD_SAFE __attribute__((no_sanitize("address", "undefined")))
extern char **environ;
struct cc_process { int pidfd, execfd; cc_process_result result; };
struct child_setup {
    int keep[CC_PROCESS_MAX_FDS + 2U]; size_t num_keep;
    const int *passed; size_t num_passed;
    char *const *arguments;
    int error_fd, parent_pid;
    uint32_t flags;
    bool close_range, stop_exec;
};
static int now_ms(uint64_t *out)
{
    struct timespec time;
    if (clock_gettime(CLOCK_MONOTONIC, &time)) return -errno;
    *out = (uint64_t)time.tv_sec * 1000U + (uint64_t)time.tv_nsec / 1000000U;
    return 0;
}
static int remaining(uint64_t deadline, bool infinite, int *timeout)
{
    if (infinite) { *timeout = -1; return 0; }
    uint64_t now = 0; int ret = now_ms(&now); if (ret < 0) return ret;
    uint64_t delta = deadline > now ? deadline - now : 0;
    *timeout = delta > INT_MAX ? INT_MAX : (int)delta;
    return 0;
}
static CHILD_SAFE bool kept(const struct child_setup *setup, int fd)
{
    size_t low = 0, high = setup->num_keep;
    while (low < high) {
        size_t middle = low + (high - low) / 2;
        if (setup->keep[middle] == fd) return true;
        if (setup->keep[middle] < fd) low = middle + 1; else high = middle;
    }
    return false;
}
static CHILD_SAFE int close_proc_fds(const struct child_setup *setup)
{
    int directory = open("/proc/self/fd", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory < 0) return -errno;
    union { uint64_t alignment; unsigned char bytes[8192]; } buffer;
    int error = 0;
    for (;;) {
        long size = syscall(SYS_getdents64, directory, buffer.bytes, sizeof(buffer.bytes));
        if (size < 0) { if (errno == EINTR) continue; error = -errno; break; }
        if (!size) break;
        size_t offset = 0;
        while (offset < (size_t)size) {
            /* linux_dirent64: ino64/off64/reclen16/type8/name[]. */
            if ((size_t)size - offset < 20) { error = -EPROTO; break; }
            uint16_t length; memcpy(&length, buffer.bytes + offset + 16, sizeof(length));
            if (length < 20 || length > (size_t)size - offset) { error = -EPROTO; break; }
            const unsigned char *name = buffer.bytes + offset + 19;
            size_t capacity = length - 19U, index = 0; unsigned int fd = 0;
            bool numeric = name[0] >= '0' && name[0] <= '9';
            while (index < capacity && name[index]) {
                unsigned int digit = (unsigned int)name[index] - '0';
                if (digit > 9 || fd > ((unsigned int)INT_MAX - digit) / 10U) { numeric = false; break; }
                fd = fd * 10U + digit; ++index;
            }
            if (index == capacity) { error = -EPROTO; break; }
            if (numeric && (int)fd != directory && !kept(setup, (int)fd)) close((int)fd);
            offset += length;
        }
        if (error) break;
    }
    close(directory); return error;
}
static CHILD_SAFE int close_other_fds(const struct child_setup *setup)
{
#ifdef SYS_close_range
    if (setup->close_range) {
        unsigned int previous = 0; bool complete = true;
        for (size_t i = 0; i < setup->num_keep; ++i) {
            unsigned int current = (unsigned int)setup->keep[i];
            if (previous < current && syscall(SYS_close_range, previous, current - 1U, 0)) { complete = false; break; }
            previous = current + 1U;
        }
        if (complete && !syscall(SYS_close_range, previous, UINT_MAX, 0)) return 0;
    }
#endif
    return close_proc_fds(setup);
}
static CHILD_SAFE __attribute__((noreturn)) void child_error(int fd, int error)
{
    int value = error > 0 ? error : EIO;
    ssize_t n; do { n = write(fd, &value, sizeof(value)); } while (n < 0 && errno == EINTR);
    _exit(127);
}
static CHILD_SAFE __attribute__((noreturn)) void child_exec(const struct child_setup *setup)
{
    sigset_t all, original; sigfillset(&all);
    if (sigprocmask(SIG_SETMASK, &all, &original)) child_error(setup->error_fd, errno);
    if (setup->flags & CC_PROCESS_PARENT_DEATH) {
        if (prctl(PR_SET_PDEATHSIG, SIGKILL)) child_error(setup->error_fd, errno);
        if (getppid() != setup->parent_pid) child_error(setup->error_fd, ESRCH);
    }
    if (setup->stop_exec && kill(getpid(), SIGSTOP)) child_error(setup->error_fd, errno);
    int ret = close_other_fds(setup); if (ret < 0) child_error(setup->error_fd, -ret);
    const int modes[3] = {O_RDONLY, O_WRONLY, O_WRONLY};
    for (int expected = 0; expected < 3; ++expected) {
        int fd = open("/dev/null", modes[expected]);
        if (fd < 0) child_error(setup->error_fd, errno);
        if (fd != expected) close(fd);
    }
    for (size_t i = 0; i < setup->num_passed; ++i)
        if (fcntl(setup->passed[i], F_SETFD, 0)) child_error(setup->error_fd, errno);
    if (fcntl(STDERR_FILENO, F_SETFD, 0)) child_error(setup->error_fd, errno);
    /* Match exec's signal disposition before restoring the mask, so an
     * inherited application handler cannot open new descriptors in this gap. */
    for (int signal = 1; signal < NSIG; ++signal) {
        if (signal == SIGKILL || signal == SIGSTOP) continue;
        struct sigaction action;
        if (!sigaction(signal, NULL, &action) && action.sa_handler != SIG_IGN) {
            memset(&action, 0, sizeof(action)); action.sa_handler = SIG_DFL; sigemptyset(&action.sa_mask);
            if (sigaction(signal, &action, NULL)) child_error(setup->error_fd, errno);
        }
    }
    if (sigprocmask(SIG_SETMASK, &original, NULL)) child_error(setup->error_fd, errno);
    execve(setup->arguments[0], setup->arguments, environ);
    child_error(setup->error_fd, errno);
}
/* Keep compiler-inserted sanitizer callbacks out of the post-clone branch. */
static CHILD_SAFE long clone_child(struct clone_args *arguments, const struct child_setup *setup)
{
#if defined(SYS_clone3) && defined(SYS_pidfd_send_signal)
    long pid = syscall(SYS_clone3, arguments, sizeof(*arguments));
#else
    (void)arguments; errno = ENOSYS; long pid = -1;
#endif
    if (!pid) child_exec(setup);
    return pid;
}
static int compare_fds(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b; return (x > y) - (x < y);
}
static int notify_pipe(int descriptors[2])
{
    descriptors[0] = descriptors[1] = -1;
    if (pipe2(descriptors, O_CLOEXEC | O_NONBLOCK)) return -errno;
    for (size_t i = 0; i < 2; ++i) if (descriptors[i] < 3) {
        int fd = fcntl(descriptors[i], F_DUPFD_CLOEXEC, 3);
        if (fd < 0) {
            int error = -errno; close(descriptors[0]); close(descriptors[1]);
            descriptors[0] = descriptors[1] = -1; return error;
        }
        close(descriptors[i]); descriptors[i] = fd;
    }
    return 0;
}
int cc_process_pid(const cc_process *process) { return process ? process->result.pid : -EINVAL; }
int cc_process_fd(const cc_process *process) { return !process ? -EINVAL : process->pidfd < 0 ? -ENODATA : process->pidfd; }
int cc_process_exec_fd(const cc_process *process) { return !process ? -EINVAL : process->execfd < 0 ? -ENODATA : process->execfd; }
static int collect_exec(cc_process *process)
{
    if (process->execfd < 0) return process->result.exec_error;
    int error = 0; ssize_t size;
    do { size = read(process->execfd, &error, sizeof(error)); } while (size < 0 && errno == EINTR);
    if (size < 0 && errno == EAGAIN) return -EAGAIN;
    if (size == 0) error = 0;
    else if (size == sizeof(error)) error = error > 0 && error <= 4095 ? -error : -EPROTO;
    else error = size < 0 ? -errno : -EPROTO;
    close(process->execfd); process->execfd = -1;
    process->result.exec_complete = 1; process->result.exec_error = error;
    return error;
}
int cc_process_get_result(const cc_process *process, cc_process_result *out)
{
    if (!process || !out) return -EINVAL;
    *out = process->result; return 0;
}
int cc_process_signal(cc_process *process, int signal)
{
    if (!process || signal < 0 || signal >= NSIG) return -EINVAL;
    if (process->pidfd < 0) return -ESRCH;
#ifdef SYS_pidfd_send_signal
    return syscall(SYS_pidfd_send_signal, process->pidfd, signal, NULL, 0) < 0 ? -errno : 0;
#else
    return -ENOSYS;
#endif
}
int cc_process_dispatch(cc_process *process)
{
    if (!process) return -EINVAL;
    if (process->execfd >= 0) collect_exec(process);
    if (process->result.status != CC_PROCESS_RUNNING) return 0;
    siginfo_t info = {0}; int ret;
    do { ret = waitid((idtype_t)3 /* P_PIDFD */, (id_t)process->pidfd, &info, WEXITED | WNOHANG); } while (ret && errno == EINTR);
    if (ret) {
        if (errno != ECHILD) return -errno;
        struct pollfd fd = {.fd = process->pidfd, .events = POLLIN};
        ret = poll(&fd, 1, 0);
        if (ret < 0) return -errno;
        if (!ret) return -EAGAIN;
        if (fd.revents & POLLNVAL) return -EBADF;
        process->result.status = CC_PROCESS_EXIT_UNKNOWN;
    } else {
        if (!info.si_pid) return -EAGAIN;
        if (info.si_pid != process->result.pid) return -EPROTO;
        if (info.si_code == CLD_EXITED) { process->result.status = CC_PROCESS_EXITED; process->result.exit_code = info.si_status; }
        else {
            process->result.status = CC_PROCESS_SIGNALLED; process->result.signal = info.si_status;
            process->result.core_dumped = info.si_code == CLD_DUMPED;
        }
    }
    if (process->execfd >= 0) {
        collect_exec(process);
        if (process->execfd >= 0) {
            close(process->execfd); process->execfd = -1;
            process->result.exec_complete = 1; process->result.exec_error = -ECANCELED;
        }
    }
    close(process->pidfd); process->pidfd = -1; return 0;
}
int cc_process_wait(cc_process *process, cc_process_result *out, int timeout_ms)
{
    if (!process || !out || timeout_ms < -1) return -EINVAL;
    uint64_t deadline = 0;
    if (timeout_ms >= 0) { int ret = now_ms(&deadline); if (ret < 0) return ret; deadline += (uint32_t)timeout_ms; }
    for (;;) {
        int ret = cc_process_dispatch(process);
        if (!ret) { *out = process->result; return 0; }
        if (ret != -EAGAIN) return ret;
        int timeout; ret = remaining(deadline, timeout_ms < 0, &timeout); if (ret < 0) return ret;
        struct pollfd fd = {.fd = process->pidfd, .events = POLLIN};
        ret = poll(&fd, 1, timeout);
        if (ret < 0) { if (errno == EINTR) continue; return -errno; }
        if (!ret) return -ETIMEDOUT;
        if (fd.revents & POLLNVAL) return -EBADF;
    }
}
int cc_process_close(cc_process *process)
{
    if (!process) return 0;
    if (process->pidfd >= 0) {
        int ret = cc_process_signal(process, SIGKILL);
        if (ret < 0 && ret != -ESRCH) return ret;
        cc_process_result result;
        ret = cc_process_wait(process, &result, -1);
        if (ret < 0) return ret;
    }
    if (process->execfd >= 0) close(process->execfd);
    free(process); return 0;
}
int cc_process_wait_exec(cc_process *process, int timeout_ms)
{
    if (!process || timeout_ms < -1) return -EINVAL;
    uint64_t deadline = 0;
    if (timeout_ms >= 0) { int ret = now_ms(&deadline); if (ret < 0) return ret; deadline += (uint32_t)timeout_ms; }
    for (;;) {
        collect_exec(process);
        if (process->execfd < 0) return process->result.exec_error;
        int timeout; int ret = remaining(deadline, timeout_ms < 0, &timeout); if (ret < 0) return ret;
        struct pollfd fd = {.fd = process->execfd, .events = POLLIN};
        ret = poll(&fd, 1, timeout);
        if (ret < 0) { if (errno == EINTR) continue; return -errno; }
        if (!ret) return -ETIMEDOUT;
        if (fd.revents & POLLNVAL) return -EBADF;
    }
}
int cc_process_spawn_mode(const cc_process_options *options, uint32_t mode, cc_process **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if ((mode & ~(CC_PROCESS_TEST_PROC_FDS | CC_PROCESS_TEST_STOP_EXEC)) ||
        !options || !options->path || !options->path[0] ||
        options->num_arguments > CC_PROCESS_MAX_ARGUMENTS || (options->num_arguments && !options->arguments) ||
        options->num_fds > CC_PROCESS_MAX_FDS || (options->num_fds && !options->fds) ||
        (options->flags & ~(uint32_t)(CC_PROCESS_NEW_USER | CC_PROCESS_NEW_NET | CC_PROCESS_PARENT_DEATH))) return -EINVAL;
    for (size_t i = 0; i < options->num_fds; ++i) {
        if (options->fds[i] < 0) return -EBADF;
        if (fcntl(options->fds[i], F_GETFD) < 0) return -errno;
    }
    char *arguments[CC_PROCESS_MAX_ARGUMENTS + 2U] = {0};
    const char *values[CC_PROCESS_MAX_ARGUMENTS + 1U];
    size_t lengths[CC_PROCESS_MAX_ARGUMENTS + 1U], text = 0;
    for (size_t i = 0; i <= options->num_arguments; ++i) {
        const char *value = i ? options->arguments[i - 1] : options->path;
        if (!value) return -EINVAL;
        size_t size = strnlen(value, PROCESS_TEXT_LIMIT + 1U);
        if (size >= PROCESS_TEXT_LIMIT - text) return -E2BIG;
        values[i] = value; lengths[i] = size + 1; text += size + 1;
    }
    char *argument_text = malloc(text);
    if (!argument_text) return -ENOMEM;
    size_t offset = 0;
    for (size_t i = 0; i <= options->num_arguments; ++i) {
        arguments[i] = argument_text + offset;
        memcpy(arguments[i], values[i], lengths[i]); offset += lengths[i];
    }
    int ret = 0;
    cc_process *process = NULL; int notification[2] = {-1, -1};
    process = calloc(1, sizeof(*process)); if (!process) { ret = -ENOMEM; goto done; }
    process->pidfd = process->execfd = -1; process->result.exit_code = -1;
    ret = notify_pipe(notification); if (ret < 0) goto done;
    struct child_setup setup = {.arguments = arguments, .passed = options->fds, .num_passed = options->num_fds,
        .error_fd = notification[1], .parent_pid = getpid(), .flags = options->flags,
        .stop_exec = (mode & CC_PROCESS_TEST_STOP_EXEC) != 0};
    for (size_t i = 0; i < options->num_fds; ++i) setup.keep[setup.num_keep++] = options->fds[i];
    setup.keep[setup.num_keep++] = STDERR_FILENO; setup.keep[setup.num_keep++] = notification[1];
    qsort(setup.keep, setup.num_keep, sizeof(setup.keep[0]), compare_fds);
    size_t unique = 0;
    for (size_t i = 0; i < setup.num_keep; ++i)
        if (!unique || setup.keep[i] != setup.keep[unique - 1]) setup.keep[unique++] = setup.keep[i];
    setup.num_keep = unique;
#ifdef SYS_close_range
    setup.close_range = !(mode & CC_PROCESS_TEST_PROC_FDS) && syscall(SYS_close_range, UINT_MAX, 0, 0) < 0 && errno == EINVAL;
#endif
    struct clone_args clone = {.flags = CLONE_PIDFD, .pidfd = (uintptr_t)&process->pidfd, .exit_signal = SIGCHLD};
    if (options->flags & CC_PROCESS_NEW_USER) clone.flags |= CLONE_NEWUSER;
    if (options->flags & CC_PROCESS_NEW_NET) clone.flags |= CLONE_NEWNET;
    long pid = clone_child(&clone, &setup);
    if (pid < 0) { ret = -errno; process->pidfd = -1; goto done; }
    process->result.pid = (int)pid;
    close(notification[1]); notification[1] = -1;
    process->execfd = notification[0]; notification[0] = -1;
done:
    free(argument_text);
    for (size_t i = 0; i < 2; ++i) if (notification[i] >= 0) close(notification[i]);
    if (ret < 0) { cc_process_close(process); return ret; }
    *out = process; return 0;
}
int cc_process_spawn(const cc_process_options *options, cc_process **out) { return cc_process_spawn_mode(options, 0, out); }
