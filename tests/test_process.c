/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "process.h"
#include "process_report.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
static const char *helper;
static size_t descriptors(void)
{
    DIR *directory = opendir("/proc/self/fd"); CHECK(directory); struct dirent *entry; size_t n = 0;
    while ((entry = readdir(directory))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++n;
    OK(closedir(directory)); return n;
}
static uint64_t hash(const char *value)
{
    uint64_t result = UINT64_C(14695981039346656037);
    while (*value) { result ^= (unsigned char)*value++; result *= UINT64_C(1099511628211); } return result;
}
static uint64_t namespace_inode(const char *name)
{
    struct stat status; CHECK(!stat(name, &status)); return status.st_ino;
}
static int available(uint32_t flags)
{
    const char *arguments[] = {"exit", "0"}; cc_process_options options = {.path = helper,
        .arguments = arguments, .num_arguments = 2, .flags = flags}; cc_process *process = NULL;
    int ret = cc_process_spawn(&options, &process);
    if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) {
        fprintf(stderr, "Kernel process/namespace support unavailable: %s\n", strerror(-ret)); return 77;
    }
    CHECK(!ret && process); OK(cc_process_wait_exec(process, 2000)); cc_process_result result;
    OK(cc_process_wait(process, &result, 2000)); CHECK(result.status == CC_PROCESS_EXITED && !result.exit_code);
    OK(cc_process_close(process)); return 0;
}
static void report_case(bool fallback, uint32_t flags, bool high_secret)
{
    size_t before = descriptors(); int pipes[2]; OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK));
    int secret = open("/dev/null", O_RDONLY); CHECK(secret >= 0);
    if (high_secret) { int high = fcntl(secret, F_DUPFD, 512); CHECK(high >= 512); OK(close(secret)); secret = high; }
    char output[24], hidden[24]; CHECK(snprintf(output, sizeof(output), "%d", pipes[1]) > 0);
    CHECK(snprintf(hidden, sizeof(hidden), "%d", secret) > 0);
    const char text[] = "argument with spaces, quotes: '\" and utf8: \xc3\xb8";
    const char *arguments[] = {"report", output, hidden, text}; int passed[] = {pipes[1], pipes[1]};
    cc_process_options options = {.path = helper, .arguments = arguments, .num_arguments = 4,
        .fds = passed, .num_fds = 2, .flags = flags};
    struct rlimit original;
    if (high_secret) { OK(getrlimit(RLIMIT_NOFILE, &original)); struct rlimit low = original; low.rlim_cur = 64; OK(setrlimit(RLIMIT_NOFILE, &low)); }
    cc_process *process = NULL; OK(cc_process_spawn_mode(&options, fallback ? CC_PROCESS_TEST_PROC_FDS : 0, &process));
    if (high_secret) OK(setrlimit(RLIMIT_NOFILE, &original));
    CHECK(cc_process_pid(process) > 0); CHECK(fcntl(cc_process_fd(process), F_GETFD) & FD_CLOEXEC);
    CHECK(fcntl(pipes[1], F_GETFD) & FD_CLOEXEC); CHECK(fcntl(secret, F_GETFD) >= 0);
    OK(cc_process_wait_exec(process, 2000)); ERROR(cc_process_exec_fd(process), ENODATA);
    OK(close(pipes[1])); cc_process_result result; OK(cc_process_wait(process, &result, 2000));
    CHECK(result.status == CC_PROCESS_EXITED && result.exit_code == 23 && !result.signal && result.exec_complete && !result.exec_error);
    struct process_report report; CHECK(read(pipes[0], &report, sizeof(report)) == sizeof(report));
    CHECK(report.pid == cc_process_pid(process) && report.parent_pid == getpid());
    CHECK(report.passed_flags == 0 && !report.secret_open && !report.extra_fds && report.stdin_null && report.stdout_null);
    CHECK(report.argument_hash == hash(text) && report.environment_hash == hash("inherited-value"));
    struct stat stderr_status; OK(fstat(STDERR_FILENO, &stderr_status));
    CHECK(report.stderr_device == stderr_status.st_dev && report.stderr_inode == stderr_status.st_ino);
    CHECK((report.user_namespace != namespace_inode("/proc/self/ns/user")) == ((flags & CC_PROCESS_NEW_USER) != 0));
    CHECK((report.net_namespace != namespace_inode("/proc/self/ns/net")) == ((flags & CC_PROCESS_NEW_NET) != 0));
    ERROR(cc_process_fd(process), ENODATA); ERROR(cc_process_signal(process, SIGTERM), ESRCH);
    OK(cc_process_dispatch(process)); OK(cc_process_wait(process, &result, 0)); OK(cc_process_close(process));
    OK(close(secret)); OK(close(pipes[0])); CHECK(descriptors() == before);
}
static void waits_and_signals(void)
{
    const char *arguments[] = {"wait"}; cc_process_options options = {.path = helper, .arguments = arguments,
        .num_arguments = 1, .flags = CC_PROCESS_PARENT_DEATH};
    cc_process *process = NULL; OK(cc_process_spawn(&options, &process)); OK(cc_process_wait_exec(process, 2000));
    cc_process_result result, original; memset(&result, 0xa5, sizeof(result)); original = result;
    ERROR(cc_process_wait(process, &result, 0), ETIMEDOUT); CHECK(!memcmp(&result, &original, sizeof(result)));
    ERROR(cc_process_wait(process, &result, 2), ETIMEDOUT); CHECK(!memcmp(&result, &original, sizeof(result)));
    ERROR(cc_process_dispatch(process), EAGAIN); OK(cc_process_get_result(process, &result)); CHECK(result.status == CC_PROCESS_RUNNING);
    OK(cc_process_signal(process, 0)); OK(cc_process_signal(process, SIGTERM));
    OK(cc_process_wait(process, &result, 2000)); CHECK(result.status == CC_PROCESS_SIGNALLED && result.signal == SIGTERM && result.exit_code == -1);
    OK(cc_process_close(process));
    /* Closing one process must not signal/reap an unrelated live child. */
    cc_process *a = NULL, *b = NULL; OK(cc_process_spawn(&options, &a)); OK(cc_process_spawn(&options, &b));
    OK(cc_process_wait_exec(a, 2000)); OK(cc_process_wait_exec(b, 2000)); OK(cc_process_close(a));
    OK(cc_process_signal(b, 0)); ERROR(cc_process_dispatch(b), EAGAIN); OK(cc_process_close(b));
}
static void exec_deadline(void)
{
    const char *arguments[] = {"exit", "0"}; cc_process_options options = {.path = helper,
        .arguments = arguments, .num_arguments = 2, .flags = CC_PROCESS_PARENT_DEATH};
    cc_process *process = NULL;
    OK(cc_process_spawn_mode(&options, CC_PROCESS_TEST_STOP_EXEC, &process));
    siginfo_t info;
    OK(waitid(P_PID, (id_t)cc_process_pid(process), &info, WSTOPPED));
    CHECK(info.si_code == CLD_STOPPED && info.si_status == SIGSTOP);
    ERROR(cc_process_wait_exec(process, 0), ETIMEDOUT);
    ERROR(cc_process_wait_exec(process, 2), ETIMEDOUT);
    cc_process_result result; OK(cc_process_get_result(process, &result));
    CHECK(result.status == CC_PROCESS_RUNNING && !result.exec_complete);
    OK(cc_process_signal(process, SIGCONT)); OK(cc_process_wait_exec(process, 2000));
    OK(cc_process_wait(process, &result, 2000));
    CHECK(result.status == CC_PROCESS_EXITED && !result.exit_code); OK(cc_process_close(process));
    OK(cc_process_spawn_mode(&options, CC_PROCESS_TEST_STOP_EXEC, &process));
    OK(waitid(P_PID, (id_t)cc_process_pid(process), &info, WSTOPPED));
    OK(cc_process_close(process));
}
static void exec_errors(void)
{
    cc_process_options options = {.path = "/ccamera-no-such-program"}; cc_process *process = NULL;
    OK(cc_process_spawn(&options, &process)); ERROR(cc_process_wait_exec(process, 2000), ENOENT);
    cc_process_result result; OK(cc_process_wait(process, &result, 2000));
    CHECK(result.exec_complete && result.exec_error == -ENOENT && result.status == CC_PROCESS_EXITED && result.exit_code == 127);
    OK(cc_process_close(process));
    char file[] = "/tmp/ccamera-exec-XXXXXX"; int fd = mkstemp(file); CHECK(fd >= 0);
    CHECK(write(fd, "not an executable\n", 18) == 18); OK(close(fd)); options.path = file;
    OK(cc_process_spawn(&options, &process)); ERROR(cc_process_wait_exec(process, 2000), EACCES); OK(cc_process_close(process));
    OK(chmod(file, 0700)); OK(cc_process_spawn(&options, &process)); ERROR(cc_process_wait_exec(process, 2000), ENOEXEC);
    OK(cc_process_close(process)); OK(unlink(file));
}
static void reaped_elsewhere(void)
{
    const char *arguments[] = {"exit", "7"}; cc_process_options options = {.path = helper, .arguments = arguments, .num_arguments = 2};
    cc_process *process = NULL; OK(cc_process_spawn(&options, &process)); OK(cc_process_wait_exec(process, 2000));
    int status; CHECK(waitpid(cc_process_pid(process), &status, 0) == cc_process_pid(process));
    cc_process_result result; OK(cc_process_wait(process, &result, 2000)); CHECK(result.status == CC_PROCESS_EXIT_UNKNOWN);
    OK(cc_process_close(process));
    struct sigaction old, ignore = {.sa_handler = SIG_IGN}; sigemptyset(&ignore.sa_mask); OK(sigaction(SIGCHLD, &ignore, &old));
    OK(cc_process_spawn(&options, &process)); OK(cc_process_wait_exec(process, 2000));
    OK(cc_process_wait(process, &result, 2000)); CHECK(result.status == CC_PROCESS_EXIT_UNKNOWN); OK(cc_process_close(process));
    OK(sigaction(SIGCHLD, &old, NULL));
}
static void parent_death(void)
{
    int subreaper = 0; OK(prctl(PR_GET_CHILD_SUBREAPER, &subreaper)); OK(prctl(PR_SET_CHILD_SUBREAPER, 1));
    int pipes[2]; OK(pipe2(pipes, O_CLOEXEC)); pid_t launcher = fork(); CHECK(launcher >= 0);
    if (!launcher) {
        close(pipes[0]); const char *arguments[] = {"wait"};
        cc_process_options options = {.path = helper, .arguments = arguments, .num_arguments = 1, .flags = CC_PROCESS_PARENT_DEATH};
        cc_process *process = NULL; OK(cc_process_spawn(&options, &process)); OK(cc_process_wait_exec(process, 2000));
        int pid = cc_process_pid(process); CHECK(write(pipes[1], &pid, sizeof(pid)) == sizeof(pid)); _exit(0);
    }
    OK(close(pipes[1])); int pid; CHECK(read(pipes[0], &pid, sizeof(pid)) == sizeof(pid)); OK(close(pipes[0]));
    int status; CHECK(waitpid(launcher, &status, 0) == launcher && WIFEXITED(status) && !WEXITSTATUS(status));
    CHECK(waitpid(pid, &status, 0) == pid && WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL);
    OK(prctl(PR_SET_CHILD_SUBREAPER, subreaper));
}
static atomic_bool stop_pressure;
static void *pressure(void *unused)
{
    (void)unused;
    while (!atomic_load(&stop_pressure)) {
        void *memory = malloc(8192); CHECK(memory); memset(memory, 0x5a, 8192);
        atomic_signal_fence(memory_order_seq_cst); free(memory);
    }
    return NULL;
}
static void threaded(void)
{
    pthread_t thread; OK(pthread_create(&thread, NULL, pressure, NULL));
    const char *arguments[] = {"exit", "0"}; cc_process_options options = {.path = helper, .arguments = arguments, .num_arguments = 2};
    for (unsigned i = 0; i < 20; ++i) {
        cc_process *process = NULL; OK(cc_process_spawn(&options, &process)); OK(cc_process_wait_exec(process, 2000));
        cc_process_result result; OK(cc_process_wait(process, &result, 2000)); CHECK(result.status == CC_PROCESS_EXITED && !result.exit_code);
        OK(cc_process_close(process));
    }
    atomic_store(&stop_pressure, true); OK(pthread_join(thread, NULL));
}
static void fd_zero(void)
{
    int saved = dup(STDIN_FILENO); (void)close(STDIN_FILENO);
    const char *arguments[] = {"exit", "0"}; cc_process_options options = {.path = helper, .arguments = arguments, .num_arguments = 2};
    cc_process *process = NULL; OK(cc_process_spawn(&options, &process)); CHECK(cc_process_fd(process) == 0);
    OK(cc_process_wait_exec(process, 2000)); cc_process_result result; OK(cc_process_wait(process, &result, 2000));
    CHECK(result.status == CC_PROCESS_EXITED && !result.exit_code); OK(cc_process_close(process));
    CHECK(fcntl(0, F_GETFD) < 0 && errno == EBADF);
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
}
static void arguments(void)
{
    cc_process *process = (cc_process *)(uintptr_t)1; cc_process_options options = {0};
    ERROR(cc_process_spawn(NULL, &process), EINVAL); CHECK(!process); ERROR(cc_process_spawn(&options, &process), EINVAL);
    options.path = helper; options.num_arguments = CC_PROCESS_MAX_ARGUMENTS + 1; ERROR(cc_process_spawn(&options, &process), EINVAL);
    options.num_arguments = 1; ERROR(cc_process_spawn(&options, &process), EINVAL); options.num_arguments = 0;
    options.num_fds = CC_PROCESS_MAX_FDS + 1; ERROR(cc_process_spawn(&options, &process), EINVAL); options.num_fds = 1;
    int bad = -1; options.fds = &bad; ERROR(cc_process_spawn(&options, &process), EBADF); options.num_fds = 0;
    options.flags = UINT32_MAX; ERROR(cc_process_spawn(&options, &process), EINVAL); options.flags = 0;
    const char *bad_arguments[] = {NULL}; options.arguments = bad_arguments; options.num_arguments = 1;
    ERROR(cc_process_spawn(&options, &process), EINVAL);
    char *long_text = malloc(1024 * 1024 + 2); CHECK(long_text); memset(long_text, 'x', 1024 * 1024 + 1); long_text[1024 * 1024 + 1] = 0;
    bad_arguments[0] = long_text; ERROR(cc_process_spawn(&options, &process), E2BIG); free(long_text);
    const char *many[CC_PROCESS_MAX_ARGUMENTS]; for (size_t i = 0; i < CC_PROCESS_MAX_ARGUMENTS; ++i) many[i] = i ? "x" : "arguments";
    options.arguments = many; options.num_arguments = CC_PROCESS_MAX_ARGUMENTS;
    OK(cc_process_spawn(&options, &process)); OK(cc_process_wait_exec(process, 2000)); cc_process_result result;
    OK(cc_process_wait(process, &result, 2000)); CHECK(result.status == CC_PROCESS_EXITED && !result.exit_code); OK(cc_process_close(process));
    OK(cc_process_close(NULL)); ERROR(cc_process_fd(NULL), EINVAL); ERROR(cc_process_pid(NULL), EINVAL);
    ERROR(cc_process_exec_fd(NULL), EINVAL); ERROR(cc_process_wait_exec(NULL, 0), EINVAL);
    ERROR(cc_process_dispatch(NULL), EINVAL); ERROR(cc_process_signal(NULL, 0), EINVAL);
    ERROR(cc_process_get_result(NULL, &result), EINVAL); ERROR(cc_process_wait(NULL, &result, 0), EINVAL);
}
int main(int argc, char **argv)
{
    CHECK(argc == 2 || argc == 3); helper = argv[1]; size_t before = descriptors();
    uint32_t flags = argc == 3 ? CC_PROCESS_NEW_USER | CC_PROCESS_NEW_NET : 0;
    int ret = available(flags); if (ret) return ret;
    OK(setenv("CCAMERA_PROCESS_TEST_ENV", "inherited-value", 1));
    if (argc == 3) { report_case(false, flags, false); report_case(true, flags, false); }
    else {
        arguments(); report_case(false, 0, false); report_case(true, 0, true);
        waits_and_signals(); exec_deadline(); exec_errors(); reaped_elsewhere(); fd_zero(); threaded(); parent_death();
    }
    CHECK(descriptors() == before); int status; CHECK(waitpid(-1, &status, WNOHANG) < 0 && errno == ECHILD);
    puts("Process exec, pidfd supervision, descriptor allowlist, namespace and reaping checks passed");
    return 0;
}
