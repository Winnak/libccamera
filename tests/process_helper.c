/* SPDX-License-Identifier: CC0-1.0 */
#define _GNU_SOURCE
#include "process_report.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static uint64_t hash(const char *value)
{
    uint64_t result = UINT64_C(14695981039346656037);
    if (value) while (*value) { result ^= (unsigned char)*value++; result *= UINT64_C(1099511628211); }
    return result;
}
static int number(const char *value)
{
    char *end; errno = 0; long result = strtol(value, &end, 10);
    return errno || !value[0] || *end || result < 0 || result > INT_MAX ? -1 : (int)result;
}
static int report(int fd, int secret, const char *argument)
{
    struct process_report result = {.pid = getpid(), .parent_pid = getppid(),
        .passed_flags = fcntl(fd, F_GETFD), .secret_open = secret >= 0 && fcntl(secret, F_GETFD) >= 0,
        .argument_hash = hash(argument), .environment_hash = hash(getenv("CCAMERA_PROCESS_TEST_ENV"))};
    struct stat status, null;
    if (stat("/proc/self/ns/user", &status)) return 100;
    result.user_namespace = status.st_ino;
    if (stat("/proc/self/ns/net", &status)) return 101;
    result.net_namespace = status.st_ino;
    if (fstat(STDERR_FILENO, &status)) return 102;
    result.stderr_device = status.st_dev; result.stderr_inode = status.st_ino;
    if (stat("/dev/null", &null)) return 103;
    if (fstat(STDIN_FILENO, &status)) return 104;
    result.stdin_null = S_ISCHR(status.st_mode) && status.st_rdev == null.st_rdev &&
        (fcntl(STDIN_FILENO, F_GETFL) & O_ACCMODE) == O_RDONLY;
    if (fstat(STDOUT_FILENO, &status)) return 105;
    result.stdout_null = S_ISCHR(status.st_mode) && status.st_rdev == null.st_rdev &&
        (fcntl(STDOUT_FILENO, F_GETFL) & O_ACCMODE) == O_WRONLY;
    DIR *directory = opendir("/proc/self/fd"); if (!directory) return 106;
    struct dirent *entry;
    while ((entry = readdir(directory))) {
        int value = number(entry->d_name);
        if (value >= 3 && value != fd && value != dirfd(directory)) ++result.extra_fds;
    }
    closedir(directory);
    if (write(fd, &result, sizeof(result)) != sizeof(result)) return 107;
    return 23;
}
int main(int argc, char **argv)
{
    if (argc < 2) return 110;
    if (!strcmp(argv[1], "exit")) return argc == 3 ? number(argv[2]) : 111;
    if (!strcmp(argv[1], "wait")) { for (;;) pause(); }
    if (!strcmp(argv[1], "report")) return argc == 5 ? report(number(argv[2]), number(argv[3]), argv[4]) : 112;
    if (!strcmp(argv[1], "arguments")) return argc == 129 ? 0 : 113;
    return 114;
}
