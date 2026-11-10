/* SPDX-License-Identifier: CC0-1.0 */
#ifndef CCAMERA_TEST_PROCESS_REPORT_H
#define CCAMERA_TEST_PROCESS_REPORT_H
#include <stdint.h>
struct process_report {
    uint64_t user_namespace, net_namespace, argument_hash, environment_hash;
    uint64_t stderr_device, stderr_inode;
    int32_t pid, parent_pid, passed_flags, secret_open, extra_fds, stdin_null, stdout_null;
};
#endif
