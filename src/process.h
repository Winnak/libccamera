/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PROCESS_INTERNAL_H
#define CCAMERA_PROCESS_INTERNAL_H
#include <ccamera/process.h>
#include <stdbool.h>
/* Private fault-injection modes for the native child setup. */
#define CC_PROCESS_TEST_PROC_FDS 1U
#define CC_PROCESS_TEST_STOP_EXEC 2U
int cc_process_spawn_mode(const cc_process_options *options, uint32_t mode, cc_process **process);
#endif
