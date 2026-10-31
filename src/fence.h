/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_FENCE_H
#define CCAMERA_FENCE_H
#include "internal.h"
#include <stddef.h>
#include <sys/ioctl.h>
/* Stable Linux sync_file UAPI, private so older build headers are sufficient.
 * Query only the aggregate header; never allocate an array of native fences. */
struct cc_sync_file_info {
    char name[32];
    int32_t status;
    uint32_t flags, num_fences, pad;
    uint64_t sync_fence_info;
};
#define CC_SYNC_FILE_INFO _IOWR('>', 4, struct cc_sync_file_info)
_Static_assert(sizeof(struct cc_sync_file_info) == 56, "sync_file UAPI size");
_Static_assert(offsetof(struct cc_sync_file_info, sync_fence_info) == 48, "sync_file UAPI pointer");
struct cc_fence {
    struct cc_io io;
    int fd;
    bool valid;
};
void cc_fence_clear(struct cc_fence *fence);
int cc_fence_set(struct cc_fence *fence, int fd, const struct cc_io *io);
int cc_fence_take(struct cc_fence *fence);
/* 1 ready (consumes fd), 0 pending/absent, negative error (retains fd). */
int cc_fence_update(struct cc_fence *fence);
#endif
