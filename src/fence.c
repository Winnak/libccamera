/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fence.h"
#include <errno.h>

static int query(const struct cc_fence *fence, int32_t *status)
{
    struct cc_sync_file_info info = {0};
    int ret = cc_ioctl(&fence->io, fence->fd, CC_SYNC_FILE_INFO, &info);
    if (ret < 0) return ret == -ENOTTY ? -EINVAL : ret;
    if (ret || info.flags || info.pad || info.sync_fence_info || info.status > 1 || info.status < -4095)
        return -EPROTO;
    *status = info.status;
    return 0;
}
void cc_fence_clear(struct cc_fence *fence)
{
    if (fence->valid) fence->io.ops->close(fence->io.ctx, fence->fd);
    *fence = (struct cc_fence){0};
}
int cc_fence_set(struct cc_fence *fence, int fd, const struct cc_io *io)
{
    if (fd < -1) return -EINVAL;
    if (fd == -1) { cc_fence_clear(fence); return 0; }
    if (!io || !io->ops || !io->ops->duplicate || !io->ops->close || !io->ops->ioctl) return -ENOTSUP;
    int duplicate = io->ops->duplicate(io->ctx, fd);
    if (duplicate < 0) return duplicate;
    struct cc_fence copy = {.io=*io,.fd=duplicate,.valid=true};
    int32_t status;
    int ret = query(&copy, &status);
    if (ret < 0) { cc_fence_clear(&copy); return ret; }
    cc_fence_clear(fence); *fence = copy;
    return 0;
}
int cc_fence_take(struct cc_fence *fence)
{
    if (!fence->valid) return -ENOENT;
    int fd = fence->fd; *fence = (struct cc_fence){0}; return fd;
}
int cc_fence_update(struct cc_fence *fence)
{
    if (!fence->valid) return 0;
    int32_t status;
    int ret = query(fence, &status);
    if (ret < 0) return ret;
    if (status <= 0) return status;
    cc_fence_clear(fence); return 1;
}
