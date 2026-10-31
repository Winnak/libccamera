/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "request_buffers.h"
#include <errno.h>

void cc_request_buffer_clear(struct cc_request_buffer *buffer)
{
    for (uint32_t i = 0; i < buffer->count; ++i)
        buffer->io.ops->close(buffer->io.ctx, buffer->planes[i].fd);
    *buffer = (struct cc_request_buffer){0};
}
bool cc_request_buffer_overlaps(const struct cc_request_buffer *a, const struct cc_request_buffer *b)
{
    for (uint32_t i = 0; i < a->count; ++i) for (uint32_t j = 0; j < b->count; ++j)
        if (a->identities[i].device == b->identities[j].device && a->identities[i].inode == b->identities[j].inode)
            return true;
    return false;
}
int cc_request_buffer_copy(struct cc_request_buffer *buffer, const cc_dmabuf_plane *planes,
                            uint32_t count, const struct cc_io *io)
{
    if (count > CC_MAX_PLANES || (count && !planes) || (!count && planes)) return -EINVAL;
    if (!io || !io->ops || !io->ops->duplicate || !io->ops->close || !io->ops->identity) return -ENOTSUP;
    struct cc_request_buffer copy = {.io = *io};
    for (uint32_t i = 0; i < count; ++i) if (planes[i].fd < 0 || !planes[i].length) return -EINVAL;
    /* Inspect every borrowed fd before allocating duplicates: a closed input
     * number must not become accidentally valid when an earlier dup reuses it. */
    for (uint32_t i = 0; i < count; ++i) {
        int ret = io->ops->identity(io->ctx, planes[i].fd, &copy.identities[i].device, &copy.identities[i].inode);
        if (ret < 0) return ret;
        for (uint32_t j = 0; j < i; ++j)
            if (copy.identities[j].device == copy.identities[i].device && copy.identities[j].inode == copy.identities[i].inode)
                return -EEXIST;
    }
    for (uint32_t i = 0; i < count; ++i) {
        int ret = io->ops->duplicate(io->ctx, planes[i].fd);
        if (ret < 0) { cc_request_buffer_clear(&copy); return ret; }
        copy.planes[i] = (cc_dmabuf_plane){ret, planes[i].length}; ++copy.count;
        struct cc_buffer_identity actual;
        ret = io->ops->identity(io->ctx, copy.planes[i].fd, &actual.device, &actual.inode);
        if (ret < 0) { cc_request_buffer_clear(&copy); return ret; }
        if (actual.device != copy.identities[i].device || actual.inode != copy.identities[i].inode) {
            cc_request_buffer_clear(&copy); return -ESTALE;
        }
    }
    cc_request_buffer_clear(buffer); *buffer = copy; return 0;
}
int cc_request_buffer_validate(const struct cc_request_buffer *buffer, const cc_format *format)
{
    if (buffer->count != format->num_planes) return -EINVAL;
    for (uint32_t i = 0; i < buffer->count; ++i)
        if (buffer->planes[i].length < format->planes[i].size) return -ENOSPC;
    return 0;
}
