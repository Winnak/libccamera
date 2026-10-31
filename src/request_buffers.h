/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_REQUEST_BUFFERS_H
#define CCAMERA_REQUEST_BUFFERS_H
#include "internal.h"
struct cc_buffer_identity { uint64_t device, inode; };
struct cc_request_buffer {
    struct cc_io io;
    uint32_t count;
    cc_dmabuf_plane planes[CC_MAX_PLANES];
    struct cc_buffer_identity identities[CC_MAX_PLANES];
};
void cc_request_buffer_clear(struct cc_request_buffer *buffer);
int cc_request_buffer_copy(struct cc_request_buffer *buffer, const cc_dmabuf_plane *planes,
                            uint32_t count, const struct cc_io *io);
bool cc_request_buffer_overlaps(const struct cc_request_buffer *a, const struct cc_request_buffer *b);
int cc_request_buffer_validate(const struct cc_request_buffer *buffer, const cc_format *format);
#endif
