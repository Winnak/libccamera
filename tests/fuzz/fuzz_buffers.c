/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "request_buffers.h"
#include <errno.h>

enum { ORIGINALS = 16, FDS = 64 };
struct driver {
    bool open[FDS];
    uint8_t object[FDS];
    int duplicate_count, identity_count, fail_duplicate, fail_identity;
};
static int duplicate(void *context, int fd)
{
    struct driver *d = context;
    if (++d->duplicate_count == d->fail_duplicate) return -EMFILE;
    if (fd < 0 || fd >= FDS || !d->open[fd]) return -EBADF;
    for (int i = ORIGINALS; i < FDS; ++i) if (!d->open[i]) {
        d->open[i] = true; d->object[i] = d->object[fd]; return i;
    }
    return -EMFILE;
}
static int close_fd(void *context, int fd)
{
    struct driver *d = context;
    FUZZ_CHECK(fd >= ORIGINALS && fd < FDS && d->open[fd]); d->open[fd] = false; return 0;
}
static int identity(void *context, int fd, uint64_t *device, uint64_t *inode)
{
    struct driver *d = context;
    if (fd < 0 || fd >= FDS || !d->open[fd]) return -EBADF;
    if (++d->identity_count == d->fail_identity) return -EIO;
    *device = 1; *inode = d->object[fd]; return 0;
}
static const struct cc_io_ops ops = {.duplicate=duplicate,.close=close_fd,.identity=identity};
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size}; struct driver d = {0};
    for (int i = 0; i < ORIGINALS; ++i) { d.open[i] = true; d.object[i] = (uint8_t)(fuzz_u8(&in) % 8); }
    const struct cc_io io = {&ops, &d};
    struct cc_request_buffer buffers[2] = {{0}};
    for (unsigned int step = 0; step < 16; ++step) {
        unsigned int target = fuzz_u8(&in) % 2;
        uint8_t flags = fuzz_u8(&in); uint32_t count = fuzz_u8(&in) % (CC_MAX_PLANES + 2U);
        d.fail_duplicate = flags & 1U ? d.duplicate_count + 1 + fuzz_u8(&in) % 8 : 0;
        d.fail_identity = flags & 2U ? d.identity_count + 1 + fuzz_u8(&in) % 8 : 0;
        cc_dmabuf_plane planes[CC_MAX_PLANES];
        for (uint32_t p = 0; p < CC_MAX_PLANES; ++p) {
            planes[p].fd = flags & 4U ? (int)fuzz_u8(&in) - 1 : fuzz_u8(&in) % ORIGINALS;
            planes[p].length = fuzz_u32(&in);
        }
        struct cc_request_buffer previous = buffers[target];
        bool open_before[FDS]; memcpy(open_before, d.open, sizeof(open_before));
        uint8_t input_objects[CC_MAX_PLANES] = {0}; bool input_valid = count <= CC_MAX_PLANES;
        for (uint32_t p = 0; p < count && p < CC_MAX_PLANES; ++p) {
            if (planes[p].fd < 0 || planes[p].fd >= FDS || !d.open[planes[p].fd]) input_valid = false;
            else input_objects[p] = d.object[planes[p].fd];
        }
        int ret = cc_request_buffer_copy(&buffers[target], flags & 8U ? NULL : planes, count, &io);
        if (ret) {
            FUZZ_CHECK(!memcmp(&previous, &buffers[target], sizeof(previous)));
            FUZZ_CHECK(!memcmp(open_before, d.open, sizeof(open_before)));
        }
        else {
            FUZZ_CHECK(buffers[target].count == count && count <= CC_MAX_PLANES && input_valid);
            for (uint32_t p = 0; p < count; ++p) {
                int fd = buffers[target].planes[p].fd;
                FUZZ_CHECK(d.open[fd] && fd >= ORIGINALS && buffers[target].planes[p].length == planes[p].length);
                FUZZ_CHECK(d.object[fd] == input_objects[p]);
                for (uint32_t q = 0; q < p; ++q) FUZZ_CHECK(d.object[fd] != d.object[buffers[target].planes[q].fd]);
            }
        }
        bool expected = false;
        for (uint32_t p = 0; p < buffers[0].count; ++p) for (uint32_t q = 0; q < buffers[1].count; ++q)
            expected |= d.object[buffers[0].planes[p].fd] == d.object[buffers[1].planes[q].fd];
        FUZZ_CHECK(cc_request_buffer_overlaps(&buffers[0], &buffers[1]) == expected);
        cc_format format = {.num_planes=1U + fuzz_u8(&in) % CC_MAX_PLANES};
        for (uint32_t p = 0; p < format.num_planes; ++p) format.planes[p].size = fuzz_u32(&in);
        int expected_ret = buffers[target].count == format.num_planes ? 0 : -EINVAL;
        if (!expected_ret) for (uint32_t p = 0; p < format.num_planes; ++p)
            if (buffers[target].planes[p].length < format.planes[p].size) expected_ret = -ENOSPC;
        FUZZ_CHECK(cc_request_buffer_validate(&buffers[target], &format) == expected_ret);
        if (flags & 16U) cc_request_buffer_clear(&buffers[target]);
    }
    cc_request_buffer_clear(&buffers[0]); cc_request_buffer_clear(&buffers[1]);
    for (int i = 0; i < FDS; ++i) FUZZ_CHECK(d.open[i] == (i < ORIGINALS));
    return 0;
}
