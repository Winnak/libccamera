/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "fence.h"
#include <errno.h>

enum { FDS = 32, ORIGINAL_BASE = 16, ORIGINAL_COUNT = 8 };
struct driver {
    bool open[FDS], fail_duplicate;
    uint8_t object[FDS];
    int error;
    struct cc_sync_file_info reply;
};
static int duplicate(void *context, int fd)
{
    struct driver *d = context;
    if (d->fail_duplicate) return -EMFILE;
    if (fd < 0 || fd >= FDS || !d->open[fd]) return -EBADF;
    for (int i = 0; i < ORIGINAL_BASE; ++i) if (!d->open[i]) {
        d->open[i] = true; d->object[i] = d->object[fd]; return i;
    }
    return -EMFILE;
}
static int close_fd(void *context, int fd)
{
    struct driver *d = context;
    FUZZ_CHECK(fd >= 0 && fd < ORIGINAL_BASE && d->open[fd]); d->open[fd] = false; return 0;
}
static int query(void *context, int fd, unsigned long request, void *argument)
{
    struct driver *d = context; struct cc_sync_file_info *info = argument;
    FUZZ_CHECK(fd >= 0 && fd < ORIGINAL_BASE && d->open[fd] && request == CC_SYNC_FILE_INFO);
    struct cc_sync_file_info empty = {0}; FUZZ_CHECK(!memcmp(info, &empty, sizeof(empty)));
    if (d->error) return d->error;
    *info = d->reply; return 0;
}
static const struct cc_io_ops ops = {.duplicate=duplicate,.close=close_fd,.ioctl=query};
static int query_result(const struct driver *d)
{
    if (d->error) return d->error == -ENOTTY ? -EINVAL : d->error < 0 ? d->error : -EPROTO;
    if (d->reply.flags || d->reply.pad || d->reply.sync_fence_info || d->reply.status > 1 || d->reply.status < -4095)
        return -EPROTO;
    return 0;
}
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size}; struct driver d = {0};
    const struct cc_io io = {&ops, &d}; struct cc_fence fences[3] = {{0}};
    for (int i = ORIGINAL_BASE; i < ORIGINAL_BASE + ORIGINAL_COUNT; ++i) { d.open[i] = true; d.object[i] = (uint8_t)(i - ORIGINAL_BASE); }
    const int errors[] = {0, -EIO, -ENOTTY, -EINVAL, -ENOMEM, 1};
    const int32_t statuses[] = {0, 1, -EIO, -ECANCELED, 2, INT32_MIN};
    for (size_t step = 0; step < 32; ++step) {
        unsigned int target = fuzz_u8(&in) % 3, operation = fuzz_u8(&in) % 4;
        uint8_t mode = fuzz_u8(&in);
        d.fail_duplicate = (mode & 1U) != 0;
        d.error = errors[fuzz_u8(&in) % 6];
        d.reply = (struct cc_sync_file_info){.status=statuses[fuzz_u8(&in) % 6],.num_fences=fuzz_u32(&in)};
        if (mode & 2U) fuzz_patch(&in, &d.reply, sizeof(d.reply));
        struct cc_fence before; memcpy(&before, &fences[target], sizeof(before));
        bool open_before[FDS]; memcpy(open_before, d.open, sizeof(open_before));
        int expected, ret;
        if (operation == 0) {
            int fd = mode & 4U ? (int)fuzz_u8(&in) - 2 : ORIGINAL_BASE + fuzz_u8(&in) % ORIGINAL_COUNT;
            expected = fd < -1 ? -EINVAL : fd == -1 ? 0 : d.fail_duplicate ? -EMFILE :
                fd >= FDS || !d.open[fd] ? -EBADF : query_result(&d);
            uint8_t object = fd >= 0 && fd < FDS ? d.object[fd] : 0;
            ret = cc_fence_set(&fences[target], fd, &io);
            FUZZ_CHECK(ret == expected);
            if (!ret) {
                FUZZ_CHECK(fences[target].valid == (fd >= 0));
                if (fd >= 0) FUZZ_CHECK(d.object[fences[target].fd] == object);
            }
        } else if (operation == 1) {
            expected = before.valid ? query_result(&d) : 0;
            if (before.valid && !expected) expected = d.reply.status;
            ret = cc_fence_update(&fences[target]); FUZZ_CHECK(ret == expected);
            if (ret <= 0) FUZZ_CHECK(!memcmp(&before, &fences[target], sizeof(before)));
            else FUZZ_CHECK(!fences[target].valid && !d.open[before.fd]);
        } else if (operation == 2) {
            ret = cc_fence_take(&fences[target]); FUZZ_CHECK(ret == (before.valid ? before.fd : -ENOENT));
            if (ret >= 0) { FUZZ_CHECK(d.open[ret] && !fences[target].valid); close_fd(&d, ret); }
        } else { cc_fence_clear(&fences[target]); ret = 0; FUZZ_CHECK(!fences[target].valid); }
        if (ret < 0) {
            FUZZ_CHECK(!memcmp(&before, &fences[target], sizeof(before)));
            FUZZ_CHECK(!memcmp(open_before, d.open, sizeof(open_before)));
        }
        for (int fd = 0; fd < FDS; ++fd) {
            bool owned = fd >= ORIGINAL_BASE && fd < ORIGINAL_BASE + ORIGINAL_COUNT;
            for (size_t i = 0; i < 3; ++i) if (fences[i].valid && fences[i].fd == fd) {
                FUZZ_CHECK(fd < ORIGINAL_BASE && !owned); owned = true;
            }
            FUZZ_CHECK(d.open[fd] == owned);
        }
    }
    for (size_t i = 0; i < 3; ++i) cc_fence_clear(&fences[i]);
    for (int fd = 0; fd < FDS; ++fd) FUZZ_CHECK(d.open[fd] == (fd >= ORIGINAL_BASE && fd < ORIGINAL_BASE + ORIGINAL_COUNT));
    return 0;
}
