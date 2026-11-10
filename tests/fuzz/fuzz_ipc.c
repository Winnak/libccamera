/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "ipc.h"
#include <errno.h>

static void header(const void *data, size_t size)
{
    uint32_t length = UINT32_MAX, descriptors = UINT32_MAX;
    int ret = cc_ipc_decode_header(data, size, &length, &descriptors);
    if (ret < 0) FUZZ_CHECK(length == UINT32_MAX && descriptors == UINT32_MAX);
    else FUZZ_CHECK(size == 8 && length <= CC_IPC_MAX_DATA && descriptors <= CC_IPC_MAX_FDS && (length || descriptors));
}
static void rights(const void *data, size_t size)
{
    struct { int descriptors[CC_IPC_RIGHTS_CAPACITY]; uint64_t guard; } output;
    memset(&output, 0xa5, sizeof(output)); output.guard = UINT64_C(0x123456789abcdef0);
    uint32_t count = UINT32_MAX;
    int ret = cc_ipc_decode_rights(data, size, output.descriptors, &count);
    FUZZ_CHECK(count <= CC_IPC_RIGHTS_CAPACITY && output.guard == UINT64_C(0x123456789abcdef0));
    if (!ret) for (uint32_t i = 0; i < count; ++i) FUZZ_CHECK(output.descriptors[i] >= 0);
}
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    header(data, size); rights(data, size);
    struct fuzz_input in = {data, size};
    unsigned char unaligned[CC_IPC_CONTROL_SIZE + 1], *control = unaligned + 1;
    memset(control, 0, CC_IPC_CONTROL_SIZE);
    uint8_t mode = fuzz_u8(&in); uint32_t length = fuzz_u32(&in);
    unsigned char wire[8] = {0}; memcpy(wire, &length, 4); wire[4] = fuzz_u8(&in);
    wire[5] = fuzz_u8(&in); wire[6] = fuzz_u8(&in); wire[7] = fuzz_u8(&in); header(wire, sizeof(wire));
    struct cmsghdr record = {.cmsg_level = SOL_SOCKET, .cmsg_type = SCM_RIGHTS};
    size_t count = fuzz_u16(&in) % (CC_IPC_RIGHTS_CAPACITY + 1U);
    record.cmsg_len = CMSG_LEN(count * sizeof(int)); memcpy(control, &record, sizeof(record));
    for (size_t i = 0; i < count; ++i) {
        int fd = (int)(fuzz_u32(&in) & INT32_MAX);
        memcpy(control + CMSG_LEN(0) + i * sizeof(fd), &fd, sizeof(fd));
    }
    size_t bytes = CMSG_SPACE(count * sizeof(int));
    if (mode & 1U) fuzz_patch(&in, control, CC_IPC_CONTROL_SIZE);
    if (mode & 2U) bytes = fuzz_u16(&in) % (CC_IPC_CONTROL_SIZE + 1U);
    rights(control, bytes);
    /* No descriptor values are used as OS handles by this pure-parser target. */
    return 0;
}
