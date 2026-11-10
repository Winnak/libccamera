/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_message.h"
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define MESSAGE_VERSION 1U
#define MESSAGE_SEALS (F_SEAL_SEAL | F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_WRITE)
struct envelope { uint32_t version, storage, size; };
_Static_assert(sizeof(struct envelope) == 12, "IPA message envelope layout");
_Static_assert(CC_IPA_WIRE_MAX_FDS + 1U <= CC_IPC_MAX_FDS, "Snapshot plus native control descriptors fit transport");
static int sealed_snapshot(const void *data, size_t size)
{
    int fd = memfd_create("ccamera-ipa", MFD_CLOEXEC | MFD_ALLOW_SEALING);
    if (fd < 0) return -errno;
    size_t offset = 0; int error = 0;
    while (offset < size) {
        ssize_t n = write(fd, (const unsigned char *)data + offset, size - offset);
        if (n < 0) { if (errno == EINTR) continue; error = -errno; break; }
        if (!n) { error = -EIO; break; }
        offset += (size_t)n;
    }
    if (!error && fcntl(fd, F_ADD_SEALS, MESSAGE_SEALS)) error = -errno;
    if (error) { close(fd); return error; }
    return fd;
}
static int prepare(const void *data, size_t size, const int *handles, uint32_t num_handles,
                     int fds[CC_IPA_WIRE_MAX_FDS + 1U], cc_ipc_payload *payload, void **storage, int *fd)
{
    *storage = NULL; *fd = -1;
    if (size && !data) return -EINVAL;
    if (num_handles > CC_IPA_WIRE_MAX_FDS || (num_handles && !handles)) return -EINVAL;
    if (size > CC_IPA_WIRE_MAX_DATA) return -E2BIG;
    size_t inline_limit = CC_IPC_MAX_DATA - sizeof(cc_ipc_message_header) - sizeof(struct envelope);
    bool large = size > inline_limit;
    struct envelope header = {MESSAGE_VERSION, (num_handles << 1U) | (large ? 1U : 0U), (uint32_t)size};
    size_t length = sizeof(header) + (large ? 0 : size);
    unsigned char *bytes = malloc(length); if (!bytes) return -ENOMEM;
    memcpy(bytes, &header, sizeof(header));
    if (large) {
        int handle = sealed_snapshot(data, size);
        if (handle < 0) { free(bytes); return handle; }
        *fd = handle;
        fds[0] = handle;
    } else if (size) memcpy(bytes + sizeof(header), data, size);
    *storage = bytes;
    if (num_handles) memcpy(fds + (large ? 1U : 0U), handles, num_handles * sizeof(*handles));
    *payload = (cc_ipc_payload){.data = bytes, .size = length, .fds = fds, .num_fds = num_handles + (large ? 1U : 0U)};
    return 0;
}
int cc_ipa_message_send_fds(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size,
                            const int *handles, uint32_t num_handles)
{
    if (!pipe || !header) return -EINVAL;
    cc_ipc_payload payload; void *storage; int fd, fds[CC_IPA_WIRE_MAX_FDS + 1U];
    int ret = prepare(data, size, handles, num_handles, fds, &payload, &storage, &fd); if (ret < 0) return ret;
    ret = cc_ipc_pipe_send(pipe, header, &payload);
    free(storage); if (fd >= 0) close(fd); return ret;
}
int cc_ipa_message_send(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size)
{
    return cc_ipa_message_send_fds(pipe, header, data, size, NULL, 0);
}
int cc_ipa_message_call_fds(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size,
                            const int *handles, uint32_t num_handles, cc_ipc_packet **response, int timeout_ms)
{
    if (response) *response = NULL;
    if (!pipe || !header || timeout_ms < -1) return -EINVAL;
    cc_ipc_payload payload; void *storage; int fd, fds[CC_IPA_WIRE_MAX_FDS + 1U];
    int ret = prepare(data, size, handles, num_handles, fds, &payload, &storage, &fd); if (ret < 0) return ret;
    ret = cc_ipc_pipe_call(pipe, header, &payload, response, timeout_ms);
    free(storage); if (fd >= 0) close(fd); return ret;
}
int cc_ipa_message_call(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size,
                         cc_ipc_packet **response, int timeout_ms)
{
    return cc_ipa_message_call_fds(pipe, header, data, size, NULL, 0, response, timeout_ms);
}
int cc_ipa_message_decode_fds(const cc_ipc_packet *packet, void **out, size_t *out_size, const int **handles, uint32_t *num_handles)
{
    if (out) *out = NULL;
    if (out_size) *out_size = 0;
    if (handles) *handles = NULL;
    if (num_handles) *num_handles = 0;
    if (!packet || !out || !out_size || !handles || !num_handles) return -EINVAL;
    size_t size; const unsigned char *bytes = cc_ipc_packet_data(packet, &size);
    if (size < sizeof(cc_ipc_message_header) + sizeof(struct envelope)) return -EPROTO;
    bytes += sizeof(cc_ipc_message_header); size -= sizeof(cc_ipc_message_header);
    struct envelope header; memcpy(&header, bytes, sizeof(header));
    uint32_t count; const int *fds = cc_ipc_packet_fds(packet, &count);
    uint32_t large = header.storage & 1U, extra = header.storage >> 1U;
    if (header.version != MESSAGE_VERSION || extra > CC_IPA_WIRE_MAX_FDS || header.size > CC_IPA_WIRE_MAX_DATA ||
        count != extra + large) return -EPROTO;
    if (!large) {
        if (header.size != size - sizeof(header)) return -EPROTO;
    } else {
        if (size != sizeof(header) || fds[0] < 0) return -EPROTO;
        int seals = fcntl(fds[0], F_GET_SEALS);
        if (seals < 0 || (seals & MESSAGE_SEALS) != MESSAGE_SEALS) return -EPROTO;
        struct stat status;
        if (fstat(fds[0], &status)) return -errno;
        if (!S_ISREG(status.st_mode) || status.st_size < 0 || (uint64_t)status.st_size != header.size) return -EPROTO;
    }
    void *data = header.size ? malloc(header.size) : NULL;
    if (header.size && !data) return -ENOMEM;
    if (!large) {
        if (header.size) memcpy(data, bytes + sizeof(header), header.size);
    } else {
        size_t offset = 0;
        while (offset < header.size) {
            ssize_t n = pread(fds[0], (unsigned char *)data + offset, header.size - offset, (off_t)offset);
            if (n < 0) { if (errno == EINTR) continue; int error = -errno; free(data); return error; }
            if (!n) { free(data); return -EPROTO; }
            offset += (size_t)n;
        }
    }
    *out = data; *out_size = header.size; *handles = extra ? fds + large : NULL; *num_handles = extra; return 0;
}
int cc_ipa_message_decode(const cc_ipc_packet *packet, void **out, size_t *out_size)
{
    const int *fds; uint32_t count; int ret = cc_ipa_message_decode_fds(packet, out, out_size, &fds, &count);
    if (!ret && count) { free(*out); *out = NULL; *out_size = 0; return -EPROTO; }
    return ret;
}
