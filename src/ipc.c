/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Datagram framing, SCM_RIGHTS and command/cookie layout follow libcamera's
 * ipc_unixsocket.cpp and ipc_pipe.cpp. Copyright (C) 2019-2020 Google Inc. */
#define _GNU_SOURCE
#include "ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/un.h>
#include <unistd.h>
_Static_assert(sizeof(cc_ipc_message_header) == 8, "IPCMessage command/cookie layout");
/* Linux's descriptor-bearing pidfd ancillary type; older libc headers omit it. */
#define CC_SCM_PIDFD 0x04

struct cc_ipc_packet {
    void *data;
    size_t size;
    uint32_t num_fds;
    int fds[CC_IPC_RIGHTS_CAPACITY];
};
struct cc_ipc {
    int fd, error;
    cc_ipc_packet *send, *receive;
    bool sent_header;
    uint32_t receive_fds;
    const struct cc_ipc_io *io;
    void *context;
};
union control { struct cmsghdr alignment; unsigned char bytes[CC_IPC_CONTROL_SIZE]; };
static ssize_t native_send(void *context, int fd, const struct msghdr *message, int flags)
{
    (void)context; ssize_t n;
    do { n = sendmsg(fd, message, flags); } while (n < 0 && errno == EINTR);
    return n < 0 ? -errno : n;
}
static ssize_t native_receive(void *context, int fd, struct msghdr *message, int flags)
{
    (void)context; ssize_t n;
    size_t capacity = message->msg_controllen;
    do {
        message->msg_controllen = capacity; message->msg_flags = 0;
        n = recvmsg(fd, message, flags);
    } while (n < 0 && errno == EINTR);
    return n < 0 ? -errno : n;
}
static const struct cc_ipc_io native_io = {native_send, native_receive};
void cc_ipc_set_io(cc_ipc *channel, const struct cc_ipc_io *io, void *context)
{
    channel->io = io ? io : &native_io; channel->context = context;
}
void cc_ipc_packet_destroy(cc_ipc_packet *packet)
{
    if (!packet) return;
    for (uint32_t i = 0; i < packet->num_fds; ++i) if (packet->fds[i] >= 0) close(packet->fds[i]);
    free(packet->data); free(packet);
}
static int failed(cc_ipc *channel, int error)
{
    channel->error = error;
    cc_ipc_packet_destroy(channel->send); channel->send = NULL;
    cc_ipc_packet_destroy(channel->receive); channel->receive = NULL;
    if (channel->fd >= 0) close(channel->fd);
    channel->fd = -1;
    return error;
}
void cc_ipc_close(cc_ipc *channel)
{
    if (!channel) return;
    failed(channel, -ENOTCONN); free(channel);
}
int cc_ipc_create(cc_ipc **out, int *remote)
{
    if (out) *out = NULL;
    if (remote) *remote = -1;
    if (!out || !remote) return -EINVAL;
    cc_ipc *channel = calloc(1, sizeof(*channel));
    if (!channel) return -ENOMEM;
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, sockets)) {
        int error = -errno; free(channel); return error;
    }
    channel->fd = sockets[0]; channel->io = &native_io;
    *out = channel; *remote = sockets[1]; return 0;
}
int cc_ipc_bind(int fd, cc_ipc **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (fd < 0) return -EINVAL;
    int type; socklen_t length = sizeof(type);
    if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &length)) return -errno;
    if (length != sizeof(type) || type != SOCK_DGRAM) return -EPROTOTYPE;
    struct sockaddr_un address; length = sizeof(address);
    if (getsockname(fd, (struct sockaddr *)&address, &length)) return -errno;
    if (length < sizeof(sa_family_t) || address.sun_family != AF_UNIX) return -EAFNOSUPPORT;
    length = sizeof(address);
    if (getpeername(fd, (struct sockaddr *)&address, &length)) return -errno;
    if (length < sizeof(sa_family_t) || address.sun_family != AF_UNIX) return -EAFNOSUPPORT;
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) return -errno;
    if (!(flags & O_NONBLOCK)) return -EINVAL;
    cc_ipc *channel = calloc(1, sizeof(*channel));
    if (!channel) return -ENOMEM;
    channel->fd = fcntl(fd, F_DUPFD_CLOEXEC, 0);
    if (channel->fd < 0) { int error = -errno; free(channel); return error; }
    channel->io = &native_io; *out = channel; return 0;
}
int cc_ipc_fd(const cc_ipc *channel) { return !channel ? -EINVAL : channel->error ? channel->error : channel->fd; }
int cc_ipc_events(const cc_ipc *channel, short *events)
{
    if (!channel || !events) return -EINVAL;
    if (channel->error) return channel->error;
    *events = (short)(POLLIN | (channel->send ? POLLOUT : 0)); return 0;
}
int cc_ipc_decode_header(const void *data, size_t size, uint32_t *length, uint32_t *fds)
{
    if (!data || !length || !fds) return -EINVAL;
    if (size != 8) return -EPROTO;
    uint32_t bytes; memcpy(&bytes, data, sizeof(bytes));
    uint32_t descriptors = ((const unsigned char *)data)[4];
    if (bytes > CC_IPC_MAX_DATA || descriptors > CC_IPC_MAX_FDS || (!bytes && !descriptors)) return -EPROTO;
    *length = bytes; *fds = descriptors; return 0;
}
int cc_ipc_decode_rights(const void *data, size_t size, int fds[CC_IPC_RIGHTS_CAPACITY], uint32_t *count)
{
    if (!fds || !count || (size && !data)) return -EINVAL;
    *count = 0;
    if (size > CC_IPC_CONTROL_SIZE) return -EPROTO;
    size_t offset = 0; int error = 0;
    while (offset < size) {
        if (size - offset < sizeof(struct cmsghdr)) return -EPROTO;
        struct cmsghdr header; memcpy(&header, (const unsigned char *)data + offset, sizeof(header));
        if (header.cmsg_len < CMSG_LEN(0) || header.cmsg_len > size - offset) return -EPROTO;
        size_t bytes = header.cmsg_len - CMSG_LEN(0);
        bool rights = header.cmsg_level == SOL_SOCKET && header.cmsg_type == SCM_RIGHTS;
        bool pidfd = header.cmsg_level == SOL_SOCKET && header.cmsg_type == CC_SCM_PIDFD;
        if (!rights) error = -EPROTO;
        if (rights || pidfd) {
            if (bytes % sizeof(int)) error = -EPROTO;
            if (pidfd && bytes != sizeof(int)) error = -EPROTO;
            size_t num = bytes / sizeof(int);
            if (num > CC_IPC_RIGHTS_CAPACITY - *count) return -EPROTO;
            memcpy(fds + *count, (const unsigned char *)data + offset + CMSG_LEN(0), num * sizeof(int));
            for (size_t i = 0; i < num; ++i) if (fds[*count + i] < 0) error = -EPROTO;
            *count += (uint32_t)num;
        }
        size_t aligned = CMSG_ALIGN(header.cmsg_len);
        if (aligned > size - offset) return header.cmsg_len == size - offset ? error : -EPROTO;
        offset += aligned;
    }
    return error;
}
int cc_ipc_flush(cc_ipc *channel)
{
    if (!channel) return -EINVAL;
    if (channel->error) return channel->error;
    if (!channel->send) return 0;
    cc_ipc_packet *packet = channel->send;
    if (!channel->sent_header) {
        unsigned char header[8] = {0}; uint32_t size = (uint32_t)packet->size;
        memcpy(header, &size, sizeof(size)); header[4] = (unsigned char)packet->num_fds;
        struct iovec vector = {header, sizeof(header)};
        struct msghdr message = {.msg_iov = &vector, .msg_iovlen = 1};
        ssize_t n = channel->io->send_message(channel->context, channel->fd, &message, MSG_DONTWAIT | MSG_NOSIGNAL);
        if (n == -EAGAIN || n == -EWOULDBLOCK) return -EAGAIN;
        if (n < 0 || (size_t)n != sizeof(header)) return failed(channel, n < 0 ? (int)n : -EPROTO);
        channel->sent_header = true;
    }
    struct iovec vector = {packet->data, packet->size};
    struct msghdr message = {.msg_iov = &vector, .msg_iovlen = 1};
    union control control = {0};
    if (packet->num_fds) {
        message.msg_control = control.bytes;
        message.msg_controllen = CMSG_SPACE(packet->num_fds * sizeof(int));
        struct cmsghdr *header = (struct cmsghdr *)control.bytes;
        header->cmsg_len = CMSG_LEN(packet->num_fds * sizeof(int));
        header->cmsg_level = SOL_SOCKET; header->cmsg_type = SCM_RIGHTS;
        memcpy(CMSG_DATA(header), packet->fds, packet->num_fds * sizeof(int));
    }
    ssize_t n = channel->io->send_message(channel->context, channel->fd, &message, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (n == -EAGAIN || n == -EWOULDBLOCK) return -EAGAIN;
    if (n < 0 || (size_t)n != packet->size) return failed(channel, n < 0 ? (int)n : -EPROTO);
    cc_ipc_packet_destroy(packet); channel->send = NULL; channel->sent_header = false;
    return 0;
}
static int send_payload(cc_ipc *channel, const cc_ipc_message_header *header, const cc_ipc_payload *payload)
{
    if (!channel || !payload || (payload->size && !payload->data) || (payload->num_fds && !payload->fds)) return -EINVAL;
    if (channel->error) return channel->error;
    if (channel->send) return -EBUSY;
    size_t prefix = header ? sizeof(*header) : 0;
    if (payload->size > CC_IPC_MAX_DATA - prefix || payload->num_fds > CC_IPC_MAX_FDS) return -EMSGSIZE;
    if (!prefix && !payload->size && !payload->num_fds) return -EINVAL;
    cc_ipc_packet *packet = calloc(1, sizeof(*packet));
    if (!packet) return -ENOMEM;
    packet->size = prefix + payload->size;
    if (packet->size) {
        packet->data = malloc(packet->size);
        if (!packet->data) { cc_ipc_packet_destroy(packet); return -ENOMEM; }
        if (header) memcpy(packet->data, header, prefix);
        if (payload->size) memcpy((unsigned char *)packet->data + prefix, payload->data, payload->size);
    }
    for (uint32_t i = 0; i < payload->num_fds; ++i) {
        if (payload->fds[i] < 0) { cc_ipc_packet_destroy(packet); return -EBADF; }
        int fd = fcntl(payload->fds[i], F_DUPFD_CLOEXEC, 0);
        if (fd < 0) { int error = -errno; cc_ipc_packet_destroy(packet); return error; }
        packet->fds[packet->num_fds++] = fd;
    }
    channel->send = packet;
    int ret = cc_ipc_flush(channel);
    return ret == -EAGAIN ? 0 : ret;
}
int cc_ipc_send(cc_ipc *channel, const cc_ipc_payload *payload) { return send_payload(channel, NULL, payload); }
int cc_ipc_send_message(cc_ipc *channel, const cc_ipc_message_header *header, const cc_ipc_payload *payload)
{
    if (!header) return -EINVAL;
    return send_payload(channel, header, payload);
}
static ssize_t receive_packet(cc_ipc *channel, void *data, size_t size, cc_ipc_packet *packet, int *flags)
{
    union control control = {0};
    struct iovec vector = {data, size};
    struct msghdr message = {.msg_iov = &vector, .msg_iovlen = 1,
        .msg_control = control.bytes, .msg_controllen = sizeof(control.bytes)};
    ssize_t n = channel->io->receive_message(channel->context, channel->fd, &message,
        MSG_DONTWAIT | MSG_CMSG_CLOEXEC | MSG_TRUNC);
    if (n < 0) return n;
    int ret = cc_ipc_decode_rights(control.bytes, message.msg_controllen, packet->fds, &packet->num_fds);
    *flags = message.msg_flags;
    if (ret < 0 || (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC))) return -EPROTO;
    return n;
}
int cc_ipc_receive(cc_ipc *channel, cc_ipc_packet **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!channel) return -EINVAL;
    if (channel->error) return channel->error;
    if (!channel->receive) {
        unsigned char header[8]; cc_ipc_packet descriptors = {0}; int flags = 0;
        ssize_t n = receive_packet(channel, header, sizeof(header), &descriptors, &flags);
        for (uint32_t i = 0; i < descriptors.num_fds; ++i) if (descriptors.fds[i] >= 0) close(descriptors.fds[i]);
        if (n == -EAGAIN || n == -EWOULDBLOCK) return -EAGAIN;
        if (n < 0 || descriptors.num_fds) return failed(channel, n < 0 ? (int)n : -EPROTO);
        uint32_t size, fds;
        if (cc_ipc_decode_header(header, (size_t)n, &size, &fds) < 0) return failed(channel, -EPROTO);
        channel->receive = calloc(1, sizeof(*channel->receive));
        if (!channel->receive) return failed(channel, -ENOMEM);
        channel->receive->size = size; channel->receive_fds = fds;
        if (size) {
            channel->receive->data = malloc(size);
            if (!channel->receive->data) return failed(channel, -ENOMEM);
        }
    }
    cc_ipc_packet *packet = channel->receive; int flags = 0;
    ssize_t n = receive_packet(channel, packet->data, packet->size, packet, &flags);
    if (n == -EAGAIN || n == -EWOULDBLOCK) return -EAGAIN;
    if (n < 0 || (size_t)n != packet->size || packet->num_fds != channel->receive_fds)
        return failed(channel, n < 0 && n != -EPROTO ? (int)n : -EPROTO);
    channel->receive = NULL; channel->receive_fds = 0; *out = packet;
    return 0;
}
const void *cc_ipc_packet_data(const cc_ipc_packet *packet, size_t *size)
{
    if (size) *size = packet ? packet->size : 0;
    return packet ? packet->data : NULL;
}
const int *cc_ipc_packet_fds(const cc_ipc_packet *packet, uint32_t *count)
{
    if (count) *count = packet ? packet->num_fds : 0;
    return packet && packet->num_fds ? packet->fds : NULL;
}
int cc_ipc_packet_take_fd(cc_ipc_packet *packet, uint32_t index)
{
    if (!packet) return -EINVAL;
    if (index >= packet->num_fds || packet->fds[index] < 0) return -ENOENT;
    int fd = packet->fds[index]; packet->fds[index] = -1; return fd;
}
int cc_ipc_packet_header(const cc_ipc_packet *packet, cc_ipc_message_header *header)
{
    if (!packet || !header) return -EINVAL;
    if (packet->size < sizeof(*header)) return -EMSGSIZE;
    memcpy(header, packet->data, sizeof(*header)); return 0;
}
