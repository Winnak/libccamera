/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Cookie correlation and the 2s call policy follow libcamera's
 * ipc_pipe_unixsocket.cpp. Copyright (C) 2020 Google Inc. */
#define _GNU_SOURCE
#include <ccamera/ipc_pipe.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

struct cc_ipc_pipe {
    cc_ipc *channel;
    int peer_fd, error;
    cc_ipc_packet *pending[CC_IPC_PIPE_MAX_PENDING], *reply;
    size_t head, count, bytes;
    uint32_t fds, cookie;
    bool calling, accepted;
};
struct pipe_deadline { uint64_t end; bool infinite; };
static int current_ms(uint64_t *out)
{
    struct timespec time;
    if (clock_gettime(CLOCK_MONOTONIC, &time)) return -errno;
    *out = (uint64_t)time.tv_sec * 1000U + (uint64_t)time.tv_nsec / 1000000U;
    return 0;
}
static int deadline_start(int timeout, struct pipe_deadline *deadline)
{
    if (timeout < -1) return -EINVAL;
    deadline->end = 0; deadline->infinite = timeout < 0;
    if (timeout >= 0) {
        int ret = current_ms(&deadline->end); if (ret < 0) return ret;
        deadline->end += (uint32_t)timeout;
    }
    return 0;
}
static int fail(cc_ipc_pipe *pipe, int error)
{
    if (!pipe->error) pipe->error = error;
    cc_ipc_close(pipe->channel); pipe->channel = NULL;
    if (pipe->peer_fd >= 0) close(pipe->peer_fd);
    pipe->peer_fd = -1; return pipe->error;
}
int cc_ipc_pipe_open(cc_ipc **channel, int peer_fd, cc_ipc_pipe **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!channel || !*channel || peer_fd < -1) return -EINVAL;
    int ret = cc_ipc_fd(*channel); if (ret < 0) return ret;
    cc_ipc_pipe *pipe = calloc(1, sizeof(*pipe)); if (!pipe) return -ENOMEM;
    pipe->peer_fd = -1;
    if (peer_fd >= 0) {
        pipe->peer_fd = fcntl(peer_fd, F_DUPFD_CLOEXEC, 0);
        if (pipe->peer_fd < 0) { ret = -errno; free(pipe); return ret; }
    }
    pipe->channel = *channel; *channel = NULL; *out = pipe; return 0;
}
void cc_ipc_pipe_close(cc_ipc_pipe *pipe)
{
    if (!pipe) return;
    fail(pipe, -ENOTCONN);
    for (size_t i = 0; i < pipe->count; ++i)
        cc_ipc_packet_destroy(pipe->pending[(pipe->head + i) % CC_IPC_PIPE_MAX_PENDING]);
    cc_ipc_packet_destroy(pipe->reply); free(pipe);
}
int cc_ipc_pipe_fd(const cc_ipc_pipe *pipe) { return !pipe ? -EINVAL : pipe->error ? pipe->error : cc_ipc_fd(pipe->channel); }
int cc_ipc_pipe_peer_fd(const cc_ipc_pipe *pipe) { return !pipe ? -EINVAL : pipe->error ? pipe->error : pipe->peer_fd < 0 ? -ENODATA : pipe->peer_fd; }
int cc_ipc_pipe_events(const cc_ipc_pipe *pipe, short *events)
{
    if (!pipe || !events) return -EINVAL;
    return pipe->error ? pipe->error : cc_ipc_events(pipe->channel, events);
}
size_t cc_ipc_pipe_pending(const cc_ipc_pipe *pipe) { return pipe ? pipe->count : 0; }
static int enqueue(cc_ipc_pipe *pipe, cc_ipc_packet *packet)
{
    size_t size = 0; uint32_t fds = 0;
    cc_ipc_packet_data(packet, &size); cc_ipc_packet_fds(packet, &fds);
    if (pipe->count == CC_IPC_PIPE_MAX_PENDING || size > CC_IPC_PIPE_MAX_PENDING_BYTES - pipe->bytes ||
        fds > CC_IPC_PIPE_MAX_PENDING_FDS - pipe->fds) return -ENOBUFS;
    pipe->pending[(pipe->head + pipe->count++) % CC_IPC_PIPE_MAX_PENDING] = packet;
    pipe->bytes += size; pipe->fds += fds; return 0;
}
static cc_ipc_packet *dequeue(cc_ipc_pipe *pipe)
{
    cc_ipc_packet *packet = pipe->pending[pipe->head];
    pipe->head = (pipe->head + 1) % CC_IPC_PIPE_MAX_PENDING; --pipe->count;
    size_t size = 0; uint32_t fds = 0; cc_ipc_packet_data(packet, &size); cc_ipc_packet_fds(packet, &fds);
    pipe->bytes -= size; pipe->fds -= fds; return packet;
}
int cc_ipc_pipe_dispatch(cc_ipc_pipe *pipe)
{
    if (!pipe) return -EINVAL;
    if (pipe->error) return pipe->error;
    bool drained = false;
    for (size_t i = 0; i < CC_IPC_PIPE_MAX_PENDING; ++i) {
        cc_ipc_packet *packet = NULL; int ret = cc_ipc_receive(pipe->channel, &packet);
        if (ret == -EAGAIN) { drained = true; break; }
        if (ret < 0) return fail(pipe, ret);
        cc_ipc_message_header header; ret = cc_ipc_packet_header(packet, &header);
        if (ret < 0) ret = -EPROTO;
        if (!ret && pipe->calling && pipe->accepted && !pipe->reply && header.cookie == pipe->cookie)
            pipe->reply = packet;
        else if (!ret) ret = enqueue(pipe, packet);
        if (ret < 0) { cc_ipc_packet_destroy(packet); return fail(pipe, ret); }
    }
    int ret = cc_ipc_flush(pipe->channel);
    if (ret < 0 && ret != -EAGAIN) return fail(pipe, ret);
    /* Drain the socket before reporting peer death, including final replies.
     * If the receive budget was exhausted, defer exit until a later dispatch. */
    if (drained && pipe->peer_fd >= 0) {
        struct pollfd fd = {.fd = pipe->peer_fd, .events = POLLIN};
        do { ret = poll(&fd, 1, 0); } while (ret < 0 && errno == EINTR);
        if (ret < 0) return fail(pipe, -errno);
        if (fd.revents & POLLNVAL) return fail(pipe, -EBADF);
        if (fd.revents) return fail(pipe, -EPIPE);
    }
    return 0;
}
int cc_ipc_pipe_send(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const cc_ipc_payload *payload)
{
    if (!pipe) return -EINVAL;
    if (pipe->error) return pipe->error;
    int ret = cc_ipc_send_message(pipe->channel, header, payload);
    if (ret < 0 && cc_ipc_fd(pipe->channel) < 0) return fail(pipe, ret);
    return ret;
}
static int wait_ready(cc_ipc_pipe *pipe, const struct pipe_deadline *deadline)
{
    if (pipe->error) return pipe->error;
    int timeout = -1;
    if (!deadline->infinite) {
        uint64_t now = 0; int ret = current_ms(&now); if (ret < 0) return ret;
        if (now >= deadline->end) return -ETIMEDOUT;
        uint64_t delta = deadline->end > now ? deadline->end - now : 0;
        timeout = delta > INT_MAX ? INT_MAX : (int)delta;
    }
    struct pollfd fds[2] = {{.fd = cc_ipc_fd(pipe->channel)}, {.fd = pipe->peer_fd, .events = POLLIN}};
    int ret = cc_ipc_events(pipe->channel, &fds[0].events); if (ret < 0) return fail(pipe, ret);
    ret = poll(fds, pipe->peer_fd < 0 ? 1U : 2U, timeout);
    if (ret < 0) return errno == EINTR ? 0 : fail(pipe, -errno);
    if (!ret) return -ETIMEDOUT;
    if ((fds[0].revents | fds[1].revents) & POLLNVAL) return fail(pipe, -EBADF);
    return 0;
}
int cc_ipc_pipe_receive(cc_ipc_pipe *pipe, cc_ipc_packet **out, int timeout_ms)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!pipe || timeout_ms < -1) return -EINVAL;
    if (pipe->calling) return -EBUSY;
    struct pipe_deadline deadline; int ret = deadline_start(timeout_ms, &deadline); if (ret < 0) return ret;
    for (;;) {
        if (pipe->count) { *out = dequeue(pipe); return 0; }
        ret = cc_ipc_pipe_dispatch(pipe);
        if (pipe->count) { *out = dequeue(pipe); return 0; }
        if (ret < 0) return ret;
        ret = wait_ready(pipe, &deadline);
        if (ret < 0) return !timeout_ms && ret == -ETIMEDOUT ? -EAGAIN : ret;
    }
}
int cc_ipc_pipe_call(cc_ipc_pipe *pipe, const cc_ipc_message_header *header,
                     const cc_ipc_payload *payload, cc_ipc_packet **out, int timeout_ms)
{
    if (out) *out = NULL;
    if (!pipe || !header || !payload || timeout_ms < -1) return -EINVAL;
    if (pipe->calling) return -EBUSY;
    if (pipe->error) return pipe->error;
    struct pipe_deadline deadline; int ret = deadline_start(timeout_ms, &deadline); if (ret < 0) return ret;
    pipe->calling = true; pipe->accepted = false; pipe->cookie = header->cookie;
    for (;;) {
        ret = cc_ipc_pipe_dispatch(pipe);
        if (pipe->reply) {
            if (out) *out = pipe->reply; else cc_ipc_packet_destroy(pipe->reply);
            pipe->reply = NULL; ret = 0; break;
        }
        if (ret < 0) break;
        if (!pipe->accepted) {
            ret = cc_ipc_pipe_send(pipe, header, payload);
            if (!ret) { pipe->accepted = true; continue; }
            if (ret != -EBUSY) break;
        }
        ret = wait_ready(pipe, &deadline);
        if (ret < 0) { if (pipe->accepted) ret = fail(pipe, ret); break; }
    }
    pipe->calling = false; pipe->accepted = false; return ret;
}
