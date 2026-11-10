/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipc.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
struct ipc_vector {
    unsigned message; uint32_t command, cookie, size, fds; int status; uint64_t hash;
    unsigned char header[8], prefix[8];
};
#include "ipc_vectors.h"
union ancillary { struct cmsghdr alignment; unsigned char bytes[CC_IPC_CONTROL_SIZE]; };
static size_t descriptors(void)
{
    DIR *directory = opendir("/proc/self/fd"); CHECK(directory); size_t n = 0; struct dirent *entry;
    while ((entry = readdir(directory))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++n;
    OK(closedir(directory)); return n;
}
static uint64_t hash(const unsigned char *data, size_t size)
{
    uint64_t value = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < size; ++i) { value ^= data[i]; value *= UINT64_C(1099511628211); }
    return value;
}
static void native_fields(unsigned char *data, size_t fields)
{
    const uint32_t endian = 1;
    if (*(const unsigned char *)&endian) return;
    for (size_t i = 0; i < fields; ++i) {
        unsigned char *p = data + 4 * i, a = p[0], b = p[1]; p[0] = p[3]; p[1] = p[2]; p[2] = b; p[3] = a;
    }
}
static void raw_send(int fd, const void *data, size_t size, const int *fds, uint32_t count)
{
    union ancillary control = {0}; struct iovec vector = {(void *)data, size};
    struct msghdr message = {.msg_iov = &vector, .msg_iovlen = 1};
    if (count) {
        CHECK(count <= CC_IPC_MAX_FDS); message.msg_control = control.bytes;
        message.msg_controllen = CMSG_SPACE(count * sizeof(int));
        struct cmsghdr *header = CMSG_FIRSTHDR(&message);
        header->cmsg_level = SOL_SOCKET; header->cmsg_type = SCM_RIGHTS; header->cmsg_len = CMSG_LEN(count * sizeof(int));
        memcpy(CMSG_DATA(header), fds, count * sizeof(int));
    }
    CHECK(sendmsg(fd, &message, MSG_DONTWAIT | MSG_NOSIGNAL) == (ssize_t)size);
}
static void raw_header(int fd, uint32_t size, uint32_t fds)
{
    unsigned char header[8] = {0}; memcpy(header, &size, sizeof(size)); header[4] = (unsigned char)fds;
    raw_send(fd, header, sizeof(header), NULL, 0);
}
static cc_ipc_packet *receive(cc_ipc *channel)
{
    cc_ipc_packet *packet = NULL;
    for (unsigned i = 0; i < 10; ++i) {
        int ret = cc_ipc_receive(channel, &packet);
        if (!ret) return packet;
        CHECK(ret == -EAGAIN && !packet);
        struct pollfd fd = {.fd = cc_ipc_fd(channel), .events = POLLIN};
        CHECK(poll(&fd, 1, 1000) == 1);
    }
    CHECK(0); return NULL;
}
static void wire_vectors(void)
{
    size_t before = descriptors(); cc_ipc *channel = NULL; int peer = -1;
    OK(cc_ipc_create(&channel, &peer));
    int capacity = 4 * CC_IPC_MAX_DATA; OK(setsockopt(cc_ipc_fd(channel), SOL_SOCKET, SO_SNDBUF, &capacity, sizeof(capacity)));
    int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0);
    int fds[CC_IPC_MAX_FDS]; for (size_t i = 0; i < CC_IPC_MAX_FDS; ++i) fds[i] = file;
    unsigned char data[CC_IPC_MAX_DATA], body[CC_IPC_MAX_DATA];
    for (size_t i = 0; i < sizeof(data); ++i) data[i] = (unsigned char)(i * 17 + 3);
    for (size_t i = 0; i < sizeof(ipc_vectors) / sizeof(ipc_vectors[0]); ++i) {
        const struct ipc_vector *v = &ipc_vectors[i];
        cc_ipc_payload payload = {data, v->size, fds, v->fds}; cc_ipc_message_header prefix = {v->command, v->cookie};
        int ret = v->message ? cc_ipc_send_message(channel, &prefix, &payload) : cc_ipc_send(channel, &payload);
        CHECK(ret == v->status);
        if (ret < 0) { CHECK(recv(peer, body, sizeof(body), MSG_DONTWAIT) < 0 && errno == EAGAIN); continue; }
        OK(cc_ipc_flush(channel)); unsigned char header[8], expected[8], expected_prefix[8];
        CHECK(recv(peer, header, sizeof(header), MSG_DONTWAIT | MSG_TRUNC) == 8);
        memcpy(expected, v->header, 8); native_fields(expected, 1); CHECK(!memcmp(header, expected, 8));
        union ancillary control = {0}; struct iovec vector = {body, sizeof(body)};
        struct msghdr message = {.msg_iov = &vector, .msg_iovlen = 1, .msg_control = control.bytes, .msg_controllen = sizeof(control.bytes)};
        size_t offset = v->message ? 8U : 0U;
        CHECK(recvmsg(peer, &message, MSG_DONTWAIT | MSG_CMSG_CLOEXEC | MSG_TRUNC) == (ssize_t)(v->size + offset));
        CHECK(!(message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)));
        if (v->message) {
            memcpy(expected_prefix, v->prefix, 8); native_fields(expected_prefix, 2); CHECK(!memcmp(body, expected_prefix, 8));
        }
        CHECK(hash(body + offset, v->size) == v->hash);
        uint32_t count = 0;
        for (struct cmsghdr *c = CMSG_FIRSTHDR(&message); c; c = CMSG_NXTHDR(&message, c)) {
            CHECK(c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS);
            size_t num = (c->cmsg_len - CMSG_LEN(0)) / sizeof(int);
            for (size_t j = 0; j < num; ++j) {
                int fd; memcpy(&fd, CMSG_DATA(c) + j * sizeof(fd), sizeof(fd));
                CHECK(fcntl(fd, F_GETFD) & FD_CLOEXEC); struct stat status; OK(fstat(fd, &status)); CHECK(S_ISCHR(status.st_mode)); OK(close(fd));
            }
            count += (uint32_t)num;
        }
        CHECK(count == v->fds);
    }
    OK(close(file)); OK(close(peer)); cc_ipc_close(channel); CHECK(descriptors() == before);
}
static void bidirectional(void)
{
    size_t before = descriptors(); cc_ipc *a = NULL, *b = NULL; int peer = -1;
    OK(cc_ipc_create(&a, &peer)); OK(cc_ipc_bind(peer, &b));
    CHECK(fcntl(peer, F_GETFD) & FD_CLOEXEC); OK(close(peer));
    for (unsigned direction = 0; direction < 2; ++direction) for (unsigned count = 0; count < 3; ++count) {
        cc_ipc *from = direction ? b : a, *to = direction ? a : b;
        int pipe_fds[2]; OK(pipe2(pipe_fds, O_CLOEXEC | O_NONBLOCK));
        const char text[] = "snapshot"; int files[] = {pipe_fds[0], pipe_fds[1]};
        cc_ipc_payload payload = {count == 1 ? NULL : text, count == 1 ? 0 : sizeof(text), files, count};
        OK(cc_ipc_send(from, &payload)); OK(close(pipe_fds[0])); OK(close(pipe_fds[1]));
        cc_ipc_packet *packet = receive(to); size_t size = 999; uint32_t num = 999;
        const void *data = cc_ipc_packet_data(packet, &size); const int *fds = cc_ipc_packet_fds(packet, &num);
        CHECK(size == payload.size && num == count && (!size || !memcmp(data, text, size)));
        for (uint32_t i = 0; i < num; ++i) CHECK(fds[i] >= 0 && (fcntl(fds[i], F_GETFD) & FD_CLOEXEC));
        if (count == 2) { CHECK(write(fds[1], "x", 1) == 1); char c; CHECK(read(fds[0], &c, 1) == 1 && c == 'x'); }
        cc_ipc_message_header header = {123, 456};
        if (size < 8) { ERROR(cc_ipc_packet_header(packet, &header), EMSGSIZE); CHECK(header.command == 123 && header.cookie == 456); }
        if (count) {
            int taken = cc_ipc_packet_take_fd(packet, 0); CHECK(taken >= 0); ERROR(cc_ipc_packet_take_fd(packet, 0), ENOENT);
            cc_ipc_packet_destroy(packet); CHECK(fcntl(taken, F_GETFD) >= 0); OK(close(taken));
        } else cc_ipc_packet_destroy(packet);
    }
    cc_ipc_close(a); cc_ipc_close(b); CHECK(descriptors() == before);
}
struct fault { unsigned calls, at; int error; bool short_write, truncated; };
static ssize_t faulty_send(void *context, int fd, const struct msghdr *message, int flags)
{
    struct fault *fault = context;
    if (++fault->calls == fault->at) return fault->short_write ? (ssize_t)message->msg_iov[0].iov_len - 1 : -fault->error;
    ssize_t n = sendmsg(fd, message, flags); return n < 0 ? -errno : n;
}
static ssize_t faulty_receive(void *context, int fd, struct msghdr *message, int flags)
{
    struct fault *fault = context;
    if (++fault->calls == fault->at) return -fault->error;
    ssize_t n = recvmsg(fd, message, flags);
    if (n >= 0 && (message->msg_flags & MSG_CTRUNC)) fault->truncated = true;
    return n < 0 ? -errno : n;
}
static const struct cc_ipc_io fault_io = {faulty_send, faulty_receive};
static void partial_sends(void)
{
    size_t before = descriptors();
    for (unsigned at = 1; at <= 2; ++at) {
        cc_ipc *a = NULL, *b = NULL; int peer = -1; OK(cc_ipc_create(&a, &peer)); OK(cc_ipc_bind(peer, &b)); OK(close(peer));
        struct fault fault = {.at = at, .error = EAGAIN}; cc_ipc_set_io(a, &fault_io, &fault);
        char text[] = "before"; int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0);
        cc_ipc_payload payload = {text, sizeof(text), &file, 1};
        OK(cc_ipc_send(a, &payload)); OK(close(file)); memcpy(text, "after!", sizeof(text));
        short events = 0; OK(cc_ipc_events(a, &events)); CHECK(events == (POLLIN | POLLOUT));
        ERROR(cc_ipc_send(a, &payload), EBUSY);
        cc_ipc_packet *packet = (cc_ipc_packet *)(uintptr_t)1; ERROR(cc_ipc_receive(b, &packet), EAGAIN); CHECK(!packet);
        cc_ipc_set_io(a, NULL, NULL); OK(cc_ipc_flush(a)); OK(cc_ipc_events(a, &events)); CHECK(events == POLLIN);
        packet = receive(b); size_t size; CHECK(!strcmp(cc_ipc_packet_data(packet, &size), "before") && size == sizeof(text));
        uint32_t count; const int *fds = cc_ipc_packet_fds(packet, &count); CHECK(count == 1 && fcntl(fds[0], F_GETFD) >= 0);
        cc_ipc_packet_destroy(packet); payload = (cc_ipc_payload){"next", 5, NULL, 0}; OK(cc_ipc_send(a, &payload));
        packet = receive(b); CHECK(!strcmp(cc_ipc_packet_data(packet, NULL), "next")); cc_ipc_packet_destroy(packet);
        cc_ipc_close(a); cc_ipc_close(b); CHECK(descriptors() == before);
    }
    for (unsigned short_write = 0; short_write < 2; ++short_write) {
        cc_ipc *a = NULL; int peer = -1; OK(cc_ipc_create(&a, &peer));
        struct fault fault = {.at = 2, .error = EIO, .short_write = short_write != 0}; cc_ipc_set_io(a, &fault_io, &fault);
        int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0); cc_ipc_payload payload = {"x", 1, &file, 1};
        int error = short_write ? EPROTO : EIO;
        ERROR(cc_ipc_send(a, &payload), error); ERROR(cc_ipc_flush(a), error); ERROR(cc_ipc_fd(a), error);
        short events = 777; ERROR(cc_ipc_events(a, &events), error); CHECK(events == 777);
        CHECK(fcntl(file, F_GETFD) >= 0); OK(close(file)); OK(close(peer)); cc_ipc_close(a); CHECK(descriptors() == before);
    }
    /* A receive syscall error retains its errno and clears all channel state. */
    cc_ipc *a = NULL; int peer; OK(cc_ipc_create(&a, &peer)); struct fault fault = {.at = 1, .error = EIO};
    cc_ipc_set_io(a, &fault_io, &fault); cc_ipc_packet *packet = NULL; ERROR(cc_ipc_receive(a, &packet), EIO);
    CHECK(!packet); cc_ipc_close(a); OK(close(peer)); CHECK(descriptors() == before);
}
static void kernel_backpressure(void)
{
    size_t before = descriptors(); int sockets[2]; OK(socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, sockets));
    int size = 4096; OK(setsockopt(sockets[0], SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)));
    cc_ipc *a = NULL, *b = NULL; OK(cc_ipc_bind(sockets[0], &a)); OK(cc_ipc_bind(sockets[1], &b));
    OK(close(sockets[0])); OK(close(sockets[1])); cc_ipc_payload payload = {"x", 1, NULL, 0};
    size_t accepted = 0; short events = 0;
    do { CHECK(accepted < 4096); OK(cc_ipc_send(a, &payload)); ++accepted; OK(cc_ipc_events(a, &events)); } while (!(events & POLLOUT));
    ERROR(cc_ipc_send(a, &payload), EBUSY); size_t delivered = 0; cc_ipc_packet *packet = NULL;
    for (;;) {
        int ret = cc_ipc_receive(b, &packet); if (ret == -EAGAIN) break;
        CHECK(!ret && packet); ++delivered; cc_ipc_packet_destroy(packet);
    }
    CHECK(delivered < accepted); OK(cc_ipc_flush(a));
    while (delivered < accepted) { packet = receive(b); ++delivered; cc_ipc_packet_destroy(packet); }
    cc_ipc_close(a); cc_ipc_close(b); CHECK(descriptors() == before);
}
static void process_echo(void)
{
    size_t before = descriptors(); cc_ipc *a = NULL; int peer; OK(cc_ipc_create(&a, &peer));
    pid_t child = fork(); CHECK(child >= 0);
    if (!child) {
        cc_ipc_close(a); cc_ipc *b = NULL; OK(cc_ipc_bind(peer, &b)); OK(close(peer));
        cc_ipc_packet *packet = receive(b); cc_ipc_message_header header; OK(cc_ipc_packet_header(packet, &header));
        CHECK(header.command == 99 && header.cookie == UINT32_MAX); header.command = 100;
        size_t size; const unsigned char *data = cc_ipc_packet_data(packet, &size); uint32_t count;
        const int *fds = cc_ipc_packet_fds(packet, &count);
        cc_ipc_payload payload = {data + 8, size - 8, fds, count}; OK(cc_ipc_send_message(b, &header, &payload));
        OK(cc_ipc_flush(b)); cc_ipc_packet_destroy(packet); cc_ipc_close(b); _exit(0);
    }
    OK(close(peer)); int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0);
    cc_ipc_message_header header = {99, UINT32_MAX}; cc_ipc_payload payload = {"echo", 5, &file, 1};
    OK(cc_ipc_send_message(a, &header, &payload)); OK(close(file)); cc_ipc_packet *packet = receive(a);
    OK(cc_ipc_packet_header(packet, &header)); CHECK(header.command == 100 && header.cookie == UINT32_MAX);
    size_t size; const unsigned char *data = cc_ipc_packet_data(packet, &size); CHECK(size == 13 && !memcmp(data + 8, "echo", 5));
    uint32_t count; const int *fds = cc_ipc_packet_fds(packet, &count); CHECK(count == 1 && fcntl(fds[0], F_GETFD) >= 0);
    cc_ipc_packet_destroy(packet); cc_ipc_close(a); int status; CHECK(waitpid(child, &status, 0) == child && WIFEXITED(status) && !WEXITSTATUS(status));
    CHECK(descriptors() == before);
}
static void fd_zero(void)
{
    int saved = dup(STDIN_FILENO); (void)close(STDIN_FILENO);
    cc_ipc *a = NULL; int peer = -1; OK(cc_ipc_create(&a, &peer)); CHECK(cc_ipc_fd(a) == 0); cc_ipc_close(a); OK(close(peer));
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
    OK(cc_ipc_create(&a, &peer)); int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0);
    raw_header(peer, 0, 1); raw_send(peer, NULL, 0, &file, 1); OK(close(file));
    saved = dup(STDIN_FILENO); (void)close(STDIN_FILENO);
    cc_ipc_packet *packet = receive(a); uint32_t count; const int *fds = cc_ipc_packet_fds(packet, &count);
    CHECK(count == 1 && fds[0] == 0 && (fcntl(0, F_GETFD) & FD_CLOEXEC)); CHECK(cc_ipc_packet_take_fd(packet, 0) == 0);
    cc_ipc_packet_destroy(packet); CHECK(fcntl(0, F_GETFD) >= 0); OK(close(0));
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
    cc_ipc_close(a); OK(close(peer));
}
static void malformed(void)
{
    size_t before = descriptors();
    for (unsigned mode = 0; mode < 10; ++mode) {
        cc_ipc *a = NULL; int peer; OK(cc_ipc_create(&a, &peer)); int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0);
        unsigned char header[9] = {0}; uint32_t size = 1; memcpy(header, &size, sizeof(size));
        if (mode == 0) raw_send(peer, header, 7, NULL, 0);
        if (mode == 1) raw_send(peer, header, 9, NULL, 0);
        if (mode == 2) raw_header(peer, CC_IPC_MAX_DATA + 1, 0);
        if (mode == 3) raw_header(peer, 1, CC_IPC_MAX_FDS + 1);
        if (mode == 4) raw_header(peer, 0, 0);
        if (mode == 5) raw_send(peer, header, 8, &file, 1);
        if (mode >= 6) {
            raw_header(peer, mode == 7 ? 2 : 1, mode == 8 ? 1 : mode == 9 ? 2 : 0);
            raw_send(peer, "xy", mode == 6 ? 2 : 1, mode == 9 ? &file : NULL, mode == 9 ? 1 : 0);
        }
        cc_ipc_packet *packet = (cc_ipc_packet *)(uintptr_t)1; ERROR(cc_ipc_receive(a, &packet), EPROTO); CHECK(!packet);
        ERROR(cc_ipc_receive(a, &packet), EPROTO); cc_ipc_close(a); OK(close(peer)); OK(close(file)); CHECK(descriptors() == before);
    }
    /* Unexpected ancillary credentials + maximal rights force MSG_CTRUNC.
     * This changes a native socket option solely to exercise cleanup. */
    cc_ipc *a = NULL; int peer; OK(cc_ipc_create(&a, &peer)); int enabled = 1;
    raw_header(peer, 1, CC_IPC_MAX_FDS); cc_ipc_packet *packet = NULL;
    ERROR(cc_ipc_receive(a, &packet), EAGAIN); CHECK(!packet);
    OK(setsockopt(cc_ipc_fd(a), SOL_SOCKET, SO_PASSCRED, &enabled, sizeof(enabled)));
    struct fault truncation = {0}; cc_ipc_set_io(a, &fault_io, &truncation);
    int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0); int fds[CC_IPC_MAX_FDS];
    for (size_t i = 0; i < CC_IPC_MAX_FDS; ++i) fds[i] = file;
    raw_send(peer, "x", 1, fds, CC_IPC_MAX_FDS);
    ERROR(cc_ipc_receive(a, &packet), EPROTO); CHECK(!packet);
    CHECK(truncation.truncated);
    cc_ipc_close(a); OK(close(peer)); OK(close(file)); CHECK(descriptors() == before);
    /* On newer kernels, unsupported pidfd ancillary records also own a handle. */
    OK(cc_ipc_create(&a, &peer)); raw_header(peer, 1, 0);
    ERROR(cc_ipc_receive(a, &packet), EAGAIN);
    int pass_pidfd = 76; /* SO_PASSPIDFD, absent from older libc headers. */
    if (!setsockopt(cc_ipc_fd(a), SOL_SOCKET, pass_pidfd, &enabled, sizeof(enabled))) {
        raw_send(peer, "x", 1, NULL, 0); ERROR(cc_ipc_receive(a, &packet), EPROTO); CHECK(!packet);
    } else CHECK(errno == ENOPROTOOPT || errno == EINVAL);
    cc_ipc_close(a); OK(close(peer)); CHECK(descriptors() == before);
    /* Rejection must also release descriptors still queued in the kernel. */
    OK(cc_ipc_create(&a, &peer)); int pipes[2]; OK(pipe2(pipes, O_NONBLOCK | O_CLOEXEC));
    unsigned char bad = 0; raw_send(peer, &bad, 1, NULL, 0); raw_send(peer, "x", 1, &pipes[1], 1); OK(close(pipes[1]));
    ERROR(cc_ipc_receive(a, &packet), EPROTO); char c; CHECK(read(pipes[0], &c, 1) == 0);
    cc_ipc_close(a); OK(close(peer)); OK(close(pipes[0])); CHECK(descriptors() == before);
}
static void arguments(void)
{
    cc_ipc *a = (cc_ipc *)(uintptr_t)1; int peer = 999;
    ERROR(cc_ipc_create(&a, NULL), EINVAL); CHECK(!a); ERROR(cc_ipc_create(NULL, &peer), EINVAL); CHECK(peer == -1);
    ERROR(cc_ipc_bind(-1, &a), EINVAL); CHECK(!a); ERROR(cc_ipc_bind(0, NULL), EINVAL);
    int file = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(file >= 0); ERROR(cc_ipc_bind(file, &a), ENOTSOCK); CHECK(!a);
    int sockets[2]; OK(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, sockets));
    ERROR(cc_ipc_bind(sockets[0], &a), EPROTOTYPE); OK(close(sockets[0])); OK(close(sockets[1]));
    OK(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets)); ERROR(cc_ipc_bind(sockets[0], &a), EINVAL);
    CHECK(!(fcntl(sockets[0], F_GETFL) & O_NONBLOCK)); OK(close(sockets[0])); OK(close(sockets[1]));
    int unconnected = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0); CHECK(unconnected >= 0);
    ERROR(cc_ipc_bind(unconnected, &a), ENOTCONN); OK(close(unconnected));
    int other_family = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0); CHECK(other_family >= 0);
    ERROR(cc_ipc_bind(other_family, &a), EAFNOSUPPORT); OK(close(other_family));
    OK(socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0, sockets));
    int original_flags = fcntl(sockets[0], F_GETFD); OK(cc_ipc_bind(sockets[0], &a));
    CHECK(fcntl(sockets[0], F_GETFD) == original_flags && (fcntl(cc_ipc_fd(a), F_GETFD) & FD_CLOEXEC));
    cc_ipc_close(a); OK(close(sockets[0])); OK(close(sockets[1]));
    OK(cc_ipc_create(&a, &peer)); cc_ipc_payload payload = {0}; ERROR(cc_ipc_send(a, &payload), EINVAL);
    payload = (cc_ipc_payload){"x", CC_IPC_MAX_DATA + 1, NULL, 0}; ERROR(cc_ipc_send(a, &payload), EMSGSIZE);
    payload = (cc_ipc_payload){"x", 1, &file, CC_IPC_MAX_FDS + 1}; ERROR(cc_ipc_send(a, &payload), EMSGSIZE);
    int invalid[] = {file, -1}; payload = (cc_ipc_payload){"x", 1, invalid, 2}; ERROR(cc_ipc_send(a, &payload), EBADF);
    char byte; CHECK(recv(peer, &byte, 1, MSG_DONTWAIT) < 0 && errno == EAGAIN); CHECK(fcntl(file, F_GETFD) >= 0);
    payload = (cc_ipc_payload){"x", CC_IPC_MAX_DATA - 7, NULL, 0}; cc_ipc_message_header header = {0};
    ERROR(cc_ipc_send_message(a, &header, &payload), EMSGSIZE); OK(close(file)); cc_ipc_close(a); OK(close(peer));
    size_t size = 99; uint32_t count = 99; CHECK(!cc_ipc_packet_data(NULL, &size) && !size);
    CHECK(!cc_ipc_packet_fds(NULL, &count) && !count); ERROR(cc_ipc_packet_take_fd(NULL, 0), EINVAL);
    cc_ipc_packet_destroy(NULL); cc_ipc_close(NULL); ERROR(cc_ipc_flush(NULL), EINVAL); ERROR(cc_ipc_fd(NULL), EINVAL);
}
int main(int argc, char **argv)
{
    size_t before = descriptors();
    if (argc == 2 && !strcmp(argv[1], "malformed")) { arguments(); malformed(); }
    else { wire_vectors(); bidirectional(); partial_sends(); kernel_backpressure(); process_echo(); fd_zero(); }
    CHECK(descriptors() == before);
    puts("IPC wire compatibility, nonblocking ownership, native descriptor passing and cleanup passed");
    return 0;
}
