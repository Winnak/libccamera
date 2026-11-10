/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipc.h"
#include <ccamera/ipc_pipe.h>
#include <ccamera/process.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) do { int actual_error = (x); if (actual_error != -(e)) fprintf(stderr, "%s: expected %d, got %d\n", #x, -(e), actual_error); CHECK(actual_error == -(e)); } while (0)
static const char *self;
static size_t descriptors(void)
{
    DIR *directory = opendir("/proc/self/fd"); CHECK(directory); size_t count = 0; struct dirent *entry;
    while ((entry = readdir(directory))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++count;
    OK(closedir(directory)); return count;
}
static uint64_t current_ms(void)
{
    struct timespec now; OK(clock_gettime(CLOCK_MONOTONIC, &now));
    return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
}
static void flush(cc_ipc_pipe *pipe)
{
    for (;;) {
        short events; OK(cc_ipc_pipe_dispatch(pipe)); OK(cc_ipc_pipe_events(pipe, &events));
        if (!(events & POLLOUT)) return;
        struct pollfd fd = {.fd = cc_ipc_pipe_fd(pipe), .events = events}; CHECK(poll(&fd, 1, 2000) > 0);
    }
}
static void send_packet(cc_ipc_pipe *pipe, cc_ipc_message_header header, cc_ipc_payload payload)
{
    OK(cc_ipc_pipe_send(pipe, &header, &payload)); flush(pipe);
}
static int child(int fd, const char *mode)
{
    cc_ipc *channel = NULL; OK(cc_ipc_bind(fd, &channel)); OK(close(fd));
    cc_ipc_pipe *pipe = NULL; OK(cc_ipc_pipe_open(&channel, -1, &pipe));
    cc_ipc_packet *packet = NULL; OK(cc_ipc_pipe_receive(pipe, &packet, 2000));
    if (!strcmp(mode, "async-first")) {
        cc_ipc_message_header prelude; OK(cc_ipc_packet_header(packet, &prelude)); CHECK(prelude.command == 99);
        cc_ipc_packet_destroy(packet); OK(cc_ipc_pipe_receive(pipe, &packet, 2000));
    }
    cc_ipc_message_header header; OK(cc_ipc_packet_header(packet, &header));
    if (!strcmp(mode, "exit")) { cc_ipc_packet_destroy(packet); cc_ipc_pipe_close(pipe); return 9; }
    if (!strcmp(mode, "idle")) { for (;;) pause(); }
    if (!strcmp(mode, "malformed")) {
        /* Valid raw framing, too short for IPCMessage. */
        uint8_t raw_header[8] = {0}; uint8_t value = 0; uint32_t length = 1;
        memcpy(raw_header, &length, sizeof(length));
        CHECK(send(cc_ipc_pipe_fd(pipe), raw_header, sizeof(raw_header), 0) == sizeof(raw_header));
        CHECK(send(cc_ipc_pipe_fd(pipe), &value, sizeof(value), 0) == sizeof(value));
    } else {
        if (!strcmp(mode, "notify")) {
            cc_ipc_message_header notice = {.command = 700, .cookie = header.cookie + 1U};
            const char text[] = "notification"; int pipes[2]; OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK));
            send_packet(pipe, notice, (cc_ipc_payload){text, sizeof(text), &pipes[1], 1});
            OK(close(pipes[1]));
            OK(close(pipes[0]));
        }
        size_t size = 0; const unsigned char *data = cc_ipc_packet_data(packet, &size);
        uint32_t count = 0; const int *fds = cc_ipc_packet_fds(packet, &count);
        cc_ipc_payload payload = {data + sizeof(header), size - sizeof(header), fds, count};
        send_packet(pipe, header, payload);
    }
    cc_ipc_packet_destroy(packet); cc_ipc_pipe_close(pipe); return 0;
}
static int spawn(const char *mode, cc_ipc_pipe **pipe, cc_process **process)
{
    cc_ipc *channel = NULL; int fd; OK(cc_ipc_create(&channel, &fd));
    char number[24]; CHECK(snprintf(number, sizeof(number), "%d", fd) > 0);
    const char *arguments[] = {"--child", number, mode};
    cc_process_options options = {.path = self, .arguments = arguments, .num_arguments = 3,
        .fds = &fd, .num_fds = 1, .flags = CC_PROCESS_PARENT_DEATH};
    int ret = cc_process_spawn(&options, process); OK(close(fd));
    if (ret < 0) { cc_ipc_close(channel); return ret; }
    OK(cc_process_wait_exec(*process, 2000));
    OK(cc_ipc_pipe_open(&channel, cc_process_fd(*process), pipe)); CHECK(!channel);
    CHECK(cc_ipc_pipe_peer_fd(*pipe) != cc_process_fd(*process));
    CHECK(fcntl(cc_ipc_pipe_peer_fd(*pipe), F_GETFD) & FD_CLOEXEC); return 0;
}
static void finish(cc_ipc_pipe *pipe, cc_process *process, int code)
{
    cc_process_result result; OK(cc_process_wait(process, &result, 2000));
    CHECK(result.status == CC_PROCESS_EXITED && result.exit_code == code);
    cc_ipc_pipe_close(pipe); OK(cc_process_close(process));
}
static void echo(bool notifications, uint32_t cookie)
{
    cc_ipc_pipe *pipe = NULL; cc_process *process = NULL; OK(spawn(notifications ? "notify" : "echo", &pipe, &process));
    cc_ipc_message_header header = {12, cookie}; const char text[] = "RPC payload \xc3\xb8";
    int pipes[2]; OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK));
    cc_ipc_payload payload = {text, sizeof(text), &pipes[1], 1}; cc_ipc_packet *reply = NULL;
    OK(cc_ipc_pipe_call(pipe, &header, &payload, &reply, CC_IPC_PIPE_DEFAULT_TIMEOUT_MS)); OK(close(pipes[1]));
    CHECK(reply); cc_ipc_message_header response; OK(cc_ipc_packet_header(reply, &response));
    CHECK(response.command == header.command && response.cookie == cookie);
    size_t size; const unsigned char *data = cc_ipc_packet_data(reply, &size);
    CHECK(size == sizeof(header) + sizeof(text) && !memcmp(data + sizeof(header), text, sizeof(text)));
    uint32_t count; const int *fds = cc_ipc_packet_fds(reply, &count); CHECK(count == 1 && (fcntl(fds[0], F_GETFD) & FD_CLOEXEC));
    char value; CHECK(read(pipes[0], &value, 1) < 0 && errno == EAGAIN);
    if (notifications) {
        cc_ipc_packet *notice = NULL; CHECK(cc_ipc_pipe_pending(pipe) == 1);
        OK(cc_ipc_pipe_receive(pipe, &notice, 0)); OK(cc_ipc_packet_header(notice, &response));
        CHECK(response.command == 700 && response.cookie == cookie + 1U);
        cc_ipc_packet_destroy(notice);
    }
    finish(pipe, process, 0); /* Reply owns its fds independently of pipe/process. */
    CHECK(read(pipes[0], &value, 1) < 0 && errno == EAGAIN);
    cc_ipc_packet_destroy(reply); CHECK(read(pipes[0], &value, 1) == 0); OK(close(pipes[0]));
}
static void failures(void)
{
    const cc_ipc_message_header header = {1, 2}; const cc_ipc_payload empty = {0};
    cc_ipc_pipe *pipe = NULL; cc_process *process = NULL; cc_ipc_packet *reply = (void *)(uintptr_t)1;
    OK(spawn("exit", &pipe, &process)); uint64_t start = current_ms();
    ERROR(cc_ipc_pipe_call(pipe, &header, &empty, &reply, 2000), EPIPE); CHECK(!reply && current_ms() - start < 1500);
    ERROR(cc_ipc_pipe_fd(pipe), EPIPE); finish(pipe, process, 9);
    OK(spawn("idle", &pipe, &process)); start = current_ms();
    ERROR(cc_ipc_pipe_call(pipe, &header, &empty, &reply, 20), ETIMEDOUT);
    CHECK(!reply && current_ms() - start >= 15 && current_ms() - start < 1500);
    ERROR(cc_ipc_pipe_send(pipe, &header, &empty), ETIMEDOUT); cc_ipc_pipe_close(pipe); OK(cc_process_close(process));
    OK(spawn("malformed", &pipe, &process));
    ERROR(cc_ipc_pipe_call(pipe, &header, &empty, &reply, 2000), EPROTO); CHECK(!reply); finish(pipe, process, 0);
    OK(spawn("echo", &pipe, &process)); OK(cc_ipc_pipe_call(pipe, &header, &empty, NULL, 2000)); finish(pipe, process, 0);
}
static void local_pair(cc_ipc_pipe **a, cc_ipc_pipe **b)
{
    cc_ipc *first = NULL, *second = NULL; int remote; OK(cc_ipc_create(&first, &remote));
    OK(cc_ipc_bind(remote, &second)); OK(close(remote));
    OK(cc_ipc_pipe_open(&first, -1, a)); OK(cc_ipc_pipe_open(&second, -1, b));
}
static void local(void)
{
    cc_ipc_pipe *a = NULL, *b = NULL; local_pair(&a, &b); ERROR(cc_ipc_pipe_peer_fd(a), ENODATA);
    cc_ipc_packet *packet = (void *)(uintptr_t)1; ERROR(cc_ipc_pipe_receive(a, &packet, 0), EAGAIN); CHECK(!packet);
    ERROR(cc_ipc_pipe_receive(a, &packet, 2), ETIMEDOUT);
    cc_ipc_message_header header = {17, 19}; cc_ipc_payload empty = {0}; send_packet(a, header, empty);
    OK(cc_ipc_pipe_receive(b, &packet, 0)); cc_ipc_packet_destroy(packet);
    cc_ipc_pipe_close(a); cc_ipc_pipe_close(b);
    /* Notification ring wraparound, full-message queue and fatal overflow. */
    local_pair(&a, &b);
    for (uint32_t i = 0; i < 2U * CC_IPC_PIPE_MAX_PENDING; ++i) {
        header.cookie = i; send_packet(a, header, empty); OK(cc_ipc_pipe_receive(b, &packet, 0));
        cc_ipc_message_header actual; OK(cc_ipc_packet_header(packet, &actual)); CHECK(actual.cookie == i); cc_ipc_packet_destroy(packet);
    }
    for (uint32_t i = 0; i < CC_IPC_PIPE_MAX_PENDING; ++i) { send_packet(a, header, empty); OK(cc_ipc_pipe_dispatch(b)); }
    CHECK(cc_ipc_pipe_pending(b) == CC_IPC_PIPE_MAX_PENDING); send_packet(a, header, empty);
    ERROR(cc_ipc_pipe_dispatch(b), ENOBUFS);
    for (uint32_t i = 0; i < CC_IPC_PIPE_MAX_PENDING; ++i) { OK(cc_ipc_pipe_receive(b, &packet, 0)); cc_ipc_packet_destroy(packet); }
    ERROR(cc_ipc_pipe_receive(b, &packet, 0), ENOBUFS); cc_ipc_pipe_close(a); cc_ipc_pipe_close(b);
}
static void arguments(void)
{
    cc_ipc_pipe *pipe = (void *)(uintptr_t)1; cc_ipc_packet *packet = pipe ? (void *)(uintptr_t)1 : NULL;
    cc_ipc_message_header header = {0}; cc_ipc_payload empty = {0}; short events;
    ERROR(cc_ipc_pipe_open(NULL, -1, &pipe), EINVAL); CHECK(!pipe); ERROR(cc_ipc_pipe_fd(NULL), EINVAL);
    ERROR(cc_ipc_pipe_peer_fd(NULL), EINVAL); ERROR(cc_ipc_pipe_events(NULL, &events), EINVAL);
    CHECK(!cc_ipc_pipe_pending(NULL)); ERROR(cc_ipc_pipe_dispatch(NULL), EINVAL);
    ERROR(cc_ipc_pipe_send(NULL, &header, &empty), EINVAL); ERROR(cc_ipc_pipe_receive(NULL, &packet, 0), EINVAL); CHECK(!packet);
    ERROR(cc_ipc_pipe_call(NULL, &header, &empty, &packet, 0), EINVAL); CHECK(!packet); cc_ipc_pipe_close(NULL);
    cc_ipc *channel = NULL; int remote; OK(cc_ipc_create(&channel, &remote));
    ERROR(cc_ipc_pipe_open(&channel, -2, &pipe), EINVAL); CHECK(channel && !pipe);
    ERROR(cc_ipc_pipe_open(&channel, 999999, &pipe), EBADF); CHECK(channel && !pipe);
    cc_ipc_close(channel); OK(close(remote));
}
struct send_fault { unsigned sent, at, remaining; bool permanent; };
static ssize_t fault_send(void *context, int fd, const struct msghdr *message, int flags)
{
    struct send_fault *fault = context;
    if (fault->sent == fault->at && (fault->permanent || fault->remaining)) {
        if (fault->remaining) --fault->remaining;
        return -EAGAIN;
    }
    ssize_t size; do { size = sendmsg(fd, message, flags); } while (size < 0 && errno == EINTR);
    if (size >= 0) ++fault->sent;
    return size < 0 ? -errno : size;
}
static ssize_t fault_receive(void *context, int fd, struct msghdr *message, int flags)
{
    (void)context; ssize_t size; do { size = recvmsg(fd, message, flags); } while (size < 0 && errno == EINTR);
    return size < 0 ? -errno : size;
}
static const struct cc_ipc_io fault_io = {fault_send, fault_receive};
static void backpressure(void)
{
    for (unsigned stage = 0; stage < 2; ++stage) {
        cc_ipc *channel = NULL; int remote; OK(cc_ipc_create(&channel, &remote));
        char number[24]; CHECK(snprintf(number, sizeof(number), "%d", remote) > 0);
        const char *args[] = {"--child", number, "async-first"};
        cc_process_options options = {.path = self, .arguments = args, .num_arguments = 3,
            .fds = &remote, .num_fds = 1, .flags = CC_PROCESS_PARENT_DEATH};
        cc_process *process = NULL; OK(cc_process_spawn(&options, &process)); OK(close(remote));
        OK(cc_process_wait_exec(process, 2000));
        struct send_fault fault = {.at = stage, .remaining = 4}; cc_ipc_set_io(channel, &fault_io, &fault);
        cc_ipc_pipe *pipe = NULL; OK(cc_ipc_pipe_open(&channel, cc_process_fd(process), &pipe));
        cc_ipc_message_header prelude = {99, 7}, request = {100, 8}; cc_ipc_payload empty = {0};
        OK(cc_ipc_pipe_send(pipe, &prelude, &empty)); short events; OK(cc_ipc_pipe_events(pipe, &events)); CHECK(events & POLLOUT);
        ERROR(cc_ipc_pipe_send(pipe, &request, &empty), EBUSY);
        cc_ipc_packet *packet = NULL; OK(cc_ipc_pipe_call(pipe, &request, &empty, &packet, 2000));
        cc_ipc_message_header actual; OK(cc_ipc_packet_header(packet, &actual)); CHECK(actual.cookie == request.cookie);
        CHECK(fault.sent == 4); cc_ipc_packet_destroy(packet); finish(pipe, process, 0);
    }
    /* A timed-out request with a transmitted header must discard its pending
     * payload/fds and fail the pipe, even if POLLOUT remains spuriously ready. */
    cc_ipc *channel = NULL, *peer = NULL; int remote; OK(cc_ipc_create(&channel, &remote));
    OK(cc_ipc_bind(remote, &peer)); OK(close(remote));
    struct send_fault fault = {.at = 1, .permanent = true}; cc_ipc_set_io(channel, &fault_io, &fault);
    cc_ipc_pipe *pipe = NULL; OK(cc_ipc_pipe_open(&channel, -1, &pipe));
    int fds[2]; OK(pipe2(fds, O_CLOEXEC | O_NONBLOCK));
    cc_ipc_message_header header = {1, 2}; cc_ipc_payload payload = {.fds = &fds[1], .num_fds = 1};
    cc_ipc_packet *packet = NULL; ERROR(cc_ipc_pipe_call(pipe, &header, &payload, &packet, 2), ETIMEDOUT);
    CHECK(!packet && fault.sent == 1); OK(close(fds[1])); char value; CHECK(read(fds[0], &value, 1) == 0);
    OK(close(fds[0])); cc_ipc_close(peer); cc_ipc_pipe_close(pipe);
}
static void queue_limits(void)
{
    cc_ipc_pipe *a = NULL, *b = NULL; local_pair(&a, &b);
    cc_ipc_message_header header = {11, 22};
    unsigned char *bytes = calloc(1, CC_IPC_MAX_DATA - sizeof(header)); CHECK(bytes);
    cc_ipc_payload payload = {.data = bytes, .size = CC_IPC_MAX_DATA - sizeof(header)};
    for (unsigned i = 0; i < CC_IPC_PIPE_MAX_PENDING_BYTES / CC_IPC_MAX_DATA; ++i) {
        send_packet(a, header, payload); OK(cc_ipc_pipe_dispatch(b));
    }
    CHECK(cc_ipc_pipe_pending(b) == CC_IPC_PIPE_MAX_PENDING_BYTES / CC_IPC_MAX_DATA);
    send_packet(a, header, payload); ERROR(cc_ipc_pipe_dispatch(b), ENOBUFS);
    cc_ipc_pipe_close(a); cc_ipc_pipe_close(b); free(bytes);
    local_pair(&a, &b); int pipes[2]; OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK));
    int fds[CC_IPC_MAX_FDS]; for (unsigned i = 0; i < CC_IPC_MAX_FDS; ++i) fds[i] = pipes[1];
    payload = (cc_ipc_payload){.fds = fds, .num_fds = CC_IPC_MAX_FDS};
    send_packet(a, header, payload); OK(cc_ipc_pipe_dispatch(b));
    payload.num_fds = CC_IPC_PIPE_MAX_PENDING_FDS - CC_IPC_MAX_FDS;
    send_packet(a, header, payload); OK(cc_ipc_pipe_dispatch(b));
    payload.num_fds = 1; send_packet(a, header, payload); ERROR(cc_ipc_pipe_dispatch(b), ENOBUFS);
    OK(close(pipes[1])); char value; CHECK(read(pipes[0], &value, 1) < 0 && errno == EAGAIN);
    cc_ipc_pipe_close(b); CHECK(read(pipes[0], &value, 1) == 0); OK(close(pipes[0])); cc_ipc_pipe_close(a);
}
int main(int argc, char **argv)
{
    self = argv[0];
    if (argc == 4 && !strcmp(argv[1], "--child")) return child(atoi(argv[2]), argv[3]);
    CHECK(argc == 1); size_t before = descriptors(); arguments(); local(); queue_limits();
    cc_ipc_pipe *pipe = NULL; cc_process *process = NULL; int ret = spawn("echo", &pipe, &process);
    if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) { fprintf(stderr, "clone3 unavailable: %s\n", strerror(-ret)); return 77; }
    OK(ret); cc_ipc_message_header header = {0}; cc_ipc_payload empty = {0};
    OK(cc_ipc_pipe_call(pipe, &header, &empty, NULL, -1)); finish(pipe, process, 0);
    echo(false, 0); echo(true, UINT32_MAX); failures(); backpressure(); CHECK(descriptors() == before);
    int status; CHECK(waitpid(-1, &status, WNOHANG) < 0 && errno == ECHILD);
    puts("IPC RPC cookies, notifications, deadlines, peer exit and ownership checks passed"); return 0;
}
