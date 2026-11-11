/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_proxy.h"
#include "ipa_vimc.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int descriptor(const char *text, bool optional)
{
    if (optional && !strcmp(text, "-1")) return -1;
    errno = 0; char *end; long value = strtol(text, &end, 10);
    if (errno || !text[0] || *end || value < 0 || value > INT_MAX) return -2;
    int fd = (int)value;
    if (fcntl(fd, F_SETFD, FD_CLOEXEC)) return -2;
    return fd;
}
static uint64_t current_ms(void)
{
    struct timespec time;
    if (clock_gettime(CLOCK_MONOTONIC, &time)) return UINT64_MAX;
    return (uint64_t)time.tv_sec * 1000U + (uint64_t)time.tv_nsec / 1000000U;
}
static int flush_reply(cc_ipc_pipe *pipe)
{
    uint64_t start = current_ms(); if (start == UINT64_MAX) return -EIO;
    for (;;) {
        int ret = cc_ipc_pipe_dispatch(pipe); if (ret < 0) return ret;
        struct pollfd fds[2] = {{.fd = cc_ipc_pipe_fd(pipe)}, {.fd = cc_ipc_pipe_peer_fd(pipe), .events = POLLIN}};
        ret = cc_ipc_pipe_events(pipe, &fds[0].events); if (ret < 0) return ret;
        if (!(fds[0].events & POLLOUT)) return 0;
        uint64_t now = current_ms(); if (now == UINT64_MAX || now - start >= 2000U) return -ETIMEDOUT;
        ret = poll(fds, 2, (int)(2000U - (now - start)));
        if (ret < 0) { if (errno == EINTR) continue; return -errno; }
        if (!ret) return -ETIMEDOUT;
    }
}
static int respond_fds(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, int status, const void *data, size_t size,
                         const int *fds, uint32_t num_fds)
{
    if (status > 0 || status < -4095) status = -EPROTO;
    if (status) { data = NULL; size = 0; num_fds = 0; }
    if (size > CC_IPA_WIRE_MAX_DATA - sizeof(int32_t)) { status = -E2BIG; data = NULL; size = 0; num_fds = 0; }
    unsigned char *bytes = malloc(sizeof(int32_t) + size); if (!bytes) return -ENOMEM;
    int32_t result = status; memcpy(bytes, &result, sizeof(result));
    if (size) memcpy(bytes + sizeof(result), data, size);
    int ret = cc_ipa_message_send_fds(pipe, header, bytes, sizeof(result) + size, fds, num_fds); free(bytes);
    return ret < 0 ? ret : flush_reply(pipe);
}
static int respond(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, int status, const void *data, size_t size)
{
    return respond_fds(pipe, header, status, data, size, NULL, 0);
}
static int initialise(const void *data, size_t size, int module_fd, const char *path, int trace_fd,
                        cc_ipa **ipa, struct cc_ipa_vimc **vimc, uint32_t *flags)
{
    if (size < sizeof(struct cc_ipa_proxy_init)) return -EPROTO;
    struct cc_ipa_proxy_init init; memcpy(&init, data, sizeof(init));
    if (init.protocol != CC_IPA_WIRE_VERSION || init.abi != CC_IPA_ABI_VERSION) return -EPROTONOSUPPORT;
    const unsigned char *bytes = (const unsigned char *)data + sizeof(init); size -= sizeof(init);
    if (init.kind == CC_IPA_PROXY_MODULE) {
        if (module_fd < 0 || size < sizeof(struct cc_ipa_module_transfer)) return -EPROTO;
        struct cc_ipa_module_transfer transfer; memcpy(&transfer, bytes, sizeof(transfer));
        cc_ipa_module *module = NULL; int ret = cc_ipa_module_import(path, module_fd, &transfer, &module);
        if (!ret) ret = cc_ipa_module_create(module, bytes + sizeof(transfer), size - sizeof(transfer), ipa);
        cc_ipa_module_close(module); return ret;
    }
    if (module_fd >= 0) return -EPROTO;
    if (init.kind == CC_IPA_PROXY_VIMC) {
        if (size <= sizeof(struct cc_ipa_vimc_init)) return -EINVAL;
        struct cc_ipa_vimc_init options; memcpy(&options, bytes, sizeof(options));
        cc_ipa_vimc_options controller = {.configuration = bytes + sizeof(options), .configuration_size = size - sizeof(options),
            .trace_fd = trace_fd, .operation = (enum cc_ipa_vimc_operation)options.operation, .flags = options.flags};
        return cc_ipa_vimc_create(&controller, ipa, vimc, flags);
    }
    if (trace_fd >= 0) return -EPROTO;
    if (init.kind == CC_IPA_PROXY_VC4_DEFAULT) return size ? -EPROTO : cc_ipa_vc4_create(NULL, ipa);
    enum cc_ipa_wire_kind kind = init.kind == CC_IPA_PROXY_REFERENCE ? CC_IPA_WIRE_REFERENCE : CC_IPA_WIRE_VC4;
    if (init.kind != CC_IPA_PROXY_REFERENCE && init.kind != CC_IPA_PROXY_VC4) return -EPROTO;
    cc_ipa_wire *wire = NULL; int ret = cc_ipa_wire_decode(kind, bytes, size, &wire);
    if (!ret) ret = init.kind == CC_IPA_PROXY_REFERENCE ? cc_ipa_reference_create(cc_ipa_wire_object(wire), ipa) :
                                                        cc_ipa_vc4_create(cc_ipa_wire_object(wire), ipa);
    cc_ipa_wire_destroy(wire); return ret;
}
static int vimc_map(struct cc_ipa_vimc *vimc, const cc_ipc_packet *packet)
{
    size_t size; const unsigned char *bytes = cc_ipc_packet_data(packet, &size);
    if (size < sizeof(cc_ipc_message_header) + sizeof(uint32_t)) return -EPROTO;
    bytes += sizeof(cc_ipc_message_header); size -= sizeof(cc_ipc_message_header);
    uint32_t count; memcpy(&count, bytes, sizeof(count)); bytes += sizeof(count); size -= sizeof(count);
    if (count > CC_IPA_VIMC_MAX_BUFFERS) return -EPROTO;
    uint32_t num_fds, used = 0; const int *fds = cc_ipc_packet_fds(packet, &num_fds);
    cc_ipa_vimc_buffer buffers[CC_IPA_VIMC_MAX_BUFFERS] = {0};
    for (size_t i = 0; i < count; ++i) {
        struct cc_ipa_vimc_map_header header;
        if (size < sizeof(header)) return -EPROTO;
        memcpy(&header, bytes, sizeof(header)); bytes += sizeof(header); size -= sizeof(header);
        if (!header.num_planes || header.num_planes > CC_MAX_PLANES || header.num_planes > num_fds - used) return -EPROTO;
        buffers[i].id = header.id; buffers[i].num_planes = header.num_planes;
        for (size_t j = 0; j < header.num_planes; ++j) {
            struct cc_ipa_vimc_map_plane plane;
            if (size < sizeof(plane)) return -EPROTO;
            memcpy(&plane, bytes, sizeof(plane)); bytes += sizeof(plane); size -= sizeof(plane);
            buffers[i].planes[j] = (cc_ipa_vimc_plane){fds[used++], plane.offset, plane.length};
        }
    }
    return size || used != num_fds ? -EPROTO : cc_ipa_vimc_map(vimc, buffers, count);
}
static int vimc_command(struct cc_ipa_vimc *vimc, cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const cc_ipc_packet *packet)
{
    if (header->command == CC_IPA_PROXY_VIMC_MAP) return respond(pipe, header, vimc_map(vimc, packet), NULL, 0);
    void *data = NULL; size_t size = 0; int ret = cc_ipa_message_decode(packet, &data, &size);
    if (ret < 0) return ret;
    if (header->command == CC_IPA_PROXY_VIMC_UNMAP) {
        uint32_t ids[CC_IPA_VIMC_MAX_BUFFERS + 1U];
        if (size < sizeof(uint32_t) || size > sizeof(ids)) ret = -EPROTO;
        else {
            memcpy(ids, data, size);
            if (ids[0] > CC_IPA_VIMC_MAX_BUFFERS || size != sizeof(uint32_t) * (1U + ids[0])) ret = -EPROTO;
            else cc_ipa_vimc_unmap(vimc, ids + 1, ids[0]);
        }
        ret = respond(pipe, header, ret, NULL, 0);
    } else if (header->command == CC_IPA_PROXY_VIMC_QUEUE) {
        cc_ipa_wire *wire = NULL; ret = cc_ipa_wire_decode(CC_IPA_WIRE_REQUEST, data, size, &wire); cc_ipa_wire_destroy(wire);
    } else if (header->command == CC_IPA_PROXY_VIMC_COMPUTE) {
        uint32_t arguments[2];
        if (size != sizeof(arguments)) ret = -EPROTO;
        else {
            memcpy(arguments, data, size); cc_ipa_vimc_event event;
            ret = cc_ipa_vimc_compute(vimc, arguments[1], &event);
            if (!ret) {
                cc_ipc_message_header notification = {CC_IPA_PROXY_VIMC_PARAMS, 0};
                ret = cc_ipa_message_send(pipe, &notification, &event, sizeof(event));
                if (!ret) ret = flush_reply(pipe);
            } else if (ret == -ENOENT) ret = 0;
        }
    } else ret = -EPROTO;
    free(data); return ret;
}
static int invoke(cc_ipa *ipa, uint32_t command, const void *data, size_t size, const int *input_fds, uint32_t num_input_fds,
                     void **output, size_t *output_size, int output_fds[CC_IPA_WIRE_MAX_FDS], uint32_t *num_output_fds)
{
    *output = NULL; *output_size = 0; *num_output_fds = 0;
    if (command == CC_IPA_PROXY_START || command == CC_IPA_PROXY_STOP || command == CC_IPA_PROXY_DESTROY) {
        if (size || num_input_fds) return -EPROTO;
        if (command == CC_IPA_PROXY_START) return cc_ipa_start(ipa);
        if (command == CC_IPA_PROXY_STOP) cc_ipa_stop(ipa);
        else cc_ipa_destroy(ipa);
        return 0;
    }
    enum cc_ipa_wire_kind kind;
    switch (command) {
    case CC_IPA_PROXY_CONFIGURE: kind = CC_IPA_WIRE_CONFIGURATION; break;
    case CC_IPA_PROXY_PREPARE: kind = CC_IPA_WIRE_REQUEST; break;
    case CC_IPA_PROXY_PROCESS: kind = CC_IPA_WIRE_FRAME; break;
    default: return -ENOSYS;
    }
    cc_ipa_wire *wire = NULL; int ret = cc_ipa_wire_decode_fds(kind, data, size, input_fds, num_input_fds, &wire);
    if (ret < 0) return ret;
    if (command == CC_IPA_PROXY_CONFIGURE) ret = cc_ipa_configure(ipa, cc_ipa_wire_object(wire));
    else if (command == CC_IPA_PROXY_PREPARE) {
        cc_ipa_settings settings = {0}; ret = cc_ipa_prepare(ipa, cc_ipa_wire_object(wire), &settings);
        if (!ret) ret = cc_ipa_wire_encode_fds(CC_IPA_WIRE_SETTINGS, &settings, output, output_size, output_fds, num_output_fds);
    } else {
        cc_ipa_metadata metadata = {0}; ret = cc_ipa_process(ipa, cc_ipa_wire_object(wire), &metadata);
        if (!ret) ret = cc_ipa_wire_encode_fds(CC_IPA_WIRE_METADATA, &metadata, output, output_size, output_fds, num_output_fds);
    }
    cc_ipa_wire_destroy(wire); return ret;
}
int main(int argc, char **argv)
{
    if (argc != 6) return 2;
    struct sigaction ignore = {.sa_handler = SIG_IGN}; sigemptyset(&ignore.sa_mask);
    if (sigaction(SIGPIPE, &ignore, NULL)) return 2;
    int socket = descriptor(argv[1], false), parent = descriptor(argv[2], false), module = descriptor(argv[3], true);
    int trace_fd = descriptor(argv[5], true);
    if (socket < 0 || parent < 0 || module < -1 || trace_fd < -1 || socket == parent || socket == module || parent == module ||
        (trace_fd >= 0 && (trace_fd == socket || trace_fd == parent || trace_fd == module))) return 2;
    cc_ipc *channel = NULL; int ret = cc_ipc_bind(socket, &channel); close(socket);
    if (ret < 0) return 3;
    cc_ipc_pipe *pipe = NULL; ret = cc_ipc_pipe_open(&channel, parent, &pipe); close(parent);
    if (ret < 0) { cc_ipc_close(channel); return 3; }
    cc_ipa *ipa = NULL; struct cc_ipa_vimc *vimc = NULL; bool ready = false; int exit_code = 0;
    for (;;) {
        cc_ipc_packet *packet = NULL; ret = cc_ipc_pipe_receive(pipe, &packet, -1);
        if (ret < 0) { exit_code = ret == -EPIPE ? 0 : 4; break; }
        cc_ipc_message_header header; ret = cc_ipc_packet_header(packet, &header);
        if (!ret && vimc && header.command >= CC_IPA_PROXY_VIMC_MAP) {
            ret = vimc_command(vimc, pipe, &header, packet); cc_ipc_packet_destroy(packet);
            if (ret < 0) { exit_code = 4; break; }
            continue;
        }
        void *input = NULL, *output = NULL; size_t input_size = 0, output_size = 0;
        const int *input_fds = NULL; uint32_t num_input_fds = 0;
        if (!ret) ret = cc_ipa_message_decode_fds(packet, &input, &input_size, &input_fds, &num_input_fds);
        if (!ret && !ready && num_input_fds) ret = -EPROTO;
        if (ret < 0) { cc_ipc_packet_destroy(packet); free(input); exit_code = 4; break; }
        if (!ready) {
            uint32_t vimc_flags = 0;
            ret = header.command == CC_IPA_PROXY_INIT ? initialise(input, input_size, module, argv[4], trace_fd, &ipa, &vimc, &vimc_flags) : -EPROTO;
            if (module >= 0) { close(module); module = -1; }
            if (trace_fd >= 0) { close(trace_fd); trace_fd = -1; }
            uint32_t capabilities[] = {cc_ipa_get_capabilities(ipa), vimc_flags};
            int sent = respond(pipe, &header, ret, capabilities, vimc ? sizeof(capabilities) : sizeof(capabilities[0]));
            cc_ipc_packet_destroy(packet); free(input);
            /* The parent can exit immediately after reading the reply, before
             * flush_reply finishes its peer check. Treat that disconnect like
             * the normal -EPIPE receive path above. */
            if (ret < 0 || sent < 0) { exit_code = sent == -EPIPE ? 0 : 5; break; }
            ready = true; continue;
        }
        int output_fds[CC_IPA_WIRE_MAX_FDS]; uint32_t num_output_fds = 0;
        ret = invoke(ipa, header.command, input, input_size, input_fds, num_input_fds, &output, &output_size, output_fds, &num_output_fds);
        cc_ipc_packet_destroy(packet); free(input);
        bool destroyed = header.command == CC_IPA_PROXY_DESTROY && !ret;
        if (destroyed) ipa = NULL;
        int sent = respond_fds(pipe, &header, ret, output, output_size, output_fds, num_output_fds); free(output);
        if (sent < 0) { exit_code = sent == -EPIPE ? 0 : 4; break; }
        if (destroyed) break;
    }
    if (module >= 0) close(module);
    if (trace_fd >= 0) close(trace_fd);
    cc_ipa_destroy(ipa); cc_ipc_pipe_close(pipe); return exit_code;
}
