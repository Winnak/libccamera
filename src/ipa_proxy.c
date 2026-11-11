/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_proxy.h"
#include "ipa_vimc.h"
#include "process_compat.h"
#include "source_paths.h"
#include <ccamera/ipa_proxy.h>
#include <ccamera/process.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/auxv.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

struct proxy {
    cc_process *process;
    cc_ipc_pipe *pipe;
    cc_ipa_wire *held;
    int timeout, error;
    uint32_t cookie;
    enum cc_ipa_proxy_kind kind;
    uint32_t vimc_ids[CC_IPA_VIMC_MAX_BUFFERS], expected_params;
    size_t num_vimc_ids;
    cc_ipa_vimc_event events[CC_IPC_PIPE_MAX_PENDING];
    size_t event_head, num_events;
};
struct wire_handles { cc_ipc_packet *packet; const int *fds; uint32_t count; };
static int map_vimc(struct proxy *, const cc_ipa_vimc_buffer *, size_t);
static int unmap_vimc(struct proxy *, const uint32_t *, size_t);
static int queue_vimc(struct proxy *, const cc_ipa_request *);
static int compute_vimc(struct proxy *, uint32_t, uint32_t);
static int broken(struct proxy *proxy, int error)
{
    if (!proxy->error) proxy->error = error;
    cc_ipc_pipe_close(proxy->pipe); proxy->pipe = NULL;
    if (proxy->process) (void)cc_process_signal(proxy->process, SIGKILL);
    return proxy->error;
}
static void clear_held(struct proxy *proxy) { cc_ipa_wire_destroy(proxy->held); proxy->held = NULL; }
static uint32_t next_cookie(struct proxy *proxy)
{
    if (!++proxy->cookie) ++proxy->cookie; /* Zero is reserved for VIMC notifications. */
    return proxy->cookie;
}
static int event_packet(const cc_ipc_packet *packet, cc_ipa_vimc_event *event)
{
    cc_ipc_message_header header; int ret = cc_ipc_packet_header(packet, &header);
    if (ret < 0 || header.command != CC_IPA_PROXY_VIMC_PARAMS || header.cookie) return -EPROTO;
    void *data = NULL; size_t size = 0; ret = cc_ipa_message_decode(packet, &data, &size);
    if (!ret) {
        if (size != sizeof(*event)) ret = -EPROTO;
        else { memcpy(event, data, sizeof(*event)); if (!event->buffer_id || event->flags) ret = -EPROTO; }
    }
    free(data); return ret;
}
static int collect_response(struct proxy *proxy, const cc_ipc_message_header *header, cc_ipc_packet *packet,
                              void **out, size_t *out_size, struct wire_handles *handles)
{
    cc_ipc_message_header reply; int ret = cc_ipc_packet_header(packet, &reply);
    void *bytes = NULL; size_t length = 0;
    if (!ret && (reply.command != header->command || reply.cookie != header->cookie)) ret = -EPROTO;
    while (!ret && proxy->kind == CC_IPA_PROXY_VIMC && cc_ipc_pipe_pending(proxy->pipe)) {
        cc_ipc_packet *notification = NULL;
        ret = cc_ipc_pipe_receive(proxy->pipe, &notification, 0);
        cc_ipa_vimc_event event = {0};
        if (!ret) ret = event_packet(notification, &event);
        cc_ipc_packet_destroy(notification);
        if (!ret && proxy->expected_params && event.buffer_id == proxy->expected_params) proxy->expected_params = 0;
        else if (!ret) {
            if (proxy->num_events == CC_IPC_PIPE_MAX_PENDING) ret = -ENOBUFS;
            else proxy->events[(proxy->event_head + proxy->num_events++) % CC_IPC_PIPE_MAX_PENDING] = event;
        }
    }
    if (!ret && proxy->expected_params) ret = -EPROTO;
    if (!ret && cc_ipc_pipe_pending(proxy->pipe)) ret = -EPROTO;
    const int *fds = NULL; uint32_t count = 0;
    if (!ret) ret = cc_ipa_message_decode_fds(packet, &bytes, &length, &fds, &count);
    if (!ret && count && !handles) ret = -EPROTO;
    if (ret < 0) { cc_ipc_packet_destroy(packet); free(bytes); return broken(proxy, ret); }
    if (length < sizeof(int32_t)) { cc_ipc_packet_destroy(packet); free(bytes); return broken(proxy, -EPROTO); }
    int32_t status; memcpy(&status, bytes, sizeof(status));
    if (status > 0 || status < -4095 || (status && (length != sizeof(status) || count))) {
        cc_ipc_packet_destroy(packet); free(bytes); return broken(proxy, -EPROTO);
    }
    if (status) { cc_ipc_packet_destroy(packet); free(bytes); return status; }
    if (handles) *handles = (struct wire_handles){packet, fds, count};
    else cc_ipc_packet_destroy(packet);
    length -= sizeof(status);
    memmove(bytes, (unsigned char *)bytes + sizeof(status), length);
    *out = bytes; *out_size = length; return 0;
}
static int exchange_fds(struct proxy *proxy, uint32_t command, const void *data, size_t size,
                          const int *fds, uint32_t count, void **out, size_t *out_size, struct wire_handles *handles)
{
    *out = NULL; *out_size = 0;
    if (proxy->error) return proxy->error;
    cc_ipc_message_header header = {command, next_cookie(proxy)}; cc_ipc_packet *packet = NULL;
    int ret = cc_ipa_message_call_fds(proxy->pipe, &header, data, size, fds, count, &packet, proxy->timeout);
    return ret < 0 ? broken(proxy, ret) : collect_response(proxy, &header, packet, out, out_size, handles);
}
static int exchange(struct proxy *proxy, uint32_t command, const void *data, size_t size, void **out, size_t *out_size)
{
    return exchange_fds(proxy, command, data, size, NULL, 0, out, out_size, NULL);
}
static int empty_call(struct proxy *proxy, uint32_t command)
{
    clear_held(proxy); void *output = NULL; size_t size = 0;
    int ret = exchange(proxy, command, NULL, 0, &output, &size); free(output);
    return !ret && size ? broken(proxy, -EPROTO) : ret;
}
static int object_call(struct proxy *proxy, uint32_t command, enum cc_ipa_wire_kind input_kind,
                        const void *input, enum cc_ipa_wire_kind output_kind, cc_ipa_wire **output)
{
    if (output) *output = NULL;
    if (proxy->error) return proxy->error;
    void *bytes = NULL; size_t size = 0;
    /* The input can borrow payloads from the preceding prepare result. */
    int fds[CC_IPA_WIRE_MAX_FDS]; uint32_t count = 0;
    int ret = cc_ipa_wire_encode_fds(input_kind, input, &bytes, &size, fds, &count); if (ret < 0) return ret;
    void *response = NULL; size_t response_size = 0; struct wire_handles handles = {0};
    ret = exchange_fds(proxy, command, bytes, size, fds, count, &response, &response_size, &handles); free(bytes);
    if (!ret) {
        if (output_kind) ret = cc_ipa_wire_decode_fds(output_kind, response, response_size, handles.fds, handles.count, output);
        else if (response_size || handles.count) ret = -EPROTO;
        if (ret < 0) ret = broken(proxy, ret);
    }
    clear_held(proxy); cc_ipc_packet_destroy(handles.packet); free(response); return ret;
}
static int configure(void *context, const cc_ipa_configuration *configuration)
{
    return object_call(context, CC_IPA_PROXY_CONFIGURE, CC_IPA_WIRE_CONFIGURATION, configuration, 0, NULL);
}
static int start(void *context)
{
    struct proxy *proxy = context;
    if (proxy->kind == CC_IPA_PROXY_VIMC && !proxy->num_vimc_ids) {
        int fds[2] = {-1, -1}; cc_ipa_vimc_buffer buffers[2] = {0}; int ret = 0;
        for (size_t i = 0; i < 2; ++i) {
            fds[i] = memfd_create("ccamera-vimc-params", MFD_CLOEXEC);
            if (fds[i] < 0) { ret = -errno; break; }
            if (ftruncate(fds[i], 160 * 120 * 3)) { ret = -errno; break; }
            buffers[i] = (cc_ipa_vimc_buffer){.id = (uint32_t)i + 1U, .num_planes = 1,
                .planes = {{fds[i], 0, 160U * 120U * 3U}}};
        }
        if (!ret) ret = map_vimc(proxy, buffers, 2);
        for (size_t i = 0; i < 2; ++i) if (fds[i] >= 0) close(fds[i]);
        if (ret < 0) return ret;
    }
    return empty_call(context, CC_IPA_PROXY_START);
}
static void stop(void *context)
{
    struct proxy *proxy = context;
    if (proxy->kind == CC_IPA_PROXY_VIMC && !proxy->error) (void)unmap_vimc(proxy, proxy->vimc_ids, proxy->num_vimc_ids);
    (void)empty_call(context, CC_IPA_PROXY_STOP);
}
static int prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    struct proxy *proxy = context; cc_ipa_wire *wire = NULL;
    if (proxy->kind == CC_IPA_PROXY_VIMC) { int ret = queue_vimc(proxy, request); if (ret < 0) return ret; }
    int ret = object_call(proxy, CC_IPA_PROXY_PREPARE, CC_IPA_WIRE_REQUEST, request, CC_IPA_WIRE_SETTINGS, &wire);
    if (ret < 0) return ret;
    *settings = *(const cc_ipa_settings *)cc_ipa_wire_object(wire); proxy->held = wire; return 0;
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    struct proxy *proxy = context;
    if (proxy->kind == CC_IPA_PROXY_VIMC) {
        if (!proxy->num_vimc_ids) return -EPIPE;
        int ret = compute_vimc(proxy, (uint32_t)frame->sequence, proxy->vimc_ids[0]); if (ret < 0) return ret;
        proxy->expected_params = proxy->vimc_ids[0];
    }
    cc_ipa_wire *wire = NULL;
    int ret = object_call(context, CC_IPA_PROXY_PROCESS, CC_IPA_WIRE_FRAME, frame, CC_IPA_WIRE_METADATA, &wire);
    if (!ret) *metadata = *(const cc_ipa_metadata *)cc_ipa_wire_object(wire);
    cc_ipa_wire_destroy(wire); return ret;
}
static void destroy(void *context)
{
    struct proxy *proxy = context;
    if (!proxy->error && proxy->pipe) (void)empty_call(proxy, CC_IPA_PROXY_DESTROY);
    clear_held(proxy); cc_ipc_pipe_close(proxy->pipe);
    if (proxy->process) {
        cc_process_result result;
        (void)cc_process_wait(proxy->process, &result, proxy->timeout);
        (void)cc_process_close(proxy->process);
    }
    free(proxy);
}
static int candidate(const char *directory, char **out)
{
    static const char suffix[] = "/ccamera-ipa-worker"; size_t size = strlen(directory);
    if (size > 65536U - sizeof(suffix)) return -ENAMETOOLONG;
    char *path = malloc(size + sizeof(suffix)); if (!path) return -ENOMEM;
    memcpy(path, directory, size); memcpy(path + size, suffix, sizeof(suffix));
    char *canonical = realpath(path, NULL); int error = errno; free(path);
    if (!canonical) return error == ENOENT || error == ENOTDIR || error == EACCES ? 0 : -error;
    struct stat status;
    if (stat(canonical, &status) || !S_ISREG(status.st_mode) || access(canonical, X_OK)) { free(canonical); return 0; }
    *out = canonical; return 1;
}
static int worker_path(const cc_ipa_proxy_options *options, char **out)
{
    *out = NULL;
    if (options && options->worker_path) {
        if (!options->worker_path[0]) return -EINVAL;
        char *path = realpath(options->worker_path, NULL); if (!path) return -errno;
        struct stat status; int error = 0;
        if (stat(path, &status)) error = -errno;
        else if (!S_ISREG(status.st_mode)) error = -EINVAL;
        else if (access(path, X_OK)) error = -errno;
        if (error) { free(path); return error; }
        *out = path; return 0;
    }
    bool privileged = getauxval(AT_SECURE) != 0 || getuid() != geteuid() || getgid() != getegid();
    const char *environment = privileged ? NULL : getenv("CCAMERA_IPA_PROXY_PATH");
    if (environment && *environment) {
        size_t size = strnlen(environment, 65537U); if (size > 65536U) return -E2BIG;
        char *paths = strdup(environment); if (!paths) return -ENOMEM;
        char *position = paths; unsigned count = 0; int ret = 0;
        while (position) {
            char *next = strchr(position, ':'); if (next) *next++ = 0;
            if (*position) {
                if (++count > 64U) { ret = -E2BIG; break; }
                ret = candidate(position, out); if (ret) break;
            }
            position = next;
        }
        free(paths); if (ret) return ret < 0 ? ret : 0;
    }
    char *root = NULL; int ret = cc_build_root(&root); if (ret < 0) return ret;
    if (root) { ret = candidate(root, out); free(root); if (ret) return ret < 0 ? ret : 0; }
    ret = candidate(CCAMERA_IPA_WORKER_DIR, out); return ret < 0 ? ret : ret ? 0 : -ENOENT;
}
static int create_proxy(enum cc_ipa_proxy_kind kind, cc_ipa_module *module, const void *data, size_t size,
                         const cc_ipa_proxy_options *options, int trace_fd, uint32_t *vimc_flags, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if ((size && !data) || (options && options->timeout_ms < -1)) return -EINVAL;
    size_t extra = sizeof(struct cc_ipa_proxy_init) + (kind == CC_IPA_PROXY_MODULE ? sizeof(struct cc_ipa_module_transfer) : 0);
    if (size > CC_IPA_WIRE_MAX_DATA - extra) return -E2BIG;
    struct proxy *proxy = calloc(1, sizeof(*proxy)); if (!proxy) return -ENOMEM;
    proxy->timeout = options && options->timeout_ms ? options->timeout_ms : CC_IPC_PIPE_DEFAULT_TIMEOUT_MS;
    proxy->kind = kind;
    char *worker = NULL; unsigned char *input = NULL; cc_ipc *channel = NULL;
    int fds[4] = {-1, -1, -1, -1}; int ret = worker_path(options, &worker); if (ret < 0) goto done;
    if (trace_fd >= 0) { fds[3] = fcntl(trace_fd, F_DUPFD_CLOEXEC, 0); if (fds[3] < 0) { ret = -errno; goto done; } }
    input = malloc(extra + size); if (!input) { ret = -ENOMEM; goto done; }
    struct cc_ipa_proxy_init init = {CC_IPA_WIRE_VERSION, CC_IPA_ABI_VERSION, (uint32_t)kind};
    memcpy(input, &init, sizeof(init));
    if (kind == CC_IPA_PROXY_MODULE) {
        struct cc_ipa_module_transfer transfer;
        ret = cc_ipa_module_export(module, &fds[2], &transfer); if (ret < 0) goto done;
        memcpy(input + sizeof(init), &transfer, sizeof(transfer));
    }
    if (size) memcpy(input + extra, data, size);
    ret = cc_ipc_create(&channel, &fds[0]); if (ret < 0) goto done;
#ifdef SYS_pidfd_open
    fds[1] = (int)syscall(SYS_pidfd_open, getpid(), 0);
    if (fds[1] < 0) { ret = -errno; goto done; }
#else
    ret = -ENOSYS; goto done;
#endif
    char socket_number[24], parent_number[24], module_number[24], trace_number[24];
    (void)snprintf(socket_number, sizeof(socket_number), "%d", fds[0]);
    (void)snprintf(parent_number, sizeof(parent_number), "%d", fds[1]);
    (void)snprintf(module_number, sizeof(module_number), "%d", fds[2]);
    (void)snprintf(trace_number, sizeof(trace_number), "%d", fds[3]);
    const char *arguments[] = {socket_number, parent_number, module_number,
        kind == CC_IPA_PROXY_MODULE ? cc_ipa_module_get_path(module) : "-", trace_number};
    int passed[4]; size_t num_passed = 0;
    for (size_t i = 0; i < 4; ++i) if (fds[i] >= 0) passed[num_passed++] = fds[i];
    cc_process_options spawn = {.path = worker, .arguments = arguments, .num_arguments = 5,
        .fds = passed, .num_fds = num_passed, .flags = CC_PROCESS_NEW_USER | CC_PROCESS_NEW_NET};
    ret = cc_process_spawn(&spawn, &proxy->process); if (ret < 0) goto done;
    for (size_t i = 0; i < 4; ++i) if (fds[i] >= 0) { close(fds[i]); fds[i] = -1; }
    ret = cc_process_wait_exec(proxy->process, proxy->timeout); if (ret < 0) goto done;
    ret = cc_ipc_pipe_open(&channel, cc_process_fd(proxy->process), &proxy->pipe); if (ret < 0) goto done;
    void *response = NULL; size_t response_size = 0;
    ret = exchange(proxy, CC_IPA_PROXY_INIT, input, extra + size, &response, &response_size);
    uint32_t capabilities = 0, flags = 0;
    if (!ret) {
        size_t expected = kind == CC_IPA_PROXY_VIMC ? 2U * sizeof(uint32_t) : sizeof(uint32_t);
        if (response_size != expected) ret = -EPROTO;
        else {
            memcpy(&capabilities, response, sizeof(capabilities));
            if (vimc_flags) memcpy(&flags, (unsigned char *)response + sizeof(capabilities), sizeof(flags));
            if (kind == CC_IPA_PROXY_VIMC && (capabilities || flags != CC_IPA_VIMC_FLAG1)) ret = -EPROTO;
        }
    }
    free(response);
    if (ret < 0) goto done;
    cc_ipa_ops ops = {.abi_version = CC_IPA_ABI_VERSION, .size = sizeof(ops), .configure = configure, .start = start,
        .stop = stop, .prepare = prepare, .process = process, .destroy = destroy, .capabilities = capabilities};
    ret = cc_ipa_create(&ops, proxy, out);
    if (!ret && vimc_flags) *vimc_flags = flags;
done:
    free(input); free(worker); cc_ipc_close(channel);
    for (size_t i = 0; i < 4; ++i) if (fds[i] >= 0) close(fds[i]);
    if (ret < 0) { broken(proxy, ret); destroy(proxy); }
    return ret;
}
int cc_ipa_module_create_proxy(cc_ipa_module *module, const void *data, size_t size,
                                const cc_ipa_proxy_options *options, cc_ipa **out)
{
    if (!module) { if (out) *out = NULL; return -EINVAL; }
    return create_proxy(CC_IPA_PROXY_MODULE, module, data, size, options, -1, NULL, out);
}
int cc_ipa_module_create_auto(cc_ipa_module *module, const void *data, size_t size,
                              const cc_ipa_auto_options *options, cc_ipa **out, cc_ipa_execution_mode *mode)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!module || (size && !data) || (options &&
        ((options->flags & ~(uint32_t)CC_IPA_AUTO_FORCE_ISOLATION) || options->proxy.timeout_ms < -1))) return -EINVAL;
    cc_ipa_signature_status status = CC_IPA_SIGNATURE_UNAVAILABLE;
    int ret = 0;
    if (!options || !(options->flags & CC_IPA_AUTO_FORCE_ISOLATION))
        ret = cc_ipa_module_verify_signature(module, &status);
    if (ret < 0) return ret;
    cc_ipa_execution_mode selected = status == CC_IPA_SIGNATURE_VALID ?
        CC_IPA_EXECUTION_IN_PROCESS : CC_IPA_EXECUTION_ISOLATED;
    ret = selected == CC_IPA_EXECUTION_IN_PROCESS ? cc_ipa_module_create(module, data, size, out) :
        cc_ipa_module_create_proxy(module, data, size, options ? &options->proxy : NULL, out);
    if (!ret && mode) *mode = selected;
    return ret;
}
int cc_ipa_reference_create_proxy(const cc_ipa_reference_tuning *tuning, const cc_ipa_proxy_options *options, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL; void *data = NULL; size_t size = 0;
    int ret = cc_ipa_wire_encode(CC_IPA_WIRE_REFERENCE, tuning, &data, &size);
    if (!ret) ret = create_proxy(CC_IPA_PROXY_REFERENCE, NULL, data, size, options, -1, NULL, out);
    free(data); return ret;
}
int cc_ipa_vc4_create_proxy(const cc_ipa_vc4_options *controller, const cc_ipa_proxy_options *options, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL; void *data = NULL; size_t size = 0; int ret = 0;
    if (controller) ret = cc_ipa_wire_encode(CC_IPA_WIRE_VC4, controller, &data, &size);
    if (!ret) ret = create_proxy(controller ? CC_IPA_PROXY_VC4 : CC_IPA_PROXY_VC4_DEFAULT, NULL, data, size, options, -1, NULL, out);
    free(data); return ret;
}
static struct proxy *vimc_proxy(cc_ipa *ipa)
{
    struct proxy *proxy = cc_ipa_context(ipa, configure);
    return proxy && proxy->kind == CC_IPA_PROXY_VIMC ? proxy : NULL;
}
int cc_ipa_vimc_create_proxy(const cc_ipa_vimc_options *controller, const cc_ipa_proxy_options *options, cc_ipa **out, uint32_t *flags)
{
    if (out) *out = NULL;
    if (!out || !flags || !controller || !controller->configuration || !controller->configuration_size || controller->trace_fd < -1 ||
        controller->operation < CC_IPA_VIMC_NONE || controller->operation > CC_IPA_VIMC_STOP || controller->flags & ~15U) return -EINVAL;
    if (controller->configuration_size > CC_IPA_WIRE_MAX_DATA - sizeof(struct cc_ipa_vimc_init)) return -E2BIG;
    size_t size = sizeof(struct cc_ipa_vimc_init) + controller->configuration_size;
    unsigned char *bytes = malloc(size); if (!bytes) return -ENOMEM;
    struct cc_ipa_vimc_init init = {(uint32_t)controller->operation, controller->flags}; memcpy(bytes, &init, sizeof(init));
    memcpy(bytes + sizeof(init), controller->configuration, controller->configuration_size);
    int ret = create_proxy(CC_IPA_PROXY_VIMC, NULL, bytes, size, options, controller->trace_fd, flags, out);
    free(bytes); return ret;
}
static int map_vimc(struct proxy *proxy, const cc_ipa_vimc_buffer *buffers, size_t count)
{
    if (proxy->error) return proxy->error;
    if (count > CC_IPA_VIMC_MAX_BUFFERS || (count && !buffers)) return -EINVAL;
    size_t size = sizeof(uint32_t); uint32_t num_fds = 0;
    int fds[CC_IPA_VIMC_MAX_BUFFERS * CC_MAX_PLANES];
    if (count > CC_IPA_VIMC_MAX_BUFFERS - proxy->num_vimc_ids) return -ENOSPC;
    for (size_t i = 0; i < count; ++i) {
        if (!buffers[i].id || !buffers[i].num_planes || buffers[i].num_planes > CC_MAX_PLANES) return -EINVAL;
        for (size_t j = 0; j < i; ++j) if (buffers[j].id == buffers[i].id) return -EEXIST;
        for (size_t j = 0; j < proxy->num_vimc_ids; ++j) if (proxy->vimc_ids[j] == buffers[i].id) return -EEXIST;
        for (size_t j = 0; j < buffers[i].num_planes; ++j) {
            if (buffers[i].planes[j].fd < 0) return -EBADF;
            if (fcntl(buffers[i].planes[j].fd, F_GETFD) < 0) return -errno;
            fds[num_fds++] = buffers[i].planes[j].fd;
        }
        size += sizeof(struct cc_ipa_vimc_map_header) + sizeof(struct cc_ipa_vimc_map_plane) * buffers[i].num_planes;
    }
    unsigned char *bytes = malloc(size); if (!bytes) return -ENOMEM;
    uint32_t total = (uint32_t)count; memcpy(bytes, &total, sizeof(total)); size_t offset = sizeof(total);
    for (size_t i = 0; i < count; ++i) {
        struct cc_ipa_vimc_map_header header = {buffers[i].id, buffers[i].num_planes};
        memcpy(bytes + offset, &header, sizeof(header)); offset += sizeof(header);
        for (size_t j = 0; j < buffers[i].num_planes; ++j) {
            struct cc_ipa_vimc_map_plane plane = {buffers[i].planes[j].offset, buffers[i].planes[j].length};
            memcpy(bytes + offset, &plane, sizeof(plane)); offset += sizeof(plane);
        }
    }
    cc_ipc_message_header header = {CC_IPA_PROXY_VIMC_MAP, next_cookie(proxy)};
    cc_ipc_payload payload = {bytes, size, fds, num_fds}; cc_ipc_packet *packet = NULL;
    int ret = cc_ipc_pipe_call(proxy->pipe, &header, &payload, &packet, proxy->timeout); free(bytes);
    if (ret < 0) return broken(proxy, ret);
    void *output = NULL; size_t output_size = 0;
    ret = collect_response(proxy, &header, packet, &output, &output_size, NULL); free(output);
    if (!ret && output_size) ret = broken(proxy, -EPROTO);
    if (!ret) for (size_t i = 0; i < count; ++i) proxy->vimc_ids[proxy->num_vimc_ids++] = buffers[i].id;
    return ret;
}
static int unmap_vimc(struct proxy *proxy, const uint32_t *ids, size_t count)
{
    if (count > CC_IPA_VIMC_MAX_BUFFERS || (count && !ids)) return -EINVAL;
    uint32_t values[CC_IPA_VIMC_MAX_BUFFERS + 1U]; values[0] = (uint32_t)count;
    if (count) memcpy(values + 1, ids, count * sizeof(*ids));
    void *output = NULL; size_t size = 0;
    int ret = exchange(proxy, CC_IPA_PROXY_VIMC_UNMAP, values, (count + 1U) * sizeof(*values), &output, &size); free(output);
    if (!ret && size) ret = broken(proxy, -EPROTO);
    if (!ret) for (size_t i = 0; i < count; ++i) {
        for (size_t j = 0; j < proxy->num_vimc_ids; ++j) if (proxy->vimc_ids[j] == values[i + 1]) {
            --proxy->num_vimc_ids;
            memmove(proxy->vimc_ids + j, proxy->vimc_ids + j + 1, (proxy->num_vimc_ids - j) * sizeof(uint32_t)); break;
        }
    }
    return ret;
}
static int queue_vimc(struct proxy *proxy, const cc_ipa_request *request)
{
    if (proxy->error) return proxy->error;
    void *data = NULL; size_t size = 0;
    int ret = cc_ipa_wire_encode(CC_IPA_WIRE_REQUEST, request, &data, &size);
    cc_ipc_message_header header = {CC_IPA_PROXY_VIMC_QUEUE, 0};
    if (!ret) ret = cc_ipa_message_send(proxy->pipe, &header, data, size);
    free(data);
    if (ret < 0 && cc_ipc_pipe_fd(proxy->pipe) < 0) return broken(proxy, ret);
    return ret;
}
static int compute_vimc(struct proxy *proxy, uint32_t frame, uint32_t id)
{
    if (proxy->error) return proxy->error;
    uint32_t data[] = {frame, id}; cc_ipc_message_header header = {CC_IPA_PROXY_VIMC_COMPUTE, 0};
    int ret = cc_ipa_message_send(proxy->pipe, &header, data, sizeof(data));
    return ret < 0 && cc_ipc_pipe_fd(proxy->pipe) < 0 ? broken(proxy, ret) : ret;
}
int cc_ipa_vimc_map_buffers(cc_ipa *ipa, const cc_ipa_vimc_buffer *buffers, size_t count)
{
    struct proxy *proxy = vimc_proxy(ipa); return proxy ? map_vimc(proxy, buffers, count) : -EINVAL;
}
int cc_ipa_vimc_unmap_buffers(cc_ipa *ipa, const uint32_t *ids, size_t count)
{
    struct proxy *proxy = vimc_proxy(ipa); return proxy ? unmap_vimc(proxy, ids, count) : -EINVAL;
}
int cc_ipa_vimc_queue_request(cc_ipa *ipa, uint32_t frame, const cc_control *controls, size_t count)
{
    struct proxy *proxy = vimc_proxy(ipa); if (!proxy) return -EINVAL;
    if (count > CC_IPA_MAX_CONTROLS || (count && !controls)) return -EINVAL;
    cc_ipa_request request = {.sequence = frame, .sensor_controls = controls, .num_sensor_controls = count};
    return queue_vimc(proxy, &request);
}
int cc_ipa_vimc_compute_params(cc_ipa *ipa, uint32_t frame, uint32_t id)
{
    struct proxy *proxy = vimc_proxy(ipa); return proxy ? compute_vimc(proxy, frame, id) : -EINVAL;
}
int cc_ipa_vimc_dispatch(cc_ipa *ipa)
{
    struct proxy *proxy = vimc_proxy(ipa); if (!proxy) return -EINVAL;
    if (proxy->error) return proxy->error;
    int ret = cc_ipc_pipe_dispatch(proxy->pipe); return ret < 0 ? broken(proxy, ret) : ret;
}
int cc_ipa_vimc_dequeue_event(cc_ipa *ipa, cc_ipa_vimc_event *event, int timeout_ms)
{
    struct proxy *proxy = vimc_proxy(ipa); if (!proxy || !event || timeout_ms < -1) return -EINVAL;
    if (proxy->num_events) {
        *event = proxy->events[proxy->event_head];
        proxy->event_head = (proxy->event_head + 1) % CC_IPC_PIPE_MAX_PENDING; --proxy->num_events; return 0;
    }
    if (proxy->error) return proxy->error;
    cc_ipc_packet *packet = NULL; int ret = cc_ipc_pipe_receive(proxy->pipe, &packet, timeout_ms);
    if (ret < 0) return ret == -EAGAIN || ret == -ETIMEDOUT ? ret : broken(proxy, ret);
    cc_ipa_vimc_event result = {0}; ret = event_packet(packet, &result); cc_ipc_packet_destroy(packet);
    if (ret < 0) return broken(proxy, ret);
    *event = result; return 0;
}
