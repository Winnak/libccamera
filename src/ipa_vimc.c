/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Native adaptation of src/ipa/vimc/vimc.cpp, Copyright (C) 2019 Google Inc. */
#define _GNU_SOURCE
#include "ipa_vimc.h"
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

struct mapped_plane { void *data; size_t length; };
struct mapped_buffer { uint32_t id, num_planes; struct mapped_plane planes[CC_MAX_PLANES]; size_t bytes; };
struct cc_ipa_vimc { int trace_fd; struct mapped_buffer buffers[CC_IPA_VIMC_MAX_BUFFERS]; size_t bytes; };
static void release(struct mapped_buffer *buffer)
{
    for (size_t i = 0; i < buffer->num_planes; ++i)
        if (buffer->planes[i].data != MAP_FAILED && buffer->planes[i].length)
            munmap(buffer->planes[i].data, buffer->planes[i].length);
    *buffer = (struct mapped_buffer){0};
}
static void trace(struct cc_ipa_vimc *context, uint32_t operation)
{
    if (context->trace_fd < 0) return;
    ssize_t size; do { size = write(context->trace_fd, &operation, sizeof(operation)); } while (size < 0 && errno == EINTR);
}
static int configure(void *context, const cc_ipa_configuration *configuration)
{
    (void)context; return configuration->pipeline == CC_IPA_VIMC ? 0 : -EINVAL;
}
static int start(void *context) { trace(context, CC_IPA_VIMC_START); return 0; }
static void stop(void *context) { trace(context, CC_IPA_VIMC_STOP); }
static int prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    (void)context; (void)request; *settings = (cc_ipa_settings){0}; return 0;
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    (void)context; (void)frame; *metadata = (cc_ipa_metadata){0}; return 0;
}
static void destroy(void *context)
{
    struct cc_ipa_vimc *vimc = context;
    for (size_t i = 0; i < CC_IPA_VIMC_MAX_BUFFERS; ++i) release(&vimc->buffers[i]);
    if (vimc->trace_fd >= 0) close(vimc->trace_fd);
    free(vimc);
}
int cc_ipa_vimc_create(const cc_ipa_vimc_options *options, cc_ipa **ipa, struct cc_ipa_vimc **context, uint32_t *flags)
{
    if (!ipa || !context || !flags) return -EINVAL;
    *ipa = NULL; *context = NULL;
    if (!options || !options->configuration || !options->configuration_size || options->trace_fd < -1 ||
        options->operation < CC_IPA_VIMC_NONE || options->operation > CC_IPA_VIMC_STOP || options->flags & ~15U) return -EINVAL;
    struct cc_ipa_vimc *vimc = calloc(1, sizeof(*vimc)); if (!vimc) return -ENOMEM;
    vimc->trace_fd = -1;
    if (options->trace_fd >= 0) {
        vimc->trace_fd = fcntl(options->trace_fd, F_DUPFD_CLOEXEC, 0);
        if (vimc->trace_fd < 0) { int error = -errno; destroy(vimc); return error; }
    }
    trace(vimc, CC_IPA_VIMC_INIT);
    cc_ipa_ops ops = {.abi_version = CC_IPA_ABI_VERSION, .size = sizeof(ops), .configure = configure, .start = start,
        .stop = stop, .prepare = prepare, .process = process, .destroy = destroy};
    int ret = cc_ipa_create(&ops, vimc, ipa);
    if (ret < 0) { destroy(vimc); return ret; }
    *context = vimc; *flags = CC_IPA_VIMC_FLAG1; return 0;
}
void cc_ipa_vimc_unmap(struct cc_ipa_vimc *context, const uint32_t *ids, size_t count)
{
    for (size_t i = 0; i < count; ++i) for (size_t j = 0; j < CC_IPA_VIMC_MAX_BUFFERS; ++j)
        if (context->buffers[j].id == ids[i] && ids[i]) {
            context->bytes -= context->buffers[j].bytes; release(&context->buffers[j]); break;
        }
}
int cc_ipa_vimc_map(struct cc_ipa_vimc *context, const cc_ipa_vimc_buffer *buffers, size_t count)
{
    if (!context || count > CC_IPA_VIMC_MAX_BUFFERS || (count && !buffers)) return -EINVAL;
    struct mapped_buffer staged[CC_IPA_VIMC_MAX_BUFFERS] = {0}; size_t bytes = context->bytes, free_slots = 0;
    for (size_t i = 0; i < CC_IPA_VIMC_MAX_BUFFERS; ++i) if (!context->buffers[i].id) ++free_slots;
    if (count > free_slots) return -ENOSPC;
    long page_size = sysconf(_SC_PAGESIZE); if (page_size <= 0) return -EIO;
    int error = 0;
    for (size_t i = 0; i < count && !error; ++i) {
        const cc_ipa_vimc_buffer *input = &buffers[i];
        if (!input->id || !input->num_planes || input->num_planes > CC_MAX_PLANES) { error = -EINVAL; break; }
        for (size_t j = 0; j < CC_IPA_VIMC_MAX_BUFFERS; ++j) if (context->buffers[j].id == input->id) error = -EEXIST;
        for (size_t j = 0; j < i; ++j) if (buffers[j].id == input->id) error = -EEXIST;
        staged[i].id = input->id;
        for (size_t j = 0; j < input->num_planes && !error; ++j) {
            const cc_ipa_vimc_plane *plane = &input->planes[j];
            uint64_t delta = plane->offset % (uint64_t)page_size;
            if (plane->fd < 0 || !plane->length || plane->offset > INT64_MAX ||
                plane->length > CC_IPA_VIMC_MAX_MAPPED_BYTES - bytes ||
                delta > SIZE_MAX - plane->length || plane->offset > UINT64_MAX - plane->length) { error = -EINVAL; break; }
            struct stat status;
            if (fstat(plane->fd, &status)) { error = -errno; break; }
            if (S_ISREG(status.st_mode) && (status.st_size < 0 || plane->offset + plane->length > (uint64_t)status.st_size)) { error = -EINVAL; break; }
            size_t length = (size_t)(plane->length + delta);
            void *data = mmap(NULL, length, PROT_READ, MAP_SHARED, plane->fd, (off_t)(plane->offset - delta));
            if (data == MAP_FAILED) { error = -errno; break; }
            staged[i].planes[staged[i].num_planes++] = (struct mapped_plane){data, length};
            staged[i].bytes += (size_t)plane->length; bytes += (size_t)plane->length;
        }
    }
    if (error) { for (size_t i = 0; i < count; ++i) release(&staged[i]); return error; }
    size_t next = 0;
    for (size_t i = 0; i < CC_IPA_VIMC_MAX_BUFFERS && next < count; ++i)
        if (!context->buffers[i].id) context->buffers[i] = staged[next++];
    context->bytes = bytes; return 0;
}
int cc_ipa_vimc_compute(struct cc_ipa_vimc *context, uint32_t id, cc_ipa_vimc_event *event)
{
    for (size_t i = 0; i < CC_IPA_VIMC_MAX_BUFFERS; ++i)
        if (context->buffers[i].id == id && id) { *event = (cc_ipa_vimc_event){id, 0}; return 0; }
    return -ENOENT;
}
