/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#define CC_VC4_MAX_BUFFERS 256U
#define CC_VC4_STATISTICS CC_FOURCC('B', 'S', 'T', 'A')

struct cc_vc4_isp {
    struct cc_io io;
    struct cc_media *media;
    cc_video_device *devices[4];
    cc_stream *streams[4];
    bool configured, fatal;
    uint32_t count, done;
    int active;
    unsigned char states[CC_VC4_MAX_BUFFERS]; /* 0 free, 1 active, 2 held result. */
    cc_vc4_configuration configuration;
    cc_vc4_result pending;
    int shading_fd;
    void *shading_map;
};

cc_stream *cc_vc4_image_stream(cc_vc4_isp *isp, uint32_t output)
{
    return isp && output < 2 ? isp->streams[output + 1] : NULL;
}
cc_stream *cc_vc4_statistics_stream(cc_vc4_isp *isp) { return isp ? isp->streams[CC_VC4_STATS] : NULL; }

int cc_vc4_capture_info(cc_vc4_isp *isp, cc_vc4_configuration *configuration, uint32_t *count)
{
    if (!isp) return -EINVAL;
    if (isp->fatal) return -ENODEV;
    if (!isp->configured || !isp->count) return -EPIPE;
    if (isp->active >= 0) return -EBUSY;
    for (uint32_t i = 0; i < isp->count; ++i) if (isp->states[i]) return -EBUSY;
    *configuration = isp->configuration; *count = isp->count;
    return 0;
}

size_t cc_vc4_pending_fds(const cc_vc4_isp *isp, struct pollfd *fds)
{
    size_t count = 0;
    if (isp->active >= 0)
        for (size_t i = 0; i < 4; ++i)
            if (!(isp->done & (1U << i)))
                fds[count++] = (struct pollfd) {cc_stream_fd(isp->streams[i]), i ? POLLIN : POLLOUT, 0};
    return count;
}

static int usable(const cc_vc4_isp *isp)
{
    return !isp ? -EINVAL : isp->fatal ? -ENODEV : 0;
}

static bool allocated(const cc_vc4_isp *isp)
{
    for (size_t i = 0; i < 4; ++i)
        if (isp->streams[i] && isp->streams[i]->allocated)
            return true;
    return false;
}

static struct cc_entity *named_entity(struct cc_media *media, const char *name)
{
    for (size_t i = 0; i < media->num_entities; ++i)
        if (!strcmp(cc_entity_name(&media->entities[i]), name))
            return &media->entities[i];
    return NULL;
}

int cc_vc4_isp_open_io(const char *path, const struct cc_io *io, cc_vc4_isp **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!path || !io)
        return -EINVAL;
    cc_vc4_isp *isp = calloc(1, sizeof(*isp));
    if (!isp)
        return -ENOMEM;
    isp->io = *io;
    isp->active = -1;
    isp->shading_fd = -1;
    int ret = cc_media_open(io, path, true, &isp->media);
    if (ret < 0)
        goto fail;
    if (strncmp(isp->media->info.driver, "bcm2835-isp", sizeof(isp->media->info.driver))) {
        ret = -ENOTSUP;
        goto fail;
    }
    struct cc_entity *input = NULL;
    for (size_t i = 0; i < isp->media->num_entities; ++i) {
        struct cc_entity *e = &isp->media->entities[i];
        size_t n = strlen(cc_entity_name(e));
        if (n > 8 && !strncmp(cc_entity_name(e), "bcm2835-isp", 11) && !strcmp(cc_entity_name(e) + n - 8, "-output0")) {
            if (input) { ret = -ENOTSUP; goto fail; }
            input = e;
        }
    }
    if (!input) { ret = -ENODEV; goto fail; }
    size_t prefix = strlen(cc_entity_name(input)) - 8;
    for (size_t i = 0; i < 4; ++i) {
        char name[80];
        if (!i)
            cc_copy_string(name, sizeof(name), cc_entity_name(input), strlen(cc_entity_name(input)));
        else
            snprintf(name, sizeof(name), "%.*s-capture%u", (int)prefix, cc_entity_name(input), (unsigned int)i);
        struct cc_entity *entity = named_entity(isp->media, name);
        if (!entity || !entity->path[0]) { ret = -ENODEV; goto fail; }
        ret = cc_video_open_io(entity->path, io, &isp->devices[i]);
        if (ret < 0)
            goto fail;
        uint32_t caps = isp->devices[i]->capabilities;
        enum cc_stream_type type;
        if (i == CC_VC4_STATS)
            type = CC_STREAM_META_CAPTURE;
        else if (!i)
            type = caps & V4L2_CAP_VIDEO_OUTPUT ? CC_STREAM_OUTPUT : CC_STREAM_OUTPUT_MPLANE;
        else
            type = caps & V4L2_CAP_VIDEO_CAPTURE ? CC_STREAM_CAPTURE : CC_STREAM_CAPTURE_MPLANE;
        ret = cc_video_create_stream(isp->devices[i], type, &isp->streams[i]);
        if (ret < 0)
            goto fail;
    }
    *out = isp;
    return 0;
fail:
    cc_vc4_isp_close(isp);
    return ret;
}

int cc_vc4_isp_open(const char *media_path, cc_vc4_isp **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (media_path)
        return cc_vc4_isp_open_io(media_path, &cc_system_io, out);
    struct dirent **nodes = NULL;
    int count = scandir("/dev", &nodes, NULL, alphasort);
    if (count < 0)
        return -errno;
    int ret = -ENODEV;
    for (int i = 0; i < count; ++i) {
        const char *name = nodes[i]->d_name;
        if (strncmp(name, "media", 5) || !name[5] || strspn(name + 5, "0123456789") != strlen(name + 5))
            continue;
        char path[CC_PATH_MAX];
        int n = snprintf(path, sizeof(path), "/dev/%s", name);
        if (n < 0 || (size_t)n >= sizeof(path))
            continue;
        int candidate = cc_vc4_isp_open_io(path, &cc_system_io, out);
        if (!candidate) { ret = 0; break; }
        if (candidate == -ENOMEM) { ret = candidate; break; }
        if (candidate != -ENODEV && candidate != -ENOTSUP && candidate != -ENOENT)
            ret = candidate;
    }
    for (int i = 0; i < count; ++i)
        free(nodes[i]);
    free(nodes);
    return ret;
}

static bool same_input(const cc_format *source, const cc_format *input)
{
    if (source->fourcc != input->fourcc || source->width != input->width ||
        source->height != input->height || source->num_planes != input->num_planes)
        return false;
    for (uint32_t p = 0; p < source->num_planes; ++p)
        if (source->planes[p].stride != input->planes[p].stride || source->planes[p].size != input->planes[p].size)
            return false;
    return true;
}

static cc_rectangle centered_crop(const cc_format *input, const cc_format *output)
{
    uint32_t width = input->width, height = input->height;
    if ((uint64_t)width * output->height > (uint64_t)height * output->width)
        width = (uint32_t)((uint64_t)height * output->width / output->height);
    else
        height = (uint32_t)((uint64_t)width * output->height / output->width);
    width &= ~1U;
    height &= ~1U;
    return (cc_rectangle) { (int32_t)(((input->width - width) / 2) & ~1U),
                             (int32_t)(((input->height - height) / 2) & ~1U), width, height };
}

static bool crop_valid(const cc_format *input, const cc_rectangle *crop)
{
    return crop->x >= 0 && crop->y >= 0 && crop->width && crop->height &&
           (uint64_t)(uint32_t)crop->x + crop->width <= input->width &&
           (uint64_t)(uint32_t)crop->y + crop->height <= input->height;
}

int cc_vc4_configuration_ready(cc_vc4_isp *isp)
{
    int ret=usable(isp); return ret<0 ? ret:allocated(isp) ? -EBUSY:0;
}
void cc_vc4_invalidate_configuration(cc_vc4_isp *isp) { if (isp) isp->configured=false; }
int cc_vc4_isp_get_configuration(cc_vc4_isp *isp, cc_vc4_configuration *out)
{
    int ret=usable(isp); if (ret<0) return ret;
    if (!out) return -EINVAL;
    if (!isp->configured) return -EPIPE;
    *out=isp->configuration; return 0;
}

static int negotiate_stream(cc_stream *stream, cc_format *format, bool active)
{
    return active ? cc_stream_configure(stream,format):cc_stream_try_format(stream,format);
}

static int negotiate_configuration(cc_vc4_isp *isp, cc_vc4_configuration *configuration, bool active)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    if (!configuration || !configuration->input.width || !configuration->input.height ||
        !configuration->input.fourcc || !configuration->input.num_planes ||
        configuration->input.num_planes > CC_MAX_PLANES || configuration->input.width > INT32_MAX ||
        configuration->input.height > INT32_MAX || !configuration->output[0].width ||
        !configuration->output[0].height || !configuration->output[0].fourcc)
        return -EINVAL;
    if (active && allocated(isp))
        return -EBUSY;
    cc_vc4_configuration c = *configuration;
    if ((c.crop.width || c.crop.height) && !crop_valid(&c.input, &c.crop))
        return -EINVAL;
    if (active) isp->configured = false;
    ret = negotiate_stream(isp->streams[CC_VC4_INPUT], &c.input,active);
    if (ret < 0)
        return ret;
    if (!same_input(&configuration->input, &c.input))
        return -EPIPE;
    ret = negotiate_stream(isp->streams[CC_VC4_MAIN], &c.output[0],active);
    if (ret < 0)
        return ret;
    if (!c.output[1].fourcc && !c.output[1].width && !c.output[1].height) {
        uint32_t w = c.output[0].width / 2, h = c.output[0].height / 2;
        uint32_t largest = w > h ? w : h;
        if (largest > 1200) {
            w = (uint32_t)((uint64_t)w * 1200 / largest);
            h = (uint32_t)((uint64_t)h * 1200 / largest);
        }
        c.output[1] = (cc_format) { .fourcc = CC_FOURCC('Y', 'U', '1', '2'),
            .width = w > 2 ? w & ~1U : 2, .height = h > 2 ? h & ~1U : 2,
            .colorspace = c.output[0].colorspace, .ycbcr_enc = c.output[0].ycbcr_enc,
            .quantization = c.output[0].quantization, .xfer_func = c.output[0].xfer_func };
    }
    ret = negotiate_stream(isp->streams[CC_VC4_LORES], &c.output[1],active);
    if (ret < 0)
        return ret;
    if (c.output[1].width > c.output[0].width || c.output[1].height > c.output[0].height)
        return -EINVAL;
    c.statistics = (cc_format) { .fourcc = CC_VC4_STATISTICS };
    ret = negotiate_stream(isp->streams[CC_VC4_STATS], &c.statistics,active);
    if (ret < 0)
        return ret;
    if (c.statistics.fourcc != CC_VC4_STATISTICS)
        return -ENOTSUP;
    if (!c.crop.width && !c.crop.height)
        c.crop = centered_crop(&c.input, &c.output[0]);
    if (!crop_valid(&c.input, &c.crop))
        return -EINVAL;
    if (active) {
        ret = cc_stream_set_selection(isp->streams[CC_VC4_INPUT], V4L2_SEL_TGT_CROP, 0, &c.crop);
        if (ret < 0) return ret;
    }
    if (!crop_valid(&c.input, &c.crop))
        return -EPROTO;
    if (active) { isp->configuration = c; isp->configured = true; }
    *configuration = c;
    return 0;
}

int cc_vc4_isp_configure(cc_vc4_isp *isp, cc_vc4_configuration *configuration)
{
    return negotiate_configuration(isp,configuration,true);
}
int cc_vc4_isp_try_configuration(cc_vc4_isp *isp, cc_vc4_configuration *configuration)
{
    return negotiate_configuration(isp,configuration,false);
}

int cc_vc4_isp_stop(cc_vc4_isp *isp)
{
    if (!isp)
        return -EINVAL;
    int first_error = isp->fatal ? -ENODEV : 0;
    /* Finish/abort every port before unbinding any input or unmapping output. */
    for (size_t i = 0; i < 4; ++i) {
        if (!isp->streams[i])
            continue;
        int ret = cc_stream_stop(isp->streams[i]);
        if (ret < 0 && !first_error)
            first_error = ret;
    }
    if (!first_error) for (size_t i = 0; i < 4; ++i) {
        int ret = cc_stream_clear_imports(isp->streams[i]);
        if (ret < 0) { first_error = ret; break; }
    }
    if (first_error) {
        for (size_t i = 0; i < 4; ++i) {
            cc_video_device *d = isp->devices[i];
            if (d && d->fd >= 0) {
                d->io.ops->close(d->io.ctx, d->fd);
                d->fd = -1;
            }
        }
        for (size_t i = 0; i < 4; ++i)
            if (isp->streams[i] && isp->streams[i]->memory == V4L2_MEMORY_DMABUF)
                (void)cc_stream_free_buffers(isp->streams[i]);
        isp->fatal = true;
        isp->configured = false;
    }
    isp->active = -1;
    isp->done = 0;
    memset(isp->states, 0, sizeof(isp->states));
    return first_error;
}

int cc_vc4_isp_free_buffers(cc_vc4_isp *isp)
{
    if (!isp)
        return -EINVAL;
    int ret = cc_vc4_isp_stop(isp);
    for (size_t i = 0; i < 4; ++i) {
        if (!isp->streams[i])
            continue;
        int freed = cc_stream_free_buffers(isp->streams[i]);
        if (freed < 0 && !ret)
            ret = freed;
    }
    isp->count = 0;
    return ret;
}

void cc_vc4_isp_close(cc_vc4_isp *isp)
{
    if (!isp)
        return;
    cc_vc4_isp_stop(isp);
    for (size_t i = 0; i < 4; ++i) {
        cc_stream_close(isp->streams[i]);
        cc_video_close(isp->devices[i]);
    }
    if (isp->shading_map) isp->io.ops->unmap(isp->io.ctx,isp->shading_map,64U*49U*8U);
    if (isp->shading_fd>=0) isp->io.ops->close(isp->io.ctx,isp->shading_fd);
    cc_media_close(isp->media);
    free(isp);
}

int cc_vc4_isp_allocate_buffers_with_memory(cc_vc4_isp *isp, const enum cc_memory outputs[2], uint32_t *count)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    if (!count || !*count || *count > CC_VC4_MAX_BUFFERS)
        return -EINVAL;
    if (!outputs || (outputs[0] != CC_MEMORY_MMAP && outputs[0] != CC_MEMORY_DMABUF) ||
        (outputs[1] != CC_MEMORY_MMAP && outputs[1] != CC_MEMORY_DMABUF)) return -EINVAL;
    if (allocated(isp))
        return -EBUSY;
    if (!isp->configured)
        return -EPIPE;
    uint32_t minimum = UINT32_MAX;
    for (size_t i = 0; i < 4; ++i) {
        uint32_t n = *count;
        enum cc_memory memory = !i ? CC_MEMORY_DMABUF : i == 3 ? CC_MEMORY_MMAP : outputs[i - 1];
        ret = cc_stream_allocate_buffers(isp->streams[i], memory, &n);
        if (ret < 0) {
            cc_vc4_isp_free_buffers(isp);
            return ret;
        }
        if (n < minimum)
            minimum = n;
    }
    isp->count = minimum;
    *count = minimum;
    return 0;
}

int cc_vc4_isp_allocate_buffers(cc_vc4_isp *isp, uint32_t *count)
{
    const enum cc_memory outputs[2] = {CC_MEMORY_MMAP, CC_MEMORY_MMAP};
    return cc_vc4_isp_allocate_buffers_with_memory(isp, outputs, count);
}

int cc_vc4_isp_import_output(cc_vc4_isp *isp, uint32_t output, uint32_t index,
                            const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    int ret = usable(isp);
    if (ret < 0) return ret;
    if (output > 1 || index >= isp->count) return -EINVAL;
    if (isp->states[index]) return -EBUSY;
    return cc_stream_import(isp->streams[output + 1], index, planes, num_planes);
}

static int cancel_error(cc_vc4_isp *isp, int error)
{
    int ret = cc_vc4_isp_stop(isp);
    return ret < 0 ? ret : error;
}

int cc_vc4_isp_submit(cc_vc4_isp *isp, uint32_t index, const cc_dmabuf_plane *planes,
                     uint32_t num_planes, const cc_frame *input, uint64_t cookie)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    if (!isp->count)
        return -EPIPE;
    if (index >= isp->count || !input || num_planes != input->num_planes ||
        num_planes != isp->configuration.input.num_planes || !planes ||
        input->timestamp_ns / UINT64_C(1000000000) > LONG_MAX)
        return -EINVAL;
    if (isp->active >= 0 || isp->states[index])
        return -EBUSY;
    for (size_t i = 0; i < 4; ++i)
        if (!isp->streams[i]->configured || !isp->streams[i]->allocated)
            return -EPIPE;
    for (size_t i = 1; i < 4; ++i)
        if (!isp->streams[i]->buffers[index].bound) return -EPIPE;
    for (uint32_t p = 0; p < num_planes; ++p) {
        if (input->planes[p].bytes_used > planes[p].length ||
            input->planes[p].bytes_used < isp->configuration.input.planes[p].size || input->planes[p].data_offset)
            return -EINVAL;
    }
    ret = cc_stream_import(isp->streams[0], index, planes, num_planes);
    if (ret < 0)
        return ret;
    for (size_t i = 1; i < 4; ++i) {
        ret = cc_stream_queue(isp->streams[i], index, NULL);
        if (ret < 0)
            return cancel_error(isp, ret);
    }
    ret = cc_stream_queue(isp->streams[0], index, input);
    if (ret < 0)
        return cancel_error(isp, ret);
    for (size_t n = 0; n < 4; ++n) {
        size_t i = (n + 1) % 4;
        if (isp->streams[i]->streaming)
            continue;
        ret = cc_stream_start(isp->streams[i]);
        if (ret < 0)
            return cancel_error(isp, ret);
    }
    isp->pending = (cc_vc4_result) { .index = index, .cookie = cookie, .input = *input };
    for (uint32_t p = 0; p < num_planes; ++p)
        isp->pending.input.planes[p].data = NULL;
    isp->active = (int)index;
    isp->states[index] = 1;
    isp->done = 0;
    return 0;
}

static int accept_completion(cc_vc4_isp *isp, size_t port, const cc_frame *frame)
{
    if (frame->index != (uint32_t)isp->active)
        return cancel_error(isp, -EPROTO);
    if (port == CC_VC4_STATS)
        isp->pending.statistics = *frame;
    else if (port != CC_VC4_INPUT)
        isp->pending.output[port - 1] = *frame;
    isp->done |= 1U << port;
    return 0;
}

int cc_vc4_isp_dequeue(cc_vc4_isp *isp, cc_vc4_result *result, int timeout_ms)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    if (!result || timeout_ms < -1)
        return -EINVAL;
    if (isp->active < 0)
        return -EAGAIN;
    uint64_t deadline = 0;
    if (timeout_ms > 0) {
        ret = isp->io.ops->now(isp->io.ctx, &deadline);
        if (ret < 0)
            return cancel_error(isp, ret);
        uint64_t duration = (uint64_t)timeout_ms * UINT64_C(1000000);
        deadline = deadline > UINT64_MAX - duration ? UINT64_MAX : deadline + duration;
    }
    for (;;) {
        for (size_t i = 0; i < 4; ++i) {
            if (isp->done & (1U << i))
                continue;
            cc_frame frame;
            ret = cc_stream_dequeue(isp->streams[i], &frame, 0);
            if (ret == -EAGAIN)
                continue;
            if (ret < 0)
                return cancel_error(isp, ret);
            ret = accept_completion(isp, i, &frame);
            if (ret < 0)
                return ret;
        }
        if (isp->done == 15) {
            ret = cc_stream_unimport(isp->streams[0], (uint32_t)isp->active);
            if (ret < 0)
                return cancel_error(isp, ret);
            isp->states[isp->active] = 2;
            isp->active = -1;
            *result = isp->pending;
            return 0;
        }
        if (!timeout_ms)
            return -EAGAIN;
        int remaining = -1;
        if (timeout_ms > 0) {
            uint64_t now;
            ret = isp->io.ops->now(isp->io.ctx, &now);
            if (ret < 0)
                return cancel_error(isp, ret);
            if (now >= deadline)
                return -ETIMEDOUT;
            uint64_t ns = deadline - now;
            uint64_t ms = ns / UINT64_C(1000000) + (ns % UINT64_C(1000000) != 0);
            remaining = ms > INT_MAX ? INT_MAX : (int)ms;
        }
        for (size_t i = 0; i < 4; ++i) {
            if (isp->done & (1U << i))
                continue;
            cc_frame frame;
            ret = cc_stream_dequeue(isp->streams[i], &frame, remaining);
            if (ret == -ETIMEDOUT)
                return ret;
            if (ret < 0)
                return cancel_error(isp, ret);
            ret = accept_completion(isp, i, &frame);
            if (ret < 0)
                return ret;
            break;
        }
    }
}

int cc_vc4_isp_release(cc_vc4_isp *isp, uint32_t index)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    if (index >= isp->count || isp->states[index] != 2)
        return -EINVAL;
    for (size_t i = 1; i < 3; ++i) if (isp->streams[i]->memory == V4L2_MEMORY_DMABUF) {
        ret = cc_stream_unimport(isp->streams[i], index);
        if (ret < 0) return cancel_error(isp, ret);
    }
    isp->states[index] = 0;
    return 0;
}

int cc_vc4_isp_fd(const cc_vc4_isp *isp, enum cc_vc4_port port)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    return (unsigned int)port > CC_VC4_STATS ? -EINVAL : cc_stream_fd(isp->streams[port]);
}

int cc_vc4_isp_export_plane(cc_vc4_isp *isp, enum cc_vc4_port port, uint32_t index, uint32_t plane)
{
    int ret = usable(isp);
    if (ret < 0)
        return ret;
    if (port < CC_VC4_MAIN || port > CC_VC4_STATS || index >= isp->count || isp->states[index] != 2)
        return -EINVAL;
    return cc_stream_export_plane(isp->streams[port], index, plane);
}

int cc_vc4_isp_query_control(cc_vc4_isp *isp, uint32_t id, cc_control_info *info)
{
    int ret = usable(isp);
    return ret < 0 ? ret : cc_video_query_control(isp->devices[0], id, info);
}
int cc_vc4_isp_query_menu(cc_vc4_isp *isp,uint32_t id,uint32_t index,cc_control_menu *entry)
{
    int ret=usable(isp);
    return ret<0 ? ret:cc_video_query_menu(isp->devices[0],id,index,entry);
}

int cc_vc4_isp_next_control(cc_vc4_isp *isp, uint32_t after_id, cc_control_info *info)
{
    int ret = usable(isp);
    return ret < 0 ? ret : cc_video_next_control(isp->devices[0], after_id, info);
}

int cc_vc4_isp_get_controls(cc_vc4_isp *isp, cc_control *controls, size_t count, size_t *error_index)
{
    int ret = usable(isp);
    return ret < 0 ? ret : cc_video_get_controls(isp->devices[0], controls, count, error_index);
}

int cc_vc4_isp_set_controls(cc_vc4_isp *isp, const cc_control *controls, size_t count, size_t *error_index)
{
    int ret = usable(isp);
    return ret < 0 ? ret : cc_video_set_controls(isp->devices[0], controls, count, error_index);
}

int cc_vc4_isp_set_tuning(cc_vc4_isp *isp,const cc_vc4_tuning *t)
{
    int ret=usable(isp);
    if (ret<0) return ret;
    if (!isp->configured) return -EPIPE;
    if (isp->active>=0) return -EBUSY;
    ret=cc_vc4_validate_tuning(t);
    if (ret<0) return ret;
    struct bcm2835_isp_lens_shading ls={0};
    if (t->lens_shading_enabled) {
        if (isp->shading_fd<0) {
            const char *paths[]={"/dev/dma_heap/linux,cma","/dev/dma_heap/reserved"};
            int heap=-1;
            for (unsigned i=0;i<2;++i) {
                heap=isp->io.ops->open(isp->io.ctx,paths[i],O_RDWR|O_CLOEXEC);
                if (heap>=0) break;
            }
            if (heap<0) return heap;
            struct dma_heap_allocation_data a={.len=64U*49U*8U,.fd_flags=O_RDWR|O_CLOEXEC};
            ret=cc_ioctl(&isp->io,heap,DMA_HEAP_IOCTL_ALLOC,&a);
            isp->io.ops->close(isp->io.ctx,heap);
            if (ret<0) return ret;
            if (a.fd>INT_MAX) return -EPROTO;
            isp->shading_fd=(int)a.fd;
        }
        if (!isp->shading_map) {
            ret=isp->io.ops->map(isp->io.ctx,isp->shading_fd,64U*49U*8U,0,&isp->shading_map);
            if (ret<0) return ret; /* Retain fd until ISP handles close, even on a failed map. */
        }
        uint32_t cell=16,w,h;
        for (;;) {
            w=(isp->configuration.input.width+cell-1)/cell;
            h=(isp->configuration.input.height+cell-1)/cell;
            if (w<64 && h<=48) break;
            if (cell==256) return -ERANGE;
            cell*=2;
        }
        ++w;++h;
        struct dma_buf_sync sync={.flags=DMA_BUF_SYNC_START|DMA_BUF_SYNC_WRITE};
        ret=cc_ioctl(&isp->io,isp->shading_fd,DMA_BUF_IOCTL_SYNC,&sync);
    if (ret<0) return ret;
        uint16_t *grid=isp->shading_map;
        for (unsigned ch=0;ch<4;++ch) for (uint32_t y=0;y<h;++y) for (uint32_t x=0;x<w;++x) {
            double u=(double)x*16/(w-1)-0.5,v=(double)y*12/(h-1)-0.5;
            if (u<0) u=0;
    if (u>15) u=15;
    if (v<0) v=0;
    if (v>11) v=11;
            unsigned x0=(unsigned)u,y0=(unsigned)v,x1=x0<15 ? x0+1:x0,y1=y0<11 ? y0+1:y0;
            double a=u-x0,b=v-y0;const double *s=t->lens_shading[ch==3 ? 2:ch==0 ? 0:1];
            double gain=(s[y0*16+x0]*(1-a)+s[y0*16+x1]*a)*(1-b)+(s[y1*16+x0]*(1-a)+s[y1*16+x1]*a)*b;
            grid[ch*w*h+y*w+x]=(uint16_t)(gain*1024+0.5);
        }
        sync.flags=DMA_BUF_SYNC_END|DMA_BUF_SYNC_WRITE;
        ret=cc_ioctl(&isp->io,isp->shading_fd,DMA_BUF_IOCTL_SYNC,&sync);
    if (ret<0) return ret;
        ls=(struct bcm2835_isp_lens_shading){.enabled=1,.grid_cell_size=cell,.grid_width=w,.grid_height=h,
            .grid_stride=w,.dmabuf=isp->shading_fd,.corner_sampled=1,.gain_format=GAIN_FORMAT_U4P10};
    }
    cc_control controls[]={
#define PAYLOAD(cid,member) {.id=cid,.data=(void *)&t->member,.size=sizeof(t->member)}
        PAYLOAD(V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL,black),PAYLOAD(V4L2_CID_USER_BCM2835_ISP_GAMMA,gamma),
        PAYLOAD(V4L2_CID_USER_BCM2835_ISP_CC_MATRIX,ccm),PAYLOAD(V4L2_CID_USER_BCM2835_ISP_GEQ,geq),
        PAYLOAD(V4L2_CID_USER_BCM2835_ISP_DENOISE,denoise),PAYLOAD(V4L2_CID_USER_BCM2835_ISP_CDN,cdn),
        PAYLOAD(V4L2_CID_USER_BCM2835_ISP_SHARPEN,sharpen),PAYLOAD(V4L2_CID_USER_BCM2835_ISP_DPC,dpc),
        {.id=V4L2_CID_USER_BCM2835_ISP_LENS_SHADING,.data=&ls,.size=sizeof(ls)}
#undef PAYLOAD
    };
    return cc_vc4_isp_set_controls(isp,controls,sizeof(controls)/sizeof(controls[0]),NULL);
}
int cc_vc4_isp_submit_tuned(cc_vc4_isp *isp,uint32_t index,const cc_dmabuf_plane *planes,
    uint32_t n,const cc_frame *input,uint64_t cookie,const cc_vc4_tuning *t)
{
    int ret=cc_vc4_isp_set_tuning(isp,t);
    return ret<0 ? ret:cc_vc4_isp_submit(isp,index,planes,n,input,cookie);
}
