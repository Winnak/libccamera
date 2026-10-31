/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>

#define CC_BUFFER_LIMIT 256U

static bool is_output(const cc_stream *stream)
{
    return stream->type == V4L2_BUF_TYPE_VIDEO_OUTPUT ||
           stream->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
           stream->type == V4L2_BUF_TYPE_META_OUTPUT;
}

static bool is_meta(const cc_stream *stream)
{
    return stream->type == V4L2_BUF_TYPE_META_CAPTURE || stream->type == V4L2_BUF_TYPE_META_OUTPUT;
}

static int usable(const cc_stream *stream)
{
    if (!stream)
        return -EINVAL;
    return stream->device->fd < 0 ? -ENODEV : 0;
}

int cc_video_open_io(const char *path, const struct cc_io *io, cc_video_device **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!path || !path[0] || !io)
        return -EINVAL;
    cc_video_device *device = calloc(1, sizeof(*device));
    if (!device)
        return -ENOMEM;
    device->io = *io;
    device->references = 1;
    device->fd = io->ops->open(io->ctx, path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    int ret = device->fd;
    if (ret < 0)
        goto fail;
    ret = io->ops->lock(io->ctx, device->fd);
    if (ret < 0)
        goto fail;
    ret = cc_ioctl(io, device->fd, VIDIOC_QUERYCAP, &device->caps);
    if (ret < 0)
        goto fail;
    device->capabilities = device->caps.capabilities & V4L2_CAP_DEVICE_CAPS ?
                           device->caps.device_caps : device->caps.capabilities;
    if (!(device->capabilities & V4L2_CAP_STREAMING)) {
        ret = -ENOTSUP;
        goto fail;
    }
    *out = device;
    return 0;
fail:
    cc_video_close(device);
    return ret;
}

int cc_video_open(const char *path, cc_video_device **device)
{
    return cc_video_open_io(path, &cc_system_io, device);
}

void cc_video_close(cc_video_device *device)
{
    if (!device || --device->references)
        return;
    if (device->fd >= 0)
        device->io.ops->close(device->io.ctx, device->fd);
    free(device);
}

int cc_video_create_stream(cc_video_device *device, enum cc_stream_type type, cc_stream **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!device)
        return -EINVAL;
    if (device->fd < 0)
        return -ENODEV;
    uint32_t capability;
    switch (type) {
    case CC_STREAM_CAPTURE: capability = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_M2M; break;
    case CC_STREAM_OUTPUT: capability = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_VIDEO_M2M; break;
    case CC_STREAM_CAPTURE_MPLANE: capability = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_M2M_MPLANE; break;
    case CC_STREAM_OUTPUT_MPLANE: capability = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_VIDEO_M2M_MPLANE; break;
    case CC_STREAM_META_CAPTURE: capability = V4L2_CAP_META_CAPTURE; break;
    case CC_STREAM_META_OUTPUT: capability = V4L2_CAP_META_OUTPUT; break;
    default: return -EINVAL;
    }
    if (!(device->capabilities & capability))
        return -ENOTSUP;
    /* The SP/MP and image/metadata APIs address the same queue in a driver;
     * never allow two wrappers to take ownership of the same direction. */
    uint32_t direction = type == CC_STREAM_OUTPUT || type == CC_STREAM_OUTPUT_MPLANE ||
                         type == CC_STREAM_META_OUTPUT ? 2U : 1U;
    if (device->active_types & direction)
        return -EBUSY;
    if (device->references == UINT32_MAX)
        return -EOVERFLOW;
    cc_stream *stream = calloc(1, sizeof(*stream));
    if (!stream)
        return -ENOMEM;
    stream->device = device;
    stream->type = (enum v4l2_buf_type)type;
    stream->next = device->streams;
    device->streams = stream;
    device->active_types |= direction;
    ++device->references;
    *out = stream;
    return 0;
}

int cc_stream_fd(const cc_stream *stream)
{
    int ret = usable(stream);
    return ret < 0 ? ret : stream->device->fd;
}

int cc_stream_poll_events(const cc_stream *stream)
{
    int ret = usable(stream);
    return ret < 0 ? ret : is_output(stream) ? POLLOUT : POLLIN;
}

int cc_stream_enum_format(cc_stream *stream, uint32_t index, cc_format_info *format)
{
    int ret = usable(stream);
    if (ret < 0 || !format)
        return ret < 0 ? ret : -EINVAL;
    struct v4l2_fmtdesc desc = { .index = index, .type = stream->type };
    ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_ENUM_FMT, &desc);
    if (ret < 0)
        return ret == -EINVAL ? -ENOENT : ret;
    *format = (cc_format_info) { .fourcc = desc.pixelformat, .flags = desc.flags };
    cc_copy_string(format->description, sizeof(format->description),
                    (const char *)desc.description, sizeof(desc.description));
    return 0;
}

/* Access the stable metadata UAPI prefix through raw_data. Older installed
 * headers have only dataformat/buffersize; line metadata adds three u32 fields
 * without changing struct v4l2_format or its ioctl number. */
struct cc_meta_format { uint32_t fourcc, size, width, height, stride; };

static void format_to_v4l2(const cc_stream *stream, const cc_format *format, struct v4l2_format *v4l2)
{
    memset(v4l2, 0, sizeof(*v4l2));
    v4l2->type = stream->type;
    if (is_meta(stream)) {
        struct cc_meta_format meta = { format->fourcc, format->planes[0].size,
                                       format->width, format->height, format->planes[0].stride };
        memcpy(v4l2->fmt.raw_data, &meta, sizeof(meta));
    } else if (cc_multiplanar(stream)) {
        struct v4l2_pix_format_mplane *p = &v4l2->fmt.pix_mp;
        p->width = format->width;
        p->height = format->height;
        p->pixelformat = format->fourcc;
        p->field = format->field;
        p->colorspace = format->colorspace;
        p->ycbcr_enc = (uint8_t)format->ycbcr_enc;
        p->quantization = (uint8_t)format->quantization;
        p->xfer_func = (uint8_t)format->xfer_func;
        p->num_planes = (uint8_t)format->num_planes;
        for (uint32_t i = 0; i < format->num_planes; ++i) {
            p->plane_fmt[i].bytesperline = format->planes[i].stride;
            p->plane_fmt[i].sizeimage = format->planes[i].size;
        }
    } else {
        struct v4l2_pix_format *p = &v4l2->fmt.pix;
        p->width = format->width;
        p->height = format->height;
        p->pixelformat = format->fourcc;
        p->field = format->field;
        p->colorspace = format->colorspace;
        p->priv = stream->device->capabilities & V4L2_CAP_EXT_PIX_FORMAT ? V4L2_PIX_FMT_PRIV_MAGIC : 0;
        p->ycbcr_enc = format->ycbcr_enc;
        p->quantization = format->quantization;
        p->xfer_func = format->xfer_func;
        p->bytesperline = format->planes[0].stride;
        p->sizeimage = format->planes[0].size;
    }
}

static int format_from_v4l2(const cc_stream *stream, const struct v4l2_format *v4l2, cc_format *out)
{
    cc_format f = {0};
    if (is_meta(stream)) {
        struct cc_meta_format meta;
        memcpy(&meta, v4l2->fmt.raw_data, sizeof(meta));
        f.fourcc = meta.fourcc;
        f.width = meta.width;
        f.height = meta.height;
        f.num_planes = 1;
        f.planes[0] = (cc_plane_format) { meta.stride, meta.size };
    } else if (cc_multiplanar(stream)) {
        const struct v4l2_pix_format_mplane *p = &v4l2->fmt.pix_mp;
        f.width = p->width;
        f.height = p->height;
        f.fourcc = p->pixelformat;
        f.field = p->field;
        f.colorspace = p->colorspace;
        f.ycbcr_enc = p->ycbcr_enc;
        f.quantization = p->quantization;
        f.xfer_func = p->xfer_func;
        f.num_planes = p->num_planes;
        if (!f.num_planes || f.num_planes > CC_MAX_PLANES)
            return -EPROTO;
        for (uint32_t i = 0; i < f.num_planes; ++i)
            f.planes[i] = (cc_plane_format) { p->plane_fmt[i].bytesperline, p->plane_fmt[i].sizeimage };
    } else {
        const struct v4l2_pix_format *p = &v4l2->fmt.pix;
        f.width = p->width;
        f.height = p->height;
        f.fourcc = p->pixelformat;
        f.field = p->field;
        f.colorspace = p->colorspace;
        if ((stream->device->capabilities & V4L2_CAP_EXT_PIX_FORMAT) && p->priv == V4L2_PIX_FMT_PRIV_MAGIC) {
            f.ycbcr_enc = p->ycbcr_enc;
            f.quantization = p->quantization;
            f.xfer_func = p->xfer_func;
        }
        f.num_planes = 1;
        f.planes[0] = (cc_plane_format) { p->bytesperline, p->sizeimage };
    }
    if (!f.fourcc || (!is_meta(stream) && (!f.width || !f.height)))
        return -EPROTO;
    for (uint32_t i = 0; i < f.num_planes; ++i)
        if (!f.planes[i].size)
            return -EPROTO;
    *out = f;
    return 0;
}

int cc_stream_get_format(cc_stream *stream, cc_format *format)
{
    int ret = usable(stream);
    if (ret < 0 || !format)
        return ret < 0 ? ret : -EINVAL;
    struct v4l2_format v4l2 = { .type = stream->type };
    ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_G_FMT, &v4l2);
    return ret < 0 ? ret : format_from_v4l2(stream, &v4l2, format);
}

static int negotiate_format(cc_stream *stream, cc_format *format, bool active)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (!format || !format->fourcc || format->num_planes > CC_MAX_PLANES ||
        ((!cc_multiplanar(stream)) && format->num_planes > 1) ||
        (!is_meta(stream) && (!format->width || !format->height)) ||
        format->ycbcr_enc > UINT8_MAX || format->quantization > UINT8_MAX || format->xfer_func > UINT8_MAX)
        return -EINVAL;
    if (active && (stream->allocated || stream->streaming))
        return -EBUSY;
    if (active)
        stream->configured = false;
    struct v4l2_format v4l2;
    format_to_v4l2(stream, format, &v4l2);
    ret = cc_ioctl(&stream->device->io, stream->device->fd, active ? VIDIOC_S_FMT : VIDIOC_TRY_FMT, &v4l2);
    if (ret < 0)
        return ret;
    cc_format actual;
    ret = format_from_v4l2(stream, &v4l2, &actual);
    if (ret < 0)
        return ret;
    if (active) {
        stream->format = actual;
        stream->configured = true;
    }
    *format = actual;
    return 0;
}

int cc_stream_try_format(cc_stream *stream, cc_format *format) { return negotiate_format(stream, format, false); }
int cc_stream_configure(cc_stream *stream, cc_format *format) { return negotiate_format(stream, format, true); }

static int selection(cc_stream *stream, uint32_t target, uint32_t flags, cc_rectangle *rectangle, bool set)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (!rectangle || (set && (!rectangle->width || !rectangle->height)))
        return -EINVAL;
    if (is_meta(stream))
        return -ENOTSUP;
    if (stream->poisoned || stream->release_pending)
        return -EPIPE;
    struct v4l2_selection sel = { .type = stream->type, .target = target, .flags = flags };
    if (set)
        sel.r = (struct v4l2_rect) { rectangle->x, rectangle->y, rectangle->width, rectangle->height };
    ret = cc_ioctl(&stream->device->io, stream->device->fd, set ? VIDIOC_S_SELECTION : VIDIOC_G_SELECTION, &sel);
    if (ret < 0)
        return ret;
    if (!sel.r.width || !sel.r.height) {
        if (set) {
            stream->configured = false;
            stream->poisoned = stream->allocated;
        }
        return -EPROTO;
    }
    if (set && stream->configured) {
        cc_format actual;
        ret = cc_stream_get_format(stream, &actual);
        if (ret < 0 || memcmp(&actual, &stream->format, sizeof(actual))) {
            stream->configured = false;
            stream->poisoned = stream->allocated;
            return ret < 0 ? ret : -EPIPE;
        }
    }
    *rectangle = (cc_rectangle) { sel.r.left, sel.r.top, sel.r.width, sel.r.height };
    return 0;
}

int cc_stream_get_selection(cc_stream *stream, uint32_t target, cc_rectangle *rectangle)
{
    return selection(stream, target, 0, rectangle, false);
}

int cc_stream_set_selection(cc_stream *stream, uint32_t target, uint32_t flags, cc_rectangle *rectangle)
{
    return selection(stream, target, flags, rectangle, true);
}

static void init_buffer(const cc_stream *stream, uint32_t index,
                         struct v4l2_buffer *buffer, struct v4l2_plane *planes)
{
    memset(buffer, 0, sizeof(*buffer));
    memset(planes, 0, CC_MAX_PLANES * sizeof(*planes));
    buffer->type = stream->type;
    buffer->memory = stream->memory;
    buffer->index = index;
    if (cc_multiplanar(stream)) {
        buffer->m.planes = planes;
        buffer->length = CC_MAX_PLANES;
    }
}

static int unmap_buffers(cc_stream *stream)
{
    const struct cc_io *io = &stream->device->io;
    int first_error = 0;
    for (uint32_t i = 0; i < stream->num_buffers; ++i) {
        struct cc_buffer *slot = &stream->buffers[i];
        for (uint32_t j = 0; j < slot->frame.num_planes; ++j) {
            if (!slot->mapped[j])
                continue;
            cc_plane *plane = &slot->frame.planes[j];
            int ret = io->ops->unmap(io->ctx, plane->data, plane->length);
            if (ret < 0) {
                if (!first_error)
                    first_error = ret;
                continue;
            }
            slot->mapped[j] = false;
            plane->data = NULL;
        }
    }
    return first_error;
}

static void clear_buffers(cc_stream *stream)
{
    const struct cc_io *io = &stream->device->io;
    for (uint32_t i = 0; i < stream->num_buffers; ++i)
        for (uint32_t j = 0; j < stream->buffers[i].frame.num_planes; ++j)
            if (stream->buffers[i].fds[j] >= 0)
                io->ops->close(io->ctx, stream->buffers[i].fds[j]);
    free(stream->buffers);
    stream->buffers = NULL;
    stream->num_buffers = 0;
    stream->queued = 0;
    stream->allocated = stream->release_pending = false;
}

int cc_stream_stop(cc_stream *stream)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (stream->streaming || stream->queued || stream->poisoned) {
        enum v4l2_buf_type type = stream->type;
        ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_STREAMOFF, &type);
        if (ret < 0)
            return ret;
    }
    stream->streaming = stream->poisoned = false;
    stream->queued = 0;
    for (uint32_t i = 0; i < stream->num_buffers; ++i) {
        stream->buffers[i].queued = false;
        for (uint32_t j = 0; j < stream->buffers[i].frame.num_planes; ++j) {
            stream->buffers[i].frame.planes[j].bytes_used = 0;
            stream->buffers[i].frame.planes[j].data_offset = 0;
        }
    }
    return 0;
}

int cc_stream_free_buffers(cc_stream *stream)
{
    if (!stream)
        return -EINVAL;
    if (!stream->allocated)
        return 0;
    bool alive = stream->device->fd >= 0;
    if (alive) {
        int ret = cc_stream_stop(stream);
        if (ret < 0)
            return ret;
    }
    stream->release_pending = true;
    int ret = unmap_buffers(stream);
    if (ret < 0)
        return ret;
    /* Canonical MMAP exports retain stable DMA-BUF identity for request graph
     * alias checks. Drop our references before asking vb2 to release backing
     * storage; caller exports can still make REQBUFS(0) return EBUSY. */
    if (stream->memory == V4L2_MEMORY_MMAP)
        for (uint32_t i = 0; i < stream->num_buffers; ++i)
            for (uint32_t p = 0; p < stream->buffers[i].frame.num_planes; ++p)
                if (stream->buffers[i].fds[p] >= 0) {
                    stream->device->io.ops->close(stream->device->io.ctx, stream->buffers[i].fds[p]);
                    stream->buffers[i].fds[p] = -1;
                }
    if (alive) {
        struct v4l2_requestbuffers request = { .type = stream->type, .memory = stream->memory };
        ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_REQBUFS, &request);
        if (ret < 0)
            return ret;
    }
    clear_buffers(stream);
    return 0;
}

void cc_stream_close(cc_stream *stream)
{
    if (!stream)
        return;
    cc_video_device *device = stream->device;
    if (cc_stream_free_buffers(stream) < 0) {
        /* A queue with live DMA must never outlive its mappings or imported
         * fds. Close the shared handle if normal cancellation is impossible. */
        if (device->fd >= 0) {
            device->io.ops->close(device->io.ctx, device->fd);
            device->fd = -1;
        }
        unmap_buffers(stream);
        clear_buffers(stream);
    }
    device->active_types &= is_output(stream) ? ~2U : ~1U;
    cc_stream **entry = &device->streams;
    while (*entry != stream)
        entry = &(*entry)->next;
    *entry = stream->next;
    free(stream);
    cc_video_close(device);
}

int cc_stream_allocate_buffers(cc_stream *stream, enum cc_memory memory, uint32_t *count)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (!count || !*count || *count > CC_BUFFER_LIMIT ||
        (memory != CC_MEMORY_MMAP && memory != CC_MEMORY_DMABUF))
        return -EINVAL;
    if (stream->allocated || stream->streaming)
        return -EBUSY;
    if (!stream->configured)
        return -EPIPE;
    cc_format current;
    ret = cc_stream_get_format(stream, &current);
    if (ret < 0)
        return ret;
    /* Another queue's S_FMT (notably on M2M devices) can change this queue. */
    if (memcmp(&current, &stream->format, sizeof(current))) {
        stream->configured = false;
        return -EPIPE;
    }
    struct v4l2_requestbuffers request = {
        .count = *count, .type = stream->type, .memory = (uint32_t)memory
    };
    ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_REQBUFS, &request);
    if (ret < 0)
        return ret;
    stream->memory = (enum v4l2_memory)memory;
    stream->allocated = true;
    if (!request.count || request.count > CC_BUFFER_LIMIT) {
        ret = request.count ? -EOVERFLOW : -ENOMEM;
        goto release;
    }
    stream->buffers = calloc(request.count, sizeof(*stream->buffers));
    if (!stream->buffers) {
        ret = -ENOMEM;
        goto release;
    }
    stream->num_buffers = request.count;
    for (uint32_t i = 0; i < request.count; ++i) {
        struct cc_buffer *slot = &stream->buffers[i];
        slot->frame.index = i;
        slot->frame.num_planes = stream->format.num_planes;
        for (uint32_t j = 0; j < CC_MAX_PLANES; ++j)
            slot->fds[j] = -1;
    }
    if (memory == CC_MEMORY_DMABUF) {
        *count = stream->num_buffers;
        return 0;
    }
    const struct cc_io *io = &stream->device->io;
    for (uint32_t i = 0; i < request.count; ++i) {
        struct v4l2_buffer buffer;
        struct v4l2_plane planes[CC_MAX_PLANES];
        init_buffer(stream, i, &buffer, planes);
        ret = cc_ioctl(io, stream->device->fd, VIDIOC_QUERYBUF, &buffer);
        if (ret < 0)
            goto release;
        uint32_t num_planes = cc_multiplanar(stream) ? buffer.length : 1;
        if (num_planes != stream->format.num_planes) {
            ret = -EPROTO;
            goto release;
        }
        struct cc_buffer *slot = &stream->buffers[i];
        for (uint32_t j = 0; j < num_planes; ++j) {
            uint32_t length = cc_multiplanar(stream) ? planes[j].length : buffer.length;
            off_t offset = (off_t)(cc_multiplanar(stream) ? planes[j].m.mem_offset : buffer.m.offset);
            if (length < stream->format.planes[j].size) {
                ret = -EPROTO;
                goto release;
            }
            ret = io->ops->map(io->ctx, stream->device->fd, length, offset, &slot->frame.planes[j].data);
            if (ret < 0)
                goto release;
            slot->mapped[j] = true;
            slot->frame.planes[j].length = length;
            slot->offsets[j] = offset;
        }
        slot->bound = true;
    }
    *count = stream->num_buffers;
    return 0;
release:
    cc_stream_free_buffers(stream);
    return ret;
}

static int accessible(cc_stream *stream, uint32_t index)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (index >= stream->num_buffers)
        return -EINVAL;
    if (stream->release_pending)
        return -ENXIO;
    if (stream->poisoned)
        return -EPIPE;
    return 0;
}

int cc_stream_import(cc_stream *stream, uint32_t index, const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    int ret = accessible(stream, index);
    if (ret < 0)
        return ret;
    if (stream->memory != V4L2_MEMORY_DMABUF || !planes || num_planes != stream->format.num_planes)
        return -EINVAL;
    struct cc_buffer *slot = &stream->buffers[index];
    if (slot->queued)
        return -EBUSY;
    const struct cc_io *io = &stream->device->io;
    if (!io->ops->duplicate)
        return -ENOTSUP;
    for (uint32_t i = 0; i < num_planes; ++i)
        if (planes[i].fd < 0 || planes[i].length < stream->format.planes[i].size)
            return -EINVAL;
    int fds[CC_MAX_PLANES];
    uint32_t created = 0;
    for (; created < num_planes; ++created) {
        ret = io->ops->duplicate(io->ctx, planes[created].fd);
        if (ret < 0) {
            while (created)
                io->ops->close(io->ctx, fds[--created]);
            return ret;
        }
        fds[created] = ret;
    }
    for (uint32_t i = 0; i < num_planes; ++i) {
        if (slot->fds[i] >= 0)
            io->ops->close(io->ctx, slot->fds[i]);
        slot->fds[i] = fds[i];
        slot->frame.planes[i] = (cc_plane) { .length = planes[i].length };
    }
    slot->bound = true;
    return 0;
}

int cc_stream_unimport(cc_stream *stream, uint32_t index)
{
    int ret = accessible(stream, index);
    if (ret < 0)
        return ret;
    if (stream->memory != V4L2_MEMORY_DMABUF)
        return -EINVAL;
    struct cc_buffer *slot = &stream->buffers[index];
    if (slot->queued)
        return -EBUSY;
    const struct cc_io *io = &stream->device->io;
    for (uint32_t i = 0; i < slot->frame.num_planes; ++i) {
        if (slot->fds[i] >= 0)
            io->ops->close(io->ctx, slot->fds[i]);
        slot->fds[i] = -1;
        slot->frame.planes[i] = (cc_plane) {0};
    }
    slot->bound = false;
    return 0;
}

int cc_stream_clear_imports(cc_stream *stream)
{
    if (!stream || !stream->allocated || stream->memory != V4L2_MEMORY_DMABUF) return 0;
    for (uint32_t i = 0; i < stream->num_buffers; ++i) {
        int ret = cc_stream_unimport(stream, i);
        if (ret < 0) return ret;
    }
    return 0;
}

int cc_stream_buffer(cc_stream *stream, uint32_t index, cc_frame *buffer)
{
    int ret = accessible(stream, index);
    if (ret < 0 || !buffer)
        return ret < 0 ? ret : -EINVAL;
    struct cc_buffer *slot = &stream->buffers[index];
    if (slot->queued)
        return -EBUSY;
    if (!slot->bound)
        return -ENXIO;
    *buffer = slot->frame;
    return 0;
}

int cc_stream_export_plane(cc_stream *stream, uint32_t index, uint32_t plane)
{
    int ret = accessible(stream, index);
    if (ret < 0)
        return ret;
    struct cc_buffer *slot = &stream->buffers[index];
    if (plane >= slot->frame.num_planes)
        return -EINVAL;
    if (!slot->bound)
        return -ENXIO;
    const struct cc_io *io = &stream->device->io;
    if (stream->memory == V4L2_MEMORY_DMABUF || slot->fds[plane] >= 0)
        return io->ops->duplicate(io->ctx, slot->fds[plane]);
    struct v4l2_exportbuffer export = {
        .type = stream->type, .index = index, .plane = plane, .flags = O_CLOEXEC
    };
    ret = cc_ioctl(io, stream->device->fd, VIDIOC_EXPBUF, &export);
    return ret < 0 ? ret : export.fd;
}

int cc_stream_canonical_export(cc_stream *stream, uint32_t index, uint32_t plane)
{
    int ret = accessible(stream, index);
    if (ret < 0) return ret;
    struct cc_buffer *slot = &stream->buffers[index];
    if (plane >= slot->frame.num_planes || !slot->bound) return -EINVAL;
    const struct cc_io *io = &stream->device->io;
    if (!io->ops->duplicate) return -ENOTSUP;
    if (stream->memory == V4L2_MEMORY_MMAP && slot->fds[plane] < 0) {
        ret = cc_stream_export_plane(stream, index, plane);
        if (ret < 0) return ret;
        slot->fds[plane] = ret;
    }
    return cc_stream_export_plane(stream, index, plane);
}

int cc_stream_queue(cc_stream *stream, uint32_t index, const cc_frame *payload)
{
    int ret = accessible(stream, index);
    if (ret < 0)
        return ret;
    struct cc_buffer *slot = &stream->buffers[index];
    if (slot->queued)
        return -EBUSY;
    if (!slot->bound)
        return -ENXIO;
    if (is_output(stream) != (payload != NULL))
        return -EINVAL;
    struct v4l2_buffer buffer;
    struct v4l2_plane planes[CC_MAX_PLANES];
    init_buffer(stream, index, &buffer, planes);
    if (payload) {
        if (payload->num_planes != slot->frame.num_planes || payload->timestamp_ns / UINT64_C(1000000000) > LONG_MAX)
            return -EINVAL;
        for (uint32_t i = 0; i < payload->num_planes; ++i) {
            if (payload->planes[i].bytes_used > slot->frame.planes[i].length ||
                payload->planes[i].data_offset > payload->planes[i].bytes_used ||
                (!cc_multiplanar(stream) && payload->planes[i].data_offset))
                return -EINVAL;
        }
        buffer.field = payload->field;
        buffer.timestamp.tv_sec = (long)(payload->timestamp_ns / UINT64_C(1000000000));
        buffer.timestamp.tv_usec = (long)((payload->timestamp_ns % UINT64_C(1000000000)) / 1000);
    }
    if (cc_multiplanar(stream)) {
        buffer.length = slot->frame.num_planes;
        for (uint32_t i = 0; i < buffer.length; ++i) {
            planes[i].length = (uint32_t)slot->frame.planes[i].length;
            if (stream->memory == V4L2_MEMORY_DMABUF)
                planes[i].m.fd = slot->fds[i];
            else
                planes[i].m.mem_offset = (uint32_t)slot->offsets[i];
            if (payload) {
                planes[i].bytesused = (uint32_t)payload->planes[i].bytes_used;
                planes[i].data_offset = (uint32_t)payload->planes[i].data_offset;
            }
        }
    } else {
        buffer.length = (uint32_t)slot->frame.planes[0].length;
        if (stream->memory == V4L2_MEMORY_DMABUF)
            buffer.m.fd = slot->fds[0];
        else
            buffer.m.offset = (uint32_t)slot->offsets[0];
        if (payload)
            buffer.bytesused = (uint32_t)payload->planes[0].bytes_used;
    }
    ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_QBUF, &buffer);
    if (ret < 0)
        return ret;
    slot->queued = true;
    ++stream->queued;
    return 0;
}

int cc_stream_start(cc_stream *stream)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (stream->streaming)
        return -EBUSY;
    if (!stream->configured || !stream->allocated || !stream->queued || stream->poisoned || stream->release_pending)
        return -EPIPE;
    enum v4l2_buf_type type = stream->type;
    ret = cc_ioctl(&stream->device->io, stream->device->fd, VIDIOC_STREAMON, &type);
    if (ret < 0)
        return ret;
    stream->streaming = true;
    return 0;
}

static int finish_dequeue(cc_stream *stream, const struct v4l2_buffer *buffer,
                           const struct v4l2_plane *planes, cc_frame *out)
{
    stream->poisoned = true; /* Cleared only once all completion metadata is valid. */
    if (buffer->index >= stream->num_buffers || !stream->buffers[buffer->index].queued)
        return -EPROTO;
    struct cc_buffer *slot = &stream->buffers[buffer->index];
    slot->queued = false;
    --stream->queued;
    if (cc_multiplanar(stream) && buffer->length != slot->frame.num_planes)
        return -EPROTO;
    cc_frame frame = slot->frame;
    frame.sequence = buffer->sequence;
    frame.flags = buffer->flags;
    frame.field = buffer->field;
    if (buffer->timestamp.tv_sec < 0 || buffer->timestamp.tv_usec < 0 || buffer->timestamp.tv_usec >= 1000000 ||
        (uint64_t)buffer->timestamp.tv_sec > (UINT64_MAX - UINT64_C(999999999)) / UINT64_C(1000000000))
        return -EPROTO;
    frame.timestamp_ns = (uint64_t)buffer->timestamp.tv_sec * UINT64_C(1000000000) +
                         (uint64_t)buffer->timestamp.tv_usec * UINT64_C(1000);
    for (uint32_t i = 0; i < frame.num_planes; ++i) {
        size_t used = cc_multiplanar(stream) ? planes[i].bytesused : buffer->bytesused;
        size_t offset = cc_multiplanar(stream) ? planes[i].data_offset : 0;
        if (used > frame.planes[i].length || offset > used)
            return -EPROTO;
        frame.planes[i].bytes_used = used;
        frame.planes[i].data_offset = offset;
    }
    slot->frame = frame;
    *out = frame;
    stream->poisoned = false;
    return 0;
}

int cc_stream_dequeue(cc_stream *stream, cc_frame *frame, int timeout_ms)
{
    int ret = usable(stream);
    if (ret < 0)
        return ret;
    if (!frame || timeout_ms < -1)
        return -EINVAL;
    if (!stream->streaming || stream->poisoned)
        return -EPIPE;
    const struct cc_io *io = &stream->device->io;
    uint64_t deadline = 0;
    if (timeout_ms > 0) {
        ret = io->ops->now(io->ctx, &deadline);
        if (ret < 0)
            return ret;
        uint64_t duration = (uint64_t)timeout_ms * UINT64_C(1000000);
        deadline = deadline > UINT64_MAX - duration ? UINT64_MAX : deadline + duration;
    }
    for (;;) {
        struct v4l2_buffer buffer;
        struct v4l2_plane planes[CC_MAX_PLANES];
        init_buffer(stream, 0, &buffer, planes);
        ret = cc_ioctl(io, stream->device->fd, VIDIOC_DQBUF, &buffer);
        if (ret >= 0)
            return finish_dequeue(stream, &buffer, planes, frame);
        if (ret != -EAGAIN || !timeout_ms)
            return ret;
        int remaining = -1;
        if (timeout_ms > 0) {
            uint64_t now;
            ret = io->ops->now(io->ctx, &now);
            if (ret < 0)
                return ret;
            if (now >= deadline)
                return -ETIMEDOUT;
            uint64_t ns = deadline - now;
            uint64_t ms = ns / UINT64_C(1000000) + (ns % UINT64_C(1000000) != 0);
            remaining = ms > INT_MAX ? INT_MAX : (int)ms;
        }
        ret = io->ops->wait(io->ctx, stream->device->fd, (short)cc_stream_poll_events(stream), remaining);
        if (ret < 0 && ret != -EINTR && ret != -EAGAIN)
            return ret;
    }
}
