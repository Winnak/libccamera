/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x, e) CHECK((x) == -(e))
#define SLOTS 3
#define OBJECTS 128
#define FDS 512

struct object { void *data; size_t size; int refs, maps; bool kernel; int owner, type; };
struct submission { bool queued, done; cc_frame payload; int objects[CC_MAX_PLANES]; };
struct queue {
    cc_format format;
    uint32_t count, memory;
    bool streaming;
    int objects[SLOTS][CC_MAX_PLANES];
    struct submission submissions[SLOTS];
};
struct node { uint32_t caps; bool open; int fd; struct queue queues[15]; };
struct fd { int kind, object; };
struct fake {
    struct node nodes[3];
    struct object objects[OBJECTS];
    struct fd fds[FDS];
    int next_fd, next_object, mapped, open_fds, waits, duplicates, fail_duplicate;
    int fail_errno, fail_nth;
    unsigned long fail_request;
    short last_events;
    bool waiting, corrupt, process;
    int processed_object;
    uint64_t now;
};

static bool output(uint32_t type) { return type == 2 || type == 10 || type == 14; }
static bool multi(uint32_t type) { return type == 9 || type == 10; }
static bool meta(uint32_t type) { return type == 13 || type == 14; }

static void object_maybe_free(struct object *o)
{
    if (!o->kernel && !o->refs && !o->maps) {
        free(o->data);
        o->data = NULL;
    }
}

static int make_fd(struct fake *f, int kind, int object)
{
    CHECK(f->next_fd < FDS);
    int fd = f->next_fd++;
    f->fds[fd] = (struct fd) { kind, object };
    if (kind == 2)
        ++f->objects[object].refs;
    ++f->open_fds;
    return fd;
}

static int make_object(struct fake *f, uint32_t size, int owner, int type)
{
    CHECK(f->next_object < OBJECTS);
    int id = f->next_object++;
    f->objects[id] = (struct object) { .data = calloc(1, size), .size = size,
                                      .owner = owner, .type = type, .kernel = true };
    CHECK(f->objects[id].data);
    return id;
}

static int fake_open(void *ctx, const char *path, int flags)
{
    struct fake *f = ctx;
    CHECK((flags & (O_RDWR | O_NONBLOCK | O_CLOEXEC)) == (O_RDWR | O_NONBLOCK | O_CLOEXEC));
    int node = !strcmp(path, "/dev/raw") ? 0 : !strcmp(path, "/dev/isp") ? 1 :
               !strcmp(path, "/dev/meta") ? 2 : -1;
    if (node < 0)
        return -ENOENT;
    CHECK(!f->nodes[node].open);
    f->nodes[node].open = true;
    f->nodes[node].fd = make_fd(f, 1, node);
    return f->nodes[node].fd;
}

static int fake_close(void *ctx, int fd)
{
    struct fake *f = ctx;
    CHECK(fd >= 0 && fd < f->next_fd && f->fds[fd].kind);
    if (f->fds[fd].kind == 2) {
        struct object *o = &f->objects[f->fds[fd].object];
        --o->refs;
        object_maybe_free(o);
    } else {
        struct node *node = &f->nodes[f->fds[fd].object];
        node->open = false;
        for (int type = 0; type < 15; ++type) {
            struct queue *q = &node->queues[type];
            q->streaming = false;
            if (q->memory == V4L2_MEMORY_MMAP)
                for (uint32_t i = 0; i < q->count; ++i)
                    for (uint32_t p = 0; p < q->format.num_planes; ++p) {
                        struct object *o = &f->objects[q->objects[i][p]];
                        o->kernel = false;
                        object_maybe_free(o);
                    }
            q->count = 0;
        }
    }
    f->fds[fd].kind = 0;
    --f->open_fds;
    return 0;
}

static int fake_duplicate(void *ctx, int fd)
{
    struct fake *f = ctx;
    if (++f->duplicates == f->fail_duplicate)
        return -EMFILE;
    if (fd < 0 || fd >= f->next_fd || f->fds[fd].kind != 2)
        return -EBADF;
    return make_fd(f, 2, f->fds[fd].object);
}

static int fake_lock(void *ctx, int fd) { (void)ctx; (void)fd; return 0; }
static int fake_now(void *ctx, uint64_t *out) { *out = ((struct fake *)ctx)->now; return 0; }
static int fake_wait(void *ctx, int fd, short events, int timeout)
{
    struct fake *f = ctx;
    CHECK(f->fds[fd].kind == 1 && timeout > 0);
    f->last_events = events;
    f->now += UINT64_C(1000000);
    ++f->waits;
    f->waiting = false;
    return f->waits == 1 ? -EINTR : 0;
}

static int fake_map(void *ctx, int fd, size_t length, off_t offset, void **out)
{
    struct fake *f = ctx;
    CHECK(f->fds[fd].kind == 1);
    struct node *node = &f->nodes[f->fds[fd].object];
    size_t encoded = (size_t)offset / 4096;
    size_t type = encoded / 100, slot = encoded % 100 / 8, plane = encoded % 100 % 8;
    CHECK(type < 15 && slot < node->queues[type].count && plane < node->queues[type].format.num_planes);
    struct object *o = &f->objects[node->queues[type].objects[slot][plane]];
    CHECK(o->kernel && o->size == length);
    ++o->maps;
    ++f->mapped;
    *out = o->data;
    return 0;
}

static int fake_unmap(void *ctx, void *data, size_t length)
{
    struct fake *f = ctx;
    for (int i = 0; i < f->next_object; ++i) {
        struct object *o = &f->objects[i];
        if (o->data == data && o->maps) {
            CHECK(o->size == length && !f->nodes[o->owner].queues[o->type].streaming);
            --o->maps;
            --f->mapped;
            object_maybe_free(o);
            return 0;
        }
    }
    CHECK(false);
    return -EINVAL;
}

static void read_format(struct queue *q, struct v4l2_format *v, bool set)
{
    uint32_t type = v->type;
    if (set) {
        cc_format format = { .num_planes = 1 };
        if (meta(type)) {
            uint32_t words[5];
            memcpy(words, v->fmt.raw_data, sizeof(words));
            format.fourcc = words[0];
            format.planes[0].size = words[1] ? words[1] : 128;
            format.width = words[2];
            format.height = words[3];
            format.planes[0].stride = words[4];
        } else if (multi(type)) {
            format.fourcc = v->fmt.pix_mp.pixelformat;
            format.width = v->fmt.pix_mp.width;
            format.height = v->fmt.pix_mp.height;
            format.num_planes = format.fourcc == V4L2_PIX_FMT_NV12M ? 2 : 1;
            for (uint32_t p = 0; p < format.num_planes; ++p) {
                format.planes[p].stride = v->fmt.pix_mp.plane_fmt[p].bytesperline;
                format.planes[p].size = v->fmt.pix_mp.plane_fmt[p].sizeimage;
            }
        } else {
            format.fourcc = v->fmt.pix.pixelformat;
            format.width = v->fmt.pix.width;
            format.height = v->fmt.pix.height;
            format.planes[0] = (cc_plane_format) { v->fmt.pix.bytesperline, v->fmt.pix.sizeimage };
        }
        if (!meta(type)) {
            format.field = V4L2_FIELD_NONE;
            format.colorspace = V4L2_COLORSPACE_RAW;
            for (uint32_t p = 0; p < format.num_planes; ++p) {
                if (!format.planes[p].stride) format.planes[p].stride = format.width * 2;
                if (!format.planes[p].size) format.planes[p].size = format.planes[p].stride * format.height;
            }
        }
        q->format = format;
    }
    cc_format *format = &q->format;
    if (meta(type)) {
        uint32_t words[] = { format->fourcc, format->planes[0].size, format->width,
                             format->height, format->planes[0].stride };
        memcpy(v->fmt.raw_data, words, sizeof(words));
    } else if (multi(type)) {
        v->fmt.pix_mp.width = format->width;
        v->fmt.pix_mp.height = format->height;
        v->fmt.pix_mp.pixelformat = format->fourcc;
        v->fmt.pix_mp.field = format->field;
        v->fmt.pix_mp.colorspace = format->colorspace;
        v->fmt.pix_mp.num_planes = (uint8_t)format->num_planes;
        for (uint32_t p = 0; p < format->num_planes; ++p) {
            v->fmt.pix_mp.plane_fmt[p].bytesperline = format->planes[p].stride;
            v->fmt.pix_mp.plane_fmt[p].sizeimage = format->planes[p].size;
        }
    } else {
        v->fmt.pix.width = format->width;
        v->fmt.pix.height = format->height;
        v->fmt.pix.pixelformat = format->fourcc;
        v->fmt.pix.field = format->field;
        v->fmt.pix.colorspace = format->colorspace;
        v->fmt.pix.bytesperline = format->planes[0].stride;
        v->fmt.pix.sizeimage = format->planes[0].size;
    }
}

static void process_isp(struct fake *f)
{
    struct node *node = &f->nodes[1];
    struct queue *input = &node->queues[V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE];
    struct queue *capture = &node->queues[V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE];
    if (!input->streaming || !capture->streaming)
        return;
    for (uint32_t i = 0; i < input->count; ++i) {
        struct submission *in = &input->submissions[i];
        if (!in->queued || in->done)
            continue;
        for (uint32_t j = 0; j < capture->count; ++j) {
            struct submission *out = &capture->submissions[j];
            if (!out->queued || out->done)
                continue;
            struct object *source = &f->objects[in->objects[0]];
            struct object *dest = &f->objects[out->objects[0]];
            CHECK(source->size == dest->size);
            memcpy(dest->data, source->data, source->size);
            out->payload.timestamp_ns = in->payload.timestamp_ns;
            out->payload.planes[0].bytes_used = in->payload.planes[0].bytes_used;
            f->processed_object = in->objects[0];
            in->done = out->done = true;
            return;
        }
    }
}

static int fake_ioctl(void *ctx, int fd, unsigned long req, void *arg)
{
    struct fake *f = ctx;
    CHECK(fd >= 0 && fd < f->next_fd && f->fds[fd].kind == 1);
    if (f->fail_request == req && f->fail_nth > 0 && --f->fail_nth == 0)
        return -f->fail_errno;
    int owner = f->fds[fd].object;
    struct node *node = &f->nodes[owner];
    if (req == VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *query = arg;
        if (query->id != V4L2_CID_HFLIP)
            return -EINVAL;
        query->type = V4L2_CTRL_TYPE_BOOLEAN;
        query->flags = V4L2_CTRL_FLAG_MODIFY_LAYOUT;
        query->minimum = 0;
        query->maximum = 1;
        query->step = 1;
        return 0;
    }
    if (req == VIDIOC_S_EXT_CTRLS) {
        struct v4l2_ext_controls *controls = arg;
        CHECK(controls->count == 1 && controls->controls[0].id == V4L2_CID_HFLIP);
        return 0;
    }
    if (req == VIDIOC_QUERYCAP) {
        struct v4l2_capability *caps = arg;
        caps->capabilities = V4L2_CAP_DEVICE_CAPS;
        caps->device_caps = node->caps | V4L2_CAP_STREAMING;
        return 0;
    }
    uint32_t type;
    memcpy(&type, arg, sizeof(type));
    CHECK(type < 15);
    struct queue *q = &node->queues[type];
    if (req == VIDIOC_S_FMT || req == VIDIOC_G_FMT || req == VIDIOC_TRY_FMT) {
        struct queue scratch = *q;
        read_format(req == VIDIOC_TRY_FMT ? &scratch : q, arg, req != VIDIOC_G_FMT);
        return 0;
    }
    if (req == VIDIOC_REQBUFS) {
        struct v4l2_requestbuffers *r = arg;
        /* requestbuffers begins with count, not type. */
        q = &node->queues[r->type];
        type = r->type;
        CHECK(r->memory == V4L2_MEMORY_MMAP || r->memory == V4L2_MEMORY_DMABUF);
        if (!r->count) {
            CHECK(!q->streaming);
            if (q->memory == V4L2_MEMORY_MMAP) {
                for (uint32_t i = 0; i < q->count; ++i)
                    for (uint32_t p = 0; p < q->format.num_planes; ++p) {
                        struct object *o = &f->objects[q->objects[i][p]];
                        CHECK(!o->maps);
                        if (o->refs)
                            return -EBUSY;
                    }
                for (uint32_t i = 0; i < q->count; ++i)
                    for (uint32_t p = 0; p < q->format.num_planes; ++p) {
                        struct object *o = &f->objects[q->objects[i][p]];
                        o->kernel = false;
                        object_maybe_free(o);
                    }
            }
            q->count = 0;
        } else {
            CHECK(!q->count);
            q->count = r->count = SLOTS;
            q->memory = r->memory;
            if (q->memory == V4L2_MEMORY_MMAP)
                for (uint32_t i = 0; i < q->count; ++i)
                    for (uint32_t p = 0; p < q->format.num_planes; ++p)
                        q->objects[i][p] = make_object(f, q->format.planes[p].size, owner, (int)type);
        }
        return 0;
    }
    if (req == VIDIOC_STREAMON || req == VIDIOC_STREAMOFF) {
        if (req == VIDIOC_STREAMON)
            q->streaming = true;
        else {
            q->streaming = false;
            memset(q->submissions, 0, sizeof(q->submissions));
        }
        return 0;
    }
    if (req == VIDIOC_EXPBUF) {
        struct v4l2_exportbuffer *e = arg;
        CHECK(e->flags == O_CLOEXEC && e->index < q->count && e->plane < q->format.num_planes);
        e->fd = make_fd(f, 2, q->objects[e->index][e->plane]);
        return 0;
    }
    struct v4l2_buffer *b = arg;
    /* v4l2_buffer begins with index, then type. */
    type = b->type;
    CHECK(type < 15);
    q = &node->queues[type];
    CHECK(b->memory == q->memory);
    if (req == VIDIOC_QUERYBUF) {
        CHECK(q->memory == V4L2_MEMORY_MMAP && b->index < q->count);
        if (multi(type)) {
            CHECK(b->length == CC_MAX_PLANES);
            b->length = q->format.num_planes;
            for (uint32_t p = 0; p < b->length; ++p) {
                b->m.planes[p].length = q->format.planes[p].size;
                b->m.planes[p].m.mem_offset = (type * 100 + b->index * 8 + p) * 4096;
            }
        } else {
            b->length = q->format.planes[0].size;
            b->m.offset = (type * 100 + b->index * 8) * 4096;
        }
        return 0;
    }
    if (req == VIDIOC_QBUF) {
        CHECK(b->index < q->count && b->flags == 0 && b->reserved2 == 0);
        struct submission *s = &q->submissions[b->index];
        CHECK(!s->queued);
        s->queued = true;
        s->done = false;
        s->payload.num_planes = q->format.num_planes;
        s->payload.timestamp_ns = (uint64_t)b->timestamp.tv_sec * UINT64_C(1000000000) +
                                  (uint64_t)b->timestamp.tv_usec * 1000;
        s->payload.field = b->field;
        CHECK(!multi(type) || b->length == q->format.num_planes);
        for (uint32_t p = 0; p < q->format.num_planes; ++p) {
            int imported_fd = multi(type) ? b->m.planes[p].m.fd : b->m.fd;
            if (q->memory == V4L2_MEMORY_DMABUF) {
                CHECK(imported_fd >= 0 && imported_fd < f->next_fd && f->fds[imported_fd].kind == 2);
                s->objects[p] = f->fds[imported_fd].object;
            } else {
                s->objects[p] = q->objects[b->index][p];
            }
            uint32_t length = multi(type) ? b->m.planes[p].length : b->length;
            CHECK(length == f->objects[s->objects[p]].size);
            s->payload.planes[p].bytes_used = multi(type) ? b->m.planes[p].bytesused : b->bytesused;
            s->payload.planes[p].data_offset = multi(type) ? b->m.planes[p].data_offset : 0;
            if (!output(type)) CHECK(s->payload.planes[p].bytes_used == 0);
        }
        return 0;
    }
    CHECK(req == VIDIOC_DQBUF && q->streaming);
    if (f->waiting)
        return -EAGAIN;
    if (f->process && owner == 1)
        process_isp(f);
    uint32_t index = 0;
    while (index < q->count && (!q->submissions[index].queued ||
          (f->process && owner == 1 && !q->submissions[index].done)))
        ++index;
    if (index == q->count)
        return -EAGAIN;
    struct submission *s = &q->submissions[index];
    s->queued = false;
    b->index = index;
    b->sequence = 9;
    uint64_t timestamp = output(type) || (f->process && owner == 1) ?
                         s->payload.timestamp_ns : UINT64_C(123456789000);
    b->timestamp.tv_sec = (long)(timestamp / UINT64_C(1000000000));
    b->timestamp.tv_usec = (long)((timestamp % UINT64_C(1000000000)) / 1000);
    b->flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
    b->field = s->payload.field;
    if (multi(type)) b->length = q->format.num_planes;
    for (uint32_t p = 0; p < q->format.num_planes; ++p) {
        uint32_t used = output(type) || (f->process && owner == 1) ?
                        (uint32_t)s->payload.planes[p].bytes_used : q->format.planes[p].size;
        if (f->corrupt) used = q->format.planes[p].size + 1;
        if (multi(type)) {
            b->m.planes[p].bytesused = used;
            b->m.planes[p].data_offset = (uint32_t)s->payload.planes[p].data_offset;
        } else b->bytesused = used;
    }
    return 0;
}

static const struct cc_io_ops ops = {
    .open = fake_open, .close = fake_close, .ioctl = fake_ioctl, .map = fake_map, .unmap = fake_unmap,
    .lock = fake_lock, .wait = fake_wait, .now = fake_now, .duplicate = fake_duplicate
};

static void init(struct fake *f)
{
    memset(f, 0, sizeof(*f));
    f->next_fd = 10;
    f->processed_object = -1;
    f->nodes[0].caps = V4L2_CAP_VIDEO_CAPTURE;
    f->nodes[1].caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE;
    f->nodes[2].caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_META_OUTPUT;
}

static cc_video_device *open_device(struct fake *f, const char *path)
{
    const struct cc_io io = { &ops, f };
    cc_video_device *device = NULL;
    OK(cc_video_open_io(path, &io, &device));
    return device;
}

static cc_stream *make_stream(cc_video_device *device, enum cc_stream_type type, uint32_t fourcc)
{
    cc_stream *stream = NULL;
    OK(cc_video_create_stream(device, type, &stream));
    cc_format format = { .fourcc = fourcc };
    if (!meta((uint32_t)type)) { format.width = 32; format.height = 16; }
    OK(cc_stream_configure(stream, &format));
    return stream;
}

static void allocate(cc_stream *stream, enum cc_memory memory)
{
    uint32_t count = 2;
    OK(cc_stream_allocate_buffers(stream, memory, &count));
    CHECK(count == SLOTS);
}

static void finish(struct fake *f)
{
    CHECK(f->mapped == 0 && f->open_fds == 0);
    for (int i = 0; i < f->next_object; ++i)
        CHECK(!f->objects[i].kernel && !f->objects[i].maps && !f->objects[i].refs && !f->objects[i].data);
}

static void test_metadata(void)
{
    for (int out = 0; out < 2; ++out) {
        struct fake f;
        init(&f);
        cc_video_device *device = open_device(&f, "/dev/meta");
        cc_stream *stream = make_stream(device, out ? CC_STREAM_META_OUTPUT : CC_STREAM_META_CAPTURE,
                                         CC_FOURCC('C', 'F', 'E', 'C'));
        allocate(stream, CC_MEMORY_MMAP);
        cc_frame frame;
        OK(cc_stream_buffer(stream, 0, &frame));
        CHECK(frame.num_planes == 1 && frame.planes[0].length == 128);
        memset(frame.planes[0].data, 0x37, frame.planes[0].length);
        frame.planes[0].bytes_used = 96;
        frame.timestamp_ns = UINT64_C(12345000);
        ERROR(cc_stream_queue(stream, 0, out ? NULL : &frame), EINVAL);
        OK(cc_stream_queue(stream, 0, out ? &frame : NULL));
        OK(cc_stream_start(stream));
        f.waiting = true;
        OK(cc_stream_dequeue(stream, &frame, 5));
        CHECK(f.last_events == (out ? POLLOUT : POLLIN));
        CHECK(frame.planes[0].bytes_used == (out ? 96U : 128U));
        CHECK(frame.timestamp_ns == (out ? UINT64_C(12345000) : UINT64_C(123456789000)));
        cc_stream_close(stream);
        cc_video_close(device);
        finish(&f);
    }
}

static void test_dmabuf_transfer(void)
{
    struct fake f;
    init(&f);
    cc_video_device *source = open_device(&f, "/dev/raw");
    cc_video_device *isp = open_device(&f, "/dev/isp");
    cc_stream *raw = make_stream(source, CC_STREAM_CAPTURE, V4L2_PIX_FMT_SRGGB10P);
    cc_stream *input = make_stream(isp, CC_STREAM_OUTPUT_MPLANE, V4L2_PIX_FMT_SRGGB10P);
    cc_stream *result = make_stream(isp, CC_STREAM_CAPTURE_MPLANE, V4L2_PIX_FMT_SRGGB10P);
    cc_stream *duplicate = (cc_stream *)(uintptr_t)1;
    ERROR(cc_video_create_stream(isp, CC_STREAM_CAPTURE, &duplicate), EBUSY);
    CHECK(!duplicate);
    allocate(raw, CC_MEMORY_MMAP);
    allocate(input, CC_MEMORY_DMABUF);
    allocate(result, CC_MEMORY_MMAP);
    ERROR(cc_stream_queue(input, 0, &(cc_frame) { .num_planes = 1 }), ENXIO);
    OK(cc_stream_queue(raw, 0, NULL));
    OK(cc_stream_start(raw));
    cc_frame frame;
    OK(cc_stream_dequeue(raw, &frame, 0));
    memset(frame.planes[0].data, 0xa5, frame.planes[0].bytes_used);
    int fd = cc_stream_export_plane(raw, 0, 0);
    CHECK(fd >= 0);
    int object = f.fds[fd].object;
    cc_dmabuf_plane plane = { fd, (uint32_t)frame.planes[0].length };
    OK(cc_stream_import(input, 0, &plane, 1));
    OK(fake_close(&f, fd)); /* Library owns an independent duplicate now. */
    OK(cc_stream_queue(input, 0, &frame));
    ERROR(cc_stream_import(input, 0, &plane, 1), EBUSY);
    OK(cc_stream_queue(result, 0, NULL));
    OK(cc_stream_start(input));
    OK(cc_stream_start(result));
    cc_video_close(isp); /* Both queues keep the same M2M open handle alive. */
    CHECK(cc_stream_fd(input) == cc_stream_fd(result));
    f.process = true;
    cc_frame completed;
    OK(cc_stream_dequeue(result, &completed, 0));
    CHECK(f.processed_object == object);
    CHECK(f.objects[object].data == frame.planes[0].data);
    CHECK(completed.timestamp_ns == frame.timestamp_ns);
    CHECK(completed.planes[0].bytes_used == frame.planes[0].bytes_used);
    CHECK(!memcmp(completed.planes[0].data, frame.planes[0].data, frame.planes[0].bytes_used));
    OK(cc_stream_dequeue(input, &completed, 0));
    CHECK(completed.planes[0].data == NULL); /* Imports stay unmapped. */
    ERROR(cc_stream_free_buffers(raw), EBUSY); /* The ISP still owns a dma fd. */
    cc_stream_close(input);
    OK(cc_stream_free_buffers(raw));
    CHECK(cc_stream_fd(result) >= 0); /* Closing input did not close the M2M fd. */
    cc_stream_close(result);
    cc_stream_close(raw);
    cc_video_close(source);
    finish(&f);
}

static void test_import_failure_and_offsets(void)
{
    struct fake f;
    init(&f);
    cc_video_device *device = open_device(&f, "/dev/isp");
    cc_stream *source = make_stream(device, CC_STREAM_CAPTURE_MPLANE, V4L2_PIX_FMT_NV12M);
    cc_stream *sink = make_stream(device, CC_STREAM_OUTPUT_MPLANE, V4L2_PIX_FMT_NV12M);
    allocate(source, CC_MEMORY_MMAP);
    allocate(sink, CC_MEMORY_DMABUF);
    cc_frame frame;
    OK(cc_stream_buffer(source, 0, &frame));
    cc_dmabuf_plane planes[2];
    for (uint32_t p = 0; p < 2; ++p) {
        planes[p].fd = cc_stream_export_plane(source, 0, p);
        CHECK(planes[p].fd >= 0);
        planes[p].length = (uint32_t)frame.planes[p].length;
        frame.planes[p].bytes_used = planes[p].length;
        frame.planes[p].data_offset = 8;
    }
    OK(cc_stream_import(sink, 0, planes, 2));
    int fds_before = f.open_fds;
    f.fail_duplicate = f.duplicates + 2;
    ERROR(cc_stream_import(sink, 0, planes, 2), EMFILE);
    CHECK(f.open_fds == fds_before);
    cc_dmabuf_plane bad = { .fd = -1, .length = 4096 };
    ERROR(cc_stream_import(sink, 1, &bad, 1), EINVAL);
    for (uint32_t p = 0; p < 2; ++p)
        OK(fake_close(&f, planes[p].fd));
    /* Failed rebind preserved the earlier import, even after caller fds close. */
    OK(cc_stream_queue(sink, 0, &frame));
    OK(cc_stream_start(sink));
    cc_frame done;
    OK(cc_stream_dequeue(sink, &done, 0));
    CHECK(done.planes[0].data_offset == 8 && done.planes[1].data_offset == 8);
    int dup = cc_stream_export_plane(sink, 0, 1);
    CHECK(dup >= 0);
    OK(fake_close(&f, dup));
    frame.planes[1].data_offset = frame.planes[1].bytes_used + 1;
    ERROR(cc_stream_queue(sink, 0, &frame), EINVAL);
    cc_stream_close(sink);
    cc_stream_close(source);
    cc_video_close(device);
    finish(&f);
}

static void test_shared_failure(void)
{
    struct fake f;
    init(&f);
    cc_video_device *device = open_device(&f, "/dev/isp");
    cc_stream *capture = make_stream(device, CC_STREAM_CAPTURE, V4L2_PIX_FMT_GREY);
    cc_stream *input = make_stream(device, CC_STREAM_OUTPUT, V4L2_PIX_FMT_GREY);
    allocate(capture, CC_MEMORY_MMAP);
    allocate(input, CC_MEMORY_MMAP);
    cc_frame frame;
    OK(cc_stream_buffer(input, 0, &frame));
    frame.planes[0].bytes_used = 42;
    frame.planes[0].data_offset = 1;
    ERROR(cc_stream_queue(input, 0, &frame), EINVAL);
    frame.planes[0].data_offset = 0;
    OK(cc_stream_queue(input, 0, &frame));
    OK(cc_stream_start(input));
    OK(cc_stream_queue(capture, 0, NULL));
    OK(cc_stream_start(capture));
    f.fail_request = VIDIOC_STREAMOFF;
    f.fail_nth = 1;
    f.fail_errno = EIO;
    cc_stream_close(input);
    ERROR(cc_stream_fd(capture), ENODEV);
    ERROR(cc_stream_dequeue(capture, &frame, 0), ENODEV);
    cc_stream *new_stream = NULL;
    ERROR(cc_video_create_stream(device, CC_STREAM_OUTPUT, &new_stream), ENODEV);
    cc_stream_close(capture);
    cc_video_close(device);
    finish(&f);
}

static void test_format_and_corruption(void)
{
    struct fake f;
    init(&f);
    cc_video_device *device = open_device(&f, "/dev/meta");
    cc_stream *stream = make_stream(device, CC_STREAM_META_CAPTURE, CC_FOURCC('S', 'T', 'A', 'T'));
    cc_format format = { .fourcc = CC_FOURCC('S', 'T', 'A', 'T'), .width = 16, .height = 4,
        .num_planes = 1, .planes = { { 32, 128 } } };
    OK(cc_stream_try_format(stream, &format));
    cc_format old;
    OK(cc_stream_get_format(stream, &old));
    CHECK(old.width == 0 && old.height == 0);
    OK(cc_stream_configure(stream, &format));
    CHECK(format.width == 16 && format.height == 4 && format.planes[0].stride == 32);
    /* Device-side changes invalidate the cached layout before allocation. */
    f.nodes[2].queues[13].format.planes[0].size = 256;
    uint32_t count = 2;
    ERROR(cc_stream_allocate_buffers(stream, CC_MEMORY_MMAP, &count), EPIPE);
    OK(cc_stream_configure(stream, &format));
    allocate(stream, CC_MEMORY_MMAP);
    OK(cc_stream_queue(stream, 0, NULL));
    OK(cc_stream_start(stream));
    f.corrupt = true;
    cc_frame frame = { .index = 987 };
    ERROR(cc_stream_dequeue(stream, &frame, 0), EPROTO);
    CHECK(frame.index == 987);
    ERROR(cc_stream_queue(stream, 1, NULL), EPIPE);
    OK(cc_stream_stop(stream));
    f.corrupt = false;
    OK(cc_stream_queue(stream, 0, NULL));
    OK(cc_stream_start(stream));
    OK(cc_stream_dequeue(stream, &frame, 0));
    cc_stream_close(stream);
    cc_video_close(device);
    finish(&f);
}

static void test_queue_types_and_memory(void)
{
    const enum cc_stream_type types[] = { CC_STREAM_CAPTURE, CC_STREAM_OUTPUT, CC_STREAM_CAPTURE_MPLANE,
        CC_STREAM_OUTPUT_MPLANE, CC_STREAM_META_CAPTURE, CC_STREAM_META_OUTPUT };
    for (size_t t = 0; t < sizeof(types) / sizeof(types[0]); ++t) {
        for (int imported = 0; imported < 2; ++imported) {
            struct fake f;
            init(&f);
            f.nodes[0].caps |= V4L2_CAP_VIDEO_CAPTURE_MPLANE;
            f.nodes[1].caps |= V4L2_CAP_META_CAPTURE | V4L2_CAP_META_OUTPUT;
            uint32_t type = (uint32_t)types[t];
            cc_video_device *device = open_device(&f, "/dev/isp");
            uint32_t fourcc = meta(type) ? CC_FOURCC('M', 'E', 'T', 'A') :
                              multi(type) ? V4L2_PIX_FMT_NV12M : V4L2_PIX_FMT_GREY;
            cc_stream *stream = make_stream(device, types[t], fourcc);
            allocate(stream, imported ? CC_MEMORY_DMABUF : CC_MEMORY_MMAP);
            cc_video_device *producer = NULL;
            cc_stream *memory = NULL;
            if (imported) {
                producer = open_device(&f, "/dev/raw");
                memory = make_stream(producer, multi(type) ? CC_STREAM_CAPTURE_MPLANE : CC_STREAM_CAPTURE,
                                       multi(type) ? V4L2_PIX_FMT_NV12M : V4L2_PIX_FMT_GREY);
                allocate(memory, CC_MEMORY_MMAP);
                cc_frame storage;
                OK(cc_stream_buffer(memory, 0, &storage));
                cc_dmabuf_plane planes[CC_MAX_PLANES];
                for (uint32_t p = 0; p < storage.num_planes; ++p) {
                    planes[p].fd = cc_stream_export_plane(memory, 0, p);
                    CHECK(planes[p].fd >= 0);
                    planes[p].length = (uint32_t)storage.planes[p].length;
                }
                OK(cc_stream_import(stream, 0, planes, storage.num_planes));
                for (uint32_t p = 0; p < storage.num_planes; ++p)
                    OK(fake_close(&f, planes[p].fd));
            }
            cc_frame frame;
            OK(cc_stream_buffer(stream, 0, &frame));
            for (uint32_t p = 0; p < frame.num_planes; ++p) {
                CHECK((frame.planes[p].data == NULL) == (imported != 0));
                frame.planes[p].bytes_used = frame.planes[p].length - 16;
                frame.planes[p].data_offset = multi(type) ? 8 : 0;
            }
            OK(cc_stream_queue(stream, 0, output(type) ? &frame : NULL));
            OK(cc_stream_start(stream));
            cc_frame done;
            OK(cc_stream_dequeue(stream, &done, 0));
            CHECK(done.num_planes == frame.num_planes);
            for (uint32_t p = 0; p < done.num_planes; ++p) {
                CHECK(done.planes[p].data == frame.planes[p].data);
                if (output(type)) {
                    CHECK(done.planes[p].bytes_used == frame.planes[p].bytes_used);
                    CHECK(done.planes[p].data_offset == frame.planes[p].data_offset);
                }
            }
            cc_stream_close(stream);
            cc_stream_close(memory);
            cc_video_close(producer);
            cc_video_close(device);
            finish(&f);
        }
    }
}

static void test_camera_imports(void)
{
    struct fake f;
    init(&f);
    f.nodes[0].caps |= V4L2_CAP_VIDEO_CAPTURE_MPLANE;
    cc_video_device *producer = open_device(&f, "/dev/isp");
    cc_stream *memory = make_stream(producer, CC_STREAM_CAPTURE_MPLANE, V4L2_PIX_FMT_NV12M);
    allocate(memory, CC_MEMORY_MMAP);
    struct cc_io io = { &ops, &f };
    cc_camera_info info = { .pipeline = CC_PIPELINE_V4L2, .video_path = "/dev/raw" };
    cc_camera *camera = NULL;
    OK(cc_camera_open_io(&info, &io, &camera));
    cc_format format = { .fourcc = V4L2_PIX_FMT_NV12M, .width = 32, .height = 16 };
    OK(cc_camera_configure(camera, &format));
    uint32_t count = 2;
    OK(cc_camera_prepare_imports(camera, &count));
    CHECK(count == SLOTS);
    cc_frame storage;
    OK(cc_stream_buffer(memory, 0, &storage));
    cc_dmabuf_plane planes[2];
    for (uint32_t p = 0; p < 2; ++p) {
        planes[p].fd = cc_stream_export_plane(memory, 0, p);
        CHECK(planes[p].fd >= 0);
        planes[p].length = (uint32_t)storage.planes[p].length;
    }
    OK(cc_camera_import_buffer(camera, 0, planes, 2));
    for (uint32_t p = 0; p < 2; ++p)
        OK(fake_close(&f, planes[p].fd));
    OK(cc_camera_queue(camera, 0));
    OK(cc_camera_start(camera));
    cc_frame frame;
    OK(cc_camera_dequeue(camera, &frame, 0));
    CHECK(frame.num_planes == 2 && !frame.planes[0].data && !frame.planes[1].data);
    CHECK(frame.planes[0].bytes_used == storage.planes[0].length);
    int fd = cc_camera_export_plane(camera, 0, 1);
    CHECK(fd >= 0);
    OK(fake_close(&f, fd));
    cc_camera_close(camera);
    cc_stream_close(memory);
    cc_video_close(producer);
    finish(&f);
}

static void test_shared_layout_control(void)
{
    struct fake f;
    init(&f);
    cc_video_device *device = open_device(&f, "/dev/isp");
    cc_stream *input = make_stream(device, CC_STREAM_OUTPUT_MPLANE, V4L2_PIX_FMT_SRGGB10P);
    cc_stream *capture = make_stream(device, CC_STREAM_CAPTURE_MPLANE, V4L2_PIX_FMT_SRGGB10P);
    cc_control control = { .id = V4L2_CID_HFLIP, .value = 1 };
    OK(cc_video_set_controls(device, &control, 1, NULL));
    uint32_t count = 2;
    ERROR(cc_stream_allocate_buffers(input, CC_MEMORY_MMAP, &count), EPIPE);
    ERROR(cc_stream_allocate_buffers(capture, CC_MEMORY_MMAP, &count), EPIPE);
    cc_format format = { .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = 32, .height = 16 };
    OK(cc_stream_configure(input, &format));
    OK(cc_stream_configure(capture, &format));
    allocate(capture, CC_MEMORY_DMABUF);
    ERROR(cc_video_set_controls(device, &control, 1, NULL), EBUSY);
    allocate(input, CC_MEMORY_MMAP); /* Rejected control left both configurations valid. */
    cc_stream_close(capture);
    cc_stream_close(input);
    cc_video_close(device);
    finish(&f);
}

int main(void)
{
    test_metadata();
    test_dmabuf_transfer();
    test_import_failure_and_offsets();
    test_shared_failure();
    test_format_and_corruption();
    test_queue_types_and_memory();
    test_camera_imports();
    test_shared_layout_control();
    puts("shared image/metadata queues, DMA-BUF ownership, output payloads and failure recovery passed");
    return 0;
}
