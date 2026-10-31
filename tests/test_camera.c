/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/capture.h>
#include "media_v2_driver.h"
#include "uvc_profiles.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/media-bus-format.h>
#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>

/* These checks stay active in Release builds; none of the driver interactions
 * are hidden in assert(), which NDEBUG could otherwise remove. */
#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x, e) CHECK((x) == -(e))

#define VIDEO_FD 20
#define MEDIA_FD 10
#define SENSOR_FD 11
#define MUX_FD 14
#define FE_FD 15
#define FE_VIDEO_BASE 30
#define CONTROL_SCALAR V4L2_CID_EXPOSURE
#define CONTROL_I64 V4L2_CID_PIXEL_RATE
#define CONTROL_PAYLOAD (V4L2_CID_USER_BASE + 0x2000U)
#define CONTROL_LAYOUT V4L2_CID_HFLIP
#define CONTROL_MASK (V4L2_CID_USER_BASE + 0x2001U)
#define CONTROL_VBLANK V4L2_CID_VBLANK

struct fake {
    bool uvc_controls, uvc_media, uvc_v2, uvc_extra_default, uvc_missing_default;
    bool uvc_clamp, uvc_bad_ack, uvc_bad_menu;
    uint32_t uvc_missing;
    int uvc_writes;
    int32_t uvc_values[7];
    bool pi, cfe, frontend, streams, legacy_routing, multiplanar, streaming, not_ready, wrong_capture_size, wrong_planes;
    bool corrupt_bytes, corrupt_offset, corrupt_index, corrupt_timestamp;
    bool mc_only, m2m, ext_colors, closed_while_streaming;
    bool fail_lock, fail_open;
    int opened, closed, live_maps, map_calls, fail_map_at;
    int wait_calls, interrupt_waits, wait_error, wait_ready_at;
    int export_count, streamoff_calls, sensor_set_calls, mux_set_calls, routing_calls;
    int import_count;
    int fail_nth, fail_errno, fail_fd;
    unsigned long fail_request;
    uint32_t allocated, force_count, sequence;
    bool queued[256];
    uint64_t now;
    cc_format active;
    struct v4l2_mbus_framefmt sensor_active, sensor_try, mux_active, mux_try;
    struct media_link_desc links[8];
    struct {
        cc_format format;
        uint32_t count;
        uint32_t memory;
        bool streaming, queued[4];
        void *data[4];
    } fe_nodes[4];
    struct v4l2_mbus_framefmt fe_format;
    struct pisp_fe_config last_parameters;
    bool fe_bad_pair, fe_bad_image1, fe_fail_stop, fe_parameter_error;
    bool clean_frames;
    bool ipa_scene, ipa_truncated;
    int64_t scene_baseline;
    int fe_delayed, fe_starts;
    uint32_t fe_sequence;
    int64_t scalar, i64, bitmask;
    int64_t vblank;
    int64_t analogue_gain, hblank;
    bool sensor_read_only;
    char sensor_model[32];
    unsigned char payload[8];
    bool event_support, event_auto, event_subscribed, event_hold, sensor_clamp;
    bool frame_during_sensor_write;
    uint32_t event_serial, event_current;
    struct v4l2_event events[256];
    size_t event_head, event_count;
    struct { uint32_t frame, id; int64_t value; } sensor_writes[256];
    size_t sensor_write_count;
};
static int64_t effective_write(const struct fake *f, uint32_t frame, uint32_t id, uint32_t delay, int64_t baseline_value);

static void frame_event(struct fake *f, uint32_t sequence)
{
    if (!f->event_subscribed) return;
    CHECK(f->event_count < 256);
    struct v4l2_event *e = &f->events[(f->event_head + f->event_count++) % 256];
    *e = (struct v4l2_event) {.type = V4L2_EVENT_FRAME_SYNC, .sequence = f->event_serial++};
    e->u.frame_sync.frame_sequence = sequence;
}

static struct fake fake_new(bool pi, bool multiplanar)
{
    struct fake f = { .pi = pi, .multiplanar = multiplanar, .force_count = 3,
        .scalar = 42, .i64 = INT64_C(4000000001), .bitmask = UINT32_C(0x80000001), .vblank = 50 };
    f.active = (cc_format) { .width = 64, .height = 32,
        .fourcc = multiplanar ? V4L2_PIX_FMT_NV12M : V4L2_PIX_FMT_SRGGB10P,
        .field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_RAW,
        .num_planes = multiplanar ? 2 : 1,
        .planes = { {80, 4096}, {64, 2048} } };
    f.sensor_active = (struct v4l2_mbus_framefmt) { .width = 64, .height = 32,
        .code = MEDIA_BUS_FMT_SRGGB10_1X10, .field = V4L2_FIELD_NONE,
        .colorspace = V4L2_COLORSPACE_RAW };
    f.sensor_try = f.mux_active = f.mux_try = f.sensor_active;
    f.links[0] = (struct media_link_desc) {
        .source = { .entity = 1, .index = 0, .flags = MEDIA_PAD_FL_SOURCE },
        .sink = { .entity = 4, .index = 0, .flags = MEDIA_PAD_FL_SINK }
    };
    f.links[1] = f.links[0];
    f.links[1].source.entity = 3;
    f.links[1].flags = MEDIA_LNK_FL_ENABLED;
    f.links[2] = (struct media_link_desc) {
        .source = { .entity = 4, .index = 1, .flags = MEDIA_PAD_FL_SOURCE },
        .sink = { .entity = 2, .index = 0, .flags = MEDIA_PAD_FL_SINK },
        .flags = MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE
    };
    memcpy(f.payload, "payload", 8);
    return f;
}

static int fake_open(void *ctx, const char *path, int flags)
{
    struct fake *f = ctx;
    if (f->fail_open)
        return -EACCES;
    CHECK(flags & O_CLOEXEC);
    ++f->opened;
    if (!strcmp(path, "/dev/video0"))
        return VIDEO_FD;
    if (!strcmp(path, "/dev/media0"))
        return MEDIA_FD;
    if (!strcmp(path, "/dev/v4l-subdev0"))
        return SENSOR_FD;
    if (!strcmp(path, "/dev/v4l-subdev2"))
        return MUX_FD;
    if (f->frontend && !strcmp(path, "/dev/v4l-subdev3"))
        return FE_FD;
    if (f->frontend)
        for (int i = 0; i < 4; ++i) {
            char name[32];
            snprintf(name, sizeof(name), "/dev/video%d", i + 1);
            if (!strcmp(path, name)) return FE_VIDEO_BASE + i;
        }
    CHECK(false);
    return -ENOENT;
}

static int fake_close(void *ctx, int fd)
{
    struct fake *f = ctx;
    if (fd == (f->frontend ? FE_VIDEO_BASE : VIDEO_FD)) {
        f->event_subscribed = false; f->event_count = f->event_head = 0;
    }
    if (fd == VIDEO_FD) {
        f->closed_while_streaming = f->streaming;
        f->streaming = false;
        f->allocated = 0;
    }
    if (fd >= FE_VIDEO_BASE && fd < FE_VIDEO_BASE + 4)
        f->fe_nodes[fd - FE_VIDEO_BASE].streaming = false;
    if (fd == 1000) --f->import_count;
    else if (fd == 99)
        --f->export_count;
    else
        ++f->closed;
    return 0;
}

static int fake_duplicate(void *ctx, int fd)
{
    struct fake *f = ctx;
    CHECK(fd == 101 || fd == 102);
    ++f->import_count;
    return 1000;
}

static int fake_map(void *ctx, int fd, size_t size, off_t offset, void **out)
{
    struct fake *f = ctx;
    CHECK((fd == VIDEO_FD || (f->frontend && fd >= FE_VIDEO_BASE && fd < FE_VIDEO_BASE + 4)) && size > 0 && offset >= 0 && offset % 4096 == 0);
    if (++f->map_calls == f->fail_map_at)
        return -ENOMEM;
    *out = malloc(size);
    CHECK(*out);
    memset(*out, 0x5a, size);
    if (fd >= FE_VIDEO_BASE) {
        CHECK(offset / 65536 < 4);
        f->fe_nodes[fd - FE_VIDEO_BASE].data[offset / 65536] = *out;
    }
    ++f->live_maps;
    return 0;
}

static int fake_unmap(void *ctx, void *p, size_t size)
{
    struct fake *f = ctx;
    CHECK(!f->streaming && size > 0);
    if (f->frontend)
        for (int i = 0; i < 4; ++i) {
            CHECK(!f->fe_nodes[i].streaming);
            for (int j = 0; j < 4; ++j)
                if (f->fe_nodes[i].data[j] == p) f->fe_nodes[i].data[j] = NULL;
        }
    free(p);
    --f->live_maps;
    CHECK(f->live_maps >= 0);
    return 0;
}

static int fake_lock(void *ctx, int fd)
{
    (void)fd;
    return ((struct fake *)ctx)->fail_lock ? -EWOULDBLOCK : 0;
}

static int fake_now(void *ctx, uint64_t *ns)
{
    *ns = ((struct fake *)ctx)->now;
    return 0;
}

static int fake_wait(void *ctx, int fd, short events, int timeout)
{
    struct fake *f = ctx;
    CHECK((fd == VIDEO_FD || (f->frontend && fd >= FE_VIDEO_BASE && fd < FE_VIDEO_BASE + 4)) &&
          (events == POLLIN || events == POLLPRI || events == (POLLIN | POLLPRI) ||
           (f->frontend && events == POLLOUT)) && (timeout == -1 || timeout > 0));
    ++f->wait_calls;
    f->now += UINT64_C(1000000);
    if (f->interrupt_waits > 0) {
        --f->interrupt_waits;
        return -EINTR;
    }
    if (f->wait_error)
        return -f->wait_error;
    if (f->wait_ready_at > 0 && f->wait_calls >= f->wait_ready_at) {
        f->not_ready = false;
        return 0;
    }
    return -EAGAIN; /* A spurious wakeup must still respect the deadline. */
}

static int fake_wait_many(void *ctx, struct pollfd *fds, size_t count, int timeout)
{
    struct fake *f = ctx;
    CHECK(count > 0 && count <= 4);
    for (size_t i = 0; i < count; ++i) {
        CHECK(fds[i].fd == VIDEO_FD || (fds[i].fd >= FE_VIDEO_BASE && fds[i].fd < FE_VIDEO_BASE + 4));
        CHECK(fds[i].events == (fds[i].fd == FE_VIDEO_BASE + 2 ? POLLOUT : POLLIN) ||
              (f->event_subscribed && (fds[i].events == POLLPRI || fds[i].events == (POLLIN | POLLPRI))));
    }
    int ret = fake_wait(ctx, fds[0].fd, fds[0].events, timeout);
    if (!ret && f->frontend) f->fe_delayed = -1;
    return ret;
}

static int fake_resolve(void *ctx, unsigned int major_id, unsigned int minor_id,
                         char *path, size_t size)
{
    struct fake *f = ctx;
    CHECK(major_id == 81);
    if (f->frontend && minor_id >= 5)
        snprintf(path, size, "/dev/video%u", minor_id - 4);
    else if (minor_id == 0)
        snprintf(path, size, "/dev/video0");
    else
        snprintf(path, size, "/dev/v4l-subdev%u", minor_id - 1);
    return 0;
}

static struct media_entity_desc entity_desc(const struct fake *f, uint32_t id)
{
    struct media_entity_desc entity = { .id = id, .pads = id == 4 ? 2 : 1 };
    entity.dev.major = 81;
    if (id == 2) {
        entity.type = MEDIA_ENT_F_IO_V4L;
        strcpy(entity.name, f->cfe ? "rp1-cfe-csi2-ch0" : "unicam-image");
        entity.links = 1;
        if (f->uvc_media && !f->uvc_missing_default) entity.flags=MEDIA_ENT_FL_DEFAULT;
    } else if (id == 4) {
        entity.type = f->cfe ? MEDIA_ENT_F_VID_IF_BRIDGE : MEDIA_ENT_F_VID_MUX;
        strcpy(entity.name, f->cfe ? "csi2" : "camera-mux");
        entity.links = f->cfe ? 4 : 3;
        if (f->cfe)
            entity.pads = f->streams ? 5 : 8;
        entity.dev.minor = 3;
    } else if (id == 5) {
        entity.type = MEDIA_ENT_F_PROC_VIDEO_ISP;
        strcpy(entity.name, "pisp-fe");
        entity.links = f->frontend ? 5 : 1;
        entity.pads = f->frontend ? 5 : 1;
        entity.dev.major = f->frontend ? 81 : 0;
        entity.dev.minor = 4;
    } else if (f->frontend && id >= 6) {
        entity.type = MEDIA_ENT_F_IO_V4L;
        entity.links = 1;
        entity.dev.minor = id - 1;
        const char *names[] = { "rp1-cfe-fe-image0", "rp1-cfe-fe-stats", "rp1-cfe-fe-config", "rp1-cfe-fe-image1" };
        strcpy(entity.name, names[id - 6]);
    } else {
        entity.type = MEDIA_ENT_F_CAM_SENSOR;
        strcpy(entity.name, id == 1 ? (f->sensor_model[0] ? f->sensor_model : "imx219 10-0010") : "imx219 11-0010");
        entity.links = 1;
        entity.dev.minor = id == 1 ? 1 : 2;
    }
    if (id==3 && f->uvc_extra_default) { entity.flags=MEDIA_ENT_FL_DEFAULT; entity.type=MEDIA_ENT_F_IO_V4L; }
    return entity;
}

static int fake_media_ioctl(struct fake *f, unsigned long req, void *arg)
{
    if (req == MEDIA_IOC_DEVICE_INFO) {
        struct media_device_info *info = arg;
        strcpy(info->driver, f->uvc_media ? "uvcvideo":f->cfe ? "rp1-cfe" : "unicam");
        strcpy(info->model, "unicam");
        if (f->uvc_media) { strcpy(info->model,"USB camera"); info->media_version=f->uvc_v2 ? 0x060c00U:0x041200U; }
        return 0;
    }
    if (req == MEDIA_IOC_ENUM_ENTITIES) {
        struct media_entity_desc *entity = arg;
        CHECK(entity->id & MEDIA_ENT_ID_FLAG_NEXT);
        uint32_t id = entity->id & ~MEDIA_ENT_ID_FLAG_NEXT;
        if (id >= (f->frontend ? 9U : f->cfe ? 5U : 4U))
            return -EINVAL;
        *entity = entity_desc(f, id + 1);
        return 0;
    }
    if (req == MEDIA_IOC_ENUM_LINKS) {
        struct media_links_enum *enumeration = arg;
        uint32_t id = enumeration->entity;
        struct media_entity_desc desc = entity_desc(f, id);
        for (uint16_t i = 0; i < desc.pads; ++i)
            enumeration->pads[i] = (struct media_pad_desc) {
                .entity = id, .index = i,
                .flags = id == 2 || (id == 5 && (!f->frontend || i < 2)) || (id >= 6 && id != 8) ||
                         (id == 4 && i < (f->cfe && !f->streams ? 4U : 1U)) ?
                         MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE
            };
        size_t n = 0;
        for (size_t i = 0; i < (f->frontend ? 8U : f->cfe ? 4U : 3U); ++i)
            if (f->links[i].source.entity == id || f->links[i].sink.entity == id)
                enumeration->links[n++] = f->links[i];
        CHECK(n == desc.links);
        return 0;
    }
    if (req == MEDIA_IOC_SETUP_LINK) {
        struct media_link_desc *link = arg;
        for (size_t i = 0; i < (f->frontend ? 8U : f->cfe ? 4U : 3U); ++i) {
            if (link->source.entity == f->links[i].source.entity &&
                link->sink.entity == f->links[i].sink.entity) {
                CHECK(!(f->links[i].flags & MEDIA_LNK_FL_IMMUTABLE));
                for (size_t j = 0; j < (f->frontend ? 8U : f->cfe ? 4U : 3U); ++j) {
                    if (j == i || !(link->flags & MEDIA_LNK_FL_ENABLED) ||
                        !(f->links[j].flags & MEDIA_LNK_FL_ENABLED))
                        continue;
                    if ((link->sink.entity == f->links[j].sink.entity &&
                         link->sink.index == f->links[j].sink.index) ||
                        (f->cfe && link->source.entity == 4 &&
                         link->source.entity == f->links[j].source.entity &&
                         link->source.index == f->links[j].source.index))
                        return -EBUSY;
                }
                f->links[i] = *link;
                return 0;
            }
        }
        CHECK(false);
    }
    CHECK(false);
    return -ENOTTY;
}

static int fake_uvc_control(struct fake *f,unsigned long req,void *arg)
{
    cc_uvc_profile p=test_uvc_profile();
    if (req==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=arg;
        if (q->id==CONTROL_MASK) {
            q->type=V4L2_CTRL_TYPE_INTEGER_MENU; q->minimum=0; q->maximum=3; q->step=1; q->elems=1; return 0;
        }
        size_t i=0; while (i<7 && q->id!=test_uvc_ids[i]) ++i;
        if (i==7 || (f->uvc_missing&(1U<<i))) return -EINVAL;
        const cc_control_info *r=&p.native[i];
        q->type=r->type; q->minimum=r->minimum; q->maximum=r->maximum;
        q->default_value=r->default_value; q->step=r->step; q->elems=1; q->elem_size=4;
        if (i==4 && f->uvc_values[3]==3) q->flags=V4L2_CTRL_FLAG_INACTIVE;
        return 0;
    }
    if (req==VIDIOC_QUERYMENU) {
        struct v4l2_querymenu *q=arg;
        if (q->id==CONTROL_MASK) { if (q->index!=2) return -EINVAL; q->value=INT64_MIN; }
        else {
            CHECK(q->id==V4L2_CID_EXPOSURE_AUTO);
            if (q->index>3 || !(p.exposure_menu&(1U<<q->index))) return -EINVAL;
            memset(q->name,'X',sizeof(q->name));
        }
        if (f->uvc_bad_menu) ++q->index;
        return 0;
    }
    CHECK(req==VIDIOC_G_EXT_CTRLS || req==VIDIOC_S_EXT_CTRLS);
    struct v4l2_ext_controls *batch=arg; CHECK(batch->count<=7);
    for (uint32_t n=0;n<batch->count;++n) {
        struct v4l2_ext_control *v=&batch->controls[n]; size_t i=0;
        while (i<7 && v->id!=test_uvc_ids[i]) ++i;
        CHECK(i<7);
        if (req==VIDIOC_S_EXT_CTRLS) {
            CHECK(v->value>=p.native[i].minimum && v->value<=p.native[i].maximum);
            CHECK((uint64_t)(v->value-p.native[i].minimum)%p.native[i].step==0);
            if (i==3) CHECK(p.exposure_menu&(1U<<(uint32_t)v->value));
            if (i==4 && f->uvc_values[3]==3) return -EACCES;
            f->uvc_values[i]=i==0 && f->uvc_clamp && v->value>30 ? 30:v->value;
        }
        v->value=f->uvc_bad_ack ? INT32_MAX:f->uvc_values[i];
    }
    if (req==VIDIOC_S_EXT_CTRLS) ++f->uvc_writes;
    return 0;
}
static int fake_control_ioctl(struct fake *f, int fd, unsigned long req, void *arg)
{
    if (f->uvc_controls) { CHECK(fd==VIDEO_FD); return fake_uvc_control(f,req,arg); }
    CHECK(fd == (f->pi ? SENSOR_FD : VIDEO_FD));
    if (req == VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *query = arg;
        if (query->id & V4L2_CTRL_FLAG_NEXT_CTRL) {
            uint32_t id = query->id & ~(V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND);
            if (id >= CONTROL_SCALAR)
                return -EINVAL;
            query->id = CONTROL_SCALAR;
        }
        query->minimum = 0;
        query->maximum = 100;
        query->default_value = 42;
        query->step = 1;
        query->elem_size = 4;
        query->elems = 1;
        query->type = V4L2_CTRL_TYPE_INTEGER;
        strcpy(query->name, "test control");
        switch (query->id) {
        case CONTROL_SCALAR: case CONTROL_VBLANK: case V4L2_CID_ANALOGUE_GAIN:
            if (f->sensor_read_only) query->flags = V4L2_CTRL_FLAG_READ_ONLY;
            if (query->id == V4L2_CID_ANALOGUE_GAIN) query->maximum = 240;
            return 0;
        case V4L2_CID_HBLANK:
            query->maximum = 1000;
            query->flags = V4L2_CTRL_FLAG_READ_ONLY;
            return 0;
        case CONTROL_I64:
            query->type = V4L2_CTRL_TYPE_INTEGER64;
            query->maximum = INT64_MAX;
            query->elem_size = 8;
            return 0;
        case CONTROL_PAYLOAD:
            query->type = V4L2_CTRL_TYPE_U8;
            query->flags = V4L2_CTRL_FLAG_HAS_PAYLOAD;
            query->elem_size = 1;
            query->elems = 8;
            query->nr_of_dims = 1;
            query->dims[0] = 8;
            return 0;
        case CONTROL_LAYOUT:
            query->flags = V4L2_CTRL_FLAG_MODIFY_LAYOUT;
            return 0;
        case CONTROL_MASK:
            query->type = V4L2_CTRL_TYPE_BITMASK;
            return 0;
        default: return -EINVAL;
        }
    }
    CHECK(req == VIDIOC_S_EXT_CTRLS || req == VIDIOC_G_EXT_CTRLS);
    struct v4l2_ext_controls *batch = arg;
    CHECK(batch->which == V4L2_CTRL_WHICH_CUR_VAL);
    bool get = req == VIDIOC_G_EXT_CTRLS;
    for (uint32_t i = 0; i < batch->count; ++i) {
        struct v4l2_ext_control *control = &batch->controls[i];
        switch (control->id) {
        case V4L2_CID_ANALOGUE_GAIN:
            if (get) control->value = (int32_t)f->analogue_gain;
            else f->analogue_gain = control->value;
            break;
        case V4L2_CID_HBLANK:
            CHECK(get);
            control->value = (int32_t)f->hblank;
            break;
        case CONTROL_SCALAR:
            if (get) control->value = (int32_t)f->scalar;
            else {
                if (f->sensor_clamp && control->value > f->vblank) return -ERANGE;
                if (f->sensor_clamp && control->value > 99) control->value = 99;
                f->scalar = control->value;
            }
            break;
        case CONTROL_VBLANK:
            if (get) control->value = (int32_t)f->vblank;
            else f->vblank = control->value;
            break;
        case CONTROL_I64:
            if (get) control->value64 = f->i64;
            else {
                if (f->sensor_clamp && control->value64 > INT64_C(5000000000)) control->value64 = INT64_C(5000000000);
                f->i64 = control->value64;
            }
            break;
        case CONTROL_PAYLOAD:
            if (control->size < 8) {
                control->size = 8;
                batch->error_idx = i;
                return -ENOSPC;
            }
            if (get) memcpy(control->ptr, f->payload, 8);
            else memcpy(f->payload, control->ptr, 8);
            control->size = 8;
            break;
        case CONTROL_LAYOUT: break;
        case CONTROL_MASK:
            if (get) control->value = (int32_t)f->bitmask;
            else f->bitmask = (uint32_t)control->value;
            break;
        default: CHECK(false);
        }
        if (!get && f->event_subscribed) {
            CHECK(f->sensor_write_count < 256);
            size_t n = f->sensor_write_count++;
            f->sensor_writes[n].frame = f->event_current;
            f->sensor_writes[n].id = control->id;
            f->sensor_writes[n].value = control->id == CONTROL_I64 ? control->value64 : control->value;
            if (f->frame_during_sensor_write) {
                f->frame_during_sensor_write = false;
                frame_event(f, f->event_current + 1U);
            }
        }
    }
    return 0;
}

static int fake_sensor_ioctl(struct fake *f, int fd, unsigned long req, void *arg)
{
    CHECK(fd == SENSOR_FD || fd == MUX_FD || (f->frontend && fd == FE_FD));
    if (req==CC_SD_G_FRAME_INTERVAL || req==CC_SD_S_FRAME_INTERVAL) {
        CHECK(fd==SENSOR_FD); struct cc_subdev_frame_interval *q=arg;
        CHECK(!q->pad && !q->stream && !q->which);
        q->interval=(struct v4l2_fract){1001,req==CC_SD_G_FRAME_INTERVAL ? 30000U:24000U}; return 0;
    }
    if (req==CC_SD_ENUM_FRAME_INTERVAL) {
        CHECK(fd==SENSOR_FD); struct cc_subdev_frame_interval_enum *q=arg;
        CHECK(!q->pad && !q->stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE && q->code && q->width && q->height);
        if (q->index) return -EINVAL;
        q->interval=(struct v4l2_fract){1,30}; return 0;
    }
    if (fd == FE_FD) {
        CHECK(req == VIDIOC_SUBDEV_S_FMT || req == VIDIOC_SUBDEV_G_FMT);
        struct v4l2_subdev_format *format = arg;
        CHECK(format->pad == 0 || format->pad == 2 || format->pad == 3);
        if (req == VIDIOC_SUBDEV_S_FMT) {
            CHECK(format->format.code == MEDIA_BUS_FMT_SRGGB16_1X16);
            f->fe_format = format->format;
        }
        format->format = f->fe_format;
        return 0;
    }
    if (req == CC_SD_QUERYCAP) {
        struct cc_subdev_capability *caps = arg;
        caps->capabilities = f->streams && fd == MUX_FD ? CC_SD_CAP_STREAMS : 0;
        return 0;
    }
    if (req == CC_SD_S_CLIENT_CAP) {
        struct cc_subdev_client_capability *client = arg;
        CHECK(f->streams && fd == MUX_FD && client->capabilities == 1);
        return 0;
    }
    if (req == CC_SD_S_ROUTING || req == CC_SD_S_ROUTING_LEGACY) {
        CHECK(f->cfe && f->streams && fd == MUX_FD);
        if (req == CC_SD_S_ROUTING && f->legacy_routing)
            return -ENOTTY;
        uint64_t ptr;
        uint32_t which;
        if (req == CC_SD_S_ROUTING) {
            struct cc_subdev_routing *routing = arg;
            CHECK(routing->num_routes == 1 && routing->len_routes == 1);
            ptr = routing->routes;
            which = routing->which;
        } else {
            CHECK(f->legacy_routing);
            struct cc_subdev_routing_legacy *routing = arg;
            CHECK(routing->num_routes == 1);
            ptr = routing->routes;
            which = routing->which;
        }
        const struct cc_subdev_route *route = (const void *)(uintptr_t)ptr;
        CHECK(route->sink_pad == 0 && route->source_pad == 1 &&
              route->sink_stream == 0 && route->source_stream == 0 && route->flags == CC_SD_ROUTE_ACTIVE);
        CHECK(which == V4L2_SUBDEV_FORMAT_TRY || which == V4L2_SUBDEV_FORMAT_ACTIVE);
        ++f->routing_calls;
        return 0;
    }
    if (req == VIDIOC_SUBDEV_ENUM_MBUS_CODE) {
        struct v4l2_subdev_mbus_code_enum *code = arg;
        CHECK(code->pad == 0);
        if (code->index)
            return -EINVAL;
        code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
        return 0;
    }
    if (req == VIDIOC_SUBDEV_ENUM_FRAME_SIZE) {
        struct v4l2_subdev_frame_size_enum *size = arg;
        CHECK(size->code == MEDIA_BUS_FMT_SRGGB10_1X10 && size->pad == 0);
        if (size->index)
            return -EINVAL;
        size->min_width = size->max_width = 64;
        size->min_height = size->max_height = 32;
        return 0;
    }
    CHECK(req == VIDIOC_SUBDEV_S_FMT || req == VIDIOC_SUBDEV_G_FMT);
    struct v4l2_subdev_format *format = arg;
    struct v4l2_mbus_framefmt *current;
    if (fd == SENSOR_FD) {
        CHECK(format->pad == 0);
        current = format->which == V4L2_SUBDEV_FORMAT_TRY ? &f->sensor_try : &f->sensor_active;
    } else {
        current = format->which == V4L2_SUBDEV_FORMAT_TRY ? &f->mux_try : &f->mux_active;
    }
    if (f->frontend && fd == MUX_FD && format->pad == (f->streams ? 1U : 4U) && req == VIDIOC_SUBDEV_S_FMT) {
        CHECK(format->format.code == MEDIA_BUS_FMT_SRGGB16_1X16);
        f->mux_active = format->format;
        return 0;
    }
    if (req == VIDIOC_SUBDEV_S_FMT) {
        if (format->format.code != MEDIA_BUS_FMT_SRGGB10_1X10)
            return -EINVAL;
        format->format.width = 64;
        format->format.height = f->frontend ? 64 : 32;
        if (fd == MUX_FD) {
            CHECK(format->pad == 0);
            ++f->mux_set_calls;
        } else {
            ++f->sensor_set_calls;
        }
        *current = format->format;
    } else if (fd == MUX_FD) {
        CHECK(format->pad == (f->cfe && !f->streams ? 4U : 1U));
    }
    format->format = *current;
    return 0;
}

static int fake_frontend_ioctl(struct fake *f, int fd, unsigned long req, void *arg)
{
    int port = fd - FE_VIDEO_BASE;
    CHECK(port >= 0 && port < 4);
    if (req == VIDIOC_QUERYCAP) {
        struct v4l2_capability *cap = arg;
        cap->capabilities = V4L2_CAP_DEVICE_CAPS;
        cap->device_caps = V4L2_CAP_STREAMING | (port == 0 || port == 3 ? V4L2_CAP_VIDEO_CAPTURE :
            port == 1 ? V4L2_CAP_META_CAPTURE : V4L2_CAP_META_OUTPUT);
        return 0;
    }
    if (req == VIDIOC_S_FMT || req == VIDIOC_G_FMT) {
        struct v4l2_format *v = arg;
        if (req == VIDIOC_S_FMT) {
            if (!port || port == 3) {
                f->fe_nodes[port].format = (cc_format) { .width = v->fmt.pix.width,
                    .height = v->fmt.pix.height, .fourcc = v->fmt.pix.pixelformat,
                    .num_planes = 1, .field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_RAW,
                    .planes = { {v->fmt.pix.bytesperline, v->fmt.pix.bytesperline * v->fmt.pix.height} } };
            } else {
                f->fe_nodes[port].format = (cc_format) { .fourcc = v->fmt.meta.dataformat,
                    .num_planes = 1, .planes = { { 0, port == 1 ? sizeof(struct pisp_statistics) : sizeof(struct pisp_fe_config) } } };
            }
        }
        cc_format *format = &f->fe_nodes[port].format;
        if (!port || port == 3) {
            v->fmt.pix.width = format->width;
            v->fmt.pix.height = format->height;
            v->fmt.pix.pixelformat = format->fourcc;
            v->fmt.pix.field = format->field;
            v->fmt.pix.colorspace = format->colorspace;
            v->fmt.pix.bytesperline = format->planes[0].stride;
            v->fmt.pix.sizeimage = format->planes[0].size;
        } else {
            v->fmt.meta.dataformat = format->fourcc;
            v->fmt.meta.buffersize = format->planes[0].size;
        }
        return 0;
    }
    if (req == VIDIOC_REQBUFS) {
        struct v4l2_requestbuffers *r = arg;
        CHECK(r->memory == V4L2_MEMORY_MMAP || ((port == 0 || port == 3) && r->memory == V4L2_MEMORY_DMABUF));
        f->fe_nodes[port].memory = r->memory;
        if (r->count) r->count = f->fe_nodes[port].count = port == 2 ? 2 : 3;
        else {
            CHECK(!f->fe_nodes[port].streaming);
            for (size_t i = 0; i < 4; ++i) CHECK(!f->fe_nodes[port].data[i]);
            if (!port && f->export_count) return -EBUSY;
            f->fe_nodes[port].count = 0;
        }
        return 0;
    }
    if (req == VIDIOC_STREAMON) {
        f->fe_nodes[port].streaming = true; ++f->fe_starts;
        if (f->event_auto && port == 2) frame_event(f, f->fe_sequence - 1U);
        return 0;
    }
    if (req == VIDIOC_STREAMOFF) {
        if (!port && f->fe_fail_stop) return -EIO;
        f->fe_nodes[port].streaming = false;
        if (f->event_auto && !port) f->fe_sequence = 0;
        memset(f->fe_nodes[port].queued, 0, sizeof(f->fe_nodes[port].queued));
        return 0;
    }
    if (req == VIDIOC_EXPBUF) {
        struct v4l2_exportbuffer *e = arg;
        CHECK(e->index < f->fe_nodes[port].count && !e->plane && e->flags == O_CLOEXEC);
        e->fd = 99;
        ++f->export_count;
        return 0;
    }
    struct v4l2_buffer *b = arg;
    if (req == VIDIOC_QUERYBUF) {
        CHECK(b->index < f->fe_nodes[port].count);
        b->length = f->fe_nodes[port].format.planes[0].size;
        b->m.offset = b->index * 65536;
        return 0;
    }
    if (req == VIDIOC_QBUF) {
        CHECK(b->index < f->fe_nodes[port].count && !f->fe_nodes[port].queued[b->index]);
        if (f->fe_nodes[port].memory == V4L2_MEMORY_DMABUF) CHECK(b->m.fd == 1000);
        if (port == 2) {
            CHECK(b->bytesused == sizeof(struct pisp_fe_config));
            memcpy(&f->last_parameters, f->fe_nodes[port].data[b->index], sizeof(f->last_parameters));
            CHECK(f->last_parameters.global.enables & PISP_FE_ENABLE_INPUT);
            CHECK(f->last_parameters.global.enables & PISP_FE_ENABLE_OUTPUT0);
            CHECK(f->last_parameters.input.streaming == 1 && f->last_parameters.input.format.width == 64);
            CHECK(f->last_parameters.ch[0].output.format.stride == 128);
            CHECK(f->last_parameters.output_buffer[0].addr_lo == 0);
            ++f->fe_sequence;
            if (f->event_auto && f->fe_nodes[port].streaming) frame_event(f, f->fe_sequence - 1U);
        } else CHECK(!b->bytesused);
        f->fe_nodes[port].queued[b->index] = true;
        return 0;
    }
    CHECK(req == VIDIOC_DQBUF && f->fe_nodes[port].streaming);
    if (f->fe_delayed == port) return -EAGAIN;
    for (uint32_t i = 0; i < f->fe_nodes[port].count; ++i) {
        if (!f->fe_nodes[port].queued[i]) continue;
        f->fe_nodes[port].queued[i] = false;
        b->index = i;
        b->sequence = f->fe_sequence - (f->event_auto ? 1U : 0U) + (f->fe_bad_pair && port == 1 ? 1U : 0U);
        if (f->fe_bad_image1 && port == 3) ++b->sequence;
        if (port == 1 && f->ipa_scene) {
            struct pisp_statistics *stats = f->fe_nodes[port].data[i];
            memset(stats,0,sizeof(*stats));
            int64_t exposure = effective_write(f,b->sequence,CONTROL_SCALAR,2,f->scene_baseline);
            int64_t gain = effective_write(f,b->sequence,V4L2_CID_ANALOGUE_GAIN,1,0);
            CHECK(exposure >= 0 && gain >= 0 && gain < 256);
            uint64_t bin = (uint64_t)exposure * 1024 * 256 / (500U * (256U-(uint64_t)gain));
            if (bin > 1023) bin=1023;
            stats->agc.histogram[bin]=10000;
        }
        b->timestamp.tv_sec = 123;
        b->timestamp.tv_usec = 456789;
        b->bytesused = f->fe_nodes[port].format.planes[0].size;
        if (port==1 && f->ipa_truncated) b->bytesused=1;
        b->flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        if (port == 2 && f->fe_parameter_error) b->flags |= V4L2_BUF_FLAG_ERROR;
        return 0;
    }
    return -EAGAIN;
}

static int fake_legacy_media(void *context,int fd,unsigned long req,void *arg)
{
    CHECK(fd==MEDIA_FD); return fake_media_ioctl(context,req,arg);
}
static int fake_ioctl(void *ctx, int fd, unsigned long req, void *arg)
{
    struct fake *f = ctx;
    if (f->fail_request == req && (!f->fail_fd || f->fail_fd == fd) && f->fail_nth > 0 && --f->fail_nth == 0)
        return -f->fail_errno;
    if (req == VIDIOC_SUBSCRIBE_EVENT || req == VIDIOC_UNSUBSCRIBE_EVENT || req == VIDIOC_DQEVENT) {
        CHECK(fd == (f->frontend ? FE_VIDEO_BASE : VIDEO_FD));
        if (!f->event_support) return -ENOTTY;
        if (req == VIDIOC_DQEVENT) {
            if (!f->event_count || f->event_hold) return -EAGAIN;
            struct v4l2_event *event = arg;
            *event = f->events[f->event_head++ % 256]; --f->event_count;
            event->pending = (uint32_t)f->event_count;
            f->event_current = event->u.frame_sync.frame_sequence;
        } else {
            struct v4l2_event_subscription *sub = arg;
            CHECK(sub->type == V4L2_EVENT_FRAME_SYNC && !sub->flags && !sub->id);
            f->event_subscribed = req == VIDIOC_SUBSCRIBE_EVENT;
            if (!f->event_subscribed) f->event_count = f->event_head = 0;
        }
        return 0;
    }
    if (fd==MEDIA_FD && req==MEDIA_IOC_G_TOPOLOGY)
        return f->uvc_v2 ? test_v2_topology(f,fd,req,arg,fake_legacy_media,false):-ENOTTY;
    if (fd == MEDIA_FD)
        return fake_media_ioctl(f, req, arg);
    if (req == VIDIOC_QUERY_EXT_CTRL || req == VIDIOC_S_EXT_CTRLS || req == VIDIOC_G_EXT_CTRLS || req==VIDIOC_QUERYMENU)
        return fake_control_ioctl(f, fd, req, arg);
    if (f->frontend && fd >= FE_VIDEO_BASE && fd < FE_VIDEO_BASE + 4)
        return fake_frontend_ioctl(f, fd, req, arg);
    if (fd != VIDEO_FD)
        return fake_sensor_ioctl(f, fd, req, arg);
    if (req == VIDIOC_QUERYCAP) {
        struct v4l2_capability *caps = arg;
        strcpy((char *)caps->driver, f->uvc_media ? "uvcvideo":f->cfe ? "rp1-cfe" : f->pi ? "unicam" : "uvcvideo");
        strcpy((char *)caps->card, "Fake camera");
        caps->capabilities = V4L2_CAP_DEVICE_CAPS;
        caps->device_caps = V4L2_CAP_STREAMING | (f->multiplanar ?
            V4L2_CAP_VIDEO_CAPTURE_MPLANE : V4L2_CAP_VIDEO_CAPTURE);
        if (f->pi || f->mc_only) caps->device_caps |= V4L2_CAP_IO_MC;
        if (f->m2m) caps->device_caps |= V4L2_CAP_VIDEO_M2M;
        if (f->ext_colors) caps->device_caps |= V4L2_CAP_EXT_PIX_FORMAT;
        return 0;
    }
    if (req == VIDIOC_ENUM_FMT) {
        struct v4l2_fmtdesc *format = arg;
        if (format->index)
            return -EINVAL;
        format->pixelformat = f->active.fourcc;
        strcpy((char *)format->description, "Fake format");
        return 0;
    }
    if (req == VIDIOC_ENUM_FRAMESIZES) {
        struct v4l2_frmsizeenum *size = arg;
        if (size->index)
            return -EINVAL;
        size->type = V4L2_FRMSIZE_TYPE_DISCRETE;
        size->discrete.width = 64;
        size->discrete.height = 32;
        return 0;
    }
    if (req == VIDIOC_S_FMT || req == VIDIOC_G_FMT || req == VIDIOC_TRY_FMT) {
        struct v4l2_format *format = arg;
        CHECK(format->type == (f->multiplanar ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE));
        if (f->multiplanar) {
            struct v4l2_pix_format_mplane *p = &format->fmt.pix_mp;
            p->width = f->active.width;
            p->height = f->active.height;
            p->pixelformat = f->active.fourcc;
            p->field = f->active.field;
            p->colorspace = f->active.colorspace;
            p->num_planes = 2;
            for (unsigned int i = 0; i < 2; ++i) {
                p->plane_fmt[i].bytesperline = f->active.planes[i].stride;
                p->plane_fmt[i].sizeimage = f->active.planes[i].size;
            }
        } else {
            struct v4l2_pix_format *p = &format->fmt.pix;
            if (f->ext_colors) {
                if (req != VIDIOC_G_FMT)
                    CHECK(p->priv == V4L2_PIX_FMT_PRIV_MAGIC);
                p->priv = V4L2_PIX_FMT_PRIV_MAGIC;
            }
            p->width = f->active.width + (f->wrong_capture_size ? 1U : 0U);
            p->height = f->active.height;
            p->pixelformat = f->active.fourcc;
            p->field = f->active.field;
            p->colorspace = f->active.colorspace;
            p->bytesperline = f->active.planes[0].stride;
            p->sizeimage = f->active.planes[0].size;
            p->ycbcr_enc = f->active.ycbcr_enc;
            p->quantization = f->active.quantization;
            p->xfer_func = f->active.xfer_func;
        }
        return 0;
    }
    if (req == VIDIOC_REQBUFS) {
        struct v4l2_requestbuffers *buffers = arg;
        CHECK(buffers->memory == V4L2_MEMORY_MMAP);
        if (!buffers->count) {
            CHECK(!f->live_maps);
            if (f->export_count)
                return -EBUSY;
            f->allocated = 0;
        } else {
            buffers->count = f->force_count;
            f->allocated = buffers->count;
        }
        return 0;
    }
    if (req == VIDIOC_QUERYBUF || req == VIDIOC_QBUF) {
        struct v4l2_buffer *buffer = arg;
        CHECK(buffer->index < f->allocated && buffer->memory == V4L2_MEMORY_MMAP);
        if (req == VIDIOC_QBUF) {
            CHECK(!f->queued[buffer->index]);
            if (f->multiplanar)
                CHECK(buffer->length == 2 && buffer->m.planes[0].bytesused == 0);
            f->queued[buffer->index] = true;
            if (f->event_auto && f->streaming) frame_event(f, f->sequence);
        } else if (f->multiplanar) {
            CHECK(buffer->length == CC_MAX_PLANES);
            buffer->length = f->wrong_planes ? 1 : 2;
            for (uint32_t i = 0; i < buffer->length; ++i) {
                buffer->m.planes[i].length = f->active.planes[i].size;
                buffer->m.planes[i].m.mem_offset = (buffer->index * 8 + i) * 4096;
            }
        } else {
            buffer->length = f->active.planes[0].size;
            buffer->m.offset = buffer->index * 8 * 4096;
        }
        return 0;
    }
    if (req == VIDIOC_DQBUF) {
        CHECK(f->streaming);
        if (f->not_ready)
            return -EAGAIN;
        struct v4l2_buffer *buffer = arg;
        uint32_t i = 0;
        while (i < f->allocated && !f->queued[i])
            ++i;
        if (i == f->allocated)
            return -EAGAIN;
        f->queued[i] = false;
        buffer->index = f->corrupt_index ? f->allocated : i;
        buffer->sequence = f->sequence++;
        buffer->flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_SOE | (f->clean_frames ? 0U : V4L2_BUF_FLAG_ERROR);
        buffer->field = V4L2_FIELD_NONE;
        buffer->timestamp.tv_sec = 123;
        buffer->timestamp.tv_usec = f->corrupt_timestamp ? -1 : 456789;
        if (f->multiplanar) {
            buffer->length = f->wrong_planes ? 1 : 2;
            for (uint32_t j = 0; j < buffer->length; ++j) {
                buffer->m.planes[j].bytesused = f->active.planes[j].size - 16;
                buffer->m.planes[j].data_offset = f->corrupt_offset ? f->active.planes[j].size : 8;
            }
            if (f->corrupt_bytes)
                buffer->m.planes[0].bytesused = f->active.planes[0].size + 1;
        } else {
            buffer->bytesused = f->active.planes[0].size - 16;
            if (f->corrupt_bytes)
                buffer->bytesused = f->active.planes[0].size + 1;
        }
        return 0;
    }
    if (req == VIDIOC_STREAMON) {
        f->streaming = true;
        if (f->event_auto) frame_event(f, f->sequence);
        return 0;
    }
    if (req == VIDIOC_STREAMOFF) {
        ++f->streamoff_calls;
        f->streaming = false;
        if (f->event_auto) f->sequence = 0;
        memset(f->queued, 0, sizeof(f->queued));
        return 0;
    }
    if (req == VIDIOC_EXPBUF) {
        struct v4l2_exportbuffer *export = arg;
        CHECK(export->flags & O_CLOEXEC);
        export->fd = 99;
        ++f->export_count;
        return 0;
    }
    fprintf(stderr, "Unexpected ioctl: %lu\n", req);
    CHECK(false);
    return -ENOTTY;
}

static const struct cc_io_ops fake_ops = {
    .open = fake_open, .close = fake_close, .ioctl = fake_ioctl,
    .map = fake_map, .unmap = fake_unmap, .lock = fake_lock,
    .wait = fake_wait, .now = fake_now, .resolve = fake_resolve, .wait_many = fake_wait_many,
    .duplicate = fake_duplicate
};

static cc_camera_info camera_info(bool pi)
{
    cc_camera_info info = { .pipeline = pi ? CC_PIPELINE_RPI_UNICAM : CC_PIPELINE_V4L2,
        .sensor_entity = 1, .video_path = "/dev/video0", .media_path = "/dev/media0" };
    return info;
}

static cc_camera *open_camera(struct fake *f)
{
    cc_camera *camera = NULL;
    cc_camera_info info = camera_info(f->pi);
    if (f->cfe)
        info.pipeline = CC_PIPELINE_RPI_CFE;
    struct cc_io io = { &fake_ops, f };
    OK(cc_camera_open_io(&info, &io, &camera));
    CHECK(camera);
    return camera;
}

static void configure(cc_camera *camera, struct fake *f)
{
    cc_format format = { .fourcc = f->active.fourcc, .width = 60, .height = 30 };
    OK(cc_camera_configure(camera, &format));
    CHECK(format.width == 64 && format.height == 32);
    CHECK(format.num_planes == (f->multiplanar ? 2U : 1U));
    CHECK(format.planes[0].size == 4096 && format.planes[0].stride == 80);
}

static void allocate(cc_camera *camera, struct fake *f)
{
    configure(camera, f);
    uint32_t count = 2;
    OK(cc_camera_allocate_buffers(camera, &count));
    CHECK(count == f->force_count);
    CHECK(f->live_maps == (int)(count * f->active.num_planes));
}

static void capture(cc_camera *camera, struct fake *f)
{
    allocate(camera, f);
    for (uint32_t i = 0; i < f->force_count; ++i)
        OK(cc_camera_queue(camera, i));
    OK(cc_camera_start(camera));
}

static void close_camera(cc_camera *camera, struct fake *f)
{
    cc_camera_close(camera);
    CHECK(f->opened == f->closed && f->live_maps == 0);
}

static void test_lifecycle(bool multiplanar)
{
    struct fake f = fake_new(false, multiplanar);
    cc_camera *camera = open_camera(&f);
    CHECK(cc_camera_fd(camera) == VIDEO_FD);
    CHECK(!strcmp(cc_camera_get_info(camera)->driver, "uvcvideo"));
    uint32_t count = 2;
    ERROR(cc_camera_allocate_buffers(camera, &count), EPIPE);
    ERROR(cc_camera_start(camera), EPIPE);
    cc_frame frame;
    ERROR(cc_camera_dequeue(camera, &frame, 0), EPIPE);
    cc_format_info format;
    OK(cc_camera_enum_format(camera, 0, &format));
    CHECK(format.fourcc == f.active.fourcc);
    ERROR(cc_camera_enum_format(camera, 1, &format), ENOENT);
    cc_size_range size;
    OK(cc_camera_enum_size(camera, format.fourcc, 0, &size));
    CHECK(size.min_width == 64 && size.max_height == 32);
    cc_sensor_mode mode;
    ERROR(cc_camera_enum_sensor_mode(camera, 0, 0, &mode), ENOTSUP);
    allocate(camera, &f);
    ERROR(cc_camera_allocate_buffers(camera, &count), EBUSY);
    cc_format current;
    OK(cc_camera_get_format(camera, &current));
    ERROR(cc_camera_configure(camera, &current), EBUSY);
    OK(cc_camera_buffer(camera, 0, &frame));
    CHECK(((const unsigned char *)frame.planes[0].data)[0] == 0x5a);
    ERROR(cc_camera_queue(camera, 3), EINVAL);
    OK(cc_camera_queue(camera, 0));
    ERROR(cc_camera_queue(camera, 0), EBUSY);
    ERROR(cc_camera_buffer(camera, 0, &frame), EBUSY);
    OK(cc_camera_start(camera));
    ERROR(cc_camera_start(camera), EBUSY);
    OK(cc_camera_dequeue(camera, &frame, 0));
    CHECK(frame.index == 0 && frame.sequence == 0);
    CHECK(frame.timestamp_ns == UINT64_C(123456789000));
    CHECK(frame.flags & V4L2_BUF_FLAG_ERROR);
    CHECK(frame.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC);
    CHECK(frame.planes[0].bytes_used == 4080);
    CHECK(frame.planes[0].data_offset == (multiplanar ? 8U : 0U));
    if (multiplanar)
        CHECK(frame.planes[1].bytes_used == 2032);
    OK(cc_camera_queue(camera, frame.index));
    OK(cc_camera_stop(camera));
    OK(cc_camera_buffer(camera, 0, &frame));
    CHECK(frame.planes[0].bytes_used == 0);
    OK(cc_camera_queue(camera, 1)); /* Prequeued buffers must also be cancelled. */
    OK(cc_camera_free_buffers(camera));
    CHECK(f.live_maps == 0 && f.allocated == 0 && f.streamoff_calls == 2);
    allocate(camera, &f);
    close_camera(camera, &f);
}

static void test_waits(void)
{
    struct fake f = fake_new(false, false);
    cc_camera *camera = open_camera(&f);
    capture(camera, &f);
    cc_frame frame;
    f.not_ready = true;
    ERROR(cc_camera_dequeue(camera, &frame, 0), EAGAIN);
    CHECK(f.wait_calls == 0);
    f.interrupt_waits = 10;
    ERROR(cc_camera_dequeue(camera, &frame, 3), ETIMEDOUT);
    CHECK(f.wait_calls == 3);
    f.interrupt_waits = 0;
    f.wait_ready_at = 5;
    OK(cc_camera_dequeue(camera, &frame, 3));
    CHECK(f.wait_calls == 5);
    f.not_ready = true;
    f.wait_ready_at = 6;
    OK(cc_camera_dequeue(camera, &frame, -1));
    f.not_ready = true;
    f.wait_error = ENODEV;
    ERROR(cc_camera_dequeue(camera, &frame, 10), ENODEV);
    close_camera(camera, &f);
}

static void fail_next(struct fake *f, unsigned long request, int nth, int error)
{
    f->fail_request = request;
    f->fail_nth = nth;
    f->fail_errno = error;
}

static void test_failures(void)
{
    for (int which = 0; which < 6; ++which) {
        struct fake f = fake_new(false, true);
        cc_camera *camera = open_camera(&f);
        configure(camera, &f);
        if (which == 0) fail_next(&f, VIDIOC_REQBUFS, 1, ENOMEM);
        if (which == 1) fail_next(&f, VIDIOC_QUERYBUF, 2, EIO);
        if (which == 2) f.fail_map_at = 3;
        if (which == 3) f.wrong_planes = true;
        if (which == 4) f.force_count = 0;
        if (which == 5) f.force_count = 257;
        uint32_t count = 2;
        CHECK(cc_camera_allocate_buffers(camera, &count) < 0);
        CHECK(f.live_maps == 0 && f.allocated == 0);
        f.fail_map_at = 0;
        f.wrong_planes = false;
        f.force_count = 3;
        OK(cc_camera_allocate_buffers(camera, &count));
        fail_next(&f, VIDIOC_QBUF, 1, EIO);
        ERROR(cc_camera_queue(camera, 0), EIO);
        cc_frame frame;
        OK(cc_camera_buffer(camera, 0, &frame));
        OK(cc_camera_queue(camera, 0));
        fail_next(&f, VIDIOC_STREAMON, 1, EIO);
        ERROR(cc_camera_start(camera), EIO);
        OK(cc_camera_start(camera));
        fail_next(&f, VIDIOC_STREAMOFF, 1, EIO);
        ERROR(cc_camera_free_buffers(camera), EIO);
        CHECK(f.live_maps == 6 && f.streaming);
        fail_next(&f, VIDIOC_STREAMOFF, 1, ENODEV);
        close_camera(camera, &f);
        CHECK(f.closed_while_streaming);
    }
}

static void test_bad_metadata(void)
{
    for (int which = 0; which < 5; ++which) {
        struct fake f = fake_new(false, true);
        cc_camera *camera = open_camera(&f);
        capture(camera, &f);
        if (which == 0) f.corrupt_bytes = true;
        if (which == 1) f.corrupt_offset = true;
        if (which == 2) f.corrupt_index = true;
        if (which == 3) f.corrupt_timestamp = true;
        if (which == 4) f.wrong_planes = true;
        cc_frame frame = { .index = 999 };
        ERROR(cc_camera_dequeue(camera, &frame, 0), EPROTO);
        CHECK(frame.index == 999); /* Caller never sees unsafe payload metadata. */
        OK(cc_camera_stop(camera));
        f.corrupt_bytes = f.corrupt_offset = f.corrupt_index = f.corrupt_timestamp = f.wrong_planes = false;
        OK(cc_camera_queue(camera, 0));
        OK(cc_camera_start(camera));
        OK(cc_camera_dequeue(camera, &frame, 0));
        close_camera(camera, &f);
    }
}

static void test_exports(void)
{
    struct fake f = fake_new(false, false);
    cc_camera *camera = open_camera(&f);
    allocate(camera, &f);
    int fd = cc_camera_export_plane(camera, 0, 0);
    CHECK(fd == 99);
    ERROR(cc_camera_export_plane(camera, 0, 1), EINVAL);
    ERROR(cc_camera_free_buffers(camera), EBUSY);
    CHECK(f.live_maps == 0);
    ERROR(cc_camera_queue(camera, 0), ENXIO);
    OK(fake_close(&f, fd));
    OK(cc_camera_free_buffers(camera));
    close_camera(camera, &f);
}

static void test_color_metadata(void)
{
    for (int extended = 0; extended < 2; ++extended) {
        struct fake f = fake_new(false, false);
        f.ext_colors = extended != 0;
        f.active.colorspace = V4L2_COLORSPACE_REC709;
        f.active.ycbcr_enc = V4L2_YCBCR_ENC_709;
        f.active.quantization = V4L2_QUANTIZATION_LIM_RANGE;
        f.active.xfer_func = V4L2_XFER_FUNC_709;
        cc_camera *camera = open_camera(&f);
        configure(camera, &f);
        cc_format format;
        OK(cc_camera_get_format(camera, &format));
        CHECK(format.colorspace == V4L2_COLORSPACE_REC709);
        CHECK(format.ycbcr_enc == (extended ? V4L2_YCBCR_ENC_709 : 0U));
        CHECK(format.quantization == (extended ? V4L2_QUANTIZATION_LIM_RANGE : 0U));
        CHECK(format.xfer_func == (extended ? V4L2_XFER_FUNC_709 : 0U));
        close_camera(camera, &f);
    }
}

static void test_controls(bool pi)
{
    struct fake f = fake_new(pi, false);
    cc_camera *camera = open_camera(&f);
    cc_control_info info;
    OK(cc_camera_query_control(camera, CONTROL_SCALAR, &info));
    CHECK(info.minimum == 0 && info.maximum == 100 && info.step == 1);
    OK(cc_camera_next_control(camera, 0, &info));
    CHECK(info.id == CONTROL_SCALAR);
    ERROR(cc_camera_next_control(camera, CONTROL_SCALAR, &info), ENOENT);
    unsigned char payload[16] = {0};
    cc_control controls[] = {
        { .id = CONTROL_SCALAR }, { .id = CONTROL_I64 },
        { .id = CONTROL_PAYLOAD, .data = payload, .size = sizeof(payload) }, { .id = CONTROL_MASK }
    };
    size_t error;
    OK(cc_camera_get_controls(camera, controls, 4, &error));
    CHECK(controls[0].value == 42 && controls[1].value == INT64_C(4000000001));
    CHECK(controls[2].size == 8 && !memcmp(payload, "payload", 8));
    CHECK(controls[3].value == UINT32_C(0x80000001));
    CHECK(error == 4);
    controls[0].value = 27;
    controls[1].value = INT64_C(10000000001);
    controls[3].value = UINT32_C(0x80000002);
    memcpy(payload, "changed", 8);
    OK(cc_camera_set_controls(camera, controls, 4, &error));
    CHECK(f.scalar == 27 && f.i64 == INT64_C(10000000001));
    CHECK(f.bitmask == UINT32_C(0x80000002) && !memcmp(f.payload, "changed", 8));
    controls[0].value = INT64_MAX;
    ERROR(cc_camera_set_controls(camera, controls, 4, &error), ERANGE);
    CHECK(error == 0);
    controls[0].value = 42;
    controls[2].size = 1;
    ERROR(cc_camera_get_controls(camera, controls, 4, &error), ENOSPC);
    CHECK(error == 2 && controls[2].size == 8);
    allocate(camera, &f);
    cc_control layout = { .id = CONTROL_LAYOUT, .value = 1 };
    ERROR(cc_camera_set_controls(camera, &layout, 1, NULL), EBUSY);
    OK(cc_camera_free_buffers(camera));
    OK(cc_camera_set_controls(camera, &layout, 1, NULL));
    uint32_t count = 2;
    ERROR(cc_camera_allocate_buffers(camera, &count), EPIPE);
    close_camera(camera, &f);
}

static void test_unicam(void)
{
    struct fake f = fake_new(true, false);
    cc_camera *camera = open_camera(&f);
    CHECK(f.opened == 4); /* Media, sensor, mux, capture node. */
    cc_sensor_mode mode;
    OK(cc_camera_enum_sensor_mode(camera, 0, 0, &mode));
    CHECK(mode.mbus_code == MEDIA_BUS_FMT_SRGGB10_1X10 && mode.sizes.max_width == 64);
    ERROR(cc_camera_enum_sensor_mode(camera, 1, 0, &mode), ENOENT);
    ERROR(cc_camera_enum_sensor_mode(camera, 0, 1, &mode), ENOENT);
    cc_format format = { .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = 60, .height = 30 };
    OK(cc_camera_try_format(camera, &format));
    CHECK(!(f.links[0].flags & MEDIA_LNK_FL_ENABLED));
    CHECK(f.links[1].flags & MEDIA_LNK_FL_ENABLED);
    CHECK(format.width == 64);
    capture(camera, &f);
    CHECK(f.links[0].flags & MEDIA_LNK_FL_ENABLED);
    CHECK(!(f.links[1].flags & MEDIA_LNK_FL_ENABLED));
    CHECK(f.sensor_set_calls == 2 && f.mux_set_calls == 2);
    cc_frame frame;
    OK(cc_camera_dequeue(camera, &frame, 10));
    close_camera(camera, &f);
    CHECK(!(f.links[0].flags & MEDIA_LNK_FL_ENABLED));
    CHECK(f.links[1].flags & MEDIA_LNK_FL_ENABLED);
    CHECK(f.links[2].flags == (MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE));

    f = fake_new(true, false);
    f.links[1].flags |= MEDIA_LNK_FL_IMMUTABLE;
    camera = open_camera(&f);
    ERROR(cc_camera_configure(camera, &format), EPERM);
    close_camera(camera, &f);

    f = fake_new(true, false);
    camera = open_camera(&f);
    f.wrong_capture_size = true;
    ERROR(cc_camera_configure(camera, &format), EPIPE);
    uint32_t count = 2;
    ERROR(cc_camera_allocate_buffers(camera, &count), EPIPE);
    f.wrong_capture_size = false;
    configure(camera, &f);
    fail_next(&f, VIDIOC_SUBDEV_S_FMT, 2, EIO); /* Sensor set, bridge fails. */
    ERROR(cc_camera_configure(camera, &format), EIO);
    ERROR(cc_camera_allocate_buffers(camera, &count), EPIPE);
    close_camera(camera, &f);
}

static void test_open_failures(void)
{
    for (int which = 0; which < 7; ++which) {
        struct fake f = fake_new(which >= 4, false);
        cc_camera_info info = camera_info(f.pi);
        struct cc_io io = { &fake_ops, &f };
        cc_camera *camera = (cc_camera *)(uintptr_t)1;
        if (which == 0) f.mc_only = true;
        if (which == 1) f.m2m = true;
        if (which == 2) f.fail_lock = true;
        if (which == 3) f.fail_open = true;
        if (which == 4) fail_next(&f, MEDIA_IOC_ENUM_LINKS, 2, EIO);
        if (which == 5) info.sensor_entity = 999;
        if (which == 6) f.fail_lock = true;
        CHECK(cc_camera_open_io(&info, &io, &camera) < 0 && camera == NULL);
        CHECK(f.opened == f.closed && f.live_maps == 0);
    }
    cc_camera_close(NULL);
    ERROR(cc_camera_fd(NULL), EINVAL);
    ERROR(cc_camera_configure(NULL, NULL), EINVAL);
    ERROR(cc_camera_open_video(NULL, NULL), EINVAL);
    CHECK(!strcmp(cc_version(), "0.1.0"));
}

static void test_cfe(void)
{
    for (int variant = 0; variant < 3; ++variant) {
        struct fake f = fake_new(true, false);
        f.cfe = true;
        f.streams = variant != 0;
        f.legacy_routing = variant == 1;
        f.links[2].source.index = f.streams ? 1 : 4;
        f.links[2].flags = 0;
        f.links[3] = f.links[2];
        f.links[3].sink.entity = 5;
        f.links[3].flags = MEDIA_LNK_FL_ENABLED;
        cc_camera *camera = open_camera(&f);
        cc_format format = { .fourcc = f.active.fourcc, .width = 64, .height = 32 };
        OK(cc_camera_try_format(camera, &format));
        CHECK(f.links[3].flags & MEDIA_LNK_FL_ENABLED);
        CHECK(!(f.links[2].flags & MEDIA_LNK_FL_ENABLED));
        capture(camera, &f);
        CHECK(!(f.links[3].flags & MEDIA_LNK_FL_ENABLED));
        CHECK(f.links[2].flags & MEDIA_LNK_FL_ENABLED);
        CHECK(f.routing_calls == (f.streams ? 2 : 0));
        cc_frame frame;
        OK(cc_camera_dequeue(camera, &frame, 10));
        close_camera(camera, &f);
        CHECK(f.links[3].flags & MEDIA_LNK_FL_ENABLED);
        CHECK(!(f.links[2].flags & MEDIA_LNK_FL_ENABLED));
    }
}

static struct fake frontend_fake(int variant)
{
    struct fake f = fake_new(true, false);
    f.frontend = f.cfe = true;
    f.streams = variant != 0;
    f.legacy_routing = variant == 1;
    f.fe_delayed = -1;
    f.active.height = 64;
    f.active.planes[0].size = 5120;
    f.links[2].source.index = f.streams ? 1 : 4;
    f.links[2].flags = 0;
    f.links[3] = f.links[2];
    f.links[3].sink.entity = 5;
    f.links[3].flags = MEDIA_LNK_FL_ENABLED;
    /* Put the FE link first to catch enable-before-disable mux mistakes. */
    struct media_link_desc swap = f.links[2];
    f.links[2] = f.links[3];
    f.links[3] = swap;
    for (uint32_t i = 4; i < 8; ++i) {
        f.links[i] = (struct media_link_desc) {
            .source = { .entity = 5, .flags = MEDIA_PAD_FL_SOURCE,
                        .index = i == 4 ? 2 : i == 5 ? 4 : 3 },
            .sink = { .entity = i == 7 ? 9 : i + 2, .index = 0, .flags = MEDIA_PAD_FL_SINK },
            .flags = MEDIA_LNK_FL_ENABLED
        };
    }
    f.links[6] = (struct media_link_desc) {
        .source = { .entity = 8, .index = 0, .flags = MEDIA_PAD_FL_SOURCE },
        .sink = { .entity = 5, .index = 1, .flags = MEDIA_PAD_FL_SINK },
        .flags = MEDIA_LNK_FL_ENABLED
    };
    return f;
}

static cc_cfe *open_frontend(struct fake *f, cc_cfe_configuration *config)
{
    struct cc_io io = { &fake_ops, f };
    cc_camera_info info = camera_info(true);
    info.pipeline = CC_PIPELINE_RPI_CFE;
    cc_cfe *cfe = NULL;
    OK(cc_cfe_open_io(&info, &io, &cfe));
    *config = (cc_cfe_configuration) { .sensor = { .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = 64, .height = 64 } };
    OK(cc_cfe_configure(cfe, config));
    CHECK(config->image.fourcc == V4L2_PIX_FMT_SRGGB16 && config->image.planes[0].stride == 128);
    CHECK(config->statistics.planes[0].size == sizeof(struct pisp_statistics));
    CHECK(config->parameters.planes[0].size == sizeof(struct pisp_fe_config));
    CHECK((f->links[2].flags & MEDIA_LNK_FL_ENABLED) && !(f->links[3].flags & MEDIA_LNK_FL_ENABLED));
    CHECK(!(f->links[7].flags & MEDIA_LNK_FL_ENABLED));
    return cfe;
}

static void test_frontend_controller(void)
{
    for (int variant = 0; variant < 3; ++variant) {
        struct fake f = frontend_fake(variant);
        struct media_link_desc original[8];
        memcpy(original, f.links, sizeof(original));
        cc_cfe_configuration config;
        cc_cfe *cfe = open_frontend(&f, &config);
        cc_fraction interval={1,25}; OK(cc_cfe_set_frame_interval(cfe,&interval));
        CHECK(interval.numerator==1001 && interval.denominator==24000);
        OK(cc_cfe_get_frame_interval(cfe,&interval)); CHECK(interval.denominator==30000);
        OK(cc_cfe_enum_sensor_frame_interval(cfe,MEDIA_BUS_FMT_SRGGB10_1X10,64,64,0,&interval)); CHECK(interval.denominator==30);
        uint32_t count = 3;
        OK(cc_cfe_allocate_buffers(cfe, &count));
        CHECK(cc_cfe_set_frame_interval(cfe,&interval)==-EBUSY);
        CHECK(count == 2 && f.live_maps == 8);
        cc_control control = { .id = CONTROL_LAYOUT, .value = 1 };
        ERROR(cc_cfe_set_controls(cfe, &control, 1, NULL), EBUSY);
        control = (cc_control) { .id = CONTROL_SCALAR, .value = 27 };
        OK(cc_cfe_set_controls(cfe, &control, 1, NULL));
        CHECK(f.scalar == 27);
        OK(cc_cfe_queue(cfe, 1, NULL, 42));
        ERROR(cc_cfe_queue(cfe, 0, NULL, 43), EBUSY);
        f.fe_delayed = CC_CFE_STATISTICS;
        cc_cfe_result result = { .cookie = 987 };
        ERROR(cc_cfe_dequeue(cfe, &result, 0), EAGAIN);
        ERROR(cc_cfe_dequeue(cfe, &result, 3), ETIMEDOUT);
        CHECK(result.cookie == 987 && f.fe_starts == 3 && f.wait_calls == 3);
        f.fe_delayed = -1;
        OK(cc_cfe_dequeue(cfe, &result, 0));
        CHECK(result.cookie == 42 && result.index == 1 && result.image.sequence == result.statistics.sequence);
        CHECK(result.image.timestamp_ns == result.statistics.timestamp_ns && result.image.planes[0].length == 8192);
        int fd = cc_cfe_export_plane(cfe, CC_CFE_IMAGE, 1, 0);
        CHECK(fd == 99);
        ERROR(cc_cfe_queue(cfe, 1, NULL, 99), EBUSY); /* Held result. */
        struct pisp_fe_config tune = {0};
        tune.global.enables = PISP_FE_ENABLE_BLA | PISP_FE_ENABLE_AGC_STATS;
        tune.bla.black_level_r = 1024;
        OK(cc_cfe_queue(cfe, 0, &tune, 123));
        CHECK(f.last_parameters.bla.black_level_r == 1024 && f.fe_starts == 3);
        OK(cc_cfe_release(cfe, 1));
        OK(cc_cfe_dequeue(cfe, &result, 100));
        CHECK(result.cookie == 123 && result.index == 0);
        OK(cc_cfe_release(cfe, 0));
        ERROR(cc_cfe_free_buffers(cfe), EBUSY);
        OK(fake_close(&f, fd));
        OK(cc_cfe_free_buffers(cfe));
        cc_cfe_close(cfe);
        CHECK(f.opened == f.closed && !f.live_maps);
        CHECK(!memcmp(original, f.links, sizeof(original)));
    }
    for (int fault = 0; fault < 10; ++fault) {
        struct fake f = frontend_fake(2);
        cc_cfe_configuration config;
        cc_cfe *cfe = open_frontend(&f, &config);
        uint32_t count = 3;
        OK(cc_cfe_allocate_buffers(cfe, &count));
        if (fault < 6) {
            fail_next(&f, fault < 3 ? VIDIOC_QBUF : VIDIOC_STREAMON, 1, EIO);
            f.fail_fd = FE_VIDEO_BASE + fault % 3;
            ERROR(cc_cfe_queue(cfe, 0, NULL, 0), EIO);
        } else {
            OK(cc_cfe_queue(cfe, 0, NULL, 0));
            if (fault < 9) {
                fail_next(&f, VIDIOC_DQBUF, 1, EIO);
                f.fail_fd = FE_VIDEO_BASE + fault % 3;
            } else f.fe_bad_pair = true;
            cc_cfe_result result = { .cookie = 456 };
            CHECK(cc_cfe_dequeue(cfe, &result, 0) == (fault < 9 ? -EIO : -EPROTO));
            CHECK(result.cookie == 456);
        }
        for (size_t i = 0; i < 3; ++i) CHECK(!f.fe_nodes[i].streaming);
        f.fe_bad_pair = false;
        OK(cc_cfe_queue(cfe, 0, NULL, 1));
        f.fe_fail_stop = true;
        ERROR(cc_cfe_stop(cfe), EIO);
        ERROR(cc_cfe_fd(cfe, CC_CFE_IMAGE), ENODEV);
        cc_cfe_close(cfe);
        CHECK(f.opened == f.closed && !f.live_maps);
    }
    for (int fail = 1; fail <= 8; ++fail) {
        struct fake f = frontend_fake(2);
        cc_cfe_configuration config;
        cc_cfe *cfe = open_frontend(&f, &config);
        f.fail_map_at = fail;
        uint32_t count = 3;
        ERROR(cc_cfe_allocate_buffers(cfe, &count), ENOMEM);
        CHECK(!f.live_maps);
        cc_cfe_close(cfe);
        CHECK(f.opened == f.closed);
    }
}

static cc_cfe_configuration parameter_configuration(uint32_t width, uint32_t height)
{
    uint32_t stride = (width * 2 + 63) & ~63U;
    return (cc_cfe_configuration) { .sensor = { .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = width, .height = height },
        .image = { .fourcc = V4L2_PIX_FMT_SRGGB16, .width = width, .height = height, .num_planes = 1,
                   .planes = { {stride, stride * height} } } };
}

static void test_frontend_dual_output(void)
{
    for (int fault = 0; fault < 7; ++fault) {
        struct fake f = frontend_fake(2);
        cc_cfe_configuration c;
        cc_cfe *fe = open_frontend(&f, &c);
        c.image.width = c.image.height = 0;
        c.crop[0] = (cc_rectangle) {2, 4, 32, 32};
        c.downscale[0] = (struct pisp_fe_downscale_config) {.xin=2,.xout=1,.yin=2,.yout=1};
        c.image1 = (cc_format) {.fourcc=CC_FOURCC('P','C','3','R')};
        c.compression_offset1 = 2048;
        c.downscale[1] = c.downscale[0];
        OK(cc_cfe_configure(fe, &c));
        CHECK(c.image.width == 16 && c.image.height == 16 && c.image.planes[0].stride == 128);
        CHECK(c.image1.width == 32 && c.image1.height == 32 && c.image1.planes[0].stride == 64);
        CHECK(f.fe_nodes[0].format.width == 64 && f.fe_nodes[3].format.height == 64);
        CHECK(f.links[7].flags & MEDIA_LNK_FL_ENABLED);
        uint32_t count = 3;
        if (fault == 6) {
            const enum cc_memory images[2] = {CC_MEMORY_DMABUF, CC_MEMORY_DMABUF};
            OK(cc_cfe_allocate_outputs(fe, images, &count));
            ERROR(cc_cfe_queue(fe, 0, NULL, 19), EPIPE);
            cc_dmabuf_plane plane = {101, c.image.planes[0].size};
            OK(cc_cfe_import_output(fe, 0, 0, &plane, 1));
            ERROR(cc_cfe_queue(fe, 0, NULL, 19), EPIPE);
            plane = (cc_dmabuf_plane) {102, c.image1.planes[0].size};
            OK(cc_cfe_import_output(fe, 1, 0, &plane, 1));
            CHECK(f.import_count == 2);
        } else OK(cc_cfe_allocate_buffers(fe, &count));
        CHECK(count == 2 && cc_cfe_fd(fe, CC_CFE_IMAGE1) == FE_VIDEO_BASE + 3);
        if (fault == 4) {
            cc_capture_devices devices = {.frontend=fe};
            cc_capture *capture;
            OK(cc_capture_create(&devices, &capture, &count));
            cc_request *request;
            OK(cc_request_create(37, &request));
            OK(cc_capture_queue(capture, request));
            cc_request *completed;
            OK(cc_capture_dequeue(capture, &completed, 20));
            CHECK(completed == request);
            const cc_capture_result *r = cc_request_get_result(request);
            CHECK(r && r->status == CC_REQUEST_COMPLETE && r->frontend_image1.num_planes == 1);
            int fd = cc_capture_export_plane(capture, request, CC_CAPTURE_FRONTEND_IMAGE1, 0);
            CHECK(fd == 99); OK(fake_close(&f, fd));
            OK(cc_capture_release(capture, request));
            cc_request_destroy(request); cc_capture_close(capture);
            CHECK(f.live_maps == 0 && f.opened == f.closed);
            continue;
        }
        if (fault == 1 || fault == 2) {
            f.fail_request = fault == 1 ? VIDIOC_QBUF : VIDIOC_STREAMON;
            f.fail_fd = FE_VIDEO_BASE + 3; f.fail_nth = 1; f.fail_errno = EIO;
            ERROR(cc_cfe_queue(fe, 0, NULL, 19), EIO);
        } else {
            OK(cc_cfe_queue(fe, 0, NULL, 19));
            CHECK(f.last_parameters.input.format.width == 64 && f.last_parameters.ch[0].crop.offset_x == 2);
            CHECK(f.last_parameters.ch[0].downscale.flags == DOWNSCALE_BAYER);
            CHECK(f.last_parameters.ch[1].compress.mode == 3 && f.last_parameters.ch[1].compress.offset == 2048);
            cc_cfe_result result = {.cookie=23};
            if (fault == 3 || fault == 5) {
                if (fault == 3) {
                    f.fail_request = VIDIOC_DQBUF; f.fail_fd = FE_VIDEO_BASE + 3; f.fail_nth = 1; f.fail_errno = EIO;
                } else f.fe_bad_image1 = true;
                CHECK(cc_cfe_dequeue(fe, &result, 0) == (fault == 3 ? -EIO : -EPROTO));
                f.fe_bad_image1 = false;
                CHECK(result.cookie == 23);
            } else {
                f.fe_delayed = 3;
                ERROR(cc_cfe_dequeue(fe, &result, 2), ETIMEDOUT);
                CHECK(result.cookie == 23);
                f.fe_delayed = -1;
                OK(cc_cfe_dequeue(fe, &result, 0));
                CHECK(result.cookie == 19 && result.image1.num_planes == 1);
                CHECK(result.image1.timestamp_ns == result.image.timestamp_ns);
                if (fault != 6) {
                    int fd = cc_cfe_export_plane(fe, CC_CFE_IMAGE1, 0, 0);
                    CHECK(fd == 99); OK(fake_close(&f, fd));
                }
                OK(cc_cfe_release(fe, 0));
                if (fault == 6) { CHECK(!f.import_count); ERROR(cc_cfe_queue(fe, 0, NULL, 19), EPIPE); }
            }
        }
        OK(cc_cfe_free_buffers(fe));
        c.image = (cc_format) {0}; c.image1 = (cc_format) {0};
        memset(c.crop, 0, sizeof(c.crop)); memset(c.downscale, 0, sizeof(c.downscale));
        OK(cc_cfe_configure(fe, &c));
        CHECK(!(f.links[7].flags & MEDIA_LNK_FL_ENABLED));
        ERROR(cc_cfe_fd(fe, CC_CFE_IMAGE1), ENOENT);
        count = 2; OK(cc_cfe_allocate_buffers(fe, &count));
        CHECK(!f.fe_nodes[3].count);
        OK(cc_cfe_queue(fe, 0, NULL, 20));
        cc_cfe_result result;
        OK(cc_cfe_dequeue(fe, &result, 0)); CHECK(!result.image1.num_planes);
        cc_cfe_close(fe);
        CHECK(f.live_maps == 0 && f.opened == f.closed);
    }
}

static void test_frontend_parameters(void)
{
    const uint32_t sensors[] = { V4L2_PIX_FMT_SRGGB8, V4L2_PIX_FMT_SRGGB10P, V4L2_PIX_FMT_SRGGB12P,
        V4L2_PIX_FMT_SGBRG10, V4L2_PIX_FMT_SBGGR12, V4L2_PIX_FMT_SGRBG10P, V4L2_PIX_FMT_Y10P,
        CC_FOURCC('Y', '1', '2', 'P') };
    const uint32_t outputs[] = { V4L2_PIX_FMT_SRGGB16, V4L2_PIX_FMT_SRGGB16, V4L2_PIX_FMT_SRGGB16,
        V4L2_PIX_FMT_SGBRG16, V4L2_PIX_FMT_SBGGR16, V4L2_PIX_FMT_SGRBG16, V4L2_PIX_FMT_Y16,
        V4L2_PIX_FMT_Y16 };
    const uint8_t orders[] = {0, 0, 0, 1, 2, 3, 128, 128};
    for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); ++i) {
        cc_cfe_configuration c = parameter_configuration(1640, 1232);
        c.sensor.fourcc = sensors[i]; c.image.fourcc = outputs[i];
        struct pisp_fe_config parameters;
        OK(cc_cfe_build_parameters(&c, NULL, &parameters));
        CHECK(parameters.global.bayer_order == orders[i]);
        CHECK(parameters.input.streaming == 1 && parameters.ch[0].output.format.width == 1640);
        CHECK(parameters.awb_stats.size_x == 50 && parameters.awb_stats.size_y == 38);
        CHECK(parameters.agc_stats.size_x == 102 && parameters.agc_stats.size_y == 76);
        CHECK(parameters.cdaf_stats.size_x == 204 && parameters.cdaf_stats.size_y == 154);
        CHECK(parameters.output_axi.maxlen_flags == 0xaf && parameters.output_axi.throttle == 0x4100);
        CHECK(parameters.output_buffer[0].addr_lo == 0 && parameters.stats_buffer.addr_hi == 0);
        CHECK(!(parameters.global.enables & PISP_FE_ENABLE_OUTPUT1));
    }
    cc_cfe_configuration c = parameter_configuration(640, 480);
    struct pisp_fe_config tune = {0}, parameters;
    tune.global.enables = PISP_FE_ENABLE_LSC;
    tune.output_buffer[0].addr_lo = 0xdeadbeef;
    OK(cc_cfe_build_parameters(&c, &tune, &parameters));
    CHECK(parameters.lsc.centre_x == 320 && parameters.lsc.centre_y == 240);
    CHECK(parameters.lsc.shift == 7 && parameters.lsc.scale == 786 && !parameters.output_buffer[0].addr_lo);
    c = parameter_configuration(8000, 6000);
    OK(cc_cfe_build_parameters(&c, NULL, &parameters));
    CHECK(parameters.global.enables & PISP_FE_ENABLE_DECIMATE);
    CHECK(parameters.awb_stats.size_x == 126 && parameters.awb_stats.size_y == 94);
    CHECK(parameters.ch[0].output.format.width == 8000 && parameters.input.format.height == 6000);
    CHECK(parameters.floating_stats.regions[0].size_x == 4000);
    for (int invalid = 0; invalid < 6; ++invalid) {
        c = parameter_configuration(640, 480);
        tune = (struct pisp_fe_config) {0};
        if (invalid == 0) c.image.planes[0].size = 1;
        if (invalid == 1) tune.global.enables = PISP_FE_ENABLE_OUTPUT1;
        if (invalid == 2) { tune.global.enables = PISP_FE_ENABLE_STATS_CROP; tune.stats_crop.width = 641; tune.stats_crop.height = 480; }
        if (invalid == 3) { tune.global.enables = PISP_FE_ENABLE_LSC; tune.lsc.centre_x = 1000; }
        if (invalid == 4) { tune.global.enables = PISP_FE_ENABLE_AGC_STATS; tune.agc_stats.offset_x = 400; }
        if (invalid == 5) c.sensor.fourcc = V4L2_PIX_FMT_RGB24;
        memset(&parameters, 0xa5, sizeof(parameters));
        struct pisp_fe_config unchanged = parameters;
        CHECK(cc_cfe_build_parameters(&c, &tune, &parameters) < 0);
        CHECK(!memcmp(&parameters, &unchanged, sizeof(parameters)));
    }
}

static cc_capture *request_graph(struct fake *f, bool frontend, uint32_t *count)
{
    cc_capture_devices devices = {0};
    if (frontend) {
        cc_cfe_configuration config;
        devices.frontend = open_frontend(f, &config);
        *count = 3;
        OK(cc_cfe_allocate_buffers(devices.frontend, count));
    } else {
        devices.camera = open_camera(f);
        allocate(devices.camera, f);
    }
    cc_capture *capture = NULL;
    OK(cc_capture_create(&devices, &capture, count));
    CHECK(capture && !devices.camera && !devices.frontend);
    return capture;
}

static void test_request_capture(bool frontend)
{
    struct fake f = frontend ? frontend_fake(2) : fake_new(false, true);
    f.clean_frames = true;
    uint32_t count = 0;
    cc_capture *graph = request_graph(&f, frontend, &count);
    CHECK(count == (frontend ? 2U : 3U));
    cc_control control = {.id = CONTROL_SCALAR, .value = 27};
    cc_control_info info;
    OK(cc_capture_query_control(graph, CONTROL_SCALAR, &info));
    CHECK(info.type == V4L2_CTRL_TYPE_INTEGER);
    OK(cc_capture_next_control(graph, 0, &info));
    OK(cc_capture_set_controls(graph, &control, 1, NULL));
    control.value = 0;
    OK(cc_capture_get_controls(graph, &control, 1, NULL)); CHECK(control.value == 27);
    control = (cc_control) {.id = CONTROL_LAYOUT, .value = 1};
    ERROR(cc_capture_set_controls(graph, &control, 1, NULL), EBUSY);
    cc_request *requests[5];
    for (uint64_t i = 0; i < 5; ++i) {
        OK(cc_request_create(100 + i, &requests[i]));
        if (frontend) {
            struct pisp_fe_config tuning = {0};
            tuning.global.enables = PISP_FE_ENABLE_BLA;
            tuning.bla.black_level_r = (uint16_t)(1000 + i);
            OK(cc_request_set_tuning(requests[i], &tuning, NULL));
            tuning.bla.black_level_r = 0;
        }
        OK(cc_capture_queue(graph, requests[i]));
        ERROR(cc_capture_queue(graph, requests[i]), EBUSY);
        ERROR(cc_request_destroy(requests[i]), EBUSY);
        ERROR(cc_request_set_cookie(requests[i], 0), EBUSY);
        CHECK(cc_request_get_status(requests[i]) == CC_REQUEST_PENDING && !cc_request_get_result(requests[i]));
    }
    cc_request *result = (cc_request *)(uintptr_t)1;
    if (frontend) f.fe_delayed = CC_CFE_STATISTICS;
    else f.not_ready = true;
    ERROR(cc_capture_dequeue(graph, &result, 3), ETIMEDOUT);
    CHECK(result == (cc_request *)(uintptr_t)1 && f.wait_calls == 3);
    struct pollfd fds[3];
    size_t n = 0;
    OK(cc_capture_pollfds(graph, NULL, &n)); CHECK(n == 1);
    memset(fds, 0xa5, sizeof(fds));
    struct pollfd unchanged[3]; memcpy(unchanged, fds, sizeof(fds));
    n = 0;
    ERROR(cc_capture_pollfds(graph, fds, &n), ENOSPC);
    CHECK(n == 1 && !memcmp(fds, unchanged, sizeof(fds)));
    n = 3;
    OK(cc_capture_pollfds(graph, fds, &n));
    CHECK(fds[0].fd == (frontend ? FE_VIDEO_BASE + 1 : VIDEO_FD) && fds[0].events == POLLIN);
    if (frontend) {
        CHECK(f.last_parameters.bla.black_level_r == 1000);
        f.fe_delayed = -1;
    } else f.not_ready = false;
    for (uint64_t i = 0; i < 5; ++i) {
        OK(cc_capture_dequeue(graph, &result, 0)); CHECK(result == requests[i]);
        const cc_capture_result *r = cc_request_get_result(result);
        CHECK(r && r->status == CC_REQUEST_COMPLETE && !r->error && r->cookie == 100 + i && r->sequence == i);
        CHECK(r->raw.planes[0].data && r->raw.num_planes == (frontend ? 1U : 2U));
        if (frontend) {
            const struct pisp_fe_config *p = r->frontend_parameters.planes[0].data;
            CHECK(p->bla.black_level_r == 1000 + i && r->statistics.sequence == r->raw.sequence);
        }
        int fd = cc_capture_export_plane(graph, result, CC_CAPTURE_RAW, 0);
        CHECK(fd == 99); OK(fake_close(&f, fd));
        if (i == 0) {
            CHECK(cc_request_get_status(requests[count]) == CC_REQUEST_PENDING);
            CHECK(!cc_request_get_result(requests[1])); /* Not delivered yet. */
        }
        OK(cc_capture_release(graph, result));
        CHECK(cc_request_get_status(result) == CC_REQUEST_IDLE && !cc_request_get_result(result));
        ERROR(cc_capture_release(graph, result), EINVAL);
    }
    ERROR(cc_capture_dequeue(graph, &result, 0), EAGAIN);
    OK(cc_request_set_cookie(requests[0], 987));
    OK(cc_capture_queue(graph, requests[0]));
    OK(cc_capture_dequeue(graph, &result, 100));
    const cc_capture_result *held = cc_request_get_result(result);
    CHECK(held->sequence == 5 && held->cookie == 987);
    OK(cc_capture_queue(graph, requests[1]));
    OK(cc_capture_stop(graph));
    CHECK(held->status == CC_REQUEST_CANCELLED && !held->raw.planes[0].data);
    OK(cc_capture_release(graph, requests[0]));
    OK(cc_capture_dequeue(graph, &result, 0)); CHECK(result == requests[1]);
    CHECK(cc_request_get_result(result)->error == -ECANCELED);
    OK(cc_capture_release(graph, result));
    OK(cc_capture_queue(graph, requests[0]));
    cc_capture_close(graph); /* Detach an undelivered request. */
    CHECK(cc_request_get_status(requests[0]) == CC_REQUEST_CANCELLED && cc_request_get_result(requests[0]));
    for (size_t i = 0; i < 5; ++i) OK(cc_request_destroy(requests[i]));
    CHECK(f.opened == f.closed && !f.live_maps && !f.export_count);
}

static void test_request_failures(void)
{
    for (int fault = 0; fault < 5; ++fault) {
        struct fake f = fake_new(false, false);
        uint32_t count;
        cc_capture *c = request_graph(&f, false, &count);
        cc_request *requests[2], *r;
        for (size_t i = 0; i < 2; ++i) { OK(cc_request_create(i, &requests[i])); OK(cc_capture_queue(c, requests[i])); }
        if (fault == 0) f.corrupt_bytes = true;
        if (fault == 1) { f.fail_request = VIDIOC_QBUF; f.fail_nth = 1; f.fail_errno = EIO; }
        if (fault == 2) { f.fail_request = VIDIOC_STREAMON; f.fail_nth = 1; f.fail_errno = EIO; }
        if (fault == 3) { f.not_ready = true; f.wait_error = ENODEV; }
        if (fault == 4) { f.not_ready = true; f.fail_request = VIDIOC_STREAMOFF; f.fail_nth = 1; f.fail_errno = EIO; }
        if (fault == 4) {
            OK(cc_capture_dispatch(c));
            ERROR(cc_capture_stop(c), EIO);
            CHECK(f.opened == f.closed && !f.live_maps);
        }
        for (size_t i = 0; i < 2; ++i) {
            OK(cc_capture_dequeue(c, &r, 10)); CHECK(r == requests[i]);
            const cc_capture_result *result = cc_request_get_result(r);
            CHECK(result && result->error < 0 && !result->raw.planes[0].data);
            CHECK(result->status == (i == 0 && fault < 4 ? CC_REQUEST_FAILED : CC_REQUEST_CANCELLED));
            OK(cc_capture_release(c, r));
        }
        CHECK(cc_capture_dequeue(c, &r, 0) < 0);
        cc_capture_close(c);
        for (size_t i = 0; i < 2; ++i) OK(cc_request_destroy(requests[i]));
        CHECK(f.opened == f.closed && !f.live_maps);
    }
    struct fake f = fake_new(false, false);
    uint32_t count;
    cc_capture *c = request_graph(&f, false, &count);
    cc_request *r, *result;
    OK(cc_request_create(1, &r)); OK(cc_capture_queue(c, r));
    OK(cc_capture_dequeue(c, &result, 0));
    CHECK(cc_request_get_result(result)->status == CC_REQUEST_FAILED && cc_request_get_result(result)->error == -EIO);
    CHECK(cc_request_get_result(result)->raw.planes[0].data); /* Buffer error retains its lease. */
    cc_capture_close(c); OK(cc_request_destroy(r));
    CHECK(f.opened == f.closed && !f.live_maps);
}

static void test_request_validation(void)
{
    struct fake f = fake_new(false, false);
    cc_camera *camera = open_camera(&f);
    cc_capture_devices devices = {.camera = camera};
    cc_capture *capture = (cc_capture *)(uintptr_t)1;
    uint32_t count = 123;
    ERROR(cc_capture_create(&devices, &capture, &count), EPIPE);
    CHECK(!capture && devices.camera == camera && count == 123);
    allocate(camera, &f);
    OK(cc_camera_queue(camera, 0));
    ERROR(cc_capture_create(&devices, &capture, &count), EBUSY);
    CHECK(!capture && devices.camera == camera);
    OK(cc_camera_stop(camera));
    OK(cc_capture_create(&devices, &capture, &count));
    cc_request *r;
    OK(cc_request_create(0, &r));
    struct pisp_fe_config fe = {0};
    struct pisp_be_config be = {0};
    OK(cc_request_set_tuning(r, &fe, NULL)); ERROR(cc_capture_queue(capture, r), ENOTSUP);
    OK(cc_request_set_tuning(r, NULL, &be)); ERROR(cc_capture_queue(capture, r), ENOTSUP);
    OK(cc_request_set_tuning(r, NULL, NULL));
    cc_control control = {.id = CONTROL_SCALAR, .value = 1};
    OK(cc_request_set_isp_controls(r, &control, 1)); ERROR(cc_capture_queue(capture, r), ENOTSUP);
    control.size = 8;
    ERROR(cc_request_set_isp_controls(r, &control, 1), EINVAL);
    OK(cc_request_set_isp_controls(r, NULL, 0));
    OK(cc_request_destroy(r));
    cc_request *requests[257];
    for (size_t i = 0; i < 257; ++i) {
        OK(cc_request_create(i, &requests[i]));
        if (i < 256) OK(cc_capture_queue(capture, requests[i]));
        else ERROR(cc_capture_queue(capture, requests[i]), ENOSPC);
    }
    cc_capture_close(capture);
    for (size_t i = 0; i < 257; ++i) {
        CHECK(cc_request_get_status(requests[i]) == (i < 256 ? CC_REQUEST_CANCELLED : CC_REQUEST_IDLE));
        OK(cc_request_destroy(requests[i]));
    }
    CHECK(f.opened == f.closed && !f.live_maps);
}

static void configure_delays(cc_capture *c)
{
    const cc_sensor_control_delay params[] = {{CONTROL_SCALAR,2,0}, {CONTROL_I64,1,0},
        {CONTROL_VBLANK,2,CC_SENSOR_CONTROL_PRIORITY}};
    OK(cc_capture_configure_sensor_delays(c, params, 3));
}
static void schedule_delays(cc_capture *c)
{
    cc_control controls[] = {{.id=CONTROL_SCALAR,.value=120}, {.id=CONTROL_I64,.value=INT64_C(5000000002)},
                            {.id=CONTROL_VBLANK,.value=200}};
    OK(cc_capture_schedule_sensor_controls(c, 2, controls, 3, 55));
    memset(controls, 0, sizeof(controls));
}
static void test_sensor_delays(bool frontend)
{
    struct fake f = frontend ? frontend_fake(2) : fake_new(true, false);
    f.clean_frames = f.event_support = f.event_auto = f.sensor_clamp = true;
    uint32_t count;
    cc_capture *c = request_graph(&f, frontend, &count);
    configure_delays(c); CHECK(f.event_subscribed);
    schedule_delays(c);
    cc_control immediate = {.id=CONTROL_SCALAR,.value=20};
    size_t error_index = 999;
    ERROR(cc_capture_set_controls(c, &immediate, 1, &error_index), EBUSY);
    CHECK(error_index == 0 && f.scalar == 42);
    immediate.value = INT64_MAX;
    ERROR(cc_capture_schedule_sensor_controls(c, 2, &immediate, 1, 99), ERANGE);
    cc_request *requests[2], *r;
    OK(cc_request_create(1, &requests[0])); OK(cc_request_create(2, &requests[1]));
    OK(cc_capture_queue(c, requests[0])); OK(cc_capture_queue(c, requests[1]));
    ERROR(cc_capture_configure_sensor_delays(c, NULL, 0), EBUSY);
    OK(cc_capture_dequeue(c, &r, 0)); CHECK(r == requests[0]);
    const cc_capture_result *held = cc_request_get_result(r);
    CHECK(held->status == CC_REQUEST_COMPLETE && held->sensor.count == 3 && !held->sensor.cookie);
    CHECK(held->sensor.frame_sequence == 0 && held->sensor.controls[0].value == 42);
    CHECK(held->sensor.controls[1].value == INT64_C(4000000001) && held->sensor.controls[2].value == 50);
    OK(cc_capture_dequeue(c, &r, 0)); CHECK(r == requests[1]);
    CHECK(cc_request_get_result(r)->sensor.frame_sequence == 1 && !cc_request_get_result(r)->sensor.cookie);
    OK(cc_capture_release(c, r));
    for (uint32_t frame = 2; frame <= 140; ++frame) {
        OK(cc_capture_queue(c, r));
        OK(cc_capture_dequeue(c, &r, 0));
        const cc_capture_result *result = cc_request_get_result(r);
        CHECK(result->status == CC_REQUEST_COMPLETE && result->raw.sequence == frame);
        CHECK(result->sensor.cookie == 55 && result->sensor.frame_sequence == frame);
        CHECK(result->sensor.controls[0].value == 99 && result->sensor.controls[1].value == INT64_C(5000000000));
        CHECK(result->sensor.controls[2].value == 200);
        OK(cc_capture_release(c, r));
    }
    CHECK(f.sensor_write_count == 3);
    CHECK(f.sensor_writes[0].frame == 0 && f.sensor_writes[0].id == CONTROL_VBLANK);
    CHECK(f.sensor_writes[1].frame == 0 && f.sensor_writes[1].id == CONTROL_SCALAR && f.sensor_writes[1].value == 99);
    CHECK(f.sensor_writes[2].frame == 1 && f.sensor_writes[2].id == CONTROL_I64);
    CHECK(held->sensor.controls[0].value == 42 && !held->sensor.cookie); /* Snapshot survived history eviction. */
    struct pollfd fds[4]; size_t n = 4;
    OK(cc_capture_pollfds(c, fds, &n));
    CHECK(n == 1 && fds[0].events == POLLPRI && fds[0].fd == (frontend ? FE_VIDEO_BASE : VIDEO_FD));
    OK(cc_capture_stop(c)); CHECK(!f.event_subscribed && !held->sensor.count);
    OK(cc_capture_release(c, requests[0]));
    OK(cc_capture_queue(c, r));
    OK(cc_capture_dequeue(c, &r, 0));
    const cc_capture_result *result = cc_request_get_result(r);
    CHECK(f.event_subscribed && result->sensor.frame_sequence == 0 && !result->sensor.cookie);
    CHECK(result->sensor.controls[0].value == 99 && result->sensor.controls[2].value == 200);
    OK(cc_capture_release(c, r)); OK(cc_capture_stop(c));
    OK(cc_capture_configure_sensor_delays(c, NULL, 0));
    immediate.value = 20; OK(cc_capture_set_controls(c, &immediate, 1, NULL));
    cc_capture_close(c);
    OK(cc_request_destroy(requests[0])); OK(cc_request_destroy(requests[1]));
    CHECK(f.opened == f.closed && !f.live_maps && !f.event_subscribed);
}

static int64_t effective_write(const struct fake *f, uint32_t frame, uint32_t id, uint32_t delay, int64_t baseline_value)
{
    int64_t value = baseline_value;
    for (size_t i = 0; i < f->sensor_write_count; ++i)
        if (f->sensor_writes[i].id == id && frame - (f->sensor_writes[i].frame + delay) <= INT32_MAX)
            value = f->sensor_writes[i].value;
    return value;
}
static void check_sensor_request(const struct fake *f, const cc_capture_result *result, uint32_t frame,
                                 uint64_t context, int64_t exposure, int64_t rate, int64_t vblank)
{
    CHECK(result && result->status == CC_REQUEST_COMPLETE && !result->error && result->raw.sequence == frame);
    CHECK(result->sensor.frame_sequence == frame && result->sensor.count == 3 && result->sensor.cookie == context);
    CHECK(result->sensor.controls[0].value == exposure && result->sensor.controls[1].value == rate &&
          result->sensor.controls[2].value == vblank);
    /* Check the recorded sensor writes independently of the scheduler's
     * metadata model, using the simulated sensor's fixed 2/1/2-frame latencies. */
    CHECK(effective_write(f, frame, CONTROL_SCALAR, 2, 42) == exposure);
    CHECK(effective_write(f, frame, CONTROL_I64, 1, INT64_C(4000000001)) == rate);
    CHECK(effective_write(f, frame, CONTROL_VBLANK, 2, 50) == vblank);
}
static void test_request_sensor_controls(bool frontend)
{
    struct fake f = frontend ? frontend_fake(2) : fake_new(true, false);
    f.clean_frames = f.event_support = f.event_auto = f.sensor_clamp = true;
    uint32_t count;
    cc_capture *c = request_graph(&f, frontend, &count);
    configure_delays(c);
    cc_request *requests[3], *r;
    cc_control first[] = {{.id=CONTROL_SCALAR,.value=120}, {.id=CONTROL_I64,.value=INT64_C(5000000002)},
                          {.id=CONTROL_VBLANK,.value=200}};
    cc_control second[] = {{.id=CONTROL_SCALAR,.value=12}, {.id=CONTROL_I64,.value=4000},
                           {.id=CONTROL_VBLANK,.value=60}};
    /* Duplicate/zero application cookies are valid, not internal request IDs. */
    for (size_t i = 0; i < 3; ++i) OK(cc_request_create(0, &requests[i]));
    OK(cc_request_set_sensor_controls(requests[0], first, 3));
    OK(cc_request_set_sensor_controls(requests[1], second, 3));
    memset(first, 0, sizeof(first)); memset(second, 0, sizeof(second));
    for (size_t i = 0; i < 3; ++i) OK(cc_capture_queue(c, requests[i]));
    ERROR(cc_request_set_sensor_controls(requests[0], NULL, 0), EBUSY);
    ERROR(cc_capture_schedule_sensor_controls(c, 10, NULL, 0, 9), EBUSY);
    OK(cc_capture_dequeue(c, &r, 0)); CHECK(r == requests[0]);
    const cc_capture_result *held = cc_request_get_result(r);
    check_sensor_request(&f, held, 2, 0, 99, INT64_C(5000000000), 200);
    OK(cc_capture_dequeue(c, &r, 0)); CHECK(r == requests[1]);
    check_sensor_request(&f, cc_request_get_result(r), 5, 0, 12, 4000, 60);
    check_sensor_request(&f, held, 2, 0, 99, INT64_C(5000000000), 200);
    if (count == 2) {
        cc_request *unchanged = r;
        ERROR(cc_capture_dequeue(c, &r, 0), EAGAIN); CHECK(r == unchanged);
    }
    OK(cc_capture_release(c, requests[0]));
    OK(cc_capture_dequeue(c, &r, 0)); CHECK(r == requests[2]);
    check_sensor_request(&f, cc_request_get_result(r), 6, 0, 12, 4000, 60);
    OK(cc_capture_release(c, requests[1])); OK(cc_capture_release(c, requests[2]));
    CHECK(f.sensor_write_count == 6 && f.sensor_writes[0].id == CONTROL_VBLANK);
    CHECK(f.sensor_writes[0].frame == 0 && f.sensor_writes[2].frame == 1);
    CHECK(f.sensor_writes[3].frame == 3 && f.sensor_writes[5].frame == 4);
    OK(cc_request_set_cookie(requests[0], 999));
    OK(cc_capture_queue(c, requests[0])); OK(cc_capture_dequeue(c, &r, 0));
    check_sensor_request(&f, cc_request_get_result(r), 9, 999, 99, INT64_C(5000000000), 200);
    OK(cc_capture_release(c, r)); OK(cc_capture_stop(c));
    OK(cc_capture_queue(c, requests[0])); OK(cc_capture_dequeue(c, &r, 0));
    CHECK(r == requests[0] && cc_request_get_result(r)->raw.sequence == 2 && cc_request_get_result(r)->sensor.cookie == 999);
    OK(cc_capture_release(c, r)); OK(cc_capture_stop(c));
    OK(cc_request_set_sensor_controls(requests[0], NULL, 0));
    OK(cc_capture_queue(c, requests[0])); OK(cc_capture_dequeue(c, &r, 0));
    CHECK(cc_request_get_result(r)->raw.sequence == 0 && !cc_request_get_result(r)->sensor.cookie);
    OK(cc_capture_release(c, r)); cc_capture_close(c);
    for (size_t i = 0; i < 3; ++i) OK(cc_request_destroy(requests[i]));
    CHECK(f.opened == f.closed && !f.live_maps && !f.export_count && !f.event_subscribed);
}
static void test_request_sensor_validation(void)
{
    struct fake f = fake_new(true, false);
    f.clean_frames = f.event_support = f.event_auto = true;
    uint32_t count;
    cc_capture *c = request_graph(&f, false, &count);
    cc_request *r, *out;
    OK(cc_request_create(17, &r));
    cc_control controls[2] = {{.id=CONTROL_SCALAR,.value=88}, {.id=CONTROL_SCALAR,.value=90}};
    ERROR(cc_request_set_sensor_controls(NULL, controls, 1), EINVAL);
    ERROR(cc_request_set_sensor_controls(r, NULL, 1), EINVAL);
    ERROR(cc_request_set_sensor_controls(r, controls, CC_SENSOR_MAX_CONTROLS + 1), EINVAL);
    OK(cc_request_set_sensor_controls(r, controls, 1));
    ERROR(cc_capture_queue(c, r), EPIPE);
    configure_delays(c);
    ERROR(cc_request_set_sensor_controls(r, controls, 2), EEXIST);
    controls[1].id = 0; ERROR(cc_request_set_sensor_controls(r, controls + 1, 1), EINVAL);
    controls[1].id = CONTROL_SCALAR; controls[1].size = 1;
    ERROR(cc_request_set_sensor_controls(r, controls + 1, 1), ENOTSUP);
    /* Setter failures preserve the previous snapshot. Queue validation must
     * not subscribe, write controls or adopt requests on invalid settings. */
    OK(cc_capture_queue(c, r)); OK(cc_capture_dequeue(c, &out, 0));
    CHECK(cc_request_get_result(out)->raw.sequence == 2 && cc_request_get_result(out)->sensor.controls[0].value == 88);
    OK(cc_capture_release(c, out)); OK(cc_capture_stop(c));
    controls[0].value = INT64_MAX;
    OK(cc_request_set_sensor_controls(r, controls, 1)); ERROR(cc_capture_queue(c, r), ERANGE);
    controls[0].value = 88; controls[0].id = CONTROL_MASK;
    OK(cc_request_set_sensor_controls(r, controls, 1)); ERROR(cc_capture_queue(c, r), ENOENT);
    CHECK(cc_request_get_status(r) == CC_REQUEST_IDLE);
    OK(cc_capture_schedule_sensor_controls(c, 2, NULL, 0, 1)); /* Invalid queue did not select request mode. */
    controls[0].id = CONTROL_SCALAR;
    OK(cc_request_set_sensor_controls(r, controls, 1)); ERROR(cc_capture_queue(c, r), EBUSY);
    OK(cc_capture_stop(c));
    OK(cc_capture_queue(c, r));
    cc_capture_close(c); CHECK(cc_request_get_status(r) == CC_REQUEST_CANCELLED);
    OK(cc_request_set_sensor_controls(r, NULL, 0)); OK(cc_request_destroy(r));
    CHECK(f.opened == f.closed && !f.live_maps && !f.event_subscribed);
}
static void test_request_sensor_faults(bool frontend)
{
    for (int fault = 0; fault < 6; ++fault) {
        struct fake f = frontend ? frontend_fake(2) : fake_new(true, false);
        f.clean_frames = f.event_support = f.event_auto = true;
        uint32_t count;
        cc_capture *c = request_graph(&f, frontend, &count);
        configure_delays(c);
        cc_request *requests[2], *out = NULL;
        cc_control controls[] = {{.id=CONTROL_SCALAR,.value=88}, {.id=CONTROL_VBLANK,.value=200}};
        for (size_t i = 0; i < 2; ++i) {
            OK(cc_request_create(i + 1, &requests[i]));
            OK(cc_request_set_sensor_controls(requests[i], controls, 2)); OK(cc_capture_queue(c, requests[i]));
        }
        if (fault == 0) {
            f.event_hold = true;
            ERROR(cc_capture_dequeue(c, &out, 3), ETIMEDOUT); CHECK(!out && !f.sensor_write_count);
            CHECK(cc_request_get_status(requests[0]) == CC_REQUEST_PENDING && !cc_request_get_result(requests[0]));
            struct pollfd fds[4]; size_t n = 4;
            OK(cc_capture_pollfds(c, fds, &n)); CHECK(n == 1 && fds[0].events == POLLPRI);
            f.event_hold = false;
        } else if (fault == 1) {
            f.fail_request = VIDIOC_S_EXT_CTRLS; f.fail_nth = 2; f.fail_errno = EIO;
        } else if (fault == 2) {
            f.fail_request = VIDIOC_QBUF; f.fail_nth = frontend ? 4 : 2; f.fail_errno = EIO;
        } else if (fault == 3) f.frame_during_sensor_write = true;
        else if (fault == 4) {
            f.event_hold = true; OK(cc_capture_dispatch(c)); OK(cc_capture_stop(c));
        } else {
            f.clean_frames = false; f.fe_parameter_error = true;
        }
        for (size_t i = 0; i < 2; ++i) {
            OK(cc_capture_dequeue(c, &out, 0)); CHECK(out == requests[i]);
            const cc_capture_result *result = cc_request_get_result(out);
            if (!fault) CHECK(result->status == CC_REQUEST_COMPLETE && result->raw.sequence == (i ? 5U : 2U));
            else if (fault == 5) {
                CHECK(result->status == CC_REQUEST_FAILED && result->error == -EIO && result->raw.planes[0].data);
                CHECK(result->raw.sequence == i); /* Never hide errors in discarded settling frames. */
            } else {
                CHECK(result->status == (!i && fault != 4 ? CC_REQUEST_FAILED : CC_REQUEST_CANCELLED));
                CHECK(!result->raw.planes[0].data && !result->sensor.count);
            }
            OK(cc_capture_release(c, out));
        }
        cc_capture_close(c);
        for (size_t i = 0; i < 2; ++i) OK(cc_request_destroy(requests[i]));
        CHECK(f.opened == f.closed && !f.live_maps && !f.export_count && !f.event_subscribed);
    }
}

static void test_request_sensor_wrap(bool frontend)
{
    struct fake f = frontend ? frontend_fake(2) : fake_new(true, false);
    f.clean_frames = f.event_support = f.event_auto = true;
    uint32_t count;
    cc_capture *c = request_graph(&f, frontend, &count);
    configure_delays(c);
    cc_request *r, *out;
    OK(cc_request_create(123, &r));
    const uint32_t steps[] = {0, UINT32_C(0x7fffffff), UINT32_C(0xfffffffe)};
    for (size_t i = 0; i < 3; ++i) {
        f.sequence = f.fe_sequence = steps[i];
        OK(cc_capture_queue(c, r)); OK(cc_capture_dequeue(c, &out, 0));
        CHECK(out == r && cc_request_get_result(r)->raw.sequence == steps[i]);
        OK(cc_capture_release(c, r));
    }
    cc_control control = {.id=CONTROL_SCALAR,.value=88};
    OK(cc_request_set_sensor_controls(r, &control, 1));
    OK(cc_capture_queue(c, r)); OK(cc_capture_dequeue(c, &out, 0));
    const cc_capture_result *result = cc_request_get_result(r);
    CHECK(result->status == CC_REQUEST_COMPLETE && result->raw.sequence == 1 && result->sensor.cookie == 123);
    CHECK(result->sensor.controls[0].value == 88 && f.sensor_write_count == 1);
    CHECK(f.sensor_writes[0].frame == UINT32_MAX);
    OK(cc_capture_release(c, r)); OK(cc_capture_stop(c));
    cc_sensor_control_delay zero = {CONTROL_SCALAR,0,0};
    OK(cc_capture_configure_sensor_delays(c, &zero, 1));
    control.value = 44; OK(cc_request_set_sensor_controls(r, &control, 1));
    for (uint32_t frame = 0; frame < 2; ++frame) {
        OK(cc_capture_queue(c, r)); OK(cc_capture_dequeue(c, &out, 0));
        result = cc_request_get_result(r);
        CHECK(result->status == CC_REQUEST_COMPLETE && result->raw.sequence == frame);
        CHECK(result->sensor.cookie == 123 && result->sensor.controls[0].value == 44);
        OK(cc_capture_release(c, r));
    }
    cc_capture_close(c); OK(cc_request_destroy(r));
    CHECK(f.opened == f.closed && !f.live_maps && !f.event_subscribed);
}

static void test_ipa_cfe_feedback(void)
{
    struct fake f = frontend_fake(2);
    f.clean_frames=f.event_support=f.event_auto=f.ipa_scene=true;
    f.i64=96000000; f.hblank=144; f.scene_baseline=42;
    uint32_t count; cc_capture *c=request_graph(&f,true,&count);
    OK(cc_capture_configure_sensor_defaults(c));
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AE,.target_luminance=0.18,
        .exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4};
    cc_ipa *ipa;
    OK(cc_ipa_reference_create(&tuning,&ipa)); OK(cc_capture_set_ipa(c,&ipa)); CHECK(!ipa);
    ERROR(cc_capture_schedule_sensor_controls(c,2,NULL,0,0),EBUSY);
    cc_request *request,*out;
    OK(cc_request_create(123,&request));
    for (unsigned i=0;i<20;++i) {
        OK(cc_capture_queue(c,request)); ERROR(cc_capture_set_ipa(c,NULL),EBUSY);
        OK(cc_capture_dequeue(c,&out,0)); CHECK(out==request);
        const cc_capture_result *result=cc_request_get_result(out);
        CHECK(result->status==CC_REQUEST_COMPLETE && result->ipa.flags & CC_IPA_METADATA_AE);
        CHECK(result->sensor.cookie==123 && result->ipa.analogue_gain>=1);
        if (i>10) CHECK(result->ipa.flags & CC_IPA_AE_CONVERGED);
        OK(cc_capture_release(c,out));
    }
    CHECK(f.scalar>=85 && f.scalar<=95);
    /* Manual request controls have precedence, and algorithm snapshots never
     * mutate those reusable settings. Clearing the override restores feedback. */
    cc_control manual={.id=CONTROL_SCALAR,.value=10};
    OK(cc_request_set_sensor_controls(request,&manual,1));
    for (unsigned i=0;i<2;++i) {
        OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
        const cc_capture_result *result=cc_request_get_result(out);
        CHECK(result->sensor.controls[0].value==10 && result->ipa.luminance<0.03);
        OK(cc_capture_release(c,out));
    }
    OK(cc_request_set_sensor_controls(request,NULL,0));
    for (unsigned i=0;i<20;++i) {
        OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
        if (i>10) CHECK(cc_request_get_result(out)->ipa.flags & CC_IPA_AE_CONVERGED);
        OK(cc_capture_release(c,out));
    }
    OK(cc_capture_stop(c));
    f.scene_baseline=f.scalar; f.sensor_write_count=0;
    OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
    CHECK(cc_request_get_result(out)->ipa.flags & CC_IPA_AE_CONVERGED); /* Live restart baseline. */
    OK(cc_capture_release(c,out)); OK(cc_capture_stop(c)); OK(cc_capture_set_ipa(c,NULL));
    cc_capture_close(c); OK(cc_request_destroy(request));
    CHECK(f.opened==f.closed && !f.live_maps && !f.event_subscribed);
}

struct ipa_probe { int fault, configured, started, stopped, prepared, processed, destroyed; };
static int ipa_probe_configure(void *context, const cc_ipa_configuration *configuration)
{
    struct ipa_probe *p=context; ++p->configured;
    CHECK(configuration->pipeline==CC_IPA_CFE);
    return p->fault==0 ? -EIO : 0;
}
static int ipa_probe_start(void *context)
{
    struct ipa_probe *p=context; ++p->started; return p->fault==1 ? -EIO : 0;
}
static void ipa_probe_stop(void *context) { ++((struct ipa_probe *)context)->stopped; }
static void ipa_probe_destroy(void *context) { ++((struct ipa_probe *)context)->destroyed; }
static int ipa_probe_prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    struct ipa_probe *p=context; ++p->prepared; (void)request;
    if (p->fault==4) settings->num_sensor_controls=CC_IPA_MAX_CONTROLS+1;
    return p->fault==2 ? -EIO : 0;
}
static int ipa_probe_process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    struct ipa_probe *p=context; ++p->processed; CHECK(frame->statistics.flags & CC_IPA_STATS_LUMA);
    if (p->fault==5) { metadata->flags=CC_IPA_METADATA_AWB; metadata->red_gain=-1; metadata->blue_gain=1; }
    return p->fault==3 ? -EIO : 0;
}
static void test_ipa_graph_failures(void)
{
    for (int fault=0;fault<8;++fault) {
        struct fake f=frontend_fake(2); f.ipa_scene=f.event_auto=f.clean_frames=true; f.scene_baseline=42;
        uint32_t count; cc_capture *c=request_graph(&f,true,&count);
        struct ipa_probe probe={.fault=fault};
        cc_ipa_ops ops={CC_IPA_ABI_VERSION,sizeof(ops),ipa_probe_configure,ipa_probe_start,ipa_probe_stop,
            ipa_probe_prepare,ipa_probe_process,ipa_probe_destroy,0};
        cc_ipa *ipa; OK(cc_ipa_create(&ops,&probe,&ipa));
        if (!fault) {
            ERROR(cc_capture_set_ipa(c,&ipa),EIO); CHECK(ipa && probe.configured==1 && !probe.destroyed);
            cc_ipa_destroy(ipa); cc_capture_close(c); CHECK(probe.destroyed==1 && !probe.started);
        } else {
            OK(cc_capture_set_ipa(c,&ipa)); CHECK(!ipa);
            cc_request *requests[2],*out;
            for (size_t i=0;i<2;++i) { OK(cc_request_create(i,&requests[i])); OK(cc_capture_queue(c,requests[i])); }
            if (fault==6) {
                f.fe_delayed=1; f.fe_fail_stop=true;
                OK(cc_capture_dispatch(c)); ERROR(cc_capture_stop(c),EIO);
            } else if (fault==7) f.ipa_truncated=true;
            for (size_t i=0;i<2;++i) {
                OK(cc_capture_dequeue(c,&out,0)); CHECK(out==requests[i]);
                const cc_capture_result *result=cc_request_get_result(out);
                CHECK(result->status==(!i && fault!=6 ? CC_REQUEST_FAILED : CC_REQUEST_CANCELLED));
                CHECK(!result->raw.planes[0].data && !result->ipa.flags);
                OK(cc_capture_release(c,out));
            }
            CHECK(probe.configured==2 && probe.started==1 && probe.stopped==(fault==1 ? 0 : 1));
            if (fault==1 || fault==2 || fault==4) CHECK(!f.fe_starts);
            if (fault==7) CHECK(!probe.processed);
            cc_capture_close(c); CHECK(probe.destroyed==1);
            for (size_t i=0;i<2;++i) OK(cc_request_destroy(requests[i]));
        }
        CHECK(f.opened==f.closed && !f.live_maps);
    }
}

static void test_sensor_delay_failures(void)
{
    for (int fault = 0; fault < 8; ++fault) {
        struct fake f = fake_new(true, false);
        f.clean_frames = f.event_support = f.event_auto = true;
        uint32_t count;
        cc_capture *c = request_graph(&f, false, &count);
        configure_delays(c); schedule_delays(c);
        cc_request *request, *r = NULL;
        OK(cc_request_create(1, &request)); OK(cc_capture_queue(c, request));
        if (fault == 0 || fault == 1 || fault == 5) {
            f.event_auto = false;
            OK(cc_capture_dispatch(c)); /* Image dequeued, waiting only for SOF metadata. */
            struct pollfd fds[4]; size_t n = 4;
            OK(cc_capture_pollfds(c, fds, &n)); CHECK(n == 1 && fds[0].events == POLLPRI);
            if (fault == 0) { frame_event(&f, 0); frame_event(&f, 1); } /* Missed write deadline. */
            if (fault == 1) {
                frame_event(&f, 0); f.events[f.event_head % 256].type = V4L2_EVENT_CTRL;
            }
            if (fault == 5) {
                ERROR(cc_capture_dequeue(c, &r, 3), ETIMEDOUT); CHECK(!r);
                frame_event(&f, 0); /* Late notification, still the correct frame; no frame has overtaken it. */
            }
        }
        if (fault == 2 || fault == 3) {
            f.fail_request = VIDIOC_S_EXT_CTRLS; f.fail_nth = fault == 2 ? 1 : 2; f.fail_errno = EIO;
        }
        if (fault == 4) {
            f.not_ready = true; OK(cc_capture_dispatch(c));
            f.event_serial += 1; frame_event(&f, 1); f.not_ready = false; /* Lost event counter. */
        }
        if (fault == 7) f.frame_during_sensor_write = true;
        if (fault == 6) {
            OK(cc_capture_dispatch(c));
            f.fail_request = VIDIOC_UNSUBSCRIBE_EVENT; f.fail_nth = 1; f.fail_errno = EIO;
            ERROR(cc_capture_stop(c), EIO);
            CHECK(!f.live_maps && f.opened == f.closed && !f.event_subscribed);
        }
        OK(cc_capture_dequeue(c, &r, 10)); CHECK(r == request);
        const cc_capture_result *result = cc_request_get_result(r);
        if (fault == 5) CHECK(result->status == CC_REQUEST_COMPLETE && result->sensor.count == 3);
        else {
            CHECK(result->status == (fault == 6 ? CC_REQUEST_CANCELLED : CC_REQUEST_FAILED));
            CHECK(result->error < 0 && !result->sensor.count && !result->raw.planes[0].data);
        }
        OK(cc_capture_release(c, r)); cc_capture_close(c); OK(cc_request_destroy(request));
        CHECK(f.opened == f.closed && !f.live_maps && !f.event_subscribed);
    }
    struct fake f = fake_new(true, false);
    uint32_t count;
    cc_capture *c = request_graph(&f, false, &count);
    cc_sensor_control_delay param = {CONTROL_SCALAR,2,0};
    ERROR(cc_capture_configure_sensor_delays(c, &param, 1), ENOTTY);
    f.event_support = true;
    param.id = CONTROL_PAYLOAD; ERROR(cc_capture_configure_sensor_delays(c, &param, 1), ENOTSUP);
    param.id = CONTROL_LAYOUT; ERROR(cc_capture_configure_sensor_delays(c, &param, 1), ENOTSUP);
    param.id = CONTROL_SCALAR; param.frames = 65; ERROR(cc_capture_configure_sensor_delays(c, &param, 1), EINVAL);
    param.frames = 2; param.flags = 2; ERROR(cc_capture_configure_sensor_delays(c, &param, 1), EINVAL);
    cc_capture_close(c); CHECK(f.opened == f.closed && !f.live_maps);
}

static void test_sensor_defaults(bool frontend)
{
    struct fake f = frontend ? frontend_fake(2) : fake_new(true, false);
    f.clean_frames = f.event_support = f.event_auto = true;
    f.i64 = 96000000; f.hblank = 144;
    uint32_t count;
    cc_capture *c = request_graph(&f, frontend, &count);
    const cc_sensor_model *model;
    OK(cc_capture_get_sensor(c, &model));
    CHECK(!strcmp(cc_sensor_get_properties(model)->model, "imx219"));
    cc_sensor_timing timing;
    OK(cc_capture_get_sensor_timing(c, &timing));
    CHECK(timing.width == 64 && timing.height == (frontend ? 64U : 32U));
    CHECK(timing.pixel_rate == 96000000 && timing.hblank == 144 && timing.vblank == 50);
    cc_fraction native_interval; OK(cc_capture_get_frame_interval(c,&native_interval));
    CHECK(native_interval.numerator==1001 && native_interval.denominator==30000);
    uint64_t duration;
    OK(cc_sensor_frame_duration(&timing, &duration));
    CHECK(duration == (frontend ? UINT64_C(247000) : UINT64_C(177667)));
    cc_sensor_timing_limits limits;
    OK(cc_capture_get_sensor_timing_limits(c,&limits));
    CHECK(limits.pixel_rate==timing.pixel_rate && limits.width==timing.width && limits.height==timing.height);
    CHECK(limits.min_hblank==144 && limits.max_hblank==144 && limits.min_vblank==0 && limits.max_vblank==100);
    CHECK(limits.min_exposure==1 && limits.exposure_step==1);
    cc_sensor_timing_limits saved=limits;
    f.hblank=2000; ERROR(cc_capture_get_sensor_timing_limits(c,&limits),EPROTO);
    CHECK(!memcmp(&saved,&limits,sizeof(limits))); f.hblank=144;
    OK(cc_capture_configure_sensor_defaults(c)); CHECK(f.event_subscribed);
    cc_control_info range;
    OK(cc_capture_query_control(c, V4L2_CID_ANALOGUE_GAIN, &range));
    cc_control controls[] = {{.id=CONTROL_SCALAR,.value=99}, {0}, {.id=CONTROL_VBLANK,.value=200}};
    double actual;
    OK(cc_sensor_quantize_gain(model, &range, 2, &controls[1], &actual));
    CHECK(controls[1].value == 128 && actual == 2);
    OK(cc_capture_schedule_sensor_controls(c, 2, controls, 3, 200));
    cc_request *request, *r;
    OK(cc_request_create(0, &request));
    for (uint32_t frame = 0; frame < 3; ++frame) {
        OK(cc_capture_queue(c, request));
        ERROR(cc_capture_configure_sensor_defaults(c), EBUSY);
        OK(cc_capture_dequeue(c, &r, 0));
        const cc_capture_result *result = cc_request_get_result(r);
        CHECK(result->sensor.count == 3); /* Read-only HBLANK was not scheduled. */
        CHECK(result->sensor.controls[1].id == V4L2_CID_ANALOGUE_GAIN);
        CHECK(result->sensor.controls[1].value == (frame == 2 ? 128 : 0));
        CHECK(result->sensor.cookie == (frame == 2 ? 200U : 0U));
        OK(cc_capture_release(c, r));
    }
    CHECK(f.sensor_write_count == 3 && f.sensor_writes[0].id == CONTROL_VBLANK);
    CHECK(f.sensor_writes[1].frame == 0 && f.sensor_writes[1].id == CONTROL_SCALAR);
    CHECK(f.sensor_writes[2].frame == 1 && f.sensor_writes[2].id == V4L2_CID_ANALOGUE_GAIN);
    cc_capture_close(c); OK(cc_request_destroy(request));
    CHECK(f.opened == f.closed && !f.live_maps);
}
static void test_sensor_detection_errors(void)
{
    const char *names[] = {"unknown 10-0010", "imx219bad 10-0010", "imx327 10-001a", "imx219 10-0010"};
    for (size_t i = 0; i < 4; ++i) {
        struct fake f = fake_new(true, false);
        strcpy(f.sensor_model, names[i]); f.event_support = true;
        f.sensor_read_only = i == 3;
        uint32_t count;
        cc_capture *c = request_graph(&f, false, &count);
        ERROR(cc_capture_configure_sensor_defaults(c), i < 2 ? ENOENT : i == 2 ? ENODATA : ENOTSUP);
        CHECK(!f.event_subscribed);
        cc_capture_close(c); CHECK(f.opened == f.closed && !f.live_maps);
    }
    struct fake f = fake_new(true, false);
    cc_camera *camera = open_camera(&f);
    strcpy(camera->info.name, "ov5647 misleading display name");
    const cc_sensor_model *model;
    OK(cc_camera_get_sensor(camera, &model));
    CHECK(!strcmp(cc_sensor_get_properties(model)->model, "imx219"));
    cc_sensor_timing timing;
    ERROR(cc_camera_get_sensor_timing(camera, &timing), EPIPE);
    allocate(camera, &f);
    f.i64 = -1;
    memset(&timing, 0xa5, sizeof(timing));
    cc_sensor_timing original = timing;
    ERROR(cc_camera_get_sensor_timing(camera, &timing), ERANGE);
    CHECK(!memcmp(&timing, &original, sizeof(timing)));
    close_camera(camera, &f);
    f = fake_new(false, false);
    camera = open_camera(&f); model = (const cc_sensor_model *)(uintptr_t)1;
    ERROR(cc_camera_get_sensor(camera, &model), ENOTSUP); CHECK(!model);
    close_camera(camera, &f);
}

static void test_uvc_graph(bool multiplanar,bool media,bool v2)
{
    struct fake f=fake_new(false,multiplanar); f.uvc_controls=true; f.uvc_media=media; f.uvc_v2=v2; f.clean_frames=true;
    f.uvc_values[3]=3;
    struct cc_io io={&fake_ops,&f}; cc_camera_info info=camera_info(false);
    if (media) {
        cc_camera_info *list=NULL; size_t n=0;
        OK(cc_enumerate_media_io(&io,"/dev/media0",&list,&n));
        CHECK(n==1 && list[0].pipeline==CC_PIPELINE_UVC && !strcmp(list[0].video_path,"/dev/video0"));
        info=list[0]; cc_camera_list_free(list);
    }
    cc_camera *camera; OK(cc_camera_open_io(&info,&io,&camera));
    CHECK(cc_camera_get_info(camera)->pipeline==CC_PIPELINE_UVC && !camera->subdev_fds);
    cc_uvc_profile profile; OK(cc_camera_get_uvc_profile(camera,&profile)); CHECK(profile.flags==127 && profile.exposure_menu==10);
    cc_control_menu menu; OK(cc_camera_query_menu(camera,V4L2_CID_EXPOSURE_AUTO,3,&menu));
    CHECK(menu.id==V4L2_CID_EXPOSURE_AUTO && menu.index==3 && menu.type==V4L2_CTRL_TYPE_MENU && strlen(menu.name)==31);
    cc_control_menu saved=menu;
    ERROR(cc_camera_query_menu(camera,V4L2_CID_EXPOSURE_AUTO,0,&menu),ENOENT); CHECK(!memcmp(&menu,&saved,sizeof(menu)));
    ERROR(cc_camera_query_menu(camera,V4L2_CID_EXPOSURE_AUTO,UINT32_MAX,&menu),ENOENT);
    ERROR(cc_camera_query_menu(camera,V4L2_CID_BRIGHTNESS,0,&menu),ENOTSUP);
    OK(cc_video_query_menu(camera->device,CONTROL_MASK,2,&menu)); CHECK(menu.value==INT64_MIN && !menu.name[0]);
    f.uvc_bad_menu=true; saved=menu; ERROR(cc_camera_query_menu(camera,V4L2_CID_EXPOSURE_AUTO,3,&menu),EPROTO);
    CHECK(!memcmp(&menu,&saved,sizeof(menu))); f.uvc_bad_menu=false;
    f.uvc_missing=CC_UVC_GAMMA; OK(cc_camera_get_uvc_profile(camera,&profile)); CHECK(profile.flags==63); f.uvc_missing=0;
    allocate(camera,&f); CHECK(!f.sensor_set_calls && !f.mux_set_calls);
    uint32_t count=3; cc_capture_devices devices={.camera=camera}; cc_capture *capture;
    OK(cc_capture_create(&devices,&capture,&count)); cc_request *requests[3],*out;
    for (size_t i=0;i<3;++i) OK(cc_request_create(200+i,&requests[i]));
    cc_uvc_controls settings={.flags=127,.brightness=0.25,.contrast=1.25,.saturation=0.5,.gamma=2.36,
        .analogue_gain=2,.exposure_us=34567,.exposure_mode=CC_UVC_EXPOSURE_MANUAL};
    OK(cc_request_set_uvc_controls(requests[0],&settings)); settings.brightness=-1;
    f.not_ready=true; OK(cc_capture_queue(capture,requests[0])); OK(cc_capture_dispatch(capture));
    CHECK(f.uvc_writes==1 && f.uvc_values[0]==36 && f.uvc_values[3]==1 && f.uvc_values[4]==345);
    OK(cc_capture_dispatch(capture)); CHECK(f.uvc_writes==1);
    ERROR(cc_request_set_uvc_controls(requests[0],NULL),EBUSY);
    f.not_ready=false; OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[0]);
    const cc_capture_result *r=cc_request_get_result(out);
    CHECK(r->status==CC_REQUEST_COMPLETE && r->uvc.flags==127 && r->uvc.brightness==0.25 &&
        r->uvc.exposure_us==34500 && r->uvc.gamma==2.35 && r->uvc.exposure_mode==CC_UVC_EXPOSURE_MANUAL);
    CHECK(r->raw.num_planes==(multiplanar ? 2U:1U));
    cc_uvc_controls held=r->uvc;
    settings=(cc_uvc_controls){.flags=CC_UVC_BRIGHTNESS,.brightness=1};
    f.uvc_clamp=true; OK(cc_request_set_uvc_controls(requests[1],&settings));
    OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[1]);
    CHECK(cc_request_get_result(out)->uvc.brightness==14.0/80 && f.uvc_values[0]==30);
    CHECK(!memcmp(&held,&cc_request_get_result(requests[0])->uvc,sizeof(held)));
    OK(cc_capture_release(capture,out)); OK(cc_capture_release(capture,requests[0]));
    OK(cc_request_set_uvc_controls(requests[0],NULL));
    int writes=f.uvc_writes; OK(cc_capture_queue(capture,requests[0])); OK(cc_capture_dequeue(capture,&out,0));
    CHECK(!cc_request_get_result(out)->uvc.flags && f.uvc_writes==writes); OK(cc_capture_release(capture,out));
    OK(cc_capture_stop(capture));
    OK(cc_capture_query_menu(capture,V4L2_CID_EXPOSURE_AUTO,1,&menu)); CHECK(menu.index==1);
    f.fail_request=VIDIOC_S_EXT_CTRLS; f.fail_nth=1; f.fail_errno=EIO;
    OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_queue(capture,requests[2]));
    ERROR(cc_capture_dispatch(capture),EIO);
    OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[1] && cc_request_get_result(out)->status==CC_REQUEST_FAILED &&
        !cc_request_get_result(out)->uvc.flags); OK(cc_capture_release(capture,out));
    OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[2] && cc_request_get_result(out)->status==CC_REQUEST_CANCELLED);
    OK(cc_capture_release(capture,out)); f.fail_request=0; OK(cc_capture_stop(capture));
    OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_dequeue(capture,&out,0));
    CHECK(cc_request_get_result(out)->status==CC_REQUEST_COMPLETE); OK(cc_capture_release(capture,out));
    cc_capture_close(capture); for (size_t i=0;i<3;++i) OK(cc_request_destroy(requests[i]));
    CHECK(f.opened==f.closed && !f.live_maps && !f.streaming && !f.sensor_set_calls && !f.mux_set_calls);
}
static void test_uvc_media_errors(void)
{
    for (int variant=0;variant<2;++variant) {
        struct fake f=fake_new(false,false); f.uvc_media=true; f.uvc_v2=true;
        f.uvc_missing_default=variant==0; f.uvc_extra_default=variant==1;
        struct cc_io io={&fake_ops,&f}; cc_camera_info *list=NULL; size_t count=0;
        OK(cc_enumerate_media_io(&io,"/dev/media0",&list,&count)); CHECK(!count && !list);
        cc_camera_info info=camera_info(false); info.pipeline=CC_PIPELINE_UVC;
        cc_camera *camera=NULL; ERROR(cc_camera_open_io(&info,&io,&camera),variant ? ENOTUNIQ:ENODEV);
        CHECK(!camera && f.opened==f.closed);
    }
}
static void test_uvc_immediate_and_faults(void)
{
    {
        struct fake f=fake_new(false,false); f.uvc_controls=true;
        cc_camera *camera=open_camera(&f);
        cc_uvc_controls settings={.flags=CC_UVC_BRIGHTNESS,.brightness=0.25};
        OK(cc_camera_set_uvc_controls(camera,&settings)); CHECK(settings.brightness==0.25 && f.uvc_writes==1);
        cc_uvc_controls original=settings;
        f.uvc_missing=CC_UVC_BRIGHTNESS; ERROR(cc_camera_set_uvc_controls(camera,&settings),ENOTSUP);
        CHECK(!memcmp(&settings,&original,sizeof(settings)) && f.uvc_writes==1); f.uvc_missing=0;
        f.uvc_bad_ack=true; ERROR(cc_camera_set_uvc_controls(camera,&settings),EPROTO);
        CHECK(!memcmp(&settings,&original,sizeof(settings))); f.uvc_bad_ack=false;
        cc_uvc_profile profile; memset(&profile,0xa5,sizeof(profile)); cc_uvc_profile saved=profile;
        f.fail_request=VIDIOC_QUERY_EXT_CTRL; f.fail_nth=1; f.fail_errno=EIO;
        ERROR(cc_camera_get_uvc_profile(camera,&profile),EIO); CHECK(!memcmp(&profile,&saved,sizeof(profile)));
        f.fail_request=0; close_camera(camera,&f);
    }
    const unsigned long failures[]={VIDIOC_S_EXT_CTRLS,VIDIOC_QBUF,VIDIOC_STREAMON,VIDIOC_DQBUF};
    for (size_t i=0;i<sizeof(failures)/sizeof(failures[0]);++i) {
        struct fake f=fake_new(false,false); f.uvc_controls=true; f.clean_frames=true;
        uint32_t count; cc_capture *capture=request_graph(&f,false,&count);
        cc_request *request,*waiting,*done; OK(cc_request_create(1,&request)); OK(cc_request_create(2,&waiting));
        cc_uvc_controls controls={.flags=CC_UVC_BRIGHTNESS,.brightness=0.5}; OK(cc_request_set_uvc_controls(request,&controls));
        OK(cc_capture_queue(capture,request)); OK(cc_capture_queue(capture,waiting));
        f.fail_request=failures[i]; f.fail_nth=1; f.fail_errno=EIO;
        ERROR(cc_capture_dispatch(capture),EIO);
        OK(cc_capture_dequeue(capture,&done,0)); CHECK(done==request);
        const cc_capture_result *result=cc_request_get_result(done);
        CHECK(result->status==CC_REQUEST_FAILED && result->error==-EIO && !result->uvc.flags && !result->raw.num_planes);
        OK(cc_capture_release(capture,done)); OK(cc_capture_dequeue(capture,&done,0)); CHECK(done==waiting);
        CHECK(cc_request_get_result(done)->status==CC_REQUEST_CANCELLED); OK(cc_capture_release(capture,done));
        f.fail_request=0; OK(cc_capture_stop(capture));
        OK(cc_capture_queue(capture,request)); OK(cc_capture_dequeue(capture,&done,0));
        CHECK(cc_request_get_result(done)->status==CC_REQUEST_COMPLETE); OK(cc_capture_release(capture,done));
        cc_capture_close(capture); OK(cc_request_destroy(request)); OK(cc_request_destroy(waiting));
        CHECK(f.opened==f.closed && !f.live_maps && !f.streaming);
    }
    struct fake f=fake_new(true,false); uint32_t count; cc_capture *capture=request_graph(&f,false,&count);
    cc_request *request; OK(cc_request_create(1,&request));
    cc_uvc_controls controls={.flags=CC_UVC_GAMMA,.gamma=1}; OK(cc_request_set_uvc_controls(request,&controls));
    ERROR(cc_capture_queue(capture,request),ENOTSUP); CHECK(cc_request_get_status(request)==CC_REQUEST_IDLE);
    cc_capture_close(capture); OK(cc_request_destroy(request)); CHECK(f.opened==f.closed && !f.live_maps);
}
static void test_startup_policy_trace(bool frontend)
{
    struct fake f=frontend ? frontend_fake(2):fake_new(true,false);
    f.clean_frames=f.event_support=f.event_auto=true;
    uint32_t count;cc_capture *c=request_graph(&f,frontend,&count);
    OK(cc_capture_configure_sensor_defaults(c));OK(cc_capture_enable_sensor_trace(c,1));
    cc_sensor_mode_policy p={.hide_startup=2,.mistrust_startup=4,.hide_mode_switch=2,.mistrust_mode_switch=4};
    OK(cc_capture_configure_sensor_policy(c,&p));
    cc_control change={.id=CONTROL_SCALAR,.value=99};
    OK(cc_capture_schedule_sensor_controls(c,2,&change,1,55));
    cc_request *r,*out;OK(cc_request_create(0,&r));
    for (uint32_t i=0;i<3;++i) {
        OK(cc_capture_queue(c,r));ERROR(cc_capture_configure_sensor_policy(c,NULL),EBUSY);
        OK(cc_capture_dequeue(c,&out,0));const cc_capture_result *result=cc_request_get_result(out);
        CHECK(result->raw.sequence==i+2 && result->untrusted==(i<2));
        CHECK(result->sensor.cookie==55 && result->sensor.controls[0].value==99);
        OK(cc_capture_release(c,out));
    }
    cc_sensor_trace trace[8];size_t n;uint64_t lost;
    OK(cc_capture_read_sensor_trace(c,trace,8,&n,&lost));
    CHECK(n==1 && !lost && trace[0].id==CONTROL_SCALAR && trace[0].frame_sequence==0 && trace[0].target_sequence==2);
    CHECK(trace[0].requested==99 && trace[0].acknowledged==99 && !trace[0].error);
    OK(cc_capture_stop(c));OK(cc_capture_queue(c,r));OK(cc_capture_dequeue(c,&out,0));
    CHECK(cc_request_get_result(out)->raw.sequence==2 && cc_request_get_result(out)->untrusted);
    OK(cc_capture_release(c,out));cc_capture_close(c);OK(cc_request_destroy(r));
    CHECK(f.opened==f.closed && !f.live_maps);
}

int main(int argc, char **argv)
{
    if (argc==2 && !strcmp(argv[1],"uvc")) {
        test_uvc_graph(false,false,false); test_uvc_graph(true,false,false);
        test_uvc_graph(false,true,false); test_uvc_graph(true,true,true); test_uvc_media_errors();
        test_uvc_immediate_and_faults();
        puts("UVC default media selection, menus, live profiles, request controls, acknowledgements and restart passed"); return 0;
    }
    if (argc==2 && !strcmp(argv[1],"ipa")) {
        test_ipa_cfe_feedback(); test_ipa_graph_failures();
        puts("CFE C IPA native-statistics feedback, sensor delays, overrides and restart passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "request-controls")) {
        test_request_sensor_controls(false); test_request_sensor_controls(true); test_request_sensor_validation();
        test_request_sensor_faults(false); test_request_sensor_faults(true);
        test_request_sensor_wrap(false); test_request_sensor_wrap(true);
        puts("Request sensor controls, settled frames, snapshots, backpressure, validation, timeout and cancellation passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "sensor")) {
        test_sensor_defaults(false); test_sensor_defaults(true); test_sensor_detection_errors();
        test_startup_policy_trace(false); test_startup_policy_trace(true);
        puts("Live sensor identity/timing, known delay defaults and writable-control filtering passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "delays")) {
        test_sensor_delays(false); test_sensor_delays(true); test_sensor_delay_failures();
        puts("Sensor SOF scheduling, priority/clamped writes, metadata snapshots, restart and failures passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "requests")) {
        test_request_capture(false); test_request_capture(true); test_request_failures(); test_request_validation();
        puts("Raw/CFE request queues, backpressure, snapshots, polling, timeout and cancellation passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "cfe")) {
        test_frontend_parameters();
        test_frontend_controller();
        test_frontend_dual_output();
        puts("CFE C17 configuration, sensor/CSI/FE routing, paired image/statistics/config jobs and failures passed");
        return 0;
    }
    test_lifecycle(false);
    test_lifecycle(true);
    test_waits();
    test_failures();
    test_bad_metadata();
    test_exports();
    test_color_metadata();
    test_controls(false);
    test_controls(true);
    test_unicam();
    test_cfe();
    test_open_failures();
    puts("camera lifecycle, CSI media routes, controls, timeouts and failure cleanup passed");
    return 0;
}
