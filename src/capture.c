/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/capture.h>
#include "internal.h"
#include "delayed_controls.h"
#include "embedded_capture.h"
#include "request_buffers.h"
#include "fence.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CC_CAPTURE_MAX_REQUESTS 256U
#define CC_CAPTURE_MAX_FDS (16U + 3U * CC_CAPTURE_MAX_REQUESTS)
enum phase { WAITING, CAPTURING, READY, PROCESSING, FINISHED };
enum sensor_mode { SENSOR_UNSET, SENSOR_EXPLICIT, SENSOR_REQUEST };
struct cc_request {
    cc_capture *owner;
    cc_request *next, *done_next;
    enum phase phase;
    bool delivered, enqueued, captured, processed, have_fe, have_be;
    uint32_t slot;
    uint64_t cookie;
    struct pisp_fe_config fe;
    struct pisp_be_config be;
    bool have_vc4;
    cc_vc4_tuning vc4;
    cc_control *controls;
    int *control_fds; /* Owned native lens-shading descriptors, separate from mutable payloads. */
    size_t num_controls;
    cc_control sensor_controls[CC_SENSOR_MAX_CONTROLS];
    size_t num_sensor_controls;
    uint32_t sensor_target;
    cc_request *work; /* Private IPA-generated settings, separate from caller settings. */
    bool have_af, lens_write;
    cc_af_controls af_controls;
    cc_af_status af_status;
    cc_vimc_controls vimc_controls;
    cc_uvc_controls uvc_controls;
    struct cc_request_buffer buffers[3];
    struct cc_fence fences[3];
    uint32_t unresolved_fences;
    int fence_timeout_ms;
    uint64_t fence_deadline;
    cc_capture_result result;
    struct { cc_request *source; enum cc_capture_stream stream; } history[2];
    uint32_t history_users;
};
struct cc_capture {
    cc_capture_devices devices;
    struct cc_io io;
    cc_cfe_configuration fe;
    cc_pisp_be_configuration be;
    cc_format source_format, statistics_format;
    cc_stream *images[3];
    cc_stream *auxiliary[7]; /* Statistics/configuration/embedded/FE image1/BE raw outputs. */
    struct cc_buffer_identity *default_identities;
    size_t num_default_identities;
    cc_ipa *ipa;
    bool ipa_running;
    cc_control_info lens_range;
    bool lens_current_valid;
    int32_t lens_current;
    uint32_t count, owned;
    uint64_t sequence;
    int error;
    bool dead;
    bool started, subscribed, event_seen;
    uint32_t event_sequence;
    cc_sensor_mode_policy policy;
    bool policy_seen, ever_started, policy_mode_switch;
    uint32_t policy_first;
    bool trace_enabled;
    cc_sensor_trace trace[256];
    size_t trace_head, trace_count;
    uint64_t trace_dropped;
    size_t num_delays;
    cc_sensor_control_delay delays[CC_SENSOR_MAX_CONTROLS];
    uint32_t delay_types[CC_SENSOR_MAX_CONTROLS];
    struct cc_delayed_controls *delayed;
    enum sensor_mode sensor_mode;
    cc_request *head, *tail, *done_head, *done_tail, *capturing, *processing;
    cc_request *slots[CC_CAPTURE_MAX_REQUESTS];
};

static bool same_layout(const cc_format *a, const cc_format *b)
{
    if (a->width != b->width || a->height != b->height || a->fourcc != b->fourcc || a->num_planes != b->num_planes)
        return false;
    for (uint32_t i = 0; i < a->num_planes; ++i)
        if (a->planes[i].stride != b->planes[i].stride || a->planes[i].size != b->planes[i].size) return false;
    return true;
}

static const cc_frame *history_frame(const cc_request *source, enum cc_capture_stream stream)
{
    if (stream == CC_CAPTURE_RAW) return &source->result.raw;
    if (stream == CC_CAPTURE_FRONTEND_IMAGE1) return &source->result.frontend_image1;
    if (stream == CC_CAPTURE_TDN_OUTPUT) return &source->result.backend_auxiliary[1];
    if (stream == CC_CAPTURE_STITCH_OUTPUT) return &source->result.backend_auxiliary[3];
    return NULL;
}

static const cc_format *history_layout(const cc_capture *c, enum cc_capture_stream stream, uint32_t *offset)
{
    *offset = 0;
    if (stream == CC_CAPTURE_RAW) {
        if (c->devices.frontend) *offset = c->fe.compression_offset;
        return &c->source_format;
    }
    if (stream == CC_CAPTURE_FRONTEND_IMAGE1 && c->fe.image1.fourcc) {
        *offset = c->fe.compression_offset1;
        return &c->fe.image1;
    }
    if (stream == CC_CAPTURE_TDN_OUTPUT || stream == CC_CAPTURE_STITCH_OUTPUT) {
        uint32_t i = stream == CC_CAPTURE_TDN_OUTPUT ? 1U : 3U;
        if (c->be.auxiliary[i].format.fourcc) {
            *offset = c->be.auxiliary[i].compression_offset;
            return &c->be.auxiliary[i].format;
        }
    }
    return NULL;
}

static void drop_history(cc_request *r, uint32_t index)
{
    cc_request *source = r->history[index].source;
    if (source && source != r) --source->history_users;
    r->history[index].source = NULL;
}
static void clear_history(cc_request *r)
{
    for (uint32_t i = 0; i < 2; ++i) drop_history(r,i);
}

static int validate_history(cc_capture *c, const cc_request *r, const struct pisp_be_config *tuning)
{
    if (!c->devices.backend)
        return r->history[0].source || r->history[1].source ? -ENOTSUP : 0;
    struct pisp_be_tiles_config p;
    int ret = cc_pisp_be_build_configuration(&c->be,tuning,&p);
    if (ret < 0) return ret;
    const uint32_t enables[2] = {PISP_BE_BAYER_ENABLE_TDN_INPUT,PISP_BE_BAYER_ENABLE_STITCH_INPUT};
    for (uint32_t i = 0; i < 2; ++i) {
        const cc_request *source = r->history[i].source;
        bool enabled = (p.config.global.bayer_enables & enables[i]) != 0;
        if (enabled != (source != NULL)) return -EINVAL;
        if (!source) continue;
        if (source != r && (source->owner != c || !source->delivered ||
            source->result.status != CC_REQUEST_COMPLETE || source->slot == UINT32_MAX)) return -ECANCELED;
        uint32_t offset;
        const cc_format *format = history_layout(c,r->history[i].stream,&offset);
        const cc_pisp_be_aux_configuration *reader = &c->be.auxiliary[2U*i];
        if (!format || !same_layout(format,&reader->format) || offset != reader->compression_offset) return -EPIPE;
        if (source != r && history_frame(source,r->history[i].stream)->num_planes != 1) return -ENODATA;
    }
    return 0;
}

static int default_buffer_identities(cc_capture *c, uint32_t mask)
{
    if (!mask) return 0;
    if (!c->io.ops->identity) return -ENOTSUP;
    size_t capacity = 0;
    for (uint32_t i = 0; i < 10; ++i) {
        cc_stream *s = i < 3 ? c->images[i] : c->auxiliary[i - 3];
        if (s && !(i < 3 && (mask & (1U << i)))) capacity += (size_t)s->num_buffers * s->format.num_planes;
    }
    if (!capacity) return 0;
    c->default_identities = calloc(capacity, sizeof(*c->default_identities));
    if (!c->default_identities) return -ENOMEM;
    for (uint32_t stream = 0; stream < 10; ++stream) {
        cc_stream *s = stream < 3 ? c->images[stream] : c->auxiliary[stream - 3];
        if (!s || (stream < 3 && (mask & (1U << stream)))) continue;
        for (uint32_t i = 0; i < s->num_buffers; ++i) for (uint32_t p = 0; p < s->format.num_planes; ++p) {
            int fd = cc_stream_canonical_export(s, i, p);
            if (fd < 0) return fd;
            struct cc_buffer_identity identity;
            int ret = c->io.ops->identity(c->io.ctx, fd, &identity.device, &identity.inode);
            c->io.ops->close(c->io.ctx, fd);
            if (ret < 0) return ret;
            c->default_identities[c->num_default_identities++] = identity;
        }
    }
    return 0;
}

int cc_capture_create(cc_capture_devices *devices, cc_capture **out, uint32_t *count)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!devices || !count || (!devices->camera == !devices->frontend) ||
        (devices->vc4 && (devices->backend || devices->frontend)) || (devices->request_buffers & ~UINT32_C(7))) return -EINVAL;
    cc_capture *c = calloc(1, sizeof(*c));
    if (!c) return -ENOMEM;
    const struct cc_io *io;
    cc_format source;
    int ret = 0;
    if (devices->frontend) {
        ret = cc_cfe_capture_info(devices->frontend, &io, &c->fe, &c->count);
        if (ret < 0) goto fail;
        source = c->fe.image;
        c->statistics_format = c->fe.statistics;
        c->images[0] = cc_cfe_image_stream(devices->frontend);
        for (uint32_t i = 0; i < 3; ++i) c->auxiliary[i] = cc_cfe_aux_stream(devices->frontend, i);
        c->auxiliary[4] = cc_cfe_aux_stream(devices->frontend, 3);
        if (c->auxiliary[4] && c->auxiliary[4]->memory == V4L2_MEMORY_DMABUF) { ret = -EINVAL; goto fail; }
    } else {
        cc_camera *camera = devices->camera;
        cc_stream *s = camera->stream;
        if (!s->configured || !s->allocated || !s->num_buffers || s->release_pending) { ret = -EPIPE; goto fail; }
        if (s->queued || s->streaming || s->poisoned) { ret = -EBUSY; goto fail; }
        c->images[0] = s;
        io = &camera->io; source = s->format; c->count = s->num_buffers;
        if (camera->embedded) {
            c->auxiliary[0] = camera->embedded->stream;
            if (!camera->embedded->source_count || !camera->embedded->stream->allocated) { ret=-EPIPE; goto fail; }
            if (camera->embedded->stream->streaming || camera->embedded->image_pending) { ret=-EBUSY; goto fail; }
            if (camera->embedded->source_count<c->count) c->count=camera->embedded->source_count;
        }
    }
    if (devices->vc4) {
        cc_vc4_configuration v;
        uint32_t n;
        ret = cc_vc4_capture_info(devices->vc4, &v, &n);
        if (ret < 0) goto fail;
        if (!same_layout(&source, &v.input)) { ret = -EPIPE; goto fail; }
        c->statistics_format = v.statistics;
        if (n < c->count) c->count = n;
    }
    if (devices->backend) {
        uint32_t n;
        ret = cc_be_capture_info(devices->backend, &c->be, &n);
        if (ret < 0) goto fail;
        if (!same_layout(&source, &c->be.input)) { ret = -EPIPE; goto fail; }
        if (devices->frontend && c->fe.compression_offset != c->be.compression_offset) { ret = -EPIPE; goto fail; }
        if (n < c->count) c->count = n;
    }
    if (c->count > CC_CAPTURE_MAX_REQUESTS) c->count = CC_CAPTURE_MAX_REQUESTS;
    for (uint32_t output = 0; output < 2; ++output)
        c->images[output + 1] = devices->backend ? cc_be_image_stream(devices->backend, output) :
            devices->vc4 ? cc_vc4_image_stream(devices->vc4, output) : NULL;
    c->auxiliary[3] = devices->backend ? cc_be_parameters_stream(devices->backend) :
        devices->vc4 ? cc_vc4_statistics_stream(devices->vc4) : NULL;
    for (uint32_t i = 0; i < 2; ++i)
        c->auxiliary[5U+i] = cc_be_aux_output_stream(devices->backend,i);
    for (uint32_t i = 0; i < 3; ++i) {
        cc_stream *s = c->images[i];
        if (devices->request_buffers & (1U << i)) {
            if (!s) { ret = -ENOTSUP; goto fail; }
            if (s->memory != V4L2_MEMORY_DMABUF) { ret = -EINVAL; goto fail; }
        } else if (s && s->memory == V4L2_MEMORY_DMABUF) {
            /* Preserve camera's pre-bound import mode. New CFE/ISP import
             * queues require explicit per-request assignment. */
            if (i || devices->frontend) { ret = -EINVAL; goto fail; }
            for (uint32_t j = 0; j < s->num_buffers; ++j)
                if (!s->buffers[j].bound) { ret = -EPIPE; goto fail; }
        }
    }
    c->source_format = source;
    c->io = *io;
    ret = default_buffer_identities(c, devices->request_buffers);
    if (ret < 0) goto fail;
    c->devices = *devices;
    *devices = (cc_capture_devices) {0}; *count = c->count; *out = c;
    return 0;
fail:
    free(c->default_identities);
    free(c);
    return ret;
}

static void free_controls(cc_request *r)
{
    for (size_t i = 0; i < r->num_controls; ++i) {
        if (r->control_fds[i] >= 0) close(r->control_fds[i]);
        free(r->controls[i].data);
    }
    free(r->control_fds); r->control_fds = NULL;
    free(r->controls); r->controls = NULL; r->num_controls = 0;
}

int cc_capture_query_control(cc_capture *c, uint32_t id, cc_control_info *info)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_query_control(c->devices.frontend, id, info) :
                                cc_camera_query_control(c->devices.camera, id, info);
}
int cc_capture_query_lens(cc_capture *c, cc_control_info *info)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_query_lens(c->devices.frontend,info):cc_camera_query_lens(c->devices.camera,info);
}
int cc_capture_query_menu(cc_capture *c,uint32_t id,uint32_t index,cc_control_menu *entry)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_query_menu(c->devices.frontend,id,index,entry):
                                cc_camera_query_menu(c->devices.camera,id,index,entry);
}
int cc_capture_get_lens_position(cc_capture *c, int32_t *position)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_lens_position(c->devices.frontend,position):
        cc_camera_get_lens_position(c->devices.camera,position);
}
static int apply_lens(cc_capture *c, int32_t *position)
{
    return c->devices.frontend ? cc_cfe_set_lens_position(c->devices.frontend,position):
        cc_camera_set_lens_position(c->devices.camera,position);
}
int cc_capture_set_lens_position(cc_capture *c, int32_t *position)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    if (c->started && (cc_ipa_get_capabilities(c->ipa) & CC_IPA_CAP_LENS)) return -EBUSY;
    int ret=apply_lens(c,position);
    if (!ret) { c->lens_current=*position; c->lens_current_valid=true; }
    return ret;
}
int cc_capture_next_control(cc_capture *c, uint32_t after_id, cc_control_info *info)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_next_control(c->devices.frontend, after_id, info) :
                                cc_camera_next_control(c->devices.camera, after_id, info);
}
int cc_capture_get_controls(cc_capture *c, cc_control *controls, size_t count, size_t *error_index)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_controls(c->devices.frontend, controls, count, error_index) :
                                cc_camera_get_controls(c->devices.camera, controls, count, error_index);
}
int cc_capture_set_controls(cc_capture *c, const cc_control *controls, size_t count, size_t *error_index)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    if ((count && !controls) || count > UINT32_MAX || count > SIZE_MAX / sizeof(struct v4l2_ext_control)) return -EINVAL;
    for (size_t i = 0; i < count; ++i)
        for (size_t j = 0; j < c->num_delays; ++j) if (controls[i].id == c->delays[j].id) {
            if (error_index) *error_index = i;
            return -EBUSY;
        }
    return c->devices.frontend ? cc_cfe_set_controls(c->devices.frontend, controls, count, error_index) :
                                cc_camera_set_controls(c->devices.camera, controls, count, error_index);
}

int cc_capture_get_sensor(cc_capture *c, const cc_sensor_model **model)
{
    if (model) *model = NULL;
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_sensor(c->devices.frontend, model) : cc_camera_get_sensor(c->devices.camera, model);
}
int cc_capture_get_properties(cc_capture *c, cc_camera_properties *properties)
{
    if (!c || !properties) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_properties(c->devices.frontend, properties) :
                                cc_camera_get_properties(c->devices.camera, properties);
}
const cc_camera_info *cc_capture_get_info(const cc_capture *c)
{
    if (!c || c->dead) return NULL;
    return c->devices.frontend ? cc_cfe_get_info(c->devices.frontend):cc_camera_get_info(c->devices.camera);
}
int cc_capture_get_orientation(cc_capture *c, enum cc_orientation *orientation)
{
    if (!c || !orientation) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_orientation(c->devices.frontend,orientation):
                                cc_camera_get_orientation(c->devices.camera,orientation);
}
int cc_capture_get_sensor_timing(cc_capture *c, cc_sensor_timing *timing)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_sensor_timing(c->devices.frontend, timing) :
                                cc_camera_get_sensor_timing(c->devices.camera, timing);
}
int cc_capture_get_frame_interval(cc_capture *c,cc_fraction *interval)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_frame_interval(c->devices.frontend,interval):
                                cc_camera_get_frame_interval(c->devices.camera,interval);
}
int cc_capture_get_sensor_streams(cc_capture *c, cc_sensor_streams *streams)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_sensor_streams(c->devices.frontend,streams):
                                cc_camera_get_sensor_streams(c->devices.camera,streams);
}
int cc_capture_get_sensor_timing_limits(cc_capture *c, cc_sensor_timing_limits *limits)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_sensor_timing_limits(c->devices.frontend,limits):
                                cc_camera_get_sensor_timing_limits(c->devices.camera,limits);
}
int cc_capture_get_embedded_configuration(cc_capture *c, cc_embedded_configuration *configuration)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.frontend ? cc_cfe_get_embedded_configuration(c->devices.frontend,configuration):
                                cc_camera_get_embedded_configuration(c->devices.camera,configuration);
}
int cc_capture_get_sensor_geometry(cc_capture *c,cc_sensor_geometry *g)
{
    if (!c || !g) return -EINVAL;
    if (c->dead) return -ENODEV;
    return c->devices.camera ? cc_camera_get_sensor_geometry(c->devices.camera,g):-ENOTSUP;
}
int cc_capture_configure_sensor_policy(cc_capture *c,const cc_sensor_mode_policy *p)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    if (c->owned || c->started) return -EBUSY;
    if (p && (p->hide_startup>64 || p->hide_mode_switch>64 || p->mistrust_startup>64 || p->mistrust_mode_switch>64)) return -EINVAL;
    c->policy=p ? *p:(cc_sensor_mode_policy){0}; c->policy_seen=false;return 0;
}
int cc_capture_enable_sensor_trace(cc_capture *c,int enable)
{
    if (!c) return -EINVAL;
    if (c->owned || c->started) return -EBUSY;
    if (c->dead) return -ENODEV;
    if (enable && !c->io.ops->now) return -ENOTSUP;
    c->trace_enabled=enable!=0;c->trace_head=c->trace_count=0;c->trace_dropped=0;return 0;
}
int cc_capture_read_sensor_trace(cc_capture *c,cc_sensor_trace *out,size_t capacity,size_t *count,uint64_t *dropped)
{
    if (!c || !count || !dropped || (capacity && !out)) return -EINVAL;
    size_t n=c->trace_count<capacity ? c->trace_count:capacity;
    for (size_t i=0;i<n;++i) out[i]=c->trace[(c->trace_head+i)%256];
    c->trace_head=(c->trace_head+n)%256;c->trace_count-=n;
    *count=n;*dropped=c->trace_dropped;c->trace_dropped=0;return 0;
}
static void trace_write(cc_capture *c,const struct cc_delayed_batch *b,size_t i,
                        int64_t wanted,uint64_t sof,uint64_t start,uint64_t end,int error)
{
    if (!c->trace_enabled) return;
    if (c->trace_count==256) { ++c->trace_dropped;c->trace_head=(c->trace_head+1)%256;--c->trace_count; }
    cc_sensor_trace *t=&c->trace[(c->trace_head+c->trace_count++)%256];
    *t=(cc_sensor_trace){.frame_sequence=(uint32_t)b->frame,.target_sequence=(uint32_t)b->targets[i],
        .id=b->controls[i].id,.requested=wanted,.acknowledged=b->controls[i].value,
        .sof_ns=sof,.write_start_ns=start,.write_end_ns=end,.error=error};
}
int cc_capture_configure_sensor_defaults(cc_capture *c)
{
    if (!c) return -EINVAL;
    if (c->dead) return -ENODEV;
    if (c->owned || c->started) return -EBUSY;
    const cc_sensor_model *model;
    int ret = cc_capture_get_sensor(c, &model);
    if (ret < 0) return ret;
    const cc_sensor_properties *properties = cc_sensor_get_properties(model);
    if (!(properties->features & CC_SENSOR_DELAYS)) return -ENODATA;
    cc_sensor_control_delay selected[4];
    size_t count = 0;
    for (uint32_t i = 0; i < properties->num_delays; ++i) {
        cc_control_info info;
        ret = cc_capture_query_control(c, properties->delays[i].id, &info);
        if (ret == -EINVAL || ret == -ENOENT || ret == -ENOTTY) continue;
        if (ret < 0) return ret;
        if (info.flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_INACTIVE)) continue;
        selected[count++] = properties->delays[i];
    }
    return count ? cc_capture_configure_sensor_delays(c, selected, count) : -ENOTSUP;
}

static int source_fd(const cc_capture *c)
{
    return c->devices.frontend ? cc_cfe_fd(c->devices.frontend, CC_CFE_IMAGE) : cc_camera_fd(c->devices.camera);
}
static int subscribe(cc_capture *c, bool enable)
{
    if (c->subscribed == enable) return 0;
    struct v4l2_event_subscription event = {.type = V4L2_EVENT_FRAME_SYNC};
    int fd = source_fd(c);
    if (fd < 0) return fd;
    int ret = cc_ioctl(&c->io, fd, enable ? VIDIOC_SUBSCRIBE_EVENT : VIDIOC_UNSUBSCRIBE_EVENT, &event);
    if (!ret) { c->subscribed = enable; c->event_seen = false; }
    return ret;
}
static int baseline(cc_capture *c)
{
    if (!c->delayed) return 0;
    cc_control values[CC_SENSOR_MAX_CONTROLS] = {{0}};
    for (size_t i = 0; i < c->num_delays; ++i) values[i].id = c->delays[i].id;
    int ret = cc_capture_get_controls(c, values, c->num_delays, NULL);
    if (!ret) ret = cc_delayed_reset(c->delayed, values);
    if (!ret) c->sensor_mode = SENSOR_UNSET;
    return ret;
}
int cc_capture_configure_sensor_delays(cc_capture *c, const cc_sensor_control_delay *params, size_t count)
{
    if (!c || (count && !params) || count > CC_SENSOR_MAX_CONTROLS) return -EINVAL;
    if (c->dead) return -ENODEV;
    if (c->owned || c->started) return -EBUSY;
    if (!count) {
        int ret = subscribe(c, false);
        if (ret < 0) return ret;
        cc_delayed_destroy(c->delayed); c->delayed = NULL; c->num_delays = 0;
        c->sensor_mode = SENSOR_UNSET;
        return 0;
    }
    cc_control initial[CC_SENSOR_MAX_CONTROLS] = {{0}};
    uint32_t types[CC_SENSOR_MAX_CONTROLS];
    for (size_t i = 0; i < count; ++i) {
        cc_control_info info;
        int ret = cc_capture_query_control(c, params[i].id, &info);
        if (ret < 0) return ret;
        if (info.flags & (V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_WRITE_ONLY |
                          V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_MODIFY_LAYOUT | V4L2_CTRL_FLAG_DISABLED |
                          V4L2_CTRL_FLAG_INACTIVE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE)) return -ENOTSUP;
        if (info.type != V4L2_CTRL_TYPE_INTEGER && info.type != V4L2_CTRL_TYPE_INTEGER64 &&
            info.type != V4L2_CTRL_TYPE_BOOLEAN && info.type != V4L2_CTRL_TYPE_MENU &&
            info.type != V4L2_CTRL_TYPE_INTEGER_MENU && info.type != V4L2_CTRL_TYPE_BITMASK) return -ENOTSUP;
        initial[i].id = params[i].id;
        types[i] = info.type;
    }
    int ret = cc_capture_get_controls(c, initial, count, NULL);
    if (ret < 0) return ret;
    struct cc_delayed_controls *d = NULL;
    ret = cc_delayed_create(params, initial, count, &d);
    if (ret < 0) return ret;
    ret = subscribe(c, true);
    if (ret < 0) { cc_delayed_destroy(d); return ret; }
    cc_delayed_destroy(c->delayed); c->delayed = d; c->num_delays = count;
    c->sensor_mode = SENSOR_UNSET;
    memcpy(c->delays, params, count * sizeof(*params));
    memcpy(c->delay_types, types, count * sizeof(*types));
    return 0;
}
static int validate_sensor_controls(const cc_capture *c, const cc_control *controls, size_t count)
{
    if ((count && !controls) || count > c->num_delays) return -EINVAL;
    for (size_t i = 0; i < count; ++i) {
        if (controls[i].data || controls[i].size) return -ENOTSUP;
        for (size_t j = 0; j < i; ++j) if (controls[i].id == controls[j].id) return -EEXIST;
        size_t j = 0;
        while (j < c->num_delays && controls[i].id != c->delays[j].id) ++j;
        if (j == c->num_delays) return -ENOENT;
        if (c->delay_types[j] != V4L2_CTRL_TYPE_INTEGER64 &&
            (controls[i].value < INT32_MIN ||
             controls[i].value > (c->delay_types[j] == V4L2_CTRL_TYPE_BITMASK ? (int64_t)UINT32_MAX : INT32_MAX)))
            return -ERANGE;
    }
    return 0;
}
int cc_capture_schedule_sensor_controls(cc_capture *c, uint32_t target, const cc_control *controls,
                                        size_t count, uint64_t cookie)
{
    if (!c) return -EINVAL;
    if (c->error || c->dead) return c->error ? c->error : -ENODEV;
    if (c->sensor_mode == SENSOR_REQUEST || c->ipa) return -EBUSY;
    int ret = validate_sensor_controls(c, controls, count);
    if (ret < 0) return ret;
    if (!c->delayed) return -EPIPE;
    ret = cc_delayed_schedule(c->delayed, target, controls, count, cookie);
    if (!ret) c->sensor_mode = SENSOR_EXPLICIT;
    return ret;
}

static int sensor_events(cc_capture *c)
{
    if (!c->subscribed || !c->started) return 0;
    uint32_t sequence = 0;
    uint64_t sof=0;
    bool received = false;
    for (size_t n = 0;; ++n) {
        struct v4l2_event event = {0};
        int ret = cc_ioctl(&c->io, source_fd(c), VIDIOC_DQEVENT, &event);
        if (ret == -EAGAIN) break;
        if (ret < 0) return ret;
        if (event.type != V4L2_EVENT_FRAME_SYNC || event.id) return -EPROTO;
        if (n >= CC_SENSOR_CONTROL_HISTORY ||
            (c->event_seen && event.sequence != c->event_sequence + 1U)) return -EOVERFLOW;
        c->event_seen = true; c->event_sequence = event.sequence;
        sequence = event.u.frame_sync.frame_sequence; received = true;
        if (event.timestamp.tv_sec>=0 && event.timestamp.tv_nsec>=0)
            sof=(uint64_t)event.timestamp.tv_sec*UINT64_C(1000000000)+(uint64_t)event.timestamp.tv_nsec;
    }
    if (!received) return 0;
    /* Process the newest available SOF. Writing for an older queued event
     * would apply controls late. The planner rejects any missed due write. */
    struct cc_delayed_batch b;
    int ret = cc_delayed_prepare(c->delayed, sequence, &b);
    if (ret == -EALREADY) return 0;
    if (ret < 0) return ret;
    uint64_t write_start=0,write_end=0;
    for (size_t i = 0; i < b.count; ++i) if (b.priority[i]) {
        int64_t wanted=b.controls[i].value;
        if (c->trace_enabled) (void)c->io.ops->now(c->io.ctx,&write_start);
        ret = c->devices.frontend ? cc_cfe_apply_controls(c->devices.frontend, &b.controls[i], 1) :
                                   cc_camera_apply_controls(c->devices.camera, &b.controls[i], 1);
        if (c->trace_enabled) (void)c->io.ops->now(c->io.ctx,&write_end);
        trace_write(c,&b,i,wanted,sof,write_start,write_end,ret);
        if (ret < 0) return ret;
    }
    cc_control batch[CC_SENSOR_MAX_CONTROLS];
    size_t map[CC_SENSOR_MAX_CONTROLS], count = 0;
    for (size_t i = 0; i < b.count; ++i) if (!b.priority[i]) { map[count] = i; batch[count++] = b.controls[i]; }
    if (count) {
        if (c->trace_enabled) (void)c->io.ops->now(c->io.ctx,&write_start);
        ret = c->devices.frontend ? cc_cfe_apply_controls(c->devices.frontend, batch, count) :
                                   cc_camera_apply_controls(c->devices.camera, batch, count);
        if (c->trace_enabled) (void)c->io.ops->now(c->io.ctx,&write_end);
        for (size_t i=0;i<count;++i) {
            int64_t wanted=b.controls[map[i]].value;b.controls[map[i]]=batch[i];
            trace_write(c,&b,map[i],wanted,sof,write_start,write_end,ret);
        }
        if (ret < 0) return ret;
    }
    if (b.count) {
        /* If a new SOF arrived during the writes, their effective frame is
         * ambiguous. Never publish a model that claims the old deadline. */
        struct v4l2_event event = {0};
        ret = cc_ioctl(&c->io, source_fd(c), VIDIOC_DQEVENT, &event);
        if (!ret) return -ETIME;
        if (ret != -EAGAIN) return ret;
    }
    return cc_delayed_commit(c->delayed, &b);
}

int cc_request_create(uint64_t cookie, cc_request **out)
{
    if (!out) return -EINVAL;
    *out = calloc(1, sizeof(**out));
    if (!*out) return -ENOMEM;
    (*out)->cookie = cookie;
    (*out)->slot = UINT32_MAX;
    (*out)->fence_timeout_ms = 300;
    (*out)->result.cookie = cookie;
    return 0;
}
int cc_request_destroy(cc_request *r)
{
    if (!r) return 0;
    if (r->owner || r->history_users) return -EBUSY;
    clear_history(r);
    cc_request_destroy(r->work); free_controls(r);
    for (size_t i = 0; i < 3; ++i) { cc_request_buffer_clear(&r->buffers[i]); cc_fence_clear(&r->fences[i]); }
    free(r);
    return 0;
}
int cc_request_set_cookie(cc_request *r, uint64_t cookie)
{
    if (!r) return -EINVAL;
    if (r->owner) return -EBUSY;
    r->cookie = cookie; r->result.cookie = cookie;
    return 0;
}

int cc_request_set_pisp_input(cc_request *r, enum cc_pisp_be_port input,
                              cc_request *source, enum cc_capture_stream stream)
{
    if (!r || (input != CC_PISP_BE_TDN_INPUT && input != CC_PISP_BE_STITCH_INPUT)) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (source) {
        if (!history_frame(source,stream)) return -EINVAL;
        if (source == r) {
            if (stream != CC_CAPTURE_RAW && stream != CC_CAPTURE_FRONTEND_IMAGE1) return -EINVAL;
        } else {
            if (!source->owner || !source->delivered || source->result.status != CC_REQUEST_COMPLETE) return -ENODATA;
            if (history_frame(source,stream)->num_planes != 1) return -ENODATA;
            if (source->history_users == UINT32_MAX) return -EOVERFLOW;
        }
    }
    uint32_t index = input == CC_PISP_BE_TDN_INPUT ? 0U : 1U;
    if (source && source != r) ++source->history_users;
    drop_history(r,index);
    r->history[index].source = source; r->history[index].stream = stream;
    return 0;
}
int cc_request_set_buffer_io(cc_request *r, unsigned int stream, const cc_dmabuf_plane *planes,
                             uint32_t num_planes, const struct cc_io *io)
{
    if (!r || stream > CC_CAPTURE_OUTPUT1) return -EINVAL;
    if (r->owner) return -EBUSY;
    struct cc_request_buffer copy = {0};
    int ret = cc_request_buffer_copy(&copy, planes, num_planes, io);
    if (ret < 0) return ret;
    for (size_t i = 0; i < 3; ++i) if (i != stream && cc_request_buffer_overlaps(&copy, &r->buffers[i])) {
        cc_request_buffer_clear(&copy); return -EEXIST;
    }
    cc_request_buffer_clear(&r->buffers[stream]); r->buffers[stream] = copy;
    return 0;
}
int cc_request_set_buffer(cc_request *r, enum cc_capture_stream stream, const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    return cc_request_set_buffer_io(r, (unsigned int)stream, planes, num_planes, &cc_system_io);
}
int cc_request_set_acquire_fence_io(cc_request *r, unsigned int stream, int fd, const struct cc_io *io)
{
    if (!r || stream > CC_CAPTURE_OUTPUT1) return -EINVAL;
    if (r->owner) return -EBUSY;
    int ret = cc_fence_set(&r->fences[stream], fd, io);
    if (!ret) r->unresolved_fences &= ~(1U << stream);
    return ret;
}
int cc_request_set_acquire_fence(cc_request *r, enum cc_capture_stream stream, int fd)
{
    return cc_request_set_acquire_fence_io(r, (unsigned int)stream, fd, &cc_system_io);
}
int cc_request_set_fence_timeout(cc_request *r, int timeout_ms)
{
    if (!r || timeout_ms < 0) return -EINVAL;
    if (r->owner) return -EBUSY;
    r->fence_timeout_ms = timeout_ms; return 0;
}
int cc_request_take_acquire_fence(cc_request *r, enum cc_capture_stream stream)
{
    if (!r || (unsigned int)stream > CC_CAPTURE_OUTPUT1) return -EINVAL;
    if (r->owner && !r->delivered) return -EBUSY;
    int fd = cc_fence_take(&r->fences[stream]);
    if (fd >= 0) r->unresolved_fences &= ~(1U << stream);
    return fd;
}
int cc_request_set_tuning(cc_request *r, const struct pisp_fe_config *fe, const struct pisp_be_config *be)
{
    if (!r) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (fe) r->fe = *fe;
    if (be) r->be = *be;
    r->have_fe = fe != NULL; r->have_be = be != NULL;
    return 0;
}
int cc_request_set_vc4_tuning(cc_request *r,const cc_vc4_tuning *t)
{
    if (!r) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (t && cc_vc4_validate_tuning(t)<0) return -EINVAL;
    if (t) r->vc4=*t;
    r->have_vc4=t!=NULL;return 0;
}
int cc_request_set_isp_controls(cc_request *r, const cc_control *controls, size_t count)
{
    if (!r || (count && !controls) || count > UINT32_MAX || count > SIZE_MAX / sizeof(*controls)) return -EINVAL;
    if (r->owner) return -EBUSY;
    cc_control *copy = count ? calloc(count, sizeof(*copy)) : NULL;
    if (count && !copy) return -ENOMEM;
    int *fds = count ? malloc(count * sizeof(*fds)) : NULL;
    if (count && !fds) { free(copy); return -ENOMEM; }
    for (size_t i = 0; i < count; ++i) fds[i] = -1;
    int ret = 0;
    size_t n = 0;
    for (; n < count; ++n) {
        if (!controls[n].data != !controls[n].size) { ret = -EINVAL; break; }
        copy[n] = controls[n]; copy[n].data = NULL;
        if (controls[n].size) {
            copy[n].data = malloc(controls[n].size);
            if (!copy[n].data) { ret = -ENOMEM; break; }
            memcpy(copy[n].data, controls[n].data, controls[n].size);
            if (controls[n].id == V4L2_CID_USER_BCM2835_ISP_LENS_SHADING) {
                struct bcm2835_isp_lens_shading lens;
                if (controls[n].size != sizeof(lens)) { ret = -EINVAL; break; }
                memcpy(&lens, copy[n].data, sizeof(lens));
                if (lens.enabled) {
                    if (lens.dmabuf < 0) { ret = -EBADF; break; }
                    fds[n] = fcntl(lens.dmabuf, F_DUPFD_CLOEXEC, 0);
                    if (fds[n] < 0) { ret = -errno; break; }
                    lens.dmabuf = fds[n];
                } else lens.dmabuf = -1;
                memcpy(copy[n].data, &lens, sizeof(lens));
            }
        }
    }
    if (ret) {
        for (size_t i = 0; i < count; ++i) { if (fds[i] >= 0) close(fds[i]); free(copy[i].data); }
        free(fds); free(copy); return ret;
    }
    free_controls(r); r->controls = copy; r->control_fds = fds; r->num_controls = count;
    return 0;
}
int cc_request_set_sensor_controls(cc_request *r, const cc_control *controls, size_t count)
{
    if (!r || (count && !controls) || count > CC_SENSOR_MAX_CONTROLS) return -EINVAL;
    if (r->owner) return -EBUSY;
    for (size_t i = 0; i < count; ++i) {
        if (!controls[i].id) return -EINVAL;
        if (controls[i].data || controls[i].size) return -ENOTSUP;
        for (size_t j = 0; j < i; ++j) if (controls[i].id == controls[j].id) return -EEXIST;
    }
    if (count) memcpy(r->sensor_controls, controls, count * sizeof(*controls));
    r->num_sensor_controls = count;
    return 0;
}
int cc_request_set_af_controls(cc_request *r, const cc_af_controls *controls)
{
    if (!r) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (controls && cc_af_validate_controls(controls)<0) return -EINVAL;
    r->have_af=controls!=NULL;
    r->af_controls=controls ? *controls:(cc_af_controls){0};
    return 0;
}
int cc_request_set_vimc_controls(cc_request *r, const cc_vimc_controls *controls)
{
    if (!r) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (controls) {
        size_t count = 0;
        int ret = cc_vimc_controls_to_native(controls, NULL, &count);
        if (ret < 0) return ret;
    }
    r->vimc_controls = controls ? *controls : (cc_vimc_controls){0};
    return 0;
}
int cc_request_set_uvc_controls(cc_request *r,const cc_uvc_controls *controls)
{
    if (!r) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (controls) { int ret=cc_uvc_validate_settings(controls); if (ret<0) return ret; }
    r->uvc_controls=controls ? *controls:(cc_uvc_controls){0}; return 0;
}
int cc_request_get_status(const cc_request *r) { return r ? (int)r->result.status : -EINVAL; }
const cc_capture_result *cc_request_get_result(const cc_request *r)
{
    return !r || r->result.status == CC_REQUEST_IDLE || r->result.status == CC_REQUEST_PENDING ||
        (r->owner && !r->delivered) ? NULL : &r->result;
}

static void done(cc_capture *c, cc_request *r)
{
    clear_history(r);
    r->phase = FINISHED;
    if (r->enqueued || r->delivered) return;
    r->enqueued = true; r->done_next = NULL;
    if (c->done_tail) c->done_tail->done_next = r;
    else c->done_head = r;
    c->done_tail = r;
}
static uint32_t fence_mask(const cc_request *r)
{
    uint32_t mask = 0;
    for (uint32_t i = 0; i < 3; ++i) if (r->fences[i].valid) mask |= 1U << i;
    return mask;
}
static void cancel_preparation(cc_capture *c, cc_request *r, int error)
{
    r->unresolved_fences |= fence_mask(r); r->fence_deadline = 0;
    r->result = (cc_capture_result){.cookie=r->cookie,.sequence=r->result.sequence,.status=CC_REQUEST_CANCELLED,.error=error};
    done(c, r);
}
static int prepare_fences(cc_capture *c)
{
    uint64_t now = 0; bool have_now = false;
    for (cc_request *r = c->head; r; r = r->next) if (r->phase == WAITING) {
        int error = 0;
        for (size_t i = 0; i < 3; ++i) {
            int ret = cc_fence_update(&r->fences[i]);
            if (ret < 0) { error = ret; break; }
        }
        if (error) { cancel_preparation(c, r, error); continue; }
        if (!fence_mask(r)) { r->fence_deadline = 0; continue; }
        if (!r->fence_deadline) continue;
        if (!have_now) {
            int ret = c->io.ops->now(c->io.ctx, &now);
            if (ret < 0) return ret;
            have_now = true;
        }
        if (now >= r->fence_deadline) cancel_preparation(c, r, -ETIMEDOUT);
    }
    return 0;
}
static void close_devices(cc_capture *c)
{
    cc_pisp_be_close(c->devices.backend); cc_vc4_isp_close(c->devices.vc4);
    cc_cfe_close(c->devices.frontend); cc_camera_close(c->devices.camera);
    c->devices = (cc_capture_devices) {0}; c->dead = true;
}
static int halt(cc_capture *c, cc_request *failed, int error)
{
    int stopped = 0, ret;
    if (!c->dead) {
        if (c->devices.backend) { ret = cc_pisp_be_stop(c->devices.backend); if (ret < 0) stopped = ret; }
        if (c->devices.vc4) { ret = cc_vc4_isp_stop(c->devices.vc4); if (ret < 0 && !stopped) stopped = ret; }
        ret = c->devices.frontend ? cc_cfe_stop(c->devices.frontend) : cc_camera_stop(c->devices.camera);
        if (ret < 0 && !stopped) stopped = ret;
        if (!stopped && c->devices.camera && (c->devices.request_buffers & CC_CAPTURE_REQUEST_RAW))
            stopped = cc_stream_clear_imports(c->images[0]);
        if (!stopped) { ret = subscribe(c, false); if (ret < 0) stopped = ret; }
        if (!stopped) { ret = baseline(c); if (ret < 0) stopped = ret; }
        if (stopped) close_devices(c);
    } else stopped = c->error ? c->error : -ENODEV;
    cc_ipa_stop(c->ipa); c->ipa_running = false; c->lens_current_valid=false;
    c->error = stopped ? stopped : error;
    c->capturing = c->processing = NULL;
    c->started = false; c->policy_seen=false;
    memset(c->slots, 0, sizeof(c->slots));
    for (cc_request *r = c->head; r; r = r->next) {
        cc_request_destroy(r->work); r->work = NULL;
        uint64_t sequence = r->result.sequence;
        r->slot = UINT32_MAX; r->captured = r->processed = false;
        r->unresolved_fences |= fence_mask(r); r->fence_deadline = 0;
        r->result = (cc_capture_result) { .cookie = r->cookie, .sequence = sequence,
            .status = r == failed ? CC_REQUEST_FAILED : CC_REQUEST_CANCELLED,
            .error = r == failed ? c->error : -ECANCELED };
        done(c, r);
    }
    return c->error;
}
int cc_capture_stop(cc_capture *c) { return c ? halt(c, NULL, 0) : -EINVAL; }
void cc_capture_close(cc_capture *c)
{
    if (!c) return;
    halt(c, NULL, 0);
    if (!c->dead) close_devices(c);
    for (cc_request *r = c->head; r;) {
        cc_request *next = r->next;
        r->owner = NULL; r->next = r->done_next = NULL; r->enqueued = false;
        r = next;
    }
    cc_delayed_destroy(c->delayed);
    cc_ipa_destroy(c->ipa);
    free(c->default_identities);
    free(c);
}

int cc_capture_queue(cc_capture *c, cc_request *r)
{
    if (c && r && r->have_vc4 && !c->devices.vc4) return -ENOTSUP;
    if (!c || !r) return -EINVAL;
    if (r->owner) return -EBUSY;
    if (c->error || c->dead) return c->error ? c->error : -ENODEV;
    if (c->owned >= CC_CAPTURE_MAX_REQUESTS) return -ENOSPC;
    if (c->sequence == UINT64_MAX) return -EOVERFLOW;
    if (r->unresolved_fences) return -EEXIST;
    for (uint32_t stream = 0; stream < 3; ++stream) {
        if (r->fences[stream].valid && !c->images[stream]) return -ENOTSUP;
        const struct cc_request_buffer *buffer = &r->buffers[stream];
        if (c->devices.request_buffers & (1U << stream)) {
            int ret = cc_request_buffer_validate(buffer, &c->images[stream]->format);
            if (ret < 0) return ret;
        } else if (buffer->count) return -ENOTSUP;
        for (uint32_t p = 0; p < buffer->count; ++p) for (size_t i = 0; i < c->num_default_identities; ++i)
            if (buffer->identities[p].device == c->default_identities[i].device &&
                buffer->identities[p].inode == c->default_identities[i].inode) return -EEXIST;
        for (cc_request *owned = c->head; owned; owned = owned->next) for (size_t i = 0; i < 3; ++i)
            if (cc_request_buffer_overlaps(buffer, &owned->buffers[i])) return -EBUSY;
    }
    if ((r->have_fe && !c->devices.frontend) || (r->have_be && !c->devices.backend) ||
        (r->num_controls && !c->devices.vc4) || (r->have_af && !c->ipa)) return -ENOTSUP;
    if (r->vimc_controls.flags && (!c->devices.camera ||
        c->devices.camera->info.pipeline != CC_PIPELINE_VIMC)) return -ENOTSUP;
    if (r->uvc_controls.flags && (!c->devices.camera ||
        c->devices.camera->info.pipeline!=CC_PIPELINE_UVC)) return -ENOTSUP;
    if (r->num_sensor_controls) {
        if (!c->delayed) return -EPIPE;
        if (c->sensor_mode == SENSOR_EXPLICIT) return -EBUSY;
        int ret = validate_sensor_controls(c, r->sensor_controls, r->num_sensor_controls);
        if (ret < 0) return ret;
    }
    if (r->have_fe) {
        struct pisp_fe_config p;
        int ret = cc_cfe_build_parameters(&c->fe, &r->fe, &p);
        if (ret < 0) return ret;
    }
    int history_ret = validate_history(c,r,r->have_be ? &r->be : NULL);
    if (history_ret < 0) return history_ret;
    uint64_t fence_deadline = 0;
    if (fence_mask(r) && r->fence_timeout_ms) {
        uint64_t now;
        int ret = c->io.ops->now(c->io.ctx, &now);
        if (ret < 0) return ret;
        uint64_t duration = (uint64_t)r->fence_timeout_ms * UINT64_C(1000000);
        fence_deadline = now > UINT64_MAX - duration ? UINT64_MAX : now + duration;
    }
    if (r->num_sensor_controls) c->sensor_mode = SENSOR_REQUEST;
    r->owner = c; r->phase = WAITING; r->slot = UINT32_MAX;
    r->delivered = r->enqueued = r->captured = r->processed = false;
    r->next = r->done_next = NULL;
    r->fence_deadline = fence_deadline;
    r->result = (cc_capture_result) { .status = CC_REQUEST_PENDING, .cookie = r->cookie, .sequence = c->sequence++ };
    if (c->tail) c->tail->next = r;
    else c->head = r;
    c->tail = r; ++c->owned;
    return 0;
}

static cc_request *effective(cc_request *r) { return r->work ? r->work : r; }

static int ipa_configuration(cc_capture *c, const cc_ipa *ipa, cc_ipa_configuration *out)
{
    cc_ipa_configuration configuration = {.source=c->source_format,.sensor_format=c->source_format,
        .statistics_fourcc=c->statistics_format.fourcc,.num_sensor_controls=(uint32_t)c->num_delays};
    if (c->devices.frontend) {
        configuration.pipeline = c->devices.backend ? CC_IPA_PISP : CC_IPA_CFE;
        configuration.sensor_format = c->fe.sensor;
    } else if (c->devices.vc4) configuration.pipeline = CC_IPA_VC4;
    else if (c->devices.camera && c->devices.camera->info.pipeline == CC_PIPELINE_VIMC) configuration.pipeline = CC_IPA_VIMC;
    else return -ENOTSUP;
    int ret = cc_capture_get_sensor(c, &configuration.sensor);
    if (ret < 0 && ret != -ENOTSUP && ret != -ENOENT) return ret;
    ret = cc_capture_get_sensor_timing(c, &configuration.timing);
    if (ret < 0 && ret != -ENOTSUP && ret != -ENOENT) return ret;
    if (cc_ipa_get_capabilities(ipa)&CC_IPA_CAP_SENSOR_TIMING) {
        ret=cc_capture_get_sensor_timing_limits(c,&configuration.timing_limits);
        if (ret<0) return ret;
        memcpy(configuration.sensor_delays,c->delays,c->num_delays*sizeof(*c->delays));
    }
    if (cc_ipa_get_capabilities(ipa)&CC_IPA_CAP_SENSOR_GEOMETRY) {
        ret=cc_capture_get_sensor_geometry(c,&configuration.geometry);
        if (ret<0) return ret;
    }
    ret=cc_capture_get_embedded_configuration(c,&configuration.embedded);
    if (ret<0 && ret!=-ENOENT) return ret;
    for (size_t i = 0; i < c->num_delays; ++i) {
        ret = cc_capture_query_control(c, c->delays[i].id, &configuration.sensor_controls[i]);
        if (ret < 0) return ret;
        configuration.initial_controls[i].id = c->delays[i].id;
    }
    ret = cc_capture_get_controls(c, configuration.initial_controls, c->num_delays, NULL);
    if (ret < 0) return ret;
    if (c->devices.vc4) {
        cc_control_info *ranges[] = {&configuration.red_balance, &configuration.blue_balance, &configuration.digital_gain};
        const uint32_t ids[] = {V4L2_CID_RED_BALANCE,V4L2_CID_BLUE_BALANCE,V4L2_CID_DIGITAL_GAIN};
        for (size_t i = 0; i < 3; ++i) {
            ret = cc_vc4_isp_query_control(c->devices.vc4, ids[i], ranges[i]);
            if (ret < 0) {
                if (ret != -EINVAL && ret != -ENOENT && ret != -ENOTTY) return ret;
                *ranges[i] = (cc_control_info) {0};
            }
        }
    }
    if (cc_ipa_get_capabilities(ipa) & CC_IPA_CAP_LENS) {
        ret=cc_capture_query_lens(c,&configuration.lens);
        if (ret<0) return ret;
    }
    *out = configuration;
    return 0;
}
int cc_capture_set_ipa(cc_capture *c, cc_ipa **ipa)
{
    if (!c || (ipa && !*ipa)) return -EINVAL;
    if (c->dead) return -ENODEV;
    if (c->owned || c->started || c->ipa_running) return -EBUSY;
    if (ipa) {
        if (*ipa == c->ipa) return -EINVAL;
        if (c->sensor_mode == SENSOR_EXPLICIT) return -EBUSY;
        cc_ipa_configuration configuration;
        int ret = ipa_configuration(c, *ipa, &configuration);
        if (!ret) ret = cc_ipa_configure(*ipa, &configuration);
        if (ret < 0) return ret;
    }
    cc_ipa_destroy(c->ipa); c->ipa = ipa ? *ipa : NULL;
    if (ipa) *ipa = NULL;
    return 0;
}
static int merge_controls(const cc_control *defaults, size_t count, const cc_control *overrides, size_t num_overrides,
                           cc_control merged[CC_IPA_MAX_CONTROLS], size_t *out_count)
{
    if (count > CC_IPA_MAX_CONTROLS || num_overrides > CC_IPA_MAX_CONTROLS) return -ENOSPC;
    if (count) memcpy(merged, defaults, count * sizeof(*merged));
    for (size_t i = 0; i < num_overrides; ++i) {
        size_t j = 0;
        while (j < count && merged[j].id != overrides[i].id) ++j;
        if (j == count) { if (count == CC_IPA_MAX_CONTROLS) return -ENOSPC; ++count; }
        merged[j] = overrides[i];
    }
    *out_count = count;
    return 0;
}
static int prepare_ipa(cc_capture *c, cc_request *r)
{
    if (!c->ipa) return 0;
    int ret;
    if (!c->ipa_running) {
        cc_ipa_configuration configuration;
        ret = ipa_configuration(c, c->ipa, &configuration);
        if (!ret) ret = cc_ipa_configure(c->ipa, &configuration);
        if (!ret) ret = cc_ipa_start(c->ipa);
        if (ret < 0) return ret;
        c->ipa_running = true;
        c->lens_range=configuration.lens;
    }
    cc_ipa_settings *settings = calloc(1, sizeof(*settings));
    if (!settings) return -ENOMEM;
    cc_ipa_request input = {.sequence=r->result.sequence,.cookie=r->cookie,
        .sensor_controls=r->sensor_controls,.num_sensor_controls=r->num_sensor_controls,
        .af=r->have_af ? &r->af_controls:NULL};
    ret = cc_capture_get_sensor_timing(c, &input.timing);
    if (ret == -ENOTSUP || ret == -ENOENT) ret = 0;
    if (!ret && (cc_ipa_get_capabilities(c->ipa)&CC_IPA_CAP_SENSOR_TIMING))
        ret=cc_capture_get_sensor_timing_limits(c,&input.timing_limits);
    if (!ret) ret = cc_ipa_prepare(c->ipa, &input, settings);
    cc_request *work = NULL;
    if (!ret) ret = cc_request_create(r->cookie, &work);
    if (!ret) {
        work->af_status=settings->af; work->lens_write=(settings->flags & CC_IPA_LENS)!=0;
        if (work->lens_write && (!c->lens_range.id || work->af_status.lens_setting<c->lens_range.minimum ||
            work->af_status.lens_setting>c->lens_range.maximum)) ret=-ERANGE;
    }
    cc_control merged[CC_IPA_MAX_CONTROLS]; size_t count;
    if (!ret) ret = merge_controls(settings->sensor_controls, settings->num_sensor_controls,
                                   r->sensor_controls, r->num_sensor_controls, merged, &count);
    if (!ret) ret = cc_request_set_sensor_controls(work, merged, count);
    if (!ret && count) ret = c->delayed ? validate_sensor_controls(c, merged, count) : -EPIPE;
    if (!ret) ret = merge_controls(settings->isp_controls, settings->num_isp_controls,
                                   r->controls, r->num_controls, merged, &count);
    if (!ret) ret = cc_request_set_isp_controls(work, merged, count);
    const struct pisp_fe_config *fe = r->have_fe ? &r->fe : settings->flags & CC_IPA_FRONTEND ? &settings->frontend : NULL;
    const struct pisp_be_config *be = r->have_be ? &r->be : settings->flags & CC_IPA_BACKEND ? &settings->backend : NULL;
    if (!ret) ret = cc_request_set_tuning(work, fe, be);
    const cc_vc4_tuning *vc4=r->have_vc4 ? &r->vc4:settings->flags & CC_IPA_VC4_TUNING ? &settings->vc4:NULL;
    if (!ret && vc4 && !c->devices.vc4) ret=-ENOTSUP;
    if (!ret) ret=cc_request_set_vc4_tuning(work,vc4);
    if (!ret && ((fe && !c->devices.frontend) || (be && !c->devices.backend) ||
                 (work->num_controls && !c->devices.vc4))) ret = -ENOTSUP;
    if (!ret && fe) { struct pisp_fe_config p; ret = cc_cfe_build_parameters(&c->fe, fe, &p); }
    if (!ret) ret = validate_history(c,r,be);
    free(settings);
    if (ret < 0) cc_request_destroy(work);
    else { r->work = work; if (work->num_sensor_controls) c->sensor_mode = SENSOR_REQUEST; }
    return ret;
}
static int finish(cc_capture *c, cc_request *r)
{
    uint32_t flags = r->result.raw.flags | r->result.output[0].flags | r->result.output[1].flags |
        r->result.statistics.flags | r->result.frontend_parameters.flags | r->result.backend_parameters.flags |
        r->result.embedded.flags | r->result.frontend_image1.flags;
    for (uint32_t i = 0; i < 4; ++i) flags |= r->result.backend_auxiliary[i].flags;
    r->result.error = flags & V4L2_BUF_FLAG_ERROR ? -EIO : 0;
    r->result.status = r->result.error ? CC_REQUEST_FAILED : CC_REQUEST_COMPLETE;
    if (!r->result.error && c->ipa && !r->result.untrusted) {
        cc_frame *stats = &r->result.statistics;
        bool vimc = c->devices.camera && c->devices.camera->info.pipeline == CC_PIPELINE_VIMC;
        if ((!vimc || stats->num_planes) &&
            (stats->num_planes != 1 || !stats->planes[0].data || stats->planes[0].bytes_used < stats->planes[0].data_offset))
            return -EPROTO;
        cc_request *settings = effective(r);
        cc_ipa_frame frame = {.sequence=r->result.sequence,.cookie=r->cookie,.sensor=r->result.sensor,
            .timestamp_ns=r->result.raw.timestamp_ns,
            .source_sequence=r->result.raw.sequence,.untrusted=r->result.untrusted,
            .request_sensor_controls=r->sensor_controls,.num_request_sensor_controls=r->num_sensor_controls,
            .settings_sensor_controls=settings->sensor_controls,.num_settings_sensor_controls=settings->num_sensor_controls,
            .vc4=settings->have_vc4 ? &settings->vc4:NULL,
            .statistics_fourcc=c->statistics_format.fourcc,
            .statistics_data=stats->num_planes ? (const unsigned char *)stats->planes[0].data + stats->planes[0].data_offset : NULL,
            .statistics_size=stats->num_planes ? stats->planes[0].bytes_used - stats->planes[0].data_offset : 0,
            .isp_controls=settings->controls,.num_isp_controls=settings->num_controls,
            .backend=settings->have_be ? &settings->be : NULL,.af=settings->af_status};
        cc_frame *embedded=&r->result.embedded;
        if (embedded->num_planes) {
            if (embedded->num_planes!=1 || !embedded->planes[0].data ||
                embedded->planes[0].bytes_used<embedded->planes[0].data_offset) return -EPROTO;
            frame.embedded_data=(const unsigned char *)embedded->planes[0].data+embedded->planes[0].data_offset;
            frame.embedded_size=embedded->planes[0].bytes_used-embedded->planes[0].data_offset;
        }
        int ret = vimc && !stats->num_planes ? 0 :
            cc_ipa_parse_statistics(frame.statistics_fourcc, frame.statistics_data, frame.statistics_size, &frame.statistics);
        if (!ret) ret = cc_ipa_process(c->ipa, &frame, &r->result.ipa);
        if (ret < 0) return ret;
    }
    done(c, r);
    return 0;
}
static int queue_source(cc_capture *c, cc_request *r)
{
    cc_request *settings = effective(r);
    if (c->devices.request_buffers & CC_CAPTURE_REQUEST_RAW) {
        const struct cc_request_buffer *buffer = &r->buffers[CC_CAPTURE_RAW];
        int ret = c->devices.frontend ? cc_cfe_import_buffer(c->devices.frontend, r->slot, buffer->planes, buffer->count) :
            cc_camera_import_buffer(c->devices.camera, r->slot, buffer->planes, buffer->count);
        if (ret < 0) return ret;
    }
    if (c->devices.frontend)
        return cc_cfe_queue(c->devices.frontend, r->slot, settings->have_fe ? &settings->fe : NULL, r->result.sequence);
    int ret = cc_camera_queue(c->devices.camera, r->slot);
    if (!ret && !c->devices.camera->stream->streaming) ret = cc_camera_start(c->devices.camera);
    return ret;
}
static int captured(cc_capture *c, cc_request *r)
{
    int ret;
    if (!r->captured && c->devices.frontend) {
        cc_cfe_result f;
        ret = cc_cfe_dequeue(c->devices.frontend, &f, 0);
        if (ret < 0) return ret;
        if (f.index != r->slot || f.cookie != r->result.sequence) return -EPROTO;
        r->result.raw = f.image; r->result.statistics = f.statistics; r->result.frontend_parameters = f.parameters;
        r->result.frontend_image1 = f.image1;
        r->result.embedded=f.embedded;
    } else if (!r->captured) {
        cc_frame frame;
        ret = cc_camera_dequeue(c->devices.camera, &frame, 0);
        if (ret < 0) return ret;
        if (frame.index != r->slot) return -EPROTO;
        r->result.raw = frame;
        ret=cc_camera_embedded_frame(c->devices.camera,r->slot,&r->result.embedded);
        if (ret<0 && ret!=-ENOENT && ret!=-ENODATA) return ret;
    }
    if (!c->policy_seen) { c->policy_first=r->result.raw.sequence;c->policy_seen=true; }
    uint32_t age=r->result.raw.sequence-c->policy_first;
    uint32_t mistrust=c->policy_mode_switch ? c->policy.mistrust_mode_switch:c->policy.mistrust_startup;
    uint32_t hide=c->policy_mode_switch ? c->policy.hide_mode_switch:c->policy.hide_startup;
    r->result.untrusted=age<mistrust;
    if (age<hide && !(r->result.raw.flags & V4L2_BUF_FLAG_ERROR)) {
        int released=c->devices.frontend ? cc_cfe_release(c->devices.frontend,r->slot):0;
        if (released<0) return released;
        r->result.raw=(cc_frame){0};r->result.statistics=(cc_frame){0};
        r->result.embedded=(cc_frame){0};r->result.frontend_parameters=(cc_frame){0};
        r->result.frontend_image1=(cc_frame){0};return queue_source(c,r);
    }
    r->captured = true;
    if (c->delayed) {
        ret = cc_delayed_get(c->delayed, r->result.raw.sequence, &r->result.sensor);
        if (ret < 0) return ret;
    }
    uint32_t source_flags = r->result.raw.flags | r->result.statistics.flags | r->result.frontend_parameters.flags |
        r->result.embedded.flags | r->result.frontend_image1.flags;
    if (effective(r)->num_sensor_controls && !(source_flags & V4L2_BUF_FLAG_ERROR)) {
        if (r->result.raw.sequence - r->sensor_target > INT32_MAX) {
            /* This frame precedes the controls' effective frame. Recycle only
             * this request's source slot; it has never been exposed or imported
             * by a processor. CFE image/statistics/configuration retire together. */
            if (c->devices.frontend) {
                ret = cc_cfe_release(c->devices.frontend, r->slot);
                if (ret < 0) return ret;
            }
            r->captured = false;
            r->result.raw = (cc_frame) {0}; r->result.statistics = (cc_frame) {0};
            r->result.frontend_parameters = (cc_frame) {0}; r->result.sensor = (cc_sensor_controls) {0};
            r->result.embedded=(cc_frame){0};
            r->result.frontend_image1=(cc_frame){0};
            return queue_source(c, r);
        }
        if (r->result.sensor.cookie != r->cookie) return -EPROTO;
    }
    c->capturing = NULL;
    if ((!c->devices.vc4 && !c->devices.backend) || source_flags & V4L2_BUF_FLAG_ERROR) return finish(c, r);
    else r->phase = READY;
    return 0;
}
static int processed(cc_capture *c, cc_request *r)
{
    if (c->devices.backend) {
        cc_pisp_be_result b;
        int ret = cc_pisp_be_dequeue(c->devices.backend, &b, 0);
        if (ret < 0) return ret;
        if (b.index != r->slot || b.cookie != r->result.sequence) return -EPROTO;
        r->result.output[0] = b.output; r->result.output[1] = b.output1;
        r->result.backend_parameters = b.parameters; r->result.raw.flags |= b.input.flags & V4L2_BUF_FLAG_ERROR;
        memcpy(r->result.backend_auxiliary,b.auxiliary,sizeof(b.auxiliary));
    } else {
        cc_vc4_result v;
        int ret = cc_vc4_isp_dequeue(c->devices.vc4, &v, 0);
        if (ret < 0) return ret;
        if (v.index != r->slot || v.cookie != r->result.sequence) return -EPROTO;
        r->result.output[0] = v.output[0]; r->result.output[1] = v.output[1];
        r->result.statistics = v.statistics; r->result.raw.flags |= v.input.flags & V4L2_BUF_FLAG_ERROR;
    }
    r->processed = true; c->processing = NULL;
    return finish(c, r);
}
static int submit_processor(cc_capture *c, cc_request *r)
{
    cc_request *settings = effective(r);
    cc_dmabuf_plane planes[CC_MAX_PLANES];
    uint32_t n = 0;
    cc_pisp_be_aux_input auxiliary[2] = {0};
    int auxiliary_fds[2] = {-1,-1};
    int ret = 0;
    if (settings->have_vc4) {
        ret=cc_vc4_isp_set_tuning(c->devices.vc4,&settings->vc4);
        if (ret<0) return ret;
    }
    if (settings->num_controls) {
        ret = cc_vc4_isp_set_controls(c->devices.vc4, settings->controls, settings->num_controls, NULL);
        if (ret < 0) return ret;
    }
    for (uint32_t stream = CC_CAPTURE_OUTPUT0; stream <= CC_CAPTURE_OUTPUT1; ++stream)
        if (c->devices.request_buffers & (1U << stream)) {
            const struct cc_request_buffer *buffer = &r->buffers[stream];
            ret = c->devices.backend ? cc_pisp_be_import_output(c->devices.backend, stream - 1, r->slot, buffer->planes, buffer->count) :
                cc_vc4_isp_import_output(c->devices.vc4, stream - 1, r->slot, buffer->planes, buffer->count);
            if (ret < 0) return ret;
        }
    for (; n < r->result.raw.num_planes; ++n) {
        if (r->result.raw.planes[n].length > UINT32_MAX) { ret = -ERANGE; break; }
        int fd = c->devices.frontend ? cc_cfe_export_plane(c->devices.frontend, CC_CFE_IMAGE, r->slot, n) :
                                      cc_camera_export_plane(c->devices.camera, r->slot, n);
        if (fd < 0) { ret = fd; break; }
        planes[n] = (cc_dmabuf_plane) {fd, (uint32_t)r->result.raw.planes[n].length};
    }
    for (uint32_t i = 0; !ret && i < 2; ++i) {
        cc_request *source = r->history[i].source;
        if (!source) continue;
        enum cc_capture_stream stream = r->history[i].stream;
        const cc_frame *frame = history_frame(source,stream);
        if (frame->num_planes != 1 || frame->planes[0].length > UINT32_MAX) { ret = -EPIPE; break; }
        int fd;
        if (stream == CC_CAPTURE_RAW)
            fd = c->devices.frontend ? cc_cfe_export_plane(c->devices.frontend,CC_CFE_IMAGE,source->slot,0) :
                cc_camera_export_plane(c->devices.camera,source->slot,0);
        else if (stream == CC_CAPTURE_FRONTEND_IMAGE1)
            fd = cc_cfe_export_plane(c->devices.frontend,CC_CFE_IMAGE1,source->slot,0);
        else fd = cc_pisp_be_export_port_plane(c->devices.backend,
            stream == CC_CAPTURE_TDN_OUTPUT ? CC_PISP_BE_TDN_OUTPUT : CC_PISP_BE_STITCH_OUTPUT,source->slot,0);
        if (fd < 0) { ret = fd; break; }
        auxiliary_fds[i] = fd;
        auxiliary[i].plane = (cc_dmabuf_plane) {fd,(uint32_t)frame->planes[0].length};
        auxiliary[i].frame = *frame;
    }
    if (!ret) ret = c->devices.backend ?
        cc_pisp_be_submit_with_auxiliary(c->devices.backend, r->slot, planes, n, &r->result.raw,auxiliary,
                          settings->have_be ? &settings->be : NULL, r->result.sequence) :
        cc_vc4_isp_submit(c->devices.vc4, r->slot, planes, n, &r->result.raw, r->result.sequence);
    for (uint32_t i = 0; i < n; ++i) c->io.ops->close(c->io.ctx, planes[i].fd);
    for (uint32_t i = 0; i < 2; ++i) if (auxiliary_fds[i] >= 0) c->io.ops->close(c->io.ctx,auxiliary_fds[i]);
    if (!ret) { r->phase = PROCESSING; c->processing = r; }
    return ret;
}

int cc_capture_dispatch(cc_capture *c)
{
    if (!c) return -EINVAL;
    if (c->error || c->dead) return c->error ? c->error : -ENODEV;
    bool progress;
    do {
        progress = false;
        int prepared = prepare_fences(c);
        if (prepared < 0) return halt(c, c->capturing ? c->capturing : c->processing, prepared);
        int event_ret = sensor_events(c);
        if (event_ret < 0) return halt(c, c->capturing ? c->capturing : c->processing, event_ret);
        if (c->processing) {
            cc_request *r = c->processing;
            int ret = processed(c, r);
            if (ret != -EAGAIN) { if (ret < 0) return halt(c, r, ret); progress = true; }
        }
        if (c->capturing) {
            cc_request *r = c->capturing;
            int ret = captured(c, r);
            if (ret != -EAGAIN) { if (ret < 0) return halt(c, r, ret); progress = true; }
        }
        if (!c->processing) {
            for (cc_request *r = c->head; r; r = r->next) if (r->phase == READY) {
                int ret = submit_processor(c, r);
                if (ret < 0) return halt(c, r, ret);
                progress = true; break;
            }
        }
        if (!c->capturing) {
            uint32_t slot = 0;
            while (slot < c->count && c->slots[slot]) ++slot;
            if (slot < c->count) for (cc_request *r = c->head; r; r = r->next) if (r->phase == WAITING) {
                if (fence_mask(r)) break;
                r->slot = slot; c->slots[slot] = r;
                int ret = prepare_ipa(c, r);
                if (ret < 0) return halt(c, r, ret);
                if (c->delayed) {
                    ret = subscribe(c, true);
                    if (ret < 0) return halt(c, r, ret);
                }
                cc_request *settings = effective(r);
                if (settings->num_sensor_controls) {
                    ret = cc_delayed_schedule_next(c->delayed, settings->sensor_controls, settings->num_sensor_controls,
                                                   r->cookie, &r->sensor_target);
                    if (ret < 0) return halt(c, r, ret);
                }
                if (settings->lens_write) {
                    ret=apply_lens(c,&settings->af_status.lens_setting);
                    if (ret<0) return halt(c,r,ret);
                    c->lens_current=settings->af_status.lens_setting; c->lens_current_valid=true;
                } else if (settings->af_status.flags & CC_AF_LENS_VALID) {
                    if (!c->lens_current_valid) {
                        ret=cc_capture_get_lens_position(c,&c->lens_current);
                        if (ret<0) return halt(c,r,ret);
                        c->lens_current_valid=true;
                    }
                    settings->af_status.lens_setting=c->lens_current;
                }
                if (r->vimc_controls.flags) {
                    cc_vimc_controls acknowledged = r->vimc_controls;
                    ret = cc_camera_set_vimc_controls(c->devices.camera, &acknowledged);
                    if (ret < 0) return halt(c, r, ret);
                    r->result.vimc = acknowledged;
                }
                if (r->uvc_controls.flags) {
                    cc_uvc_controls acknowledged=r->uvc_controls;
                    ret=cc_camera_set_uvc_controls(c->devices.camera,&acknowledged);
                    if (ret<0) return halt(c,r,ret);
                    r->result.uvc=acknowledged;
                }
                if (!c->started) { c->policy_mode_switch=c->ever_started;c->ever_started=true; }
                c->started = true;
                ret = queue_source(c, r);
                if (ret < 0) return halt(c, r, ret);
                r->phase = CAPTURING; c->capturing = r; progress = true; break;
            }
        }
    } while (progress);
    return 0;
}

int cc_capture_pollfds(const cc_capture *c, struct pollfd *fds, size_t *count)
{
    if (!c || !count) return -EINVAL;
    struct pollfd pending[CC_CAPTURE_MAX_FDS];
    size_t n = 0;
    if (!c->dead) {
        if (c->devices.frontend) n += cc_cfe_pending_fds(c->devices.frontend, pending + n);
        else if (c->capturing && !c->capturing->captured)
            n+=cc_camera_pending_fds(c->devices.camera,pending+n);
        if (c->devices.backend) n += cc_be_pending_fds(c->devices.backend, pending + n);
        if (c->devices.vc4) n += cc_vc4_pending_fds(c->devices.vc4, pending + n);
        for (const cc_request *r = c->head; r; r = r->next) if (r->phase == WAITING)
            for (size_t i = 0; i < 3; ++i) if (r->fences[i].valid)
                pending[n++] = (struct pollfd){r->fences[i].fd, POLLIN, 0};
        if (c->subscribed && c->started) {
            int fd = source_fd(c);
            size_t i = 0;
            while (i < n && pending[i].fd != fd) ++i;
            if (i < n) pending[i].events |= POLLPRI;
            else pending[n++] = (struct pollfd) {fd, POLLPRI, 0};
        }
    }
    size_t capacity = *count;
    *count = n;
    if (fds && capacity < n) return -ENOSPC;
    if (fds) memcpy(fds, pending, n * sizeof(*fds));
    return 0;
}

int cc_capture_get_timeout(const cc_capture *c, int *timeout_ms)
{
    if (!c || !timeout_ms) return -EINVAL;
    uint64_t deadline = UINT64_MAX; bool found = false;
    if (!c->dead) for (const cc_request *r = c->head; r; r = r->next)
        if (r->phase == WAITING && r->fence_deadline && (!found || r->fence_deadline < deadline)) {
            deadline = r->fence_deadline; found = true;
        }
    int timeout = -1;
    if (found) {
        uint64_t now;
        int ret = c->io.ops->now(c->io.ctx, &now);
        if (ret < 0) return ret;
        uint64_t ns = deadline > now ? deadline - now : 0;
        uint64_t ms = ns / UINT64_C(1000000) + (ns % UINT64_C(1000000) != 0);
        timeout = ms > INT_MAX ? INT_MAX : (int)ms;
    }
    *timeout_ms = timeout; return 0;
}

int cc_capture_dequeue(cc_capture *c, cc_request **out, int timeout_ms)
{
    if (!c || !out || timeout_ms < -1) return -EINVAL;
    uint64_t deadline = 0;
    if (timeout_ms > 0) {
        int ret = c->io.ops->now(c->io.ctx, &deadline);
        if (ret < 0) halt(c, c->processing ? c->processing : c->capturing, ret);
        else {
            uint64_t delta = (uint64_t)timeout_ms * UINT64_C(1000000);
            deadline = deadline > UINT64_MAX - delta ? UINT64_MAX : deadline + delta;
        }
    }
    for (;;) {
        int ret = cc_capture_dispatch(c);
        if (c->done_head) {
            cc_request *r = c->done_head;
            c->done_head = r->done_next;
            if (!c->done_head) c->done_tail = NULL;
            r->done_next = NULL; r->enqueued = false; r->delivered = true; *out = r;
            return 0;
        }
        if (ret < 0) return ret;
        if (!timeout_ms) return -EAGAIN;
        struct pollfd fds[CC_CAPTURE_MAX_FDS];
        size_t n = CC_CAPTURE_MAX_FDS;
        ret = cc_capture_pollfds(c, fds, &n);
        if (ret < 0) { halt(c, NULL, ret); continue; }
        if (!n) return -EAGAIN;
        int remaining = -1;
        if (timeout_ms > 0) {
            uint64_t now;
            ret = c->io.ops->now(c->io.ctx, &now);
            if (ret < 0) { halt(c, c->processing ? c->processing : c->capturing, ret); continue; }
            if (now >= deadline) return -ETIMEDOUT;
            uint64_t ns = deadline - now, ms = ns / UINT64_C(1000000) + (ns % UINT64_C(1000000) != 0);
            remaining = ms > INT_MAX ? INT_MAX : (int)ms;
        }
        int fence_timeout;
        ret = cc_capture_get_timeout(c, &fence_timeout);
        if (ret < 0) { halt(c, NULL, ret); continue; }
        if (fence_timeout >= 0 && (remaining < 0 || fence_timeout < remaining)) remaining = fence_timeout;
        ret = c->io.ops->wait_many ? c->io.ops->wait_many(c->io.ctx, fds, n, remaining) : -ENOTSUP;
        if (ret == -ETIMEDOUT) continue;
        if (ret < 0 && ret != -EINTR && ret != -EAGAIN)
            halt(c, c->processing ? c->processing : c->capturing, ret);
    }
}

int cc_capture_release(cc_capture *c, cc_request *r)
{
    if (!c || !r || r->owner != c || !r->delivered) return -EINVAL;
    if (r->history_users) return -EBUSY;
    if (r->slot != UINT32_MAX) {
        int ret = 0;
        if (r->processed) ret = c->devices.backend ? cc_pisp_be_release(c->devices.backend, r->slot) :
                                                   cc_vc4_isp_release(c->devices.vc4, r->slot);
        if (!ret && r->captured && c->devices.frontend) ret = cc_cfe_release(c->devices.frontend, r->slot);
        if (!ret && r->captured && c->devices.camera && c->devices.camera->embedded)
            ret=cc_embedded_release(c->devices.camera->embedded,r->slot);
        if (!ret && r->captured && c->devices.camera && (c->devices.request_buffers & CC_CAPTURE_REQUEST_RAW))
            ret = cc_stream_unimport(c->images[0], r->slot);
        if (ret < 0) return halt(c, r, ret);
        c->slots[r->slot] = NULL;
    }
    cc_request **link = &c->head, *previous = NULL;
    while (*link != r) { previous = *link; link = &(*link)->next; }
    *link = r->next;
    if (c->tail == r) c->tail = previous;
    --c->owned; r->owner = NULL; r->next = NULL; r->slot = UINT32_MAX;
    cc_request_destroy(r->work); r->work = NULL;
    r->result = (cc_capture_result) { .cookie = r->cookie };
    return 0;
}

int cc_capture_export_plane(cc_capture *c, const cc_request *r, enum cc_capture_stream stream, uint32_t plane)
{
    if (!c || !r || r->owner != c || !r->delivered) return -EINVAL;
    if (r->slot == UINT32_MAX || c->dead) return -ECANCELED;
    if (stream == CC_CAPTURE_RAW)
        return c->devices.frontend ? cc_cfe_export_plane(c->devices.frontend, CC_CFE_IMAGE, r->slot, plane) :
                                    cc_camera_export_plane(c->devices.camera, r->slot, plane);
    if (stream == CC_CAPTURE_FRONTEND_IMAGE1) {
        if (!c->devices.frontend || !r->result.frontend_image1.num_planes) return -ENODATA;
        return cc_cfe_export_plane(c->devices.frontend, CC_CFE_IMAGE1, r->slot, plane);
    }
    if (stream == CC_CAPTURE_TDN_OUTPUT || stream == CC_CAPTURE_STITCH_OUTPUT) {
        if (!c->devices.backend || !history_frame(r,stream)->num_planes) return -ENODATA;
        return cc_pisp_be_export_port_plane(c->devices.backend,
            stream == CC_CAPTURE_TDN_OUTPUT ? CC_PISP_BE_TDN_OUTPUT : CC_PISP_BE_STITCH_OUTPUT,r->slot,plane);
    }
    if (stream==CC_CAPTURE_EMBEDDED) {
        if (!r->result.embedded.num_planes) return -ENODATA;
        return c->devices.frontend ? cc_cfe_export_plane(c->devices.frontend,CC_CFE_EMBEDDED,r->slot,plane):
            cc_camera_export_embedded_plane(c->devices.camera,r->slot,plane);
    }
    if (stream == CC_CAPTURE_STATISTICS) {
        if (c->devices.frontend) return cc_cfe_export_plane(c->devices.frontend, CC_CFE_STATISTICS, r->slot, plane);
        if (r->processed && c->devices.vc4) return cc_vc4_isp_export_plane(c->devices.vc4, CC_VC4_STATS, r->slot, plane);
    }
    if (stream == CC_CAPTURE_OUTPUT0 || stream == CC_CAPTURE_OUTPUT1) {
        if (!r->processed) return -ENOENT;
        uint32_t output = stream == CC_CAPTURE_OUTPUT1 ? 1U : 0U;
        return c->devices.backend ? cc_pisp_be_export_output_plane(c->devices.backend, output, r->slot, plane) :
            cc_vc4_isp_export_plane(c->devices.vc4, output ? CC_VC4_LORES : CC_VC4_MAIN, r->slot, plane);
    }
    return -ENOTSUP;
}
