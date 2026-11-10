/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "pisp_format.h"
#include "embedded_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>

#define CC_CFE_MAX_BUFFERS 256U
#define CC_FE_CONFIG_FORMAT CC_FOURCC('R', 'P', 'F', 'C')
#define CC_FE_STATS_FORMAT CC_FOURCC('R', 'P', 'F', 'S')

struct cc_cfe {
    cc_camera *camera; /* Owns media lock, sensor, mux/CSI and restoration log. */
    cc_video_device *devices[4];
    cc_stream *streams[4];
    int fe_fd;
    uint32_t fe_entity, image_entity, stats_entity, config_entity, image1_entity;
    bool configured, fatal;
    cc_cfe_configuration configuration;
    uint32_t count, done;
    int active;
    unsigned char states[CC_CFE_MAX_BUFFERS];
    cc_cfe_result pending;
};

static int usable(const cc_cfe *cfe) { return !cfe ? -EINVAL : cfe->fatal ? -ENODEV : 0; }
static uint32_t ports(const cc_cfe *cfe) { return cfe->configuration.image1.fourcc ? 15U : 7U; }
cc_stream *cc_cfe_image_stream(cc_cfe *cfe) { return cfe ? cfe->streams[CC_CFE_IMAGE] : NULL; }
cc_stream *cc_cfe_aux_stream(cc_cfe *cfe, uint32_t index)
{
    if (!cfe || index > 3) return NULL;
    if (index == 3) return cfe->configuration.image1.fourcc ? cfe->streams[3] : NULL;
    return index < 2 ? cfe->streams[index + 1] : cfe->camera->embedded ? cfe->camera->embedded->stream : NULL;
}
static bool allocated(const cc_cfe *cfe)
{
    for (size_t i = 0; i < 4; ++i)
        if (cfe->streams[i] && cfe->streams[i]->allocated)
            return true;
    return false;
}
cc_camera *cc_cfe_source(cc_cfe *cfe) { return cfe && !cfe->fatal ? cfe->camera:NULL; }
int cc_cfe_configuration_ready(cc_cfe *cfe)
{
    int ret=usable(cfe); return ret<0 ? ret:allocated(cfe) ? -EBUSY:0;
}
void cc_cfe_invalidate_configuration(cc_cfe *cfe)
{
    if (cfe) { cfe->configured=false; cfe->camera->stream->configured=false; }
}
int cc_cfe_get_configuration(cc_cfe *cfe, cc_cfe_configuration *out)
{
    int ret=usable(cfe); if (ret<0) return ret;
    if (!out) return -EINVAL;
    if (!cfe->configured || !cfe->camera->stream->configured) return -EPIPE;
    *out=cfe->configuration; return 0;
}

int cc_cfe_capture_info(cc_cfe *cfe, const struct cc_io **io, cc_cfe_configuration *configuration, uint32_t *count)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    if (!cfe->configured || !cfe->count) return -EPIPE;
    if (cfe->active >= 0) return -EBUSY;
    for (uint32_t i = 0; i < cfe->count; ++i) if (cfe->states[i]) return -EBUSY;
    *io = &cfe->camera->io; *configuration = cfe->configuration; *count = cfe->count;
    return 0;
}

size_t cc_cfe_pending_fds(const cc_cfe *cfe, struct pollfd *fds)
{
    size_t count = 0;
    if (cfe->active >= 0)
        for (size_t i = 0; i < 4; ++i)
            if ((ports(cfe) & (1U << i)) && !(cfe->done & (1U << i)))
                fds[count++] = (struct pollfd) {cc_stream_fd(cfe->streams[i]), i == 2 ? POLLOUT : POLLIN, 0};
    if (cfe->active>=0 && cfe->camera->embedded && (cfe->done&1U) && !(cfe->done&16U))
        fds[count++]=(struct pollfd){cc_camera_embedded_fd(cfe->camera),POLLIN,0};
    return count;
}
static struct cc_entity *named(struct cc_media *media, const char *name)
{
    for (size_t i = 0; i < media->num_entities; ++i)
        if (!strcmp(cc_entity_name(&media->entities[i]), name))
            return &media->entities[i];
    return NULL;
}

static int open_image1(cc_cfe *cfe)
{
    if (cfe->streams[3]) return 0;
    struct cc_entity *e = named(cfe->camera->media, "rp1-cfe-fe-image1");
    if (!e || !e->path[0]) return -ENODEV;
    cc_video_device *device;
    int ret = cc_video_open_io(e->path, &cfe->camera->io, &device);
    if (ret < 0) return ret;
    cc_stream *stream;
    ret = cc_video_create_stream(device, device->capabilities & V4L2_CAP_VIDEO_CAPTURE ?
        CC_STREAM_CAPTURE : CC_STREAM_CAPTURE_MPLANE, &stream);
    if (ret < 0) { cc_video_close(device); return ret; }
    cfe->devices[3] = device; cfe->streams[3] = stream; cfe->image1_entity = e->desc.id;
    return 0;
}

int cc_cfe_open_io(const cc_camera_info *info, const struct cc_io *io, cc_cfe **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!info || !io)
        return -EINVAL;
    if (info->pipeline != CC_PIPELINE_RPI_CFE)
        return -ENOTSUP;
    cc_cfe *cfe = calloc(1, sizeof(*cfe));
    if (!cfe)
        return -ENOMEM;
    cfe->fe_fd = cfe->active = -1;
    int ret = cc_camera_open_io(info, io, &cfe->camera);
    if (ret < 0)
        goto fail;
    struct cc_media *media = cfe->camera->media;
    struct cc_entity *fe = named(media, "pisp-fe");
    if (!fe || !fe->path[0]) { ret = -ENODEV; goto fail; }
    cfe->fe_entity = fe->desc.id;
    cfe->fe_fd = io->ops->open(io->ctx, fe->path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (cfe->fe_fd < 0) { ret = cfe->fe_fd; goto fail; }
    ret = io->ops->lock(io->ctx, cfe->fe_fd);
    if (ret < 0)
        goto fail;
    static const char *const names[] = { "rp1-cfe-fe-image0", "rp1-cfe-fe-stats", "rp1-cfe-fe-config" };
    for (size_t i = 0; i < 3; ++i) {
        struct cc_entity *e = named(media, names[i]);
        if (!e || !e->path[0]) { ret = -ENODEV; goto fail; }
        if (!i) cfe->image_entity = e->desc.id;
        else if (i == 1) cfe->stats_entity = e->desc.id;
        else cfe->config_entity = e->desc.id;
        ret = cc_video_open_io(e->path, io, &cfe->devices[i]);
        if (ret < 0)
            goto fail;
        enum cc_stream_type type = i == 2 ? CC_STREAM_META_OUTPUT : i == 1 ? CC_STREAM_META_CAPTURE :
            cfe->devices[i]->capabilities & V4L2_CAP_VIDEO_CAPTURE ? CC_STREAM_CAPTURE : CC_STREAM_CAPTURE_MPLANE;
        ret = cc_video_create_stream(cfe->devices[i], type, &cfe->streams[i]);
        if (ret < 0)
            goto fail;
    }
    *out = cfe;
    return 0;
fail:
    cc_cfe_close(cfe);
    return ret;
}

int cc_cfe_open(const cc_camera_info *info, cc_cfe **out) { return cc_cfe_open_io(info, &cc_system_io, out); }

int cc_cfe_open_id_io(const char *id, const struct cc_io *io, cc_cfe **out)
{
    if (!out) return -EINVAL;
    *out=NULL;
    cc_camera_info info;
    int ret=cc_camera_find_id_io(id,io,&info);
    return ret<0 ? ret:cc_cfe_open_io(&info,io,out);
}

int cc_cfe_open_by_id(const char *id, cc_cfe **out)
{
    return cc_cfe_open_id_io(id,&cc_system_io,out);
}

const cc_camera_info *cc_cfe_get_info(const cc_cfe *cfe)
{
    return cfe ? cc_camera_get_info(cfe->camera):NULL;
}

static int frontend_links(cc_cfe *cfe, bool second)
{
    cc_camera *camera = cfe->camera;
    struct cc_media *media = camera->media;
    const struct media_link_desc *raw = &media->links[camera->route[camera->route_count - 1]];
    uint32_t csi = raw->source.entity;
    uint16_t pad = raw->source.index;
    uint32_t found = 0;
    /* Some kernels reject simultaneous DMA/FE links on the same CSI output.
     * Disable all unwanted paths before enabling any selected path. */
    for (int phase = 0; phase < 2; ++phase) {
        for (size_t i = 0; i < media->num_links; ++i) {
            const struct media_link_desc *link = &media->links[i];
            bool enable;
            if (link->source.entity == csi) {
                enable = link->source.index == pad && link->sink.entity == cfe->fe_entity && link->sink.index == 0;
                if (enable) found |= 1;
            } else if (link->source.entity == cfe->fe_entity) {
                enable = link->sink.entity == cfe->image_entity || link->sink.entity == cfe->stats_entity ||
                         (second && link->sink.entity == cfe->image1_entity);
                if (link->sink.entity == cfe->image_entity) found |= 2;
                if (link->sink.entity == cfe->stats_entity) found |= 4;
                if (second && link->sink.entity == cfe->image1_entity) found |= 16;
            } else if (link->sink.entity == cfe->fe_entity && link->sink.index == 1) {
                enable = link->source.entity == cfe->config_entity;
                if (enable) found |= 8;
            } else {
                continue;
            }
            if (enable != (phase == 1))
                continue;
            uint32_t flags = enable ? link->flags | MEDIA_LNK_FL_ENABLED : link->flags & ~MEDIA_LNK_FL_ENABLED;
            int ret = cc_camera_change_link(camera, i, flags);
            if (ret < 0)
                return ret;
        }
    }
    return found == (second ? 31U : 15U) ? 0 : -ENODEV;
}

static int set_pad(const struct cc_io *io, int fd, uint32_t pad, const cc_format *image, bool active)
{
    uint32_t code = cc_bayer_mbus(image->fourcc);
    uint32_t which=active ? V4L2_SUBDEV_FORMAT_ACTIVE:V4L2_SUBDEV_FORMAT_TRY;
    struct v4l2_subdev_format format = { .which = which, .pad = pad,
        .format = { .width = image->width, .height = image->height, .code = code,
                    .field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_RAW } };
    int ret = cc_ioctl(io, fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret < 0)
        return ret;
    if (format.which!=which || format.pad!=pad) return -EPROTO;
    format = (struct v4l2_subdev_format) { .which = which, .pad = pad };
    ret = cc_ioctl(io, fd, VIDIOC_SUBDEV_G_FMT, &format);
    if (ret < 0)
        return ret;
    if (format.which!=which || format.pad!=pad) return -EPROTO;
    return format.format.code == code && format.format.width == image->width && format.format.height == image->height ? 0 : -EPIPE;
}

static int negotiate_frontend(cc_cfe *cfe, cc_cfe_configuration *configuration, enum cc_orientation *orientation, bool active)
{
    int ret = usable(cfe);
    if (ret < 0)
        return ret;
    if (!configuration || !configuration->sensor.width || !configuration->sensor.height)
        return -EINVAL;
    if (orientation && ((unsigned)*orientation<1 || (unsigned)*orientation>8)) return -EINVAL;
    if (!cc_cfe_output_fourcc(configuration->sensor.fourcc))
        return -ENOTSUP;
    if (active && allocated(cfe))
        return -EBUSY;
    if (active) cfe->configured = false;
    cc_cfe_configuration c = *configuration;
    enum cc_orientation actual=orientation ? *orientation:CC_ORIENTATION_ROTATE_0;
    ret = orientation ? (active ? cc_camera_configure_oriented(cfe->camera,&c.sensor,&actual):
                                  cc_camera_try_format_oriented(cfe->camera,&c.sensor,&actual)) :
                        active ? cc_camera_configure(cfe->camera, &c.sensor):cc_camera_try_format(cfe->camera,&c.sensor);
    if (ret < 0) return ret;
    if (orientation) {
        enum cc_camera_cfa cfa;
        ret=cc_bayer_code_info(cc_bayer_mbus(c.sensor.fourcc),NULL,&cfa);
        if (ret<0) return ret;
        cc_format *images[]={&c.image,&c.image1};
        for (size_t i=0; i<2; ++i) if (images[i]->fourcc) {
            uint32_t fourcc=cc_bayer_reorder(images[i]->fourcc,cfa);
            if (!fourcc) return -ENOTSUP;
            images[i]->fourcc=fourcc;
        }
    }
    if (!c.image.fourcc) c.image.fourcc = cc_cfe_output_fourcc(c.sensor.fourcc);
    if (c.image1.fourcc) {
        ret = open_image1(cfe);
        if (ret < 0) return ret;
    } else {
        c.image1 = (cc_format) {0}; c.crop[1] = (cc_rectangle) {0};
        c.downscale[1] = (struct pisp_fe_downscale_config) {0}; c.compression_offset1 = 0;
    }
    for (uint32_t i = 0; i < (c.image1.fourcc ? 2U : 1U); ++i) {
        cc_format *image = i ? &c.image1 : &c.image;
        struct cc_pisp_raw_layout raw, expected;
        if (cc_pisp_raw_format(image->fourcc, &raw) < 0 ||
            cc_pisp_raw_format(cc_cfe_output_fourcc(c.sensor.fourcc), &expected) < 0 ||
            raw.bayer_order != expected.bayer_order) return -ENOTSUP;
        struct pisp_fe_output_branch_config channel;
        uint32_t enables = 0;
        ret = cc_cfe_geometry(&c, i, &channel, &enables);
        if (ret < 0) return ret;
        uint32_t width = channel.output.format.width, height = channel.output.format.height;
        if ((image->width && image->width != width) || (image->height && image->height != height))
            return -EINVAL;
        /* The kernel validates video-node dimensions against full-sized FE
         * source pads. Allocate that envelope; native parameters set the actual
         * output size. Its logical layout retains the envelope stride/capacity. */
        cc_format envelope = { .width = c.sensor.width, .height = c.sensor.height,
            .fourcc = image->fourcc, .field = V4L2_FIELD_NONE,
            .colorspace = V4L2_COLORSPACE_RAW, .num_planes = 1 };
        envelope.planes[0].stride = (envelope.width * raw.pixel_bytes + 63U) & ~63U;
        envelope.planes[0].size = envelope.planes[0].stride * envelope.height;
        ret = active ? cc_stream_configure(cfe->streams[i ? 3 : 0], &envelope):
                       cc_stream_try_format(cfe->streams[i ? 3 : 0], &envelope);
        if (ret < 0) return ret;
        if (envelope.width != c.sensor.width || envelope.height != c.sensor.height ||
            envelope.fourcc != image->fourcc) return -EPIPE;
        *image = envelope;
        image->width = width; image->height = height;
        c.downscale[i] = channel.downscale;
    }
    c.statistics = (cc_format) { .fourcc = CC_FE_STATS_FORMAT };
    c.parameters = (cc_format) { .fourcc = CC_FE_CONFIG_FORMAT };
    ret = active ? cc_stream_configure(cfe->streams[1], &c.statistics):cc_stream_try_format(cfe->streams[1], &c.statistics);
    if (ret < 0)
        return ret;
    ret = active ? cc_stream_configure(cfe->streams[2], &c.parameters):cc_stream_try_format(cfe->streams[2], &c.parameters);
    if (ret < 0)
        return ret;
    if (c.statistics.fourcc != CC_FE_STATS_FORMAT || c.parameters.fourcc != CC_FE_CONFIG_FORMAT ||
        c.statistics.planes[0].size < sizeof(struct pisp_statistics) ||
        c.parameters.planes[0].size < sizeof(struct pisp_fe_config))
        return -EPROTO;
    if (active) {
        ret = frontend_links(cfe, c.image1.fourcc != 0);
        if (ret < 0) return ret;
    }
    cc_camera *camera = cfe->camera;
    struct media_link_desc *raw = &camera->media->links[camera->route[camera->route_count - 1]];
    struct cc_entity *csi = cc_media_entity(camera->media, raw->source.entity);
    size_t entity_index = (size_t)(csi - camera->media->entities);
    /* The streaming link always carries 16-bit samples. PiSP compression
     * describes the FE memory writer and shares the uncompressed mbus code. */
    cc_format transport = c.sensor;
    transport.fourcc = cc_cfe_output_fourcc(c.sensor.fourcc);
    ret = set_pad(&camera->io, camera->subdev_fds[entity_index], raw->source.index, &transport,active);
    if (ret < 0)
        return ret;
    ret = set_pad(&camera->io, cfe->fe_fd, 0, &transport,active);
    if (ret < 0)
        return ret;
    ret = set_pad(&camera->io, cfe->fe_fd, 2, &transport,active);
    if (ret < 0)
        return ret;
    if (c.image1.fourcc) {
        ret = set_pad(&camera->io, cfe->fe_fd, 3, &transport,active);
        if (ret < 0) return ret;
    }
    struct pisp_fe_config parameters;
    ret = cc_cfe_build_parameters(&c, NULL, &parameters);
    if (ret < 0)
        return ret;
    if (active) { cfe->configuration = c; cfe->configured = true; }
    *configuration = c;
    if (orientation) *orientation=actual;
    return 0;
}

int cc_cfe_configure(cc_cfe *cfe, cc_cfe_configuration *configuration)
{
    return negotiate_frontend(cfe,configuration,NULL,true);
}

static int try_configuration(cc_cfe *cfe, cc_cfe_configuration *configuration, enum cc_orientation *orientation)
{
    int ret=usable(cfe);
    if (ret<0) return ret;
    bool had_image1=cfe->streams[3]!=NULL;
    uint32_t entity=cfe->image1_entity;
    ret=negotiate_frontend(cfe,configuration,orientation,false);
    if (!had_image1 && cfe->streams[3]) {
        cc_stream_close(cfe->streams[3]); cc_video_close(cfe->devices[3]);
        cfe->streams[3]=NULL; cfe->devices[3]=NULL; cfe->image1_entity=entity;
    }
    return ret;
}
int cc_cfe_try_configuration(cc_cfe *cfe, cc_cfe_configuration *configuration)
{
    return try_configuration(cfe,configuration,NULL);
}
int cc_cfe_try_configuration_oriented(cc_cfe *cfe, cc_cfe_configuration *configuration, enum cc_orientation *orientation)
{
    if (!orientation) return -EINVAL;
    return try_configuration(cfe,configuration,orientation);
}

int cc_cfe_configure_oriented(cc_cfe *cfe, cc_cfe_configuration *configuration, enum cc_orientation *orientation)
{
    if (!orientation) return -EINVAL;
    return negotiate_frontend(cfe,configuration,orientation,true);
}

int cc_cfe_stop(cc_cfe *cfe)
{
    if (!cfe)
        return -EINVAL;
    int first = cfe->fatal ? -ENODEV : 0;
    for (size_t i = 0; i < 4; ++i) {
        if (!cfe->streams[i]) continue;
        int ret = cc_stream_stop(cfe->streams[i]);
        if (ret < 0 && !first) first = ret;
    }
    if (cfe->camera && cfe->camera->embedded) {
        int ret=cc_embedded_stop(cfe->camera->embedded);
        if (ret<0 && !first) first=ret;
    }
    for (size_t i = 0; i < 4; i += 3) {
        if (!cfe->streams[i]) continue;
        int ret = cc_stream_clear_imports(cfe->streams[i]);
        if (ret < 0 && !first) first = ret;
    }
    if (first) {
        /* The FE shares sensor/CSI state across all enabled video nodes. */
        for (size_t i = 0; i < 4; ++i) {
            cc_video_device *d = cfe->devices[i];
            if (d && d->fd >= 0) {
                d->io.ops->close(d->io.ctx, d->fd);
                d->fd = -1;
            }
        }
        cfe->fatal = true;
        cfe->configured = false;
        if (cfe->camera && cfe->camera->embedded) {
            cc_video_device *d=cfe->camera->embedded->device;
            if (d->fd>=0) { d->io.ops->close(d->io.ctx,d->fd); d->fd=-1; }
        }
        for (size_t i = 0; i < 4; i += 3)
            if (cfe->streams[i] && cfe->streams[i]->memory == V4L2_MEMORY_DMABUF)
                (void)cc_stream_free_buffers(cfe->streams[i]);
    }
    cfe->active = -1;
    cfe->done = 0;
    memset(cfe->states, 0, sizeof(cfe->states));
    return first;
}

int cc_cfe_free_buffers(cc_cfe *cfe)
{
    if (!cfe)
        return -EINVAL;
    int ret = cc_cfe_stop(cfe);
    for (size_t i = 0; i < 4; ++i) {
        if (!cfe->streams[i]) continue;
        int freed = cc_stream_free_buffers(cfe->streams[i]);
        if (freed < 0 && !ret) ret = freed;
    }
    if (cfe->camera && cfe->camera->embedded) {
        int freed=cc_embedded_free(cfe->camera->embedded);
        if (freed<0 && !ret) ret=freed;
    }
    cfe->count = 0;
    return ret;
}

void cc_cfe_close(cc_cfe *cfe)
{
    if (!cfe)
        return;
    cc_cfe_stop(cfe);
    for (size_t i = 0; i < 4; ++i) {
        cc_stream_close(cfe->streams[i]);
        cc_video_close(cfe->devices[i]);
    }
    if (cfe->fe_fd >= 0)
        cfe->camera->io.ops->close(cfe->camera->io.ctx, cfe->fe_fd);
    cc_camera_close(cfe->camera);
    free(cfe);
}

int cc_cfe_allocate_outputs(cc_cfe *cfe, const enum cc_memory images[2], uint32_t *count)
{
    int ret = usable(cfe);
    if (ret < 0)
        return ret;
    if (!count || !*count || *count > CC_CFE_MAX_BUFFERS || !images)
        return -EINVAL;
    for (uint32_t i = 0; i < 2; ++i)
        if (images[i] != CC_MEMORY_MMAP && images[i] != CC_MEMORY_DMABUF) return -EINVAL;
    if (allocated(cfe))
        return -EBUSY;
    if (!cfe->configured || !cfe->camera->stream->configured)
        return -EPIPE;
    uint32_t minimum = UINT32_MAX;
    for (size_t i = 0; i < 4; ++i) {
        if (!(ports(cfe) & (1U << i))) continue;
        uint32_t n = *count;
        ret = cc_stream_allocate_buffers(cfe->streams[i], i == 0 ? images[0] : i == 3 ? images[1] : CC_MEMORY_MMAP, &n);
        if (ret < 0) {
            cc_cfe_free_buffers(cfe);
            return ret;
        }
        if (n < minimum) minimum = n;
    }
    if (cfe->camera->embedded) {
        ret=cc_embedded_allocate(cfe->camera->embedded,&minimum);
        if (ret<0) { (void)cc_cfe_free_buffers(cfe); return ret; }
    }
    *count = cfe->count = minimum;
    return 0;
}

int cc_cfe_allocate_buffers_with_memory(cc_cfe *cfe, enum cc_memory image, uint32_t *count)
{
    const enum cc_memory images[2] = {image, CC_MEMORY_MMAP};
    return cc_cfe_allocate_outputs(cfe, images, count);
}

int cc_cfe_allocate_buffers(cc_cfe *cfe, uint32_t *count)
{
    return cc_cfe_allocate_buffers_with_memory(cfe, CC_MEMORY_MMAP, count);
}

int cc_cfe_import_buffer(cc_cfe *cfe, uint32_t index, const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    return cc_cfe_import_output(cfe, 0, index, planes, num_planes);
}

int cc_cfe_import_output(cc_cfe *cfe, uint32_t output, uint32_t index,
                         const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    if (output > 1 || index >= cfe->count) return -EINVAL;
    if (output && !cfe->configuration.image1.fourcc) return -ENOENT;
    if (cfe->states[index]) return -EBUSY;
    return cc_stream_import(cfe->streams[output ? 3 : 0], index, planes, num_planes);
}

static int cancel_error(cc_cfe *cfe, int error)
{
    int stopped = cc_cfe_stop(cfe);
    return stopped < 0 ? stopped : error;
}

int cc_cfe_queue(cc_cfe *cfe, uint32_t index, const struct pisp_fe_config *tuning, uint64_t cookie)
{
    int ret = usable(cfe);
    if (ret < 0)
        return ret;
    if (!cfe->configured || !cfe->count || !cfe->camera->stream->configured)
        return -EPIPE;
    if (index >= cfe->count)
        return -EINVAL;
    if (cfe->active >= 0 || cfe->states[index])
        return -EBUSY;
    if (!cfe->streams[0]->buffers[index].bound) return -EPIPE;
    if (cfe->configuration.image1.fourcc && !cfe->streams[3]->buffers[index].bound) return -EPIPE;
    struct pisp_fe_config config;
    ret = cc_cfe_build_parameters(&cfe->configuration, tuning, &config);
    if (ret < 0)
        return ret;
    cc_frame parameters;
    ret = cc_stream_buffer(cfe->streams[2], index, &parameters);
    if (ret < 0)
        return ret;
    memcpy(parameters.planes[0].data, &config, sizeof(config));
    parameters.planes[0].bytes_used = sizeof(config);
    parameters.planes[0].data_offset = 0;
    parameters.timestamp_ns = 0;
    parameters.field = V4L2_FIELD_NONE;
    if (cfe->camera->embedded) {
        ret=cc_embedded_start(cfe->camera->embedded);
        if (ret<0) return cancel_error(cfe,ret);
    }
    static const size_t order[4] = {0, 3, 1, 2};
    for (size_t n = 0; n < 4; ++n) {
        size_t i = order[n];
        if (!(ports(cfe) & (1U << i))) continue;
        ret = cc_stream_queue(cfe->streams[i], index, i == 2 ? &parameters : NULL);
        if (ret < 0)
            return cancel_error(cfe, ret);
    }
    for (size_t n = 0; n < 4; ++n) {
        size_t i = order[n];
        if (!(ports(cfe) & (1U << i))) continue;
        if (cfe->streams[i]->streaming) continue;
        ret = cc_stream_start(cfe->streams[i]);
        if (ret < 0)
            return cancel_error(cfe, ret);
    }
    cfe->active = (int)index;
    cfe->states[index] = 1;
    cfe->done = 0;
    cfe->pending = (cc_cfe_result) { .index = index, .cookie = cookie };
    return 0;
}

static int accept(cc_cfe *cfe, size_t port, const cc_frame *frame)
{
    if (frame->index != (uint32_t)cfe->active)
        return cancel_error(cfe, -EPROTO);
    if (!port) cfe->pending.image = *frame;
    else if (port == 1) cfe->pending.statistics = *frame;
    else if (port == 2) cfe->pending.parameters = *frame;
    else cfe->pending.image1 = *frame;
    cfe->done |= 1U << port;
    return 0;
}

int cc_cfe_dequeue(cc_cfe *cfe, cc_cfe_result *result, int timeout_ms)
{
    int ret = usable(cfe);
    if (ret < 0)
        return ret;
    if (!result || timeout_ms < -1)
        return -EINVAL;
    if (cfe->active < 0)
        return -EAGAIN;
    const struct cc_io *io = &cfe->camera->io;
    uint64_t deadline = 0;
    if (timeout_ms > 0) {
        ret = io->ops->now(io->ctx, &deadline);
        if (ret < 0) return cancel_error(cfe, ret);
        uint64_t delta = (uint64_t)timeout_ms * UINT64_C(1000000);
        deadline = deadline > UINT64_MAX - delta ? UINT64_MAX : deadline + delta;
    }
    for (;;) {
        for (size_t i = 0; i < 4; ++i) {
            if (!(ports(cfe) & (1U << i))) continue;
            if (cfe->done & (1U << i)) continue;
            cc_frame frame;
            ret = cc_stream_dequeue(cfe->streams[i], &frame, 0);
            if (ret == -EAGAIN) continue;
            if (ret < 0) return cancel_error(cfe, ret);
            ret = accept(cfe, i, &frame);
            if (ret < 0) return ret;
        }
        if (cfe->camera->embedded && (cfe->done&1U) && !(cfe->done&16U)) {
            ret=cc_embedded_collect(cfe->camera->embedded,&cfe->pending.image,&cfe->pending.embedded);
            if (!ret) cfe->done|=16U;
            else if (ret!=-EAGAIN) return cancel_error(cfe,ret);
        }
        if (cfe->done == (ports(cfe) | (cfe->camera->embedded ? 16U : 0U))) {
            if (cfe->pending.image.sequence != cfe->pending.statistics.sequence ||
                cfe->pending.image.timestamp_ns != cfe->pending.statistics.timestamp_ns)
                return cancel_error(cfe, -EPROTO);
            if (cfe->configuration.image1.fourcc &&
                (cfe->pending.image.sequence != cfe->pending.image1.sequence ||
                 cfe->pending.image.timestamp_ns != cfe->pending.image1.timestamp_ns))
                return cancel_error(cfe, -EPROTO);
            cfe->states[cfe->active] = 2;
            cfe->active = -1;
            *result = cfe->pending;
            return 0;
        }
        if (!timeout_ms) return -EAGAIN;
        int remaining = -1;
        if (timeout_ms > 0) {
            uint64_t now;
            ret = io->ops->now(io->ctx, &now);
            if (ret < 0) return cancel_error(cfe, ret);
            if (now >= deadline) return -ETIMEDOUT;
            uint64_t ns = deadline - now;
            uint64_t ms = ns / UINT64_C(1000000) + (ns % UINT64_C(1000000) != 0);
            remaining = ms > INT_MAX ? INT_MAX : (int)ms;
        }
        struct pollfd fds[5]; size_t count=cc_cfe_pending_fds(cfe,fds);
        if (!count) return cancel_error(cfe,-EPROTO);
        ret=io->ops->wait_many ? io->ops->wait_many(io->ctx,fds,count,remaining):
            io->ops->wait(io->ctx,fds[0].fd,fds[0].events,remaining);
        if (ret==-ETIMEDOUT) return ret;
        if (ret<0 && ret!=-EAGAIN && ret!=-EINTR) return cancel_error(cfe,ret);
    }
}

int cc_cfe_release(cc_cfe *cfe, uint32_t index)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    if (index >= cfe->count || cfe->states[index] != 2) return -EINVAL;
    if (cfe->camera->embedded) {
        ret=cc_embedded_release(cfe->camera->embedded,index);
        if (ret<0) return cancel_error(cfe,ret);
    }
    for (size_t i = 0; i < 4; i += 3) {
        if (!(ports(cfe) & (1U << i)) || cfe->streams[i]->memory != V4L2_MEMORY_DMABUF) continue;
        ret = cc_stream_unimport(cfe->streams[i], index);
        if (ret < 0) return cancel_error(cfe, ret);
    }
    cfe->states[index] = 0;
    return 0;
}

int cc_cfe_export_plane(cc_cfe *cfe, enum cc_cfe_port port, uint32_t index, uint32_t plane)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    if (index >= cfe->count || cfe->states[index] != 2)
        return -EINVAL;
    if (port==CC_CFE_EMBEDDED) return cc_camera_export_embedded_plane(cfe->camera,index,plane);
    if (port==CC_CFE_IMAGE1) return cfe->configuration.image1.fourcc ?
        cc_stream_export_plane(cfe->streams[3], index, plane) : -ENOENT;
    if ((unsigned int)port>CC_CFE_STATISTICS) return -EINVAL;
    return cc_stream_export_plane(cfe->streams[port], index, plane);
}

int cc_cfe_fd(const cc_cfe *cfe, enum cc_cfe_port port)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    if (port==CC_CFE_EMBEDDED) return cc_camera_embedded_fd(cfe->camera);
    if (port==CC_CFE_IMAGE1) return cfe->configuration.image1.fourcc ? cc_stream_fd(cfe->streams[3]) : -ENOENT;
    return (unsigned int)port > CC_CFE_PARAMETERS ? -EINVAL : cc_stream_fd(cfe->streams[port]);
}
int cc_cfe_configure_embedded(cc_cfe *cfe, cc_embedded_configuration *configuration)
{
    int ret=usable(cfe);
    if (ret<0) return ret;
    if (allocated(cfe)) return -EBUSY;
    if (!cfe->configured) return -EPIPE;
    ret=cc_camera_configure_embedded(cfe->camera,configuration);
    if (ret<0 && !cfe->camera->stream->configured) cfe->configured=false;
    return ret;
}
int cc_cfe_get_embedded_configuration(cc_cfe *cfe, cc_embedded_configuration *configuration)
{
    int ret=usable(cfe);
    return ret<0 ? ret:cc_camera_get_embedded_configuration(cfe->camera,configuration);
}

int cc_cfe_query_control(cc_cfe *cfe, uint32_t id, cc_control_info *info)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_query_control(cfe->camera, id, info);
}
int cc_cfe_query_menu(cc_cfe *cfe,uint32_t id,uint32_t index,cc_control_menu *entry)
{
    int ret=usable(cfe);
    return ret<0 ? ret:cc_camera_query_menu(cfe->camera,id,index,entry);
}
int cc_cfe_query_lens(cc_cfe *cfe, cc_control_info *info)
{
    int ret=usable(cfe); return ret<0 ? ret:cc_camera_query_lens(cfe->camera,info);
}
int cc_cfe_get_lens_position(cc_cfe *cfe, int32_t *position)
{
    int ret=usable(cfe); return ret<0 ? ret:cc_camera_get_lens_position(cfe->camera,position);
}
int cc_cfe_set_lens_position(cc_cfe *cfe, int32_t *position)
{
    int ret=usable(cfe); return ret<0 ? ret:cc_camera_set_lens_position(cfe->camera,position);
}
int cc_cfe_next_control(cc_cfe *cfe, uint32_t after_id, cc_control_info *info)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_next_control(cfe->camera, after_id, info);
}
int cc_cfe_get_controls(cc_cfe *cfe, cc_control *controls, size_t count, size_t *error_index)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_get_controls(cfe->camera, controls, count, error_index);
}
int cc_cfe_set_controls(cc_cfe *cfe, const cc_control *controls, size_t count, size_t *error_index)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    bool changed;
    ret = cc_controls_io(&cfe->camera->io, cfe->camera->sensor_fd, controls, NULL, count,
                          error_index, allocated(cfe), &changed);
    if (changed) cfe->configured = cfe->camera->stream->configured = false;
    return ret;
}

int cc_cfe_apply_controls(cc_cfe *cfe, cc_control *controls, size_t count)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_controls_apply_io(&cfe->camera->io, cfe->camera->sensor_fd, controls, count, true);
}

int cc_cfe_get_sensor(cc_cfe *cfe, const cc_sensor_model **model)
{
    if (model) *model = NULL;
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_get_sensor(cfe->camera, model);
}
int cc_cfe_get_properties(cc_cfe *cfe, cc_camera_properties *properties)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_get_properties(cfe->camera, properties);
}
int cc_cfe_try_orientation(cc_cfe *cfe, enum cc_orientation *orientation, enum cc_transform *transform)
{
    int ret=usable(cfe);
    return ret<0 ? ret:cc_camera_try_orientation(cfe->camera,orientation,transform);
}
int cc_cfe_get_orientation(cc_cfe *cfe, enum cc_orientation *orientation)
{
    int ret=usable(cfe);
    return ret<0 ? ret:cc_camera_get_orientation(cfe->camera,orientation);
}
int cc_cfe_select_sensor_mode(cc_cfe *cfe, const cc_sensor_mode_request *request, cc_sensor_format *format)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_select_sensor_mode(cfe->camera, request, format);
}
int cc_cfe_select_rpi_sensor_mode(cc_cfe *cfe, const cc_rpi_sensor_mode_request *request, cc_sensor_format *format)
{
    int ret = usable(cfe);
    return ret < 0 ? ret : cc_camera_select_rpi_sensor_mode(cfe->camera, request, format);
}
int cc_cfe_get_sensor_timing(cc_cfe *cfe, cc_sensor_timing *timing)
{
    int ret = usable(cfe);
    if (ret < 0) return ret;
    if (!cfe->configured) return -EPIPE;
    return cc_camera_get_sensor_timing(cfe->camera, timing);
}
int cc_cfe_get_frame_interval(cc_cfe *cfe,cc_fraction *interval)
{
    int ret=usable(cfe); if (ret<0) return ret;
    if (!cfe->configured) return -EPIPE;
    return cc_camera_get_frame_interval(cfe->camera,interval);
}
int cc_cfe_set_frame_interval(cc_cfe *cfe,cc_fraction *interval)
{
    int ret=usable(cfe); if (ret<0) return ret;
    if (!cfe->configured) return -EPIPE;
    if (allocated(cfe)) return -EBUSY;
    return cc_camera_set_frame_interval(cfe->camera,interval);
}
int cc_cfe_enum_sensor_frame_interval(cc_cfe *cfe,uint32_t code,uint32_t width,uint32_t height,uint32_t index,cc_fraction *interval)
{
    int ret=usable(cfe);
    return ret<0 ? ret:cc_camera_enum_sensor_frame_interval(cfe->camera,code,width,height,index,interval);
}
int cc_cfe_get_sensor_streams(cc_cfe *cfe, cc_sensor_streams *streams)
{
    int ret=usable(cfe);
    return ret<0 ? ret:cc_camera_get_sensor_streams(cfe->camera,streams);
}
int cc_cfe_get_sensor_timing_limits(cc_cfe *cfe, cc_sensor_timing_limits *limits)
{
    int ret=usable(cfe);
    if (ret<0) return ret;
    if (!cfe->configured) return -EPIPE;
    return cc_camera_get_sensor_timing_limits(cfe->camera,limits);
}
