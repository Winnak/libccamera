/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include "embedded_capture.h"
#include <ccamera/sensor.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/media-bus-format.h>
#include <stdlib.h>
#include <string.h>

const char *cc_version(void) { return "0.1.0"; }

int cc_camera_get_sensor(cc_camera *camera, const cc_sensor_model **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!camera) return -EINVAL;
    if (!camera->media || camera->sensor_fd < 0) return -ENOTSUP;
    struct cc_entity *sensor = cc_media_entity(camera->media, camera->info.sensor_entity);
    return sensor ? cc_sensor_lookup(cc_entity_name(sensor), out) : -ENODEV;
}
int cc_camera_get_sensor_timing(cc_camera *camera, cc_sensor_timing *out)
{
    if (!camera || !out) return -EINVAL;
    if (camera->sensor_fd < 0 || camera->info.pipeline == CC_PIPELINE_VIMC) return -ENOTSUP;
    if (!camera->stream->configured) return -EPIPE;
    cc_control controls[] = {{.id=V4L2_CID_PIXEL_RATE}, {.id=V4L2_CID_HBLANK}, {.id=V4L2_CID_VBLANK}};
    int ret = cc_camera_get_controls(camera, controls, 3, NULL);
    if (ret < 0) return ret;
    if (controls[0].value <= 0 || controls[1].value < 0 || controls[1].value > UINT32_MAX ||
        controls[2].value < 0 || controls[2].value > UINT32_MAX) return -ERANGE;
    cc_sensor_timing timing = {.pixel_rate=(uint64_t)controls[0].value,
        .width=camera->stream->format.width, .height=camera->stream->format.height,
        .hblank=(uint32_t)controls[1].value, .vblank=(uint32_t)controls[2].value};
    uint64_t duration;
    ret = cc_sensor_frame_duration(&timing, &duration);
    if (!ret) *out = timing;
    return ret;
}

static int capture_type(const struct v4l2_capability *caps, enum v4l2_buf_type *type)
{
    uint32_t capabilities = caps->capabilities & V4L2_CAP_DEVICE_CAPS ?
                            caps->device_caps : caps->capabilities;
    if (!(capabilities & V4L2_CAP_STREAMING) ||
        capabilities & (V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE))
        return -ENOTSUP;
    if (capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        *type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    else if (capabilities & V4L2_CAP_VIDEO_CAPTURE)
        *type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    else
        return -ENOTSUP;
    return 0;
}

int cc_query_capture(const struct cc_io *io, int fd, struct v4l2_capability *caps,
                     enum v4l2_buf_type *type)
{
    memset(caps, 0, sizeof(*caps));
    int ret = cc_ioctl(io, fd, VIDIOC_QUERYCAP, caps);
    return ret < 0 ? ret : capture_type(caps, type);
}

static int open_locked(const struct cc_io *io, const char *path)
{
    int fd = io->ops->open(io->ctx, path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        return fd;
    int ret = io->ops->lock(io->ctx, fd);
    if (ret < 0) {
        io->ops->close(io->ctx, fd);
        return ret;
    }
    return fd;
}

static int open_media_camera(cc_camera *camera)
{
    int ret = cc_media_open(&camera->io, camera->info.media_path, true, &camera->media);
    if (ret < 0)
        return ret;
    struct cc_media *media = camera->media;
    bool cfe = camera->info.pipeline == CC_PIPELINE_RPI_CFE;
    bool vimc = camera->info.pipeline == CC_PIPELINE_VIMC;
    const char *driver = vimc ? "vimc" : cfe ? "rp1-cfe" : "unicam";
    const char *node = vimc ? "RGB/YUV Capture" : cfe ? "rp1-cfe-csi2-ch0" : "unicam-image";
    if (strncmp(media->info.driver, driver, sizeof(media->info.driver)))
        return -ENOTSUP;
    struct cc_entity *sensor = cc_media_entity(media, camera->info.sensor_entity);
    if (!sensor || sensor->desc.type != MEDIA_ENT_F_CAM_SENSOR)
        return -ENODEV;
    struct cc_entity *video = NULL;
    for (size_t i = 0; i < media->num_entities; ++i) {
        if (!strcmp(cc_entity_name(&media->entities[i]), node) &&
            !strcmp(media->entities[i].path, camera->info.video_path))
            video = &media->entities[i];
    }
    if (!video)
        return -ENODEV;
    if (vimc) {
        struct cc_entity *selected, *processed, *raw;
        ret = cc_vimc_match(media, &selected, &processed, &raw);
        if (ret < 0) return ret;
        if (selected != sensor || processed != video) return -ENODEV;
        ret = cc_vimc_route(media, &camera->route, &camera->route_count);
    } else ret = cc_media_route(media, sensor->desc.id, video->desc.id,
                                &camera->route, &camera->route_count);
    if (ret < 0)
        return ret;
    camera->subdev_fds = malloc(media->num_entities * sizeof(*camera->subdev_fds));
    camera->subdev_streams = calloc(media->num_entities, sizeof(*camera->subdev_streams));
    camera->saved_link_flags = calloc(media->num_links, sizeof(*camera->saved_link_flags));
    camera->changed_links = calloc(media->num_links, sizeof(*camera->changed_links));
    if (camera->subdev_fds)
        for (size_t i = 0; i < media->num_entities; ++i)
            camera->subdev_fds[i] = -1;
    if (!camera->subdev_fds || !camera->subdev_streams ||
        !camera->saved_link_flags || !camera->changed_links)
        return -ENOMEM;
    /* Every source along the route is a subdevice; the final sink is the
     * capture node. Open the sensor and all intervening mux/bridge nodes. */
    for (size_t i = 0; i < camera->route_count; ++i) {
        struct media_link_desc *link = &media->links[camera->route[i]];
        struct cc_entity *entity = cc_media_entity(media, link->source.entity);
        if (!entity || !entity->path[0])
            return -ENODEV;
        size_t at = (size_t)(entity - media->entities);
        int fd = open_locked(&camera->io, entity->path);
        if (fd < 0)
            return fd;
        camera->subdev_fds[at] = fd;
        struct cc_subdev_capability caps = {0};
        ret = cc_ioctl(&camera->io, fd, CC_SD_QUERYCAP, &caps);
        if (ret < 0 && ret != -ENOTTY)
            return ret;
        if (caps.capabilities & CC_SD_CAP_READONLY)
            return -EACCES;
        if (caps.capabilities & CC_SD_CAP_STREAMS) {
            struct cc_subdev_client_capability client = { .capabilities = 1 };
            ret = cc_ioctl(&camera->io, fd, CC_SD_S_CLIENT_CAP, &client);
            if (ret < 0 && ret != -ENOTTY)
                return ret;
            if (!ret && !(client.capabilities & 1U))
                return -EPROTO;
            camera->subdev_streams[at] = true;
        }
        if (!i) {
            camera->sensor_fd = fd;
            camera->sensor_pad = link->source.index;
        }
    }
    return 0;
}

int cc_camera_open_io(const cc_camera_info *info, const struct cc_io *io, cc_camera **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!info || !io || !info->video_path[0] ||
        !memchr(info->video_path, '\0', sizeof(info->video_path)) ||
        !memchr(info->media_path, '\0', sizeof(info->media_path)) ||
        !memchr(info->id, '\0', sizeof(info->id)) || (unsigned)info->id_type>CC_CAMERA_ID_VIDEO)
        return -EINVAL;
    if (info->pipeline != CC_PIPELINE_V4L2 && info->pipeline != CC_PIPELINE_RPI_UNICAM &&
        info->pipeline != CC_PIPELINE_RPI_CFE && info->pipeline != CC_PIPELINE_VIMC && info->pipeline != CC_PIPELINE_UVC)
        return -ENOTSUP;
    cc_camera *camera = calloc(1, sizeof(*camera));
    if (!camera)
        return -ENOMEM;
    camera->io = *io;
    camera->info = *info;
    camera->info.name[sizeof(camera->info.name) - 1] = '\0';
    camera->info.driver[sizeof(camera->info.driver) - 1] = '\0';
    camera->fd = -1;
    camera->sensor_fd = -1;
    camera->lens_fd = -1;
    int ret;
    if (info->pipeline==CC_PIPELINE_UVC && info->media_path[0]) {
        ret=cc_uvc_open_media(camera);
        if (ret<0) goto fail;
    } else if (info->pipeline != CC_PIPELINE_V4L2 && info->pipeline!=CC_PIPELINE_UVC) {
        ret = open_media_camera(camera);
        if (ret < 0)
            goto fail;
        if (info->pipeline == CC_PIPELINE_VIMC) {
            ret = cc_vimc_open(camera);
            if (ret < 0) goto fail;
        }
    }
    ret = cc_video_open_io(info->video_path, io, &camera->device);
    if (ret < 0)
        goto fail;
    camera->fd = camera->device->fd;
    struct v4l2_capability caps = camera->device->caps;
    bool uvc=!strncmp((const char *)caps.driver,"uvcvideo",sizeof(caps.driver));
    if (info->pipeline==CC_PIPELINE_UVC && !uvc) { ret=-ENOTSUP; goto fail; }
    if (info->pipeline==CC_PIPELINE_V4L2 && uvc) camera->info.pipeline=CC_PIPELINE_UVC;
    if (camera->info.pipeline==CC_PIPELINE_UVC) camera->info.sensor_entity=0;
    enum v4l2_buf_type type;
    ret = capture_type(&caps, &type);
    if (ret < 0)
        goto fail;
    if (info->pipeline == CC_PIPELINE_V4L2 && camera->device->capabilities & V4L2_CAP_IO_MC) {
        ret = -ENOTSUP;
        goto fail;
    }
    ret = cc_video_create_stream(camera->device, (enum cc_stream_type)type, &camera->stream);
    if (ret < 0)
        goto fail;
    if (!camera->info.name[0])
        cc_copy_string(camera->info.name, sizeof(camera->info.name),
                        (const char *)caps.card, sizeof(caps.card));
    cc_copy_string(camera->info.driver, sizeof(camera->info.driver),
                    (const char *)caps.driver, sizeof(caps.driver));
    if (camera->media && camera->subdev_streams) {
        struct cc_entity *sensor=cc_media_entity(camera->media,camera->info.sensor_entity);
        if (sensor && camera->subdev_streams[sensor-camera->media->entities]) {
            cc_sensor_streams streams;
            ret=cc_camera_get_sensor_streams(camera,&streams);
            if (ret<0) goto fail;
            camera->sensor_stream=streams.image.stream;
        }
    }
    const char *model=NULL;
    if (camera->sensor_fd>=0) {
        struct cc_entity *sensor=cc_media_entity(camera->media,camera->info.sensor_entity);
        if (!sensor) { ret=-ENODEV; goto fail; }
        model=cc_entity_name(sensor);
    }
    camera->info.id[0]='\0'; camera->info.id_type=CC_CAMERA_ID_NONE;
    ret=cc_camera_identify_io(io,camera->sensor_fd>=0 ? camera->sensor_fd:camera->fd,
        0,0,camera->info.pipeline,model,&camera->info);
    if (ret<0) {
        if (info->id[0] || !cc_identity_unavailable(ret)) goto fail;
    } else if (info->id[0] && (strcmp(info->id,camera->info.id) ||
        (info->id_type!=CC_CAMERA_ID_NONE && info->id_type!=camera->info.id_type))) {
        ret=-ESTALE; goto fail;
    }
    *out = camera;
    return 0;
fail:
    cc_camera_close(camera);
    return ret;
}

int cc_camera_open(const cc_camera_info *info, cc_camera **out)
{
    return cc_camera_open_io(info, &cc_system_io, out);
}

int cc_camera_open_video(const char *path, cc_camera **out)
{
    if (out)
        *out = NULL;
    if (!path || !out)
        return -EINVAL;
    if (strlen(path) >= CC_PATH_MAX)
        return -ENAMETOOLONG;
    cc_camera_info info = { .pipeline = CC_PIPELINE_V4L2 };
    cc_copy_string(info.video_path, sizeof(info.video_path), path, strlen(path));
    return cc_camera_open(&info, out);
}

void cc_camera_close(cc_camera *camera)
{
    if (!camera)
        return;
    if (camera->embedded) {
        (void)cc_camera_stop(camera);
        cc_embedded_close(camera->embedded); camera->embedded=NULL;
    }
    cc_stream_close(camera->stream);
    cc_video_close(camera->device);
    cc_vimc_close(camera);
    if (camera->lens_fd>=0) camera->io.ops->close(camera->io.ctx,camera->lens_fd);
    if (camera->media) {
        if (camera->subdev_fds)
            for (size_t i = 0; i < camera->media->num_entities; ++i)
                if (camera->subdev_fds[i] >= 0)
                    camera->io.ops->close(camera->io.ctx, camera->subdev_fds[i]);
        /* Disable our inputs/outputs before re-enabling previous ones. A mux
         * may reject two enabled links even while no video node is streaming. */
        if (camera->changed_links) {
            for (int phase = 0; phase < 2; ++phase)
                for (size_t i = camera->media->num_links; i > 0; --i) {
                    size_t j = i - 1;
                    bool was_enabled = (camera->saved_link_flags[j] & MEDIA_LNK_FL_ENABLED) != 0;
                    if (camera->changed_links[j] && was_enabled == (phase == 1))
                        cc_media_set_link(camera->media, j, camera->saved_link_flags[j]);
                }
        }
        cc_media_close(camera->media);
    }
    free(camera->subdev_fds);
    free(camera->subdev_streams);
    free(camera->saved_link_flags);
    free(camera->changed_links);
    free(camera->route);
    free(camera);
}

const cc_camera_info *cc_camera_get_info(const cc_camera *camera)
{
    return camera ? &camera->info : NULL;
}

int cc_camera_fd(const cc_camera *camera)
{
    return camera ? cc_stream_fd(camera->stream) : -EINVAL;
}

int cc_camera_enum_format(cc_camera *camera, uint32_t index, cc_format_info *format)
{
    if (camera && camera->info.pipeline == CC_PIPELINE_VIMC)
        return cc_vimc_enum_format(camera, index, format);
    return cc_stream_enum_format(camera ? camera->stream : NULL, index, format);
}

int cc_camera_enum_size(cc_camera *camera, uint32_t fourcc, uint32_t index, cc_size_range *size)
{
    if (!camera || !size)
        return -EINVAL;
    if (camera->info.pipeline == CC_PIPELINE_VIMC)
        return cc_vimc_enum_size(camera, fourcc, index, size);
    struct v4l2_frmsizeenum desc = { .index = index, .pixel_format = fourcc };
    int ret = cc_ioctl(&camera->io, camera->fd, VIDIOC_ENUM_FRAMESIZES, &desc);
    if (ret < 0)
        return ret == -EINVAL ? -ENOENT : ret;
    if (desc.index != index || desc.pixel_format != fourcc)
        return -EPROTO;
    cc_size_range result;
    if (desc.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        result = (cc_size_range) { desc.discrete.width, desc.discrete.width, 1,
                                 desc.discrete.height, desc.discrete.height, 1 };
    } else if (desc.type == V4L2_FRMSIZE_TYPE_STEPWISE || desc.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
        if (index) return -EPROTO;
        result = (cc_size_range) { desc.stepwise.min_width, desc.stepwise.max_width,
                                 desc.stepwise.step_width, desc.stepwise.min_height,
                                 desc.stepwise.max_height, desc.stepwise.step_height };
    } else {
        return -EPROTO;
    }
    if (!result.min_width || !result.min_height || result.max_width < result.min_width ||
        result.max_height < result.min_height || !result.width_step || !result.height_step)
        return -EPROTO;
    *size = result;
    return 0;
}

int cc_camera_get_format(cc_camera *camera, cc_format *format)
{
    return cc_stream_get_format(camera ? camera->stream : NULL, format);
}

int cc_camera_change_link(cc_camera *camera, size_t index, uint32_t flags)
{
    uint32_t old_flags = camera->media->links[index].flags;
    if (old_flags == flags)
        return 0;
    int ret = cc_media_set_link(camera->media, index, flags);
    if (ret < 0)
        return ret;
    if (!camera->changed_links[index]) {
        camera->changed_links[index] = true;
        camera->saved_link_flags[index] = old_flags;
    }
    return 0;
}

static int configure_links(cc_camera *camera)
{
    struct cc_media *media = camera->media;
    if (camera->info.pipeline == CC_PIPELINE_RPI_CFE) {
        /* Raw CSI DMA capture must disconnect the FE path and unused CSI DMA
         * outputs. Enabled nodes in this media component otherwise have to be
         * streamed together by the CFE driver. The whole CFE is locked. */
        size_t final = camera->route[camera->route_count - 1];
        uint32_t csi = media->links[final].source.entity;
        struct cc_entity *receiver = cc_media_entity(media, csi);
        if (!receiver || strcmp(cc_entity_name(receiver), "csi2"))
            return -ENOTSUP;
        for (size_t j = 0; j < media->num_links; ++j) {
            struct media_link_desc *link = &media->links[j];
            if (j != final && link->source.entity == csi && link->flags & MEDIA_LNK_FL_ENABLED) {
                int ret = cc_camera_change_link(camera, j, link->flags & ~MEDIA_LNK_FL_ENABLED);
                if (ret < 0)
                    return ret;
            }
        }
    }
    for (size_t r = 0; r < camera->route_count; ++r) {
        size_t route_index = camera->route[r];
        struct media_link_desc route = media->links[route_index];
        /* Select this input on a mux/receiver, preserving unrelated pipelines. */
        for (size_t j = 0; j < media->num_links; ++j) {
            struct media_link_desc *other = &media->links[j];
            if (j != route_index && other->sink.entity == route.sink.entity &&
                other->sink.index == route.sink.index && other->flags & MEDIA_LNK_FL_ENABLED) {
                int ret = cc_camera_change_link(camera, j, other->flags & ~MEDIA_LNK_FL_ENABLED);
                if (ret < 0)
                    return ret;
            }
        }
        int ret = cc_camera_change_link(camera, route_index, route.flags | MEDIA_LNK_FL_ENABLED);
        if (ret < 0)
            return ret;
    }
    return 0;
}

static int configure_subdev_route(cc_camera *camera, size_t entity_index,
                                   uint32_t sink_pad, uint32_t sink_stream, uint32_t source_pad, bool active)
{
    if (!camera->subdev_streams[entity_index])
        return sink_stream ? -ENOTSUP:0;
    /* Carry the sensor's discovered stream onto the selected output stream 0.
     * Embedded data and further exposures need their own receiver routes. */
    struct cc_subdev_route route = {
        .sink_pad = sink_pad, .sink_stream=sink_stream, .source_pad = source_pad, .flags = CC_SD_ROUTE_ACTIVE
    };
    struct cc_subdev_routing routing = {
        .which = active ? V4L2_SUBDEV_FORMAT_ACTIVE : V4L2_SUBDEV_FORMAT_TRY,
        .len_routes = 1, .num_routes = 1, .routes = (uint64_t)(uintptr_t)&route
    };
    int fd = camera->subdev_fds[entity_index];
    int ret = cc_ioctl(&camera->io, fd, CC_SD_S_ROUTING, &routing);
    if (ret == -ENOTTY) {
        struct cc_subdev_routing_legacy legacy = {
            .which = routing.which, .num_routes = 1, .routes = routing.routes
        };
        ret = cc_ioctl(&camera->io, fd, CC_SD_S_ROUTING_LEGACY, &legacy);
        if (!ret && (legacy.which != routing.which || legacy.routes != routing.routes)) return -EPROTO;
        routing.num_routes = legacy.num_routes;
    }
    if (ret < 0)
        return ret;
    if (routing.num_routes != 1 || routing.len_routes != 1 || routing.routes != (uint64_t)(uintptr_t)&route ||
        routing.which != (active ? V4L2_SUBDEV_FORMAT_ACTIVE:V4L2_SUBDEV_FORMAT_TRY) ||
        route.sink_pad != sink_pad || route.source_pad != source_pad ||
        route.sink_stream != sink_stream || route.source_stream || !(route.flags & CC_SD_ROUTE_ACTIVE))
        return -EPROTO;
    return 0;
}

static int negotiate_sensor(cc_camera *camera, cc_format *format, bool active, bool orient)
{
    uint32_t code = cc_bayer_mbus(format->fourcc);
    if (!code)
        return -ENOTSUP;
    uint32_t which=active ? V4L2_SUBDEV_FORMAT_ACTIVE:V4L2_SUBDEV_FORMAT_TRY, stream;
    int ret=cc_sensor_streams_image_route(camera,which,&stream);
    if (ret<0) return ret;
    struct cc_subdev_format sf = {
        .which = which, .stream=stream,
        .pad = camera->sensor_pad,
        .format = { .width = format->width, .height = format->height,
                    .code = code, .field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_RAW }
    };
    ret = cc_ioctl(&camera->io, camera->sensor_fd, VIDIOC_SUBDEV_S_FMT, &sf);
    if (ret < 0)
        return ret;
    if (sf.pad != camera->sensor_pad || sf.stream != stream || sf.which != which)
        return -EPROTO;
    if (sf.format.code != code) {
        uint32_t depth, actual_depth;
        enum cc_camera_cfa cfa, actual_cfa;
        if (!orient || cc_bayer_code_info(code,&depth,&cfa)<0 ||
            cc_bayer_code_info(sf.format.code,&actual_depth,&actual_cfa)<0 || depth!=actual_depth ||
            (cfa==CC_CAMERA_CFA_MONO)!=(actual_cfa==CC_CAMERA_CFA_MONO)) return -ENOTSUP;
        uint32_t fourcc=cc_bayer_reorder(format->fourcc,actual_cfa);
        if (!fourcc) return -ENOTSUP;
        format->fourcc=fourcc; code=sf.format.code;
    }
    /* The incoming link sets a bridge's sink format. Read its selected source
     * to propagate actual pad dimensions and codes to the next bridge. */
    for (size_t i = 1; i < camera->route_count; ++i) {
        struct media_link_desc *incoming = &camera->media->links[camera->route[i - 1]];
        struct media_link_desc *outgoing = &camera->media->links[camera->route[i]];
        struct cc_entity *entity = cc_media_entity(camera->media, incoming->sink.entity);
        size_t entity_index = (size_t)(entity - camera->media->entities);
        int fd = camera->subdev_fds[entity_index];
        ret = configure_subdev_route(camera, entity_index, incoming->sink.index, stream,
                                      outgoing->source.index, active);
        if (ret < 0)
            return ret;
        sf.pad = incoming->sink.index;
        sf.stream=stream;
        ret = cc_ioctl(&camera->io, fd, VIDIOC_SUBDEV_S_FMT, &sf);
        if (ret < 0)
            return ret;
        if (sf.pad != incoming->sink.index || sf.stream != stream || sf.which != which) return -EPROTO;
        stream=0; sf.stream=0; sf.pad = outgoing->source.index;
        ret = cc_ioctl(&camera->io, fd, VIDIOC_SUBDEV_G_FMT, &sf);
        if (ret < 0)
            return ret;
        if (sf.format.code != code)
            return -ENOTSUP;
        if (sf.pad != outgoing->source.index || sf.stream || sf.which != which) return -EPROTO;
    }
    format->width = sf.format.width;
    format->height = sf.format.height;
    format->field = sf.format.field;
    format->colorspace = sf.format.colorspace;
    format->ycbcr_enc = sf.format.ycbcr_enc;
    format->quantization = sf.format.quantization;
    format->xfer_func = sf.format.xfer_func;
    return 0;
}

static int negotiate_format(cc_camera *camera, cc_format *format, bool active, bool orient)
{
    if (!camera || !format || !format->width || !format->height || !format->fourcc ||
        format->ycbcr_enc > UINT8_MAX || format->quantization > UINT8_MAX ||
        format->xfer_func > UINT8_MAX)
        return -EINVAL;
    if (active && (camera->stream->allocated || camera->stream->streaming ||
        (camera->embedded && camera->embedded->stream->allocated)))
        return -EBUSY;
    if (active && camera->embedded) {
        int disabled=cc_camera_configure_embedded(camera,NULL);
        if (disabled<0) return disabled;
    }
    if (active)
        camera->stream->configured = false; /* Any partial hardware change needs reconfigure. */
    cc_format requested = *format;
    int ret;
    if (camera->info.pipeline == CC_PIPELINE_VIMC) {
        ret = cc_vimc_negotiate(camera, &requested, active);
        if (!ret) *format = requested;
        return ret;
    }
    if (camera->sensor_fd >= 0) {
        if (active) {
            ret = configure_links(camera);
            if (ret < 0)
                return ret;
        }
        ret = negotiate_sensor(camera, &requested, active, orient);
        if (ret < 0)
            return ret;
    }
    cc_format actual = requested;
    ret = active ? cc_stream_configure(camera->stream, &actual) :
                   cc_stream_try_format(camera->stream, &actual);
    if (ret < 0)
        return ret;
    if (camera->sensor_fd >= 0 && (actual.width != requested.width ||
        actual.height != requested.height || actual.fourcc != requested.fourcc)) {
        if (active)
            camera->stream->configured = false;
        return -EPIPE;
    }
    *format = actual;
    return 0;
}

int cc_camera_try_format(cc_camera *camera, cc_format *format)
{
    return negotiate_format(camera, format, false, false);
}

int cc_camera_configure(cc_camera *camera, cc_format *format)
{
    return negotiate_format(camera, format, true, false);
}

int cc_camera_try_format_oriented(cc_camera *camera, cc_format *format, enum cc_orientation *orientation)
{
    if (!camera || !format || !orientation) return -EINVAL;
    enum cc_orientation actual=*orientation; uint32_t delta;
    int ret=cc_camera_predict_orientation(camera,&actual,&delta);
    if (ret<0) return ret;
    cc_format tried=*format;
    ret=negotiate_format(camera,&tried,false,true);
    if (ret<0) return ret;
    uint32_t code=cc_bayer_mbus(tried.fourcc); enum cc_camera_cfa cfa;
    if (delta && !cc_bayer_code_info(code,NULL,&cfa) && cfa!=CC_CAMERA_CFA_MONO)
        tried.fourcc=cc_bayer_reorder(tried.fourcc,(enum cc_camera_cfa)((uint32_t)cfa^delta));
    *format=tried; *orientation=actual; return 0;
}

int cc_camera_configure_oriented(cc_camera *camera, cc_format *format, enum cc_orientation *orientation)
{
    if (!camera || !format || !orientation || !format->width || !format->height || !format->fourcc ||
        format->ycbcr_enc>UINT8_MAX || format->quantization>UINT8_MAX || format->xfer_func>UINT8_MAX)
        return -EINVAL;
    int ret=cc_camera_fd(camera);
    if (ret<0) return ret;
    if (camera->stream->allocated || camera->stream->streaming ||
        (camera->embedded && camera->embedded->stream->allocated)) return -EBUSY;
    if (camera->sensor_fd>=0 && camera->info.pipeline!=CC_PIPELINE_VIMC && !cc_bayer_mbus(format->fourcc))
        return -ENOTSUP;
    enum cc_orientation actual=*orientation;
    ret=cc_camera_apply_orientation(camera,&actual);
    if (ret<0) return ret;
    cc_format configured=*format;
    ret=negotiate_format(camera,&configured,true,true);
    if (!ret) ret=cc_camera_get_orientation(camera,&actual);
    if (ret<0) { camera->stream->configured=false; return ret; }
    *format=configured; *orientation=actual;
    return 0;
}

static int allocate_camera(cc_camera *camera, enum cc_memory memory, uint32_t *count)
{
    if (!camera || !count) return -EINVAL;
    uint32_t n=*count;
    int ret=cc_stream_allocate_buffers(camera->stream,memory,&n);
    if (ret<0) return ret;
    if (camera->embedded) {
        ret=cc_embedded_allocate(camera->embedded,&n);
        if (ret<0) { (void)cc_camera_free_buffers(camera); return ret; }
    }
    *count=n; return 0;
}
int cc_camera_allocate_buffers(cc_camera *camera, uint32_t *count)
{
    return allocate_camera(camera,CC_MEMORY_MMAP,count);
}

int cc_camera_prepare_imports(cc_camera *camera, uint32_t *count)
{
    return allocate_camera(camera,CC_MEMORY_DMABUF,count);
}

int cc_camera_import_buffer(cc_camera *camera, uint32_t index,
                            const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    return cc_stream_import(camera ? camera->stream : NULL, index, planes, num_planes);
}

int cc_camera_free_buffers(cc_camera *camera)
{
    if (!camera) return -EINVAL;
    int ret=0;
    if (camera->embedded) {
        ret=cc_camera_stop(camera);
        if (!ret) ret=cc_embedded_free(camera->embedded);
    }
    int image=cc_stream_free_buffers(camera->stream);
    return ret<0 ? ret:image;
}

int cc_camera_buffer(cc_camera *camera, uint32_t index, cc_frame *buffer)
{
    return cc_stream_buffer(camera ? camera->stream : NULL, index, buffer);
}

int cc_camera_export_plane(cc_camera *camera, uint32_t index, uint32_t plane)
{
    return cc_stream_export_plane(camera ? camera->stream : NULL, index, plane);
}

int cc_camera_queue(cc_camera *camera, uint32_t index)
{
    if (!camera) return -EINVAL;
    struct cc_embedded_capture *e=camera->embedded;
    if (e && (index>=e->source_count || (e->image_pending && e->image.index==index)))
        return index>=e->source_count ? -EINVAL:-EBUSY;
    int ret=cc_stream_queue(camera->stream,index,NULL);
    if (!ret && e) {
        ret=cc_embedded_release(e,index);
        if (ret<0) (void)cc_camera_stop(camera);
    }
    return ret;
}

int cc_camera_start(cc_camera *camera)
{
    if (!camera) return -EINVAL;
    if (!camera->embedded) return cc_stream_start(camera->stream);
    int ret=cc_embedded_start(camera->embedded);
    if (!ret) ret=cc_stream_start(camera->stream);
    if (ret<0) (void)cc_camera_stop(camera);
    return ret;
}

size_t cc_camera_pending_fds(const cc_camera *camera, struct pollfd *fds)
{
    const struct cc_embedded_capture *e=camera->embedded;
    fds[0]=(struct pollfd){e && e->image_pending ? cc_stream_fd(e->stream):cc_camera_fd(camera),POLLIN,0};
    return 1;
}

int cc_camera_dequeue(cc_camera *camera, cc_frame *frame, int timeout_ms)
{
    if (!camera || !frame || timeout_ms < -1) return -EINVAL;
    struct cc_embedded_capture *e=camera->embedded;
    if (!e) return cc_stream_dequeue(camera->stream,frame,timeout_ms);
    uint64_t deadline=0;
    if (timeout_ms>0) {
        int ret=camera->io.ops->now(camera->io.ctx,&deadline);
        if (ret<0) return ret;
        uint64_t delta=(uint64_t)timeout_ms*UINT64_C(1000000);
        deadline=deadline>UINT64_MAX-delta ? UINT64_MAX:deadline+delta;
    }
    for (;;) {
        int ret=0;
        if (!e->image_pending) {
            ret=cc_stream_dequeue(camera->stream,&e->image,0);
            if (!ret) e->image_pending=true;
        }
        if (!ret) {
            cc_frame metadata;
            ret=cc_embedded_collect(e,&e->image,&metadata);
            if (!ret) { *frame=e->image; e->image_pending=false; return 0; }
        }
        if (ret!=-EAGAIN) { (void)cc_camera_stop(camera); return ret; }
        if (!timeout_ms) return -EAGAIN;
        int remaining=-1;
        if (timeout_ms>0) {
            uint64_t now;
            ret=camera->io.ops->now(camera->io.ctx,&now);
            if (ret<0) { (void)cc_camera_stop(camera); return ret; }
            if (now>=deadline) return -ETIMEDOUT;
            uint64_t ms=(deadline-now)/UINT64_C(1000000)+((deadline-now)%UINT64_C(1000000)!=0);
            remaining=ms>INT_MAX ? INT_MAX:(int)ms;
        }
        struct pollfd fd; cc_camera_pending_fds(camera,&fd);
        ret=camera->io.ops->wait_many ? camera->io.ops->wait_many(camera->io.ctx,&fd,1,remaining):
            camera->io.ops->wait(camera->io.ctx,fd.fd,fd.events,remaining);
        if (ret==-ETIMEDOUT) return ret;
        if (ret<0 && ret!=-EAGAIN && ret!=-EINTR) { (void)cc_camera_stop(camera); return ret; }
    }
}

int cc_camera_stop(cc_camera *camera)
{
    if (!camera) return -EINVAL;
    int ret=cc_stream_stop(camera->stream);
    if (camera->embedded) {
        int metadata=cc_embedded_stop(camera->embedded);
        if (!ret) ret=metadata;
        if (ret<0) {
            cc_video_device *devices[]={camera->device,camera->embedded->device};
            for (size_t i=0;i<2;++i) if (devices[i]->fd>=0) {
                camera->io.ops->close(camera->io.ctx,devices[i]->fd); devices[i]->fd=-1;
            }
            camera->stream->configured=false;
        }
    }
    return ret;
}

static int control_fd(const cc_camera *camera)
{
    return camera->sensor_fd >= 0 ? camera->sensor_fd : camera->fd;
}

int cc_camera_query_control(cc_camera *camera, uint32_t id, cc_control_info *info)
{
    return camera ? cc_control_query_io(&camera->io, control_fd(camera), id, info) : -EINVAL;
}
int cc_camera_query_menu(cc_camera *camera,uint32_t id,uint32_t index,cc_control_menu *entry)
{
    return camera ? cc_control_menu_io(&camera->io,control_fd(camera),id,index,entry):-EINVAL;
}

int cc_camera_next_control(cc_camera *camera, uint32_t after_id, cc_control_info *info)
{
    int ret = cc_camera_query_control(camera, after_id | V4L2_CTRL_FLAG_NEXT_CTRL |
                                      V4L2_CTRL_FLAG_NEXT_COMPOUND, info);
    return ret == -EINVAL ? -ENOENT : ret;
}

static int camera_controls(cc_camera *camera, const cc_control *input, cc_control *output,
                             size_t count, size_t *error_index)
{
    if (!camera)
        return -EINVAL;
    bool changed;
    int ret = cc_controls_io(&camera->io, control_fd(camera), input, output, count,
                              error_index, camera->stream->allocated, &changed);
    if (changed)
        camera->stream->configured = false;
    return ret;
}

int cc_camera_apply_controls(cc_camera *camera, cc_control *controls, size_t count)
{
    return camera ? cc_controls_apply_io(&camera->io, control_fd(camera), controls, count, true) : -EINVAL;
}

int cc_camera_get_controls(cc_camera *camera, cc_control *controls, size_t count, size_t *error_index)
{
    return camera_controls(camera, controls, controls, count, error_index);
}

int cc_camera_set_controls(cc_camera *camera, const cc_control *controls, size_t count, size_t *error_index)
{
    return camera_controls(camera, controls, NULL, count, error_index);
}

static int lens_fd(cc_camera *camera)
{
    if (!camera) return -EINVAL;
    if (!camera->media || camera->info.pipeline==CC_PIPELINE_UVC) return camera->fd;
    if (camera->lens_fd>=0) return camera->lens_fd;
    struct cc_entity *lens;
    int ret=cc_media_lens(camera->media,camera->info.sensor_entity,&lens);
    if (ret<0) return ret;
    if (!lens->path[0]) return -ENODEV;
    ret=open_locked(&camera->io,lens->path);
    if (ret>=0) camera->lens_fd=ret;
    return ret;
}
int cc_camera_query_lens(cc_camera *camera, cc_control_info *info)
{
    if (!info) return -EINVAL;
    int fd=lens_fd(camera);
    return fd<0 ? fd:cc_control_query_io(&camera->io,fd,V4L2_CID_FOCUS_ABSOLUTE,info);
}
int cc_camera_get_lens_position(cc_camera *camera, int32_t *position)
{
    if (!position) return -EINVAL;
    int fd=lens_fd(camera); if (fd<0) return fd;
    cc_control value={.id=V4L2_CID_FOCUS_ABSOLUTE}; bool changed;
    int ret=cc_controls_io(&camera->io,fd,&value,&value,1,NULL,false,&changed);
    if (!ret) {
        if (value.value<INT32_MIN || value.value>INT32_MAX) return -ERANGE;
        *position=(int32_t)value.value;
    }
    return ret;
}
int cc_camera_set_lens_position(cc_camera *camera, int32_t *position)
{
    if (!position) return -EINVAL;
    int fd=lens_fd(camera); if (fd<0) return fd;
    cc_control value={.id=V4L2_CID_FOCUS_ABSOLUTE,.value=*position};
    int ret=cc_controls_apply_io(&camera->io,fd,&value,1,true);
    if (!ret) {
        if (value.value<INT32_MIN || value.value>INT32_MAX) return -ERANGE;
        *position=(int32_t)value.value;
    }
    return ret;
}
