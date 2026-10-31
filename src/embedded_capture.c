/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "embedded_capture.h"
#include "subdev_compat.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <stdlib.h>
#include <string.h>

static void reset(struct cc_embedded_capture *e)
{
    for (size_t i=0;i<256;++i) e->associated[i]=-1;
    e->future=-1; e->image_pending=e->have_timestamp=false;
    e->image=e->future_frame=(cc_frame){0};
}
static bool image_link(const cc_camera *camera, size_t link)
{
    for (size_t i=0;i<camera->route_count;++i) if (camera->route[i]==link) return true;
    return false;
}
static int subdevice(cc_camera *camera, uint32_t id, size_t *index)
{
    struct cc_entity *entity=cc_media_entity(camera->media,id);
    if (!entity || !entity->path[0]) return -ENODEV;
    *index=(size_t)(entity-camera->media->entities);
    if (camera->subdev_fds[*index]>=0) return camera->subdev_fds[*index];
    int fd=camera->io.ops->open(camera->io.ctx,entity->path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if (fd<0) return fd;
    camera->subdev_fds[*index]=fd;
    int ret=camera->io.ops->lock(camera->io.ctx,fd);
    if (ret<0) goto fail;
    struct cc_subdev_capability caps={0};
    ret=cc_ioctl(&camera->io,fd,CC_SD_QUERYCAP,&caps);
    if (ret<0 && ret!=-ENOTTY) goto fail;
    if (caps.capabilities&CC_SD_CAP_READONLY) { ret=-EACCES; goto fail; }
    if (caps.capabilities&CC_SD_CAP_STREAMS) {
        struct cc_subdev_client_capability client={.capabilities=1};
        ret=cc_ioctl(&camera->io,fd,CC_SD_S_CLIENT_CAP,&client);
        if (ret<0 && ret!=-ENOTTY) goto fail;
        if (!ret && !(client.capabilities&1U)) { ret=-EPROTO; goto fail; }
        camera->subdev_streams[*index]=true;
    }
    return fd;
fail:
    camera->io.ops->close(camera->io.ctx,fd); camera->subdev_fds[*index]=-1;
    camera->subdev_streams[*index]=false;
    return ret;
}
static int get_pad(cc_camera *camera, int fd, uint32_t pad, uint32_t stream, struct cc_subdev_format *format)
{
    *format=(struct cc_subdev_format){.which=V4L2_SUBDEV_FORMAT_ACTIVE,.pad=pad,.stream=stream};
    int ret=cc_ioctl(&camera->io,fd,VIDIOC_SUBDEV_G_FMT,format);
    if (!ret && (format->which!=V4L2_SUBDEV_FORMAT_ACTIVE || format->pad!=pad || format->stream!=stream)) ret=-EPROTO;
    return ret;
}
static int set_pad(cc_camera *camera, int fd, const struct cc_subdev_format *expected)
{
    struct cc_subdev_format actual=*expected;
    int ret=cc_ioctl(&camera->io,fd,VIDIOC_SUBDEV_S_FMT,&actual);
    if (ret<0) return ret;
    if (actual.which!=expected->which || actual.pad!=expected->pad || actual.stream!=expected->stream) return -EPROTO;
    ret=get_pad(camera,fd,expected->pad,expected->stream,&actual);
    if (ret<0) return ret;
    return actual.format.code==expected->format.code && actual.format.width==expected->format.width &&
        actual.format.height==expected->format.height ? 0:-EPIPE;
}
static int receiver_routes(cc_camera *camera, int fd, struct cc_subdev_route *routes, uint32_t count)
{
    struct cc_subdev_route expected[2]; memcpy(expected,routes,count*sizeof(*routes));
    struct cc_subdev_routing r={.which=V4L2_SUBDEV_FORMAT_ACTIVE,.len_routes=count,.num_routes=count,
        .routes=(uint64_t)(uintptr_t)routes};
    int ret=cc_ioctl(&camera->io,fd,CC_SD_S_ROUTING,&r);
    if (ret==-ENOTTY) {
        struct cc_subdev_routing_legacy old={.which=r.which,.num_routes=count,.routes=r.routes};
        ret=cc_ioctl(&camera->io,fd,CC_SD_S_ROUTING_LEGACY,&old);
        if (!ret && (old.which!=r.which || old.routes!=r.routes)) return -EPROTO;
        r.num_routes=old.num_routes;
    }
    if (ret<0) return ret;
    if (r.which!=V4L2_SUBDEV_FORMAT_ACTIVE || r.routes!=(uint64_t)(uintptr_t)routes ||
        r.len_routes!=count || r.num_routes!=count) return -EPROTO;
    for (uint32_t i=0;i<count;++i) {
        bool found=false;
        for (uint32_t j=0;j<count;++j)
            if (routes[j].sink_pad==expected[i].sink_pad && routes[j].sink_stream==expected[i].sink_stream &&
                routes[j].source_pad==expected[i].source_pad && routes[j].source_stream==expected[i].source_stream &&
                (routes[j].flags&CC_SD_ROUTE_ACTIVE)==(expected[i].flags&CC_SD_ROUTE_ACTIVE)) found=true;
        if (!found) return -EPROTO;
    }
    return 0;
}
static int configure_path(cc_camera *camera, struct cc_embedded_capture *e, const cc_sensor_streams *sensor, bool enable)
{
    struct cc_media *media=camera->media;
    /* Change physical ownership first, so even a persistent routing failure
     * during cleanup cannot leave an unowned metadata node enabled. */
    for (size_t i=0;i<e->path_count;++i) {
        size_t at=e->path[i];
        if (image_link(camera,at)) continue;
        uint32_t flags=media->links[at].flags;
        int changed=cc_camera_change_link(camera,at,enable ? flags|MEDIA_LNK_FL_ENABLED:flags&~MEDIA_LNK_FL_ENABLED);
        if (changed<0) return changed;
    }
    uint32_t image_stream;
    int ret=cc_sensor_streams_route(camera,V4L2_SUBDEV_FORMAT_ACTIVE,enable,&image_stream);
    if (ret<0) return ret;
    uint32_t stream=sensor->embedded.stream;
    struct cc_subdev_format format={.which=V4L2_SUBDEV_FORMAT_ACTIVE,.format={.code=sensor->embedded.mbus_code,
        .width=sensor->embedded.width,.height=sensor->embedded.height,.field=V4L2_FIELD_NONE}};
    for (size_t i=1;i<e->path_count;++i) {
        const struct media_link_desc *in=&media->links[e->path[i-1]], *out=&media->links[e->path[i]];
        size_t entity;
        int fd=subdevice(camera,in->sink.entity,&entity);
        if (fd<0) return fd;
        const struct media_link_desc *image_in=NULL,*image_out=NULL;
        uint32_t image_input_stream=0;
        for (size_t j=1;j<camera->route_count;++j) {
            const struct media_link_desc *a=&media->links[camera->route[j-1]], *b=&media->links[camera->route[j]];
            if (a->sink.entity!=in->sink.entity) continue;
            image_in=a; image_out=b; image_input_stream=j==1 ? image_stream:0;
            break;
        }
        uint32_t output_stream=0;
        if (camera->subdev_streams[entity]) {
            struct cc_subdev_route routes[2]; uint32_t count=0;
            struct cc_subdev_format image_input={0},image_output={0};
            if (image_in) {
                ret=get_pad(camera,fd,image_in->sink.index,image_input_stream,&image_input);
                if (!ret) ret=get_pad(camera,fd,image_out->source.index,0,&image_output);
                if (ret<0) return ret;
                if (in->sink.index==image_in->sink.index && stream==image_input_stream) return -ENOTSUP;
                routes[count++]=(struct cc_subdev_route){.sink_pad=image_in->sink.index,.sink_stream=image_input_stream,
                    .source_pad=image_out->source.index,.flags=CC_SD_ROUTE_ACTIVE};
                if (out->source.index==image_out->source.index) output_stream=1;
            }
            routes[count++]=(struct cc_subdev_route){.sink_pad=in->sink.index,.sink_stream=stream,
                .source_pad=out->source.index,.source_stream=output_stream,.flags=enable ? CC_SD_ROUTE_ACTIVE:0};
            ret=receiver_routes(camera,fd,routes,count);
            if (!ret && image_in) ret=set_pad(camera,fd,&image_input);
            if (!ret && image_in) ret=set_pad(camera,fd,&image_output);
            if (ret<0) return ret;
        } else if (stream) return -ENOTSUP;
        if (enable) {
            format.pad=in->sink.index; format.stream=stream;
            ret=set_pad(camera,fd,&format);
            if (ret<0) return ret;
            ret=get_pad(camera,fd,out->source.index,output_stream,&format);
            if (ret<0) return ret;
            if (format.format.code!=sensor->embedded.mbus_code || format.format.width!=sensor->embedded.width ||
                format.format.height!=sensor->embedded.height) return -EPIPE;
        }
        stream=output_stream;
    }
    return 0;
}
static uint32_t sample_bits(uint32_t code)
{
    switch (code) {
#define DEPTH(order,bits) case MEDIA_BUS_FMT_S##order##bits##_1X##bits: return bits;
#define ORDER(order) DEPTH(order,8) DEPTH(order,10) DEPTH(order,12) DEPTH(order,14) DEPTH(order,16)
    ORDER(BGGR) ORDER(GBRG) ORDER(GRBG) ORDER(RGGB)
#undef ORDER
#undef DEPTH
    case MEDIA_BUS_FMT_Y8_1X8: return 8;
    case MEDIA_BUS_FMT_Y10_1X10: return 10;
    case MEDIA_BUS_FMT_Y12_1X12: return 12;
    case MEDIA_BUS_FMT_Y14_1X14: return 14;
    case MEDIA_BUS_FMT_Y16_1X16: return 16;
    default: return 0;
    }
}
int cc_camera_configure_embedded(cc_camera *camera, cc_embedded_configuration *configuration)
{
    if (!camera) return -EINVAL;
    if (!camera->media || camera->sensor_fd<0 || camera->info.pipeline==CC_PIPELINE_VIMC) return -ENOTSUP;
    if (camera->stream->allocated || camera->stream->streaming) return -EBUSY;
    if (!camera->stream->configured) return -EPIPE;
    if (!configuration && !camera->embedded) return 0;
    cc_sensor_streams sensor;
    int ret=cc_camera_get_sensor_streams(camera,&sensor);
    if (ret<0) return ret;
    if (camera->embedded) {
        struct cc_embedded_capture *old=camera->embedded;
        ret=configure_path(camera,old,&sensor,false);
        camera->embedded=NULL; cc_embedded_close(old);
        if (ret<0) { camera->stream->configured=false; return ret; }
    }
    if (!configuration) return 0;
    if (!(sensor.flags&CC_SENSOR_STREAMS_EMBEDDED)) return -ENOTSUP;
    const char *name=camera->info.pipeline==CC_PIPELINE_RPI_CFE ? "rp1-cfe-csi2-ch1":"unicam-embedded";
    struct cc_entity *node=NULL;
    for (size_t i=0;i<camera->media->num_entities;++i) {
        struct cc_entity *candidate=&camera->media->entities[i];
        if (strcmp(cc_entity_name(candidate),name)) continue;
        if (node) return -ENOTUNIQ;
        node=candidate;
    }
    if (!node || !node->path[0]) return -ENOENT;
    uint32_t bpp, fourcc;
    bool legacy=sensor.embedded.mbus_code==0x7001;
    if (legacy) { bpp=sample_bits(sensor.image.mbus_code); fourcc=CC_FOURCC('S','E','N','S'); }
    else if (sensor.embedded.mbus_code>=0x8001 && sensor.embedded.mbus_code<=0x8004) {
        bpp=8+(sensor.embedded.mbus_code-0x8001)*2;
        fourcc=bpp==8 ? CC_FOURCC('M','E','T','8'):CC_FOURCC('M','C','1','A'+bpp-10);
    } else return -ENOTSUP;
    if (!bpp) return -ENOTSUP;
    if (!sensor.embedded.width || !sensor.embedded.height) return -ENODATA;
    uint64_t stride=legacy ? sensor.embedded.width:((uint64_t)sensor.embedded.width*bpp+7)/8;
    if (stride>UINT32_MAX) return -EOVERFLOW;
    uint64_t capacity=stride*sensor.embedded.height;
    if (capacity>UINT32_MAX) return -EOVERFLOW;
    struct cc_embedded_capture *e=calloc(1,sizeof(*e));
    if (!e) return -ENOMEM;
    reset(e);
    ret=cc_media_route_pad(camera->media,camera->info.sensor_entity,sensor.embedded.pad,node->desc.id,&e->path,&e->path_count);
    if (ret<0) goto fail;
    /* Reject a path that would displace the image's physical input. */
    for (size_t i=0;i<e->path_count;++i) for (size_t j=0;j<camera->route_count;++j) {
        const struct media_link_desc *a=&camera->media->links[e->path[i]], *b=&camera->media->links[camera->route[j]];
        if (e->path[i]!=camera->route[j] && a->sink.entity==b->sink.entity && a->sink.index==b->sink.index) {
            ret=-ENOTSUP; goto fail;
        }
    }
    ret=cc_video_open_io(node->path,&camera->io,&e->device);
    if (!ret) ret=cc_video_create_stream(e->device,CC_STREAM_META_CAPTURE,&e->stream);
    if (ret<0) goto fail;
    cc_format format={.fourcc=fourcc,.num_planes=1,.planes={{0,(uint32_t)capacity}}};
    if (!legacy) { format.width=sensor.embedded.width; format.height=sensor.embedded.height; format.planes[0].stride=(uint32_t)stride; }
    ret=cc_stream_configure(e->stream,&format);
    if (ret<0) goto fail;
    if (format.fourcc!=fourcc || format.planes[0].size<capacity ||
        (!legacy && (format.width!=sensor.embedded.width || format.height!=sensor.embedded.height || format.planes[0].stride<stride))) {
        ret=-EPIPE; goto fail;
    }
    if (legacy) { format.width=sensor.embedded.width; format.height=sensor.embedded.height; format.planes[0].stride=(uint32_t)stride; }
    e->configuration=(cc_embedded_configuration){.format=format,.bits_per_pixel=bpp};
    ret=configure_path(camera,e,&sensor,true);
    if (ret<0) {
        (void)configure_path(camera,e,&sensor,false); camera->stream->configured=false;
        goto fail;
    }
    camera->embedded=e; *configuration=e->configuration;
    return 0;
fail:
    cc_embedded_close(e);
    return ret;
}
int cc_camera_get_embedded_configuration(cc_camera *camera, cc_embedded_configuration *configuration)
{
    if (!camera || !configuration) return -EINVAL;
    if (!camera->embedded) return -ENOENT;
    *configuration=camera->embedded->configuration; return 0;
}
int cc_camera_embedded_fd(const cc_camera *camera)
{
    return !camera ? -EINVAL:!camera->embedded ? -ENOENT:cc_stream_fd(camera->embedded->stream);
}
int cc_camera_embedded_frame(cc_camera *camera, uint32_t index, cc_frame *frame)
{
    if (!camera || !frame) return -EINVAL;
    struct cc_embedded_capture *e=camera->embedded;
    if (!e) return -ENOENT;
    if (index>=e->source_count) return -EINVAL;
    if (e->associated[index]<0) return -ENODATA;
    return cc_stream_buffer(e->stream,(uint32_t)e->associated[index],frame);
}
int cc_camera_export_embedded_plane(cc_camera *camera, uint32_t index, uint32_t plane)
{
    if (!camera) return -EINVAL;
    struct cc_embedded_capture *e=camera->embedded;
    if (!e) return -ENOENT;
    if (index>=e->source_count) return -EINVAL;
    if (e->associated[index]<0) return -ENODATA;
    return cc_stream_export_plane(e->stream,(uint32_t)e->associated[index],plane);
}
int cc_embedded_allocate(struct cc_embedded_capture *e, uint32_t *count)
{
    uint32_t n=*count<256 ? *count+1:256;
    int ret=cc_stream_allocate_buffers(e->stream,CC_MEMORY_MMAP,&n);
    if (ret<0) return ret;
    if (n<2) { (void)cc_stream_free_buffers(e->stream); return -ENOSPC; }
    if (*count>=n) *count=n-1;
    e->source_count=*count; reset(e); return 0;
}
int cc_embedded_start(struct cc_embedded_capture *e)
{
    if (e->stream->streaming) return 0;
    if (!e->source_count) return -EPIPE;
    for (uint32_t i=0;i<e->stream->num_buffers;++i) {
        int ret=cc_stream_queue(e->stream,i,NULL);
        if (ret<0) return ret;
    }
    return cc_stream_start(e->stream);
}
int cc_embedded_stop(struct cc_embedded_capture *e)
{
    int ret=cc_stream_stop(e->stream);
    if (!ret) reset(e);
    return ret;
}
int cc_embedded_free(struct cc_embedded_capture *e)
{
    int ret=cc_embedded_stop(e);
    if (!ret) ret=cc_stream_free_buffers(e->stream);
    if (!ret) e->source_count=0;
    return ret;
}
void cc_embedded_close(struct cc_embedded_capture *e)
{
    if (!e) return;
    cc_stream_close(e->stream); cc_video_close(e->device); free(e->path); free(e);
}
int cc_embedded_release(struct cc_embedded_capture *e, uint32_t index)
{
    if (index>=e->source_count) return -EINVAL;
    if (e->image_pending && e->image.index==index) return -EBUSY;
    if (e->associated[index]<0) return 0;
    int ret=cc_stream_queue(e->stream,(uint32_t)e->associated[index],NULL);
    if (!ret) e->associated[index]=-1;
    return ret;
}
int cc_embedded_collect(struct cc_embedded_capture *e, const cc_frame *image, cc_frame *metadata)
{
    if (image->index>=e->source_count || e->associated[image->index]>=0) return -EPROTO;
    if (image->flags&V4L2_BUF_FLAG_ERROR) { *metadata=(cc_frame){0}; return 0; }
    for (uint32_t attempt=0;attempt<e->stream->num_buffers;++attempt) {
        cc_frame frame;
        if (e->future>=0) { frame=e->future_frame; e->future=-1; }
        else {
            int ret=cc_stream_dequeue(e->stream,&frame,0);
            if (ret<0) return ret;
            if (e->have_timestamp && frame.timestamp_ns<e->last_timestamp) return -EPROTO;
            e->have_timestamp=true; e->last_timestamp=frame.timestamp_ns;
        }
        uint32_t a=image->flags&V4L2_BUF_FLAG_TIMESTAMP_MASK,b=frame.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK;
        if (a && b && a!=b) return -EPROTO;
        if (!image->timestamp_ns != !frame.timestamp_ns) return -EPROTO;
        bool same,older;
        if (image->timestamp_ns) { same=frame.timestamp_ns==image->timestamp_ns; older=frame.timestamp_ns<image->timestamp_ns; }
        else {
            uint32_t distance=frame.sequence-image->sequence;
            if (distance==UINT32_C(0x80000000)) return -EPROTO;
            same=!distance; older=distance>INT32_MAX;
        }
        if (same) {
            e->associated[image->index]=(int)frame.index; *metadata=frame; return 0;
        }
        if (!older) {
            e->future=(int)frame.index; e->future_frame=frame; *metadata=(cc_frame){0}; return 0;
        }
        int ret=cc_stream_queue(e->stream,frame.index,NULL);
        if (ret<0) return ret;
    }
    return -EAGAIN;
}
