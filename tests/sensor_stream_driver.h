/* Shared native sensor/receiver test driver. */
#ifndef CCAMERA_TEST_SENSOR_STREAM_DRIVER_H
#define CCAMERA_TEST_SENSOR_STREAM_DRIVER_H
/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/capture.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
enum { SENSOR=11, RECEIVER=12, VIDEO=20 };
struct fake {
    cc_camera camera;
    struct cc_media media;
    struct cc_entity entities[3];
    struct media_pad_desc sensor_pads[4], receiver_pads[2], video_pad;
    struct media_link_desc links[2];
    size_t path[2]; int fds[3]; bool streams[3], changed[2]; uint32_t saved[2];
    cc_video_device video; cc_stream stream;
    struct cc_subdev_route routes[2][4]; uint32_t count[2], image[2];
    struct v4l2_mbus_framefmt receiver_format[2]; struct v4l2_format video_format;
    bool legacy, fixed_sensor, metadata, duplicate_image, duplicate_embedded, mixed_codes, endless_codes;
    bool bad_format_stream, bad_code_stream, bad_get_pointer, bad_get_count, empty_routes, bad_set, reverse_set;
    bool bad_receiver, receiver_fixed;
    unsigned long fail_request; int fail_fd;
    unsigned sensor_sets, receiver_sets, current_gets, old_gets, current_sets, old_sets, receiver_routes;
    uint32_t received_stream, enum_stream, size_stream;
    unsigned opened, closed;
    bool reject_client;
    bool bad_interval, interval_modern;
    uint32_t interval_stream;
};
static int code_kind(struct fake *f, uint32_t which, uint32_t pad, uint32_t stream)
{
    if (f->fixed_sensor) {
        if (pad==0 && stream==0) return 1;
        if (pad==1 && stream==0 && f->metadata) return 2;
        if (pad==2 && stream==0 && f->duplicate_embedded) return 2;
        return 0;
    }
    if (pad==2 && stream==f->image[which]) return 1;
    if (pad==2 && stream==42) return 2;
    if (pad==1 && stream==0) return 3;
    if (pad==2 && stream==99) return f->duplicate_image ? 1:2;
    if (pad==3 && stream==0) return f->duplicate_embedded ? 3:2;
    return 0;
}
static int fake_ioctl(void *context, int fd, unsigned long request, void *argument)
{
    struct fake *f=context;
    if (request==f->fail_request && (!f->fail_fd || f->fail_fd==fd)) return -EIO;
    if (request==CC_SD_G_FRAME_INTERVAL || request==CC_SD_S_FRAME_INTERVAL) {
        CHECK(fd==SENSOR); struct cc_subdev_frame_interval *q=argument;
        CHECK(q->pad==f->camera.sensor_pad && !q->which);
        f->interval_stream=q->stream;
        q->interval=(struct v4l2_fract){1001,request==CC_SD_G_FRAME_INTERVAL ? 30000U:24000U};
        q->which=f->interval_modern ? V4L2_SUBDEV_FORMAT_ACTIVE:0;
        if (f->bad_interval) ++q->stream;
        return 0;
    }
    if (request==CC_SD_ENUM_FRAME_INTERVAL) {
        CHECK(fd==SENSOR); struct cc_subdev_frame_interval_enum *q=argument;
        CHECK(q->pad==f->camera.sensor_pad && q->which==V4L2_SUBDEV_FORMAT_ACTIVE && q->code && q->width && q->height);
        f->interval_stream=q->stream;
        if (q->index) return -EINVAL;
        q->interval=(struct v4l2_fract){1,30}; if (f->bad_interval) ++q->width; return 0;
    }
    if (request==MEDIA_IOC_DEVICE_INFO) {
        CHECK(fd==10); struct media_device_info *info=argument; strcpy(info->driver,"rp1-cfe"); return 0;
    }
    if (request==MEDIA_IOC_ENUM_ENTITIES) {
        CHECK(fd==10); struct media_entity_desc *entity=argument;
        uint32_t previous=entity->id&~MEDIA_ENT_ID_FLAG_NEXT;
        if (previous>=3) return -EINVAL;
        *entity=f->entities[previous].desc; entity->links=previous==1 ? 2:1;
        entity->dev.major=81; entity->dev.minor=previous==0 ? SENSOR:previous==1 ? RECEIVER:VIDEO;
        if (previous==0) strcpy(entity->name,"imx708");
        if (previous==2) strcpy(entity->name,"rp1-cfe-csi2-ch0");
        return 0;
    }
    if (request==MEDIA_IOC_ENUM_LINKS) {
        CHECK(fd==10); struct media_links_enum *links=argument;
        CHECK(links->entity>=1 && links->entity<=3);
        struct cc_entity *entity=&f->entities[links->entity-1];
        memcpy(links->pads,entity->pads,entity->desc.pads*sizeof(*entity->pads));
        size_t n=0;
        for (size_t i=0;i<2;++i)
            if (f->links[i].source.entity==links->entity || f->links[i].sink.entity==links->entity)
                links->links[n++]=f->links[i];
        return 0;
    }
    if (request==VIDIOC_QUERYCAP) {
        CHECK(fd==VIDEO); struct v4l2_capability *caps=argument;
        caps->capabilities=V4L2_CAP_STREAMING|V4L2_CAP_VIDEO_CAPTURE; return 0;
    }
    if (request==CC_SD_QUERYCAP) {
        CHECK(fd==SENSOR || fd==RECEIVER); struct cc_subdev_capability *caps=argument;
        caps->capabilities=(fd==SENSOR && f->fixed_sensor) ? 0U:CC_SD_CAP_STREAMS; return 0;
    }
    if (request==CC_SD_S_CLIENT_CAP) {
        CHECK(fd==SENSOR || fd==RECEIVER); struct cc_subdev_client_capability *client=argument;
        CHECK(client->capabilities==1); if (f->reject_client) client->capabilities=0; return 0;
    }
    if (request==CC_SD_G_ROUTING || request==CC_SD_G_ROUTING_LEGACY) {
        CHECK(fd==SENSOR && !f->fixed_sensor);
        uint32_t which,capacity; uint64_t pointer;
        if (request==CC_SD_G_ROUTING) {
            ++f->current_gets; if (f->legacy) return -ENOTTY;
            struct cc_subdev_routing *r=argument;
            which=r->which; capacity=r->len_routes; pointer=r->routes;
            r->num_routes=f->bad_get_count ? capacity+1:f->empty_routes ? 0:f->count[which];
            if (f->bad_get_pointer) r->routes=0;
        } else {
            ++f->old_gets; CHECK(f->legacy);
            struct cc_subdev_routing_legacy *r=argument;
            which=r->which; capacity=r->num_routes; pointer=r->routes;
            r->num_routes=f->bad_get_count ? capacity+1:f->empty_routes ? 0:f->count[which];
            if (f->bad_get_pointer) r->routes=0;
        }
        CHECK(which<=1 && capacity>=f->count[which]);
        memcpy((void *)(uintptr_t)pointer,f->routes[which],f->count[which]*sizeof(f->routes[which][0]));
        return 0;
    }
    if (request==CC_SD_S_ROUTING || request==CC_SD_S_ROUTING_LEGACY) {
        uint32_t which,count; uint64_t pointer;
        if (request==CC_SD_S_ROUTING) {
            ++f->current_sets; if (f->legacy) return -ENOTTY;
            struct cc_subdev_routing *r=argument; which=r->which; count=r->num_routes; pointer=r->routes;
            CHECK(r->len_routes==count);
        } else {
            ++f->old_sets; CHECK(f->legacy);
            struct cc_subdev_routing_legacy *r=argument; which=r->which; count=r->num_routes; pointer=r->routes;
        }
        CHECK(which<=1); struct cc_subdev_route *r=(void *)(uintptr_t)pointer;
        if (fd==SENSOR) {
            CHECK(count==3 && !f->fixed_sensor);
            CHECK(r[0].source_stream==f->image[which] && (r[0].flags&CC_SD_ROUTE_ACTIVE));
            CHECK(r[1].source_stream==42 && !(r[1].flags&CC_SD_ROUTE_ACTIVE));
            CHECK(r[2].source_stream==99 && (r[2].flags&CC_SD_ROUTE_ACTIVE));
            if (f->reverse_set) { struct cc_subdev_route tmp=r[0]; r[0]=r[2]; r[2]=tmp; }
            if (f->bad_set) r[0].flags^=CC_SD_ROUTE_ACTIVE;
            memcpy(f->routes[which],r,count*sizeof(*r));
        } else {
            CHECK(fd==RECEIVER && count==1);
            CHECK(r[0].sink_pad==0 && r[0].sink_stream==f->image[which] && r[0].source_pad==1 &&
                  !r[0].source_stream && (r[0].flags&CC_SD_ROUTE_ACTIVE));
            f->received_stream=r[0].sink_stream; ++f->receiver_routes;
            if (f->bad_receiver) ++r[0].sink_stream;
        }
        return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_MBUS_CODE) {
        CHECK(fd==SENSOR); struct cc_subdev_mbus_code_enum *code=argument;
        CHECK(code->which<=1); f->enum_stream=code->stream;
        int type=code_kind(f,code->which,code->pad,code->stream);
        if (!type || (code->index && (type!=1 || code->index>1) && !f->endless_codes)) return -EINVAL;
        code->code=type==1 ? (code->index ? MEDIA_BUS_FMT_SRGGB12_1X12:MEDIA_BUS_FMT_SRGGB10_1X10):
            type==2 ? UINT32_C(0x8002):UINT32_C(0x9001);
        if (f->mixed_codes && type==1 && code->index==1) code->code=0x8002;
        if (f->bad_code_stream) ++code->stream;
        return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_FRAME_SIZE) {
        CHECK(fd==SENSOR); struct cc_subdev_frame_size_enum *size=argument;
        CHECK(size->pad==(f->fixed_sensor ? 0U:2U) && size->stream==f->image[size->which]);
        f->size_stream=size->stream;
        if (size->index) return -EINVAL;
        size->min_width=size->max_width=640; size->min_height=size->max_height=480;
        return 0;
    }
    if (request==VIDIOC_SUBDEV_G_FMT || request==VIDIOC_SUBDEV_S_FMT) {
        struct cc_subdev_format *format=argument; CHECK(format->which<=1);
        if (fd==SENSOR) {
            int type=code_kind(f,format->which,format->pad,format->stream);
            if (!type) return -EINVAL;
            if (request==VIDIOC_SUBDEV_S_FMT) { CHECK(type==1); ++f->sensor_sets; }
            format->format=(struct v4l2_mbus_framefmt){.code=type==1 ? MEDIA_BUS_FMT_SRGGB10_1X10:
                f->fixed_sensor ? UINT32_C(0x7001):type==2 ? UINT32_C(0x8002):UINT32_C(0x9001),
                .width=type==1 ? 640U:800U,.height=type==1 ? 480U:4U,.field=V4L2_FIELD_NONE};
            if (f->bad_format_stream) ++format->stream;
        } else {
            CHECK(fd==RECEIVER);
            if (request==VIDIOC_SUBDEV_S_FMT) {
                CHECK(!format->pad && format->stream==f->image[format->which]);
                f->receiver_format[format->which]=format->format; ++f->receiver_sets;
            } else {
                CHECK(format->pad==1 && !format->stream);
                format->format=f->receiver_format[format->which];
            }
        }
        return 0;
    }
    if (request==VIDIOC_S_FMT || request==VIDIOC_G_FMT || request==VIDIOC_TRY_FMT) {
        CHECK(fd==VIDEO); struct v4l2_format *format=argument;
        CHECK(format->type==V4L2_BUF_TYPE_VIDEO_CAPTURE);
        if (request!=VIDIOC_G_FMT) {
            format->fmt.pix.bytesperline=format->fmt.pix.width*2;
            format->fmt.pix.sizeimage=format->fmt.pix.bytesperline*format->fmt.pix.height;
            if (request==VIDIOC_S_FMT) f->video_format=*format;
        } else *format=f->video_format;
        return 0;
    }
    fprintf(stderr,"unexpected ioctl %lu on %d\n",request,fd); CHECK(false); return -ENOTTY;
}
static int fake_open(void *context, const char *path, int flags)
{
    struct fake *f=context; CHECK(flags&O_CLOEXEC);
    ++f->opened;
    if (!strcmp(path,"/media")) return 10;
    if (!strcmp(path,"/sensor")) return SENSOR;
    if (!strcmp(path,"/receiver")) return RECEIVER;
    CHECK(!strcmp(path,"/video")); return VIDEO;
}
static int fake_close(void *context, int fd)
{
    struct fake *f=context; CHECK(fd==10 || fd==SENSOR || fd==RECEIVER || fd==VIDEO); ++f->closed; return 0;
}
static int fake_lock(void *context, int fd) { (void)context; (void)fd; return 0; }
static int fake_resolve(void *context, unsigned major_id, unsigned minor_id, char *path, size_t size)
{
    (void)context; CHECK(major_id==81);
    const char *name=minor_id==SENSOR ? "/sensor":minor_id==RECEIVER ? "/receiver":"/video";
    CHECK(minor_id==SENSOR || minor_id==RECEIVER || minor_id==VIDEO);
    CHECK(strlen(name)<size); strcpy(path,name); return 0;
}
static const struct cc_io_ops ops={.ioctl=fake_ioctl,.open=fake_open,.close=fake_close,.lock=fake_lock,.resolve=fake_resolve};
static void initialize(struct fake *f, bool fixed)
{
    memset(f,0,sizeof(*f)); f->fixed_sensor=fixed; f->metadata=true;
    struct cc_io io={&ops,f};
    f->camera=(cc_camera){.io=io,.sensor_fd=SENSOR,.fd=VIDEO,.lens_fd=-1,.media=&f->media,
        .sensor_pad=fixed ? 0U:2U,.sensor_stream=fixed ? 0U:7U,.subdev_fds=f->fds,.subdev_streams=f->streams,
        .route=f->path,.route_count=2,.changed_links=f->changed,.saved_link_flags=f->saved,.stream=&f->stream,.device=&f->video};
    f->camera.info.sensor_entity=1; f->camera.info.pipeline=CC_PIPELINE_RPI_CFE;
    f->media=(struct cc_media){.io=io,.fd=10,.entities=f->entities,.num_entities=3,.links=f->links,.num_links=2};
    f->entities[0].desc=(struct media_entity_desc){.id=1,.pads=4,.type=MEDIA_ENT_F_CAM_SENSOR};
    f->entities[0].pads=f->sensor_pads; f->entities[1].desc=(struct media_entity_desc){.id=2,.pads=2};
    strcpy(f->entities[1].desc.name,"csi2"); f->entities[1].pads=f->receiver_pads;
    f->entities[2].desc=(struct media_entity_desc){.id=3,.pads=1}; f->entities[2].pads=&f->video_pad;
    for (uint16_t p=0;p<4;++p) f->sensor_pads[p]=(struct media_pad_desc){.entity=1,.index=p,
        .flags=fixed || p==2 ? MEDIA_PAD_FL_SOURCE:MEDIA_PAD_FL_SINK};
    f->receiver_pads[0]=(struct media_pad_desc){.entity=2,.index=0,.flags=MEDIA_PAD_FL_SINK};
    f->receiver_pads[1]=(struct media_pad_desc){.entity=2,.index=1,.flags=MEDIA_PAD_FL_SOURCE};
    f->video_pad=(struct media_pad_desc){.entity=3,.index=0,.flags=MEDIA_PAD_FL_SINK};
    f->links[0]=(struct media_link_desc){.source={.entity=1,.index=(uint16_t)f->camera.sensor_pad},
        .sink={.entity=2,.index=0},.flags=MEDIA_LNK_FL_ENABLED};
    f->links[1]=(struct media_link_desc){.source={.entity=2,.index=1},.sink={.entity=3,.index=0},.flags=MEDIA_LNK_FL_ENABLED};
    f->path[0]=0; f->path[1]=1; f->fds[0]=SENSOR; f->fds[1]=RECEIVER; f->fds[2]=-1;
    f->streams[0]=!fixed; f->streams[1]=true;
    f->video=(cc_video_device){.io=io,.fd=VIDEO,.capabilities=V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_STREAMING};
    f->stream=(cc_stream){.device=&f->video,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE};
    for (unsigned w=0;w<2;++w) {
        f->image[w]=fixed ? 0U:7U; f->count[w]=3;
        f->routes[w][0]=(struct cc_subdev_route){.sink_pad=0,.source_pad=2,.source_stream=7};
        f->routes[w][1]=(struct cc_subdev_route){.sink_pad=1,.source_pad=2,.source_stream=42,.flags=CC_SD_ROUTE_ACTIVE};
        f->routes[w][2]=(struct cc_subdev_route){.sink_pad=3,.source_pad=2,.source_stream=99,.flags=CC_SD_ROUTE_ACTIVE};
    }
}
static cc_format raw_format(void)
{
    return (cc_format){.fourcc=V4L2_PIX_FMT_SRGGB10P,.width=640,.height=480};
}

#endif
