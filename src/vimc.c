/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Native C17 VIMC path, following the supplied libcamera pipeline handler.
 * Reference: Copyright (C) 2018 Google Inc. */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/vimc.h>
#include <errno.h>
#include <float.h>
#include <linux/media-bus-format.h>
#include <stdlib.h>
#include <string.h>

struct cc_vimc { cc_video_device *raw_device; cc_stream *raw; };
static int named(struct cc_media *media,const char *name,struct cc_entity **out)
{
    struct cc_entity *found=NULL;
    for (size_t i=0;i<media->num_entities;++i) if (!strcmp(cc_entity_name(&media->entities[i]),name)) {
        if (found) return -ENOTUNIQ;
        found=&media->entities[i];
    }
    if (!found) return -ENODEV;
    *out=found; return 0;
}
int cc_vimc_match(struct cc_media *media,struct cc_entity **sensor,struct cc_entity **video,struct cc_entity **raw)
{
    if (strncmp(media->info.driver,"vimc",sizeof(media->info.driver))) return -ENOTSUP;
    static const char *const names[]={"Raw Capture 0","Raw Capture 1","RGB/YUV Capture","Sensor A","Sensor B",
        "Debayer A","Debayer B","RGB/YUV Input","Scaler"};
    for (size_t i=0;i<sizeof(names)/sizeof(names[0]);++i) {
        struct cc_entity *entity; int ret=named(media,names[i],&entity);
        if (ret<0) return ret;
    }
    int ret=named(media,"Sensor B",sensor);
    if (!ret) ret=named(media,"RGB/YUV Capture",video);
    if (!ret) ret=named(media,"Raw Capture 1",raw);
    if (ret<0) return ret;
    if ((*sensor)->desc.type!=MEDIA_ENT_F_CAM_SENSOR || !(*sensor)->path[0] || !(*video)->path[0] || !(*raw)->path[0]) return -ENODEV;
    return 0;
}
int cc_vimc_route(struct cc_media *media,size_t **out,size_t *count)
{
    static const char *const names[]={"Sensor B","Debayer B","Scaler","RGB/YUV Capture"};
    struct cc_entity *entities[4];
    for (size_t i=0;i<4;++i) { int ret=named(media,names[i],&entities[i]); if (ret<0) return ret; }
    size_t path[3];
    for (size_t i=0;i<3;++i) {
        bool found=false;
        for (size_t j=0;j<media->num_links;++j) {
            const struct media_link_desc *link=&media->links[j];
            if (link->source.entity!=entities[i]->desc.id || link->source.index!=(i ? 1:0) ||
                link->sink.entity!=entities[i+1]->desc.id || link->sink.index) continue;
            if (found) return -ENOTUNIQ;
            path[i]=j; found=true;
        }
        if (!found) return -ENODEV;
    }
    size_t *route=malloc(sizeof(path));
    if (!route) return -ENOMEM;
    memcpy(route,path,sizeof(path)); *out=route; *count=3; return 0;
}
int cc_vimc_open(cc_camera *camera)
{
    struct cc_entity *sensor,*video,*raw;
    int ret=cc_vimc_match(camera->media,&sensor,&video,&raw);
    if (ret<0) return ret;
    struct cc_vimc *v=calloc(1,sizeof(*v));
    if (!v) return -ENOMEM;
    camera->vimc=v;
    ret=cc_video_open_io(raw->path,&camera->io,&v->raw_device);
    if (ret<0) return ret;
    if (v->raw_device->capabilities&(V4L2_CAP_VIDEO_M2M|V4L2_CAP_VIDEO_M2M_MPLANE)) return -ENOTSUP;
    enum cc_stream_type type=v->raw_device->capabilities&V4L2_CAP_VIDEO_CAPTURE_MPLANE ? CC_STREAM_CAPTURE_MPLANE:CC_STREAM_CAPTURE;
    return cc_video_create_stream(v->raw_device,type,&v->raw);
}
void cc_vimc_close(cc_camera *camera)
{
    if (!camera->vimc) return;
    cc_stream_close(camera->vimc->raw); cc_video_close(camera->vimc->raw_device); free(camera->vimc); camera->vimc=NULL;
}
static uint32_t supported_fourcc(const cc_camera *camera,uint32_t index)
{
    if (!index) return V4L2_PIX_FMT_RGB24;
    if (index==1 && camera->media->info.media_version>=UINT32_C(0x050700)) return V4L2_PIX_FMT_BGR24;
    return 0;
}
int cc_vimc_enum_format(cc_camera *camera,uint32_t index,cc_format_info *out)
{
    if (!out) return -EINVAL;
    uint32_t fourcc=supported_fourcc(camera,index);
    if (!fourcc) return -ENOENT;
    cc_format_info format={.fourcc=fourcc};
    strcpy(format.description,index ? "VIMC BGR 8:8:8":"VIMC RGB 8:8:8"); *out=format; return 0;
}
int cc_vimc_enum_size(cc_camera *camera,uint32_t fourcc,uint32_t index,cc_size_range *size)
{
    if (!size) return -EINVAL;
    if (index || !fourcc || (fourcc!=supported_fourcc(camera,0) && fourcc!=supported_fourcc(camera,1))) return -ENOENT;
    uint32_t alignment=camera->media->info.media_version<UINT32_C(0x051000) ? 6U:2U;
    *size=(cc_size_range){alignment*8,4096/alignment*alignment,alignment,alignment*8,2160,alignment}; return 0;
}
static int pad_format(cc_camera *camera,int fd,uint32_t pad,bool active,uint32_t code,uint32_t width,uint32_t height,
                       struct cc_subdev_format *actual)
{
    uint32_t which=active ? V4L2_SUBDEV_FORMAT_ACTIVE:V4L2_SUBDEV_FORMAT_TRY;
    struct cc_subdev_format format={.which=which,.pad=pad,.format={.code=code,.width=width,.height=height,.field=V4L2_FIELD_NONE,
        .colorspace=code==MEDIA_BUS_FMT_SGRBG8_1X8 ? V4L2_COLORSPACE_RAW:V4L2_COLORSPACE_SRGB}};
    int ret=cc_ioctl(&camera->io,fd,VIDIOC_SUBDEV_S_FMT,&format);
    if (ret<0) return ret;
    if (format.which!=which || format.pad!=pad || format.stream || format.format.code!=code ||
        format.format.width!=width || format.format.height!=height) return -EPIPE;
    *actual=format; return 0;
}
static int configure_links(cc_camera *camera)
{
    /* Match the upstream single processed camera: no mutable competing paths
     * remain enabled while Sensor B feeds the scaler. Restoration uses the
     * common camera log and happens on close. */
    for (int phase=0;phase<2;++phase) for (size_t i=0;i<camera->media->num_links;++i) {
        const struct media_link_desc *link=&camera->media->links[i];
        bool selected=false;
        for (size_t j=0;j<camera->route_count;++j) if (camera->route[j]==i) selected=true;
        if (selected!=(phase==1) || ((link->flags&MEDIA_LNK_FL_IMMUTABLE) && !selected)) continue;
        int ret=cc_camera_change_link(camera,i,selected ? link->flags|MEDIA_LNK_FL_ENABLED:link->flags&~MEDIA_LNK_FL_ENABLED);
        if (ret<0) return ret;
        if (((camera->media->links[i].flags&MEDIA_LNK_FL_ENABLED)!=0)!=selected) return -EPIPE;
    }
    return 0;
}
int cc_vimc_negotiate(cc_camera *camera,cc_format *format,bool active)
{
    cc_size_range range;
    if (format->fourcc!=supported_fourcc(camera,0) && format->fourcc!=supported_fourcc(camera,1)) format->fourcc=V4L2_PIX_FMT_RGB24;
    int ret=cc_vimc_enum_size(camera,format->fourcc,0,&range);
    if (ret<0) return ret;
    uint32_t width=format->width<range.min_width ? range.min_width:format->width>range.max_width ? range.max_width:format->width;
    uint32_t height=format->height<range.min_height ? range.min_height:format->height>range.max_height ? range.max_height:format->height;
    width=width/range.width_step*range.width_step; height=height/range.height_step*range.height_step;
    cc_format requested={.fourcc=format->fourcc,.width=width,.height=height,.field=V4L2_FIELD_NONE,.colorspace=V4L2_COLORSPACE_SRGB,.num_planes=1};
    if (!active) {
        ret=cc_stream_try_format(camera->stream,&requested);
        if (ret<0) return ret;
        if (requested.width!=width || requested.height!=height || requested.fourcc!=format->fourcc) return -EPIPE;
        *format=requested; return 0;
    }
    ret=configure_links(camera);
    if (ret<0) return ret;
    uint32_t sensor_width=camera->media->info.media_version<UINT32_C(0x051000) ? width/3:1920;
    uint32_t sensor_height=camera->media->info.media_version<UINT32_C(0x051000) ? height/3:1080;
    struct cc_subdev_format pad;
    ret=pad_format(camera,camera->sensor_fd,0,true,MEDIA_BUS_FMT_SGRBG8_1X8,sensor_width,sensor_height,&pad);
    if (ret<0) return ret;
    struct cc_entity *debayer,*scaler;
    ret=named(camera->media,"Debayer B",&debayer);
    if (!ret) ret=named(camera->media,"Scaler",&scaler);
    if (ret<0) return ret;
    int debayer_fd=camera->subdev_fds[debayer-camera->media->entities],scaler_fd=camera->subdev_fds[scaler-camera->media->entities];
    uint32_t code=format->fourcc==V4L2_PIX_FMT_RGB24 ? MEDIA_BUS_FMT_RGB888_1X24:MEDIA_BUS_FMT_BGR888_1X24;
    ret=pad_format(camera,debayer_fd,0,true,MEDIA_BUS_FMT_SGRBG8_1X8,sensor_width,sensor_height,&pad);
    if (!ret) ret=pad_format(camera,debayer_fd,1,true,code,sensor_width,sensor_height,&pad);
    if (!ret) ret=pad_format(camera,scaler_fd,0,true,code,sensor_width,sensor_height,&pad);
    if (ret<0) return ret;
    if (camera->media->info.media_version>=UINT32_C(0x050600)) {
        struct v4l2_subdev_selection crop={.which=V4L2_SUBDEV_FORMAT_ACTIVE,.pad=0,.target=V4L2_SEL_TGT_CROP,
            .r={.width=sensor_width,.height=sensor_height}};
        ret=cc_ioctl(&camera->io,scaler_fd,VIDIOC_SUBDEV_S_SELECTION,&crop);
        if (ret<0) return ret;
        if (crop.pad || crop.which!=V4L2_SUBDEV_FORMAT_ACTIVE || crop.target!=V4L2_SEL_TGT_CROP || crop.r.left || crop.r.top ||
            crop.r.width!=sensor_width || crop.r.height!=sensor_height) return -EPIPE;
    }
    ret=pad_format(camera,scaler_fd,1,true,code,width,height,&pad);
    if (ret<0) return ret;
    ret=cc_stream_configure(camera->stream,&requested);
    if (ret<0) return ret;
    if (requested.width!=width || requested.height!=height || requested.fourcc!=format->fourcc) {
        camera->stream->configured=false; return -EPIPE;
    }
    cc_format raw={.fourcc=V4L2_PIX_FMT_SGRBG8,.width=sensor_width,.height=sensor_height,.field=V4L2_FIELD_NONE,.num_planes=1};
    ret=cc_stream_configure(camera->vimc->raw,&raw);
    if (!ret && (raw.width!=sensor_width || raw.height!=sensor_height || raw.fourcc!=V4L2_PIX_FMT_SGRBG8)) ret=-EPIPE;
    if (ret<0) { camera->stream->configured=false; return ret; }
    *format=requested; return 0;
}
int cc_vimc_controls_to_native(const cc_vimc_controls *settings,cc_control *controls,size_t *count)
{
    if (!settings || !count || (settings->flags&~UINT32_C(7))) return -EINVAL;
    const double values[]={settings->brightness,settings->contrast,settings->saturation};
    const uint32_t ids[]={V4L2_CID_BRIGHTNESS,V4L2_CID_CONTRAST,V4L2_CID_SATURATION};
    cc_control native[3]; size_t n=0;
    for (size_t i=0;i<3;++i) if (settings->flags&(1U<<i)) {
        double v=values[i],minimum=i ? 0:-1,maximum=i ? 2:1;
        if (!(v>=-DBL_MAX && v<=DBL_MAX)) return -EINVAL;
        if (v<minimum) v=minimum;
        if (v>maximum) v=maximum;
        int64_t value=(int64_t)(v*128+(i ? 0:128)+0.5);
        native[n++]=(cc_control){.id=ids[i],.value=value>255 ? 255:value};
    }
    size_t capacity=*count;
    if (controls && capacity<n) { *count=n; return -ENOSPC; }
    if (controls) memcpy(controls,native,n*sizeof(*native));
    *count=n; return 0;
}
int cc_camera_set_vimc_controls(cc_camera *camera,cc_vimc_controls *settings)
{
    if (!camera || !settings) return -EINVAL;
    if (camera->info.pipeline!=CC_PIPELINE_VIMC) return -ENOTSUP;
    cc_control controls[3]; size_t n=3;
    int ret=cc_vimc_controls_to_native(settings,controls,&n);
    if (ret<0) return ret;
    ret=cc_controls_apply_io(&camera->io,camera->sensor_fd,controls,n,true);
    if (ret<0) return ret;
    cc_vimc_controls result={.flags=settings->flags};
    for (size_t i=0;i<n;++i) {
        if (controls[i].value<0 || controls[i].value>255) return -EPROTO;
        double value=(double)controls[i].value/128;
        if (controls[i].id==V4L2_CID_BRIGHTNESS) result.brightness=value-1;
        else if (controls[i].id==V4L2_CID_CONTRAST) result.contrast=value;
        else result.saturation=value;
    }
    *settings=result; return 0;
}
