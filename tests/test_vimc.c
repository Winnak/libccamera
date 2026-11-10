/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <ccamera/configuration.h>
#include "subdev_compat.h"
#include "media_v2_driver.h"
#include <ccamera/capture.h>
#include <ccamera/ipa_vimc.h>
#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <linux/media-bus-format.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
#define ERROR(x,e) CHECK((x)==-(e))
static const char *proxy_worker;
static void attach_ipa(cc_capture *capture)
{
    if (!proxy_worker) return;
    const char configuration[] = "VIMC mock configuration";
    cc_ipa_vimc_options controller = {configuration, sizeof(configuration), -1, CC_IPA_VIMC_INIT, CC_IPA_VIMC_FLAG2};
    cc_ipa_proxy_options options = {.worker_path = proxy_worker}; cc_ipa *ipa = NULL; uint32_t flags = 0;
    OK(cc_ipa_vimc_create_proxy(&controller, &options, &ipa, &flags)); CHECK(flags == CC_IPA_VIMC_FLAG1);
    OK(cc_capture_set_ipa(capture, &ipa)); CHECK(!ipa);
}
enum { MEDIA_FD=10, SENSOR_FD=12, DEBAYER_FD=14, RAW_FD=16, SCALER_FD=17, VIDEO_FD=18, SLOTS=3 };
static const char *const names[]={"Sensor A","Sensor B","Debayer A","Debayer B","Raw Capture 0",
    "Raw Capture 1","Scaler","RGB/YUV Capture","RGB/YUV Input"};
struct fake {
    uint32_t version, missing, unavailable, count, memory, sequence;
    bool v2, ready, streaming, queued[SLOTS], opened[256], clamp, bad_ack, bad_pad, bad_crop, bad_raw;
    int opens, closes, maps, next_fd, writes, starts, crops, formats, sets, raw_formats;
    int fail_fd, fail_nth, fail_open;
    unsigned long fail_request;
    struct media_link_desc links[8], original[8];
    struct v4l2_mbus_framefmt pads[9][2];
    struct v4l2_pix_format pixels[2];
    int32_t controls[3];
    void *data[SLOTS];
};
static struct fake fake_new(uint32_t version,bool v2)
{
    struct fake f={.version=version,.v2=v2,.ready=true,.next_fd=128,.controls={128,128,128}};
    const uint32_t endpoints[][4]={{1,0,3,0},{1,0,5,0},{2,0,4,0},{2,0,6,0},
        {3,1,7,0},{4,1,7,0},{9,0,7,0},{7,1,8,0}};
    for (size_t i=0;i<8;++i) {
        f.links[i]=(struct media_link_desc){.source={.entity=endpoints[i][0],.index=(uint16_t)endpoints[i][1],.flags=MEDIA_PAD_FL_SOURCE},
            .sink={.entity=endpoints[i][2],.index=(uint16_t)endpoints[i][3],.flags=MEDIA_PAD_FL_SINK}};
        if (i<4 || i==7) f.links[i].flags=MEDIA_LNK_FL_ENABLED|MEDIA_LNK_FL_IMMUTABLE;
        if (i==4) f.links[i].flags=MEDIA_LNK_FL_ENABLED;
    }
    memcpy(f.original,f.links,sizeof(f.links)); return f;
}
static bool two_pads(uint32_t id) { return id==3 || id==4 || id==7; }
static bool source_pad(uint32_t id,uint32_t pad) { return two_pads(id) ? pad==1 : id==1 || id==2 || id==9; }
static int fake_open(void *context,const char *path,int flags)
{
    struct fake *f=context; CHECK(flags&O_CLOEXEC);
    int fd=0;
    if (!strcmp(path,"/dev/media-vimc")) fd=MEDIA_FD;
    else for (uint32_t id=1;id<=9;++id) {
        char candidate[64]; snprintf(candidate,sizeof(candidate),"/dev/vimc-%u",id);
        if (!strcmp(candidate,path)) fd=MEDIA_FD+(int)id;
    }
    CHECK(fd && !f->opened[fd]);
    if (fd==f->fail_open) return -EACCES;
    f->opened[fd]=true; ++f->opens; return fd;
}
static int fake_close(void *context,int fd)
{
    struct fake *f=context; CHECK(fd>=0 && fd<256 && f->opened[fd]);
    f->opened[fd]=false; ++f->closes;
    if (fd==VIDEO_FD) { f->streaming=false; f->count=0; memset(f->queued,0,sizeof(f->queued)); }
    return 0;
}
static int fake_lock(void *context,int fd) { CHECK(((struct fake *)context)->opened[fd]); return 0; }
static int fake_resolve(void *context,unsigned int major,unsigned int minor,char *out,size_t size)
{
    struct fake *f=context; CHECK(major==81 && minor>=1 && minor<=9);
    if (minor==f->unavailable) return -ENOENT;
    snprintf(out,size,"/dev/vimc-%u",minor); return 0;
}
static int legacy(void *context,int fd,unsigned long request,void *arg)
{
    struct fake *f=context; CHECK(fd==MEDIA_FD);
    if (request==MEDIA_IOC_ENUM_ENTITIES) {
        struct media_entity_desc *e=arg; uint32_t id=(e->id&~MEDIA_ENT_ID_FLAG_NEXT)+1;
        if (id>9) return -EINVAL;
        *e=(struct media_entity_desc){.id=id,.type=id<=2 ? MEDIA_ENT_F_CAM_SENSOR:
            id==5 || id==6 || id==8 || id==9 ? MEDIA_ENT_F_IO_V4L:MEDIA_ENT_F_PROC_VIDEO_SCALER,
            .pads=two_pads(id) ? 2:1,.dev={81,id}};
        strcpy(e->name,id==f->missing ? "absent":names[id-1]);
        for (size_t i=0;i<8;++i) if (f->links[i].source.entity==id || f->links[i].sink.entity==id) ++e->links;
        return 0;
    }
    CHECK(request==MEDIA_IOC_ENUM_LINKS);
    struct media_links_enum *e=arg; uint32_t id=e->entity;
    for (uint16_t i=0;i<(two_pads(id) ? 2:1);++i)
        e->pads[i]=(struct media_pad_desc){.entity=id,.index=i,.flags=source_pad(id,i) ? MEDIA_PAD_FL_SOURCE:MEDIA_PAD_FL_SINK};
    size_t n=0;
    for (size_t i=0;i<8;++i) if (f->links[i].source.entity==id || f->links[i].sink.entity==id) e->links[n++]=f->links[i];
    return 0;
}
static int fake_ioctl(void *context,int fd,unsigned long request,void *arg)
{
    struct fake *f=context; CHECK(f->opened[fd]);
    if (request==f->fail_request && (!f->fail_fd || fd==f->fail_fd) && --f->fail_nth==0) return -EIO;
    if (request==MEDIA_IOC_DEVICE_INFO) {
        struct media_device_info *info=arg; CHECK(fd==MEDIA_FD); memset(info,0,sizeof(*info));
        strcpy(info->driver,"vimc"); strcpy(info->bus_info,"platform:vimc"); info->media_version=f->version; return 0;
    }
    if (request==MEDIA_IOC_G_TOPOLOGY) return f->v2 ? test_v2_topology(context,fd,request,arg,legacy,false):-ENOTTY;
    if (request==MEDIA_IOC_ENUM_ENTITIES || request==MEDIA_IOC_ENUM_LINKS) { CHECK(!f->v2); return legacy(context,fd,request,arg); }
    if (request==MEDIA_IOC_SETUP_LINK) {
        struct media_link_desc *link=arg; CHECK(fd==MEDIA_FD);
        for (size_t i=0;i<8;++i) if (link->source.entity==f->links[i].source.entity && link->sink.entity==f->links[i].sink.entity) {
            CHECK(!(f->links[i].flags&MEDIA_LNK_FL_IMMUTABLE)); f->links[i].flags=link->flags; ++f->sets; return 0;
        }
        CHECK(false);
    }
    if (request==CC_SD_QUERYCAP) return -ENOTTY;
    if (request==VIDIOC_SUBDEV_G_FMT) {
        struct cc_subdev_format *q=arg;
        CHECK(fd==SENSOR_FD && !q->pad && !q->stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        q->format=(struct v4l2_mbus_framefmt){.code=MEDIA_BUS_FMT_SGRBG8_1X8,.width=640,.height=480}; return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_MBUS_CODE) {
        struct cc_subdev_mbus_code_enum *q=arg;
        CHECK(fd==SENSOR_FD && !q->pad && !q->stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        if (q->index) return -EINVAL;
        q->code=MEDIA_BUS_FMT_SGRBG8_1X8; return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_FRAME_SIZE) {
        struct cc_subdev_frame_size_enum *q=arg;
        CHECK(fd==SENSOR_FD && !q->pad && !q->stream && q->code==MEDIA_BUS_FMT_SGRBG8_1X8 && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        if (q->index) return -EINVAL;
        q->min_width=16; q->max_width=4096; q->min_height=16; q->max_height=2160; return 0;
    }
    if (request==VIDIOC_SUBDEV_S_FMT) {
        struct cc_subdev_format *format=arg;
        CHECK((fd==SENSOR_FD || fd==DEBAYER_FD || fd==SCALER_FD) && format->which==V4L2_SUBDEV_FORMAT_ACTIVE && !format->stream);
        CHECK(format->pad<(fd==SENSOR_FD ? 1U:2U));
        if (fd==SENSOR_FD || (fd==DEBAYER_FD && !format->pad)) CHECK(format->format.code==MEDIA_BUS_FMT_SGRBG8_1X8);
        else CHECK(format->format.code==MEDIA_BUS_FMT_RGB888_1X24 || format->format.code==MEDIA_BUS_FMT_BGR888_1X24);
        f->pads[fd-MEDIA_FD-1][format->pad]=format->format; ++f->formats;
        if (f->bad_pad) ++format->format.width;
        return 0;
    }
    if (request==VIDIOC_SUBDEV_S_SELECTION) {
        struct v4l2_subdev_selection *crop=arg;
        CHECK(fd==SCALER_FD && f->version>=0x050600 && !crop->pad && crop->which==V4L2_SUBDEV_FORMAT_ACTIVE &&
            crop->target==V4L2_SEL_TGT_CROP && !crop->r.left && !crop->r.top);
        CHECK(crop->r.width==f->pads[6][0].width && crop->r.height==f->pads[6][0].height); ++f->crops;
        if (f->bad_crop) crop->r.left=1;
        return 0;
    }
    if (request==VIDIOC_QUERYCAP) {
        struct v4l2_capability *cap=arg; CHECK(fd==VIDEO_FD || fd==RAW_FD); memset(cap,0,sizeof(*cap));
        strcpy((char *)cap->driver,"vimc"); strcpy((char *)cap->card,"VIMC capture");
        cap->capabilities=V4L2_CAP_STREAMING|V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_IO_MC; return 0;
    }
    if (request==VIDIOC_S_FMT || request==VIDIOC_TRY_FMT || request==VIDIOC_G_FMT) {
        struct v4l2_format *v=arg; CHECK((fd==VIDEO_FD || fd==RAW_FD) && v->type==V4L2_BUF_TYPE_VIDEO_CAPTURE);
        size_t n=fd==RAW_FD;
        if (request==VIDIOC_G_FMT) v->fmt.pix=f->pixels[n];
        else {
            struct v4l2_pix_format *p=&v->fmt.pix;
            CHECK(p->width && p->height && p->field==V4L2_FIELD_NONE);
            if (n) CHECK(p->pixelformat==V4L2_PIX_FMT_SGRBG8);
            else CHECK(p->pixelformat==V4L2_PIX_FMT_RGB24 || (f->version>=0x050700 && p->pixelformat==V4L2_PIX_FMT_BGR24));
            p->bytesperline=p->width*(n ? 1U:3U); p->sizeimage=p->bytesperline*p->height;
            p->colorspace=n ? V4L2_COLORSPACE_RAW:V4L2_COLORSPACE_SRGB;
            if (request==VIDIOC_S_FMT) {
                f->pixels[n]=*p;
                if (n) { ++f->raw_formats; if (f->bad_raw) ++p->width; }
            }
        }
        return 0;
    }
    if (request==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=arg; CHECK(fd==SENSOR_FD);
        if (q->id==V4L2_CID_CAMERA_ORIENTATION || q->id==V4L2_CID_CAMERA_SENSOR_ROTATION ||
            q->id==V4L2_CID_HFLIP || q->id==V4L2_CID_VFLIP) return -EINVAL;
        CHECK(q->id>=V4L2_CID_BRIGHTNESS && q->id<=V4L2_CID_SATURATION);
        *q=(struct v4l2_query_ext_ctrl){.id=q->id,.type=V4L2_CTRL_TYPE_INTEGER,.minimum=0,.maximum=255,.step=1,.default_value=128};
        strcpy(q->name,"VIMC image control"); return 0;
    }
    if (request==VIDIOC_S_EXT_CTRLS || request==VIDIOC_G_EXT_CTRLS) {
        struct v4l2_ext_controls *batch=arg; CHECK(fd==SENSOR_FD && batch->count<=3);
        for (uint32_t i=0;i<batch->count;++i) {
            struct v4l2_ext_control *c=&batch->controls[i];
            CHECK(c->id>=V4L2_CID_BRIGHTNESS && c->id<=V4L2_CID_SATURATION);
            size_t index=c->id-V4L2_CID_BRIGHTNESS;
            if (request==VIDIOC_S_EXT_CTRLS) {
                CHECK(c->value>=0 && c->value<=255); f->controls[index]=f->clamp && c->value>200 ? 200:c->value;
            }
            c->value=f->bad_ack ? 256:f->controls[index];
        }
        if (request==VIDIOC_S_EXT_CTRLS) ++f->writes;
        return 0;
    }
    CHECK(fd==VIDEO_FD); /* The raw tap is configured, never allocated/streamed. */
    if (request==VIDIOC_REQBUFS) {
        struct v4l2_requestbuffers *r=arg; CHECK(!f->streaming && r->type==V4L2_BUF_TYPE_VIDEO_CAPTURE);
        if (r->count>SLOTS) r->count=SLOTS;
        f->count=r->count; f->memory=r->memory; return 0;
    }
    if (request==VIDIOC_QUERYBUF) {
        struct v4l2_buffer *b=arg; CHECK(b->index<f->count && b->memory==V4L2_MEMORY_MMAP);
        b->length=f->pixels[0].sizeimage; b->m.offset=b->index*UINT32_C(0x1000000); return 0;
    }
    if (request==VIDIOC_EXPBUF) {
        struct v4l2_exportbuffer *e=arg; CHECK(e->index<f->count && !e->plane && e->flags&O_CLOEXEC);
        CHECK(f->next_fd<256); e->fd=f->next_fd++; f->opened[e->fd]=true; ++f->opens; return 0;
    }
    if (request==VIDIOC_QBUF) {
        struct v4l2_buffer *b=arg; CHECK(b->index<f->count && b->memory==f->memory && !f->queued[b->index]);
        if (b->memory==V4L2_MEMORY_DMABUF) CHECK(f->opened[b->m.fd] && b->length>=f->pixels[0].sizeimage);
        f->queued[b->index]=true; return 0;
    }
    if (request==VIDIOC_STREAMON) {
        CHECK(!f->streaming && f->raw_formats && (f->queued[0] || f->queued[1] || f->queued[2]));
        CHECK(!(f->links[4].flags&MEDIA_LNK_FL_ENABLED) && (f->links[5].flags&MEDIA_LNK_FL_ENABLED));
        CHECK(f->pixels[1].width==f->pads[1][0].width && f->pixels[1].height==f->pads[1][0].height);
        f->streaming=true; ++f->starts; return 0;
    }
    if (request==VIDIOC_STREAMOFF) { f->streaming=false; memset(f->queued,0,sizeof(f->queued)); return 0; }
    CHECK(request==VIDIOC_DQBUF);
    if (!f->streaming || !f->ready) return -EAGAIN;
    struct v4l2_buffer *b=arg;
    for (uint32_t i=0;i<f->count;++i) if (f->queued[i]) {
        f->queued[i]=false; b->index=i; b->bytesused=f->pixels[0].sizeimage; b->length=b->bytesused;
        b->sequence=f->sequence++; b->timestamp.tv_sec=1; b->timestamp.tv_usec=(long)b->sequence*1000;
        b->flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC; return 0;
    }
    return -EAGAIN;
}
static int fake_map(void *context,int fd,size_t length,off_t offset,void **out)
{
    struct fake *f=context; CHECK(fd==VIDEO_FD && length==f->pixels[0].sizeimage);
    size_t i=(size_t)offset/0x1000000; CHECK(i<SLOTS && !f->data[i]);
    f->data[i]=calloc(1,length); CHECK(f->data[i]); *out=f->data[i]; ++f->maps; return 0;
}
static int fake_unmap(void *context,void *data,size_t length)
{
    struct fake *f=context; CHECK(!f->streaming && length==f->pixels[0].sizeimage);
    for (size_t i=0;i<SLOTS;++i) if (f->data[i]==data) { free(data); f->data[i]=NULL; --f->maps; return 0; }
    CHECK(false); return -EINVAL;
}
static int fake_duplicate(void *context,int fd)
{
    struct fake *f=context; CHECK(f->opened[fd] && f->next_fd<256);
    int duplicate=f->next_fd++; f->opened[duplicate]=true; ++f->opens; return duplicate;
}
static int fake_now(void *context,uint64_t *now) { (void)context; *now=0; return 0; }
static int fake_device_number(void *context,int fd,unsigned int *major_id,unsigned int *minor_id)
{
    CHECK(((struct fake *)context)->opened[fd] && fd==SENSOR_FD);
    *major_id=81; *minor_id=2; return 0;
}
static int fake_canonical(void *context,const char *path,char *out,size_t capacity)
{
    (void)context;
    if (strcmp(path,"/sys/dev/char/81:2/device")) return -ENOENT;
    const char *physical="/sys/devices/platform/vimc.0";
    CHECK(strlen(physical)<capacity); memcpy(out,physical,strlen(physical)+1); return 0;
}
static int fake_read_text(void *context,const char *path,char *out,size_t capacity)
{
    (void)context; (void)path; (void)out; (void)capacity; return -ENOENT;
}
static const struct cc_io_ops ops={.open=fake_open,.close=fake_close,.ioctl=fake_ioctl,.map=fake_map,.unmap=fake_unmap,
    .lock=fake_lock,.resolve=fake_resolve,.duplicate=fake_duplicate,.now=fake_now,
    .device_number=fake_device_number,.canonical=fake_canonical,.read_text=fake_read_text};
static cc_camera_info camera_info(void)
{
    cc_camera_info info={.pipeline=CC_PIPELINE_VIMC,.sensor_entity=2};
    strcpy(info.media_path,"/dev/media-vimc"); strcpy(info.video_path,"/dev/vimc-8"); return info;
}
static cc_camera *open_camera(struct fake *f)
{
    struct cc_io io={&ops,f}; cc_camera_info info=camera_info(); cc_camera *camera=NULL;
    OK(cc_camera_open_io(&info,&io,&camera)); CHECK(camera && f->opened[RAW_FD]); return camera;
}
static void clean(const struct fake *f)
{
    CHECK(f->opens==f->closes && !f->maps && !f->streaming);
    CHECK(!memcmp(f->links,f->original,sizeof(f->links)));
}
static cc_format format_new(void) { return (cc_format){.fourcc=V4L2_PIX_FMT_RGB24,.width=64,.height=48}; }
static void test_formats(uint32_t version,bool v2)
{
    struct fake f=fake_new(version,v2); struct cc_io io={&ops,&f}; cc_camera_info *list=NULL; size_t n=0;
    OK(cc_enumerate_media_io(&io,"/dev/media-vimc",&list,&n));
    CHECK(n==1 && list[0].pipeline==CC_PIPELINE_VIMC && list[0].sensor_entity==2 && !strcmp(list[0].video_path,"/dev/vimc-8"));
    CHECK(list[0].id_type==CC_CAMERA_ID_PLATFORM && !strcmp(list[0].id,"platform/vimc.0 Sensor B"));
    cc_camera_list_free(list); clean(&f);
    cc_camera *camera=open_camera(&f); cc_format_info desc; cc_size_range sizes;
    OK(cc_camera_enum_format(camera,0,&desc)); CHECK(desc.fourcc==V4L2_PIX_FMT_RGB24);
    if (version<0x050700) ERROR(cc_camera_enum_format(camera,1,&desc),ENOENT);
    else { OK(cc_camera_enum_format(camera,1,&desc)); CHECK(desc.fourcc==V4L2_PIX_FMT_BGR24); }
    ERROR(cc_camera_enum_format(camera,2,&desc),ENOENT);
    ERROR(cc_camera_enum_size(camera,0,0,&sizes),ENOENT);
    OK(cc_camera_enum_size(camera,V4L2_PIX_FMT_RGB24,0,&sizes));
    uint32_t alignment=version<0x051000 ? 6:2;
    CHECK(sizes.min_width==alignment*8 && sizes.width_step==alignment && sizes.max_width==4096/alignment*alignment);
    for (uint32_t c=0;c<2;++c) {
        cc_format format={.width=UINT32_MAX,.height=1,.fourcc=c ? V4L2_PIX_FMT_BGR24:V4L2_PIX_FMT_YUYV};
        OK(cc_camera_try_format(camera,&format)); CHECK(!f.formats && !f.sets && !f.raw_formats);
        CHECK(format.width==sizes.max_width && format.height==sizes.min_height);
        CHECK(format.fourcc==(c && version>=0x050700 ? V4L2_PIX_FMT_BGR24:V4L2_PIX_FMT_RGB24));
    }
    for (uint32_t c=0;c<(version<0x050700 ? 1U:2U);++c) {
        cc_format format={.width=641,.height=481,.fourcc=c ? V4L2_PIX_FMT_BGR24:V4L2_PIX_FMT_RGB24};
        OK(cc_camera_configure(camera,&format)); CHECK(format.width==641/alignment*alignment && format.height==481/alignment*alignment);
        CHECK(format.colorspace==V4L2_COLORSPACE_SRGB && format.planes[0].stride==format.width*3);
        uint32_t sw=version<0x051000 ? format.width/3:1920, sh=version<0x051000 ? format.height/3:1080;
        CHECK(f.pads[1][0].width==sw && f.pads[1][0].height==sh && f.pixels[1].width==sw && f.pixels[1].height==sh);
        CHECK(f.pads[3][1].code==(c ? MEDIA_BUS_FMT_BGR888_1X24:MEDIA_BUS_FMT_RGB888_1X24));
        CHECK(f.pads[6][0].width==sw && f.pads[6][1].width==format.width);
    }
    CHECK((f.crops>0)==(version>=0x050600));
    struct v4l2_mbus_framefmt saved_pads[9][2]; memcpy(saved_pads,f.pads,sizeof(saved_pads));
    struct v4l2_pix_format saved_pixels[2]; memcpy(saved_pixels,f.pixels,sizeof(saved_pixels));
    cc_format proposed={.width=57,.height=53,.fourcc=V4L2_PIX_FMT_RGB24}; int mutations=f.formats+f.sets+f.raw_formats+f.crops;
    OK(cc_camera_try_format(camera,&proposed));
    CHECK(camera->stream->configured && mutations==f.formats+f.sets+f.raw_formats+f.crops);
    CHECK(!memcmp(saved_pads,f.pads,sizeof(saved_pads)) && !memcmp(saved_pixels,f.pixels,sizeof(saved_pixels)));
    cc_sensor_timing timing; ERROR(cc_camera_get_sensor_timing(camera,&timing),ENOTSUP);
    cc_embedded_configuration embedded={0}; ERROR(cc_camera_configure_embedded(camera,&embedded),ENOTSUP);
    cc_capture_devices devices={.camera=camera}; cc_pipeline_configuration configuration;
    const enum cc_stream_role roles[]={CC_ROLE_VIEWFINDER,CC_ROLE_RAW};
    CHECK(cc_pipeline_generate_configuration(&devices,roles,2,&configuration)>=0);
    CHECK(configuration.num_streams==1 && configuration.streams[0].format.fourcc==V4L2_PIX_FMT_RGB24 &&
        configuration.streams[0].format.width==1920 && configuration.streams[0].format.height==1080);
    CHECK(cc_pipeline_validate_configuration(&devices,&configuration)==CC_CONFIGURATION_VALID);
    configuration.streams[0].format=(cc_format){.fourcc=UINT32_MAX,.width=UINT32_MAX,.height=1};
    configuration.orientation=CC_ORIENTATION_ROTATE_180;
    CHECK(cc_pipeline_validate_configuration(&devices,&configuration)==CC_CONFIGURATION_ADJUSTED);
    CHECK(configuration.streams[0].format.width==sizes.max_width && configuration.streams[0].format.height==sizes.min_height &&
        configuration.orientation==CC_ORIENTATION_ROTATE_0);
    CHECK(cc_pipeline_configure(&devices,&configuration)>=0);
    cc_camera_close(camera); clean(&f);
}
static void test_controls(void)
{
    cc_vimc_controls settings={.flags=7,.brightness=-1,.contrast=2,.saturation=1};
    cc_control values[3]; size_t n=3; OK(cc_vimc_controls_to_native(&settings,values,&n));
    CHECK(n==3 && values[0].value==0 && values[1].value==255 && values[2].value==128);
    settings=(cc_vimc_controls){.flags=7,.brightness=DBL_MAX,.contrast=-DBL_MAX,.saturation=DBL_MAX};
    n=3; OK(cc_vimc_controls_to_native(&settings,values,&n)); CHECK(values[0].value==255 && values[1].value==0 && values[2].value==255);
    settings=(cc_vimc_controls){.flags=7,.brightness=1.0/256,.contrast=1.0/256,.saturation=3.0/256};
    n=3; OK(cc_vimc_controls_to_native(&settings,values,&n)); CHECK(values[0].value==129 && values[1].value==1 && values[2].value==2);
    cc_control previous[3]; memcpy(previous,values,sizeof(values)); n=2;
    ERROR(cc_vimc_controls_to_native(&settings,values,&n),ENOSPC); CHECK(n==3 && !memcmp(previous,values,sizeof(values)));
    uint64_t nan_bits=UINT64_C(0x7ff8000000000001); memcpy(&settings.brightness,&nan_bits,sizeof(nan_bits)); n=3;
    ERROR(cc_vimc_controls_to_native(&settings,values,&n),EINVAL); CHECK(n==3 && !memcmp(previous,values,sizeof(values)));
    settings.flags=6; OK(cc_vimc_controls_to_native(&settings,NULL,&n)); CHECK(n==2); /* Omitted NaN is irrelevant. */
    cc_request *request=NULL; OK(cc_request_create(0,&request));
    settings.flags=7; ERROR(cc_request_set_vimc_controls(request,&settings),EINVAL);
    settings.flags=8; ERROR(cc_request_set_vimc_controls(request,&settings),EINVAL);
    OK(cc_request_set_vimc_controls(request,NULL)); OK(cc_request_destroy(request));
    struct fake f=fake_new(0x060c00,true); cc_camera *camera=open_camera(&f); f.clamp=true;
    settings=(cc_vimc_controls){.flags=7,.brightness=1,.contrast=0.5,.saturation=2};
    OK(cc_camera_set_vimc_controls(camera,&settings));
    CHECK(settings.brightness==0.5625 && settings.contrast==0.5 && settings.saturation==1.5625 && f.writes==1);
    cc_vimc_controls original=settings; f.bad_ack=true;
    ERROR(cc_camera_set_vimc_controls(camera,&settings),EPROTO); CHECK(!memcmp(&settings,&original,sizeof(settings)));
    cc_camera_close(camera); clean(&f);
}
static void test_capture(bool imports,bool v2)
{
    struct fake f=fake_new(0x060c00,v2); cc_camera *camera=open_camera(&f); cc_format format=format_new();
    enum cc_orientation orientation=CC_ORIENTATION_ROTATE_90;
    OK(cc_camera_configure_oriented(camera,&format,&orientation)); CHECK(orientation==CC_ORIENTATION_ROTATE_0);
    uint32_t count=SLOTS;
    if (imports) {
        OK(cc_camera_prepare_imports(camera,&count));
        for (uint32_t i=0;i<count;++i) {
            int external=f.next_fd++; f.opened[external]=true; ++f.opens;
            cc_dmabuf_plane plane={external,format.planes[0].size}; OK(cc_camera_import_buffer(camera,i,&plane,1));
            OK(fake_close(&f,external)); /* Library owns a duplicate. */
        }
    } else {
        OK(cc_camera_allocate_buffers(camera,&count));
        int exported=cc_camera_export_plane(camera,0,0); CHECK(exported>=128); OK(fake_close(&f,exported));
    }
    ERROR(cc_camera_configure(camera,&format),EBUSY);
    cc_capture_devices devices={.camera=camera}; cc_capture *capture=NULL; OK(cc_capture_create(&devices,&capture,&count));
    attach_ipa(capture);
    cc_camera_properties properties; OK(cc_capture_get_properties(capture,&properties));
    CHECK(!strcmp(cc_capture_get_info(capture)->id,"platform/vimc.0 Sensor B"));
    OK(cc_capture_get_orientation(capture,&orientation)); CHECK(orientation==CC_ORIENTATION_ROTATE_0);
    CHECK(!strcmp(properties.model,"Sensor B") && properties.pixel_array_width==4096 &&
        properties.pixel_array_height==2160 && !(properties.flags&CC_CAMERA_PROPERTY_CFA));
    cc_request *requests[SLOTS]; const cc_capture_result *held=NULL; int before=f.writes;
    for (uint32_t i=0;i<SLOTS;++i) {
        OK(cc_request_create(100+i,&requests[i]));
        cc_vimc_controls controls={.flags=7,.brightness=(double)i/2-1,.contrast=1,.saturation=0.5};
        OK(cc_request_set_vimc_controls(requests[i],&controls)); controls.brightness=1; /* Setter copies. */
        OK(cc_capture_queue(capture,requests[i]));
        ERROR(cc_request_set_vimc_controls(requests[i],NULL),EBUSY);
    }
    f.ready=false; OK(cc_capture_dispatch(capture)); CHECK(f.writes==before+1);
    OK(cc_capture_dispatch(capture)); CHECK(f.writes==before+1); /* No retry write while waiting. */
    f.ready=true;
    for (uint32_t i=0;i<SLOTS;++i) {
        cc_request *done=NULL; OK(cc_capture_dequeue(capture,&done,0)); CHECK(done==requests[i]);
        const cc_capture_result *result=cc_request_get_result(done);
        CHECK(result && result->status==CC_REQUEST_COMPLETE && result->cookie==100+i && result->raw.num_planes==1);
        CHECK(result->vimc.flags==7 && result->vimc.brightness==(double)i/2-1 && result->vimc.contrast==1);
        CHECK((result->raw.planes[0].data!=NULL)==!imports);
        if (!i) held=result;
        CHECK(held->cookie==100 && held->vimc.brightness==-1);
    }
    CHECK(f.writes==before+SLOTS && f.starts==1);
    for (size_t i=0;i<SLOTS;++i) OK(cc_capture_release(capture,requests[i]));
    OK(cc_capture_stop(capture));
    OK(cc_request_set_vimc_controls(requests[0],NULL)); OK(cc_capture_queue(capture,requests[0]));
    cc_request *done=NULL; OK(cc_capture_dequeue(capture,&done,0));
    CHECK(cc_request_get_result(done)->vimc.flags==0 && f.writes==before+SLOTS && f.starts==2);
    OK(cc_capture_release(capture,done)); cc_capture_close(capture);
    for (size_t i=0;i<SLOTS;++i) OK(cc_request_destroy(requests[i]));
    clean(&f);
}
static void test_failures(void)
{
    for (uint32_t missing=1;missing<=9;++missing) {
        struct fake f=fake_new(0x060c00,true); f.missing=missing;
        struct cc_io io={&ops,&f}; cc_camera_info *list=NULL; size_t n=0;
        OK(cc_enumerate_media_io(&io,"/dev/media-vimc",&list,&n)); CHECK(!n); free(list);
        cc_camera_info info=camera_info(); cc_camera *camera=NULL;
        ERROR(cc_camera_open_io(&info,&io,&camera),ENODEV); CHECK(!camera); clean(&f);
    }
    const int failed_nodes[]={MEDIA_FD,SENSOR_FD,DEBAYER_FD,SCALER_FD,RAW_FD,VIDEO_FD};
    for (size_t i=0;i<sizeof(failed_nodes)/sizeof(failed_nodes[0]);++i) {
        struct fake f=fake_new(0x060c00,true); f.fail_open=failed_nodes[i]; struct cc_io io={&ops,&f};
        cc_camera_info info=camera_info(); cc_camera *camera=NULL;
        ERROR(cc_camera_open_io(&info,&io,&camera),EACCES); CHECK(!camera); clean(&f);
    }
    const uint32_t unavailable[]={2,4,6,7,8};
    for (size_t i=0;i<sizeof(unavailable)/sizeof(unavailable[0]);++i) {
        struct fake f=fake_new(0x060c00,true); f.unavailable=unavailable[i]; struct cc_io io={&ops,&f};
        cc_camera_info info=camera_info(); cc_camera *camera=NULL;
        ERROR(cc_camera_open_io(&info,&io,&camera),ENODEV); CHECK(!camera); clean(&f);
    }
    {
        struct fake f=fake_new(0x060c00,true); struct cc_io io={&ops,&f}; cc_camera_info info=camera_info();
        info.sensor_entity=1; cc_camera *camera=NULL;
        ERROR(cc_camera_open_io(&info,&io,&camera),ENODEV); CHECK(!camera); clean(&f);
        f.links[5].sink.entity=5; memcpy(f.original,f.links,sizeof(f.links));
        info.sensor_entity=2; ERROR(cc_camera_open_io(&info,&io,&camera),ENODEV); CHECK(!camera); clean(&f);
    }
    for (int bad=0;bad<3;++bad) {
        struct fake f=fake_new(0x060c00,true); cc_camera *camera=open_camera(&f); cc_format format=format_new(),original=format;
        f.bad_pad=bad==0; f.bad_crop=bad==1; f.bad_raw=bad==2;
        ERROR(cc_camera_configure(camera,&format),EPIPE); CHECK(!camera->stream->configured && !memcmp(&format,&original,sizeof(format)));
        f.bad_pad=f.bad_crop=f.bad_raw=false; OK(cc_camera_configure(camera,&format)); cc_camera_close(camera); clean(&f);
    }
    const struct { unsigned long request; int fd,nth; } configure_failures[]={
        {MEDIA_IOC_SETUP_LINK,MEDIA_FD,1},{MEDIA_IOC_SETUP_LINK,MEDIA_FD,2},
        {VIDIOC_SUBDEV_S_FMT,SENSOR_FD,1},{VIDIOC_SUBDEV_S_FMT,DEBAYER_FD,1},{VIDIOC_SUBDEV_S_FMT,DEBAYER_FD,2},
        {VIDIOC_SUBDEV_S_FMT,SCALER_FD,1},{VIDIOC_SUBDEV_S_FMT,SCALER_FD,2},
        {VIDIOC_SUBDEV_S_SELECTION,SCALER_FD,1},{VIDIOC_S_FMT,VIDEO_FD,1},{VIDIOC_S_FMT,RAW_FD,1}};
    for (size_t i=0;i<sizeof(configure_failures)/sizeof(configure_failures[0]);++i) {
        struct fake f=fake_new(0x060c00,true); cc_camera *camera=open_camera(&f);
        f.fail_request=configure_failures[i].request; f.fail_fd=configure_failures[i].fd; f.fail_nth=configure_failures[i].nth;
        cc_format format=format_new(),original=format;
        ERROR(cc_camera_configure(camera,&format),EIO); CHECK(!camera->stream->configured && !memcmp(&format,&original,sizeof(format)));
        f.fail_request=0; cc_camera_close(camera); clean(&f);
    }
    {
        struct fake f=fake_new(0x060c00,true);
        f.links[2].flags=MEDIA_LNK_FL_IMMUTABLE; memcpy(f.original,f.links,sizeof(f.links));
        cc_camera *camera=open_camera(&f); cc_format format=format_new();
        ERROR(cc_camera_configure(camera,&format),EPERM); CHECK(!camera->stream->configured);
        cc_camera_close(camera); clean(&f);
    }
    const unsigned long failures[]={VIDIOC_S_EXT_CTRLS,VIDIOC_QBUF,VIDIOC_STREAMON,VIDIOC_DQBUF};
    for (size_t i=0;i<sizeof(failures)/sizeof(failures[0]);++i) {
        struct fake f=fake_new(0x060c00,true); cc_camera *camera=open_camera(&f); cc_format format=format_new();
        OK(cc_camera_configure(camera,&format)); uint32_t count=SLOTS; OK(cc_camera_allocate_buffers(camera,&count));
        cc_capture_devices devices={.camera=camera}; cc_capture *capture=NULL; OK(cc_capture_create(&devices,&capture,&count));
        attach_ipa(capture);
        cc_request *request,*waiting; OK(cc_request_create(1,&request)); OK(cc_request_create(2,&waiting));
        cc_vimc_controls controls={.flags=1,.brightness=0}; OK(cc_request_set_vimc_controls(request,&controls));
        OK(cc_capture_queue(capture,request)); OK(cc_capture_queue(capture,waiting));
        f.fail_request=failures[i]; f.fail_nth=1;
        ERROR(cc_capture_dispatch(capture),EIO);
        cc_request *done=NULL; OK(cc_capture_dequeue(capture,&done,0)); CHECK(done==request);
        const cc_capture_result *result=cc_request_get_result(done);
        CHECK(result->status==CC_REQUEST_FAILED && result->error==-EIO && !result->raw.num_planes && !result->vimc.flags);
        OK(cc_capture_release(capture,done)); OK(cc_capture_dequeue(capture,&done,0)); CHECK(done==waiting);
        CHECK(cc_request_get_result(done)->status==CC_REQUEST_CANCELLED); OK(cc_capture_release(capture,done));
        OK(cc_capture_stop(capture)); f.fail_request=0;
        OK(cc_capture_queue(capture,request)); OK(cc_capture_dequeue(capture,&done,0));
        CHECK(cc_request_get_result(done)->status==CC_REQUEST_COMPLETE); OK(cc_capture_release(capture,done));
        cc_capture_close(capture); OK(cc_request_destroy(request)); OK(cc_request_destroy(waiting)); clean(&f);
    }
}
int main(int argc, char **argv)
{
    CHECK(argc == 1 || argc == 2);
    if (argc == 2) {
        proxy_worker = argv[1]; cc_ipa *probe = NULL; uint32_t flags;
        const char config[] = "mock"; cc_ipa_vimc_options controller = {config, sizeof(config), -1, CC_IPA_VIMC_INIT, 0};
        cc_ipa_proxy_options options = {.worker_path = proxy_worker};
        int ret = cc_ipa_vimc_create_proxy(&controller, &options, &probe, &flags);
        if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) return 77;
        OK(ret); cc_ipa_destroy(probe);
    }
    const uint32_t versions[]={0x050500,0x050600,0x050700,0x050f00,0x051000,0x060c00};
    for (size_t i=0;i<sizeof(versions)/sizeof(versions[0]);++i) for (int v2=0;v2<2;++v2) test_formats(versions[i],v2!=0);
    test_controls();
    for (int v2=0;v2<2;++v2) for (int imports=0;imports<2;++imports) test_capture(imports!=0,v2!=0);
    test_failures(); puts("VIMC native pipeline, kernel compatibility, controls and request lifecycle passed"); return 0;
}
