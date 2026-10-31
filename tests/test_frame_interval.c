/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
_Static_assert(CC_SD_G_FRAME_INTERVAL==VIDIOC_SUBDEV_G_FRAME_INTERVAL,"Native interval ioctl ABI");
_Static_assert(CC_SD_ENUM_FRAME_INTERVAL==VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL,"Native interval enum ABI");
struct fake { uint32_t type,kind,bad; int error,gets,sets,enums; bool fixed; cc_fraction active; };
static int ioctl_interval(void *context,int fd,unsigned long request,void *argument)
{
    struct fake *d=context; CHECK(fd==7);
    if (d->error) return d->error;
    if (request==VIDIOC_ENUM_FRAMEINTERVALS) {
        ++d->enums; struct v4l2_frmivalenum *e=argument;
        CHECK(e->pixel_format==V4L2_PIX_FMT_RGB24 && e->width==640 && e->height==480);
        if (e->index) return -EINVAL;
        e->type=d->kind;
        if (e->type==1) e->discrete=(struct v4l2_fract){1001,30000};
        else { e->stepwise.min=(struct v4l2_fract){UINT32_MAX-1,UINT32_MAX};
            e->stepwise.max=(struct v4l2_fract){UINT32_MAX,UINT32_MAX-1}; e->stepwise.step=(struct v4l2_fract){1,1000}; }
        if (d->bad==1) ++e->height;
        if (d->bad==2) e->discrete.denominator=0;
        if (d->bad==3) e->type=99;
        if (d->bad==4) { e->stepwise.min=(struct v4l2_fract){2,1}; e->stepwise.max=(struct v4l2_fract){1,1}; }
        return 0;
    }
    struct v4l2_streamparm *p=argument; CHECK(p->type==d->type);
    if (request==VIDIOC_G_PARM) {
        ++d->gets; p->parm.capture=(struct v4l2_captureparm){.capability=d->fixed ? 0:V4L2_CAP_TIMEPERFRAME,
            .capturemode=0x23,.extendedmode=7,.readbuffers=4,.timeperframe={d->active.numerator,d->active.denominator}};
    } else {
        CHECK(request==VIDIOC_S_PARM); ++d->sets;
        CHECK(p->parm.capture.capturemode==0x23 && p->parm.capture.extendedmode==7 && !p->parm.capture.readbuffers);
        for (size_t i=0;i<4;++i) CHECK(!p->parm.capture.reserved[i]);
        d->active=p->parm.capture.timeperframe.numerator ? (cc_fraction){1001,24000}:(cc_fraction){1,30};
        p->parm.capture.timeperframe=(struct v4l2_fract){d->active.numerator,d->active.denominator};
    }
    if (d->bad==1) ++p->type;
    if (d->bad==2) p->parm.capture.timeperframe.denominator=0;
    return 0;
}
static const struct cc_io_ops ops={.ioctl=ioctl_interval};
static void test_video(uint32_t type)
{
    struct fake d={.type=type,.kind=1,.active={1001,30000}};
    cc_video_device device={.fd=7,.io={&ops,&d}};
    cc_stream stream={.device=&device,.type=(enum v4l2_buf_type)type,.configured=true}; device.streams=&stream;
    cc_camera camera={.stream=&stream,.device=&device,.sensor_fd=-1};
    cc_fraction f; OK(cc_camera_get_frame_interval(&camera,&f)); CHECK(f.numerator==1001 && f.denominator==30000);
    f=(cc_fraction){1,25}; OK(cc_camera_set_frame_interval(&camera,&f)); CHECK(f.numerator==1001 && f.denominator==24000 && d.sets==1);
    f=(cc_fraction){0,0}; OK(cc_stream_set_frame_interval(&stream,&f)); CHECK(f.numerator==1 && f.denominator==30);
    cc_fraction original=f;
    stream.allocated=true; CHECK(cc_stream_set_frame_interval(&stream,&f)==-EBUSY && d.sets==2); stream.allocated=false;
    cc_stream other={.device=&device,.allocated=true}; stream.next=&other;
    CHECK(cc_stream_set_frame_interval(&stream,&f)==-EBUSY); stream.next=NULL;
    stream.configured=false; CHECK(cc_stream_get_frame_interval(&stream,&f)==-EPIPE); stream.configured=true;
    d.fixed=true; OK(cc_stream_get_frame_interval(&stream,&f)); CHECK(cc_stream_set_frame_interval(&stream,&f)==-ENOTSUP);
    d.active=(cc_fraction){0,0}; CHECK(cc_stream_get_frame_interval(&stream,&f)==-ENOTSUP);
    d.active=(cc_fraction){1,30}; d.fixed=false;
    d.bad=1; CHECK(cc_stream_get_frame_interval(&stream,&f)==-EPROTO && !memcmp(&f,&original,sizeof(f)));
    d.bad=2; CHECK(cc_stream_set_frame_interval(&stream,&f)==-EPROTO && !memcmp(&f,&original,sizeof(f))); d.bad=0;
    d.error=-ENOTTY; CHECK(cc_stream_get_frame_interval(&stream,&f)==-ENOTSUP); d.error=0;
    f=(cc_fraction){1,0}; CHECK(cc_stream_set_frame_interval(&stream,&f)==-EINVAL);
    cc_frame_interval_range range,saved; memset(&range,0xa5,sizeof(range)); saved=range;
    for (d.kind=1;d.kind<=3;++d.kind) {
        OK(cc_camera_enum_frame_interval(&camera,V4L2_PIX_FMT_RGB24,640,480,0,&range));
        CHECK((uint32_t)range.type==d.kind && range.minimum.numerator);
        if (d.kind==1) CHECK(!memcmp(&range.minimum,&range.maximum,sizeof(cc_fraction)) && !range.step.numerator);
        saved=range; CHECK(cc_stream_enum_frame_interval(&stream,V4L2_PIX_FMT_RGB24,640,480,1,&range)==-ENOENT);
        CHECK(!memcmp(&range,&saved,sizeof(range)));
    }
    for (d.bad=1;d.bad<=4;++d.bad) {
        d.kind=d.bad==4 ? 3:1;
        CHECK(cc_stream_enum_frame_interval(&stream,V4L2_PIX_FMT_RGB24,640,480,0,&range)==-EPROTO);
        CHECK(!memcmp(&range,&saved,sizeof(range)));
    }
    stream.type=V4L2_BUF_TYPE_META_CAPTURE; CHECK(cc_stream_get_frame_interval(&stream,&f)==-ENOTSUP);
    device.fd=-1; CHECK(cc_stream_get_frame_interval(&stream,&f)==-ENODEV);
}
int main(void)
{
    test_video(1); test_video(2); test_video(9); test_video(10);
    puts("Native rational intervals, queue directions, ranges, preserved modes, allocation guards and atomic errors passed"); return 0;
}
