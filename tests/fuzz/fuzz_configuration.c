/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "internal.h"
#include <ccamera/configuration.h>
#include <errno.h>
struct driver {
    uint32_t formats[4],width[4],height[4],flags;
    unsigned formats_count,sizes_count,calls;
};
static int query(void *ctx,int fd,unsigned long req,void *arg)
{
    struct driver *d=ctx; FUZZ_CHECK(fd==0 && ++d->calls<5000);
    if (req==VIDIOC_ENUM_FMT) {
        struct v4l2_fmtdesc *q=arg;
        if (q->index>=d->formats_count && !(d->flags&16U)) return -EINVAL;
        q->pixelformat=d->flags&16U ? q->index+1:d->formats[q->index];
        if (d->flags&4U) ++q->index;
        return 0;
    }
    if (req==VIDIOC_ENUM_FRAMESIZES) {
        struct v4l2_frmsizeenum *q=arg;
        if (q->index>=d->sizes_count && !(d->flags&32U)) return -EINVAL;
        unsigned i=d->flags&32U ? 0:q->index;
        q->type=V4L2_FRMSIZE_TYPE_DISCRETE;
        q->discrete=(struct v4l2_frmsize_discrete){d->width[i],d->height[i]};
        if (d->flags&8U) q->type=99;
        return 0;
    }
    FUZZ_CHECK(req==VIDIOC_TRY_FMT); /* Neither validation nor generation may write ACTIVE state. */
    struct v4l2_format *q=arg; struct v4l2_pix_format *p=&q->fmt.pix;
    uint64_t stride=(uint64_t)p->width*3;
    uint64_t size=stride<=UINT32_MAX ? stride*p->height:UINT64_MAX;
    p->bytesperline=stride<=UINT32_MAX ? (uint32_t)stride:0;
    p->sizeimage=size<=UINT32_MAX ? (uint32_t)size:0;
    if (d->flags&2U) ++q->type;
    p->field=V4L2_FIELD_NONE; p->colorspace=V4L2_COLORSPACE_SRGB;
    return 0;
}
static const struct cc_io_ops ops={.ioctl=query};
int LLVMFuzzerTestOneInput(const uint8_t *data,size_t size)
{
    struct fuzz_input in={data,size}; struct driver d={.flags=fuzz_u8(&in)};
    d.formats_count=fuzz_u8(&in)%4U+1; d.sizes_count=fuzz_u8(&in)%4U+1;
    const uint32_t formats[]={V4L2_PIX_FMT_RGB24,V4L2_PIX_FMT_YUYV,V4L2_PIX_FMT_NV12,V4L2_PIX_FMT_BGR24};
    for (unsigned i=0;i<4;++i) {
        d.formats[i]=formats[i]; d.width[i]=fuzz_u32(&in); d.height[i]=fuzz_u32(&in);
        if (d.flags&1U) { d.width[i]=d.width[i]%4096+16; d.height[i]=d.height[i]%2160+16; }
    }
    cc_video_device device={.io={&ops,&d},.fd=0,.capabilities=V4L2_CAP_STREAMING|V4L2_CAP_VIDEO_CAPTURE};
    cc_stream stream={.device=&device,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE}; device.streams=&stream;
    cc_camera camera={.io={&ops,&d},.device=&device,.stream=&stream,.fd=0,.sensor_fd=-1,.info={.pipeline=CC_PIPELINE_UVC}};
    cc_capture_devices devices={.camera=&camera};
    cc_pipeline_configuration c={.num_streams=fuzz_u8(&in)%5U,.orientation=(enum cc_orientation)(fuzz_u8(&in)%10U)};
    enum cc_stream_role roles[3];
    for (size_t i=0;i<3;++i) {
        roles[i]=(enum cc_stream_role)(fuzz_u8(&in)%6U);
        c.streams[i].role=roles[i]; c.streams[i].format.fourcc=formats[fuzz_u8(&in)%4U];
        c.streams[i].format.width=fuzz_u32(&in); c.streams[i].format.height=fuzz_u32(&in);
        c.streams[i].buffer_count=fuzz_u32(&in);
    }
    if (d.flags&64U) c.sensor=(cc_sensor_format){1,1,1};
    cc_pipeline_configuration saved=c;
    int ret=cc_pipeline_validate_configuration(&devices,&c);
    if (ret<0) FUZZ_CHECK(!memcmp(&c,&saved,sizeof(c)));
    else {
        FUZZ_CHECK(ret<=CC_CONFIGURATION_ADJUSTED && c.num_streams==1 && c.buffer_count==4 &&
            c.streams[0].buffer_count==4 && c.streams[0].stream==CC_CAPTURE_RAW && c.source_format.num_planes==1);
        bool offered=false;
        for (unsigned i=0;i<d.sizes_count;++i)
            if (c.source_format.width==d.width[i] && c.source_format.height==d.height[i]) offered=true;
        FUZZ_CHECK(offered && c.source_format.planes[0].size && !stream.configured);
    }
    d.calls=0; c=saved;
    ret=cc_pipeline_generate_configuration(&devices,roles,saved.num_streams,&c);
    if (ret<0) FUZZ_CHECK(!memcmp(&c,&saved,sizeof(c)));
    else FUZZ_CHECK(ret<=CC_CONFIGURATION_ADJUSTED && c.num_streams<=1 && !stream.configured);
    return 0;
}
