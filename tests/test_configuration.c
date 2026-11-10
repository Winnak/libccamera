/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <ccamera/configuration.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)>=0)
#define ERROR(x,e) CHECK((x)==-(e))
struct fake {
    uint32_t formats[4]; size_t count;
    cc_size_range sizes[16]; size_t num_sizes;
    bool range,endless_formats,endless_sizes,read_only,clamp;
    uint32_t error_index; unsigned long error_request; int error;
    unsigned opened,closed,sets,tries;
    cc_format active;
};
struct configuration_vector { uint32_t kind,width,height,fourcc,orientation,count,out_width,out_height,out_fourcc,status; };
#include "configuration_vectors.h"
static int open_node(void *ctx,const char *path,int flags)
{
    (void)flags; struct fake *f=ctx; CHECK(!strcmp(path,"/dev/video0")); ++f->opened; return 0;
}
static int close_node(void *ctx,int fd) { CHECK(fd==0); ++((struct fake *)ctx)->closed; return 0; }
static int lock_node(void *ctx,int fd) { (void)ctx; CHECK(fd==0); return 0; }
static int ioctl_node(void *ctx,int fd,unsigned long req,void *arg)
{
    struct fake *f=ctx; CHECK(fd==0);
    CHECK(!f->read_only || (req!=VIDIOC_S_FMT && req!=VIDIOC_REQBUFS));
    if (req==VIDIOC_QUERYCAP) {
        struct v4l2_capability *q=arg;
        q->capabilities=V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_STREAMING|V4L2_CAP_EXT_PIX_FORMAT;
        strcpy((char *)q->driver,"uvcvideo"); strcpy((char *)q->card,"role camera"); return 0;
    }
    if (req==VIDIOC_ENUM_FMT) {
        struct v4l2_fmtdesc *q=arg;
        if (req==f->error_request && q->index==f->error_index) return -f->error;
        if (!f->endless_formats && q->index>=f->count) return -EINVAL;
        q->pixelformat=f->endless_formats ? q->index+1:f->formats[q->index]; return 0;
    }
    if (req==VIDIOC_ENUM_FRAMESIZES) {
        struct v4l2_frmsizeenum *q=arg;
        if (req==f->error_request && q->index==f->error_index) return -f->error;
        if (!f->endless_sizes && q->index>=f->num_sizes) return -EINVAL;
        cc_size_range r=f->sizes[f->endless_sizes ? 0:q->index];
        q->type=f->range ? V4L2_FRMSIZE_TYPE_STEPWISE:V4L2_FRMSIZE_TYPE_DISCRETE;
        if (f->range) q->stepwise=(struct v4l2_frmsize_stepwise){r.min_width,r.max_width,r.width_step,r.min_height,r.max_height,r.height_step};
        else { q->discrete.width=r.min_width; q->discrete.height=r.min_height; }
        return 0;
    }
    if (req==VIDIOC_TRY_FMT || req==VIDIOC_S_FMT || req==VIDIOC_G_FMT) {
        if (req==f->error_request) return -f->error;
        struct v4l2_format *q=arg; struct v4l2_pix_format *p=&q->fmt.pix;
        cc_format actual=f->active;
        if (req!=VIDIOC_G_FMT) {
            actual=(cc_format){.fourcc=p->pixelformat,.width=p->width+(f->clamp ? 2U:0U),.height=p->height,
                .field=V4L2_FIELD_NONE,.colorspace=V4L2_COLORSPACE_JPEG,.num_planes=1,
                .quantization=V4L2_QUANTIZATION_FULL_RANGE,.ycbcr_enc=V4L2_YCBCR_ENC_601,.xfer_func=V4L2_XFER_FUNC_SRGB};
            actual.planes[0]=(cc_plane_format){actual.width*3,actual.width*3*actual.height};
            if (req==VIDIOC_S_FMT) { ++f->sets; f->active=actual; }
            else ++f->tries;
        }
        *p=(struct v4l2_pix_format){.width=actual.width,.height=actual.height,.pixelformat=actual.fourcc,
            .bytesperline=actual.planes[0].stride,.sizeimage=actual.planes[0].size,.field=actual.field,
            .colorspace=actual.colorspace,.priv=V4L2_PIX_FMT_PRIV_MAGIC,.quantization=(uint8_t)actual.quantization,
            .ycbcr_enc=(uint8_t)actual.ycbcr_enc,.xfer_func=(uint8_t)actual.xfer_func}; return 0;
    }
    CHECK(false); return -ENOTTY;
}
static const struct cc_io_ops ops={.open=open_node,.close=close_node,.lock=lock_node,.ioctl=ioctl_node};
static cc_camera *initialize(struct fake *f)
{
    *f=(struct fake){.formats={V4L2_PIX_FMT_BGR24,V4L2_PIX_FMT_RGB24},.count=2,.num_sizes=5};
    const uint32_t sizes[][2]={{1280,720},{640,480},{1920,1080},{1024,300},{800,600}};
    for (size_t i=0;i<5;++i) f->sizes[i]=(cc_size_range){sizes[i][0],sizes[i][0],1,sizes[i][1],sizes[i][1],1};
    f->active=(cc_format){.fourcc=V4L2_PIX_FMT_RGB24,.width=640,.height=480,.num_planes=1,.planes={{1920,921600}}};
    struct cc_io io={&ops,f}; cc_camera_info info={.pipeline=CC_PIPELINE_UVC,.video_path="/dev/video0"};
    cc_camera *camera=NULL; CHECK(!cc_camera_open_io(&info,&io,&camera)); f->read_only=true; return camera;
}
static void test_roles(void)
{
    struct fake f; cc_camera *camera=initialize(&f); cc_capture_devices d={.camera=camera};
    enum cc_stream_role roles[]={CC_ROLE_RAW,CC_ROLE_VIEWFINDER,CC_ROLE_VIDEO_RECORDING};
    cc_pipeline_configuration c;
    OK(cc_pipeline_generate_configuration(&d,NULL,0,&c)); CHECK(!c.num_streams);
    ERROR(cc_pipeline_validate_configuration(&d,&c),EINVAL);
    OK(cc_pipeline_generate_configuration(&d,roles,3,&c));
    CHECK(c.num_streams==1 && c.streams[0].format.fourcc==V4L2_PIX_FMT_RGB24 &&
        c.streams[0].format.width==1920 && c.streams[0].format.height==1080 && c.buffer_count==4 &&
        c.streams[0].stream==CC_CAPTURE_RAW && !f.sets);
    CHECK(cc_pipeline_validate_configuration(&d,&c)==CC_CONFIGURATION_VALID);
    c.streams[0].format=(cc_format){.fourcc=UINT32_MAX,.width=1000,.height=700};
    c.orientation=CC_ORIENTATION_ROTATE_90; c.streams[0].buffer_count=1;
    CHECK(cc_pipeline_validate_configuration(&d,&c)==CC_CONFIGURATION_ADJUSTED);
    CHECK(c.streams[0].format.fourcc==V4L2_PIX_FMT_RGB24 && c.streams[0].format.width==800 &&
        c.streams[0].format.height==600 && c.orientation==CC_ORIENTATION_ROTATE_0 && c.streams[0].buffer_count==4);
    c.streams[0].format.width=1000; c.streams[0].format.height=400;
    OK(cc_pipeline_validate_configuration(&d,&c)); CHECK(c.streams[0].format.width==1024 && c.streams[0].format.height==300);
    c.streams[0].format.width=c.streams[0].format.height=1;
    OK(cc_pipeline_validate_configuration(&d,&c)); CHECK(c.streams[0].format.width==640 && c.streams[0].format.height==480);
    cc_pipeline_configuration saved=c;
    f.error_request=VIDIOC_ENUM_FMT; f.error_index=1; f.error=EIO;
    ERROR(cc_pipeline_validate_configuration(&d,&c),EIO); CHECK(!memcmp(&c,&saved,sizeof(c)));
    f.error_request=0; f.read_only=false; OK(cc_pipeline_configure(&d,&c)); CHECK(f.sets==1 && camera->stream->configured);
    saved=c; camera->stream->allocated=true; f.read_only=true;
    ERROR(cc_pipeline_configure(&d,&c),EBUSY); CHECK(!memcmp(&c,&saved,sizeof(c)) && f.sets==1);
    camera->stream->allocated=false; f.error_request=VIDIOC_S_FMT; f.error=EIO; f.read_only=false;
    ERROR(cc_pipeline_configure(&d,&c),EIO); CHECK(!memcmp(&c,&saved,sizeof(c)) && !camera->stream->configured);
    f.error_request=0; OK(cc_pipeline_configure(&d,&c)); cc_camera_close(camera); CHECK(f.opened==f.closed);
}
static void test_ranges_and_bounds(void)
{
    struct fake f; cc_camera *camera=initialize(&f); cc_capture_devices d={.camera=camera};
    enum cc_stream_role role=CC_ROLE_VIEWFINDER; cc_pipeline_configuration c;
    f.range=true; f.num_sizes=1; f.sizes[0]=(cc_size_range){160,1280,32,128,960,16};
    OK(cc_pipeline_generate_configuration(&d,&role,1,&c)); CHECK(c.streams[0].format.width==1280 && c.streams[0].format.height==800);
    cc_pipeline_configuration saved=c;
    f.num_sizes=2; ERROR(cc_pipeline_validate_configuration(&d,&c),EPROTO); CHECK(!memcmp(&c,&saved,sizeof(c)));
    f.range=false; f.num_sizes=1; f.endless_sizes=true;
    ERROR(cc_pipeline_validate_configuration(&d,&c),E2BIG); CHECK(!memcmp(&c,&saved,sizeof(c)));
    f.endless_sizes=false; f.endless_formats=true;
    ERROR(cc_pipeline_validate_configuration(&d,&c),E2BIG); CHECK(!memcmp(&c,&saved,sizeof(c)));
    f.endless_formats=false; f.formats[1]=f.formats[0]; ERROR(cc_pipeline_validate_configuration(&d,&c),EPROTO);
    f.count=0; ERROR(cc_pipeline_validate_configuration(&d,&c),ENOENT); CHECK(!memcmp(&c,&saved,sizeof(c)));
    ERROR(cc_pipeline_generate_configuration(&d,NULL,1,&c),EINVAL);
    ERROR(cc_pipeline_generate_configuration(&d,&role,4,&c),E2BIG);
    ERROR(cc_pipeline_validate_configuration(NULL,&c),EINVAL); ERROR(cc_pipeline_configure(NULL,&c),EINVAL);
    ERROR(cc_pipeline_generate_configuration(NULL,NULL,0,&c),EINVAL);
    ERROR(cc_pipeline_validate_configuration(&d,NULL),EINVAL); ERROR(cc_pipeline_configure(&d,NULL),EINVAL);
    cc_capture_devices invalid=d; invalid.frontend=(cc_cfe *)(uintptr_t)1;
    ERROR(cc_pipeline_validate_configuration(&invalid,&c),EINVAL);
    cc_camera_close(camera); CHECK(f.opened==f.closed);
}
static void test_reference_vectors(void)
{
    struct fake f; cc_camera *camera=initialize(&f); cc_capture_devices d={.camera=camera};
    for (size_t i=0;i<sizeof(configuration_vectors)/sizeof(configuration_vectors[0]);++i) {
        const struct configuration_vector *v=&configuration_vectors[i];
        f.range=v->kind!=0; f.num_sizes=f.range ? 1:5;
        if (v->kind==1) f.sizes[0]=(cc_size_range){160,1280,32,128,960,16};
        else if (v->kind==2) f.sizes[0]=(cc_size_range){640,4096,64,480,2160,16};
        else f.sizes[0]=(cc_size_range){1280,1280,1,720,720,1};
        cc_pipeline_configuration c={.num_streams=v->count,.orientation=(enum cc_orientation)v->orientation};
        for (size_t j=0;j<c.num_streams;++j) c.streams[j]=(cc_stream_configuration){.role=CC_ROLE_VIEWFINDER,.buffer_count=4,
            .format={.fourcc=v->fourcc,.width=v->width,.height=v->height}};
        int ret=cc_pipeline_validate_configuration(&d,&c);
        CHECK(ret==(int)v->status && c.num_streams==1 && c.orientation==CC_ORIENTATION_ROTATE_0 &&
            c.streams[0].format.fourcc==v->out_fourcc && c.streams[0].format.width==v->out_width &&
            c.streams[0].format.height==v->out_height && c.streams[0].buffer_count==4);
    }
    cc_camera_close(camera); CHECK(f.opened==f.closed);
}
int main(void)
{
    test_roles(); test_ranges_and_bounds(); test_reference_vectors();
    puts("Native role generation, reference size ordering, ranges, adjustment, bounded enumeration and atomic errors passed"); return 0;
}
