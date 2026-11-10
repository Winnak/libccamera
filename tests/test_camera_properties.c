/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/capture.h>
#include <errno.h>
#include <linux/media-bus-format.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
#define R10 MEDIA_BUS_FMT_SRGGB10_1X10
#define B10 MEDIA_BUS_FMT_SBGGR10_1X10
_Static_assert(CC_SD_G_SELECTION==VIDIOC_SUBDEV_G_SELECTION,"Native selection ioctl ABI");

struct mode { uint32_t code,width,height; };
struct fake {
    cc_camera camera;
    cc_video_device device;
    cc_stream stream;
    struct cc_media media;
    struct cc_entity entity;
    struct media_pad_desc pads[3];
    bool streams[1];
    uint32_t codes[4], count, image_stream;
    struct mode modes[8];
    size_t num_modes;
    bool location, rotation, bounds, active, endless_codes, endless_sizes, ranges;
    int32_t orientation, degrees;
    unsigned bad_control, bad_selection, bad_format, bad_size;
    unsigned long fail_request;
    uint32_t fail_index;
    int fail_error, sysfs_error;
    const char *removable;
    bool bad_sysfs;
    bool flips, flip_layout, bad_flip_value;
    uint32_t flip;
    unsigned calls, selections;
};

static uint32_t advertised_code(const struct fake *f, uint32_t code)
{
    if (!f->flips || !f->flip_layout) return code;
    /* Readout orders for the four actual hardware flip combinations. */
    static const uint32_t red[]={R10,MEDIA_BUS_FMT_SGRBG10_1X10,MEDIA_BUS_FMT_SGBRG10_1X10,B10};
    static const uint32_t blue[]={B10,MEDIA_BUS_FMT_SGBRG10_1X10,MEDIA_BUS_FMT_SGRBG10_1X10,R10};
    static const uint32_t green_red[]={MEDIA_BUS_FMT_SGRBG10_1X10,R10,B10,MEDIA_BUS_FMT_SGBRG10_1X10};
    static const uint32_t green_blue[]={MEDIA_BUS_FMT_SGBRG10_1X10,B10,R10,MEDIA_BUS_FMT_SGRBG10_1X10};
    return code==R10 ? red[f->flip]:code==B10 ? blue[f->flip]:
        code==MEDIA_BUS_FMT_SGRBG10_1X10 ? green_red[f->flip]:
        code==MEDIA_BUS_FMT_SGBRG10_1X10 ? green_blue[f->flip]:code;
}

static int fake_ioctl(void *context, int fd, unsigned long request, void *argument)
{
    struct fake *f=context;
    ++f->calls;
    if (request==f->fail_request && request!=VIDIOC_ENUM_FMT && request!=VIDIOC_ENUM_FRAMESIZES &&
        request!=VIDIOC_SUBDEV_ENUM_MBUS_CODE && request!=VIDIOC_SUBDEV_ENUM_FRAME_SIZE) return -f->fail_error;
    if (request==VIDIOC_QUERY_EXT_CTRL) {
        CHECK(fd==11);
        struct v4l2_query_ext_ctrl *q=argument;
        bool location=q->id==V4L2_CID_CAMERA_ORIENTATION;
        if (!location && q->id!=V4L2_CID_CAMERA_SENSOR_ROTATION) {
            CHECK(q->id==V4L2_CID_HFLIP || q->id==V4L2_CID_VFLIP);
            if (!f->flips) return -EINVAL;
            q->type=V4L2_CTRL_TYPE_BOOLEAN; q->maximum=1; q->step=1; q->elems=1; q->elem_size=4;
            q->flags=f->flip_layout ? V4L2_CTRL_FLAG_MODIFY_LAYOUT:0;
            return 0;
        }
        if (location ? !f->location:!f->rotation) return -EINVAL;
        q->type=location ? V4L2_CTRL_TYPE_MENU:V4L2_CTRL_TYPE_INTEGER;
        q->maximum=1000; q->minimum=-1000; q->step=1; q->elems=1; q->elem_size=4;
        q->flags=V4L2_CTRL_FLAG_READ_ONLY;
        q->default_value=location ? f->orientation:f->degrees;
        switch (f->bad_control) {
        case 1: ++q->id; break;
        case 2: q->type=V4L2_CTRL_TYPE_STRING; break;
        case 3: q->minimum=1001; break;
        case 4: q->default_value=1001; break;
        case 5: q->elems=2; break;
        case 6: q->nr_of_dims=1; break;
        case 7: q->flags|=V4L2_CTRL_FLAG_HAS_PAYLOAD; break;
        case 8: q->maximum=INT64_MAX; break;
        case 9: q->flags|=V4L2_CTRL_FLAG_DISABLED; break;
        }
        return 0;
    }
    if (request==VIDIOC_G_EXT_CTRLS) {
        CHECK(fd==11 && f->flips && f->flip_layout);
        struct v4l2_ext_controls *q=argument;
        CHECK(q->count==2);
        for (uint32_t i=0; i<q->count; ++i) {
            CHECK(q->controls[i].id==(i ? V4L2_CID_VFLIP:V4L2_CID_HFLIP));
            q->controls[i].value=f->bad_flip_value ? 2:(int32_t)((f->flip>>i)&1U);
        }
        return 0;
    }
    if (request==CC_SD_G_ROUTING) {
        CHECK(fd==11 && f->streams[0]);
        struct cc_subdev_routing *q=argument;
        CHECK(q->which==V4L2_SUBDEV_FORMAT_ACTIVE && q->len_routes>=1);
        struct cc_subdev_route *routes=(void *)(uintptr_t)q->routes;
        routes[0]=(struct cc_subdev_route){.sink_pad=1,.source_pad=2,
            .source_stream=f->image_stream,.flags=CC_SD_ROUTE_ACTIVE};
        q->num_routes=1;
        return 0;
    }
    if (request==VIDIOC_SUBDEV_G_FMT) {
        CHECK(fd==11);
        struct cc_subdev_format *q=argument;
        CHECK(q->pad==2 && q->stream==f->image_stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        q->format=(struct v4l2_mbus_framefmt){.code=advertised_code(f,f->codes[0]),.width=640,.height=480};
        return 0;
    }
    if (request==CC_SD_G_SELECTION) {
        CHECK(fd==11);
        ++f->selections;
        struct cc_subdev_selection *q=argument;
        CHECK(q->pad==2 && q->stream==f->image_stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        if (q->target==V4L2_SEL_TGT_CROP_BOUNDS) {
            if (!f->bounds) return -ENOTTY;
            q->r=(struct v4l2_rect){0,0,3300,2480};
        } else {
            CHECK(q->target==V4L2_SEL_TGT_CROP_DEFAULT);
            if (!f->active) return -EINVAL;
            q->r=(struct v4l2_rect){8,8,3280,2464};
        }
        switch (f->bad_selection) {
        case 1: ++q->pad; break;
        case 2: ++q->stream; break;
        case 3: q->which=V4L2_SUBDEV_FORMAT_TRY; break;
        case 4: ++q->target; break;
        case 5: q->r.left=-1; break;
        case 6: q->r.top=-1; break;
        case 7: q->r.width=0; break;
        case 8: q->r.height=0; break;
        case 9: if (q->target==V4L2_SEL_TGT_CROP_DEFAULT) q->r.left=100; break;
        }
        return 0;
    }
    bool sensor=request==VIDIOC_SUBDEV_ENUM_MBUS_CODE || request==VIDIOC_SUBDEV_ENUM_FRAME_SIZE;
    CHECK(fd==(sensor ? 11:20));
    if (request==VIDIOC_SUBDEV_ENUM_MBUS_CODE || request==VIDIOC_ENUM_FMT) {
        uint32_t index=sensor ? ((struct cc_subdev_mbus_code_enum *)argument)->index:
                              ((struct v4l2_fmtdesc *)argument)->index;
        if (request==f->fail_request && index==f->fail_index) return -f->fail_error;
        if (!f->endless_codes && index>=f->count) return -EINVAL;
        uint32_t code=f->endless_codes ? f->codes[0]+index:f->codes[index];
        if (sensor) {
            struct cc_subdev_mbus_code_enum *q=argument;
            CHECK(q->pad==2 && q->stream==f->image_stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
            q->code=advertised_code(f,code);
            switch (f->bad_format) {
            case 1: ++q->index; break;
            case 2: ++q->pad; break;
            case 3: q->code=0; break;
            }
        } else {
            struct v4l2_fmtdesc *q=argument;
            CHECK(q->type==V4L2_BUF_TYPE_VIDEO_CAPTURE);
            q->pixelformat=code;
            switch (f->bad_format) {
            case 1: ++q->index; break;
            case 2: ++q->type; break;
            case 3: q->pixelformat=0; break;
            }
        }
        return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_FRAME_SIZE || request==VIDIOC_ENUM_FRAMESIZES) {
        struct cc_subdev_frame_size_enum *sd=argument;
        struct v4l2_frmsizeenum *video=argument;
        uint32_t index=sensor ? sd->index:video->index, code=sensor ? sd->code:video->pixel_format;
        /* All Bayer orders in this fake share the same mode geometry. */
        uint32_t native_code=advertised_code(f,code); /* Flips are self-inverse. */
        if (request==f->fail_request && index==f->fail_index) return -f->fail_error;
        const struct mode *mode=NULL;
        uint32_t found=0;
        for (size_t i=0; i<f->num_modes; ++i)
            if (f->modes[i].code==(sensor ? native_code:code) && found++==index) { mode=&f->modes[i]; break; }
        if (!mode && f->endless_sizes) mode=&f->modes[0];
        if (!mode) return -EINVAL;
        if (sensor) {
            CHECK(sd->pad==2 && sd->stream==f->image_stream && sd->which==V4L2_SUBDEV_FORMAT_ACTIVE);
            sd->min_width=f->ranges ? 16:mode->width; sd->max_width=mode->width;
            sd->min_height=f->ranges ? 16:mode->height; sd->max_height=mode->height;
            switch (f->bad_size) {
            case 1: ++sd->index; break;
            case 2: ++sd->code; break;
            case 3: sd->min_width=0; break;
            case 4: sd->min_height=sd->max_height+1; break;
            }
        } else {
            video->type=V4L2_FRMSIZE_TYPE_DISCRETE;
            video->discrete=(struct v4l2_frmsize_discrete){mode->width,mode->height};
            if (f->ranges) {
                video->type=V4L2_FRMSIZE_TYPE_STEPWISE;
                video->stepwise=(struct v4l2_frmsize_stepwise){16,mode->width,2,16,mode->height,2};
            }
            switch (f->bad_size) {
            case 1: ++video->index; break;
            case 2: ++video->pixel_format; break;
            case 3: video->discrete.width=0; break;
            case 4: video->type=99; break;
            case 5: video->stepwise.step_width=0; break;
            }
        }
        return 0;
    }
    /* Property inspection must never write a control, format, route or queue. */
    CHECK(false);
    return -ENOTTY;
}

static int fake_sysfs(void *context, int fd, const char *attribute, char *out, size_t size)
{
    struct fake *f=context;
    CHECK(fd==20 && !strcmp(attribute,"../removable"));
    if (f->sysfs_error) return -f->sysfs_error;
    if (f->bad_sysfs) { memset(out,'a',size); return 0; }
    cc_copy_string(out,size,f->removable,strlen(f->removable));
    return 0;
}
static const struct cc_io_ops ops={.ioctl=fake_ioctl,.sysfs_read=fake_sysfs};

static void initialize(struct fake *f, bool sensor)
{
    memset(f,0,sizeof(*f));
    f->camera=(cc_camera){.io={&ops,f},.fd=20,.sensor_fd=sensor ? 11:-1,.sensor_pad=2,
        .media=&f->media,.subdev_streams=f->streams,.device=&f->device,.stream=&f->stream,
        .info={.pipeline=sensor ? CC_PIPELINE_RPI_UNICAM:CC_PIPELINE_UVC,.sensor_entity=1}};
    f->device=(cc_video_device){.io={&ops,f},.fd=20};
    strcpy((char *)f->device.caps.card,"Video model");
    f->stream=(cc_stream){.device=&f->device,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE};
    f->media=(struct cc_media){.entities=&f->entity,.num_entities=1};
    strcpy(f->media.info.model,"USB media model");
    f->entity.desc.id=1; f->entity.desc.pads=3; f->entity.pads=f->pads;
    strcpy(f->entity.name,"imx219 10-0010");
    f->pads[1]=(struct media_pad_desc){.entity=1,.index=1,.flags=MEDIA_PAD_FL_SINK};
    f->pads[2]=(struct media_pad_desc){.entity=1,.index=2,.flags=MEDIA_PAD_FL_SOURCE};
    f->location=f->rotation=f->bounds=f->active=true;
    f->orientation=V4L2_CAMERA_ORIENTATION_BACK; f->degrees=180;
    f->codes[0]=sensor ? R10:V4L2_PIX_FMT_RGB24;
    f->codes[1]=sensor ? B10:V4L2_PIX_FMT_MJPEG; f->count=2;
    f->modes[0]=(struct mode){f->codes[0],640,480};
    f->modes[1]=(struct mode){f->codes[0],3280,2464};
    f->modes[2]=(struct mode){f->codes[1],1920,1080}; f->num_modes=3;
    f->removable="fixed\n";
    f->fail_error=EIO;
}

static void expect_error(struct fake *f, int error)
{
    cc_camera_properties p; memset(&p,0xa5,sizeof(p)); cc_camera_properties saved=p;
    CHECK(cc_camera_get_properties(&f->camera,&p)==-error);
    CHECK(!memcmp(&p,&saved,sizeof(p)));
}

static void test_sensors(void)
{
    struct fake f; cc_camera_properties p;
    for (unsigned multiplexed=0; multiplexed<2; ++multiplexed) {
        initialize(&f,true); f.streams[0]=multiplexed!=0; f.image_stream=multiplexed ? 73:0;
        OK(cc_camera_get_properties(&f.camera,&p));
        CHECK(p.flags==127 && !strcmp(p.model,"imx219") && p.location==CC_CAMERA_LOCATION_BACK && p.rotation==180);
        CHECK(p.pixel_array_width==3300 && p.pixel_array_height==2480);
        CHECK(p.active_area.x==8 && p.active_area.y==8 && p.active_area.width==3280 && p.active_area.height==2464);
        CHECK(p.unit_cell_width_nm==1120 && p.unit_cell_height_nm==1120 && p.cfa==CC_CAMERA_CFA_BGGR);
        CHECK(f.camera.sensor_stream==0 && !f.stream.configured && !f.stream.allocated);
        f.stream.configured=f.stream.allocated=f.stream.streaming=true;
        cc_camera_properties saved=p; OK(cc_camera_get_properties(&f.camera,&p)); CHECK(!memcmp(&p,&saved,sizeof(p)));
    }
    initialize(&f,true); f.location=f.rotation=f.bounds=f.active=false;
    OK(cc_camera_get_properties(&f.camera,&p));
    CHECK(!(p.flags&CC_CAMERA_PROPERTY_LOCATION) && p.flags&CC_CAMERA_PROPERTY_ROTATION && !p.rotation);
    CHECK(p.pixel_array_width==3280 && p.pixel_array_height==2464 && !p.active_area.x && p.active_area.width==3280);
    f.orientation=99; f.location=true; OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.location==CC_CAMERA_LOCATION_EXTERNAL);
    f.orientation=V4L2_CAMERA_ORIENTATION_FRONT; f.rotation=true; f.degrees=45;
    OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.location==CC_CAMERA_LOCATION_FRONT && p.rotation==45);
    /* The native property preserves even non-quarter-turn rotation values. */
    f.bad_control=9; OK(cc_camera_get_properties(&f.camera,&p)); CHECK(!(p.flags&CC_CAMERA_PROPERTY_LOCATION) && !p.rotation);
    initialize(&f,true); f.camera.info.pipeline=CC_PIPELINE_VIMC; strcpy(f.entity.name,"Sensor B");
    OK(cc_camera_get_properties(&f.camera,&p));
    CHECK(!strcmp(p.model,"Sensor B") && p.pixel_array_width==3280 && !f.selections);
    CHECK(!(p.flags&(CC_CAMERA_PROPERTY_CFA|CC_CAMERA_PROPERTY_UNIT_CELL)));
    initialize(&f,true); f.location=f.rotation=false; f.count=0; f.bounds=f.active=false;
    OK(cc_camera_get_properties(&f.camera,&p)); CHECK(!(p.flags&(CC_CAMERA_PROPERTY_PIXEL_ARRAY|CC_CAMERA_PROPERTY_ACTIVE_AREA)));
    initialize(&f,true); f.fail_request=VIDIOC_SUBDEV_ENUM_MBUS_CODE; f.fail_error=ENOTTY;
    OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.pixel_array_width==3300 && !(p.flags&CC_CAMERA_PROPERTY_CFA));
}

static void test_models_and_cfa(void)
{
    static const struct { const char *name,*model; } models[]={
        {"imx219 10-0010","imx219"}, {"jt8ew9 pixel_array 0-0010","jt8ew9"},
        {"Sensor B","Sensor B"}, {"Other Sensor","Other Sensor"},
        {"sensor 0-ABCD","sensor 0-ABCD"}, {"sensor 0-abcz","sensor 0-abcz"},
        {"sensor 12-abcdef","sensor"}, {"Ca\xc3\xa9m 1-0010","Cam"}, {"",""}
    };
    struct fake f; cc_camera_properties p;
    for (size_t i=0; i<sizeof(models)/sizeof(models[0]); ++i) {
        initialize(&f,true); strcpy(f.entity.name,models[i].name);
        OK(cc_camera_get_properties(&f.camera,&p)); CHECK(!strcmp(p.model,models[i].model));
    }
    static const struct { uint32_t code; enum cc_camera_cfa cfa; } cfas[]={
#define CFA(bits) \
        {MEDIA_BUS_FMT_SRGGB##bits##_1X##bits,CC_CAMERA_CFA_RGGB}, \
        {MEDIA_BUS_FMT_SGRBG##bits##_1X##bits,CC_CAMERA_CFA_GRBG}, \
        {MEDIA_BUS_FMT_SGBRG##bits##_1X##bits,CC_CAMERA_CFA_GBRG}, \
        {MEDIA_BUS_FMT_SBGGR##bits##_1X##bits,CC_CAMERA_CFA_BGGR}, \
        {MEDIA_BUS_FMT_Y##bits##_1X##bits,CC_CAMERA_CFA_MONO}
        CFA(8), CFA(10), CFA(12), CFA(14), CFA(16)
#undef CFA
    };
    for (size_t i=0; i<sizeof(cfas)/sizeof(cfas[0]); ++i) {
        initialize(&f,true); f.count=1; f.codes[0]=cfas[i].code;
        for (size_t m=0; m<f.num_modes; ++m) f.modes[m].code=f.codes[0];
        OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.flags&CC_CAMERA_PROPERTY_CFA && p.cfa==cfas[i].cfa);
    }
    for (unsigned layout=0; layout<2; ++layout) for (uint32_t flip=0; flip<4; ++flip) {
        initialize(&f,true); f.flips=true; f.flip_layout=layout!=0; f.flip=flip;
        OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.flags&CC_CAMERA_PROPERTY_CFA && p.cfa==CC_CAMERA_CFA_BGGR);
        f.count=1; OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.cfa==CC_CAMERA_CFA_RGGB);
        f.codes[0]=MEDIA_BUS_FMT_Y10_1X10;
        for (size_t m=0; m<f.num_modes; ++m) f.modes[m].code=f.codes[0];
        OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.cfa==CC_CAMERA_CFA_MONO);
    }
    initialize(&f,true); f.flips=f.flip_layout=f.bad_flip_value=true; expect_error(&f,EPROTO);
    f.bad_flip_value=false; f.fail_request=VIDIOC_G_EXT_CTRLS; expect_error(&f,EIO);
    f.fail_error=ENOTTY; OK(cc_camera_get_properties(&f.camera,&p)); CHECK(!(p.flags&CC_CAMERA_PROPERTY_CFA));
}

static void test_video(void)
{
    struct fake f; cc_camera_properties p;
    initialize(&f,false); OK(cc_camera_get_properties(&f.camera,&p));
    CHECK(p.flags==(CC_CAMERA_PROPERTY_MODEL|CC_CAMERA_PROPERTY_LOCATION|CC_CAMERA_PROPERTY_PIXEL_ARRAY|CC_CAMERA_PROPERTY_ACTIVE_AREA));
    CHECK(!strcmp(p.model,"USB media model") && p.location==CC_CAMERA_LOCATION_FRONT && p.pixel_array_width==3280);
    f.camera.media=NULL; f.removable="removable\n"; OK(cc_camera_get_properties(&f.camera,&p));
    CHECK(!strcmp(p.model,"Video model") && p.location==CC_CAMERA_LOCATION_EXTERNAL);
    const int errors[]={ENOENT,EACCES,EIO,ENOTSUP};
    for (size_t i=0; i<sizeof(errors)/sizeof(errors[0]); ++i) {
        f.sysfs_error=errors[i]; OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.location==CC_CAMERA_LOCATION_EXTERNAL);
    }
    f.camera.info.pipeline=CC_PIPELINE_V4L2; OK(cc_camera_get_properties(&f.camera,&p));
    CHECK(!(p.flags&(CC_CAMERA_PROPERTY_LOCATION|CC_CAMERA_PROPERTY_ROTATION)));
    f.sysfs_error=0; f.bad_sysfs=true; f.camera.info.pipeline=CC_PIPELINE_UVC; expect_error(&f,EPROTO);
    initialize(&f,false); f.count=1; f.num_modes=1; f.ranges=true;
    OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.pixel_array_width==640 && p.active_area.height==480);
    f.modes[1]=(struct mode){f.codes[0],640,480}; f.num_modes=2; expect_error(&f,EPROTO); /* Stepwise index must be zero. */
    initialize(&f,false); f.modes[0]=(struct mode){f.codes[0],UINT32_MAX,UINT32_MAX};
    OK(cc_camera_get_properties(&f.camera,&p)); CHECK(p.pixel_array_width==UINT32_MAX && p.pixel_array_height==UINT32_MAX);
    f.fail_request=VIDIOC_ENUM_FMT; f.fail_index=1; f.fail_error=ENOTTY;
    OK(cc_camera_get_properties(&f.camera,&p)); CHECK(!(p.flags&(CC_CAMERA_PROPERTY_PIXEL_ARRAY|CC_CAMERA_PROPERTY_ACTIVE_AREA)));
}

static void test_failures_and_bounds(void)
{
    struct fake f; cc_camera_properties p;
    CHECK(cc_camera_get_properties(NULL,&p)==-EINVAL);
    CHECK(cc_cfe_get_properties(NULL,&p)==-EINVAL && cc_capture_get_properties(NULL,&p)==-EINVAL);
    initialize(&f,true); CHECK(cc_camera_get_properties(&f.camera,NULL)==-EINVAL);
    f.device.fd=-1; expect_error(&f,ENODEV); f.device.fd=20;
    for (unsigned bad=1; bad<=8; ++bad) { f.bad_control=bad; expect_error(&f,EPROTO); }
    f.bad_control=0;
    for (unsigned bad=1; bad<=9; ++bad) { f.bad_selection=bad; expect_error(&f,EPROTO); }
    f.bad_selection=0;
    const unsigned long failures[]={VIDIOC_QUERY_EXT_CTRL,VIDIOC_SUBDEV_G_FMT,CC_SD_G_SELECTION,
        VIDIOC_SUBDEV_ENUM_MBUS_CODE,VIDIOC_SUBDEV_ENUM_FRAME_SIZE};
    for (size_t i=0; i<sizeof(failures)/sizeof(failures[0]); ++i) {
        f.fail_request=failures[i]; expect_error(&f,EIO);
    }
    for (unsigned sensor=0; sensor<2; ++sensor) {
        initialize(&f,sensor!=0);
        for (unsigned bad=1; bad<=3; ++bad) { f.bad_format=bad; expect_error(&f,EPROTO); }
        f.bad_format=0;
        for (unsigned bad=1; bad<=4; ++bad) { f.bad_size=bad; expect_error(&f,EPROTO); }
        f.bad_size=0;
        f.codes[1]=f.codes[0]; expect_error(&f,EPROTO); initialize(&f,sensor!=0);
        f.endless_codes=true; expect_error(&f,E2BIG); f.endless_codes=false;
        f.endless_sizes=true; expect_error(&f,E2BIG); f.endless_sizes=false;
        f.fail_request=sensor ? VIDIOC_SUBDEV_ENUM_FRAME_SIZE:VIDIOC_ENUM_FRAMESIZES;
        f.fail_index=1; expect_error(&f,EIO);
        initialize(&f,sensor!=0); f.endless_codes=true;
        f.fail_request=sensor ? VIDIOC_SUBDEV_ENUM_MBUS_CODE:VIDIOC_ENUM_FMT;
        f.fail_index=256; f.fail_error=EINVAL;
        OK(cc_camera_get_properties(&f.camera,&p));
        initialize(&f,sensor!=0); f.endless_sizes=true; f.count=1;
        f.fail_request=sensor ? VIDIOC_SUBDEV_ENUM_FRAME_SIZE:VIDIOC_ENUM_FRAMESIZES;
        f.fail_index=4096; f.fail_error=EINVAL;
        OK(cc_camera_get_properties(&f.camera,&p));
    }
}

int main(void)
{
    test_sensors(); test_models_and_cfa(); test_video(); test_failures_and_bounds();
    puts("Camera properties: sensor/UVC/VIMC policies, arbitrary streams, read-only queries and atomic failures passed");
    return 0;
}
