/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/sensor.h>
#include <errno.h>
#include <linux/media-bus-format.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
#define R10 MEDIA_BUS_FMT_SRGGB10_1X10
#define R12 MEDIA_BUS_FMT_SRGGB12_1X12
#define B10 MEDIA_BUS_FMT_SBGGR10_1X10

struct native_mode { uint32_t code, width, height; };
struct fake {
    cc_camera camera;
    struct cc_media media;
    struct cc_entity entity;
    struct media_pad_desc pads[3];
    bool streams[1];
    cc_stream stream;
    uint32_t codes[3];
    size_t count;
    struct native_mode modes[16];
    size_t mode_count;
    unsigned code_calls, size_calls, format_calls;
    unsigned long fail_request;
    uint32_t fail_index;
    int fail_error;
    unsigned bad_code, bad_size;
    bool endless_codes, endless_sizes, range;
};

static int fake_ioctl(void *context, int fd, unsigned long request, void *argument)
{
    struct fake *f=context;
    CHECK(fd==11);
    if (request==VIDIOC_SUBDEV_G_FMT) {
        struct cc_subdev_format *q=argument;
        CHECK(q->pad==2 && !q->stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        ++f->format_calls;
        q->format=(struct v4l2_mbus_framefmt){.code=R10,.width=640,.height=480};
        return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_MBUS_CODE) {
        struct cc_subdev_mbus_code_enum *q=argument;
        CHECK(q->pad==2 && !q->stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        ++f->code_calls;
        if (request==f->fail_request && q->index==f->fail_index) return -f->fail_error;
        if (!f->endless_codes && q->index>=f->count) return -EINVAL;
        q->code=f->endless_codes ? R10+q->index : f->codes[q->index];
        switch (f->bad_code) {
        case 1: ++q->pad; break;
        case 2: ++q->stream; break;
        case 3: ++q->index; break;
        case 4: q->which=V4L2_SUBDEV_FORMAT_TRY; break;
        case 5: q->code=0; break;
        }
        return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_FRAME_SIZE) {
        struct cc_subdev_frame_size_enum *q=argument;
        CHECK(q->pad==2 && !q->stream && q->which==V4L2_SUBDEV_FORMAT_ACTIVE);
        ++f->size_calls;
        if (request==f->fail_request && q->index==f->fail_index) return -f->fail_error;
        const struct native_mode *mode=NULL;
        uint32_t index=0;
        for (size_t i=0; i<f->mode_count; ++i)
            if (f->modes[i].code==q->code && index++==q->index) { mode=&f->modes[i]; break; }
        if (!mode && f->endless_sizes) mode=&f->modes[0];
        if (!mode) return -EINVAL;
        q->min_width=q->max_width=mode->width;
        q->min_height=q->max_height=mode->height;
        if (f->range) { q->min_width=16; q->min_height=16; }
        switch (f->bad_size) {
        case 1: ++q->pad; break;
        case 2: ++q->stream; break;
        case 3: ++q->index; break;
        case 4: q->which=V4L2_SUBDEV_FORMAT_TRY; break;
        case 5: ++q->code; break;
        case 6: q->min_width=0; break;
        case 7: q->min_height=0; break;
        case 8: q->max_width=q->min_width-1; break;
        case 9: q->max_height=q->min_height-1; break;
        }
        return 0;
    }
    /* Selection must never set formats, links, routes, controls or buffers. */
    CHECK(false);
    return -ENOTTY;
}
static const struct cc_io_ops fake_ops={.ioctl=fake_ioctl};

static void initialize(struct fake *f)
{
    memset(f,0,sizeof(*f));
    f->camera=(cc_camera){.io={.ops=&fake_ops,.ctx=f},.sensor_fd=11,.sensor_pad=2,
        .media=&f->media,.subdev_streams=f->streams,.stream=&f->stream,.info={.sensor_entity=1}};
    f->media=(struct cc_media){.entities=&f->entity,.num_entities=1};
    f->entity.desc.id=1; f->entity.desc.pads=3; f->entity.pads=f->pads;
    f->pads[2]=(struct media_pad_desc){.entity=1,.index=2,.flags=MEDIA_PAD_FL_SOURCE};
    f->codes[0]=R10; f->codes[1]=R12; f->codes[2]=B10; f->count=3;
    const struct native_mode modes[]={
        {R10,640,480}, {R10,1920,1080}, {R10,2592,1944}, {R10,1280,720},
        {R12,1280,960}, {R12,1920,1080}, {R12,3840,2160}, {B10,800,600}, {B10,2592,1944}
    };
    memcpy(f->modes,modes,sizeof(modes)); f->mode_count=sizeof(modes)/sizeof(modes[0]);
}
static cc_sensor_mode_request request(const uint32_t *codes, size_t count, uint32_t width, uint32_t height)
{
    return (cc_sensor_mode_request){.mbus_codes=codes,.num_mbus_codes=count,.width=width,.height=height};
}
static void expect(struct fake *f, const cc_sensor_mode_request *r, uint32_t code, uint32_t width, uint32_t height)
{
    cc_sensor_format out;
    OK(cc_camera_select_sensor_mode(&f->camera,r,&out));
    CHECK(out.mbus_code==code && out.width==width && out.height==height);
}
static void expect_error(struct fake *f, const cc_sensor_mode_request *r, int error)
{
    cc_sensor_format out={0xdeadbeef,1234,5678}, saved=out;
    CHECK(cc_camera_select_sensor_mode(&f->camera,r,&out)==-error);
    CHECK(!memcmp(&out,&saved,sizeof(out)));
}
static void test_ranking(void)
{
    struct fake f; initialize(&f);
    const uint32_t codes[]={R12,R10,B10};
    cc_sensor_mode_request r=request(codes,3,640,480);
    expect(&f,&r,R10,640,480); /* Area beats code preference. */
    r.width=1000; r.height=750; expect(&f,&r,R12,1280,960);
    r.width=1280; r.height=720; expect(&f,&r,R10,1280,720);
    r.width=1920; r.height=1080; expect(&f,&r,R12,1920,1080); /* Preference breaks exact ties. */
    r.width=1800; r.height=1200; expect(&f,&r,R10,2592,1944); /* Ratio beats smaller area. */
    r.max_width=2000; r.max_height=1500; expect_error(&f,&r,ENOENT); /* Each dimension must fit. */
    r.width=1600; r.height=1000; expect(&f,&r,R12,1920,1080);
    r.width=2592; r.height=1944; r.max_width=2592; r.max_height=1944;
    expect(&f,&r,R10,2592,1944);
    f.stream.configured=true; f.stream.streaming=true; f.stream.allocated=true;
    expect(&f,&r,R10,2592,1944);
    CHECK(f.stream.configured && f.stream.streaming && f.stream.allocated && !f.camera.sensor_stream);
    /* An accepted code without sizes must not terminate enumeration. */
    f.codes[0]=0x3021; f.codes[1]=R10; f.count=2;
    const uint32_t empty_codes[]={0x3021,R10}; r=request(empty_codes,2,640,480);
    expect(&f,&r,R10,640,480);
    f.range=true; r.width=1000; r.height=750;
    expect(&f,&r,R10,2592,1944); /* Range maxima only; no invented intermediate mode. */
    r.max_width=2000; r.max_height=1500;
    expect(&f,&r,R10,1920,1080);
}
static void test_large_dimensions(void)
{
    struct fake f; initialize(&f); f.count=1;
    const uint32_t codes[]={R10};
    f.modes[0]=(struct native_mode){R10,UINT32_MAX,UINT32_MAX};
    f.modes[1]=(struct native_mode){R10,100000,100000}; f.mode_count=2;
    cc_sensor_mode_request r=request(codes,1,70000,70000);
    expect(&f,&r,R10,100000,100000); /* Areas exceed 32 bits. */
    f.modes[0]=(struct native_mode){R10,UINT32_MAX,UINT32_MAX-1};
    f.modes[1]=(struct native_mode){R10,UINT32_MAX-1,UINT32_MAX-2};
    r.width=UINT32_MAX-1; r.height=UINT32_MAX-2;
    expect(&f,&r,R10,UINT32_MAX-1,UINT32_MAX-2);
    r.width=2000000000; r.height=2000000000;
    expect(&f,&r,R10,UINT32_MAX,UINT32_MAX-1); /* Closer ratio beats smaller area beyond double precision. */
    f.modes[0]=(struct native_mode){R10,UINT32_MAX,UINT32_MAX};
    f.modes[1]=(struct native_mode){R10,UINT32_MAX-1,UINT32_MAX-1};
    r.width=3000000000; r.height=1000000000;
    expect(&f,&r,R10,UINT32_MAX-1,UINT32_MAX-1); /* Ratio cross-products would exceed 64 bits. */
    f.modes[0]=(struct native_mode){R10,UINT32_MAX,3};
    f.modes[1]=(struct native_mode){R10,UINT32_MAX-1,2};
    r.width=UINT32_MAX-1; r.height=1;
    expect(&f,&r,R10,UINT32_MAX-1,2); /* Compare unequal integer quotients. */
}
static void test_reference_vectors(void)
{
    /* Generated by compiling the unmodified CameraSensorLegacy::getFormat()
     * from the archived libcamera 0.7.2 camera_sensor_legacy.cpp, using the
     * mode table in initialize() and code preference {R12,R10,B10}.
     * Archive SHA-256: ce4d60131c5fd218153f99cf1271375271815cc183a9ac7b270f18d3588015d5.
     * Reference C++ is not needed to build or run these tests. */
    static const struct {
        uint32_t width,height,max_width,max_height,code,out_width,out_height;
    } vectors[]={
        {640,480,0,0,R10,640,480},
        {1000,750,0,0,R12,1280,960},
        {1280,720,0,0,R10,1280,720},
        {1920,1080,0,0,R12,1920,1080},
        {1800,1200,0,0,R10,2592,1944},
        {1800,1200,2000,1500,0,0,0},
        {1600,1000,2000,1500,R12,1920,1080},
        {2592,1944,2592,1944,R10,2592,1944},
        {1,1,0,0,R10,640,480},
        {100,900,0,0,R12,1280,960},
        {2000,1100,0,0,R12,3840,2160},
        {4000,3000,0,0,0,0,0},
        {640,481,1280,960,B10,800,600},
        {2000,1000,3840,2160,R12,3840,2160},
        {640,480,799,600,R10,640,480},
        {2592,1944,3000,2000,R10,2592,1944}
    };
    struct fake f; initialize(&f);
    const uint32_t codes[]={R12,R10,B10};
    for (size_t i=0; i<sizeof(vectors)/sizeof(vectors[0]); ++i) {
        cc_sensor_mode_request r=request(codes,3,vectors[i].width,vectors[i].height);
        r.max_width=vectors[i].max_width; r.max_height=vectors[i].max_height;
        if (!vectors[i].code) expect_error(&f,&r,ENOENT);
        else expect(&f,&r,vectors[i].code,vectors[i].out_width,vectors[i].out_height);
    }
}
static void test_errors_and_bounds(void)
{
    struct fake f; initialize(&f);
    const uint32_t codes[]={R10,R12};
    cc_sensor_mode_request r=request(codes,2,640,480);
    cc_sensor_format out;
    CHECK(cc_camera_select_sensor_mode(NULL,&r,&out)==-EINVAL);
    CHECK(cc_camera_select_sensor_mode(&f.camera,NULL,&out)==-EINVAL);
    CHECK(cc_camera_select_sensor_mode(&f.camera,&r,NULL)==-EINVAL);
    CHECK(cc_cfe_select_sensor_mode(NULL,&r,&out)==-EINVAL);
    cc_sensor_mode_request invalid=r; invalid.mbus_codes=NULL; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.num_mbus_codes=0; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.num_mbus_codes=257; expect_error(&f,&invalid,E2BIG);
    invalid=r; invalid.width=0; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.height=0; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.max_width=1920; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.max_height=1080; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.max_width=639; invalid.max_height=480; expect_error(&f,&invalid,EINVAL);
    invalid=r; invalid.max_width=640; invalid.max_height=479; expect_error(&f,&invalid,EINVAL);
    const uint32_t zero=0; invalid=r; invalid.mbus_codes=&zero; invalid.num_mbus_codes=1;
    expect_error(&f,&invalid,EINVAL);
    CHECK(!f.code_calls && !f.size_calls && !f.format_calls);
    f.camera.sensor_fd=-1; expect_error(&f,&r,ENOTSUP); f.camera.sensor_fd=11;
    for (unsigned bad=1; bad<=5; ++bad) { f.bad_code=bad; expect_error(&f,&r,EPROTO); }
    f.bad_code=0;
    for (unsigned bad=1; bad<=9; ++bad) { f.bad_size=bad; expect_error(&f,&r,EPROTO); }
    f.bad_size=0;
    f.codes[1]=R10; expect_error(&f,&r,EPROTO); f.codes[1]=R12;
    f.count=0; expect_error(&f,&r,ENOENT); f.count=3;
    f.fail_request=VIDIOC_SUBDEV_ENUM_MBUS_CODE; f.fail_index=1; f.fail_error=EIO;
    expect_error(&f,&r,EIO); /* A later failure must not return the earlier candidate. */
    f.fail_index=0; f.fail_error=ENOTTY; expect_error(&f,&r,ENOTSUP);
    f.fail_request=VIDIOC_SUBDEV_ENUM_FRAME_SIZE; f.fail_error=EIO; f.fail_index=1;
    expect_error(&f,&r,EIO);
    f.fail_index=0; f.fail_error=ENOTTY; expect_error(&f,&r,ENOTSUP); f.fail_request=0;
    f.endless_codes=true; expect_error(&f,&r,E2BIG); f.endless_codes=false;
    f.endless_codes=true; f.fail_request=VIDIOC_SUBDEV_ENUM_MBUS_CODE;
    f.fail_index=256; f.fail_error=EINVAL;
    expect(&f,&r,R10,640,480); /* Exactly 256 codes are accepted. */
    f.endless_codes=false; f.fail_request=0;
    f.endless_sizes=true; expect_error(&f,&r,E2BIG); f.endless_sizes=false;
    /* Exactly 4096 advertised sizes are accepted, not mistaken for overflow. */
    f.count=1; f.endless_sizes=true; f.fail_request=VIDIOC_SUBDEV_ENUM_FRAME_SIZE;
    f.fail_index=4096; f.fail_error=EINVAL;
    expect(&f,&r,R10,640,480);
    cc_sensor_mode mode={0}, saved=mode;
    f.bad_size=2; CHECK(cc_camera_enum_sensor_mode(&f.camera,0,0,&mode)==-EPROTO);
    CHECK(!memcmp(&mode,&saved,sizeof(mode)));
}

int main(void)
{
    test_ranking(); test_large_dimensions(); test_reference_vectors(); test_errors_and_bounds();
    puts("Sensor mode ranking, exact arithmetic, read-only negotiation and bounded native replies passed");
    return 0;
}
