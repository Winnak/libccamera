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
#include "rpi_sensor_mode_vectors.h"
struct fake {
    cc_camera camera;
    struct cc_media media;
    struct cc_entity entity;
    struct media_pad_desc pads[3];
    bool streams[1];
    cc_stream stream;
    uint32_t codes[32];
    size_t count;
    struct native_mode modes[32];
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

static void expect_rpi(struct fake *f, const cc_rpi_sensor_mode_request *r,
                        uint32_t code, uint32_t width, uint32_t height)
{
    cc_sensor_format out;
    OK(cc_camera_select_rpi_sensor_mode(&f->camera,r,&out));
    CHECK(out.mbus_code==code && out.width==width && out.height==height);
}
static void expect_rpi_error(struct fake *f, const cc_rpi_sensor_mode_request *r, int error)
{
    cc_sensor_format out={0xdeadbeef,1234,5678}, saved=out;
    CHECK(cc_camera_select_rpi_sensor_mode(&f->camera,r,&out)==-error);
    CHECK(!memcmp(&out,&saved,sizeof(out)));
}
static void test_rpi_reference(void)
{
    struct fake f; initialize(&f);
    f.count=0; f.mode_count=sizeof(rpi_reference_modes)/sizeof(rpi_reference_modes[0]);
    memcpy(f.modes,rpi_reference_modes,sizeof(rpi_reference_modes));
    for (size_t m=0; m<f.mode_count; ++m) {
        size_t c=0;
        while (c<f.count && f.codes[c]!=f.modes[m].code) ++c;
        if (c==f.count) f.codes[f.count++]=f.modes[m].code;
    }
    uint32_t codes[32]; memcpy(codes,f.codes,sizeof(codes));
    cc_rpi_sensor_mode_request r={.mbus_codes=codes,.num_mbus_codes=f.count};
    for (unsigned permutation=0; permutation<2; ++permutation) {
        for (size_t v=0; v<sizeof(rpi_reference_vectors)/sizeof(rpi_reference_vectors[0]); ++v) {
            r.width=rpi_reference_vectors[v].width; r.height=rpi_reference_vectors[v].height;
            r.bit_depth=rpi_reference_vectors[v].depth;
            expect_rpi(&f,&r,rpi_reference_vectors[v].code,
                rpi_reference_vectors[v].out_width,rpi_reference_vectors[v].out_height);
        }
        /* Driver and caller order must not affect upstream's sorted tie rules. */
        for (size_t i=0; i<f.count/2; ++i) {
            uint32_t t=f.codes[i]; f.codes[i]=f.codes[f.count-1-i]; f.codes[f.count-1-i]=t;
            t=codes[i]; codes[i]=codes[f.count-1-i]; codes[f.count-1-i]=t;
        }
        for (size_t i=0; i<f.mode_count/2; ++i) {
            struct native_mode t=f.modes[i]; f.modes[i]=f.modes[f.mode_count-1-i]; f.modes[f.mode_count-1-i]=t;
        }
    }
}
static void test_rpi_ranking(void)
{
    struct fake f; initialize(&f);
    const uint32_t codes[]={R12,R10,B10};
    cc_rpi_sensor_mode_request r={.mbus_codes=codes,.num_mbus_codes=3,.width=640,.height=480,.bit_depth=12};
    expect_rpi(&f,&r,R12,1280,960); /* Bit-depth penalty beats exact dimensions. */
    r.bit_depth=10; r.width=5000; r.height=4000;
    expect_rpi(&f,&r,R12,3840,2160); /* Smaller-than-request fallback. */
    f.count=2; f.codes[0]=B10; f.codes[1]=R10;
    f.mode_count=2; f.modes[0]=(struct native_mode){R10,640,480};
    f.modes[1]=(struct native_mode){B10,640,480};
    r.width=640; r.height=480;
    expect_rpi(&f,&r,R10,640,480); /* Greater code wins, regardless of code order. */
    f.count=1; f.codes[0]=R10; r.num_mbus_codes=1; r.mbus_codes=&f.codes[0];
    f.modes[0]=(struct native_mode){R10,2080,1560};
    f.modes[1]=(struct native_mode){R10,640,480}; r.width=800; r.height=600;
    expect_rpi(&f,&r,R10,2080,1560); /* Equal score: larger area wins. */
    f.modes[0]=(struct native_mode){R10,800,100};
    f.modes[1]=(struct native_mode){R10,100,800}; r.width=10; r.height=10;
    expect_rpi(&f,&r,R10,800,100); /* Equal area and score: larger width wins. */
    f.modes[0]=(struct native_mode){R10,UINT32_MAX,UINT32_MAX};
    f.modes[1]=(struct native_mode){R10,640,480}; r.width=UINT32_MAX; r.height=UINT32_MAX;
    expect_rpi(&f,&r,R10,UINT32_MAX,UINT32_MAX);
    f.range=true; r.width=1000; r.height=750;
    expect_rpi(&f,&r,R10,640,480); /* No invented intermediate range values. */
    f.stream.configured=f.stream.streaming=f.stream.allocated=true;
    expect_rpi(&f,&r,R10,640,480);
    CHECK(f.stream.configured && f.stream.streaming && f.stream.allocated && !f.camera.sensor_stream);
    static const struct { uint32_t code,depth; } formats[]={
#define RAW(bits) \
        {MEDIA_BUS_FMT_SBGGR##bits##_1X##bits,bits}, {MEDIA_BUS_FMT_SGBRG##bits##_1X##bits,bits}, \
        {MEDIA_BUS_FMT_SGRBG##bits##_1X##bits,bits}, {MEDIA_BUS_FMT_SRGGB##bits##_1X##bits,bits}, \
        {MEDIA_BUS_FMT_Y##bits##_1X##bits,bits}
        RAW(8), RAW(10), RAW(12), RAW(14), RAW(16)
#undef RAW
    };
    f.range=false; f.mode_count=1; r.width=640; r.height=480;
    for (size_t i=0; i<sizeof(formats)/sizeof(formats[0]); ++i) {
        f.codes[0]=formats[i].code; r.bit_depth=formats[i].depth;
        f.modes[0]=(struct native_mode){f.codes[0],640,480};
        expect_rpi(&f,&r,f.codes[0],640,480);
    }
}
static void test_rpi_errors(void)
{
    struct fake f; initialize(&f);
    const uint32_t codes[]={R10,R12};
    cc_rpi_sensor_mode_request r={.mbus_codes=codes,.num_mbus_codes=2,.width=640,.height=480,.bit_depth=10};
    cc_sensor_format out;
    CHECK(cc_camera_select_rpi_sensor_mode(NULL,&r,&out)==-EINVAL);
    CHECK(cc_camera_select_rpi_sensor_mode(&f.camera,&r,NULL)==-EINVAL);
    CHECK(cc_cfe_select_rpi_sensor_mode(NULL,&r,&out)==-EINVAL);
    expect_rpi_error(&f,NULL,EINVAL);
    cc_rpi_sensor_mode_request invalid=r; invalid.mbus_codes=NULL; expect_rpi_error(&f,&invalid,EINVAL);
    invalid=r; invalid.num_mbus_codes=0; expect_rpi_error(&f,&invalid,EINVAL);
    invalid=r; invalid.num_mbus_codes=257; expect_rpi_error(&f,&invalid,E2BIG);
    invalid=r; invalid.width=0; expect_rpi_error(&f,&invalid,EINVAL);
    invalid=r; invalid.height=0; expect_rpi_error(&f,&invalid,EINVAL);
    invalid=r; invalid.bit_depth=0; expect_rpi_error(&f,&invalid,EINVAL);
    invalid=r; invalid.bit_depth=11; expect_rpi_error(&f,&invalid,EINVAL);
    invalid=r; invalid.num_mbus_codes=1;
    const uint32_t zero=0, unsupported=MEDIA_BUS_FMT_UYVY8_2X8;
    invalid.mbus_codes=&zero; expect_rpi_error(&f,&invalid,EINVAL);
    invalid.mbus_codes=&unsupported; expect_rpi_error(&f,&invalid,ENOTSUP);
    CHECK(!f.code_calls && !f.size_calls && !f.format_calls);
    f.camera.sensor_fd=-1; expect_rpi_error(&f,&r,ENOTSUP); f.camera.sensor_fd=11;
    for (unsigned bad=1; bad<=5; ++bad) { f.bad_code=bad; expect_rpi_error(&f,&r,EPROTO); }
    f.bad_code=0;
    for (unsigned bad=1; bad<=9; ++bad) { f.bad_size=bad; expect_rpi_error(&f,&r,EPROTO); }
    f.bad_size=0;
    f.codes[1]=R10; expect_rpi_error(&f,&r,EPROTO); f.codes[1]=R12;
    f.count=0; expect_rpi_error(&f,&r,ENOENT); f.count=3;
    f.mode_count=0; expect_rpi_error(&f,&r,ENOENT); initialize(&f);
    f.fail_request=VIDIOC_SUBDEV_ENUM_MBUS_CODE; f.fail_index=1; f.fail_error=EIO;
    expect_rpi_error(&f,&r,EIO);
    f.fail_index=0; f.fail_error=ENOTTY; expect_rpi_error(&f,&r,ENOTSUP);
    f.fail_request=VIDIOC_SUBDEV_ENUM_FRAME_SIZE; f.fail_error=EIO; f.fail_index=1;
    expect_rpi_error(&f,&r,EIO);
    f.fail_index=0; f.fail_error=ENOTTY; expect_rpi_error(&f,&r,ENOTSUP); f.fail_request=0;
    f.endless_codes=true; expect_rpi_error(&f,&r,E2BIG); f.endless_codes=false;
    f.endless_sizes=true; expect_rpi_error(&f,&r,E2BIG); f.endless_sizes=false;
    f.count=1; f.endless_sizes=true; f.fail_request=VIDIOC_SUBDEV_ENUM_FRAME_SIZE;
    f.fail_index=4096; f.fail_error=EINVAL;
    expect_rpi(&f,&r,R10,640,480);
}

int main(int argc, char **argv)
{
    if (argc==2 && !strcmp(argv[1],"rpi")) {
        test_rpi_reference(); test_rpi_ranking(); test_rpi_errors();
        puts("Pi mode scoring, upstream fixtures, tie rules and bounded read-only queries passed");
        return 0;
    }
    test_ranking(); test_large_dimensions(); test_reference_vectors(); test_errors_and_bounds();
    puts("Sensor mode ranking, exact arithmetic, read-only negotiation and bounded native replies passed");
    return 0;
}
