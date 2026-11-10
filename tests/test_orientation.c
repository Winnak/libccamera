/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <ccamera/capture.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
#include "orientation_vectors.h"

struct fake {
    cc_camera camera;
    cc_video_device device;
    cc_stream stream;
    int32_t rotation;
    uint32_t present, read_only, disabled, write_only, flip;
    bool no_rotation;
    unsigned bad;
    unsigned long fail_request;
    int error;
    unsigned calls, gets;
};

static int fake_ioctl(void *context, int fd, unsigned long request, void *argument)
{
    struct fake *f=context;
    CHECK(fd==11);
    ++f->calls;
    if (request==f->fail_request) return -f->error;
    if (request==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=argument;
        q->elems=1; q->elem_size=4; q->step=1;
        if (q->id==V4L2_CID_CAMERA_SENSOR_ROTATION) {
            if (f->no_rotation) return -EINVAL;
            q->type=V4L2_CTRL_TYPE_INTEGER; q->minimum=INT32_MIN; q->maximum=INT32_MAX;
            q->default_value=f->rotation;
        } else {
            CHECK(q->id==V4L2_CID_HFLIP || q->id==V4L2_CID_VFLIP);
            uint32_t bit=q->id==V4L2_CID_HFLIP ? 1U:2U;
            if (!(f->present&bit)) return -EINVAL;
            q->type=V4L2_CTRL_TYPE_BOOLEAN; q->maximum=1;
            if (f->read_only&bit) q->flags|=V4L2_CTRL_FLAG_READ_ONLY;
            if (f->disabled&bit) q->flags|=V4L2_CTRL_FLAG_DISABLED;
            if (f->write_only&bit) q->flags|=V4L2_CTRL_FLAG_WRITE_ONLY;
        }
        switch (f->bad) {
        case 1: ++q->id; break;
        case 2: q->type=V4L2_CTRL_TYPE_STRING; break;
        case 3: q->elems=2; break;
        case 4: q->nr_of_dims=1; break;
        case 5: q->default_value=q->maximum+1; break;
        case 6: if (q->id==V4L2_CID_HFLIP) q->step=0; break;
        }
        return 0;
    }
    if (request==VIDIOC_G_EXT_CTRLS) {
        ++f->gets;
        struct v4l2_ext_controls *q=argument;
        CHECK(q->count<=2);
        for (uint32_t i=0; i<q->count; ++i) {
            uint32_t bit=q->controls[i].id==V4L2_CID_HFLIP ? 1U:2U;
            q->controls[i].value=f->bad==7 ? 2:(f->flip&bit)!=0;
        }
        return 0;
    }
    /* Read-only orientation queries must never touch hardware configuration. */
    CHECK(false);
    return -ENOTTY;
}
static const struct cc_io_ops ops={.ioctl=fake_ioctl};
static void initialize(struct fake *f)
{
    memset(f,0,sizeof(*f)); f->present=3; f->error=EIO;
    f->device=(cc_video_device){.fd=20};
    f->stream=(cc_stream){.device=&f->device};
    f->camera=(cc_camera){.io={&ops,f},.fd=20,.sensor_fd=11,.stream=&f->stream,.device=&f->device};
}

static void test_reference_algebra(void)
{
    for (size_t i=0; i<sizeof(rotation_vectors)/sizeof(rotation_vectors[0]); ++i) {
        enum cc_orientation out=CC_ORIENTATION_ROTATE_270_MIRROR;
        int ret=cc_orientation_from_rotation(rotation_vectors[i].angle,&out);
        if (rotation_vectors[i].valid) CHECK(!ret && (uint32_t)out==rotation_vectors[i].orientation);
        else CHECK(ret==-EINVAL && out==CC_ORIENTATION_ROTATE_270_MIRROR);
    }
    for (uint32_t a=0; a<8; ++a) {
        enum cc_transform inverse;
        OK(cc_transform_inverse((enum cc_transform)a,&inverse)); CHECK((uint32_t)inverse==inverse_vectors[a]);
        for (uint32_t b=0; b<8; ++b) {
            enum cc_transform result;
            OK(cc_transform_compose((enum cc_transform)a,(enum cc_transform)b,&result));
            CHECK((uint32_t)result==compose_vectors[a][b]);
            enum cc_orientation o;
            OK(cc_orientation_apply((enum cc_orientation)(a+1),(enum cc_transform)b,&o));
            CHECK((uint32_t)o==apply_vectors[a][b]);
            OK(cc_orientation_transform((enum cc_orientation)(a+1),(enum cc_orientation)(b+1),&result));
            CHECK((uint32_t)result==between_vectors[a][b]);
            OK(cc_orientation_apply((enum cc_orientation)(b+1),result,&o)); CHECK((uint32_t)o==a+1);
        }
    }
    enum cc_transform t=CC_TRANSFORM_VFLIP;
    enum cc_orientation o=CC_ORIENTATION_ROTATE_180;
    const int invalid[]={-1,8,INT_MAX,INT_MIN};
    for (size_t i=0; i<sizeof(invalid)/sizeof(invalid[0]); ++i) {
        CHECK(cc_transform_compose((enum cc_transform)invalid[i],CC_TRANSFORM_IDENTITY,&t)==-EINVAL);
        CHECK(cc_transform_compose(CC_TRANSFORM_IDENTITY,(enum cc_transform)invalid[i],&t)==-EINVAL);
        CHECK(cc_transform_inverse((enum cc_transform)invalid[i],&t)==-EINVAL);
        CHECK(cc_orientation_apply(CC_ORIENTATION_ROTATE_0,(enum cc_transform)invalid[i],&o)==-EINVAL);
    }
    CHECK(t==CC_TRANSFORM_VFLIP && o==CC_ORIENTATION_ROTATE_180);
    CHECK(cc_orientation_transform((enum cc_orientation)0,CC_ORIENTATION_ROTATE_0,&t)==-EINVAL);
    CHECK(cc_orientation_transform(CC_ORIENTATION_ROTATE_0,(enum cc_orientation)9,&t)==-EINVAL);
    CHECK(cc_orientation_apply((enum cc_orientation)-1,CC_TRANSFORM_IDENTITY,&o)==-EINVAL);
    CHECK(cc_transform_inverse(CC_TRANSFORM_IDENTITY,NULL)==-EINVAL);
    CHECK(cc_transform_compose(CC_TRANSFORM_IDENTITY,CC_TRANSFORM_IDENTITY,NULL)==-EINVAL);
    CHECK(cc_orientation_from_rotation(0,NULL)==-EINVAL);
    CHECK(cc_orientation_transform(CC_ORIENTATION_ROTATE_0,CC_ORIENTATION_ROTATE_0,NULL)==-EINVAL);
    CHECK(cc_orientation_apply(CC_ORIENTATION_ROTATE_0,CC_TRANSFORM_IDENTITY,NULL)==-EINVAL);
}

static void test_reference_sensor_policy(void)
{
    struct fake f;
    for (size_t i=0; i<sizeof(sensor_orientation_vectors)/sizeof(sensor_orientation_vectors[0]); ++i) {
        initialize(&f); f.rotation=sensor_orientation_vectors[i].rotation;
        f.read_only=sensor_orientation_vectors[i].support ? 0:3;
        enum cc_orientation o=(enum cc_orientation)sensor_orientation_vectors[i].requested;
        enum cc_transform t;
        OK(cc_camera_try_orientation(&f.camera,&o,&t));
        CHECK((uint32_t)o==sensor_orientation_vectors[i].orientation && (uint32_t)t==sensor_orientation_vectors[i].transform);
        CHECK(!f.gets && !f.stream.configured && !f.stream.allocated);
        f.stream.configured=f.stream.allocated=f.stream.streaming=true;
        o=(enum cc_orientation)sensor_orientation_vectors[i].requested;
        OK(cc_camera_try_orientation(&f.camera,&o,&t));
        CHECK((uint32_t)o==sensor_orientation_vectors[i].orientation && (uint32_t)t==sensor_orientation_vectors[i].transform);
    }
}

static void test_live_orientation(void)
{
    struct fake f; enum cc_orientation o;
    const int32_t rotations[]={0,90,180,270};
    const uint32_t base[]={1,8,3,6};
    for (size_t r=0; r<4; ++r) for (uint32_t flips=0; flips<4; ++flips) {
        initialize(&f); f.rotation=rotations[r]; f.flip=flips;
        OK(cc_camera_get_orientation(&f.camera,&o));
        CHECK((uint32_t)o==apply_vectors[base[r]-1][flips] && f.gets==1);
    }
    initialize(&f); f.camera.sensor_fd=-1;
    OK(cc_camera_get_orientation(&f.camera,&o)); CHECK(o==CC_ORIENTATION_ROTATE_0 && !f.calls);
    enum cc_transform t;
    o=CC_ORIENTATION_ROTATE_90; OK(cc_camera_try_orientation(&f.camera,&o,&t));
    CHECK(o==CC_ORIENTATION_ROTATE_0 && t==CC_TRANSFORM_IDENTITY && !f.calls);
    for (uint32_t present=0; present<4; ++present) {
        initialize(&f); f.present=present; f.rotation=180; f.flip=present;
        o=CC_ORIENTATION_ROTATE_0; OK(cc_camera_try_orientation(&f.camera,&o,&t));
        CHECK(o==(present==3 ? CC_ORIENTATION_ROTATE_0:CC_ORIENTATION_ROTATE_180));
        CHECK(t==(present==3 ? CC_TRANSFORM_ROTATE_180:CC_TRANSFORM_IDENTITY));
        OK(cc_camera_get_orientation(&f.camera,&o)); CHECK((uint32_t)o==apply_vectors[2][present]);
    }
    initialize(&f); f.disabled=3; f.flip=3; f.no_rotation=true;
    OK(cc_camera_get_orientation(&f.camera,&o)); CHECK(o==CC_ORIENTATION_ROTATE_0 && !f.gets);
    initialize(&f); f.write_only=1;
    CHECK(cc_camera_get_orientation(&f.camera,&o)==-ENOTSUP);
}

static void test_atomic_failures(void)
{
    struct fake f; initialize(&f);
    enum cc_orientation o=CC_ORIENTATION_ROTATE_270_MIRROR;
    enum cc_transform t=CC_TRANSFORM_TRANSPOSE;
    CHECK(cc_camera_try_orientation(NULL,&o,&t)==-EINVAL && o==CC_ORIENTATION_ROTATE_270_MIRROR && t==CC_TRANSFORM_TRANSPOSE);
    CHECK(cc_camera_try_orientation(&f.camera,NULL,&t)==-EINVAL);
    CHECK(cc_camera_try_orientation(&f.camera,&o,NULL)==-EINVAL);
    CHECK(cc_camera_get_orientation(NULL,&o)==-EINVAL && cc_camera_get_orientation(&f.camera,NULL)==-EINVAL);
    CHECK(cc_cfe_try_orientation(NULL,&o,&t)==-EINVAL && cc_cfe_get_orientation(NULL,&o)==-EINVAL);
    CHECK(cc_capture_get_orientation(NULL,&o)==-EINVAL);
    CHECK(cc_camera_configure_oriented(NULL,NULL,&o)==-EINVAL && cc_cfe_configure_oriented(NULL,NULL,&o)==-EINVAL);
    f.device.fd=-1;
    CHECK(cc_camera_try_orientation(&f.camera,&o,&t)==-ENODEV && cc_camera_get_orientation(&f.camera,&o)==-ENODEV);
    f.device.fd=20;
    for (f.bad=1; f.bad<=6; ++f.bad) {
        CHECK(cc_camera_try_orientation(&f.camera,&o,&t)==-EPROTO);
        CHECK(o==CC_ORIENTATION_ROTATE_270_MIRROR && t==CC_TRANSFORM_TRANSPOSE);
        CHECK(cc_camera_get_orientation(&f.camera,&o)==-EPROTO && o==CC_ORIENTATION_ROTATE_270_MIRROR);
    }
    f.bad=7; CHECK(cc_camera_get_orientation(&f.camera,&o)==-EPROTO && o==CC_ORIENTATION_ROTATE_270_MIRROR);
    f.bad=0; f.fail_request=VIDIOC_QUERY_EXT_CTRL;
    CHECK(cc_camera_try_orientation(&f.camera,&o,&t)==-EIO && o==CC_ORIENTATION_ROTATE_270_MIRROR && t==CC_TRANSFORM_TRANSPOSE);
    f.fail_request=VIDIOC_G_EXT_CTRLS;
    CHECK(cc_camera_get_orientation(&f.camera,&o)==-EIO && o==CC_ORIENTATION_ROTATE_270_MIRROR);
}

int main(void)
{
    test_reference_algebra(); test_reference_sensor_policy(); test_live_orientation(); test_atomic_failures();
    puts("Orientation algebra, 144 archived sensor-policy fixtures, live flips and read-only failure guarantees passed");
    return 0;
}
