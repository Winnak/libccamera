/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Algebra follows libcamera 0.7.2 orientation.cpp and transform.cpp,
 * Copyright (C) 2023 Ideas On Board Oy / (C) 2020 Raspberry Pi Ltd.
 * Sensor policy follows CameraSensorLegacy/Raw::computeTransform(). */
#include "internal.h"
#include <errno.h>

static bool valid_orientation(enum cc_orientation o) { return (unsigned)o>=1 && (unsigned)o<=8; }
static bool valid_transform(enum cc_transform t) { return (unsigned)t<=7; }
static const uint32_t orientation_transform[]={0,0,1,3,2,4,5,7,6};
static const enum cc_orientation transform_orientation[]={1,2,4,3,5,6,8,7};

int cc_orientation_from_rotation(int32_t angle, enum cc_orientation *out)
{
    if (!out) return -EINVAL;
    angle%=360;
    if (angle<0) angle+=360;
    switch (angle) {
    case 0: *out=CC_ORIENTATION_ROTATE_0; return 0;
    case 90: *out=CC_ORIENTATION_ROTATE_90; return 0;
    case 180: *out=CC_ORIENTATION_ROTATE_180; return 0;
    case 270: *out=CC_ORIENTATION_ROTATE_270; return 0;
    default: return -EINVAL;
    }
}

int cc_transform_compose(enum cc_transform first, enum cc_transform second, enum cc_transform *out)
{
    if (!out || !valid_transform(first) || !valid_transform(second)) return -EINVAL;
    uint32_t reordered=(uint32_t)second;
    if (first&CC_TRANSFORM_TRANSPOSE)
        reordered=(reordered&4U)|((reordered&1U)<<1)|((reordered&2U)>>1);
    *out=(enum cc_transform)(reordered^(uint32_t)first);
    return 0;
}

int cc_transform_inverse(enum cc_transform t, enum cc_transform *out)
{
    if (!out || !valid_transform(t)) return -EINVAL;
    *out=t==CC_TRANSFORM_ROTATE_90 ? CC_TRANSFORM_ROTATE_270:
         t==CC_TRANSFORM_ROTATE_270 ? CC_TRANSFORM_ROTATE_90:t;
    return 0;
}

int cc_orientation_transform(enum cc_orientation desired, enum cc_orientation base, enum cc_transform *out)
{
    if (!out || !valid_orientation(desired) || !valid_orientation(base)) return -EINVAL;
    enum cc_transform inverse;
    int ret=cc_transform_inverse((enum cc_transform)orientation_transform[base],&inverse);
    if (ret<0) return ret;
    return cc_transform_compose(inverse,(enum cc_transform)orientation_transform[desired],out);
}

int cc_orientation_apply(enum cc_orientation base, enum cc_transform t, enum cc_orientation *out)
{
    if (!out || !valid_orientation(base) || !valid_transform(t)) return -EINVAL;
    enum cc_transform result;
    int ret=cc_transform_compose((enum cc_transform)orientation_transform[base],t,&result);
    if (ret<0) return ret;
    *out=transform_orientation[result];
    return 0;
}

struct orientation_info {
    enum cc_orientation mounting;
    cc_control_info controls[2];
    bool present[2], writable[2];
};

static int orientation_info(cc_camera *camera, struct orientation_info *out)
{
    if (!camera) return -EINVAL;
    int ret=cc_camera_fd(camera);
    if (ret<0) return ret;
    struct orientation_info info={.mounting=CC_ORIENTATION_ROTATE_0};
    if (camera->sensor_fd>=0) {
        cc_control_info rotation;
        ret=cc_sensor_property_control(camera,V4L2_CID_CAMERA_SENSOR_ROTATION,V4L2_CTRL_TYPE_INTEGER,&rotation);
        if (!ret) (void)cc_orientation_from_rotation((int32_t)rotation.default_value,&info.mounting);
        else if (ret!=-ENOENT) return ret;
        const uint32_t ids[]={V4L2_CID_HFLIP,V4L2_CID_VFLIP};
        for (size_t i=0; i<2; ++i) {
            cc_control_info *c=&info.controls[i];
            ret=cc_sensor_property_control(camera,ids[i],V4L2_CTRL_TYPE_BOOLEAN,c);
            if (ret==-ENOENT) continue;
            if (ret<0) return ret;
            if (c->minimum<0 || c->maximum>1 || c->step!=1) return -EPROTO;
            info.present[i]=true;
            info.writable[i]=!(c->flags&(V4L2_CTRL_FLAG_READ_ONLY|V4L2_CTRL_FLAG_INACTIVE|V4L2_CTRL_FLAG_WRITE_ONLY)) &&
                c->minimum==0 && c->maximum==1;
        }
    }
    *out=info;
    return 0;
}

static int select_orientation(const struct orientation_info *info, enum cc_orientation *o, enum cc_transform *t)
{
    int ret=cc_orientation_transform(*o,info->mounting,t);
    if (ret<0) return ret;
    if (!info->writable[0] || !info->writable[1] || (*t&CC_TRANSFORM_TRANSPOSE)) {
        *o=info->mounting;
        *t=CC_TRANSFORM_IDENTITY;
    }
    return 0;
}

int cc_camera_try_orientation(cc_camera *camera, enum cc_orientation *orientation, enum cc_transform *transform)
{
    if (!orientation || !transform || !valid_orientation(*orientation)) return -EINVAL;
    struct orientation_info info;
    int ret=orientation_info(camera,&info);
    if (ret<0) return ret;
    enum cc_orientation o=*orientation;
    enum cc_transform t;
    ret=select_orientation(&info,&o,&t);
    if (ret<0) return ret;
    *orientation=o; *transform=t;
    return 0;
}

int cc_camera_get_orientation(cc_camera *camera, enum cc_orientation *out)
{
    if (!out) return -EINVAL;
    struct orientation_info info;
    int ret=orientation_info(camera,&info);
    if (ret<0) return ret;
    cc_control controls[2];
    size_t count=0;
    for (size_t i=0; i<2; ++i) if (info.present[i]) {
        if (info.controls[i].flags&V4L2_CTRL_FLAG_WRITE_ONLY) return -ENOTSUP;
        controls[count++]=(cc_control){.id=info.controls[i].id};
    }
    if (count) {
        ret=cc_camera_get_controls(camera,controls,count,NULL);
        if (ret<0) return ret==-ENOTTY ? -ENOTSUP:ret;
    }
    uint32_t transform=0;
    for (size_t i=0; i<count; ++i) {
        if (controls[i].value<0 || controls[i].value>1) return -EPROTO;
        if (controls[i].value) transform|=controls[i].id==V4L2_CID_HFLIP ? 1U:2U;
    }
    return cc_orientation_apply(info.mounting,(enum cc_transform)transform,out);
}

/* Predict the controls configure_oriented would apply, including fixed flips.
 * Only layout-changing controls alter the Bayer phase prediction. */
int cc_camera_predict_orientation(cc_camera *camera, enum cc_orientation *orientation, uint32_t *delta)
{
    if (!orientation || !delta || !valid_orientation(*orientation)) return -EINVAL;
    struct orientation_info info;
    int ret=orientation_info(camera,&info);
    if (ret<0) return ret;
    enum cc_orientation o=*orientation; enum cc_transform target;
    ret=select_orientation(&info,&o,&target);
    if (ret<0) return ret;
    cc_control controls[2]; size_t count=0;
    for (size_t i=0; i<2; ++i) if (info.present[i]) {
        if (info.controls[i].flags&V4L2_CTRL_FLAG_WRITE_ONLY) return -ENOTSUP;
        controls[count++]=(cc_control){.id=info.controls[i].id};
    }
    if (count && (ret=cc_camera_get_controls(camera,controls,count,NULL))) return ret;
    uint32_t applied=0,change=0;
    for (size_t i=0; i<count; ++i) {
        size_t axis=controls[i].id==V4L2_CID_HFLIP ? 0U:1U;
        if (controls[i].value<0 || controls[i].value>1) return -EPROTO;
        uint32_t current=(uint32_t)controls[i].value;
        uint32_t next=info.writable[axis] ? ((uint32_t)target>>axis)&1U:current;
        applied|=next<<axis;
        if (info.controls[axis].flags&V4L2_CTRL_FLAG_MODIFY_LAYOUT) change|=(current^next)<<axis;
    }
    ret=cc_orientation_apply(info.mounting,(enum cc_transform)applied,&o);
    if (ret<0) return ret;
    *orientation=o; *delta=change; return 0;
}

/* Called only by oriented format negotiation after allocation/busy guards. */
int cc_camera_apply_orientation(cc_camera *camera, enum cc_orientation *orientation)
{
    if (!orientation || !valid_orientation(*orientation)) return -EINVAL;
    struct orientation_info info;
    int ret=orientation_info(camera,&info);
    if (ret<0) return ret;
    enum cc_orientation o=*orientation;
    enum cc_transform transform;
    ret=select_orientation(&info,&o,&transform);
    if (ret<0) return ret;
    cc_control controls[2];
    size_t count=0;
    for (size_t i=0; i<2; ++i) if (info.writable[i])
        controls[count++]=(cc_control){.id=info.controls[i].id,.value=((uint32_t)transform>>i)&1U};
    if (count) {
        camera->stream->configured=false;
        cc_control expected[2];
        for (size_t i=0; i<count; ++i) expected[i]=controls[i];
        ret=cc_controls_apply_io(&camera->io,camera->sensor_fd,controls,count,false);
        if (ret<0) return ret;
        for (size_t i=0; i<count; ++i) if (controls[i].value!=expected[i].value) return -ERANGE;
    }
    *orientation=o;
    return 0;
}
