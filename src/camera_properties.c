/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Property policies follow the archived libcamera 0.7.2 CameraSensorLegacy
 * and UVC handler (Copyright (C) 2018-2019, Google Inc.), plus the subdevice
 * model-name heuristic. No active controls or sensor formats are modified. */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/sensor.h>
#include <errno.h>
#include <limits.h>
#include <string.h>

#define PROPERTY_FORMAT_LIMIT 256U
#define PROPERTY_SIZE_LIMIT 4096U

static bool optional_absent(int ret)
{
    return ret == -EINVAL || ret == -ENOENT || ret == -ENOTTY || ret == -ENOTSUP;
}

size_t cc_camera_model_length(const char *name, size_t length, bool sensor)
{
    size_t size=strnlen(name,length);
    if (sensor) {
        /* Match " [0-9]+-[0-9a-f]{4}", as the reference's regex does. */
        for (size_t i=0; i<size; ++i) {
            if (name[i] != ' ') continue;
            size_t at=i+1, digits=at;
            while (at<size && name[at]>='0' && name[at]<='9') ++at;
            if (at==digits || at==size || name[at++]!='-' || size-at<4) continue;
            size_t hex=0;
            while (hex<4 && ((name[at+hex]>='0' && name[at+hex]<='9') ||
                            (name[at+hex]>='a' && name[at+hex]<='f'))) ++hex;
            if (hex==4) { size=strcspn(name," "); break; }
        }
    }
    return size;
}

void cc_camera_model(char *out, size_t capacity, const char *name, size_t length, bool sensor)
{
    size_t size=cc_camera_model_length(name,length,sensor),written=0;
    for (size_t i=0; i<size; ++i) {
        unsigned char c=(unsigned char)name[i];
        if (!(c&0x80) && written<capacity-1) out[written++]=(char)c;
    }
    out[written]='\0';
}

int cc_sensor_property_control(cc_camera *camera, uint32_t id, uint32_t type, cc_control_info *out)
{
    cc_control_info info;
    int ret=cc_control_query_io(&camera->io,camera->sensor_fd,id,&info);
    if (ret<0) return optional_absent(ret) ? -ENOENT:ret;
    if (info.id!=id || info.type!=type || info.minimum<INT32_MIN || info.maximum>INT32_MAX ||
        info.maximum<info.minimum || info.default_value<info.minimum || info.default_value>info.maximum ||
        info.elems!=1 || info.nr_of_dims || info.flags&V4L2_CTRL_FLAG_HAS_PAYLOAD) return -EPROTO;
    if (info.flags&V4L2_CTRL_FLAG_DISABLED) return -ENOENT;
    *out=info;
    return 0;
}

static int control_default(cc_camera *camera, uint32_t id, uint32_t type, int32_t *value)
{
    cc_control_info info;
    int ret=cc_sensor_property_control(camera,id,type,&info);
    if (!ret) *value=(int32_t)info.default_value;
    return ret;
}

static int sensor_selection(cc_camera *camera, const cc_sensor_stream_info *image,
                             uint32_t target, cc_rectangle *out)
{
    struct cc_subdev_selection q={.which=V4L2_SUBDEV_FORMAT_ACTIVE,
        .pad=image->pad,.stream=image->stream,.target=target};
    int ret=cc_ioctl(&camera->io,camera->sensor_fd,CC_SD_G_SELECTION,&q);
    if (ret<0) return optional_absent(ret) ? -ENOENT:ret;
    if (q.which!=V4L2_SUBDEV_FORMAT_ACTIVE || q.pad!=image->pad || q.stream!=image->stream ||
        q.target!=target || q.r.left<0 || q.r.top<0 || q.r.width<=0 || q.r.height<=0) return -EPROTO;
    *out=(cc_rectangle){q.r.left,q.r.top,(uint32_t)q.r.width,(uint32_t)q.r.height};
    return 0;
}

static int sensor_cfa_transform(cc_camera *camera, uint32_t *transform)
{
    const uint32_t ids[]={V4L2_CID_HFLIP,V4L2_CID_VFLIP};
    cc_control controls[2];
    size_t count=0;
    bool changes_layout=false, readable=true;
    for (size_t i=0; i<2; ++i) {
        cc_control_info info;
        int ret=cc_control_query_io(&camera->io,camera->sensor_fd,ids[i],&info);
        if (ret<0) { if (optional_absent(ret)) continue; return ret; }
        if (info.id!=ids[i] || info.type!=V4L2_CTRL_TYPE_BOOLEAN || info.minimum<0 || info.maximum>1 ||
            info.maximum<info.minimum || info.default_value<info.minimum || info.default_value>info.maximum ||
            info.elems!=1 || info.nr_of_dims || info.flags&V4L2_CTRL_FLAG_HAS_PAYLOAD) return -EPROTO;
        if (info.flags&V4L2_CTRL_FLAG_DISABLED) continue;
        controls[count++]=(cc_control){.id=ids[i]};
        changes_layout|=(info.flags&V4L2_CTRL_FLAG_MODIFY_LAYOUT)!=0;
        readable&=(info.flags&V4L2_CTRL_FLAG_WRITE_ONLY)==0;
    }
    if (!changes_layout) return 0;
    if (!readable) return -ENOTSUP;
    int ret=cc_camera_get_controls(camera,controls,count,NULL);
    if (ret<0) return optional_absent(ret) ? -ENOTSUP:ret;
    for (size_t i=0; i<count; ++i) {
        if (controls[i].value<0 || controls[i].value>1) return -EPROTO;
        if (controls[i].value) *transform|=controls[i].id==V4L2_CID_HFLIP ? 1U:2U;
    }
    return 0;
}

static int sensor_properties(cc_camera *camera, cc_camera_properties *p)
{
    struct cc_entity *entity=camera->media ? cc_media_entity(camera->media,camera->info.sensor_entity):NULL;
    if (!entity) return -ENODEV;
    const char *name=cc_entity_name(entity);
    cc_camera_model(p->model,sizeof(p->model),name,
                entity->name[0] ? sizeof(entity->name):sizeof(entity->desc.name),true);
    if (p->model[0]) p->flags|=CC_CAMERA_PROPERTY_MODEL;
    const cc_sensor_model *model;
    int ret=p->model[0] ? cc_sensor_lookup(p->model,&model):-ENOENT;
    if (!ret) {
        const cc_sensor_properties *s=cc_sensor_get_properties(model);
        if (s->features&CC_SENSOR_CELL_SIZE) {
            p->unit_cell_width_nm=s->cell_width_nm; p->unit_cell_height_nm=s->cell_height_nm;
            p->flags|=CC_CAMERA_PROPERTY_UNIT_CELL;
        }
    } else if (ret!=-ENOENT) return ret;
    int32_t value;
    ret=control_default(camera,V4L2_CID_CAMERA_ORIENTATION,V4L2_CTRL_TYPE_MENU,&value);
    if (!ret) {
        p->location=value==V4L2_CAMERA_ORIENTATION_FRONT ? CC_CAMERA_LOCATION_FRONT :
                    value==V4L2_CAMERA_ORIENTATION_BACK ? CC_CAMERA_LOCATION_BACK:CC_CAMERA_LOCATION_EXTERNAL;
        p->flags|=CC_CAMERA_PROPERTY_LOCATION;
    } else if (ret!=-ENOENT) return ret;
    ret=control_default(camera,V4L2_CID_CAMERA_SENSOR_ROTATION,V4L2_CTRL_TYPE_INTEGER,&value);
    if (!ret) p->rotation=value;
    else if (ret!=-ENOENT) return ret;
    p->flags|=CC_CAMERA_PROPERTY_ROTATION;

    uint32_t cfa_flip=0;
    int cfa_ret=camera->info.pipeline==CC_PIPELINE_VIMC ? 0:sensor_cfa_transform(camera,&cfa_flip);
    if (cfa_ret<0 && cfa_ret!=-ENOTSUP) return cfa_ret;
    ret=cc_sensor_mode_properties(camera,cfa_flip,p);
    if (ret<0 && !optional_absent(ret)) return ret;
    /* Ignore optional enumeration as a whole, including any partial result. */
    if (ret<0) {
        p->flags&=~(uint32_t)(CC_CAMERA_PROPERTY_PIXEL_ARRAY|CC_CAMERA_PROPERTY_CFA);
        p->pixel_array_width=p->pixel_array_height=0; p->cfa=0;
    }
    if (cfa_ret<0) { p->flags&=~(uint32_t)CC_CAMERA_PROPERTY_CFA; p->cfa=0; }
    if (camera->info.pipeline!=CC_PIPELINE_VIMC) {
        cc_sensor_streams streams;
        ret=cc_camera_get_sensor_streams(camera,&streams);
        if (ret<0 && !optional_absent(ret)) return ret;
        if (!ret) {
            cc_rectangle bounds;
            ret=sensor_selection(camera,&streams.image,V4L2_SEL_TGT_CROP_BOUNDS,&bounds);
            bool have_bounds=!ret;
            if (!ret) {
                p->pixel_array_width=bounds.width; p->pixel_array_height=bounds.height;
                p->flags|=CC_CAMERA_PROPERTY_PIXEL_ARRAY;
            } else if (ret!=-ENOENT) return ret;
            ret=sensor_selection(camera,&streams.image,V4L2_SEL_TGT_CROP_DEFAULT,&p->active_area);
            if (!ret) {
                const cc_rectangle *a=&p->active_area;
                if (have_bounds && (a->x<bounds.x || a->y<bounds.y ||
                    (uint64_t)(uint32_t)a->x+a->width>(uint64_t)(uint32_t)bounds.x+bounds.width ||
                    (uint64_t)(uint32_t)a->y+a->height>(uint64_t)(uint32_t)bounds.y+bounds.height)) return -EPROTO;
                p->flags|=CC_CAMERA_PROPERTY_ACTIVE_AREA;
            } else if (ret!=-ENOENT) return ret;
        }
    }
    if (!(p->flags&CC_CAMERA_PROPERTY_ACTIVE_AREA) && p->flags&CC_CAMERA_PROPERTY_PIXEL_ARRAY) {
        p->active_area=(cc_rectangle){0,0,p->pixel_array_width,p->pixel_array_height};
        p->flags|=CC_CAMERA_PROPERTY_ACTIVE_AREA;
    }
    return 0;
}

static int video_geometry(cc_camera *camera, cc_camera_properties *p)
{
    uint32_t seen[PROPERTY_FORMAT_LIMIT], sizes=0;
    uint64_t best_area=0;
    for (uint32_t f=0; ; ++f) {
        cc_format_info format;
        int ret=cc_camera_enum_format(camera,f,&format);
        if (ret==-ENOENT) break;
        if (ret<0) return ret;
        if (f==PROPERTY_FORMAT_LIMIT) return -E2BIG;
        if (!format.fourcc) return -EPROTO;
        for (uint32_t i=0; i<f; ++i) if (seen[i]==format.fourcc) return -EPROTO;
        seen[f]=format.fourcc;
        for (uint32_t s=0; ; ++s) {
            cc_size_range size;
            ret=cc_camera_enum_size(camera,format.fourcc,s,&size);
            if (ret==-ENOENT) break;
            if (ret<0) return ret;
            if (sizes++==PROPERTY_SIZE_LIMIT) return -E2BIG;
            if (!size.min_width || !size.min_height || size.max_width<size.min_width ||
                size.max_height<size.min_height) return -EPROTO;
            uint64_t area=(uint64_t)size.max_width*size.max_height;
            if (area>best_area || (area==best_area && size.max_width>p->pixel_array_width)) {
                best_area=area; p->pixel_array_width=size.max_width; p->pixel_array_height=size.max_height;
                p->flags|=CC_CAMERA_PROPERTY_PIXEL_ARRAY|CC_CAMERA_PROPERTY_ACTIVE_AREA;
                p->active_area=(cc_rectangle){0,0,size.max_width,size.max_height};
            }
        }
    }
    return 0;
}

int cc_camera_get_properties(cc_camera *camera, cc_camera_properties *out)
{
    if (!camera || !out) return -EINVAL;
    int ret=cc_camera_fd(camera);
    if (ret<0) return ret;
    cc_camera_properties p={0};
    if (camera->sensor_fd>=0) ret=sensor_properties(camera,&p);
    else {
        if (!camera->device) return -ENODEV;
        bool uvc=camera->info.pipeline==CC_PIPELINE_UVC;
        const char *name=uvc && camera->media ? camera->media->info.model:(const char *)camera->device->caps.card;
        cc_camera_model(p.model,sizeof(p.model),name,sizeof(camera->device->caps.card),false);
        if (p.model[0]) p.flags|=CC_CAMERA_PROPERTY_MODEL;
        if (uvc) {
            p.location=CC_CAMERA_LOCATION_EXTERNAL;
            if (camera->io.ops->sysfs_read) {
                char removable[32]={0};
                int query=camera->io.ops->sysfs_read(camera->io.ctx,camera->fd,"../removable",removable,sizeof(removable));
                if (!query) {
                    if (!memchr(removable,'\0',sizeof(removable))) return -EPROTO;
                    removable[strcspn(removable,"\r\n")]='\0';
                    if (!strcmp(removable,"fixed")) p.location=CC_CAMERA_LOCATION_FRONT;
                }
            }
            p.flags|=CC_CAMERA_PROPERTY_LOCATION;
        }
        ret=video_geometry(camera,&p);
        if (ret<0 && optional_absent(ret)) {
            p.flags&=~(uint32_t)(CC_CAMERA_PROPERTY_PIXEL_ARRAY|CC_CAMERA_PROPERTY_ACTIVE_AREA);
            p.pixel_array_width=p.pixel_array_height=0; p.active_area=(cc_rectangle){0}; ret=0;
        }
    }
    if (ret<0) return ret;
    *out=p;
    return 0;
}
