/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* UVC mappings/selection follow the supplied handler, Copyright (C) 2019 Google Inc. */
#include "internal.h"
#include <ccamera/uvc.h>
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <string.h>

#define UVC_ALL 127U
static const uint32_t ids[CC_UVC_CONTROL_COUNT]={V4L2_CID_BRIGHTNESS,V4L2_CID_CONTRAST,V4L2_CID_SATURATION,
    V4L2_CID_EXPOSURE_AUTO,V4L2_CID_EXPOSURE_ABSOLUTE,V4L2_CID_GAIN,V4L2_CID_GAMMA};
struct mapping { double scale,offset; cc_uvc_range range; };
static double clamp(double x,double low,double high) { return x<low ? low:x>high ? high:x; }
static bool finite_value(double x) { return x>=-DBL_MAX && x<=DBL_MAX; }
static double setting_value(const cc_uvc_controls *s,uint32_t i)
{
    switch (i) {
    case CC_UVC_CONTROL_BRIGHTNESS: return s->brightness;
    case CC_UVC_CONTROL_CONTRAST: return s->contrast;
    case CC_UVC_CONTROL_SATURATION: return s->saturation;
    case CC_UVC_CONTROL_ANALOGUE_GAIN: return s->analogue_gain;
    case CC_UVC_CONTROL_GAMMA: return s->gamma;
    default: return 0;
    }
}
int cc_uvc_validate_settings(const cc_uvc_controls *s)
{
    if (!s || s->flags&~UVC_ALL) return -EINVAL;
    for (uint32_t i=0;i<CC_UVC_CONTROL_COUNT;++i)
        if ((s->flags&(1U<<i)) && !finite_value(setting_value(s,i))) return -EINVAL;
    if ((s->flags&CC_UVC_EXPOSURE_MODE) && s->exposure_mode!=CC_UVC_EXPOSURE_AUTO &&
        s->exposure_mode!=CC_UVC_EXPOSURE_MANUAL) return -EINVAL;
    return 0;
}
static int native_valid(const cc_control_info *r,uint32_t index)
{
    if (r->id!=ids[index] || r->minimum<INT32_MIN || r->maximum>INT32_MAX || r->maximum<r->minimum ||
        r->default_value<r->minimum || r->default_value>r->maximum || !r->step || r->step>UINT32_MAX ||
        r->elems>1 || r->nr_of_dims || r->flags&(V4L2_CTRL_FLAG_DISABLED|V4L2_CTRL_FLAG_READ_ONLY|
            V4L2_CTRL_FLAG_HAS_PAYLOAD|V4L2_CTRL_FLAG_MODIFY_LAYOUT|V4L2_CTRL_FLAG_EXECUTE_ON_WRITE)) return -ENOTSUP;
    if (index==CC_UVC_CONTROL_EXPOSURE_MODE)
        return r->type==V4L2_CTRL_TYPE_MENU && r->minimum>=0 ? 0:-ENOTSUP;
    if (r->type!=V4L2_CTRL_TYPE_INTEGER || ((uint64_t)(r->default_value-r->minimum))%r->step) return -ENOTSUP;
    if ((index==CC_UVC_CONTROL_EXPOSURE_TIME || index==CC_UVC_CONTROL_GAMMA) && r->minimum<0) return -ENOTSUP;
    return 0;
}
static int mode_code(const cc_uvc_profile *p,enum cc_uvc_exposure_mode mode,uint32_t *code)
{
    if (p->exposure_menu&~15U) return -EINVAL;
    const uint32_t choices[2][2]={{V4L2_EXPOSURE_AUTO,V4L2_EXPOSURE_APERTURE_PRIORITY},
        {V4L2_EXPOSURE_SHUTTER_PRIORITY,V4L2_EXPOSURE_MANUAL}};
    if (mode!=CC_UVC_EXPOSURE_AUTO && mode!=CC_UVC_EXPOSURE_MANUAL) return -EINVAL;
    const cc_control_info *r=&p->native[CC_UVC_CONTROL_EXPOSURE_MODE];
    for (size_t i=0;i<2;++i) {
        uint32_t c=choices[mode][i];
        if ((p->exposure_menu&(1U<<c)) && c>=(uint64_t)r->minimum && c<=(uint64_t)r->maximum) { *code=c; return 0; }
    }
    return -ENOTSUP;
}
static int map_control(const cc_uvc_profile *p,uint32_t i,struct mapping *m)
{
    if (!p || i>=CC_UVC_CONTROL_COUNT || p->flags&~UVC_ALL) return -EINVAL;
    if (!(p->flags&(1U<<i))) return -ENOENT;
    const cc_control_info *r=&p->native[i];
    int ret=native_valid(r,i); if (ret<0) return ret;
    double min=(double)r->minimum,max=(double)r->maximum,def=(double)r->default_value;
    *m=(struct mapping){.scale=1};
    if (i==CC_UVC_CONTROL_EXPOSURE_MODE) {
        uint32_t code; bool automatic=!mode_code(p,CC_UVC_EXPOSURE_AUTO,&code);
        bool manual=!mode_code(p,CC_UVC_EXPOSURE_MANUAL,&code);
        if (!automatic && !manual) return -ENOTSUP;
        bool default_manual=r->default_value==V4L2_EXPOSURE_MANUAL || r->default_value==V4L2_EXPOSURE_SHUTTER_PRIORITY;
        bool default_present=r->default_value<=3 && (p->exposure_menu&(1U<<(uint32_t)r->default_value));
        m->range=(cc_uvc_range){automatic ? 0:1,manual ? 1:0,
            default_present ? (default_manual ? 1:0):automatic ? 0:1};
        return 0;
    }
    switch (i) {
    case CC_UVC_CONTROL_BRIGHTNESS: m->scale=max-def>def-min ? max-def:def-min; m->offset=def; break;
    case CC_UVC_CONTROL_SATURATION:
        if (def==min && max!=min) return -ENOTSUP;
        m->scale=def-min; m->offset=min; break;
    case CC_UVC_CONTROL_CONTRAST: case CC_UVC_CONTROL_ANALOGUE_GAIN:
        m->scale=(max-def)/3;
        if ((def-min)>m->scale*0.5) m->scale=(def-min)*2;
        m->offset=def-m->scale; break;
    case CC_UVC_CONTROL_GAMMA: m->scale=100; break;
    case CC_UVC_CONTROL_EXPOSURE_TIME: m->scale=0.01; break;
    default: return -EINVAL;
    }
    if (min==max && m->scale==0) {
        m->scale=1; m->offset=def-(i==CC_UVC_CONTROL_BRIGHTNESS ? 0:1);
    }
    if (!(m->scale>0 && m->scale<=DBL_MAX)) return -ENOTSUP;
    m->range=(cc_uvc_range){(min-m->offset)/m->scale,(max-m->offset)/m->scale,(def-m->offset)/m->scale};
    return 0;
}
int cc_uvc_get_control_range(const cc_uvc_profile *p,enum cc_uvc_control_index control,cc_uvc_range *range)
{
    if (!range) return -EINVAL;
    struct mapping m; int ret=map_control(p,(uint32_t)control,&m);
    if (!ret) *range=m.range;
    return ret;
}
int cc_uvc_controls_to_native(const cc_uvc_profile *p,const cc_uvc_controls *s,cc_control *controls,size_t *count)
{
    if (!p || !count || p->flags&~UVC_ALL) return -EINVAL;
    int ret=cc_uvc_validate_settings(s); if (ret<0) return ret;
    if (s->flags&~p->flags) return -ENOTSUP;
    cc_control result[CC_UVC_CONTROL_COUNT]; size_t n=0;
    /* Place the auto/manual selector before the other members of its cluster. */
    const uint32_t order[]={3,0,1,2,4,5,6};
    for (size_t at=0;at<CC_UVC_CONTROL_COUNT;++at) {
        uint32_t i=order[at];
        if (!(s->flags&(1U<<i))) continue;
        struct mapping m; ret=map_control(p,i,&m); if (ret<0) return ret;
        const cc_control_info *r=&p->native[i];
        int64_t value;
        if (i==CC_UVC_CONTROL_EXPOSURE_MODE) {
            uint32_t code; ret=mode_code(p,s->exposure_mode,&code); if (ret<0) return ret;
            value=code;
        } else {
            double native=i==CC_UVC_CONTROL_EXPOSURE_TIME ? (double)(s->exposure_us/100):
                m.offset+m.scale*clamp(setting_value(s,i),m.range.minimum,m.range.maximum);
            native=clamp(native,(double)r->minimum,(double)r->maximum);
            value=(int64_t)(native+(native<0 ? -0.5:0.5));
            value=r->minimum+(int64_t)((uint64_t)(value-r->minimum)/r->step*r->step);
        }
        result[n++]=(cc_control){.id=ids[i],.value=value};
    }
    if (controls && *count<n) { *count=n; return -ENOSPC; }
    if (controls) memcpy(controls,result,n*sizeof(*controls));
    *count=n; return 0;
}
static int decode(const cc_uvc_profile *p,const cc_control *native,size_t count,cc_uvc_controls *settings)
{
    cc_uvc_controls result={0};
    double *values[]={&result.brightness,&result.contrast,&result.saturation,NULL,NULL,&result.analogue_gain,&result.gamma};
    for (size_t at=0;at<count;++at) {
        uint32_t i=0; while (i<CC_UVC_CONTROL_COUNT && native[at].id!=ids[i]) ++i;
        if (i==CC_UVC_CONTROL_COUNT || result.flags&(1U<<i)) return -EPROTO;
        const cc_control_info *r=&p->native[i]; int64_t value=native[at].value;
        if (value<r->minimum || value>r->maximum) return -EPROTO;
        result.flags|=1U<<i;
        if (i==CC_UVC_CONTROL_EXPOSURE_MODE) {
            if (value>3 || !(p->exposure_menu&(1U<<(uint32_t)value))) return -EPROTO;
            result.exposure_mode=value==V4L2_EXPOSURE_AUTO || value==V4L2_EXPOSURE_APERTURE_PRIORITY ?
                CC_UVC_EXPOSURE_AUTO:CC_UVC_EXPOSURE_MANUAL;
        } else {
            if ((uint64_t)(value-r->minimum)%r->step) return -EPROTO;
            if (i==CC_UVC_CONTROL_EXPOSURE_TIME) result.exposure_us=(uint64_t)value*100;
            else { struct mapping m; int ret=map_control(p,i,&m); if (ret<0) return ret; *values[i]=((double)value-m.offset)/m.scale; }
        }
    }
    *settings=result; return 0;
}
int cc_camera_get_uvc_profile(cc_camera *camera,cc_uvc_profile *out)
{
    if (!camera || !out) return -EINVAL;
    if (camera->info.pipeline!=CC_PIPELINE_UVC) return -ENOTSUP;
    int fd=cc_camera_fd(camera); if (fd<0) return fd;
    cc_uvc_profile p={0};
    for (uint32_t i=0;i<CC_UVC_CONTROL_COUNT;++i) {
        cc_control_info *r=&p.native[i]; int ret=cc_camera_query_control(camera,ids[i],r);
        if (ret<0) { if (ret==-EINVAL || ret==-ENOENT || ret==-ENOTTY) continue; return ret; }
        if (r->id!=ids[i]) return -EPROTO;
        if (native_valid(r,i)<0) continue;
        if (i==CC_UVC_CONTROL_EXPOSURE_MODE) {
            for (uint32_t mode=0;mode<4;++mode) {
                cc_control_menu entry; ret=cc_camera_query_menu(camera,ids[i],mode,&entry);
                if (!ret) p.exposure_menu|=1U<<mode;
                else if (ret!=-ENOENT && ret!=-ENOTTY) return ret;
            }
        }
        p.flags|=1U<<i;
        struct mapping m;
        if (map_control(&p,i,&m)<0) p.flags&=~(1U<<i);
    }
    *out=p; return 0;
}
int cc_camera_set_uvc_controls(cc_camera *camera,cc_uvc_controls *settings)
{
    if (!settings) return -EINVAL;
    cc_uvc_profile profile; int ret=cc_camera_get_uvc_profile(camera,&profile);
    cc_control native[CC_UVC_CONTROL_COUNT]; size_t count=CC_UVC_CONTROL_COUNT;
    if (!ret) ret=cc_uvc_controls_to_native(&profile,settings,native,&count);
    if (!ret && count) ret=cc_camera_apply_controls(camera,native,count);
    cc_uvc_controls result;
    if (!ret) ret=decode(&profile,native,count,&result);
    if (!ret) *settings=result;
    return ret;
}
int cc_uvc_match(struct cc_media *media,struct cc_entity **out)
{
    if (!media || !out) return -EINVAL;
    if (strncmp(media->info.driver,"uvcvideo",sizeof(media->info.driver))) return -ENOTSUP;
    struct cc_entity *found=NULL;
    for (size_t i=0;i<media->num_entities;++i) {
        struct cc_entity *e=&media->entities[i];
        if (!(e->desc.flags&MEDIA_ENT_FL_DEFAULT) || e->desc.type!=MEDIA_ENT_F_IO_V4L || !e->path[0]) continue;
        if (found) return -ENOTUNIQ;
        found=e;
    }
    if (!found) return -ENODEV;
    *out=found; return 0;
}
int cc_uvc_open_media(cc_camera *camera)
{
    int ret=cc_media_open(&camera->io,camera->info.media_path,true,&camera->media);
    if (ret<0) return ret;
    struct cc_entity *video;
    ret=cc_uvc_match(camera->media,&video);
    if (!ret && strcmp(video->path,camera->info.video_path)) ret=-ENODEV;
    return ret;
}
