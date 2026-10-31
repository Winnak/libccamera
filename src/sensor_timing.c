/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* RPi helper policy/planning reference: Copyright (C) 2019-2024 Raspberry Pi Ltd.
 * Native integer arithmetic and driver-range validation are implemented in C17.
 * See LICENSES/BSD-2-Clause.txt. */
#include "internal.h"
#include "integer_math.h"
#include <ccamera/sensor.h>
#include <float.h>
#include <limits.h>
#include <string.h>

static int limits_valid(const cc_sensor_timing_limits *l)
{
    if (!l || !l->width || !l->height || !l->pixel_rate || l->pixel_rate>INT64_MAX ||
        !l->hblank_step || !l->vblank_step || !l->exposure_step || !l->min_exposure ||
        l->max_hblank<l->min_hblank || l->max_vblank<l->min_vblank) return -EINVAL;
    if ((uint64_t)l->width+l->max_hblank>UINT32_MAX || (uint64_t)l->height+l->max_vblank>UINT32_MAX) return -ERANGE;
    return l->pixel_rate>((uint64_t)l->width+l->min_hblank)*UINT64_C(1000000000) ? -ERANGE:0;
}
static uint32_t align_down(uint32_t value,uint32_t minimum,uint32_t step)
{
    return minimum+(value-minimum)/step*step;
}
static cc_sensor_timing timing(const cc_sensor_timing_limits *l,uint32_t hblank,uint32_t vblank)
{
    return (cc_sensor_timing){l->pixel_rate,l->width,l->height,hblank,vblank};
}
int cc_sensor_timing_bounds(const cc_sensor_timing_limits *l,uint64_t *minimum_ns,uint64_t *maximum_ns)
{
    if (!minimum_ns || !maximum_ns) return -EINVAL;
    int ret=limits_valid(l); if (ret<0) return ret;
    cc_sensor_timing low=timing(l,l->min_hblank,l->min_vblank);
    cc_sensor_timing high=timing(l,align_down(l->max_hblank,l->min_hblank,l->hblank_step),
        align_down(l->max_vblank,l->min_vblank,l->vblank_step));
    uint64_t minimum,maximum;
    ret=cc_sensor_frame_duration(&low,&minimum);
    if (!ret) ret=cc_sensor_frame_duration(&high,&maximum);
    if (ret<0) return ret;
    *minimum_ns=minimum; *maximum_ns=maximum; return 0;
}
static uint64_t clamp64(uint64_t value,uint64_t low,uint64_t high)
{
    return value<low ? low:value>high ? high:value;
}
int cc_sensor_plan_exposure(const cc_sensor_model *model,const cc_sensor_timing_limits *l,uint64_t exposure_ns,
                            uint64_t min_ns,uint64_t max_ns,cc_sensor_exposure_plan *out)
{
    if (!out || !model || !exposure_ns || (min_ns && max_ns && min_ns>max_ns)) return -EINVAL;
    const cc_sensor_properties *properties=cc_sensor_get_properties(model);
    if (!(properties->features&CC_SENSOR_EXPOSURE)) return -ENOTSUP;
    uint64_t physical_min,physical_max;
    int ret=cc_sensor_timing_bounds(l,&physical_min,&physical_max); if (ret<0) return ret;
    uint64_t requested_min=min_ns,requested_max=max_ns;
    min_ns=clamp64(min_ns ? min_ns:physical_min,physical_min,physical_max);
    max_ns=clamp64(max_ns ? max_ns:physical_max,physical_min,physical_max);
    if (min_ns>max_ns) return -EINVAL;
    uint32_t margin=properties->frame_integration_margin;
    uint32_t max_vblank=align_down(l->max_vblank,l->min_vblank,l->vblank_step);
    uint32_t frame_min=l->height+l->min_vblank,frame_max=l->height+max_vblank;
    if (frame_min<=margin || frame_max<=margin) return -ERANGE;
    uint64_t line_pixels=(uint64_t)l->width+l->min_hblank, line_ns=line_pixels*UINT64_C(1000000000);
    uint64_t requested_lines,frame_low,frame_high;
    ret=cc_mul_div_u64(exposure_ns,l->pixel_rate,line_ns,false,&requested_lines);
    if (!ret) ret=cc_mul_div_u64(min_ns,l->pixel_rate,line_ns,false,&frame_low);
    if (!ret) ret=cc_mul_div_u64(max_ns,l->pixel_rate,line_ns,false,&frame_high);
    if (ret<0) return ret;
    if (frame_low<frame_min) frame_low=frame_min;
    if (frame_high<frame_low) frame_high=frame_low;
    if (requested_lines>UINT32_MAX-margin) requested_lines=UINT32_MAX-margin;
    uint64_t wanted=clamp64(requested_lines+margin,frame_low,frame_high);
    uint32_t hblank=l->min_hblank,frame_length;
    if (wanted>frame_max) {
        uint64_t pixels;
        ret=cc_mul_div_u64(line_pixels,wanted,frame_max,false,&pixels); if (ret<0) return ret;
        uint32_t maximum=align_down(l->max_hblank,l->min_hblank,l->hblank_step);
        uint64_t blank=pixels>l->width ? pixels-l->width:0;
        hblank=align_down((uint32_t)clamp64(blank,l->min_hblank,maximum),l->min_hblank,l->hblank_step);
        frame_length=frame_max;
    } else frame_length=frame_min+((uint32_t)wanted-frame_min)/l->vblank_step*l->vblank_step;
    uint32_t shift=0;
    bool long_capable=!strcmp(properties->model,"imx477") || !strcmp(properties->model,"imx519") || !strcmp(properties->model,"imx708");
    if (long_capable) {
        uint32_t scaled=frame_length;
        while (scaled>UINT32_C(0xffdc)) {
            if (++shift>7) { shift=7; scaled=UINT32_C(0xffdc); break; }
            scaled>>=1;
        }
        uint32_t bound=scaled<<shift;
        if (bound<frame_min) return -ERANGE;
        frame_length=frame_min+(bound-frame_min)/l->vblank_step*l->vblank_step;
        uint32_t scale=1U<<shift;
        /* At most scale/gcd(scale,step) native grid points are needed. */
        for (uint32_t tries=0;frame_length%scale;++tries) {
            if (tries>=127 || frame_length-frame_min<l->vblank_step) return -ERANGE;
            frame_length-=l->vblank_step;
        }
    }
    if (frame_length<=margin) return -ERANGE;
    cc_sensor_timing actual=timing(l,hblank,frame_length-l->height);
    line_ns=((uint64_t)l->width+hblank)*UINT64_C(1000000000);
    uint64_t generic_lines,clipped_ns;
    ret=cc_mul_div_u64(exposure_ns,l->pixel_rate,line_ns,false,&generic_lines);
    if (ret<0) return ret;
    generic_lines=clamp64(generic_lines,0,frame_length-margin);
    ret=cc_mul_div_u64(generic_lines,line_ns,l->pixel_rate,true,&clipped_ns); if (ret<0) return ret;
    uint64_t shutter_max_ns;
    ret=cc_sensor_exposure_time(model,&actual,frame_length-margin,&shutter_max_ns); if (ret<0) return ret;
    if (clipped_ns>shutter_max_ns) clipped_ns=shutter_max_ns;
    uint32_t exposure;
    ret=cc_sensor_exposure_lines(model,&actual,clipped_ns,&exposure); if (ret<0) return ret;
    if (exposure>frame_length-margin) exposure=frame_length-margin;
    if (exposure<l->min_exposure) exposure=l->min_exposure;
    exposure=align_down(exposure,l->min_exposure,l->exposure_step);
    if (exposure>frame_length-margin) return -ERANGE;
    cc_sensor_exposure_plan plan={.exposure_lines=exposure,.hblank=hblank,.vblank=actual.vblank,
        .frame_length=frame_length,.long_exposure_shift=shift};
    ret=cc_sensor_exposure_time(model,&actual,exposure,&plan.exposure_ns);
    if (!ret) ret=cc_sensor_frame_duration(&actual,&plan.frame_duration_ns);
    if (ret<0) return ret;
    uint64_t available;
    ret=cc_mul_div_u64(frame_length-margin,line_ns,l->pixel_rate,true,&available);
    if (ret<0) return ret;
    if (plan.exposure_ns>available) return -ERANGE;
    if (plan.exposure_ns!=exposure_ns) plan.flags|=CC_SENSOR_PLAN_EXPOSURE_ADJUSTED;
    if ((requested_min && min_ns!=requested_min) || (requested_max && max_ns!=requested_max) ||
        plan.frame_duration_ns<min_ns || plan.frame_duration_ns>max_ns) plan.flags|=CC_SENSOR_PLAN_DURATION_ADJUSTED;
    if (shift) plan.flags|=CC_SENSOR_PLAN_LONG_EXPOSURE;
    *out=plan; return 0;
}
int cc_sensor_get_mode_policy(const cc_sensor_model *model,const cc_sensor_mode_description *mode,cc_sensor_mode_policy *out)
{
    if (!model || !mode || !out || !mode->width || !mode->height || !mode->binning_x ||
        !(mode->scale_x>=1 && mode->scale_x<=DBL_MAX) || !mode->min_frame_duration_ns) return -EINVAL;
    const cc_sensor_properties *p=cc_sensor_get_properties(model);
    if (!(p->features&CC_SENSOR_EXPOSURE)) return -ENOTSUP;
    cc_sensor_mode_policy policy={.sensitivity=1,.mistrust_startup=1,.min_debin_factor=2};
    if (!strcmp(p->model,"imx290") || !strcmp(p->model,"imx327") || !strcmp(p->model,"imx462") || !strcmp(p->model,"imx415"))
        policy.hide_startup=policy.hide_mode_switch=1;
    if (!strcmp(p->model,"imx283")) policy.hide_mode_switch=1;
    if (!strcmp(p->model,"imx219")) policy.mistrust_mode_switch=1;
    if (!strcmp(p->model,"ov5647")) policy.hide_startup=policy.hide_mode_switch=policy.mistrust_startup=policy.mistrust_mode_switch=2;
    if (!strcmp(p->model,"imx708")) {
        policy.sensitivity=mode->width>2304 ? 1:2; policy.min_debin_factor=4;
        if (mode->width==2304 && mode->height==1296 && mode->min_frame_duration_ns>UINT64_C(31250000))
            policy.hide_startup=policy.hide_mode_switch=1;
    }
    if (!strcmp(p->model,"ov64a40") && mode->binning_x>=2)
        policy.sensitivity=mode->scale_x>=4 ? 4:mode->scale_x>=2 ? 2:1;
    *out=policy; return 0;
}
static int native_range(const cc_control_info *info,uint32_t *minimum,uint32_t *maximum,uint32_t *step)
{
    if ((info->type!=V4L2_CTRL_TYPE_INTEGER && info->type!=V4L2_CTRL_TYPE_INTEGER64) || info->minimum<0 ||
        info->maximum<info->minimum || info->maximum>UINT32_MAX || !info->step || info->step>UINT32_MAX ||
        info->elems>1 || info->flags&(V4L2_CTRL_FLAG_DISABLED|V4L2_CTRL_FLAG_INACTIVE|V4L2_CTRL_FLAG_HAS_PAYLOAD)) return -ENOTSUP;
    *minimum=(uint32_t)info->minimum; *maximum=(uint32_t)info->maximum; *step=(uint32_t)info->step; return 0;
}
int cc_camera_get_sensor_timing_limits(cc_camera *camera,cc_sensor_timing_limits *out)
{
    if (!camera || !out) return -EINVAL;
    cc_sensor_timing current; int ret=cc_camera_get_sensor_timing(camera,&current); if (ret<0) return ret;
    cc_sensor_timing_limits limits={.pixel_rate=current.pixel_rate,.width=current.width,.height=current.height};
    uint32_t *minimum[]={&limits.min_hblank,&limits.min_vblank,&limits.min_exposure};
    uint32_t *maximum[]={&limits.max_hblank,&limits.max_vblank,NULL};
    uint32_t *steps[]={&limits.hblank_step,&limits.vblank_step,&limits.exposure_step};
    const uint32_t ids[]={V4L2_CID_HBLANK,V4L2_CID_VBLANK,V4L2_CID_EXPOSURE};
    const uint32_t values[]={current.hblank,current.vblank};
    for (size_t i=0;i<3;++i) {
        cc_control_info info; ret=cc_camera_query_control(camera,ids[i],&info); if (ret<0) return ret;
        uint32_t high; ret=native_range(&info,minimum[i],&high,steps[i]); if (ret<0) return ret;
        if (info.id!=ids[i]) return -EPROTO;
        if (maximum[i]) {
            if (values[i]<*minimum[i] || values[i]>high || (values[i]-*minimum[i])%*steps[i]) return -EPROTO;
            *maximum[i]=high;
            if (info.flags&V4L2_CTRL_FLAG_READ_ONLY) *minimum[i]=*maximum[i]=values[i];
        } else if (!limits.min_exposure) limits.min_exposure=limits.exposure_step;
    }
    uint64_t low,high; ret=cc_sensor_timing_bounds(&limits,&low,&high);
    if (!ret) *out=limits;
    return ret;
}
