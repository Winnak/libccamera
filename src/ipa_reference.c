/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa.h>
#include "integer_math.h"
#include <linux/videodev2.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

struct reference {
    cc_ipa_reference_tuning tuning;
    cc_ipa_configuration configuration;
    const cc_sensor_model *sensor;
    cc_control_info exposure, gain;
    uint32_t exposure_lines, gain_code;
    uint64_t desired_exposure_ns;
    bool variable_hblank;
    double red, blue;
    cc_af_tuning af_tuning;
    cc_af *af;
    cc_af_status af_status;
    enum cc_af_mode af_mode;
    bool af_have_status, af_clock_valid;
    uint64_t af_frame_count, af_update_count, af_timestamp, af_update_timestamp;
    cc_sensor_pdaf pdaf;
    bool have_pdaf;
};
static double clamp(double value, double low, double high) { return value < low ? low : value > high ? high : value; }
static double distance(double a, double b) { return a > b ? a - b : b - a; }
static const cc_control *find(const cc_control *controls, size_t count, uint32_t id)
{
    for (size_t i = 0; i < count; ++i) if (controls[i].id == id) return &controls[i];
    return NULL;
}
static int control_range(const cc_control_info *range)
{
    if (!range->id || !range->step || range->minimum < 0 || range->maximum < range->minimum ||
        range->maximum > INT32_MAX || range->type != V4L2_CTRL_TYPE_INTEGER || range->elems > 1 ||
        range->flags & (V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_INACTIVE |
                        V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_MODIFY_LAYOUT)) return -ENOTSUP;
    return 0;
}
static uint32_t quantize(const cc_control_info *range, double wanted)
{
    uint64_t value = (uint64_t)clamp(wanted, (double)range->minimum, (double)range->maximum);
    uint64_t first = (uint64_t)range->minimum;
    return (uint32_t)(first + (value - first) / range->step * range->step);
}
static int exposure_range(struct reference *r, cc_control_info *range)
{
    *range = r->exposure;
    const cc_sensor_properties *p = cc_sensor_get_properties(r->sensor);
    uint64_t frame_lines = (uint64_t)r->configuration.timing.height + r->configuration.timing.vblank;
    if (frame_lines <= p->frame_integration_margin) return -ERANGE;
    frame_lines -= p->frame_integration_margin;
    if (frame_lines < (uint64_t)range->maximum) range->maximum = (int64_t)frame_lines;
    if (r->tuning.max_exposure_ns) {
        uint32_t lines;
        int ret = cc_sensor_exposure_lines(r->sensor, &r->configuration.timing, r->tuning.max_exposure_ns, &lines);
        if (ret < 0) return ret;
        if (lines < (uint64_t)range->maximum) range->maximum = lines;
    }
    if (!range->minimum) {
        if (range->step > INT32_MAX) return -ERANGE;
        range->minimum = (int64_t)range->step;
    }
    return range->maximum < range->minimum ? -ERANGE : 0;
}
static bool duration_planning(const struct reference *r) { return r->tuning.max_frame_duration_ns!=0; }
static int planning_limits(const struct reference *r,const cc_ipa_request *request,cc_sensor_timing_limits *out)
{
    cc_sensor_timing_limits limits=request && request->timing_limits.pixel_rate ?
        request->timing_limits:r->configuration.timing_limits;
    cc_sensor_timing current=request && request->timing.pixel_rate ? request->timing:r->configuration.timing;
    if (limits.pixel_rate!=current.pixel_rate || limits.width!=current.width || limits.height!=current.height)
        return -EINVAL;
    const cc_control *exposure=request ? find(request->sensor_controls,request->num_sensor_controls,V4L2_CID_EXPOSURE):NULL;
    const uint32_t ids[]={V4L2_CID_HBLANK,V4L2_CID_VBLANK};
    uint32_t *low[]={&limits.min_hblank,&limits.min_vblank},*high[]={&limits.max_hblank,&limits.max_vblank};
    uint32_t step[]={limits.hblank_step,limits.vblank_step};
    for (size_t i=0;i<2;++i) {
        const cc_control *override=request ? find(request->sensor_controls,request->num_sensor_controls,ids[i]):NULL;
        if (override) {
            if (override->value<*low[i] || override->value>*high[i] || !step[i] ||
                ((uint64_t)override->value-*low[i])%step[i]) return -ERANGE;
            *low[i]=*high[i]=(uint32_t)override->value;
        } else if (!i && (!r->variable_hblank || exposure)) {
            /* A manual native shutter code retains the caller's line period. */
            if (current.hblank<*low[i] || current.hblank>*high[i] || !step[i] ||
                (current.hblank-*low[i])%step[i]) return -ERANGE;
            *low[i]=*high[i]=current.hblank;
        }
    }
    /* The actual model's exposure grid is checked again by the kernel. */
    if (limits.min_exposure!=(uint64_t)(r->exposure.minimum ? r->exposure.minimum:(int64_t)r->exposure.step) ||
        limits.exposure_step!=r->exposure.step) return -ENOTSUP;
    *out=limits;
    return 0;
}
static int plan_exposure(const struct reference *r,const cc_ipa_request *request,uint64_t wanted,cc_sensor_exposure_plan *plan)
{
    cc_sensor_timing_limits limits;
    int ret=planning_limits(r,request,&limits);
    if (ret<0) return ret;
    const cc_control *manual=request ? find(request->sensor_controls,request->num_sensor_controls,V4L2_CID_EXPOSURE):NULL;
    if (manual) {
        if (manual->value<limits.min_exposure || manual->value>UINT32_MAX ||
            ((uint64_t)manual->value-limits.min_exposure)%limits.exposure_step) return -ERANGE;
        cc_sensor_timing t={limits.pixel_rate,limits.width,limits.height,limits.min_hblank,limits.min_vblank};
        ret=cc_sensor_exposure_time(r->sensor,&t,(uint32_t)manual->value,&wanted);
        if (ret<0) return ret;
        /* The fixed-period IMX296 shutter need not end on a generic line.
         * Reserve the whole line containing its end before adding the margin. */
        uint64_t line_ns=((uint64_t)t.width+t.hblank)*UINT64_C(1000000000),lines;
        ret=cc_mul_div_u64(wanted,t.pixel_rate,line_ns,true,&lines);
        if (!ret) ret=cc_mul_div_u64(lines,line_ns,t.pixel_rate,true,&wanted);
        if (ret<0) return ret;
    } else if (r->tuning.max_exposure_ns && wanted>r->tuning.max_exposure_ns) wanted=r->tuning.max_exposure_ns;
    ret=cc_sensor_plan_exposure(r->sensor,&limits,wanted,r->tuning.min_frame_duration_ns,r->tuning.max_frame_duration_ns,plan);
    /* Reject an impossible manual shutter/frame pair before any control write. */
    if (!ret && manual) {
        uint32_t margin=cc_sensor_get_properties(r->sensor)->frame_integration_margin;
        if ((uint64_t)manual->value+margin>plan->frame_length) return -ERANGE;
        cc_sensor_timing t={limits.pixel_rate,limits.width,limits.height,plan->hblank,plan->vblank};
        uint64_t available;
        ret=cc_mul_div_u64(plan->frame_length-margin,((uint64_t)t.width+t.hblank)*UINT64_C(1000000000),
                           t.pixel_rate,true,&available);
        if (!ret) ret=cc_sensor_exposure_time(r->sensor,&t,(uint32_t)manual->value,&plan->exposure_ns);
        if (!ret && plan->exposure_ns>available) ret=-ERANGE;
        if (!ret) plan->exposure_lines=(uint32_t)manual->value;
    }
    return ret;
}
static int configure_planning(struct reference *r)
{
    const cc_ipa_configuration *c=&r->configuration;
    const cc_sensor_control_delay *exposure=NULL,*vblank=NULL,*hblank=NULL;
    for (uint32_t i=0;i<c->num_sensor_controls;++i) {
        const cc_control_info *info=&c->sensor_controls[i];
        const cc_sensor_control_delay *delay=&c->sensor_delays[i];
        if (delay->id!=info->id || delay->frames>CC_SENSOR_MAX_DELAY || delay->flags&~CC_SENSOR_CONTROL_PRIORITY)
            return -EINVAL;
        if (info->id==V4L2_CID_EXPOSURE) exposure=delay;
        if (info->id==V4L2_CID_VBLANK) {
            if (control_range(info)<0) return -ENOTSUP;
            if (c->timing_limits.min_vblank!=(uint64_t)info->minimum ||
                c->timing_limits.max_vblank!=(uint64_t)info->maximum || c->timing_limits.vblank_step!=info->step)
                return -EINVAL;
            vblank=delay;
        }
        if (info->id==V4L2_CID_HBLANK && !control_range(info)) hblank=delay;
    }
    if (!exposure || !vblank || !(vblank->flags&CC_SENSOR_CONTROL_PRIORITY) || vblank->frames<exposure->frames)
        return -ENOTSUP;
    r->variable_hblank=hblank && hblank->frames>=exposure->frames;
    cc_sensor_exposure_plan plan;
    return plan_exposure(r,NULL,r->desired_exposure_ns,&plan);
}
static int configure(void *context, const cc_ipa_configuration *configuration)
{
    struct reference next = *(struct reference *)context;
    next.configuration = *configuration;
    if ((configuration->pipeline == CC_IPA_VC4 && configuration->statistics_fourcc != CC_FOURCC('B','S','T','A')) ||
        ((configuration->pipeline == CC_IPA_CFE || configuration->pipeline == CC_IPA_PISP) &&
         configuration->statistics_fourcc != CC_FOURCC('R','P','F','S')) ||
        configuration->pipeline < CC_IPA_VC4 || configuration->pipeline > CC_IPA_PISP) return -ENOTSUP;
    next.sensor = configuration->sensor ? configuration->sensor : next.tuning.sensor;
    if (configuration->sensor && next.tuning.sensor && configuration->sensor != next.tuning.sensor) return -EINVAL;
    if (next.tuning.flags & CC_IPA_REFERENCE_AE) {
        const cc_sensor_properties *properties = cc_sensor_get_properties(next.sensor);
        if (!properties || (properties->features & (CC_SENSOR_GAIN | CC_SENSOR_EXPOSURE)) !=
            (CC_SENSOR_GAIN | CC_SENSOR_EXPOSURE)) return -ENOTSUP;
        next.exposure.id = next.gain.id = 0;
        for (uint32_t i = 0; i < configuration->num_sensor_controls; ++i) {
            const cc_control_info *info = &configuration->sensor_controls[i];
            if (info->id == V4L2_CID_EXPOSURE) next.exposure = *info;
            if (info->id == V4L2_CID_ANALOGUE_GAIN) next.gain = *info;
        }
        if (control_range(&next.exposure) < 0 || control_range(&next.gain) < 0) return -ENOTSUP;
        uint64_t duration;
        int ret = cc_sensor_frame_duration(&configuration->timing, &duration);
        if (ret < 0) return ret;
        cc_control_info range;
        ret = exposure_range(&next, &range);
        if (ret < 0) return ret;
        const cc_control *exposure = find(configuration->initial_controls, configuration->num_sensor_controls, V4L2_CID_EXPOSURE);
        const cc_control *gain = find(configuration->initial_controls, configuration->num_sensor_controls, V4L2_CID_ANALOGUE_GAIN);
        if (!exposure || !gain || gain->value < 0 || gain->value > UINT32_MAX) return -ENODATA;
        double actual;
        ret = cc_sensor_gain(next.sensor, (uint32_t)gain->value, &actual);
        if (ret < 0) return ret;
        cc_control initial_gain;
        ret = cc_sensor_quantize_gain(next.sensor, &next.gain, actual, &initial_gain, &actual);
        if (ret < 0) return ret;
        next.gain_code = (uint32_t)initial_gain.value;
        next.exposure_lines = quantize(&range, (double)exposure->value);
        ret=cc_sensor_exposure_time(next.sensor,&configuration->timing,next.exposure_lines,&next.desired_exposure_ns);
        if (!ret && duration_planning(&next)) ret=configure_planning(&next);
        if (ret<0) return ret;
    }
    if (next.tuning.flags & CC_IPA_REFERENCE_AWB) {
        if (configuration->pipeline == CC_IPA_CFE) return -ENOTSUP; /* Raw output has no white-balance stage. */
        if (configuration->pipeline == CC_IPA_VC4 &&
            (control_range(&configuration->red_balance) < 0 || control_range(&configuration->blue_balance) < 0 ||
             control_range(&configuration->digital_gain) < 0)) return -ENOTSUP;
        if (configuration->pipeline == CC_IPA_VC4 && (!quantize(&configuration->red_balance,1000) ||
            !quantize(&configuration->blue_balance,1000) || !quantize(&configuration->digital_gain,1000))) return -ENOTSUP;
    }
    next.red = next.blue = 1;
    cc_af *af=NULL;
    if (next.tuning.flags & CC_IPA_REFERENCE_AF) {
        cc_rectangle region={0,0,configuration->source.width,configuration->source.height};
        int ret=cc_af_create(&next.af_tuning,&af);
        if (!ret) ret=cc_af_configure(af,&region,&configuration->lens);
        if (ret<0) { cc_af_destroy(af); return ret; }
    }
    cc_af_destroy(next.af); next.af=af;
    next.af_have_status=next.af_clock_valid=false; next.af_mode=CC_AF_MANUAL;
    next.af_frame_count=next.af_update_count=next.af_timestamp=next.af_update_timestamp=0;
    next.af_status=(cc_af_status){0};
    next.have_pdaf=false;
    *(struct reference *)context = next;
    return 0;
}
static int start(void *context) { (void)context; return 0; }
static void stop(void *context) { (void)context; }
static void destroy(void *context) { struct reference *r=context; cc_af_destroy(r->af); free(r); }
static int prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    struct reference *r = context;
    if (request->af && !r->af) return -ENOTSUP;
    if (r->af) {
        bool force=request->af && ((request->af->flags & (CC_AF_SET_LENS|CC_AF_CANCEL|CC_AF_SET_PAUSE|
            CC_AF_SET_RANGE|CC_AF_SET_SPEED|CC_AF_SET_METERING|CC_AF_SET_WINDOWS)) ||
            ((request->af->flags & CC_AF_SET_MODE) && request->af->mode!=r->af_mode) ||
            ((request->af->flags & CC_AF_TRIGGER) && r->af_status.state!=CC_AF_SCANNING));
        int ret=request->af ? cc_af_set_controls(r->af,request->af):0;
        if (request->af && (request->af->flags & CC_AF_SET_MODE)) r->af_mode=request->af->mode;
        bool due=!r->af_have_status || force || (r->af_frame_count!=r->af_update_count &&
            (!r->af_clock_valid || r->af_timestamp<r->af_update_timestamp ||
             r->af_timestamp-r->af_update_timestamp>=UINT64_C(1000000000)/30));
        if (!ret && due) {
            cc_af_pdaf_grid grid={r->pdaf.rows,r->pdaf.cols,r->pdaf.regions};
            ret=cc_af_prepare(r->af,r->have_pdaf ? &grid:NULL,&r->af_status);
            r->have_pdaf=false;
            r->af_have_status=true; r->af_update_count=r->af_frame_count; r->af_update_timestamp=r->af_timestamp;
        }
        if (ret<0) return ret;
        settings->af=r->af_status;
        if (due && (settings->af.flags & CC_AF_LENS_VALID)) settings->flags|=CC_IPA_LENS;
    }
    if (r->tuning.flags & CC_IPA_REFERENCE_AE) {
        if (duration_planning(r)) {
            cc_sensor_exposure_plan plan;
            int ret=plan_exposure(r,request,r->desired_exposure_ns,&plan);
            if (ret<0) return ret;
            settings->sensor_controls[0]=(cc_control){.id=V4L2_CID_EXPOSURE,.value=plan.exposure_lines};
            settings->sensor_controls[1]=(cc_control){.id=V4L2_CID_ANALOGUE_GAIN,.value=r->gain_code};
            settings->sensor_controls[2]=(cc_control){.id=V4L2_CID_VBLANK,.value=plan.vblank};
            settings->num_sensor_controls=3;
            if (r->variable_hblank)
                settings->sensor_controls[settings->num_sensor_controls++]=(cc_control){.id=V4L2_CID_HBLANK,.value=plan.hblank};
            if (request->timing_limits.pixel_rate) r->configuration.timing_limits=request->timing_limits;
            if (request->timing.pixel_rate) r->configuration.timing=request->timing;
        } else {
            struct reference planned = *r;
            if (request->timing.pixel_rate) planned.configuration.timing = request->timing;
            const uint32_t ids[] = {V4L2_CID_VBLANK,V4L2_CID_HBLANK};
            uint32_t *values[] = {&planned.configuration.timing.vblank,&planned.configuration.timing.hblank};
            for (size_t i=0;i<2;++i) {
                const cc_control *override = find(request->sensor_controls,request->num_sensor_controls,ids[i]);
                if (override) {
                    if (override->value < 0 || override->value > UINT32_MAX) return -EINVAL;
                    *values[i] = (uint32_t)override->value;
                }
            }
            cc_control_info bounds;
            int ret = exposure_range(&planned,&bounds);
            if (ret < 0) return ret;
            settings->sensor_controls[0] = (cc_control) {.id=V4L2_CID_EXPOSURE,.value=quantize(&bounds,r->exposure_lines)};
            settings->sensor_controls[1] = (cc_control) {.id=V4L2_CID_ANALOGUE_GAIN,.value=r->gain_code};
            settings->num_sensor_controls = 2;
        }
    }
    if (r->tuning.flags & CC_IPA_REFERENCE_AWB) {
        double minimum = r->red < r->blue ? r->red : r->blue;
        if (minimum > 1) minimum = 1;
        if (r->configuration.pipeline == CC_IPA_VC4) {
            const cc_ipa_configuration *c = &r->configuration;
            settings->isp_controls[0] = (cc_control) {.id=V4L2_CID_RED_BALANCE,.value=quantize(&c->red_balance, r->red * 1000)};
            settings->isp_controls[1] = (cc_control) {.id=V4L2_CID_BLUE_BALANCE,.value=quantize(&c->blue_balance, r->blue * 1000)};
            settings->isp_controls[2] = (cc_control) {.id=V4L2_CID_DIGITAL_GAIN,.value=quantize(&c->digital_gain, 1000 / minimum)};
            settings->num_isp_controls = 3;
        } else {
            settings->flags |= CC_IPA_FRONTEND | CC_IPA_BACKEND;
            cc_cfe_configuration c = {.sensor=r->configuration.sensor_format,.image=r->configuration.source};
            int ret = cc_cfe_build_parameters(&c, NULL, &settings->frontend);
            if (ret < 0) return ret;
            settings->frontend.global.enables = PISP_FE_ENABLE_AGC_STATS | PISP_FE_ENABLE_AWB_STATS |
                PISP_FE_ENABLE_CDAF_STATS | PISP_FE_ENABLE_RGBY;
            double red = r->red / minimum, green = 1 / minimum, blue = r->blue / minimum;
            settings->frontend.rgby.gain_r = (uint16_t)(306 * red);
            settings->frontend.rgby.gain_g = (uint16_t)(601 * green);
            settings->frontend.rgby.gain_b = (uint16_t)(117 * blue);
            settings->backend.global.bayer_enables = PISP_BE_BAYER_ENABLE_WBG;
            settings->backend.wbg = (struct pisp_wbg_config) {
                .gain_r=(uint16_t)(1024 * red), .gain_g=(uint16_t)(1024 * green), .gain_b=(uint16_t)(1024 * blue)};
            settings->backend.demosaic.sharper = 8; settings->backend.demosaic.fc_mode = 1;
        }
    }
    return 0;
}
static void sensor_statistics(struct reference *next, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    next->have_pdaf=false;
    const cc_embedded_configuration *configuration=&next->configuration.embedded;
    const cc_sensor_properties *sensor=cc_sensor_get_properties(next->sensor);
    if (!sensor || !frame->embedded_size || !configuration->format.fourcc) return;
    uint32_t stride=configuration->format.planes[0].stride, height=configuration->format.height;
    cc_smia_layout layout={configuration->bits_per_pixel,stride,height<2 ? height:2};
    if ((sensor->features&CC_SENSOR_EMBEDDED) &&
        !cc_sensor_parse_embedded_status(next->sensor,&layout,frame->embedded_data,frame->embedded_size,&metadata->sensor_status))
        metadata->flags|=CC_IPA_METADATA_SENSOR;
    const unsigned char *data=frame->embedded_data;
    size_t offset=(size_t)stride*2;
    if ((sensor->features&CC_SENSOR_PDAF) && height>=3 && offset<frame->embedded_size) {
        size_t size=frame->embedded_size-offset; if (size>stride) size=stride;
        if (!cc_sensor_parse_pdaf(next->sensor,configuration->bits_per_pixel,data+offset,size,&next->pdaf)) {
            next->have_pdaf=true; metadata->flags|=CC_IPA_METADATA_PDAF;
        }
    }
    offset=(size_t)stride*3;
    if ((sensor->features&CC_SENSOR_HDR_STATISTICS) && height>=4 && offset<frame->embedded_size) {
        size_t size=frame->embedded_size-offset; if (size>stride) size=stride;
        if (!cc_sensor_parse_hdr_statistics(next->sensor,configuration->bits_per_pixel,data+offset,size,&metadata->hdr))
            metadata->flags|=CC_IPA_METADATA_HDR;
    }
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    struct reference *r = context;
    cc_ipa_statistics statistics=frame->statistics;
    const cc_ipa_statistics *s = &statistics;
    struct reference next = *r;
    sensor_statistics(&next,frame,metadata);
    if (metadata->flags&CC_IPA_METADATA_HDR) {
        statistics.flags|=CC_IPA_STATS_LUMA;
        statistics.luminance=clamp((double)metadata->hdr.average*4/65536,0,1);
    }
    if (r->af) {
        int ret=cc_af_process_native(r->af,frame->statistics_fourcc,frame->statistics_data,frame->statistics_size);
        if (ret<0) return ret;
        next.af_clock_valid=frame->timestamp_ns>next.af_timestamp && next.af_timestamp!=0;
        next.af_timestamp=frame->timestamp_ns; ++next.af_frame_count;
        metadata->flags|=CC_IPA_METADATA_AF; metadata->af=frame->af;
        if (frame->af.flags & CC_AF_LENS_VALID) {
            ret=cc_af_lens_position(r->af,frame->af.lens_setting,&metadata->af.lens_position);
            if (ret<0) return ret;
        }
    }
    if ((r->tuning.flags & CC_IPA_REFERENCE_AE) && (s->flags & CC_IPA_STATS_LUMA)) {
        if (!(s->luminance >= 0 && s->luminance <= 1)) return -EPROTO;
        const cc_control *exposure = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_EXPOSURE);
        const cc_control *gain = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_ANALOGUE_GAIN);
        if (!exposure || !gain || exposure->value <= 0 || exposure->value > UINT32_MAX ||
            gain->value < 0 || gain->value > UINT32_MAX) return -ENODATA;
        const cc_control *vblank = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_VBLANK);
        if (vblank) {
            if (vblank->value < 0 || vblank->value > UINT32_MAX) return -EPROTO;
            next.configuration.timing.vblank = (uint32_t)vblank->value;
        }
        const cc_control *hblank = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_HBLANK);
        if (hblank) {
            if (hblank->value < 0 || hblank->value > UINT32_MAX) return -EPROTO;
            next.configuration.timing.hblank = (uint32_t)hblank->value;
        }
        uint64_t exposure_ns;
        double actual_gain;
        int ret = cc_sensor_exposure_time(r->sensor, &next.configuration.timing, (uint32_t)exposure->value, &exposure_ns);
        if (!ret) ret = cc_sensor_gain(r->sensor, (uint32_t)gain->value, &actual_gain);
        if (ret < 0) return ret;
        double ratio = s->luminance > 0 ? r->tuning.target_luminance / s->luminance : 2;
        ratio = 1 + r->tuning.exposure_speed * (clamp(ratio, 0.5, 2) - 1);
        double product = (double)exposure_ns * actual_gain * ratio;
        uint64_t quantized_ns;
        if (duration_planning(&next)) {
            cc_sensor_exposure_plan plan;
            uint64_t wanted=product>=(double)UINT64_MAX ? UINT64_MAX:product<1 ? 1:(uint64_t)product;
            ret=plan_exposure(&next,NULL,wanted,&plan);
            if (ret<0) return ret;
            next.exposure_lines=plan.exposure_lines;
            next.desired_exposure_ns=wanted;
            quantized_ns=plan.exposure_ns;
        } else {
            cc_control_info range;
            ret = exposure_range(&next, &range);
            if (ret < 0) return ret;
            uint64_t max_ns;
            ret = cc_sensor_exposure_time(r->sensor, &next.configuration.timing, quantize(&range, (double)range.maximum), &max_ns);
            if (ret < 0) return ret;
            uint32_t lines;
            uint64_t wanted_ns = product >= (double)max_ns ? max_ns : (uint64_t)product;
            ret = cc_sensor_exposure_lines(r->sensor, &next.configuration.timing, wanted_ns, &lines);
            if (ret < 0) return ret;
            next.exposure_lines = quantize(&range, lines);
            ret = cc_sensor_exposure_time(r->sensor, &next.configuration.timing, next.exposure_lines, &quantized_ns);
        }
        if (ret < 0 || !quantized_ns) return ret < 0 ? ret : -ERANGE;
        cc_control control;
        double quantized_gain;
        ret = cc_sensor_quantize_gain(r->sensor, &r->gain, product / (double)quantized_ns,
                                      &control, &quantized_gain);
        /* At the exposure floor the requested product can be below unity gain. */
        if (ret == -EINVAL && product < (double)quantized_ns)
            ret = cc_sensor_quantize_gain(r->sensor, &r->gain, 1, &control, &quantized_gain);
        if (ret < 0) return ret;
        next.gain_code = (uint32_t)control.value;
        metadata->flags |= CC_IPA_METADATA_AE;
        metadata->exposure_ns = exposure_ns; metadata->analogue_gain = actual_gain; metadata->luminance = s->luminance;
        if (distance(s->luminance, r->tuning.target_luminance) <= r->tuning.target_luminance * 0.05)
            metadata->flags |= CC_IPA_AE_CONVERGED;
    }
    if ((r->tuning.flags & CC_IPA_REFERENCE_AWB) && (s->flags & CC_IPA_STATS_COLOUR)) {
        if (!(s->red >= 0 && s->red <= 1 && s->green >= 0 && s->green <= 1 && s->blue >= 0 && s->blue <= 1)) return -EPROTO;
        if (s->red > 0 && s->green > 0 && s->blue > 0) {
            double red = clamp(s->green / s->red, r->tuning.min_colour_gain, r->tuning.max_colour_gain);
            double blue = clamp(s->green / s->blue, r->tuning.min_colour_gain, r->tuning.max_colour_gain);
            next.red += r->tuning.white_balance_speed * (red - next.red);
            next.blue += r->tuning.white_balance_speed * (blue - next.blue);
            double applied_red = 1, applied_blue = 1;
            if (r->configuration.pipeline == CC_IPA_VC4) {
                const cc_control *r_gain = find(frame->isp_controls, frame->num_isp_controls, V4L2_CID_RED_BALANCE);
                const cc_control *b_gain = find(frame->isp_controls, frame->num_isp_controls, V4L2_CID_BLUE_BALANCE);
                if (!r_gain || !b_gain) return -ENODATA;
                applied_red = (double)r_gain->value / 1000; applied_blue = (double)b_gain->value / 1000;
            } else if (frame->backend && frame->backend->global.bayer_enables & PISP_BE_BAYER_ENABLE_WBG) {
                if (!frame->backend->wbg.gain_g) return -EPROTO;
                applied_red = (double)frame->backend->wbg.gain_r / frame->backend->wbg.gain_g;
                applied_blue = (double)frame->backend->wbg.gain_b / frame->backend->wbg.gain_g;
            }
            if (!(applied_red > 0) || !(applied_blue > 0)) return -EPROTO;
            metadata->flags |= CC_IPA_METADATA_AWB;
            metadata->red_gain = applied_red; metadata->blue_gain = applied_blue;
            if (distance(applied_red, red) <= red * 0.05 && distance(applied_blue, blue) <= blue * 0.05)
                metadata->flags |= CC_IPA_AWB_CONVERGED;
        }
    }
    *r = next;
    return 0;
}
int cc_ipa_reference_create(const cc_ipa_reference_tuning *tuning, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!tuning || !tuning->flags || tuning->flags & ~(uint32_t)(CC_IPA_REFERENCE_AE | CC_IPA_REFERENCE_AWB | CC_IPA_REFERENCE_AF) ||
        ((tuning->min_frame_duration_ns || tuning->max_frame_duration_ns) &&
         (!(tuning->flags&CC_IPA_REFERENCE_AE) || !tuning->min_frame_duration_ns ||
          tuning->min_frame_duration_ns>tuning->max_frame_duration_ns)) ||
        ((tuning->flags & CC_IPA_REFERENCE_AF) && !tuning->af) ||
        !(tuning->target_luminance > 0 && tuning->target_luminance < 1) ||
        !(tuning->exposure_speed > 0 && tuning->exposure_speed <= 1) ||
        !(tuning->white_balance_speed > 0 && tuning->white_balance_speed <= 1) ||
        !(tuning->min_colour_gain >= 0.5 && tuning->min_colour_gain <= 1) ||
        !(tuning->max_colour_gain >= 1 && tuning->max_colour_gain <= 4)) return -EINVAL;
    struct reference *r = calloc(1, sizeof(*r));
    if (!r) return -ENOMEM;
    r->tuning = *tuning;
    if (tuning->flags & CC_IPA_REFERENCE_AF) {
        r->af_tuning=*tuning->af; r->tuning.af=&r->af_tuning;
        int result=cc_af_create(&r->af_tuning,&r->af);
        if (result<0) { free(r); return result; }
    }
    const cc_ipa_ops ops = {CC_IPA_ABI_VERSION, sizeof(cc_ipa_ops), configure, start, stop, prepare, process, destroy,
        (tuning->flags & CC_IPA_REFERENCE_AF ? CC_IPA_CAP_LENS:0U) |
        (tuning->max_frame_duration_ns ? CC_IPA_CAP_SENSOR_TIMING:0U)};
    int ret = cc_ipa_create(&ops, r, out);
    if (ret < 0) destroy(r);
    return ret;
}
