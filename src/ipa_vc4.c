/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Single-exposure VC4/PiSP controller. Calibration, capped Bayesian colour-error
 * likelihood and lux priors derived from Raspberry Pi Ltd libcamera 0.7.2.
 * Search is synchronous and bounded; adaptive ALSC/contrast/HDR are excluded. */
#include "vc4_profile.h"
#include "pisp_format.h"
#include "photo_processing.h"
#include <ccamera/ipa.h>
#include <errno.h>
#include <float.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
struct photo_statistics {
    unsigned zones, bins, scale;
    uint32_t histogram[1024];
    struct bcm2835_isp_stats_region awb_stats[1024], agc_stats[AGC_REGIONS];
};
struct vc4_controller {
    cc_ipa_configuration config;
    cc_ipa_vc4_options options;
    const struct cc_vc4_profile_data *profile;
    bool pisp;
    cc_control_info gain;
    double exposure_product, red, blue, temperature, lux;
    double next_red, next_blue, next_temperature;
    cc_photo_capabilities capabilities;
    cc_af *af;
    cc_af_status af_status;
    cc_sensor_pdaf pdaf;
    bool have_pdaf, af_have_status, af_clock_valid;
    enum cc_af_mode af_mode;
    uint64_t af_frame_count, af_update_count, af_timestamp, af_update_timestamp;
};
static double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static double absolute(double x) { return x < 0 ? -x : x; }
static const cc_control *find(const cc_control *v, size_t n, uint32_t id)
{
    for (size_t i = 0; i < n; ++i)
        if (v[i].id == id)
            return &v[i];
    return NULL;
}
static double pwl(const double *v, size_t n, double x)
{
    if (x <= v[0])
        return v[1];
    for (size_t i = 2; i < n; i += 2)
        if (x <= v[i])
            return v[i - 1] + (v[i + 1] - v[i - 1]) * (x - v[i - 2]) / (v[i] - v[i - 2]);
    return v[n - 1];
}
static void ct_values(const struct cc_vc4_profile_data *p, double t, double *r, double *b)
{
    unsigned i = 0;
    while (i + 2 < p->num_ct && t > p->ct[i + 1][0])
        ++i;
    double f = clamp((t - p->ct[i][0]) / (p->ct[i + 1][0] - p->ct[i][0]), 0, 1);
    *r = p->ct[i][1] * (1 - f) + p->ct[i + 1][1] * f;
    *b = p->ct[i][2] * (1 - f) + p->ct[i + 1][2] * f;
}
static double prior(const struct cc_vc4_profile_data *p, double lux, double ct)
{
    size_t i = 0;
    while (i + 2 < p->num_priors && lux > p->priors[i + 1].lux)
        ++i;
    const struct cc_vc4_prior *a = &p->priors[i], *b = &p->priors[i + 1];
    double lo = pwl(a->pwl, a->count, ct), hi = pwl(b->pwl, b->count, ct);
    return lo + (hi - lo) * clamp((lux - a->lux) / (b->lux - a->lux), 0, 1);
}
static int range_valid(const cc_control_info *r)
{
    return r->type == V4L2_CTRL_TYPE_INTEGER && r->minimum >= 0 && r->maximum >= r->minimum &&
           r->maximum <= INT32_MAX && r->step && r->elems <= 1 &&
           !(r->flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE |
                         V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_WRITE_ONLY | V4L2_CTRL_FLAG_MODIFY_LAYOUT));
}
static int native_gain(const struct vc4_controller *v, int64_t code, double *gain)
{
    if (code < v->gain.minimum || code > v->gain.maximum ||
        (uint64_t)(code - v->gain.minimum) % v->gain.step || code > UINT32_MAX)
        return -ERANGE;
    return cc_sensor_gain(v->config.sensor, (uint32_t)code, gain);
}
static int photo_capabilities(void *context, cc_photo_capabilities *out)
{
    *out=((struct vc4_controller *)context)->capabilities;
    return 0;
}
static int build_capabilities(struct vc4_controller *v, const cc_ipa_configuration *c,
    const struct cc_vc4_profile_data *p, cc_photo_capabilities *out)
{
    cc_photo_capabilities caps={.controls=CC_PHOTO_ALL, .metering_modes=7,
        .white_balance_modes=p->bayes ? 255U : (1U<<CC_PHOTO_WB_AUTO)|(1U<<CC_PHOTO_WB_MANUAL),
        .denoise_modes=3, .profile=p->info.profile, .defaults=cc_photo_defaults(&v->options),
        .colour_gain={0.5,4,1}, .exposure_value={-8,8,0}, .brightness={-1,1,0},
        .contrast={0,4,1}, .saturation={0,2,1}, .sharpness={0,4,1},
        .native_gain=v->gain, .timing=c->timing_limits, .num_delays=c->num_sensor_controls};
    caps.timing.min_hblank=caps.timing.max_hblank=c->timing.hblank;
    memcpy(caps.delays,c->sensor_delays,caps.num_delays*sizeof(caps.delays[0]));
    const cc_sensor_properties *properties=cc_sensor_get_properties(c->sensor);
    if (properties->features & CC_SENSOR_DRIVER_DEPENDENT_DELAYS) caps.flags|=CC_PHOTO_CAP_PROVISIONAL_DELAYS;
    int ret=cc_sensor_timing_bounds(&caps.timing,&caps.min_frame_duration_ns,&caps.max_frame_duration_ns);
    if (ret<0) return ret;
    cc_sensor_exposure_plan low, high;
    ret=cc_sensor_plan_exposure(c->sensor,&caps.timing,1,0,0,&low);
    if (!ret) ret=cc_sensor_plan_exposure(c->sensor,&caps.timing,UINT64_MAX,0,0,&high);
    if (ret<0) return ret;
    caps.min_exposure_ns=low.exposure_ns; caps.max_exposure_ns=high.exposure_ns;
    caps.min_auto_exposure_ns=(uint64_t)p->shutter[0]*1000;
    if (caps.min_auto_exposure_ns<caps.min_exposure_ns) caps.min_auto_exposure_ns=caps.min_exposure_ns;
    caps.max_auto_exposure_ns=(uint64_t)p->shutter[p->num_exposure-1]*1000;
    if (caps.max_auto_exposure_ns>caps.max_exposure_ns) caps.max_auto_exposure_ns=caps.max_exposure_ns;
    for (unsigned i=0; i<7; ++i) caps.white_balance_temperature[i]=(cc_photo_range){p->wb_modes[i][0],p->wb_modes[i][1],0};
    cc_control code;
    ret=cc_sensor_quantize_gain(c->sensor,&v->gain,1,&code,&caps.analogue_gain.minimum);
    if (!ret) ret=cc_sensor_quantize_gain(c->sensor,&v->gain,64,&code,&caps.analogue_gain.maximum);
    if (ret<0) return ret;
    caps.analogue_gain.default_value=0; /* Automatic. */
    ret=cc_sensor_quantize_gain(c->sensor,&v->gain,p->gain[0],&code,&caps.automatic_gain.minimum);
    if (!ret) ret=cc_sensor_quantize_gain(c->sensor,&v->gain,p->gain[p->num_exposure-1],&code,&caps.automatic_gain.maximum);
    if (ret<0) return ret;
    if (caps.analogue_gain.minimum>64) return -ENOTSUP;
    /* Capture consumes only hide/mistrust fields. The supported modules' trust
     * policies use image dimensions and minimum frame duration, not binning. */
    cc_sensor_mode_description mode={.width=c->sensor_format.width ? c->sensor_format.width:c->source.width,
        .height=c->sensor_format.height ? c->sensor_format.height:c->source.height,
        .binning_x=1, .scale_x=1, .min_frame_duration_ns=caps.min_frame_duration_ns};
    ret=cc_sensor_get_mode_policy(c->sensor,&mode,&caps.policy);
    if (ret<0) return ret;
    if (v->options.autofocus) {
        if (!p->af) return -ENOTSUP;
        caps.flags|=CC_PHOTO_CAP_AF;
        if (c->embedded.format.fourcc && (properties->features & CC_SENSOR_PDAF)) caps.flags|=CC_PHOTO_CAP_PDAF;
        caps.focus=*p->af; caps.lens=c->lens;
    }
    *out=caps; return 0;
}
static int configure(void *context, const cc_ipa_configuration *c)
{
    struct vc4_controller *v = context;
    if (c->pipeline != (v->pisp ? CC_IPA_PISP : CC_IPA_VC4) ||
        c->statistics_fourcc != (v->pisp ? CC_FOURCC('R','P','F','S') : CC_FOURCC('B','S','T','A')) || !c->sensor)
        return -ENOTSUP;
    enum cc_vc4_profile selected;
    int ret = v->pisp ? cc_pisp_select_profile(c->sensor,v->options.profile,&selected) :
        cc_vc4_select_profile(c->sensor,v->options.profile,&selected);
    if (ret < 0)
        return ret;
    const struct cc_vc4_profile_data *p = v->pisp ? cc_pisp_profile_data(selected) : cc_vc4_profile_data(selected);
    v->gain = (cc_control_info){0};
    const cc_control *e = find(c->initial_controls, c->num_sensor_controls, V4L2_CID_EXPOSURE);
    const cc_control *g = find(c->initial_controls, c->num_sensor_controls, V4L2_CID_ANALOGUE_GAIN);
    bool have_e = false, have_v = false;
    for (uint32_t i = 0; i < c->num_sensor_controls; ++i) {
        if (c->sensor_delays[i].id!=c->sensor_controls[i].id || c->sensor_delays[i].frames>CC_SENSOR_MAX_DELAY ||
            c->sensor_delays[i].flags & ~CC_SENSOR_CONTROL_PRIORITY) return -ENOTSUP;
        if (c->sensor_controls[i].id == V4L2_CID_ANALOGUE_GAIN)
            v->gain = c->sensor_controls[i];
        if (c->sensor_controls[i].id == V4L2_CID_EXPOSURE)
            have_e = range_valid(&c->sensor_controls[i]) != 0;
        if (c->sensor_controls[i].id == V4L2_CID_VBLANK)
            have_v = range_valid(&c->sensor_controls[i]) != 0;
    }
    if (!e || !g || !have_e || !have_v || !range_valid(&v->gain) || e->value < 1 || e->value > UINT32_MAX ||
        !c->timing.pixel_rate || !c->timing_limits.pixel_rate ||
        (!v->pisp && (!range_valid(&c->red_balance) || !range_valid(&c->blue_balance) || !range_valid(&c->digital_gain))))
        return -ENOTSUP;
    if (!v->pisp && (c->red_balance.id!=V4L2_CID_RED_BALANCE || c->blue_balance.id!=V4L2_CID_BLUE_BALANCE ||
        c->digital_gain.id!=V4L2_CID_DIGITAL_GAIN || c->red_balance.minimum>500 || c->red_balance.maximum<4000 ||
        c->blue_balance.minimum>500 || c->blue_balance.maximum<4000 ||
        c->digital_gain.minimum>1000 || c->digital_gain.maximum<2000)) return -ENOTSUP;
    const cc_sensor_control_delay *ed = NULL, *vd = NULL;
    for (uint32_t i = 0; i < c->num_sensor_controls; ++i) {
        if (c->sensor_delays[i].id == V4L2_CID_EXPOSURE)
            ed = &c->sensor_delays[i];
        if (c->sensor_delays[i].id == V4L2_CID_VBLANK)
            vd = &c->sensor_delays[i];
    }
    if (!ed || !vd || vd->frames < ed->frames || (ed->flags & CC_SENSOR_CONTROL_PRIORITY) ||
        !(vd->flags & CC_SENSOR_CONTROL_PRIORITY))
        return -ENOTSUP;
    uint64_t minimum_ns, maximum_ns;
    int bounds = cc_sensor_timing_bounds(&c->timing_limits, &minimum_ns, &maximum_ns);
    if (bounds < 0)
        return bounds;
    if (v->pisp && (c->source.width != c->sensor_format.width || c->source.height != c->sensor_format.height ||
        c->geometry.width != c->source.width || c->geometry.height != c->source.height)) return -ENOTSUP;
    if (v->pisp) {
        struct cc_pisp_raw_layout raw;
        ret=cc_pisp_raw_layout(&c->source,&raw);
        if (ret<0) return ret;
        if (raw.bayer_order==PISP_BAYER_ORDER_GREYSCALE) return -ENOTSUP;
        struct pisp_fe_config fe; struct pisp_be_config be;
        ret = cc_pisp_build_tuning_with_shading(selected, &c->geometry, 4500, 1, 1, 1,
            &v->options.lens_shading, &fe, &be);
    } else {
        cc_vc4_tuning test;
        ret = cc_vc4_build_tuning_with_shading(selected, &c->geometry, 4500, 1,
            &v->options.lens_shading, &test);
    }
    if (ret < 0)
        return ret;
    uint64_t ns;
    ret = cc_sensor_exposure_time(c->sensor, &c->timing, (uint32_t)e->value, &ns);
    if (ret < 0)
        return ret;
    v->config = *c;
    v->profile = p;
    double initial_gain;
    ret = native_gain(v, g->value, &initial_gain);
    if (ret < 0)
        return ret;
    v->exposure_product = (double)ns * initial_gain;
    v->temperature = v->next_temperature = 4500;
    double r = 1, b = 1;
    if (p->bayes)
        ct_values(p, 4500, &r, &b);
    v->red = v->next_red = v->options.red_gain > 0 ? v->options.red_gain : p->sensitivity_r / r;
    v->blue = v->next_blue = v->options.blue_gain > 0 ? v->options.blue_gain : p->sensitivity_b / b;
    if (v->options.red_gain > 0 && p->bayes) {
        double best = DBL_MAX;
        for (unsigned i = 0; i <= 110; ++i) {
            double ct = 2500 + 50.0 * i;
            ct_values(p, ct, &r, &b);
            double dr = p->sensitivity_r / r - v->red, db = p->sensitivity_b / b - v->blue;
            double error = dr * dr + db * db;
            if (error < best) {
                best = error;
                v->temperature = v->next_temperature = ct;
            }
        }
    }
    v->lux = 400;
    cc_photo_capabilities caps;
    ret=build_capabilities(v,c,p,&caps);
    if (ret<0) return ret;
    cc_af *af=NULL;
    if (v->options.autofocus) {
        ret=cc_af_create(p->af,&af);
        if (!ret) ret=cc_af_configure(af,&c->geometry.crop,&c->lens);
        if (ret<0) { cc_af_destroy(af); return ret; }
    }
    cc_af_destroy(v->af); v->af=af;
    v->af_have_status=v->af_clock_valid=v->have_pdaf=false;
    v->af_mode=CC_AF_MANUAL; v->af_status=(cc_af_status){0};
    v->af_frame_count=v->af_update_count=v->af_timestamp=v->af_update_timestamp=0;
    v->capabilities=caps;
    return 0;
}
static int start(void *p)
{
    (void)p;
    return 0;
}
static void stop(void *p) { (void)p; }
static void destroy(void *p) { struct vc4_controller *v=p; cc_af_destroy(v->af); free(v); }
static void choose_exposure(const struct cc_vc4_profile_data *p, double product, double *shutter, double *gain)
{
    *shutter = p->shutter[0] * 1000;
    *gain = p->gain[0];
    for (size_t i = 1; i < p->num_exposure; ++i) {
        *shutter = clamp(product / *gain, *shutter, p->shutter[i] * 1000);
        if (*shutter * *gain >= product)
            return;
        *gain = clamp(product / *shutter, *gain, p->gain[i]);
        if (*shutter * *gain >= product)
            return;
    }
}
static int wb_control(const cc_control_info *range, double gain, cc_control *out)
{
    double native = gain * 1000;
    if (!(native >= (double)range->minimum && native <= (double)range->maximum))
        return -ERANGE;
    int64_t value = range->minimum +
                    (int64_t)((native - (double)range->minimum) / (double)range->step) * (int64_t)range->step;
    *out = (cc_control){.id = range->id, .value = value};
    return 0;
}
static int prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *s)
{
    struct vc4_controller *v = context;
    const struct cc_vc4_profile_data *p = v->profile;
    cc_photo_controls photo=cc_photo_resolve(&v->options,request->photo);
    if (!(v->capabilities.white_balance_modes & (1U<<photo.white_balance))) return -ENOTSUP;
    if (photo.min_frame_duration_ns && (photo.min_frame_duration_ns>v->capabilities.max_frame_duration_ns ||
        photo.max_frame_duration_ns<v->capabilities.min_frame_duration_ns)) return -ERANGE;
    if (request->af && !v->af) return -ENOTSUP;
    const cc_control *manual_e =
        find(request->sensor_controls, request->num_sensor_controls, V4L2_CID_EXPOSURE);
    const cc_control *manual_g =
        find(request->sensor_controls, request->num_sensor_controls, V4L2_CID_ANALOGUE_GAIN);
    cc_sensor_timing_limits limits = request->timing_limits;
    cc_sensor_timing timing = request->timing;
    if (!limits.pixel_rate || !limits.hblank_step || !limits.vblank_step || !limits.exposure_step ||
        timing.pixel_rate != limits.pixel_rate || timing.width != limits.width ||
        timing.height != limits.height)
        return -EINVAL;
    if (timing.hblank!=v->config.timing.hblank || timing.pixel_rate!=v->config.timing.pixel_rate)
        return -ENOTSUP; /* Reconfigure after changing the line clock. */
    limits.min_hblank = limits.max_hblank = timing.hblank;
    const cc_control *hb = find(request->sensor_controls, request->num_sensor_controls, V4L2_CID_HBLANK);
    if (hb && hb->value != timing.hblank)
        return -ENOTSUP;
    const cc_control *vb = find(request->sensor_controls, request->num_sensor_controls, V4L2_CID_VBLANK);
    if (vb) {
        if (vb->value < limits.min_vblank || vb->value > limits.max_vblank ||
            ((uint64_t)vb->value - limits.min_vblank) % limits.vblank_step)
            return -ERANGE;
        limits.min_vblank = limits.max_vblank = (uint32_t)vb->value;
    }
    double shutter, gain;
    choose_exposure(p, v->exposure_product, &shutter, &gain);
    if (manual_g) {
        int ret = native_gain(v, manual_g->value, &gain);
        if (ret < 0)
            return ret;
        shutter = clamp(v->exposure_product / gain, p->shutter[0] * 1000,
                        p->shutter[p->num_exposure - 1] * 1000);
    }
    if (!manual_g && photo.analogue_gain>0) {
        if (photo.analogue_gain<v->capabilities.analogue_gain.minimum ||
            photo.analogue_gain>v->capabilities.analogue_gain.maximum) return -ERANGE;
        gain=photo.analogue_gain;
        shutter=clamp(v->exposure_product/gain,p->shutter[0]*1000,p->shutter[p->num_exposure-1]*1000);
    }
    uint64_t wanted = (uint64_t)shutter;
    if (manual_e) {
        if (manual_e->value < limits.min_exposure || manual_e->value > UINT32_MAX ||
            ((uint64_t)manual_e->value - limits.min_exposure) % limits.exposure_step)
            return -ERANGE;
        int ret = cc_sensor_exposure_time(v->config.sensor, &timing, (uint32_t)manual_e->value, &wanted);
        if (ret < 0)
            return ret;
    } else if (photo.exposure_ns) {
        wanted=photo.exposure_ns;
        if (wanted<v->capabilities.min_exposure_ns || wanted>v->capabilities.max_exposure_ns) return -ERANGE;
    } else {
        int ret = cc_sensor_flicker_exposure(v->config.sensor, &timing, wanted, photo.mains_hz, &wanted);
        if (ret < 0)
            return ret;
    }
    cc_sensor_exposure_plan plan;
    int ret = cc_sensor_plan_exposure(v->config.sensor, &limits, wanted,
        photo.min_frame_duration_ns, photo.max_frame_duration_ns, &plan);
    if (ret < 0)
        return ret;
    if (manual_e && plan.exposure_lines != (uint32_t)manual_e->value)
        return -ERANGE;
    if (!manual_g && photo.analogue_gain==0)
        gain = clamp(v->exposure_product / (double)plan.exposure_ns, p->gain[0], p->gain[p->num_exposure - 1]);
    cc_control control;
    double actual_gain;
    ret = cc_sensor_quantize_gain(v->config.sensor, &v->gain, gain, &control, &actual_gain);
    if (ret < 0)
        return ret;
    if (manual_g) {
        control = *manual_g;
        ret = native_gain(v, manual_g->value, &actual_gain);
        if (ret < 0)
            return ret;
    }
    s->sensor_controls[0] = (cc_control){.id = V4L2_CID_VBLANK, .value = plan.vblank};
    s->sensor_controls[1] = (cc_control){.id = V4L2_CID_EXPOSURE, .value = plan.exposure_lines};
    s->sensor_controls[2] = control;
    s->num_sensor_controls = 3;
    v->red = v->next_red;
    v->blue = v->next_blue;
    v->temperature = v->next_temperature;
    if (photo.white_balance==CC_PHOTO_WB_MANUAL) {
        v->red=photo.red_gain; v->blue=photo.blue_gain;
        /* A manual WB pair determines the nearest calibrated CT for CCM/LSC. */
        double best=DBL_MAX;
        if (p->bayes) for (unsigned i=0; i<=110; ++i) {
            double ct=2500+50.0*i, r, b; ct_values(p,ct,&r,&b);
            double dr=p->sensitivity_r/r-v->red, db=p->sensitivity_b/b-v->blue;
            double error=dr*dr+db*db;
            if (error<best) { best=error; v->temperature=ct; }
        }
    } else if (p->bayes) {
        v->temperature=clamp(v->temperature,p->wb_modes[photo.white_balance][0],p->wb_modes[photo.white_balance][1]);
    }
    s->photo=photo;
    s->colour_temperature = v->temperature;
    if (v->pisp) {
        ret = cc_pisp_build_tuning_with_shading(p->info.profile, &v->config.geometry, v->temperature, actual_gain,
            v->red, v->blue, &v->options.lens_shading, &s->frontend, &s->backend);
        if (ret<0) return ret;
        cc_photo_apply_pisp(p,&photo,&s->frontend,&s->backend);
        s->flags |= CC_IPA_FRONTEND | CC_IPA_BACKEND;
    } else {
        ret = cc_vc4_build_tuning_with_shading(p->info.profile, &v->config.geometry, v->temperature, actual_gain,
            &v->options.lens_shading, &s->vc4);
        if (ret < 0)
            return ret;
        cc_photo_apply_vc4(&photo,&s->vc4);
        s->flags |= CC_IPA_VC4_TUNING;
        double minimum = v->red < v->blue ? v->red : v->blue;
        if (minimum > 1)
            minimum = 1;
        ret = wb_control(&v->config.red_balance, v->red, &s->isp_controls[0]);
        if (!ret)
            ret = wb_control(&v->config.blue_balance, v->blue, &s->isp_controls[1]);
        if (!ret)
            ret = wb_control(&v->config.digital_gain, 1 / minimum, &s->isp_controls[2]);
        s->num_isp_controls = 3;
    }
    if (ret<0) return ret;
    if (v->af) {
        bool force=request->af && ((request->af->flags & (CC_AF_SET_LENS|CC_AF_CANCEL|CC_AF_SET_PAUSE|
            CC_AF_SET_RANGE|CC_AF_SET_SPEED|CC_AF_SET_METERING|CC_AF_SET_WINDOWS)) ||
            ((request->af->flags & CC_AF_SET_MODE) && request->af->mode!=v->af_mode) ||
            ((request->af->flags & CC_AF_TRIGGER) && v->af_status.state!=CC_AF_SCANNING));
        ret=request->af ? cc_af_set_controls(v->af,request->af):0;
        if (ret<0) return ret;
        if (request->af && (request->af->flags & CC_AF_SET_MODE)) v->af_mode=request->af->mode;
        bool due=!v->af_have_status || force || (v->af_frame_count!=v->af_update_count &&
            (!v->af_clock_valid || v->af_timestamp<v->af_update_timestamp ||
             v->af_timestamp-v->af_update_timestamp>=UINT64_C(1000000000)/30));
        if (due) {
            cc_af_pdaf_grid grid={v->pdaf.rows,v->pdaf.cols,v->pdaf.regions};
            ret=cc_af_prepare(v->af,v->have_pdaf ? &grid:NULL,&v->af_status);
            v->have_pdaf=false;
            if (ret<0) return ret;
            v->af_have_status=true; v->af_update_count=v->af_frame_count; v->af_update_timestamp=v->af_timestamp;
        }
        s->af=v->af_status;
        if (due && (s->af.flags & CC_AF_LENS_VALID)) s->flags|=CC_IPA_LENS;
    }
    return ret;
}
static int region_valid(const struct bcm2835_isp_stats_region *z)
{
    uint64_t limit = (uint64_t)z->counted * 8191;
    return z->r_sum <= limit && z->g_sum <= limit && z->b_sum <= limit;
}
struct colour_zone { double r, g, b; };
static int compare_red(const void *a, const void *b)
{
    const struct colour_zone *x = a, *y = b;
    double d = x->g * y->r - y->g * x->r;
    return (d > 0) - (d < 0);
}
static int compare_blue(const void *a, const void *b)
{
    const struct colour_zone *x = a, *y = b;
    double d = x->g * y->b - y->g * x->b;
    return (d > 0) - (d < 0);
}
static bool grey_world(struct vc4_controller *v, const struct photo_statistics *stats)
{
    struct colour_zone zones[1024];
    size_t count = 0;
    for (unsigned i = 0; i < stats->zones; ++i) {
        const struct bcm2835_isp_stats_region *z = &stats->awb_stats[i];
        if (!z->counted || (uint64_t)z->counted * 10 < (uint64_t)z->counted + z->notcounted ||
            z->g_sum < (uint64_t)z->counted * (stats->scale/256))
            continue;
        zones[count++] = (struct colour_zone){(double)z->r_sum / z->counted,
            (double)z->g_sum / z->counted, (double)z->b_sum / z->counted};
    }
    if (count < 10)
        return false;
    /* The reference's middle-half average excludes coloured outlier regions.
     * NoIR has no meaningful calibrated temperature locus; retain 4500 K. */
    size_t discard = count / 4;
    double r = 0, rg = 0, b = 0, bg = 0;
    qsort(zones, count, sizeof(zones[0]), compare_red);
    for (size_t i = discard; i < count - discard; ++i) {
        r += zones[i].r;
        rg += zones[i].g;
    }
    qsort(zones, count, sizeof(zones[0]), compare_blue);
    for (size_t i = discard; i < count - discard; ++i) {
        b += zones[i].b;
        bg += zones[i].g;
    }
    v->next_red = v->red + 0.4 * (clamp(rg / (r + 1), 0.5, 4) - v->red);
    v->next_blue = v->blue + 0.4 * (clamp(bg / (b + 1), 0.5, 4) - v->blue);
    v->next_temperature = 4500;
    return true;
}
static bool awb(struct vc4_controller *v, const struct photo_statistics *stats, enum cc_photo_white_balance mode)
{
    const struct cc_vc4_profile_data *p = v->profile;
    if (!p->bayes)
        return grey_world(v, stats);
    double zr[1024], zb[1024];
    unsigned count = 0;
    for (unsigned i = 0; i < stats->zones; ++i) {
        const struct bcm2835_isp_stats_region *z = &stats->awb_stats[i];
        if (!z->counted || (uint64_t)z->counted * 10 < (uint64_t)z->counted + z->notcounted ||
            z->g_sum < (uint64_t)z->counted * (stats->scale/256))
            continue;
        zr[count] = (double)z->r_sum * p->sensitivity_r / ((double)z->g_sum + 0.125);
        zb[count] = (double)z->b_sum * p->sensitivity_b / ((double)z->g_sum + 0.125);
        ++count;
    }
    if (count < 10)
        return false;
    double best = DBL_MAX, best_t = 4500, best_r = 1, best_b = 1;
    /* Bounded Bayesian likelihood search along/off the calibrated CT locus.
     * Same capped colour-error and interpolated lux prior as upstream; finer
     * deterministic sampling replaces upstream's quadratic search refinements. */
    double lo=p->wb_modes[mode][0], hi=p->wb_modes[mode][1];
    for (unsigned step = 0; step <= 128; ++step) {
        double ct = lo+(hi-lo)*step/128, r, b;
        ct_values(p, ct, &r, &b);
        double r0, b0, r1, b1;
        ct_values(p, ct - 25, &r0, &b0);
        ct_values(p, ct + 25, &r1, &b1);
        double tr = b1 - b0, tb = r0 - r1, root = tr * tr + tb * tb;
        if (root > 0) {
            double x = root < 1 ? 1 : root;
            for (unsigned j = 0; j < 24; ++j)
                x = 0.5 * (x + root / x);
            tr /= x;
            tb /= x;
        }
        for (unsigned k = 0; k <= 8; ++k) {
            double delta = -p->transverse_neg + (p->transverse_neg + p->transverse_pos) * (double)k / 8;
            double rr = r + delta * tr, bb = b + delta * tb;
            double energy = -prior(p, v->lux, ct) * (double)count / stats->zones;
            for (unsigned z = 0; z < count; ++z) {
                double dr = zr[z] / rr - 1, db = zb[z] / bb - 1, d = dr * dr + db * db;
                energy += d < 0.2 ? d : 0.2;
            }
            if (energy < best) {
                best = energy;
                best_t = ct;
                best_r = p->sensitivity_r / rr;
                best_b = p->sensitivity_b / bb;
            }
        }
    }
    v->next_red = v->red + 0.4 * (clamp(best_r, 0.5, 4) - v->red);
    v->next_blue = v->blue + 0.4 * (clamp(best_b, 0.5, 4) - v->blue);
    v->next_temperature = v->temperature + 0.4 * (best_t - v->temperature);
    return true;
}
static double shading_gain(uint32_t packed, unsigned channel)
{
    unsigned range=packed>>30, code=(packed>>(channel*10))&1023U;
    return range ? (double)code/(1024U>>range) : (double)(code+512)/1024;
}
static double stats_shading(const struct pisp_be_config *b, const struct pisp_fe_config *f,
    unsigned x, unsigned y, unsigned ch, const cc_sensor_geometry *g)
{
    if (!(b->global.bayer_enables & PISP_BE_BAYER_ENABLE_LSC)) return 1;
    unsigned width=g->width, height=g->height, ox=0, oy=0;
    if (f->global.enables&PISP_FE_ENABLE_STATS_CROP) {
        width=f->stats_crop.width; height=f->stats_crop.height;
        ox=f->stats_crop.offset_x; oy=f->stats_crop.offset_y;
    }
    unsigned sx=f->awb_stats.size_x, sy=f->awb_stats.size_y;
    if (!sx) sx=width>2U*f->awb_stats.offset_x ? ((width-2U*f->awb_stats.offset_x)/32)&~1U:0;
    if (!sy) sy=height>2U*f->awb_stats.offset_y ? ((height-2U*f->awb_stats.offset_y)/32)&~1U:0;
    double factor=f->global.enables&PISP_FE_ENABLE_DECIMATE ? 2:1;
    double u=clamp((ox+factor*(f->awb_stats.offset_x+((double)x+0.5)*sx))*b->lsc.grid_step_x/(1U<<18),0,32);
    double v=clamp((oy+factor*(f->awb_stats.offset_y+((double)y+0.5)*sy))*b->lsc.grid_step_y/(1U<<18),0,32);
    unsigned x0=(unsigned)u, y0=(unsigned)v, x1=x0<32 ? x0+1:x0, y1=y0<32 ? y0+1:y0;
    u-=x0; v-=y0;
    return (shading_gain(b->lsc.lut_packed[y0][x0],ch)*(1-u)+
            shading_gain(b->lsc.lut_packed[y0][x1],ch)*u)*(1-v)+
           (shading_gain(b->lsc.lut_packed[y1][x0],ch)*(1-u)+
            shading_gain(b->lsc.lut_packed[y1][x1],ch)*u)*v;
}
static int decode_statistics(const struct vc4_controller *v, const cc_ipa_frame *frame,
    struct photo_statistics *stats)
{
    memset(stats,0,sizeof(*stats));
    if (v->pisp) {
        struct pisp_statistics native;
        if (frame->statistics_size<sizeof(native) || !frame->backend || !frame->frontend) return -ENODATA;
        memcpy(&native,frame->statistics_data,sizeof(native));
        stats->zones=1024; stats->bins=1024; stats->scale=65536;
        memcpy(stats->histogram,native.agc.histogram,sizeof(stats->histogram));
        for (unsigned i=0; i<1024; ++i) {
            const struct pisp_awb_statistics_zone *z=&native.awb.zones[i];
            stats->awb_stats[i]=(struct bcm2835_isp_stats_region){.counted=z->counted,
                .r_sum=(uint64_t)(z->R_sum*stats_shading(frame->backend,frame->frontend,i%32,i/32,0,&v->config.geometry)),
                .g_sum=(uint64_t)(z->G_sum*stats_shading(frame->backend,frame->frontend,i%32,i/32,1,&v->config.geometry)),
                .b_sum=(uint64_t)(z->B_sum*stats_shading(frame->backend,frame->frontend,i%32,i/32,2,&v->config.geometry))};
        }
    } else {
        struct bcm2835_isp_stats native;
        if (frame->statistics_size<sizeof(native) || !frame->vc4) return -ENODATA;
        memcpy(&native,frame->statistics_data,sizeof(native));
        stats->zones=192; stats->bins=128; stats->scale=8192;
        memcpy(stats->histogram,native.hist[0].g_hist,sizeof(native.hist[0].g_hist));
        memcpy(stats->awb_stats,native.awb_stats,sizeof(native.awb_stats));
        memcpy(stats->agc_stats,native.agc_stats,sizeof(native.agc_stats));
        for (unsigned i=0; i<AGC_REGIONS; ++i) if (!region_valid(&stats->agc_stats[i])) return -EPROTO;
    }
    return 0;
}
static void sensor_statistics(struct vc4_controller *v, const cc_ipa_frame *frame, cc_ipa_metadata *m)
{
    v->have_pdaf=false;
    const cc_embedded_configuration *c=&v->config.embedded;
    const cc_sensor_properties *sensor=cc_sensor_get_properties(v->config.sensor);
    if (!frame->embedded_size || !c->format.fourcc) return;
    uint32_t stride=c->format.planes[0].stride, height=c->format.height;
    cc_smia_layout layout={c->bits_per_pixel,stride,height<2 ? height:2};
    if ((sensor->features & CC_SENSOR_EMBEDDED) &&
        !cc_sensor_parse_embedded_status(v->config.sensor,&layout,frame->embedded_data,frame->embedded_size,&m->sensor_status))
        m->flags|=CC_IPA_METADATA_SENSOR;
    const unsigned char *data=frame->embedded_data;
    size_t offset=(size_t)stride*2;
    if ((sensor->features & CC_SENSOR_PDAF) && height>=3 && offset<frame->embedded_size) {
        size_t size=frame->embedded_size-offset; if (size>stride) size=stride;
        if (!cc_sensor_parse_pdaf(v->config.sensor,c->bits_per_pixel,data+offset,size,&v->pdaf)) {
            v->have_pdaf=true; m->flags|=CC_IPA_METADATA_PDAF;
        }
    }
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *m)
{
    struct vc4_controller *v=context;
    const struct cc_vc4_profile_data *p=v->profile;
    cc_photo_controls photo=cc_photo_resolve(&v->options,&frame->photo);
    if (cc_photo_validate_controls(&photo)<0) return -EINVAL;
    const cc_control *e=find(frame->sensor.controls,frame->sensor.count,V4L2_CID_EXPOSURE);
    const cc_control *g=find(frame->sensor.controls,frame->sensor.count,V4L2_CID_ANALOGUE_GAIN);
    if (!e || !g || e->value<1 || e->value>UINT32_MAX) return -ENODATA;
    cc_sensor_timing timing=v->config.timing;
    const cc_control *vb=find(frame->sensor.controls,frame->sensor.count,V4L2_CID_VBLANK);
    const cc_control *hb=find(frame->sensor.controls,frame->sensor.count,V4L2_CID_HBLANK);
    if (vb) {
        if (vb->value<0 || vb->value>UINT32_MAX) return -EPROTO;
        timing.vblank=(uint32_t)vb->value;
    }
    if (hb) {
        if (hb->value<0 || hb->value>UINT32_MAX) return -EPROTO;
        timing.hblank=(uint32_t)hb->value;
    }
    uint64_t ns;
    int ret=cc_sensor_exposure_time(v->config.sensor,&timing,(uint32_t)e->value,&ns);
    if (!ret) ret=cc_sensor_frame_duration(&timing,&m->frame_duration_ns);
    double gain;
    if (!ret) ret=native_gain(v,g->value,&gain);
    if (ret<0) return ret;
    bool manual_exposure=photo.exposure_ns ||
        find(frame->request_sensor_controls,frame->num_request_sensor_controls,V4L2_CID_EXPOSURE);
    bool manual_gain=photo.analogue_gain>0 ||
        find(frame->request_sensor_controls,frame->num_request_sensor_controls,V4L2_CID_ANALOGUE_GAIN);
    const cc_control *expected_e=find(frame->settings_sensor_controls,frame->num_settings_sensor_controls,V4L2_CID_EXPOSURE);
    const cc_control *expected_g=find(frame->settings_sensor_controls,frame->num_settings_sensor_controls,V4L2_CID_ANALOGUE_GAIN);
    bool applied=expected_e && expected_g && e->value==expected_e->value && g->value==expected_g->value;
    double applied_red, applied_blue;
    const cc_control *red=find(frame->isp_controls,frame->num_isp_controls,V4L2_CID_RED_BALANCE);
    const cc_control *blue=find(frame->isp_controls,frame->num_isp_controls,V4L2_CID_BLUE_BALANCE);
    if (v->pisp) {
        if (!frame->backend || !frame->backend->wbg.gain_g) return -ENODATA;
        applied_red=(double)frame->backend->wbg.gain_r/frame->backend->wbg.gain_g;
        applied_blue=(double)frame->backend->wbg.gain_b/frame->backend->wbg.gain_g;
        m->colour_temperature=frame->colour_temperature;
    } else {
        if (!red || !blue || !frame->vc4) return -ENODATA;
        applied_red=(double)red->value/1000; applied_blue=(double)blue->value/1000;
        m->colour_temperature=frame->vc4->colour_temperature;
    }
    m->flags=CC_IPA_METADATA_PHOTO | CC_IPA_METADATA_AE | CC_IPA_METADATA_AWB |
        (v->pisp ? CC_IPA_METADATA_PISP:CC_IPA_METADATA_VC4);
    m->exposure_ns=ns; m->analogue_gain=gain; m->red_gain=applied_red; m->blue_gain=applied_blue;
    m->photo=photo; m->photo.exposure_ns=ns; m->photo.analogue_gain=gain;
    m->photo.red_gain=applied_red; m->photo.blue_gain=applied_blue;
    m->photo_status=CC_PHOTO_EXPECTED_SENSOR;
    if (!manual_exposure) m->photo_status|=CC_PHOTO_AUTO_EXPOSURE;
    if (!manual_gain) m->photo_status|=CC_PHOTO_AUTO_GAIN;
    if (photo.white_balance!=CC_PHOTO_WB_MANUAL) m->photo_status|=CC_PHOTO_AUTO_WB;
    if (frame->photo_overrides) {
        m->photo_status|=CC_PHOTO_NATIVE_OVERRIDE;
        /* Sensor values retain their units after native overrides. WB gains
         * remain in red_gain/blue_gain, but a partial native WB override cannot
         * be described by the normalized automatic/manual policy. */
        m->photo.flags &= ~(frame->photo_overrides & ~(uint32_t)(CC_PHOTO_EXPOSURE|CC_PHOTO_GAIN));
        if (frame->photo_overrides & CC_PHOTO_WB) m->photo_status &= ~(uint32_t)CC_PHOTO_AUTO_WB;
    }
    uint64_t quantum=0;
    ret=cc_sensor_exposure_time(v->config.sensor,&timing,v->config.timing_limits.exposure_step,&quantum);
    if (ret<0) return ret;
    if ((photo.exposure_ns && ns<photo.exposure_ns && photo.exposure_ns-ns>quantum) ||
        (photo.min_frame_duration_ns && m->frame_duration_ns+quantum<photo.min_frame_duration_ns))
        m->photo_status|=CC_PHOTO_LIMITED;
    m->lux=0; /* No stale scene estimate on an untrusted frame. */
    if (v->af) {
        m->flags|=CC_IPA_METADATA_AF; m->af=frame->af;
        if (frame->af.flags & CC_AF_LENS_VALID) {
            ret=cc_af_lens_position(v->af,frame->af.lens_setting,&m->af.lens_position);
            if (ret<0) return ret;
        }
    }
    /* Applied values are valid during startup too, but no convergence or
     * feedback may be inferred from untrusted statistics. */
    if (frame->untrusted) { v->have_pdaf=false; return 0; }
    if (frame->statistics_fourcc!=v->config.statistics_fourcc) return -ENODATA;
    cc_ipa_statistics parsed;
    ret=cc_ipa_parse_statistics(frame->statistics_fourcc,frame->statistics_data,frame->statistics_size,&parsed);
    if (ret<0) return ret;
    struct photo_statistics stats;
    ret=decode_statistics(v,frame,&stats);
    if (ret<0) return ret;
    m->photo_status|=CC_PHOTO_STATS_TRUSTED;
    sensor_statistics(v,frame,m);
    if (v->af) {
        ret=cc_af_process_native(v->af,frame->statistics_fourcc,frame->statistics_data,frame->statistics_size);
        if (ret<0) return ret;
        v->af_clock_valid=frame->timestamp_ns>v->af_timestamp && v->af_timestamp!=0;
        v->af_timestamp=frame->timestamp_ns; ++v->af_frame_count;
    }
    double luma=0, count=0;
    const double *weights=p->metering[photo.metering];
    for (unsigned i=0; !v->pisp && i<15; ++i) {
        const struct bcm2835_isp_stats_region *z=&stats.agc_stats[i];
        luma+=weights[i]*(0.299*(double)z->r_sum*applied_red+0.587*(double)z->g_sum+
            0.114*(double)z->b_sum*applied_blue);
        count+=(double)z->counted*weights[i];
    }
    luma=count>0 ? clamp(luma/count/8192,0,1):parsed.luminance;
    v->lux=clamp(parsed.luminance*65536/p->lux_y*p->lux_shutter/(double)ns*p->lux_gain/gain*p->lux_reference,0,1e9);
    double ev=cc_photo_ev_multiplier(photo.exposure_value);
    double target=clamp(pwl(p->target,p->num_target,v->lux)*p->base_ev*ev,0,1);
    double ratio=luma>0 ? target/luma:2;
    uint64_t total=0;
    for (unsigned i=0; i<stats.bins; ++i) total+=stats.histogram[i];
    if (total) {
        double needed=(double)total*0.02, used=0, weighted=0;
        for (int i=(int)stats.bins-1; i>=0 && used<needed; --i) {
            double n=stats.histogram[i]; if (n>needed-used) n=needed-used;
            weighted+=n*((double)i+0.5)/stats.bins; used+=n;
        }
        double constraint=clamp(p->constraint_target*ev,0,1);
        if (weighted>0 && ratio<constraint*used/weighted) ratio=constraint*used/weighted;
    }
    bool have_luma=(parsed.flags & CC_IPA_STATS_LUMA) || count>0;
    if (applied && manual_exposure && manual_gain) m->flags|=CC_IPA_AE_CONVERGED;
    if (have_luma) {
        bool limited=applied && !(manual_exposure && manual_gain) &&
            ((ratio>1.05 && v->exposure_product>(double)ns*gain*1.05) ||
             (ratio<0.95 && v->exposure_product<(double)ns*gain*0.95));
        double proposed=v->exposure_product+(absolute(ratio-1)<0.025 ? 0:
            (double)ns*gain*p->speed*(clamp(ratio,0.5,2)-1));
        double maximum=p->shutter[p->num_exposure-1]*1000*p->gain[p->num_exposure-1];
        if (!(manual_exposure && manual_gain))
            v->exposure_product=clamp(proposed,p->shutter[0]*1000*p->gain[0],maximum);
        m->luminance=luma;
        if (applied && ((manual_exposure && manual_gain) || absolute(ratio-1)<0.05)) m->flags|=CC_IPA_AE_CONVERGED;
        if (limited || v->exposure_product>=maximum || (gain>=v->capabilities.automatic_gain.maximum && ratio>1.05)) {
            m->flags|=CC_IPA_EXPOSURE_LIMIT; m->photo_status|=CC_PHOTO_LIMITED;
        }
    }
    v->red=applied_red; v->blue=applied_blue; v->temperature=m->colour_temperature;
    bool manual_wb=photo.white_balance==CC_PHOTO_WB_MANUAL;
    bool wb_valid=manual_wb || awb(v,&stats,photo.white_balance);
    double tolerance=v->pisp ? 0.002:(double)(v->config.red_balance.step>v->config.blue_balance.step ?
        v->config.red_balance.step:v->config.blue_balance.step)/1000+0.001;
    bool manual_applied=absolute(photo.red_gain-applied_red)<=tolerance && absolute(photo.blue_gain-applied_blue)<=tolerance;
    if (!(frame->photo_overrides & CC_PHOTO_WB) && wb_valid && ((manual_wb && manual_applied) || (!manual_wb && absolute(v->next_red-applied_red)<0.02*applied_red &&
        absolute(v->next_blue-applied_blue)<0.02*applied_blue))) m->flags|=CC_IPA_AWB_CONVERGED;
    m->lux=v->lux;
    return 0;
}
static int create(const cc_ipa_vc4_options *options, bool pisp, cc_ipa **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    cc_ipa_vc4_options o = options ? *options : (cc_ipa_vc4_options){.mains_hz = 50};
    if ((o.profile != CC_VC4_PROFILE_AUTO &&
         !(pisp ? cc_pisp_get_profile_info(o.profile) : cc_vc4_get_profile_info(o.profile))) ||
        !cc_rpi_lens_shading_valid(&o.lens_shading) ||
        (o.autofocus>1) ||
        (o.mains_hz && o.mains_hz != 50 && o.mains_hz != 60) ||
        !((o.red_gain == 0 && o.blue_gain == 0) ||
          (o.red_gain >= 0.5 && o.red_gain <= 4 && o.blue_gain >= 0.5 && o.blue_gain <= 4)))
        return -EINVAL;
    struct vc4_controller *v = calloc(1, sizeof(*v));
    if (!v)
        return -ENOMEM;
    v->options = o;
    v->pisp = pisp;
    const cc_ipa_ops ops = {CC_IPA_ABI_VERSION,
                            sizeof(ops),
                            configure,
                            start,
                            stop,
                            prepare,
                            process,
                            destroy,
                            CC_IPA_CAP_SENSOR_TIMING | CC_IPA_CAP_SENSOR_GEOMETRY | CC_IPA_CAP_PHOTO |
                                (o.autofocus ? CC_IPA_CAP_LENS:0), photo_capabilities};
    int ret = cc_ipa_create(&ops, v, out);
    if (ret < 0)
        free(v);
    return ret;
}
int cc_ipa_vc4_create(const cc_ipa_vc4_options *options, cc_ipa **out) { return create(options,false,out); }
int cc_ipa_pisp_create(const cc_ipa_pisp_options *options, cc_ipa **out) { return create(options,true,out); }
