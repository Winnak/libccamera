/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Single-exposure VC4/PiSP controller. Calibration, capped Bayesian colour-error
 * likelihood and lux priors derived from Raspberry Pi Ltd libcamera 0.7.2.
 * Search is synchronous and bounded; adaptive ALSC/contrast/HDR are excluded. */
#include "vc4_profile.h"
#include "pisp_format.h"
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
                         V4L2_CTRL_FLAG_HAS_PAYLOAD));
}
static int native_gain(const struct vc4_controller *v, int64_t code, double *gain)
{
    if (code < v->gain.minimum || code > v->gain.maximum ||
        (uint64_t)(code - v->gain.minimum) % v->gain.step || code > UINT32_MAX)
        return -ERANGE;
    return cc_sensor_gain(v->config.sensor, (uint32_t)code, gain);
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
    const cc_sensor_control_delay *ed = NULL, *vd = NULL;
    for (uint32_t i = 0; i < c->num_sensor_controls; ++i) {
        if (c->sensor_delays[i].id == V4L2_CID_EXPOSURE)
            ed = &c->sensor_delays[i];
        if (c->sensor_delays[i].id == V4L2_CID_VBLANK)
            vd = &c->sensor_delays[i];
    }
    if (!ed || !vd || vd->frames < ed->frames || !(vd->flags & CC_SENSOR_CONTROL_PRIORITY))
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
        ret = cc_pisp_build_tuning(selected, &c->geometry, 4500, 1, 1, 1, &fe, &be);
    } else {
        cc_vc4_tuning test;
        ret = cc_vc4_build_tuning(selected, &c->geometry, 4500, 1, &test);
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
    return 0;
}
static int start(void *p)
{
    (void)p;
    return 0;
}
static void stop(void *p) { (void)p; }
static void destroy(void *p) { free(p); }
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
    uint64_t wanted = (uint64_t)shutter;
    if (manual_e) {
        if (manual_e->value < limits.min_exposure || manual_e->value > UINT32_MAX ||
            ((uint64_t)manual_e->value - limits.min_exposure) % limits.exposure_step)
            return -ERANGE;
        int ret = cc_sensor_exposure_time(v->config.sensor, &timing, (uint32_t)manual_e->value, &wanted);
        if (ret < 0)
            return ret;
    } else {
        int ret = cc_sensor_flicker_exposure(v->config.sensor, &timing, wanted, v->options.mains_hz, &wanted);
        if (ret < 0)
            return ret;
    }
    cc_sensor_exposure_plan plan;
    int ret = cc_sensor_plan_exposure(v->config.sensor, &limits, wanted, 0, 0, &plan);
    if (ret < 0)
        return ret;
    if (manual_e && plan.exposure_lines != (uint32_t)manual_e->value)
        return -ERANGE;
    if (!manual_g)
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
    s->colour_temperature = v->temperature;
    if (v->pisp) {
        ret = cc_pisp_build_tuning(p->info.profile, &v->config.geometry, v->temperature, actual_gain,
            v->red, v->blue, &s->frontend, &s->backend);
        if (!ret) s->flags = CC_IPA_FRONTEND | CC_IPA_BACKEND;
        return ret;
    }
    ret = cc_vc4_build_tuning(p->info.profile, &v->config.geometry, v->temperature, actual_gain, &s->vc4);
    if (ret < 0)
        return ret;
    s->flags = CC_IPA_VC4_TUNING;
    double minimum = v->red < v->blue ? v->red : v->blue;
    if (minimum > 1)
        minimum = 1;
    ret = wb_control(&v->config.red_balance, v->red, &s->isp_controls[0]);
    if (!ret)
        ret = wb_control(&v->config.blue_balance, v->blue, &s->isp_controls[1]);
    if (!ret)
        ret = wb_control(&v->config.digital_gain, 1 / minimum, &s->isp_controls[2]);
    s->num_isp_controls = 3;
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
static bool awb(struct vc4_controller *v, const struct photo_statistics *stats)
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
    for (unsigned step = 0; step <= 110; ++step) {
        double ct = 2500 + 50.0 * step, r, b;
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
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *m)
{
    struct vc4_controller *v = context;
    const struct cc_vc4_profile_data *p = v->profile;
    if (frame->untrusted)
        return 0;
    if (frame->statistics_fourcc != v->config.statistics_fourcc)
        return -ENODATA;
    cc_ipa_statistics parsed;
    int ret = cc_ipa_parse_statistics(frame->statistics_fourcc, frame->statistics_data, frame->statistics_size, &parsed);
    if (ret < 0)
        return ret;
    struct photo_statistics stats;
    ret=decode_statistics(v,frame,&stats);
    if (ret<0) return ret;
    const cc_control *e = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_EXPOSURE);
    const cc_control *g = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_ANALOGUE_GAIN);
    if (!e || !g || e->value < 1 || e->value > UINT32_MAX)
        return -ENODATA;
    cc_sensor_timing timing = v->config.timing;
    const cc_control *vb = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_VBLANK);
    const cc_control *hb = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_HBLANK);
    if (vb) {
        if (vb->value < 0 || vb->value > UINT32_MAX)
            return -EPROTO;
        timing.vblank = (uint32_t)vb->value;
    }
    if (hb) {
        if (hb->value < 0 || hb->value > UINT32_MAX)
            return -EPROTO;
        timing.hblank = (uint32_t)hb->value;
    }
    uint64_t ns;
    ret = cc_sensor_exposure_time(v->config.sensor, &timing, (uint32_t)e->value, &ns);
    if (ret < 0)
        return ret;
    ret = cc_sensor_frame_duration(&timing, &m->frame_duration_ns);
    if (ret < 0)
        return ret;
    bool manual_exposure =
        find(frame->request_sensor_controls, frame->num_request_sensor_controls, V4L2_CID_EXPOSURE) != NULL;
    bool manual_gain = find(frame->request_sensor_controls, frame->num_request_sensor_controls,
                            V4L2_CID_ANALOGUE_GAIN) != NULL;
    const cc_control *expected_e =
        find(frame->settings_sensor_controls, frame->num_settings_sensor_controls, V4L2_CID_EXPOSURE);
    const cc_control *expected_g =
        find(frame->settings_sensor_controls, frame->num_settings_sensor_controls, V4L2_CID_ANALOGUE_GAIN);
    double gain, applied_red = v->red, applied_blue = v->blue;
    ret = native_gain(v, g->value, &gain);
    if (ret < 0)
        return ret;
    const cc_control *red = find(frame->isp_controls, frame->num_isp_controls, V4L2_CID_RED_BALANCE);
    const cc_control *blue = find(frame->isp_controls, frame->num_isp_controls, V4L2_CID_BLUE_BALANCE);
    if (red)
        applied_red = (double)red->value / 1000;
    if (blue)
        applied_blue = (double)blue->value / 1000;
    if (v->pisp) {
        if (!frame->backend->wbg.gain_g || !(frame->colour_temperature>=2000 && frame->colour_temperature<=13000))
            return -ENODATA;
        applied_red=(double)frame->backend->wbg.gain_r/frame->backend->wbg.gain_g;
        applied_blue=(double)frame->backend->wbg.gain_b/frame->backend->wbg.gain_g;
    }
    double luma = 0, count = 0;
    for (unsigned i = 0; !v->pisp && i < 15; ++i) {
        const struct bcm2835_isp_stats_region *z = &stats.agc_stats[i];
        luma += p->weights[i] * (0.299 * (double)z->r_sum * applied_red + 0.587 * (double)z->g_sum +
                              0.114 * (double)z->b_sum * applied_blue);
        count += (double)z->counted * p->weights[i];
    }
    if (count > 0)
        luma = clamp(luma / count / 8192, 0, 1);
    else
        luma = parsed.luminance;
    v->lux = clamp(parsed.luminance * 65536 / p->lux_y * p->lux_shutter / (double)ns *
                   p->lux_gain / gain * p->lux_reference, 0, 1e9);
    double target = pwl(p->target, p->num_target, v->lux) * p->base_ev;
    double ratio = luma > 0 ? target / luma : 2;
    /* Profile's normal lower constraint on the brightest 2% of the histogram. */
    uint64_t total = 0;
    for (unsigned i = 0; i < stats.bins; ++i)
        total += stats.histogram[i];
    if (total) {
        double needed = (double)total * 0.02, used = 0, weighted = 0;
        for (int i = (int)stats.bins-1; i >= 0 && used < needed; --i) {
            double n = stats.histogram[i];
            if (n > needed - used)
                n = needed - used;
            weighted += n * ((double)i + 0.5) / stats.bins;
            used += n;
        }
        if (weighted > 0 && ratio < p->constraint_target * used / weighted)
            ratio = p->constraint_target * used / weighted;
    }
    bool have_luma = (parsed.flags & CC_IPA_STATS_LUMA) != 0;
    if (have_luma) {
        /* Retain sub-step corrections across frames. Rebuilding from the
         * quantized applied product each time can stall below the next native
         * gain step, preventing convergence even in a constant scene. */
        double proposed =
            v->exposure_product +
            (absolute(ratio - 1) < 0.025 ? 0 : (double)ns * gain * p->speed * (clamp(ratio, 0.5, 2) - 1));
        double maximum = p->shutter[p->num_exposure - 1] * 1000 * p->gain[p->num_exposure - 1];
        if (!(manual_exposure && manual_gain))
            v->exposure_product = clamp(proposed, p->shutter[0] * 1000 * p->gain[0], maximum);
        m->flags |= CC_IPA_METADATA_AE;
        m->exposure_ns = ns;
        m->analogue_gain = gain;
        m->luminance = luma;
        if ((manual_exposure && manual_gain) ||
            (absolute(ratio - 1) < 0.05 && expected_e && expected_g && e->value == expected_e->value &&
             g->value == expected_g->value))
            m->flags |= CC_IPA_AE_CONVERGED;
        if (v->exposure_product >= maximum || (gain >= p->gain[p->num_exposure - 1] && ratio > 1.05))
            m->flags |= CC_IPA_EXPOSURE_LIMIT;
    }
    v->red = applied_red;
    v->blue = applied_blue;
    v->temperature = v->pisp ? frame->colour_temperature : frame->vc4->colour_temperature;
    bool wb_valid = v->options.red_gain > 0 ? true : awb(v, &stats);
    if (wb_valid) {
        m->flags |= CC_IPA_METADATA_AWB;
        m->red_gain = applied_red;
        m->blue_gain = applied_blue;
        if (v->options.red_gain > 0 || (absolute(v->next_red - applied_red) < 0.02 * applied_red &&
                                        absolute(v->next_blue - applied_blue) < 0.02 * applied_blue))
            m->flags |= CC_IPA_AWB_CONVERGED;
    }
    m->flags |= v->pisp ? CC_IPA_METADATA_PISP : CC_IPA_METADATA_VC4;
    m->colour_temperature = v->temperature;
    m->lux = v->lux;
    return 0;
}
static int create(const cc_ipa_vc4_options *options, bool pisp, cc_ipa **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    cc_ipa_vc4_options o = options ? *options : (cc_ipa_vc4_options){.mains_hz = 50};
    if ((o.profile != CC_VC4_PROFILE_AUTO && !cc_vc4_get_profile_info(o.profile)) ||
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
                            CC_IPA_CAP_SENSOR_TIMING | CC_IPA_CAP_SENSOR_GEOMETRY};
    int ret = cc_ipa_create(&ops, v, out);
    if (ret < 0)
        free(v);
    return ret;
}
int cc_ipa_vc4_create(const cc_ipa_vc4_options *options, cc_ipa **out) { return create(options,false,out); }
int cc_ipa_pisp_create(const cc_ipa_pisp_options *options, cc_ipa **out) { return create(options,true,out); }
