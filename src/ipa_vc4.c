/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Single-exposure VC4 controller. Calibration, capped Bayesian colour-error
 * likelihood and lux priors derived from Raspberry Pi Ltd libcamera 0.7.2.
 * Search is synchronous and bounded; adaptive ALSC/contrast/HDR are excluded. */
#include "ov5647_profile.h"
#include <ccamera/ipa.h>
#include <errno.h>
#include <float.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
struct vc4_controller {
    cc_ipa_configuration config;
    cc_ipa_vc4_options options;
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
static void ct_values(double t, double *r, double *b)
{
    unsigned i = 0;
    while (i < 5 && t > ov_ct[i + 1][0])
        ++i;
    double f = clamp((t - ov_ct[i][0]) / (ov_ct[i + 1][0] - ov_ct[i][0]), 0, 1);
    *r = ov_ct[i][1] * (1 - f) + ov_ct[i + 1][1] * f;
    *b = ov_ct[i][2] * (1 - f) + ov_ct[i + 1][2] * f;
}
static double prior(double lux, double ct)
{
    double values[] = {pwl(ov_prior_0, sizeof(ov_prior_0) / sizeof(double), ct),
                       pwl(ov_prior_1, sizeof(ov_prior_1) / sizeof(double), ct),
                       pwl(ov_prior_2, sizeof(ov_prior_2) / sizeof(double), ct)};
    unsigned i = lux > ov_prior_lux[1] ? 1 : 0;
    return values[i] + (values[i + 1] - values[i]) *
                           clamp((lux - ov_prior_lux[i]) / (ov_prior_lux[i + 1] - ov_prior_lux[i]), 0, 1);
}
static int range_valid(const cc_control_info *r)
{
    return r->type == V4L2_CTRL_TYPE_INTEGER && r->minimum >= 0 && r->maximum >= r->minimum &&
           r->maximum <= INT32_MAX && r->step && r->elems <= 1 &&
           !(r->flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE |
                         V4L2_CTRL_FLAG_HAS_PAYLOAD));
}
static int configure(void *context, const cc_ipa_configuration *c)
{
    struct vc4_controller *v = context;
    if (c->pipeline != CC_IPA_VC4 || c->statistics_fourcc != CC_FOURCC('B', 'S', 'T', 'A') || !c->sensor ||
        strcmp(cc_sensor_get_properties(c->sensor)->model, "ov5647"))
        return -ENOTSUP;
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
        g->value < 16 || g->value > 1023 || !c->timing.pixel_rate || !c->timing_limits.pixel_rate ||
        !range_valid(&c->red_balance) || !range_valid(&c->blue_balance) || !range_valid(&c->digital_gain))
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
    cc_vc4_tuning test;
    int ret = cc_vc4_ov5647_build_tuning(&c->geometry, 4500, 1, &test);
    if (ret < 0)
        return ret;
    uint64_t ns;
    ret = cc_sensor_exposure_time(c->sensor, &c->timing, (uint32_t)e->value, &ns);
    if (ret < 0)
        return ret;
    v->config = *c;
    v->exposure_product = (double)ns * (double)g->value / 16;
    v->temperature = v->next_temperature = 4500;
    double r, b;
    ct_values(4500, &r, &b);
    v->red = v->next_red = v->options.red_gain > 0 ? v->options.red_gain : 1.05 / r;
    v->blue = v->next_blue = v->options.blue_gain > 0 ? v->options.blue_gain : 1.05 / b;
    if (v->options.red_gain > 0) {
        double best = DBL_MAX;
        for (unsigned i = 0; i <= 110; ++i) {
            double ct = 2500 + 50.0 * i;
            ct_values(ct, &r, &b);
            double dr = 1.05 / r - v->red, db = 1.05 / b - v->blue, error = dr * dr + db * db;
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
static void choose_exposure(double product, double *shutter, double *gain)
{
    *shutter = ov_shutter[0] * 1000;
    *gain = ov_gain[0];
    for (unsigned i = 1; i < 5; ++i) {
        *shutter = clamp(product / *gain, *shutter, ov_shutter[i] * 1000);
        if (*shutter * *gain >= product)
            return;
        *gain = clamp(product / *shutter, *gain, ov_gain[i]);
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
    choose_exposure(v->exposure_product, &shutter, &gain);
    if (manual_g) {
        if (manual_g->value < 0 || manual_g->value > UINT32_MAX)
            return -ERANGE;
        int ret = cc_sensor_gain(v->config.sensor, (uint32_t)manual_g->value, &gain);
        if (ret < 0)
            return ret;
        shutter = clamp(v->exposure_product / gain, 100000, 66666000);
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
        gain = clamp(v->exposure_product / (double)plan.exposure_ns, 1, 8);
    cc_control control;
    double actual_gain;
    ret = cc_sensor_quantize_gain(v->config.sensor, &v->gain, gain, &control, &actual_gain);
    if (ret < 0)
        return ret;
    if (manual_g) {
        control = *manual_g;
        ret = cc_sensor_gain(v->config.sensor, (uint32_t)manual_g->value, &actual_gain);
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
    ret = cc_vc4_ov5647_build_tuning(&v->config.geometry, v->temperature, actual_gain, &s->vc4);
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
static bool awb(struct vc4_controller *v, const struct bcm2835_isp_stats *stats)
{
    double zr[192], zb[192];
    unsigned count = 0;
    for (unsigned i = 0; i < 192; ++i) {
        const struct bcm2835_isp_stats_region *z = &stats->awb_stats[i];
        if (!z->counted || (uint64_t)z->counted * 10 < (uint64_t)z->counted + z->notcounted ||
            z->g_sum < (uint64_t)z->counted * 32)
            continue;
        zr[count] = (double)z->r_sum * 1.05 / ((double)z->g_sum + 0.125);
        zb[count] = (double)z->b_sum * 1.05 / ((double)z->g_sum + 0.125);
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
        ct_values(ct, &r, &b);
        double r0, b0, r1, b1;
        ct_values(ct - 25, &r0, &b0);
        ct_values(ct + 25, &r1, &b1);
        double tr = b1 - b0, tb = r0 - r1, root = tr * tr + tb * tb;
        if (root > 0) {
            double x = root < 1 ? 1 : root;
            for (unsigned j = 0; j < 24; ++j)
                x = 0.5 * (x + root / x);
            tr /= x;
            tb /= x;
        }
        for (unsigned k = 0; k <= 8; ++k) {
            double delta = -0.04313 + (0.04313 + 0.0321) * (double)k / 8;
            double rr = r + delta * tr, bb = b + delta * tb;
            double energy = -prior(v->lux, ct) * (double)count / 192;
            for (unsigned z = 0; z < count; ++z) {
                double dr = zr[z] / rr - 1, db = zb[z] / bb - 1, d = dr * dr + db * db;
                energy += d < 0.2 ? d : 0.2;
            }
            if (energy < best) {
                best = energy;
                best_t = ct;
                best_r = 1.05 / rr;
                best_b = 1.05 / bb;
            }
        }
    }
    v->next_red = v->red + 0.4 * (clamp(best_r, 0.5, 4) - v->red);
    v->next_blue = v->blue + 0.4 * (clamp(best_b, 0.5, 4) - v->blue);
    v->next_temperature = v->temperature + 0.4 * (best_t - v->temperature);
    return true;
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *m)
{
    struct vc4_controller *v = context;
    if (frame->untrusted)
        return 0;
    if (frame->statistics_fourcc != CC_FOURCC('B', 'S', 'T', 'A') ||
        frame->statistics_size < sizeof(struct bcm2835_isp_stats) || !frame->vc4)
        return -ENODATA;
    struct bcm2835_isp_stats stats;
    memcpy(&stats, frame->statistics_data, sizeof(stats));
    cc_ipa_statistics parsed;
    int ret = cc_ipa_parse_statistics(frame->statistics_fourcc, &stats, sizeof(stats), &parsed);
    if (ret < 0)
        return ret;
    for (unsigned i = 0; i < AGC_REGIONS; ++i)
        if (!region_valid(&stats.agc_stats[i]))
            return -EPROTO;
    const cc_control *e = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_EXPOSURE);
    const cc_control *g = find(frame->sensor.controls, frame->sensor.count, V4L2_CID_ANALOGUE_GAIN);
    if (!e || !g || e->value < 1 || e->value > UINT32_MAX || g->value < 16 || g->value > 1023)
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
    double gain = (double)g->value / 16, applied_red = v->red, applied_blue = v->blue;
    const cc_control *red = find(frame->isp_controls, frame->num_isp_controls, V4L2_CID_RED_BALANCE);
    const cc_control *blue = find(frame->isp_controls, frame->num_isp_controls, V4L2_CID_BLUE_BALANCE);
    if (red)
        applied_red = (double)red->value / 1000;
    if (blue)
        applied_blue = (double)blue->value / 1000;
    double luma = 0, count = 0;
    const unsigned weights[15] = {3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0};
    for (unsigned i = 0; i < 15; ++i) {
        const struct bcm2835_isp_stats_region *z = &stats.agc_stats[i];
        luma += weights[i] * (0.299 * (double)z->r_sum * applied_red + 0.587 * (double)z->g_sum +
                              0.114 * (double)z->b_sum * applied_blue);
        count += (double)z->counted * weights[i];
    }
    if (count > 0)
        luma = clamp(luma / count / 8192, 0, 1);
    else
        luma = parsed.luminance;
    v->lux = clamp(parsed.luminance * 65536 / 8961 * 21663000 / (double)ns / gain * 987, 0, 1e9);
    double target = (0.16 + 0.01 * clamp(v->lux / 10000, 0, 1)) * 1.25;
    double ratio = luma > 0 ? target / luma : 2;
    /* Normal-mode lower histogram constraint: top 2% should reach 0.5. */
    uint64_t total = 0;
    for (unsigned i = 0; i < 128; ++i)
        total += stats.hist[0].g_hist[i];
    if (total) {
        double needed = (double)total * 0.02, used = 0, weighted = 0;
        for (int i = 127; i >= 0 && used < needed; --i) {
            double n = stats.hist[0].g_hist[i];
            if (n > needed - used)
                n = needed - used;
            weighted += n * ((double)i + 0.5) / 128;
            used += n;
        }
        if (weighted > 0 && ratio < 0.5 * used / weighted)
            ratio = 0.5 * used / weighted;
    }
    bool have_luma = (parsed.flags & CC_IPA_STATS_LUMA) != 0;
    if (have_luma) {
        /* Retain sub-step corrections across frames. Rebuilding from the
         * quantized applied product each time can stall below the next native
         * gain step, preventing convergence even in a constant scene. */
        double proposed =
            v->exposure_product +
            (absolute(ratio - 1) < 0.025 ? 0 : (double)ns * gain * 0.3 * (clamp(ratio, 0.5, 2) - 1));
        if (!(manual_exposure && manual_gain))
            v->exposure_product = clamp(proposed, 100000, 66666000.0 * 8);
        m->flags |= CC_IPA_METADATA_AE;
        m->exposure_ns = ns;
        m->analogue_gain = gain;
        m->luminance = luma;
        if ((manual_exposure && manual_gain) ||
            (absolute(ratio - 1) < 0.05 && expected_e && expected_g && e->value == expected_e->value &&
             g->value == expected_g->value))
            m->flags |= CC_IPA_AE_CONVERGED;
        if (v->exposure_product >= 66666000.0 * 8 || (gain >= 8 && ratio > 1.05))
            m->flags |= CC_IPA_EXPOSURE_LIMIT;
    }
    v->red = applied_red;
    v->blue = applied_blue;
    v->temperature = frame->vc4->colour_temperature;
    bool wb_valid = v->options.red_gain > 0 ? true : awb(v, &stats);
    if (wb_valid) {
        m->flags |= CC_IPA_METADATA_AWB;
        m->red_gain = applied_red;
        m->blue_gain = applied_blue;
        if (v->options.red_gain > 0 || (absolute(v->next_red - applied_red) < 0.02 * applied_red &&
                                        absolute(v->next_blue - applied_blue) < 0.02 * applied_blue))
            m->flags |= CC_IPA_AWB_CONVERGED;
    }
    m->flags |= CC_IPA_METADATA_VC4;
    m->colour_temperature = frame->vc4->colour_temperature;
    m->lux = v->lux;
    return 0;
}
int cc_ipa_vc4_create(const cc_ipa_vc4_options *options, cc_ipa **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    cc_ipa_vc4_options o = options ? *options : (cc_ipa_vc4_options){.mains_hz = 50};
    if ((o.mains_hz && o.mains_hz != 50 && o.mains_hz != 60) ||
        !((o.red_gain == 0 && o.blue_gain == 0) ||
          (o.red_gain >= 0.5 && o.red_gain <= 4 && o.blue_gain >= 0.5 && o.blue_gain <= 4)))
        return -EINVAL;
    struct vc4_controller *v = calloc(1, sizeof(*v));
    if (!v)
        return -ENOMEM;
    v->options = o;
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
