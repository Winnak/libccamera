/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* VC4 mappings/calibration adapted from Raspberry Pi Ltd's libcamera 0.7.2.
 * See the calibration JSON in data/ and LICENSES/BSD-2-Clause.txt. */
#include "integer_math.h"
#include "internal.h"
#include "vc4_profile.h"
#include "subdev_compat.h"
#include <ccamera/vc4_tuning.h>
#include <errno.h>
#include <float.h>
#include <string.h>

static double clamp(double x, double a, double b) { return x < a ? a : x > b ? b : x; }
static int finite_value(double x) { return x >= -DBL_MAX && x <= DBL_MAX; }
int cc_sensor_geometry_valid(const cc_sensor_geometry *g)
{
    return g && g->width && g->height && g->width <= 16384 && g->height <= 16384 && g->active.width &&
           g->active.height && g->active.x >= 0 && g->active.y >= 0 && g->crop.width && g->crop.height &&
           g->crop.x >= g->active.x && g->crop.y >= g->active.y &&
           (uint64_t)(uint32_t)g->crop.x + g->crop.width <=
               (uint64_t)(uint32_t)g->active.x + g->active.width &&
           (uint64_t)(uint32_t)g->crop.y + g->crop.height <=
               (uint64_t)(uint32_t)g->active.y + g->active.height &&
           g->hflip <= 1 && g->vflip <= 1;
}
/* Keep stream layout compatible with older installed Linux headers. */
struct selection {
    uint32_t which, pad, target, flags;
    struct v4l2_rect r;
    uint32_t stream, reserved[7];
};
_Static_assert(sizeof(struct selection) == 64, "Linux subdevice selection ABI");
int cc_camera_get_sensor_geometry(cc_camera *camera, cc_sensor_geometry *out)
{
    if (!camera || !out)
        return -EINVAL;
    if (camera->sensor_fd < 0)
        return -ENOTSUP;
    cc_sensor_geometry g = {0};
    struct cc_subdev_format fmt = {
        .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = camera->sensor_pad, .stream = camera->sensor_stream};
    int ret = cc_ioctl(&camera->io, camera->sensor_fd, VIDIOC_SUBDEV_G_FMT, &fmt);
    if (ret < 0)
        return ret;
    g.width = fmt.format.width;
    g.height = fmt.format.height;
    cc_rectangle *rects[] = {&g.active, &g.crop};
    const uint32_t targets[] = {V4L2_SEL_TGT_CROP_DEFAULT, V4L2_SEL_TGT_CROP};
    for (size_t i = 0; i < 2; ++i) {
        struct selection s = {.which = V4L2_SUBDEV_FORMAT_ACTIVE,
                              .pad = camera->sensor_pad,
                              .target = targets[i],
                              .stream = camera->sensor_stream};
        ret = cc_ioctl(&camera->io, camera->sensor_fd, _IOWR('V', 61, struct selection), &s);
        if (ret < 0)
            return ret;
        if (s.r.width <= 0 || s.r.height <= 0)
            return -EPROTO;
        *rects[i] = (cc_rectangle){s.r.left, s.r.top, (uint32_t)s.r.width, (uint32_t)s.r.height};
    }
    cc_control flips[] = {{.id = V4L2_CID_HFLIP}, {.id = V4L2_CID_VFLIP}};
    ret = cc_camera_get_controls(camera, flips, 2, NULL);
    if (ret < 0)
        return ret;
    if (flips[0].value < 0 || flips[0].value > 1 || flips[1].value < 0 || flips[1].value > 1)
        return -EPROTO;
    g.hflip = (uint32_t)flips[0].value;
    g.vflip = (uint32_t)flips[1].value;
    if (!cc_sensor_geometry_valid(&g))
        return -EPROTO;
    *out = g;
    return 0;
}
static double pwl(const double *v, size_t count, double x)
{
    if (x <= v[0])
        return v[1];
    for (size_t i = 2; i < count; i += 2)
        if (x <= v[i])
            return v[i - 1] + (v[i + 1] - v[i - 1]) * (x - v[i - 2]) / (v[i] - v[i - 2]);
    return v[count - 1];
}
double cc_rpi_sample_grid(const double *v, unsigned width, unsigned height, double x, double y)
{
    x = clamp(x, 0, width - 1);
    y = clamp(y, 0, height - 1);
    unsigned a = (unsigned)x, b = (unsigned)y, c = a + 1 < width ? a + 1 : a,
             d = b + 1 < height ? b + 1 : b;
    double u = x - a, w = y - b;
    return (v[b * width + a] * (1 - u) + v[b * width + c] * u) * (1 - w) +
           (v[d * width + a] * (1 - u) + v[d * width + c] * u) * w;
}
int cc_rpi_lens_shading_valid(const cc_rpi_lens_shading *shading)
{
    if (!shading || !shading->enabled)
        return 1;
    if (shading->enabled != 1)
        return 0;
    for (unsigned ch = 0; ch < 3; ++ch)
        for (unsigned i = 0; i < CC_VC4_LSC_SAMPLES; ++i)
            if (!(shading->gains[ch][i] >= 1 && shading->gains[ch][i] <= 7.99))
                return 0;
    return 1;
}
void cc_rpi_resample_shading(const double *table, unsigned width, unsigned height,
    const cc_sensor_geometry *g, double *output, unsigned out_width, unsigned out_height)
{
    double minimum = DBL_MAX;
    for (unsigned y = 0; y < out_height; ++y)
        for (unsigned x = 0; x < out_width; ++x) {
            double u = (x + 0.5) / out_width, v = (y + 0.5) / out_height;
            if (g->hflip)
                u = 1 - u;
            if (g->vflip)
                v = 1 - v;
            double sx = (g->crop.x - g->active.x + u * g->crop.width) / g->active.width * width - 0.5;
            double sy = (g->crop.y - g->active.y + v * g->crop.height) / g->active.height * height - 0.5;
            double value = cc_rpi_sample_grid(table, width, height, sx, sy);
            output[y * out_width + x] = value;
            if (value < minimum)
                minimum = value;
        }
    for (unsigned i = 0; i < out_width * out_height; ++i)
        output[i] /= minimum;
}
double cc_rpi_mode_noise_factor(const cc_sensor_geometry *g)
{
    /* The reference treats up to twofold scaling on each axis as binning. */
    int axes = (g->crop.width / g->width >= 2) + (g->crop.height / g->height >= 2);
    static const double factors[] = {1, 1.4142135623730951, 2};
    return factors[axes];
}
static struct bcm2835_isp_rational rational(double x)
{
    return (struct bcm2835_isp_rational){(int32_t)(x * 1000), 1000};
}
const cc_vc4_profile_info *cc_vc4_get_profile_info(enum cc_vc4_profile profile)
{
    const struct cc_vc4_profile_data *p = cc_vc4_profile_data(profile);
    return p ? &p->info : NULL;
}
int cc_vc4_lookup_profile(const char *name, enum cc_vc4_profile *out)
{
    if (!name || !out)
        return -EINVAL;
    for (unsigned i = CC_VC4_PROFILE_OV5647; i <= CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++i) {
        const cc_vc4_profile_info *p = cc_vc4_get_profile_info((enum cc_vc4_profile)i);
        if (p && !strcmp(name, p->name)) {
            *out = p->profile;
            return 0;
        }
    }
    return -ENOENT;
}
int cc_vc4_select_profile(const cc_sensor_model *sensor, enum cc_vc4_profile override,
                          enum cc_vc4_profile *out)
{
    if (!sensor || !out)
        return -EINVAL;
    const cc_sensor_properties *s = cc_sensor_get_properties(sensor);
    if (!s)
        return -EINVAL;
    if (override != CC_VC4_PROFILE_AUTO) {
        const cc_vc4_profile_info *p = cc_vc4_get_profile_info(override);
        if (!p)
            return -EINVAL;
        if (strcmp(s->model, p->sensor_model))
            return -ENOTSUP;
        *out = override;
        return 0;
    }
    return cc_vc4_lookup_profile(s->model, out) == 0 ? 0 : -ENOTSUP;
}
int cc_vc4_build_tuning_with_shading(enum cc_vc4_profile profile, const cc_sensor_geometry *g,
    double ct, double gain, const cc_rpi_lens_shading *shading, cc_vc4_tuning *out)
{
    const struct cc_vc4_profile_data *p = cc_vc4_profile_data(profile);
    if (!p || !out || !cc_sensor_geometry_valid(g) || !cc_rpi_lens_shading_valid(shading) ||
        !(ct >= 2000 && ct <= 13000) || !(gain >= 1 && gain <= 64))
        return -EINVAL;
    cc_vc4_tuning t = {0};
    t.colour_temperature = ct;
    t.black = (struct bcm2835_isp_black_level){
        .enabled = 1, .black_level_r = p->black, .black_level_g = p->black, .black_level_b = p->black};
    t.gamma.enabled = 1;
    for (unsigned i = 0; i < 33; ++i) {
        unsigned x = i < 16   ? i * 1024
                     : i < 24 ? (i - 16) * 2048 + 16384
                     : i < 32 ? (i - 24) * 4096 + 32768
                              : 65535;
        t.gamma.x[i] = (uint16_t)x;
        t.gamma.y[i] = (uint16_t)clamp(pwl(p->gamma, 66, x), 0, 65535);
    }
    unsigned ci = 0;
    while (ci + 2 < p->num_ccm && ct > p->ccm[ci + 1][0])
        ++ci;
    double f = clamp((ct - p->ccm[ci][0]) / (p->ccm[ci + 1][0] - p->ccm[ci][0]), 0, 1);
    t.ccm.enabled = 1;
    for (unsigned i = 0; i < 9; ++i)
        t.ccm.ccm.ccm[i / 3][i % 3] = rational(p->ccm[ci][i + 1] * (1 - f) + p->ccm[ci + 1][i + 1] * f);
    t.geq = (struct bcm2835_isp_geq){.enabled = 1,
                                     .offset = (uint32_t)clamp(p->geq_offset * gain, 0, 65535),
                                     .slope = rational(clamp(p->geq_slope * gain, 0, 0.99999))};
    /* Bounded Newton iteration avoids a mandatory libm dependency. */
    double root = gain;
    for (unsigned i = 0; i < 16; ++i)
        root = (root + gain / root) * 0.5;
    double mode_factor = cc_rpi_mode_noise_factor(g);
    root /= mode_factor;
    t.denoise = (struct bcm2835_isp_denoise){
        .enabled = 1, .constant = (uint32_t)(p->noise_constant * root * p->sdn_deviation),
        .slope = rational(p->noise_slope * root * p->sdn_deviation), .strength = rational(p->sdn_strength)};
    t.cdn = (struct bcm2835_isp_cdn){1, CDN_MODE_HIGH_QUALITY};
    t.sharpen = (struct bcm2835_isp_sharpen){
        .enabled = 1, .threshold = rational(p->sharpen_threshold * mode_factor),
        .strength = rational(p->sharpen_strength / mode_factor), .limit = rational(p->sharpen_limit / mode_factor)};
    t.dpc = (struct bcm2835_isp_dpc){1, DPC_MODE_NORMAL};
    unsigned li = 0;
    while (li + 2 < p->num_lsc && ct > p->lsc_ct[li + 1])
        ++li;
    f = clamp((ct - p->lsc_ct[li]) / (p->lsc_ct[li + 1] - p->lsc_ct[li]), 0, 1);
    double full[3][192];
    for (unsigned i = 0; i < 192; ++i) {
        double lum = 1 + p->luminance_strength * (p->lsc_y[i] - 1);
        full[0][i] = lum / (p->lsc_r[li * 192 + i] * (1 - f) + p->lsc_r[(li + 1) * 192 + i] * f);
        full[1][i] = lum;
        full[2][i] = lum / (p->lsc_b[li * 192 + i] * (1 - f) + p->lsc_b[(li + 1) * 192 + i] * f);
    }
    for (unsigned ch = 0; ch < 3; ++ch)
        cc_rpi_resample_shading(shading && shading->enabled ? shading->gains[ch] : full[ch],
            16, 12, g, t.lens_shading[ch], 16, 12);
    t.lens_shading_enabled = 1;
    *out = t;
    return 0;
}
int cc_vc4_build_tuning(enum cc_vc4_profile profile, const cc_sensor_geometry *g,
    double ct, double gain, cc_vc4_tuning *out)
{
    return cc_vc4_build_tuning_with_shading(profile, g, ct, gain, NULL, out);
}
int cc_vc4_validate_tuning(const cc_vc4_tuning *t)
{
    if (!t || t->lens_shading_enabled > 1 || t->black.enabled > 1 || t->gamma.enabled > 1 ||
        t->ccm.enabled > 1 || t->geq.enabled > 1 || t->denoise.enabled > 1 || t->cdn.enabled > 1 ||
        t->sharpen.enabled > 1 || t->dpc.enabled > 1 || t->cdn.mode > CDN_MODE_HIGH_QUALITY ||
        t->dpc.strength > DPC_MODE_STRONG || !finite_value(t->colour_temperature))
        return -EINVAL;
    if (t->gamma.enabled &&
        (t->gamma.x[0] || t->gamma.x[32] != 65535))
        return -EINVAL;
    for (unsigned i = 1; i < 33 && t->gamma.enabled; ++i)
        if (t->gamma.x[i] <= t->gamma.x[i - 1] || t->gamma.y[i] < t->gamma.y[i - 1])
            return -EINVAL;
    for (unsigned i = 0; i < 9; ++i)
        if (t->ccm.enabled && !t->ccm.ccm.ccm[i / 3][i % 3].den)
            return -EINVAL;
    const struct bcm2835_isp_rational *r[] = {&t->geq.slope,        &t->denoise.slope,
                                              &t->denoise.strength, &t->sharpen.threshold,
                                              &t->sharpen.strength, &t->sharpen.limit};
    for (unsigned i = 0; i < 6; ++i)
        if (!r[i]->den || r[i]->num < 0)
            return -EINVAL;
    if ((uint32_t)t->denoise.strength.num > t->denoise.strength.den)
        return -EINVAL;
    for (unsigned c = 0; c < 3 && t->lens_shading_enabled; ++c)
        for (unsigned i = 0; i < 192; ++i)
            if (!(t->lens_shading[c][i] >= 1 && t->lens_shading[c][i] <= 15.999))
                return -EINVAL;
    return 0;
}
int cc_vc4_ov5647_build_tuning(const cc_sensor_geometry *g, double ct, double gain, cc_vc4_tuning *out)
{
    return cc_vc4_build_tuning(CC_VC4_PROFILE_OV5647, g, ct, gain, out);
}
const char *cc_vc4_ov5647_profile_id(void)
{
    return cc_vc4_get_profile_info(CC_VC4_PROFILE_OV5647)->calibration_id;
}
int cc_sensor_flicker_exposure(const cc_sensor_model *model, const cc_sensor_timing *timing, uint64_t wanted,
                               uint32_t hz, uint64_t *out)
{
    if (!out || !wanted || (hz && hz != 50 && hz != 60))
        return -EINVAL;
    uint64_t actual = wanted;
    if (hz) {
        uint64_t cycles;
        int ret = cc_mul_div_u64(wanted, 2U * hz, UINT64_C(1000000000), false, &cycles);
        if (ret < 0)
            return ret;
        if (cycles) {
            ret = cc_mul_div_u64(cycles, UINT64_C(1000000000), 2U * hz, false, &actual);
            if (ret < 0)
                return ret;
        }
    }
    uint32_t lines;
    int ret = cc_sensor_exposure_lines(model, timing, actual, &lines);
    if (!ret)
        ret = cc_sensor_exposure_time(model, timing, lines, &actual);
    if (!ret)
        *out = actual;
    return ret;
}
