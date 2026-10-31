/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* VC4 mappings/calibration adapted from Raspberry Pi Ltd's libcamera 0.7.2.
 * See data/ov5647-vc4.json and LICENSES/BSD-2-Clause.txt. */
#include "integer_math.h"
#include "internal.h"
#include "ov5647_profile.h"
#include "subdev_compat.h"
#include <ccamera/vc4_tuning.h>
#include <errno.h>
#include <float.h>
#include <string.h>

static double clamp(double x, double a, double b) { return x < a ? a : x > b ? b : x; }
static int finite_value(double x) { return x >= -DBL_MAX && x <= DBL_MAX; }
static int geometry_valid(const cc_sensor_geometry *g)
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
    if (!geometry_valid(&g))
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
static double sample(const double *v, double x, double y)
{
    x = clamp(x, 0, 15);
    y = clamp(y, 0, 11);
    unsigned a = (unsigned)x, b = (unsigned)y, c = a < 15 ? a + 1 : a, d = b < 11 ? b + 1 : b;
    double u = x - a, w = y - b;
    return (v[b * 16 + a] * (1 - u) + v[b * 16 + c] * u) * (1 - w) +
           (v[d * 16 + a] * (1 - u) + v[d * 16 + c] * u) * w;
}
static struct bcm2835_isp_rational rational(double x)
{
    return (struct bcm2835_isp_rational){(int32_t)(x * 1000), 1000};
}
int cc_vc4_ov5647_build_tuning(const cc_sensor_geometry *g, double ct, double gain, cc_vc4_tuning *out)
{
    if (!out || !geometry_valid(g) || !(ct >= 2000 && ct <= 13000) || !(gain >= 1 && gain <= 64))
        return -EINVAL;
    cc_vc4_tuning t = {0};
    t.colour_temperature = ct;
    t.black = (struct bcm2835_isp_black_level){
        .enabled = 1, .black_level_r = 1024, .black_level_g = 1024, .black_level_b = 1024};
    t.gamma.enabled = 1;
    for (unsigned i = 0; i < 33; ++i) {
        unsigned x = i < 16   ? i * 1024
                     : i < 24 ? (i - 16) * 2048 + 16384
                     : i < 32 ? (i - 24) * 4096 + 32768
                              : 65535;
        t.gamma.x[i] = (uint16_t)x;
        t.gamma.y[i] = (uint16_t)clamp(pwl(ov_gamma, 66, x), 0, 65535);
    }
    unsigned ci = 0;
    while (ci < 4 && ct > ov_ccm[ci + 1][0])
        ++ci;
    double f = clamp((ct - ov_ccm[ci][0]) / (ov_ccm[ci + 1][0] - ov_ccm[ci][0]), 0, 1);
    t.ccm.enabled = 1;
    for (unsigned i = 0; i < 9; ++i)
        t.ccm.ccm.ccm[i / 3][i % 3] = rational(ov_ccm[ci][i + 1] * (1 - f) + ov_ccm[ci + 1][i + 1] * f);
    t.geq = (struct bcm2835_isp_geq){.enabled = 1,
                                     .offset = (uint32_t)clamp(401 * gain, 0, 65535),
                                     .slope = rational(clamp(0.05619 * gain, 0, 0.99999))};
    /* Bounded Newton iteration avoids a mandatory libm dependency. */
    double root = gain;
    for (unsigned i = 0; i < 16; ++i)
        root = (root + gain / root) * 0.5;
    t.denoise = (struct bcm2835_isp_denoise){
        .enabled = 1, .constant = 0, .slope = rational(4.25 * root * 3.2), .strength = rational(0.75)};
    t.cdn = (struct bcm2835_isp_cdn){1, CDN_MODE_HIGH_QUALITY};
    t.sharpen = (struct bcm2835_isp_sharpen){
        .enabled = 1, .threshold = rational(0.75), .strength = rational(1), .limit = rational(0.5)};
    t.dpc = (struct bcm2835_isp_dpc){1, DPC_MODE_NORMAL};
    unsigned li = ct > 5000 ? 1 : 0;
    f = clamp((ct - ov_lsc_ct[li]) / (ov_lsc_ct[li + 1] - ov_lsc_ct[li]), 0, 1);
    double full[3][192];
    for (unsigned i = 0; i < 192; ++i) {
        double lum = 1 + 0.5 * (ov_lsc_y[i] - 1);
        full[0][i] = lum / (ov_lsc_r[li][i] * (1 - f) + ov_lsc_r[li + 1][i] * f);
        full[1][i] = lum;
        full[2][i] = lum / (ov_lsc_b[li][i] * (1 - f) + ov_lsc_b[li + 1][i] * f);
    }
    for (unsigned ch = 0; ch < 3; ++ch) {
        double minimum = DBL_MAX;
        for (unsigned y = 0; y < 12; ++y)
            for (unsigned x = 0; x < 16; ++x) {
                double u = ((double)x + 0.5) / 16, v = ((double)y + 0.5) / 12;
                if (g->hflip)
                    u = 1 - u;
                if (g->vflip)
                    v = 1 - v;
                double sx =
                    ((double)(g->crop.x - g->active.x) + u * g->crop.width) / g->active.width * 16 - 0.5;
                double sy =
                    ((double)(g->crop.y - g->active.y) + v * g->crop.height) / g->active.height * 12 - 0.5;
                double value = sample(full[ch], sx, sy);
                t.lens_shading[ch][y * 16 + x] = value;
                if (value < minimum)
                    minimum = value;
            }
        for (unsigned i = 0; i < 192; ++i)
            t.lens_shading[ch][i] /= minimum;
    }
    t.lens_shading_enabled = 1;
    *out = t;
    return 0;
}
int cc_vc4_validate_tuning(const cc_vc4_tuning *t)
{
    if (!t || t->lens_shading_enabled > 1 || t->black.enabled > 1 || t->gamma.enabled > 1 ||
        t->ccm.enabled > 1 || t->geq.enabled > 1 || t->denoise.enabled > 1 || t->cdn.enabled > 1 ||
        t->sharpen.enabled > 1 || t->dpc.enabled > 1 || t->cdn.mode > CDN_MODE_HIGH_QUALITY ||
        t->dpc.strength > DPC_MODE_STRONG || !finite_value(t->colour_temperature))
        return -EINVAL;
    if (t->gamma.enabled &&
        (t->gamma.x[0] || t->gamma.y[0] || t->gamma.x[32] != 65535 || t->gamma.y[32] != 65535))
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
const char *cc_vc4_ov5647_profile_id(void) { return CC_OV5647_PROFILE_ID; }
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
