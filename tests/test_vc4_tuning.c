/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/capture.h>
#include <ccamera/ipa_proxy.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vc4_tuning_vectors.h"
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x)) {                                                                                          \
            fprintf(stderr, "%s:%d %s\n", __FILE__, __LINE__, #x);                                           \
            exit(1);                                                                                         \
        }                                                                                                    \
    } while (0)
#define OK(x) CHECK((x) == 0)
static const char *proxy_worker;
static int create_controller(const cc_ipa_vc4_options *controller, cc_ipa **ipa)
{
    if (!proxy_worker) return cc_ipa_vc4_create(controller, ipa);
    cc_ipa_proxy_options options = {.worker_path = proxy_worker};
    return cc_ipa_vc4_create_proxy(controller, &options, ipa);
}
static cc_sensor_geometry geometry(void)
{
    return (cc_sensor_geometry){
        .active = {16, 6, 2592, 1944}, .crop = {16, 6, 2592, 1944}, .width = 2592, .height = 1944};
}
static cc_control_info range(uint32_t id, int64_t low, int64_t high)
{
    return (cc_control_info){
        .id = id, .type = V4L2_CTRL_TYPE_INTEGER, .minimum = low, .maximum = high, .step = 1, .elems = 1};
}
static cc_ipa_configuration configuration(void)
{
    cc_ipa_configuration c = {.pipeline = CC_IPA_VC4,
                              .statistics_fourcc = CC_FOURCC('B', 'S', 'T', 'A'),
                              .source = {.width = 2592, .height = 1944, .fourcc = V4L2_PIX_FMT_SGBRG10P},
                              .timing = {100000000, 2592, 1944, 408, 56},
                              .num_sensor_controls = 3,
                              .geometry = geometry(),
                              .timing_limits = {.pixel_rate = 100000000,
                                                .width = 2592,
                                                .height = 1944,
                                                .min_hblank = 408,
                                                .max_hblank = 5599,
                                                .hblank_step = 1,
                                                .min_vblank = 24,
                                                .max_vblank = 30823,
                                                .vblank_step = 1,
                                                .min_exposure = 4,
                                                .exposure_step = 1}};
    OK(cc_sensor_lookup("ov5647", &c.sensor));
    c.sensor_controls[0] = range(V4L2_CID_EXPOSURE, 4, 1996);
    c.sensor_controls[1] = range(V4L2_CID_ANALOGUE_GAIN, 16, 1023);
    c.sensor_controls[2] = range(V4L2_CID_VBLANK, 24, 30823);
    c.initial_controls[0] = (cc_control){.id = V4L2_CID_EXPOSURE, .value = 500};
    c.initial_controls[1] = (cc_control){.id = V4L2_CID_ANALOGUE_GAIN, .value = 16};
    c.initial_controls[2] = (cc_control){.id = V4L2_CID_VBLANK, .value = 56};
    for (unsigned i = 0; i < 3; ++i)
        c.sensor_delays[i] =
            (cc_sensor_control_delay){c.initial_controls[i].id, 2, i == 2 ? CC_SENSOR_CONTROL_PRIORITY : 0};
    c.red_balance = range(V4L2_CID_RED_BALANCE, 1, 8000);
    c.blue_balance = range(V4L2_CID_BLUE_BALANCE, 1, 8000);
    c.digital_gain = range(V4L2_CID_DIGITAL_GAIN, 1, 8000);
    return c;
}
struct geometry_driver {
    int error;
    bool bad;
};
static int geometry_ioctl(void *context, int fd, unsigned long command, void *arg)
{
    struct geometry_driver *d = context;
    CHECK(fd == 7);
    if (d->error)
        return d->error;
    if (command == VIDIOC_SUBDEV_G_FMT) {
        struct cc_subdev_format *f = arg;
        CHECK(f->pad == 3 && f->which == V4L2_SUBDEV_FORMAT_ACTIVE);
        f->format.width = 2592;
        f->format.height = 1944;
        return 0;
    }
    if (_IOC_NR(command) == 61) {
        struct v4l2_subdev_selection *q = arg;
        CHECK(q->pad == 3);
        q->r = (struct v4l2_rect){.left = d->bad ? -1 : 16, .top = 6, .width = 2592, .height = 1944};
        return 0;
    }
    if (command == VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q = arg;
        CHECK(q->id == V4L2_CID_HFLIP || q->id == V4L2_CID_VFLIP);
        q->type = V4L2_CTRL_TYPE_BOOLEAN;
        q->minimum = 0;
        q->maximum = 1;
        q->step = 1;
        q->elems = 1;
        q->elem_size = 4;
        return 0;
    }
    CHECK(command == VIDIOC_G_EXT_CTRLS);
    struct v4l2_ext_controls *q = arg;
    for (uint32_t i = 0; i < q->count; ++i)
        q->controls[i].value = q->controls[i].id == V4L2_CID_HFLIP;
    return 0;
}
static void live_geometry(void)
{
    struct geometry_driver driver = {0};
    const struct cc_io_ops ops = {.ioctl = geometry_ioctl};
    cc_stream stream = {0};
    cc_camera camera = {.io = {&ops, &driver}, .sensor_fd = 7, .sensor_pad = 3, .stream = &stream};
    cc_sensor_geometry g;
    OK(cc_camera_get_sensor_geometry(&camera, &g));
    CHECK(g.hflip == 1 && g.vflip == 0 && g.crop.x == 16 && g.crop.y == 6);
    cc_sensor_geometry saved = g;
    driver.error = -ENOTTY;
    CHECK(cc_camera_get_sensor_geometry(&camera, &g) == -ENOTTY);
    CHECK(!memcmp(&g, &saved, sizeof(g)));
    driver.error = 0;
    driver.bad = true;
    CHECK(cc_camera_get_sensor_geometry(&camera, &g) == -EPROTO);
    CHECK(!memcmp(&g, &saved, sizeof(g)));
}

static void profile(void)
{
    cc_vc4_tuning t, flipped;
    cc_sensor_geometry g = geometry();
    OK(cc_vc4_ov5647_build_tuning(&g, 5890, 1, &t));
    OK(cc_vc4_validate_tuning(&t));
    CHECK(t.black.black_level_r == 1024 && t.gamma.x[1] == 1024 && t.gamma.y[1] == 5040 &&
          t.gamma.x[32] == 65535);
    CHECK(t.ccm.ccm.ccm[0][0].num == 2136 && t.ccm.ccm.ccm[0][0].den == 1000);
    CHECK(t.cdn.mode == CDN_MODE_HIGH_QUALITY && t.denoise.slope.num == 13600);
    OK(cc_vc4_ov5647_build_tuning(&g, 3000, 1, &flipped));
    /* Full-field gains from multiplying archive chroma and luminance tables. */
    CHECK(fabs(flipped.lens_shading[0][0] - 1.728993563261075) < 1e-12);
    CHECK(fabs(flipped.lens_shading[2][191] - 1.467671070986057) < 1e-12);
    g.hflip = g.vflip = 1;
    OK(cc_vc4_ov5647_build_tuning(&g, 5890, 1, &flipped));
    for (unsigned c = 0; c < 3; ++c)
        for (unsigned i = 0; i < 192; ++i)
            CHECK(fabs(t.lens_shading[c][i] - flipped.lens_shading[c][191 - i]) < 1e-12);
    g = geometry();
    g.crop = (cc_rectangle){688, 434, 1920, 1080};
    g.width = 1920;
    g.height = 1080;
    OK(cc_vc4_ov5647_build_tuning(&g, 3000, 4, &t));
    OK(cc_vc4_validate_tuning(&t));
    g.crop.x = -1;
    CHECK(cc_vc4_ov5647_build_tuning(&g, 5000, 1, &t) == -EINVAL);
    t.gamma.x[3] = t.gamma.x[2];
    CHECK(cc_vc4_validate_tuning(&t) == -EINVAL);
    g = geometry();
    OK(cc_vc4_ov5647_build_tuning(&g, 5000, 1, &t));
    t.lens_shading[0][0] = NAN;
    CHECK(cc_vc4_validate_tuning(&t) == -EINVAL);
}
static void profile_vectors(void)
{
    cc_sensor_geometry g = {.active = {16, 8, 4096, 3072}, .width = 4096, .height = 3072};
    for (size_t i = 0; i < sizeof(vc4_vectors) / sizeof(vc4_vectors[0]); ++i) {
        const struct vc4_vector *v = &vc4_vectors[i];
        g.crop = v->crop ? (cc_rectangle){1040, 776, 2048, 1536} : g.active;
        g.hflip = v->flips & 1U;
        g.vflip = (v->flips >> 1) & 1U;
        cc_vc4_tuning t;
        OK(cc_vc4_build_tuning(v->profile, &g, v->ct, v->gain, &t));
        OK(cc_vc4_validate_tuning(&t));
        CHECK(t.black.black_level_r == v->black && t.black.black_level_g == v->black &&
              t.black.black_level_b == v->black);
        for (unsigned j = 0; j < 9; ++j) {
            struct bcm2835_isp_rational r = t.ccm.ccm.ccm[j / 3][j % 3];
            CHECK(r.den == 1000 && abs(r.num - v->ccm[j]) <= 1);
        }
        for (unsigned j = 0; j < 33; ++j)
            CHECK(abs((int)t.gamma.y[j] - (int)v->gamma[j]) <= 1);
        CHECK(t.geq.offset == v->geq_offset && abs(t.geq.slope.num - v->geq_slope) <= 1 &&
              abs(t.denoise.slope.num - v->noise_slope) <= 1);
        for (unsigned ch = 0; ch < 3; ++ch)
            for (unsigned j = 0; j < 6; ++j)
                CHECK(fabs(t.lens_shading[ch][lsc_indices[j]] - v->lsc[ch][j]) < 1e-11);
        cc_vc4_tuning saved = t;
        CHECK(cc_vc4_build_tuning(v->profile, &g, NAN, 1, &t) == -EINVAL);
        CHECK(!memcmp(&saved, &t, sizeof(t)));
        CHECK(cc_vc4_build_tuning(v->profile, &g, 4500, INFINITY, &t) == -EINVAL);
        CHECK(!memcmp(&saved, &t, sizeof(t)));
        CHECK(cc_vc4_build_tuning(CC_VC4_PROFILE_AUTO, &g, 4500, 1, &t) == -EINVAL);
        CHECK(!memcmp(&saved, &t, sizeof(t)));
    }
}
static void measured_shading(void)
{
    cc_ipa_vc4_options options = {.profile = CC_VC4_PROFILE_IMX477, .lens_shading.enabled = 1};
    for (unsigned y = 0; y < 12; ++y)
        for (unsigned x = 0; x < 16; ++x) {
            options.lens_shading.gains[0][y * 16 + x] = 1 + x / 16.0;
            options.lens_shading.gains[1][y * 16 + x] = 1;
            options.lens_shading.gains[2][y * 16 + x] = 1 + y / 8.0;
        }
    cc_sensor_geometry g = geometry();
    cc_vc4_tuning full, cropped;
    OK(cc_vc4_build_tuning_with_shading(options.profile, &g, 4500, 1, &options.lens_shading, &full));
    CHECK(full.lens_shading[0][15] == 1 + 15 / 16.0 && full.lens_shading[2][191] == 1 + 11 / 8.0);
    g.crop.width /= 2;
    g.width /= 4;
    g.hflip = 1;
    OK(cc_vc4_build_tuning_with_shading(options.profile, &g, 4500, 1, &options.lens_shading, &cropped));
    CHECK(cropped.lens_shading[0][0] == 1 + 7.25 / 16 && cropped.lens_shading[0][15] == 1);
    const double mode_factor = 1.4142135623730951;
    CHECK(abs((int)(cropped.denoise.slope.num * mode_factor) - full.denoise.slope.num) <= 2);
    CHECK(abs((int)(cropped.sharpen.strength.num * mode_factor) - full.sharpen.strength.num) <= 2);
    CHECK(abs((int)(full.sharpen.threshold.num * mode_factor) - cropped.sharpen.threshold.num) <= 1);

    cc_ipa_configuration c = configuration();
    OK(cc_sensor_lookup("imx477", &c.sensor));
    c.sensor_controls[1].minimum = 0;
    c.sensor_controls[1].maximum = 960;
    c.initial_controls[1].value = 0;
    cc_ipa *ipa;
    OK(create_controller(&options, &ipa));
    options.lens_shading.gains[0][0] = NAN;
    cc_vc4_tuning saved = full;
    CHECK(cc_vc4_build_tuning_with_shading(options.profile, &g, 4500, 1, &options.lens_shading, &full) == -EINVAL);
    CHECK(!memcmp(&full, &saved, sizeof(full)));
    cc_ipa *invalid = NULL;
    CHECK(create_controller(&options, &invalid) == -EINVAL && !invalid);
    OK(cc_ipa_configure(ipa, &c));
    OK(cc_ipa_start(ipa));
    cc_ipa_request request = {.timing = c.timing, .timing_limits = c.timing_limits};
    cc_ipa_settings s;
    OK(cc_ipa_prepare(ipa, &request, &s));
    CHECK(s.flags & CC_IPA_VC4_TUNING);
    CHECK(!memcmp(full.lens_shading, s.vc4.lens_shading, sizeof(full.lens_shading)));
    cc_ipa_destroy(ipa);
}
static void selection(void)
{
    CHECK(!cc_vc4_get_profile_info(CC_VC4_PROFILE_AUTO));
    CHECK(!cc_vc4_get_profile_info((enum cc_vc4_profile)-1));
    for (unsigned i = CC_VC4_PROFILE_OV5647; i <= CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++i) {
        enum cc_vc4_profile id = (enum cc_vc4_profile)i, selected = CC_VC4_PROFILE_AUTO;
        const cc_vc4_profile_info *p = cc_vc4_get_profile_info(id);
        CHECK(p && p->profile == id && strstr(p->calibration_id, p->name));
        OK(cc_vc4_lookup_profile(p->name, &selected)); CHECK(selected == id);
        const cc_sensor_model *model;
        OK(cc_sensor_lookup(p->sensor_model, &model));
        OK(cc_vc4_select_profile(model, id, &selected)); CHECK(selected == id);
        OK(cc_vc4_select_profile(model, CC_VC4_PROFILE_AUTO, &selected));
        CHECK(!strcmp(cc_vc4_get_profile_info(selected)->name, p->sensor_model));
        CHECK((p->flags & CC_VC4_PROFILE_NOIR) == (strstr(p->name, "noir") ? CC_VC4_PROFILE_NOIR : 0));
        CHECK(cc_vc4_select_profile(model, (enum cc_vc4_profile)999, &selected) == -EINVAL);
        enum cc_vc4_profile saved = selected;
        CHECK(cc_vc4_lookup_profile("imx708_typo", &selected) == -ENOENT && selected == saved);
        CHECK(cc_vc4_lookup_profile(NULL, &selected) == -EINVAL && selected == saved);
    }
    const cc_sensor_model *sensor;
    OK(cc_sensor_lookup("imx219", &sensor));
    enum cc_vc4_profile selected = CC_VC4_PROFILE_IMX477;
    CHECK(cc_vc4_select_profile(sensor, CC_VC4_PROFILE_IMX708_WIDE, &selected) == -ENOTSUP);
    CHECK(selected == CC_VC4_PROFILE_IMX477);
    OK(cc_sensor_lookup("imx290", &sensor));
    CHECK(cc_vc4_select_profile(sensor, CC_VC4_PROFILE_AUTO, &selected) == -ENOTSUP);
    CHECK(selected == CC_VC4_PROFILE_IMX477);
}

static void module_controllers(void)
{
    for (unsigned i = CC_VC4_PROFILE_OV5647; i <= CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++i) {
        enum cc_vc4_profile id = (enum cc_vc4_profile)i;
        const cc_vc4_profile_info *p = cc_vc4_get_profile_info(id);
        cc_ipa_configuration c = configuration();
        OK(cc_sensor_lookup(p->sensor_model, &c.sensor));
        if (strcmp(p->sensor_model, "ov5647")) {
            c.sensor_controls[1].minimum = 0;
            c.sensor_controls[1].maximum = !strcmp(p->sensor_model, "imx219") ? 232 : 960;
            c.initial_controls[1].value = 0;
        }
        cc_ipa_vc4_options options = {.mains_hz = 50, .profile = id};
        cc_ipa *ipa;
        OK(create_controller(&options, &ipa));
        OK(cc_ipa_configure(ipa, &c));
        OK(cc_ipa_start(ipa));
        uint32_t code;
        OK(cc_sensor_gain_code(c.sensor, 2, &code));
        cc_control manual[] = {{.id = V4L2_CID_EXPOSURE, .value = 500},
            {.id = V4L2_CID_ANALOGUE_GAIN, .value = code}};
        cc_ipa_request request = {.timing = c.timing, .timing_limits = c.timing_limits,
            .sensor_controls = manual, .num_sensor_controls = 2};
        cc_ipa_settings s;
        OK(cc_ipa_prepare(ipa, &request, &s));
        CHECK(s.flags == CC_IPA_VC4_TUNING && s.sensor_controls[2].value == code);
        cc_vc4_tuning expected;
        OK(cc_vc4_build_tuning(id, &c.geometry, 4500, 2, &expected));
        CHECK(!memcmp(&s.vc4, &expected, sizeof(expected)));
        struct bcm2835_isp_stats stats = {.size = sizeof(stats)};
        for (unsigned j = 0; j < 192; ++j)
            stats.awb_stats[j] = (struct bcm2835_isp_stats_region){.counted = 1000,
                .r_sum = 1000000, .g_sum = 2000000, .b_sum = 1500000};
        for (unsigned j = 0; j < 15; ++j) stats.agc_stats[j] = stats.awb_stats[j];
        if (p->flags & CC_VC4_PROFILE_NOIR) {
            /* Both tails are coloured outliers. The middle half still has
             * R/G/B = 1/2/1.5, independently for each channel's ordering. */
            for (unsigned j = 0; j < 96; ++j) {
                stats.awb_stats[j].r_sum = j < 48 ? 100000 : 4000000;
                stats.awb_stats[j].b_sum = j < 48 ? 4000000 : 800000;
            }
        }
        stats.hist[0].g_hist[32] = 1000;
        cc_ipa_frame frame = {.vc4 = &s.vc4, .statistics_fourcc = c.statistics_fourcc,
            .statistics_data = &stats, .statistics_size = sizeof(stats), .isp_controls = s.isp_controls,
            .num_isp_controls = s.num_isp_controls, .request_sensor_controls = manual,
            .num_request_sensor_controls = 2};
        frame.settings_sensor_controls=s.sensor_controls; frame.num_settings_sensor_controls=3;
        frame.sensor.count = 3;
        memcpy(frame.sensor.controls, s.sensor_controls, sizeof(cc_control) * 3);
        cc_ipa_metadata m;
        OK(cc_ipa_process(ipa, &frame, &m));
        CHECK(m.analogue_gain == 2 && m.flags & CC_IPA_AE_CONVERGED && m.flags & CC_IPA_METADATA_AWB);
        OK(cc_ipa_prepare(ipa, &request, &s));
        if (p->flags & CC_VC4_PROFILE_NOIR) {
            CHECK(s.vc4.colour_temperature == 4500);
            CHECK(s.isp_controls[0].value == 1399 && s.isp_controls[1].value == 1133);
        }
        manual[1].value = c.sensor_controls[1].maximum + 1;
        CHECK(cc_ipa_prepare(ipa, &request, &s) == -ERANGE);
        cc_ipa_stop(ipa);
        /* Restart and profile mismatch must not reuse a previous calibration. */
        OK(cc_ipa_configure(ipa, &c)); OK(cc_ipa_start(ipa)); cc_ipa_stop(ipa);
        OK(cc_sensor_lookup(!strcmp(p->sensor_model, "ov5647") ? "imx219" : "ov5647", &c.sensor));
        CHECK(cc_ipa_configure(ipa, &c) == -ENOTSUP);
        cc_ipa_destroy(ipa);
    }
    cc_ipa *ipa = NULL;
    cc_ipa_vc4_options bad = {.profile = (enum cc_vc4_profile)999};
    CHECK(create_controller(&bad, &ipa) == -EINVAL && !ipa);
}
static void flicker(void)
{
    cc_ipa_configuration c = configuration();
    uint64_t result = 0;
    OK(cc_sensor_flicker_exposure(c.sensor, &c.timing, 55000000, 50, &result));
    CHECK(result == 49980000);
    OK(cc_sensor_flicker_exposure(c.sensor, &c.timing, 55000000, 60, &result));
    CHECK(result == 49980000);
    OK(cc_sensor_flicker_exposure(c.sensor, &c.timing, 6000000, 50, &result));
    CHECK(result == 6000000);
    CHECK(cc_sensor_flicker_exposure(c.sensor, &c.timing, 10000000, 100, &result) == -EINVAL);
    CHECK(cc_sensor_flicker_exposure(c.sensor, &c.timing, UINT64_MAX, 50, &result) < 0);
}
static void controller(void)
{
    cc_ipa *ipa = NULL;
    cc_ipa_configuration c = configuration();
    OK(create_controller(NULL, &ipa));
    OK(cc_ipa_configure(ipa, &c));
    OK(cc_ipa_start(ipa));
    cc_ipa_request request = {.timing = c.timing, .timing_limits = c.timing_limits};
    cc_ipa_settings s;
    OK(cc_ipa_prepare(ipa, &request, &s));
    CHECK(s.flags == CC_IPA_VC4_TUNING && s.num_sensor_controls == 3);
    CHECK(s.sensor_controls[0].id == V4L2_CID_VBLANK && s.sensor_controls[1].value >= 4);
    struct bcm2835_isp_stats stats = {.size = sizeof(stats)};
    for (unsigned i = 0; i < 192; ++i)
        stats.awb_stats[i] = (struct bcm2835_isp_stats_region){
            .counted = 1000, .r_sum = 1000000, .g_sum = 1640000, .b_sum = 1200000};
    for (unsigned i = 0; i < 15; ++i)
        stats.agc_stats[i] = stats.awb_stats[i];
    stats.hist[0].g_hist[24] = 10000;
    cc_ipa_frame frame = {.vc4 = &s.vc4,
                          .statistics_fourcc = c.statistics_fourcc,
                          .statistics_data = &stats,
                          .statistics_size = sizeof(stats),
                          .isp_controls = s.isp_controls,
                          .num_isp_controls = s.num_isp_controls,
                          .settings_sensor_controls = s.sensor_controls,
                          .num_settings_sensor_controls = s.num_sensor_controls};
    frame.sensor.count = 3;
    memcpy(frame.sensor.controls, s.sensor_controls, sizeof(cc_control) * 3);
    cc_ipa_metadata m;
    OK(cc_ipa_process(ipa, &frame, &m));
    CHECK(m.flags & CC_IPA_METADATA_VC4);
    CHECK(m.colour_temperature == 4500 && m.frame_duration_ns > 0);
    frame.untrusted = 1;
    OK(cc_ipa_process(ipa, &frame, &m));
    CHECK((m.flags & CC_IPA_METADATA_PHOTO) && !(m.photo_status & CC_PHOTO_STATS_TRUSTED));
    frame.untrusted = 0;
    /* Bright saturation with the firmware's AGC count sentinel. */
    stats.agc_stats[0] = (struct bcm2835_isp_stats_region){.counted = 26334, .notcounted = UINT32_MAX,
        .r_sum = 215857735, .g_sum = 216094609, .b_sum = 216485308};
    OK(cc_ipa_process(ipa, &frame, &m));
    CHECK((m.photo_status & CC_PHOTO_STATS_TRUSTED) && m.luminance >= 0 && m.luminance <= 1);
    stats.agc_stats[0].r_sum = UINT64_MAX;
    CHECK(cc_ipa_process(ipa, &frame, &m) == -EPROTO);
    stats.agc_stats[0] = stats.awb_stats[0];
    /* Low-count WB regions must not claim convergence. */
    memset(stats.awb_stats, 0, sizeof(stats.awb_stats));
    stats.awb_stats[0] =
        (struct bcm2835_isp_stats_region){.counted = 1, .g_sum = 100, .r_sum = 100, .b_sum = 100};
    OK(cc_ipa_process(ipa, &frame, &m));
    CHECK(!(m.flags & CC_IPA_AWB_CONVERGED));
    /* Manual long shutter grows frame length, without automatic 66.7ms cap. */
    cc_control manual[] = {{.id = V4L2_CID_EXPOSURE, .value = 6000},
                           {.id = V4L2_CID_ANALOGUE_GAIN, .value = 32}};
    request.sensor_controls = manual;
    request.num_sensor_controls = 2;
    OK(cc_ipa_prepare(ipa, &request, &s));
    CHECK(s.sensor_controls[1].value == 6000 && s.sensor_controls[0].value >= 4060);
    cc_control blank = {.id=V4L2_CID_VBLANK,.value=56};
    request.sensor_controls=&blank;request.num_sensor_controls=1;request.timing_limits.vblank_step=0;
    CHECK(cc_ipa_prepare(ipa,&request,&s)==-EINVAL);
    request.sensor_controls=manual;request.num_sensor_controls=2;request.timing_limits=c.timing_limits;
    manual[0].value = UINT32_MAX;
    CHECK(cc_ipa_prepare(ipa, &request, &s) == -ERANGE);
    cc_ipa_stop(ipa);
    OK(cc_ipa_configure(ipa, &c));
    OK(cc_ipa_start(ipa));
    cc_ipa_destroy(ipa);
    c.pipeline = CC_IPA_PISP;
    OK(create_controller(NULL, &ipa));
    CHECK(cc_ipa_configure(ipa, &c) == -ENOTSUP);
    cc_ipa_destroy(ipa);
    cc_ipa_vc4_options bad = {.mains_hz = 42};
    CHECK(create_controller(&bad, &ipa) == -EINVAL && !ipa);
}
static void feedback(enum cc_vc4_profile profile)
{
    cc_ipa_configuration c = configuration();
    const cc_vc4_profile_info *p = cc_vc4_get_profile_info(profile);
    OK(cc_sensor_lookup(p->sensor_model, &c.sensor));
    bool bright_start = profile == CC_VC4_PROFILE_OV5647;
    if (bright_start) {
        /* Reopen in daylight with the shutter/gain left by a dim scene. */
        c.initial_controls[0].value = 1000;
        c.initial_controls[1].value = 48;
    }
    if (strcmp(p->sensor_model, "ov5647")) {
        c.sensor_controls[1].minimum = 0;
        c.sensor_controls[1].maximum = !strcmp(p->sensor_model, "imx219") ? 232 : 960;
        c.initial_controls[1].value = 0;
    }
    /* Exercise AUTO selection too: no variant flags means the standard profile. */
    cc_ipa_vc4_options o = {.mains_hz = 50, .red_gain = 1, .blue_gain = 1,
        .profile = p->flags ? profile : CC_VC4_PROFILE_AUTO};
    cc_ipa *ipa;
    OK(create_controller(&o, &ipa));
    OK(cc_ipa_configure(ipa, &c));
    OK(cc_ipa_start(ipa));
    cc_ipa_request request = {.timing = c.timing, .timing_limits = c.timing_limits};
    unsigned stable = 0;
    for (unsigned iteration = 0; iteration < 160; ++iteration) {
        cc_ipa_settings s;
        OK(cc_ipa_prepare(ipa, &request, &s));
        uint32_t exposure = (uint32_t)s.sensor_controls[1].value;
        double gain;
        OK(cc_sensor_gain(c.sensor, (uint32_t)s.sensor_controls[2].value, &gain));
        uint64_t ns;
        OK(cc_sensor_exposure_time(c.sensor, &request.timing, exposure, &ns));
        double scene = iteration < 80 ? (bright_start ? 2500000 : 40000000) : 160000000;
        double level = (double)ns * gain / scene;
        if (level > 0.99)
            level = 0.99;
        uint64_t sum = (uint64_t)(level * 8192 * 1000);
        struct bcm2835_isp_stats stats = {.size = sizeof(stats)};
        for (unsigned i = 0; i < 192; ++i)
            stats.awb_stats[i] =
                (struct bcm2835_isp_stats_region){.counted = 1000, .r_sum = sum, .g_sum = sum, .b_sum = sum};
        for (unsigned i = 0; i < 15; ++i)
            stats.agc_stats[i] = stats.awb_stats[i];
        stats.hist[0].g_hist[(unsigned)(level * 128)] = 1000;
        cc_ipa_frame frame = {.vc4 = &s.vc4,
                              .statistics_fourcc = c.statistics_fourcc,
                              .statistics_data = &stats,
                              .statistics_size = sizeof(stats),
                              .isp_controls = s.isp_controls,
                              .num_isp_controls = s.num_isp_controls,
                              .settings_sensor_controls = s.sensor_controls,
                              .num_settings_sensor_controls = 3};
        frame.sensor.count = 3;
        memcpy(frame.sensor.controls, s.sensor_controls, 3 * sizeof(cc_control));
        cc_ipa_metadata m;
        OK(cc_ipa_process(ipa, &frame, &m));
        if (iteration == 80)
            stable = 0;
        stable = (m.flags & CC_IPA_AE_CONVERGED) ? stable + 1 : 0;
        CHECK(m.flags & CC_IPA_AWB_CONVERGED);
        if ((bright_start && iteration == 24) || iteration == 79 || iteration == 159)
            CHECK(stable >= 3);
        request.timing.vblank = (uint32_t)s.sensor_controls[0].value;
    }
    cc_ipa_destroy(ipa);
}

int main(int argc, char **argv)
{
    CHECK(argc == 1 || argc == 2);
    if (argc == 2) {
        proxy_worker = argv[1]; cc_ipa *probe = NULL;
        int ret = create_controller(NULL, &probe);
        if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) return 77;
        OK(ret); cc_ipa_destroy(probe);
    } else {
        live_geometry();
        profile();
        profile_vectors();
        selection();
        flicker();
    }
    measured_shading();
    module_controllers();
    controller();
    for (unsigned i = CC_VC4_PROFILE_OV5647; i <= CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++i)
        feedback((enum cc_vc4_profile)i);
    puts("VC4 calibration and still controllers passed");
    return 0;
}
