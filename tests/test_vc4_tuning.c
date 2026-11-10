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
    /* Independent full-field calibration-only reference values. */
    CHECK(fabs(flipped.lens_shading[0][0] - 1.5080933156311813) < 1e-12);
    CHECK(fabs(flipped.lens_shading[2][191] - 1.7570788655247644) < 1e-12);
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
    CHECK(!m.flags);
    frame.untrusted = 0;
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
static void feedback(void)
{
    cc_ipa_configuration c = configuration();
    cc_ipa_vc4_options o = {.mains_hz = 50, .red_gain = 1, .blue_gain = 1};
    cc_ipa *ipa;
    OK(create_controller(&o, &ipa));
    OK(cc_ipa_configure(ipa, &c));
    OK(cc_ipa_start(ipa));
    cc_ipa_request request = {.timing = c.timing, .timing_limits = c.timing_limits};
    unsigned stable = 0;
    for (unsigned iteration = 0; iteration < 100; ++iteration) {
        cc_ipa_settings s;
        OK(cc_ipa_prepare(ipa, &request, &s));
        uint32_t exposure = (uint32_t)s.sensor_controls[1].value;
        double gain = (double)s.sensor_controls[2].value / 16;
        uint64_t ns;
        OK(cc_sensor_exposure_time(c.sensor, &request.timing, exposure, &ns));
        double level = (double)ns * gain / (iteration < 50 ? 40000000 : 160000000);
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
        if (iteration == 50)
            stable = 0;
        stable = (m.flags & CC_IPA_AE_CONVERGED) ? stable + 1 : 0;
        CHECK(m.flags & CC_IPA_AWB_CONVERGED);
        if (iteration == 49 || iteration == 99)
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
    }
    live_geometry();
    profile();
    flicker();
    controller();
    feedback();
    puts("OV5647 native profile, geometry, flicker and still controller passed");
    return 0;
}
