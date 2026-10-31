/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/sensor.h>
#include <errno.h>
#include <float.h>
#include <time.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)

static const cc_sensor_model *lookup(const char *name)
{
    const cc_sensor_model *model;
    OK(cc_sensor_lookup(name, &model)); CHECK(model);
    return model;
}
static void test_properties(void)
{
    const cc_sensor_model *m = lookup("imx219 10-0010");
    const cc_sensor_properties *p = cc_sensor_get_properties(m);
    CHECK(!strcmp(p->model, "imx219") && p->cell_width_nm == 1120 && p->frame_integration_margin == 4);
    CHECK(p->num_delays == 4 && p->delays[0].id == V4L2_CID_EXPOSURE && p->delays[0].frames == 2);
    CHECK(p->delays[1].id == V4L2_CID_ANALOGUE_GAIN && p->delays[1].frames == 1);
    CHECK(p->delays[2].id == V4L2_CID_VBLANK && p->delays[2].flags == CC_SENSOR_CONTROL_PRIORITY);
    p = cc_sensor_get_properties(lookup("imx477"));
    CHECK(p->cell_width_nm == 1550 && p->frame_integration_margin == 22 && p->delays[1].frames == 2);
    CHECK(p->delays[2].frames == 3 && p->delays[3].frames == 3);
    p = cc_sensor_get_properties(lookup("imx327"));
    CHECK(p->features & CC_SENSOR_GAIN && !(p->features & CC_SENSOR_DELAYS) && !p->num_delays);
    p = cc_sensor_get_properties(lookup("ar0521"));
    CHECK(p->features == CC_SENSOR_CELL_SIZE && p->cell_width_nm == 2200);
    double gain = 123;
    CHECK(cc_sensor_gain(lookup("ar0521"), 0, &gain) == -ENOTSUP && gain == 123);
    p = cc_sensor_get_properties(lookup("ov5647")); CHECK(p->features & CC_SENSOR_DRIVER_DEPENDENT_DELAYS);
    CHECK(lookup("imx708_wide_noir 0-001a") == lookup("imx708"));
    const char *bad[] = {"unknown", "x-imx219", "imx219bad", "imx219_noir", "imx708_wide_noir2"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        m = (const cc_sensor_model *)(uintptr_t)1;
        CHECK(cc_sensor_lookup(bad[i], &m) == -ENOENT && !m);
    }
}
/* Independent series evaluation of the exponential gain equation, used only
 * in tests. Production uses checked-in binary64 values and needs no libm. */
static double exponential(double x)
{
    double sum = 1, term = 1;
    for (unsigned i = 1; i < 100; ++i) { term *= x / i; sum += term; }
    return sum;
}
static void test_gain_models(void)
{
    const struct {const char *name; unsigned family, parameter;} cases[] = {
        {"imx219",0,256},{"imx283",0,2048},{"imx477",0,1024},{"imx519",0,1024},{"imx708",0,1024},
        {"vd55g1",0,32},{"vd56g3",0,32},{"ov5647",1,16},{"ov7251",1,16},{"ov9281",1,16},{"ov64a40",1,128},
        {"imx290",2,240},{"imx327",2,240},{"imx462",2,240},{"imx415",2,240},{"imx678",2,240},{"imx296",3,239}
    };
    for (size_t n = 0; n < sizeof(cases) / sizeof(cases[0]); ++n) {
        const cc_sensor_model *m = lookup(cases[n].name);
        unsigned family = cases[n].family, parameter = cases[n].parameter;
        uint32_t first = family == 1 ? parameter : 0;
        uint32_t last = family == 0 ? parameter - 1U : family == 1 ? 4096 : parameter;
        for (uint32_t code = first; code <= last; ++code) {
            double gain, expected;
            OK(cc_sensor_gain(m, code, &gain));
            if (family == 0) expected = (double)parameter / (parameter - code);
            else if (family == 1) expected = (double)code / parameter;
            else expected = exponential(2.302585092994045684 * code * (family == 2 ? 0.015 : 0.005));
            CHECK(gain > expected * (1 - 2e-14) && gain < expected * (1 + 2e-14));
            uint32_t encoded;
            OK(cc_sensor_gain_code(m, gain, &encoded)); CHECK(encoded == code);
            if (code < last) {
                double next;
                OK(cc_sensor_gain(m, code + 1U, &next));
                OK(cc_sensor_gain_code(m, (gain + next) / 2, &encoded)); CHECK(encoded == code);
            }
        }
        uint32_t encoded = 123;
        CHECK(cc_sensor_gain_code(m, NAN, &encoded) == -EINVAL && encoded == 123);
        CHECK(cc_sensor_gain_code(m, INFINITY, &encoded) == -EINVAL && encoded == 123);
        CHECK(cc_sensor_gain_code(m, 0.9, &encoded) == -EINVAL && encoded == 123);
        OK(cc_sensor_gain_code(m, 2, &encoded));
        CHECK(encoded == (family == 0 ? parameter / 2U : family == 1 ? parameter * 2U : family == 2 ? 20U : 60U));
        if (family != 1) {
            double value = 999;
            CHECK(cc_sensor_gain(m, last + 1U, &value) == -ERANGE && value == 999);
        }
    }
}
static void test_quantization(void)
{
    const cc_sensor_model *m = lookup("imx219");
    cc_control_info limits = {.id=V4L2_CID_ANALOGUE_GAIN, .type=V4L2_CTRL_TYPE_INTEGER,
        .minimum=16, .maximum=232, .step=3, .elems=1};
    cc_control control; double actual;
    OK(cc_sensor_quantize_gain(m, &limits, 2, &control, &actual));
    CHECK(control.value == 127 && actual == 256.0 / 129.0 && !control.data && !control.size);
    OK(cc_sensor_quantize_gain(m, &limits, 1, &control, &actual)); CHECK(control.value == 16);
    OK(cc_sensor_quantize_gain(m, &limits, DBL_MAX, &control, &actual)); CHECK(control.value == 232);
    for (int test = 0; test < 6; ++test) {
        cc_control_info bad = limits;
        if (test == 0) bad.step = 0;
        if (test == 1) bad.minimum = -1;
        if (test == 2) bad.minimum = 300;
        if (test == 3) bad.type = V4L2_CTRL_TYPE_STRING;
        if (test == 4) bad.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD;
        if (test == 5) { bad.minimum = 256; bad.maximum = 300; }
        control = (cc_control) {.id=123,.value=123}; actual = 123;
        CHECK(cc_sensor_quantize_gain(m, &bad, 2, &control, &actual) < 0);
        CHECK(control.id == 123 && control.value == 123 && actual == 123);
    }
    limits = (cc_control_info) {.id=V4L2_CID_ANALOGUE_GAIN,.type=V4L2_CTRL_TYPE_INTEGER64,
        .minimum=0,.maximum=UINT32_MAX,.step=32,.elems=1};
    OK(cc_sensor_quantize_gain(lookup("ov5647"), &limits, 1, &control, &actual));
    CHECK(control.value == 32 && actual == 2);
    limits.step = UINT64_MAX;
    CHECK(cc_sensor_quantize_gain(lookup("ov5647"), &limits, 1, &control, &actual) == -ERANGE);
}
struct timing_vector {
    cc_sensor_timing timing;
    uint64_t requested;
    uint32_t lines;
    int frame_error, encode_error, decode_error;
    uint64_t frame_ns, encoded, decoded_ns;
};
#include "sensor_timing_vectors.h"
static void test_timing(void)
{
    const cc_sensor_model *m = lookup("imx219");
    cc_sensor_timing timing = {.pixel_rate=96000000,.width=1920,.height=1080,.hblank=280,.vblank=45};
    uint64_t duration;
    uint32_t lines;
    OK(cc_sensor_frame_duration(&timing, &duration)); CHECK(duration == 25781250);
    OK(cc_sensor_exposure_lines(m, &timing, 10000000, &lines)); CHECK(lines == 436);
    OK(cc_sensor_exposure_time(m, &timing, lines, &duration)); CHECK(duration == 9991667);
    OK(cc_sensor_exposure_lines(m, &timing, duration, &lines)); CHECK(lines == 436);
    const cc_sensor_model *global = lookup("imx296");
    OK(cc_sensor_exposure_lines(global, NULL, 0, &lines)); CHECK(lines == 1);
    OK(cc_sensor_exposure_time(global, NULL, 1, &duration)); CHECK(duration == 29075);
    for (uint32_t i = 1; i < 1000; ++i) {
        OK(cc_sensor_exposure_time(global, NULL, i, &duration));
        OK(cc_sensor_exposure_lines(global, NULL, duration, &lines)); CHECK(lines == i);
    }
    for (size_t i = 0; i < sizeof(timing_vectors) / sizeof(timing_vectors[0]); ++i) {
        const struct timing_vector *v = &timing_vectors[i];
        duration = 123; lines = 123;
        CHECK(cc_sensor_frame_duration(&v->timing, &duration) == -v->frame_error);
        CHECK(duration == (v->frame_error ? 123 : v->frame_ns));
        CHECK(cc_sensor_exposure_lines(m, &v->timing, v->requested, &lines) == -v->encode_error);
        CHECK(lines == (v->encode_error ? 123 : v->encoded));
        duration = 123;
        CHECK(cc_sensor_exposure_time(m, &v->timing, v->lines, &duration) == -v->decode_error);
        CHECK(duration == (v->decode_error ? 123 : v->decoded_ns));
    }
    timing.pixel_rate = 0; duration = 123;
    CHECK(cc_sensor_frame_duration(&timing, &duration) == -EINVAL && duration == 123);
}
struct blanking_vector { const char *model; uint64_t requested; uint32_t exposure,hblank,vblank; };
#include "sensor_blanking_vectors.h"
static void test_planning(void)
{
    cc_sensor_timing_limits l={.pixel_rate=100000000,.width=640,.height=480,
        .min_hblank=360,.max_hblank=360,.hblank_step=1,.min_vblank=20,.max_vblank=9999520,.vblank_step=1,
        .min_exposure=1,.exposure_step=1};
    uint64_t low,high;
    OK(cc_sensor_timing_bounds(&l,&low,&high)); CHECK(low==5000000 && high==UINT64_C(100000000000));
    for (size_t i=0;i<sizeof(blanking_vectors)/sizeof(blanking_vectors[0]);++i) {
        const struct blanking_vector *v=&blanking_vectors[i];
        cc_sensor_exposure_plan p;
        OK(cc_sensor_plan_exposure(lookup(v->model),&l,v->requested,low,high,&p));
        CHECK(p.exposure_lines==v->exposure && p.hblank==v->hblank && p.vblank==v->vblank);
        CHECK(p.frame_length==l.height+p.vblank && p.frame_duration_ns==(uint64_t)p.frame_length*10000);
        CHECK(p.exposure_ns==(uint64_t)p.exposure_lines*10000 && p.exposure_ns<=v->requested);
        if (p.long_exposure_shift) {
            CHECK(p.long_exposure_shift<=7 && p.frame_length%(1U<<p.long_exposure_shift)==0);
            CHECK(p.frame_length>>p.long_exposure_shift<=0xffdc);
            CHECK(p.flags&CC_SENSOR_PLAN_LONG_EXPOSURE);
        }
    }
    /* Horizontal stretching and non-unit native grids are not represented in
     * the floating reference. Check the native outputs against exact equations. */
    l.max_hblank=1360; l.max_vblank=20;
    cc_sensor_exposure_plan p;
    OK(cc_sensor_plan_exposure(lookup("imx219"),&l,8000000,5000000,10000000,&p));
    CHECK(p.hblank==968 && p.vblank==20 && p.exposure_lines==496);
    CHECK(p.frame_duration_ns==8040000 && p.exposure_ns==7975680);
    l.hblank_step=13; l.vblank_step=3; l.max_vblank=23; l.min_exposure=2; l.exposure_step=7;
    OK(cc_sensor_plan_exposure(lookup("imx219"),&l,8000000,5000000,10000000,&p));
    CHECK((p.hblank-l.min_hblank)%13==0 && (p.vblank-l.min_vblank)%3==0 && (p.exposure_lines-2)%7==0);
    CHECK(p.exposure_ns<=8000000 && p.exposure_lines+4<=p.frame_length);
    cc_sensor_exposure_plan saved=p;
    CHECK(cc_sensor_plan_exposure(lookup("ar0521"),&l,8000000,0,0,&p)==-ENOTSUP);
    CHECK(!memcmp(&p,&saved,sizeof(p)));
    CHECK(cc_sensor_plan_exposure(lookup("imx219"),&l,1,2,1,&p)==-EINVAL);
    l.vblank_step=0; CHECK(cc_sensor_plan_exposure(lookup("imx219"),&l,1,0,0,&p)==-EINVAL);
    CHECK(!memcmp(&p,&saved,sizeof(p)));
    low=123; high=456; CHECK(cc_sensor_timing_bounds(&l,&low,&high)==-EINVAL && low==123 && high==456);
    l.vblank_step=1; l.max_vblank=UINT32_MAX;
    CHECK(cc_sensor_timing_bounds(&l,&low,&high)==-ERANGE && low==123 && high==456);
    l=(cc_sensor_timing_limits){1,1,1,0,0,1,4,UINT32_MAX-1,1,1,1};
    CHECK(cc_sensor_timing_bounds(&l,&low,&high)==0 && high==UINT64_C(4294967295000000000));
    OK(cc_sensor_plan_exposure(lookup("imx219"),&l,UINT64_MAX,0,0,&p));
    CHECK(p.frame_length==UINT32_MAX && p.exposure_lines==UINT32_MAX-4);
    OK(cc_sensor_plan_exposure(lookup("imx296"),&l,UINT64_MAX,0,0,&p));
    CHECK(p.frame_length==UINT32_MAX && p.exposure_lines==UINT32_MAX-4);
    /* A shifted frame must be representable on the driver's VBLANK grid. */
    l=(cc_sensor_timing_limits){100000000,640,480,360,360,1,20,1000000,3,1,1};
    OK(cc_sensor_plan_exposure(lookup("imx708"),&l,UINT64_C(9000000000),0,0,&p));
    CHECK((p.vblank-20)%3==0 && p.frame_length%(1U<<p.long_exposure_shift)==0);
    l.min_vblank=21; l.max_vblank=65521; l.vblank_step=2;
    CHECK(cc_sensor_plan_exposure(lookup("imx708"),&l,UINT64_C(700000000),0,0,&p)==-ERANGE);
    /* IMX296's fixed shutter equation remains distinct from frame line timing. */
    l=(cc_sensor_timing_limits){100000000,640,480,360,360,1,20,2000,1,1,1};
    OK(cc_sensor_plan_exposure(lookup("imx296"),&l,1000000,0,0,&p));
    CHECK(p.exposure_lines==66 && p.exposure_ns==992038 && p.frame_length==500);
}
static void test_mode_policy(void)
{
    const char *names[]={"imx219","imx283","imx290","imx296","imx327","imx415","imx462","imx477","imx519",
        "imx678","imx708","ov5647","ov64a40","ov7251","ov9281","vd55g1","vd56g3"};
    cc_sensor_mode_description mode={4608,2592,1,1,31250000};
    for (size_t i=0;i<sizeof(names)/sizeof(names[0]);++i) {
        cc_sensor_mode_policy p;
        OK(cc_sensor_get_mode_policy(lookup(names[i]),&mode,&p));
        uint32_t hidden=!strcmp(names[i],"ov5647") ? 2U:
            (!strcmp(names[i],"imx290") || !strcmp(names[i],"imx327") || !strcmp(names[i],"imx462") || !strcmp(names[i],"imx415")) ? 1U:0U;
        CHECK(p.sensitivity==1 && p.hide_startup==hidden && p.mistrust_startup==(!strcmp(names[i],"ov5647") ? 2U:1U));
        CHECK(p.hide_mode_switch==(!strcmp(names[i],"imx283") ? 1U:hidden));
        CHECK(p.mistrust_mode_switch==(!strcmp(names[i],"ov5647") ? 2U:!strcmp(names[i],"imx219") ? 1U:0U));
        CHECK(p.min_debin_factor==(!strcmp(names[i],"imx708") ? 4U:2U));
    }
    cc_sensor_mode_policy p;
    mode=(cc_sensor_mode_description){2304,1296,2,2,31250000};
    OK(cc_sensor_get_mode_policy(lookup("imx708_wide"),&mode,&p)); CHECK(p.sensitivity==2 && !p.hide_startup);
    ++mode.min_frame_duration_ns;
    OK(cc_sensor_get_mode_policy(lookup("imx708"),&mode,&p)); CHECK(p.hide_startup==1 && p.hide_mode_switch==1);
    for (uint32_t bin=1;bin<=2;++bin) for (uint32_t scale=1;scale<=4;++scale) {
        mode.binning_x=bin; mode.scale_x=scale;
        OK(cc_sensor_get_mode_policy(lookup("ov64a40"),&mode,&p));
        CHECK(p.sensitivity==(bin==1 ? 1:scale>=4 ? 4:scale>=2 ? 2:1));
    }
    cc_sensor_mode_policy saved=p;
    mode.scale_x=NAN;
    CHECK(cc_sensor_get_mode_policy(lookup("imx708"),&mode,&p)==-EINVAL && !memcmp(&p,&saved,sizeof(p)));
}
int main(void)
{
    test_properties(); test_gain_models(); test_quantization(); test_timing(); test_planning(); test_mode_policy();
    puts("Sensor profiles, gain equations/quantization and exact checked shutter/frame timing passed");
    return 0;
}
