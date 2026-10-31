/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include <ccamera/sensor.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <time.h>
#include <linux/videodev2.h>

/* Independent wide-integer oracle for production's portable 64-bit arithmetic. */
__extension__ typedef unsigned __int128 wide_uint;
static int reference(uint64_t a, uint64_t b, uint64_t d, int ceil, uint64_t *out)
{
    wide_uint product = (wide_uint)a * b;
    wide_uint q = product / d + (ceil && product % d != 0);
    if (q > UINT64_MAX) return -ERANGE;
    *out = (uint64_t)q;
    return 0;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    const char *names[] = {"imx219", "imx283", "imx290", "imx296", "imx477", "imx708",
        "ov5647", "ov64a40", "vd55g1", "ar0521", "imx708_wide_noir"};
    const cc_sensor_model *m;
    FUZZ_CHECK(cc_sensor_lookup(names[fuzz_u8(&in) % 11], &m) == 0);
    cc_sensor_timing t = {.pixel_rate = fuzz_u64(&in)};
    t.width = fuzz_u32(&in); t.height = fuzz_u32(&in);
    t.hblank = fuzz_u32(&in); t.vblank = fuzz_u32(&in);
    if (!(mode & 1U)) {
        t.pixel_rate = t.pixel_rate % UINT64_C(1000000000) + 1;
        t.width = t.width % 65536U + 1; t.height = t.height % 65536U + 1;
    }
    uint64_t exposure = fuzz_u64(&in), got = UINT64_MAX, expected = UINT64_MAX;
    uint32_t lines = fuzz_u32(&in), encoded = UINT32_MAX;
    uint64_t n = ((uint64_t)t.width + t.hblank) * UINT64_C(1000000000), d = t.pixel_rate;
    int valid = !t.width || !t.height || !d || d > INT64_MAX ? -EINVAL : d > n ? -ERANGE : 0;
    int error = valid ? valid : reference((uint64_t)t.height + t.vblank, n, d, 1, &expected);
    FUZZ_CHECK(cc_sensor_frame_duration(&t, &got) == error && got == expected);

    const cc_sensor_properties *props = cc_sensor_get_properties(m);
    bool fixed = !strcmp(props->model, "imx296");
    uint64_t offset = fixed ? 14260U : 0;
    if (fixed) { n = 4400000; d = 297; valid = 0; }
    if (!(props->features & CC_SENSOR_EXPOSURE)) valid = -ENOTSUP;
    expected = UINT64_MAX; got = UINT64_MAX;
    error = valid ? valid : reference(fixed && !lines ? 1U : lines, n, d, 1, &expected);
    if (!error) {
        if (expected > UINT64_MAX - offset) { error = -ERANGE; expected = UINT64_MAX; }
        else expected += offset;
    }
    FUZZ_CHECK(cc_sensor_exposure_time(m, fixed ? NULL : &t, lines, &got) == error && got == expected);
    uint64_t wanted = 0;
    error = valid ? valid : reference(exposure > offset ? exposure - offset : 0, d, n, 0, &wanted);
    if (!error && wanted > UINT32_MAX) error = -ERANGE;
    uint32_t expected_lines = error ? UINT32_MAX : (uint32_t)(fixed && !wanted ? 1U : wanted);
    FUZZ_CHECK(cc_sensor_exposure_lines(m, &t, exposure, &encoded) == error && encoded == expected_lines);

    uint64_t bits = fuzz_u64(&in);
    double gain;
    _Static_assert(sizeof(gain) == sizeof(bits), "fuzzer needs binary64 double");
    memcpy(&gain, &bits, sizeof(gain));
    if (!(mode & 2U)) gain = 1.0 + (double)(bits % 1000000U) / 256.0;
    uint32_t code = UINT32_MAX;
    if (!cc_sensor_gain_code(m, gain, &code)) {
        double decoded;
        uint32_t roundtrip;
        FUZZ_CHECK(cc_sensor_gain(m, code, &decoded) == 0 && decoded >= 1.0);
        FUZZ_CHECK(cc_sensor_gain_code(m, decoded, &roundtrip) == 0 && roundtrip == code);
    } else FUZZ_CHECK(code == UINT32_MAX);
    double decoded = -1;
    code = fuzz_u32(&in);
    if (!cc_sensor_gain(m, code, &decoded)) {
        uint32_t roundtrip;
        FUZZ_CHECK(cc_sensor_gain_code(m, decoded, &roundtrip) == 0 && roundtrip == code);
    } else FUZZ_CHECK(decoded == -1);

    cc_control_info range = {.id = V4L2_CID_ANALOGUE_GAIN, .type = V4L2_CTRL_TYPE_INTEGER64,
        .minimum = (int64_t)fuzz_u64(&in), .maximum = (int64_t)fuzz_u64(&in), .step = fuzz_u64(&in), .elems = 1};
    if (!(mode & 4U)) {
        range.minimum = (int64_t)((uint64_t)range.minimum % 256U);
        range.maximum = range.minimum + (int64_t)((uint64_t)range.maximum % 4096U);
        range.step = range.step % 256U + 1U;
    }
    cc_control control = {.id = 123, .value = 456};
    double actual = -1;
    if (!cc_sensor_quantize_gain(m, &range, gain, &control, &actual)) {
        FUZZ_CHECK(control.value >= range.minimum && control.value <= range.maximum);
        FUZZ_CHECK(((uint64_t)control.value - (uint64_t)range.minimum) % range.step == 0);
        FUZZ_CHECK(!control.data && !control.size && control.id == range.id);
        FUZZ_CHECK(cc_sensor_gain(m, (uint32_t)control.value, &decoded) == 0 && actual == decoded);
    } else FUZZ_CHECK(control.id == 123 && control.value == 456 && actual == -1);
    char name[128];
    fuzz_bytes(&in, name, sizeof(name) - 1); name[sizeof(name) - 1] = '\0';
    const cc_sensor_model *found = NULL;
    int ret = cc_sensor_lookup(name, &found);
    FUZZ_CHECK(ret <= 0 && (ret ? found == NULL : cc_sensor_get_properties(found) != NULL));
    cc_sensor_timing_limits limits={.pixel_rate=fuzz_u64(&in),.width=fuzz_u32(&in),.height=fuzz_u32(&in),
        .min_hblank=fuzz_u32(&in),.max_hblank=fuzz_u32(&in),.hblank_step=fuzz_u32(&in),
        .min_vblank=fuzz_u32(&in),.max_vblank=fuzz_u32(&in),.vblank_step=fuzz_u32(&in),
        .min_exposure=fuzz_u32(&in),.exposure_step=fuzz_u32(&in)};
    if (!(mode&8U)) {
        limits.pixel_rate=limits.pixel_rate%UINT64_C(1000000000)+1;
        limits.width=limits.width%4096U+64; limits.height=limits.height%4096U+64;
        limits.min_hblank%=4096; limits.max_hblank=limits.min_hblank+limits.max_hblank%65536U;
        limits.hblank_step=limits.hblank_step%64U+1;
        limits.min_vblank%=4096; limits.max_vblank=limits.min_vblank+limits.max_vblank%UINT32_C(16000000);
        limits.vblank_step=limits.vblank_step%128U+1;
        limits.min_exposure=limits.min_exposure%16U+1; limits.exposure_step=limits.exposure_step%16U+1;
    }
    uint64_t low=123,high=456;
    int bounds=cc_sensor_timing_bounds(&limits,&low,&high);
    if (bounds) FUZZ_CHECK(low==123 && high==456);
    else {
        uint32_t hb=limits.min_hblank+(limits.max_hblank-limits.min_hblank)/limits.hblank_step*limits.hblank_step;
        uint32_t vb=limits.min_vblank+(limits.max_vblank-limits.min_vblank)/limits.vblank_step*limits.vblank_step;
        FUZZ_CHECK(reference((uint64_t)limits.height+limits.min_vblank,((uint64_t)limits.width+limits.min_hblank)*UINT64_C(1000000000),
                             limits.pixel_rate,1,&expected)==0 && low==expected);
        FUZZ_CHECK(reference((uint64_t)limits.height+vb,((uint64_t)limits.width+hb)*UINT64_C(1000000000),
                             limits.pixel_rate,1,&expected)==0 && high==expected && high>=low);
    }
    uint64_t requested=fuzz_u64(&in),min_ns=fuzz_u64(&in),max_ns=fuzz_u64(&in);
    if (!(mode&16U) && !bounds) {
        requested=requested%(high ? high:1)+1;
        min_ns=low; max_ns=high;
    }
    cc_sensor_exposure_plan plan; memset(&plan,0xa5,sizeof(plan));
    cc_sensor_exposure_plan saved; memcpy(&saved,&plan,sizeof(plan));
    ret=cc_sensor_plan_exposure(m,&limits,requested,min_ns,max_ns,&plan);
    if (ret) FUZZ_CHECK(!memcmp(&plan,&saved,sizeof(plan)));
    else {
        FUZZ_CHECK(!bounds && plan.hblank>=limits.min_hblank && plan.hblank<=limits.max_hblank &&
                   (plan.hblank-limits.min_hblank)%limits.hblank_step==0);
        FUZZ_CHECK(plan.vblank>=limits.min_vblank && plan.vblank<=limits.max_vblank &&
                   (plan.vblank-limits.min_vblank)%limits.vblank_step==0);
        FUZZ_CHECK(plan.frame_length==(uint64_t)limits.height+plan.vblank && plan.exposure_lines>=limits.min_exposure &&
                   (plan.exposure_lines-limits.min_exposure)%limits.exposure_step==0 &&
                   (uint64_t)plan.exposure_lines+props->frame_integration_margin<=plan.frame_length);
        cc_sensor_timing actual_timing={limits.pixel_rate,limits.width,limits.height,plan.hblank,plan.vblank};
        FUZZ_CHECK(cc_sensor_exposure_time(m,&actual_timing,plan.exposure_lines,&expected)==0 && expected==plan.exposure_ns);
        FUZZ_CHECK(reference(plan.frame_length,((uint64_t)limits.width+plan.hblank)*UINT64_C(1000000000),limits.pixel_rate,1,&expected)==0 &&
                   expected==plan.frame_duration_ns);
        FUZZ_CHECK(plan.long_exposure_shift<=7);
        if (plan.long_exposure_shift) FUZZ_CHECK(plan.flags&CC_SENSOR_PLAN_LONG_EXPOSURE &&
            (plan.frame_length>>plan.long_exposure_shift)<=0xffdc && plan.frame_length%(1U<<plan.long_exposure_shift)==0);
    }
    return 0;
}
