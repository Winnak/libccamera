/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include <ccamera/ipa.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <linux/videodev2.h>
#include <errno.h>

static double binary_double(struct fuzz_input *in)
{
    uint64_t bits=fuzz_u64(in); double value;
    _Static_assert(sizeof(value)==sizeof(bits),"binary64 test inputs");
    memcpy(&value,&bits,sizeof(value)); return value;
}
static cc_control_info range(uint32_t id, int64_t low, int64_t high)
{
    return (cc_control_info) {.id=id,.type=V4L2_CTRL_TYPE_INTEGER,.minimum=low,.maximum=high,.step=1,.elems=1};
}
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in={data,size}; uint8_t mode=fuzz_u8(&in);
    union native { struct bcm2835_isp_stats vc4; struct pisp_statistics pisp; } native;
    memset(&native,0,sizeof(native));
    uint32_t fourcc=mode&1U ? CC_FOURCC('R','P','F','S') : CC_FOURCC('B','S','T','A');
    size_t native_size=mode&1U ? sizeof(native.pisp) : sizeof(native.vc4);
    if (mode&1U) {
        native.pisp.agc.histogram[fuzz_u16(&in)%1024U]=fuzz_u32(&in);
        native.pisp.awb.zones[0]=(struct pisp_awb_statistics_zone) {
            .R_sum=fuzz_u32(&in),.G_sum=fuzz_u32(&in),.B_sum=fuzz_u32(&in),.counted=fuzz_u32(&in)};
    } else {
        native.vc4.size=(uint32_t)sizeof(native.vc4);
        native.vc4.hist[0].g_hist[fuzz_u8(&in)%128U]=fuzz_u32(&in);
        native.vc4.awb_stats[0]=(struct bcm2835_isp_stats_region) {
            .r_sum=fuzz_u64(&in),.g_sum=fuzz_u64(&in),.b_sum=fuzz_u64(&in),.counted=fuzz_u32(&in)};
    }
    if (mode&2U) fuzz_patch(&in,&native,native_size);
    cc_ipa_statistics stats; memset(&stats,0xa5,sizeof(stats));
    cc_ipa_statistics original; memcpy(&original,&stats,sizeof(stats));
    int ret=cc_ipa_parse_statistics(fourcc,&native,native_size-(mode&4U ? 1U : 0U),&stats);
    if (ret<0) FUZZ_CHECK(!memcmp(&stats,&original,sizeof(stats)));
    else {
        if (stats.flags & CC_IPA_STATS_LUMA) FUZZ_CHECK(stats.luminance>=0 && stats.luminance<=1);
        if (stats.flags & CC_IPA_STATS_COLOUR) FUZZ_CHECK(stats.red>=0 && stats.red<=1 && stats.green>=0 && stats.green<=1 && stats.blue>=0 && stats.blue<=1);
    }
    cc_ipa_reference_tuning t={.flags=CC_IPA_REFERENCE_AE | CC_IPA_REFERENCE_AWB,
        .target_luminance=(1.0+fuzz_u8(&in))/257,.exposure_speed=(1.0+fuzz_u8(&in))/256,
        .white_balance_speed=(1.0+fuzz_u8(&in))/256,.min_colour_gain=0.5,.max_colour_gain=4};
    if (mode&8U) t.target_luminance=binary_double(&in);
    if (mode&32U) { t.min_frame_duration_ns=5000000; t.max_frame_duration_ns=20000000; }
    cc_ipa *ipa=NULL;
    ret=cc_ipa_reference_create(&t,&ipa);
    if (ret<0) { FUZZ_CHECK(ret==-EINVAL && !ipa); return 0; }
    cc_ipa_configuration c={.pipeline=CC_IPA_VC4,.statistics_fourcc=CC_FOURCC('B','S','T','A'),
        .timing={.pixel_rate=100000000,.width=640,.height=480,.hblank=360,.vblank=20},.num_sensor_controls=2};
    FUZZ_CHECK(cc_sensor_lookup("imx219",&c.sensor)==0);
    c.sensor_controls[0]=range(V4L2_CID_EXPOSURE,1,496);
    c.sensor_controls[1]=range(V4L2_CID_ANALOGUE_GAIN,0,240);
    c.sensor_controls[1].step=1U+fuzz_u8(&in)%8U;
    c.initial_controls[0]=(cc_control) {.id=V4L2_CID_EXPOSURE,.value=1U+fuzz_u16(&in)%496U};
    c.initial_controls[1]=(cc_control) {.id=V4L2_CID_ANALOGUE_GAIN,.value=fuzz_u8(&in)%241U};
    c.red_balance=range(V4L2_CID_RED_BALANCE,1,8000); c.blue_balance=range(V4L2_CID_BLUE_BALANCE,1,8000);
    c.digital_gain=range(V4L2_CID_DIGITAL_GAIN,1,8000);
    if (mode&32U) {
        c.num_sensor_controls=3;
        c.sensor_controls[2]=range(V4L2_CID_VBLANK,20,1520);
        c.initial_controls[2]=(cc_control){.id=V4L2_CID_VBLANK,.value=20};
        c.sensor_delays[0]=(cc_sensor_control_delay){V4L2_CID_EXPOSURE,2,0};
        c.sensor_delays[1]=(cc_sensor_control_delay){V4L2_CID_ANALOGUE_GAIN,1,0};
        c.sensor_delays[2]=(cc_sensor_control_delay){V4L2_CID_VBLANK,2,CC_SENSOR_CONTROL_PRIORITY};
        c.timing_limits=(cc_sensor_timing_limits){100000000,640,480,360,360,1,20,1520,1,1,1};
    }
    FUZZ_CHECK(cc_ipa_configure(ipa,&c)==0 && cc_ipa_start(ipa)==0);
    unsigned steps=1U+fuzz_u8(&in)%16U;
    for (unsigned step=0;step<steps;++step) {
        cc_ipa_settings settings; cc_ipa_request request={.sequence=step};
        FUZZ_CHECK(cc_ipa_prepare(ipa,&request,&settings)==0 && settings.num_sensor_controls==c.num_sensor_controls);
        int64_t lines=settings.sensor_controls[0].value, code=settings.sensor_controls[1].value;
        FUZZ_CHECK(lines>=1 && lines<=(mode&32U ? 1996:496) && code>=0 && code<=240 && (uint64_t)code%c.sensor_controls[1].step==0);
        cc_ipa_frame frame={.statistics={.flags=CC_IPA_STATS_LUMA | CC_IPA_STATS_COLOUR,
            .luminance=(double)fuzz_u8(&in)/255,.red=(1.0+fuzz_u8(&in))/256,
            .green=(1.0+fuzz_u8(&in))/256,.blue=(1.0+fuzz_u8(&in))/256},
            .isp_controls=settings.isp_controls,.num_isp_controls=settings.num_isp_controls};
        frame.sensor.count=settings.num_sensor_controls;
        memcpy(frame.sensor.controls,settings.sensor_controls,frame.sensor.count*sizeof(cc_control));
        if (mode&16U) frame.statistics.luminance=binary_double(&in);
        cc_ipa_metadata metadata={.exposure_ns=123};
        ret=cc_ipa_process(ipa,&frame,&metadata);
        if (ret<0) FUZZ_CHECK(ret==-EPROTO && metadata.exposure_ns==123);
        else {
            FUZZ_CHECK(metadata.exposure_ns==(uint64_t)lines*10000);
            FUZZ_CHECK(metadata.analogue_gain==256.0/(256.0-(double)code));
            FUZZ_CHECK(metadata.red_gain>0 && metadata.blue_gain>0);
        }
    }
    cc_ipa_destroy(ipa);
    return 0;
}
