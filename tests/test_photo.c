/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/capture.h>
#include <ccamera/ipa_proxy.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
static const char *worker;
static cc_control_info range(uint32_t id, int64_t lo, int64_t hi)
{
    return (cc_control_info){.id=id,.type=V4L2_CTRL_TYPE_INTEGER,.minimum=lo,.maximum=hi,.step=1,.elems=1};
}
static cc_ipa_configuration configuration(enum cc_vc4_profile profile, bool pisp)
{
    const cc_vc4_profile_info *info=cc_vc4_get_profile_info(profile);
    cc_ipa_configuration c={.pipeline=pisp ? CC_IPA_PISP:CC_IPA_VC4,
        .statistics_fourcc=pisp ? CC_FOURCC('R','P','F','S'):CC_FOURCC('B','S','T','A'),
        .source={.fourcc=pisp ? V4L2_PIX_FMT_SRGGB16:V4L2_PIX_FMT_SRGGB10P,.width=1024,.height=768,
            .num_planes=1,.planes={{2048,2048*768}}},
        .sensor_format={.fourcc=V4L2_PIX_FMT_SRGGB10P,.width=1024,.height=768},
        .geometry={.active={0,0,1024,768},.crop={0,0,1024,768},.width=1024,.height=768},
        .timing={100000000,1024,768,976,232},.timing_limits={100000000,1024,768,976,976,1,32,16000,1,4,1},
        .num_sensor_controls=3};
    OK(cc_sensor_lookup(info->sensor_model,&c.sensor));
    c.sensor_controls[0]=range(V4L2_CID_EXPOSURE,4,996);
    c.sensor_controls[1]=range(V4L2_CID_ANALOGUE_GAIN,profile<=CC_VC4_PROFILE_OV5647_NOIR ? 16:0,
        profile<=CC_VC4_PROFILE_OV5647_NOIR ? 1023:profile<=CC_VC4_PROFILE_IMX219_NOIR ? 232:960);
    c.sensor_controls[2]=range(V4L2_CID_VBLANK,32,16000);
    for (unsigned i=0; i<3; ++i) {
        c.initial_controls[i]=(cc_control){.id=c.sensor_controls[i].id,.value=c.sensor_controls[i].minimum};
        c.sensor_delays[i]=(cc_sensor_control_delay){c.initial_controls[i].id,2,i==2 ? CC_SENSOR_CONTROL_PRIORITY:0};
    }
    c.initial_controls[0].value=500; c.initial_controls[2].value=232;
    c.red_balance=range(V4L2_CID_RED_BALANCE,1,8000);
    c.blue_balance=range(V4L2_CID_BLUE_BALANCE,1,8000);
    c.digital_gain=range(V4L2_CID_DIGITAL_GAIN,1,8000);
    c.lens=range(V4L2_CID_FOCUS_ABSOLUTE,0,1023);
    return c;
}
static cc_ipa *create(enum cc_vc4_profile profile, bool pisp, bool af, cc_ipa_configuration *c)
{
    cc_ipa_vc4_options o={.profile=profile,.autofocus=af,.red_gain=1,.blue_gain=1};
    cc_ipa_proxy_options proxy={.worker_path=worker}; cc_ipa *ipa=NULL;
    int ret=worker ? (pisp ? cc_ipa_pisp_create_proxy(&o,&proxy,&ipa):cc_ipa_vc4_create_proxy(&o,&proxy,&ipa)):
        (pisp ? cc_ipa_pisp_create(&o,&ipa):cc_ipa_vc4_create(&o,&ipa));
    OK(ret);
    cc_photo_capabilities caps={.controls=123};
    CHECK(cc_ipa_get_photo_capabilities(ipa,&caps)==-EPIPE && caps.controls==123);
    ret=cc_ipa_configure(ipa,c);
    if (ret) fprintf(stderr,"profile %u pisp %d af %d configure: %d\n",(unsigned)profile,pisp,af,ret);
    OK(ret); OK(cc_ipa_start(ipa)); return ipa;
}
struct scene { struct bcm2835_isp_stats vc4; struct pisp_statistics pisp; };
static cc_ipa_frame observation(const cc_ipa_configuration *c, const cc_ipa_settings *s, struct scene *scene, double luma)
{
    memset(scene,0,sizeof(*scene)); scene->vc4.size=sizeof(scene->vc4);
    unsigned bin=(unsigned)(luma*128); if (bin>127) bin=127;
    scene->vc4.hist[0].g_hist[bin]=1000;
    bin=(unsigned)(luma*1024); if (bin>1023) bin=1023;
    scene->pisp.agc.histogram[bin]=1000;
    for (unsigned i=0; i<192; ++i) scene->vc4.awb_stats[i]=(struct bcm2835_isp_stats_region){
        .counted=100,.r_sum=(uint64_t)(luma*8192*100),.g_sum=(uint64_t)(luma*8192*100),.b_sum=(uint64_t)(luma*8192*100)};
    for (unsigned i=0; i<15; ++i) scene->vc4.agc_stats[i]=scene->vc4.awb_stats[i];
    for (unsigned i=0; i<1024; ++i) scene->pisp.awb.zones[i]=(struct pisp_awb_statistics_zone){.counted=100,
        .R_sum=(uint32_t)(luma*65536*100),.G_sum=(uint32_t)(luma*65536*100),.B_sum=(uint32_t)(luma*65536*100)};
    bool pisp=c->pipeline==CC_IPA_PISP;
    cc_ipa_frame f={.statistics_fourcc=c->statistics_fourcc,
        .statistics_data=pisp ? (const void *)&scene->pisp:(const void *)&scene->vc4,
        .statistics_size=pisp ? sizeof(scene->pisp):sizeof(scene->vc4),.vc4=pisp ? NULL:&s->vc4,
        .backend=&s->backend,.frontend=&s->frontend,.colour_temperature=s->colour_temperature,
        .isp_controls=s->isp_controls,.num_isp_controls=s->num_isp_controls,
        .settings_sensor_controls=s->sensor_controls,.num_settings_sensor_controls=s->num_sensor_controls,
        .af=s->af,.photo=s->photo};
    f.sensor.count=s->num_sensor_controls;
    memcpy(f.sensor.controls,s->sensor_controls,s->num_sensor_controls*sizeof(cc_control)); return f;
}
static void controls(enum cc_vc4_profile profile, bool pisp)
{
    cc_ipa_configuration c=configuration(profile,pisp); cc_ipa *ipa=create(profile,pisp,false,&c);
    cc_photo_capabilities caps; OK(cc_ipa_get_photo_capabilities(ipa,&caps));
    CHECK(caps.controls==CC_PHOTO_ALL && caps.profile==profile && caps.metering_modes==7 && caps.denoise_modes==3);
    CHECK(caps.min_exposure_ns==80000 && caps.min_frame_duration_ns==16000000 && caps.max_exposure_ns>300000000);
    CHECK(caps.analogue_gain.minimum==1 && caps.analogue_gain.maximum>=10 && caps.num_delays==3);
    CHECK(caps.policy.hide_startup==(profile<=CC_VC4_PROFILE_OV5647_NOIR ? 2U:0U));
    CHECK(!!(caps.flags & CC_PHOTO_CAP_PROVISIONAL_DELAYS)==(profile<=CC_VC4_PROFILE_OV5647_NOIR));
    cc_photo_controls photo={.flags=CC_PHOTO_ALL,.exposure_ns=12345678,.analogue_gain=2.3,.exposure_value=1.5,
        .metering=CC_PHOTO_MATRIX,.white_balance=CC_PHOTO_WB_MANUAL,.red_gain=1.5,.blue_gain=2,
        .brightness=0.1,.contrast=0.5,.saturation=0,.sharpness=0,.denoise=CC_PHOTO_DENOISE_OFF,.mains_hz=60,
        .min_frame_duration_ns=20000000,.max_frame_duration_ns=20000000};
    cc_ipa_request r={.timing=c.timing,.timing_limits=c.timing_limits,.photo=&photo};
    cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&r,&s));
    CHECK(s.sensor_controls[1].value==617 && s.photo.mains_hz==60 && s.photo.metering==CC_PHOTO_MATRIX);
    double native_gain; OK(cc_sensor_gain(c.sensor,(uint32_t)s.sensor_controls[2].value,&native_gain));
    CHECK(native_gain<=2.3 && native_gain>2.2);
    if (pisp) {
        CHECK(!(s.backend.global.bayer_enables & (PISP_BE_BAYER_ENABLE_SDN|PISP_BE_BAYER_ENABLE_CDN)));
        CHECK(!(s.backend.global.rgb_enables & PISP_BE_RGB_ENABLE_SHARPEN));
        CHECK(s.backend.ccm.coeffs[0]==s.backend.ccm.coeffs[3] && s.backend.ccm.coeffs[3]==s.backend.ccm.coeffs[6]);
        CHECK((s.backend.gamma.lut[0]&65535U)>20000);
        for (unsigned y=0; y<15; ++y) for (unsigned x=0; x<15; ++x)
            CHECK(((s.frontend.agc_stats.weights[y*8+x/2]>>(4*(x&1U)))&15U)==1);
    } else {
        CHECK(!s.vc4.denoise.enabled && !s.vc4.cdn.enabled && !s.vc4.sharpen.enabled);
        CHECK(s.vc4.gamma.y[0]>20000 && s.vc4.gamma.y[32]<60000);
        CHECK(s.vc4.ccm.ccm.ccm[0][0].num==s.vc4.ccm.ccm.ccm[1][0].num);
    }
    struct scene scene; cc_ipa_frame f=observation(&c,&s,&scene,0.2); cc_ipa_metadata m;
    OK(cc_ipa_process(ipa,&f,&m));
    CHECK((m.flags & CC_IPA_METADATA_PHOTO) && m.exposure_ns==12340000 && m.analogue_gain==native_gain);
    CHECK(m.photo.exposure_ns==m.exposure_ns && m.photo.analogue_gain==native_gain && m.photo.exposure_value==1.5);
    CHECK(m.photo_status==(CC_PHOTO_EXPECTED_SENSOR|CC_PHOTO_STATS_TRUSTED));
    CHECK((m.flags & CC_IPA_AE_CONVERGED) && (m.flags & CC_IPA_AWB_CONVERGED) && m.frame_duration_ns==20000000);
    CHECK(fabs(m.red_gain-1.5)<0.002 && fabs(m.blue_gain-2)<0.002);
    f.untrusted=1; OK(cc_ipa_process(ipa,&f,&m));
    CHECK(m.photo.exposure_ns==12340000 && !(m.photo_status & CC_PHOTO_STATS_TRUSTED));
    CHECK(!(m.flags & (CC_IPA_AE_CONVERGED|CC_IPA_AWB_CONVERGED)));
    f.untrusted=0; f.sensor.controls[1].value=600;
    OK(cc_ipa_process(ipa,&f,&m)); CHECK(m.exposure_ns==12000000 && !(m.flags & CC_IPA_AE_CONVERGED));
    f.photo_overrides=CC_PHOTO_CONTRAST|CC_PHOTO_EXPOSURE;
    OK(cc_ipa_process(ipa,&f,&m)); CHECK((m.photo_status & CC_PHOTO_NATIVE_OVERRIDE) && !(m.photo.flags & CC_PHOTO_CONTRAST));
    /* A later prepare must never replace the frame's WB/IQ/EV snapshot. */
    cc_ipa_settings newer; photo.brightness=-0.2; photo.exposure_value=-1;
    OK(cc_ipa_prepare(ipa,&r,&newer)); OK(cc_ipa_process(ipa,&f,&m));
    CHECK(m.photo.brightness==0.1 && m.photo.exposure_value==1.5);
    /* Native sensor commands take precedence over normalized manual values. */
    cc_control overrides[]={{.id=V4L2_CID_EXPOSURE,.value=800},{.id=V4L2_CID_ANALOGUE_GAIN,.value=c.sensor_controls[1].minimum}};
    r.sensor_controls=overrides; r.num_sensor_controls=2;
    OK(cc_ipa_prepare(ipa,&r,&s)); CHECK(s.sensor_controls[1].value==800 && s.sensor_controls[2].value==overrides[1].value);
    r.sensor_controls=NULL; r.num_sensor_controls=0;
    r.timing.hblank+=1; CHECK(cc_ipa_prepare(ipa,&r,&s)==-ENOTSUP); r.timing=c.timing;
    photo.exposure_ns=1; CHECK(cc_ipa_prepare(ipa,&r,&s)==-ERANGE);
    photo.exposure_ns=12345678; photo.analogue_gain=64;
    CHECK(cc_ipa_prepare(ipa,&r,&s)==-ERANGE);
    photo.analogue_gain=2; photo.flags|=1U<<31; CHECK(cc_ipa_prepare(ipa,&r,&s)==-ENOTSUP);
    photo.flags=CC_PHOTO_ALL;
    bool noir=!!(cc_vc4_get_profile_info(profile)->flags & CC_VC4_PROFILE_NOIR);
    for (unsigned mode=0; mode<=CC_PHOTO_WB_MANUAL; ++mode) {
        photo.white_balance=(enum cc_photo_white_balance)mode;
        int ret=cc_ipa_prepare(ipa,&r,&s);
        if (noir && mode!=CC_PHOTO_WB_AUTO && mode!=CC_PHOTO_WB_MANUAL) CHECK(ret==-ENOTSUP);
        else {
            OK(ret); f=observation(&c,&s,&scene,0.2); OK(cc_ipa_process(ipa,&f,&m));
            CHECK(m.photo.white_balance==(enum cc_photo_white_balance)mode);
            if (!noir && mode<CC_PHOTO_WB_MANUAL) {
                CHECK(m.colour_temperature>=caps.white_balance_temperature[mode].minimum &&
                    m.colour_temperature<=caps.white_balance_temperature[mode].maximum);
            }
        }
    }
    /* Clearing the commands restores all defaults, including calibrated NR. */
    r.photo=NULL; OK(cc_ipa_prepare(ipa,&r,&s));
    CHECK(s.photo.exposure_ns==0 && s.photo.analogue_gain==0 && s.photo.exposure_value==0 && s.photo.sharpness==1);
    CHECK(s.photo.denoise==CC_PHOTO_DENOISE_NORMAL && s.photo.white_balance==CC_PHOTO_WB_MANUAL);
    cc_ipa_stop(ipa); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    OK(cc_ipa_prepare(ipa,&r,&s)); CHECK(s.photo.brightness==0); cc_ipa_destroy(ipa);
}
static double ev_response(bool pisp, double ev)
{
    cc_ipa_configuration c=configuration(CC_VC4_PROFILE_OV5647,pisp);
    cc_ipa *ipa=create(CC_VC4_PROFILE_OV5647,pisp,false,&c);
    cc_photo_controls photo={.flags=CC_PHOTO_EXPOSURE|CC_PHOTO_EV,.exposure_ns=10000000,.exposure_value=ev};
    cc_ipa_request r={.timing=c.timing,.timing_limits=c.timing_limits,.photo=&photo};
    double gain=0; struct scene scene;
    for (unsigned i=0; i<80; ++i) {
        cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&r,&s));
        OK(cc_sensor_gain(c.sensor,(uint32_t)s.sensor_controls[2].value,&gain));
        cc_ipa_frame f=observation(&c,&s,&scene,gain/5); cc_ipa_metadata m;
        OK(cc_ipa_process(ipa,&f,&m));
        CHECK((m.photo_status & CC_PHOTO_AUTO_GAIN) && !(m.photo_status & CC_PHOTO_AUTO_EXPOSURE));
    }
    cc_ipa_destroy(ipa); return gain;
}
static void metering(bool pisp)
{
    cc_ipa_configuration c=configuration(CC_VC4_PROFILE_OV5647,pisp);
    cc_ipa *ipa=create(CC_VC4_PROFILE_OV5647,pisp,false,&c);
    cc_photo_controls photo={.flags=CC_PHOTO_METERING};
    cc_ipa_request r={.timing=c.timing,.timing_limits=c.timing_limits,.photo=&photo};
    double values[3]={0};
    for (unsigned i=0; i<3; ++i) {
        photo.metering=(enum cc_photo_metering)i; cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&r,&s));
        if (pisp && i==CC_PHOTO_SPOT) {
            CHECK(s.frontend.agc_stats.weights[0]==0 && (s.frontend.agc_stats.weights[7*8+3]&0xf0)==0x30);
        }
        struct scene scene; cc_ipa_frame f=observation(&c,&s,&scene,0.1);
        scene.vc4.agc_stats[0].r_sum=scene.vc4.agc_stats[0].g_sum=scene.vc4.agc_stats[0].b_sum=655300;
        cc_ipa_metadata m; OK(cc_ipa_process(ipa,&f,&m)); values[i]=m.luminance;
    }
    if (!pisp) CHECK(values[CC_PHOTO_SPOT]>values[CC_PHOTO_CENTRE_WEIGHTED] && values[CC_PHOTO_CENTRE_WEIGHTED]>values[CC_PHOTO_MATRIX]);
    cc_ipa_destroy(ipa);
}
static void autofocus(enum cc_vc4_profile profile, bool pisp, bool phase)
{
    cc_ipa_configuration c=configuration(profile,pisp);
    c.embedded=(cc_embedded_configuration){.bits_per_pixel=10,.format={.fourcc=CC_FOURCC('S','E','N','S'),
        .width=1024,.height=3,.num_planes=1,.planes={{1024,3072}}}};
    cc_ipa *ipa=create(profile,pisp,true,&c); cc_photo_capabilities caps;
    OK(cc_ipa_get_photo_capabilities(ipa,&caps)); CHECK((caps.flags & (CC_PHOTO_CAP_AF|CC_PHOTO_CAP_PDAF))==(CC_PHOTO_CAP_AF|CC_PHOTO_CAP_PDAF));
    bool wide=profile>=CC_VC4_PROFILE_IMX708_WIDE;
    CHECK(caps.focus.map[0].code==(wide ? 420:445) && caps.focus.ranges[CC_AF_MACRO].maximum==(wide ? 32:15));
    CHECK(caps.focus.check_for_ir==!!(cc_vc4_get_profile_info(profile)->flags & CC_VC4_PROFILE_NOIR));
    cc_af_controls af={.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO};
    cc_ipa_request r={.timing=c.timing,.timing_limits=c.timing_limits,.af=&af};
    bool focused=false; double position=1;
    for (unsigned i=0; i<220; ++i) {
        cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&r,&s)); r.af=NULL;
        if (s.af.flags & CC_AF_LENS_VALID) position=s.af.lens_position;
        double delta=position-4.3; uint64_t contrast=(uint64_t)(100000/(1+delta*delta));
        struct scene scene; cc_ipa_frame f=observation(&c,&s,&scene,0.2);
        f.timestamp_ns=(uint64_t)(i+1)*40000000;
        for (unsigned j=0; j<FOCUS_REGIONS; ++j) scene.vc4.focus_stats[j].contrast_val[1][1]=contrast*1000;
        for (unsigned j=0; j<PISP_CDAF_STATS_NUM_FOMS; ++j) scene.pisp.cdaf.foms[j]=contrast<<20;
        unsigned char packet[3072]={0};
        if (phase) {
            int ph=(int)(delta*50); if (ph>1023) ph=1023; if (ph< -1024) ph= -1024;
            unsigned encoded=(unsigned)ph & 2047U;
            for (unsigned j=0; j<192; ++j) {
                unsigned char *entry=packet+2048+(j+2)*5;
                entry[0]=16; entry[1]=(unsigned char)(encoded>>6); entry[2]=(unsigned char)(encoded<<2);
            }
            f.embedded_data=packet; f.embedded_size=sizeof(packet);
        }
        cc_ipa_metadata m; OK(cc_ipa_process(ipa,&f,&m));
        CHECK((m.flags & CC_IPA_METADATA_AF) && !!(m.flags & CC_IPA_METADATA_PDAF)==phase);
        CHECK(m.af.lens_setting==s.af.lens_setting);
        if (m.af.state==CC_AF_FOCUSED) { CHECK(fabs(position-4.3)<0.7); focused=true; break; }
    }
    CHECK(focused);
    /* A missing/malformed packet clears PDAF metadata; manual code conversion
     * uses this module's map and acknowledges a driver-adjusted native code. */
    af=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_SET_LENS,.mode=CC_AF_MANUAL,.lens_position=5}; r.af=&af;
    cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&r,&s));
    CHECK((s.flags & CC_IPA_LENS) && fabs(s.af.lens_position-5)<0.05);
    struct scene scene; cc_ipa_frame f=observation(&c,&s,&scene,0.2); f.af.lens_setting+=1;
    cc_ipa_metadata m; OK(cc_ipa_process(ipa,&f,&m));
    CHECK(!(m.flags & CC_IPA_METADATA_PDAF) && m.af.lens_position>s.af.lens_position);
    cc_ipa_stop(ipa); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa)); r.af=NULL;
    OK(cc_ipa_prepare(ipa,&r,&s)); CHECK(!(s.af.flags & CC_AF_LENS_VALID)); cc_ipa_destroy(ipa);
}
static void automatic_policies(bool pisp)
{
    cc_ipa_configuration c=configuration(CC_VC4_PROFILE_OV5647,pisp);
    c.initial_controls[0].value=750;
    cc_ipa *ipa=create(CC_VC4_PROFILE_OV5647,pisp,false,&c);
    cc_photo_controls photo={.flags=CC_PHOTO_GAIN|CC_PHOTO_FLICKER,.analogue_gain=1,.mains_hz=50};
    cc_ipa_request r={.timing=c.timing,.timing_limits=c.timing_limits,.photo=&photo};
    cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&r,&s)); CHECK(s.sensor_controls[1].value==500);
    photo.mains_hz=60; OK(cc_ipa_prepare(ipa,&r,&s)); CHECK(s.sensor_controls[1].value==416);
    struct scene scene; cc_ipa_frame f=observation(&c,&s,&scene,0.9); f.untrusted=1;
    cc_ipa_metadata m; OK(cc_ipa_process(ipa,&f,&m));
    CHECK((m.photo_status & CC_PHOTO_AUTO_EXPOSURE) && !(m.photo_status & CC_PHOTO_AUTO_GAIN) && m.lux==0);
    cc_ipa_settings after; OK(cc_ipa_prepare(ipa,&r,&after));
    CHECK(after.sensor_controls[1].value==s.sensor_controls[1].value);
    /* Shutter priority grows timing for a long manual shutter; a frame ceiling
     * clamps the shutter and explicitly reports a limit. */
    photo=(cc_photo_controls){.flags=CC_PHOTO_EXPOSURE|CC_PHOTO_GAIN|CC_PHOTO_FRAME_DURATION,
        .exposure_ns=100000000,.analogue_gain=1,.min_frame_duration_ns=20000000,.max_frame_duration_ns=20000000};
    OK(cc_ipa_prepare(ipa,&r,&s)); f=observation(&c,&s,&scene,0.2); OK(cc_ipa_process(ipa,&f,&m));
    CHECK(m.frame_duration_ns<=20000000 && m.exposure_ns<20000000 && (m.photo_status & CC_PHOTO_LIMITED));
    photo.min_frame_duration_ns=photo.max_frame_duration_ns=0;
    OK(cc_ipa_prepare(ipa,&r,&s)); f=observation(&c,&s,&scene,0.2); OK(cc_ipa_process(ipa,&f,&m));
    CHECK(m.exposure_ns==100000000 && m.frame_duration_ns>=100000000);
    cc_ipa_destroy(ipa);
}
static void validation(void)
{
    cc_photo_controls c={.flags=CC_PHOTO_EV,.exposure_value=NAN}; CHECK(cc_photo_validate_controls(&c)==-EINVAL);
    c.exposure_value=INFINITY; CHECK(cc_photo_validate_controls(&c)==-EINVAL);
    c.exposure_value=-8; OK(cc_photo_validate_controls(&c)); c.flags=0; c.exposure_value=NAN; OK(cc_photo_validate_controls(&c));
    c=(cc_photo_controls){.flags=CC_PHOTO_FRAME_DURATION,.min_frame_duration_ns=1}; CHECK(cc_photo_validate_controls(&c)==-EINVAL);
    c=(cc_photo_controls){.flags=CC_PHOTO_WB,.white_balance=CC_PHOTO_WB_MANUAL,.red_gain=1}; CHECK(cc_photo_validate_controls(&c)==-EINVAL);
    cc_request *request; OK(cc_request_create(1,&request)); CHECK(cc_request_set_photo_controls(request,&c)==-EINVAL);
    OK(cc_request_set_photo_controls(request,NULL)); OK(cc_request_destroy(request));
    cc_ipa_configuration config=configuration(CC_VC4_PROFILE_OV5647,false);
    cc_ipa *ipa; cc_ipa_vc4_options o={.autofocus=1}; OK(cc_ipa_vc4_create(&o,&ipa));
    CHECK(cc_ipa_configure(ipa,&config)==-ENOTSUP); cc_ipa_destroy(ipa);
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AE,.target_luminance=0.18,
        .exposure_speed=0.25,.white_balance_speed=0.25,.min_colour_gain=0.5,.max_colour_gain=4};
    OK(cc_ipa_reference_create(&tuning,&ipa)); OK(cc_ipa_configure(ipa,&config)); OK(cc_ipa_start(ipa));
    cc_photo_capabilities caps; CHECK(cc_ipa_get_photo_capabilities(ipa,&caps)==-ENOTSUP);
    cc_ipa_request r={.photo=&c}; cc_ipa_settings settings;
    CHECK(cc_ipa_prepare(ipa,&r,&settings)==-ENOTSUP); cc_ipa_destroy(ipa);
}
int main(int argc, char **argv)
{
    CHECK(argc==1 || argc==2); if (argc==2) worker=argv[1]; validation();
    for (unsigned pipeline=0; pipeline<2; ++pipeline) {
        bool pisp=pipeline!=0;
        for (unsigned id=CC_VC4_PROFILE_OV5647; id<=CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++id)
            controls((enum cc_vc4_profile)id,pisp);
        metering(pisp); automatic_policies(pisp);
        double normal=ev_response(pisp,0), brighter=ev_response(pisp,1), darker=ev_response(pisp,-1);
        CHECK(brighter>normal*1.6 && darker<normal*0.7);
        for (unsigned id=CC_VC4_PROFILE_IMX708; id<=CC_VC4_PROFILE_IMX708_WIDE_NOIR; ++id)
            for (unsigned phase=0; phase<2; ++phase) autofocus((enum cc_vc4_profile)id,pisp,phase!=0);
    }
    puts("Photo controls, live capabilities, applied metadata, EV/metering and calibrated V3 AF passed"); return 0;
}
