/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
static int close_to(double a, double b) { double d = a > b ? a-b : b-a; return d < 1e-8; }
static int64_t value(const cc_control *controls, size_t count, uint32_t id)
{
    for (size_t i = 0; i < count; ++i) if (controls[i].id == id) return controls[i].value;
    CHECK(0); return 0;
}
static cc_control_info range(uint32_t id, int64_t minimum, int64_t maximum)
{
    return (cc_control_info) {.id=id,.type=V4L2_CTRL_TYPE_INTEGER,.minimum=minimum,.maximum=maximum,.step=1,.elems=1};
}
static cc_ipa_configuration configuration(void)
{
    cc_ipa_configuration c = {.pipeline=CC_IPA_VC4,.statistics_fourcc=CC_FOURCC('B','S','T','A'),
        .source={.fourcc=V4L2_PIX_FMT_SRGGB16,.width=640,.height=480,.num_planes=1,.planes={{1280,614400}}},
        .timing={.pixel_rate=100000000,.width=640,.height=480,.hblank=360,.vblank=20},.num_sensor_controls=2};
    c.sensor_format = c.source; c.sensor_format.fourcc = V4L2_PIX_FMT_SRGGB10P;
    OK(cc_sensor_lookup("imx219", &c.sensor));
    c.sensor_controls[0] = range(V4L2_CID_EXPOSURE,1,1000);
    c.sensor_controls[1] = range(V4L2_CID_ANALOGUE_GAIN,0,240);
    c.initial_controls[0] = (cc_control) {.id=V4L2_CID_EXPOSURE,.value=100};
    c.initial_controls[1] = (cc_control) {.id=V4L2_CID_ANALOGUE_GAIN,.value=0};
    c.red_balance = range(V4L2_CID_RED_BALANCE,0,8000);
    c.blue_balance = range(V4L2_CID_BLUE_BALANCE,0,8000);
    c.digital_gain = range(V4L2_CID_DIGITAL_GAIN,0,8000);
    return c;
}
static cc_ipa_reference_tuning tuning(void)
{
    return (cc_ipa_reference_tuning) {.flags=CC_IPA_REFERENCE_AE | CC_IPA_REFERENCE_AWB,
        .target_luminance=0.2,.exposure_speed=1,.white_balance_speed=1,.min_colour_gain=0.5,.max_colour_gain=4};
}
static void test_statistics(void)
{
    struct bcm2835_isp_stats vc4 = {.size=sizeof(vc4)};
    vc4.hist[0].g_hist[32] = 7;
    vc4.hist[0].r_hist[127] = UINT32_MAX; /* The luminance proxy is specifically hist[0].green. */
    vc4.awb_stats[0] = (struct bcm2835_isp_stats_region) {.counted=4,.r_sum=4096,.g_sum=8192,.b_sum=16384};
    unsigned char unaligned[sizeof(vc4)+1]; memcpy(unaligned+1,&vc4,sizeof(vc4));
    cc_ipa_statistics s;
    OK(cc_ipa_parse_statistics(CC_FOURCC('B','S','T','A'),unaligned+1,sizeof(vc4),&s));
    CHECK(s.flags == (CC_IPA_STATS_LUMA | CC_IPA_STATS_COLOUR) && s.colour_samples == 4);
    CHECK(close_to(s.luminance,32.5/128) && close_to(s.red,0.125) && close_to(s.green,0.25) && close_to(s.blue,0.5));
    cc_ipa_statistics unchanged; memcpy(&unchanged,&s,sizeof(s));
    ERROR(cc_ipa_parse_statistics(CC_FOURCC('B','S','T','A'),&vc4,sizeof(vc4)-1,&s),EMSGSIZE);
    CHECK(!memcmp(&s,&unchanged,sizeof(s)));
    vc4.size = 1; ERROR(cc_ipa_parse_statistics(CC_FOURCC('B','S','T','A'),&vc4,sizeof(vc4),&s),EPROTO);
    vc4.size = sizeof(vc4); vc4.awb_stats[0].r_sum = UINT64_MAX;
    ERROR(cc_ipa_parse_statistics(CC_FOURCC('B','S','T','A'),&vc4,sizeof(vc4),&s),EPROTO);
    CHECK(!memcmp(&s,&unchanged,sizeof(s)));
    struct pisp_statistics pisp = {0};
    pisp.agc.histogram[128] = 10;
    pisp.awb.zones[0] = (struct pisp_awb_statistics_zone) {.counted=2,.R_sum=16384,.G_sum=32768,.B_sum=65536};
    OK(cc_ipa_parse_statistics(CC_FOURCC('R','P','F','S'),&pisp,sizeof(pisp),&s));
    CHECK(close_to(s.luminance,128.5/1024) && close_to(s.red,0.125) && close_to(s.green,0.25) && close_to(s.blue,0.5));
    memset(&pisp,0,sizeof(pisp));
    OK(cc_ipa_parse_statistics(CC_FOURCC('R','P','F','S'),&pisp,sizeof(pisp),&s)); CHECK(!s.flags);
    ERROR(cc_ipa_parse_statistics(0,&pisp,sizeof(pisp),&s),ENOTSUP);
    pisp.awb.zones[0].counted = 1; pisp.awb.zones[0].R_sum = 65536;
    ERROR(cc_ipa_parse_statistics(CC_FOURCC('R','P','F','S'),&pisp,sizeof(pisp),&s),EPROTO);
}
struct lifecycle { int configured, started, stopped, prepared, processed, destroyed, error; bool bad, bad_lens; };
static int module_configure(void *p, const cc_ipa_configuration *c) { (void)c; ++((struct lifecycle *)p)->configured; return ((struct lifecycle *)p)->error; }
static int module_start(void *p) { ++((struct lifecycle *)p)->started; return ((struct lifecycle *)p)->error; }
static void module_stop(void *p) { ++((struct lifecycle *)p)->stopped; }
static void module_destroy(void *p) { ++((struct lifecycle *)p)->destroyed; }
static int module_prepare(void *p, const cc_ipa_request *r, cc_ipa_settings *s)
{
    (void)r; struct lifecycle *l = p; ++l->prepared;
    s->flags = l->bad ? UINT32_MAX : 0;
    if (l->bad_lens) s->af.flags=CC_AF_LENS_VALID; /* Holding a lens still requires the capability. */
    return l->error;
}
static int module_process(void *p, const cc_ipa_frame *f, cc_ipa_metadata *m)
{
    (void)f; struct lifecycle *l = p; ++l->processed;
    if (l->bad) { m->flags=CC_IPA_METADATA_AWB; m->red_gain=-1; m->blue_gain=1; }
    if (l->bad_lens) { m->flags=CC_IPA_METADATA_AF; m->af.state=(enum cc_af_state)99; }
    return l->error;
}
static void test_lifecycle(void)
{
    struct lifecycle l = {0};
    cc_ipa_ops ops = {CC_IPA_ABI_VERSION,sizeof(ops),module_configure,module_start,module_stop,module_prepare,module_process,module_destroy,0};
    cc_ipa *ipa = NULL;
    ops.abi_version = 99; ERROR(cc_ipa_create(&ops,&l,&ipa),EPROTONOSUPPORT); CHECK(!ipa);
    ops.abi_version = CC_IPA_ABI_VERSION; --ops.size; ERROR(cc_ipa_create(&ops,&l,&ipa),EPROTONOSUPPORT);
    ++ops.size; OK(cc_ipa_create(&ops,&l,&ipa));
    cc_ipa_request request = {0}; cc_ipa_settings settings = {0};
    cc_ipa_frame frame = {0}; cc_ipa_metadata metadata = {0};
    ERROR(cc_ipa_start(ipa),EPIPE); ERROR(cc_ipa_prepare(ipa,&request,&settings),EPIPE);
    cc_ipa_configuration c = configuration(); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    ERROR(cc_ipa_start(ipa),EBUSY); ERROR(cc_ipa_configure(ipa,&c),EBUSY);
    OK(cc_ipa_prepare(ipa,&request,&settings)); OK(cc_ipa_process(ipa,&frame,&metadata));
    settings.flags = 123; metadata.flags = 234;
    l.bad_lens=true;
    ERROR(cc_ipa_prepare(ipa,&request,&settings),EPROTO); CHECK(settings.flags==123);
    ERROR(cc_ipa_process(ipa,&frame,&metadata),EPROTO); CHECK(metadata.flags==234);
    l.bad_lens=false;
    l.bad = true;
    ERROR(cc_ipa_prepare(ipa,&request,&settings),EPROTO); CHECK(settings.flags == 123);
    ERROR(cc_ipa_process(ipa,&frame,&metadata),EPROTO); CHECK(metadata.flags == 234);
    l.bad = false; l.error = 1; /* Positive callback returns are protocol errors. */
    ERROR(cc_ipa_prepare(ipa,&request,&settings),EPROTO); CHECK(settings.flags == 123);
    l.error = -EIO; ERROR(cc_ipa_process(ipa,&frame,&metadata),EIO);
    cc_ipa_stop(ipa); cc_ipa_stop(ipa); CHECK(l.stopped == 1);
    ERROR(cc_ipa_configure(ipa,&c),EIO); ERROR(cc_ipa_start(ipa),EPIPE);
    l.error = 0; OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    cc_ipa_destroy(ipa); CHECK(l.started == 2 && l.stopped == 2 && l.destroyed == 1);
}
static cc_ipa_frame observation(const cc_ipa_settings *settings, double luminance)
{
    cc_ipa_frame f = {.statistics={.flags=CC_IPA_STATS_LUMA | CC_IPA_STATS_COLOUR,
        .luminance=luminance,.red=0.1,.green=0.2,.blue=0.4,.colour_samples=100},
        .isp_controls=settings->isp_controls,.num_isp_controls=settings->num_isp_controls,
        .backend=settings->flags & CC_IPA_BACKEND ? &settings->backend : NULL};
    f.sensor.count = settings->num_sensor_controls;
    memcpy(f.sensor.controls,settings->sensor_controls,f.sensor.count*sizeof(cc_control));
    return f;
}
static void test_reference(void)
{
    cc_ipa_configuration c = configuration();
    cc_ipa_reference_tuning t = tuning(); cc_ipa *ipa;
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    cc_ipa_request request = {0}; cc_ipa_settings s;
    OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE) == 100);
    cc_ipa_frame f = observation(&s,0.1); cc_ipa_metadata m;
    OK(cc_ipa_process(ipa,&f,&m)); CHECK(m.exposure_ns == 1000000 && m.analogue_gain == 1 && m.red_gain == 1);
    OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE) == 200);
    CHECK(value(s.isp_controls,s.num_isp_controls,V4L2_CID_RED_BALANCE) == 2000);
    CHECK(value(s.isp_controls,s.num_isp_controls,V4L2_CID_BLUE_BALANCE) == 500);
    CHECK(value(s.isp_controls,s.num_isp_controls,V4L2_CID_DIGITAL_GAIN) == 2000);
    f = observation(&s,0.2); OK(cc_ipa_process(ipa,&f,&m));
    CHECK((m.flags & (CC_IPA_AE_CONVERGED | CC_IPA_AWB_CONVERGED)) == (CC_IPA_AE_CONVERGED | CC_IPA_AWB_CONVERGED));
    f = observation(&s,0.8); OK(cc_ipa_process(ipa,&f,&m)); OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE) == 100);
    cc_ipa_destroy(ipa);
    c=configuration(); c.initial_controls[0].value=490;
    t=tuning(); OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    request.timing=c.timing; request.timing.vblank=0;
    OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE)==476);
    cc_control blank={.id=V4L2_CID_VBLANK,.value=20};
    request.sensor_controls=&blank; request.num_sensor_controls=1;
    OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE)==490);
    cc_ipa_destroy(ipa); request=(cc_ipa_request){0}; c=configuration();
    t.max_exposure_ns = 200000;
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE) == 20);
    f = observation(&s,0.01); OK(cc_ipa_process(ipa,&f,&m)); OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE) == 20);
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_ANALOGUE_GAIN) == 128); /* IMX219 2x. */
    cc_ipa_destroy(ipa);
    /* Closed loop against an independent exposure/gain plant, with non-unit
     * damping and native quantization. It must converge rather than oscillate. */
    t = tuning(); t.exposure_speed=0.5; c.initial_controls[0].value=10;
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    for (unsigned i = 0; i < 40; ++i) {
        OK(cc_ipa_prepare(ipa,&request,&s));
        double lines = (double)value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE);
        double code = (double)value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_ANALOGUE_GAIN);
        double light = lines * 0.002 * 256 / (256-code); if (light > 1) light=1;
        f = observation(&s,light); OK(cc_ipa_process(ipa,&f,&m));
        if (i > 20) CHECK(m.flags & CC_IPA_AE_CONVERGED);
    }
    cc_ipa_destroy(ipa);
}
static void test_pisp_and_invalid(void)
{
    cc_ipa_configuration c = configuration(); c.pipeline=CC_IPA_PISP; c.statistics_fourcc=CC_FOURCC('R','P','F','S');
    cc_ipa_reference_tuning t=tuning(); t.flags=CC_IPA_REFERENCE_AWB; cc_ipa *ipa;
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    cc_ipa_request request={0}; cc_ipa_settings s; cc_ipa_metadata m;
    OK(cc_ipa_prepare(ipa,&request,&s)); cc_ipa_frame f=observation(&s,0);
    OK(cc_ipa_process(ipa,&f,&m)); OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(s.flags == (CC_IPA_FRONTEND | CC_IPA_BACKEND));
    CHECK(s.backend.wbg.gain_r == 4096 && s.backend.wbg.gain_g == 2048 && s.backend.wbg.gain_b == 1024);
    cc_cfe_configuration fe={.sensor=c.sensor_format,.image=c.source}; struct pisp_fe_config parameters;
    OK(cc_cfe_build_parameters(&fe,&s.frontend,&parameters)); CHECK(parameters.rgby.gain_r == 1224);
    cc_pisp_be_configuration be={.input=c.source,.output={.fourcc=V4L2_PIX_FMT_RGB24,.width=640,.height=480,
        .num_planes=1,.planes={{1920,921600}}}};
    struct pisp_be_tiles_config tiles; OK(cc_pisp_be_build_configuration(&be,&s.backend,&tiles));
    CHECK(tiles.config.wbg.gain_r == 4096);
    cc_ipa_destroy(ipa);
    t.target_luminance=0; ERROR(cc_ipa_reference_create(&t,&ipa),EINVAL); CHECK(!ipa);
    t=tuning(); t.max_colour_gain=5; ERROR(cc_ipa_reference_create(&t,&ipa),EINVAL);
    t=tuning(); c.pipeline=CC_IPA_CFE; OK(cc_ipa_reference_create(&t,&ipa)); ERROR(cc_ipa_configure(ipa,&c),ENOTSUP);
    cc_ipa_destroy(ipa);
    c=configuration(); c.sensor_controls[0].flags=V4L2_CTRL_FLAG_READ_ONLY;
    OK(cc_ipa_reference_create(&t,&ipa)); ERROR(cc_ipa_configure(ipa,&c),ENOTSUP); cc_ipa_destroy(ipa);
    c=configuration(); c.red_balance.maximum=0;
    OK(cc_ipa_reference_create(&t,&ipa)); ERROR(cc_ipa_configure(ipa,&c),ENOTSUP); cc_ipa_destroy(ipa);
}
static void test_af_update_rate(void)
{
    for (unsigned int timed=0;timed<2;++timed) {
        cc_af_tuning af; cc_af_reference_tuning(&af); af.speeds[0].pdaf_gain=0;
        cc_ipa_reference_tuning t=tuning(); t.flags=CC_IPA_REFERENCE_AF; t.af=&af;
        cc_ipa *ipa; OK(cc_ipa_reference_create(&t,&ipa));
        cc_ipa_configuration c=configuration();
        c.lens=(cc_control_info){.id=V4L2_CID_FOCUS_ABSOLUTE,.type=V4L2_CTRL_TYPE_INTEGER,
            .minimum=0,.maximum=1023,.step=1,.elems=1};
        OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
        cc_af_controls controls={.flags=CC_AF_SET_MODE,.mode=CC_AF_CONTINUOUS};
        cc_ipa_request request={.af=&controls}; unsigned int updates=0;
        struct bcm2835_isp_stats stats={.size=sizeof(stats)};
        for (size_t i=0;i<FOCUS_REGIONS;++i) stats.focus_stats[i].contrast_val[1][1]=1000000;
        for (unsigned int frame=0;frame<100;++frame) {
            cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&request,&s));
            updates+=(s.flags & CC_IPA_LENS)!=0;
            request.af=NULL;
            /* Another queued request with no completed statistics cannot
             * advance settling counters or move the lens a second time. */
            cc_ipa_settings second; OK(cc_ipa_prepare(ipa,&request,&second)); CHECK(!(second.flags & CC_IPA_LENS));
            cc_ipa_frame f={.af=s.af,.statistics_fourcc=CC_FOURCC('B','S','T','A'),
                .statistics_data=&stats,.statistics_size=sizeof(stats),
                .timestamp_ns=timed ? UINT64_C(1000000000)+(uint64_t)frame*10000000:0};
            cc_ipa_metadata m; OK(cc_ipa_process(ipa,&f,&m)); CHECK(m.flags & CC_IPA_METADATA_AF);
        }
        CHECK(timed ? updates>=20 && updates<=30:updates==100);
        cc_ipa_destroy(ipa);
    }
}
static void test_embedded_hdr_feedback(void)
{
    cc_ipa_configuration c=configuration(); OK(cc_sensor_lookup("imx708",&c.sensor));
    c.embedded=(cc_embedded_configuration){.bits_per_pixel=10,.format={.fourcc=CC_FOURCC('S','E','N','S'),
        .width=1024,.height=4,.num_planes=1,.planes={{1024,4096}}}};
    cc_ipa_reference_tuning t=tuning(); t.flags=CC_IPA_REFERENCE_AE;
    cc_ipa *ipa; OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    cc_ipa_request request={0}; cc_ipa_settings settings; OK(cc_ipa_prepare(ipa,&request,&settings));
    uint8_t packet[4096]={0};
    for (size_t i=0;i<137;++i) packet[3072+i*5+3]=0x55;
    packet[3072+2*5+2]=16; /* Four pixels in linear bin 2; centre=1280/65536. */
    cc_ipa_frame frame=observation(&settings,0.9); frame.embedded_data=packet; frame.embedded_size=sizeof(packet);
    cc_ipa_metadata metadata; OK(cc_ipa_process(ipa,&frame,&metadata));
    CHECK((metadata.flags&CC_IPA_METADATA_HDR) && metadata.hdr.average==1280 && metadata.hdr.counted==4 && metadata.luminance==0.078125);
    OK(cc_ipa_prepare(ipa,&request,&settings));
    CHECK(value(settings.sensor_controls,settings.num_sensor_controls,V4L2_CID_EXPOSURE)==200);
    frame=observation(&settings,0.9); OK(cc_ipa_process(ipa,&frame,&metadata));
    CHECK(!(metadata.flags&(CC_IPA_METADATA_HDR|CC_IPA_METADATA_PDAF|CC_IPA_METADATA_SENSOR)) && metadata.luminance==0.9);
    cc_ipa_destroy(ipa);
}
static cc_ipa_configuration planning_configuration(void)
{
    cc_ipa_configuration c=configuration(); c.sensor_controls[0].maximum=496;
    c.num_sensor_controls=3;
    c.sensor_controls[2]=range(V4L2_CID_VBLANK,20,1520);
    c.initial_controls[2]=(cc_control){.id=V4L2_CID_VBLANK,.value=20};
    c.sensor_delays[0]=(cc_sensor_control_delay){V4L2_CID_EXPOSURE,2,0};
    c.sensor_delays[1]=(cc_sensor_control_delay){V4L2_CID_ANALOGUE_GAIN,1,0};
    c.sensor_delays[2]=(cc_sensor_control_delay){V4L2_CID_VBLANK,2,CC_SENSOR_CONTROL_PRIORITY};
    c.timing_limits=(cc_sensor_timing_limits){100000000,640,480,360,360,1,20,1520,1,1,1};
    return c;
}
static void test_duration_planning(void)
{
    cc_ipa_configuration c=planning_configuration();
    cc_ipa_reference_tuning t=tuning(); t.flags=CC_IPA_REFERENCE_AE;
    t.min_frame_duration_ns=5000000; t.max_frame_duration_ns=20000000;
    cc_ipa *ipa; OK(cc_ipa_reference_create(&t,&ipa));
    CHECK(cc_ipa_get_capabilities(ipa)==CC_IPA_CAP_SENSOR_TIMING);
    OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    cc_ipa_request request={0}; cc_ipa_settings s; cc_ipa_metadata m;
    for (unsigned int i=0;i<4;++i) {
        OK(cc_ipa_prepare(ipa,&request,&s));
        CHECK(s.num_sensor_controls==3 && value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE)==(100<<i));
        if (i==3) CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_VBLANK)==324);
        cc_ipa_frame f=observation(&s,0); OK(cc_ipa_process(ipa,&f,&m));
        CHECK(m.exposure_ns==(UINT64_C(1000000)<<i));
    }
    cc_control overrides[]={{.id=V4L2_CID_EXPOSURE,.value=700},{.id=V4L2_CID_VBLANK,.value=20}};
    request.sensor_controls=overrides; request.num_sensor_controls=1;
    OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE)==700 &&
          value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_VBLANK)==224);
    request.num_sensor_controls=2; s.flags=123;
    ERROR(cc_ipa_prepare(ipa,&request,&s),ERANGE); CHECK(s.flags==123);
    request.num_sensor_controls=1; overrides[0].value=701;
    request.timing_limits=c.timing_limits; request.timing_limits.max_vblank=220;
    ERROR(cc_ipa_prepare(ipa,&request,&s),ERANGE); CHECK(s.flags==123);
    cc_ipa_destroy(ipa);
    for (unsigned int i=0;i<4;++i) {
        c=planning_configuration();
        if (i==0) c.sensor_delays[2].flags=0;
        if (i==1) c.sensor_delays[2].frames=1;
        if (i==2) c.timing_limits.pixel_rate=0;
        if (i==3) c.timing_limits.exposure_step=2;
        OK(cc_ipa_reference_create(&t,&ipa)); CHECK(cc_ipa_configure(ipa,&c)<0); cc_ipa_destroy(ipa);
    }
    c=planning_configuration(); c.num_sensor_controls=4;
    c.sensor_controls[3]=range(V4L2_CID_HBLANK,360,1360);
    c.initial_controls[3]=(cc_control){.id=V4L2_CID_HBLANK,.value=360};
    c.sensor_delays[3]=(cc_sensor_control_delay){V4L2_CID_HBLANK,2,0};
    c.timing_limits.max_hblank=1360; c.timing_limits.max_vblank=20;
    c.sensor_controls[2].maximum=20;
    t.max_frame_duration_ns=10000000; c.initial_controls[0].value=400;
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    request=(cc_ipa_request){0}; OK(cc_ipa_prepare(ipa,&request,&s));
    cc_ipa_frame f=observation(&s,0); OK(cc_ipa_process(ipa,&f,&m)); OK(cc_ipa_prepare(ipa,&request,&s));
    CHECK(s.num_sensor_controls==4 && value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_HBLANK)==968);
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE)==496);
    overrides[0].value=400; request.sensor_controls=overrides; request.num_sensor_controls=1;
    OK(cc_ipa_prepare(ipa,&request,&s)); CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_HBLANK)==360);
    cc_ipa_destroy(ipa);
    c=planning_configuration(); OK(cc_sensor_lookup("imx296",&c.sensor));
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    overrides[0].value=400;
    OK(cc_ipa_prepare(ipa,&request,&s)); CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE)==400);
    CHECK(value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_VBLANK)>=119);
    cc_ipa_destroy(ipa);
    c=planning_configuration(); OK(cc_sensor_lookup("imx708",&c.sensor));
    c.sensor_controls[0].maximum=452; c.initial_controls[0].value=400;
    c.sensor_controls[2].maximum=c.timing_limits.max_vblank=9999520;
    t.max_frame_duration_ns=UINT64_C(100000000000);
    OK(cc_ipa_reference_create(&t,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
    request=(cc_ipa_request){0};
    for (unsigned int i=0;i<13;++i) {
        OK(cc_ipa_prepare(ipa,&request,&s));
        uint32_t lines=(uint32_t)value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_EXPOSURE);
        uint32_t frame_length=480U+(uint32_t)value(s.sensor_controls,s.num_sensor_controls,V4L2_CID_VBLANK);
        CHECK(lines+48<=frame_length && lines>0);
        if (i>8) CHECK(frame_length>0xffdc && frame_length%2==0);
        f=observation(&s,0); OK(cc_ipa_process(ipa,&f,&m)); CHECK(m.exposure_ns==(uint64_t)lines*10000);
    }
    cc_ipa_destroy(ipa);
    t.min_frame_duration_ns=0; ERROR(cc_ipa_reference_create(&t,&ipa),EINVAL); CHECK(!ipa);
}
int main(void)
{
    test_embedded_hdr_feedback(); test_duration_planning();
    test_statistics(); test_lifecycle(); test_reference(); test_pisp_and_invalid(); test_af_update_rate();
    puts("C IPA ABI/lifecycle, native statistics, AE/AGC/AWB convergence, quantization and PiSP tuning passed");
    return 0;
}
