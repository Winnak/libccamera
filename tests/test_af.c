/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/af.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "af_reference_vectors.h"
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
static double distance(double a, double b) { return a>b ? a-b:b-a; }
static cc_af *controller(const cc_af_tuning *t)
{
    cc_af *a; OK(cc_af_create(t,&a));
    cc_control_info lens={.id=V4L2_CID_FOCUS_ABSOLUTE,.type=V4L2_CTRL_TYPE_INTEGER,.minimum=0,.maximum=1023,.step=1,.elems=1};
    cc_rectangle region={0,0,640,480}; OK(cc_af_configure(a,&region,&lens)); return a;
}
static cc_af_status frame(cc_af *a, double peak, bool flat, bool pdaf, bool native, bool pisp)
{
    static double current;
    cc_af_pdaf p[12];
    for (size_t i=0;i<12;++i) p[i]=(cc_af_pdaf){128,(int16_t)((current-peak)*50)};
    cc_af_pdaf_grid grid={3,4,p}; cc_af_status s;
    OK(cc_af_prepare(a,pdaf ? &grid:NULL,&s));
    CHECK(s.state>=CC_AF_IDLE && s.state<=CC_AF_FAILED);
    if (s.flags & CC_AF_LENS_VALID) current=(double)(s.lens_setting-445)/32;
    double delta=current-peak;
    uint64_t contrast=flat ? 1000U : (uint64_t)(100000/(1+delta*delta));
    if (!native) {
        uint64_t focus[12]; for (size_t i=0;i<12;++i) focus[i]=contrast;
        cc_af_statistics stats={.focus_rows=3,.focus_cols=4,.focus=focus}; OK(cc_af_process(a,&stats));
    } else if (pisp) {
        struct pisp_statistics stats={0};
        for (size_t i=0;i<PISP_CDAF_STATS_NUM_FOMS;++i) stats.cdaf.foms[i]=contrast<<20;
        OK(cc_af_process_native(a,CC_FOURCC('R','P','F','S'),&stats,sizeof(stats)));
    } else {
        struct bcm2835_isp_stats stats={.size=sizeof(stats)};
        for (size_t i=0;i<FOCUS_REGIONS;++i) stats.focus_stats[i].contrast_val[1][1]=contrast*1000;
        OK(cc_af_process_native(a,CC_FOURCC('B','S','T','A'),&stats,sizeof(stats)));
    }
    return s;
}
static void test_scans(void)
{
    for (unsigned int variant=0;variant<5;++variant) {
        cc_af_tuning t; cc_af_reference_tuning(&t);
        if (variant!=4) { t.speeds[0].pdaf_gain=0; t.speeds[1].pdaf_gain=0; }
        cc_af *a=controller(&t);
        cc_af_controls c={.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO}; OK(cc_af_set_controls(a,&c));
        bool finished=false;
        for (unsigned int i=0;i<240;++i) {
            cc_af_status s=frame(a,4.3,variant==1,variant==4,variant==2 || variant==3,variant==3);
            if (s.state==CC_AF_FOCUSED || s.state==CC_AF_FAILED) {
                CHECK(s.flags & CC_AF_LENS_VALID);
                if (variant==1) CHECK(s.state==CC_AF_FAILED);
                else CHECK(s.state==CC_AF_FOCUSED && distance(s.lens_position,4.3)<0.35);
                finished=true; break;
            }
        }
        CHECK(finished); cc_af_destroy(a);
    }
}
static void test_controls_and_scene(void)
{
    cc_af_tuning t; cc_af_reference_tuning(&t); cc_af *a=controller(&t);
    cc_af_controls c={.flags=CC_AF_SET_MODE,.mode=CC_AF_CONTINUOUS}; OK(cc_af_set_controls(a,&c));
    cc_af_status s={0};
    for (unsigned int i=0;i<200;++i) s=frame(a,3,false,false,false,false);
    CHECK(s.state==CC_AF_FOCUSED && distance(s.lens_position,3)<0.35);
    for (unsigned int i=0;i<240;++i) s=frame(a,8,false,false,false,false);
    CHECK(s.state==CC_AF_FOCUSED && distance(s.lens_position,8)<0.35);
    c=(cc_af_controls){.flags=CC_AF_SET_PAUSE,.pause=CC_AF_PAUSE_IMMEDIATE}; OK(cc_af_set_controls(a,&c));
    int32_t code=s.lens_setting;
    for (unsigned int i=0;i<80;++i) {
        s=frame(a,1,false,false,false,false); CHECK(s.pause_state==CC_AF_PAUSED && s.lens_setting==code);
    }
    c.pause=CC_AF_RESUME; OK(cc_af_set_controls(a,&c));
    for (unsigned int i=0;i<240;++i) s=frame(a,1,false,false,false,false);
    CHECK(s.state==CC_AF_FOCUSED && distance(s.lens_position,1)<0.35);
    c=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_SET_LENS,.mode=CC_AF_MANUAL,.lens_position=9};
    OK(cc_af_set_controls(a,&c));
    for (unsigned int i=0;i<10;++i) s=frame(a,1,false,false,false,false);
    CHECK(s.state==CC_AF_IDLE && s.lens_setting==733 && s.lens_position==9);
    c=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO}; OK(cc_af_set_controls(a,&c));
    s=frame(a,1,false,false,false,false); CHECK(s.state==CC_AF_SCANNING);
    c=(cc_af_controls){.flags=CC_AF_CANCEL}; OK(cc_af_set_controls(a,&c));
    s=frame(a,1,false,false,false,false); CHECK(s.state==CC_AF_IDLE);
    cc_af_destroy(a);
}
static void test_validation(void)
{
    cc_af_tuning t; cc_af_reference_tuning(&t); cc_af *a;
    t.map[1].dioptres=0; CHECK(cc_af_create(&t,&a)==-EINVAL && !a);
    cc_af_reference_tuning(&t); t.speeds[0].step_coarse=0.000001; CHECK(cc_af_create(&t,&a)==-E2BIG);
    cc_af_reference_tuning(&t); a=controller(&t);
    cc_af_controls c={.flags=CC_AF_SET_WINDOWS,.num_windows=11}; CHECK(cc_af_set_controls(a,&c)==-EINVAL);
    c=(cc_af_controls){.flags=CC_AF_TRIGGER|CC_AF_CANCEL}; CHECK(cc_af_set_controls(a,&c)==-EINVAL);
    uint64_t f=UINT64_MAX; cc_af_statistics stats={.focus_rows=2,.focus_cols=2,.focus=&f};
    stats.focus_rows=1; stats.focus_cols=0; CHECK(cc_af_process(a,&stats)==-EINVAL);
    stats.focus_rows=stats.focus_cols=1; CHECK(cc_af_process(a,&stats)==0);
    struct pisp_statistics p={0}; CHECK(cc_af_process_native(a,CC_FOURCC('R','P','F','S'),&p,1)==-EMSGSIZE);
    cc_af_destroy(a);
}
static uint64_t hash_value(uint64_t hash, uint32_t value)
{
    for (unsigned int byte=0;byte<4;++byte) { hash^=(value>>(byte*8))&255U; hash*=UINT64_C(1099511628211); }
    return hash;
}
static void test_reference_traces(void)
{
    for (uint32_t scenario=0;scenario<6;++scenario) {
        cc_af_tuning tuning; cc_af_reference_tuning(&tuning);
        if (scenario<2 || scenario==4) tuning.speeds[0].pdaf_gain=0;
        if (scenario==5) tuning.check_for_ir=1;
        cc_af *a=controller(&tuning);
        cc_af_controls c={.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=scenario==3 ? CC_AF_CONTINUOUS:CC_AF_AUTO};
        OK(cc_af_set_controls(a,&c));
        if (scenario==4) {
            c=(cc_af_controls){.flags=CC_AF_SET_WINDOWS|CC_AF_SET_METERING,.use_windows=1,.num_windows=1,
                .windows={{0,0,160,480}}}; OK(cc_af_set_controls(a,&c));
        }
        int32_t setting=445; uint64_t hash=UINT64_C(1469598103934665603);
        for (uint32_t f=0;f<300;++f) {
            if (f==150) {
                c=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_SET_LENS,.mode=CC_AF_MANUAL,.lens_position=9};
                OK(cc_af_set_controls(a,&c));
            }
            if (f==170) {
                c=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO}; OK(cc_af_set_controls(a,&c));
            }
            double peak=scenario==3 && f>=80 ? 8:4.25;
            cc_af_pdaf p[12];
            for (size_t i=0;i<12;++i) p[i]=(cc_af_pdaf){128,(int16_t)((((double)setting-445)/32-peak)*50)};
            bool phase=scenario>=2 && scenario!=4 && !(scenario==3 && f>=30 && f<100);
            cc_af_pdaf_grid grid={3,4,p}; cc_af_status status; OK(cc_af_prepare(a,phase ? &grid:NULL,&status));
            if (status.flags & CC_AF_LENS_VALID) setting=status.lens_setting;
            hash=hash_value(hash,(uint32_t)status.state); hash=hash_value(hash,(uint32_t)status.pause_state);
            hash=hash_value(hash,status.flags & CC_AF_LENS_VALID ? 1U:0U);
            hash=hash_value(hash,status.flags & CC_AF_LENS_VALID ? (uint32_t)status.lens_setting:0U);
            uint64_t focus[12]; cc_af_colour colour[192];
            for (size_t i=0;i<12;++i) {
                double local_peak=scenario==4 ? (i%4==0 ? 2:8):peak;
                double delta=((double)setting-445)/32-local_peak;
                focus[i]=scenario==1 ? 1000U:(uint64_t)(100000/(1+delta*delta));
            }
            for (size_t i=0;i<192;++i) colour[i]=(cc_af_colour){1000,scenario==5 ? 1000U:2000U,scenario==5 ? 1000U:4000U,100};
            cc_af_statistics stats={3,4,12,16,focus,colour}; OK(cc_af_process(a,&stats));
        }
        CHECK(hash==cc_af_reference_hashes[scenario]); cc_af_destroy(a);
    }
}
int main(void)
{
    test_scans(); test_controls_and_scene(); test_validation(); test_reference_traces();
    puts("Hybrid AF contrast/phase convergence, native adapters, CAF scene changes, controls and pause passed"); return 0;
}
