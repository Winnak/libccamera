/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "uvc_profiles.h"
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
static int64_t value(const cc_control *controls,size_t count,uint32_t id)
{
    for (size_t i=0;i<count;++i) if (controls[i].id==id) return controls[i].value;
    CHECK(0); return 0;
}
static void test_mappings(void)
{
    cc_uvc_profile p=test_uvc_profile();
    cc_uvc_range range;
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_BRIGHTNESS,&range));
    CHECK(range.minimum==-1 && range.maximum==0.6 && range.default_value==0);
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_CONTRAST,&range));
    CHECK(range.minimum==0.5 && range.maximum==383.0/256 && range.default_value==1);
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_ANALOGUE_GAIN,&range));
    CHECK(range.minimum==1 && range.maximum==4 && range.default_value==1);
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_SATURATION,&range));
    CHECK(range.minimum==0 && range.maximum==255.0/128 && range.default_value==1);
    cc_uvc_controls s={.flags=127,.brightness=0.25,.contrast=1.25,.saturation=0.5,
        .analogue_gain=2,.gamma=2.36,.exposure_us=34567,.exposure_mode=CC_UVC_EXPOSURE_MANUAL};
    cc_control out[7]; size_t count=7; OK(cc_uvc_controls_to_native(&p,&s,out,&count));
    CHECK(count==7 && out[0].id==V4L2_CID_EXPOSURE_AUTO && out[0].value==V4L2_EXPOSURE_MANUAL);
    const int64_t expected[]={36,192,64,1,345,341,235};
    for (size_t i=0;i<7;++i) CHECK(value(out,count,test_uvc_ids[i])==expected[i]);
    s.exposure_mode=CC_UVC_EXPOSURE_AUTO;
    OK(cc_uvc_controls_to_native(&p,&s,out,&count)); CHECK(out[0].value==V4L2_EXPOSURE_APERTURE_PRIORITY);
    p.exposure_menu=15; s.exposure_mode=CC_UVC_EXPOSURE_MANUAL;
    OK(cc_uvc_controls_to_native(&p,&s,out,&count)); CHECK(out[0].value==V4L2_EXPOSURE_SHUTTER_PRIORITY);
    s.exposure_mode=CC_UVC_EXPOSURE_AUTO;
    OK(cc_uvc_controls_to_native(&p,&s,out,&count)); CHECK(out[0].value==V4L2_EXPOSURE_AUTO);
    s.brightness=-DBL_MAX; s.contrast=DBL_MAX; s.saturation=DBL_MAX; s.gamma=DBL_MAX;
    s.analogue_gain=DBL_MAX; s.exposure_us=UINT64_MAX;
    OK(cc_uvc_controls_to_native(&p,&s,out,&count));
    CHECK(value(out,count,V4L2_CID_BRIGHTNESS)==-64 && value(out,count,V4L2_CID_CONTRAST)==255 &&
        value(out,count,V4L2_CID_SATURATION)==255 && value(out,count,V4L2_CID_GAIN)==1023 &&
        value(out,count,V4L2_CID_GAMMA)==500 && value(out,count,V4L2_CID_EXPOSURE_ABSOLUTE)==10000);
    p.native[0].minimum=-128; p.native[0].maximum=128; p.native[0].default_value=0;
    s=(cc_uvc_controls){.flags=CC_UVC_BRIGHTNESS,.brightness=-1.0/256}; count=7;
    OK(cc_uvc_controls_to_native(&p,&s,out,&count)); CHECK(count==1 && out[0].value==-1);
    /* Finite extreme native spans must not overflow signed min/max differences. */
    p.native[0].minimum=INT32_MIN; p.native[0].maximum=INT32_MAX;
    s.brightness=DBL_MAX; OK(cc_uvc_controls_to_native(&p,&s,out,&count)); CHECK(out[0].value==INT32_MAX);
    s.brightness=-DBL_MAX; OK(cc_uvc_controls_to_native(&p,&s,out,&count)); CHECK(out[0].value==INT32_MIN);
}
static void test_rejection(void)
{
    cc_uvc_profile p=test_uvc_profile();
    cc_uvc_controls s={.flags=CC_UVC_BRIGHTNESS,.brightness=0};
    cc_control out[7],saved[7]; memset(out,0xa5,sizeof(out)); memcpy(saved,out,sizeof(out));
    size_t count=0; CHECK(cc_uvc_controls_to_native(&p,&s,out,&count)==-ENOSPC && count==1 && !memcmp(out,saved,sizeof(out)));
    count=123; OK(cc_uvc_controls_to_native(&p,&s,NULL,&count)); CHECK(count==1);
    for (size_t i=0;i<6;++i) {
        p=test_uvc_profile(); s=(cc_uvc_controls){.flags=1,.brightness=0}; count=7;
        if (i==0) s.brightness=NAN;
        if (i==1) s.brightness=INFINITY;
        if (i==2) s.flags=128;
        if (i==3) p.native[0].step=0;
        if (i==4) p.native[0].default_value=999;
        if (i==5) p.native[0].flags=V4L2_CTRL_FLAG_READ_ONLY;
        CHECK(cc_uvc_controls_to_native(&p,&s,out,&count)<0 && count==7 && !memcmp(out,saved,sizeof(out)));
    }
    p=test_uvc_profile(); s=(cc_uvc_controls){.flags=CC_UVC_EXPOSURE_MODE,.exposure_mode=CC_UVC_EXPOSURE_AUTO};
    p.exposure_menu=2; CHECK(cc_uvc_controls_to_native(&p,&s,out,&count)==-ENOTSUP);
    cc_uvc_range r={1,2,3};
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_EXPOSURE_MODE,&r)); CHECK(r.minimum==1 && r.maximum==1 && r.default_value==1);
    p.flags=0; r=(cc_uvc_range){1,2,3}; CHECK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_GAMMA,&r)==-ENOENT);
    CHECK(r.minimum==1 && r.maximum==2 && r.default_value==3);
    p=test_uvc_profile(); p.native[0].minimum=p.native[0].maximum=p.native[0].default_value=10;
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_BRIGHTNESS,&r)); CHECK(r.minimum==0 && r.maximum==0);
    p.native[1].default_value=p.native[1].maximum;
    OK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_CONTRAST,&r)); CHECK(r.minimum==0.5 && r.maximum==1);
    p.native[2].default_value=p.native[2].minimum;
    CHECK(cc_uvc_get_control_range(&p,CC_UVC_CONTROL_SATURATION,&r)==-ENOTSUP);
}
int main(void)
{
    test_mappings(); test_rejection(); puts("UVC normalized ranges, mode choices, rounding, native grids and atomic rejection passed"); return 0;
}
