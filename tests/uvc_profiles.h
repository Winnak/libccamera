/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_TEST_UVC_PROFILES_H
#define CCAMERA_TEST_UVC_PROFILES_H
#include <ccamera/uvc.h>
#include <linux/videodev2.h>
static const uint32_t test_uvc_ids[]={V4L2_CID_BRIGHTNESS,V4L2_CID_CONTRAST,V4L2_CID_SATURATION,
    V4L2_CID_EXPOSURE_AUTO,V4L2_CID_EXPOSURE_ABSOLUTE,V4L2_CID_GAIN,V4L2_CID_GAMMA};
static cc_uvc_profile test_uvc_profile(void)
{
    cc_uvc_profile p={.flags=127,.exposure_menu=10};
    const int64_t bounds[7][4]={{-64,64,16,1},{0,255,128,1},{0,255,128,1},{0,3,3,1},
        {1,10000,100,1},{0,1023,0,1},{100,500,220,5}};
    for (size_t i=0;i<7;++i) p.native[i]=(cc_control_info){.id=test_uvc_ids[i],
        .type=i==3 ? V4L2_CTRL_TYPE_MENU:V4L2_CTRL_TYPE_INTEGER,
        .minimum=bounds[i][0],.maximum=bounds[i][1],.default_value=bounds[i][2],
        .step=(uint64_t)bounds[i][3],.elems=1,.elem_size=4};
    return p;
}
#endif
