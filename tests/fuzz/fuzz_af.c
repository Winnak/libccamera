/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include <ccamera/af.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <linux/videodev2.h>
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in={data,size}; uint8_t mode=fuzz_u8(&in);
    cc_af_tuning tuning; cc_af_reference_tuning(&tuning);
    if (mode & 1U) fuzz_patch(&in,&tuning,sizeof(tuning));
    cc_af *a=NULL; int ret=cc_af_create(&tuning,&a);
    FUZZ_CHECK(ret<=0); if (ret) { FUZZ_CHECK(!a); return 0; }
    cc_rectangle region={0,0,640,480};
    cc_control_info lens={.id=V4L2_CID_FOCUS_ABSOLUTE,.type=V4L2_CTRL_TYPE_INTEGER,
        .minimum=0,.maximum=1023,.step=1,.elems=1};
    if (cc_af_configure(a,&region,&lens)<0) { cc_af_destroy(a); return 0; }
    cc_af_pdaf phase[12]; cc_af_colour colour[12]; uint64_t focus[12];
    uint32_t count=1U+(fuzz_u8(&in)%64U);
    for (uint32_t frame=0;frame<count;++frame) {
        cc_af_controls controls={.flags=CC_AF_SET_MODE|CC_AF_SET_LENS,.mode=CC_AF_CONTINUOUS};
        controls.mode=(enum cc_af_mode)(fuzz_u8(&in)%3U); controls.lens_position=(double)fuzz_u16(&in)/100;
        controls.flags|=fuzz_u16(&in)&511U;
        controls.range=(enum cc_af_range)(fuzz_u8(&in)%3U); controls.speed=(enum cc_af_speed)(fuzz_u8(&in)%2U);
        controls.pause=(enum cc_af_pause)(fuzz_u8(&in)%3U);
        controls.use_windows=fuzz_u8(&in)%2U; controls.num_windows=fuzz_u8(&in)%11U;
        for (uint32_t i=0;i<controls.num_windows;++i)
            controls.windows[i]=(cc_rectangle){(int32_t)fuzz_u32(&in),(int32_t)fuzz_u32(&in),fuzz_u32(&in),fuzz_u32(&in)};
        FUZZ_CHECK(cc_af_set_controls(a,&controls)<=0);
        for (size_t i=0;i<12;++i) {
            phase[i]=(cc_af_pdaf){fuzz_u16(&in),(int16_t)fuzz_u16(&in)};
            focus[i]=fuzz_u64(&in);
            colour[i]=(cc_af_colour){fuzz_u64(&in),fuzz_u64(&in),fuzz_u64(&in),fuzz_u32(&in)};
        }
        cc_af_pdaf_grid g={3,4,phase}; cc_af_status status;
        FUZZ_CHECK(cc_af_prepare(a,mode & 2U ? &g:NULL,&status)==0);
        FUZZ_CHECK(status.state>=CC_AF_IDLE && status.state<=CC_AF_FAILED && status.pause_state<=CC_AF_PAUSED);
        if (status.flags & CC_AF_LENS_VALID) {
            FUZZ_CHECK(status.lens_setting>=0 && status.lens_setting<=1023);
            FUZZ_CHECK(status.lens_position>=-1000 && status.lens_position<=1000);
        }
        cc_af_statistics stats={3,4,3,4,focus,colour}; FUZZ_CHECK(cc_af_process(a,&stats)<=0);
        if (mode & 4U) {
            struct pisp_statistics p={0}; fuzz_patch(&in,&p,sizeof(p));
            FUZZ_CHECK(cc_af_process_native(a,CC_FOURCC('R','P','F','S'),&p,sizeof(p))<=0);
        }
    }
    cc_af_destroy(a); return 0;
}
