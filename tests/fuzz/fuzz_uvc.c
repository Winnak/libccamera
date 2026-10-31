/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "../uvc_profiles.h"
#include "internal.h"
#include <errno.h>

static double binary_double(struct fuzz_input *in)
{
    uint64_t bits=fuzz_u64(in); double value;
    _Static_assert(sizeof(value)==sizeof(bits),"binary64 fuzz inputs");
    memcpy(&value,&bits,sizeof(value)); return value;
}
struct menu_driver { cc_control_info info; uint32_t index; int error; bool bad; };
static int menu_ioctl(void *context,int fd,unsigned long request,void *argument)
{
    struct menu_driver *d=context; FUZZ_CHECK(fd==7);
    if (request==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=argument;
        q->id=d->info.id; q->type=d->info.type; q->minimum=d->info.minimum; q->maximum=d->info.maximum;
        q->step=d->info.step; return 0;
    }
    FUZZ_CHECK(request==VIDIOC_QUERYMENU);
    struct v4l2_querymenu *q=argument; FUZZ_CHECK(q->index==d->index);
    if (d->error) return d->error;
    if (d->info.type==V4L2_CTRL_TYPE_INTEGER_MENU) q->value=INT64_MIN;
    else memset(q->name,'M',sizeof(q->name));
    if (d->bad) ++q->index;
    return 0;
}
int LLVMFuzzerTestOneInput(const uint8_t *data,size_t size)
{
    struct fuzz_input in={data,size}; uint8_t mode=fuzz_u8(&in);
    cc_uvc_profile p=test_uvc_profile();
    if (mode&1U) fuzz_patch(&in,&p.native[fuzz_u8(&in)%7U],sizeof(p.native[0]));
    if (mode&2U) { p.exposure_menu=fuzz_u32(&in); p.flags=fuzz_u32(&in); }
    cc_uvc_controls s={.flags=fuzz_u8(&in),.exposure_mode=(enum cc_uvc_exposure_mode)(fuzz_u8(&in)%3U),
        .brightness=binary_double(&in),.contrast=binary_double(&in),.saturation=binary_double(&in),
        .analogue_gain=binary_double(&in),.gamma=binary_double(&in),.exposure_us=fuzz_u64(&in)};
    if (!(mode&4U)) {
        s.flags&=127U; s.exposure_mode=(enum cc_uvc_exposure_mode)((uint32_t)s.exposure_mode%2U);
        s.brightness=(double)fuzz_u8(&in)/64-2; s.contrast=(double)fuzz_u8(&in)/32;
        s.saturation=(double)fuzz_u8(&in)/64; s.analogue_gain=(double)fuzz_u8(&in)/32; s.gamma=(double)fuzz_u8(&in)/32;
    }
    cc_control out[7],saved[7]; memset(out,0xa5,sizeof(out)); memcpy(saved,out,sizeof(saved));
    size_t count=fuzz_u8(&in)%8U,initial=count;
    int ret=cc_uvc_controls_to_native(&p,&s,out,&count);
    if (ret<0) {
        FUZZ_CHECK(!memcmp(out,saved,sizeof(out)));
        FUZZ_CHECK(ret==-ENOSPC ? count>initial && count<=7:count==initial);
    } else {
        uint32_t flags=0; FUZZ_CHECK(count<=7 && count<=initial);
        for (size_t n=0;n<count;++n) {
            size_t i=0; while (i<7 && out[n].id!=test_uvc_ids[i]) ++i;
            FUZZ_CHECK(i<7 && !(flags&(1U<<i)) && !out[n].data && !out[n].size);
            flags|=1U<<i; const cc_control_info *r=&p.native[i];
            FUZZ_CHECK(out[n].value>=r->minimum && out[n].value<=r->maximum);
            if (i==3) {
                FUZZ_CHECK(out[n].value<=3 && (p.exposure_menu&(1U<<(uint32_t)out[n].value)));
                FUZZ_CHECK((out[n].value==V4L2_EXPOSURE_AUTO || out[n].value==V4L2_EXPOSURE_APERTURE_PRIORITY)==
                    (s.exposure_mode==CC_UVC_EXPOSURE_AUTO));
            } else FUZZ_CHECK(((uint64_t)(out[n].value-r->minimum))%r->step==0);
        }
        FUZZ_CHECK(flags==s.flags);
    }
    for (uint32_t i=0;i<7;++i) {
        cc_uvc_range r={123,456,789},saved_range=r;
        ret=cc_uvc_get_control_range(&p,(enum cc_uvc_control_index)i,&r);
        if (ret) FUZZ_CHECK(!memcmp(&r,&saved_range,sizeof(r)));
        else FUZZ_CHECK(r.minimum<=r.default_value && r.default_value<=r.maximum);
    }
    struct menu_driver d={.info=p.native[fuzz_u8(&in)%7U],.index=fuzz_u32(&in),
        .bad=(mode&8U)!=0,.error=mode&16U ? -EIO:0};
    if (!(mode&32U)) {
        d.info.type=mode&64U ? V4L2_CTRL_TYPE_INTEGER_MENU:V4L2_CTRL_TYPE_MENU;
        d.info.minimum=0; d.info.maximum=UINT32_MAX;
    }
    static const struct cc_io_ops ops={.ioctl=menu_ioctl};
    cc_video_device device={.fd=7,.io={&ops,&d}};
    cc_control_menu entry,original; memset(&entry,0xa5,sizeof(entry)); memcpy(&original,&entry,sizeof(entry));
    ret=cc_video_query_menu(&device,d.info.id,d.index,&entry);
    if (ret) FUZZ_CHECK(!memcmp(&entry,&original,sizeof(entry)));
    else {
        FUZZ_CHECK(entry.id==d.info.id && entry.index==d.index && !d.error && !d.bad);
        if (d.info.type==V4L2_CTRL_TYPE_INTEGER_MENU) FUZZ_CHECK(entry.value==INT64_MIN && !entry.name[0]);
        else FUZZ_CHECK(entry.name[31]=='\0' && strlen(entry.name)==31 && !entry.value);
    }
    return 0;
}
