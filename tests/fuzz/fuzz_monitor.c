/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "monitor.h"
#include <sys/inotify.h>

static void uevent(const uint8_t *data,size_t size)
{
    struct cc_monitor_notice notice; memset(&notice,0xa5,sizeof(notice));
    struct cc_monitor_notice saved=notice;
    int ret=cc_monitor_parse_uevent(data,size,&notice);
    FUZZ_CHECK(ret<=1);
    if (ret!=1) FUZZ_CHECK(!memcmp(&notice,&saved,sizeof(notice)));
    else {
        FUZZ_CHECK(notice.type==CC_NOTICE_CHANGE || notice.type==CC_NOTICE_REMOVE);
        FUZZ_CHECK(memchr(notice.path,0,sizeof(notice.path)));
        FUZZ_CHECK(!notice.path[0] || !strncmp(notice.path,"/dev/",5));
    }
}
static void inotify(const uint8_t *data,size_t size,size_t offset)
{
    while (offset<=size) {
        size_t before=offset;
        struct cc_monitor_notice notice; memset(&notice,0xa5,sizeof(notice));
        struct cc_monitor_notice saved=notice;
        int ret=cc_monitor_parse_inotify(data,size,0,&offset,&notice);
        FUZZ_CHECK(ret<=1);
        if (ret<0) FUZZ_CHECK(offset==before);
        else FUZZ_CHECK(offset<=size);
        if (ret!=1) { FUZZ_CHECK(!memcmp(&notice,&saved,sizeof(notice))); break; }
        FUZZ_CHECK(offset>before);
        FUZZ_CHECK(notice.type>=CC_NOTICE_CHANGE && notice.type<=CC_NOTICE_LOST);
        FUZZ_CHECK(memchr(notice.path,0,sizeof(notice.path)));
        FUZZ_CHECK(notice.type==CC_NOTICE_LOST || !strncmp(notice.path,"/dev/",5));
    }
}
int LLVMFuzzerTestOneInput(const uint8_t *data,size_t size)
{
    if (!size) return 0;
    unsigned mode=*data++%4; --size;
    if (!mode) uevent(data,size);
    else if (mode==1) inotify(data,size,0);
    else if (mode==2) {
        static const char prefix[]="add@/x\0ACTION=add\0DEVPATH=/x\0SUBSYSTEM=video4linux\0DEVNAME=";
        if (size>32700) return 0;
        uint8_t *packet=malloc(sizeof(prefix)+size); FUZZ_CHECK(packet);
        memcpy(packet,prefix,sizeof(prefix)-1); memcpy(packet+sizeof(prefix)-1,data,size);
        packet[sizeof(prefix)-1+size]=0;
        uevent(packet,sizeof(prefix)+size); free(packet);
    } else {
        struct fuzz_input input={data,size};
        struct inotify_event event={.wd=(int8_t)fuzz_u8(&input),.mask=fuzz_u32(&input)};
        if (input.size>32700) return 0;
        event.len=(uint32_t)input.size;
        uint8_t *packet=malloc(sizeof(event)+input.size); FUZZ_CHECK(packet);
        memcpy(packet,&event,sizeof(event)); memcpy(packet+sizeof(event),input.data,input.size);
        inotify(packet,sizeof(event)+input.size,0); free(packet);
    }
    return 0;
}
