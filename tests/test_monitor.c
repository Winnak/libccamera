/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "monitor.h"
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/inotify.h>
#include <sys/resource.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
#define ERROR(x,e) CHECK((x)==-(e))

static void test_uevents(void)
{
    struct cc_monitor_notice notice,saved;
    const char added[]="add@/devices/test/video0\0ACTION=add\0DEVPATH=/devices/test/video0\0SUBSYSTEM=video4linux\0DEVNAME=video17";
    CHECK(cc_monitor_parse_uevent(added,sizeof(added),&notice)==1);
    CHECK(notice.type==CC_NOTICE_CHANGE && !strcmp(notice.path,"/dev/video17"));
    const char removed[]="remove@/devices/test\0ACTION=remove\0DEVPATH=/devices/test\0SUBSYSTEM=media\0DEVNAME=media2";
    CHECK(cc_monitor_parse_uevent(removed,sizeof(removed),&notice)==1 && notice.type==CC_NOTICE_REMOVE);
    CHECK(!strcmp(notice.path,"/dev/media2"));
    const char no_name[]="unbind@/devices/test\0ACTION=unbind\0DEVPATH=/devices/test\0SUBSYSTEM=video4linux";
    CHECK(cc_monitor_parse_uevent(no_name,sizeof(no_name),&notice)==1 && notice.type==CC_NOTICE_REMOVE && !notice.path[0]);
    const char other[]="add@/devices/test\0ACTION=add\0DEVPATH=/devices/test\0SUBSYSTEM=usb";
    const char ignored[]="online@/devices/test\0ACTION=online\0DEVPATH=/devices/test\0SUBSYSTEM=media";
    memset(&notice,0xa5,sizeof(notice)); saved=notice;
    CHECK(!cc_monitor_parse_uevent(other,sizeof(other),&notice) && !memcmp(&notice,&saved,sizeof(notice)));
    CHECK(!cc_monitor_parse_uevent(ignored,sizeof(ignored),&notice) && !memcmp(&notice,&saved,sizeof(notice)));
    for (size_t n=0; n<sizeof(added)-1; ++n) {
        notice=saved;
        int ret=cc_monitor_parse_uevent(added,n,&notice);
        /* A complete prefix through SUBSYSTEM is valid without DEVNAME. */
        if (ret==1) CHECK(notice.type==CC_NOTICE_CHANGE && !notice.path[0]);
        else CHECK(ret<0 && !memcmp(&notice,&saved,sizeof(notice)));
    }
    notice=saved;
    char packet[sizeof(added)];
    memcpy(packet,added,sizeof(packet)); packet[0]='x';
    ERROR(cc_monitor_parse_uevent(packet,sizeof(packet),&notice),EPROTO);
    const char duplicate[]="add@/x\0ACTION=add\0ACTION=add\0DEVPATH=/x\0SUBSYSTEM=media";
    const char mismatch[]="add@/x\0ACTION=remove\0DEVPATH=/x\0SUBSYSTEM=media";
    const char traversal[]="add@/x\0ACTION=add\0DEVPATH=/x\0SUBSYSTEM=media\0DEVNAME=../media1";
    const char empty[]="add@/x\0ACTION=add\0DEVPATH=/x\0SUBSYSTEM=media\0DEVNAME=";
    const char missing[]="add@/x\0ACTION=add\0DEVPATH=/x";
    ERROR(cc_monitor_parse_uevent(duplicate,sizeof(duplicate),&notice),EPROTO);
    ERROR(cc_monitor_parse_uevent(mismatch,sizeof(mismatch),&notice),EPROTO);
    ERROR(cc_monitor_parse_uevent(traversal,sizeof(traversal),&notice),EPROTO);
    ERROR(cc_monitor_parse_uevent(empty,sizeof(empty),&notice),EPROTO);
    ERROR(cc_monitor_parse_uevent(missing,sizeof(missing),&notice),EPROTO);
    ERROR(cc_monitor_parse_uevent(added,32769,&notice),EPROTO);
    ERROR(cc_monitor_parse_uevent(NULL,0,&notice),EINVAL);
    CHECK(!memcmp(&notice,&saved,sizeof(notice)));
}
static size_t inotify_packet(unsigned char *data,int wd,uint32_t mask,const char *name)
{
    size_t length=name ? strlen(name)+1:0;
    struct inotify_event event={.wd=wd,.mask=mask,.len=(uint32_t)length};
    memcpy(data,&event,sizeof(event));
    if (length) memcpy(data+sizeof(event),name,length);
    return sizeof(event)+length;
}
static void test_inotify(void)
{
    unsigned char storage[512]; unsigned char *data=storage+1; /* Unaligned input. */
    size_t n=inotify_packet(data,2,IN_CREATE,"unrelated");
    n+=inotify_packet(data+n,2,IN_CREATE|IN_ISDIR,"video0");
    n+=inotify_packet(data+n,3,IN_DELETE,"media0");
    size_t start=n; n+=inotify_packet(data+n,2,IN_MOVED_FROM,"v4l-subdev71");
    n+=inotify_packet(data+n,2,IN_ATTRIB,"video1");
    size_t offset=0; struct cc_monitor_notice notice;
    CHECK(cc_monitor_parse_inotify(data,n,2,&offset,&notice)==1 && offset>start);
    CHECK(notice.type==CC_NOTICE_REMOVE && !strcmp(notice.path,"/dev/v4l-subdev71"));
    CHECK(cc_monitor_parse_inotify(data,n,2,&offset,&notice)==1);
    CHECK(notice.type==CC_NOTICE_CHANGE && !strcmp(notice.path,"/dev/video1"));
    CHECK(!cc_monitor_parse_inotify(data,n,2,&offset,&notice) && offset==n);
    n=inotify_packet(data,-1,IN_Q_OVERFLOW,NULL); offset=0;
    CHECK(cc_monitor_parse_inotify(data,n,2,&offset,&notice)==1 && notice.type==CC_NOTICE_LOST && !notice.path[0]);
    n=inotify_packet(data,2,IN_IGNORED,NULL); offset=0;
    CHECK(cc_monitor_parse_inotify(data,n,2,&offset,&notice)==1 && notice.type==CC_NOTICE_LOST && !strcmp(notice.path,"/dev"));
    n=inotify_packet(data,2,IN_CREATE,"video0");
    for (size_t i=1; i<n; ++i) {
        offset=0; memset(&notice,0xa5,sizeof(notice)); struct cc_monitor_notice saved=notice;
        ERROR(cc_monitor_parse_inotify(data,i,2,&offset,&notice),EPROTO);
        CHECK(!offset && !memcmp(&notice,&saved,sizeof(notice)));
    }
    data[n-1]='x'; offset=0;
    ERROR(cc_monitor_parse_inotify(data,n,2,&offset,&notice),EPROTO);
    offset=n+1; ERROR(cc_monitor_parse_inotify(data,n,2,&offset,&notice),EINVAL);
    struct inotify_event huge={.wd=2,.mask=IN_CREATE,.len=UINT32_MAX};
    memcpy(data,&huge,sizeof(huge)); offset=0;
    ERROR(cc_monitor_parse_inotify(data,sizeof(huge),2,&offset,&notice),EPROTO);
}

struct fake {
    unsigned count,shift,identity,opens,closes,closed,waits,interrupts,wait_change;
    int open_error,probe_error,scan_error,ready_error,retry_error,next_error,clock_error;
    bool ready,missing,identity_missing,rename,spurious;
    uint64_t inode,now;
    uint32_t retry;
    struct cc_monitor_notice notices[600]; size_t head,tail;
};
static int scan(void *ctx,char ***out,size_t *count)
{
    struct fake *f=ctx;
    if (f->scan_error) return -f->scan_error;
    char **list=calloc(f->count ? f->count:1,sizeof(*list)); CHECK(list);
    for (unsigned i=0; i<f->count; ++i) {
        char path[64]; snprintf(path,sizeof(path),"/dev/video%u",i+f->shift);
        list[i]=strdup(path); CHECK(list[i]);
    }
    *out=list; *count=f->count; return 0;
}
static int open_node(void *ctx,const char *path,int flags)
{
    struct fake *f=ctx; CHECK(flags&O_CLOEXEC);
    if (f->open_error) return -f->open_error;
    unsigned index=0; CHECK(sscanf(path,"/dev/video%u",&index)==1 && index>=f->shift && index-f->shift<f->count);
    ++f->opens; return (int)(index-f->shift); /* Include descriptor zero. */
}
static int close_node(void *ctx,int fd) { struct fake *f=ctx; CHECK(fd>=0); ++f->closes; return 0; }
static int ioctl_node(void *ctx,int fd,unsigned long req,void *arg)
{
    struct fake *f=ctx; CHECK(fd>=0 && req==VIDIOC_QUERYCAP);
    struct v4l2_capability *caps=arg;
    *caps=(struct v4l2_capability){.capabilities=V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_STREAMING};
    strcpy((char *)caps->driver,"test-video"); strcpy((char *)caps->card,f->rename ? "renamed":"test camera");
    return 0;
}
static int device_number(void *ctx,int fd,unsigned *major_id,unsigned *minor_id)
{
    (void)ctx; *major_id=81; *minor_id=(unsigned)fd; return 0;
}
static int canonical(void *ctx,const char *path,char *out,size_t size)
{
    struct fake *f=ctx;
    if (f->identity_missing) return -ENOTSUP;
    unsigned index; int consumed=0;
    if (sscanf(path,"/sys/dev/char/81:%u/device%n",&index,&consumed)!=1 || path[consumed]) return -ENOENT;
    int n=snprintf(out,size,"/sys/devices/platform/test%u-%u",f->identity,index);
    CHECK(n>0 && (size_t)n<size); return 0;
}
static int text(void *ctx,const char *path,char *out,size_t size)
{
    (void)ctx;
    if (!strstr(path,"/index")) return -ENOENT;
    CHECK(size>=2); memcpy(out,"0",2); return 0;
}
static const struct cc_io_ops io_ops={.scan_nodes=scan,.open=open_node,.close=close_node,
    .ioctl=ioctl_node,.device_number=device_number,.canonical=canonical,.read_text=text};
static int next_notice(void *ctx,struct cc_monitor_notice *out)
{
    struct fake *f=ctx;
    if (f->next_error) return -f->next_error;
    if (f->head==f->tail) return 0;
    *out=f->notices[f->head++]; return 1;
}
static int ready(void *ctx,bool value)
{
    struct fake *f=ctx; if (f->ready_error) return -f->ready_error; f->ready=value; return 0;
}
static int retry(void *ctx,uint32_t delay)
{
    struct fake *f=ctx; if (f->retry_error) return -f->retry_error; f->retry=delay; return 0;
}
static int probe(void *ctx,const char *path,struct cc_node_token *out)
{
    struct fake *f=ctx;
    if (f->probe_error) return -f->probe_error;
    unsigned index;
    CHECK(sscanf(path,"/dev/video%u",&index)==1);
    if (f->missing || index<f->shift || index-f->shift>=f->count) return -ENOENT;
    *out=(struct cc_node_token){index,f->inode+index}; return 0;
}
static void notice(struct fake *f,enum cc_notice_type type,const char *path)
{
    if (f->head==f->tail) f->head=f->tail=0;
    CHECK(f->tail<600);
    struct cc_monitor_notice *n=&f->notices[f->tail++]; *n=(struct cc_monitor_notice){.type=type};
    if (path) { CHECK(strlen(path)<sizeof(n->path)); strcpy(n->path,path); }
}
static int wait_notice(void *ctx,int timeout)
{
    struct fake *f=ctx; CHECK(timeout>=-1); ++f->waits;
    if (f->interrupts) { --f->interrupts; f->now+=1000000; return -EINTR; }
    if (f->wait_change) { --f->wait_change; ++f->count; notice(f,CC_NOTICE_CHANGE,NULL); return 0; }
    if (f->spurious) { f->now+=3000000; notice(f,CC_NOTICE_CHANGE,NULL); return 0; }
    CHECK(timeout>0); f->now+=(uint64_t)timeout*1000000; return -ETIMEDOUT;
}
static int now(void *ctx,uint64_t *out)
{
    struct fake *f=ctx; if (f->clock_error) return -f->clock_error; *out=f->now; return 0;
}
static void close_backend(void *ctx) { ++((struct fake *)ctx)->closed; }
static const struct cc_monitor_backend_ops backend_ops={.next=next_notice,.ready=ready,.retry=retry,
    .probe=probe,.wait=wait_notice,.now=now,.close=close_backend};
static cc_camera_monitor *create(struct fake *f)
{
    struct cc_io io={&io_ops,f}; struct cc_monitor_backend b={&backend_ops,f,0};
    cc_camera_monitor *m=NULL; OK(cc_camera_monitor_create_io(&io,&b,&m));
    CHECK(m && !b.ops && b.fd==-1 && !cc_camera_monitor_fd(m)); return m;
}
static uint64_t snapshot(cc_camera_monitor *m,size_t expected,cc_camera_info *first)
{
    cc_camera_info *list=NULL; size_t count=SIZE_MAX; uint64_t generation=0;
    OK(cc_camera_monitor_snapshot(m,&list,&count,&generation)); CHECK(count==expected && generation);
    if (first) { CHECK(count); *first=list[0]; }
    cc_camera_list_free(list); return generation;
}
static cc_camera_event take(cc_camera_monitor *m,enum cc_camera_event_type type)
{
    cc_camera_event event; OK(cc_camera_monitor_dequeue(m,&event,0)); CHECK(event.type==type); return event;
}
static void empty(cc_camera_monitor *m)
{
    cc_camera_event event; memset(&event,0xa5,sizeof(event)); cc_camera_event saved=event;
    ERROR(cc_camera_monitor_dequeue(m,&event,0),EAGAIN); CHECK(!memcmp(&event,&saved,sizeof(event)));
}
static void test_registry(void)
{
    struct fake f={.count=1,.inode=100}; cc_camera_monitor *m=create(&f);
    CHECK(f.ready && !f.retry);
    cc_camera_info original; uint64_t generation=snapshot(m,1,&original); CHECK(generation==1);
    CHECK(!strcmp(original.id,"platform/test0-0-video0"));
    cc_camera_event event=take(m,CC_CAMERA_ADDED); CHECK(event.generation==1 && !strcmp(event.camera.id,original.id));
    CHECK(!f.ready); empty(m);
    OK(cc_camera_monitor_refresh(m)); CHECK(snapshot(m,1,NULL)==generation); empty(m);
    /* Access and sysfs failures do not invent disconnect/reconnect pairs. */
    f.open_error=EBUSY; notice(&f,CC_NOTICE_CHANGE,NULL); OK(cc_camera_monitor_dispatch(m));
    CHECK(snapshot(m,1,NULL)==generation && f.retry); empty(m);
    f.open_error=EACCES; OK(cc_camera_monitor_refresh(m)); CHECK(snapshot(m,1,NULL)==generation); empty(m);
    f.open_error=0; f.identity_missing=true; OK(cc_camera_monitor_refresh(m));
    cc_camera_info current; CHECK(snapshot(m,1,&current)==generation && !strcmp(current.id,original.id)); empty(m);
    f.identity_missing=false; f.rename=true; notice(&f,CC_NOTICE_CHANGE,"/dev/video0"); OK(cc_camera_monitor_dispatch(m));
    event=take(m,CC_CAMERA_CHANGED); CHECK(!strcmp(event.previous.name,original.name) && !strcmp(event.camera.name,"renamed"));
    CHECK(event.generation==++generation && snapshot(m,1,NULL)==generation); empty(m);
    /* Observe both edges even when removal and replacement happen between dispatches. */
    notice(&f,CC_NOTICE_REMOVE,"/dev/video0"); notice(&f,CC_NOTICE_CHANGE,"/dev/video0");
    OK(cc_camera_monitor_dispatch(m)); event=take(m,CC_CAMERA_REMOVED); CHECK(event.generation==generation+1 && f.ready);
    event=take(m,CC_CAMERA_ADDED); CHECK(event.generation==++generation && !f.ready); empty(m);
    notice(&f,CC_NOTICE_REMOVE,"/dev/video0"); OK(cc_camera_monitor_dispatch(m));
    take(m,CC_CAMERA_REMOVED); CHECK(snapshot(m,0,NULL)==++generation);
    OK(cc_camera_monitor_refresh(m)); CHECK(snapshot(m,0,NULL)==generation); empty(m);
    notice(&f,CC_NOTICE_CHANGE,"/dev/video0"); OK(cc_camera_monitor_dispatch(m));
    take(m,CC_CAMERA_ADDED); CHECK(snapshot(m,1,NULL)==++generation);
    /* An unobserved inode replacement is also a reconnect. */
    ++f.inode; OK(cc_camera_monitor_refresh(m)); take(m,CC_CAMERA_REMOVED); take(m,CC_CAMERA_ADDED); ++generation;
    f.identity=1; OK(cc_camera_monitor_refresh(m)); event=take(m,CC_CAMERA_REMOVED);
    CHECK(!strcmp(event.camera.id,original.id)); event=take(m,CC_CAMERA_ADDED); CHECK(strcmp(event.camera.id,original.id)); ++generation;
    f.shift=10; notice(&f,CC_NOTICE_CHANGE,NULL); OK(cc_camera_monitor_dispatch(m));
    event=take(m,CC_CAMERA_REMOVED); CHECK(!strcmp(event.camera.video_path,"/dev/video0"));
    event=take(m,CC_CAMERA_ADDED); CHECK(!strcmp(event.camera.video_path,"/dev/video10")); ++generation;
    f.count=0; OK(cc_camera_monitor_refresh(m)); take(m,CC_CAMERA_REMOVED); CHECK(snapshot(m,0,NULL)==++generation);
    /* New cameras wait until their device node is usable. */
    f.count=1; f.probe_error=EACCES; OK(cc_camera_monitor_refresh(m)); CHECK(snapshot(m,0,NULL)==generation && f.retry); empty(m);
    f.probe_error=0; notice(&f,CC_NOTICE_TIMER,NULL); OK(cc_camera_monitor_dispatch(m)); take(m,CC_CAMERA_ADDED);
    CHECK(snapshot(m,1,NULL)==++generation);
    notice(&f,CC_NOTICE_LOST,NULL); OK(cc_camera_monitor_dispatch(m)); event=take(m,CC_CAMERA_RESYNC);
    CHECK(event.flags==CC_CAMERA_EVENTS_LOST && event.generation==++generation && snapshot(m,1,NULL)==generation);
    empty(m); cc_camera_monitor_close(m); CHECK(f.closed==1 && f.opens==f.closes);
}
static void test_atomicity(void)
{
    struct fake f={.count=1}; cc_camera_monitor *m=create(&f); take(m,CC_CAMERA_ADDED);
    cc_camera_info original; uint64_t generation=snapshot(m,1,&original);
    f.rename=true; f.scan_error=ENOMEM; notice(&f,CC_NOTICE_REMOVE,"/dev/video0");
    notice(&f,CC_NOTICE_CHANGE,"/dev/video0");
    ERROR(cc_camera_monitor_dispatch(m),ENOMEM); CHECK(snapshot(m,1,NULL)==generation && f.retry==1000);
    f.scan_error=0; f.retry_error=EIO;
    ERROR(cc_camera_monitor_dispatch(m),EIO); CHECK(snapshot(m,1,NULL)==generation);
    f.retry_error=0; f.ready_error=EIO;
    ERROR(cc_camera_monitor_dispatch(m),EIO); CHECK(snapshot(m,1,NULL)==generation);
    f.ready_error=0; OK(cc_camera_monitor_dispatch(m));
    cc_camera_event event=take(m,CC_CAMERA_REMOVED); CHECK(!strcmp(event.camera.name,original.name));
    f.ready_error=EIO; memset(&event,0xa5,sizeof(event)); cc_camera_event saved=event;
    ERROR(cc_camera_monitor_dequeue(m,&event,0),EIO); CHECK(!memcmp(&event,&saved,sizeof(event)));
    f.ready_error=0; event=take(m,CC_CAMERA_ADDED); CHECK(!strcmp(event.camera.name,"renamed")); empty(m);
    f.probe_error=EINVAL; ERROR(cc_camera_monitor_refresh(m),EINVAL); CHECK(snapshot(m,1,NULL)==generation+1);
    f.probe_error=0; OK(cc_camera_monitor_dispatch(m)); empty(m);
    f.next_error=EIO; ERROR(cc_camera_monitor_dispatch(m),EIO); f.next_error=0;
    notice(&f,CC_NOTICE_LOST,NULL); f.scan_error=EIO;
    ERROR(cc_camera_monitor_dispatch(m),EIO); CHECK(snapshot(m,1,NULL)==generation+1);
    f.scan_error=0; OK(cc_camera_monitor_dispatch(m)); event=take(m,CC_CAMERA_RESYNC);
    CHECK(event.flags==CC_CAMERA_EVENTS_LOST); cc_camera_monitor_close(m);
    CHECK(f.closed==1 && f.opens==f.closes);
    /* A failed creation retains ownership of the supplied backend. */
    struct cc_io io={&io_ops,&f}; struct cc_monitor_backend b={&backend_ops,&f,0};
    f.scan_error=EIO; m=(cc_camera_monitor *)(uintptr_t)1;
    ERROR(cc_camera_monitor_create_io(&io,&b,&m),EIO); CHECK(!m && b.ops && f.closed==1);
}
static void test_deadlines_and_overflow(void)
{
    struct fake f={0}; cc_camera_monitor *m=create(&f); snapshot(m,0,NULL);
    cc_camera_event event,saved; memset(&event,0xa5,sizeof(event)); saved=event;
    f.interrupts=3; ERROR(cc_camera_monitor_dequeue(m,&event,10),ETIMEDOUT);
    CHECK(f.now==10000000 && f.waits==4 && !memcmp(&event,&saved,sizeof(event)));
    f.spurious=true; uint64_t start=f.now;
    ERROR(cc_camera_monitor_dequeue(m,&event,10),ETIMEDOUT); CHECK(f.now-start==12000000);
    f.spurious=false; f.wait_change=1;
    OK(cc_camera_monitor_dequeue(m,&event,-1)); CHECK(event.type==CC_CAMERA_ADDED);
    f.clock_error=EIO; ERROR(cc_camera_monitor_dequeue(m,&event,10),EIO); f.clock_error=0;
    /* More than a dispatch batch remains available for the following call. */
    for (unsigned i=0; i<300; ++i) notice(&f,CC_NOTICE_CHANGE,NULL);
    OK(cc_camera_monitor_dispatch(m)); CHECK(f.tail-f.head==44); OK(cc_camera_monitor_dispatch(m)); CHECK(f.tail==f.head);
    /* Accumulated application events overflow into an explicit snapshot marker. */
    for (unsigned i=0; i<1030; ++i) { f.rename=!f.rename; OK(cc_camera_monitor_refresh(m)); }
    event=take(m,CC_CAMERA_RESYNC); CHECK(event.flags==(CC_CAMERA_EVENTS_LOST|CC_CAMERA_EVENT_QUEUE_OVERFLOW));
    CHECK(snapshot(m,1,NULL)>event.generation);
    while (!cc_camera_monitor_dequeue(m,&event,0)) CHECK(event.type==CC_CAMERA_CHANGED);
    cc_camera_monitor_close(m); CHECK(f.opens==f.closes && f.closed==1);
    f=(struct fake){.count=1025}; m=create(&f);
    event=take(m,CC_CAMERA_RESYNC); CHECK(event.flags&CC_CAMERA_EVENT_QUEUE_OVERFLOW); snapshot(m,1025,NULL); empty(m);
    cc_camera_monitor_close(m); CHECK(f.opens==f.closes && f.closed==1);
}
static void test_arguments(void)
{
    cc_camera_event event; cc_camera_info *list=(cc_camera_info *)(uintptr_t)1; size_t count=7; uint64_t generation=9;
    ERROR(cc_camera_monitor_open(NULL),EINVAL); cc_camera_monitor_close(NULL);
    ERROR(cc_camera_monitor_fd(NULL),EINVAL); ERROR(cc_camera_monitor_dispatch(NULL),EINVAL);
    ERROR(cc_camera_monitor_refresh(NULL),EINVAL); ERROR(cc_camera_monitor_dequeue(NULL,&event,0),EINVAL);
    ERROR(cc_camera_monitor_snapshot(NULL,&list,&count,&generation),EINVAL);
    CHECK(list==(cc_camera_info *)(uintptr_t)1 && count==7 && generation==9);
    struct fake f={0}; cc_camera_monitor *m=create(&f);
    ERROR(cc_camera_monitor_dequeue(m,&event,-2),EINVAL); ERROR(cc_camera_monitor_dequeue(m,NULL,0),EINVAL);
    ERROR(cc_camera_monitor_snapshot(m,NULL,&count,&generation),EINVAL);
    ERROR(cc_camera_monitor_snapshot(m,&list,NULL,&generation),EINVAL);
    ERROR(cc_camera_monitor_snapshot(m,&list,&count,NULL),EINVAL);
    cc_camera_monitor_close(m);
}
static void test_settling(void)
{
    struct fake f={.count=1,.open_error=EACCES}; cc_camera_monitor *m=create(&f);
    CHECK(f.retry==20); snapshot(m,0,NULL);
    const uint32_t backoff[]={40,80,160,320,640,1000,1000,1000};
    for (size_t i=0; i<sizeof(backoff)/sizeof(backoff[0]); ++i) {
        notice(&f,CC_NOTICE_TIMER,NULL); OK(cc_camera_monitor_dispatch(m)); CHECK(f.retry==backoff[i]);
    }
    f.open_error=0; notice(&f,CC_NOTICE_TIMER,NULL); OK(cc_camera_monitor_dispatch(m));
    take(m,CC_CAMERA_ADDED); CHECK(!f.retry);
    notice(&f,CC_NOTICE_CHANGE,NULL); OK(cc_camera_monitor_dispatch(m)); CHECK(f.retry==20);
    for (unsigned i=0; i<8; ++i) { notice(&f,CC_NOTICE_TIMER,NULL); OK(cc_camera_monitor_dispatch(m)); }
    CHECK(!f.retry); empty(m); cc_camera_monitor_close(m);
    /* A failing refresh must also preserve already queued event history. */
    f=(struct fake){.count=1}; m=create(&f); f.rename=true; f.scan_error=EIO;
    ERROR(cc_camera_monitor_refresh(m),EIO); CHECK(snapshot(m,1,NULL)==1);
    cc_camera_event event=take(m,CC_CAMERA_ADDED); CHECK(!strcmp(event.camera.name,"test camera"));
    f.scan_error=0; OK(cc_camera_monitor_dispatch(m)); event=take(m,CC_CAMERA_CHANGED);
    CHECK(!strcmp(event.previous.name,"test camera") && !strcmp(event.camera.name,"renamed"));
    cc_camera_monitor_close(m); CHECK(f.opens==f.closes);
}
static void test_native(void)
{
    struct cc_monitor_backend b;
    OK(cc_monitor_native_open(&b)); CHECK(fcntl(b.fd,F_GETFD)&FD_CLOEXEC);
    struct cc_node_token token; OK(b.ops->probe(b.ctx,"/dev/null",&token)); CHECK(token.device);
    ERROR(b.ops->probe(b.ctx,"/dev",&token),ENODEV);
    OK(b.ops->ready(b.ctx,true)); OK(b.ops->wait(b.ctx,0));
    OK(b.ops->ready(b.ctx,true)); OK(b.ops->ready(b.ctx,false));
    ERROR(b.ops->wait(b.ctx,1),ETIMEDOUT);
    OK(b.ops->retry(b.ctx,1)); OK(b.ops->wait(b.ctx,1000));
    struct cc_monitor_notice notice;
    CHECK(b.ops->next(b.ctx,&notice)==1 && notice.type==CC_NOTICE_TIMER);
    CHECK(!b.ops->next(b.ctx,&notice)); ERROR(b.ops->wait(b.ctx,1),ETIMEDOUT);
    OK(b.ops->retry(b.ctx,0)); b.ops->close(b.ctx);
    /* Public lifecycle exercises real Linux subscription and enumeration. No
     * device files or driver configuration are modified. */
    cc_camera_monitor *m=NULL; OK(cc_camera_monitor_open(&m));
    cc_camera_info *list=NULL; size_t count=0; uint64_t generation=0;
    OK(cc_camera_monitor_snapshot(m,&list,&count,&generation)); CHECK(generation==1);
    cc_camera_list_free(list); OK(cc_camera_monitor_dispatch(m)); cc_camera_monitor_close(m);
    /* Failure after partial setup must close all descriptors already acquired. */
    DIR *directory=opendir("/proc/self/fd"); CHECK(directory);
    int before=0; while (readdir(directory)) ++before; CHECK(!closedir(directory));
    struct rlimit limit; CHECK(!getrlimit(RLIMIT_NOFILE,&limit));
    int lowest=open("/dev/null",O_RDONLY|O_CLOEXEC); CHECK(lowest>=0); CHECK(!close(lowest));
    struct rlimit limited=limit; limited.rlim_cur=(rlim_t)lowest+2;
    CHECK(limited.rlim_cur<=limit.rlim_cur && !setrlimit(RLIMIT_NOFILE,&limited));
    int result=cc_monitor_native_open(&b);
    CHECK(!setrlimit(RLIMIT_NOFILE,&limit)); CHECK(result==-EMFILE);
    directory=opendir("/proc/self/fd"); CHECK(directory);
    int after=0; while (readdir(directory)) ++after; CHECK(!closedir(directory) && before==after);
    int saved=fcntl(STDIN_FILENO,F_DUPFD_CLOEXEC,3); CHECK(saved>=0);
    CHECK(!close(STDIN_FILENO)); OK(cc_monitor_native_open(&b)); b.ops->close(b.ctx);
    CHECK(dup2(saved,STDIN_FILENO)==STDIN_FILENO && !close(saved));
}
int main(int argc,char **argv)
{
    if (argc==2 && !strcmp(argv[1],"native")) { test_native(); puts("Linux notification descriptors, readiness, timers and public lifecycle passed"); return 0; }
    test_uevents(); test_inotify(); test_arguments(); test_registry(); test_atomicity(); test_deadlines_and_overflow(); test_settling();
    puts("Hotplug parsing, registry, reconnects, atomic publication, deadlines and overflow passed"); return 0;
}
