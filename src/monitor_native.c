/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "monitor.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/netlink.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/inotify.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/timerfd.h>
#include <unistd.h>

#define NOTICE_BYTES 32768U
static bool node_name(const char *name)
{
    const char *prefixes[]={"media","video","v4l-subdev"};
    for (size_t i=0; i<3; ++i) {
        size_t n=strlen(prefixes[i]);
        if (strncmp(name,prefixes[i],n) || !name[n]) continue;
        const char *p=name+n;
        while (*p>='0' && *p<='9') ++p;
        if (!*p) return true;
    }
    return false;
}

int cc_monitor_parse_uevent(const void *data,size_t size,struct cc_monitor_notice *out)
{
    if (!data || !out) return -EINVAL;
    if (!size || size>NOTICE_BYTES) return -EPROTO;
    const char *bytes=data,*end=bytes+size,*header_end=memchr(bytes,'\0',size);
    if (!header_end || header_end==bytes) return -EPROTO;
    const char *at=memchr(bytes,'@',(size_t)(header_end-bytes));
    if (!at || at==bytes || at+1==header_end || at[1]!='/') return -EPROTO;
    const char *action=NULL,*path=NULL,*subsystem=NULL,*name=NULL;
    for (const char *p=header_end+1; p<end;) {
        const char *nul=memchr(p,'\0',(size_t)(end-p));
        if (!nul) return -EPROTO;
        const char **slot=NULL; size_t prefix=0;
        if (!strncmp(p,"ACTION=",7) && (size_t)(nul-p)>=7) { slot=&action; prefix=7; }
        else if (!strncmp(p,"DEVPATH=",8) && (size_t)(nul-p)>=8) { slot=&path; prefix=8; }
        else if (!strncmp(p,"SUBSYSTEM=",10) && (size_t)(nul-p)>=10) { slot=&subsystem; prefix=10; }
        else if (!strncmp(p,"DEVNAME=",8) && (size_t)(nul-p)>=8) { slot=&name; prefix=8; }
        if (slot) { if (*slot || p+prefix==nul) return -EPROTO; *slot=p+prefix; }
        p=nul+1;
    }
    if (!action || !path || !subsystem || strlen(action)!=(size_t)(at-bytes) ||
        memcmp(action,bytes,(size_t)(at-bytes)) || strcmp(path,at+1)) return -EPROTO;
    if (strcmp(subsystem,"media") && strcmp(subsystem,"video4linux")) return 0;
    bool removal=!strcmp(action,"remove") || !strcmp(action,"unbind");
    if (!removal && strcmp(action,"add") && strcmp(action,"change") && strcmp(action,"bind") && strcmp(action,"move")) return 0;
    struct cc_monitor_notice notice={.type=removal ? CC_NOTICE_REMOVE:CC_NOTICE_CHANGE};
    if (name) {
        if (!node_name(name)) return -EPROTO;
        int n=snprintf(notice.path,sizeof(notice.path),"/dev/%s",name);
        if (n<0 || (size_t)n>=sizeof(notice.path)) return -E2BIG;
    }
    *out=notice;
    return 1;
}

int cc_monitor_parse_inotify(const void *data,size_t size,int watch,size_t *offset,struct cc_monitor_notice *out)
{
    if ((!data && size) || !offset || !out || *offset>size) return -EINVAL;
    const unsigned char *bytes=data;
    size_t pos=*offset;
    while (pos<size) {
        struct inotify_event event;
        if (size-pos<sizeof(event)) return -EPROTO;
        memcpy(&event,bytes+pos,sizeof(event));
        if ((size_t)event.len>size-pos-sizeof(event)) return -EPROTO;
        const char *name=(const char *)bytes+pos+sizeof(event);
        pos+=sizeof(event)+event.len;
        struct cc_monitor_notice notice={0};
        if (event.mask&IN_Q_OVERFLOW) notice.type=CC_NOTICE_LOST;
        else if (event.wd!=watch) continue;
        else if (event.mask&(IN_IGNORED|IN_DELETE_SELF|IN_MOVE_SELF)) {
            notice.type=CC_NOTICE_LOST; memcpy(notice.path,"/dev",5);
        } else {
            if (!event.len || !memchr(name,'\0',event.len)) return -EPROTO;
            if (event.mask&IN_ISDIR || !node_name(name)) continue;
            if (event.mask&(IN_DELETE|IN_MOVED_FROM)) notice.type=CC_NOTICE_REMOVE;
            else if (event.mask&(IN_CREATE|IN_MOVED_TO|IN_ATTRIB)) notice.type=CC_NOTICE_CHANGE;
            else continue;
            int n=snprintf(notice.path,sizeof(notice.path),"/dev/%s",name);
            if (n<0 || (size_t)n>=sizeof(notice.path)) return -E2BIG;
        }
        *offset=pos; *out=notice; return 1;
    }
    *offset=pos;
    return 0;
}

struct native_monitor {
    int poll_fd,netlink_fd,inotify_fd,timer_fd,wake_fd,watch;
    bool queued,wake_on;
    unsigned turn;
    unsigned char buffer[NOTICE_BYTES];
    size_t used,offset;
};

static int wake_update(struct native_monitor *b)
{
    bool ready=b->queued || b->offset<b->used;
    if (ready==b->wake_on) return 0;
    uint64_t value=1;
    ssize_t ret;
    do { ret=ready ? write(b->wake_fd,&value,sizeof(value)):read(b->wake_fd,&value,sizeof(value)); } while (ret<0 && errno==EINTR);
    if (ret!=(ssize_t)sizeof(value)) return ret<0 ? -errno:-EIO;
    b->wake_on=ready; return 0;
}
static int native_ready(void *context,bool ready)
{
    struct native_monitor *b=context; b->queued=ready; return wake_update(b);
}
static int add_watch(struct native_monitor *b)
{
    b->watch=inotify_add_watch(b->inotify_fd,"/dev",IN_CREATE|IN_DELETE|IN_MOVED_FROM|IN_MOVED_TO|IN_ATTRIB|IN_DELETE_SELF|IN_MOVE_SELF);
    return b->watch<0 ? -errno:0;
}
static int native_retry(void *context,uint32_t ms)
{
    struct native_monitor *b=context;
    if (b->watch<0 && add_watch(b)<0 && !ms) ms=1000;
    struct itimerspec timer={.it_value={ms/1000,(long)(ms%1000)*1000000}};
    return timerfd_settime(b->timer_fd,0,&timer,NULL)<0 ? -errno:0;
}
static int native_probe(void *context,const char *path,struct cc_node_token *out)
{
    (void)context;
    struct stat st;
    if (stat(path,&st)<0) return -errno;
    if (!S_ISCHR(st.st_mode)) return -ENODEV;
    *out=(struct cc_node_token){((uint64_t)major(st.st_rdev)<<32)|minor(st.st_rdev),(uint64_t)st.st_ino};
    return 0;
}
static int native_wait(void *context,int timeout)
{
    struct native_monitor *b=context;
    struct pollfd fd={b->poll_fd,POLLIN,0};
    int ret=poll(&fd,1,timeout);
    if (ret<0) return -errno;
    if (!ret) return -ETIMEDOUT;
    if (fd.revents&(POLLERR|POLLHUP|POLLNVAL)) return -EIO;
    return 0;
}
static int native_now(void *context,uint64_t *out)
{
    (void)context;
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC,&t)<0) return -errno;
    *out=(uint64_t)t.tv_sec*1000000000U+(uint64_t)t.tv_nsec; return 0;
}
static int next_inotify(struct native_monitor *b,struct cc_monitor_notice *out)
{
    if (b->offset==b->used) {
        ssize_t n;
        do { n=read(b->inotify_fd,b->buffer,sizeof(b->buffer)); } while (n<0 && errno==EINTR);
        if (n<0) return errno==EAGAIN ? 0:-errno;
        if (!n) return -EIO;
        b->used=(size_t)n; b->offset=0;
    }
    int ret=cc_monitor_parse_inotify(b->buffer,b->used,b->watch,&b->offset,out);
    if (ret<0) { b->offset=b->used; *out=(struct cc_monitor_notice){.type=CC_NOTICE_LOST}; ret=1; }
    if (ret==1 && out->type==CC_NOTICE_LOST && out->path[0]) {
        if (b->watch>=0) (void)inotify_rm_watch(b->inotify_fd,b->watch);
        b->watch=-1;
    }
    int wake=wake_update(b);
    return wake<0 ? wake:ret;
}
static int next_netlink(struct native_monitor *b,struct cc_monitor_notice *out)
{
    for (unsigned skip=0; skip<64; ++skip) {
        unsigned char data[NOTICE_BYTES];
        struct sockaddr_nl peer={0};
        struct iovec vector={data,sizeof(data)};
        struct msghdr msg={.msg_name=&peer,.msg_namelen=sizeof(peer),.msg_iov=&vector,.msg_iovlen=1};
        ssize_t n;
        do { n=recvmsg(b->netlink_fd,&msg,MSG_DONTWAIT); } while (n<0 && errno==EINTR);
        if (n<0) {
            if (errno==EAGAIN) return 0;
            if (errno!=ENOBUFS) return -errno;
            *out=(struct cc_monitor_notice){.type=CC_NOTICE_LOST}; return 1;
        }
        /* Only the kernel owns port ID zero. Ignore all userspace senders. */
        if (msg.msg_namelen!=sizeof(peer) || peer.nl_family!=AF_NETLINK || peer.nl_pid) continue;
        int ret=msg.msg_flags&(MSG_TRUNC|MSG_CTRUNC) ? -EPROTO:cc_monitor_parse_uevent(data,(size_t)n,out);
        if (ret<0) { *out=(struct cc_monitor_notice){.type=CC_NOTICE_LOST}; return 1; }
        /* Raw kernel uevents can precede devtmpfs removal. Inotify supplies
         * node deletion edges; raw removals trigger a scan without revoking a
         * newer node already observed through the other notification source. */
        if (ret==1 && out->type==CC_NOTICE_REMOVE) {
            out->type=CC_NOTICE_CHANGE; out->path[0]='\0';
        }
        if (ret) return ret;
    }
    return 0;
}
static int native_next(void *context,struct cc_monitor_notice *out)
{
    struct native_monitor *b=context;
    for (unsigned i=0; i<3; ++i) {
        unsigned source=b->turn++%3;
        int ret;
        if (source==0) ret=next_inotify(b,out);
        else if (source==1) ret=next_netlink(b,out);
        else {
            uint64_t expirations;
            ssize_t n;
            do { n=read(b->timer_fd,&expirations,sizeof(expirations)); } while (n<0 && errno==EINTR);
            if (n<0 && errno==EAGAIN) ret=0;
            else if (n!=(ssize_t)sizeof(expirations)) ret=n<0 ? -errno:-EPROTO;
            else { *out=(struct cc_monitor_notice){.type=CC_NOTICE_TIMER}; ret=1; }
        }
        if (ret) return ret;
    }
    return 0;
}
static void native_close(void *context)
{
    struct native_monitor *b=context;
    const int fds[]={b->poll_fd,b->wake_fd,b->timer_fd,b->inotify_fd,b->netlink_fd};
    for (size_t i=0; i<5; ++i) if (fds[i]>=0) (void)close(fds[i]);
    free(b);
}
static const struct cc_monitor_backend_ops native_ops={
    .next=native_next,.ready=native_ready,.retry=native_retry,.probe=native_probe,
    .wait=native_wait,.now=native_now,.close=native_close
};

int cc_monitor_native_open(struct cc_monitor_backend *out)
{
    if (!out) return -EINVAL;
    struct native_monitor *b=calloc(1,sizeof(*b));
    if (!b) return -ENOMEM;
    b->poll_fd=b->netlink_fd=b->inotify_fd=b->timer_fd=b->wake_fd=b->watch=-1;
    b->netlink_fd=socket(AF_NETLINK,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,NETLINK_KOBJECT_UEVENT);
    int ret=-errno;
    if (b->netlink_fd<0) goto fail;
    struct sockaddr_nl address={.nl_family=AF_NETLINK,.nl_groups=1};
    if (bind(b->netlink_fd,(struct sockaddr *)&address,sizeof(address))<0) { ret=-errno; goto fail; }
    int capacity=1024*1024;
    if (setsockopt(b->netlink_fd,SOL_SOCKET,SO_RCVBUF,&capacity,sizeof(capacity))<0) { ret=-errno; goto fail; }
    b->inotify_fd=inotify_init1(IN_NONBLOCK|IN_CLOEXEC);
    if (b->inotify_fd<0) { ret=-errno; goto fail; }
    ret=add_watch(b); if (ret<0) goto fail;
    b->timer_fd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC);
    if (b->timer_fd<0) { ret=-errno; goto fail; }
    b->wake_fd=eventfd(0,EFD_NONBLOCK|EFD_CLOEXEC);
    if (b->wake_fd<0) { ret=-errno; goto fail; }
    b->poll_fd=epoll_create1(EPOLL_CLOEXEC);
    if (b->poll_fd<0) { ret=-errno; goto fail; }
    const int fds[]={b->netlink_fd,b->inotify_fd,b->timer_fd,b->wake_fd};
    for (size_t i=0; i<4; ++i) {
        struct epoll_event event={.events=EPOLLIN,.data.fd=fds[i]};
        if (epoll_ctl(b->poll_fd,EPOLL_CTL_ADD,fds[i],&event)<0) { ret=-errno; goto fail; }
    }
    *out=(struct cc_monitor_backend){&native_ops,b,b->poll_fd}; return 0;
fail:
    native_close(b); return ret;
}
