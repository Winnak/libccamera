/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/monitor.h>
#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static volatile sig_atomic_t stopped;
static void stop(int signal_number) { (void)signal_number; stopped=1; }
static void print_camera(const char *action,uint64_t generation,const cc_camera_info *camera)
{
    printf("%s generation=%" PRIu64 " id=%s video=%s media=%s name=%s\n",action,generation,
        camera->id[0] ? camera->id:"(unavailable)",camera->video_path,camera->media_path,camera->name);
}
int main(int argc,char **argv)
{
    if (argc!=1) {
        bool help=argc==2 && (!strcmp(argv[1],"--help") || !strcmp(argv[1],"-h"));
        fprintf(help ? stdout:stderr,"Usage: %s\nWatch camera arrivals, changes and removals; Ctrl-C stops.\n",argv[0]);
        return help ? 0:1;
    }
    struct sigaction action={.sa_handler=stop};
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT,&action,NULL)<0 || sigaction(SIGTERM,&action,NULL)<0) { perror("sigaction"); return 1; }
    cc_camera_monitor *monitor=NULL;
    int ret=cc_camera_monitor_open(&monitor);
    if (ret) { fprintf(stderr,"monitor: %s\n",strerror(-ret)); return 1; }
    struct pollfd fd={.fd=cc_camera_monitor_fd(monitor),.events=POLLIN};
    while (!stopped) {
        cc_camera_event event;
        ret=cc_camera_monitor_dequeue(monitor,&event,0);
        if (!ret) {
            if (event.type==CC_CAMERA_RESYNC) {
                printf("resync generation=%" PRIu64 " flags=0x%x\n",event.generation,event.flags);
                cc_camera_info *cameras=NULL; size_t count=0; uint64_t generation=0;
                ret=cc_camera_monitor_snapshot(monitor,&cameras,&count,&generation);
                if (ret) break;
                for (size_t i=0; i<count; ++i) print_camera("present",generation,&cameras[i]);
                cc_camera_list_free(cameras);
            } else print_camera(event.type==CC_CAMERA_ADDED ? "added":event.type==CC_CAMERA_REMOVED ? "removed":"changed",
                                event.generation,&event.camera);
            fflush(stdout); continue;
        }
        if (ret!=-EAGAIN) break;
        int status=poll(&fd,1,-1);
        if (status<0) { if (errno==EINTR) continue; ret=-errno; break; }
        if (fd.revents&(POLLERR|POLLHUP|POLLNVAL)) { ret=-EIO; break; }
    }
    cc_camera_monitor_close(monitor);
    if (stopped) return 0;
    fprintf(stderr,"monitor: %s\n",strerror(-ret)); return 1;
}
