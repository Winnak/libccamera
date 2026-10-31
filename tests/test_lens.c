/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
struct fake { int fault, handles, queries, opens, position; };
static int open_lens(void *ctx, const char *path, int flags)
{
    struct fake *f=ctx; (void)flags;
    CHECK(!strcmp(path,"/dev/v4l-subdev-lens")); ++f->opens;
    if (f->fault==7) return -EACCES;
    ++f->handles; return 53;
}
static int close_lens(void *ctx, int fd)
{
    CHECK(fd==10 || fd==53); --((struct fake *)ctx)->handles; return 0;
}
static int lock_lens(void *ctx, int fd) { CHECK(fd==53); return ((struct fake *)ctx)->fault==8 ? -EWOULDBLOCK:0; }
static int ioctl_lens(void *ctx, int fd, unsigned long req, void *arg)
{
    struct fake *f=ctx;
    if (fd==10) {
        CHECK(req==MEDIA_IOC_G_TOPOLOGY); ++f->queries;
        if (f->fault==1) return -ENOTTY;
        struct media_v2_topology *t=arg;
        uint32_t count=f->fault==2 ? 4097U:f->fault==4 ? 2U:f->fault==3 ? 0U:1U;
        if (t->ptr_links) {
            CHECK(t->num_links==count);
            struct media_v2_link *l=(void *)(uintptr_t)t->ptr_links;
            for (uint32_t i=0;i<count;++i) l[i]=(struct media_v2_link){.id=10+i,.source_id=1,
                .sink_id=f->fault==5 ? 99U:2U+i,.flags=MEDIA_LNK_FL_ANCILLARY_LINK};
        }
        t->num_links=count; t->topology_version=f->fault==6 ? (unsigned int)f->queries:1U;
        return 0;
    }
    CHECK(fd==53);
    if (req==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=arg; CHECK(q->id==V4L2_CID_FOCUS_ABSOLUTE);
        q->type=V4L2_CTRL_TYPE_INTEGER; q->minimum=0; q->maximum=1023; q->step=2; q->elems=1; q->elem_size=4;
        return 0;
    }
    struct v4l2_ext_controls *c=arg; CHECK(c->count==1 && c->controls[0].id==V4L2_CID_FOCUS_ABSOLUTE);
    if (req==VIDIOC_G_EXT_CTRLS) c->controls[0].value=f->position;
    else { CHECK(req==VIDIOC_S_EXT_CTRLS); c->controls[0].value-=2; f->position=c->controls[0].value; }
    return 0;
}
static const struct cc_io_ops ops={.open=open_lens,.close=close_lens,.ioctl=ioctl_lens,.lock=lock_lens};
static cc_camera *camera(struct fake *f)
{
    cc_camera *c=calloc(1,sizeof(*c)); CHECK(c);
    c->io=(struct cc_io){&ops,f}; c->fd=c->sensor_fd=c->lens_fd=-1; c->info.sensor_entity=1;
    c->media=calloc(1,sizeof(*c->media)); CHECK(c->media);
    c->media->io=c->io; c->media->fd=10; f->handles=1;
    c->media->num_entities=3; c->media->entities=calloc(3,sizeof(*c->media->entities)); CHECK(c->media->entities);
    for (size_t i=0;i<3;++i) {
        struct cc_entity *e=&c->media->entities[i]; e->desc.id=(uint32_t)i+1;
        e->desc.type=i ? MEDIA_ENT_F_LENS:MEDIA_ENT_F_CAM_SENSOR;
        strcpy(e->path,i==1 ? "/dev/v4l-subdev-lens":"/dev/unrelated");
    }
    return c;
}
int main(void)
{
    const int errors[]={0,ENOTSUP,EOVERFLOW,ENOENT,ENOTUNIQ,ESTALE,EAGAIN,EACCES,EWOULDBLOCK};
    for (int fault=0;fault<9;++fault) {
        struct fake f={.fault=fault}; cc_camera *c=camera(&f); cc_control_info info;
        CHECK(cc_camera_query_lens(c,&info)==-errors[fault]);
        if (!fault) {
            CHECK(info.id==V4L2_CID_FOCUS_ABSOLUTE && info.step==2);
            int32_t position=700;
            CHECK(!cc_camera_set_lens_position(c,&position) && position==698);
            position=0; CHECK(!cc_camera_get_lens_position(c,&position) && position==698);
            CHECK(f.opens==1 && f.queries==2); /* Cached handle belongs to this camera. */
        } else if (fault==6) CHECK(f.queries==6);
        cc_camera_close(c); CHECK(!f.handles);
    }
    puts("Associated lens discovery, topology snapshots, ambiguity, native acknowledgements and ownership passed"); return 0;
}
