/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "media_topology.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
struct fake {
    struct media_v2_entity entities[6]; struct media_v2_interface interfaces[6];
    struct media_v2_pad pads[6]; struct media_v2_link links[12];
    uint32_t num_entities,num_interfaces,num_pads,num_links,media_version;
    uint64_t version;
    unsigned opened,closed,queries,legacy_queries,setups,resolves;
    unsigned race; int unsupported;
    bool bad_pointer,bad_ack,fail_resolve,legacy_seen;
};
static void initialize(struct fake *f)
{
    memset(f,0,sizeof(*f)); f->num_entities=5; f->num_interfaces=5; f->num_pads=4; f->num_links=10;
    f->media_version=UINT32_C(0x060c00); f->version=0;
    const uint32_t ids[]={30,10,20,40,50},functions[]={MEDIA_ENT_F_IO_V4L,MEDIA_ENT_F_CAM_SENSOR,
        MEDIA_ENT_F_VID_IF_BRIDGE,MEDIA_ENT_F_LENS,MEDIA_ENT_F_FLASH};
    for (uint32_t i=0;i<5;++i) { f->entities[i].id=ids[i]; f->entities[i].function=functions[i]; }
    strcpy(f->entities[0].name,"long-shared-prefix-for-media-v2-entities-that-differs-capture");
    strcpy(f->entities[1].name,"imx708 10-001a"); strcpy(f->entities[2].name,"csi2");
    strcpy(f->entities[3].name,"long-shared-prefix-for-media-v2-entities-that-differs-lens");
    strcpy(f->entities[4].name,"flash"); f->entities[1].flags=MEDIA_ENT_FL_DEFAULT;
    for (uint32_t i=0;i<5;++i) {
        f->interfaces[i]=(struct media_v2_interface){.id=100+i,
            .intf_type=i==2 ? MEDIA_INTF_T_V4L_VIDEO:i==4 ? MEDIA_INTF_T_DVB_FE:MEDIA_INTF_T_V4L_SUBDEV,
            .devnode={i==4 ? 212U:81U,i}};
    }
    f->pads[0]=(struct media_v2_pad){.id=200,.entity_id=20,.index=1,.flags=MEDIA_PAD_FL_SOURCE};
    f->pads[1]=(struct media_v2_pad){.id=201,.entity_id=10,.index=0,.flags=MEDIA_PAD_FL_SOURCE};
    f->pads[2]=(struct media_v2_pad){.id=202,.entity_id=30,.index=0,.flags=MEDIA_PAD_FL_SINK};
    f->pads[3]=(struct media_v2_pad){.id=203,.entity_id=20,.index=0,.flags=MEDIA_PAD_FL_SINK};
    f->links[0]=(struct media_v2_link){.id=300,.source_id=201,.sink_id=203};
    f->links[1]=(struct media_v2_link){.id=301,.source_id=200,.sink_id=202,.flags=MEDIA_LNK_FL_ENABLED};
    const uint32_t associated[]={10,20,30,40,50};
    for (uint32_t i=0;i<5;++i) f->links[2+i]=(struct media_v2_link){.id=302+i,.source_id=100+i,
        .sink_id=associated[i],.flags=MEDIA_LNK_FL_INTERFACE_LINK|MEDIA_LNK_FL_ENABLED|MEDIA_LNK_FL_IMMUTABLE};
    f->links[7]=(struct media_v2_link){.id=307,.source_id=10,.sink_id=40,.flags=MEDIA_LNK_FL_ANCILLARY_LINK};
    f->links[8]=(struct media_v2_link){.id=308,.source_id=10,.sink_id=50,.flags=MEDIA_LNK_FL_ANCILLARY_LINK};
    f->links[9]=(struct media_v2_link){.id=309,.source_id=10,.sink_id=20,.flags=UINT32_C(0x30000000)};
}
static int driver_open(void *context,const char *path,int flags)
{
    struct fake *f=context; (void)flags; CHECK(!strcmp(path,"/media")); ++f->opened; return 10;
}
static int driver_close(void *context,int fd) { struct fake *f=context; CHECK(fd==10); ++f->closed; return 0; }
static int driver_lock(void *context,int fd) { (void)context; CHECK(fd==10); return 0; }
static int driver_resolve(void *context,unsigned major_id,unsigned minor_id,char *out,size_t size)
{
    struct fake *f=context; CHECK(major_id==81 && minor_id<4); ++f->resolves;
    if (f->fail_resolve) return -ENOENT;
    int n=snprintf(out,size,"/dev/node%u",minor_id); CHECK(n>0 && (size_t)n<size); return 0;
}
static void legacy_entity(const struct fake *f,uint32_t index,struct media_entity_desc *out)
{
    const struct media_v2_entity *e=&f->entities[index];
    *out=(struct media_entity_desc){.id=e->id,.type=e->function,.flags=e->flags};
    cc_copy_string(out->name,sizeof(out->name),e->name,sizeof(e->name));
    for (uint32_t p=0;p<f->num_pads;++p) if (f->pads[p].entity_id==e->id) ++out->pads;
    for (uint32_t l=0;l<2;++l) {
        for (uint32_t p=0;p<f->num_pads;++p)
            if (f->pads[p].entity_id==e->id && (f->pads[p].id==f->links[l].source_id || f->pads[p].id==f->links[l].sink_id)) ++out->links;
    }
    if (e->id!=50) { out->dev.major=81; out->dev.minor=e->id==10 ? 0:e->id==20 ? 1:e->id==30 ? 2:3; }
}
static int driver_ioctl(void *context,int fd,unsigned long request,void *argument)
{
    struct fake *f=context; CHECK(fd==10);
    if (request==MEDIA_IOC_DEVICE_INFO) { struct media_device_info *info=argument; info->media_version=f->media_version; return 0; }
    if (request==MEDIA_IOC_G_TOPOLOGY) {
        ++f->queries; if (f->unsupported) return -f->unsupported;
        struct media_v2_topology *t=argument;
        bool filling=t->ptr_entities || t->ptr_interfaces || t->ptr_pads || t->ptr_links;
        if (filling) {
            CHECK(t->num_entities>=f->num_entities && t->num_interfaces>=f->num_interfaces &&
                t->num_pads>=f->num_pads && t->num_links>=f->num_links);
            if (f->race==3 && f->queries==2) { ++f->version; return -ENOSPC; }
            if (t->ptr_entities) memcpy((void *)(uintptr_t)t->ptr_entities,f->entities,f->num_entities*sizeof(*f->entities));
            if (t->ptr_interfaces) memcpy((void *)(uintptr_t)t->ptr_interfaces,f->interfaces,f->num_interfaces*sizeof(*f->interfaces));
            if (t->ptr_pads) memcpy((void *)(uintptr_t)t->ptr_pads,f->pads,f->num_pads*sizeof(*f->pads));
            if (t->ptr_links) memcpy((void *)(uintptr_t)t->ptr_links,f->links,f->num_links*sizeof(*f->links));
            if (f->race==2 || (f->race==1 && f->queries==2)) ++f->version;
            if (f->bad_pointer) ++t->ptr_pads;
        }
        t->num_entities=f->num_entities; t->num_interfaces=f->num_interfaces; t->num_pads=f->num_pads; t->num_links=f->num_links;
        if (filling && f->race==4) ++t->num_links;
        t->topology_version=f->version; return 0;
    }
    if (request==MEDIA_IOC_SETUP_LINK) {
        ++f->setups; struct media_link_desc *link=argument;
        CHECK(link->source.entity==10 && link->source.index==0 && link->sink.entity==20 && link->sink.index==0);
        if (f->bad_ack) ++link->sink.entity;
        return 0;
    }
    f->legacy_seen=true; ++f->legacy_queries;
    if (request==MEDIA_IOC_ENUM_ENTITIES) {
        struct media_entity_desc *entity=argument; uint32_t previous=entity->id&~MEDIA_ENT_ID_FLAG_NEXT;
        uint32_t best=UINT32_MAX,at=0;
        for (uint32_t i=0;i<f->num_entities;++i) if (f->entities[i].id>previous && f->entities[i].id<best) { best=f->entities[i].id; at=i; }
        if (best==UINT32_MAX) return -EINVAL;
        legacy_entity(f,at,entity); return 0;
    }
    CHECK(request==MEDIA_IOC_ENUM_LINKS); struct media_links_enum *e=argument;
    size_t n=0;
    for (uint32_t i=0;i<f->num_pads;++i) if (f->pads[i].entity_id==e->entity)
        e->pads[f->pads[i].index]=(struct media_pad_desc){.entity=e->entity,.index=(uint16_t)f->pads[i].index,.flags=f->pads[i].flags};
    for (uint32_t l=0;l<2;++l) {
        const struct media_v2_pad *a=NULL,*b=NULL;
        for (uint32_t p=0;p<f->num_pads;++p) {
            if (f->pads[p].id==f->links[l].source_id) a=&f->pads[p];
            if (f->pads[p].id==f->links[l].sink_id) b=&f->pads[p];
        }
        CHECK(a && b);
        if (a->entity_id!=e->entity && b->entity_id!=e->entity) continue;
        e->links[n++]=(struct media_link_desc){.source={.entity=a->entity_id,.index=(uint16_t)a->index,.flags=a->flags},
            .sink={.entity=b->entity_id,.index=(uint16_t)b->index,.flags=b->flags},.flags=f->links[l].flags};
    }
    return 0;
}
static const struct cc_io_ops ops={.open=driver_open,.close=driver_close,.lock=driver_lock,.ioctl=driver_ioctl,.resolve=driver_resolve};
static int open_graph(struct fake *f,struct cc_media **media)
{
    struct cc_io io={&ops,f}; return cc_media_open(&io,"/media",true,media);
}
static void test_graph(void)
{
    struct fake f; initialize(&f); struct cc_media *media;
    OK(open_graph(&f,&media)); CHECK(media && media->topology && !f.legacy_seen && f.queries==2);
    CHECK(media->num_entities==5 && media->num_links==2 && media->topology->info.num_links==10);
    struct cc_entity *sensor=cc_media_entity(media,10),*receiver=cc_media_entity(media,20),*video=cc_media_entity(media,30),*lens=NULL;
    CHECK(sensor && receiver && video && sensor->desc.flags==MEDIA_ENT_FL_DEFAULT);
    CHECK(receiver->desc.pads==2 && receiver->pads[1].flags==MEDIA_PAD_FL_SOURCE && receiver->pads[0].flags==MEDIA_PAD_FL_SINK);
    CHECK(!strcmp(cc_entity_name(video),f.entities[0].name) && strlen(cc_entity_name(video))>31);
    CHECK(!strcmp(video->path,"/dev/node2") && !cc_media_entity(media,50)->path[0] && f.resolves==4);
    size_t *route=NULL,count=0; OK(cc_media_route_pad(media,10,0,30,&route,&count)); CHECK(count==2); free(route);
    OK(cc_media_lens(media,10,&lens)); CHECK(lens->desc.id==40);
    OK(cc_media_set_link(media,0,MEDIA_LNK_FL_ENABLED)); CHECK(f.setups==1 && (media->topology->links[0].flags&MEDIA_LNK_FL_ENABLED));
    CHECK(cc_media_set_link(media,0,MEDIA_LNK_FL_INTERFACE_LINK)==-EINVAL);
    ++f.version; lens=(struct cc_entity *)(uintptr_t)1;
    CHECK(cc_media_lens(media,10,&lens)==-ESTALE && !lens);
    CHECK(cc_media_set_link(media,0,0)==-ESTALE && f.setups==1);
    cc_media_close(media); CHECK(f.opened==f.closed);
    initialize(&f); f.fail_resolve=true; OK(open_graph(&f,&media)); CHECK(!cc_media_entity(media,10)->path[0]); cc_media_close(media);
    initialize(&f); f.bad_ack=true; OK(open_graph(&f,&media));
    CHECK(cc_media_set_link(media,0,MEDIA_LNK_FL_ENABLED)==-EPROTO && !media->links[0].flags); cc_media_close(media);
}
static void expect_error(struct fake *f,int expected)
{
    struct cc_media *media=(struct cc_media *)(uintptr_t)1;
    CHECK(open_graph(f,&media)==-expected && !media && f->opened==f->closed && !f->legacy_seen);
}
static void test_rejection(void)
{
    struct fake f;
    initialize(&f); f.entities[1].id=f.entities[0].id; expect_error(&f,EPROTO);
    initialize(&f); f.pads[0].id=f.interfaces[0].id; expect_error(&f,EPROTO);
    initialize(&f); f.links[0].id=0; expect_error(&f,EPROTO);
    initialize(&f); memset(f.entities[0].name,'x',64); expect_error(&f,EPROTO);
    initialize(&f); strcpy(f.entities[0].name,f.entities[1].name); expect_error(&f,EPROTO);
    initialize(&f); f.pads[0].entity_id=999; expect_error(&f,EPROTO);
    initialize(&f); f.pads[0].index=0; expect_error(&f,EPROTO);
    initialize(&f); f.pads[0].index=2; expect_error(&f,EPROTO);
    initialize(&f); f.pads[0].flags=MEDIA_PAD_FL_SOURCE|MEDIA_PAD_FL_SINK; expect_error(&f,EPROTO);
    initialize(&f); f.links[0].source_id=10; expect_error(&f,EPROTO);
    initialize(&f); f.links[0].sink_id=201; expect_error(&f,EPROTO);
    initialize(&f); f.links[1].source_id=f.links[0].source_id; f.links[1].sink_id=f.links[0].sink_id; expect_error(&f,EPROTO);
    initialize(&f); f.links[2].source_id=10; expect_error(&f,EPROTO);
    initialize(&f); f.links[7].sink_id=203; expect_error(&f,EPROTO);
    initialize(&f); f.links[9].sink_id=999; expect_error(&f,EPROTO);
    initialize(&f); f.links[3].sink_id=10; expect_error(&f,ENOTUNIQ);
    initialize(&f); f.bad_pointer=true; expect_error(&f,EPROTO);
    for (unsigned kind=0;kind<4;++kind) {
        initialize(&f);
        if (kind==0) f.num_entities=4097;
        if (kind==1) f.num_interfaces=4097;
        if (kind==2) f.num_pads=4097;
        if (kind==3) f.num_links=4097;
        expect_error(&f,EOVERFLOW); CHECK(f.queries==1);
    }
    initialize(&f); f.unsupported=EACCES; expect_error(&f,EACCES);
}
static void test_retries_and_fallback(void)
{
    for (unsigned race=1;race<=4;++race) {
        struct fake f; initialize(&f); f.race=race; struct cc_media *media;
        if (race==2 || race==4) { expect_error(&f,EAGAIN); CHECK(f.queries==6); }
        else { OK(open_graph(&f,&media)); CHECK(f.queries==4); cc_media_close(media); }
    }
    for (unsigned fallback=0;fallback<3;++fallback) {
        struct fake f; initialize(&f);
        if (fallback==0) f.media_version=UINT32_C(0x041200);
        else f.unsupported=fallback==1 ? ENOTTY:EINVAL;
        struct cc_media *media; OK(open_graph(&f,&media));
        CHECK(!media->topology && f.legacy_seen && media->num_entities==5 && media->num_links==2);
        CHECK(f.queries==(fallback ? 1U:0U)); cc_media_close(media); CHECK(f.opened==f.closed);
    }
    struct fake f; initialize(&f); f.num_entities=f.num_interfaces=f.num_pads=f.num_links=0;
    struct cc_media *media; OK(open_graph(&f,&media)); CHECK(media->topology && !media->num_entities && !media->num_links); cc_media_close(media);
}
int main(void)
{
    test_graph(); test_rejection(); test_retries_and_fallback();
    puts("Media-v2 complete snapshots, names/interfaces/pads/links, stale guards, retries and legacy fallback passed"); return 0;
}
