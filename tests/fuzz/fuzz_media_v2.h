/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_FUZZ_MEDIA_V2_H
#define CCAMERA_FUZZ_MEDIA_V2_H
#include "media_topology.h"
#include <errno.h>
struct v2_driver {
    struct media_v2_entity entities[16]; struct media_v2_interface interfaces[16];
    struct media_v2_pad pads[32]; struct media_v2_link links[64];
    uint32_t ne,ni,np,nl;
    uint8_t options,race;
    unsigned opens,closes,queries;
    uint64_t version;
};
static int v2_open(void *context,const char *path,int flags)
{
    struct v2_driver *d=context; (void)path; (void)flags; ++d->opens; return 42;
}
static int v2_close(void *context,int fd) { struct v2_driver *d=context; FUZZ_CHECK(fd==42); ++d->closes; return 0; }
static int v2_resolve(void *context,unsigned major_id,unsigned minor_id,char *path,size_t size)
{
    (void)context;
    if (major_id!=81 || minor_id>=16) return -ENOENT;
    int n=snprintf(path,size,"/v2-node%u",minor_id); FUZZ_CHECK(n>0 && (size_t)n<size); return 0;
}
static int v2_ioctl(void *context,int fd,unsigned long request,void *argument)
{
    struct v2_driver *d=context; FUZZ_CHECK(fd==42);
    if (request==MEDIA_IOC_DEVICE_INFO) {
        struct media_device_info *info=argument; info->media_version=UINT32_C(0x060c00); return 0;
    }
    if (request==MEDIA_IOC_G_TOPOLOGY) {
        ++d->queries; if (d->options&32U) return -EIO;
        struct media_v2_topology *t=argument;
        bool filling=t->ptr_entities || t->ptr_interfaces || t->ptr_pads || t->ptr_links;
        if (filling) {
            FUZZ_CHECK(t->num_entities>=d->ne && t->num_interfaces>=d->ni && t->num_pads>=d->np && t->num_links>=d->nl);
            if (d->race==3 && d->queries==2) { ++d->version; return -ENOSPC; }
            memcpy((void *)(uintptr_t)t->ptr_entities,d->entities,d->ne*sizeof(*d->entities));
            memcpy((void *)(uintptr_t)t->ptr_interfaces,d->interfaces,d->ni*sizeof(*d->interfaces));
            memcpy((void *)(uintptr_t)t->ptr_pads,d->pads,d->np*sizeof(*d->pads));
            memcpy((void *)(uintptr_t)t->ptr_links,d->links,d->nl*sizeof(*d->links));
            if (d->race==2 || (d->race==1 && d->queries==2)) ++d->version;
            if (d->race==4) ++t->ptr_entities;
        }
        t->num_entities=d->ne; t->num_interfaces=d->ni; t->num_pads=d->np; t->num_links=d->nl;
        if (filling && d->race==5) ++t->num_pads;
        t->topology_version=d->version; return 0;
    }
    FUZZ_CHECK(request==MEDIA_IOC_SETUP_LINK);
    if (d->options&16U) return -EIO;
    if (d->options&64U) ++((struct media_link_desc *)argument)->sink.index;
    return 0;
}
static void fuzz_media_v2(struct fuzz_input *in)
{
    struct v2_driver d={0};
    d.options=fuzz_u8(in); d.ne=1U+fuzz_u8(in)%16U; d.ni=d.ne; d.np=d.ne*2;
    uint32_t edges=fuzz_u8(in)%33U;
    d.race=(d.options&2U) ? (uint8_t)(fuzz_u8(in)%6U):0;
    for (uint32_t i=0;i<d.ne;++i) {
        d.entities[i]=(struct media_v2_entity){.id=i+1,.function=i==0 ? MEDIA_ENT_F_CAM_SENSOR:
            (d.options&4U) && i==d.ne-1 ? MEDIA_ENT_F_LENS:MEDIA_ENT_F_IO_V4L};
        int n=snprintf(d.entities[i].name,sizeof(d.entities[i].name),"long-v2-entity-name-beyond-thirty-two-bytes-%u",i);
        FUZZ_CHECK(n>0 && (size_t)n<sizeof(d.entities[i].name));
        d.interfaces[i]=(struct media_v2_interface){.id=100+i,.intf_type=i ? MEDIA_INTF_T_V4L_VIDEO:MEDIA_INTF_T_V4L_SUBDEV,
            .devnode={81,i}};
        d.pads[i*2]=(struct media_v2_pad){.id=200+i*2,.entity_id=i+1,.index=0,.flags=MEDIA_PAD_FL_SINK};
        d.pads[i*2+1]=(struct media_v2_pad){.id=201+i*2,.entity_id=i+1,.index=1,.flags=MEDIA_PAD_FL_SOURCE};
        d.links[d.nl]=(struct media_v2_link){.id=300+d.nl,.source_id=100+i,.sink_id=i+1,
            .flags=MEDIA_LNK_FL_INTERFACE_LINK|MEDIA_LNK_FL_ENABLED}; ++d.nl;
    }
    bool used[16][16]={{false}};
    for (uint32_t i=0;i<edges;++i) {
        uint32_t a=fuzz_u8(in)%d.ne,b=fuzz_u8(in)%d.ne;
        if (used[a][b]) continue;
        used[a][b]=true;
        d.links[d.nl]=(struct media_v2_link){.id=300+d.nl,.source_id=201+a*2,.sink_id=200+b*2,
            .flags=fuzz_u8(in)&(MEDIA_LNK_FL_ENABLED|MEDIA_LNK_FL_IMMUTABLE)}; ++d.nl;
    }
    if ((d.options&4U) && d.ne>1) {
        d.links[d.nl]=(struct media_v2_link){.id=300+d.nl,.source_id=1,.sink_id=d.ne,.flags=MEDIA_LNK_FL_ANCILLARY_LINK}; ++d.nl;
    }
    if (d.options&1U) {
        switch (fuzz_u8(in)%4U) {
        case 0: fuzz_patch(in,d.entities,d.ne*sizeof(*d.entities)); break;
        case 1: fuzz_patch(in,d.interfaces,d.ni*sizeof(*d.interfaces)); break;
        case 2: fuzz_patch(in,d.pads,d.np*sizeof(*d.pads)); break;
        case 3: fuzz_patch(in,d.links,d.nl*sizeof(*d.links)); break;
        }
    }
    const struct cc_io_ops ops={.open=v2_open,.close=v2_close,.ioctl=v2_ioctl,.resolve=v2_resolve};
    const struct cc_io io={&ops,&d}; struct cc_media *media=NULL;
    int ret=cc_media_open(&io,"/v2",false,&media); FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(!media);
    else {
        FUZZ_CHECK(media && media->topology && media->num_entities==d.ne && media->num_links<=d.nl);
        unsigned distance[16][16];
        for (uint32_t a=0;a<d.ne;++a) for (uint32_t b=0;b<d.ne;++b) distance[a][b]=a==b ? 0U:17U;
        size_t normalized=0;
        for (uint32_t i=0;i<d.nl;++i) {
            if ((d.links[i].flags&UINT32_C(0xf0000000))!=MEDIA_LNK_FL_DATA_LINK) continue;
            const struct media_v2_pad *a=NULL,*b=NULL;
            for (uint32_t j=0;j<d.np;++j) {
                if (d.pads[j].id==d.links[i].source_id) a=&d.pads[j];
                if (d.pads[j].id==d.links[i].sink_id) b=&d.pads[j];
            }
            FUZZ_CHECK(a && b); uint32_t ai=d.ne,bi=d.ne;
            for (uint32_t j=0;j<d.ne;++j) {
                if (d.entities[j].id==a->entity_id) ai=j;
                if (d.entities[j].id==b->entity_id) bi=j;
            }
            FUZZ_CHECK(ai<d.ne && bi<d.ne && normalized<media->num_links);
            FUZZ_CHECK(media->links[normalized].source.entity==a->entity_id && media->links[normalized].source.index==a->index &&
                media->links[normalized].sink.entity==b->entity_id && media->links[normalized].sink.index==b->index);
            ++normalized; if (ai!=bi) distance[ai][bi]=1;
        }
        FUZZ_CHECK(normalized==media->num_links);
        for (uint32_t k=0;k<d.ne;++k) for (uint32_t a=0;a<d.ne;++a) for (uint32_t b=0;b<d.ne;++b)
            if (distance[a][b]>distance[a][k]+distance[k][b]) distance[a][b]=distance[a][k]+distance[k][b];
        for (uint32_t a=0;a<d.ne;++a) for (uint32_t b=0;b<d.ne;++b) {
            size_t *route=NULL,count=0;
            ret=cc_media_route(media,d.entities[a].id,d.entities[b].id,&route,&count);
            if (a==b || distance[a][b]>16) FUZZ_CHECK(ret==-ENOENT && !route && !count);
            else FUZZ_CHECK(!ret && count==distance[a][b]);
            free(route);
        }
        for (size_t i=0;i<media->num_links;++i) {
            struct media_link_desc before=media->links[i];
            ret=cc_media_set_link(media,i,before.flags^MEDIA_LNK_FL_ENABLED);
            if (ret) FUZZ_CHECK(!memcmp(&before,&media->links[i],sizeof(before)));
        }
        for (size_t i=0;i<media->num_entities;++i)
            FUZZ_CHECK(!strcmp(cc_entity_name(&media->entities[i]),d.entities[i].name));
        ++d.version; struct cc_entity *lens=(struct cc_entity *)(uintptr_t)1;
        FUZZ_CHECK(cc_media_lens(media,1,&lens)==-ESTALE && !lens);
        cc_media_close(media);
    }
    FUZZ_CHECK(d.opens==1 && d.closes==1);
}
#endif
