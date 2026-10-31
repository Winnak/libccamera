/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "media_topology.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define CC_TOPOLOGY_LIMIT 4096U
#define CC_LINK_TYPE UINT32_C(0xf0000000)
enum object_type { ENTITY, INTERFACE, PAD, LINK };
struct object { uint32_t id,index; enum object_type type; };
struct object_map { struct object *objects; size_t count; };

void cc_media_topology_free(struct cc_media_topology *t)
{
    if (!t) return;
    free(t->entities); free(t->interfaces); free(t->pads); free(t->links); free(t->data_link_ids); free(t);
}
static bool counts_valid(const struct media_v2_topology *t)
{
    return t->num_entities<=CC_TOPOLOGY_LIMIT && t->num_interfaces<=CC_TOPOLOGY_LIMIT &&
        t->num_pads<=CC_TOPOLOGY_LIMIT && t->num_links<=CC_TOPOLOGY_LIMIT;
}
static bool same_counts(const struct media_v2_topology *a,const struct media_v2_topology *b)
{
    return a->num_entities==b->num_entities && a->num_interfaces==b->num_interfaces &&
        a->num_pads==b->num_pads && a->num_links==b->num_links;
}
static int snapshot(struct cc_media *media,struct cc_media_topology **out)
{
    for (unsigned attempt=0;attempt<3;++attempt) {
        struct media_v2_topology count={0};
        int ret=cc_ioctl(&media->io,media->fd,MEDIA_IOC_G_TOPOLOGY,&count);
        if (ret<0) return ret==-ENOTTY || ret==-EINVAL ? -ENOTSUP:ret;
        if (!counts_valid(&count)) return -EOVERFLOW;
        if (count.ptr_entities || count.ptr_interfaces || count.ptr_pads || count.ptr_links) return -EPROTO;
        struct cc_media_topology *t=calloc(1,sizeof(*t));
        if (!t) return -ENOMEM;
        t->entities=calloc(count.num_entities ? count.num_entities:1,sizeof(*t->entities));
        t->interfaces=calloc(count.num_interfaces ? count.num_interfaces:1,sizeof(*t->interfaces));
        t->pads=calloc(count.num_pads ? count.num_pads:1,sizeof(*t->pads));
        t->links=calloc(count.num_links ? count.num_links:1,sizeof(*t->links));
        if (!t->entities || !t->interfaces || !t->pads || !t->links) { cc_media_topology_free(t); return -ENOMEM; }
        t->info=count;
        t->info.ptr_entities=(uint64_t)(uintptr_t)t->entities;
        t->info.ptr_interfaces=(uint64_t)(uintptr_t)t->interfaces;
        t->info.ptr_pads=(uint64_t)(uintptr_t)t->pads;
        t->info.ptr_links=(uint64_t)(uintptr_t)t->links;
        ret=cc_ioctl(&media->io,media->fd,MEDIA_IOC_G_TOPOLOGY,&t->info);
        if (ret==-ENOSPC || (!ret && (t->info.topology_version!=count.topology_version || !same_counts(&t->info,&count)))) {
            cc_media_topology_free(t); continue;
        }
        if (ret<0) { cc_media_topology_free(t); return ret; }
        if (t->info.ptr_entities!=(uint64_t)(uintptr_t)t->entities || t->info.ptr_interfaces!=(uint64_t)(uintptr_t)t->interfaces ||
            t->info.ptr_pads!=(uint64_t)(uintptr_t)t->pads || t->info.ptr_links!=(uint64_t)(uintptr_t)t->links) {
            cc_media_topology_free(t); return -EPROTO;
        }
        *out=t; return 0;
    }
    return -EAGAIN;
}
static int compare_object(const void *a,const void *b)
{
    const struct object *x=a,*y=b;
    return x->id>y->id ? 1:x->id<y->id ? -1:0;
}
static int make_map(const struct cc_media_topology *t,struct object_map *map)
{
    map->count=(size_t)t->info.num_entities+t->info.num_interfaces+t->info.num_pads+t->info.num_links;
    map->objects=calloc(map->count ? map->count:1,sizeof(*map->objects));
    if (!map->objects) return -ENOMEM;
    size_t n=0;
    for (uint32_t i=0;i<t->info.num_entities;++i) map->objects[n++]=(struct object){t->entities[i].id,i,ENTITY};
    for (uint32_t i=0;i<t->info.num_interfaces;++i) map->objects[n++]=(struct object){t->interfaces[i].id,i,INTERFACE};
    for (uint32_t i=0;i<t->info.num_pads;++i) map->objects[n++]=(struct object){t->pads[i].id,i,PAD};
    for (uint32_t i=0;i<t->info.num_links;++i) map->objects[n++]=(struct object){t->links[i].id,i,LINK};
    qsort(map->objects,map->count,sizeof(*map->objects),compare_object);
    for (size_t i=0;i<map->count;++i)
        if (!map->objects[i].id || (i && map->objects[i-1].id==map->objects[i].id)) return -EPROTO;
    return 0;
}
static const struct object *find_object(const struct object_map *map,uint32_t id,enum object_type type)
{
    size_t lo=0,hi=map->count;
    while (lo<hi) {
        size_t mid=lo+(hi-lo)/2;
        if (map->objects[mid].id<id) lo=mid+1;
        else hi=mid;
    }
    return lo<map->count && map->objects[lo].id==id && map->objects[lo].type==type ? &map->objects[lo]:NULL;
}
static int populate_entities(struct cc_media *media,const struct object_map *map)
{
    const struct cc_media_topology *t=media->topology;
    media->entities=calloc(t->info.num_entities ? t->info.num_entities:1,sizeof(*media->entities));
    if (!media->entities) return -ENOMEM;
    media->num_entities=t->info.num_entities;
    for (uint32_t i=0;i<t->info.num_entities;++i) {
        const struct media_v2_entity *native=&t->entities[i];
        if (!native->name[0] || !memchr(native->name,'\0',sizeof(native->name))) return -EPROTO;
        for (uint32_t j=0;j<i;++j) if (!strcmp(native->name,t->entities[j].name)) return -EPROTO;
        struct cc_entity *entity=&media->entities[i];
        entity->desc.id=native->id; entity->desc.type=native->function; entity->desc.flags=native->flags;
        cc_copy_string(entity->name,sizeof(entity->name),native->name,sizeof(native->name));
        cc_copy_string(entity->desc.name,sizeof(entity->desc.name),native->name,sizeof(native->name));
    }
    /* Map globally identified pads into each entity's contiguous index array. */
    for (uint32_t i=0;i<t->info.num_pads;++i) {
        const struct object *owner=find_object(map,t->pads[i].entity_id,ENTITY);
        if (!owner) return -EPROTO;
        struct cc_entity *entity=&media->entities[owner->index];
        if (entity->desc.pads>=CC_TOPOLOGY_LIMIT) return -EOVERFLOW;
        ++entity->desc.pads;
    }
    for (size_t i=0;i<media->num_entities;++i) {
        struct cc_entity *entity=&media->entities[i];
        entity->pads=calloc(entity->desc.pads ? entity->desc.pads:1,sizeof(*entity->pads));
        if (!entity->pads) return -ENOMEM;
    }
    for (uint32_t i=0;i<t->info.num_pads;++i) {
        const struct media_v2_pad *native=&t->pads[i];
        const struct object *owner=find_object(map,native->entity_id,ENTITY);
        if (!owner) return -EPROTO;
        struct cc_entity *entity=&media->entities[owner->index];
        uint32_t direction=native->flags&(MEDIA_PAD_FL_SOURCE|MEDIA_PAD_FL_SINK);
        if (native->index>=entity->desc.pads || entity->pads[native->index].entity ||
            (direction!=MEDIA_PAD_FL_SOURCE && direction!=MEDIA_PAD_FL_SINK)) return -EPROTO;
        entity->pads[native->index]=(struct media_pad_desc){.entity=native->entity_id,.index=(uint16_t)native->index,.flags=native->flags};
    }
    return 0;
}
static int populate_links(struct cc_media *media,const struct object_map *map)
{
    struct cc_media_topology *t=media->topology;
    media->links=calloc(t->info.num_links ? t->info.num_links:1,sizeof(*media->links));
    t->data_link_ids=calloc(t->info.num_links ? t->info.num_links:1,sizeof(*t->data_link_ids));
    if (!media->links || !t->data_link_ids) return -ENOMEM;
    for (uint32_t i=0;i<t->info.num_links;++i) {
        const struct media_v2_link *native=&t->links[i];
        uint32_t type=native->flags&CC_LINK_TYPE;
        if (type==MEDIA_LNK_FL_DATA_LINK) {
            const struct object *source=find_object(map,native->source_id,PAD),*sink=find_object(map,native->sink_id,PAD);
            if (!source || !sink) return -EPROTO;
            const struct media_v2_pad *a=&t->pads[source->index],*b=&t->pads[sink->index];
            if (!(a->flags&MEDIA_PAD_FL_SOURCE) || !(b->flags&MEDIA_PAD_FL_SINK)) return -EPROTO;
            for (uint32_t j=0;j<i;++j) if ((t->links[j].flags&CC_LINK_TYPE)==type &&
                t->links[j].source_id==native->source_id && t->links[j].sink_id==native->sink_id) return -EPROTO;
            const struct object *a_owner=find_object(map,a->entity_id,ENTITY),*b_owner=find_object(map,b->entity_id,ENTITY);
            if (!a_owner || !b_owner) return -EPROTO;
            size_t at=media->num_links++;
            media->links[at]=(struct media_link_desc){.source=media->entities[a_owner->index].pads[a->index],
                .sink=media->entities[b_owner->index].pads[b->index],.flags=native->flags};
            t->data_link_ids[at]=native->id;
            ++media->entities[a_owner->index].desc.links; ++media->entities[b_owner->index].desc.links;
        } else if (type==MEDIA_LNK_FL_INTERFACE_LINK) {
            const struct object *interface=find_object(map,native->source_id,INTERFACE),*owner=find_object(map,native->sink_id,ENTITY);
            if (!interface || !owner) return -EPROTO;
            const struct media_v2_interface *node=&t->interfaces[interface->index];
            if ((node->intf_type!=MEDIA_INTF_T_V4L_VIDEO && node->intf_type!=MEDIA_INTF_T_V4L_SUBDEV) || !node->devnode.major) continue;
            struct cc_entity *entity=&media->entities[owner->index];
            if (entity->desc.dev.major && (entity->desc.dev.major!=node->devnode.major || entity->desc.dev.minor!=node->devnode.minor))
                return -ENOTUNIQ;
            entity->desc.dev.major=node->devnode.major; entity->desc.dev.minor=node->devnode.minor;
        } else if (type==MEDIA_LNK_FL_ANCILLARY_LINK) {
            if (native->source_id==native->sink_id || !find_object(map,native->source_id,ENTITY) ||
                !find_object(map,native->sink_id,ENTITY)) return -EPROTO;
        } else {
            /* Keep future link types in the native snapshot, but they never
             * become pad-to-pad routes or device-node associations. */
            bool source=false,sink=false;
            for (size_t j=0;j<map->count;++j) {
                source|=map->objects[j].id==native->source_id;
                sink|=map->objects[j].id==native->sink_id;
            }
            if (!source || !sink) return -EPROTO;
        }
    }
    for (size_t i=0;i<media->num_entities;++i) {
        struct cc_entity *entity=&media->entities[i];
        if (entity->desc.dev.major && media->io.ops->resolve(media->io.ctx,entity->desc.dev.major,entity->desc.dev.minor,
            entity->path,sizeof(entity->path))<0) entity->path[0]='\0';
    }
    return 0;
}
int cc_media_populate_v2(struct cc_media *media)
{
    if (!MEDIA_V2_PAD_HAS_INDEX(media->info.media_version)) return -ENOTSUP;
    struct cc_media_topology *t=NULL;
    int ret=snapshot(media,&t);
    if (ret<0) return ret;
    media->topology=t;
    struct object_map map={0};
    ret=make_map(t,&map);
    if (!ret) ret=populate_entities(media,&map);
    if (!ret) ret=populate_links(media,&map);
    free(map.objects);
    return ret;
}
int cc_media_topology_current(struct cc_media *media)
{
    if (!media->topology) return 0;
    struct media_v2_topology now={0};
    int ret=cc_ioctl(&media->io,media->fd,MEDIA_IOC_G_TOPOLOGY,&now);
    if (ret<0) return ret;
    if (now.ptr_entities || now.ptr_interfaces || now.ptr_pads || now.ptr_links) return -EPROTO;
    return now.topology_version==media->topology->info.topology_version && same_counts(&now,&media->topology->info) ? 0:-ESTALE;
}
int cc_media_topology_lens(struct cc_media *media,uint32_t sensor,struct cc_entity **out)
{
    int ret=cc_media_topology_current(media);
    if (ret<0) return ret;
    const struct cc_media_topology *t=media->topology;
    struct cc_entity *found=NULL;
    for (uint32_t i=0;i<t->info.num_links;++i) {
        const struct media_v2_link *link=&t->links[i];
        if ((link->flags&CC_LINK_TYPE)!=MEDIA_LNK_FL_ANCILLARY_LINK || link->source_id!=sensor) continue;
        struct cc_entity *entity=cc_media_entity(media,link->sink_id);
        if (!entity) return -ESTALE;
        if (entity->desc.type!=MEDIA_ENT_F_LENS) continue;
        if (found && found!=entity) return -ENOTUNIQ;
        found=entity;
    }
    if (!found) return -ENOENT;
    *out=found; return 0;
}
void cc_media_topology_link_changed(struct cc_media *media,size_t index,uint32_t flags)
{
    struct cc_media_topology *t=media->topology;
    if (!t) return;
    for (uint32_t i=0;i<t->info.num_links;++i)
        if (t->links[i].id==t->data_link_ids[index]) { t->links[i].flags=flags; return; }
}
