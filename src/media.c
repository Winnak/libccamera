/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "media_topology.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#define CC_MEDIA_LIMIT 4096U

struct cc_entity *cc_media_entity(struct cc_media *media, uint32_t id)
{
    for (size_t i = 0; i < media->num_entities; ++i)
        if (media->entities[i].desc.id == id)
            return &media->entities[i];
    return NULL;
}

void cc_media_close(struct cc_media *media)
{
    if (!media)
        return;
    for (size_t i = 0; i < media->num_entities; ++i)
        free(media->entities[i].pads);
    free(media->entities);
    free(media->links);
    cc_media_topology_free(media->topology);
    if (media->fd >= 0)
        media->io.ops->close(media->io.ctx, media->fd);
    free(media);
}

static int read_entity_links(struct cc_media *media, struct cc_entity *entity)
{
    const size_t num_pads = entity->desc.pads, num_links = entity->desc.links;
    if (num_pads > CC_MEDIA_LIMIT || num_links > CC_MEDIA_LIMIT)
        return -EOVERFLOW;
    if (entity->pads)
        return -EINVAL;
    /* Even empty entities get non-NULL scratch arrays, avoiding calloc(0)
     * semantics and keeping ownership local until enumeration succeeds. */
    struct media_pad_desc *pads = calloc(num_pads ? num_pads : 1, sizeof(*pads));
    struct media_link_desc *links = calloc(num_links ? num_links : 1, sizeof(*links));
    int ret = -ENOMEM;
    if (!pads || !links)
        goto done;
    struct media_links_enum enumeration = {
        .entity = entity->desc.id, .pads = pads, .links = links
    };
    ret = cc_ioctl(&media->io, media->fd, MEDIA_IOC_ENUM_LINKS, &enumeration);
    if (ret < 0)
        goto done;
    for (size_t j = 0; j < num_pads; ++j) {
        if (pads[j].entity != entity->desc.id || pads[j].index != j) {
            ret = -EPROTO;
            goto done;
        }
    }
    /* ENUM_LINKS reports each link at both ends. Keep only source copies. */
    for (size_t j = 0; j < num_links; ++j) {
        if (links[j].source.entity != entity->desc.id ||
            (links[j].flags & UINT32_C(0xf0000000)) != MEDIA_LNK_FL_DATA_LINK)
            continue;
        if (media->num_links >= CC_MEDIA_LIMIT || !cc_media_entity(media, links[j].sink.entity)) {
            ret = -EPROTO;
            goto done;
        }
        struct media_link_desc *p = realloc(media->links, (media->num_links + 1) * sizeof(*p));
        if (!p) {
            ret = -ENOMEM;
            goto done;
        }
        media->links = p;
        p[media->num_links++] = links[j];
    }
    entity->pads = pads;
    pads = NULL;
    ret = 0;
done:
    free(pads);
    free(links);
    return ret;
}

int cc_media_open(const struct cc_io *io, const char *path, bool exclusive, struct cc_media **out)
{
    if (!io || !path || !out)
        return -EINVAL;
    *out = NULL;
    struct cc_media *media = calloc(1, sizeof(*media));
    if (!media)
        return -ENOMEM;
    media->io = *io;
    media->fd = io->ops->open(io->ctx, path, O_RDWR | O_CLOEXEC);
    int ret = media->fd;
    if (ret < 0)
        goto fail;
    if (exclusive) {
        ret = io->ops->lock(io->ctx, media->fd);
        if (ret < 0)
            goto fail;
    }
    ret = cc_ioctl(io, media->fd, MEDIA_IOC_DEVICE_INFO, &media->info);
    if (ret < 0)
        goto fail;
    ret=cc_media_populate_v2(media);
    if (!ret) { *out=media; return 0; }
    if (ret!=-ENOTSUP) goto fail;

    uint32_t previous_id = 0;
    for (;;) {
        struct media_entity_desc desc = { .id = previous_id | MEDIA_ENT_ID_FLAG_NEXT };
        ret = cc_ioctl(io, media->fd, MEDIA_IOC_ENUM_ENTITIES, &desc);
        if (ret == -EINVAL)
            break;
        if (ret < 0)
            goto fail;
        if (desc.id <= previous_id || desc.id & MEDIA_ENT_ID_FLAG_NEXT ||
            media->num_entities >= CC_MEDIA_LIMIT) {
            ret = -EOVERFLOW;
            goto fail;
        }
        previous_id = desc.id;
        struct cc_entity *p = realloc(media->entities,
                                      (media->num_entities + 1) * sizeof(*p));
        if (!p) {
            ret = -ENOMEM;
            goto fail;
        }
        media->entities = p;
        struct cc_entity *entity = &p[media->num_entities++];
        memset(entity, 0, sizeof(*entity));
        entity->desc = desc;
        entity->desc.name[sizeof(entity->desc.name) - 1] = '\0';
        cc_copy_string(entity->name,sizeof(entity->name),entity->desc.name,sizeof(entity->desc.name));
        if (desc.dev.major) {
            /* Some entities have no userspace device node. Resolve only the
             * nodes that actually exist; a selected route checks them later. */
            if (io->ops->resolve(io->ctx, desc.dev.major, desc.dev.minor,
                                 entity->path, sizeof(entity->path)) < 0)
                entity->path[0] = '\0';
        }
    }

    for (size_t i = 0; i < media->num_entities; ++i) {
        ret = read_entity_links(media, &media->entities[i]);
        if (ret < 0)
            goto fail;
    }
    for (size_t i = 0; i < media->num_links; ++i) {
        struct media_link_desc *link = &media->links[i];
        struct cc_entity *source = cc_media_entity(media, link->source.entity);
        struct cc_entity *sink = cc_media_entity(media, link->sink.entity);
        if (!source || !sink || link->source.index >= source->desc.pads ||
            link->sink.index >= sink->desc.pads ||
            !(source->pads[link->source.index].flags & MEDIA_PAD_FL_SOURCE) ||
            !(sink->pads[link->sink.index].flags & MEDIA_PAD_FL_SINK)) {
            ret = -EPROTO;
            goto fail;
        }
    }
    *out = media;
    return 0;
fail:
    cc_media_close(media);
    return ret;
}

int cc_media_set_link(struct cc_media *media, size_t index, uint32_t flags)
{
    if (!media || index >= media->num_links)
        return -EINVAL;
    struct media_link_desc link = media->links[index];
    if (link.flags == flags)
        return 0;
    if (link.flags & MEDIA_LNK_FL_IMMUTABLE)
        return -EPERM;
    if ((flags & UINT32_C(0xf0000000))!=MEDIA_LNK_FL_DATA_LINK) return -EINVAL;
    int ret=cc_media_topology_current(media);
    if (ret<0) return ret;
    link.flags = flags;
    ret = cc_ioctl(&media->io, media->fd, MEDIA_IOC_SETUP_LINK, &link);
    if (ret < 0)
        return ret;
    const struct media_link_desc *previous=&media->links[index];
    if (link.source.entity!=previous->source.entity || link.source.index!=previous->source.index ||
        link.sink.entity!=previous->sink.entity || link.sink.index!=previous->sink.index ||
        (link.flags & UINT32_C(0xf0000000))!=MEDIA_LNK_FL_DATA_LINK) return -EPROTO;
    media->links[index] = link;
    cc_media_topology_link_changed(media,index,link.flags);
    return 0;
}

int cc_media_lens(struct cc_media *media, uint32_t sensor, struct cc_entity **out)
{
    if (!media || !out) return -EINVAL;
    *out=NULL;
    if (media->topology) return cc_media_topology_lens(media,sensor,out);
    /* Ancillary links connect entity IDs, not pad IDs, and are not reported
     * by legacy ENUM_LINKS. Fetch only the v2 link array. Keep it separate
     * from routing's pad-to-pad data links. */
    for (unsigned int attempt=0;attempt<3;++attempt) {
        struct media_v2_topology t={0};
        int ret=cc_ioctl(&media->io,media->fd,MEDIA_IOC_G_TOPOLOGY,&t);
        if (ret<0) return ret==-ENOTTY || ret==-EINVAL ? -ENOTSUP:ret;
        if (t.num_links>CC_MEDIA_LIMIT) return -EOVERFLOW;
        uint32_t count=t.num_links; uint64_t version=t.topology_version;
        struct media_v2_link *links=calloc(count ? count:1,sizeof(*links));
        if (!links) return -ENOMEM;
        t.ptr_links=(uint64_t)(uintptr_t)links;
        ret=cc_ioctl(&media->io,media->fd,MEDIA_IOC_G_TOPOLOGY,&t);
        if (ret==-ENOSPC || (!ret && (t.topology_version!=version || t.num_links!=count))) { free(links); continue; }
        struct cc_entity *found=NULL;
        if (!ret) for (uint32_t i=0;i<count;++i) {
            if ((links[i].flags & UINT32_C(0xf0000000))!=MEDIA_LNK_FL_ANCILLARY_LINK || links[i].source_id!=sensor) continue;
            struct cc_entity *e=cc_media_entity(media,links[i].sink_id);
            if (!e) { ret=-ESTALE; break; }
            if (e->desc.type!=MEDIA_ENT_F_LENS) continue;
            if (found && found!=e) { ret=-ENOTUNIQ; break; }
            found=e;
        }
        free(links);
        if (ret<0) return ret;
        if (!found) return -ENOENT;
        *out=found; return 0;
    }
    return -EAGAIN;
}

static int route_from_pad(struct cc_media *media, uint32_t sensor, int64_t source_pad, uint32_t video,
                          size_t **links, size_t *count)
{
    if (!media || !links || !count)
        return -EINVAL;
    *links = NULL;
    *count = 0;
    struct cc_entity *source = cc_media_entity(media, sensor);
    struct cc_entity *sink = cc_media_entity(media, video);
    if (!source || !sink || source == sink)
        return -ENOENT;
    size_t n = media->num_entities;
    size_t *parent = malloc(n * sizeof(*parent));
    size_t *queue = malloc(n * sizeof(*queue));
    if (!parent || !queue) {
        free(parent);
        free(queue);
        return -ENOMEM;
    }
    for (size_t i = 0; i < n; ++i)
        parent[i] = SIZE_MAX;
    size_t first = (size_t)(source - media->entities);
    size_t last = (size_t)(sink - media->entities);
    size_t head = 0, tail = 0;
    queue[tail++] = first;
    parent[first] = media->num_links; /* Mark root visited. */
    while (head < tail && parent[last] == SIZE_MAX) {
        struct cc_entity *e = &media->entities[queue[head++]];
        for (size_t i = 0; i < media->num_links; ++i) {
            struct media_link_desc *link = &media->links[i];
            if (link->source.entity != e->desc.id)
                continue;
            if (e == source && source_pad >= 0 && link->source.index != (uint64_t)source_pad)
                continue;
            struct cc_entity *next = cc_media_entity(media, link->sink.entity);
            if (!next)
                continue;
            size_t next_index = (size_t)(next - media->entities);
            if (parent[next_index] != SIZE_MAX)
                continue;
            parent[next_index] = i;
            queue[tail++] = next_index;
        }
    }
    int ret = -ENOENT;
    if (parent[last] != SIZE_MAX) {
        size_t length = 0;
        for (size_t at = last; at != first;) {
            queue[length++] = parent[at];
            source = cc_media_entity(media, media->links[parent[at]].source.entity);
            at = (size_t)(source - media->entities);
        }
        *links = malloc(length * sizeof(**links));
        if (!*links) {
            ret = -ENOMEM;
        } else {
            for (size_t i = 0; i < length; ++i)
                (*links)[i] = queue[length - i - 1];
            *count = length;
            ret = 0;
        }
    }
    free(parent);
    free(queue);
    return ret;
}
int cc_media_route(struct cc_media *media, uint32_t sensor, uint32_t video, size_t **links, size_t *count)
{
    return route_from_pad(media,sensor,-1,video,links,count);
}
int cc_media_route_pad(struct cc_media *media, uint32_t sensor, uint32_t pad, uint32_t video,
                        size_t **links, size_t *count)
{
    return route_from_pad(media,sensor,pad,video,links,count);
}
