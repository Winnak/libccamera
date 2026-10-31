/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "internal.h"
#include <errno.h>
#include "fuzz_media_v2.h"

enum { MAX_ENTITIES = 16, MAX_LINKS = 64 };
struct driver {
    struct media_entity_desc entities[MAX_ENTITIES];
    struct media_link_desc links[MAX_LINKS];
    size_t count, num_links, enumerated;
    unsigned opens, closes;
    uint8_t mode;
};

static int driver_open(void *ctx, const char *path, int flags)
{
    (void)path; (void)flags;
    struct driver *d = ctx; ++d->opens;
    return 42;
}
static int driver_close(void *ctx, int fd)
{
    struct driver *d = ctx;
    FUZZ_CHECK(fd == 42); ++d->closes;
    return 0;
}
static int driver_lock(void *ctx, int fd)
{
    (void)fd;
    return ((struct driver *)ctx)->mode & 128U ? -EBUSY : 0;
}
static int driver_ioctl(void *ctx, int fd, unsigned long request, void *arg)
{
    (void)fd;
    struct driver *d = ctx;
    if (request == MEDIA_IOC_DEVICE_INFO) { memset(arg, 0, sizeof(struct media_device_info)); return 0; }
    if (request == MEDIA_IOC_ENUM_ENTITIES) {
        if (d->enumerated == d->count) return -EINVAL;
        *(struct media_entity_desc *)arg = d->entities[d->enumerated++];
        return 0;
    }
    if (request == MEDIA_IOC_ENUM_LINKS) {
        struct media_links_enum *e = arg;
        size_t index = 0;
        while (index < d->count && d->entities[index].id != e->entity) ++index;
        FUZZ_CHECK(index < d->count);
        for (uint32_t i = 0; i < d->entities[index].pads; ++i) {
            e->pads[i] = (struct media_pad_desc){.entity = e->entity, .index = (uint16_t)i,
                .flags = i & 1U ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK};
            if (d->mode & 4U) e->pads[i].entity = 0;
        }
        memcpy(e->links, d->links, d->num_links * sizeof(*d->links));
        return 0;
    }
    FUZZ_CHECK(request == MEDIA_IOC_SETUP_LINK);
    return d->mode & 64U ? -EIO : 0;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    struct driver d = {.mode = fuzz_u8(&in)};
    if (d.mode&32U) { fuzz_media_v2(&in); return 0; }
    d.count = 1U + fuzz_u8(&in) % MAX_ENTITIES;
    d.num_links = fuzz_u8(&in) % (MAX_LINKS + 1U);
    for (size_t i = 0; i < d.count; ++i) {
        d.entities[i] = (struct media_entity_desc){.id = (uint32_t)i + 1,
            .pads = 2, .links = (uint16_t)d.num_links};
        memset(d.entities[i].name, 'E', sizeof(d.entities[i].name));
    }
    if (d.mode & 1U) d.entities[d.count - 1U].id = fuzz_u32(&in);
    if (d.mode & 2U) d.entities[0].pads = fuzz_u16(&in);
    for (size_t i = 0; i < d.num_links; ++i) {
        struct media_link_desc *l = &d.links[i];
        l->source.entity = 1U + fuzz_u8(&in) % (uint32_t)d.count;
        l->sink.entity = 1U + fuzz_u8(&in) % (uint32_t)d.count;
        l->source.index = 1; l->sink.index = 0;
        l->flags = fuzz_u8(&in) & (MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
        if (d.mode & 8U) { l->source.index = fuzz_u16(&in); l->sink.index = fuzz_u16(&in); }
        if (d.mode & 16U) l->sink.entity = fuzz_u32(&in);
    }
    const struct cc_io_ops ops = {.open = driver_open, .close = driver_close,
        .lock = driver_lock, .ioctl = driver_ioctl};
    const struct cc_io io = {&ops, &d};
    struct cc_media *media = NULL;
    int ret = cc_media_open(&io, "/fuzz-media", true, &media);
    FUZZ_CHECK(ret <= 0);
    if (ret) FUZZ_CHECK(!media);
    else {
        FUZZ_CHECK(media && media->num_entities == d.count && media->num_links <= d.num_links);
        /* All-pairs shortest paths are an independent oracle for BFS routing. */
        unsigned distance[MAX_ENTITIES][MAX_ENTITIES];
        for (size_t i = 0; i < d.count; ++i) for (size_t j = 0; j < d.count; ++j)
            distance[i][j] = i == j ? 0 : MAX_ENTITIES + 1;
        for (size_t i = 0; i < media->num_links; ++i) {
            struct media_link_desc *l = &media->links[i];
            struct cc_entity *source = cc_media_entity(media, l->source.entity);
            struct cc_entity *sink = cc_media_entity(media, l->sink.entity);
            FUZZ_CHECK(source && sink);
            if (source != sink) distance[source - media->entities][sink - media->entities] = 1;
        }
        for (size_t k = 0; k < d.count; ++k) for (size_t i = 0; i < d.count; ++i)
            for (size_t j = 0; j < d.count; ++j)
                if (distance[i][j] > distance[i][k] + distance[k][j])
                    distance[i][j] = distance[i][k] + distance[k][j];
        for (size_t i = 0; i < d.count; ++i) for (size_t j = 0; j < d.count; ++j) {
            size_t *route = NULL, count = 0;
            uint32_t source = media->entities[i].desc.id, sink = media->entities[j].desc.id;
            ret = cc_media_route(media, source, sink, &route, &count);
            if (i == j || distance[i][j] > MAX_ENTITIES) FUZZ_CHECK(ret == -ENOENT && !route && !count);
            else {
                FUZZ_CHECK(ret == 0 && count == distance[i][j] && count < d.count);
                for (size_t n = 0; n < count; ++n) {
                    FUZZ_CHECK(route[n] < media->num_links && media->links[route[n]].source.entity == source);
                    source = media->links[route[n]].sink.entity;
                }
                FUZZ_CHECK(source == sink);
            }
            free(route);
        }
        for (size_t i = 0; i < media->num_links; ++i) {
            struct media_link_desc before = media->links[i];
            ret = cc_media_set_link(media, i, before.flags ^ MEDIA_LNK_FL_ENABLED);
            if (ret) FUZZ_CHECK(!memcmp(&media->links[i], &before, sizeof(before)));
            else FUZZ_CHECK(media->links[i].flags == (before.flags ^ MEDIA_LNK_FL_ENABLED));
        }
        cc_media_close(media);
    }
    FUZZ_CHECK(d.opens == 1 && d.closes == 1);
    return 0;
}
