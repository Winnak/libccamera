/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <ccamera/ipa_proxy.h>
#include "subdev_compat.h"
#include "request_buffers.h"
#include "fence.h"
#include <ccamera/capture.h>
#include <ccamera/configuration.h>
#include <ccamera/ipa_module.h>
#include <ccamera/ipa_manager.h>
#include <ccamera/linux/bcm2835-isp.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/media-bus-format.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "media_v2_driver.h"

static const char *proxy_worker;
static int create_reference(const cc_ipa_reference_tuning *tuning, cc_ipa **ipa)
{
    if (!proxy_worker) return cc_ipa_reference_create(tuning, ipa);
    cc_ipa_proxy_options options = {.worker_path = proxy_worker};
    return cc_ipa_reference_create_proxy(tuning, &options, ipa);
}
static int proxy_available(void)
{
    cc_ipa_proxy_options options = {.worker_path = proxy_worker}; cc_ipa *ipa = NULL;
    int ret = cc_ipa_vc4_create_proxy(NULL, &options, &ipa); cc_ipa_destroy(ipa);
    return ret == -ENOSYS || ret == -EPERM || ret == -EACCES ? 77 : ret;
}
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x, e) CHECK((x) == -(e))
#define NODES 16
#define SLOTS 4
#define PAYLOAD_CONTROL (V4L2_CID_USER_BASE + 0x2345U)
static bool use_v2_topology;

struct buffer {
    void *data;
    uint32_t size;
    int maps, refs, attachments;
    bool kernel, queued, done, attached;
    int attached_node, attached_slot, attached_plane;
    int input_fd;
    uint64_t timestamp;
    uint32_t used;
    int32_t lens_setting;
    uint32_t sequence;
};
struct node {
    bool open, streaming;
    cc_format format;
    uint32_t memory, count;
    struct buffer buffers[SLOTS];
    struct buffer chroma[SLOTS][2];
};
struct fake {
    bool validation_only;
    bool physical_array_larger, unicam;
    unsigned try_formats;
    struct node nodes[NODES];
    struct { bool open; int node, slot, plane, fence; uint64_t identity; } fds[4096];
    struct { int refs, status; uint64_t signal_at; uint32_t malformed; } fences[16];
    int fence_queries, fence_query_error, interrupt_waits;
    bool clock_error;
    bool fresh_exports;
    bool history_ports;
    uint64_t export_identity;
    int create_error, fail_identity;
    bool shading_test, heap_open, shading_open, refuse_heap, fail_shading_map, fail_sync;
    void *shading_map;
    unsigned shading_start, shading_end, shading_controls;
    struct bcm2835_isp_lens_shading last_shading;
    struct bcm2835_isp_gamma last_gamma;
    bool media_open, fail_stop, mismatch_input, bad_crop, selection_changes_layout, wrong_driver, pisp, bad_parameters;
    int missing_port, corrupt_port;
    int handles, mappings, next_fd, duplicates, fail_duplicate, map_calls, fail_map, starts;
    int wait_calls, delayed_port, fail_node, fail_nth, fail_errno;
    unsigned long fail_req;
    uint32_t counts[NODES];
    uint32_t hardware_version;
    uint64_t now;
    cc_rectangle crop;
    int32_t gain;
    int32_t blue_gain, digital_gain;
    bool ipa_scene;
    bool timing_plan;
    unsigned char payload[8];
    void *last_input;
    struct pisp_be_tiles_config last_parameters;
    bool widen_chroma, refuse_capacity, dma_pending, force_color_defaults;
    int format_sets[NODES];
    int capture_overlap;
    int32_t submitted_gain[SLOTS];
    unsigned char submitted_payload[SLOTS][8];
    int switch_delay_at, next_delayed_port;
    bool sensor_support, event_subscribed, event_pending;
    uint32_t sensor_sequence, event_serial, event_frame;
    int32_t exposure;
    struct { uint32_t frame; int32_t value; } exposure_writes[16];
    size_t exposure_write_count, processed_count;
    unsigned char processed_exposures[16];
    bool cfe, cfe_media_open, subdev_open[4], bad_cfe_statistics;
    struct media_link_desc cfe_links[8];
    struct v4l2_mbus_framefmt mbus[3][2][8];
    struct pisp_fe_config fe_parameters;
    uint32_t cfe_frame, cfe_next_frame;
    int32_t analogue_gain, vblank;
    struct { uint32_t frame, id; int32_t value; } sensor_writes[256];
    size_t sensor_write_count;
    unsigned int cfe_processed;
    uint32_t compression;
    bool mismatch_compression_offset;
    bool af_lens, fail_lens;
    int32_t lens_position;
    double focus_peak;
    bool embedded, bad_embedded, short_embedded;
    bool flat_focus;
};

static struct buffer *buffer_plane(struct node *node, int slot, int plane)
{
    CHECK(slot >= 0 && slot < SLOTS && plane >= 0 && plane < 3);
    return plane ? &node->chroma[slot][plane - 1] : &node->buffers[slot];
}

static void maybe_free(struct buffer *b)
{
    if (!b->kernel && !b->maps && !b->refs && !b->attachments) { free(b->data); b->data = NULL; }
}

static void detach(struct fake *f, struct buffer *input)
{
    if (!input->attached) return;
    struct buffer *source = buffer_plane(&f->nodes[input->attached_node], input->attached_slot, input->attached_plane);
    CHECK(source->attachments > 0);
    --source->attachments;
    input->attached = false;
    maybe_free(source);
}

static struct buffer *memory_plane(struct fake *f, int node, int slot, int plane)
{
    struct buffer *b = buffer_plane(&f->nodes[node], slot, plane);
    return b->attached ? buffer_plane(&f->nodes[b->attached_node], b->attached_slot, b->attached_plane) : b;
}

static void init(struct fake *f)
{
    memset(f, 0, sizeof(*f));
    f->next_fd = 100;
    f->export_identity = 1000;
    f->gain = 1000;
    f->delayed_port = f->fail_node = f->missing_port = f->corrupt_port = -1;
    for (int i = 0; i < NODES; ++i) f->counts[i] = 3;
    f->counts[1] = 4;
    f->counts[2] = 2;
    f->vblank = 50;
    /* Sensor -> CSI bridge -> raw/FE; FE image/statistics and config input. */
    const uint32_t sources[] = {1,3,3,4,4,7}, sinks[] = {3,2,4,5,6,4};
    const uint16_t source_pads[] = {0,4,4,2,4,0}, sink_pads[] = {0,0,0,0,0,1};
    for (size_t i=0;i<6;++i) f->cfe_links[i]=(struct media_link_desc) {
        .source={.entity=sources[i],.index=source_pads[i],.flags=MEDIA_PAD_FL_SOURCE},
        .sink={.entity=sinks[i],.index=sink_pads[i],.flags=MEDIA_PAD_FL_SINK}};
    f->cfe_links[6]=(struct media_link_desc){.source={.entity=1,.index=1,.flags=MEDIA_PAD_FL_SOURCE},
        .sink={.entity=3,.index=1,.flags=MEDIA_PAD_FL_SINK}};
    f->cfe_links[7]=(struct media_link_desc){.source={.entity=3,.index=5,.flags=MEDIA_PAD_FL_SOURCE},
        .sink={.entity=9,.index=0,.flags=MEDIA_PAD_FL_SINK}};
}

static int open_fd(void *ctx, const char *path, int flags)
{
    struct fake *f = ctx;
    CHECK(flags & O_CLOEXEC);
    if (f->shading_test && !strncmp(path,"/dev/dma_heap/",14)) {
        if (f->refuse_heap) return -EACCES;
        CHECK(!f->heap_open);f->heap_open=true;++f->handles;return 18;
    }
    if (!strcmp(path, "/dev/media0")) {
        CHECK(!f->media_open);
        f->media_open = true;
        ++f->handles;
        return 10;
    }
    if (f->cfe && !strcmp(path,"/dev/media1")) {
        CHECK(!f->cfe_media_open); f->cfe_media_open=true; ++f->handles; return 11;
    }
    if (f->cfe) for (int i=0;i<(f->af_lens ? 4:3);++i) {
        char name[32]; snprintf(name,sizeof(name),"/dev/v4l-subdev%d",i);
        if (!strcmp(path,name)) {
            CHECK(!f->subdev_open[i]); f->subdev_open[i]=true; ++f->handles; return 50+i;
        }
    }
    for (int i = 0; i < NODES; ++i) {
        char name[32];
        snprintf(name, sizeof(name), "/dev/video%d", i);
        if (strcmp(name, path)) continue;
        CHECK(!f->nodes[i].open);
        f->nodes[i].open = true;
        ++f->handles;
        return 20 + i;
    }
    return -ENOENT;
}

static int close_fd(void *ctx, int fd)
{
    struct fake *f = ctx;
    if (f->shading_test && fd==18) {CHECK(f->heap_open);f->heap_open=false;--f->handles;return 0;}
    if (f->shading_test && fd==19) {
        CHECK(f->shading_open && !f->shading_map);
        for (unsigned i=0;i<4;++i) CHECK(!f->nodes[i].open);
        f->shading_open=false;--f->handles;return 0;
    }
    if (fd == 10) {
        CHECK(f->media_open);
        f->media_open = false;
    } else if (f->cfe && fd==11) {
        CHECK(f->cfe_media_open); f->cfe_media_open=false;
    } else if (f->cfe && fd>=50 && fd<54) {
        CHECK(f->subdev_open[fd-50]); f->subdev_open[fd-50]=false;
    } else if (fd >= 20 && fd < 20 + NODES) {
        struct node *n = &f->nodes[fd - 20];
        CHECK(n->open);
        if (fd==24) f->event_subscribed=f->event_pending=false;
        n->open = n->streaming = false;
        for (uint32_t i = 0; i < n->count; ++i) {
            n->buffers[i].queued = false;
            for (uint32_t p = 0; p < n->format.num_planes; ++p) detach(f, buffer_plane(n, (int)i, (int)p));
            if (n->memory == V4L2_MEMORY_MMAP)
                for (uint32_t p = 0; p < n->format.num_planes; ++p) {
                    struct buffer *b = buffer_plane(n, (int)i, (int)p);
                    b->kernel = false;
                    maybe_free(b);
                }
        }
        n->count = 0;
    } else {
        CHECK(fd >= 100 && fd < f->next_fd && f->fds[fd].open);
        if (f->fds[fd].fence) {
            CHECK(f->fences[f->fds[fd].fence - 1].refs > 0);
            --f->fences[f->fds[fd].fence - 1].refs; f->fds[fd].open = false; --f->handles; return 0;
        }
        struct buffer *b = buffer_plane(&f->nodes[f->fds[fd].node], f->fds[fd].slot, f->fds[fd].plane);
        for (int node = 0; node < NODES; ++node) for (uint32_t i = 0; i < f->nodes[node].count; ++i)
            for (uint32_t p = 0; p < f->nodes[node].format.num_planes; ++p) {
                const struct buffer *in = buffer_plane(&f->nodes[node], (int)i, (int)p);
                CHECK(!f->nodes[node].buffers[i].queued || f->nodes[node].buffers[i].done || !in->attached || in->input_fd != fd);
            }
        f->fds[fd].open = false;
        CHECK(b->refs > 0);
        if (f->pisp && f->fds[fd].node == 4 && b->refs == 1 && f->dma_pending)
            for (int port = 0; port < NODES; ++port)
                if (port < 4 || port >= 12) CHECK(!f->nodes[port].streaming);
        --b->refs;
        maybe_free(b);
    }
    --f->handles;
    return 0;
}

static int descriptor(struct fake *f, int node, int slot, int plane)
{
    int fd = f->next_fd++;
    CHECK(fd < 4096);
    f->fds[fd].open = true;
    f->fds[fd].node = node;
    f->fds[fd].slot = slot;
    f->fds[fd].plane = plane;
    f->fds[fd].identity = (uint64_t)(node * SLOTS * 3 + slot * 3 + plane + 1);
    ++buffer_plane(&f->nodes[node], slot, plane)->refs;
    ++f->handles;
    return fd;
}
static int fence_descriptor(struct fake *f, int fence)
{
    CHECK(fence >= 0 && fence < 16 && f->next_fd < 4096);
    int fd = f->next_fd++;
    f->fds[fd].open = true; f->fds[fd].node = -1; f->fds[fd].fence = fence + 1;
    ++f->fences[fence].refs; ++f->handles; return fd;
}

static int duplicate(void *ctx, int fd)
{
    struct fake *f = ctx;
    if (++f->duplicates == f->fail_duplicate) return -EMFILE;
    CHECK(fd >= 100 && fd < f->next_fd && f->fds[fd].open);
    if (f->fds[fd].fence) return fence_descriptor(f, f->fds[fd].fence - 1);
    int result = descriptor(f, f->fds[fd].node, f->fds[fd].slot, f->fds[fd].plane);
    f->fds[result].identity = f->fds[fd].identity;
    return result;
}
static int identity(void *ctx, int fd, uint64_t *device, uint64_t *inode)
{
    struct fake *f = ctx;
    CHECK(fd >= 100 && fd < f->next_fd && f->fds[fd].open);
    if (f->fail_identity > 0 && --f->fail_identity == 0) return -EIO;
    *device = 81;
    *inode = f->fds[fd].identity;
    return 0;
}
static int lock_fd(void *ctx, int fd) { (void)ctx; (void)fd; return 0; }
static int now(void *ctx, uint64_t *out)
{
    struct fake *f = ctx; if (f->clock_error) return -EIO;
    *out = f->now; return 0;
}
static int wait_fd(void *ctx, int fd, short events, int timeout)
{
    struct fake *f = ctx;
    CHECK(fd >= 20 && fd < 20+NODES && timeout > 0);
    CHECK(events == (fd == 20 || (f->pisp && (fd == 22 || fd == 32 || fd == 34)) || (f->cfe && fd==26) ? POLLOUT : POLLIN) ||
          (fd==24 && f->event_subscribed && (events==POLLPRI || events==(POLLIN|POLLPRI))));
    f->now += UINT64_C(1000000);
    ++f->wait_calls;
    return -EAGAIN;
}
static int wait_many(void *ctx, struct pollfd *fds, size_t count, int timeout)
{
    struct fake *f = ctx;
    CHECK(count > 0 && count <= 784 && timeout >= -1);
    bool have_fence = false, have_hardware = false;
    uint64_t next = UINT64_MAX;
    for (size_t i = 0; i < count; ++i) {
        if (fds[i].fd >= 100) {
            CHECK(fds[i].fd < f->next_fd && f->fds[fds[i].fd].open && f->fds[fds[i].fd].fence && fds[i].events == POLLIN);
            have_fence = true; int object = f->fds[fds[i].fd].fence - 1;
            uint64_t ready = f->fences[object].status ? f->now : f->fences[object].signal_at;
            if (ready && ready < next) next = ready;
            continue;
        }
        have_hardware = true;
        CHECK(fds[i].fd >= 20 && fds[i].fd < 20+NODES);
        CHECK(fds[i].events == (fds[i].fd == 20 || (f->pisp && (fds[i].fd == 22 || fds[i].fd == 32 || fds[i].fd == 34)) ||
              (f->cfe && fds[i].fd==26) ? POLLOUT : POLLIN) ||
              (fds[i].fd == 24 && f->event_subscribed &&
               (fds[i].events == POLLPRI || fds[i].events == (POLLIN | POLLPRI))));
    }
    if (f->interrupt_waits > 0) { --f->interrupt_waits; return -EINTR; }
    if (have_fence && !have_hardware) {
        ++f->wait_calls;
        uint64_t end = timeout < 0 ? UINT64_MAX : f->now + (uint64_t)timeout * UINT64_C(1000000);
        CHECK(end != UINT64_MAX || next != UINT64_MAX);
        if (next < end) { f->now = next; return 0; }
        f->now = end; return -ETIMEDOUT;
    }
    ++f->wait_calls; f->now += UINT64_C(1000000);
    if (f->wait_calls == f->switch_delay_at) f->delayed_port = f->next_delayed_port;
    return -EAGAIN;
}

static int resolve(void *ctx, unsigned int major_id, unsigned int minor_id, char *path, size_t size)
{
    struct fake *f=ctx;
    CHECK(major_id == 81);
    if (f->cfe && minor_id>=20 && minor_id<24)
        snprintf(path,size,"/dev/v4l-subdev%u",minor_id-20);
    else {
        CHECK(minor_id<NODES);
        snprintf(path, size, "/dev/video%u", minor_id);
    }
    return 0;
}
static int map(void *ctx, int fd, size_t length, off_t offset, void **out)
{
    struct fake *f = ctx;
    if (f->shading_test && fd==19) {
        CHECK(f->shading_open && !f->shading_map && length==64U*49U*8U);
        if (f->fail_shading_map) return -ENOMEM;
        f->shading_map=calloc(1,length);CHECK(f->shading_map);*out=f->shading_map;++f->mappings;return 0;
    }
    CHECK(fd >= 20 && fd < 20+NODES && offset >= 0 && offset % 65536 == 0);
    uint32_t slot = (uint32_t)(offset / 65536), plane = 0;
    if (f->pisp && (fd==20 || fd==21 || fd==23 || fd>=32)) { plane = slot % 3; slot /= 3; }
    CHECK(slot < f->nodes[fd - 20].count);
    struct buffer *b = buffer_plane(&f->nodes[fd - 20], (int)slot, (int)plane);
    CHECK(b->kernel && b->data && b->size == length);
    if (++f->map_calls == f->fail_map) return -ENOMEM;
    ++b->maps;
    ++f->mappings;
    *out = b->data;
    return 0;
}
static int unmap(void *ctx, void *data, size_t length)
{
    struct fake *f = ctx;
    if (f->shading_test && data==f->shading_map) {
        for (unsigned i=0;i<4;++i) CHECK(!f->nodes[i].open);
        CHECK(length==64U*49U*8U);free(data);f->shading_map=NULL;--f->mappings;return 0;
    }
    for (int i = 0; i < NODES; ++i)
        for (int j = 0; j < SLOTS; ++j) for (int p = 0; p < 3; ++p) {
            struct buffer *b = buffer_plane(&f->nodes[i], j, p);
            if (b->data != data) continue;
            CHECK(b->maps && b->size == length && !f->nodes[i].streaming);
            if (f->pisp && i < 4)
                for (int port = 0; port < NODES; ++port)
                if (port < 4 || port >= 12) CHECK(!f->nodes[port].streaming);
            --b->maps;
            --f->mappings;
            maybe_free(b);
            return 0;
        }
    CHECK(false);
    return -EINVAL;
}

static int media_ioctl(struct fake *f, unsigned long req, void *arg)
{
    CHECK(f->media_open);
    if (req == MEDIA_IOC_DEVICE_INFO) {
        struct media_device_info *info = arg;
        strcpy(info->driver, f->wrong_driver ? "other" : f->pisp ? "pispbe" : "bcm2835-isp");
        info->hw_revision = f->hardware_version;
        return 0;
    }
    if (req == MEDIA_IOC_ENUM_ENTITIES) {
        struct media_entity_desc *d = arg;
        uint32_t id = d->id & ~MEDIA_ENT_ID_FLAG_NEXT;
        if (id >= (f->pisp ? 8U : 4U)) return -EINVAL;
        memset(d, 0, sizeof(*d));
        d->id = id + 1;
        d->type = MEDIA_ENT_F_IO_V4L;
        d->dev.major = 81;
        d->dev.minor = id < 4 ? id : id + 8U;
        if (f->pisp) {
            static const char *const names[] = { "pispbe-input", "pispbe-output0", "pispbe-config", "pispbe-output1", "pispbe-tdn_input",
                "pispbe-tdn_output", "pispbe-stitch_input", "pispbe-stitch_output" };
            strcpy(d->name, names[id]);
        } else if (!id) strcpy(d->name, "bcm2835-isp1-output0");
        else snprintf(d->name, sizeof(d->name), "bcm2835-isp1-capture%u", id);
        if ((int)id == f->missing_port) strcpy(d->name, "unrelated-node");
        return 0;
    }
    CHECK(req == MEDIA_IOC_ENUM_LINKS);
    return 0; /* Processing links are fixed; node names define the four ports. */
}

static struct media_entity_desc cfe_entity(const struct fake *f,uint32_t id)
{
    if (id==9) {
        struct media_entity_desc e={.id=9,.type=MEDIA_ENT_F_IO_V4L,.pads=1,.links=1};
        strcpy(e.name,"rp1-cfe-csi2-ch1"); e.dev.major=81; e.dev.minor=8; return e;
    }
    if (id==8) {
        struct media_entity_desc lens={.id=8,.type=MEDIA_ENT_F_LENS};
        strcpy(lens.name,"dw9807"); lens.dev.major=81; lens.dev.minor=23; return lens;
    }
    static const char *const names[] = {"imx219 10-0010","rp1-cfe-csi2-ch0","csi2","pisp-fe",
        "rp1-cfe-fe-image0","rp1-cfe-fe-stats","rp1-cfe-fe-config"};
    static const uint32_t functions[] = {MEDIA_ENT_F_CAM_SENSOR,MEDIA_ENT_F_IO_V4L,
        MEDIA_ENT_F_VID_IF_BRIDGE,MEDIA_ENT_F_PROC_VIDEO_ISP,MEDIA_ENT_F_IO_V4L,
        MEDIA_ENT_F_IO_V4L,MEDIA_ENT_F_IO_V4L};
    static const uint32_t minors[] = {20,7,21,22,4,5,6};
    static const uint16_t pads[] = {1,1,8,5,1,1,1}, links[] = {1,1,3,4,1,1,1};
    CHECK(id>=1 && id<=7);
    struct media_entity_desc d={.id=id,.type=functions[id-1],.pads=pads[id-1],.links=links[id-1]};
    strcpy(d.name,names[id-1]); d.dev.major=81; d.dev.minor=minors[id-1];
    if (f->unicam && id==2) strcpy(d.name,"unicam-image");
    if (f->embedded && id==1) { d.pads=2; ++d.links; strcpy(d.name,"imx708 10-001a"); }
    if (f->embedded && id==3) d.links+=2;
    return d;
}

static int cfe_media_ioctl(struct fake *f, unsigned long req, void *arg)
{
    CHECK(f->cfe_media_open);
    if (req==MEDIA_IOC_G_TOPOLOGY) {
        if (!f->af_lens) return -ENOTTY;
        struct media_v2_topology *t=arg;
        if (t->ptr_links) {
            CHECK(t->num_links>=1);
            struct media_v2_link *links=(void *)(uintptr_t)t->ptr_links;
            links[0]=(struct media_v2_link){.id=100,.source_id=1,.sink_id=8,.flags=MEDIA_LNK_FL_ANCILLARY_LINK};
        }
        t->topology_version=1; t->num_links=1; return 0;
    }
    if (req==MEDIA_IOC_DEVICE_INFO) {
        strcpy(((struct media_device_info *)arg)->driver,f->unicam ? "unicam":"rp1-cfe"); return 0;
    }
    if (req==MEDIA_IOC_ENUM_ENTITIES) {
        struct media_entity_desc *d=arg;
        uint32_t id=d->id & ~MEDIA_ENT_ID_FLAG_NEXT;
        uint32_t next=id+1;
        if (next==8 && !f->af_lens) ++next;
        if (next>(f->embedded ? 9U:f->af_lens ? 8U:7U)) return -EINVAL;
        *d=cfe_entity(f,next); return 0;
    }
    if (req==MEDIA_IOC_ENUM_LINKS) {
        struct media_links_enum *e=arg;
        struct media_entity_desc d=cfe_entity(f,e->entity);
        for (uint16_t i=0;i<d.pads;++i) e->pads[i]=(struct media_pad_desc) {
            .entity=d.id,.index=i,.flags=(d.id==2 || (d.id==3 && i<4) ||
                (d.id==4 && i<2) || d.id==5 || d.id==6 || d.id==9) ? MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE};
        size_t n=0;
        for (size_t i=0;i<(f->embedded ? 8U:6U);++i)
            if (f->cfe_links[i].source.entity==d.id || f->cfe_links[i].sink.entity==d.id)
                e->links[n++]=f->cfe_links[i];
        CHECK(n==d.links); return 0;
    }
    CHECK(req==MEDIA_IOC_SETUP_LINK);
    struct media_link_desc *link=arg;
    for (size_t i=0;i<(f->embedded ? 8U:6U);++i) if (f->cfe_links[i].source.entity==link->source.entity &&
        f->cfe_links[i].source.index==link->source.index && f->cfe_links[i].sink.entity==link->sink.entity &&
        f->cfe_links[i].sink.index==link->sink.index) {
        f->cfe_links[i]=*link; return 0;
    }
    CHECK(false); return -EINVAL;
}

static int cfe_sensor_controls(struct fake *f, unsigned long req, void *arg)
{
    if (req==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=arg;
        q->type=V4L2_CTRL_TYPE_INTEGER; q->minimum=0; q->maximum=200;
        q->step=1; q->elems=1; q->elem_size=4;
        switch (q->id) {
        case V4L2_CID_EXPOSURE:
            q->minimum=1;
            if (f->timing_plan) q->maximum=(int64_t)f->nodes[4].format.height+f->vblank-4;
            break;
        case V4L2_CID_ANALOGUE_GAIN: q->maximum=240; break;
        case V4L2_CID_VBLANK: break;
        case V4L2_CID_HBLANK: q->flags=V4L2_CTRL_FLAG_READ_ONLY; break;
        case V4L2_CID_PIXEL_RATE:
            q->type=V4L2_CTRL_TYPE_INTEGER64; q->maximum=INT64_MAX;
            q->elem_size=8; q->flags=V4L2_CTRL_FLAG_READ_ONLY; break;
        default: return -EINVAL;
        }
        return 0;
    }
    struct v4l2_ext_controls *c=arg;
    bool get=req==VIDIOC_G_EXT_CTRLS;
    CHECK(get || req==VIDIOC_S_EXT_CTRLS);
    for (uint32_t i=0;i<c->count;++i) {
        struct v4l2_ext_control *v=&c->controls[i];
        int32_t *value;
        switch (v->id) {
        case V4L2_CID_EXPOSURE: value=&f->exposure; break;
        case V4L2_CID_ANALOGUE_GAIN: value=&f->analogue_gain; break;
        case V4L2_CID_VBLANK: value=&f->vblank; break;
        case V4L2_CID_HBLANK: CHECK(get); v->value=32; continue;
        case V4L2_CID_PIXEL_RATE: CHECK(get); v->value64=100000000; continue;
        default: CHECK(false); return -EINVAL;
        }
        if (get) v->value=*value;
        else {
            if (f->timing_plan) {
                if (v->id==V4L2_CID_VBLANK) CHECK(c->count==1);
                if (v->id==V4L2_CID_EXPOSURE && v->value>(int64_t)f->nodes[4].format.height+f->vblank-4)
                    return -ERANGE;
            }
            *value=v->value;
            CHECK(f->sensor_write_count<256);
            size_t j=f->sensor_write_count++;
            f->sensor_writes[j].frame=f->event_frame;
            f->sensor_writes[j].id=v->id; f->sensor_writes[j].value=v->value;
        }
    }
    return 0;
}

static int lens_ioctl(struct fake *f, unsigned long req, void *arg)
{
    if (req==VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q=arg;
        if (q->id!=V4L2_CID_FOCUS_ABSOLUTE) return -EINVAL;
        q->type=V4L2_CTRL_TYPE_INTEGER; q->minimum=0; q->maximum=1023; q->step=1; q->elems=1; q->elem_size=4;
        return 0;
    }
    CHECK(req==VIDIOC_G_EXT_CTRLS || req==VIDIOC_S_EXT_CTRLS);
    struct v4l2_ext_controls *c=arg;
    CHECK(c->count==1 && c->controls[0].id==V4L2_CID_FOCUS_ABSOLUTE);
    if (req==VIDIOC_G_EXT_CTRLS) c->controls[0].value=f->lens_position;
    else {
        if (f->fail_lens) return -EIO;
        f->lens_position=c->controls[0].value;
    }
    return 0;
}
static uint64_t focus_score(const struct fake *f, int32_t setting)
{
    if (f->flat_focus) return 1000;
    double distance=((double)setting-445)/32-f->focus_peak;
    return (uint64_t)(100000/(1+distance*distance));
}
static int cfe_subdev_ioctl(struct fake *f, int fd, unsigned long req, void *arg)
{
    int node=fd-50;
    CHECK(node>=0 && node<4 && f->subdev_open[node]);
    if (node==3) return lens_ioctl(f,req,arg);
    if (req==CC_SD_QUERYCAP) return -ENOTTY; /* Fixed pads on the legacy driver. */
    if (req==CC_SD_G_SELECTION) {
        if (!f->physical_array_larger) return -ENOTTY;
        struct cc_subdev_selection *q=arg; CHECK(!node && !q->pad && !q->stream);
        q->r=(struct v4l2_rect){0,0,128,96};
        if (q->target==V4L2_SEL_TGT_NATIVE_SIZE || q->target==V4L2_SEL_TGT_CROP_BOUNDS) { q->r.width=256; q->r.height=192; }
        return 0;
    }
    if (req==VIDIOC_SUBDEV_ENUM_MBUS_CODE) {
        struct cc_subdev_mbus_code_enum *q=arg; CHECK(!node && !q->pad && !q->stream);
        if (q->index) return -EINVAL;
        q->code=MEDIA_BUS_FMT_SRGGB10_1X10; return 0;
    }
    if (req==VIDIOC_SUBDEV_ENUM_FRAME_SIZE) {
        struct cc_subdev_frame_size_enum *q=arg; CHECK(!node && !q->pad && !q->stream && q->code==MEDIA_BUS_FMT_SRGGB10_1X10);
        if (q->index) return -EINVAL;
        q->min_width=q->max_width=128; q->min_height=q->max_height=96; return 0;
    }
    if (req==VIDIOC_QUERY_EXT_CTRL || req==VIDIOC_G_EXT_CTRLS || req==VIDIOC_S_EXT_CTRLS) {
        CHECK(!node); return cfe_sensor_controls(f,req,arg);
    }
    CHECK(req==VIDIOC_SUBDEV_G_FMT || req==VIDIOC_SUBDEV_S_FMT);
    struct v4l2_subdev_format *v=arg;
    CHECK(v->pad<8 && v->which<=V4L2_SUBDEV_FORMAT_ACTIVE);
    if (f->embedded && node==0 && v->pad==1) {
        CHECK(req==VIDIOC_SUBDEV_G_FMT);
        v->format=(struct v4l2_mbus_framefmt){.code=0x7001,.width=1280,.height=4}; return 0;
    }
    if (req==VIDIOC_SUBDEV_S_FMT) {
        f->mbus[node][v->which][v->pad]=v->format;
        if (node==1 && !v->pad) f->mbus[node][v->which][4]=v->format;
        if (node==1 && v->pad==1) f->mbus[node][v->which][5]=v->format;
    } else v->format=f->mbus[node][v->which][v->pad];
    return 0;
}

static int32_t cfe_effective(const struct fake *f, uint32_t frame, uint32_t id, uint32_t delay, int32_t baseline_value)
{
    int32_t value=baseline_value;
    for (size_t i=0;i<f->sensor_write_count;++i)
        if (f->sensor_writes[i].id==id && frame-(f->sensor_writes[i].frame+delay)<=INT32_MAX)
            value=f->sensor_writes[i].value;
    return value;
}

static void process_cfe(struct fake *f)
{
    for (int port=4;port<=6;++port) if (!f->nodes[port].streaming) return;
    for (uint32_t i=0;i<f->nodes[4].count;++i) {
        struct buffer *image=&f->nodes[4].buffers[i], *stats=&f->nodes[5].buffers[i], *params=&f->nodes[6].buffers[i];
        if (!image->queued || image->done || !stats->queued || !params->queued) continue;
        int32_t exposure=cfe_effective(f,f->cfe_frame,V4L2_CID_EXPOSURE,2,20);
        int32_t gain=cfe_effective(f,f->cfe_frame,V4L2_CID_ANALOGUE_GAIN,1,0);
        CHECK(exposure>0 && gain>=0 && gain<256);
        /* Independent IMX219 plant: scene R:G:B = 1:2:4, before WB.
         * The histogram follows the FE's submitted RGBY weights. */
        double level=(double)exposure*256/(500.0*(256-gain));
        double luma=level*(0.5*f->fe_parameters.rgby.gain_r+f->fe_parameters.rgby.gain_g+
                           2.0*f->fe_parameters.rgby.gain_b)/1024;
        uint32_t bin=luma>=1 ? 1023U : (uint32_t)(luma*1024);
        struct pisp_statistics *s=stats->data; memset(s,0,sizeof(*s));
        s->agc.histogram[bin]=10000;
        uint64_t sum=(uint64_t)(level*65536*1000);
        CHECK(sum*2<=UINT32_MAX);
        s->awb.zones[0]=(struct pisp_awb_statistics_zone) {
            .counted=1000,.R_sum=(uint32_t)(sum/2),.G_sum=(uint32_t)sum,.B_sum=(uint32_t)(sum*2)};
        struct buffer *image_memory = memory_plane(f, 4, (int)i, 0);
        memset(image_memory->data,exposure,image->size);
        image->lens_setting=f->lens_position;
        image_memory->lens_setting=f->lens_position;
        if (f->af_lens) for (size_t j=0;j<PISP_CDAF_STATS_NUM_FOMS;++j)
            s->cdaf.foms[j]=focus_score(f,image->lens_setting)<<20;
        for (int port=4;port<=6;++port) {
            struct buffer *b=&f->nodes[port].buffers[i];
            b->timestamp=UINT64_C(123456000000)+(uint64_t)f->cfe_frame*(f->af_lens ? UINT64_C(33333334):UINT64_C(1000000));
            b->used=b->size; b->done=true;
        }
        if (f->embedded) {
            CHECK(f->nodes[8].streaming);
            struct buffer *metadata=NULL;
            for (uint32_t m=f->nodes[8].count;m>0;--m)
                if (f->nodes[8].buffers[m-1].queued && !f->nodes[8].buffers[m-1].done) { metadata=&f->nodes[8].buffers[m-1]; break; }
            CHECK(metadata && metadata->size>=5120);
            uint8_t *data=metadata->data; memset(data,0,metadata->size);
            const uint16_t addresses[]={0x0202,0x0203,0x0204,0x0205,0x0340,0x0341,0x0342,0x0343,0x013a};
            uint8_t values[]={0,(uint8_t)exposure,0,(uint8_t)gain,0,(uint8_t)(96+f->vblank),0,160,33};
            uint8_t logical[57]; size_t n=0; logical[n++]=0x0a;
            for (size_t j=0;j<9;++j) {
                logical[n++]=0xaa; logical[n++]=(uint8_t)(addresses[j]>>8); logical[n++]=0xa5;
                logical[n++]=(uint8_t)addresses[j]; logical[n++]=0x5a; logical[n++]=values[j];
            }
            logical[n++]=7; logical[n++]=7; memset(data,0x55,80);
            for (size_t j=0;j<n;++j) data[j/4*5+j%4]=logical[j];
            int phase=(int)((((double)image->lens_setting-445)/32-f->focus_peak)*50);
            if (phase < -1024) phase=-1024;
            if (phase>1023) phase=1023;
            unsigned encoded=(unsigned)(phase<0 ? phase+2048:phase);
            for (size_t j=0;j<192;++j) {
                uint8_t *p=data+2560+(j+2)*5; p[0]=16; p[1]=(uint8_t)(encoded/64); p[2]=(uint8_t)(encoded%64*4);
            }
            if (f->bad_embedded) data[2560]=1;
            metadata->timestamp=image->timestamp; metadata->sequence=f->cfe_frame+5000;
            metadata->used=f->short_embedded ? 2563U:metadata->size; metadata->done=true;
        }
        if (f->bad_cfe_statistics) stats->used=1;
        ++f->cfe_processed;
    }
}

static int controls_ioctl(struct fake *f, int node, unsigned long req, void *arg)
{
    if (node==4 && f->af_lens && (req==VIDIOC_QUERY_EXT_CTRL ?
        ((struct v4l2_query_ext_ctrl *)arg)->id==V4L2_CID_FOCUS_ABSOLUTE :
        ((struct v4l2_ext_controls *)arg)->count==1 &&
        ((struct v4l2_ext_controls *)arg)->controls[0].id==V4L2_CID_FOCUS_ABSOLUTE)) return lens_ioctl(f,req,arg);
    if (node == 4) {
        if (!f->sensor_support) return -ENOTTY;
        if (req == VIDIOC_QUERY_EXT_CTRL) {
            struct v4l2_query_ext_ctrl *q = arg;
            CHECK(q->id == V4L2_CID_EXPOSURE);
            q->type = V4L2_CTRL_TYPE_INTEGER; q->minimum = 1; q->maximum = 200;
            q->step = 1; q->elems = 1; q->elem_size = 4;
        } else {
            struct v4l2_ext_controls *controls = arg;
            CHECK(controls->count == 1 && controls->controls[0].id == V4L2_CID_EXPOSURE);
            struct v4l2_ext_control *v = controls->controls;
            if (req == VIDIOC_G_EXT_CTRLS) v->value = f->exposure;
            else {
                CHECK(req == VIDIOC_S_EXT_CTRLS && f->exposure_write_count < 16);
                if (v->value > 99) v->value = 99;
                f->exposure = v->value;
                size_t i = f->exposure_write_count++;
                f->exposure_writes[i].frame = f->event_frame;
                f->exposure_writes[i].value = v->value;
            }
        }
        return 0;
    }
    CHECK(node == 0);
    if (req == VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q = arg;
        if (q->id & V4L2_CTRL_FLAG_NEXT_CTRL) {
            uint32_t after = q->id & ~(V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND);
            if (after >= V4L2_CID_RED_BALANCE) return -EINVAL;
            q->id = V4L2_CID_RED_BALANCE;
        }
        q->type = V4L2_CTRL_TYPE_INTEGER;
        q->minimum = 0;
        q->maximum = 8000;
        q->step = 1;
        q->elems = 1;
        q->elem_size = 4;
        if (q->id == V4L2_CID_HFLIP) q->flags = V4L2_CTRL_FLAG_MODIFY_LAYOUT;
        else if (f->shading_test && q->id>=V4L2_CID_USER_BCM2835_ISP_CC_MATRIX && q->id<=V4L2_CID_USER_BCM2835_ISP_CDN) {
            const uint32_t sizes[]={sizeof(struct bcm2835_isp_custom_ccm),sizeof(struct bcm2835_isp_lens_shading),
                sizeof(struct bcm2835_isp_black_level),sizeof(struct bcm2835_isp_geq),sizeof(struct bcm2835_isp_gamma),
                sizeof(struct bcm2835_isp_denoise),sizeof(struct bcm2835_isp_sharpen),sizeof(struct bcm2835_isp_dpc),sizeof(struct bcm2835_isp_cdn)};
            q->type=V4L2_CTRL_TYPE_U8;q->flags=V4L2_CTRL_FLAG_HAS_PAYLOAD;
            q->elem_size=1;q->elems=sizes[q->id-V4L2_CID_USER_BCM2835_ISP_CC_MATRIX];
        }
        else if (q->id == PAYLOAD_CONTROL) {
            q->type = V4L2_CTRL_TYPE_U8;
            q->flags = V4L2_CTRL_FLAG_HAS_PAYLOAD;
            q->elems = 8;
            q->elem_size = 1;
        } else if (q->id != V4L2_CID_RED_BALANCE &&
                   !(f->ipa_scene && (q->id==V4L2_CID_BLUE_BALANCE || q->id==V4L2_CID_DIGITAL_GAIN))) return -EINVAL;
        return 0;
    }
    struct v4l2_ext_controls *c = arg;
    CHECK(req == VIDIOC_G_EXT_CTRLS || req == VIDIOC_S_EXT_CTRLS);
    for (uint32_t i = 0; i < c->count; ++i) {
        struct v4l2_ext_control *v = &c->controls[i];
        if (f->shading_test && v->id>=V4L2_CID_USER_BCM2835_ISP_CC_MATRIX && v->id<=V4L2_CID_USER_BCM2835_ISP_CDN) {
            CHECK(req==VIDIOC_S_EXT_CTRLS);++f->shading_controls;
            if (v->id==V4L2_CID_USER_BCM2835_ISP_GAMMA) memcpy(&f->last_gamma,v->ptr,sizeof(f->last_gamma));
            if (v->id==V4L2_CID_USER_BCM2835_ISP_LENS_SHADING) {
                memcpy(&f->last_shading,v->ptr,sizeof(f->last_shading));
                if (f->last_shading.enabled) CHECK(f->shading_start==f->shading_end && f->shading_end && f->last_shading.dmabuf==19);
            }
        } else if (v->id == PAYLOAD_CONTROL) {
            CHECK(v->size == 8);
            if (req == VIDIOC_G_EXT_CTRLS) memcpy(v->ptr, f->payload, 8);
            else memcpy(f->payload, v->ptr, 8);
        } else if (v->id == V4L2_CID_RED_BALANCE) {
            if (req == VIDIOC_G_EXT_CTRLS) v->value = f->gain;
            else f->gain = v->value;
        } else if (v->id==V4L2_CID_BLUE_BALANCE) {
            if (req==VIDIOC_G_EXT_CTRLS) v->value=f->blue_gain; else f->blue_gain=v->value;
        } else if (v->id==V4L2_CID_DIGITAL_GAIN) {
            if (req==VIDIOC_G_EXT_CTRLS) v->value=f->digital_gain; else f->digital_gain=v->value;
        }
    }
    return 0;
}

static void format_ioctl(struct fake *f, int node, struct v4l2_format *v, bool set)
{
    if (f->embedded && node==8) {
        CHECK(v->type==V4L2_BUF_TYPE_META_CAPTURE);
        if (set) f->nodes[node].format=(cc_format){.fourcc=CC_FOURCC('S','E','N','S'),.num_planes=1,.planes={{0,5120}}};
        v->fmt.meta.dataformat=f->nodes[node].format.fourcc; v->fmt.meta.buffersize=f->nodes[node].format.planes[0].size; return;
    }
    if (f->cfe && (node==5 || node==6)) {
        CHECK(v->type==(node==5 ? V4L2_BUF_TYPE_META_CAPTURE : V4L2_BUF_TYPE_META_OUTPUT));
        if (set) f->nodes[node].format=(cc_format) {
            .fourcc=node==5 ? CC_FOURCC('R','P','F','S') : CC_FOURCC('R','P','F','C'),.num_planes=1,
            .planes={{0,node==5 ? sizeof(struct pisp_statistics) : sizeof(struct pisp_fe_config)}}};
        v->fmt.meta.dataformat=f->nodes[node].format.fourcc;
        v->fmt.meta.buffersize=f->nodes[node].format.planes[0].size;
        return;
    }
    if (f->pisp && node == 2) {
        CHECK(v->type == V4L2_BUF_TYPE_META_OUTPUT);
        if (set) f->nodes[node].format = (cc_format) {
            .fourcc = CC_FOURCC('R', 'P', 'B', 'C'), .num_planes = 1,
            .planes = { {0, sizeof(struct pisp_be_tiles_config)} } };
        v->fmt.meta.dataformat = f->nodes[node].format.fourcc;
        v->fmt.meta.buffersize = f->nodes[node].format.planes[0].size;
        if (f->bad_parameters) v->fmt.meta.buffersize = 1;
        return;
    }
    if (f->pisp && (node < 2 || node == 3 || node >= 12)) {
        CHECK(v->type == (node == 0 || node == 12 || node == 14 ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE));
        struct v4l2_pix_format_mplane *p = &v->fmt.pix_mp;
        cc_format *format = &f->nodes[node].format;
        if (set) {
            ++f->format_sets[node];
            *format = (cc_format) { .fourcc = p->pixelformat, .width = p->width, .height = p->height,
                .field = p->field, .colorspace = p->colorspace, .quantization = p->quantization,
                .xfer_func = p->xfer_func, .ycbcr_enc = p->ycbcr_enc, .num_planes = p->num_planes };
            CHECK(format->num_planes > 0 && format->num_planes <= 3);
            if (f->force_color_defaults) {
                format->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(format->colorspace);
                format->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(format->colorspace == V4L2_COLORSPACE_SRGB,
                                                                    format->colorspace, format->ycbcr_enc);
                format->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(format->colorspace);
            }
            for (uint32_t i = 0; i < format->num_planes; ++i) {
                format->planes[i] = (cc_plane_format) {p->plane_fmt[i].bytesperline, p->plane_fmt[i].sizeimage};
                if (i && f->widen_chroma && format->num_planes == 3) {
                    format->planes[i].stride = format->width;
                    if (f->refuse_capacity) {
                        uint32_t divisor = format->fourcc == V4L2_PIX_FMT_YUV420M || format->fourcc == V4L2_PIX_FMT_YVU420M ? 4U : 2U;
                        format->planes[i].size = format->width * format->height / divisor;
                    }
                }
            }
        }
        p->width = format->width + (!node && f->mismatch_input ? 2U : 0U);
        p->height = format->height;
        p->pixelformat = format->fourcc;
        p->field = format->field;
        p->colorspace = format->colorspace;
        p->quantization = (uint8_t)format->quantization;
        p->ycbcr_enc = (uint8_t)format->ycbcr_enc;
        p->xfer_func = (uint8_t)format->xfer_func;
        p->num_planes = (uint8_t)format->num_planes;
        for (uint32_t i = 0; i < format->num_planes; ++i) {
            p->plane_fmt[i].bytesperline = format->planes[i].stride;
            p->plane_fmt[i].sizeimage = format->planes[i].size;
        }
        return;
    }
    if (node == 3) {
        CHECK(v->type == V4L2_BUF_TYPE_META_CAPTURE);
        if (set) f->nodes[node].format = (cc_format) {
            .fourcc = CC_FOURCC('B', 'S', 'T', 'A'), .num_planes = 1,
            .planes = { {0, f->ipa_scene ? (uint32_t)sizeof(struct bcm2835_isp_stats) : 256U} } };
        v->fmt.meta.dataformat = f->nodes[node].format.fourcc;
        v->fmt.meta.buffersize = f->nodes[node].format.planes[0].size;
        return;
    }
    CHECK(v->type == (node == 0 ? V4L2_BUF_TYPE_VIDEO_OUTPUT : V4L2_BUF_TYPE_VIDEO_CAPTURE));
    struct v4l2_pix_format *p = &v->fmt.pix;
    cc_format *format = &f->nodes[node].format;
    if (set) {
        uint32_t w = p->width < 64 ? 64 : p->width, h = p->height < 64 ? 64 : p->height;
        uint32_t stride = p->bytesperline;
        if (!stride) stride = f->unicam && node==7 ? w*5/4:p->pixelformat == V4L2_PIX_FMT_RGB24 ? w * 3 : w;
        uint32_t size = p->sizeimage ? p->sizeimage : stride * h;
        if (!p->sizeimage && p->pixelformat == V4L2_PIX_FMT_YUV420) size = size * 3 / 2;
        *format = (cc_format) { .fourcc = p->pixelformat, .width = w, .height = h,
            .field = V4L2_FIELD_NONE, .colorspace = node == 0 || node == 4 || (f->unicam && node==7) ? V4L2_COLORSPACE_RAW : V4L2_COLORSPACE_SRGB,
            .num_planes = 1, .planes = { {stride, size} } };
    }
    p->width = format->width + (node == 0 && f->mismatch_input ? 2U : 0U);
    p->height = format->height;
    p->pixelformat = format->fourcc;
    p->bytesperline = format->planes[0].stride;
    p->sizeimage = format->planes[0].size;
    p->field = format->field;
    p->colorspace = format->colorspace;
}

static void process(struct fake *f)
{
    if (f->cfe) process_cfe(f);
    uint32_t mask = 15;
    if (f->pisp) {
        const struct pisp_be_config *c = &f->last_parameters.config;
        mask = c->global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1 ? 15U : 7U;
        const uint32_t bits[4] = {PISP_BE_BAYER_ENABLE_TDN_INPUT,PISP_BE_BAYER_ENABLE_TDN_OUTPUT,
            PISP_BE_BAYER_ENABLE_STITCH_INPUT,PISP_BE_BAYER_ENABLE_STITCH_OUTPUT};
        for (uint32_t i=0;i<4;++i) if (c->global.bayer_enables & bits[i]) mask |= 1U << (12U+i);
    }
    for (int port = 0; port < NODES; ++port)
        if ((mask & (1U << port)) && !f->nodes[port].streaming) return;
    for (uint32_t i = 0; i < f->nodes[0].count; ++i) {
        struct buffer *in = &f->nodes[0].buffers[i];
        if (!in->queued || in->done) continue;
        for (int port = 1; port < NODES; ++port)
            if ((mask & (1U << port)) && (i >= f->nodes[port].count || !f->nodes[port].buffers[i].queued)) return;
        CHECK(f->fds[in->input_fd].open);
        struct buffer *raw = &f->nodes[f->fds[in->input_fd].node].buffers[f->fds[in->input_fd].slot];
        f->last_input = raw->data;
        if (f->sensor_support && !f->cfe) {
            CHECK(f->processed_count < 16);
            f->processed_exposures[f->processed_count++] = *(const unsigned char *)raw->data;
        }
        for (int port = 1; port < NODES; ++port) {
            if (!(mask & (1U << port))) continue;
            struct buffer *out = &f->nodes[port].buffers[i];
            if (!f->pisp || (port != 2 && port != 12 && port != 14))
                for (uint32_t p = 0; p < f->nodes[port].format.num_planes; ++p) {
                    struct buffer *plane = memory_plane(f, port, (int)i, (int)p);
                    uint32_t written = f->nodes[port].format.planes[p].size;
                    memset(plane->data, 0x30 + port + (int)p, written);
                    if (f->sensor_support) *(unsigned char *)plane->data = *(const unsigned char *)raw->data;
                    buffer_plane(&f->nodes[port], (int)i, (int)p)->used = written;
                }
            out->timestamp = in->timestamp;
            if (!f->pisp && port==3 && f->ipa_scene) {
                struct bcm2835_isp_stats *stats=out->data;
                memset(stats,0,sizeof(*stats)); stats->size=sizeof(*stats);
                stats->hist[0].g_hist[25]=1000;
                stats->awb_stats[0]=(struct bcm2835_isp_stats_region) {
                    .counted=10,.r_sum=8192,.g_sum=16384,.b_sum=32768};
                if (f->af_lens) for (size_t j=0;j<FOCUS_REGIONS;++j)
                    stats->focus_stats[j].contrast_val[1][1]=focus_score(f,raw->lens_setting)*1000;
            }
            out->used = out->size;
            out->done = true;
        }
        in->done = true;
        f->dma_pending = false;
    }
}

static int legacy_media_adapter(void *ctx,int fd,unsigned long req,void *arg)
{
    struct fake *f=ctx; return fd==10 ? media_ioctl(f,req,arg):cfe_media_ioctl(f,req,arg);
}
static int ioctl_fd(void *ctx, int fd, unsigned long req, void *arg)
{
    struct fake *f = ctx;
    if (f->validation_only) {
        CHECK(req!=VIDIOC_S_FMT && req!=VIDIOC_S_SELECTION && req!=MEDIA_IOC_SETUP_LINK &&
              req!=VIDIOC_REQBUFS && req!=VIDIOC_STREAMON && req!=VIDIOC_STREAMOFF &&
              req!=VIDIOC_S_EXT_CTRLS && req!=VIDIOC_S_CTRL);
        if (req==VIDIOC_SUBDEV_S_FMT) CHECK(((struct v4l2_subdev_format *)arg)->which==V4L2_SUBDEV_FORMAT_TRY);
    }
    if (f->shading_test && fd==18) {
        CHECK(req==DMA_HEAP_IOCTL_ALLOC && f->heap_open && !f->shading_open);
        struct dma_heap_allocation_data *a=arg;CHECK(a->len==64U*49U*8U);
        a->fd=19;f->shading_open=true;++f->handles;return 0;
    }
    if (f->shading_test && fd==19) {
        CHECK(req==DMA_BUF_IOCTL_SYNC && f->shading_open);
        if (f->fail_sync) return -EIO;
        struct dma_buf_sync *sync=arg;
        if (sync->flags & DMA_BUF_SYNC_END) ++f->shading_end;else ++f->shading_start;
        return 0;
    }
    if (req == CC_SYNC_FILE_INFO) {
        CHECK(fd >= 100 && fd < f->next_fd && f->fds[fd].open && f->fds[fd].fence);
        struct cc_sync_file_info *info = arg;
        CHECK(!info->status && !info->flags && !info->pad && !info->sync_fence_info && !info->num_fences);
        ++f->fence_queries;
        if (f->fence_query_error) return f->fence_query_error;
        int object = f->fds[fd].fence - 1;
        if (f->fences[object].signal_at && f->now >= f->fences[object].signal_at) f->fences[object].status = 1;
        info->status = f->fences[object].status; info->num_fences = UINT32_MAX;
        uint32_t bad = f->fences[object].malformed;
        if (bad & 1U) info->status = 2;
        if (bad & 2U) info->flags = 1;
        if (bad & 4U) info->pad = 1;
        if (bad & 8U) info->sync_fence_info = 1;
        return 0;
    }
    int node = fd - 20;
    if (req == f->fail_req && (f->fail_node < 0 || f->fail_node == node) &&
        f->fail_nth > 0 && --f->fail_nth == 0) return -f->fail_errno;
    if (use_v2_topology && (fd==10 || (f->cfe && fd==11))) {
        if (req==MEDIA_IOC_DEVICE_INFO) {
            int ret=legacy_media_adapter(ctx,fd,req,arg);
            ((struct media_device_info *)arg)->media_version=UINT32_C(0x060c00); return ret;
        }
        if (req==MEDIA_IOC_G_TOPOLOGY) return test_v2_topology(ctx,fd,req,arg,legacy_media_adapter,f->cfe && fd==11 && f->af_lens);
        CHECK(req!=MEDIA_IOC_ENUM_ENTITIES && req!=MEDIA_IOC_ENUM_LINKS);
    }
    if (fd == 10) return media_ioctl(f, req, arg);
    if (f->cfe && fd==11) return cfe_media_ioctl(f,req,arg);
    if (f->cfe && fd>=50 && fd<54) return cfe_subdev_ioctl(f,fd,req,arg);
    CHECK(node >= 0 && node < NODES && f->nodes[node].open);
    struct node *n = &f->nodes[node];
    if (req == VIDIOC_SUBSCRIBE_EVENT || req == VIDIOC_UNSUBSCRIBE_EVENT || req == VIDIOC_DQEVENT) {
        CHECK(node == 4 && (f->sensor_support || f->cfe));
        if (req == VIDIOC_DQEVENT) {
            if (!f->event_pending) return -EAGAIN;
            struct v4l2_event *event = arg;
            *event = (struct v4l2_event) {.type=V4L2_EVENT_FRAME_SYNC,.sequence=f->event_serial++};
            event->u.frame_sync.frame_sequence = f->event_frame;
            f->event_pending = false;
        } else {
            f->event_subscribed = req == VIDIOC_SUBSCRIBE_EVENT;
            f->event_pending = false;
        }
        return 0;
    }
    if (req == VIDIOC_QUERYCAP) {
        struct v4l2_capability *caps = arg;
        caps->capabilities = V4L2_CAP_DEVICE_CAPS;
        caps->device_caps = V4L2_CAP_STREAMING | (node == 0 ? V4L2_CAP_VIDEO_OUTPUT :
            node == 3 ? V4L2_CAP_META_CAPTURE : V4L2_CAP_VIDEO_CAPTURE);
        if (f->pisp && (node < 4 || node >= 12))
            caps->device_caps = V4L2_CAP_STREAMING | (node == 2 ? V4L2_CAP_META_OUTPUT :
                node == 0 || node == 12 || node == 14 ? V4L2_CAP_VIDEO_OUTPUT_MPLANE : V4L2_CAP_VIDEO_CAPTURE_MPLANE);
        if (f->cfe && node>=4 && node<12) caps->device_caps=V4L2_CAP_STREAMING |
            (node==5 || node==8 ? V4L2_CAP_META_CAPTURE : node==6 ? V4L2_CAP_META_OUTPUT : V4L2_CAP_VIDEO_CAPTURE);
        return 0;
    }
    if (req == VIDIOC_QUERY_EXT_CTRL || req == VIDIOC_G_EXT_CTRLS || req == VIDIOC_S_EXT_CTRLS)
        return controls_ioctl(f, node, req, arg);
    if (req==VIDIOC_TRY_FMT) {
        cc_format saved=n->format; int sets=f->format_sets[node];
        format_ioctl(f,node,arg,true);
        n->format=saved; f->format_sets[node]=sets; ++f->try_formats; return 0;
    }
    if (req == VIDIOC_S_FMT || req == VIDIOC_G_FMT) { format_ioctl(f, node, arg, req == VIDIOC_S_FMT); return 0; }
    if (req == VIDIOC_G_SELECTION || req == VIDIOC_S_SELECTION) {
        struct v4l2_selection *s = arg;
        CHECK(node == 0 && s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT && s->target == V4L2_SEL_TGT_CROP);
        if (req == VIDIOC_S_SELECTION) {
            CHECK(s->r.width && s->r.height);
            f->crop = (cc_rectangle) { s->r.left, s->r.top, s->r.width & ~1U, s->r.height & ~1U };
            if (f->selection_changes_layout) ++n->format.planes[0].stride;
        }
        s->r = (struct v4l2_rect) { f->crop.x, f->crop.y, f->crop.width, f->crop.height };
        if (f->bad_crop) s->r.width = 0;
        return 0;
    }
    if (req == VIDIOC_REQBUFS) {
        struct v4l2_requestbuffers *r = arg;
        CHECK(r->memory == V4L2_MEMORY_MMAP || r->memory == V4L2_MEMORY_DMABUF);
        if (!node) CHECK(r->memory == V4L2_MEMORY_DMABUF);
        if ((!f->pisp && node == 3) || (f->pisp && node == 2) || (f->cfe && (node == 5 || node == 6 || node == 8)))
            CHECK(r->memory == V4L2_MEMORY_MMAP);
        if (!r->count) {
            CHECK(!n->streaming);
            for (uint32_t i = 0; i < n->count; ++i) for (uint32_t p = 0; p < n->format.num_planes; ++p) {
                struct buffer *b = buffer_plane(n, (int)i, (int)p);
                CHECK(!b->maps);
                if (b->kernel && (b->refs || b->attachments)) return -EBUSY;
            }
            for (uint32_t i = 0; i < n->count; ++i) {
                for (uint32_t p = 0; p < n->format.num_planes; ++p) detach(f, buffer_plane(n, (int)i, (int)p));
                for (uint32_t p = 0; p < n->format.num_planes; ++p) {
                    struct buffer *b = buffer_plane(n, (int)i, (int)p);
                    b->kernel = false;
                    maybe_free(b);
                }
            }
            n->count = 0;
        } else {
            CHECK(!n->count);
            n->count = r->count = f->counts[node];
            n->memory = r->memory;
            for (uint32_t i = 0; i < n->count; ++i) for (uint32_t p = 0; p < n->format.num_planes; ++p) {
                struct buffer *b = buffer_plane(n, (int)i, (int)p);
                CHECK(!b->data && !b->refs && !b->maps);
                memset(b, 0, sizeof(*b));
                b->size = n->format.planes[p].size;
                if (n->memory == V4L2_MEMORY_MMAP) {
                    b->kernel = true;
                    b->data = calloc(1, b->size);
                    CHECK(b->data);
                }
            }
        }
        return 0;
    }
    if (req == VIDIOC_STREAMON) {
        n->streaming = true; ++f->starts;
        if (node == 4 && f->event_subscribed) {
            f->event_frame = f->cfe ? f->cfe_frame : f->sensor_sequence; f->event_pending = true;
        }
        return 0;
    }
    if (req == VIDIOC_STREAMOFF) {
        if (node == 0 && f->fail_stop) return -EIO;
        n->streaming = false;
        if (node == 4 && f->sensor_support) f->sensor_sequence = 0;
        if (node == 4 && f->cfe) { f->cfe_next_frame=0; f->event_pending=false; }
        for (uint32_t i = 0; i < n->count; ++i) n->buffers[i].queued = n->buffers[i].done = false;
        return 0;
    }
    if (req == VIDIOC_EXPBUF) {
        struct v4l2_exportbuffer *e = arg;
        CHECK(e->index < n->count && e->plane < n->format.num_planes && e->flags == O_CLOEXEC);
        e->fd = descriptor(f, node, (int)e->index, (int)e->plane);
        if (f->fresh_exports) f->fds[e->fd].identity = f->export_identity++;
        return 0;
    }
    struct v4l2_buffer *v = arg;
    CHECK(v->memory == n->memory);
    bool multiplanar = f->pisp && (node < 2 || node == 3 || node >= 12);
    if (req == VIDIOC_QUERYBUF) {
        CHECK(v->index < n->count && n->memory == V4L2_MEMORY_MMAP);
        if (multiplanar) {
            v->length = n->format.num_planes;
            for (uint32_t p = 0; p < n->format.num_planes; ++p) {
                v->m.planes[p].length = buffer_plane(n, (int)v->index, (int)p)->size;
                v->m.planes[p].m.mem_offset = (v->index * 3 + p) * 65536;
            }
        } else {
            v->length = n->buffers[v->index].size;
            v->m.offset = v->index * 65536;
        }
        return 0;
    }
    if (req == VIDIOC_QBUF) {
        CHECK(v->index < n->count && !n->buffers[v->index].queued);
        struct buffer *b = &n->buffers[v->index];
        b->queued = true;
        b->done = false;
        b->used = multiplanar ? v->m.planes[0].bytesused : v->bytesused;
        b->timestamp = (uint64_t)v->timestamp.tv_sec * UINT64_C(1000000000) + (uint64_t)v->timestamp.tv_usec * 1000;
        if (node == 4)
            for (uint32_t i = 0; i < f->nodes[0].count; ++i)
                if (f->nodes[0].buffers[i].queued) ++f->capture_overlap;
        if (node == 4 && !f->cfe && n->streaming && f->event_subscribed) {
            CHECK(!f->event_pending);
            f->event_frame = f->sensor_sequence; f->event_pending = true;
        }
        if (n->memory == V4L2_MEMORY_DMABUF) for (uint32_t p = 0; p < n->format.num_planes; ++p) {
            int source_fd = multiplanar ? v->m.planes[p].m.fd : v->m.fd;
            CHECK(v->flags == 0 && source_fd >= 100 && f->fds[source_fd].open);
            struct buffer *plane = buffer_plane(n, (int)v->index, (int)p);
            detach(f, plane);
            plane->input_fd = source_fd;
            plane->attached_node = f->fds[source_fd].node;
            plane->attached_slot = f->fds[source_fd].slot;
            plane->attached_plane = f->fds[source_fd].plane;
            struct buffer *memory = buffer_plane(&f->nodes[plane->attached_node], plane->attached_slot, plane->attached_plane);
            CHECK(memory->data && memory->size >= n->format.planes[p].size);
            plane->attached = true; ++memory->attachments;
        }
        if (!node) {
            f->submitted_gain[v->index] = f->gain;
            memcpy(f->submitted_payload[v->index], f->payload, 8);
            if (f->pisp) f->dma_pending = true;
        } else if (f->cfe && node==6) {
            CHECK(b->used==sizeof(struct pisp_fe_config));
            memcpy(&f->fe_parameters,b->data,sizeof(f->fe_parameters));
            f->cfe_frame=f->cfe_next_frame++;
            if (n->streaming && f->event_subscribed) {
                CHECK(!f->event_pending); f->event_frame=f->cfe_frame; f->event_pending=true;
            }
        } else if (f->pisp && node == 2) {
            CHECK(b->used == sizeof(struct pisp_be_tiles_config));
            memcpy(&f->last_parameters, b->data, sizeof(f->last_parameters));
            CHECK(f->last_parameters.num_tiles > 0 && f->last_parameters.num_tiles <= 64);
            CHECK(f->last_parameters.config.input_format.width == 128);
            CHECK(f->last_parameters.config.output_format[0].image.stride == (int32_t)f->nodes[1].format.planes[0].stride);
            CHECK(!f->last_parameters.config.input_buffer.addr[0][0]);
        } else if (f->pisp && (node == 12 || node == 14)) {
            CHECK(b->used >= n->format.planes[0].size);
        } else {
            CHECK(!b->used);
            if (multiplanar)
                for (uint32_t p = 1; p < n->format.num_planes; ++p) CHECK(!v->m.planes[p].bytesused);
        }
        return 0;
    }
    CHECK(req == VIDIOC_DQBUF && n->streaming);
    process(f);
    if (node == f->delayed_port) return -EAGAIN;
    for (uint32_t i = 0; i < n->count; ++i) {
        struct buffer *b = &n->buffers[i];
        bool producer=(node==4 && !f->cfe) || (f->unicam && node==7);
        if (!b->queued || (!producer && !b->done)) continue;
        b->queued = false;
        if (producer) {
            memset(memory_plane(f, node, (int)i, 0)->data, 0x77, b->size); b->used = b->size; b->timestamp = UINT64_C(123456789000);
            b->lens_setting=f->lens_position;
        }
        v->index = i;
        v->bytesused = b->used;
        if (node == f->corrupt_port) v->bytesused = b->size + 1;
        if (multiplanar) {
            v->length = n->format.num_planes;
            for (uint32_t p = 0; p < n->format.num_planes; ++p) {
                v->m.planes[p].bytesused = p ? buffer_plane(n, (int)i, (int)p)->used : v->bytesused;
                v->m.planes[p].data_offset = 0;
            }
        }
        v->sequence = 8;
        if (f->cfe && node>=4 && node<=6) v->sequence=f->cfe_frame;
        if (f->embedded && node==8) v->sequence=b->sequence;
        if (node == 4 && f->sensor_support && !f->cfe) {
            v->sequence = f->sensor_sequence++;
            int32_t effective = 20;
            for (size_t j = 0; j < f->exposure_write_count; ++j)
                if (v->sequence - (f->exposure_writes[j].frame + 2U) <= INT32_MAX)
                    effective = f->exposure_writes[j].value;
            *(unsigned char *)memory_plane(f, node, (int)i, 0)->data = (unsigned char)effective;
        }
        v->timestamp.tv_sec = (long)(b->timestamp / UINT64_C(1000000000));
        v->timestamp.tv_usec = (long)((b->timestamp % UINT64_C(1000000000)) / 1000);
        v->flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
        return 0;
    }
    return -EAGAIN;
}

static const struct cc_io_ops ops = { .open = open_fd, .close = close_fd, .ioctl = ioctl_fd,
    .map = map, .unmap = unmap, .lock = lock_fd, .now = now, .wait = wait_fd,
    .duplicate = duplicate, .resolve = resolve, .wait_many = wait_many, .identity = identity };

static cc_format raw_format(void)
{
    return (cc_format) { .fourcc = V4L2_PIX_FMT_SRGGB10P, .width = 128, .height = 96,
        .num_planes = 1, .planes = { {160, 15360} } };
}
static cc_vc4_isp *open_isp(struct fake *f)
{
    const struct cc_io io = { &ops, f };
    cc_vc4_isp *isp = NULL;
    OK(cc_vc4_isp_open_io("/dev/media0", &io, &isp));
    return isp;
}
static void configure(cc_vc4_isp *isp)
{
    cc_vc4_configuration c = { .input = raw_format(),
        .output = { { .fourcc = V4L2_PIX_FMT_RGB24, .width = 64, .height = 64 } } };
    OK(cc_vc4_isp_configure(isp, &c));
    CHECK(c.crop.x == 16 && !c.crop.y && c.crop.width == 96 && c.crop.height == 96);
    CHECK(c.output[1].fourcc == V4L2_PIX_FMT_YUV420 &&
          (c.statistics.planes[0].size == 256 || c.statistics.planes[0].size == sizeof(struct bcm2835_isp_stats)));
}
static cc_camera *producer(struct fake *f, cc_frame *frame, cc_dmabuf_plane *plane)
{
    const struct cc_io io = { &ops, f };
    cc_camera_info info = { .pipeline = CC_PIPELINE_V4L2, .video_path = "/dev/video4" };
    cc_camera *camera = NULL;
    OK(cc_camera_open_io(&info, &io, &camera));
    cc_format format = raw_format();
    if (f->pisp) {
        format.fourcc = V4L2_PIX_FMT_SRGGB16;
        format.planes[0].stride = 256;
        format.planes[0].size = 256 * format.height;
    }
    OK(cc_camera_configure(camera, &format));
    uint32_t count = 2;
    OK(cc_camera_allocate_buffers(camera, &count));
    OK(cc_camera_queue(camera, 0));
    OK(cc_camera_start(camera));
    OK(cc_camera_dequeue(camera, frame, 0));
    plane->fd = cc_camera_export_plane(camera, 0, 0);
    CHECK(plane->fd >= 0);
    plane->length = (uint32_t)frame->planes[0].length;
    return camera;
}
static void allocated_isp(cc_vc4_isp *isp)
{
    configure(isp);
    uint32_t count = 3;
    OK(cc_vc4_isp_allocate_buffers(isp, &count));
    CHECK(count == 2); /* Counts differ by port, so expose their common minimum. */
}
static void finished(struct fake *f)
{
    for (size_t i = 0; i < 16; ++i) CHECK(!f->fences[i].refs);
    CHECK(!f->handles && !f->mappings && !f->media_open && !f->cfe_media_open && !f->event_subscribed);
    for (size_t i=0;i<4;++i) CHECK(!f->subdev_open[i]);
    for (int n = 0; n < NODES; ++n) for (int i = 0; i < SLOTS; ++i) for (int p = 0; p < 3; ++p) {
        struct buffer *b = buffer_plane(&f->nodes[n], i, p);
        CHECK(!b->data && !b->kernel && !b->maps && !b->refs && !b->attachments && !b->attached);
    }
}

static void test_processing(void)
{
    struct fake f;
    init(&f);
    cc_vc4_isp *isp = open_isp(&f);
    allocated_isp(isp);
    cc_frame raw;
    cc_dmabuf_plane plane;
    cc_camera *camera = producer(&f, &raw, &plane);
    OK(cc_vc4_isp_submit(isp, 1, &plane, 1, &raw, 1234));
    struct node live_nodes[NODES]; memcpy(live_nodes,f.nodes,sizeof(live_nodes));
    cc_vc4_configuration alternative={.input=raw_format(),
        .output={{.fourcc=V4L2_PIX_FMT_YUV420,.width=96,.height=80}}};
    f.validation_only=true; OK(cc_vc4_isp_try_configuration(isp,&alternative)); f.validation_only=false;
    CHECK(!memcmp(live_nodes,f.nodes,sizeof(live_nodes)));
    OK(close_fd(&f, plane.fd));
    ERROR(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 0), EBUSY);
    f.delayed_port = CC_VC4_STATS;
    cc_vc4_result result = { .cookie = 5678 };
    ERROR(cc_vc4_isp_dequeue(isp, &result, 0), EAGAIN);
    CHECK(result.cookie == 5678 && f.last_input == raw.planes[0].data);
    ERROR(cc_vc4_isp_dequeue(isp, &result, 3), ETIMEDOUT);
    CHECK(f.wait_calls == 3 && result.cookie == 5678 && f.nodes[4].buffers[0].refs == 1);
    f.delayed_port = -1;
    OK(cc_vc4_isp_dequeue(isp, &result, 0));
    CHECK(result.index == 1 && result.cookie == 1234 && !result.input.planes[0].data);
    CHECK(result.input.timestamp_ns == raw.timestamp_ns && result.output[0].timestamp_ns == raw.timestamp_ns);
    CHECK(*(unsigned char *)result.output[0].planes[0].data == 0x31);
    CHECK(*(unsigned char *)result.output[1].planes[0].data == 0x32);
    CHECK(*(unsigned char *)result.statistics.planes[0].data == 0x33);
    CHECK(f.nodes[4].buffers[0].refs == 0); /* Input refs released on complete result. */
    CHECK(f.nodes[4].buffers[0].attachments == 1); /* vb2 keeps the last imported attachment. */
    OK(cc_camera_queue(camera, raw.index));
    OK(cc_camera_dequeue(camera, &raw, 0));
    plane.fd = cc_camera_export_plane(camera, raw.index, 0);
    CHECK(plane.fd >= 0);
    ERROR(cc_vc4_isp_submit(isp, 1, &plane, 1, &raw, 0), EBUSY); /* Held result. */
    OK(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 2));
    OK(close_fd(&f, plane.fd));
    CHECK(f.starts == 5); /* Four ISP ports started once, plus the producer. */
    int exported = cc_vc4_isp_export_plane(isp, CC_VC4_MAIN, 1, 0);
    CHECK(exported >= 0);
    OK(cc_vc4_isp_release(isp, 1));
    ERROR(cc_vc4_isp_release(isp, 1), EINVAL);
    OK(cc_vc4_isp_dequeue(isp, &result, 100));
    CHECK(result.cookie == 2 && result.index == 0);
    OK(cc_vc4_isp_release(isp, 0));
    ERROR(cc_camera_free_buffers(camera), EBUSY); /* No DMA, but vb2 still references the allocation. */
    ERROR(cc_vc4_isp_free_buffers(isp), EBUSY);
    OK(cc_camera_free_buffers(camera)); /* Input queue has been freed even though an output is exported. */
    OK(close_fd(&f, exported));
    OK(cc_vc4_isp_free_buffers(isp));
    cc_vc4_isp_close(isp);
    cc_camera_close(camera);
    finished(&f);
}

static void test_failures(void)
{
    const unsigned long requests[] = { VIDIOC_QBUF, VIDIOC_STREAMON, VIDIOC_DQBUF };
    for (size_t r = 0; r < sizeof(requests) / sizeof(requests[0]); ++r) {
        for (int node = 0; node < 4; ++node) {
            struct fake f;
            init(&f);
            cc_vc4_isp *isp = open_isp(&f);
            allocated_isp(isp);
            cc_frame raw;
            cc_dmabuf_plane plane;
            cc_camera *camera = producer(&f, &raw, &plane);
            f.fail_req = requests[r]; f.fail_node = node; f.fail_nth = 1; f.fail_errno = EIO;
            if (requests[r] == VIDIOC_DQBUF) {
                OK(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 0));
                OK(close_fd(&f, plane.fd));
                cc_vc4_result result;
                ERROR(cc_vc4_isp_dequeue(isp, &result, 0), EIO);
            } else {
                ERROR(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 0), EIO);
                OK(close_fd(&f, plane.fd));
            }
            CHECK(f.nodes[4].buffers[0].refs == 0);
            for (int i = 0; i < 4; ++i) CHECK(!f.nodes[i].streaming);
            plane.fd = cc_camera_export_plane(camera, 0, 0);
            CHECK(plane.fd >= 0);
            OK(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 0)); /* Recoverable cancellation. */
            OK(close_fd(&f, plane.fd));
            f.fail_stop = true;
            ERROR(cc_vc4_isp_stop(isp), EIO);
            CHECK(f.nodes[4].buffers[0].refs == 0);
            for (int i = 0; i < 4; ++i) CHECK(!f.nodes[i].open);
            ERROR(cc_vc4_isp_fd(isp, CC_VC4_MAIN), ENODEV);
            cc_vc4_isp_close(isp);
            cc_camera_close(camera);
            finished(&f);
        }
    }
    for (int fail = 1; fail < 7; ++fail) {
        struct fake f;
        init(&f);
        cc_vc4_isp *isp = open_isp(&f);
        configure(isp);
        f.fail_map = fail;
        uint32_t count = 3;
        ERROR(cc_vc4_isp_allocate_buffers(isp, &count), ENOMEM);
        CHECK(!f.mappings);
        cc_vc4_isp_close(isp);
        finished(&f);
    }
}

static void test_controls_and_selection(void)
{
    struct fake f;
    init(&f);
    cc_vc4_isp *isp = open_isp(&f);
    configure(isp);
    cc_control_info info;
    OK(cc_vc4_isp_next_control(isp, 0, &info));
    CHECK(info.id == V4L2_CID_RED_BALANCE);
    ERROR(cc_vc4_isp_next_control(isp, info.id, &info), ENOENT);
    cc_control control = { .id = V4L2_CID_RED_BALANCE, .value = 1500 };
    OK(cc_vc4_isp_set_controls(isp, &control, 1, NULL));
    control.value = 0;
    OK(cc_vc4_isp_get_controls(isp, &control, 1, NULL));
    CHECK(control.value == 1500);
    unsigned char data[8] = "matrix";
    control = (cc_control) { .id = PAYLOAD_CONTROL, .data = data, .size = sizeof(data) };
    OK(cc_vc4_isp_set_controls(isp, &control, 1, NULL));
    memset(data, 0, sizeof(data));
    OK(cc_vc4_isp_get_controls(isp, &control, 1, NULL));
    CHECK(!memcmp(data, "matrix", 7));
    allocated_isp(isp);
    control = (cc_control) { .id = V4L2_CID_HFLIP, .value = 1 };
    ERROR(cc_vc4_isp_set_controls(isp, &control, 1, NULL), EBUSY);
    OK(cc_vc4_isp_free_buffers(isp));
    OK(cc_vc4_isp_set_controls(isp, &control, 1, NULL));
    uint32_t count = 2;
    ERROR(cc_vc4_isp_allocate_buffers(isp, &count), EPIPE);
    configure(isp);
    f.mismatch_input = true;
    cc_vc4_configuration c = { .input = raw_format(),
        .output = { { .fourcc = V4L2_PIX_FMT_RGB24, .width = 64, .height = 64 } } };
    ERROR(cc_vc4_isp_configure(isp, &c), EPIPE);
    f.mismatch_input = false;
    f.selection_changes_layout = true;
    ERROR(cc_vc4_isp_configure(isp, &c), EPIPE);
    f.selection_changes_layout = false;
    f.bad_crop = true;
    ERROR(cc_vc4_isp_configure(isp, &c), EPROTO);
    cc_vc4_isp_close(isp);
    finished(&f);
}

static void test_open_and_protocol_errors(void)
{
    for (int fault = 0; fault < 4; ++fault) {
        struct fake f;
        init(&f);
        if (fault == 0) f.wrong_driver = true;
        if (fault == 1) f.missing_port = 3;
        if (fault == 2) { f.fail_req = MEDIA_IOC_ENUM_ENTITIES; f.fail_nth = 2; f.fail_errno = EIO; }
        if (fault == 3) { f.fail_req = VIDIOC_QUERYCAP; f.fail_node = 2; f.fail_nth = 1; f.fail_errno = EIO; }
        struct cc_io io = { &ops, &f };
        cc_vc4_isp *isp = (cc_vc4_isp *)(uintptr_t)1;
        CHECK(cc_vc4_isp_open_io("/dev/media0", &io, &isp) < 0 && !isp);
        finished(&f);
    }
    struct fake f;
    init(&f);
    cc_vc4_isp *isp = open_isp(&f);
    allocated_isp(isp);
    cc_frame raw;
    cc_dmabuf_plane plane;
    cc_camera *camera = producer(&f, &raw, &plane);
    f.fail_duplicate = f.duplicates + 1;
    ERROR(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 0), EMFILE);
    CHECK(f.starts == 1); /* Only producer started; no ISP buffers queued. */
    cc_vc4_result result = { .cookie = 9999 };
    ERROR(cc_vc4_isp_dequeue(isp, &result, 0), EAGAIN);
    OK(cc_vc4_isp_submit(isp, 0, &plane, 1, &raw, 0));
    OK(close_fd(&f, plane.fd));
    f.corrupt_port = CC_VC4_STATS;
    ERROR(cc_vc4_isp_dequeue(isp, &result, 0), EPROTO);
    CHECK(result.cookie == 9999 && !f.nodes[4].buffers[0].refs);
    for (int i = 0; i < 4; ++i) CHECK(!f.nodes[i].streaming);
    cc_vc4_isp_close(isp);
    cc_camera_close(camera);
    finished(&f);
}

static void test_selection_layout_change(void)
{
    struct fake f;
    init(&f);
    const struct cc_io io = { &ops, &f };
    cc_video_device *device = NULL;
    OK(cc_video_open_io("/dev/video0", &io, &device));
    cc_stream *stream = NULL;
    OK(cc_video_create_stream(device, CC_STREAM_OUTPUT, &stream));
    cc_format format = raw_format();
    OK(cc_stream_configure(stream, &format));
    cc_rectangle rectangle = { 0, 0, 64, 64 };
    OK(cc_stream_set_selection(stream, V4L2_SEL_TGT_CROP, 0, &rectangle));
    rectangle = (cc_rectangle) {0};
    OK(cc_stream_get_selection(stream, V4L2_SEL_TGT_CROP, &rectangle));
    CHECK(rectangle.width == 64 && rectangle.height == 64);
    uint32_t count = 3;
    OK(cc_stream_allocate_buffers(stream, CC_MEMORY_DMABUF, &count));
    f.selection_changes_layout = true;
    ERROR(cc_stream_set_selection(stream, V4L2_SEL_TGT_CROP, 0, &rectangle), EPIPE);
    cc_frame frame;
    ERROR(cc_stream_buffer(stream, 0, &frame), EPIPE);
    ERROR(cc_stream_configure(stream, &format), EBUSY);
    OK(cc_stream_free_buffers(stream));
    f.selection_changes_layout = false;
    OK(cc_stream_configure(stream, &format));
    cc_stream_close(stream);
    cc_video_close(device);
    finished(&f);
}

static cc_pisp_be *open_be(struct fake *f, bool allocate)
{
    const struct cc_io io = { &ops, f };
    cc_pisp_be *be = NULL;
    OK(cc_pisp_be_open_io("/dev/media0", &io, &be));
    if (allocate) {
        cc_format input = raw_format();
        input.fourcc = V4L2_PIX_FMT_SRGGB16;
        input.planes[0].stride = 256;
        input.planes[0].size = 256 * input.height;
        cc_pisp_be_configuration c = { .input = input };
        OK(cc_pisp_be_configure(be, &c));
        CHECK(c.output.width == 128 && c.output.height == 96 && c.output.fourcc == V4L2_PIX_FMT_RGB24);
        CHECK(c.output.xfer_func == V4L2_XFER_FUNC_NONE && c.output.planes[0].stride == 384);
        CHECK(c.parameters.fourcc == CC_FOURCC('R', 'P', 'B', 'C'));
        uint32_t count = 3;
        OK(cc_pisp_be_allocate_buffers(be, &count));
        CHECK(count == 2);
    }
    return be;
}

static void test_be_history(void)
{
    struct fake f; init(&f); f.pisp = true;
    cc_frame raw; cc_dmabuf_plane plane;
    cc_camera *camera = producer(&f,&raw,&plane);
    int fd = plane.fd;
    cc_pisp_be *be = open_be(&f,false);
    cc_pisp_be_configuration c = {.input=raw_format()};
    c.input.fourcc=V4L2_PIX_FMT_SRGGB16; c.input.planes[0].stride=256;
    c.input.planes[0].size=256*c.input.height;
    for (uint32_t i=0;i<4;++i) {
        c.auxiliary[i].format.fourcc=CC_FOURCC('P','C','3','R');
        c.auxiliary[i].compression_offset=2048;
    }
    OK(cc_pisp_be_configure(be,&c));
    uint32_t count=3; OK(cc_pisp_be_allocate_buffers(be,&count)); CHECK(count==2);
    CHECK(cc_pisp_be_fd(be,CC_PISP_BE_TDN_INPUT)==32 && cc_pisp_be_fd(be,CC_PISP_BE_STITCH_OUTPUT)==35);
    struct pisp_be_config tuning={0};
    tuning.global.bayer_enables=PISP_BE_BAYER_ENABLE_TDN; tuning.tdn.reset=1;
    OK(cc_pisp_be_submit(be,0,&plane,1,&raw,&tuning,11));
    cc_pisp_be_result first;
    OK(cc_pisp_be_dequeue(be,&first,0));
    CHECK(first.cookie==11 && first.auxiliary[1].num_planes==1 && first.auxiliary[3].num_planes==1);
    CHECK(!first.auxiliary[0].num_planes && !first.auxiliary[2].num_planes);
    int history[2];
    history[0]=cc_pisp_be_export_port_plane(be,CC_PISP_BE_TDN_OUTPUT,0,0);
    history[1]=cc_pisp_be_export_port_plane(be,CC_PISP_BE_STITCH_OUTPUT,0,0);
    CHECK(history[0]>=0 && history[1]>=0);
    cc_pisp_be_aux_input aux[2] = {
        {.plane={history[0],c.auxiliary[1].format.planes[0].size},.frame=first.auxiliary[1]},
        {.plane={history[1],c.auxiliary[3].format.planes[0].size},.frame=first.auxiliary[3]}};
    tuning.tdn.reset=0; tuning.tdn.ratio=16384; tuning.tdn.noise_constant=64; tuning.tdn.threshold=32768;
    tuning.global.bayer_enables|=PISP_BE_BAYER_ENABLE_STITCH;
    tuning.stitch.exposure_ratio=32768|8192; tuning.stitch.threshold_lo=32768;
    tuning.stitch.threshold_diff_power=12; tuning.stitch.motion_threshold_256=16;
    int handles=f.handles;
    ERROR(cc_pisp_be_submit(be,1,&plane,1,&raw,&tuning,12),EINVAL);
    CHECK(f.handles==handles && !f.dma_pending);
    int source_refs=f.nodes[4].buffers[0].refs;
    f.fail_duplicate=f.duplicates+5;
    ERROR(cc_pisp_be_submit_with_auxiliary(be,1,&plane,1,&raw,aux,&tuning,12),EMFILE);
    CHECK(f.nodes[4].buffers[0].refs==source_refs && !f.dma_pending);
    OK(cc_pisp_be_submit_with_auxiliary(be,1,&plane,1,&raw,aux,&tuning,12));
    OK(close_fd(&f,history[0])); OK(close_fd(&f,history[1]));
    f.delayed_port=15;
    cc_pisp_be_result second={.cookie=37};
    ERROR(cc_pisp_be_dequeue(be,&second,2),ETIMEDOUT); CHECK(second.cookie==37);
    f.delayed_port=-1;
    OK(cc_pisp_be_dequeue(be,&second,0));
    CHECK(second.cookie==12 && second.auxiliary[0].num_planes && second.auxiliary[2].num_planes);
    CHECK(!second.auxiliary[0].planes[0].data && !second.auxiliary[2].planes[0].data);
    CHECK(f.last_parameters.config.tdn_decompress.mode==3 && f.last_parameters.config.stitch_compress.offset==2048);
    CHECK(f.last_parameters.config.stitch.motion_threshold_recip==16);
    history[0]=cc_pisp_be_export_port_plane(be,CC_PISP_BE_TDN_OUTPUT,1,0);
    aux[0].plane.fd=history[0]; aux[0].frame=second.auxiliary[1];
    tuning.global.bayer_enables=PISP_BE_BAYER_ENABLE_TDN;
    aux[1]=(cc_pisp_be_aux_input){0};
    OK(cc_pisp_be_release(be,1));
    ERROR(cc_pisp_be_submit_with_auxiliary(be,1,&plane,1,&raw,aux,&tuning,13),EEXIST);
    OK(close_fd(&f,history[0]));
    OK(cc_pisp_be_release(be,0));
    cc_pisp_be_close(be); OK(close_fd(&f,fd)); cc_camera_close(camera); finished(&f);
}

static void test_be_history_failures(void)
{
    const unsigned long requests[4]={VIDIOC_QBUF,VIDIOC_STREAMON,VIDIOC_DQBUF,VIDIOC_STREAMOFF};
    for (size_t phase=0;phase<4;++phase) for (int node=12;node<16;++node) {
        struct fake f; init(&f); f.pisp=true;
        cc_frame raw; cc_dmabuf_plane plane;
        cc_camera *camera=producer(&f,&raw,&plane);
        cc_pisp_be *be=open_be(&f,false);
        cc_pisp_be_configuration c={.input=raw_format()};
        c.input.fourcc=V4L2_PIX_FMT_SRGGB16; c.input.planes[0].stride=256;
        c.input.planes[0].size=256*c.input.height;
        for (uint32_t i=0;i<4;++i) c.auxiliary[i].format=c.input;
        OK(cc_pisp_be_configure(be,&c));
        uint32_t count=2; OK(cc_pisp_be_allocate_buffers(be,&count));
        struct pisp_be_config t={0};
        t.global.bayer_enables=PISP_BE_BAYER_ENABLE_TDN|PISP_BE_BAYER_ENABLE_STITCH;
        t.tdn.ratio=16384; t.stitch.exposure_ratio=8192;
        cc_pisp_be_aux_input aux[2]={{.plane=plane,.frame=raw},{.plane=plane,.frame=raw}};
        int source_refs=f.nodes[4].buffers[0].refs;
        f.fail_req=requests[phase]; f.fail_node=node; f.fail_nth=1; f.fail_errno=EIO;
        if (phase<2) ERROR(cc_pisp_be_submit_with_auxiliary(be,0,&plane,1,&raw,aux,&t,1),EIO);
        else {
            OK(cc_pisp_be_submit_with_auxiliary(be,0,&plane,1,&raw,aux,&t,1));
            if (phase==2) {
                cc_pisp_be_result result={.cookie=37};
                ERROR(cc_pisp_be_dequeue(be,&result,0),EIO); CHECK(result.cookie==37);
            } else {
                ERROR(cc_pisp_be_stop(be),EIO);
                ERROR(cc_pisp_be_fd(be,CC_PISP_BE_TDN_OUTPUT),ENODEV);
                for (int port=12;port<16;++port) CHECK(!f.nodes[port].open);
            }
        }
        CHECK(f.nodes[4].buffers[0].refs==source_refs);
        for (int port=0;port<NODES;++port) if (port<4 || port>=12) CHECK(!f.nodes[port].streaming);
        cc_pisp_be_close(be); OK(close_fd(&f,plane.fd)); cc_camera_close(camera); finished(&f);
    }
}

static void test_be_processing(void)
{
    struct fake f;
    init(&f); f.pisp = true;
    cc_pisp_be *be = open_be(&f, true);
    CHECK(cc_pisp_be_fd(be, CC_PISP_BE_INPUT) == 20 && cc_pisp_be_fd(be, CC_PISP_BE_PARAMETERS) == 22);
    cc_frame raw;
    cc_dmabuf_plane plane;
    cc_camera *camera = producer(&f, &raw, &plane);
    struct pisp_be_config tuning = {0};
    tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_WBG;
    tuning.wbg.gain_r = 8192; tuning.wbg.gain_g = 4096; tuning.wbg.gain_b = 6144;
    OK(cc_pisp_be_submit(be, 1, &plane, 1, &raw, &tuning, 1234));
    struct node live_nodes[NODES]; memcpy(live_nodes,f.nodes,sizeof(live_nodes));
    cc_pisp_be_configuration alternative={.input=f.nodes[0].format,
        .output={.fourcc=V4L2_PIX_FMT_YUV420M},.output1={.fourcc=V4L2_PIX_FMT_NV12M}};
    f.validation_only=true; OK(cc_pisp_be_try_configuration(be,&alternative)); f.validation_only=false;
    CHECK(!memcmp(live_nodes,f.nodes,sizeof(live_nodes)));
    tuning.wbg.gain_r = 0;
    CHECK(f.last_parameters.config.wbg.gain_r == 8192);
    OK(close_fd(&f, plane.fd));
    ERROR(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0), EBUSY);
    f.delayed_port = CC_PISP_BE_PARAMETERS;
    cc_pisp_be_result result = { .cookie = 5678 };
    ERROR(cc_pisp_be_dequeue(be, &result, 0), EAGAIN);
    CHECK(result.cookie == 5678 && f.last_input == raw.planes[0].data);
    ERROR(cc_pisp_be_dequeue(be, &result, 3), ETIMEDOUT);
    CHECK(f.wait_calls == 3 && result.cookie == 5678 && f.nodes[4].buffers[0].refs == 1);
    f.delayed_port = -1;
    OK(cc_pisp_be_dequeue(be, &result, 0));
    CHECK(result.index == 1 && result.cookie == 1234 && !result.input.planes[0].data);
    CHECK(result.input.timestamp_ns == raw.timestamp_ns && result.output.timestamp_ns == raw.timestamp_ns);
    CHECK(result.input.sequence == raw.sequence && result.output.planes[0].bytes_used == 384 * 96);
    CHECK(*(unsigned char *)result.output.planes[0].data == 0x31); /* Simulated DMA completion, not pixel processing. */
    const struct pisp_be_tiles_config *parameters = result.parameters.planes[0].data;
    CHECK(parameters->config.wbg.gain_r == 8192);
    CHECK(!f.nodes[4].buffers[0].refs && f.nodes[4].buffers[0].attachments == 1);
    OK(cc_camera_queue(camera, raw.index));
    OK(cc_camera_dequeue(camera, &raw, 0));
    plane.fd = cc_camera_export_plane(camera, raw.index, 0);
    CHECK(plane.fd >= 0);
    ERROR(cc_pisp_be_submit(be, 1, &plane, 1, &raw, NULL, 0), EBUSY);
    OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 2));
    OK(close_fd(&f, plane.fd));
    CHECK(f.starts == 4); /* Three BE ports plus the producer, started once. */
    int exported = cc_pisp_be_export_plane(be, 1, 0);
    CHECK(exported >= 0);
    OK(cc_pisp_be_release(be, 1));
    ERROR(cc_pisp_be_release(be, 1), EINVAL);
    OK(cc_pisp_be_dequeue(be, &result, 100));
    CHECK(result.cookie == 2 && result.index == 0);
    OK(cc_pisp_be_release(be, 0));
    ERROR(cc_camera_free_buffers(camera), EBUSY);
    ERROR(cc_pisp_be_free_buffers(be), EBUSY);
    OK(cc_camera_free_buffers(camera));
    OK(close_fd(&f, exported));
    OK(cc_pisp_be_free_buffers(be));
    cc_pisp_be_close(be);
    cc_camera_close(camera);
    finished(&f);
}

static void test_be_failures(void)
{
    const unsigned long requests[] = { VIDIOC_QBUF, VIDIOC_STREAMON, VIDIOC_DQBUF };
    for (size_t r = 0; r < sizeof(requests) / sizeof(requests[0]); ++r) {
        for (int node = 0; node < 3; ++node) {
            struct fake f;
            init(&f); f.pisp = true;
            cc_pisp_be *be = open_be(&f, true);
            cc_frame raw;
            cc_dmabuf_plane plane;
            cc_camera *camera = producer(&f, &raw, &plane);
            f.fail_req = requests[r]; f.fail_node = node; f.fail_nth = 1; f.fail_errno = EIO;
            if (requests[r] == VIDIOC_DQBUF) {
                OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0));
                OK(close_fd(&f, plane.fd));
                cc_pisp_be_result result = { .cookie = 9999 };
                ERROR(cc_pisp_be_dequeue(be, &result, 0), EIO);
                CHECK(result.cookie == 9999);
            } else {
                ERROR(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0), EIO);
                OK(close_fd(&f, plane.fd));
            }
            CHECK(!f.nodes[4].buffers[0].refs);
            for (int i = 0; i < 3; ++i) CHECK(!f.nodes[i].streaming);
            plane.fd = cc_camera_export_plane(camera, 0, 0);
            CHECK(plane.fd >= 0);
            OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 1));
            OK(close_fd(&f, plane.fd));
            f.fail_stop = true;
            ERROR(cc_pisp_be_stop(be), EIO);
            CHECK(!f.nodes[4].buffers[0].refs);
            for (int i = 0; i < 3; ++i) CHECK(!f.nodes[i].open);
            ERROR(cc_pisp_be_fd(be, CC_PISP_BE_OUTPUT), ENODEV);
            cc_pisp_be_close(be);
            cc_camera_close(camera);
            finished(&f);
        }
    }
    for (int fail = 1; fail <= 6; ++fail) {
        struct fake f;
        init(&f); f.pisp = true;
        cc_pisp_be *be = open_be(&f, true);
        OK(cc_pisp_be_free_buffers(be));
        f.fail_map = f.map_calls + fail;
        uint32_t count = 3;
        ERROR(cc_pisp_be_allocate_buffers(be, &count), ENOMEM);
        CHECK(!f.mappings);
        cc_pisp_be_close(be);
        finished(&f);
    }
    for (int fault = 0; fault < 8; ++fault) {
        struct fake f;
        init(&f); f.pisp = true;
        if (!fault) f.wrong_driver = true;
        else if (fault < 4) f.missing_port = fault - 1;
        else if (fault < 7) { f.fail_req = VIDIOC_QUERYCAP; f.fail_node = fault - 4; f.fail_nth = 1; f.fail_errno = EIO; }
        else { f.fail_req = MEDIA_IOC_ENUM_ENTITIES; f.fail_nth = 2; f.fail_errno = EIO; }
        const struct cc_io io = { &ops, &f };
        cc_pisp_be *be = (cc_pisp_be *)(uintptr_t)1;
        CHECK(cc_pisp_be_open_io("/dev/media0", &io, &be) < 0 && !be);
        finished(&f);
    }
    for (int fault = 0; fault < 8; ++fault) {
        struct fake f;
        init(&f); f.pisp = true;
        cc_pisp_be *be = open_be(&f, false);
        cc_format input = raw_format();
        input.fourcc = V4L2_PIX_FMT_SRGGB16;
        input.planes[0].stride = 256;
        input.planes[0].size = 256 * input.height;
        cc_pisp_be_configuration c = { .input = input };
        if (fault < 5) {
            if (fault < 3) {
                f.fail_req = VIDIOC_S_FMT;
                f.fail_node = fault; f.fail_nth = 1; f.fail_errno = EIO;
            } else if (fault == 3) f.mismatch_input = true;
            else f.bad_parameters = true;
            ERROR(cc_pisp_be_configure(be, &c), fault < 3 ? EIO : fault == 3 ? EPIPE : EPROTO);
            uint32_t count = 3;
            ERROR(cc_pisp_be_allocate_buffers(be, &count), EPIPE);
            f.mismatch_input = f.bad_parameters = false;
            OK(cc_pisp_be_configure(be, &c));
        } else {
            OK(cc_pisp_be_configure(be, &c));
            f.fail_req = VIDIOC_REQBUFS; f.fail_node = fault - 5; f.fail_nth = 1; f.fail_errno = ENOMEM;
            uint32_t count = 3;
            ERROR(cc_pisp_be_allocate_buffers(be, &count), ENOMEM);
            CHECK(!f.mappings);
        }
        cc_pisp_be_close(be);
        finished(&f);
    }
    for (int bad = 1; bad <= 2; ++bad) {
        struct fake f;
        init(&f); f.pisp = true;
        cc_pisp_be *be = open_be(&f, true);
        cc_frame raw;
        cc_dmabuf_plane plane;
        cc_camera *camera = producer(&f, &raw, &plane);
        f.fail_duplicate = f.duplicates + 1;
        ERROR(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0), EMFILE);
        CHECK(f.starts == 1);
        OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0));
        OK(close_fd(&f, plane.fd));
        f.corrupt_port = bad;
        cc_pisp_be_result result = { .cookie = 9999 };
        ERROR(cc_pisp_be_dequeue(be, &result, 0), EPROTO);
        CHECK(result.cookie == 9999 && !f.nodes[4].buffers[0].refs);
        cc_pisp_be_close(be);
        cc_camera_close(camera);
        finished(&f);
    }
}

static cc_pisp_be_configuration be_configuration(uint32_t main_format, uint32_t auxiliary_format)
{
    cc_format input = raw_format();
    input.fourcc = V4L2_PIX_FMT_SRGGB16;
    input.planes[0].stride = 256;
    input.planes[0].size = 256 * input.height;
    return (cc_pisp_be_configuration) { .input = input,
        .output = { .fourcc = main_format, .colorspace = V4L2_COLORSPACE_REC709,
                    .quantization = V4L2_QUANTIZATION_LIM_RANGE },
        .output1 = { .fourcc = auxiliary_format, .colorspace = V4L2_COLORSPACE_JPEG } };
}

static void test_be_yuv_outputs(void)
{
    const uint32_t formats[] = { V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_YVU420, V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21,
        V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_YVU420M, V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M,
        V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_YVYU, V4L2_PIX_FMT_VYUY,
        V4L2_PIX_FMT_YUV422P, V4L2_PIX_FMT_YUV422M, V4L2_PIX_FMT_YVU422M,
        V4L2_PIX_FMT_YUV444M, V4L2_PIX_FMT_YVU444M, V4L2_PIX_FMT_RGB24, V4L2_PIX_FMT_BGR24,
        V4L2_PIX_FMT_NV16,V4L2_PIX_FMT_NV61,V4L2_PIX_FMT_NV16M,V4L2_PIX_FMT_NV61M,
        CC_FOURCC('R','G','B','6'),CC_FOURCC('B','G','R','6') };
    for (size_t i = 0; i < sizeof(formats) / sizeof(formats[0]); ++i) {
        struct fake f;
        init(&f); f.pisp = f.widen_chroma = true;
        cc_pisp_be *be = open_be(&f, false);
        uint32_t aux = i % 2 ? V4L2_PIX_FMT_NV21M : V4L2_PIX_FMT_YUV420M;
        cc_pisp_be_configuration c = be_configuration(formats[i], aux);
        OK(cc_pisp_be_configure(be, &c));
        CHECK(c.output.ycbcr_enc == V4L2_YCBCR_ENC_709 && c.output1.ycbcr_enc == V4L2_YCBCR_ENC_601);
        if (formats[i] == V4L2_PIX_FMT_YUV420M) {
            CHECK(f.format_sets[1] == 2 && c.output.planes[1].stride == 128 && c.output.planes[1].size == 128 * 48);
        }
        if (aux == V4L2_PIX_FMT_YUV420M) CHECK(f.format_sets[3] == 2);
        uint32_t count = 3;
        OK(cc_pisp_be_allocate_buffers(be, &count));
        CHECK(count == 2 && cc_pisp_be_fd(be, CC_PISP_BE_OUTPUT1) == 23);
        cc_frame raw;
        cc_dmabuf_plane plane;
        cc_camera *camera = producer(&f, &raw, &plane);
        OK(cc_pisp_be_submit(be, 1, &plane, 1, &raw, NULL, 123));
        OK(close_fd(&f, plane.fd));
        CHECK(f.last_parameters.config.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1);
        CHECK(f.last_parameters.config.output_format[0].lo == 16 << 8 && !f.last_parameters.config.output_format[1].lo);
        f.delayed_port = 3;
        cc_pisp_be_result result = { .cookie = 456 };
        ERROR(cc_pisp_be_dequeue(be, &result, 0), EAGAIN);
        ERROR(cc_pisp_be_dequeue(be, &result, 3), ETIMEDOUT);
        CHECK(result.cookie == 456 && f.nodes[4].buffers[0].refs == 1);
        f.delayed_port = -1;
        OK(cc_pisp_be_dequeue(be, &result, 0));
        CHECK(result.cookie == 123 && result.output.num_planes == c.output.num_planes && result.output1.num_planes == c.output1.num_planes);
        for (uint32_t p = 0; p < c.output.num_planes; ++p) {
            CHECK(result.output.planes[p].length == c.output.planes[p].size);
            CHECK(result.output.planes[p].bytes_used == c.output.planes[p].size);
            CHECK(*(unsigned char *)result.output.planes[p].data == 0x31 + p);
        }
        for (uint32_t p = 0; p < c.output1.num_planes; ++p) {
            CHECK(result.output1.planes[p].length == c.output1.planes[p].size);
            CHECK(result.output1.planes[p].bytes_used == c.output1.planes[p].size);
            CHECK(*(unsigned char *)result.output1.planes[p].data == 0x33 + p);
        }
        int exported = cc_pisp_be_export_output_plane(be, 1, result.index, c.output1.num_planes - 1);
        CHECK(exported >= 0);
        OK(cc_camera_queue(camera, raw.index));
        OK(cc_camera_dequeue(camera, &raw, 0));
        plane.fd = cc_camera_export_plane(camera, raw.index, 0);
        CHECK(plane.fd >= 0);
        ERROR(cc_pisp_be_submit(be, 1, &plane, 1, &raw, NULL, 0), EBUSY);
        OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 999));
        OK(close_fd(&f, plane.fd));
        OK(cc_pisp_be_release(be, 1));
        OK(cc_pisp_be_dequeue(be, &result, 100));
        CHECK(result.cookie == 999 && f.starts == 5);
        OK(cc_pisp_be_release(be, 0));
        ERROR(cc_camera_free_buffers(camera), EBUSY);
        ERROR(cc_pisp_be_free_buffers(be), EBUSY); /* Exported chroma plane pins its allocation. */
        CHECK(!f.mappings);
        OK(cc_camera_free_buffers(camera));
        OK(close_fd(&f, exported));
        OK(cc_pisp_be_free_buffers(be));
        c.output1.fourcc = 0;
        OK(cc_pisp_be_configure(be, &c));
        CHECK(!c.output1.width && !c.output1.num_planes);
        count = 3;
        OK(cc_pisp_be_allocate_buffers(be, &count));
        CHECK(!f.nodes[3].count); /* Inactive branch is not allocated or queued. */
        count = 3;
        OK(cc_camera_allocate_buffers(camera, &count));
        OK(cc_camera_queue(camera, 0));
        OK(cc_camera_start(camera));
        OK(cc_camera_dequeue(camera, &raw, 0));
        plane.fd = cc_camera_export_plane(camera, raw.index, 0);
        CHECK(plane.fd >= 0);
        OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 321));
        OK(close_fd(&f, plane.fd));
        OK(cc_pisp_be_dequeue(be, &result, 100));
        CHECK(result.cookie == 321 && !result.output1.num_planes);
        CHECK(!(f.last_parameters.config.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1));
        OK(cc_pisp_be_release(be, 0));
        cc_pisp_be_close(be);
        cc_camera_close(camera);
        finished(&f);
    }
}

static void test_be_aux_failures(void)
{
    const unsigned long requests[] = { VIDIOC_QBUF, VIDIOC_STREAMON, VIDIOC_DQBUF, VIDIOC_STREAMOFF };
    for (size_t request = 0; request < sizeof(requests) / sizeof(requests[0]); ++request) {
        struct fake f;
        init(&f); f.pisp = true;
        cc_pisp_be *be = open_be(&f, false);
        cc_pisp_be_configuration c = be_configuration(V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_YUV420M);
        OK(cc_pisp_be_configure(be, &c));
        uint32_t count = 3;
        OK(cc_pisp_be_allocate_buffers(be, &count));
        cc_frame raw;
        cc_dmabuf_plane plane;
        cc_camera *camera = producer(&f, &raw, &plane);
        f.fail_req = requests[request]; f.fail_node = 3; f.fail_nth = 1; f.fail_errno = EIO;
        if (requests[request] == VIDIOC_QBUF || requests[request] == VIDIOC_STREAMON) {
            ERROR(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0), EIO);
            OK(close_fd(&f, plane.fd));
        } else {
            OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 0));
            OK(close_fd(&f, plane.fd));
            if (requests[request] == VIDIOC_STREAMOFF) {
                ERROR(cc_pisp_be_stop(be), EIO);
                ERROR(cc_pisp_be_fd(be, CC_PISP_BE_OUTPUT1), ENODEV);
            } else {
                cc_pisp_be_result result = { .cookie = 456 };
                ERROR(cc_pisp_be_dequeue(be, &result, 0), EIO);
                CHECK(result.cookie == 456);
            }
        }
        CHECK(!f.nodes[4].buffers[0].refs);
        for (int p = 0; p < 4; ++p) CHECK(!f.nodes[p].streaming);
        cc_pisp_be_close(be);
        cc_camera_close(camera);
        finished(&f);
    }
    for (int fault = 0; fault < 6; ++fault) {
        struct fake f;
        init(&f); f.pisp = true;
        cc_pisp_be *be = open_be(&f, false);
        cc_pisp_be_configuration c = be_configuration(V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_NV21M);
        if (fault < 3) {
            /* The graph is already enumerated, so missing-node failure needs
             * a fresh open to exercise deferred auxiliary-port discovery. */
            cc_pisp_be_close(be);
            if (!fault) f.missing_port = 3;
            else { f.fail_req = fault == 1 ? VIDIOC_QUERYCAP : VIDIOC_S_FMT; f.fail_node = 3; f.fail_nth = 1; f.fail_errno = EIO; }
            be = open_be(&f, false);
            ERROR(cc_pisp_be_configure(be, &c), fault ? EIO : ENODEV);
        } else if (fault == 3) {
            OK(cc_pisp_be_configure(be, &c));
            f.fail_req = VIDIOC_REQBUFS; f.fail_node = 3; f.fail_nth = 1; f.fail_errno = ENOMEM;
            uint32_t count = 3;
            ERROR(cc_pisp_be_allocate_buffers(be, &count), ENOMEM);
            CHECK(!f.mappings);
        } else {
            f.widen_chroma = f.refuse_capacity = true;
            if (fault == 5) { c.output.fourcc = V4L2_PIX_FMT_NV12; c.output1.fourcc = V4L2_PIX_FMT_YUV420M; }
            ERROR(cc_pisp_be_configure(be, &c), EINVAL);
            uint32_t count = 3;
            ERROR(cc_pisp_be_allocate_buffers(be, &count), EPIPE);
        }
        cc_pisp_be_close(be);
        finished(&f);
    }
    /* Four main slots x 3 planes, two config slots, three aux slots x 2 planes. */
    for (int fail = 1; fail <= 20; ++fail) {
        struct fake f;
        init(&f); f.pisp = true;
        cc_pisp_be *be = open_be(&f, false);
        cc_pisp_be_configuration c = be_configuration(V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_NV12M);
        OK(cc_pisp_be_configure(be, &c));
        f.fail_map = fail;
        uint32_t count = 3;
        ERROR(cc_pisp_be_allocate_buffers(be, &count), ENOMEM);
        CHECK(!f.mappings);
        cc_pisp_be_close(be);
        finished(&f);
    }
}

static void test_be_resize(void)
{
    struct fake f;
    init(&f); f.pisp = f.force_color_defaults = true;
    cc_pisp_be *be = open_be(&f, false);
    cc_pisp_be_configuration c = be_configuration(V4L2_PIX_FMT_BGR24, V4L2_PIX_FMT_NV12M);
    c.crop[0] = (cc_rectangle) {16, 8, 96, 80};
    c.output.width = 63; c.output.height = 47;
    c.output.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    c.output.xfer_func = V4L2_XFER_FUNC_NONE;
    c.output1.width = c.output1.height = 32;
    c.output1.colorspace = V4L2_COLORSPACE_SMPTE170M;
    c.output1.ycbcr_enc = V4L2_YCBCR_ENC_709; /* Deliberately differs from the driver's forced 601. */
    c.output1.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    c.output1.xfer_func = V4L2_XFER_FUNC_NONE;
    c.transform[0] = PISP_BE_TRANSFORM_ROT180;
    c.transform[1] = PISP_BE_TRANSFORM_HFLIP;
    OK(cc_pisp_be_configure(be, &c));
    CHECK(c.output.width == 63 && c.output.height == 47 && c.crop[0].x == 16 && c.crop[1].width == 128);
    CHECK(c.output.xfer_func == V4L2_XFER_FUNC_NONE && c.output1.ycbcr_enc == V4L2_YCBCR_ENC_709);
    CHECK(c.output1.quantization == V4L2_QUANTIZATION_FULL_RANGE && !c.input.quantization);
    CHECK(f.nodes[3].format.ycbcr_enc == V4L2_YCBCR_ENC_601 && f.nodes[3].format.xfer_func != c.output1.xfer_func);
    uint32_t count = 3;
    OK(cc_pisp_be_allocate_buffers(be, &count)); /* G_FMT comparison uses actual driver metadata. */
    cc_frame raw;
    cc_dmabuf_plane plane;
    cc_camera *camera = producer(&f, &raw, &plane);
    OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 123));
    OK(close_fd(&f, plane.fd));
    const struct pisp_be_config *parameters = &f.last_parameters.config;
    CHECK(parameters->resample[0].scale_factor_h == 95U * 4096U / 62U);
    CHECK(parameters->resample[0].scale_factor_v == 79U * 4096U / 46U);
    CHECK(parameters->global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE1);
    CHECK(parameters->output_format[0].transform == PISP_BE_TRANSFORM_ROT180);
    CHECK(!parameters->output_format[1].lo && parameters->csc[1].coeffs[0] == 218); /* Programmed 709/full. */
    cc_pisp_be_result result;
    OK(cc_pisp_be_dequeue(be, &result, 100));
    CHECK(result.output.planes[0].bytes_used == (size_t)c.output.planes[0].stride * 47);
    CHECK(result.output1.num_planes == 2 && result.output1.planes[1].bytes_used == c.output1.planes[1].size);
    OK(cc_pisp_be_release(be, result.index));
    cc_pisp_be_close(be);
    cc_camera_close(camera);
    finished(&f);

    init(&f); f.pisp = true;
    be = open_be(&f, false);
    c = be_configuration(V4L2_PIX_FMT_RGB24, 0);
    c.crop[0] = (cc_rectangle) {20, 30, 64, 48};
    OK(cc_pisp_be_configure(be, &c));
    CHECK(c.output.width == 64 && c.output.height == 48); /* Zero size follows the crop. */
    cc_pisp_be_close(be);
    finished(&f);

    init(&f); f.pisp = true;
    be = open_be(&f, false);
    c = be_configuration(V4L2_PIX_FMT_RGB24, 0);
    c.input.width = c.input.height = 16;
    c.input.planes[0] = (cc_plane_format) {32, 32 * 16};
    OK(cc_pisp_be_configure(be, &c));
    CHECK(c.output.width == 16 && c.output.height == 16);
    count = 2;
    OK(cc_pisp_be_allocate_buffers(be, &count));
    cc_pisp_be_close(be);
    finished(&f);
}

static void test_be_rgb32_outputs(void)
{
    const uint32_t formats[]={V4L2_PIX_FMT_RGBX32,V4L2_PIX_FMT_XRGB32,V4L2_PIX_FMT_XBGR32,V4L2_PIX_FMT_BGRX32};
    const uint32_t revisions[]={0,CC_PISP_BE_VERSION_BCM2712_C0,CC_PISP_BE_VERSION_BCM2712_D0};
    for (size_t i=0;i<4;++i) for (size_t revision=0;revision<3;++revision) {
        struct fake f; init(&f); f.pisp=true; f.hardware_version=revisions[revision];
        cc_pisp_be *be=open_be(&f,false);
        cc_pisp_be_configuration c=be_configuration(i & 1U ? V4L2_PIX_FMT_RGB24 : formats[i],
            i & 1U ? formats[i] : CC_FOURCC('R','G','B','6'));
        c.hardware_version=CC_PISP_BE_VERSION_BCM2712_D0; /* Cannot override the reported revision. */
        c.output.quantization=V4L2_QUANTIZATION_FULL_RANGE;
        if (revisions[revision]!=CC_PISP_BE_VERSION_BCM2712_D0) {
            ERROR(cc_pisp_be_configure(be,&c),ENOTSUP);
            CHECK(!f.format_sets[0] && !f.format_sets[1] && !f.format_sets[3]);
            cc_pisp_be_close(be); finished(&f); continue;
        }
        OK(cc_pisp_be_configure(be,&c));
        const cc_format *output=i & 1U ? &c.output1 : &c.output;
        CHECK(c.hardware_version==CC_PISP_BE_VERSION_BCM2712_D0 && output->num_planes==1 &&
              output->planes[0].stride==128*4 && output->planes[0].size==128*4*96);
        if (!(i & 1U)) CHECK(c.output1.planes[0].stride==128*6 && c.output1.planes[0].size==128*6*96);
        uint32_t count=2; OK(cc_pisp_be_allocate_buffers(be,&count));
        cc_frame raw; cc_dmabuf_plane plane; cc_camera *camera=producer(&f,&raw,&plane);
        OK(cc_pisp_be_submit(be,1,&plane,1,&raw,NULL,567)); OK(close_fd(&f,plane.fd));
        cc_pisp_be_result result; OK(cc_pisp_be_dequeue(be,&result,0));
        CHECK(result.cookie==567 && result.output.planes[0].bytes_used==c.output.planes[0].size &&
              result.output1.planes[0].bytes_used==c.output1.planes[0].size);
        const struct pisp_be_tiles_config *params=result.parameters.planes[0].data;
        CHECK(params->config.output_format[i & 1U].image.format & PISP_IMAGE_FORMAT_BPP_32);
        OK(cc_pisp_be_release(be,result.index)); cc_pisp_be_close(be); cc_camera_close(camera); finished(&f);
    }
}

static void test_be_native_blocks(void)
{
    const uint32_t revisions[] = {0, CC_PISP_BE_VERSION_BCM2712_C0, CC_PISP_BE_VERSION_BCM2712_D0};
    for (size_t i = 0; i < sizeof(revisions) / sizeof(revisions[0]); ++i) {
        struct fake f;
        init(&f); f.pisp = true; f.hardware_version = revisions[i];
        cc_pisp_be *be = open_be(&f, false);
        cc_pisp_be_configuration c = be_configuration(V4L2_PIX_FMT_RGB24, 0);
        c.hardware_version = CC_PISP_BE_VERSION_BCM2712_D0; /* Cannot override actual C0 hardware. */
        OK(cc_pisp_be_configure(be, &c));
        CHECK(c.hardware_version == revisions[i]);
        uint32_t count = 2;
        OK(cc_pisp_be_allocate_buffers(be, &count));
        cc_frame raw;
        cc_dmabuf_plane plane;
        cc_camera *camera = producer(&f, &raw, &plane);
        struct pisp_be_config t = {0};
        t.global.bayer_enables = PISP_BE_BAYER_ENABLE_SDN | PISP_BE_BAYER_ENABLE_CDN |
            PISP_BE_BAYER_ENABLE_CAC | PISP_BE_BAYER_ENABLE_TONEMAP | PISP_BE_BAYER_ENABLE_DEBIN;
        t.global.rgb_enables = PISP_BE_RGB_ENABLE_CCM | PISP_BE_RGB_ENABLE_YCBCR |
            PISP_BE_RGB_ENABLE_YCBCR_INVERSE | PISP_BE_RGB_ENABLE_SHARPEN | PISP_BE_RGB_ENABLE_FALSE_COLOUR |
            PISP_BE_RGB_ENABLE_SAT_CONTROL;
        t.sdn.noise_constant = t.sdn.noise_constant2 = 100;
        t.cdn.thresh = 250;
        t.cac.grid_step_x = 60000; t.cac.grid_step_y = 50000;
        t.cac_extra = (struct pisp_be_cac_extra) {4, 8};
        t.cac.lut[8][8][1][1] = -64;
        t.debin = (struct pisp_be_debin_config) {{-7,105,35,-5}, 1, 0, {0,0}};
        t.tonemap.strength = 256; t.tonemap.lut[63] = UINT32_MAX;
        t.sat_control.shift_r = 2;
        t.ccm.coeffs[0] = t.ccm.coeffs[4] = t.ccm.coeffs[8] = 1024;
        t.ycbcr = t.ycbcr_inverse = t.ccm;
        t.false_colour.distance = 2;
        t.sharpen.kernel0[12] = 31; t.sharpen.kernel0[13] = -31;
        t.sharpen.enables = 1; t.sharpen.positive_strength = 500;
        t.sh_fc_combine.c1_factor = 100;
        int duplicates = f.duplicates;
        int source_refs = f.nodes[4].buffers[0].refs;
        t.sharpen.kernel0[12] = 32;
        ERROR(cc_pisp_be_submit(be, 0, &plane, 1, &raw, &t, 0), ERANGE);
        CHECK(f.duplicates == duplicates && f.nodes[4].buffers[0].refs == source_refs &&
              !f.nodes[4].buffers[0].attachments);
        t.sharpen.kernel0[12] = 31;
        t.ccm.offsets[0] = 134217728;
        if (revisions[i] != CC_PISP_BE_VERSION_BCM2712_D0) {
            ERROR(cc_pisp_be_submit(be, 0, &plane, 1, &raw, &t, 0), ERANGE);
            CHECK(f.duplicates == duplicates);
            t.ccm.offsets[0] = 0;
        }
        OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, &t, 123));
        const struct pisp_be_tiles_config *p = &f.last_parameters;
        CHECK(p->config.sdn.noise_constant2 == 100 && p->config.cdn.thresh == 250);
        CHECK(p->config.cac.lut[8][8][1][1] == -64 && p->config.debin.coeffs[1] == 105);
        CHECK(p->config.tonemap.lut[63] == UINT32_MAX && p->config.sat_control.shift_r == 2);
        CHECK(p->config.sharpen.positive_strength == 500 && p->config.false_colour.distance == 2);
        CHECK(p->config.sh_fc_combine.c1_factor == 100 && p->config.dirty_flags_extra & PISP_BE_DIRTY_SH_FC_COMBINE);
        CHECK(p->tiles[0].cac_grid_offset_x == 240000 && p->tiles[0].cac_grid_offset_y == 400000);
        CHECK(p->config.ccm.offsets[0] == t.ccm.offsets[0]);
        memset(&t, 0, sizeof(t)); /* Submission owns a snapshot. */
        cc_pisp_be_result result;
        OK(cc_pisp_be_dequeue(be, &result, 100));
        CHECK(result.cookie == 123);
        OK(cc_pisp_be_release(be, result.index));
        OK(cc_pisp_be_submit(be, 0, &plane, 1, &raw, NULL, 456));
        CHECK(!(f.last_parameters.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC));
        CHECK(!(f.last_parameters.config.global.rgb_enables & PISP_BE_RGB_ENABLE_SHARPEN));
        CHECK(!f.last_parameters.tiles[0].cac_grid_offset_x);
        OK(close_fd(&f, plane.fd));
        OK(cc_pisp_be_dequeue(be, &result, 100));
        CHECK(result.cookie == 456);
        cc_pisp_be_close(be);
        cc_camera_close(camera);
        finished(&f);
    }
    struct fake f;
    init(&f); f.pisp = true; f.hardware_version = CC_PISP_BE_VERSION_BCM2712_D0 + 1U;
    cc_pisp_be *be = open_be(&f, false);
    cc_pisp_be_configuration c = be_configuration(V4L2_PIX_FMT_RGB24, 0);
    ERROR(cc_pisp_be_configure(be, &c), ENOTSUP);
    CHECK(!f.format_sets[0] && !f.format_sets[1]);
    cc_pisp_be_close(be);
    finished(&f);
}

static cc_capture *pipeline_requests_mode(struct fake *f, uint32_t request_buffers)
{
    cc_capture_devices d = {.request_buffers=request_buffers};
    const enum cc_memory memories[2] = {
        request_buffers & CC_CAPTURE_REQUEST_OUTPUT0 ? CC_MEMORY_DMABUF : CC_MEMORY_MMAP,
        request_buffers & CC_CAPTURE_REQUEST_OUTPUT1 ? CC_MEMORY_DMABUF : CC_MEMORY_MMAP};
    if (f->pisp) {
        d.backend = open_be(f, false);
        cc_pisp_be_configuration config = be_configuration(V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_YUV420M);
        if (f->compression) {
            config.input.fourcc=CC_FOURCC('P','C','0'+f->compression,'R');
            config.input.planes[0]=(cc_plane_format){128,128*96};
            config.compression_offset=f->mismatch_compression_offset ? 1024 : 2048;
        }
        if (f->history_ports) for (uint32_t i=0;i<4;++i) {
            config.auxiliary[i].format=config.input;
            config.auxiliary[i].compression_offset=config.compression_offset;
        }
        OK(cc_pisp_be_configure(d.backend, &config));
        uint32_t count = 3;
        OK(cc_pisp_be_allocate_buffers_with_memory(d.backend, memories, &count));
    } else {
        d.vc4 = open_isp(f); configure(d.vc4); uint32_t n = 3;
        OK(cc_vc4_isp_allocate_buffers_with_memory(d.vc4, memories, &n));
    }
    const struct cc_io io = { &ops, f };
    uint32_t count = 3;
    if (f->cfe) {
        cc_camera_info info={.pipeline=CC_PIPELINE_RPI_CFE,.media_path="/dev/media1",
            .video_path="/dev/video7",.sensor_entity=1};
        OK(cc_cfe_open_io(&info,&io,&d.frontend));
        cc_cfe_configuration config={.sensor=raw_format()};
        if (f->compression) {
            config.image.fourcc=CC_FOURCC('P','C','0'+f->compression,'R'); config.compression_offset=2048;
        }
        OK(cc_cfe_configure(d.frontend,&config));
        if (f->embedded) { cc_embedded_configuration embedded; OK(cc_cfe_configure_embedded(d.frontend,&embedded)); }
        OK(cc_cfe_allocate_buffers_with_memory(d.frontend,
            request_buffers & CC_CAPTURE_REQUEST_RAW ? CC_MEMORY_DMABUF : CC_MEMORY_MMAP,&count));
    } else {
        cc_camera_info info = { .pipeline = CC_PIPELINE_V4L2, .video_path = "/dev/video4" };
        OK(cc_camera_open_io(&info, &io, &d.camera));
        cc_format format = raw_format();
        if (f->pisp) {
            format.fourcc = V4L2_PIX_FMT_SRGGB16;
            format.planes[0] = (cc_plane_format) {256, 256 * 96};
        }
        OK(cc_camera_configure(d.camera, &format));
        if (request_buffers & CC_CAPTURE_REQUEST_RAW) OK(cc_camera_prepare_imports(d.camera, &count));
        else OK(cc_camera_allocate_buffers(d.camera, &count));
    }
    cc_capture *capture = NULL;
    if (f->mismatch_compression_offset || f->create_error) {
        int expected = f->create_error ? f->create_error : EPIPE;
        ERROR(cc_capture_create(&d,&capture,&count),expected);
        CHECK(!capture && (d.camera || d.frontend) && (d.backend || d.vc4) && count==3);
        cc_pisp_be_close(d.backend); cc_vc4_isp_close(d.vc4);
        cc_cfe_close(d.frontend); cc_camera_close(d.camera); return NULL;
    }
    OK(cc_capture_create(&d, &capture, &count));
    CHECK(count == 2 && !d.camera && !d.frontend && !d.vc4 && !d.backend);
    return capture;
}

static cc_capture *pipeline_requests(struct fake *f) { return pipeline_requests_mode(f, 0); }

static void test_graph_history(void)
{
    for (uint32_t compression=0;compression<=3;compression+=3) for (int fault=0;fault<4;++fault) {
        struct fake f; init(&f); f.pisp=f.history_ports=true; f.cfe=compression!=0; f.compression=compression;
        cc_capture *c=pipeline_requests(&f);
        cc_request *seed,*next,*unused,*completed=NULL;
        OK(cc_request_create(101,&seed)); OK(cc_request_create(102,&next)); OK(cc_request_create(103,&unused));
        ERROR(cc_request_set_pisp_input(next,CC_PISP_BE_TDN_OUTPUT,seed,CC_CAPTURE_RAW),EINVAL);
        ERROR(cc_request_set_pisp_input(next,CC_PISP_BE_TDN_INPUT,seed,CC_CAPTURE_RAW),ENODATA);
        struct pisp_be_config t={0}; t.global.bayer_enables=PISP_BE_BAYER_ENABLE_TDN; t.tdn.reset=1;
        OK(cc_request_set_tuning(seed,NULL,&t)); OK(cc_capture_queue(c,seed));
        OK(cc_capture_dequeue(c,&completed,50)); CHECK(completed==seed);
        CHECK(cc_request_get_result(seed)->backend_auxiliary[1].num_planes==1);
        CHECK(cc_request_get_result(seed)->backend_auxiliary[3].num_planes==1);
        OK(cc_request_set_pisp_input(next,CC_PISP_BE_TDN_INPUT,seed,CC_CAPTURE_TDN_OUTPUT));
        OK(cc_request_set_pisp_input(next,CC_PISP_BE_STITCH_INPUT,seed,CC_CAPTURE_STITCH_OUTPUT));
        ERROR(cc_capture_release(c,seed),EBUSY);
        ERROR(cc_request_set_pisp_input(next,CC_PISP_BE_TDN_INPUT,seed,CC_CAPTURE_OUTPUT0),EINVAL);
        ERROR(cc_capture_release(c,seed),EBUSY); /* Rejected setter retains the binding. */
        t.tdn.reset=0; t.tdn.ratio=16384;
        t.global.bayer_enables|=PISP_BE_BAYER_ENABLE_STITCH;
        t.stitch.exposure_ratio=8192;
        OK(cc_request_set_tuning(next,NULL,&t));
        OK(cc_capture_queue(c,next));
        ERROR(cc_request_set_pisp_input(next,CC_PISP_BE_TDN_INPUT,NULL,CC_CAPTURE_RAW),EBUSY);
        if (fault==2) {
            f.fail_req=VIDIOC_DQBUF; f.fail_node=14; f.fail_nth=1; f.fail_errno=EIO;
        } else f.delayed_port=15;
        if (fault!=2) {
            completed=NULL;
            ERROR(cc_capture_dequeue(c,&completed,2),ETIMEDOUT); CHECK(!completed);
            ERROR(cc_capture_release(c,seed),EBUSY);
            struct pollfd fds[16]; size_t n=16; OK(cc_capture_pollfds(c,fds,&n));
            CHECK(n==1 && fds[0].fd==35 && fds[0].events==POLLIN);
        }
        if (fault==1 || fault==3) {
            if (fault==1) OK(cc_capture_stop(c));
            else cc_capture_close(c);
            if (fault==1) {
                OK(cc_capture_dequeue(c,&completed,0)); CHECK(completed==next);
                CHECK(cc_request_get_result(next)->status==CC_REQUEST_CANCELLED);
                OK(cc_capture_release(c,seed)); OK(cc_capture_release(c,next));
            }
        } else {
            f.delayed_port=-1;
            OK(cc_capture_dequeue(c,&completed,50)); CHECK(completed==next);
            const cc_capture_result *result=cc_request_get_result(next);
            if (fault==2) {
                CHECK(result->status==CC_REQUEST_FAILED && result->error==-EIO);
                CHECK(!result->backend_auxiliary[1].num_planes);
            } else {
                CHECK(result->status==CC_REQUEST_COMPLETE);
                CHECK(result->backend_auxiliary[0].num_planes && !result->backend_auxiliary[0].planes[0].data);
                CHECK(result->backend_auxiliary[2].num_planes && result->backend_auxiliary[3].planes[0].data);
                int fd=cc_capture_export_plane(c,next,CC_CAPTURE_TDN_OUTPUT,0); CHECK(fd>=0); OK(close_fd(&f,fd));
                OK(cc_request_set_pisp_input(unused,CC_PISP_BE_TDN_INPUT,next,CC_CAPTURE_TDN_OUTPUT));
                ERROR(cc_capture_release(c,next),EBUSY);
                OK(cc_request_destroy(unused)); unused=NULL; /* Idle destination releases its pin. */
            }
            OK(cc_capture_release(c,seed)); OK(cc_capture_release(c,next));
        }
        if (fault!=3) cc_capture_close(c);
        OK(cc_request_destroy(seed)); OK(cc_request_destroy(next)); OK(cc_request_destroy(unused)); finished(&f);
    }
}

/* Independent allocator: each request chooses objects unrelated to native queue
 * indices. Keep padding so tests can detect writes beyond negotiated image size. */
static int try_bind_external(struct fake *f, cc_request *request, unsigned int stream, int object, bool short_buffer)
{
    const struct cc_io io = {&ops, f};
    int native = !stream ? 4 : stream == 1 ? 1 : f->pisp ? 3 : 2;
    const cc_format *format = &f->nodes[native].format;
    cc_dmabuf_plane planes[3];
    for (uint32_t p = 0; p < format->num_planes; ++p) {
        struct buffer *b = buffer_plane(&f->nodes[9 + stream], object, (int)p);
        if (!b->data) {
            b->size = format->planes[p].size + 64;
            b->data = malloc(b->size); CHECK(b->data); memset(b->data, 0xa5, b->size);
        }
        planes[p] = (cc_dmabuf_plane){descriptor(f, 9 + (int)stream, object, (int)p), short_buffer ? 1U : b->size};
    }
    int ret = cc_request_set_buffer_io(request, stream, planes, format->num_planes, &io);
    for (uint32_t p = 0; p < format->num_planes; ++p) OK(close_fd(f, planes[p].fd));
    return ret;
}
static void bind_external(struct fake *f, cc_request *request, unsigned int stream, int object, bool short_buffer)
{
    OK(try_bind_external(f, request, stream, object, short_buffer));
}
static void bind_images(struct fake *f, cc_request *request, uint32_t mask, int object)
{
    for (unsigned int stream = 0; stream < 3; ++stream)
        if (mask & (1U << stream)) bind_external(f, request, stream, object, false);
}
static void check_imported_result(struct fake *f, cc_capture *capture, cc_request *request, uint32_t mask, int object)
{
    const cc_capture_result *r = cc_request_get_result(request);
    CHECK(r && r->status == CC_REQUEST_COMPLETE);
    const cc_frame *frames[] = {&r->raw, &r->output[0], &r->output[1]};
    for (unsigned int stream = 0; stream < 3; ++stream) {
        const cc_frame *frame = frames[stream];
        CHECK(frame->num_planes && (frame->planes[0].data == NULL) == ((mask & (1U << stream)) != 0));
        if (!(mask & (1U << stream))) continue;
        int native = !stream ? 4 : stream == 1 ? 1 : f->pisp ? 3 : 2;
        for (uint32_t p = 0; p < frame->num_planes; ++p) {
            struct buffer *b = buffer_plane(&f->nodes[9 + stream], object, (int)p);
            uint32_t written = f->nodes[native].format.planes[p].size;
            CHECK(frame->planes[p].bytes_used == written && frame->planes[p].length == b->size);
            CHECK(((unsigned char *)b->data)[0] != 0xa5 && ((unsigned char *)b->data)[written - 1] != 0xa5);
            for (uint32_t j = written; j < b->size; ++j) CHECK(((unsigned char *)b->data)[j] == 0xa5);
            int fd = cc_capture_export_plane(capture, request, (enum cc_capture_stream)stream, p);
            CHECK(fd >= 100 && f->fds[fd].node == 9 + (int)stream && f->fds[fd].slot == object && f->fds[fd].plane == (int)p);
            OK(close_fd(f, fd));
        }
    }
    if (!f->pisp || f->cfe) CHECK(r->statistics.num_planes && r->statistics.planes[0].data);
    else CHECK(!r->statistics.num_planes);
}
static void test_request_buffers(bool pisp, bool cfe, uint32_t mask)
{
    struct fake f; init(&f); f.pisp = pisp; f.cfe = cfe; f.fresh_exports = true;
    cc_capture *capture = pipeline_requests_mode(&f, mask);
    cc_request *requests[3], *done;
    for (int i = 0; i < 3; ++i) {
        OK(cc_request_create((uint64_t)i, &requests[i]));
        ERROR(cc_capture_queue(capture, requests[i]), EINVAL); /* Missing required image(s). */
        bind_images(&f, requests[i], mask, i);
        OK(cc_capture_queue(capture, requests[i]));
        ERROR(cc_request_set_buffer(requests[i], CC_CAPTURE_RAW, NULL, 0), EBUSY);
    }
    for (int i = 0; i < 2; ++i) {
        OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == requests[i]);
        check_imported_result(&f, capture, done, mask, i);
    }
    const cc_capture_result *held = cc_request_get_result(requests[1]);
    cc_request *alias; OK(cc_request_create(99, &alias)); bind_images(&f, alias, mask, 1);
    ERROR(cc_capture_queue(capture, alias), EBUSY); OK(cc_request_destroy(alias));
    done = NULL; ERROR(cc_capture_dequeue(capture, &done, 0), EAGAIN); CHECK(!done);
    CHECK(cc_request_get_status(requests[2]) == CC_REQUEST_PENDING);
    OK(cc_capture_release(capture, requests[0]));
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == requests[2]);
    check_imported_result(&f, capture, done, mask, 2); CHECK(held->cookie == 1);
    OK(cc_capture_release(capture, done)); OK(cc_capture_release(capture, requests[1]));
    /* Same caller objects survive graph stop/restart and request reuse. */
    OK(cc_capture_stop(capture)); OK(cc_capture_queue(capture, requests[0]));
    OK(cc_capture_dequeue(capture, &done, 0)); check_imported_result(&f, capture, done, mask, 0);
    OK(cc_capture_stop(capture)); CHECK(!cc_request_get_result(done)->raw.num_planes);
    OK(cc_capture_release(capture, done));
    unsigned int selected = mask & 1U ? 0U : mask & 2U ? 1U : 2U;
    bind_external(&f, requests[0], selected, 0, true);
    ERROR(cc_capture_queue(capture, requests[0]), ENOSPC);
    bind_external(&f, requests[0], selected, 0, false);
    OK(cc_capture_queue(capture, requests[0])); OK(cc_capture_dequeue(capture, &done, 0));
    cc_capture_close(capture); /* Detached request retains fds; all views expire. */
    CHECK(cc_request_get_status(done) == CC_REQUEST_CANCELLED && !cc_request_get_result(done)->raw.num_planes);
    for (size_t i = 0; i < 3; ++i) OK(cc_request_destroy(requests[i]));
    finished(&f);
}

static void test_imported_feedback(bool embedded)
{
    struct fake f; init(&f); f.pisp = f.cfe = true; f.exposure = 20;
    f.embedded = f.af_lens = embedded;
    cc_capture *capture = pipeline_requests_mode(&f, 7);
    OK(cc_capture_configure_sensor_defaults(capture));
    cc_ipa_reference_tuning tuning = {.flags=CC_IPA_REFERENCE_AE|CC_IPA_REFERENCE_AWB,
        .target_luminance=0.18,.exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4};
    cc_ipa *ipa; OK(create_reference(&tuning, &ipa)); OK(cc_capture_set_ipa(capture, &ipa));
    cc_request *request, *done; OK(cc_request_create(88, &request)); bind_images(&f, request, 7, 0);
    for (unsigned int i = 0; i < 18; ++i) {
        OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &done, 0));
        const cc_capture_result *r = cc_request_get_result(done);
        check_imported_result(&f, capture, done, 7, 0);
        CHECK((r->embedded.num_planes != 0) == embedded);
        if (embedded) CHECK(r->embedded.planes[0].data);
        CHECK((r->ipa.flags & (CC_IPA_METADATA_AE|CC_IPA_METADATA_AWB)) == (CC_IPA_METADATA_AE|CC_IPA_METADATA_AWB));
        CHECK(*(const unsigned char *)f.nodes[9].buffers[0].data == cfe_effective(&f, r->raw.sequence, V4L2_CID_EXPOSURE, 2, 20));
        if (i > 12) CHECK((r->ipa.flags & (CC_IPA_AE_CONVERGED|CC_IPA_AWB_CONVERGED)) == (CC_IPA_AE_CONVERGED|CC_IPA_AWB_CONVERGED));
        OK(cc_capture_release(capture, done));
    }
    CHECK(f.exposure >= 43 && f.exposure <= 47);
    cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
}

static void test_request_buffer_failures(void)
{
    for (int fault = 0; fault < 5; ++fault) {
        struct fake f; init(&f); f.pisp = true; f.fresh_exports = true;
        f.create_error = fault == 1 ? EMFILE : fault == 4 ? EINVAL : EIO;
        if (fault == 0 || fault == 2) {
            f.fail_req = VIDIOC_EXPBUF; f.fail_node = fault == 0 ? 4 : 2; f.fail_nth = 2; f.fail_errno = EIO;
        }
        if (fault == 1) f.fail_duplicate = 1;
        if (fault == 3) f.fail_identity = 2;
        CHECK(!pipeline_requests_mode(&f, fault == 4 ? 8U : CC_CAPTURE_REQUEST_OUTPUT0)); finished(&f);
    }
    for (int failure = 1; failure <= 8; ++failure) {
        struct fake f; init(&f); f.pisp = f.cfe = true;
        cc_capture *capture = pipeline_requests_mode(&f, 7);
        cc_request *request, *waiting, *done;
        OK(cc_request_create(1, &request)); OK(cc_request_create(2, &waiting));
        bind_images(&f, request, 7, 0); bind_images(&f, waiting, 7, 1);
        OK(cc_capture_queue(capture, request)); OK(cc_capture_queue(capture, waiting));
        f.fail_duplicate = f.duplicates + failure;
        ERROR(cc_capture_dispatch(capture), EMFILE);
        OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == request);
        CHECK(cc_request_get_result(done)->status == CC_REQUEST_FAILED && !cc_request_get_result(done)->raw.num_planes &&
            !cc_request_get_result(done)->output[0].num_planes);
        OK(cc_capture_release(capture, done)); OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == waiting);
        CHECK(cc_request_get_result(done)->status == CC_REQUEST_CANCELLED); OK(cc_capture_release(capture, done));
        OK(cc_capture_stop(capture)); f.fail_duplicate = 0;
        OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &done, 0));
        check_imported_result(&f, capture, done, 7, 0); OK(cc_capture_release(capture, done));
        cc_capture_close(capture); OK(cc_request_destroy(request)); OK(cc_request_destroy(waiting)); finished(&f);
    }
    {
        struct fake f; init(&f); f.pisp = true;
        cc_capture *capture = pipeline_requests_mode(&f, 7);
        cc_request *request, *done; OK(cc_request_create(1, &request)); bind_images(&f, request, 7, 0);
        f.fail_duplicate = f.duplicates + 2;
        ERROR(try_bind_external(&f, request, 2, 1, false), EMFILE); f.fail_duplicate = 0;
        OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &done, 0));
        check_imported_result(&f, capture, done, 7, 0); OK(cc_capture_release(capture, done));
        cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
    }
    const unsigned long requests[] = {VIDIOC_QBUF, VIDIOC_STREAMON, VIDIOC_DQBUF};
    const int nodes[] = {0,1,2,3,4,5,6};
    for (size_t r = 0; r < sizeof(requests)/sizeof(requests[0]); ++r)
        for (size_t n = 0; n < sizeof(nodes)/sizeof(nodes[0]); ++n) {
            struct fake f; init(&f); f.pisp = f.cfe = true;
            cc_capture *capture = pipeline_requests_mode(&f, 7);
            cc_request *request, *waiting, *done; OK(cc_request_create(1, &request)); OK(cc_request_create(2, &waiting));
            bind_images(&f, request, 7, 0); bind_images(&f, waiting, 7, 1);
            OK(cc_capture_queue(capture, request)); OK(cc_capture_queue(capture, waiting));
            f.fail_req = requests[r]; f.fail_node = nodes[n]; f.fail_nth = 1; f.fail_errno = EIO;
            ERROR(cc_capture_dispatch(capture), EIO);
            OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == request && cc_request_get_status(done) == CC_REQUEST_FAILED);
            CHECK(!cc_request_get_result(done)->raw.num_planes && !cc_request_get_result(done)->output[1].num_planes);
            OK(cc_capture_release(capture, done)); OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == waiting);
            CHECK(cc_request_get_status(done) == CC_REQUEST_CANCELLED); OK(cc_capture_release(capture, done));
            cc_capture_close(capture); OK(cc_request_destroy(request)); OK(cc_request_destroy(waiting)); finished(&f);
        }
    {
        struct fake f; init(&f); f.pisp = f.cfe = true;
        cc_capture *capture = pipeline_requests_mode(&f, 7);
        cc_request *request, *done; OK(cc_request_create(1, &request)); bind_images(&f, request, 7, 0);
        OK(cc_capture_queue(capture, request));
        f.fail_req = VIDIOC_DQBUF; f.fail_node = 3; f.fail_nth = 1; f.fail_errno = EIO; f.fail_stop = true;
        CHECK(cc_capture_dispatch(capture) < 0);
        for (int i = 0; i < 7; ++i) CHECK(!f.nodes[i].open);
        OK(cc_capture_dequeue(capture, &done, 0)); CHECK(cc_request_get_status(done) == CC_REQUEST_FAILED);
        CHECK(!cc_request_get_result(done)->output[0].num_planes);
        cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
    }
    {
        struct fake f; init(&f); f.fresh_exports = true;
        cc_capture *capture = pipeline_requests_mode(&f, CC_CAPTURE_REQUEST_OUTPUT0);
        cc_request *request, *alias_request, *done; OK(cc_request_create(0, &request)); OK(cc_request_create(1, &alias_request));
        const struct cc_io io = {&ops, &f};
        bind_images(&f, request, CC_CAPTURE_REQUEST_OUTPUT0, 0);
        OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &done, 0));
        check_imported_result(&f, capture, done, CC_CAPTURE_REQUEST_OUTPUT0, 0);
        cc_dmabuf_plane alias = {cc_capture_export_plane(capture, done, CC_CAPTURE_RAW, 0), f.nodes[4].format.planes[0].size};
        CHECK(alias.fd >= 100);
        OK(cc_request_set_buffer_io(alias_request, CC_CAPTURE_OUTPUT0, &alias, 1, &io)); OK(close_fd(&f, alias.fd));
        ERROR(cc_capture_queue(capture, alias_request), EEXIST); /* Cannot overwrite an internal source pool. */
        alias.fd = cc_capture_export_plane(capture, done, CC_CAPTURE_STATISTICS, 0);
        alias.length = f.nodes[1].format.planes[0].size;
        CHECK(alias.fd >= 100);
        OK(cc_request_set_buffer_io(alias_request, CC_CAPTURE_OUTPUT0, &alias, 1, &io)); OK(close_fd(&f, alias.fd));
        ERROR(cc_capture_queue(capture, alias_request), EEXIST); /* Internal metadata is reserved too. */
        OK(cc_capture_release(capture, done)); cc_capture_close(capture);
        OK(cc_request_destroy(request)); OK(cc_request_destroy(alias_request)); finished(&f);
    }
}

static void test_real_buffer_descriptors(void)
{
    FILE *file = tmpfile(); CHECK(file);
    int fd = fileno(file); CHECK(fd >= 0);
    cc_dmabuf_plane planes[2] = {{fd, 4096}, {fd, 4096}};
    struct cc_request_buffer buffer = {0};
    OK(cc_request_buffer_copy(&buffer, planes, 1, &cc_system_io));
    int owned = buffer.planes[0].fd; CHECK(owned != fd && (fcntl(owned, F_GETFD) & FD_CLOEXEC));
    ERROR(cc_request_buffer_copy(&buffer, planes, 2, &cc_system_io), EEXIST);
    CHECK(buffer.planes[0].fd == owned && buffer.count == 1);
    cc_request *request; OK(cc_request_create(0, &request));
    OK(cc_request_set_buffer(request, CC_CAPTURE_RAW, planes, 1));
    ERROR(cc_request_set_buffer(request, CC_CAPTURE_OUTPUT0, planes, 1), EEXIST);
    ERROR(cc_request_set_buffer(request, CC_CAPTURE_STATISTICS, planes, 1), EINVAL);
    planes[0].fd = -1; ERROR(cc_request_set_buffer(request, CC_CAPTURE_RAW, planes, 1), EINVAL);
    OK(fclose(file)); CHECK(fcntl(owned, F_GETFD) >= 0);
    cc_request_buffer_clear(&buffer); CHECK(fcntl(owned, F_GETFD) < 0 && errno == EBADF);
    OK(cc_request_set_buffer(request, CC_CAPTURE_RAW, NULL, 0)); OK(cc_request_destroy(request));
}

static void set_fence(struct fake *f, cc_request *request, unsigned int stream, int object)
{
    const struct cc_io io = {&ops, f}; int fd = fence_descriptor(f, object);
    OK(cc_request_set_acquire_fence_io(request, stream, fd, &io)); OK(close_fd(f, fd));
}
static void test_acquire_fences(bool pisp, bool cfe, bool imported)
{
    struct fake f; init(&f); f.pisp = pisp; f.cfe = cfe;
    cc_capture *capture = pipeline_requests_mode(&f, imported ? 7U : 0);
    cc_request *request, *done = NULL; OK(cc_request_create(123, &request));
    if (imported) bind_images(&f, request, 7, 0);
    for (unsigned int i = 0; i < 3; ++i) set_fence(&f, request, i, (int)i);
    OK(cc_capture_queue(capture, request)); OK(cc_capture_dispatch(capture)); CHECK(!f.starts);
    ERROR(cc_request_set_acquire_fence(request, CC_CAPTURE_RAW, -1), EBUSY);
    ERROR(cc_request_set_fence_timeout(request, 1), EBUSY);
    ERROR(cc_request_take_acquire_fence(request, CC_CAPTURE_RAW), EBUSY);
    struct pollfd fds[8], before[8]; memset(fds, 0xa5, sizeof(fds)); memcpy(before, fds, sizeof(fds));
    size_t n = 1; ERROR(cc_capture_pollfds(capture, fds, &n), ENOSPC);
    CHECK(n == 3 && !memcmp(fds, before, sizeof(fds)));
    n = 8; OK(cc_capture_pollfds(capture, fds, &n)); CHECK(n == 3);
    for (size_t i = 0; i < n; ++i) CHECK(f.fds[fds[i].fd].fence && fds[i].events == POLLIN);
    int timeout; OK(cc_capture_get_timeout(capture, &timeout)); CHECK(timeout == 300);
    f.fences[0].status = 1; OK(cc_capture_dispatch(capture)); CHECK(!f.starts && !f.fences[0].refs);
    n = 8; OK(cc_capture_pollfds(capture, fds, &n)); CHECK(n == 2);
    f.fences[1].signal_at = UINT64_C(1000000); f.fences[2].signal_at = UINT64_C(2000000); f.interrupt_waits = 1;
    ERROR(cc_capture_dequeue(capture, &done, 1), ETIMEDOUT);
    CHECK(!done && !f.starts && cc_request_get_status(request) == CC_REQUEST_PENDING);
    CHECK(!f.fences[1].refs && f.fences[2].refs == 1);
    OK(cc_capture_get_timeout(capture, &timeout)); CHECK(timeout == 299);
    OK(cc_capture_dequeue(capture, &done, 10)); CHECK(done == request && cc_request_get_status(done) == CC_REQUEST_COMPLETE);
    if (imported) check_imported_result(&f, capture, done, 7, 0);
    for (int i = 0; i < 3; ++i) { CHECK(!f.fences[i].refs); ERROR(cc_request_take_acquire_fence(done, (enum cc_capture_stream)i), ENOENT); }
    OK(cc_capture_get_timeout(capture, &timeout)); CHECK(timeout == -1);
    OK(cc_capture_release(capture, done));
    int queries = f.fence_queries;
    OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &done, 0));
    CHECK(f.fence_queries == queries); OK(cc_capture_release(capture, done)); /* Fences are one-shot. */
    set_fence(&f, request, CC_CAPTURE_RAW, 3); OK(cc_capture_queue(capture, request));
    OK(cc_capture_stop(capture)); OK(cc_capture_dequeue(capture, &done, 0));
    CHECK(cc_request_get_status(done) == CC_REQUEST_CANCELLED && !cc_request_get_result(done)->raw.num_planes);
    int taken = cc_request_take_acquire_fence(done, CC_CAPTURE_RAW); CHECK(taken >= 100); OK(close_fd(&f, taken));
    OK(cc_capture_release(capture, done)); OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &done, 0));
    CHECK(cc_request_get_status(done) == CC_REQUEST_COMPLETE); OK(cc_capture_release(capture, done));
    cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
}

static void test_fence_deadlines(void)
{
    struct fake f; init(&f); f.pisp = f.cfe = true;
    cc_capture *capture = pipeline_requests(&f); cc_request *r[4], *done;
    for (size_t i = 0; i < 4; ++i) OK(cc_request_create(100 + i, &r[i]));
    set_fence(&f, r[1], CC_CAPTURE_RAW, 0); OK(cc_request_set_fence_timeout(r[1], 5));
    f.fences[1].status = -EIO; set_fence(&f, r[2], CC_CAPTURE_OUTPUT0, 1);
    for (size_t i = 0; i < 4; ++i) OK(cc_capture_queue(capture, r[i]));
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == r[2]);
    CHECK(cc_request_get_status(done) == CC_REQUEST_CANCELLED && cc_request_get_result(done)->error == -EIO);
    CHECK(!cc_request_get_result(done)->raw.num_planes && !cc_request_get_result(done)->output[0].num_planes);
    OK(cc_capture_release(capture, done)); ERROR(cc_capture_queue(capture, done), EEXIST);
    int fd = cc_request_take_acquire_fence(done, CC_CAPTURE_OUTPUT0); CHECK(fd >= 100); OK(close_fd(&f, fd));
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == r[0]); CHECK(cc_request_get_status(done) == CC_REQUEST_COMPLETE);
    const cc_capture_result *held = cc_request_get_result(done);
    f.now = UINT64_C(4999000); int timeout;
    OK(cc_capture_get_timeout(capture, &timeout)); CHECK(timeout == 1);
    OK(cc_capture_dequeue(capture, &done, 10)); CHECK(done == r[1]);
    CHECK(cc_request_get_status(done) == CC_REQUEST_CANCELLED && cc_request_get_result(done)->error == -ETIMEDOUT);
    CHECK(!cc_request_get_result(done)->raw.num_planes && held->status == CC_REQUEST_COMPLETE && held->raw.num_planes);
    OK(cc_capture_release(capture, done)); f.fences[0].status = 1;
    ERROR(cc_capture_queue(capture, done), EEXIST); /* Explicit recovery, even if it signals later. */
    OK(cc_request_set_acquire_fence(done, CC_CAPTURE_RAW, -1)); OK(cc_capture_queue(capture, done));
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == r[3] && cc_request_get_status(done) == CC_REQUEST_COMPLETE);
    cc_request *blocked = NULL; ERROR(cc_capture_dequeue(capture, &blocked, 10), EAGAIN); CHECK(!blocked);
    OK(cc_capture_release(capture, done)); OK(cc_capture_dequeue(capture, &done, 0));
    CHECK(done == r[1] && cc_request_get_status(done) == CC_REQUEST_COMPLETE);
    OK(cc_capture_release(capture, done)); OK(cc_capture_release(capture, r[0]));
    cc_capture_close(capture); for (size_t i = 0; i < 4; ++i) OK(cc_request_destroy(r[i])); finished(&f);
}

static void test_fence_timeout_during_processing(void)
{
    struct fake f; init(&f); f.pisp = f.cfe = true;
    cc_capture *capture = pipeline_requests_mode(&f, 7); cc_request *r[3], *done;
    for (int i = 0; i < 3; ++i) { OK(cc_request_create((uint64_t)i, &r[i])); bind_images(&f, r[i], 7, i); }
    set_fence(&f, r[1], CC_CAPTURE_OUTPUT0, 0); OK(cc_request_set_fence_timeout(r[1], 5));
    for (size_t i = 0; i < 3; ++i) OK(cc_capture_queue(capture, r[i]));
    f.delayed_port = 3; OK(cc_capture_dispatch(capture));
    CHECK(cc_request_get_status(r[0]) == CC_REQUEST_PENDING && f.nodes[0].streaming);
    struct pollfd fds[8]; size_t n = 8; OK(cc_capture_pollfds(capture, fds, &n)); CHECK(n == 2);
    OK(cc_capture_dequeue(capture, &done, 10)); CHECK(done == r[1]);
    CHECK(cc_request_get_result(done)->error == -ETIMEDOUT && !cc_request_get_result(done)->raw.num_planes);
    CHECK(f.nodes[0].open && f.nodes[0].streaming && cc_request_get_status(r[0]) == CC_REQUEST_PENDING);
    CHECK(cc_request_get_status(r[2]) == CC_REQUEST_PENDING && f.cfe_processed == 2 && f.nodes[3].buffers[0].queued);
    for (int stream = 0; stream < 3; ++stream) for (int p = 0; p < 3; ++p) {
        const struct buffer *b = buffer_plane(&f.nodes[9 + stream], 1, p);
        for (uint32_t i = 0; i < b->size; ++i) CHECK(((const unsigned char *)b->data)[i] == 0xa5);
    }
    OK(cc_capture_release(capture, done)); f.delayed_port = -1;
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == r[0]); check_imported_result(&f, capture, done, 7, 0);
    OK(cc_capture_release(capture, done));
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == r[2]); check_imported_result(&f, capture, done, 7, 2);
    OK(cc_capture_release(capture, done));
    cc_capture_close(capture); for (size_t i = 0; i < 3; ++i) OK(cc_request_destroy(r[i])); finished(&f);
}

static void test_fence_disabled_timeout(void)
{
    struct fake f; init(&f); cc_capture *capture = pipeline_requests(&f);
    cc_request *request, *done = NULL; OK(cc_request_create(1, &request));
    set_fence(&f, request, CC_CAPTURE_OUTPUT1, 0); OK(cc_request_set_fence_timeout(request, 0));
    OK(cc_capture_queue(capture, request)); int timeout;
    OK(cc_capture_get_timeout(capture, &timeout)); CHECK(timeout == -1);
    ERROR(cc_capture_dequeue(capture, &done, 3), ETIMEDOUT); CHECK(!done && !f.starts && f.fences[0].refs == 1);
    f.now = UINT64_C(500000000); OK(cc_capture_dispatch(capture)); CHECK(cc_request_get_status(request) == CC_REQUEST_PENDING);
    f.fences[0].signal_at = f.now + UINT64_C(10000000);
    OK(cc_capture_dequeue(capture, &done, -1)); CHECK(done == request && cc_request_get_status(done) == CC_REQUEST_COMPLETE);
    OK(cc_capture_release(capture, done));
    set_fence(&f, request, CC_CAPTURE_RAW, 1); OK(cc_capture_queue(capture, request)); cc_capture_close(capture);
    CHECK(cc_request_get_status(request) == CC_REQUEST_CANCELLED && f.fences[1].refs == 1);
    int fd = cc_request_take_acquire_fence(request, CC_CAPTURE_RAW); CHECK(fd >= 100); OK(close_fd(&f, fd));
    OK(cc_request_destroy(request)); finished(&f);
}

static void test_fence_capacity_and_clocks(void)
{
    struct fake f; init(&f); cc_capture *capture = pipeline_requests(&f);
    cc_request *requests[256]; const struct cc_io io = {&ops, &f};
    int original[3]; for (int i = 0; i < 3; ++i) original[i] = fence_descriptor(&f, i);
    for (size_t i = 0; i < 256; ++i) {
        OK(cc_request_create(i, &requests[i]));
        for (unsigned int stream = 0; stream < 3; ++stream)
            OK(cc_request_set_acquire_fence_io(requests[i], stream, original[stream], &io));
        OK(cc_capture_queue(capture, requests[i]));
    }
    for (size_t i = 0; i < 3; ++i) OK(close_fd(&f, original[i]));
    OK(cc_capture_dispatch(capture)); CHECK(!f.starts);
    size_t n = 0; OK(cc_capture_pollfds(capture, NULL, &n)); CHECK(n == 768);
    struct pollfd fds[768]; n = 768; OK(cc_capture_pollfds(capture, fds, &n)); CHECK(n == 768);
    f.now = UINT64_C(300000000); OK(cc_capture_dispatch(capture)); CHECK(!f.starts);
    n = 0; OK(cc_capture_pollfds(capture, NULL, &n)); CHECK(n == 0);
    for (size_t i = 0; i < 256; ++i) {
        cc_request *done; OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == requests[i]);
        CHECK(cc_request_get_status(done) == CC_REQUEST_CANCELLED && cc_request_get_result(done)->error == -ETIMEDOUT);
        OK(cc_capture_release(capture, done)); ERROR(cc_capture_queue(capture, done), EEXIST);
    }
    cc_capture_close(capture); for (size_t i = 0; i < 256; ++i) OK(cc_request_destroy(requests[i])); finished(&f);
    init(&f); capture = pipeline_requests(&f); cc_request *request, *done;
    OK(cc_request_create(1, &request)); set_fence(&f, request, CC_CAPTURE_RAW, 0);
    f.clock_error = true; ERROR(cc_capture_queue(capture, request), EIO); CHECK(cc_request_get_status(request) == CC_REQUEST_IDLE);
    f.clock_error = false; f.now = UINT64_MAX - 100;
    OK(cc_request_set_fence_timeout(request, INT_MAX)); OK(cc_capture_queue(capture, request));
    int timeout = 99; OK(cc_capture_get_timeout(capture, &timeout)); CHECK(timeout == 1);
    f.clock_error = true; timeout = 99; ERROR(cc_capture_get_timeout(capture, &timeout), EIO); CHECK(timeout == 99);
    f.clock_error = false; f.now = UINT64_MAX;
    OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == request && cc_request_get_result(done)->error == -ETIMEDOUT);
    cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
}

static void test_fence_primitives(void)
{
    struct fake f; init(&f); struct cc_fence fence = {0}; const struct cc_io io = {&ops, &f};
    int original = fence_descriptor(&f, 0); OK(cc_fence_set(&fence, original, &io)); OK(close_fd(&f, original));
    int retained = fence.fd; original = fence_descriptor(&f, 1);
    f.fail_duplicate = f.duplicates + 1; ERROR(cc_fence_set(&fence, original, &io), EMFILE); f.fail_duplicate = 0;
    CHECK(fence.fd == retained && f.fences[0].refs == 1 && f.fences[1].refs == 1);
    for (uint32_t malformed = 1; malformed <= 8; malformed *= 2) {
        f.fences[1].malformed = malformed; ERROR(cc_fence_set(&fence, original, &io), EPROTO);
        CHECK(fence.fd == retained && f.fences[1].refs == 1);
    }
    f.fences[1].malformed = 0; f.fence_query_error = -ENOTTY; ERROR(cc_fence_set(&fence, original, &io), EINVAL);
    f.fence_query_error = -EIO; ERROR(cc_fence_set(&fence, original, &io), EIO); f.fence_query_error = 0;
    f.fences[0].status = INT32_MIN; ERROR(cc_fence_update(&fence), EPROTO); CHECK(fence.valid);
    f.fences[0].status = -ECANCELED; ERROR(cc_fence_update(&fence), ECANCELED); CHECK(fence.valid);
    f.fences[0].status = 1; CHECK(cc_fence_update(&fence) == 1 && !fence.valid && !f.fences[0].refs);
    OK(cc_fence_set(&fence, original, &io)); OK(close_fd(&f, original));
    ERROR(cc_fence_set(&fence, -2, &io), EINVAL); OK(cc_fence_set(&fence, -1, NULL));
    ERROR(cc_fence_take(&fence), ENOENT); CHECK(!f.fences[1].refs); finished(&f);
    FILE *file = tmpfile(); CHECK(file);
    ERROR(cc_fence_set(&fence, fileno(file), &cc_system_io), EINVAL); CHECK(!fence.valid); OK(fclose(file));
    cc_request *request; OK(cc_request_create(1, &request)); ERROR(cc_request_set_fence_timeout(request, -1), EINVAL);
    ERROR(cc_request_set_acquire_fence(request, CC_CAPTURE_STATISTICS, -1), EINVAL);
    ERROR(cc_request_take_acquire_fence(request, CC_CAPTURE_EMBEDDED), EINVAL); OK(cc_request_destroy(request));
    init(&f); cc_camera *camera = NULL; cc_camera_info camera_info = {.pipeline=CC_PIPELINE_V4L2,.video_path="/dev/video4"};
    OK(cc_camera_open_io(&camera_info, &io, &camera)); cc_format format = raw_format(); OK(cc_camera_configure(camera, &format));
    uint32_t count = 3; OK(cc_camera_allocate_buffers(camera, &count)); cc_capture_devices devices = {.camera=camera};
    cc_capture *capture; OK(cc_capture_create(&devices, &capture, &count)); OK(cc_request_create(1, &request));
    set_fence(&f, request, CC_CAPTURE_OUTPUT0, 0); ERROR(cc_capture_queue(capture, request), ENOTSUP);
    int fd = cc_request_take_acquire_fence(request, CC_CAPTURE_OUTPUT0); CHECK(fd >= 100); OK(close_fd(&f, fd));
    set_fence(&f, request, CC_CAPTURE_RAW, 1); f.fences[1].status = 1; OK(cc_capture_queue(capture, request));
    cc_request *done; OK(cc_capture_dequeue(capture, &done, 0)); CHECK(done == request && cc_request_get_status(done) == CC_REQUEST_COMPLETE);
    OK(cc_capture_release(capture, done)); cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
}

static cc_capture *cfe_feedback_graph_mode(struct fake *f, uint32_t compression)
{
    init(f); f->pisp=f->cfe=true; f->exposure=20; f->compression=compression;
    cc_capture *c=pipeline_requests(f);
    OK(cc_capture_configure_sensor_defaults(c));
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AE|CC_IPA_REFERENCE_AWB,
        .target_luminance=0.18,.exposure_speed=0.5,.white_balance_speed=0.5,
        .min_colour_gain=0.5,.max_colour_gain=4};
    cc_ipa *ipa;
    OK(create_reference(&tuning,&ipa)); OK(cc_capture_set_ipa(c,&ipa)); CHECK(!ipa);
    return c;
}

static cc_capture *cfe_feedback_graph(struct fake *f) { return cfe_feedback_graph_mode(f,0); }

static void check_cfe_feedback(const struct fake *f, const cc_capture_result *r)
{
    CHECK(r->status==CC_REQUEST_COMPLETE && !r->error);
    CHECK(r->raw.sequence==r->statistics.sequence && r->raw.sequence==r->frontend_parameters.sequence);
    CHECK(r->raw.timestamp_ns==r->statistics.timestamp_ns && r->raw.timestamp_ns==r->output[0].timestamp_ns &&
          r->raw.timestamp_ns==r->output[1].timestamp_ns);
    CHECK(r->statistics.planes[0].bytes_used==sizeof(struct pisp_statistics));
    CHECK(r->output[0].num_planes==2 && r->output[1].num_planes==3);
    CHECK((r->ipa.flags & (CC_IPA_METADATA_AE|CC_IPA_METADATA_AWB))==(CC_IPA_METADATA_AE|CC_IPA_METADATA_AWB));
    int32_t exposure=cfe_effective(f,r->raw.sequence,V4L2_CID_EXPOSURE,2,20);
    CHECK(*(const unsigned char *)r->raw.planes[0].data==exposure && r->sensor.cookie==r->cookie);
    for (uint32_t i=0;i<r->sensor.count;++i)
        if (r->sensor.controls[i].id==V4L2_CID_EXPOSURE) CHECK(r->sensor.controls[i].value==exposure);
    const struct pisp_be_tiles_config *be=r->backend_parameters.planes[0].data;
    const struct pisp_fe_config *fe=r->frontend_parameters.planes[0].data;
    CHECK(be && fe && (be->config.global.bayer_enables & PISP_BE_BAYER_ENABLE_WBG));
    if (f->compression) {
        CHECK(fe->ch[0].compress.mode==f->compression && fe->ch[0].compress.offset==2048);
        CHECK(be->config.decompress.mode==f->compression && be->config.decompress.offset==2048);
        CHECK(fe->input.format.format==PISP_IMAGE_FORMAT_BPS_16 &&
              fe->ch[0].output.format.format==f->compression*PISP_IMAGE_FORMAT_COMPRESSION_MODE_1);
        CHECK(be->config.input_format.stride==128 && fe->ch[0].output.format.stride==128 &&
              r->raw.planes[0].length==128*96);
    }
    CHECK(r->ipa.red_gain==(double)be->config.wbg.gain_r/be->config.wbg.gain_g &&
          r->ipa.blue_gain==(double)be->config.wbg.gain_b/be->config.wbg.gain_g);
    CHECK(fe->rgby.gain_r==(uint16_t)(306.0*be->config.wbg.gain_r/1024) ||
          fe->rgby.gain_r==(uint16_t)(306.0*be->config.wbg.gain_r/1024)+1);
}

static void test_cfe_be_feedback(uint32_t compression)
{
    struct fake f; cc_capture *c=cfe_feedback_graph_mode(&f,compression);
    cc_request *requests[3],*out;
    for (size_t i=0;i<3;++i) OK(cc_request_create(123+i,&requests[i]));
    for (unsigned int i=0;i<18;++i) {
        OK(cc_capture_queue(c,requests[0])); OK(cc_capture_dequeue(c,&out,0));
        CHECK(out==requests[0]);
        const cc_capture_result *r=cc_request_get_result(out);
        check_cfe_feedback(&f,r);
        if (i>12) CHECK((r->ipa.flags & (CC_IPA_AE_CONVERGED|CC_IPA_AWB_CONVERGED))==
            (CC_IPA_AE_CONVERGED|CC_IPA_AWB_CONVERGED));
        OK(cc_capture_release(c,out));
    }
    CHECK(f.exposure>=43 && f.exposure<=47 && f.analogue_gain<10);
    for (size_t i=0;i<3;++i) OK(cc_capture_queue(c,requests[i]));
    f.delayed_port=2; /* BE params wait while the next CFE request captures. */
    out=NULL; ERROR(cc_capture_dequeue(c,&out,3),ETIMEDOUT);
    CHECK(!out && f.capture_overlap>0);
    struct pollfd fds[8]; size_t n=8; OK(cc_capture_pollfds(c,fds,&n));
    CHECK(n==2 && fds[0].fd==22 && fds[0].events==POLLOUT && fds[1].fd==24 && fds[1].events==POLLPRI);
    f.delayed_port=-1;
    for (size_t i=0;i<3;++i) {
        OK(cc_capture_dequeue(c,&out,0)); CHECK(out==requests[i]);
        check_cfe_feedback(&f,cc_request_get_result(out));
        if (i) OK(cc_capture_release(c,out));
    }
    const cc_capture_result *held=cc_request_get_result(requests[0]);
    struct pisp_fe_config fe=*(const struct pisp_fe_config *)held->frontend_parameters.planes[0].data;
    struct pisp_be_config be=((const struct pisp_be_tiles_config *)held->backend_parameters.planes[0].data)->config;
    fe.rgby.gain_r=306; fe.rgby.gain_g=601; fe.rgby.gain_b=117;
    fe.global.enables &= ~(uint32_t)(PISP_FE_ENABLE_INPUT|PISP_FE_ENABLE_OUTPUT0|PISP_FE_ENABLE_COMPRESS0);
    /* The public tuning API accepts processing blocks; IO is regenerated. */
    be.global.bayer_enables=PISP_BE_BAYER_ENABLE_WBG;
    be.global.rgb_enables=0;
    be.wbg=(struct pisp_wbg_config){.gain_r=1536,.gain_g=1024,.gain_b=2048};
    cc_control exposure={.id=V4L2_CID_EXPOSURE,.value=10};
    OK(cc_request_set_sensor_controls(requests[1],&exposure,1));
    OK(cc_request_set_tuning(requests[1],&fe,&be));
    fe.rgby.gain_r=0; be.wbg.gain_r=0;
    for (unsigned int i=0;i<2;++i) {
        OK(cc_capture_queue(c,requests[1])); OK(cc_capture_dequeue(c,&out,0));
        const cc_capture_result *r=cc_request_get_result(out);
        CHECK(r->status==CC_REQUEST_COMPLETE && r->ipa.red_gain==1.5 && r->ipa.blue_gain==2);
        CHECK(((const struct pisp_fe_config *)r->frontend_parameters.planes[0].data)->rgby.gain_r==306);
        CHECK(*(const unsigned char *)r->raw.planes[0].data==10);
        CHECK(held->ipa.flags & CC_IPA_AE_CONVERGED); /* Held views/metadata stay private. */
        OK(cc_capture_release(c,out));
    }
    int exported=cc_capture_export_plane(c,requests[0],CC_CAPTURE_RAW,0);
    CHECK(exported>=0); OK(cc_capture_stop(c)); CHECK(!held->raw.num_planes && !held->ipa.flags);
    OK(cc_capture_release(c,requests[0])); OK(close_fd(&f,exported));
    OK(cc_request_set_tuning(requests[1],NULL,NULL)); OK(cc_request_set_sensor_controls(requests[1],NULL,0));
    /* Restart re-reads live sensor controls and restarts automatic WB. */
    OK(cc_capture_queue(c,requests[1])); OK(cc_capture_dequeue(c,&out,0));
    check_cfe_feedback(&f,cc_request_get_result(out));
    CHECK(cc_request_get_result(out)->ipa.red_gain==1 && cc_request_get_result(out)->ipa.blue_gain==1);
    OK(cc_capture_release(c,out));
    cc_capture_close(c); for (size_t i=0;i<3;++i) OK(cc_request_destroy(requests[i])); finished(&f);
}

static void test_compression_graph_mismatch(void)
{
    struct fake f; init(&f); f.pisp=f.cfe=f.mismatch_compression_offset=true; f.compression=1;
    CHECK(!pipeline_requests(&f)); finished(&f);
}

static void test_cfe_be_failures(void)
{
    for (unsigned int fault=0;fault<5;++fault) {
        struct fake f; cc_capture *c=cfe_feedback_graph(&f);
        cc_request *requests[3],*out;
        for (size_t i=0;i<3;++i) { OK(cc_request_create(i,&requests[i])); OK(cc_capture_queue(c,requests[i])); }
        f.delayed_port=fault==2 ? 5 : 2;
        OK(cc_capture_dispatch(c));
        if (fault!=2) CHECK(f.capture_overlap>0);
        if (fault==0) f.bad_cfe_statistics=true;
        if (fault==1) { f.fail_req=VIDIOC_DQBUF; f.fail_node=2; f.fail_nth=1; f.fail_errno=ENODEV; }
        if (fault==2) f.corrupt_port=5;
        if (fault==3) { f.fail_req=VIDIOC_STREAMOFF; f.fail_node=4; f.fail_nth=1; f.fail_errno=EIO; }
        if (fault==4) f.fail_stop=true;
        if (fault>=3) CHECK(cc_capture_stop(c)<0);
        else {
            f.delayed_port=-1;
            if (fault==0) {
                /* The first two frames already have valid statistics; the
                 * next source job is malformed and must cancel the graph. */
                OK(cc_capture_dequeue(c,&out,0)); CHECK(out==requests[0]);
                OK(cc_capture_release(c,out));
            }
        }
        size_t first=fault==0 ? 1U : 0U;
        for (size_t i=first;i<3;++i) {
            OK(cc_capture_dequeue(c,&out,0)); CHECK(out==requests[i]);
            const cc_capture_result *r=cc_request_get_result(out);
            CHECK(r->status==CC_REQUEST_FAILED || r->status==CC_REQUEST_CANCELLED);
            CHECK(r->error<0 && !r->raw.num_planes && !r->statistics.num_planes && !r->output[0].num_planes &&
                  !r->output[1].num_planes && !r->frontend_parameters.num_planes &&
                  !r->backend_parameters.num_planes && !r->ipa.flags);
            OK(cc_capture_release(c,out));
        }
        cc_capture_close(c); for (size_t i=0;i<3;++i) OK(cc_request_destroy(requests[i])); finished(&f);
    }
}

static void test_pipeline_requests(bool pisp)
{
    struct fake f;
    init(&f); f.pisp = pisp;
    cc_capture *c = pipeline_requests(&f);
    cc_request *requests[3], *r = NULL;
    for (uint64_t i = 0; i < 3; ++i) {
        OK(cc_request_create(500 + i, &requests[i]));
        if (pisp) {
            struct pisp_be_config tuning = {0};
            tuning.global.bayer_enables = PISP_BE_BAYER_ENABLE_WBG;
            tuning.wbg.gain_r = (uint16_t)(8192 + i);
            OK(cc_request_set_tuning(requests[i], NULL, &tuning));
            tuning.wbg.gain_r = 0;
        } else {
            unsigned char payload[8]; memset(payload, (int)(100 + i), sizeof(payload));
            cc_control controls[] = {{.id = V4L2_CID_RED_BALANCE, .value = (int64_t)(1200 + i)},
                                     {.id = PAYLOAD_CONTROL, .data = payload, .size = sizeof(payload)}};
            OK(cc_request_set_isp_controls(requests[i], controls, 2));
            memset(payload, 0, sizeof(payload));
        }
        OK(cc_capture_queue(c, requests[i]));
    }
    f.delayed_port = 4; /* Spend one millisecond capturing, then wait on the ISP. */
    f.switch_delay_at = 1;
    f.next_delayed_port = pisp ? (int)CC_PISP_BE_PARAMETERS : (int)CC_VC4_STATS;
    ERROR(cc_capture_dequeue(c, &r, 3), ETIMEDOUT);
    CHECK(!r && f.capture_overlap > 0 && f.wait_calls == 3);
    CHECK(cc_request_get_status(requests[0]) == CC_REQUEST_PENDING);
    struct pollfd fds[8]; size_t n = 8;
    OK(cc_capture_pollfds(c, fds, &n));
    CHECK(n == 1 && fds[0].fd == 20 + f.delayed_port);
    CHECK(fds[0].events == (pisp ? POLLOUT : POLLIN));
    if (pisp) CHECK(f.last_parameters.config.wbg.gain_r == 8192);
    else CHECK(f.gain == 1200 && f.payload[0] == 100);
    f.delayed_port = -1;
    for (uint64_t i = 0; i < 3; ++i) {
        OK(cc_capture_dequeue(c, &r, 100)); CHECK(r == requests[i]);
        const cc_capture_result *result = cc_request_get_result(r);
        CHECK(result && result->status == CC_REQUEST_COMPLETE && result->cookie == 500 + i && result->sequence == i);
        CHECK(result->raw.planes[0].data && result->output[0].planes[0].data);
        CHECK(result->raw.timestamp_ns == result->output[0].timestamp_ns);
        CHECK(*(const unsigned char *)result->raw.planes[0].data == 0x77);
        if (pisp) {
            const struct pisp_be_tiles_config *p = result->backend_parameters.planes[0].data;
            CHECK(p->config.wbg.gain_r == 8192 + i);
            CHECK(result->output[0].num_planes == 2 && result->output[1].num_planes == 3);
        } else {
            CHECK(result->statistics.planes[0].data);
            CHECK(f.submitted_gain[result->raw.index] == (int32_t)(1200 + i));
            CHECK(f.submitted_payload[result->raw.index][0] == 100 + i);
        }
        int fd = cc_capture_export_plane(c, r, CC_CAPTURE_OUTPUT1, pisp ? 2 : 0);
        CHECK(fd >= 0); OK(close_fd(&f, fd));
        OK(cc_capture_release(c, r));
    }
    OK(cc_capture_stop(c));
    for (size_t i = 0; i < 3; ++i) OK(cc_request_destroy(requests[i]));
    cc_capture_close(c); finished(&f);
}

static void test_sensor_pipeline_requests(bool pisp)
{
    struct fake f;
    init(&f); f.pisp = pisp; f.sensor_support = true; f.exposure = 20;
    cc_capture *c = pipeline_requests(&f);
    cc_sensor_control_delay delay = {V4L2_CID_EXPOSURE,2,0};
    OK(cc_capture_configure_sensor_delays(c, &delay, 1));
    cc_request *requests[3], *r;
    const int64_t requested[] = {120,60,48};
    const unsigned char expected[] = {99,60,48};
    for (size_t i = 0; i < 3; ++i) {
        OK(cc_request_create(700 + i, &requests[i]));
        cc_control exposure = {.id=V4L2_CID_EXPOSURE,.value=requested[i]};
        OK(cc_request_set_sensor_controls(requests[i], &exposure, 1));
        OK(cc_capture_queue(c, requests[i]));
    }
    f.delayed_port = 4; f.switch_delay_at = 1;
    f.next_delayed_port = pisp ? (int)CC_PISP_BE_PARAMETERS : (int)CC_VC4_STATS;
    r = NULL;
    ERROR(cc_capture_dequeue(c, &r, 3), ETIMEDOUT); CHECK(!r);
    CHECK(cc_request_get_status(requests[0]) == CC_REQUEST_PENDING && f.processed_count == 1);
    struct pollfd fds[8]; size_t n = 8;
    OK(cc_capture_pollfds(c, fds, &n));
    CHECK(n == 2); /* Unfinished processor port plus the active SOF subscription. */
    f.delayed_port = -1;
    for (uint32_t i = 0; i < 3; ++i) {
        OK(cc_capture_dequeue(c, &r, 0)); CHECK(r == requests[i]);
        const cc_capture_result *result = cc_request_get_result(r);
        CHECK(result->status == CC_REQUEST_COMPLETE && result->raw.sequence == 2 + i * 3);
        CHECK(result->sensor.cookie == 700 + i && result->sensor.controls[0].value == expected[i]);
        CHECK(*(const unsigned char *)result->raw.planes[0].data == expected[i]);
        CHECK(*(const unsigned char *)result->output[0].planes[0].data == expected[i]);
        OK(cc_capture_release(c, r));
    }
    /* Nine source frames were captured, but only the three settled requests
     * reached hardware processing/DMA import. The fake sensor stamps its
     * actual two-frame-latency exposure into the source bytes independently. */
    CHECK(f.sensor_sequence == 9 && f.processed_count == 3 && f.duplicates == 3);
    CHECK(!memcmp(f.processed_exposures, expected, sizeof(expected)));
    cc_capture_close(c);
    for (size_t i = 0; i < 3; ++i) OK(cc_request_destroy(requests[i]));
    finished(&f);
}

static void create_loaded_ipa(const char *path, bool managed, uint32_t mode, cc_ipa **ipa)
{
    if (managed) {
        const char *slash = strrchr(path, '/'); CHECK(slash && slash > path);
        size_t length = (size_t)(slash - path);
        char *directory = malloc(length + 1); CHECK(directory);
        memcpy(directory, path, length); directory[length] = 0;
        cc_ipa_module_directory d = {directory, 0};
        cc_ipa_manager_options options = {.directories = &d, .num_directories = 1};
        cc_ipa_manager *manager = NULL; OK(cc_ipa_manager_open(&options, &manager)); free(directory);
        if (proxy_worker) {
            cc_ipa_proxy_options proxy_options = {.worker_path = proxy_worker};
            OK(cc_ipa_manager_create_proxy(manager, "TestPipeline", 4, 4, "alternate", &mode, sizeof(mode), &proxy_options, ipa));
        } else OK(cc_ipa_manager_create(manager, "TestPipeline", 4, 4, "alternate", &mode, sizeof(mode), ipa));
        cc_ipa_manager_close(manager);
    } else {
        cc_ipa_module *module = NULL; OK(cc_ipa_module_open(path, &module));
        if (proxy_worker) {
            cc_ipa_proxy_options proxy_options = {.worker_path = proxy_worker};
            OK(cc_ipa_module_create_proxy(module, &mode, sizeof(mode), &proxy_options, ipa));
        } else OK(cc_ipa_module_create(module, &mode, sizeof(mode), ipa));
        cc_ipa_module_close(module);
    }
}
static void test_loaded_ipa(const char *path, bool cfe, bool managed)
{
    struct fake f; init(&f); f.cfe = f.pisp = cfe; f.ipa_scene = true;
    cc_capture *capture = pipeline_requests(&f);
    cc_ipa *ipa = NULL;
    create_loaded_ipa(path, managed, 6, &ipa);
    ERROR(cc_capture_set_ipa(capture, &ipa), EPROTO); CHECK(ipa); cc_ipa_destroy(ipa);
    create_loaded_ipa(path, managed, 7, &ipa);
    OK(cc_capture_set_ipa(capture, &ipa)); CHECK(!ipa);
    cc_request *request, *out; OK(cc_request_create(77, &request));
    for (unsigned run = 0; run < 3; ++run) {
        OK(cc_capture_queue(capture, request)); OK(cc_capture_dequeue(capture, &out, 0));
        const cc_capture_result *result = cc_request_get_result(out);
        if (out != request || result->status != CC_REQUEST_COMPLETE)
            fprintf(stderr, "loaded IPA cfe=%u run=%u status=%u error=%d\n", cfe, run, result->status, result->error);
        CHECK(out == request && result->status == CC_REQUEST_COMPLETE);
        CHECK(result->ipa.flags == CC_IPA_METADATA_AE && result->ipa.exposure_ns == 128);
        OK(cc_capture_release(capture, out)); OK(cc_capture_stop(capture));
    }
    /* Close while another request is owned; callbacks must still be mapped. */
    OK(cc_capture_queue(capture, request)); cc_capture_close(capture);
    CHECK(cc_request_get_status(request) == CC_REQUEST_CANCELLED);
    OK(cc_request_destroy(request)); finished(&f);
}

static void test_ipa_vc4_feedback(void)
{
    struct fake f; init(&f); f.ipa_scene=true; f.blue_gain=f.digital_gain=1000;
    cc_capture *c=pipeline_requests(&f);
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AWB,.target_luminance=0.18,
        .exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4};
    cc_ipa *ipa;
    OK(create_reference(&tuning,&ipa)); OK(cc_capture_set_ipa(c,&ipa)); CHECK(!ipa);
    cc_request *request,*out; OK(cc_request_create(77,&request));
    for (unsigned i=0;i<12;++i) {
        OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
        CHECK(out==request && cc_request_get_result(out)->status==CC_REQUEST_COMPLETE);
        const cc_ipa_metadata *m=&cc_request_get_result(out)->ipa;
        CHECK(m->flags & CC_IPA_METADATA_AWB);
        CHECK(m->red_gain==(double)f.gain/1000 && m->blue_gain==(double)f.blue_gain/1000);
        if (i>7) CHECK(m->flags & CC_IPA_AWB_CONVERGED);
        OK(cc_capture_release(c,out));
    }
    CHECK(f.gain>=1990 && f.gain<=2000 && f.blue_gain>=500 && f.blue_gain<=510 && f.digital_gain>=1960);
    unsigned char payload[8]; memset(payload,77,sizeof(payload));
    cc_control manual[]={{.id=V4L2_CID_RED_BALANCE,.value=1250},{.id=PAYLOAD_CONTROL,.data=payload,.size=sizeof(payload)}};
    OK(cc_request_set_isp_controls(request,manual,2)); memset(payload,0,sizeof(payload));
    for (unsigned i=0;i<2;++i) {
        OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
        CHECK(f.gain==1250 && cc_request_get_result(out)->ipa.red_gain==1.25);
        CHECK(f.payload[0]==77);
        OK(cc_capture_release(c,out));
    }
    cc_capture_close(c); OK(cc_request_destroy(request)); finished(&f);
}

static void test_af_graph(bool cfe)
{
    struct fake f; init(&f); f.af_lens=f.ipa_scene=true; f.cfe=f.pisp=cfe;
    f.exposure=20; f.blue_gain=f.digital_gain=1000; f.focus_peak=4.3;
    cc_capture *c=pipeline_requests(&f);
    if (cfe) OK(cc_capture_configure_sensor_defaults(c));
    cc_af_tuning af; cc_af_reference_tuning(&af);
    af.speeds[0].pdaf_gain=0; af.speeds[1].pdaf_gain=0;
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AF|CC_IPA_REFERENCE_AWB|(cfe ? CC_IPA_REFERENCE_AE:0U),
        .target_luminance=0.18,.exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4,.af=&af};
    cc_ipa *ipa; OK(create_reference(&tuning,&ipa));
    af.map[1].code=999; /* Module owns a tuning copy. */
    OK(cc_capture_set_ipa(c,&ipa)); CHECK(!ipa);
    cc_request *requests[3],*out;
    for (size_t i=0;i<3;++i) OK(cc_request_create(50+i,&requests[i]));
    cc_af_controls controls={.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO};
    OK(cc_request_set_af_controls(requests[0],&controls));
    bool focused=false;
    for (unsigned int i=0;i<180;++i) {
        OK(cc_capture_queue(c,requests[0])); ERROR(cc_request_set_af_controls(requests[0],NULL),EBUSY);
        OK(cc_capture_dequeue(c,&out,0)); CHECK(out==requests[0]);
        const cc_capture_result *r=cc_request_get_result(out);
        CHECK(r->status==CC_REQUEST_COMPLETE && (r->ipa.flags & CC_IPA_METADATA_AF));
        CHECK(r->ipa.af.lens_setting==f.nodes[4].buffers[r->raw.index].lens_setting);
        int32_t immediate=500; ERROR(cc_capture_set_lens_position(c,&immediate),EBUSY);
        focused=r->ipa.af.state==CC_AF_FOCUSED;
        if (focused) {
            CHECK(r->ipa.af.lens_position>4 && r->ipa.af.lens_position<4.6);
            break; /* Hold this complete result while another request moves the lens. */
        }
        OK(cc_capture_release(c,out)); OK(cc_request_set_af_controls(requests[0],NULL));
    }
    CHECK(focused);
    const cc_capture_result *held=cc_request_get_result(requests[0]); int32_t held_code=held->ipa.af.lens_setting;
    controls=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_SET_LENS,.mode=CC_AF_MANUAL,.lens_position=9};
    OK(cc_request_set_af_controls(requests[1],&controls)); controls.lens_position=0;
    for (unsigned int i=0;i<2;++i) {
        OK(cc_capture_queue(c,requests[1])); OK(cc_capture_dequeue(c,&out,0));
        const cc_capture_result *r=cc_request_get_result(out);
        CHECK(r->ipa.af.state==CC_AF_IDLE && r->ipa.af.lens_setting==f.nodes[4].buffers[r->raw.index].lens_setting);
        CHECK(held->ipa.af.lens_setting==held_code && held->raw.num_planes);
        OK(cc_capture_release(c,out));
    }
    CHECK(f.lens_position==733);
    OK(cc_capture_stop(c)); CHECK(!held->ipa.flags && !held->raw.num_planes); OK(cc_capture_release(c,requests[0]));
    int32_t position=500; OK(cc_capture_set_lens_position(c,&position)); position=0;
    OK(cc_capture_get_lens_position(c,&position)); CHECK(position==500);
    OK(cc_capture_queue(c,requests[1])); OK(cc_capture_dequeue(c,&out,0));
    CHECK(cc_request_get_result(out)->ipa.af.lens_setting==733); OK(cc_capture_release(c,out));
    f.fail_lens=true;
    OK(cc_capture_queue(c,requests[1])); OK(cc_capture_queue(c,requests[2]));
    for (size_t i=1;i<3;++i) {
        OK(cc_capture_dequeue(c,&out,0)); CHECK(out==requests[i]);
        const cc_capture_result *r=cc_request_get_result(out);
        CHECK(r->status==(i==1 ? CC_REQUEST_FAILED:CC_REQUEST_CANCELLED) && !r->ipa.flags && !r->raw.num_planes);
        OK(cc_capture_release(c,out));
    }
    cc_capture_close(c); for (size_t i=0;i<3;++i) OK(cc_request_destroy(requests[i])); finished(&f);
}

static void test_embedded_af_graph(void)
{
    struct fake f; init(&f); f.cfe=f.pisp=f.embedded=f.af_lens=f.flat_focus=true; f.focus_peak=4.3; f.exposure=20;
    cc_capture *capture=pipeline_requests(&f);
    const cc_sensor_model *sensor; OK(cc_sensor_lookup("imx708",&sensor));
    cc_embedded_configuration configuration; OK(cc_capture_get_embedded_configuration(capture,&configuration));
    CHECK(configuration.bits_per_pixel==10 && configuration.format.planes[0].stride==1280);
    cc_af_tuning af; cc_af_reference_tuning(&af);
    cc_ipa_reference_tuning tuning={.flags=CC_IPA_REFERENCE_AF,.af=&af,.target_luminance=0.18,
        .exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4};
    cc_ipa *ipa; OK(create_reference(&tuning,&ipa)); OK(cc_capture_set_ipa(capture,&ipa));
    cc_request *requests[3],*out;
    for (size_t i=0;i<3;++i) OK(cc_request_create(100+i,&requests[i]));
    cc_af_controls controls={.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO};
    OK(cc_request_set_af_controls(requests[0],&controls));
    bool focused=false;
    for (unsigned i=0;i<180;++i) {
        OK(cc_capture_queue(capture,requests[0])); OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[0]);
        const cc_capture_result *result=cc_request_get_result(out);
        CHECK(result->status==CC_REQUEST_COMPLETE && result->embedded.num_planes==1);
        CHECK(result->embedded.timestamp_ns==result->raw.timestamp_ns && result->embedded.sequence==result->raw.sequence+5000);
        CHECK((result->ipa.flags&(CC_IPA_METADATA_SENSOR|CC_IPA_METADATA_PDAF))==(CC_IPA_METADATA_SENSOR|CC_IPA_METADATA_PDAF));
        CHECK(result->ipa.sensor_status.exposure_lines==20 && result->ipa.sensor_status.analogue_gain_code==0 &&
            result->ipa.sensor_status.temperature_c==33);
        cc_sensor_pdaf phase;
        OK(cc_sensor_parse_pdaf(sensor,10,(const uint8_t *)result->embedded.planes[0].data+2560,1280,&phase));
        int expected=(int)((result->ipa.af.lens_position-f.focus_peak)*50);
        CHECK(phase.regions[0].phase==expected);
        if (result->ipa.af.state==CC_AF_FOCUSED) { CHECK(result->ipa.af.lens_position>3.95 && result->ipa.af.lens_position<4.65); focused=true; break; }
        OK(cc_capture_release(capture,out)); OK(cc_request_set_af_controls(requests[0],NULL));
    }
    CHECK(focused);
    const cc_capture_result *held=cc_request_get_result(requests[0]);
    unsigned char bytes[5120]; memcpy(bytes,held->embedded.planes[0].data,sizeof(bytes));
    int export=cc_capture_export_plane(capture,requests[0],CC_CAPTURE_EMBEDDED,0); CHECK(export>=100); OK(close_fd(&f,export));
    controls=(cc_af_controls){.flags=CC_AF_SET_MODE|CC_AF_SET_LENS,.mode=CC_AF_MANUAL,.lens_position=9};
    OK(cc_request_set_af_controls(requests[1],&controls));
    f.delayed_port=8; OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_dispatch(capture));
    struct pollfd fds[16]; size_t count=16; OK(cc_capture_pollfds(capture,fds,&count)); CHECK(count==1 && fds[0].fd==28);
    ERROR(cc_capture_dequeue(capture,&out,2),ETIMEDOUT);
    CHECK(!memcmp(bytes,held->embedded.planes[0].data,sizeof(bytes)));
    f.delayed_port=-1; OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[1]);
    CHECK(cc_request_get_result(out)->ipa.af.state==CC_AF_IDLE &&
        cc_request_get_result(out)->ipa.af.lens_setting==f.lens_position); OK(cc_capture_release(capture,out));
    f.bad_embedded=true; OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_dequeue(capture,&out,0));
    CHECK(!(cc_request_get_result(out)->ipa.flags&CC_IPA_METADATA_PDAF) && (cc_request_get_result(out)->ipa.flags&CC_IPA_METADATA_SENSOR) && f.lens_position==733);
    OK(cc_capture_release(capture,out)); f.bad_embedded=false; f.short_embedded=true;
    OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_dequeue(capture,&out,0));
    CHECK(!(cc_request_get_result(out)->ipa.flags&CC_IPA_METADATA_PDAF)); OK(cc_capture_release(capture,out));
    OK(cc_capture_stop(capture)); CHECK(!held->embedded.num_planes && !held->ipa.flags); OK(cc_capture_release(capture,requests[0]));
    f.short_embedded=false; f.fail_node=8; f.fail_req=VIDIOC_DQBUF; f.fail_nth=1; f.fail_errno=EIO;
    OK(cc_capture_queue(capture,requests[1])); OK(cc_capture_queue(capture,requests[2]));
    for (size_t i=1;i<3;++i) {
        OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==requests[i]);
        const cc_capture_result *result=cc_request_get_result(out);
        CHECK(result->status==(i==1 ? CC_REQUEST_FAILED:CC_REQUEST_CANCELLED) && !result->embedded.num_planes && !result->ipa.flags);
        OK(cc_capture_release(capture,out));
    }
    cc_capture_close(capture); for (size_t i=0;i<3;++i) OK(cc_request_destroy(requests[i])); finished(&f);
}

static void test_request_pipeline_failures(bool pisp)
{
    for (int fault = 0; fault < 5; ++fault) {
        struct fake f;
        init(&f); f.pisp = pisp;
        cc_capture *c = pipeline_requests(&f);
        cc_request *requests[3], *r;
        for (size_t i = 0; i < 3; ++i) { OK(cc_request_create(i, &requests[i])); OK(cc_capture_queue(c, requests[i])); }
        f.delayed_port = pisp ? (int)CC_PISP_BE_PARAMETERS : (int)CC_VC4_STATS;
        OK(cc_capture_dispatch(c));
        if (fault == 0) f.corrupt_port = f.delayed_port;
        if (fault == 1) { f.fail_duplicate = f.duplicates + 1; f.delayed_port = -1; }
        if (fault == 2) f.fail_stop = true;
        if (fault == 3) { f.fail_req = VIDIOC_STREAMOFF; f.fail_node = 4; f.fail_nth = 1; f.fail_errno = EIO; }
        if (fault == 4) { f.fail_req = VIDIOC_DQBUF; f.fail_node = f.delayed_port; f.fail_nth = 1; f.fail_errno = ENODEV; }
        if (fault == 2 || fault == 3) CHECK(cc_capture_stop(c) < 0);
        else f.delayed_port = -1;
        for (size_t i = 0; i < 3; ++i) {
            OK(cc_capture_dequeue(c, &r, 10)); CHECK(r == requests[i]);
            const cc_capture_result *result = cc_request_get_result(r);
            /* Duplicate failure occurs on job 1, after job 0 has completed;
             * graph cancellation also invalidates job 0's undelivered views. */
            CHECK(result->error < 0 && !result->raw.planes[0].data && !result->output[0].planes[0].data);
            CHECK(result->status == CC_REQUEST_CANCELLED || result->status == CC_REQUEST_FAILED);
            OK(cc_capture_release(c, r));
        }
        cc_capture_close(c);
        for (size_t i = 0; i < 3; ++i) OK(cc_request_destroy(requests[i]));
        finished(&f);
    }
}

static void test_timing_graph(void)
{
    struct fake f; init(&f); f.cfe=f.pisp=f.timing_plan=true; f.exposure=20;
    cc_capture *c=pipeline_requests(&f); OK(cc_capture_configure_sensor_defaults(c));
    cc_sensor_timing_limits limits; OK(cc_capture_get_sensor_timing_limits(c,&limits));
    CHECK(limits.width==128 && limits.height==96 && limits.min_hblank==32 && limits.max_hblank==32);
    cc_ipa_reference_tuning t={.flags=CC_IPA_REFERENCE_AE,.target_luminance=0.18,
        .exposure_speed=0.5,.white_balance_speed=0.5,.min_colour_gain=0.5,.max_colour_gain=4,
        .min_frame_duration_ns=153600,.max_frame_duration_ns=473600};
    cc_ipa *ipa; OK(create_reference(&t,&ipa)); OK(cc_capture_set_ipa(c,&ipa)); CHECK(!ipa);
    cc_request *request,*out; OK(cc_request_create(1,&request));
    cc_control manual={.id=V4L2_CID_EXPOSURE,.value=180};
    OK(cc_request_set_sensor_controls(request,&manual,1));
    OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0)); CHECK(out==request);
    const cc_capture_result *r=cc_request_get_result(out);
    CHECK(r->status==CC_REQUEST_COMPLETE && (r->ipa.flags&CC_IPA_METADATA_AE));
    CHECK(r->ipa.exposure_ns==288000 && f.vblank==88 && f.exposure==180);
    CHECK(*(const unsigned char *)r->raw.planes[0].data==180);
    CHECK(f.sensor_write_count>=3 && f.sensor_writes[0].id==V4L2_CID_VBLANK && f.sensor_writes[0].value==88);
    CHECK(r->sensor.count==3);
    for (uint32_t i=0;i<r->sensor.count;++i) {
        if (r->sensor.controls[i].id==V4L2_CID_VBLANK) CHECK(r->sensor.controls[i].value==88);
        if (r->sensor.controls[i].id==V4L2_CID_EXPOSURE) CHECK(r->sensor.controls[i].value==180);
    }
    OK(cc_capture_release(c,out)); OK(cc_request_set_sensor_controls(request,NULL,0));
    for (unsigned int i=0;i<5;++i) {
        OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
        r=cc_request_get_result(out); CHECK(r->status==CC_REQUEST_COMPLETE && r->ipa.exposure_ns>0);
        CHECK(f.exposure<=96+f.vblank-4 && f.vblank<=200);
        OK(cc_capture_release(c,out));
    }
    OK(cc_capture_stop(c)); OK(cc_capture_queue(c,request)); OK(cc_capture_dequeue(c,&out,0));
    CHECK(cc_request_get_result(out)->status==CC_REQUEST_COMPLETE);
    OK(cc_capture_release(c,out)); cc_capture_close(c); OK(cc_request_destroy(request)); finished(&f);
}
static void test_shading_lifetime(void)
{
    for (unsigned failure=0;failure<5;++failure) {
        struct fake f;init(&f);f.shading_test=true;
        cc_vc4_isp *isp=open_isp(&f);allocated_isp(isp);
        cc_sensor_geometry g={.active={0,0,128,96},.crop={0,0,128,96},.width=128,.height=96};
        cc_vc4_tuning t;OK(cc_vc4_ov5647_build_tuning(&g,5000,1,&t));
        f.refuse_heap=failure==1;f.fail_shading_map=failure==2;f.fail_sync=failure==3;
        int ret=cc_vc4_isp_set_tuning(isp,&t);
        if (failure>0 && failure<4) CHECK(ret<0);
        else {
            OK(ret);CHECK(f.shading_controls==9 && f.last_shading.grid_cell_size==16 &&
                f.last_shading.grid_width==9 && f.last_shading.grid_height==7);
            const uint16_t *grid=f.shading_map;
            for (unsigned j=0;j<63;++j) CHECK(grid[63+j]==grid[126+j]);
            cc_frame raw;cc_dmabuf_plane plane;cc_camera *camera=producer(&f,&raw,&plane);
            OK(cc_vc4_isp_submit_tuned(isp,0,&plane,1,&raw,123,&t));
            ERROR(cc_vc4_isp_set_tuning(isp,&t),EBUSY);
            OK(close_fd(&f,plane.fd));
            if (failure==4) {
                f.fail_stop=true;ERROR(cc_vc4_isp_stop(isp),EIO);cc_vc4_isp_close(isp);isp=NULL;
                cc_camera_close(camera);finished(&f);continue;
            }
            cc_vc4_result result;OK(cc_vc4_isp_dequeue(isp,&result,0));
            OK(cc_vc4_isp_release(isp,0));OK(cc_vc4_isp_stop(isp));
            CHECK(f.shading_open && f.shading_map);
            cc_vc4_isp_close(isp);isp=NULL;cc_camera_close(camera);
        }
        cc_vc4_isp_close(isp);finished(&f);
    }
}

static void test_profile_request_snapshot(void)
{
    struct fake f;init(&f);f.shading_test=true;
    cc_capture *capture=pipeline_requests(&f);
    cc_sensor_geometry g={.active={0,0,128,96},.crop={0,0,128,96},.width=128,.height=96};
    cc_vc4_tuning t;OK(cc_vc4_ov5647_build_tuning(&g,5000,1,&t));
    cc_request *r,*out;OK(cc_request_create(8,&r));OK(cc_request_set_vc4_tuning(r,&t));
    t.gamma.y[1]=4000;OK(cc_capture_queue(capture,r));ERROR(cc_request_set_vc4_tuning(r,&t),EBUSY);
    OK(cc_capture_dequeue(capture,&out,0));CHECK(f.last_gamma.y[1]==5040);
    OK(cc_capture_release(capture,out));OK(cc_request_set_vc4_tuning(r,&t));
    OK(cc_capture_queue(capture,r));OK(cc_capture_dequeue(capture,&out,0));CHECK(f.last_gamma.y[1]==4000);
    OK(cc_capture_release(capture,out));cc_capture_close(capture);OK(cc_request_destroy(r));finished(&f);
}

static void test_isp_try_configuration(void)
{
    ERROR(cc_vc4_isp_try_configuration(NULL,NULL),EINVAL);
    struct fake f; init(&f); cc_vc4_isp *isp=open_isp(&f);
    cc_vc4_configuration wanted={.input=raw_format(),
        .output={{.fourcc=V4L2_PIX_FMT_RGB24,.width=96,.height=64}}};
    cc_rectangle original_crop=f.crop;
    f.validation_only=true; cc_vc4_configuration tried=wanted;
    OK(cc_vc4_isp_try_configuration(isp,&tried));
    CHECK(tried.output[0].width==96 && tried.output[1].fourcc==V4L2_PIX_FMT_YUV420 && f.try_formats==4);
    CHECK(!memcmp(&original_crop,&f.crop,sizeof(f.crop)));
    for (size_t i=0; i<4; ++i) CHECK(!f.nodes[i].format.fourcc);
    f.validation_only=false;
    uint32_t slots=3; ERROR(cc_vc4_isp_allocate_buffers(isp,&slots),EPIPE);
    configure(isp); OK(cc_vc4_isp_allocate_buffers(isp,&slots));
    cc_vc4_configuration baseline; uint32_t count; OK(cc_vc4_capture_info(isp,&baseline,&count));
    struct node nodes[NODES]; memcpy(nodes,f.nodes,sizeof(nodes)); original_crop=f.crop;
    f.validation_only=true; tried=wanted; OK(cc_vc4_isp_try_configuration(isp,&tried));
    CHECK(!memcmp(nodes,f.nodes,sizeof(nodes)) && !memcmp(&original_crop,&f.crop,sizeof(f.crop)));
    for (int port=0; port<4; ++port) {
        tried=wanted; cc_vc4_configuration saved=tried;
        f.fail_req=VIDIOC_TRY_FMT; f.fail_node=port; f.fail_nth=1; f.fail_errno=EIO;
        ERROR(cc_vc4_isp_try_configuration(isp,&tried),EIO); CHECK(!memcmp(&tried,&saved,sizeof(tried)));
    }
    f.fail_req=0; f.mismatch_input=true; tried=wanted; cc_vc4_configuration saved=tried;
    ERROR(cc_vc4_isp_try_configuration(isp,&tried),EPIPE); CHECK(!memcmp(&tried,&saved,sizeof(tried)));
    f.mismatch_input=false; cc_vc4_configuration actual; OK(cc_vc4_capture_info(isp,&actual,&count));
    CHECK(!memcmp(&baseline,&actual,sizeof(actual)) && !memcmp(nodes,f.nodes,sizeof(nodes)));
    f.validation_only=false; cc_vc4_isp_close(isp); finished(&f);
}
static void test_be_try_configuration(void)
{
    ERROR(cc_pisp_be_try_configuration(NULL,NULL),EINVAL);
    for (unsigned revision=0; revision<2; ++revision) {
        struct fake f; init(&f); f.pisp=f.widen_chroma=f.force_color_defaults=true;
        f.hardware_version=revision ? CC_PISP_BE_VERSION_BCM2712_D0:CC_PISP_BE_VERSION_BCM2712_C0;
        cc_pisp_be *be=open_be(&f,false);
        cc_pisp_be_configuration wanted=be_configuration(V4L2_PIX_FMT_YUV420M,V4L2_PIX_FMT_NV12M);
        wanted.crop[0]=(cc_rectangle){16,8,96,80}; wanted.output.width=64; wanted.output.height=48;
        for (size_t i=0; i<4; ++i) {
            wanted.auxiliary[i].format.fourcc=CC_FOURCC('P','C','3','R');
            wanted.auxiliary[i].compression_offset=2048;
        }
        int handles=f.handles; f.validation_only=true;
        cc_pisp_be_configuration tried=wanted; OK(cc_pisp_be_try_configuration(be,&tried));
        CHECK(tried.hardware_version==f.hardware_version && tried.output.width==64 && tried.output.height==48);
        CHECK(tried.output.planes[1].stride==64 && tried.output.planes[1].size==1536);
        CHECK(tried.output.quantization==V4L2_QUANTIZATION_LIM_RANGE && tried.output1.colorspace==V4L2_COLORSPACE_JPEG);
        CHECK(f.handles==handles && f.try_formats>=8);
        for (size_t i=0; i<NODES; ++i) CHECK(!f.nodes[i].format.fourcc);
        for (int port=3; port<8; ++port) ERROR(cc_pisp_be_fd(be,(enum cc_pisp_be_port)port),ENODEV);
        for (uint32_t compression=1; compression<=3; ++compression) {
            cc_pisp_be_configuration compressed=wanted;
            compressed.input.fourcc=CC_FOURCC('P','C','0'+compression,'R');
            compressed.input.planes[0]=(cc_plane_format){128,128*96}; compressed.compression_offset=2048;
            OK(cc_pisp_be_try_configuration(be,&compressed));
            CHECK(compressed.input.fourcc==CC_FOURCC('P','C','0'+compression,'R') && f.handles==handles);
            for (size_t i=0; i<NODES; ++i) CHECK(!f.nodes[i].format.fourcc);
        }
        f.validation_only=false; uint32_t slots=3; ERROR(cc_pisp_be_allocate_buffers(be,&slots),EPIPE);
        cc_pisp_be_configuration baseline=be_configuration(V4L2_PIX_FMT_RGB24,0);
        OK(cc_pisp_be_configure(be,&baseline)); OK(cc_pisp_be_allocate_buffers(be,&slots));
        struct node nodes[NODES]; memcpy(nodes,f.nodes,sizeof(nodes));
        handles=f.handles; int mappings=f.mappings;
        f.validation_only=true; tried=wanted; OK(cc_pisp_be_try_configuration(be,&tried));
        CHECK(f.handles==handles && f.mappings==mappings && !memcmp(nodes,f.nodes,sizeof(nodes)));
        const int ports[]={0,1,2,3,12,13,14,15};
        for (size_t stage=0; stage<sizeof(ports)/sizeof(ports[0]); ++stage) {
            tried=wanted; cc_pisp_be_configuration saved=tried;
            f.fail_req=VIDIOC_TRY_FMT; f.fail_node=ports[stage]; f.fail_nth=1; f.fail_errno=EIO;
            ERROR(cc_pisp_be_try_configuration(be,&tried),EIO);
            CHECK(!memcmp(&tried,&saved,sizeof(tried)) && f.handles==handles && !memcmp(nodes,f.nodes,sizeof(nodes)));
        }
        f.fail_req=0; f.bad_parameters=true; tried=wanted; cc_pisp_be_configuration saved=tried;
        ERROR(cc_pisp_be_try_configuration(be,&tried),EPROTO); CHECK(!memcmp(&tried,&saved,sizeof(tried)));
        f.bad_parameters=false; f.mismatch_input=true; tried=wanted; saved=tried;
        ERROR(cc_pisp_be_try_configuration(be,&tried),EPIPE); CHECK(!memcmp(&tried,&saved,sizeof(tried)));
        f.mismatch_input=false;
        f.fail_req=VIDIOC_QUERYCAP; f.fail_node=15; f.fail_nth=1; f.fail_errno=EIO;
        tried=wanted; saved=tried; ERROR(cc_pisp_be_try_configuration(be,&tried),EIO);
        CHECK(!memcmp(&tried,&saved,sizeof(tried)) && f.handles==handles && !memcmp(nodes,f.nodes,sizeof(nodes)));
        f.fail_req=0;
        cc_pisp_be_configuration actual; uint32_t count; OK(cc_be_capture_info(be,&actual,&count));
        CHECK(!memcmp(&actual,&baseline,sizeof(actual)) && count==slots && f.handles==handles);
        f.validation_only=false; cc_pisp_be_close(be); finished(&f);
    }
}

static void test_processed_roles(bool pisp)
{
    const enum cc_stream_role roles[][3]={{CC_ROLE_VIEWFINDER,CC_ROLE_RAW,CC_ROLE_VIDEO_RECORDING},
        {CC_ROLE_STILL_CAPTURE},{CC_ROLE_RAW},{CC_ROLE_VIEWFINDER},{CC_ROLE_VIDEO_RECORDING},
        {CC_ROLE_VIDEO_RECORDING},{CC_ROLE_VIEWFINDER,CC_ROLE_VIDEO_RECORDING}};
    const size_t counts[]={3,1,1,1,1,1,2};
    for (size_t sample=0;sample<7;++sample) {
        if (!pisp && sample>4) continue;
        struct fake f; init(&f); f.pisp=pisp; f.cfe=true; f.unicam=!pisp;
        f.mbus[0][0][0]=f.mbus[0][1][0]=(struct v4l2_mbus_framefmt){.code=MEDIA_BUS_FMT_SRGGB10_1X10,
            .width=128,.height=96,.field=V4L2_FIELD_NONE,.colorspace=V4L2_COLORSPACE_RAW};
        if (pisp && sample==3) f.hardware_version=CC_PISP_BE_VERSION_BCM2712_D0;
        struct cc_io io={&ops,&f}; cc_capture_devices d={0};
        if (pisp) {
            f.mbus[0][0][0]=f.mbus[0][1][0]=(struct v4l2_mbus_framefmt){.code=MEDIA_BUS_FMT_SRGGB10_1X10,
                .width=128,.height=96,.field=V4L2_FIELD_NONE,.colorspace=V4L2_COLORSPACE_RAW};
            cc_camera_info info={.pipeline=CC_PIPELINE_RPI_CFE,.media_path="/dev/media1",.video_path="/dev/video7",.sensor_entity=1};
            OK(cc_cfe_open_io(&info,&io,&d.frontend)); d.backend=open_be(&f,false);
            if (sample==1) {
                f.physical_array_larger=true; cc_camera_properties properties;
                OK(cc_cfe_get_properties(d.frontend,&properties)); CHECK(properties.pixel_array_width==256 && properties.pixel_array_height==192);
            }
        } else {
            cc_camera_info info={.pipeline=CC_PIPELINE_RPI_UNICAM,.media_path="/dev/media1",.video_path="/dev/video7",.sensor_entity=1};
            OK(cc_camera_open_io(&info,&io,&d.camera));
            d.vc4=open_isp(&f);
        }
        f.validation_only=true; cc_pipeline_configuration c;
        int ret=cc_pipeline_generate_configuration(&d,roles[sample],counts[sample],&c);
        if (ret<0) fprintf(stderr,"processed generation pisp=%d sample=%zu error=%d\n",pisp,sample,ret);
        CHECK(ret>=0 && c.num_streams==counts[sample]);
        for (size_t i=0;i<c.num_streams;++i) CHECK(c.streams[i].role==roles[sample][i]);
        if (!sample) CHECK(c.streams[0].stream==CC_CAPTURE_OUTPUT1 && c.streams[1].stream==CC_CAPTURE_RAW &&
            c.streams[2].stream==CC_CAPTURE_OUTPUT0 && c.streams[0].format.width==800 && c.streams[2].format.width==1920);
        if (sample==1 || sample==3) CHECK(c.streams[0].stream==(pisp ? CC_CAPTURE_OUTPUT1:CC_CAPTURE_OUTPUT0));
        if (sample==1) CHECK(c.streams[0].format.width==128 && c.streams[0].format.height==96);
        if (pisp && !sample) CHECK(c.streams[0].format.fourcc==V4L2_PIX_FMT_BGR24 &&
            c.streams[2].format.colorspace==V4L2_COLORSPACE_JPEG && c.streams[2].format.quantization==V4L2_QUANTIZATION_FULL_RANGE);
        if (pisp && sample==3) CHECK(c.streams[0].format.fourcc==V4L2_PIX_FMT_XBGR32);
        if (pisp && sample==4) CHECK(c.streams[0].format.colorspace==V4L2_COLORSPACE_REC709 &&
            c.streams[0].format.ycbcr_enc==V4L2_YCBCR_ENC_709 && c.streams[0].format.quantization==V4L2_QUANTIZATION_LIM_RANGE);
        if (sample==5) {
            c.streams[0].format=(cc_format){.fourcc=V4L2_PIX_FMT_GREY,.width=64,.height=48,
                .colorspace=V4L2_COLORSPACE_REC709,.xfer_func=V4L2_XFER_FUNC_709,.ycbcr_enc=V4L2_YCBCR_ENC_709,
                .quantization=V4L2_QUANTIZATION_LIM_RANGE};
            CHECK(cc_pipeline_validate_configuration(&d,&c)>=0);
            CHECK(c.streams[0].stream==CC_CAPTURE_OUTPUT1 && c.streams[0].format.fourcc==V4L2_PIX_FMT_GREY &&
                c.streams[0].format.quantization==V4L2_QUANTIZATION_LIM_RANGE);
        }
        if (sample==6) {
            c.streams[0].format.colorspace=V4L2_COLORSPACE_SRGB;
            c.streams[0].format.xfer_func=V4L2_XFER_FUNC_SRGB;
            c.streams[1].format.colorspace=V4L2_COLORSPACE_REC709;
            c.streams[1].format.xfer_func=V4L2_XFER_FUNC_709;
            c.streams[1].format.quantization=V4L2_QUANTIZATION_LIM_RANGE;
            CHECK(cc_pipeline_validate_configuration(&d,&c)>=0);
            CHECK(c.streams[1].format.colorspace==V4L2_COLORSPACE_JPEG && c.streams[1].format.quantization==V4L2_QUANTIZATION_FULL_RANGE);
        }
        CHECK(cc_pipeline_validate_configuration(&d,&c)==CC_CONFIGURATION_VALID);
        f.validation_only=false; CHECK(cc_pipeline_configure(&d,&c)>=0);
        if (pisp) {
            cc_cfe_configuration fe; cc_pisp_be_configuration be;
            OK(cc_cfe_get_configuration(d.frontend,&fe)); OK(cc_pisp_be_get_configuration(d.backend,&be));
            CHECK(be.input.fourcc==c.source_format.fourcc && fe.image.width==c.source_format.width && be.parameters.planes[0].size);
        } else { cc_vc4_configuration vc4; OK(cc_vc4_isp_get_configuration(d.vc4,&vc4)); CHECK(vc4.statistics.fourcc && vc4.input.width==128); }
        if (!sample) {
            cc_pipeline_configuration saved=c;
            f.fail_req=VIDIOC_S_FMT; f.fail_node=1; f.fail_nth=1; f.fail_errno=EIO;
            ERROR(cc_pipeline_configure(&d,&c),EIO); CHECK(!memcmp(&c,&saved,sizeof(c)));
            uint32_t slots=2;
            if (pisp) { ERROR(cc_cfe_allocate_buffers(d.frontend,&slots),EPIPE); ERROR(cc_pisp_be_allocate_buffers(d.backend,&slots),EPIPE); }
            else { ERROR(cc_camera_allocate_buffers(d.camera,&slots),EPIPE); ERROR(cc_vc4_isp_allocate_buffers(d.vc4,&slots),EPIPE); }
            f.fail_req=0; CHECK(cc_pipeline_configure(&d,&c)>=0);
            if (pisp) OK(cc_pisp_be_allocate_buffers(d.backend,&slots)); else OK(cc_vc4_isp_allocate_buffers(d.vc4,&slots));
            f.validation_only=true; ERROR(cc_pipeline_configure(&d,&c),EBUSY); f.validation_only=false;
            if (pisp) OK(cc_pisp_be_free_buffers(d.backend)); else OK(cc_vc4_isp_free_buffers(d.vc4));
        }
        uint32_t slots=2;
        if (pisp) { OK(cc_cfe_allocate_buffers(d.frontend,&slots)); OK(cc_pisp_be_allocate_buffers(d.backend,&slots)); }
        else { OK(cc_camera_allocate_buffers(d.camera,&slots)); OK(cc_vc4_isp_allocate_buffers(d.vc4,&slots)); }
        cc_capture *capture=NULL; OK(cc_capture_create(&d,&capture,&slots));
        cc_request *request=NULL,*out=NULL; OK(cc_request_create(23,&request)); OK(cc_capture_queue(capture,request));
        OK(cc_capture_dequeue(capture,&out,0)); CHECK(out==request && cc_request_get_result(out)->status==CC_REQUEST_COMPLETE);
        for (size_t i=0;i<c.num_streams;++i) {
            const cc_capture_result *result=cc_request_get_result(out);
            const cc_frame *frame=c.streams[i].stream==CC_CAPTURE_RAW ? &result->raw:
                &result->output[c.streams[i].stream==CC_CAPTURE_OUTPUT1 ? 1:0];
            CHECK(frame->num_planes==c.streams[i].format.num_planes && frame->planes[0].length==c.streams[i].format.planes[0].size);
        }
        OK(cc_capture_release(capture,out)); cc_capture_close(capture); OK(cc_request_destroy(request)); finished(&f);
    }
}

int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "proxy-feedback")) {
        proxy_worker = argv[2]; int ret = proxy_available(); if (ret == 77) return ret; OK(ret);
        test_ipa_vc4_feedback(); test_cfe_be_feedback(0); test_cfe_be_feedback(1); test_cfe_be_feedback(2);
        test_af_graph(false); test_af_graph(true); test_embedded_af_graph(); test_timing_graph();
        test_imported_feedback(false); test_imported_feedback(true);
        use_v2_topology = true;
        test_ipa_vc4_feedback(); test_cfe_be_feedback(0); test_af_graph(false); test_af_graph(true); test_embedded_af_graph();
        puts("Isolated VC4/CFE/PiSP feedback, AF/PDAF, timing, imported results and restarts passed"); return 0;
    }
    if (argc == 4 && (!strcmp(argv[1], "proxy-modules") || !strcmp(argv[1], "proxy-manager"))) {
        proxy_worker = argv[3]; int ret = proxy_available(); if (ret == 77) return ret; OK(ret);
    }
    if ((argc == 3 && (!strcmp(argv[1], "modules") || !strcmp(argv[1], "manager"))) ||
        (argc == 4 && proxy_worker)) {
        bool managed = !strcmp(argv[1], "manager") || !strcmp(argv[1], "proxy-manager");
        test_loaded_ipa(argv[2], false, managed); test_loaded_ipa(argv[2], true, managed);
        use_v2_topology = true; test_loaded_ipa(argv[2], false, managed); test_loaded_ipa(argv[2], true, managed);
        puts("Loaded IPA adoption, VC4/CFE/PiSP completion, restart and cancellation passed"); return 0;
    }
    if (argc==2 && !strcmp(argv[1],"roles")) {
        test_processed_roles(false); test_processed_roles(true);
        use_v2_topology=true; test_processed_roles(false); test_processed_roles(true);
        puts("VC4/PiSP roles, sensor/colour selection, stream assignment, apply failure recovery and adopted capture passed"); return 0;
    }
    if (argc==2 && !strcmp(argv[1],"try-configuration")) {
        test_isp_try_configuration(); test_be_try_configuration();
        use_v2_topology=true; test_isp_try_configuration(); test_be_try_configuration();
        puts("VC4/PiSP TRY layouts, crops, revisions, history ports, active-state preservation and rollback passed"); return 0;
    }
    if (argc==2 && !strcmp(argv[1],"timing")) {
        test_timing_graph(); use_v2_topology=true; test_timing_graph();
        puts("Live timing limits, frame-duration IPA planning, priority blanking and future exposure ranges passed"); return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "fences")) {
        for (int imported = 0; imported < 2; ++imported) {
            test_acquire_fences(false, false, imported != 0); test_acquire_fences(true, false, imported != 0);
            test_acquire_fences(true, true, imported != 0);
        }
        test_fence_deadlines(); test_fence_timeout_during_processing(); test_fence_disabled_timeout();
        test_fence_capacity_and_clocks(); test_fence_primitives();
        puts("Acquire-fence preparation, one-shot ownership, deadlines, polling, per-request cancellation and recovery passed"); return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "buffers")) {
        for (uint32_t mask = 1; mask < 8; ++mask) {
            test_request_buffers(false, false, mask); test_request_buffers(true, false, mask); test_request_buffers(true, true, mask);
        }
        test_imported_feedback(false); test_imported_feedback(true); test_request_buffer_failures(); test_real_buffer_descriptors();
        puts("Per-request source/VC4/PiSP image DMA-BUF ownership, alias checks, planes, feedback and cancellation passed"); return 0;
    }
    if (argc>1 && !strcmp(argv[1],"topology")) {
        use_v2_topology=true;
        test_processing(); test_be_processing(); test_pipeline_requests(false); test_pipeline_requests(true);
        test_cfe_be_feedback(0); test_af_graph(false); test_af_graph(true); test_embedded_af_graph();
        puts("VC4/PiSP/CFE native capture, processing, controls and embedded/PDAF graphs using full media-v2 topology passed"); return 0;
    }
    if (argc>1 && !strcmp(argv[1],"embedded")) { test_embedded_af_graph(); puts("Timestamp-paired CFE/PiSP sensor reports and PDAF graph feedback passed"); return 0; }
    if (argc==2 && !strcmp(argv[1],"af")) {
        test_af_graph(false); test_af_graph(true);
        puts("VC4/CFE/PiSP AF feedback, lens association, combined 3A, snapshots, restart and failures passed"); return 0;
    }
    if (argc==2 && !strcmp(argv[1],"cfe-be")) {
        test_cfe_be_feedback(0); test_cfe_be_feedback(1); test_cfe_be_feedback(2);
        test_compression_graph_mismatch(); test_cfe_be_failures();
        puts("Combined CFE/PiSP IPA feedback, native settings, overlap and manual overrides passed");
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"ipa")) {
        test_ipa_vc4_feedback();
        puts("VC4 C IPA white-balance feedback, native statistics, snapshots and manual overrides passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "requests")) {
        test_graph_history();
        test_pipeline_requests(false); test_pipeline_requests(true);
        test_sensor_pipeline_requests(false); test_sensor_pipeline_requests(true);
        test_request_pipeline_failures(false); test_request_pipeline_failures(true);
        puts("VC4/PiSP request overlap, copied settings, DMA ownership, completion and graph cancellation passed");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "pisp")) {
        test_be_history();
        test_be_history_failures();
        test_be_processing();
        test_be_failures();
        test_be_yuv_outputs();
        test_be_aux_failures();
        test_be_resize();
        test_be_rgb32_outputs();
        test_be_native_blocks();
        puts("PiSP BE RGB/YUV memory planes, dual outputs, native parameters, imported lifetime and cancellation passed");
        return 0;
    }
    test_profile_request_snapshot();
    test_shading_lifetime();
    test_processing();
    test_failures();
    test_controls_and_selection();
    test_open_and_protocol_errors();
    test_selection_layout_change();
    puts("VC4 ISP discovery, input/main/lores/stats jobs, controls, crop and cancellation passed");
    return 0;
}
