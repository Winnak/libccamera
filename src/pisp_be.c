/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "pisp_format.h"
#include "pisp_tiling.h"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CC_BE_MAX_BUFFERS 256U
#define CC_BE_PARAMETERS CC_FOURCC('R', 'P', 'B', 'C')

struct cc_pisp_be {
    struct cc_io io;
    struct cc_media *media;
    cc_video_device *devices[8];
    cc_stream *streams[8];
    bool configured, fatal;
    uint32_t count, done, job_ports;
    int active;
    unsigned char states[CC_BE_MAX_BUFFERS];
    cc_pisp_be_configuration configuration;
    cc_pisp_be_result pending;
};

static int usable(const cc_pisp_be *be) { return !be ? -EINVAL : be->fatal ? -ENODEV : 0; }
static uint32_t ports(const cc_pisp_be *be)
{
    uint32_t mask = be->configuration.output1.fourcc ? 15U : 7U;
    for (uint32_t i = 0; i < 4; ++i)
        if (be->configuration.auxiliary[i].format.fourcc) mask |= 1U << (i + 4U);
    return mask;
}
static bool input_port(size_t port) { return port == 0 || port == 4 || port == 6; }
static uint32_t job_ports(const struct pisp_be_config *c)
{
    uint32_t mask = c->global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1 ? 15U : 7U;
    const uint32_t bits[4] = {PISP_BE_BAYER_ENABLE_TDN_INPUT,PISP_BE_BAYER_ENABLE_TDN_OUTPUT,
        PISP_BE_BAYER_ENABLE_STITCH_INPUT,PISP_BE_BAYER_ENABLE_STITCH_OUTPUT};
    for (uint32_t i = 0; i < 4; ++i) if (c->global.bayer_enables & bits[i]) mask |= 1U << (i + 4U);
    return mask;
}
cc_stream *cc_be_image_stream(cc_pisp_be *be, uint32_t output)
{
    return be && output < 2 && (!output || be->configuration.output1.fourcc) ?
        be->streams[output ? CC_PISP_BE_OUTPUT1 : CC_PISP_BE_OUTPUT] : NULL;
}
cc_stream *cc_be_parameters_stream(cc_pisp_be *be) { return be ? be->streams[CC_PISP_BE_PARAMETERS] : NULL; }
cc_stream *cc_be_aux_output_stream(cc_pisp_be *be, uint32_t output)
{
    return be && output < 2 && be->configuration.auxiliary[2U*output+1U].format.fourcc ?
        be->streams[5U+2U*output] : NULL;
}
int cc_be_capture_info(cc_pisp_be *be, cc_pisp_be_configuration *configuration, uint32_t *count)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (!be->configured || !be->count) return -EPIPE;
    if (be->active >= 0) return -EBUSY;
    for (uint32_t i = 0; i < be->count; ++i) if (be->states[i]) return -EBUSY;
    *configuration = be->configuration; *count = be->count;
    return 0;
}
size_t cc_be_pending_fds(const cc_pisp_be *be, struct pollfd *fds)
{
    size_t count = 0;
    if (be->active >= 0)
        for (size_t i = 0; i < 8; ++i)
            if ((be->job_ports & (1U << i)) && !(be->done & (1U << i)))
                fds[count++] = (struct pollfd) {cc_stream_fd(be->streams[i]), input_port(i) || i == 2 ? POLLOUT : POLLIN, 0};
    return count;
}
static bool allocated(const cc_pisp_be *be)
{
    for (size_t i = 0; i < 8; ++i)
        if (be->streams[i] && be->streams[i]->allocated)
            return true;
    return false;
}

static int open_port(cc_pisp_be *be, size_t port)
{
    if (be->streams[port]) return 0;
    static const char *const names[] = { "pispbe-input", "pispbe-output0", "pispbe-config", "pispbe-output1",
        "pispbe-tdn_input", "pispbe-tdn_output", "pispbe-stitch_input", "pispbe-stitch_output" };
    struct cc_entity *entity = NULL;
    for (size_t i = 0; i < be->media->num_entities; ++i)
        if (!strcmp(cc_entity_name(&be->media->entities[i]), names[port])) entity = &be->media->entities[i];
    if (!entity || !entity->path[0]) return -ENODEV;
    cc_video_device *device = NULL;
    int ret = cc_video_open_io(entity->path, &be->io, &device);
    if (ret < 0) return ret;
    uint32_t caps = device->capabilities;
    enum cc_stream_type type = port == CC_PISP_BE_PARAMETERS ? CC_STREAM_META_OUTPUT :
        input_port(port) ? (caps & V4L2_CAP_VIDEO_OUTPUT ? CC_STREAM_OUTPUT : CC_STREAM_OUTPUT_MPLANE) :
                (caps & V4L2_CAP_VIDEO_CAPTURE ? CC_STREAM_CAPTURE : CC_STREAM_CAPTURE_MPLANE);
    cc_stream *stream = NULL;
    ret = cc_video_create_stream(device, type, &stream);
    if (ret < 0) { cc_video_close(device); return ret; }
    be->devices[port] = device;
    be->streams[port] = stream;
    return 0;
}

int cc_pisp_be_open_io(const char *path, const struct cc_io *io, cc_pisp_be **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (!path || !io)
        return -EINVAL;
    cc_pisp_be *be = calloc(1, sizeof(*be));
    if (!be)
        return -ENOMEM;
    be->io = *io;
    be->active = -1;
    int ret = cc_media_open(io, path, true, &be->media);
    if (ret < 0)
        goto fail;
    if (strncmp(be->media->info.driver, "pispbe", sizeof(be->media->info.driver))) {
        ret = -ENOTSUP;
        goto fail;
    }
    for (size_t i = 0; i < 3; ++i) {
        ret = open_port(be, i);
        if (ret < 0)
            goto fail;
    }
    *out = be;
    return 0;
fail:
    cc_pisp_be_close(be);
    return ret;
}

int cc_pisp_be_open(const char *media_path, cc_pisp_be **out)
{
    if (!out)
        return -EINVAL;
    *out = NULL;
    if (media_path)
        return cc_pisp_be_open_io(media_path, &cc_system_io, out);
    struct dirent **nodes = NULL;
    int count = scandir("/dev", &nodes, NULL, alphasort);
    if (count < 0)
        return -errno;
    int ret = -ENODEV;
    for (int i = 0; i < count; ++i) {
        const char *name = nodes[i]->d_name;
        if (strncmp(name, "media", 5) || !name[5] || strspn(name + 5, "0123456789") != strlen(name + 5))
            continue;
        char path[CC_PATH_MAX];
        int n = snprintf(path, sizeof(path), "/dev/%s", name);
        if (n < 0 || (size_t)n >= sizeof(path))
            continue;
        int candidate = cc_pisp_be_open_io(path, &cc_system_io, out);
        if (!candidate || candidate == -ENOMEM) { ret = candidate; break; }
        if (candidate != -ENODEV && candidate != -ENOTSUP && candidate != -ENOENT)
            ret = candidate;
    }
    for (int i = 0; i < count; ++i) free(nodes[i]);
    free(nodes);
    return ret;
}

static bool same_layout(const cc_format *a, const cc_format *b)
{
    if (a->width != b->width || a->height != b->height || a->fourcc != b->fourcc || a->num_planes != b->num_planes)
        return false;
    for (uint32_t i = 0; i < a->num_planes; ++i)
        if (a->planes[i].stride != b->planes[i].stride || a->planes[i].size != b->planes[i].size) return false;
    return true;
}

int cc_be_configuration_ready(cc_pisp_be *be)
{
    int ret=usable(be); return ret<0 ? ret:allocated(be) ? -EBUSY:0;
}
void cc_be_invalidate_configuration(cc_pisp_be *be) { if (be) be->configured=false; }
uint32_t cc_be_hardware_version(const cc_pisp_be *be) { return be ? be->media->info.hw_revision:0; }
int cc_pisp_be_get_configuration(cc_pisp_be *be, cc_pisp_be_configuration *out)
{
    int ret=usable(be); if (ret<0) return ret;
    if (!out) return -EINVAL;
    if (!be->configured) return -EPIPE;
    *out=be->configuration; return 0;
}

static int negotiate_stream(cc_stream *stream, cc_format *format, bool active)
{
    return active ? cc_stream_configure(stream,format):cc_stream_try_format(stream,format);
}

static int configure_output(cc_pisp_be *be, size_t port, cc_format *format, bool active)
{
    cc_format requested = *format;
    int ret = negotiate_stream(be->streams[port], format,active);
    if (ret < 0) return ret;
    cc_format adjusted = *format;
    ret = cc_pisp_prepare_output(&adjusted);
    if (ret < 0) return ret;
    /* Some kernels widen chroma strides without growing the corresponding
     * allocation. Negotiate sufficient capacities using the returned strides. */
    if (!same_layout(format, &adjusted)) {
        ret = negotiate_stream(be->streams[port], &adjusted,active);
        if (ret < 0) return ret;
        *format = adjusted;
    }
    struct cc_pisp_output_layout layout;
    ret = cc_pisp_output_layout(format, &layout);
    if (ret < 0) return ret;
    /* PiSP node S_FMT forces default colour fields, independently of the
     * parameters we program. Public output metadata describes that processing;
     * the private stream retains G_FMT's fields for allocation validation. */
    format->colorspace = requested.colorspace;
    format->ycbcr_enc = requested.ycbcr_enc;
    format->quantization = requested.quantization;
    format->xfer_func = requested.xfer_func;
    return 0;
}

static int negotiate_configuration(cc_pisp_be *be, cc_pisp_be_configuration *configuration, bool active)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (!configuration || configuration->input.width < 16 || configuration->input.height < 16 ||
        configuration->input.width > UINT16_MAX || configuration->input.height > UINT16_MAX)
        return -EINVAL;
    if (active && allocated(be)) return -EBUSY;
    cc_pisp_be_configuration c = *configuration;
    c.hardware_version = be->media->info.hw_revision;
    ret = cc_pisp_crop(&c.input, &c.crop[0], &c.crop[0]);
    if (ret < 0) return ret;
    if (!c.output.width) c.output.width = c.crop[0].width;
    if (!c.output.height) c.output.height = c.crop[0].height;
    if (!c.output.fourcc) c.output.fourcc = V4L2_PIX_FMT_RGB24;
    ret = cc_pisp_prepare_output(&c.output);
    if (ret < 0) return ret;
    if (c.output1.fourcc) {
        ret = cc_pisp_crop(&c.input, &c.crop[1], &c.crop[1]);
        if (ret < 0) return ret;
        if (!c.output1.width) c.output1.width = c.crop[1].width;
        if (!c.output1.height) c.output1.height = c.crop[1].height;
        ret = cc_pisp_prepare_output(&c.output1);
        if (ret < 0) return ret;
    } else { c.output1 = (cc_format) {0}; c.crop[1] = (cc_rectangle) {0}; c.transform[1] = 0; }
    for (uint32_t i = 0; i < 4; ++i) {
        cc_format *f = &c.auxiliary[i].format;
        if (!f->fourcc) { c.auxiliary[i] = (cc_pisp_be_aux_configuration) {0}; continue; }
        struct cc_pisp_raw_layout raw;
        ret = cc_pisp_raw_format(f->fourcc, &raw);
        if (ret < 0) return ret;
        if (!f->width) f->width = c.input.width;
        if (!f->height) f->height = c.input.height;
        if (!f->num_planes) f->num_planes = 1;
        if (!f->planes[0].stride) f->planes[0].stride = (f->width * raw.pixel_bytes + 63U) & ~63U;
        uint64_t size = (uint64_t)f->planes[0].stride * f->height;
        if (size > UINT32_MAX) return -ERANGE;
        if (!f->planes[0].size) f->planes[0].size = (uint32_t)size;
    }
    struct pisp_be_tiles_config parameters;
    ret = cc_pisp_be_build_configuration(&c, NULL, &parameters);
    if (ret < 0) return ret;
    if (active) be->configured = false;
    if (c.output1.fourcc) {
        ret = open_port(be, CC_PISP_BE_OUTPUT1);
        if (ret < 0) return ret;
    }
    for (size_t i = 4; i < 8; ++i) {
        cc_format *format = &c.auxiliary[i - 4].format;
        if (!format->fourcc) continue;
        ret = open_port(be, i);
        if (ret < 0) return ret;
        cc_format requested = *format;
        ret = negotiate_stream(be->streams[i], format,active);
        if (ret < 0) return ret;
        if (!same_layout(format, &requested)) return -EPIPE;
        *format = requested;
    }
    ret = negotiate_stream(be->streams[0], &c.input,active);
    if (ret < 0) return ret;
    const cc_format *source = &configuration->input;
    if (c.input.width != source->width || c.input.height != source->height || c.input.fourcc != source->fourcc ||
        c.input.num_planes != source->num_planes || c.input.planes[0].stride != source->planes[0].stride ||
        c.input.planes[0].size != source->planes[0].size)
        return -EPIPE;
    c.input = *source;
    ret = configure_output(be, CC_PISP_BE_OUTPUT, &c.output,active);
    if (ret < 0) return ret;
    if (c.output1.fourcc) {
        ret = configure_output(be, CC_PISP_BE_OUTPUT1, &c.output1,active);
        if (ret < 0) return ret;
    }
    ret = cc_pisp_be_build_configuration(&c, NULL, &parameters);
    if (ret < 0) return ret;
    c.parameters = (cc_format) { .fourcc = CC_BE_PARAMETERS };
    ret = negotiate_stream(be->streams[2], &c.parameters,active);
    if (ret < 0) return ret;
    if (c.parameters.fourcc != CC_BE_PARAMETERS || c.parameters.planes[0].size < sizeof(parameters))
        return -EPROTO;
    if (active) { be->configuration = c; be->configured = true; }
    *configuration = c;
    return 0;
}

int cc_pisp_be_configure(cc_pisp_be *be, cc_pisp_be_configuration *configuration)
{
    return negotiate_configuration(be,configuration,true);
}
int cc_pisp_be_try_configuration(cc_pisp_be *be, cc_pisp_be_configuration *configuration)
{
    int ret=usable(be);
    if (ret<0) return ret;
    uint32_t opened=0;
    for (size_t i=0; i<8; ++i) if (be->streams[i]) opened|=1U<<i;
    ret=negotiate_configuration(be,configuration,false);
    for (size_t i=0; i<8; ++i) if (!(opened&(1U<<i)) && be->streams[i]) {
        cc_stream_close(be->streams[i]); cc_video_close(be->devices[i]);
        be->streams[i]=NULL; be->devices[i]=NULL;
    }
    return ret;
}

int cc_pisp_be_stop(cc_pisp_be *be)
{
    if (!be) return -EINVAL;
    int first = be->fatal ? -ENODEV : 0;
    for (size_t i = 0; i < 8; ++i) {
        if (!be->streams[i]) continue;
        int ret = cc_stream_stop(be->streams[i]);
        if (ret < 0 && !first) first = ret;
    }
    if (!first) for (size_t i = 0; i < 8; ++i) {
        if (!be->streams[i]) continue;
        int ret = cc_stream_clear_imports(be->streams[i]);
        if (ret < 0) { first = ret; break; }
    }
    if (first) {
        /* Every port must stop DMA before imported references can be released. */
        for (size_t i = 0; i < 8; ++i) {
            cc_video_device *d = be->devices[i];
            if (d && d->fd >= 0) {
                d->io.ops->close(d->io.ctx, d->fd);
                d->fd = -1;
            }
        }
        for (size_t i = 0; i < 8; ++i)
            if (be->streams[i] && be->streams[i]->memory == V4L2_MEMORY_DMABUF)
                (void)cc_stream_free_buffers(be->streams[i]);
        be->fatal = true;
        be->configured = false;
    }
    be->active = -1;
    be->done = 0;
    memset(be->states, 0, sizeof(be->states));
    return first;
}

int cc_pisp_be_free_buffers(cc_pisp_be *be)
{
    if (!be) return -EINVAL;
    int ret = cc_pisp_be_stop(be);
    for (size_t i = 0; i < 8; ++i) {
        if (!be->streams[i]) continue;
        int freed = cc_stream_free_buffers(be->streams[i]);
        if (freed < 0 && !ret) ret = freed;
    }
    be->count = 0;
    return ret;
}

void cc_pisp_be_close(cc_pisp_be *be)
{
    if (!be) return;
    cc_pisp_be_stop(be);
    for (size_t i = 0; i < 8; ++i) {
        cc_stream_close(be->streams[i]);
        cc_video_close(be->devices[i]);
    }
    cc_media_close(be->media);
    free(be);
}

int cc_pisp_be_allocate_buffers_with_memory(cc_pisp_be *be, const enum cc_memory outputs[2], uint32_t *count)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (!count || !*count || *count > CC_BE_MAX_BUFFERS) return -EINVAL;
    if (!outputs || (outputs[0] != CC_MEMORY_MMAP && outputs[0] != CC_MEMORY_DMABUF) ||
        (outputs[1] != CC_MEMORY_MMAP && outputs[1] != CC_MEMORY_DMABUF)) return -EINVAL;
    if (allocated(be)) return -EBUSY;
    if (!be->configured) return -EPIPE;
    uint32_t minimum = UINT32_MAX;
    for (size_t i = 0; i < 8; ++i) {
        if (!(ports(be) & (1U << i))) continue;
        uint32_t n = *count;
        enum cc_memory memory = input_port(i) ? CC_MEMORY_DMABUF : i == 2 || i >= 4 ? CC_MEMORY_MMAP : outputs[i == 3 ? 1 : 0];
        ret = cc_stream_allocate_buffers(be->streams[i], memory, &n);
        if (ret < 0) { cc_pisp_be_free_buffers(be); return ret; }
        if (n < minimum) minimum = n;
    }
    be->count = *count = minimum;
    return 0;
}

int cc_pisp_be_allocate_buffers(cc_pisp_be *be, uint32_t *count)
{
    const enum cc_memory outputs[2] = {CC_MEMORY_MMAP, CC_MEMORY_MMAP};
    return cc_pisp_be_allocate_buffers_with_memory(be, outputs, count);
}

int cc_pisp_be_import_output(cc_pisp_be *be, uint32_t output, uint32_t index,
                            const cc_dmabuf_plane *planes, uint32_t num_planes)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    cc_stream *stream = cc_be_image_stream(be, output);
    if (!stream || index >= be->count) return -EINVAL;
    if (be->states[index]) return -EBUSY;
    return cc_stream_import(stream, index, planes, num_planes);
}

static int cancel_error(cc_pisp_be *be, int error)
{
    int ret = cc_pisp_be_stop(be);
    return ret < 0 ? ret : error;
}

int cc_pisp_be_submit(cc_pisp_be *be, uint32_t index, const cc_dmabuf_plane *planes,
                       uint32_t num_planes, const cc_frame *input, const struct pisp_be_config *tuning,
                       uint64_t cookie)
{
    return cc_pisp_be_submit_with_auxiliary(be,index,planes,num_planes,input,NULL,tuning,cookie);
}

static int check_aux_aliases(cc_pisp_be *be, uint32_t index, uint32_t mask,
                              const cc_dmabuf_plane *main, const cc_pisp_be_aux_input auxiliary[2])
{
    if (!(mask & UINT32_C(0xf0))) return 0;
    if (!be->io.ops->identity) return -ENOTSUP;
    uint64_t devices[11], inodes[11];
    size_t count = 0;
    for (size_t i = 0; i < 3; ++i) {
        if (i && !(mask & (1U << (i == 1 ? 4 : 6)))) continue;
        int fd = i ? auxiliary[i-1].plane.fd : main->fd;
        int ret = be->io.ops->identity(be->io.ctx,fd,&devices[count],&inodes[count]);
        if (ret < 0) return ret;
        ++count;
    }
    const size_t outputs[4] = {1,3,5,7};
    for (size_t i = 0; i < 4; ++i) {
        size_t port = outputs[i];
        if (!(mask & (1U << port))) continue;
        cc_stream *s = be->streams[port];
        for (uint32_t plane = 0; plane < s->format.num_planes; ++plane) {
            int fd = cc_stream_canonical_export(s,index,plane);
            if (fd < 0) return fd;
            uint64_t device, inode;
            int ret = be->io.ops->identity(be->io.ctx,fd,&device,&inode);
            be->io.ops->close(be->io.ctx,fd);
            if (ret < 0) return ret;
            for (size_t j = 0; j < count; ++j)
                if (device == devices[j] && inode == inodes[j]) return -EEXIST;
            devices[count]=device; inodes[count]=inode; ++count;
        }
    }
    return 0;
}

int cc_pisp_be_submit_with_auxiliary(cc_pisp_be *be, uint32_t index, const cc_dmabuf_plane *planes,
                       uint32_t num_planes, const cc_frame *input, const cc_pisp_be_aux_input auxiliary[2], const struct pisp_be_config *tuning,
                       uint64_t cookie)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (!be->configured || !be->count) return -EPIPE;
    if (index >= be->count || !input || !planes || num_planes != 1 || input->num_planes != 1 ||
        input->timestamp_ns / UINT64_C(1000000000) > LONG_MAX)
        return -EINVAL;
    if (be->active >= 0 || be->states[index]) return -EBUSY;
    for (uint32_t i = 0; i < 2; ++i) {
        cc_stream *stream = cc_be_image_stream(be, i);
        if (stream && (!stream->allocated || !stream->buffers[index].bound)) return -EPIPE;
    }
    if (input->planes[0].bytes_used > planes[0].length || input->planes[0].data_offset ||
        input->planes[0].bytes_used < be->configuration.input.planes[0].size)
        return -EINVAL;
    struct pisp_be_tiles_config config;
    ret = cc_pisp_be_build_configuration(&be->configuration, tuning, &config);
    if (ret < 0) return ret;
    uint32_t mask = job_ports(&config.config);
    for (uint32_t i = 0; i < 2; ++i) {
        if (!(mask & (1U << (4U+2U*i)))) {
            if (auxiliary && auxiliary[i].frame.num_planes) return -EINVAL;
            continue;
        }
        if (!auxiliary || auxiliary[i].frame.num_planes != 1 ||
            auxiliary[i].frame.planes[0].data_offset || auxiliary[i].plane.fd < 0 ||
            auxiliary[i].frame.timestamp_ns / UINT64_C(1000000000) > LONG_MAX ||
            auxiliary[i].frame.planes[0].bytes_used < be->configuration.auxiliary[2U*i].format.planes[0].size ||
            auxiliary[i].frame.planes[0].bytes_used > auxiliary[i].plane.length) return -EINVAL;
    }
    ret = check_aux_aliases(be,index,mask,planes,auxiliary);
    if (ret < 0) return ret;
    cc_frame parameters;
    ret = cc_stream_buffer(be->streams[2], index, &parameters);
    if (ret < 0) return ret;
    memcpy(parameters.planes[0].data, &config, sizeof(config));
    parameters.planes[0].bytes_used = sizeof(config);
    parameters.planes[0].data_offset = 0;
    parameters.timestamp_ns = input->timestamp_ns;
    parameters.field = V4L2_FIELD_NONE;
    ret = cc_stream_import(be->streams[0], index, planes, num_planes);
    if (ret < 0) return ret;
    for (uint32_t i = 0; i < 2; ++i) {
        size_t port = 4U+2U*i;
        if (!(mask & (1U << port))) continue;
        ret = cc_stream_import(be->streams[port],index,&auxiliary[i].plane,1);
        if (ret < 0) {
            const size_t inputs[3] = {0,4,6};
            for (size_t j = 0; j < 3; ++j)
                if (inputs[j] < port && (mask & (1U << inputs[j])))
                    (void)cc_stream_unimport(be->streams[inputs[j]],index);
            return ret;
        }
    }
    be->job_ports = mask;
    for (size_t n = 0; n < 8; ++n) {
        size_t i = (n + 1) % 8;
        if (!(be->job_ports & (1U << i))) continue;
        const cc_frame *frame = i == 2 ? &parameters : !i ? input :
            input_port(i) ? &auxiliary[(i-4U)/2U].frame : NULL;
        ret = cc_stream_queue(be->streams[i], index, frame);
        if (ret < 0) return cancel_error(be, ret);
    }
    for (size_t n = 0; n < 8; ++n) {
        size_t i = (n + 1) % 8;
        if (!(be->job_ports & (1U << i))) continue;
        if (be->streams[i]->streaming) continue;
        ret = cc_stream_start(be->streams[i]);
        if (ret < 0) return cancel_error(be, ret);
    }
    be->pending = (cc_pisp_be_result) { .index = index, .cookie = cookie, .input = *input };
    be->pending.input.planes[0].data = NULL;
    for (uint32_t i = 0; i < 2; ++i) if (mask & (1U << (4U+2U*i))) {
        be->pending.auxiliary[2U*i] = auxiliary[i].frame;
        be->pending.auxiliary[2U*i].planes[0].data = NULL;
    }
    be->active = (int)index;
    be->states[index] = 1;
    be->done = 0;
    return 0;
}

static int accept(cc_pisp_be *be, size_t port, const cc_frame *frame)
{
    if (frame->index != (uint32_t)be->active) return cancel_error(be, -EPROTO);
    if (port == CC_PISP_BE_OUTPUT) be->pending.output = *frame;
    else if (port == CC_PISP_BE_OUTPUT1) be->pending.output1 = *frame;
    else if (port == CC_PISP_BE_PARAMETERS) be->pending.parameters = *frame;
    else if (!port) be->pending.input.flags |= frame->flags & V4L2_BUF_FLAG_ERROR;
    else if (input_port(port)) be->pending.auxiliary[port-4U].flags |= frame->flags & V4L2_BUF_FLAG_ERROR;
    else be->pending.auxiliary[port-4U] = *frame;
    be->done |= 1U << port;
    return 0;
}

int cc_pisp_be_dequeue(cc_pisp_be *be, cc_pisp_be_result *result, int timeout_ms)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (!result || timeout_ms < -1) return -EINVAL;
    if (be->active < 0) return -EAGAIN;
    uint64_t deadline = 0;
    if (timeout_ms > 0) {
        ret = be->io.ops->now(be->io.ctx, &deadline);
        if (ret < 0) return cancel_error(be, ret);
        uint64_t duration = (uint64_t)timeout_ms * UINT64_C(1000000);
        deadline = deadline > UINT64_MAX - duration ? UINT64_MAX : deadline + duration;
    }
    for (;;) {
        for (size_t i = 0; i < 8; ++i) {
            if (!(be->job_ports & (1U << i)) || be->done & (1U << i)) continue;
            cc_frame frame;
            ret = cc_stream_dequeue(be->streams[i], &frame, 0);
            if (ret == -EAGAIN) continue;
            if (ret < 0) return cancel_error(be, ret);
            ret = accept(be, i, &frame);
            if (ret < 0) return ret;
        }
        if (be->done == be->job_ports) {
            const size_t inputs[3] = {0,4,6};
            for (size_t i = 0; i < 3; ++i) if (be->job_ports & (1U << inputs[i])) {
                ret = cc_stream_unimport(be->streams[inputs[i]], (uint32_t)be->active);
                if (ret < 0) return cancel_error(be, ret);
            }
            be->states[be->active] = 2;
            be->active = -1;
            *result = be->pending;
            return 0;
        }
        if (!timeout_ms) return -EAGAIN;
        int remaining = -1;
        if (timeout_ms > 0) {
            uint64_t now;
            ret = be->io.ops->now(be->io.ctx, &now);
            if (ret < 0) return cancel_error(be, ret);
            if (now >= deadline) return -ETIMEDOUT;
            uint64_t ns = deadline - now;
            uint64_t ms = ns / UINT64_C(1000000) + (ns % UINT64_C(1000000) != 0);
            remaining = ms > INT_MAX ? INT_MAX : (int)ms;
        }
        for (size_t i = 0; i < 8; ++i) {
            if (!(be->job_ports & (1U << i)) || be->done & (1U << i)) continue;
            cc_frame frame;
            ret = cc_stream_dequeue(be->streams[i], &frame, remaining);
            if (ret == -ETIMEDOUT) return ret;
            if (ret < 0) return cancel_error(be, ret);
            ret = accept(be, i, &frame);
            if (ret < 0) return ret;
            break;
        }
    }
}

int cc_pisp_be_release(cc_pisp_be *be, uint32_t index)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (index >= be->count || be->states[index] != 2) return -EINVAL;
    for (uint32_t i = 0; i < 2; ++i) {
        cc_stream *stream = cc_be_image_stream(be, i);
        if (stream && stream->memory == V4L2_MEMORY_DMABUF) {
            ret = cc_stream_unimport(stream, index);
            if (ret < 0) return cancel_error(be, ret);
        }
    }
    be->states[index] = 0;
    return 0;
}

int cc_pisp_be_fd(const cc_pisp_be *be, enum cc_pisp_be_port port)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if ((unsigned int)port > CC_PISP_BE_STITCH_OUTPUT) return -EINVAL;
    return be->streams[port] ? cc_stream_fd(be->streams[port]) : -ENODEV;
}

int cc_pisp_be_export_plane(cc_pisp_be *be, uint32_t index, uint32_t plane)
{
    return cc_pisp_be_export_output_plane(be, 0, index, plane);
}

int cc_pisp_be_export_output_plane(cc_pisp_be *be, uint32_t output, uint32_t index, uint32_t plane)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if (output > 1 || (output && !be->configuration.output1.fourcc) || index >= be->count || be->states[index] != 2)
        return -EINVAL;
    return cc_stream_export_plane(be->streams[output ? CC_PISP_BE_OUTPUT1 : CC_PISP_BE_OUTPUT], index, plane);
}

int cc_pisp_be_export_port_plane(cc_pisp_be *be, enum cc_pisp_be_port port, uint32_t index, uint32_t plane)
{
    int ret = usable(be);
    if (ret < 0) return ret;
    if ((unsigned int)port > CC_PISP_BE_STITCH_OUTPUT || input_port((size_t)port) || port == CC_PISP_BE_PARAMETERS)
        return -EINVAL;
    if (!(ports(be) & (1U << port))) return -ENOENT;
    if (index >= be->count || be->states[index] != 2) return -EINVAL;
    return cc_stream_export_plane(be->streams[port],index,plane);
}
