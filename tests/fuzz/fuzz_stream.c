/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "internal.h"
#include <errno.h>
#include <limits.h>

struct driver {
    struct v4l2_format format;
    struct v4l2_buffer completion;
    struct v4l2_plane planes[CC_MAX_PLANES];
    struct v4l2_streamparm parameters;
    struct v4l2_frmivalenum interval;
};

/* No system calls or frame-sized allocations: inject only kernel replies into
 * an otherwise valid stream. Never follow pointers supplied by the input. */
static int driver_ioctl(void *ctx, int fd, unsigned long request, void *arg)
{
    (void)fd;
    struct driver *d = ctx;
    if (request == VIDIOC_G_FMT) { memcpy(arg, &d->format, sizeof(d->format)); return 0; }
    if (request==VIDIOC_G_PARM || request==VIDIOC_S_PARM) { memcpy(arg,&d->parameters,sizeof(d->parameters)); return 0; }
    if (request==VIDIOC_ENUM_FRAMEINTERVALS) { memcpy(arg,&d->interval,sizeof(d->interval)); return 0; }
    if (request == VIDIOC_DQBUF) {
        struct v4l2_buffer *b = arg;
        struct v4l2_plane *planes = b->m.planes;
        uint32_t capacity = b->length;
        if (V4L2_TYPE_IS_MULTIPLANAR(b->type)) {
            FUZZ_CHECK(capacity <= CC_MAX_PLANES);
            memcpy(planes, d->planes, capacity * sizeof(*planes));
        }
        *b = d->completion;
        b->m.planes = planes;
        return 0;
    }
    if (request == VIDIOC_QBUF || request == VIDIOC_STREAMON || request == VIDIOC_STREAMOFF) return 0;
    return -ENOTTY;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    const enum v4l2_buf_type types[] = {V4L2_BUF_TYPE_VIDEO_CAPTURE,
        V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_BUF_TYPE_META_CAPTURE, V4L2_BUF_TYPE_META_OUTPUT};
    enum v4l2_buf_type type = types[fuzz_u8(&in) % 6];
    bool multi = V4L2_TYPE_IS_MULTIPLANAR(type);
    uint32_t num_planes = multi ? 1U + fuzz_u8(&in) % CC_MAX_PLANES : 1;
    struct driver driver = {0};
    const struct cc_io_ops ops = {.ioctl = driver_ioctl};
    cc_video_device device = {.fd = 42, .io = {&ops, &driver}, .capabilities = V4L2_CAP_EXT_PIX_FORMAT};
    struct cc_buffer buffers[2] = {{0}};
    cc_stream stream = {.device = &device, .type = type, .memory = V4L2_MEMORY_MMAP,
        .configured = true, .allocated = true, .buffers = buffers, .num_buffers = 2,
        .format = {.fourcc = V4L2_PIX_FMT_RGB24, .width = 64, .height = 64, .num_planes = num_planes}};
    for (uint32_t i = 0; i < 2; ++i) {
        buffers[i].bound = true;
        buffers[i].frame.index = i; buffers[i].frame.num_planes = num_planes;
        for (uint32_t j = 0; j < num_planes; ++j)
            buffers[i].frame.planes[j].length = 1U + fuzz_u16(&in);
    }

    driver.format.type = type;
    if (multi) {
        driver.format.fmt.pix_mp = (struct v4l2_pix_format_mplane){.width = 64, .height = 64,
            .pixelformat = V4L2_PIX_FMT_RGB24, .num_planes = (uint8_t)num_planes};
        for (uint32_t i = 0; i < num_planes; ++i) driver.format.fmt.pix_mp.plane_fmt[i].sizeimage = 4096;
    } else if (type == V4L2_BUF_TYPE_META_CAPTURE || type == V4L2_BUF_TYPE_META_OUTPUT) {
        const uint32_t meta[] = {CC_FOURCC('T','E','S','T'), 4096, 0, 0, 0};
        memcpy(driver.format.fmt.raw_data, meta, sizeof(meta));
    } else driver.format.fmt.pix = (struct v4l2_pix_format){.width = 64, .height = 64,
        .pixelformat = V4L2_PIX_FMT_RGB24, .sizeimage = 4096, .priv = V4L2_PIX_FMT_PRIV_MAGIC};
    if (mode & 1U) fuzz_patch(&in, driver.format.fmt.raw_data, sizeof(driver.format.fmt.raw_data));
    cc_format format, before_format;
    memset(&format, 0xa5, sizeof(format)); memcpy(&before_format, &format, sizeof(format));
    int ret = cc_stream_get_format(&stream, &format);
    FUZZ_CHECK(ret == 0 || ret == -EPROTO);
    if (ret) FUZZ_CHECK(!memcmp(&format, &before_format, sizeof(format)));
    else {
        FUZZ_CHECK(format.fourcc && format.num_planes && format.num_planes <= CC_MAX_PLANES);
        for (uint32_t i = 0; i < format.num_planes; ++i) FUZZ_CHECK(format.planes[i].size);
    }

    cc_frame payload = buffers[0].frame;
    for (uint32_t i = 0; i < num_planes; ++i) {
        payload.planes[i].bytes_used = mode & 2U ? fuzz_u32(&in) : fuzz_u16(&in) % (payload.planes[i].length + 1U);
        payload.planes[i].data_offset = multi ? fuzz_u8(&in) % (payload.planes[i].bytes_used + 1U) : 0;
    }
    ret = cc_stream_queue(&stream, 0, V4L2_TYPE_IS_OUTPUT(type) ? &payload : NULL);
    if (ret) {
        FUZZ_CHECK(ret == -EINVAL && !stream.queued && !buffers[0].queued);
        FUZZ_CHECK(cc_stream_queue(&stream, 0, V4L2_TYPE_IS_OUTPUT(type) ? &buffers[0].frame : NULL) == 0);
    }
    FUZZ_CHECK(cc_stream_start(&stream) == 0);
    driver.completion = (struct v4l2_buffer){.type = type, .length = num_planes};
    driver.completion.index = mode & 4U ? fuzz_u32(&in) : 0;
    driver.completion.sequence = fuzz_u32(&in);
    driver.completion.flags = fuzz_u32(&in);
    driver.completion.timestamp.tv_sec = (long)fuzz_u64(&in);
    driver.completion.timestamp.tv_usec = (long)fuzz_u64(&in);
    if (!(mode & 8U)) {
        driver.completion.timestamp.tv_sec = (long)((uint64_t)driver.completion.timestamp.tv_sec % 100000U);
        driver.completion.timestamp.tv_usec = (long)((uint64_t)driver.completion.timestamp.tv_usec % 1000000U);
    }
    if (multi && (mode & 16U)) driver.completion.length = fuzz_u32(&in);
    for (uint32_t i = 0; i < num_planes; ++i) {
        uint32_t used = mode & 32U ? fuzz_u32(&in) : fuzz_u16(&in) % ((uint32_t)buffers[0].frame.planes[i].length + 1U);
        driver.planes[i].bytesused = used;
        driver.planes[i].data_offset = mode & 32U ? fuzz_u32(&in) : fuzz_u8(&in) % (used + 1U);
        if (!multi) driver.completion.bytesused = used;
    }
    cc_frame frame, before;
    memset(&frame, 0xa5, sizeof(frame)); memcpy(&before, &frame, sizeof(frame));
    ret = cc_stream_dequeue(&stream, &frame, 0);
    FUZZ_CHECK(ret == 0 || ret == -EPROTO);
    if (ret) {
        FUZZ_CHECK(!memcmp(&frame, &before, sizeof(frame)) && stream.poisoned);
        FUZZ_CHECK(cc_stream_dequeue(&stream, &frame, 0) == -EPIPE);
    } else {
        FUZZ_CHECK(frame.index == 0 && frame.num_planes == num_planes && !stream.queued && !stream.poisoned);
        for (uint32_t i = 0; i < frame.num_planes; ++i)
            FUZZ_CHECK(frame.planes[i].data_offset <= frame.planes[i].bytes_used &&
                       frame.planes[i].bytes_used <= frame.planes[i].length);
    }
    FUZZ_CHECK(cc_stream_stop(&stream) == 0 && !stream.poisoned && !stream.queued && !stream.streaming);
    FUZZ_CHECK(cc_stream_queue(&stream, 0, V4L2_TYPE_IS_OUTPUT(type) ? &buffers[0].frame : NULL) == 0);
    FUZZ_CHECK(cc_stream_start(&stream) == 0);
    driver.completion = (struct v4l2_buffer){.type = type, .length = num_planes};
    memset(driver.planes, 0, sizeof(driver.planes));
    FUZZ_CHECK(cc_stream_dequeue(&stream, &frame, 0) == 0);
    FUZZ_CHECK(cc_stream_stop(&stream) == 0);
    driver.parameters=(struct v4l2_streamparm){.type=type};
    driver.parameters.parm.capture.capability=(fuzz_u8(&in)&1U) ? V4L2_CAP_TIMEPERFRAME:0;
    driver.parameters.parm.capture.timeperframe=(struct v4l2_fract){fuzz_u32(&in),fuzz_u32(&in)};
    driver.interval=(struct v4l2_frmivalenum){.pixel_format=V4L2_PIX_FMT_RGB24,.width=640,.height=480,
        .type=fuzz_u8(&in)%5U};
    driver.interval.stepwise=(struct v4l2_frmival_stepwise){{fuzz_u32(&in),fuzz_u32(&in)},
        {fuzz_u32(&in),fuzz_u32(&in)},{fuzz_u32(&in),fuzz_u32(&in)}};
    if (mode&64U) {
        fuzz_patch(&in,&driver.parameters,sizeof(driver.parameters));
        fuzz_patch(&in,&driver.interval,sizeof(driver.interval));
    }
    cc_video_device timing_device={.fd=42,.io={&ops,&driver}};
    cc_stream timing_stream={.device=&timing_device,.type=type,.configured=true}; timing_device.streams=&timing_stream;
    cc_fraction period={123,456},original_period=period;
    ret=cc_stream_get_frame_interval(&timing_stream,&period);
    if (ret) FUZZ_CHECK(!memcmp(&period,&original_period,sizeof(period)));
    else FUZZ_CHECK(period.numerator && period.denominator);
    period=(cc_fraction){fuzz_u32(&in),fuzz_u32(&in)};
    if (!(mode&128U)) period=(cc_fraction){1,25};
    original_period=period;
    ret=cc_stream_set_frame_interval(&timing_stream,&period);
    if (ret) FUZZ_CHECK(!memcmp(&period,&original_period,sizeof(period)));
    else FUZZ_CHECK(period.numerator && period.denominator);
    cc_frame_interval_range periods,saved_periods; memset(&periods,0xa5,sizeof(periods)); memcpy(&saved_periods,&periods,sizeof(periods));
    ret=cc_stream_enum_frame_interval(&timing_stream,V4L2_PIX_FMT_RGB24,640,480,0,&periods);
    if (ret) FUZZ_CHECK(!memcmp(&periods,&saved_periods,sizeof(periods)));
    else {
        FUZZ_CHECK(periods.minimum.numerator && periods.minimum.denominator && periods.maximum.numerator && periods.maximum.denominator);
        FUZZ_CHECK((uint64_t)periods.minimum.numerator*periods.maximum.denominator<=
            (uint64_t)periods.maximum.numerator*periods.minimum.denominator);
        if (periods.type==CC_INTERVAL_DISCRETE) FUZZ_CHECK(!memcmp(&periods.minimum,&periods.maximum,sizeof(cc_fraction)) && !periods.step.numerator);
        else FUZZ_CHECK(periods.step.numerator && periods.step.denominator);
    }
    return 0;
}
