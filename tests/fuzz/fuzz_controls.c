/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "internal.h"
#include <ccamera/vimc.h>
#include <errno.h>
#include <float.h>

enum { MAX_CONTROLS = 8, PAYLOAD_SIZE = 64 };
struct driver {
    struct v4l2_query_ext_ctrl info[MAX_CONTROLS];
    int64_t values[MAX_CONTROLS];
    uint32_t sizes[MAX_CONTROLS], error_index;
    int query_error, batch_error;
    bool called;
};

static int driver_ioctl(void *ctx, int fd, unsigned long request, void *arg)
{
    (void)fd;
    struct driver *d = ctx;
    if (request == VIDIOC_QUERY_EXT_CTRL) {
        struct v4l2_query_ext_ctrl *q = arg;
        uint32_t id = q->id & ~(V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND);
        FUZZ_CHECK(id && id <= MAX_CONTROLS);
        if (d->query_error) return d->query_error;
        *q = d->info[id - 1U];
        return 0;
    }
    FUZZ_CHECK(request == VIDIOC_G_EXT_CTRLS || request == VIDIOC_S_EXT_CTRLS);
    struct v4l2_ext_controls *batch = arg;
    FUZZ_CHECK(batch->count <= MAX_CONTROLS);
    d->called = true; batch->error_idx = d->error_index;
    for (uint32_t i = 0; i < batch->count; ++i) {
        struct v4l2_ext_control *item = &batch->controls[i];
        FUZZ_CHECK(item->id && item->id <= MAX_CONTROLS);
        uint32_t n = item->id - 1U;
        if (d->info[n].flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) {
            FUZZ_CHECK(item->ptr && item->size && item->size <= PAYLOAD_SIZE);
            if (!d->batch_error && request == VIDIOC_G_EXT_CTRLS)
                memset(item->ptr, (int)(n + 1U), item->size);
            item->size = d->sizes[n];
        } else if (d->info[n].type == V4L2_CTRL_TYPE_INTEGER64) item->value64 = d->values[n];
        else item->value = (int32_t)d->values[n];
    }
    return d->batch_error;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    struct fuzz_input vimc_input = in;
    cc_vimc_controls vimc={.flags=fuzz_u8(&vimc_input)};
    double *components[]={&vimc.brightness,&vimc.contrast,&vimc.saturation};
    bool valid=(vimc.flags&~UINT32_C(7))==0;
    size_t required=0;
    for (size_t i=0;i<3;++i) {
        uint64_t bits=fuzz_u64(&vimc_input);
        _Static_assert(sizeof(bits)==sizeof(double),"binary64 fuzz inputs");
        memcpy(components[i],&bits,sizeof(bits));
        if (vimc.flags&(1U<<i)) {
            valid &= *components[i]>=-DBL_MAX && *components[i]<=DBL_MAX;
            ++required;
        }
    }
    cc_control converted[3], prior[3];
    memset(converted,0xa5,sizeof(converted)); memcpy(prior,converted,sizeof(prior));
    size_t capacity=fuzz_u8(&vimc_input)%4, converted_count=capacity;
    int conversion=cc_vimc_controls_to_native(&vimc,converted,&converted_count);
    FUZZ_CHECK(conversion==(valid ? capacity<required ? -ENOSPC:0:-EINVAL));
    if (conversion) {
        FUZZ_CHECK(!memcmp(converted,prior,sizeof(prior)));
        FUZZ_CHECK(converted_count==(valid ? required:capacity));
    } else {
        FUZZ_CHECK(converted_count==required);
        for (size_t i=0;i<converted_count;++i) FUZZ_CHECK(converted[i].value>=0 && converted[i].value<=255 &&
            converted[i].id>=V4L2_CID_BRIGHTNESS && converted[i].id<=V4L2_CID_SATURATION && !converted[i].data && !converted[i].size);
    }
    uint8_t mode = fuzz_u8(&in);
    size_t count = fuzz_u8(&in) % (MAX_CONTROLS + 1U);
    const uint32_t types[] = {V4L2_CTRL_TYPE_INTEGER, V4L2_CTRL_TYPE_INTEGER64,
        V4L2_CTRL_TYPE_BITMASK, V4L2_CTRL_TYPE_BOOLEAN, V4L2_CTRL_TYPE_STRING, V4L2_CTRL_TYPE_U8};
    const int errors[] = {0, -ENOSPC, -EIO, -EINVAL};
    struct driver d = {.query_error = mode & 16U ? -EIO : 0,
        .batch_error = errors[fuzz_u8(&in) % 4], .error_index = fuzz_u32(&in)};
    cc_control controls[MAX_CONTROLS] = {{0}}, original[MAX_CONTROLS];
    uint8_t payloads[MAX_CONTROLS][PAYLOAD_SIZE] = {{0}};
    for (uint32_t i = 0; i < MAX_CONTROLS; ++i) {
        struct v4l2_query_ext_ctrl *q = &d.info[i];
        q->id = i + 1U; q->type = types[fuzz_u8(&in) % 6];
        q->flags = fuzz_u8(&in);
        if (q->flags & 1U) q->flags |= V4L2_CTRL_FLAG_HAS_PAYLOAD;
        if (q->flags & 2U) q->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
        q->nr_of_dims = mode & 8U ? fuzz_u32(&in) : fuzz_u8(&in) % 5U;
        q->elem_size = fuzz_u32(&in); q->elems = fuzz_u32(&in);
        memset(q->name, 'A' + (int)i, sizeof(q->name));
        d.values[i] = (int64_t)fuzz_u64(&in); d.sizes[i] = fuzz_u32(&in);
        controls[i] = (cc_control){.id = i + 1U, .value = (int64_t)fuzz_u64(&in)};
        if (!(mode & 4U)) controls[i].value = (int32_t)controls[i].value;
        if (q->flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) {
            controls[i].data = payloads[i]; controls[i].size = 1U + fuzz_u8(&in) % PAYLOAD_SIZE;
            if (mode & 32U) controls[i].size = 0;
        }
    }
    memcpy(original, controls, sizeof(controls));
    const struct cc_io_ops ops = {.ioctl = driver_ioctl};
    cc_stream stream = {.configured = true, .allocated = (mode & 64U) != 0};
    cc_video_device device = {.fd = 42, .io = {&ops, &d}, .streams = &stream};
    cc_control_info info, before;
    memset(&info, 0xa5, sizeof(info)); memcpy(&before, &info, sizeof(info));
    int ret = cc_video_query_control(&device, 1, &info);
    if (ret) FUZZ_CHECK(!memcmp(&info, &before, sizeof(info)));
    else FUZZ_CHECK(info.nr_of_dims <= 4 && info.name[sizeof(info.name) - 1] == '\0');
    size_t error_index = SIZE_MAX;
    switch (mode % 3U) {
    case 0: ret = cc_video_get_controls(&device, controls, count, &error_index); break;
    case 1: ret = cc_video_set_controls(&device, controls, count, &error_index); break;
    default: ret = cc_controls_apply_io(&device.io, device.fd, controls, count, stream.allocated); break;
    }
    FUZZ_CHECK(ret <= 0);
    if (mode % 3U != 2U) FUZZ_CHECK(error_index <= count);
    for (size_t i = 0; i < count; ++i) {
        FUZZ_CHECK(controls[i].id == original[i].id && controls[i].data == original[i].data);
        bool outputs = mode % 3U != 1U;
        bool payload = (d.info[i].flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) != 0;
        if (!outputs || !d.called) {
            FUZZ_CHECK(controls[i].value == original[i].value && controls[i].size == original[i].size);
        } else if (payload) FUZZ_CHECK(controls[i].size == d.sizes[i]);
        else if (ret) FUZZ_CHECK(controls[i].value == original[i].value);
        else if (d.info[i].type == V4L2_CTRL_TYPE_INTEGER64) FUZZ_CHECK(controls[i].value == d.values[i]);
        else if (d.info[i].type == V4L2_CTRL_TYPE_BITMASK) FUZZ_CHECK(controls[i].value == (uint32_t)d.values[i]);
        else FUZZ_CHECK(controls[i].value == (int32_t)d.values[i]);
    }
    return 0;
}
