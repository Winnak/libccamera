/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Selection policy follows CameraSensorLegacy::getFormat() in the archived
 * libcamera 0.7.2 reference, src/libcamera/sensor/camera_sensor_legacy.cpp
 * (Copyright (C) 2019, Google Inc.). Exact integer ratios and 64-bit areas
 * avoid rounding and overflow for large native dimensions. */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/sensor.h>
#include <errno.h>

#define CC_SENSOR_MODE_CODE_LIMIT 256U
#define CC_SENSOR_MODE_SIZE_LIMIT 4096U

static int enum_code(cc_camera *camera, const cc_sensor_stream_info *image,
                     uint32_t index, uint32_t *out)
{
    struct cc_subdev_mbus_code_enum q = {
        .pad=image->pad, .stream=image->stream, .index=index, .which=V4L2_SUBDEV_FORMAT_ACTIVE
    };
    int ret = cc_ioctl(&camera->io, camera->sensor_fd, VIDIOC_SUBDEV_ENUM_MBUS_CODE, &q);
    if (ret < 0) return ret == -EINVAL ? -ENOENT : ret == -ENOTTY ? -ENOTSUP : ret;
    if (q.pad != image->pad || q.stream != image->stream || q.index != index ||
        q.which != V4L2_SUBDEV_FORMAT_ACTIVE || !q.code) return -EPROTO;
    *out=q.code;
    return 0;
}

static int enum_size(cc_camera *camera, const cc_sensor_stream_info *image,
                     uint32_t code, uint32_t index, cc_sensor_mode *out)
{
    struct cc_subdev_frame_size_enum q = {
        .pad=image->pad, .stream=image->stream, .code=code, .index=index,
        .which=V4L2_SUBDEV_FORMAT_ACTIVE
    };
    int ret = cc_ioctl(&camera->io, camera->sensor_fd, VIDIOC_SUBDEV_ENUM_FRAME_SIZE, &q);
    if (ret < 0) return ret == -EINVAL ? -ENOENT : ret == -ENOTTY ? -ENOTSUP : ret;
    if (q.pad != image->pad || q.stream != image->stream || q.code != code || q.index != index ||
        q.which != V4L2_SUBDEV_FORMAT_ACTIVE || !q.min_width || !q.min_height ||
        q.max_width < q.min_width || q.max_height < q.min_height) return -EPROTO;
    *out=(cc_sensor_mode){.mbus_code=code,
        .sizes={q.min_width,q.max_width,1,q.min_height,q.max_height,1}};
    return 0;
}

int cc_camera_enum_sensor_mode(cc_camera *camera, uint32_t code_index,
                               uint32_t size_index, cc_sensor_mode *out)
{
    if (!camera || !out) return -EINVAL;
    if (camera->sensor_fd < 0) return -ENOTSUP;
    cc_sensor_streams streams;
    int ret=cc_camera_get_sensor_streams(camera,&streams);
    if (ret < 0) return ret;
    uint32_t code;
    ret=enum_code(camera,&streams.image,code_index,&code);
    return ret < 0 ? ret : enum_size(camera,&streams.image,code,size_index,out);
}

/* Compare a/b and c/d without a potentially overflowing 64x32 product.
 * After the integer quotient, remainders are below the 32-bit denominators. */
static int compare_ratio(uint64_t a, uint32_t b, uint64_t c, uint32_t d)
{
    uint64_t x=a/b, y=c/d;
    if (x != y) return x < y ? -1 : 1;
    x=(a%b)*d; y=(c%d)*b;
    return x < y ? -1 : x > y ? 1 : 0;
}

int cc_camera_select_sensor_mode(cc_camera *camera, const cc_sensor_mode_request *request,
                                 cc_sensor_format *out)
{
    if (!camera || !request || !out || !request->mbus_codes || !request->num_mbus_codes ||
        !request->width || !request->height || (!request->max_width != !request->max_height) ||
        (request->max_width && (request->max_width < request->width || request->max_height < request->height)))
        return -EINVAL;
    if (request->num_mbus_codes > CC_SENSOR_MODE_CODE_LIMIT) return -E2BIG;
    for (size_t i=0; i<request->num_mbus_codes; ++i)
        if (!request->mbus_codes[i]) return -EINVAL;
    if (camera->sensor_fd < 0) return -ENOTSUP;
    cc_sensor_streams streams;
    int ret=cc_camera_get_sensor_streams(camera,&streams);
    if (ret < 0) return ret;

    cc_sensor_format best={0};
    uint64_t best_ratio=0, best_area=0;
    size_t best_preference=0;
    uint32_t seen[CC_SENSOR_MODE_CODE_LIMIT], total_sizes=0;
    for (uint32_t c=0; ; ++c) {
        uint32_t code;
        ret=enum_code(camera,&streams.image,c,&code);
        if (ret == -ENOENT) break;
        if (ret < 0) return ret;
        if (c == CC_SENSOR_MODE_CODE_LIMIT) return -E2BIG;
        for (uint32_t i=0; i<c; ++i)
            if (seen[i] == code) return -EPROTO;
        seen[c]=code;
        size_t preference=0;
        while (preference < request->num_mbus_codes && request->mbus_codes[preference] != code) ++preference;
        /* Unaccepted codes cannot supply a candidate and need no size ioctls. */
        if (preference == request->num_mbus_codes) continue;
        for (uint32_t s=0; ; ++s) {
            cc_sensor_mode mode;
            ret=enum_size(camera,&streams.image,code,s,&mode);
            if (ret == -ENOENT) break;
            if (ret < 0) return ret;
            if (total_sizes++ == CC_SENSOR_MODE_SIZE_LIMIT) return -E2BIG;
            uint32_t width=mode.sizes.max_width, height=mode.sizes.max_height;
            if (width < request->width || height < request->height ||
                (request->max_width && (width > request->max_width || height > request->max_height))) continue;
            /* The common request height cancels in the aspect-ratio comparison. */
            uint64_t a=(uint64_t)width*request->height, b=(uint64_t)request->width*height;
            uint64_t ratio=a > b ? a-b : b-a, area=(uint64_t)width*height;
            int comparison=best.mbus_code ? compare_ratio(ratio,height,best_ratio,best.height) : -1;
            if (comparison < 0 || (!comparison && (area < best_area ||
                (area == best_area && preference < best_preference)))) {
                best=(cc_sensor_format){code,width,height};
                best_ratio=ratio; best_area=area; best_preference=preference;
            }
        }
    }
    if (!best.mbus_code) return -ENOENT;
    *out=best;
    return 0;
}
