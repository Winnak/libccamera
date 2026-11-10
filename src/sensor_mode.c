/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Selection policy follows CameraSensorLegacy::getFormat() in the archived
 * libcamera 0.7.2 reference, src/libcamera/sensor/camera_sensor_legacy.cpp
 * (Copyright (C) 2019, Google Inc.). Exact integer ratios and 64-bit areas
 * avoid rounding and overflow for large native dimensions. */
/* Pi scoring follows CameraData::scoreFormat()/findBestFormat() in the same
 * archive, src/libcamera/pipeline/rpi/common/pipeline_base.cpp
 * (Copyright (C) 2019-2023, Raspberry Pi Ltd). */
#include "internal.h"
#include "subdev_compat.h"
#include <ccamera/sensor.h>
#include <errno.h>
#include <linux/media-bus-format.h>

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

static uint32_t raw_depth(uint32_t code)
{
    uint32_t depth=0;
    (void)cc_bayer_code_info(code,&depth,NULL);
    return depth;
}

static double rpi_error(double desired, double actual)
{
    double score=desired-actual;
    if (score < 0.0) score=-score/8;
    if (actual != desired) score*=2;
    return score;
}

static double rpi_score(const cc_sensor_mode_request *request, uint32_t width, uint32_t height,
                        uint32_t depth, uint32_t desired_depth)
{
    double score=rpi_error(request->width,width);
    score+=rpi_error(request->height,height);
    score+=1500.0*rpi_error((double)request->width/request->height,(double)width/height);
    score+=(depth > desired_depth ? depth-desired_depth : desired_depth-depth)*500.0;
    return score;
}

/* desired_depth==0 selects the generic policy. Both policies share bounded,
 * read-only enumeration and publish a result only after every ioctl succeeds. */
static int select_mode(cc_camera *camera, const cc_sensor_mode_request *request,
                        uint32_t desired_depth, cc_sensor_format *out)
{
    if (!camera || !request || !out || !request->mbus_codes || !request->num_mbus_codes ||
        !request->width || !request->height || (!request->max_width != !request->max_height) ||
        (request->max_width && (request->max_width < request->width || request->max_height < request->height)))
        return -EINVAL;
    if (request->num_mbus_codes > CC_SENSOR_MODE_CODE_LIMIT) return -E2BIG;
    for (size_t i=0; i<request->num_mbus_codes; ++i) {
        if (!request->mbus_codes[i]) return -EINVAL;
        if (desired_depth && !raw_depth(request->mbus_codes[i])) return -ENOTSUP;
    }
    if (camera->sensor_fd < 0) return -ENOTSUP;
    cc_sensor_streams streams;
    int ret=cc_camera_get_sensor_streams(camera,&streams);
    if (ret < 0) return ret;

    cc_sensor_format best={0};
    uint64_t best_ratio=0, best_area=0;
    double best_score=0.0;
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
            if (!desired_depth && (width < request->width || height < request->height ||
                (request->max_width && (width > request->max_width || height > request->max_height)))) continue;
            if (desired_depth) {
                double score=rpi_score(request,width,height,raw_depth(code),desired_depth);
                uint64_t area=(uint64_t)width*height;
                /* Upstream traverses ascending code and Size order, replacing
                 * on <=. Resolve ties explicitly to avoid driver-order effects. */
                if (!best.mbus_code || score < best_score || (score == best_score &&
                    (code > best.mbus_code || (code == best.mbus_code &&
                    (area > best_area || (area == best_area && width > best.width)))))) {
                    best=(cc_sensor_format){code,width,height};
                    best_score=score; best_area=area;
                }
                continue;
            }
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

int cc_camera_select_sensor_mode(cc_camera *camera, const cc_sensor_mode_request *request,
                                 cc_sensor_format *out)
{
    return select_mode(camera,request,0,out);
}

int cc_camera_select_rpi_sensor_mode(cc_camera *camera, const cc_rpi_sensor_mode_request *request,
                                     cc_sensor_format *out)
{
    if (!request || (request->bit_depth != 8 && request->bit_depth != 10 && request->bit_depth != 12 &&
        request->bit_depth != 14 && request->bit_depth != 16)) return -EINVAL;
    cc_sensor_mode_request generic={.mbus_codes=request->mbus_codes,.num_mbus_codes=request->num_mbus_codes,
        .width=request->width,.height=request->height};
    return select_mode(camera,&generic,request->bit_depth,out);
}

/* Same bounded enumeration as selection, including arbitrary image streams.
 * Largest Size and lowest Bayer code follow the reference's sorted traversal. */
int cc_sensor_mode_properties(cc_camera *camera, uint32_t cfa_flip, cc_camera_properties *p)
{
    cc_sensor_streams streams;
    int ret=cc_camera_get_sensor_streams(camera,&streams);
    if (ret < 0) return ret;
    uint32_t seen[CC_SENSOR_MODE_CODE_LIMIT], total_sizes=0, cfa_code=UINT32_MAX;
    uint64_t best_area=0;
    for (uint32_t c=0; ; ++c) {
        uint32_t code;
        ret=enum_code(camera,&streams.image,c,&code);
        if (ret == -ENOENT) break;
        if (ret < 0) return ret;
        if (c == CC_SENSOR_MODE_CODE_LIMIT) return -E2BIG;
        for (uint32_t i=0; i<c; ++i) if (seen[i] == code) return -EPROTO;
        seen[c]=code;
        for (uint32_t s=0; ; ++s) {
            cc_sensor_mode mode;
            ret=enum_size(camera,&streams.image,code,s,&mode);
            if (ret == -ENOENT) break;
            if (ret < 0) return ret;
            if (total_sizes++ == CC_SENSOR_MODE_SIZE_LIMIT) return -E2BIG;
            uint64_t area=(uint64_t)mode.sizes.max_width*mode.sizes.max_height;
            if (area > best_area || (area == best_area && mode.sizes.max_width > p->pixel_array_width)) {
                p->pixel_array_width=mode.sizes.max_width;
                p->pixel_array_height=mode.sizes.max_height;
                best_area=area;
                p->flags|=CC_CAMERA_PROPERTY_PIXEL_ARRAY;
            }
            enum cc_camera_cfa cfa;
            if (camera->info.pipeline != CC_PIPELINE_VIMC && !cc_bayer_code_info(code,NULL,&cfa)) {
                /* Drivers can advertise the live flipped Bayer code. Undo
                 * layout-changing flips before applying upstream code order. */
                if (cfa!=CC_CAMERA_CFA_MONO) cfa=(enum cc_camera_cfa)((uint32_t)cfa^cfa_flip);
                uint32_t native_code=cc_bayer_code(raw_depth(code),cfa);
                if (native_code<cfa_code) {
                    p->cfa=cfa; cfa_code=native_code; p->flags|=CC_CAMERA_PROPERTY_CFA;
                }
            }
        }
    }
    return 0;
}
