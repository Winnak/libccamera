/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include <errno.h>
#include <linux/media-bus-format.h>
#include <string.h>

#define CC_SENSOR_ROUTE_LIMIT 64U
#define CC_SENSOR_CODE_LIMIT 256U
enum format_kind { UNKNOWN, IMAGE, METADATA, EMBEDDED };
static enum format_kind kind(uint32_t code)
{
    /* Linux UAPI categories, including specific metadata codes absent from
     * older installed headers. CCS and OV2740 describe embedded register data. */
    if (code == 0x9001 || code == 0x9002) return EMBEDDED;
    if (code == 0x7001 || (code >= 0x8001 && code <= 0x8007)) return METADATA;
    if (code >= 0x1000 && code < 0x7000) return IMAGE;
    return UNKNOWN;
}
static size_t sensor_index(const cc_camera *camera)
{
    for (size_t i = 0; i < camera->media->num_entities; ++i)
        if (camera->media->entities[i].desc.id == camera->info.sensor_entity) return i;
    return camera->media->num_entities;
}
static int routes_get(cc_camera *camera, uint32_t which, struct cc_subdev_route *routes, uint32_t *count)
{
    struct cc_subdev_routing routing = {.which=which,.len_routes=CC_SENSOR_ROUTE_LIMIT,
        .num_routes=CC_SENSOR_ROUTE_LIMIT,.routes=(uint64_t)(uintptr_t)routes};
    int ret = cc_ioctl(&camera->io,camera->sensor_fd,CC_SD_G_ROUTING,&routing);
    if (ret == -ENOTTY) {
        struct cc_subdev_routing_legacy old = {.which=which,.num_routes=CC_SENSOR_ROUTE_LIMIT,.routes=routing.routes};
        ret = cc_ioctl(&camera->io,camera->sensor_fd,CC_SD_G_ROUTING_LEGACY,&old);
        if (old.routes != routing.routes || old.which != which) return ret < 0 ? ret : -EPROTO;
        routing.num_routes = old.num_routes;
    }
    if (ret < 0) return ret;
    if (routing.routes != (uint64_t)(uintptr_t)routes || routing.len_routes != CC_SENSOR_ROUTE_LIMIT || routing.which != which)
        return -EPROTO;
    if (routing.num_routes > CC_SENSOR_ROUTE_LIMIT) return -E2BIG;
    if (!routing.num_routes) return -ENODATA;
    *count = routing.num_routes;
    return 0;
}
static int format_info(cc_camera *camera, uint32_t which, uint32_t pad, uint32_t stream,
                        cc_sensor_stream_info *info)
{
    struct cc_subdev_format format = {.which=which,.pad=pad,.stream=stream};
    int ret = cc_ioctl(&camera->io,camera->sensor_fd,VIDIOC_SUBDEV_G_FMT,&format);
    if (ret < 0) return ret;
    if (format.which != which || format.pad != pad || format.stream != stream) return -EPROTO;
    *info = (cc_sensor_stream_info){.pad=pad,.stream=stream,.mbus_code=format.format.code,
        .width=format.format.width,.height=format.format.height};
    return 0;
}
static int formats_kind(cc_camera *camera, uint32_t which, uint32_t pad, uint32_t stream,
                         enum format_kind *type, uint32_t *count)
{
    enum format_kind result = UNKNOWN;
    for (uint32_t i = 0; i < CC_SENSOR_CODE_LIMIT; ++i) {
        struct cc_subdev_mbus_code_enum code = {.pad=pad,.stream=stream,.which=which,.index=i};
        int ret = cc_ioctl(&camera->io,camera->sensor_fd,VIDIOC_SUBDEV_ENUM_MBUS_CODE,&code);
        if (ret == -EINVAL) { *type=result; *count=i; return 0; }
        if (ret < 0) return ret;
        if (code.pad != pad || code.stream != stream || code.which != which || code.index != i) return -EPROTO;
        enum format_kind next = kind(code.code);
        if (next == UNKNOWN) continue;
        if (result != UNKNOWN && result != next) return -EPROTO;
        result = next;
    }
    return -E2BIG;
}
static int discover_multiplexed(cc_camera *camera, uint32_t which, const struct cc_entity *sensor,
                                 struct cc_subdev_route *routes, uint32_t *count, cc_sensor_streams *streams)
{
    int ret = routes_get(camera,which,routes,count);
    if (ret < 0) return ret;
    bool image_found=false, embedded_found=false;
    cc_sensor_streams result = {.flags=CC_SENSOR_STREAMS_MULTIPLEXED};
    for (uint32_t i=0;i<*count;++i) {
        const struct cc_subdev_route *route = &routes[i];
        if (route->source_pad >= sensor->desc.pads || route->sink_pad >= sensor->desc.pads ||
            !(sensor->pads[route->source_pad].flags & MEDIA_PAD_FL_SOURCE) ||
            !(sensor->pads[route->sink_pad].flags & MEDIA_PAD_FL_SINK) || route->source_pad != camera->sensor_pad)
            return -EPROTO;
        for (uint32_t j=0;j<i;++j)
            if (routes[j].source_pad == route->source_pad && routes[j].source_stream == route->source_stream)
                return -EPROTO;
        enum format_kind type;
        uint32_t codes;
        ret = formats_kind(camera,which,route->source_pad,route->source_stream,&type,&codes);
        if (ret < 0) return ret;
        if (type == METADATA) {
            enum format_kind sink_type;
            ret = formats_kind(camera,which,route->sink_pad,route->sink_stream,&sink_type,&codes);
            if (ret < 0) return ret;
            if (sink_type != EMBEDDED || codes != 1) continue;
            type = EMBEDDED;
        }
        if (type != IMAGE && type != EMBEDDED) continue;
        bool *found = type == IMAGE ? &image_found : &embedded_found;
        if (*found) return -ENOTUNIQ;
        cc_sensor_stream_info *info = type == IMAGE ? &result.image : &result.embedded;
        ret = format_info(camera,which,route->source_pad,route->source_stream,info);
        if (ret < 0) return ret;
        enum format_kind actual = kind(info->mbus_code);
        if ((type == IMAGE && actual != IMAGE) ||
            (type == EMBEDDED && actual != METADATA && actual != EMBEDDED)) return -EPROTO;
        if (route->flags & CC_SD_ROUTE_ACTIVE) info->flags |= CC_SENSOR_STREAM_ACTIVE;
        *found = true;
    }
    if (!image_found) return -ENOTSUP;
    if (embedded_found) result.flags |= CC_SENSOR_STREAMS_EMBEDDED;
    *streams=result;
    return 0;
}
int cc_sensor_streams_discover(cc_camera *camera, uint32_t which, cc_sensor_streams *streams)
{
    if (!camera || !streams) return -EINVAL;
    if (!camera->media || camera->sensor_fd < 0) return -ENOTSUP;
    size_t index=sensor_index(camera);
    if (index == camera->media->num_entities) return -ENODEV;
    const struct cc_entity *sensor=&camera->media->entities[index];
    cc_sensor_streams result={0};
    if (camera->subdev_streams[index]) {
        struct cc_subdev_route routes[CC_SENSOR_ROUTE_LIMIT]={{0}};
        uint32_t count;
        int ret=discover_multiplexed(camera,which,sensor,routes,&count,&result);
        if (ret<0) return ret;
    } else {
        int ret=format_info(camera,which,camera->sensor_pad,0,&result.image);
        if (ret<0) return ret;
        if (kind(result.image.mbus_code) != IMAGE) return -ENOTSUP;
        result.image.flags=CC_SENSOR_STREAM_ACTIVE;
        for (uint32_t pad=0;pad<sensor->desc.pads;++pad) {
            if (pad == camera->sensor_pad || !(sensor->pads[pad].flags & MEDIA_PAD_FL_SOURCE)) continue;
            cc_sensor_stream_info info;
            ret=format_info(camera,which,pad,0,&info);
            if (ret == -EINVAL || ret == -ENOTTY) continue;
            if (ret<0) return ret;
            enum format_kind type=kind(info.mbus_code);
            if (type != METADATA && type != EMBEDDED) continue;
            if (result.flags & CC_SENSOR_STREAMS_EMBEDDED) return -ENOTUNIQ;
            result.flags |= CC_SENSOR_STREAMS_EMBEDDED;
            result.embedded=info; result.embedded.flags=CC_SENSOR_STREAM_ACTIVE;
        }
    }
    *streams=result;
    return 0;
}
int cc_camera_get_sensor_streams(cc_camera *camera, cc_sensor_streams *streams)
{
    return cc_sensor_streams_discover(camera,V4L2_SUBDEV_FORMAT_ACTIVE,streams);
}
int cc_sensor_streams_route(cc_camera *camera, uint32_t which, bool embedded, uint32_t *stream)
{
    size_t index=sensor_index(camera);
    if (index == camera->media->num_entities) return -ENODEV;
    if (!camera->subdev_streams[index]) { *stream=0; return 0; }
    struct cc_subdev_route routes[CC_SENSOR_ROUTE_LIMIT]={{0}};
    uint32_t count;
    cc_sensor_streams streams;
    int ret=discover_multiplexed(camera,which,&camera->media->entities[index],routes,&count,&streams);
    if (ret<0) return ret;
    if (embedded && !(streams.flags & CC_SENSOR_STREAMS_EMBEDDED)) return -ENOTSUP;
    /* Keep unrelated internal routes as reported. Select the discovered image
     * and disable embedded data until a receiver/capture queue owns that stream. */
    for (uint32_t i=0;i<count;++i) {
        if (routes[i].source_stream == streams.image.stream) routes[i].flags |= CC_SD_ROUTE_ACTIVE;
        if ((streams.flags & CC_SENSOR_STREAMS_EMBEDDED) && routes[i].source_stream == streams.embedded.stream)
            routes[i].flags=embedded ? routes[i].flags|CC_SD_ROUTE_ACTIVE:routes[i].flags&~CC_SD_ROUTE_ACTIVE;
    }
    struct cc_subdev_route expected[CC_SENSOR_ROUTE_LIMIT];
    memcpy(expected,routes,count*sizeof(*routes));
    struct cc_subdev_routing routing={.which=which,.len_routes=count,.num_routes=count,.routes=(uint64_t)(uintptr_t)routes};
    ret=cc_ioctl(&camera->io,camera->sensor_fd,CC_SD_S_ROUTING,&routing);
    if (ret == -ENOTTY) {
        struct cc_subdev_routing_legacy old={.which=which,.num_routes=count,.routes=routing.routes};
        ret=cc_ioctl(&camera->io,camera->sensor_fd,CC_SD_S_ROUTING_LEGACY,&old);
        if (old.which != which || old.routes != routing.routes) return ret<0 ? ret:-EPROTO;
        routing.num_routes=old.num_routes;
    }
    if (ret<0) return ret;
    if (routing.which != which || routing.routes != (uint64_t)(uintptr_t)routes ||
        routing.len_routes != count || routing.num_routes != count) return -EPROTO;
    for (uint32_t i=0;i<count;++i) {
        bool matched=false;
        for (uint32_t j=0;j<count;++j)
            if (routes[j].sink_pad == expected[i].sink_pad && routes[j].sink_stream == expected[i].sink_stream &&
                routes[j].source_pad == expected[i].source_pad && routes[j].source_stream == expected[i].source_stream &&
                (routes[j].flags & CC_SD_ROUTE_ACTIVE) == (expected[i].flags & CC_SD_ROUTE_ACTIVE)) { matched=true; break; }
        if (!matched) return -EPROTO;
    }
    *stream=streams.image.stream;
    if (which == V4L2_SUBDEV_FORMAT_ACTIVE) camera->sensor_stream=*stream;
    return 0;
}
int cc_sensor_streams_image_route(cc_camera *camera, uint32_t which, uint32_t *stream)
{
    return cc_sensor_streams_route(camera,which,false,stream);
}
