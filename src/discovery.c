/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool numbered_node(const char *name, const char *prefix)
{
    size_t n = strlen(prefix);
    if (strncmp(name, prefix, n) || !name[n])
        return false;
    for (const char *p = name + n; *p; ++p)
        if (*p < '0' || *p > '9')
            return false;
    return true;
}

static int append_camera(cc_camera_info **list, size_t *count, const cc_camera_info *info)
{
    if (*count >= SIZE_MAX / sizeof(**list) - 1)
        return -EOVERFLOW;
    cc_camera_info *p = realloc(*list, (*count + 1) * sizeof(*p));
    if (!p)
        return -ENOMEM;
    *list = p;
    p[(*count)++] = *info;
    return 0;
}

int cc_enumerate_media_io(const struct cc_io *io, const char *path, cc_camera_info **list, size_t *count)
{
    struct cc_media *media = NULL;
    int ret = cc_media_open(io, path, false, &media);
    if (ret < 0)
        return ret == -ENOMEM ? ret : 0;
    ret = 0;
    bool cfe = !strncmp(media->info.driver, "rp1-cfe", sizeof(media->info.driver));
    bool vimc = !strncmp(media->info.driver, "vimc", sizeof(media->info.driver));
    if (!strncmp(media->info.driver,"uvcvideo",sizeof(media->info.driver))) {
        struct cc_entity *video;
        ret=cc_uvc_match(media,&video);
        if (ret<0) { ret=0; goto done; }
        int fd=io->ops->open(io->ctx,video->path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
        if (fd<0) { ret=0; goto done; }
        struct v4l2_capability caps; enum v4l2_buf_type type;
        ret=cc_query_capture(io,fd,&caps,&type);
        io->ops->close(io->ctx,fd);
        if (ret<0 || strncmp((const char *)caps.driver,"uvcvideo",sizeof(caps.driver))) { ret=0; goto done; }
        cc_camera_info info={.pipeline=CC_PIPELINE_UVC};
        cc_copy_string(info.name,sizeof(info.name),media->info.model,sizeof(media->info.model));
        cc_copy_string(info.driver,sizeof(info.driver),media->info.driver,sizeof(media->info.driver));
        cc_copy_string(info.video_path,sizeof(info.video_path),video->path,strlen(video->path));
        cc_copy_string(info.media_path,sizeof(info.media_path),path,strlen(path));
        ret=append_camera(list,count,&info); goto done;
    }
    if (!vimc && !cfe && strncmp(media->info.driver, "unicam", sizeof(media->info.driver)))
        goto done;
    if (vimc) {
        struct cc_entity *sensor, *processed, *raw;
        ret = cc_vimc_match(media, &sensor, &processed, &raw);
        if (ret < 0) { ret = 0; goto done; }
        size_t *route = NULL, route_count;
        ret = cc_vimc_route(media, &route, &route_count);
        if (ret < 0) { if (ret != -ENOMEM) ret = 0; goto done; }
        free(route);
        cc_camera_info info = { .pipeline=CC_PIPELINE_VIMC, .sensor_entity=sensor->desc.id };
        snprintf(info.name, sizeof(info.name), "VIMC Sensor B (%.*s)", (int)sizeof(media->info.bus_info), media->info.bus_info);
        cc_copy_string(info.driver, sizeof(info.driver), media->info.driver, sizeof(media->info.driver));
        cc_copy_string(info.video_path, sizeof(info.video_path), processed->path, strlen(processed->path));
        cc_copy_string(info.media_path, sizeof(info.media_path), path, strlen(path));
        ret = append_camera(list, count, &info);
        goto done;
    }
    const char *node = cfe ? "rp1-cfe-csi2-ch0" : "unicam-image";
    struct cc_entity *video = NULL;
    for (size_t i = 0; i < media->num_entities; ++i)
        if (!strcmp(cc_entity_name(&media->entities[i]), node))
            video = &media->entities[i];
    if (!video || !video->path[0])
        goto done;
    for (size_t i = 0; i < media->num_entities; ++i) {
        struct cc_entity *entity = &media->entities[i];
        if (entity->desc.type != MEDIA_ENT_F_CAM_SENSOR || !entity->path[0])
            continue;
        size_t *route = NULL, route_count = 0;
        ret = cc_media_route(media, entity->desc.id, video->desc.id, &route, &route_count);
        if (ret == -ENOMEM)
            goto done;
        if (ret < 0) {
            ret = 0;
            continue;
        }
        free(route);
        cc_camera_info info = { .pipeline = cfe ? CC_PIPELINE_RPI_CFE : CC_PIPELINE_RPI_UNICAM,
            .sensor_entity = entity->desc.id };
        snprintf(info.name, sizeof(info.name), "%s (%.*s)", cc_entity_name(entity),
                 (int)sizeof(media->info.bus_info), media->info.bus_info);
        cc_copy_string(info.driver, sizeof(info.driver), media->info.driver, sizeof(media->info.driver));
        cc_copy_string(info.video_path, sizeof(info.video_path), video->path, strlen(video->path));
        cc_copy_string(info.media_path, sizeof(info.media_path), path, strlen(path));
        ret = append_camera(list, count, &info);
        if (ret < 0)
            goto done;
    }
done:
    cc_media_close(media);
    return ret;
}

int cc_enumerate(cc_camera_info **cameras, size_t *count)
{
    if (!cameras || !count)
        return -EINVAL;
    *cameras = NULL;
    *count = 0;
    struct dirent **nodes = NULL;
    int num_nodes = scandir("/dev", &nodes, NULL, alphasort);
    if (num_nodes < 0)
        return -errno;
    int ret = 0;
    for (int i = 0; i < num_nodes; ++i) {
        if (!numbered_node(nodes[i]->d_name, "media"))
            continue;
        char path[CC_PATH_MAX];
        int length = snprintf(path, sizeof(path), "/dev/%s", nodes[i]->d_name);
        if (length < 0 || (size_t)length >= sizeof(path))
            continue;
        ret = cc_enumerate_media_io(&cc_system_io, path, cameras, count);
        if (ret < 0)
            goto done;
    }
    for (int i = 0; i < num_nodes; ++i) {
        if (!numbered_node(nodes[i]->d_name, "video"))
            continue;
        cc_camera_info info = { .pipeline = CC_PIPELINE_V4L2 };
        int length = snprintf(info.video_path, sizeof(info.video_path), "/dev/%s", nodes[i]->d_name);
        if (length < 0 || (size_t)length >= sizeof(info.video_path))
            continue;
        int fd = cc_system_io.ops->open(NULL, info.video_path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            continue;
        struct v4l2_capability caps;
        enum v4l2_buf_type type;
        int query = cc_query_capture(&cc_system_io, fd, &caps, &type);
        cc_system_io.ops->close(NULL, fd);
        if (query < 0)
            continue;
        uint32_t capabilities = caps.capabilities & V4L2_CAP_DEVICE_CAPS ? caps.device_caps : caps.capabilities;
        if (capabilities & V4L2_CAP_IO_MC || !strncmp((const char *)caps.driver, "unicam", sizeof(caps.driver)) ||
            !strncmp((const char *)caps.driver, "rp1-cfe", sizeof(caps.driver)) ||
            !strncmp((const char *)caps.driver, "vimc", sizeof(caps.driver)))
            continue;
        bool listed=false;
        for (size_t j=0;j<*count;++j) if (!strcmp((*cameras)[j].video_path,info.video_path)) { listed=true; break; }
        if (listed) continue;
        if (!strncmp((const char *)caps.driver,"uvcvideo",sizeof(caps.driver))) info.pipeline=CC_PIPELINE_UVC;
        cc_copy_string(info.name, sizeof(info.name), (const char *)caps.card, sizeof(caps.card));
        cc_copy_string(info.driver, sizeof(info.driver), (const char *)caps.driver, sizeof(caps.driver));
        ret = append_camera(cameras, count, &info);
        if (ret < 0)
            goto done;
    }
done:
    for (int i = 0; i < num_nodes; ++i)
        free(nodes[i]);
    free(nodes);
    if (ret < 0) {
        free(*cameras);
        *cameras = NULL;
        *count = 0;
    }
    return ret;
}

void cc_camera_list_free(cc_camera_info *cameras) { free(cameras); }
