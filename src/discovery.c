/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "monitor.h"

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
    if (*count>=4096) return -E2BIG;
    if (info->id[0]) for (size_t i=0; i<*count; ++i)
        if (!strcmp((*list)[i].id,info->id)) return -ENOTUNIQ;
    if (*count >= SIZE_MAX / sizeof(**list) - 1)
        return -EOVERFLOW;
    cc_camera_info *p = realloc(*list, (*count + 1) * sizeof(*p));
    if (!p)
        return -ENOMEM;
    *list = p;
    p[(*count)++] = *info;
    return 0;
}

static bool transient(int error)
{
    return error==-EACCES || error==-EPERM || error==-EBUSY || error==-ENOENT ||
        error==-ENODEV || error==-EAGAIN || error==-ESTALE || error==-EIO;
}

static int enumerate_media(const struct cc_io *io, const char *path, cc_camera_info **list, size_t *count, bool *pending)
{
    struct cc_media *media = NULL;
    int ret = cc_media_open(io, path, false, &media);
    if (ret < 0) {
        if (pending && transient(ret)) *pending=true;
        return ret == -ENOMEM ? ret : 0;
    }
    ret = 0;
    bool cfe = !strncmp(media->info.driver, "rp1-cfe", sizeof(media->info.driver));
    bool vimc = !strncmp(media->info.driver, "vimc", sizeof(media->info.driver));
    if (!strncmp(media->info.driver,"uvcvideo",sizeof(media->info.driver))) {
        struct cc_entity *video;
        ret=cc_uvc_match(media,&video);
        if (ret<0) { if (pending) *pending=true; ret=0; goto done; }
        int fd=io->ops->open(io->ctx,video->path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
        if (fd<0) { if (pending && transient(fd)) *pending=true; ret=0; goto done; }
        struct v4l2_capability caps; enum v4l2_buf_type type;
        ret=cc_query_capture(io,fd,&caps,&type);
        cc_camera_info info={.pipeline=CC_PIPELINE_UVC};
        int identity=0;
        if (!ret && !strncmp((const char *)caps.driver,"uvcvideo",sizeof(caps.driver))) {
            identity=cc_camera_identify_io(io,fd,0,0,info.pipeline,NULL,&info);
        }
        io->ops->close(io->ctx,fd);
        if (ret<0) { if (pending && transient(ret)) *pending=true; ret=0; goto done; }
        if (strncmp((const char *)caps.driver,"uvcvideo",sizeof(caps.driver))) { ret=0; goto done; }
        if (identity<0 && !cc_identity_unavailable(identity)) { ret=identity; goto done; }
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
        if (ret < 0) { if (pending) *pending=true; ret = 0; goto done; }
        size_t *route = NULL, route_count;
        ret = cc_vimc_route(media, &route, &route_count);
        if (ret < 0) { if (pending) *pending=true; if (ret != -ENOMEM) ret = 0; goto done; }
        free(route);
        cc_camera_info info = { .pipeline=CC_PIPELINE_VIMC, .sensor_entity=sensor->desc.id };
        snprintf(info.name, sizeof(info.name), "VIMC Sensor B (%.*s)", (int)sizeof(media->info.bus_info), media->info.bus_info);
        cc_copy_string(info.driver, sizeof(info.driver), media->info.driver, sizeof(media->info.driver));
        cc_copy_string(info.video_path, sizeof(info.video_path), processed->path, strlen(processed->path));
        cc_copy_string(info.media_path, sizeof(info.media_path), path, strlen(path));
        ret=cc_camera_identify_io(io,-1,sensor->desc.dev.major,sensor->desc.dev.minor,
            info.pipeline,cc_entity_name(sensor),&info);
        if (ret<0 && !cc_identity_unavailable(ret)) goto done;
        ret = append_camera(list, count, &info);
        goto done;
    }
    const char *node = cfe ? "rp1-cfe-csi2-ch0" : "unicam-image";
    struct cc_entity *video = NULL;
    for (size_t i = 0; i < media->num_entities; ++i)
        if (!strcmp(cc_entity_name(&media->entities[i]), node))
            video = &media->entities[i];
    if (!video || !video->path[0]) {
        if (pending) *pending=true;
        goto done;
    }
    for (size_t i = 0; i < media->num_entities; ++i) {
        struct cc_entity *entity = &media->entities[i];
        if (entity->desc.type != MEDIA_ENT_F_CAM_SENSOR) continue;
        if (!entity->path[0]) { if (pending) *pending=true; continue; }
        size_t *route = NULL, route_count = 0;
        ret = cc_media_route(media, entity->desc.id, video->desc.id, &route, &route_count);
        if (ret == -ENOMEM)
            goto done;
        if (ret < 0) {
            if (pending) *pending=true;
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
        ret=cc_camera_identify_io(io,-1,entity->desc.dev.major,entity->desc.dev.minor,
            info.pipeline,cc_entity_name(entity),&info);
        if (ret<0 && !cc_identity_unavailable(ret)) goto done;
        ret = append_camera(list, count, &info);
        if (ret < 0)
            goto done;
    }
done:
    cc_media_close(media);
    return ret;
}

int cc_enumerate_media_io(const struct cc_io *io, const char *path, cc_camera_info **list, size_t *count)
{
    return enumerate_media(io,path,list,count,NULL);
}

static void free_nodes(char **nodes, size_t count)
{
    for (size_t i=0; i<count; ++i) free(nodes[i]);
    free(nodes);
}

static int scan_nodes(char ***out, size_t *count)
{
    struct dirent **entries=NULL;
    int num=scandir("/dev",&entries,NULL,alphasort);
    if (num<0) return -errno;
    char **nodes=calloc(num ? (size_t)num:1,sizeof(*nodes));
    int ret=nodes ? 0:-ENOMEM;
    size_t used=0;
    for (int i=0; i<num; ++i) {
        if (ret<0 || (!numbered_node(entries[i]->d_name,"media") && !numbered_node(entries[i]->d_name,"video"))) continue;
        char path[CC_PATH_MAX];
        int n=snprintf(path,sizeof(path),"/dev/%s",entries[i]->d_name);
        if (n<0 || (size_t)n>=sizeof(path)) continue;
        nodes[used]=strdup(path);
        if (!nodes[used]) { ret=-ENOMEM; continue; }
        ++used;
    }
    for (int i=0; i<num; ++i) free(entries[i]);
    free(entries);
    if (ret<0) { free_nodes(nodes,used); return ret; }
    *out=nodes; *count=used;
    return 0;
}

static int enumerate_video(const struct cc_io *io, const char *path, cc_camera_info **cameras, size_t *count, bool *pending)
{
    cc_camera_info info={.pipeline=CC_PIPELINE_V4L2};
    cc_copy_string(info.video_path,sizeof(info.video_path),path,strlen(path));
    int fd=io->ops->open(io->ctx,path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if (fd<0) { if (pending && transient(fd)) *pending=true; return 0; }
    struct v4l2_capability caps;
    enum v4l2_buf_type type;
    int ret=cc_query_capture(io,fd,&caps,&type);
    if (ret<0) { if (pending && transient(ret)) *pending=true; ret=0; goto done; }
    uint32_t capabilities=caps.capabilities&V4L2_CAP_DEVICE_CAPS ? caps.device_caps:caps.capabilities;
    if (capabilities&V4L2_CAP_IO_MC || !strncmp((const char *)caps.driver,"unicam",sizeof(caps.driver)) ||
        !strncmp((const char *)caps.driver,"rp1-cfe",sizeof(caps.driver)) ||
        !strncmp((const char *)caps.driver,"vimc",sizeof(caps.driver))) goto done;
    for (size_t i=0; i<*count; ++i) if (!strcmp((*cameras)[i].video_path,path)) goto done;
    if (!strncmp((const char *)caps.driver,"uvcvideo",sizeof(caps.driver))) info.pipeline=CC_PIPELINE_UVC;
    ret=cc_camera_identify_io(io,fd,0,0,info.pipeline,NULL,&info);
    if (ret<0 && !cc_identity_unavailable(ret)) goto done;
    cc_copy_string(info.name,sizeof(info.name),(const char *)caps.card,sizeof(caps.card));
    cc_copy_string(info.driver,sizeof(info.driver),(const char *)caps.driver,sizeof(caps.driver));
    ret=append_camera(cameras,count,&info);
done:
    io->ops->close(io->ctx,fd);
    return ret;
}

int cc_enumerate_state_io(const struct cc_io *io, cc_camera_info **cameras, size_t *count, bool *pending)
{
    if (!io || !io->ops || !cameras || !count) return -EINVAL;
    *cameras=NULL; *count=0;
    if (pending) *pending=false;
    char **nodes=NULL;
    size_t num=0;
    int ret;
    if (io->ops->scan_nodes) ret=io->ops->scan_nodes(io->ctx,&nodes,&num);
    else if (io->ops==cc_system_io.ops) ret=scan_nodes(&nodes,&num);
    else return -ENOTSUP;
    if (ret) { if (ret>0) ret=-EPROTO; goto done; }
    if (num && !nodes) { ret=-EPROTO; goto done; }
    for (size_t i=0; i<num; ++i) {
        if (!nodes[i] || strnlen(nodes[i],CC_PATH_MAX)==CC_PATH_MAX || strncmp(nodes[i],"/dev/",5) ||
            (!numbered_node(nodes[i]+5,"media") && !numbered_node(nodes[i]+5,"video"))) { ret=-EPROTO; goto done; }
        for (size_t j=0; j<i; ++j) if (!strcmp(nodes[i],nodes[j])) { ret=-EPROTO; goto done; }
    }
    for (unsigned phase=0; phase<2; ++phase) for (size_t i=0; i<num; ++i) {
        if (!numbered_node(nodes[i]+5,phase ? "video":"media")) continue;
        ret=phase ? enumerate_video(io,nodes[i],cameras,count,pending):enumerate_media(io,nodes[i],cameras,count,pending);
        if (ret<0) goto done;
    }
done:
    if (nodes) free_nodes(nodes,num);
    if (ret<0) { free(*cameras); *cameras=NULL; *count=0; }
    return ret;
}

int cc_enumerate_io(const struct cc_io *io, cc_camera_info **cameras, size_t *count)
{
    return cc_enumerate_state_io(io,cameras,count,NULL);
}

int cc_enumerate(cc_camera_info **cameras, size_t *count)
{
    return cc_enumerate_io(&cc_system_io,cameras,count);
}

int cc_camera_find_id_io(const char *id, const struct cc_io *io, cc_camera_info *out)
{
    if (!id || !id[0] || strnlen(id,CC_CAMERA_ID_MAX)==CC_CAMERA_ID_MAX || !out) return -EINVAL;
    cc_camera_info *list=NULL;
    size_t count=0,found=SIZE_MAX;
    int ret=cc_enumerate_io(io,&list,&count);
    if (ret<0) return ret;
    for (size_t i=0; i<count; ++i) if (!strcmp(list[i].id,id)) {
        if (found!=SIZE_MAX) { ret=-ENOTUNIQ; goto done; }
        found=i;
    }
    if (found==SIZE_MAX) ret=-ENOENT;
    else *out=list[found];
done:
    free(list);
    return ret;
}

int cc_camera_open_id_io(const char *id, const struct cc_io *io, cc_camera **out)
{
    if (!out) return -EINVAL;
    *out=NULL;
    cc_camera_info info;
    int ret=cc_camera_find_id_io(id,io,&info);
    return ret<0 ? ret:cc_camera_open_io(&info,io,out);
}

int cc_camera_open_by_id(const char *id, cc_camera **out)
{
    return cc_camera_open_id_io(id,&cc_system_io,out);
}

void cc_camera_list_free(cc_camera_info *cameras) { free(cameras); }
