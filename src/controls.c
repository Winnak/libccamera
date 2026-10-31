/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

int cc_video_query_control(cc_video_device *device, uint32_t id, cc_control_info *info)
{
    if (!device)
        return -EINVAL;
    return device->fd < 0 ? -ENODEV : cc_control_query_io(&device->io, device->fd, id, info);
}

int cc_video_next_control(cc_video_device *device, uint32_t after_id, cc_control_info *info)
{
    int ret = cc_video_query_control(device, after_id | V4L2_CTRL_FLAG_NEXT_CTRL |
                                     V4L2_CTRL_FLAG_NEXT_COMPOUND, info);
    return ret == -EINVAL ? -ENOENT : ret;
}
int cc_video_query_menu(cc_video_device *device, uint32_t id, uint32_t index, cc_control_menu *entry)
{
    if (!device) return -EINVAL;
    return device->fd<0 ? -ENODEV:cc_control_menu_io(&device->io,device->fd,id,index,entry);
}
int cc_control_menu_io(const struct cc_io *io,int fd,uint32_t id,uint32_t index,cc_control_menu *entry)
{
    if (!io || fd<0 || !entry || !id || id&(V4L2_CTRL_FLAG_NEXT_CTRL|V4L2_CTRL_FLAG_NEXT_COMPOUND)) return -EINVAL;
    cc_control_info info;
    int ret=cc_control_query_io(io,fd,id,&info);
    if (ret<0) return ret==-EINVAL ? -ENOENT:ret;
    if (info.id!=id) return -EPROTO;
    if (info.type!=V4L2_CTRL_TYPE_MENU && info.type!=V4L2_CTRL_TYPE_INTEGER_MENU) return -ENOTSUP;
    if (info.minimum<0 || info.maximum<info.minimum || info.maximum>UINT32_MAX) return -EPROTO;
    if (index<(uint64_t)info.minimum || index>(uint64_t)info.maximum) return -ENOENT;
    struct v4l2_querymenu query={.id=id,.index=index};
    ret=cc_ioctl(io,fd,VIDIOC_QUERYMENU,&query);
    if (ret<0) return ret==-EINVAL ? -ENOENT:ret;
    if (query.id!=id || query.index!=index) return -EPROTO;
    cc_control_menu result={.id=id,.index=index,.type=info.type};
    if (info.type==V4L2_CTRL_TYPE_INTEGER_MENU) result.value=query.value;
    else cc_copy_string(result.name,sizeof(result.name),(const char *)query.name,sizeof(query.name));
    *entry=result; return 0;
}

static int video_controls(cc_video_device *device, const cc_control *input, cc_control *output,
                            size_t count, size_t *error_index)
{
    if (!device)
        return -EINVAL;
    if (device->fd < 0)
        return -ENODEV;
    bool busy = false, changed;
    for (cc_stream *s = device->streams; s; s = s->next)
        busy |= s->allocated;
    int ret = cc_controls_io(&device->io, device->fd, input, output, count, error_index, busy, &changed);
    if (changed)
        for (cc_stream *s = device->streams; s; s = s->next)
            s->configured = false;
    return ret;
}

int cc_video_get_controls(cc_video_device *device, cc_control *controls, size_t count, size_t *error_index)
{
    return video_controls(device, controls, controls, count, error_index);
}

int cc_video_set_controls(cc_video_device *device, const cc_control *controls, size_t count, size_t *error_index)
{
    return video_controls(device, controls, NULL, count, error_index);
}

int cc_control_query_io(const struct cc_io *io, int fd, uint32_t id, cc_control_info *info)
{
    if (!io || fd < 0 || !info)
        return -EINVAL;
    struct v4l2_query_ext_ctrl query = { .id = id };
    int ret = cc_ioctl(io, fd, VIDIOC_QUERY_EXT_CTRL, &query);
    if (ret < 0)
        return ret;
    if (query.nr_of_dims > 4)
        return -EPROTO;
    memset(info, 0, sizeof(*info));
    info->id = query.id;
    info->type = query.type;
    info->flags = query.flags;
    info->minimum = query.minimum;
    info->maximum = query.maximum;
    info->default_value = query.default_value;
    info->step = query.step;
    info->elem_size = query.elem_size;
    info->elems = query.elems;
    info->nr_of_dims = query.nr_of_dims;
    memcpy(info->dims, query.dims, sizeof(info->dims));
    cc_copy_string(info->name, sizeof(info->name), query.name, sizeof(query.name));
    return 0;
}

static int controls_io(const struct cc_io *io, int fd, const cc_control *input, cc_control *output,
                        size_t count, size_t *error_index, bool layout_busy, bool *layout_changed, bool get)
{
    *layout_changed = false;
    if (error_index)
        *error_index = count;
    if (!io || fd < 0 || (count && !input) || count > UINT32_MAX ||
        count > SIZE_MAX / sizeof(struct v4l2_ext_control))
        return -EINVAL;
    if (!count)
        return 0;
    struct v4l2_ext_control *items = calloc(count, sizeof(*items));
    uint32_t *kinds = calloc(count, sizeof(*kinds));
    if (!items || !kinds) {
        free(items);
        free(kinds);
        return -ENOMEM;
    }
    int ret = 0;
    for (size_t i = 0; i < count; ++i) {
        cc_control_info info;
        ret = cc_control_query_io(io, fd, input[i].id, &info);
        if (ret < 0)
            goto item_error;
        kinds[i] = info.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD ? UINT32_MAX : info.type;
        if (!get && info.flags & V4L2_CTRL_FLAG_MODIFY_LAYOUT) {
            if (layout_busy) {
                ret = -EBUSY;
                goto item_error;
            }
            *layout_changed = true;
        }
        items[i].id = input[i].id;
        if (info.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) {
            if (!input[i].data || !input[i].size) {
                ret = -EINVAL;
                goto item_error;
            }
            items[i].size = input[i].size;
            items[i].ptr = input[i].data;
        } else if (info.type == V4L2_CTRL_TYPE_INTEGER64) {
            items[i].value64 = get ? 0 : input[i].value;
        } else {
            if (!get && (input[i].value < INT32_MIN ||
                input[i].value > (info.type == V4L2_CTRL_TYPE_BITMASK ? (int64_t)UINT32_MAX : INT32_MAX))) {
                ret = -ERANGE;
                goto item_error;
            }
            items[i].value = get ? 0 : (int32_t)input[i].value;
        }
        continue;
item_error:
        if (error_index)
            *error_index = i;
        free(items);
        free(kinds);
        return ret;
    }
    struct v4l2_ext_controls batch = {
        .which = V4L2_CTRL_WHICH_CUR_VAL, .count = (uint32_t)count,
        .controls = items, .error_idx = (uint32_t)count
    };
    ret = cc_ioctl(io, fd, get ? VIDIOC_G_EXT_CTRLS : VIDIOC_S_EXT_CTRLS, &batch);
    if (ret < 0) {
        if (error_index)
            *error_index = batch.error_idx < count ? batch.error_idx : count;
    }
    if (output) {
        for (size_t i = 0; i < count; ++i) {
            /* ENOSPC reports the required payload size even on failure. Scalar
             * results are meaningful only when the entire get succeeded. */
            if (kinds[i] == UINT32_MAX)
                output[i].size = items[i].size;
            else if (ret < 0)
                continue;
            else if (kinds[i] == V4L2_CTRL_TYPE_INTEGER64)
                output[i].value = items[i].value64;
            else if (kinds[i] == V4L2_CTRL_TYPE_BITMASK)
                output[i].value = (uint32_t)items[i].value;
            else
                output[i].value = items[i].value;
        }
    }
    free(items);
    free(kinds);
    return ret;
}

int cc_controls_io(const struct cc_io *io, int fd, const cc_control *input, cc_control *output,
                    size_t count, size_t *error_index, bool layout_busy, bool *layout_changed)
{
    return controls_io(io, fd, input, output, count, error_index, layout_busy, layout_changed, output != NULL);
}

int cc_controls_apply_io(const struct cc_io *io, int fd, cc_control *controls, size_t count, bool layout_busy)
{
    bool changed;
    return controls_io(io, fd, controls, controls, count, NULL, layout_busy, &changed, false);
}
