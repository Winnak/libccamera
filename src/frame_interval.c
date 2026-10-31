/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include "subdev_compat.h"
#include "embedded_capture.h"
#include <errno.h>
#include <string.h>

static bool positive(cc_fraction f) { return f.numerator && f.denominator; }
static cc_fraction fraction(struct v4l2_fract f) { return (cc_fraction){f.numerator,f.denominator}; }
static int unsupported(int ret) { return ret==-ENOTTY ? -ENOTSUP:ret; }
static int image_queue(const cc_stream *s)
{
    if (!s) return -EINVAL;
    if (s->device->fd<0) return -ENODEV;
    return s->type==V4L2_BUF_TYPE_META_CAPTURE || s->type==V4L2_BUF_TYPE_META_OUTPUT ? -ENOTSUP:0;
}
int cc_stream_enum_frame_interval(cc_stream *s,uint32_t fourcc,uint32_t width,uint32_t height,
                                  uint32_t index,cc_frame_interval_range *out)
{
    if (!out || !fourcc || !width || !height) return -EINVAL;
    int ret=image_queue(s); if (ret<0) return ret;
    struct v4l2_frmivalenum q={.pixel_format=fourcc,.width=width,.height=height,.index=index};
    ret=cc_ioctl(&s->device->io,s->device->fd,VIDIOC_ENUM_FRAMEINTERVALS,&q);
    if (ret<0) return ret==-EINVAL ? -ENOENT:unsupported(ret);
    if (q.pixel_format!=fourcc || q.width!=width || q.height!=height || q.index!=index) return -EPROTO;
    cc_frame_interval_range r={.type=(enum cc_frame_interval_type)q.type};
    if (q.type==V4L2_FRMIVAL_TYPE_DISCRETE) r.minimum=r.maximum=fraction(q.discrete);
    else if (q.type==V4L2_FRMIVAL_TYPE_STEPWISE || q.type==V4L2_FRMIVAL_TYPE_CONTINUOUS) {
        if (index) return -EPROTO;
        r.minimum=fraction(q.stepwise.min); r.maximum=fraction(q.stepwise.max); r.step=fraction(q.stepwise.step);
        if (!positive(r.step)) return -EPROTO;
    } else return -EPROTO;
    if (!positive(r.minimum) || !positive(r.maximum) ||
        (uint64_t)r.minimum.numerator*r.maximum.denominator>(uint64_t)r.maximum.numerator*r.minimum.denominator)
        return -EPROTO;
    *out=r; return 0;
}
static int video_interval(cc_stream *s,cc_fraction *out,bool set)
{
    if (!out || (set && !positive(*out) && (out->numerator || out->denominator))) return -EINVAL;
    int ret=image_queue(s); if (ret<0) return ret;
    if (!s->configured) return -EPIPE;
    if (set) for (const cc_stream *q=s->device->streams;q;q=q->next)
        if (q->allocated || q->streaming || q->queued) return -EBUSY;
    struct v4l2_streamparm p={.type=s->type};
    ret=cc_ioctl(&s->device->io,s->device->fd,VIDIOC_G_PARM,&p);
    if (ret<0) return unsupported(ret);
    if (p.type!=(uint32_t)s->type) return -EPROTO;
    bool output=s->type==V4L2_BUF_TYPE_VIDEO_OUTPUT || s->type==V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    struct v4l2_fract f=output ? p.parm.output.timeperframe:p.parm.capture.timeperframe;
    uint32_t capability=output ? p.parm.output.capability:p.parm.capture.capability;
    if (!set) {
        if (!positive(fraction(f))) return capability&V4L2_CAP_TIMEPERFRAME ? -EPROTO:-ENOTSUP;
        *out=fraction(f); return 0;
    }
    if (!(capability&V4L2_CAP_TIMEPERFRAME)) return -ENOTSUP;
    /* Keep native mode fields; buffer-count zero means preserve, not resize. */
    if (output) {
        p.parm.output.timeperframe=(struct v4l2_fract){out->numerator,out->denominator};
        p.parm.output.writebuffers=0; memset(p.parm.output.reserved,0,sizeof(p.parm.output.reserved));
    } else {
        p.parm.capture.timeperframe=(struct v4l2_fract){out->numerator,out->denominator};
        p.parm.capture.readbuffers=0; memset(p.parm.capture.reserved,0,sizeof(p.parm.capture.reserved));
    }
    ret=cc_ioctl(&s->device->io,s->device->fd,VIDIOC_S_PARM,&p);
    if (ret<0) return unsupported(ret);
    f=output ? p.parm.output.timeperframe:p.parm.capture.timeperframe;
    if (p.type!=(uint32_t)s->type || !positive(fraction(f))) return -EPROTO;
    *out=fraction(f); return 0;
}
int cc_stream_get_frame_interval(cc_stream *s,cc_fraction *f) { return video_interval(s,f,false); }
int cc_stream_set_frame_interval(cc_stream *s,cc_fraction *f) { return video_interval(s,f,true); }
int cc_camera_enum_frame_interval(cc_camera *camera,uint32_t fourcc,uint32_t width,uint32_t height,
                                  uint32_t index,cc_frame_interval_range *out)
{
    return cc_stream_enum_frame_interval(camera ? camera->stream:NULL,fourcc,width,height,index,out);
}
static int sensor_image(cc_camera *camera,cc_sensor_stream_info *image)
{
    if (!camera->media || !camera->subdev_streams) return -ENOTSUP;
    struct cc_entity *sensor=cc_media_entity(camera->media,camera->info.sensor_entity);
    if (!sensor) return -ENODEV;
    if (!camera->subdev_streams[sensor-camera->media->entities]) {
        *image=(cc_sensor_stream_info){.pad=camera->sensor_pad}; return 0;
    }
    cc_sensor_streams streams; int ret=cc_camera_get_sensor_streams(camera,&streams);
    if (ret) return ret<0 ? ret:-EPROTO;
    *image=streams.image; return 0;
}
int cc_camera_enum_sensor_frame_interval(cc_camera *camera,uint32_t code,uint32_t width,uint32_t height,
                                         uint32_t index,cc_fraction *out)
{
    if (!camera || !out || !code || !width || !height) return -EINVAL;
    if (camera->sensor_fd<0) return -ENOTSUP;
    cc_sensor_stream_info image; int ret=sensor_image(camera,&image); if (ret<0) return ret;
    struct cc_subdev_frame_interval_enum q={.pad=image.pad,.stream=image.stream,
        .which=V4L2_SUBDEV_FORMAT_ACTIVE,.code=code,.width=width,.height=height,.index=index};
    ret=cc_ioctl(&camera->io,camera->sensor_fd,CC_SD_ENUM_FRAME_INTERVAL,&q);
    if (ret<0) return ret==-EINVAL ? -ENOENT:unsupported(ret);
    if (q.pad!=image.pad || q.stream!=image.stream || q.which!=V4L2_SUBDEV_FORMAT_ACTIVE ||
        q.code!=code || q.width!=width || q.height!=height || q.index!=index || !positive(fraction(q.interval))) return -EPROTO;
    *out=fraction(q.interval); return 0;
}
static int camera_interval(cc_camera *camera,cc_fraction *out,bool set)
{
    if (!camera || !out || (set && !positive(*out) && (out->numerator || out->denominator))) return -EINVAL;
    int ret=cc_camera_fd(camera); if (ret<0) return ret;
    if (!camera->stream->configured) return -EPIPE;
    if (camera->sensor_fd<0) return video_interval(camera->stream,out,set);
    if (set) {
        if (!positive(*out)) return -EINVAL;
        if (camera->stream->allocated || camera->stream->streaming || camera->stream->queued ||
            (camera->embedded && camera->embedded->stream->allocated)) return -EBUSY;
    }
    cc_sensor_stream_info image; ret=sensor_image(camera,&image); if (ret<0) return ret;
    /* We do not negotiate INTERVAL_USES_WHICH, so get/set are ACTIVE. Zero
     * preserves old reserved words; newer kernels force which to ACTIVE. */
    struct cc_subdev_frame_interval q={.pad=image.pad,.stream=image.stream};
    if (set) q.interval=(struct v4l2_fract){out->numerator,out->denominator};
    ret=cc_ioctl(&camera->io,camera->sensor_fd,set ? CC_SD_S_FRAME_INTERVAL:CC_SD_G_FRAME_INTERVAL,&q);
    if (ret<0) return unsupported(ret);
    if (q.pad!=image.pad || q.stream!=image.stream || q.which>V4L2_SUBDEV_FORMAT_ACTIVE ||
        !positive(fraction(q.interval))) return -EPROTO;
    *out=fraction(q.interval); return 0;
}
int cc_camera_get_frame_interval(cc_camera *camera,cc_fraction *f) { return camera_interval(camera,f,false); }
int cc_camera_set_frame_interval(cc_camera *camera,cc_fraction *f) { return camera_interval(camera,f,true); }
