/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Role defaults, size ordering and stream policies follow libcamera 0.7.2
 * pipeline/rpi/{common,vc4,pisp}, pipeline/uvcvideo, pipeline/vimc and stream.cpp.
 * Copyright (C) 2018-2020 Google Inc.; (C) 2019-2023 Raspberry Pi Ltd. */
#include "internal.h"
#include "embedded_capture.h"
#include "pisp_format.h"
#include <ccamera/configuration.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define FORMAT_LIMIT 256U
#define SIZE_LIMIT 4096U
struct size { uint32_t width,height; };
struct plan {
    cc_camera *camera;
    bool pi;
    size_t raw,out[2],num_out;
    cc_format sensor;
    cc_cfe_configuration fe;
    cc_vc4_configuration vc4;
    cc_pisp_be_configuration be;
};
static const struct size common_sizes[]={
    {160,120},{240,160},{320,240},{400,240},{480,320},{640,360},{640,480},
    {720,480},{720,576},{768,480},{800,600},{854,480},{960,540},{960,640},
    {1024,576},{1024,600},{1024,768},{1152,864},{1280,1024},{1280,1080},
    {1280,720},{1280,800},{1360,768},{1366,768},{1400,1050},{1440,900},
    {1536,864},{1600,1200},{1600,900},{1680,1050},{1920,1080},{1920,1200},
    {2048,1080},{2048,1152},{2048,1536},{2160,1080},{2560,1080},{2560,1440},
    {2560,1600},{2560,2048},{2960,1440},{3200,1800},{3200,2048},{3200,2400},
    {3440,1440},{3840,1080},{3840,1600},{3840,2160},{3840,2400},{4096,2160},
    {5120,2160},{5120,2880},{7680,4320}
};
static int size_compare(struct size a,struct size b)
{
    uint64_t x=(uint64_t)a.width*a.height,y=(uint64_t)b.width*b.height;
    return x!=y ? (x<y ? -1:1):a.width!=b.width ? (a.width<b.width ? -1:1):0;
}
static bool contains(const cc_size_range *r,struct size s)
{
    return s.width>=r->min_width && s.width<=r->max_width && s.height>=r->min_height &&
        s.height<=r->max_height && !((s.width-r->min_width)%r->width_step) &&
        !((s.height-r->min_height)%r->height_step);
}
/* Compare reference DRM code/modifier keys while retaining native memory-plane
 * aliases. Unknown native extensions sort after the archived known formats. */
struct format_order { uint32_t native,code; uint64_t modifier; uint32_t encoding,depth; };
#include "configuration_formats.h"
static struct format_order format_order(uint32_t native)
{
    for (size_t i=0;i<sizeof(format_orders)/sizeof(format_orders[0]);++i)
        if (format_orders[i].native==native) return format_orders[i];
    return (struct format_order){native,UINT32_MAX,UINT64_MAX,0,0};
}
static bool format_less(uint32_t a,uint32_t b)
{
    struct format_order x=format_order(a),y=format_order(b);
    if (x.code!=y.code) return x.code<y.code;
    if (x.modifier!=y.modifier) return x.modifier<y.modifier;
    return x.native<y.native;
}
static int device_plan(const cc_capture_devices *d,struct plan *p)
{
    if (!d || (!d->camera==!d->frontend) || (d->vc4 && d->backend) || (d->frontend && d->vc4)) return -EINVAL;
    p->camera=d->camera ? d->camera:cc_cfe_source(d->frontend);
    if (!p->camera) return -ENODEV;
    int ret=cc_camera_fd(p->camera);
    if (ret<0) return ret;
    p->pi=p->camera->info.pipeline==CC_PIPELINE_RPI_UNICAM || p->camera->info.pipeline==CC_PIPELINE_RPI_CFE;
    if ((d->vc4 || d->backend) && (p->camera->info.pipeline==CC_PIPELINE_UVC || p->camera->info.pipeline==CC_PIPELINE_VIMC))
        return -ENOTSUP;
    p->raw=SIZE_MAX;
    return 0;
}
static int native_formats(cc_camera *camera,uint32_t requested,uint32_t *selected)
{
    uint32_t seen[FORMAT_LIMIT],first=0; bool found=false;
    for (uint32_t i=0;;++i) {
        cc_format_info info; int ret=cc_camera_enum_format(camera,i,&info);
        if (ret==-ENOENT) break;
        if (ret<0) return ret;
        if (i==FORMAT_LIMIT) return -E2BIG;
        for (uint32_t j=0;j<i;++j) if (seen[j]==info.fourcc) return -EPROTO;
        seen[i]=info.fourcc;
        if (!first || format_less(info.fourcc,first)) first=info.fourcc;
        if (info.fourcc==requested) found=true;
    }
    if (!first) return -ENOENT;
    *selected=found ? requested:first; return 0;
}
static int native_size(cc_camera *camera,uint32_t fourcc,struct size desired,bool largest,struct size *out)
{
    struct size *sizes=calloc(SIZE_LIMIT+sizeof(common_sizes)/sizeof(common_sizes[0]),sizeof(*sizes));
    if (!sizes) return -ENOMEM;
    size_t count=0; int ret=0; bool range=false;
    for (uint32_t i=0;;++i) {
        cc_size_range r; ret=cc_camera_enum_size(camera,fourcc,i,&r);
        if (ret==-ENOENT) { ret=0; break; }
        if (ret<0) goto done;
        if (i==SIZE_LIMIT) { ret=-E2BIG; goto done; }
        bool discrete=r.min_width==r.max_width && r.min_height==r.max_height;
        if (!discrete) {
            if (i) { ret=-EPROTO; goto done; }
            range=true;
            for (size_t j=0;j<sizeof(common_sizes)/sizeof(common_sizes[0]);++j)
                if (contains(&r,common_sizes[j])) sizes[count++]=common_sizes[j];
        } else {
            if (range) { ret=-EPROTO; goto done; }
            sizes[count++]=(struct size){r.max_width,r.max_height};
        }
    }
    if (!count) { ret=-ENOENT; goto done; }
    struct size best=sizes[0],minimum=sizes[0]; bool fit=false;
    for (size_t i=0;i<count;++i) {
        if (size_compare(sizes[i],minimum)<0) minimum=sizes[i];
        if (largest || size_compare(sizes[i],desired)<=0) {
            if (!fit || size_compare(best,sizes[i])<0) best=sizes[i];
            fit=true;
        }
    }
    *out=fit ? best:minimum;
done:
    free(sizes); return ret;
}
static uint32_t role_buffers(enum cc_stream_role role) { return role==CC_ROLE_RAW ? 2U:role==CC_ROLE_STILL_CAPTURE ? 1U:4U; }
static bool raw_format(uint32_t f)
{
    struct cc_pisp_raw_layout raw;
    return cc_bayer_mbus(f) || (f && !cc_pisp_raw_format(f,&raw));
}
static bool raw_stream(const cc_stream_configuration *s)
{
    uint32_t code=cc_bayer_mbus(s->format.fourcc); enum cc_camera_cfa cfa;
    if (code && !cc_bayer_code_info(code,NULL,&cfa) && cfa==CC_CAMERA_CFA_MONO)
        return s->role==CC_ROLE_RAW;
    return format_order(s->format.fourcc).encoding==3 || raw_format(s->format.fourcc) ||
        (!s->format.fourcc && s->role==CC_ROLE_RAW);
}
static int sensor_choice(struct plan *p,cc_pipeline_configuration *c,struct size desired,uint32_t depth)
{
    uint32_t codes[25]; size_t count=0; uint32_t exact_depth=0; enum cc_camera_cfa exact_cfa=CC_CAMERA_CFA_RGGB;
    if (c->sensor.mbus_code || c->sensor.width || c->sensor.height) {
        if (!c->sensor.mbus_code || !c->sensor.width || !c->sensor.height) return -EINVAL;
        int ret=cc_bayer_code_info(c->sensor.mbus_code,&exact_depth,&exact_cfa);
        if (ret<0) return ret;
        desired=(struct size){c->sensor.width,c->sensor.height}; depth=exact_depth;
    }
    for (uint32_t bits=8;bits<=16;bits+=2) for (unsigned phase=0;phase<=CC_CAMERA_CFA_MONO;++phase) {
        if (exact_depth && ((phase==CC_CAMERA_CFA_MONO)!=(exact_cfa==CC_CAMERA_CFA_MONO))) continue;
        codes[count++]=cc_bayer_code(bits,(enum cc_camera_cfa)phase);
    }
    cc_rpi_sensor_mode_request request={codes,count,desired.width,desired.height,depth};
    int ret=cc_camera_select_rpi_sensor_mode(p->camera,&request,&c->sensor_format);
    if (ret<0) return exact_depth && ret==-ENOENT ? -EINVAL:ret;
    uint32_t actual_depth; ret=cc_bayer_code_info(c->sensor_format.mbus_code,&actual_depth,NULL);
    if (ret<0) return ret;
    if (exact_depth && (actual_depth!=exact_depth || c->sensor_format.width!=desired.width || c->sensor_format.height!=desired.height))
        return -EINVAL;
    bool packed=true;
    if (p->raw!=SIZE_MAX && cc_bayer_mbus(c->streams[p->raw].format.fourcc)) {
        uint32_t unpacked=cc_bayer_fourcc(cc_bayer_mbus(c->streams[p->raw].format.fourcc),false);
        packed=c->streams[p->raw].format.fourcc!=unpacked;
    }
    p->sensor=(cc_format){.fourcc=cc_bayer_fourcc(c->sensor_format.mbus_code,packed),
        .width=c->sensor_format.width,.height=c->sensor_format.height,.colorspace=V4L2_COLORSPACE_RAW};
    return p->sensor.fourcc ? 0:-ENOTSUP;
}
static int source_plan(const cc_capture_devices *d,cc_pipeline_configuration *c,struct plan *p,bool active)
{
    enum cc_orientation o=c->orientation; int ret;
    if (d->frontend) {
        if (!active) p->fe=(cc_cfe_configuration){.sensor=p->sensor};
        else { p->fe.image.width=0; p->fe.image.height=0; }
        if (p->raw!=SIZE_MAX) {
            uint32_t wanted=c->streams[p->raw].format.fourcc;
            struct cc_pisp_raw_layout raw;
            uint32_t compression=1;
            if (!cc_pisp_raw_format(wanted,&raw) && raw.compression) compression=raw.compression;
            else if (cc_bayer_mbus(wanted) && cc_bayer_fourcc(cc_bayer_mbus(wanted),false)==wanted) compression=0;
            enum cc_camera_cfa phase; uint32_t depth;
            ret=cc_bayer_code_info(cc_bayer_mbus(p->sensor.fourcc),&depth,&phase); if (ret<0) return ret;
            if (depth>=14) compression=0;
            static const char phases[]={'R','G','g','B','M'};
            p->fe.image.fourcc=compression ? CC_FOURCC('P','C','0'+compression,phases[phase]):cc_cfe_output_fourcc(p->sensor.fourcc);
        }
        if (o) ret=active ? cc_cfe_configure_oriented(d->frontend,&p->fe,&o):cc_cfe_try_configuration_oriented(d->frontend,&p->fe,&o);
        else ret=active ? cc_cfe_configure(d->frontend,&p->fe):cc_cfe_try_configuration(d->frontend,&p->fe);
        if (ret<0) return ret;
        p->sensor=p->fe.sensor; c->source_format=p->fe.image;
    } else {
        if (o) ret=active ? cc_camera_configure_oriented(p->camera,&p->sensor,&o):cc_camera_try_format_oriented(p->camera,&p->sensor,&o);
        else ret=active ? cc_camera_configure(p->camera,&p->sensor):cc_camera_try_format(p->camera,&p->sensor);
        if (ret<0) return ret;
        c->source_format=p->sensor;
    }
    if (c->orientation) c->orientation=o;
    else {
        ret=cc_camera_get_orientation(p->camera,&o);
        if (ret<0 && ret!=-ENOTSUP) return ret;
        if (ret==-ENOTSUP) o=(enum cc_orientation)0;
    }
    c->applied_orientation=o;
    if (p->pi) c->sensor_format=(cc_sensor_format){cc_bayer_mbus(p->sensor.fourcc),p->sensor.width,p->sensor.height};
    if (c->sensor.mbus_code) {
        uint32_t requested_depth,depth;
        if (cc_bayer_code_info(c->sensor.mbus_code,&requested_depth,NULL) ||
            cc_bayer_code_info(c->sensor_format.mbus_code,&depth,NULL) || requested_depth!=depth ||
            c->sensor.width!=c->sensor_format.width || c->sensor.height!=c->sensor_format.height) return -ERANGE;
    }
    if (p->raw!=SIZE_MAX) {
        c->streams[p->raw].format=c->source_format; c->streams[p->raw].stream=CC_CAPTURE_RAW;
    }
    return 0;
}
static bool rgb(uint32_t f)
{
    return format_order(f).encoding==1;
}
static void colour(cc_format *f,uint32_t space)
{
    f->colorspace=space;
    f->ycbcr_enc=space==V4L2_COLORSPACE_REC709 ? V4L2_YCBCR_ENC_709:V4L2_YCBCR_ENC_601;
    f->quantization=rgb(f->fourcc) || space==V4L2_COLORSPACE_JPEG ? V4L2_QUANTIZATION_FULL_RANGE:V4L2_QUANTIZATION_LIM_RANGE;
    f->xfer_func=space==V4L2_COLORSPACE_JPEG ? V4L2_XFER_FUNC_SRGB:V4L2_XFER_FUNC_709;
}
static cc_rectangle crop_for(const cc_format *source,const cc_format *output)
{
    uint32_t w=source->width,h=source->height;
    if ((uint64_t)w*output->height>(uint64_t)h*output->width) w=(uint32_t)((uint64_t)h*output->width/output->height);
    else h=(uint32_t)((uint64_t)w*output->height/output->width);
    w&=~1U; h&=~1U;
    return (cc_rectangle){(int32_t)(((source->width-w)/2)&~1U),(int32_t)(((source->height-h)/2)&~1U),w,h};
}
static int resize_min(cc_format *output,const cc_rectangle *crop,bool downscaler)
{
    if (crop->width<16 || crop->height<16) return -ERANGE;
    uint32_t divisor=downscaler ? 64U:16U;
    uint32_t w=downscaler ? (crop->width+divisor-1)/divisor:(crop->width-1+divisor-1)/divisor+1;
    uint32_t h=downscaler ? (crop->height+divisor-1)/divisor:(crop->height-1+divisor-1)/divisor+1;
    w=(w+1)&~1U; h=(h+1)&~1U;
    if (w<16) w=16;
    if (h<16) h=16;
    if (output->width<w) {
        uint64_t height=(uint64_t)output->height*w/output->width+1;
        if (height>UINT32_MAX) return -ERANGE;
        output->height=(uint32_t)height&~1U; output->width=w;
    }
    if (output->height<h) {
        uint64_t width=(uint64_t)output->width*h/output->height+1;
        if (width>UINT32_MAX) return -ERANGE;
        output->width=(uint32_t)width&~1U; output->height=h;
    }
    return 0;
}
static int processor_plan(const cc_capture_devices *d,cc_pipeline_configuration *c,struct plan *p,bool active)
{
    if (!d->vc4 && !d->backend) return p->num_out ? -ENOTSUP:0;
    if (d->backend && (c->source_format.width<16 || c->source_format.height<16 ||
        c->source_format.width>UINT16_MAX || c->source_format.height>UINT16_MAX)) return -EINVAL;
    uint32_t shared=V4L2_COLORSPACE_JPEG;
    for (size_t i=0;i<c->num_streams;++i) if (i!=p->raw) {
        const cc_format *f=&c->streams[i].format;
        uint32_t space=f->colorspace,transfer=f->xfer_func;
        if (!space) continue;
        if (!transfer) transfer=V4L2_MAP_XFER_FUNC_DEFAULT(space);
        if ((space==V4L2_COLORSPACE_SRGB || space==V4L2_COLORSPACE_JPEG) && transfer==V4L2_XFER_FUNC_SRGB) {
            shared=V4L2_COLORSPACE_JPEG; break;
        }
        if (transfer==V4L2_XFER_FUNC_709) {
            if (space==V4L2_COLORSPACE_SRGB || space==V4L2_COLORSPACE_JPEG || space==V4L2_COLORSPACE_REC709) {
                shared=V4L2_COLORSPACE_REC709; break;
            }
            if (space==V4L2_COLORSPACE_SMPTE170M) { shared=space; break; }
        }
    }
    for (size_t i=0;i<p->num_out;++i) {
        cc_format *f=&c->streams[p->out[i]].format;
        if (!f->width || !f->height) return -EINVAL;
        if (d->backend) {
            cc_format probe={.fourcc=f->fourcc,.width=16,.height=16};
            if (cc_pisp_prepare_output(&probe)) f->fourcc=V4L2_PIX_FMT_YUV420;
            else {
                struct cc_pisp_output_layout layout;
                if (!cc_pisp_output_layout(&probe,&layout) &&
                    (layout.native.format&PISP_IMAGE_FORMAT_BPP_32) &&
                    cc_be_hardware_version(d->backend)!=CC_PISP_BE_VERSION_BCM2712_D0)
                    f->fourcc=layout.swap_rb ? V4L2_PIX_FMT_BGR24:V4L2_PIX_FMT_RGB24;
            }
        }
        colour(f,shared);
        if (i) {
            const cc_format *main=&c->streams[p->out[0]].format;
            if (f->width>main->width) f->width=main->width;
            if (f->height>main->height) f->height=main->height;
        }
    }
    if (d->vc4) {
        p->vc4=(cc_vc4_configuration){.input=c->source_format};
        p->vc4.output[0]=p->num_out ? c->streams[p->out[0]].format:
            (cc_format){.fourcc=V4L2_PIX_FMT_YUV420,.width=320,.height=240};
        if (p->num_out>1) p->vc4.output[1]=c->streams[p->out[1]].format;
        int ret=active ? cc_vc4_isp_configure(d->vc4,&p->vc4):cc_vc4_isp_try_configuration(d->vc4,&p->vc4);
        if (ret<0) return ret;
        for (size_t i=0;i<p->num_out;++i) {
            c->streams[p->out[i]].format=p->vc4.output[i];
            c->streams[p->out[i]].stream=i ? CC_CAPTURE_OUTPUT1:CC_CAPTURE_OUTPUT0;
        }
    } else {
        p->be=(cc_pisp_be_configuration){.input=c->source_format};
        size_t start=p->num_out==1 ? 1U:0U;
        if (!p->num_out || start) {
            p->be.output=(cc_format){.fourcc=V4L2_PIX_FMT_YUV420,.width=320,.height=240};
            int ret=resize_min(&p->be.output,&(cc_rectangle){0,0,c->source_format.width,c->source_format.height},false);
            if (ret<0) return ret;
            colour(&p->be.output,shared);
        }
        for (size_t i=0;i<p->num_out;++i) {
            cc_format f=c->streams[p->out[i]].format;
            cc_rectangle crop=crop_for(&c->source_format,&f);
            int ret=resize_min(&f,&crop,i+start==1); if (ret<0) return ret;
            if (i) {
                if (f.width>p->be.output.width) f.width=p->be.output.width;
                if (f.height>p->be.output.height) f.height=p->be.output.height;
            }
            p->be.crop[i+start]=crop;
            if (i+start) p->be.output1=f; else p->be.output=f;
        }
        int ret=active ? cc_pisp_be_configure(d->backend,&p->be):cc_pisp_be_try_configuration(d->backend,&p->be);
        if (ret<0) return ret;
        for (size_t i=0;i<p->num_out;++i) {
            c->streams[p->out[i]].format=i+start ? p->be.output1:p->be.output;
            c->streams[p->out[i]].stream=i+start ? CC_CAPTURE_OUTPUT1:CC_CAPTURE_OUTPUT0;
        }
    }
    return 0;
}
static bool format_adjusted(const cc_format *a,const cc_format *b)
{
    if (a->fourcc!=b->fourcc || a->width!=b->width || a->height!=b->height) return true;
    if (a->colorspace && (a->colorspace!=b->colorspace || a->ycbcr_enc!=b->ycbcr_enc ||
        a->quantization!=b->quantization || a->xfer_func!=b->xfer_func)) return true;
    if (a->num_planes && a->num_planes!=b->num_planes) return true;
    for (uint32_t i=0;i<a->num_planes && i<CC_MAX_PLANES;++i)
        if ((a->planes[i].stride && a->planes[i].stride!=b->planes[i].stride) ||
            (a->planes[i].size && a->planes[i].size!=b->planes[i].size)) return true;
    return false;
}
static int adjusted(const cc_pipeline_configuration *old,const cc_pipeline_configuration *c)
{
    if (old->num_streams!=c->num_streams || old->orientation!=c->orientation) return CC_CONFIGURATION_ADJUSTED;
    for (size_t i=0;i<c->num_streams;++i)
        if (format_adjusted(&old->streams[i].format,&c->streams[i].format) ||
            old->streams[i].buffer_count!=c->streams[i].buffer_count) return CC_CONFIGURATION_ADJUSTED;
    return CC_CONFIGURATION_VALID;
}
static int build_plan(const cc_capture_devices *d,cc_pipeline_configuration *c,struct plan *p)
{
    int ret=device_plan(d,p); if (ret<0) return ret;
    if (!c->num_streams) return -EINVAL;
    if (c->num_streams>CC_CONFIGURATION_MAX_STREAMS) return -E2BIG;
    if ((unsigned)c->orientation>8) return -EINVAL;
    c->source_format=(cc_format){0}; c->sensor_format=(cc_sensor_format){0}; c->buffer_count=0;
    for (size_t i=0;i<c->num_streams;++i) {
        cc_stream_configuration *s=&c->streams[i];
        if ((unsigned)s->role>CC_ROLE_VIEWFINDER || s->format.num_planes>CC_MAX_PLANES) return -EINVAL;
        if (!s->buffer_count) s->buffer_count=role_buffers(s->role);
        if (s->buffer_count>256) s->buffer_count=256;
        if (s->buffer_count>c->buffer_count) c->buffer_count=s->buffer_count;
    }
    if (!p->pi && !d->vc4 && !d->backend) {
        c->num_streams=1; memset(c->streams+1,0,2*sizeof(c->streams[0])); c->buffer_count=4;
        c->streams[0].buffer_count=4;
        if (c->sensor.mbus_code || c->sensor.width || c->sensor.height) return -ENOTSUP;
        if (c->orientation) c->orientation=CC_ORIENTATION_ROTATE_0;
        cc_format *f=&c->streams[0].format;
        ret=native_formats(p->camera,f->fourcc,&f->fourcc); if (ret<0) return ret;
        if (p->camera->info.pipeline!=CC_PIPELINE_VIMC) {
            struct size size; ret=native_size(p->camera,f->fourcc,(struct size){f->width,f->height},false,&size);
            if (ret<0) return ret;
            f->width=size.width; f->height=size.height;
        }
        p->raw=0; p->sensor=*f;
        return source_plan(d,c,p,false);
    }
    for (size_t i=0;i<c->num_streams;++i) {
        cc_stream_configuration *s=&c->streams[i];
        bool raw=raw_stream(s);
        if (raw) { if (p->raw!=SIZE_MAX) return -EINVAL; p->raw=i; }
        else { if (p->num_out==2) return -EINVAL; p->out[p->num_out++]=i; }
        if (!s->format.width || !s->format.height) return -EINVAL;
    }
    if (p->num_out==2 && size_compare((struct size){c->streams[p->out[0]].format.width,c->streams[p->out[0]].format.height},
        (struct size){c->streams[p->out[1]].format.width,c->streams[p->out[1]].format.height})<0) {
        size_t swap=p->out[0]; p->out[0]=p->out[1]; p->out[1]=swap;
    }
    if (p->num_out && !d->vc4 && !d->backend) return -ENOTSUP;
    if (p->pi) {
        size_t choose=p->raw!=SIZE_MAX ? p->raw:p->out[0];
        uint32_t depth=12;
        if (p->raw!=SIZE_MAX) {
            depth=c->streams[p->raw].sensor_bit_depth;
            if (!depth && cc_bayer_mbus(c->streams[p->raw].format.fourcc))
                (void)cc_bayer_code_info(cc_bayer_mbus(c->streams[p->raw].format.fourcc),&depth,NULL);
            if (!depth) {
                struct format_order order=format_order(c->streams[p->raw].format.fourcc);
                struct cc_pisp_raw_layout raw;
                bool compressed=!cc_pisp_raw_format(order.native,&raw) && raw.compression;
                if (order.encoding==3 && !compressed) depth=order.depth;
            }
            if (!depth) depth=12;
            c->streams[p->raw].sensor_bit_depth=depth;
        }
        ret=sensor_choice(p,c,(struct size){c->streams[choose].format.width,c->streams[choose].format.height},depth);
        if (ret<0) return ret;
    } else {
        if (c->sensor.mbus_code || c->sensor.width || c->sensor.height) return -ENOTSUP;
        ret=cc_camera_get_format(p->camera,&p->sensor); if (ret<0) return ret;
        if (p->raw!=SIZE_MAX) p->sensor=c->streams[p->raw].format;
        if (!raw_format(p->sensor.fourcc)) return -ENOTSUP;
    }
    ret=source_plan(d,c,p,false); if (ret<0) return ret;
    return processor_plan(d,c,p,false);
}
int cc_pipeline_validate_configuration(const cc_capture_devices *d,cc_pipeline_configuration *configuration)
{
    if (!configuration) return -EINVAL;
    cc_pipeline_configuration c=*configuration; struct plan p={0};
    int ret=build_plan(d,&c,&p); if (ret<0) return ret;
    ret=adjusted(configuration,&c); *configuration=c; return ret;
}
int cc_pipeline_generate_configuration(const cc_capture_devices *d,const enum cc_stream_role *roles,size_t count,
    cc_pipeline_configuration *configuration)
{
    if (!configuration || (!roles && count)) return -EINVAL;
    struct plan p={0}; int ret=device_plan(d,&p); if (ret<0) return ret;
    if (count>CC_CONFIGURATION_MAX_STREAMS) return -E2BIG;
    cc_pipeline_configuration c={.orientation=CC_ORIENTATION_ROTATE_0,.num_streams=count};
    if (!count) { *configuration=c; return CC_CONFIGURATION_VALID; }
    cc_camera_properties properties={0};
    /* Role defaults use the largest advertised sensor output, not the physical
     * pixel-array bounds (which can include pixels unavailable to capture). */
    if (p.pi) { ret=cc_sensor_mode_properties(p.camera,0,&properties); if (ret<0) return ret; }
    for (size_t i=0;i<count;++i) {
        if ((unsigned)roles[i]>CC_ROLE_VIEWFINDER) return -EINVAL;
        cc_stream_configuration *s=&c.streams[i]; s->role=roles[i]; s->buffer_count=role_buffers(roles[i]);
        if (!p.pi && !d->vc4 && !d->backend) {
            s->buffer_count=4;
            ret=native_formats(p.camera,0,&s->format.fourcc); if (ret<0) return ret;
            struct size size={1920,1080};
            if (p.camera->info.pipeline!=CC_PIPELINE_VIMC) {
                ret=native_size(p.camera,s->format.fourcc,size,true,&size); if (ret<0) return ret;
            }
            s->format.width=size.width; s->format.height=size.height;
        } else {
            struct size full={properties.pixel_array_width,properties.pixel_array_height};
            if (!p.pi) {
                cc_format source; ret=cc_camera_get_format(p.camera,&source); if (ret<0) return ret;
                full=(struct size){source.width,source.height};
            }
            if (!full.width || !full.height) return -ENODATA;
            if (roles[i]==CC_ROLE_RAW) {
                if (p.pi) {
                    struct plan sensor=p; ret=sensor_choice(&sensor,&c,full,12); if (ret<0) return ret;
                    s->format=sensor.sensor; s->sensor_bit_depth=12;
                } else { ret=cc_camera_get_format(p.camera,&s->format); if (ret<0) return ret; }
            } else {
                s->format=(cc_format){.fourcc=roles[i]==CC_ROLE_VIEWFINDER ? V4L2_PIX_FMT_XBGR32:V4L2_PIX_FMT_YUV420,
                    .width=roles[i]==CC_ROLE_VIDEO_RECORDING ? 1920:roles[i]==CC_ROLE_VIEWFINDER ? 800:full.width,
                    .height=roles[i]==CC_ROLE_VIDEO_RECORDING ? 1080:roles[i]==CC_ROLE_VIEWFINDER ? 600:full.height};
                colour(&s->format,roles[i]==CC_ROLE_VIDEO_RECORDING ? V4L2_COLORSPACE_REC709:V4L2_COLORSPACE_JPEG);
            }
        }
    }
    ret=cc_pipeline_validate_configuration(d,&c); if (ret<0) return ret;
    *configuration=c; return ret;
}
static void invalidate(const cc_capture_devices *d,cc_camera *camera)
{
    camera->stream->configured=false;
    cc_cfe_invalidate_configuration(d->frontend); cc_vc4_invalidate_configuration(d->vc4); cc_be_invalidate_configuration(d->backend);
}
int cc_pipeline_configure(const cc_capture_devices *d,cc_pipeline_configuration *configuration)
{
    if (!configuration) return -EINVAL;
    cc_pipeline_configuration c=*configuration; struct plan p={0};
    int ret=build_plan(d,&c,&p); if (ret<0) return ret;
    if (p.camera->stream->allocated || p.camera->stream->streaming ||
        (p.camera->embedded && p.camera->embedded->stream->allocated)) return -EBUSY;
    if (d->frontend && (ret=cc_cfe_configuration_ready(d->frontend))) return ret;
    if (d->vc4 && (ret=cc_vc4_configuration_ready(d->vc4))) return ret;
    if (d->backend && (ret=cc_be_configuration_ready(d->backend))) return ret;
    invalidate(d,p.camera);
    ret=source_plan(d,&c,&p,true);
    if (!ret) ret=processor_plan(d,&c,&p,true);
    if (ret<0) { invalidate(d,p.camera); return ret; }
    ret=adjusted(configuration,&c); *configuration=c; return ret;
}
