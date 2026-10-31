/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "internal.h"
#include "subdev_compat.h"
#include <errno.h>
#include <stdbool.h>

struct driver {
    uint8_t mode, fault;
    uint32_t image, metadata, count;
    struct cc_subdev_route routes[64];
};
static int routing_ioctl(void *context, int fd, unsigned long request, void *argument)
{
    struct driver *d=context; FUZZ_CHECK(fd==11);
    if (request==CC_SD_G_ROUTING || request==CC_SD_G_ROUTING_LEGACY) {
        uint64_t pointer; uint32_t capacity;
        if (request==CC_SD_G_ROUTING) {
            if (d->mode&2U) return -ENOTTY;
            struct cc_subdev_routing *r=argument; pointer=r->routes; capacity=r->len_routes; r->num_routes=d->count;
            if (d->fault==1) r->routes=0;
        } else {
            FUZZ_CHECK(d->mode&2U); struct cc_subdev_routing_legacy *r=argument;
            pointer=r->routes; capacity=r->num_routes; r->num_routes=d->count;
            if (d->fault==1) r->routes=0;
        }
        if (d->fault==2) return -EIO;
        size_t n=d->count<capacity ? d->count:capacity;
        FUZZ_CHECK(n<=64); memcpy((void *)(uintptr_t)pointer,d->routes,n*sizeof(*d->routes));
        return 0;
    }
    if (request==CC_SD_S_ROUTING || request==CC_SD_S_ROUTING_LEGACY) {
        uint64_t pointer; uint32_t count;
        if (request==CC_SD_S_ROUTING) {
            if (d->mode&2U) return -ENOTTY;
            struct cc_subdev_routing *r=argument; pointer=r->routes; count=r->num_routes;
            if (d->fault==3) ++r->num_routes;
        } else {
            FUZZ_CHECK(d->mode&2U); struct cc_subdev_routing_legacy *r=argument;
            pointer=r->routes; count=r->num_routes; if (d->fault==3) ++r->num_routes;
        }
        FUZZ_CHECK(count<=64); if (d->fault==4) return -EIO;
        struct cc_subdev_route *routes=(void *)(uintptr_t)pointer;
        if ((d->mode&16U) && count>1) {
            struct cc_subdev_route tmp=routes[0]; routes[0]=routes[count-1]; routes[count-1]=tmp;
        }
        if (d->fault==5 && count) routes[0].flags^=CC_SD_ROUTE_ACTIVE;
        memcpy(d->routes,routes,count*sizeof(*routes));
        return 0;
    }
    if (request==VIDIOC_SUBDEV_ENUM_MBUS_CODE) {
        struct cc_subdev_mbus_code_enum *c=argument;
        if (d->fault==6) return -EIO;
        if (c->index && d->fault!=7) return -EINVAL;
        if (c->pad==2 && c->stream==d->image) c->code=0x300f;
        else if (c->pad==2) c->code=0x8002;
        else if (c->pad==1) c->code=0x9001;
        else if (c->pad==3) c->code=0x8001;
        else return -EINVAL;
        if (d->fault==8) ++c->stream;
        return 0;
    }
    FUZZ_CHECK(request==VIDIOC_SUBDEV_G_FMT);
    struct cc_subdev_format *f=argument;
    if (d->fault==9) return -EIO;
    if (d->mode&64U) {
        if (f->pad==0) f->format.code=0x300f;
        else if (f->pad==1) f->format.code=0x7001;
        else return -EINVAL;
    } else if (f->pad==2 && f->stream==d->image) f->format.code=0x300f;
    else if (f->pad==2) f->format.code=0x8002;
    else return -EINVAL;
    f->format.width=640; f->format.height=480;
    if (d->fault==10) ++f->stream;
    return 0;
}
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in={data,size}; struct driver d={0};
    d.mode=fuzz_u8(&in); d.image=fuzz_u32(&in); d.metadata=d.image^UINT32_C(0x80000000);
    d.count=(d.mode&1U) ? fuzz_u8(&in)%71U:2U;
    d.fault=(d.mode&4U) ? (uint8_t)(fuzz_u8(&in)%11U):0;
    for (uint32_t i=0;i<64;++i) d.routes[i]=(struct cc_subdev_route){.sink_pad=i==0 ? 0U:i==1 ? 1U:3U,
        .source_pad=2,.source_stream=i==0 ? d.image:i==1 ? d.metadata:d.image+i,
        .flags=fuzz_u8(&in)&CC_SD_ROUTE_ACTIVE};
    if (d.mode&8U) {
        size_t n=d.count<64 ? d.count:64;
        if (n) fuzz_patch(&in,d.routes,n*sizeof(*d.routes));
    }
    struct media_pad_desc pads[4];
    for (uint16_t i=0;i<4;++i) pads[i]=(struct media_pad_desc){.entity=1,.index=i,
        .flags=d.mode&64U ? (i<2 ? MEDIA_PAD_FL_SOURCE:MEDIA_PAD_FL_SINK):
                           i==2 ? MEDIA_PAD_FL_SOURCE:MEDIA_PAD_FL_SINK};
    struct cc_entity entity={.desc={.id=1,.pads=4},.pads=pads};
    struct cc_media media={.entities=&entity,.num_entities=1};
    bool multiplexed=(d.mode&64U)==0;
    const struct cc_io_ops ops={.ioctl=routing_ioctl};
    cc_camera camera={.io={&ops,&d},.media=&media,.sensor_fd=11,
        .sensor_pad=multiplexed ? 2U:0U,.subdev_streams=&multiplexed};
    camera.info.sensor_entity=1;
    cc_sensor_streams streams; memset(&streams,0xe7,sizeof(streams)); cc_sensor_streams saved=streams;
    int ret=cc_camera_get_sensor_streams(&camera,&streams); FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(!memcmp(&streams,&saved,sizeof(saved)));
    else {
        FUZZ_CHECK(streams.image.pad==camera.sensor_pad && streams.image.stream==(multiplexed ? d.image:0U));
        if (streams.flags&CC_SENSOR_STREAMS_EMBEDDED) {
            bool found=!multiplexed && streams.embedded.stream==0;
            for (uint32_t i=0;i<d.count && i<64;++i)
                if (d.routes[i].source_pad==2 && d.routes[i].sink_pad==1 &&
                    d.routes[i].source_stream==streams.embedded.stream) found=true;
            FUZZ_CHECK(found);
        }
    }
    uint32_t stream=UINT32_C(0xfeedbeef);
    ret=cc_sensor_streams_image_route(&camera,V4L2_SUBDEV_FORMAT_ACTIVE,&stream); FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(stream==UINT32_C(0xfeedbeef));
    else FUZZ_CHECK(stream==(multiplexed ? d.image:0U) && (!multiplexed || camera.sensor_stream==stream));
    return 0;
}
