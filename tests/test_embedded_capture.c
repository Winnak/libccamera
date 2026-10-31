/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "sensor_stream_driver.h"
#include "embedded_capture.h"
enum { META=21, META_NODE=4, MAX_SLOTS=8, MAX_PACKETS=32 };
struct packet { uint32_t slot,sequence,flags; uint64_t timestamp; uint8_t marker; };
struct queue_fake {
    struct fake base;
    struct cc_entity entities[4];
    struct media_link_desc links[4]; size_t num_links;
    struct media_pad_desc receiver_pads[4],metadata_pad;
    struct cc_subdev_format metadata_pad_format;
    struct { bool streaming,queued[MAX_SLOTS]; uint32_t count; void *data[MAX_SLOTS]; } nodes[2];
    struct { uint32_t fourcc,size,width,height,stride; } format;
    struct packet packets[2][MAX_PACKETS]; size_t total[2],next[2];
    bool fixed,hold_metadata,fail_stop,wrong_format,bad_ack;
    uint32_t force_metadata_count;
    unsigned maps,exports,waits,ready_at; uint64_t now;
    int last_wait_fd;
};
static int queue_ioctl(void *context,int fd,unsigned long request,void *argument)
{
    struct queue_fake *q=context; struct fake *f=&q->base;
    if (request==f->fail_request && (!f->fail_fd || f->fail_fd==fd)) return -EIO;
    if (request==MEDIA_IOC_DEVICE_INFO && q->fixed) {
        struct media_device_info *info=argument; strcpy(info->driver,"unicam"); return 0;
    }
    if (request==MEDIA_IOC_ENUM_ENTITIES) {
        struct media_entity_desc *entity=argument; uint32_t previous=entity->id&~MEDIA_ENT_ID_FLAG_NEXT;
        if (previous>=4) return -EINVAL;
        *entity=q->entities[previous].desc; entity->dev.major=81;
        entity->dev.minor=previous==0 ? SENSOR:previous==1 ? RECEIVER:previous==2 ? VIDEO:META;
        entity->links=0;
        for (size_t i=0;i<q->num_links;++i)
            if (q->links[i].source.entity==entity->id || q->links[i].sink.entity==entity->id) ++entity->links;
        return 0;
    }
    if (request==MEDIA_IOC_ENUM_LINKS) {
        struct media_links_enum *links=argument; CHECK(links->entity>=1 && links->entity<=4);
        struct cc_entity *entity=&q->entities[links->entity-1];
        memcpy(links->pads,entity->pads,entity->desc.pads*sizeof(*entity->pads));
        size_t count=0;
        for (size_t i=0;i<q->num_links;++i)
            if (q->links[i].source.entity==links->entity || q->links[i].sink.entity==links->entity)
                links->links[count++]=q->links[i];
        return 0;
    }
    if (request==MEDIA_IOC_SETUP_LINK) {
        struct media_link_desc *link=argument;
        for (size_t i=0;i<q->num_links;++i) if (q->links[i].source.entity==link->source.entity &&
            q->links[i].source.index==link->source.index && q->links[i].sink.entity==link->sink.entity &&
            q->links[i].sink.index==link->sink.index) { q->links[i]=*link; return 0; }
        CHECK(false);
    }
    if (request==CC_SD_QUERYCAP && q->fixed && fd==RECEIVER) {
        struct cc_subdev_capability *caps=argument; caps->capabilities=0; return 0;
    }
    if (request==CC_SD_S_ROUTING || request==CC_SD_S_ROUTING_LEGACY) {
        if (request==CC_SD_S_ROUTING && f->legacy) return -ENOTTY;
        uint32_t count; struct cc_subdev_route *routes;
        if (request==CC_SD_S_ROUTING) { struct cc_subdev_routing *r=argument; count=r->num_routes; routes=(void *)(uintptr_t)r->routes; }
        else { struct cc_subdev_routing_legacy *r=argument; count=r->num_routes; routes=(void *)(uintptr_t)r->routes; }
        if (fd==SENSOR) {
            CHECK(count==3); memcpy(f->routes[1],routes,count*sizeof(*routes));
            if (q->bad_ack) routes[1].flags^=CC_SD_ROUTE_ACTIVE;
            return 0;
        }
        if (count==2) {
            CHECK(fd==RECEIVER && routes[0].sink_stream==7 && routes[1].sink_stream==42 && routes[1].source_pad==2);
            return 0;
        }
    }
    if ((request==VIDIOC_SUBDEV_G_FMT || request==VIDIOC_SUBDEV_S_FMT) && fd==RECEIVER) {
        struct cc_subdev_format *format=argument;
        uint32_t sink=q->fixed ? 2U:0U,source=q->fixed ? 3U:2U,input_stream=q->fixed ? 0U:42U;
        if ((format->pad==sink && format->stream==input_stream) || format->pad==source) {
            if (request==VIDIOC_SUBDEV_S_FMT) q->metadata_pad_format=*format;
            else format->format=q->metadata_pad_format.format;
            return 0;
        }
        if (request==VIDIOC_SUBDEV_S_FMT && format->pad==1) {
            CHECK(!format->stream); f->receiver_format[format->which]=format->format; return 0;
        }
        if (request==VIDIOC_SUBDEV_G_FMT && !format->pad) {
            format->format=f->receiver_format[format->which]; return 0;
        }
    }
    if (fd!=VIDEO && fd!=META) return fake_ioctl(context,fd,request,argument);
    size_t node=fd==VIDEO ? 0U:1U;
    if (request==VIDIOC_QUERYCAP && fd==META) {
        struct v4l2_capability *caps=argument; caps->capabilities=V4L2_CAP_STREAMING|V4L2_CAP_META_CAPTURE; return 0;
    }
    if ((request==VIDIOC_S_FMT || request==VIDIOC_G_FMT) && fd==META) {
        struct v4l2_format *format=argument; CHECK(format->type==V4L2_BUF_TYPE_META_CAPTURE);
        if (request==VIDIOC_S_FMT) {
            memcpy(&q->format,format->fmt.raw_data,sizeof(q->format));
            if (!q->fixed) { q->format.stride=1024; q->format.size=q->format.stride*q->format.height; }
            if (q->wrong_format) ++q->format.fourcc;
        }
        memcpy(format->fmt.raw_data,&q->format,sizeof(q->format)); return 0;
    }
    if (request==VIDIOC_REQBUFS) {
        struct v4l2_requestbuffers *r=argument;
        CHECK(r->memory==V4L2_MEMORY_MMAP);
        if (r->count) { r->count=fd==META && q->force_metadata_count ? q->force_metadata_count:3; CHECK(r->count<=MAX_SLOTS); }
        else CHECK(!q->nodes[node].streaming);
        q->nodes[node].count=r->count; memset(q->nodes[node].queued,0,sizeof(q->nodes[node].queued)); return 0;
    }
    if (request==VIDIOC_QUERYBUF) {
        struct v4l2_buffer *b=argument; CHECK(b->index<q->nodes[node].count);
        b->length=fd==META ? q->format.size:f->video_format.fmt.pix.sizeimage;
        b->m.offset=b->index*65536U; return 0;
    }
    if (request==VIDIOC_QBUF) {
        struct v4l2_buffer *b=argument; CHECK(b->index<q->nodes[node].count && !q->nodes[node].queued[b->index]);
        q->nodes[node].queued[b->index]=true; return 0;
    }
    if (request==VIDIOC_STREAMON) { q->nodes[node].streaming=true; return 0; }
    if (request==VIDIOC_STREAMOFF) {
        if (q->fail_stop && fd==META) return -EIO;
        q->nodes[node].streaming=false; memset(q->nodes[node].queued,0,sizeof(q->nodes[node].queued)); return 0;
    }
    if (request==VIDIOC_DQBUF) {
        if (!q->nodes[node].streaming || q->next[node]==q->total[node] || (fd==META && q->hold_metadata)) return -EAGAIN;
        const struct packet *p=&q->packets[node][q->next[node]];
        if (p->slot>=q->nodes[node].count || !q->nodes[node].queued[p->slot]) return -EAGAIN;
        ++q->next[node]; q->nodes[node].queued[p->slot]=false;
        struct v4l2_buffer *b=argument; b->index=p->slot; b->sequence=p->sequence; b->flags=p->flags;
        b->timestamp.tv_sec=(long)(p->timestamp/UINT64_C(1000000000));
        b->timestamp.tv_usec=(long)((p->timestamp%UINT64_C(1000000000))/1000);
        b->bytesused=fd==META ? q->format.size:16;
        memset(q->nodes[node].data[p->slot],p->marker,b->bytesused); return 0;
    }
    if (request==VIDIOC_EXPBUF) { struct v4l2_exportbuffer *e=argument; e->fd=(int)(1000+q->exports++); return 0; }
    return fake_ioctl(context,fd,request,argument);
}
static int queue_open(void *context,const char *path,int flags)
{
    struct queue_fake *q=context;
    if (!strcmp(path,"/embedded")) { CHECK(flags&O_CLOEXEC); ++q->base.opened; return META; }
    return fake_open(context,path,flags);
}
static int queue_close(void *context,int fd)
{
    struct queue_fake *q=context;
    if (fd>=1000) return 0;
    if (fd==VIDEO || fd==META) { size_t node=fd==VIDEO ? 0U:1U; q->nodes[node].streaming=false; }
    if (fd==META) { ++q->base.closed; return 0; }
    return fake_close(context,fd);
}
static int queue_resolve(void *context,unsigned major_id,unsigned minor_id,char *path,size_t size)
{
    if (minor_id==META) { CHECK(major_id==81 && size>strlen("/embedded")); strcpy(path,"/embedded"); return 0; }
    return fake_resolve(context,major_id,minor_id,path,size);
}
static int queue_map(void *context,int fd,size_t size,off_t offset,void **out)
{
    struct queue_fake *q=context; CHECK((fd==VIDEO || fd==META) && offset>=0 && offset/65536<MAX_SLOTS);
    *out=malloc(size); CHECK(*out); ++q->maps;
    q->nodes[fd==VIDEO ? 0:1].data[offset/65536]=*out; return 0;
}
static int queue_unmap(void *context,void *data,size_t size)
{
    struct queue_fake *q=context; (void)size; CHECK(q->maps); --q->maps; free(data); return 0;
}
static int queue_now(void *context,uint64_t *out) { *out=((struct queue_fake *)context)->now; return 0; }
static int queue_wait(void *context,struct pollfd *fds,size_t count,int timeout)
{
    struct queue_fake *q=context; CHECK(count && timeout!=0); q->last_wait_fd=fds[0].fd;
    ++q->waits; q->now+=UINT64_C(1000000);
    if (q->ready_at && q->waits>=q->ready_at) q->hold_metadata=false;
    return -EAGAIN;
}
static int queue_wait_one(void *context,int fd,short events,int timeout)
{
    struct pollfd p={fd,events,0}; return queue_wait(context,&p,1,timeout);
}
static const struct cc_io_ops queue_ops={.open=queue_open,.close=queue_close,.lock=fake_lock,.ioctl=queue_ioctl,
    .resolve=queue_resolve,.map=queue_map,.unmap=queue_unmap,.now=queue_now,.wait=queue_wait_one,.wait_many=queue_wait};
static cc_camera *open_camera(struct queue_fake *q,bool fixed)
{
    memset(q,0,sizeof(*q)); initialize(&q->base,fixed); q->fixed=fixed;
    memcpy(q->entities,q->base.entities,sizeof(q->base.entities));
    memcpy(q->links,q->base.links,sizeof(q->base.links)); q->num_links=fixed ? 4:3;
    strcpy(q->entities[0].desc.name,"imx708");
    strcpy(q->entities[2].desc.name,fixed ? "unicam-image":"rp1-cfe-csi2-ch0");
    q->entities[3].desc=(struct media_entity_desc){.id=META_NODE,.type=MEDIA_ENT_F_IO_V4L,.pads=1};
    strcpy(q->entities[3].desc.name,fixed ? "unicam-embedded":"rp1-cfe-csi2-ch1");
    q->metadata_pad=(struct media_pad_desc){.entity=META_NODE,.flags=MEDIA_PAD_FL_SINK}; q->entities[3].pads=&q->metadata_pad;
    memcpy(q->receiver_pads,q->base.receiver_pads,sizeof(q->base.receiver_pads));
    q->receiver_pads[2]=(struct media_pad_desc){.entity=2,.index=2,.flags=fixed ? MEDIA_PAD_FL_SINK:MEDIA_PAD_FL_SOURCE};
    q->receiver_pads[3]=(struct media_pad_desc){.entity=2,.index=3,.flags=MEDIA_PAD_FL_SOURCE};
    q->entities[1].desc.pads=fixed ? 4:3; q->entities[1].pads=q->receiver_pads;
    q->links[2]=(struct media_link_desc){.source={.entity=2,.index=fixed ? 3:2},.sink={.entity=META_NODE}};
    if (fixed) q->links[3]=(struct media_link_desc){.source={.entity=1,.index=1},.sink={.entity=2,.index=2}};
    cc_camera_info info={.pipeline=fixed ? CC_PIPELINE_RPI_UNICAM:CC_PIPELINE_RPI_CFE,.sensor_entity=1};
    strcpy(info.media_path,"/media"); strcpy(info.video_path,"/video");
    struct cc_io io={&queue_ops,q}; cc_camera *camera;
    OK(cc_camera_open_io(&info,&io,&camera));
    cc_format format=raw_format(); OK(cc_camera_configure(camera,&format)); return camera;
}
static void packet(struct queue_fake *q,size_t node,uint32_t slot,uint64_t time,uint32_t sequence,uint8_t marker)
{
    CHECK(q->total[node]<MAX_PACKETS);
    q->packets[node][q->total[node]++]=(struct packet){slot,sequence,V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,time,marker};
}
static void test_pairing(bool fixed,bool legacy)
{
    struct queue_fake q; cc_camera *camera=open_camera(&q,fixed); q.base.legacy=legacy;
    cc_embedded_configuration configuration={0}; OK(cc_camera_configure_embedded(camera,&configuration));
    CHECK(configuration.bits_per_pixel==10 && configuration.format.height==4 && configuration.format.planes[0].stride==(fixed ? 800U:1024U));
    uint32_t count=3; OK(cc_camera_allocate_buffers(camera,&count)); CHECK(count==2);
    CHECK(cc_camera_configure_embedded(camera,NULL)==-EBUSY);
    /* Different buffer indices and sequence counters, exact shared timestamps. */
    packet(&q,0,0,2000000,100,10); packet(&q,1,2,1000000,77,1); packet(&q,1,1,2000000,78,2);
    OK(cc_camera_queue(camera,0)); OK(cc_camera_start(camera)); cc_frame frame,embedded;
    OK(cc_camera_dequeue(camera,&frame,0)); OK(cc_camera_embedded_frame(camera,0,&embedded));
    CHECK(frame.sequence==100 && embedded.sequence==78 && embedded.index==1 && ((uint8_t *)embedded.planes[0].data)[0]==2);
    CHECK(q.nodes[1].queued[2] && !q.nodes[1].queued[1]);
    int fd=cc_camera_export_embedded_plane(camera,0,0); CHECK(fd>=1000); OK(queue_close(&q,fd));
    /* A future packet proves absence; preserve it for the following image. */
    packet(&q,0,1,3000000,101,11); packet(&q,1,0,4000000,79,4);
    OK(cc_camera_queue(camera,1)); OK(cc_camera_dequeue(camera,&frame,0));
    CHECK(cc_camera_embedded_frame(camera,1,&embedded)==-ENODATA);
    CHECK(((uint8_t *)camera->embedded->stream->buffers[1].frame.planes[0].data)[0]==2);
    packet(&q,0,1,4000000,102,12); OK(cc_camera_queue(camera,1)); OK(cc_camera_dequeue(camera,&frame,0));
    OK(cc_camera_embedded_frame(camera,1,&embedded)); CHECK(embedded.sequence==79 && embedded.index==0);
    OK(cc_camera_stop(camera)); CHECK(cc_camera_embedded_frame(camera,0,&embedded)==-ENODATA);
    OK(cc_camera_free_buffers(camera)); OK(cc_camera_configure_embedded(camera,NULL));
    CHECK(cc_camera_embedded_fd(camera)==-ENOENT); cc_camera_close(camera);
    CHECK(!q.maps && q.base.opened==q.base.closed);
}
static void test_partial_and_graph(void)
{
    struct queue_fake q; cc_camera *camera=open_camera(&q,false); cc_embedded_configuration cfg;
    OK(cc_camera_configure_embedded(camera,&cfg)); uint32_t count=2; OK(cc_camera_allocate_buffers(camera,&count));
    packet(&q,0,0,1000000,1,1); packet(&q,1,2,1000000,500,50); q.hold_metadata=true;
    OK(cc_camera_queue(camera,0)); OK(cc_camera_start(camera)); cc_frame frame;
    CHECK(cc_camera_dequeue(camera,&frame,2)==-ETIMEDOUT && q.last_wait_fd==META && camera->embedded->image_pending);
    q.ready_at=q.waits+1; OK(cc_camera_dequeue(camera,&frame,2)); CHECK(!camera->embedded->image_pending);
    OK(cc_camera_stop(camera));
    cc_capture_devices devices={.camera=camera}; cc_capture *capture;
    OK(cc_capture_create(&devices,&capture,&count)); CHECK(!devices.camera && count==2);
    cc_request *request; OK(cc_request_create(77,&request));
    packet(&q,0,0,2000000,2,2); packet(&q,1,1,2000000,501,51);
    OK(cc_capture_queue(capture,request)); cc_request *done; OK(cc_capture_dequeue(capture,&done,0)); CHECK(done==request);
    const cc_capture_result *result=cc_request_get_result(done);
    CHECK(result->status==CC_REQUEST_COMPLETE && result->embedded.sequence==501 && result->embedded.timestamp_ns==result->raw.timestamp_ns);
    int fd=cc_capture_export_plane(capture,done,CC_CAPTURE_EMBEDDED,0); CHECK(fd>=1000); OK(queue_close(&q,fd));
    OK(cc_capture_release(capture,done)); OK(cc_request_destroy(request)); cc_capture_close(capture);
    CHECK(!q.maps && q.base.opened==q.base.closed);
}
static void test_failures(void)
{
    for (unsigned scenario=0;scenario<6;++scenario) {
        struct queue_fake q; cc_camera *camera=open_camera(&q,false); cc_embedded_configuration cfg;
        if (scenario==0) q.wrong_format=true;
        if (scenario==1) q.bad_ack=true;
        int ret=cc_camera_configure_embedded(camera,&cfg);
        if (scenario<2) CHECK(ret==(scenario==0 ? -EPIPE:-EPROTO));
        else {
            OK(ret); uint32_t count=2;
            if (scenario==2) q.force_metadata_count=1;
            ret=cc_camera_allocate_buffers(camera,&count);
            if (scenario==2) CHECK(ret==-ENOSPC);
            else {
                OK(ret); packet(&q,0,0,2000000,1,1); packet(&q,1,0,2000000,1,2);
                OK(cc_camera_queue(camera,0)); OK(cc_camera_start(camera));
                if (scenario==3) { q.base.fail_request=VIDIOC_DQBUF; q.base.fail_fd=META; }
                if (scenario==4) q.fail_stop=true;
                if (scenario==5) q.packets[1][0].flags=V4L2_BUF_FLAG_TIMESTAMP_COPY;
                cc_frame frame;
                if (scenario==4) CHECK(cc_camera_stop(camera)==-EIO);
                else CHECK(cc_camera_dequeue(camera,&frame,0)==(scenario==3 ? -EIO:-EPROTO));
                q.base.fail_request=0;
                CHECK(!q.nodes[0].streaming && !q.nodes[1].streaming);
            }
        }
        cc_camera_close(camera); CHECK(!q.maps && q.base.opened==q.base.closed);
    }
}
int main(void)
{
    test_pairing(false,false); test_pairing(false,true); test_pairing(true,false);
    test_partial_and_graph(); test_failures();
    puts("Native embedded ownership/routing, timestamp pairing, held views, partial deadlines and graph cancellation passed"); return 0;
}
