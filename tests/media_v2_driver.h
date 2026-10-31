/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Convert the existing independent fake-driver definitions into a complete v2
 * reply. Production must consume v2 and never call legacy enumeration itself. */
#ifndef CCAMERA_TEST_MEDIA_V2_DRIVER_H
#define CCAMERA_TEST_MEDIA_V2_DRIVER_H
#include "internal.h"
#include <errno.h>
#include <string.h>
static uint32_t test_pad_id(uint32_t entity,uint32_t index)
{
    return UINT32_C(0x03000000)+entity*256U+index;
}
static int test_v2_topology(void *context,int fd,unsigned long request,void *argument,
                            int (*legacy)(void *,int,unsigned long,void *),bool lens)
{
    (void)request;
    struct media_v2_entity entities[64]; struct media_v2_interface interfaces[64];
    struct media_v2_pad pads[256]; struct media_v2_link links[512];
    uint32_t ne=0,ni=0,np=0,nl=0,previous=0;
    for (;;) {
        struct media_entity_desc entity={.id=previous|MEDIA_ENT_ID_FLAG_NEXT};
        int ret=legacy(context,fd,MEDIA_IOC_ENUM_ENTITIES,&entity);
        if (ret==-EINVAL) break;
        if (ret<0) return ret;
        if (ne>=64 || entity.pads>256 || entity.links>512 || entity.id<=previous || entity.id>=65536) return -EOVERFLOW;
        previous=entity.id;
        entities[ne]=(struct media_v2_entity){.id=entity.id,.function=entity.type,.flags=entity.flags};
        cc_copy_string(entities[ne++].name,sizeof(entities[0].name),entity.name,sizeof(entity.name));
        if (entity.dev.major) {
            interfaces[ni++]=(struct media_v2_interface){.id=UINT32_C(0x01000000)+entity.id,
                .intf_type=entity.type==MEDIA_ENT_F_IO_V4L ? MEDIA_INTF_T_V4L_VIDEO:MEDIA_INTF_T_V4L_SUBDEV,
                .devnode={entity.dev.major,entity.dev.minor}};
            if (nl>=512) return -EOVERFLOW;
            links[nl]=(struct media_v2_link){.id=UINT32_C(0x02000000)+nl,.source_id=UINT32_C(0x01000000)+entity.id,
                .sink_id=entity.id,.flags=MEDIA_LNK_FL_INTERFACE_LINK|MEDIA_LNK_FL_ENABLED|MEDIA_LNK_FL_IMMUTABLE}; ++nl;
        }
        struct media_pad_desc native_pads[256]; struct media_link_desc native_links[512];
        memset(native_pads,0,sizeof(native_pads)); memset(native_links,0,sizeof(native_links));
        struct media_links_enum enumeration={.entity=entity.id,.pads=native_pads,.links=native_links};
        ret=legacy(context,fd,MEDIA_IOC_ENUM_LINKS,&enumeration);
        if (ret<0) return ret;
        for (uint32_t i=0;i<entity.pads;++i) {
            if (np>=256) return -EOVERFLOW;
            pads[np++]=(struct media_v2_pad){.id=test_pad_id(entity.id,native_pads[i].index),
                .entity_id=entity.id,.index=native_pads[i].index,.flags=native_pads[i].flags};
        }
        for (uint32_t i=0;i<entity.links;++i) {
            const struct media_link_desc *link=&native_links[i];
            if (link->source.entity!=entity.id) continue;
            if (nl>=512) return -EOVERFLOW;
            links[nl]=(struct media_v2_link){.id=UINT32_C(0x02000000)+nl,
                .source_id=test_pad_id(link->source.entity,link->source.index),
                .sink_id=test_pad_id(link->sink.entity,link->sink.index),.flags=link->flags}; ++nl;
        }
    }
    if (lens) {
        if (nl>=512) return -EOVERFLOW;
        links[nl]=(struct media_v2_link){.id=UINT32_C(0x02000000)+nl,.source_id=1,.sink_id=8,.flags=MEDIA_LNK_FL_ANCILLARY_LINK}; ++nl;
    }
    /* The UAPI imposes no entity/pad/interface array ordering. */
    for (uint32_t i=0;i<ne/2;++i) { struct media_v2_entity e=entities[i]; entities[i]=entities[ne-1-i]; entities[ne-1-i]=e; }
    for (uint32_t i=0;i<np/2;++i) { struct media_v2_pad p=pads[i]; pads[i]=pads[np-1-i]; pads[np-1-i]=p; }
    for (uint32_t i=0;i<ni/2;++i) { struct media_v2_interface p=interfaces[i]; interfaces[i]=interfaces[ni-1-i]; interfaces[ni-1-i]=p; }
    struct media_v2_topology *out=argument;
    int ret=0;
    if ((out->ptr_entities && out->num_entities<ne) || (out->ptr_interfaces && out->num_interfaces<ni) ||
        (out->ptr_pads && out->num_pads<np) || (out->ptr_links && out->num_links<nl)) ret=-ENOSPC;
    if (!ret) {
        if (out->ptr_entities) memcpy((void *)(uintptr_t)out->ptr_entities,entities,ne*sizeof(*entities));
        if (out->ptr_interfaces) memcpy((void *)(uintptr_t)out->ptr_interfaces,interfaces,ni*sizeof(*interfaces));
        if (out->ptr_pads) memcpy((void *)(uintptr_t)out->ptr_pads,pads,np*sizeof(*pads));
        if (out->ptr_links) memcpy((void *)(uintptr_t)out->ptr_links,links,nl*sizeof(*links));
    }
    out->num_entities=ne; out->num_interfaces=ni; out->num_pads=np; out->num_links=nl; out->topology_version=1;
    return ret;
}
#endif
