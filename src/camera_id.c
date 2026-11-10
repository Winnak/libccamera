/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Firmware, virtual-sensor and UVC ID policies follow the archived libcamera
 * 0.7.2 sysfs.cpp, CameraSensorLegacy::generateId() and
 * UVCCameraData::generateId(), Copyright (C) 2019-2020 Google Inc. */
#include "internal.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>

#define ID_PATH_MAX 4096U

bool cc_identity_unavailable(int error)
{
    return error==-ENOENT || error==-ENODEV || error==-ENOTSUP || error==-EACCES ||
           error==-EPERM || error==-ENOTDIR;
}

static int join(char *out, size_t capacity, const char *base, const char *suffix)
{
    int n=snprintf(out,capacity,"%s/%s",base,suffix);
    return n<0 ? -EIO:(size_t)n>=capacity ? -ENAMETOOLONG:0;
}

static int checked_text(const char *value, size_t capacity)
{
    const char *end=memchr(value,'\0',capacity);
    if (!end) return -EPROTO;
    for (const unsigned char *p=(const unsigned char *)value; p<(const unsigned char *)end; ++p)
        if (*p<32 || *p==127) return -EPROTO;
    return value[0] ? 0:-ENOENT;
}

static int canonical(const struct cc_io *io, const char *path, char *out, size_t capacity)
{
    int ret=io->ops->canonical(io->ctx,path,out,capacity);
    if (ret) return ret<0 ? ret:-EPROTO;
    ret=checked_text(out,capacity);
    if (ret<0) return ret;
    if (out[0]!='/' || strstr(out,"//") || strstr(out,"/../") || strstr(out,"/./")) return -EPROTO;
    size_t n=strlen(out);
    if ((n>1 && out[n-1]=='/') || (n>=2 && !strcmp(out+n-2,"/.")) ||
        (n>=3 && !strcmp(out+n-3,"/.."))) return -EPROTO;
    return 0;
}

static int read_line(const struct cc_io *io, const char *path, char *out, size_t capacity)
{
    int ret=io->ops->read_text(io->ctx,path,out,capacity);
    if (ret) return ret<0 ? ret:-EPROTO;
    if (!memchr(out,'\0',capacity)) return -EPROTO;
    out[strcspn(out,"\n")]='\0';
    return checked_text(out,capacity);
}

static int firmware_path(const struct cc_io *io, const char *device, char *out, size_t capacity)
{
    char path[ID_PATH_MAX],resolved[ID_PATH_MAX];
    int ret=join(path,sizeof(path),device,"of_node");
    if (ret<0) return ret;
    ret=canonical(io,path,resolved,sizeof(resolved));
    if (!ret) {
        static const char prefix[]="/sys/firmware/devicetree";
        const char *node=resolved;
        size_t prefix_size=sizeof(prefix)-1;
        if (!strncmp(node,prefix,prefix_size) && (node[prefix_size]=='/' || !node[prefix_size])) node+=prefix_size;
        size_t length=strlen(node);
        if (!length) return -ENOENT;
        if (length>=capacity) return -ENAMETOOLONG;
        memcpy(out,node,length+1);
        return 0;
    }
    if (!cc_identity_unavailable(ret)) return ret;
    ret=join(path,sizeof(path),device,"firmware_node/path");
    return ret<0 ? ret:read_line(io,path,out,capacity);
}

static bool decimal(const char **at)
{
    const char *start=*at;
    while (**at>='0' && **at<='9') ++*at;
    return *at!=start;
}

static int usb_identity(const struct cc_io *io, const char *device, char *id, size_t capacity)
{
    const char *name=strrchr(device,'/');
    if (!name || !name[1]) return -EPROTO;
    const char *at=name+1;
    if (!decimal(&at) || *at++!='-') return -EPROTO;
    const char *ports=at;
    if (!decimal(&at)) return -EPROTO;
    while (*at=='.') { ++at; if (!decimal(&at)) return -EPROTO; }
    if (*at++!=':' || !decimal(&at) || *at++!='.' || !decimal(&at) || *at) return -EPROTO;

    char parent[ID_PATH_MAX],controller[ID_PATH_MAX];
    memcpy(parent,device,strlen(device)+1);
    for (;;) {
        char *slash=strrchr(parent,'/');
        if (!slash || slash<=parent+1) return -ENOTSUP;
        *slash='\0';
        int ret=firmware_path(io,parent,controller,sizeof(controller));
        if (!ret) break;
        if (!cc_identity_unavailable(ret)) return ret;
    }
    char path[ID_PATH_MAX],vendor[16],product[16];
    const char *attributes[]={"../idVendor","../idProduct"};
    char *values[]={vendor,product};
    for (size_t i=0; i<2; ++i) {
        int ret=join(path,sizeof(path),device,attributes[i]);
        if (ret<0) return ret;
        ret=read_line(io,path,values[i],sizeof(vendor));
        if (ret<0) return ret;
        if (strlen(values[i])!=4) return -EPROTO;
        for (size_t c=0; c<4; ++c) {
            unsigned char v=(unsigned char)values[i][c];
            if (!((v>='0' && v<='9') || (v>='a' && v<='f') || (v>='A' && v<='F'))) return -EPROTO;
        }
    }
    int n=snprintf(id,capacity,"%s-%s-%s:%s",controller,ports,vendor,product);
    return n<0 ? -EIO:(size_t)n>=capacity ? -ENAMETOOLONG:0;
}

int cc_camera_identify_io(const struct cc_io *io, int fd, unsigned int major_id, unsigned int minor_id,
                           enum cc_pipeline pipeline, const char *model, cc_camera_info *out)
{
    if (!io || !io->ops || !out) return -EINVAL;
    if ((unsigned)pipeline>CC_PIPELINE_UVC) return -EINVAL;
    if (!io->ops->canonical || !io->ops->read_text) return -ENOTSUP;
    if (fd>=0) {
        if (!io->ops->device_number) return -ENOTSUP;
        int ret=io->ops->device_number(io->ctx,fd,&major_id,&minor_id);
        if (ret) return ret<0 ? ret:-EPROTO;
    }
    if (!major_id) return -ENOTSUP;
    char device[ID_PATH_MAX],node[128];
    int n=snprintf(node,sizeof(node),"/sys/dev/char/%u:%u/device",major_id,minor_id);
    if (n<0 || (size_t)n>=sizeof(node)) return -ENAMETOOLONG;
    int ret=canonical(io,node,device,sizeof(device));
    if (ret<0) return ret;
    static const char devices[]="/sys/devices/",platform[]="/sys/devices/platform/";
    if (strncmp(device,devices,sizeof(devices)-1)) return -EPROTO;
    char id[CC_CAMERA_ID_MAX],firmware[ID_PATH_MAX];
    enum cc_camera_id_type type;
    if (pipeline==CC_PIPELINE_UVC) {
        ret=usb_identity(io,device,id,sizeof(id)); type=CC_CAMERA_ID_USB;
    } else {
        ret=firmware_path(io,device,firmware,sizeof(firmware));
        if (pipeline==CC_PIPELINE_V4L2) {
            const char *base=firmware;
            if (ret<0) {
                if (!cc_identity_unavailable(ret)) return ret;
                if (strncmp(device,platform,sizeof(platform)-1)) return -ENOTSUP;
                base=device+sizeof(devices)-1;
            }
            char index[32];
            n=snprintf(node,sizeof(node),"/sys/dev/char/%u:%u/index",major_id,minor_id);
            if (n<0 || (size_t)n>=sizeof(node)) return -ENAMETOOLONG;
            ret=read_line(io,node,index,sizeof(index));
            if (ret<0) return ret;
            uint32_t value=0;
            for (size_t i=0; index[i]; ++i) {
                if (index[i]<'0' || index[i]>'9') return -EPROTO;
                uint32_t digit=(uint32_t)(index[i]-'0');
                if (value>(UINT32_MAX-digit)/10) return -EPROTO;
                value=value*10+digit;
            }
            n=snprintf(id,sizeof(id),"%s-video%u",base,value);
            ret=n<0 ? -EIO:(size_t)n>=sizeof(id) ? -ENAMETOOLONG:0;
            type=CC_CAMERA_ID_VIDEO;
        } else if (!ret) {
            size_t length=strlen(firmware);
            if (length>=sizeof(id)) return -ENAMETOOLONG;
            memcpy(id,firmware,length+1); type=CC_CAMERA_ID_FIRMWARE;
        } else {
            if (!cc_identity_unavailable(ret)) return ret;
            if (strncmp(device,platform,sizeof(platform)-1) || !model || !model[0]) return -ENOTSUP;
            size_t length=cc_camera_model_length(model,128,true);
            n=snprintf(id,sizeof(id),"%s %.*s",device+sizeof(devices)-1,(int)length,model);
            ret=n<0 ? -EIO:(size_t)n>=sizeof(id) ? -ENAMETOOLONG:0;
            type=CC_CAMERA_ID_PLATFORM;
        }
    }
    if (ret<0) return ret;
    memcpy(out->id,id,strlen(id)+1); out->id_type=type;
    return 0;
}
