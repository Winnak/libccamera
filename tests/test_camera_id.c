/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <ccamera/capture.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
#define OK(x) CHECK((x)==0)
struct id_file { const char *path,*value; };
struct id_fixture {
    const char *name,*device,*model;
    enum cc_pipeline pipeline;
    enum cc_camera_id_type type;
    const char *expected;
    const struct id_file *links,*files;
};
#include "camera_id_vectors.h"
struct fake {
    const struct id_fixture *fixture;
    const struct id_file *links,*files;
    const char *device,*index,*fail_path;
    unsigned int major_id,minor_id;
    int error,device_error;
    unsigned bad_canonical,bad_text,canonical_calls,text_calls;
    unsigned scan_mode;
};

static int copy(char *out,size_t capacity,const char *value)
{
    if (strlen(value)>=capacity) return -EOVERFLOW;
    memcpy(out,value,strlen(value)+1); return 0;
}
static int device_number(void *context,int fd,unsigned int *major_id,unsigned int *minor_id)
{
    struct fake *f=context; CHECK(fd==20);
    if (f->device_error) return -f->device_error;
    *major_id=f->major_id; *minor_id=f->minor_id; return 0;
}
static int canonical(void *context,const char *path,char *out,size_t capacity)
{
    struct fake *f=context; ++f->canonical_calls;
    if (f->fail_path && !strcmp(path,f->fail_path)) return -f->error;
    if (f->bad_canonical==1) { memset(out,'a',capacity); return 0; }
    if (f->bad_canonical==2) return copy(out,capacity,"relative/path");
    if (f->bad_canonical==3) return copy(out,capacity,"/sys/devices/a/../b");
    if (f->bad_canonical==4) return copy(out,capacity,"/sys/devices/a//b");
    if (f->bad_canonical==5) return copy(out,capacity,"/sys/devices/a\nb");
    if (f->bad_canonical==6) return copy(out,capacity,"/sys/devices/a/.");
    if (f->bad_canonical==7) return copy(out,capacity,"/unrelated/device");
    char node[128]; snprintf(node,sizeof(node),"/sys/dev/char/%u:%u/device",f->major_id,f->minor_id);
    if (!strcmp(node,path)) return copy(out,capacity,f->device);
    for (const struct id_file *file=f->links; file->path; ++file)
        if (!strcmp(file->path,path)) return copy(out,capacity,file->value);
    return -ENOENT;
}
static int read_text(void *context,const char *path,char *out,size_t capacity)
{
    struct fake *f=context; ++f->text_calls;
    if (f->fail_path && !strcmp(path,f->fail_path)) return -f->error;
    if (f->bad_text) { memset(out,'a',capacity); return 0; }
    char node[128]; snprintf(node,sizeof(node),"/sys/dev/char/%u:%u/index",f->major_id,f->minor_id);
    if (!strcmp(node,path) && f->index) return copy(out,capacity,f->index);
    for (const struct id_file *file=f->files; file->path; ++file)
        if (!strcmp(file->path,path)) return copy(out,capacity,file->value);
    return -ENOENT;
}
static const struct cc_io_ops ops={.device_number=device_number,.canonical=canonical,.read_text=read_text};
static struct fake initialize(size_t i)
{
    return (struct fake){.fixture=&id_fixtures[i],.device=id_fixtures[i].device,
        .links=id_fixtures[i].links,.files=id_fixtures[i].files,.major_id=81,.minor_id=17,.error=EIO};
}
static void expect_error(struct fake *f,enum cc_pipeline pipeline,int expected)
{
    struct cc_io io={&ops,f};
    cc_camera_info out; memset(&out,0xa5,sizeof(out)); cc_camera_info saved=out;
    CHECK(cc_camera_identify_io(&io,20,0,0,pipeline,f->fixture->model,&out)==-expected);
    CHECK(!memcmp(&out,&saved,sizeof(out)));
}

static void test_reference(void)
{
    for (size_t i=0; i<sizeof(id_fixtures)/sizeof(id_fixtures[0]); ++i) for (unsigned fd=0; fd<2; ++fd) {
        struct fake f=initialize(i); struct cc_io io={&ops,&f};
        cc_camera_info out; memset(&out,0xa5,sizeof(out)); cc_camera_info saved=out;
        int ret=cc_camera_identify_io(&io,fd ? 20:-1,f.major_id,f.minor_id,f.fixture->pipeline,f.fixture->model,&out);
        if (!f.fixture->type) {
            CHECK(cc_identity_unavailable(ret) && !memcmp(&out,&saved,sizeof(out)));
        } else {
            CHECK(!ret && out.id_type==f.fixture->type && !strcmp(out.id,f.fixture->expected));
            CHECK(!memcmp(&out,&saved,offsetof(cc_camera_info,id_type)));
        }
    }
    CHECK(!strcmp(id_fixtures[0].expected,id_fixtures[1].expected));
    CHECK(!strcmp(id_fixtures[9].expected,id_fixtures[10].expected));
    CHECK(strcmp(id_fixtures[9].expected,id_fixtures[11].expected));
    CHECK(strcmp(id_fixtures[9].expected,id_fixtures[12].expected));
    /* Character major/minor numbers do not form part of the identity. */
    struct fake f=initialize(0); f.major_id=240; f.minor_id=UINT32_MAX;
    struct cc_io io={&ops,&f}; cc_camera_info out={0};
    OK(cc_camera_identify_io(&io,20,0,0,CC_PIPELINE_RPI_CFE,f.fixture->model,&out));
    CHECK(!strcmp(out.id,id_fixtures[0].expected));
}

static void test_video_identity(void)
{
    struct fake f=initialize(0); struct cc_io io={&ops,&f}; cc_camera_info out={0};
    f.index="001\n";
    OK(cc_camera_identify_io(&io,20,0,0,CC_PIPELINE_V4L2,NULL,&out));
    CHECK(out.id_type==CC_CAMERA_ID_VIDEO && !strcmp(out.id,"/base/axi/i2c@1000/imx219@10-video1"));
    f=initialize(4); f.index="4294967295\n";
    OK(cc_camera_identify_io(&io,20,0,0,CC_PIPELINE_V4L2,NULL,&out));
    CHECK(out.id_type==CC_CAMERA_ID_VIDEO && !strcmp(out.id,"platform/vimc.0-video4294967295"));
    const char *invalid[]={"-1"," 1","1x","4294967296"};
    for (size_t i=0; i<sizeof(invalid)/sizeof(invalid[0]); ++i) { f.index=invalid[i]; expect_error(&f,CC_PIPELINE_V4L2,EPROTO); }
    f.index=NULL; expect_error(&f,CC_PIPELINE_V4L2,ENOENT);
}

static void test_failures_and_bounds(void)
{
    struct fake f=initialize(0); struct cc_io io={&ops,&f}; cc_camera_info out;
    CHECK(cc_camera_identify_io(NULL,20,0,0,CC_PIPELINE_RPI_UNICAM,NULL,&out)==-EINVAL);
    CHECK(cc_camera_identify_io(&io,20,0,0,CC_PIPELINE_RPI_UNICAM,NULL,NULL)==-EINVAL);
    CHECK(cc_camera_identify_io(&io,20,0,0,(enum cc_pipeline)-1,NULL,&out)==-EINVAL);
    f.device_error=ENODEV; expect_error(&f,CC_PIPELINE_RPI_UNICAM,ENODEV); f.device_error=0;
    for (f.bad_canonical=1; f.bad_canonical<=7; ++f.bad_canonical) expect_error(&f,CC_PIPELINE_RPI_UNICAM,EPROTO);
    f.bad_canonical=0;
    f.fail_path="/sys/dev/char/81:17/device"; expect_error(&f,CC_PIPELINE_RPI_UNICAM,EIO);
    f.fail_path=f.links[0].path; expect_error(&f,CC_PIPELINE_RPI_UNICAM,EIO);
    f=initialize(2); f.bad_text=1; expect_error(&f,CC_PIPELINE_RPI_UNICAM,EPROTO);
    f=initialize(9);
    const char *bad_usb[]={"3","3-","x-2:1.0","3-2..4:1.0","3-2:1","3-2:1.x","3-2:1.0-extra"};
    char path[512];
    for (size_t i=0; i<sizeof(bad_usb)/sizeof(bad_usb[0]); ++i) {
        snprintf(path,sizeof(path),"/sys/devices/platform/usb/usb3/%s",bad_usb[i]); f.device=path;
        expect_error(&f,CC_PIPELINE_UVC,EPROTO);
    }
    f=initialize(9);
    struct id_file files[]={{f.files[0].path,"046z\n"},{f.files[1].path,"0825\n"},{NULL,NULL}};
    f.files=files; expect_error(&f,CC_PIPELINE_UVC,EPROTO);
    files[0].value="046dd\n"; expect_error(&f,CC_PIPELINE_UVC,EPROTO);
    files[0].value="046d\n"; files[1].value="08\n"; expect_error(&f,CC_PIPELINE_UVC,EPROTO);
    f=initialize(0);
    char large[2048]; const char *prefix="/sys/firmware/devicetree";
    size_t base=strlen(prefix); memcpy(large,prefix,base);
    struct id_file links[]={{f.links[0].path,large},{NULL,NULL}}; f.links=links;
    for (size_t n=0; n<CC_CAMERA_ID_MAX; ++n) large[base+n]=n%16 ? 'a':'/';
    large[base+CC_CAMERA_ID_MAX-1]='\0';
    OK(cc_camera_identify_io(&io,20,0,0,CC_PIPELINE_RPI_UNICAM,f.fixture->model,&out));
    CHECK(strlen(out.id)==CC_CAMERA_ID_MAX-1);
    large[base+CC_CAMERA_ID_MAX-1]='a'; large[base+CC_CAMERA_ID_MAX]='\0';
    expect_error(&f,CC_PIPELINE_RPI_UNICAM,ENAMETOOLONG);
    struct cc_io_ops missing=ops; missing.canonical=NULL; io.ops=&missing;
    CHECK(cc_camera_identify_io(&io,20,0,0,CC_PIPELINE_RPI_UNICAM,NULL,&out)==-ENOTSUP);
}

static void test_native_helpers(void)
{
    char path[]="/tmp/ccamera-id-XXXXXX";
    int fd=mkstemp(path); CHECK(fd>=0);
    const char content[]="firmware-path\n";
    CHECK(write(fd,content,sizeof(content)-1)==(ssize_t)(sizeof(content)-1));
    char out[128],saved[128]; memset(out,0xa5,sizeof(out)); memcpy(saved,out,sizeof(out));
    OK(cc_system_io.ops->read_text(NULL,path,out,sizeof(out))); CHECK(!strcmp(out,content));
    memcpy(out,saved,sizeof(out));
    CHECK(cc_system_io.ops->read_text(NULL,path,out,4)==-EOVERFLOW && !memcmp(out,saved,sizeof(out)));
    CHECK(pwrite(fd,"\0",1,0)==1);
    CHECK(cc_system_io.ops->read_text(NULL,path,out,sizeof(out))==-EPROTO && !memcmp(out,saved,sizeof(out)));
    unsigned int major_id,minor_id;
    CHECK(cc_system_io.ops->device_number(NULL,fd,&major_id,&minor_id)==-ENOTSUP);
    OK(cc_system_io.ops->canonical(NULL,"/",out,sizeof(out))); CHECK(!strcmp(out,"/"));
    memcpy(out,saved,sizeof(out));
    CHECK(cc_system_io.ops->canonical(NULL,"/",out,1)==-ENAMETOOLONG && !memcmp(out,saved,sizeof(out)));
    CHECK(close(fd)==0 && unlink(path)==0);
    CHECK(cc_system_io.ops->canonical(NULL,path,out,sizeof(out))==-ENOENT && !memcmp(out,saved,sizeof(out)));
}

static int bad_scan(void *context,char ***out,size_t *count)
{
    struct fake *f=context;
    if (f->scan_mode==6) { *out=NULL; *count=0; return 0; }
    char **nodes=calloc(2,sizeof(*nodes)); CHECK(nodes);
    *out=nodes; *count=f->scan_mode==1 ? 2:1;
    if (f->scan_mode==0) nodes[0]=strdup("/tmp/video0");
    else if (f->scan_mode==1) { nodes[0]=strdup("/dev/video0"); nodes[1]=strdup("/dev/video0"); }
    else if (f->scan_mode==2) { nodes[0]=malloc(CC_PATH_MAX); CHECK(nodes[0]); memset(nodes[0],'a',CC_PATH_MAX); }
    else if (f->scan_mode!=3) nodes[0]=strdup("/dev/video0");
    return f->scan_mode==4 ? -EIO:f->scan_mode==5 ? 1:0;
}

static void test_scan_and_api_errors(void)
{
    struct fake f=initialize(0);
    struct cc_io_ops operations=ops; operations.scan_nodes=bad_scan;
    struct cc_io io={&operations,&f};
    for (f.scan_mode=0; f.scan_mode<7; ++f.scan_mode) {
        cc_camera_info *list=(cc_camera_info *)(uintptr_t)1; size_t count=10;
        int ret=cc_enumerate_io(&io,&list,&count);
        CHECK(ret==(f.scan_mode==6 ? 0:f.scan_mode==4 ? -EIO:-EPROTO));
        CHECK(!list && !count);
    }
    f.scan_mode=6;
    cc_camera *camera=(cc_camera *)(uintptr_t)1;
    cc_cfe *fe=(cc_cfe *)(uintptr_t)1;
    CHECK(cc_camera_open_id_io("missing",&io,&camera)==-ENOENT && !camera);
    CHECK(cc_cfe_open_id_io("missing",&io,&fe)==-ENOENT && !fe);
    CHECK(cc_camera_open_by_id(NULL,&camera)==-EINVAL && !camera);
    CHECK(cc_camera_open_by_id("",&camera)==-EINVAL && !camera);
    CHECK(cc_camera_open_by_id("id",NULL)==-EINVAL);
    CHECK(cc_cfe_open_by_id(NULL,&fe)==-EINVAL && !fe);
    CHECK(cc_cfe_open_by_id("",&fe)==-EINVAL && !fe);
    CHECK(cc_cfe_open_by_id("id",NULL)==-EINVAL);
    char invalid[CC_CAMERA_ID_MAX]; memset(invalid,'x',sizeof(invalid));
    CHECK(cc_camera_open_by_id(invalid,&camera)==-EINVAL && !camera);
    cc_camera_info info={.video_path="/dev/video0"}; memset(info.id,'x',sizeof(info.id));
    CHECK(cc_camera_open_io(&info,&io,&camera)==-EINVAL && !camera);
    info.id[0]='\0'; info.id_type=(enum cc_camera_id_type)999;
    CHECK(cc_camera_open_io(&info,&io,&camera)==-EINVAL && !camera);
}

int main(void)
{
    test_reference(); test_video_identity(); test_failures_and_bounds(); test_native_helpers(); test_scan_and_api_errors();
    puts("Camera IDs: archived DT/ACPI/platform/UVC fixtures, numbering stability, bounds and native sysfs helpers passed");
    return 0;
}
