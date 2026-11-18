/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa_proxy.h>
#include <ccamera/pisp_tuning.h>
#include <ccamera/cfe.h>
#include <errno.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pisp_tuning_vectors.h"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
#define OK(x) CHECK((x)==0)
static const char *worker;
static int create(const cc_ipa_pisp_options *o, cc_ipa **ipa)
{
    cc_ipa_proxy_options proxy={.worker_path=worker};
    return worker ? cc_ipa_pisp_create_proxy(o,&proxy,ipa) : cc_ipa_pisp_create(o,ipa);
}
static cc_sensor_geometry geometry(void)
{
    return (cc_sensor_geometry){.active={16,8,4096,3072},.crop={16,8,4096,3072},.width=1024,.height=768};
}
static cc_format input(void)
{
    return (cc_format){.fourcc=V4L2_PIX_FMT_SRGGB16,.width=1024,.height=768,.num_planes=1,
        .planes={{2048,2048*768}}};
}
static cc_format output(uint32_t fourcc)
{
    return (cc_format){.fourcc=fourcc,.width=512,.height=384,.num_planes=1,
        .planes={{1536,1536*384}},.colorspace=V4L2_COLORSPACE_JPEG,
        .xfer_func=V4L2_XFER_FUNC_SRGB,.quantization=V4L2_QUANTIZATION_FULL_RANGE};
}
static double unpack(uint32_t packed,unsigned channel)
{
    unsigned range=packed>>30, code=(packed>>(10*channel))&1023U;
    return range ? (double)code/(1024U>>range) : (double)(code+512)/1024;
}
static void vectors(void)
{
    cc_sensor_geometry g=geometry();
    for (size_t n=0;n<sizeof(pisp_vectors)/sizeof(pisp_vectors[0]);++n) {
        const struct pisp_vector *v=&pisp_vectors[n];
        g.crop=v->crop ? (cc_rectangle){1040,776,2048,1536} : g.active;
        const cc_rpi_profile_info *info=cc_pisp_get_profile_info(v->profile);
        const cc_vc4_profile_info *vc4 = cc_vc4_get_profile_info(v->profile);
        CHECK(info && strstr(info->calibration_id,"-pisp-") &&
            (!vc4 || strcmp(info->calibration_id,vc4->calibration_id)));
        struct pisp_fe_config f; struct pisp_be_config b;
        OK(cc_pisp_build_tuning(v->profile,&g,v->ct,v->gain,1.5,2,&f,&b));
        CHECK(b.blc.black_level_r==v->black && f.blc.black_level_b==v->black &&
            b.sdn.black_level==v->black && !f.blc.output_black_level);
        CHECK(b.geq.offset==v->geq && b.geq.slope_sharper==v->slope && b.sdn.noise_slope==v->sdn &&
            b.cdn.thresh==v->cdn && b.sdn.leakage==13 && b.cdn.iir_strength==77);
        for (unsigned i=0;i<9;++i) CHECK(abs(b.ccm.coeffs[i]-v->ccm[i])<=1);
        for (unsigned i=0;i<64;++i) CHECK(b.gamma.lut[i]==v->gamma[i]);
        for (unsigned y=0;y<3;++y) for (unsigned x=0;x<3;++x) for (unsigned ch=0;ch<3;++ch)
            CHECK(fabs(unpack(b.lsc.lut_packed[y*16][x*16],ch)-unpack(v->shading[y*3+x],ch))<0.01);
        CHECK(b.wbg.gain_r==1536 && b.wbg.gain_g==1024 && b.wbg.gain_b==2048);
        CHECK(b.sharpen.kernel0[12]==6 && b.sharpen.positive_strength==38 &&
            b.sharpen.threshold_offset0==38 && b.sharpen.positive_limit==1000);
        bool cac = (info->flags & CC_VC4_PROFILE_LENS_CAC) != 0;
        CHECK(!!(b.global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC) == cac);
        for (unsigned y=0;y<16;++y) CHECK(!(f.agc_stats.weights[y*8+7]&0xf0));
        for (unsigned x=0;x<8;++x) CHECK(!f.agc_stats.weights[120+x]);
        struct pisp_fe_config finalized;
        cc_cfe_configuration c={.sensor={.fourcc=V4L2_PIX_FMT_SRGGB10P,.width=1024,.height=768},.image=input()};
        OK(cc_cfe_build_parameters(&c,&f,&finalized));
        CHECK(finalized.global.enables&PISP_FE_ENABLE_BLC);
        cc_pisp_be_configuration config={.input=input(),.output=output(V4L2_PIX_FMT_RGB24),
            .output1=output(V4L2_PIX_FMT_BGR24)};
        struct pisp_be_tiles_config native;
        for (unsigned rev=0;rev<2;++rev) {
            config.hardware_version=rev ? CC_PISP_BE_VERSION_BCM2712_D0 : CC_PISP_BE_VERSION_BCM2712_C0;
            OK(cc_pisp_be_build_configuration(&config,&b,&native));
            CHECK(native.num_tiles && native.config.global.rgb_enables&PISP_BE_RGB_ENABLE_GAMMA);
            CHECK(native.config.ccm.coeffs[0]==b.ccm.coeffs[0] &&
                native.config.lsc.lut_packed[0][0]==b.lsc.lut_packed[0][0]);
            CHECK(!!(native.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC) == cac);
            CHECK(!memcmp(&native.config.cac, &b.cac, sizeof(b.cac)));
        }
        struct pisp_fe_config fs=f; struct pisp_be_config bs=b;
        CHECK(cc_pisp_build_tuning(v->profile,&g,NAN,1,1,1,&f,&b)==-EINVAL);
        CHECK(!memcmp(&f,&fs,sizeof(f)) && !memcmp(&b,&bs,sizeof(b)));
        CHECK(cc_pisp_build_tuning(CC_VC4_PROFILE_AUTO,&g,4500,1,1,1,&f,&b)==-EINVAL);
        g.hflip=g.vflip=1;
        struct pisp_fe_config flip_f; struct pisp_be_config flip_b;
        OK(cc_pisp_build_tuning(v->profile,&g,v->ct,v->gain,1.5,2,&flip_f,&flip_b));
        for (unsigned y=0;y<33;++y) for (unsigned x=0;x<33;++x) for (unsigned ch=0;ch<3;++ch)
            CHECK(fabs(unpack(b.lsc.lut_packed[y][x],ch)-unpack(flip_b.lsc.lut_packed[32-y][32-x],ch))<0.02);
        g.hflip=g.vflip=0;
    }
}
static cc_control_info range(uint32_t id,int64_t low,int64_t high)
{
    return (cc_control_info){.id=id,.type=V4L2_CTRL_TYPE_INTEGER,.minimum=low,.maximum=high,.step=1,.elems=1};
}
static cc_ipa_configuration configuration(cc_rpi_profile profile)
{
    const cc_rpi_profile_info *p=cc_pisp_get_profile_info(profile);
    cc_ipa_configuration c={.pipeline=CC_IPA_PISP,.statistics_fourcc=CC_FOURCC('R','P','F','S'),
        .source=input(),.sensor_format={.fourcc=V4L2_PIX_FMT_SRGGB10P,.width=1024,.height=768},
        .geometry=geometry(),.timing={100000000,1024,768,976,232},.num_sensor_controls=3,
        .timing_limits={100000000,1024,768,976,976,1,32,16000,1,4,1}};
    OK(cc_sensor_lookup(p->sensor_model,&c.sensor));
    c.sensor_controls[0]=range(V4L2_CID_EXPOSURE,4,996);
    c.sensor_controls[1]=range(V4L2_CID_ANALOGUE_GAIN,!strcmp(p->sensor_model,"ov5647") ? 16:0,
        !strcmp(p->sensor_model,"ov5647") ? 1023 : !strcmp(p->sensor_model,"imx219") ? 232:960);
    c.sensor_controls[2]=range(V4L2_CID_VBLANK,32,16000);
    c.initial_controls[0]=(cc_control){.id=V4L2_CID_EXPOSURE,.value=500};
    c.initial_controls[1]=(cc_control){.id=V4L2_CID_ANALOGUE_GAIN,.value=c.sensor_controls[1].minimum};
    c.initial_controls[2]=(cc_control){.id=V4L2_CID_VBLANK,.value=232};
    for (unsigned i=0;i<3;++i) c.sensor_delays[i]=(cc_sensor_control_delay){c.initial_controls[i].id,2,
        i==2 ? CC_SENSOR_CONTROL_PRIORITY:0};
    return c;
}
static void controllers(void)
{
    for (unsigned id=CC_VC4_PROFILE_OV5647;id<=CC_VC4_PROFILE_IMX477_16MM;++id) {
        cc_rpi_profile profile=(cc_rpi_profile)id; cc_ipa_configuration c=configuration(profile);
        cc_ipa_pisp_options options={.mains_hz=50,.profile=profile,.red_gain=1,.blue_gain=1};
        cc_ipa *ipa; OK(create(&options,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
        cc_ipa_request request={.timing=c.timing,.timing_limits=c.timing_limits};
        unsigned stable=0;
        for (unsigned iteration=0;iteration<160;++iteration) {
            cc_ipa_settings s; OK(cc_ipa_prepare(ipa,&request,&s));
            CHECK(s.flags==(CC_IPA_FRONTEND|CC_IPA_BACKEND) && !s.num_isp_controls && s.colour_temperature>=2500);
            CHECK(!!(s.backend.global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC) ==
                  !!(cc_pisp_get_profile_info(profile)->flags & CC_VC4_PROFILE_LENS_CAC));
            double gain; uint64_t ns;
            OK(cc_sensor_gain(c.sensor,(uint32_t)s.sensor_controls[2].value,&gain));
            OK(cc_sensor_exposure_time(c.sensor,&request.timing,(uint32_t)s.sensor_controls[1].value,&ns));
            double level=(double)ns*gain/(iteration<80 ? 40000000:160000000);
            if (level>0.99) level=0.99;
            struct pisp_statistics stats={0};
            stats.agc.histogram[(unsigned)(level*1024)]=1000;
            for (unsigned i=0;i<1024;++i) stats.awb.zones[i]=(struct pisp_awb_statistics_zone){
                .R_sum=(uint32_t)(level*65536*100),.G_sum=(uint32_t)(level*65536*100),
                .B_sum=(uint32_t)(level*65536*100),.counted=100};
            cc_ipa_frame frame={.statistics_fourcc=c.statistics_fourcc,.statistics_data=&stats,
                .statistics_size=sizeof(stats),.backend=&s.backend,.frontend=&s.frontend,.colour_temperature=s.colour_temperature,
                .settings_sensor_controls=s.sensor_controls,.num_settings_sensor_controls=3};
            frame.sensor.count=3; memcpy(frame.sensor.controls,s.sensor_controls,sizeof(cc_control)*3);
            cc_ipa_metadata m; OK(cc_ipa_process(ipa,&frame,&m));
            CHECK(m.flags&CC_IPA_METADATA_PISP && m.analogue_gain==gain && m.colour_temperature==s.colour_temperature);
            if (iteration==80) stable=0;
            stable=m.flags&CC_IPA_AE_CONVERGED ? stable+1:0;
            if (iteration==79 || iteration==159) CHECK(stable>=3);
            request.timing.vblank=(uint32_t)s.sensor_controls[0].value;
            frame.untrusted=1; OK(cc_ipa_process(ipa,&frame,&m)); CHECK((m.flags & CC_IPA_METADATA_PHOTO) && !(m.photo_status & CC_PHOTO_STATS_TRUSTED));
        }
        cc_ipa_stop(ipa); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa)); cc_ipa_stop(ipa);
        c.source.width/=2; CHECK(cc_ipa_configure(ipa,&c)==-ENOTSUP);
        c=configuration(profile); c.source.fourcc=V4L2_PIX_FMT_Y16; CHECK(cc_ipa_configure(ipa,&c)==-ENOTSUP);
        c=configuration(profile); c.pipeline=CC_IPA_CFE; CHECK(cc_ipa_configure(ipa,&c)==-ENOTSUP);
        cc_ipa_destroy(ipa);
    }
}
static double shading(const struct pisp_be_config *b,double x,double y,unsigned ch)
{
    double u=x*b->lsc.grid_step_x/(1U<<18);
    double v=y*b->lsc.grid_step_y/(1U<<18);
    unsigned a=(unsigned)u,c=a<32 ? a+1:a, d=(unsigned)v,e=d<32 ? d+1:d;
    u-=a;v-=d;
    return (unpack(b->lsc.lut_packed[d][a],ch)*(1-u)+unpack(b->lsc.lut_packed[d][c],ch)*u)*(1-v)+
           (unpack(b->lsc.lut_packed[e][a],ch)*(1-u)+unpack(b->lsc.lut_packed[e][c],ch)*u)*v;
}
static void white_balance(void)
{
    for (unsigned id=CC_VC4_PROFILE_OV5647;id<=CC_VC4_PROFILE_IMX477_16MM;++id) {
        const struct pisp_vector *v=NULL;
        for (size_t i=0;i<sizeof(pisp_vectors)/sizeof(pisp_vectors[0]);++i)
            if ((unsigned)pisp_vectors[i].profile==id) { v=&pisp_vectors[i];break; }
        CHECK(v);
        cc_ipa_configuration c=configuration((cc_rpi_profile)id);
        cc_ipa_pisp_options o={.profile=(cc_rpi_profile)id}; cc_ipa *ipa;
        OK(create(&o,&ipa)); OK(cc_ipa_configure(ipa,&c)); OK(cc_ipa_start(ipa));
        struct pisp_fe_config f; struct pisp_be_config scene;
        bool noir=(cc_pisp_get_profile_info(o.profile)->flags&CC_VC4_PROFILE_NOIR)!=0;
        OK(cc_pisp_build_tuning(o.profile,&c.geometry,noir ? 4500:6500,1,1,1,&f,&scene));
        struct pisp_statistics stats={0}; stats.agc.histogram[512]=1000;
        for (unsigned i=0;i<1024;++i) {
            double x=72+((double)(i%32)+0.5)*26, y=40+((double)(i/32)+0.5)*20;
            stats.awb.zones[i]=(struct pisp_awb_statistics_zone){.counted=100,
                .R_sum=(uint32_t)(400000*v->raw_red/shading(&scene,x,y,0)),
                .G_sum=(uint32_t)(400000/shading(&scene,x,y,1)),
                .B_sum=(uint32_t)(400000*v->raw_blue/shading(&scene,x,y,2))};
        }
        cc_ipa_request request={.timing=c.timing,.timing_limits=c.timing_limits};
        cc_ipa_metadata m={0};
        for (unsigned iteration=0;iteration<20;++iteration) {
            cc_ipa_settings s;OK(cc_ipa_prepare(ipa,&request,&s));
            /* Statistics-only ROI leaves the delivered image full-size. */
            s.frontend.global.enables|=PISP_FE_ENABLE_STATS_CROP;
            s.frontend.stats_crop=(struct pisp_fe_crop_config){.offset_x=64,.offset_y=32,.width=896,.height=704};
            s.frontend.awb_stats.offset_x=s.frontend.awb_stats.offset_y=8;
            s.frontend.awb_stats.size_x=26;s.frontend.awb_stats.size_y=20;
            cc_ipa_frame frame={.backend=&s.backend,.frontend=&s.frontend,.colour_temperature=s.colour_temperature,
                .statistics_fourcc=c.statistics_fourcc,.statistics_data=&stats,.statistics_size=sizeof(stats)};
            frame.sensor.count=3;memcpy(frame.sensor.controls,s.sensor_controls,sizeof(cc_control)*3);
            OK(cc_ipa_process(ipa,&frame,&m));
            CHECK(m.flags&CC_IPA_METADATA_AWB);
            if (iteration==19) {
                cc_ipa_metadata saved=m;
                frame.statistics_size=sizeof(stats)-1;
                CHECK(cc_ipa_process(ipa,&frame,&m)==-EMSGSIZE && !memcmp(&m,&saved,sizeof(m)));
                frame.statistics_size=sizeof(stats);frame.frontend=NULL;
                CHECK(cc_ipa_process(ipa,&frame,&m)==-ENODATA && !memcmp(&m,&saved,sizeof(m)));
            }
            request.timing.vblank=(uint32_t)s.sensor_controls[0].value;
        }
        double tolerance=noir ? 0.01:0.15;
        CHECK(fabs(m.red_gain-1/v->raw_red)<tolerance/v->raw_red);
        CHECK(fabs(m.blue_gain-1/v->raw_blue)<tolerance/v->raw_blue);
        CHECK(m.flags&CC_IPA_AWB_CONVERGED);
        cc_ipa_destroy(ipa);
    }
}
static void aberration(void)
{
    for (unsigned i = 0; i < sizeof(cac_vectors) / sizeof(cac_vectors[0]); ++i) {
        const struct cac_vector *v = &cac_vectors[i];
        cc_sensor_geometry g = geometry();
        g.width = g.active.width;
        g.height = g.active.height;
        struct pisp_fe_config f;
        struct pisp_be_config full, cropped;
        OK(cc_pisp_build_tuning(v->profile, &g, 4500, 1, 1, 1, &f, &full));
        CHECK(full.cac.grid_step_x == ((8U << 20) / g.width));
        CHECK(full.cac.grid_step_y == ((8U << 20) / g.height));
        for (unsigned y = 0; y < 9; ++y)
            for (unsigned x = 0; x < 9; ++x)
                for (unsigned ch = 0; ch < 2; ++ch)
                    for (unsigned axis = 0; axis < 2; ++axis)
                        CHECK(abs(full.cac.lut[y][x][ch][axis] - v->shifts[y * 9 + x][2 * ch + axis]) <= 1);
        g.crop.width /= 2;
        g.crop.height /= 2;
        g.width /= 4;
        g.height /= 4;
        g.hflip = g.vflip = 1;
        OK(cc_pisp_build_tuning(v->profile, &g, 4500, 1, 1, 1, &f, &cropped));
        for (unsigned ch = 0; ch < 2; ++ch)
            for (unsigned axis = 0; axis < 2; ++axis) {
                CHECK(abs(2 * cropped.cac.lut[0][0][ch][axis] + v->shifts[40][2 * ch + axis]) <= 2);
                CHECK(abs(2 * cropped.cac.lut[8][8][ch][axis] + v->shifts[0][2 * ch + axis]) <= 2);
            }
    }
}
static void measured_shading(void)
{
    const cc_sensor_model *hq, *other;
    OK(cc_sensor_lookup("imx477", &hq));
    OK(cc_sensor_lookup("imx219", &other));
    cc_rpi_profile profile;
    OK(cc_pisp_select_profile(hq, CC_VC4_PROFILE_AUTO, &profile));
    CHECK(profile == CC_VC4_PROFILE_IMX477);
    OK(cc_pisp_lookup_profile("imx477_16mm", &profile));
    CHECK(profile == CC_VC4_PROFILE_IMX477_16MM && !cc_vc4_get_profile_info(profile));
    CHECK(cc_vc4_lookup_profile("imx477_16mm", &profile) == -ENOENT);
    CHECK(cc_pisp_select_profile(other, profile, &profile) == -ENOTSUP);
    CHECK(cc_pisp_lookup_profile("imx477_16mm_typo", &profile) == -ENOENT);
    CHECK(profile == CC_VC4_PROFILE_IMX477_16MM);
    OK(cc_pisp_select_profile(hq, profile, &profile));

    cc_ipa_pisp_options o = {.profile = profile, .lens_shading.enabled = 1};
    for (unsigned y = 0; y < 12; ++y)
        for (unsigned x = 0; x < 16; ++x) {
            o.lens_shading.gains[0][y * 16 + x] = 1 + x / 15.0;
            o.lens_shading.gains[1][y * 16 + x] = 1;
            o.lens_shading.gains[2][y * 16 + x] = 1 + y / 11.0;
        }
    cc_ipa_configuration c = configuration(profile);
    struct pisp_fe_config f;
    struct pisp_be_config b;
    c.geometry.hflip = 1;
    OK(cc_pisp_build_tuning_with_shading(profile, &c.geometry, 4500, 1, 1, 1, &o.lens_shading, &f, &b));
    CHECK(fabs(unpack(b.lsc.lut_packed[0][0], 0) - 2) < 0.01);
    CHECK(fabs(unpack(b.lsc.lut_packed[0][32], 0) - 1) < 0.01);
    CHECK(fabs(unpack(b.lsc.lut_packed[32][0], 2) - 2) < 0.01);
    cc_ipa *ipa;
    OK(create(&o, &ipa));
    o.lens_shading.gains[1][0] = 8;
    struct pisp_be_config saved = b;
    struct pisp_fe_config saved_fe = f;
    CHECK(cc_pisp_build_tuning_with_shading(profile, &c.geometry, 4500, 1, 1, 1, &o.lens_shading, &f, &b) == -EINVAL);
    CHECK(!memcmp(&b, &saved, sizeof(b)) && !memcmp(&f, &saved_fe, sizeof(f)));
    OK(cc_ipa_configure(ipa, &c));
    OK(cc_ipa_start(ipa));
    cc_ipa_request request = {.timing = c.timing, .timing_limits = c.timing_limits};
    cc_ipa_settings s;
    OK(cc_ipa_prepare(ipa, &request, &s));
    CHECK(!memcmp(&s.backend.lsc, &b.lsc, sizeof(b.lsc)) && !memcmp(&s.backend.cac, &b.cac, sizeof(b.cac)));
    cc_ipa_destroy(ipa);
}
int main(int argc,char **argv)
{
    CHECK(argc==1 || argc==2); if (argc==2) {
        worker=argv[1]; cc_ipa *probe=NULL; int ret=create(NULL,&probe);
        if (ret==-ENOSYS || ret==-EPERM || ret==-EACCES) return 77;
        OK(ret); cc_ipa_destroy(probe);
    }
    vectors(); aberration(); measured_shading(); controllers(); white_balance();
    puts("PiSP calibrated native blocks and feedback passed"); return 0;
}
