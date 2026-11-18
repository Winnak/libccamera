/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "pisp_native.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "native_tuning_vectors.h"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
int main(void)
{
    for (size_t i=0; i<sizeof(temporal_vectors)/sizeof(temporal_vectors[0]); ++i) {
        const struct temporal_vector *v=&temporal_vectors[i];
        const struct cc_vc4_profile_data *p=cc_pisp_profile_data((cc_rpi_profile)v->profile);
        CHECK(p);
        struct pisp_be_config b={0};
        cc_pisp_native_denoise(p,v->gain,v->mode_factor,0.5,v->frames,(int)v->hdr,&b);
        CHECK(b.tdn.reset==!v->frames && b.tdn.ratio==8192);
        CHECK(b.tdn.noise_constant==v->constant && b.tdn.noise_slope==v->slope && b.tdn.threshold==v->threshold);
        CHECK(b.sdn.noise_constant==v->sdn[0] && b.sdn.noise_slope==v->sdn[1] &&
            b.sdn.noise_constant2==v->sdn[2] && b.sdn.noise_slope2==v->sdn[3] &&
            b.sdn.leakage==v->sdn[4] && b.cdn.thresh==v->cdn);
    }
    for (unsigned id=CC_VC4_PROFILE_OV5647; id<=CC_VC4_PROFILE_IMX477_16MM; ++id)
    for (size_t i=0; i<sizeof(hdr_vectors)/sizeof(hdr_vectors[0]); ++i) {
        const struct hdr_vector *v=&hdr_vectors[i];
        struct pisp_statistics stats={0}; stats.agc.histogram[v->bin]=UINT32_MAX;
        struct pisp_be_config b={0};
        b.lsc.grid_step_x=b.lsc.grid_step_y=8192;
        struct pisp_fe_config f={0}; f.awb_stats.size_x=f.awb_stats.size_y=32;
        cc_pisp_native_hdr(cc_pisp_profile_data((cc_rpi_profile)id),0.25,&f,&stats,&b);
        CHECK(b.stitch.exposure_ratio==(PISP_BE_STITCH_STREAMING_LONG|8192) &&
            b.stitch.threshold_lo==50000 && b.stitch.threshold_diff_power==13 && b.stitch.motion_threshold_256==1);
        CHECK(b.tonemap.iir_strength==128 && b.tonemap.strength==384);
        for (unsigned j=0; j<64; ++j) {
            CHECK(abs((int)(b.tonemap.lut[j]&65535U)-(int)(v->lut[j]&65535U))<=1);
            CHECK(abs((int)(b.tonemap.lut[j]>>16)-(int)(v->lut[j]>>16))<=1);
        }
        /* Flat dark regions receive the profile's 2x shadow boost. Bright
         * regions preserve every original shading bit, including chroma. */
        uint32_t unity=512U|(512U<<10)|(512U<<20);
        for (unsigned y=0; y<33; ++y) for (unsigned x=0; x<33; ++x) b.lsc.lut_packed[y][x]=unity;
        cc_pisp_native_hdr(cc_pisp_profile_data((cc_rpi_profile)id),0.25,&f,&stats,&b);
        unsigned packed=b.lsc.lut_packed[16][16], range=packed>>30, code=packed&1023U;
        CHECK(range && (double)code/(1024U>>range)==2);
        for (unsigned z=0; z<1024; ++z) stats.awb.zones[z]=(struct pisp_awb_statistics_zone){.counted=1,.G_sum=32768};
        struct pisp_be_lsc_config saved=b.lsc;
        cc_pisp_native_hdr(cc_pisp_profile_data((cc_rpi_profile)id),0.25,&f,&stats,&b);
        CHECK(!memcmp(&saved,&b.lsc,sizeof(saved)));
    }
    for (size_t i=0; i<sizeof(spatial_vectors)/sizeof(spatial_vectors[0]); ++i) {
        const struct spatial_vector *v=&spatial_vectors[i];
        struct pisp_statistics stats={0}; stats.agc.histogram[512]=100;
        for (unsigned y=0; y<32; ++y) for (unsigned x=0; x<32; ++x)
            stats.awb.zones[y*32+x]=(struct pisp_awb_statistics_zone){.counted=1,.G_sum=(x+2*y)*65};
        struct pisp_be_config b={0};
        b.lsc.grid_step_x=(uint16_t)v->step_x; b.lsc.grid_step_y=(uint16_t)v->step_y;
        for (unsigned y=0; y<33; ++y) for (unsigned x=0; x<33; ++x)
            b.lsc.lut_packed[y][x]=512U|(512U<<10)|(512U<<20);
        struct pisp_fe_config f={0}; f.awb_stats.size_x=(uint16_t)v->size_x; f.awb_stats.size_y=(uint16_t)v->size_y;
        if (v->decimate) f.global.enables|=PISP_FE_ENABLE_DECIMATE;
        cc_pisp_native_hdr(cc_pisp_profile_data(CC_VC4_PROFILE_IMX219),0.25,&f,&stats,&b);
        for (unsigned y=0; y<3; ++y) for (unsigned x=0; x<3; ++x) {
            uint32_t packed=b.lsc.lut_packed[y*16][x*16], range=packed>>30, code=packed&1023U;
            double value=range ? (double)code/(1024U>>range):(double)(code+512)/1024;
            CHECK(value-v->gain[y*3+x]<0.004 && value-v->gain[y*3+x]>-0.004);
        }
    }
    puts("Native temporal noise, HDR tone and spatial-grid fixtures passed");
    return 0;
}
