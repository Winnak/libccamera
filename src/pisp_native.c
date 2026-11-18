/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Copyright (C) 2022-2023 Raspberry Pi Ltd */
/* Register mappings and histogram tone policy derived from Raspberry Pi Ltd's
 * archived libcamera 0.7.2 denoise.cpp, hdr.cpp and pisp.cpp. */
#include "pisp_native.h"
#include "pisp_format.h"
#include <errno.h>
#include <linux/videodev2.h>
#include <string.h>

int cc_pisp_be_configure_native_photo(cc_pisp_be *be, cc_pisp_be_configuration *configuration, uint32_t features)
{
    if (!configuration || !features || (features & ~3U)) return -EINVAL;
    cc_pisp_be_configuration c=*configuration;
    struct cc_pisp_raw_layout raw;
    int ret=cc_pisp_raw_layout(&c.input,&raw);
    if (ret<0) return ret;
    static const uint32_t formats[]={V4L2_PIX_FMT_SRGGB16,V4L2_PIX_FMT_SGBRG16,
        V4L2_PIX_FMT_SBGGR16,V4L2_PIX_FMT_SGRBG16};
    if (raw.bayer_order>=4) return -ENOTSUP;
    for (unsigned i=0; i<4; ++i) {
        c.auxiliary[i]=(cc_pisp_be_aux_configuration){0};
        if (features & (i<2 ? CC_NATIVE_PHOTO_TDN:CC_NATIVE_PHOTO_HDR))
            c.auxiliary[i].format=(cc_format){.fourcc=formats[raw.bayer_order],
                .width=c.input.width,.height=c.input.height};
    }
    ret=cc_pisp_be_configure(be,&c);
    if (!ret) *configuration=c;
    return ret;
}

static double clamp(double x, double a, double b) { return x<a ? a:x>b ? b:x; }
static uint16_t field(double x, unsigned bits, unsigned frac)
{
    return (uint16_t)clamp(x*(1U<<frac)+0.5,0,(1U<<bits)-1);
}
void cc_pisp_native_denoise(const struct cc_vc4_profile_data *p, double gain, double mode_factor,
    double ratio, uint32_t frames, int hdr, struct pisp_be_config *b)
{
    const struct cc_pisp_denoise_profile *d=&p->temporal[hdr!=0];
    double root=gain;
    for (unsigned i=0; i<16; ++i) root=(root+gain/root)*0.5;
    root /= mode_factor;
    double backoff=1;
    for (uint32_t i=0; i<frames; ++i) backoff*=d->backoff;
    double deviation=backoff*p->sdn_deviation+(1-backoff)*d->deviation;
    double deviation2=backoff*p->sdn_deviation+(1-backoff)*d->deviation2;
    double strength=backoff*p->sdn_strength+(1-backoff)*d->strength;
    double cdn=backoff*p->cdn_deviation+(1-backoff)*d->cdn_deviation;
    b->sdn=(struct pisp_be_sdn_config){.black_level=p->black,
        .leakage=(uint8_t)field(1-strength,8,8),
        .noise_constant=field(p->noise_constant*root*deviation,16,0),
        .noise_constant2=field(p->noise_constant*root*deviation2,16,0),
        .noise_slope=field(p->noise_slope*root*deviation,16,8),
        .noise_slope2=field(p->noise_slope*root*deviation2,16,8)};
    b->cdn.thresh=field(root*(cdn*p->noise_slope+p->noise_constant),16,0);
    b->tdn=(struct pisp_be_tdn_config){.black_level=p->black,.ratio=field(ratio,16,14),
        .noise_constant=field(p->noise_constant*root*d->tdn_deviation,16,0),
        .noise_slope=field(p->noise_slope*root*d->tdn_deviation,16,8),
        .threshold=field(d->tdn_threshold,16,16),.reset=frames==0};
    b->global.bayer_enables|=PISP_BE_BAYER_ENABLE_TDN;
}
/* Bounded positive-domain log/exp, avoiding a mandatory libm dependency. */
static double logarithm(double x)
{
    int exponent=0;
    while (x<1) { x*=2; --exponent; }
    while (x>=2) { x*=0.5; ++exponent; }
    double z=(x-1)/(x+1), term=z, sum=z;
    for (unsigned i=3; i<=31; i+=2) { term*=z*z; sum+=term/i; }
    return 2*sum+exponent*0.69314718055994530942;
}
static double power(double x, double p)
{
    if (x<=0) return 0;
    double y=p*logarithm(x);
    int exponent=(int)(y/0.69314718055994530942);
    y-=exponent*0.69314718055994530942;
    double term=1, sum=1;
    for (unsigned i=1; i<=18; ++i) { term*=y/i; sum+=term; }
    while (exponent<0) { sum*=0.5; ++exponent; }
    while (exponent>0) { sum*=2; --exponent; }
    return sum;
}
static double histogram_mean(const uint32_t *hist, double lo, double hi)
{
    uint64_t total=0;
    for (unsigned i=0; i<1024; ++i) total+=hist[i];
    if (!total) return 0;
    double a=lo*(double)total, z=hi*(double)total, count=0, sum=0, seen=0;
    for (unsigned i=0; i<1024; ++i) {
        double next=seen+hist[i], accepted=clamp(next,a,z)-clamp(seen,a,z);
        sum+=accepted*(i+0.5); count+=accepted; seen=next;
    }
    return count>0 ? sum/count/1024:0;
}
static void spatial_gain(const struct cc_pisp_hdr_profile *h, const struct pisp_statistics *stats,
    const struct pisp_fe_config *fe, struct pisp_be_config *b)
{
    double gains[2][1024];
    for (unsigned i=0; i<1024; ++i) {
        const struct pisp_awb_statistics_zone *z=&stats->awb.zones[i];
        double maximum=z->R_sum;
        if (z->G_sum>maximum) maximum=z->G_sum;
        if (z->B_sum>maximum) maximum=z->B_sum;
        double light=maximum/(z->counted ? z->counted:1)/65535;
        gains[0][i]=1+(h->spatial_gain-1)*clamp((0.06-light)/0.05,0,1);
    }
    for (unsigned pass=0; pass<h->diffusion; ++pass) {
        const double *input=gains[pass&1U]; double *output=gains[(pass+1)&1U];
        for (unsigned y=0; y<32; ++y) for (unsigned x=0; x<32; ++x) {
            unsigned i=y*32+x, count=1; double sum=input[i];
            if (x) { sum+=input[i-1]; ++count; }
            if (x<31) { sum+=input[i+1]; ++count; }
            if (y) { sum+=input[i-32]; ++count; }
            if (y<31) { sum+=input[i+32]; ++count; }
            output[i]=sum/count;
        }
    }
    const double *gain=gains[h->diffusion&1U];
    for (unsigned y=0; y<33; ++y) for (unsigned x=0; x<33; ++x) {
        double decimation=fe->global.enables&PISP_FE_ENABLE_DECIMATE ? 2:1;
        double px=(double)x*(1U<<18)/b->lsc.grid_step_x/decimation;
        double py=(double)y*(1U<<18)/b->lsc.grid_step_y/decimation;
        double u=clamp((px-fe->awb_stats.offset_x)/fe->awb_stats.size_x-0.5,0,31);
        double v=clamp((py-fe->awb_stats.offset_y)/fe->awb_stats.size_y-0.5,0,31);
        unsigned a=(unsigned)u, c=a<31 ? a+1:a, d=(unsigned)v, e=d<31 ? d+1:d;
        u-=a; v-=d;
        double scale=(gain[d*32+a]*(1-u)+gain[d*32+c]*u)*(1-v)+
            (gain[e*32+a]*(1-u)+gain[e*32+c]*u)*v;
        if (scale==1) continue;
        uint32_t packed=b->lsc.lut_packed[y][x], range=packed>>30;
        double rgb[3], low=16, high=0;
        for (unsigned ch=0; ch<3; ++ch) {
            uint32_t code=(packed>>(10*ch))&1023U;
            rgb[ch]=(range ? (double)code/(1024U>>range):(double)(code+512)/1024)*scale;
            if (rgb[ch]<low) low=rgb[ch];
            if (rgb[ch]>high) high=rgb[ch];
        }
        range=low>=0.5 && high<1.5 ? 0U:high<2 ? 1U:high<4 ? 2U:3U;
        double factor=range ? (double)(1024U>>range):1024, offset=range ? 0:-512;
        b->lsc.lut_packed[y][x]=(range<<30)|(uint32_t)field(offset+factor*rgb[0],10,0)|
            ((uint32_t)field(offset+factor*rgb[1],10,0)<<10)|((uint32_t)field(offset+factor*rgb[2],10,0)<<20);
    }
}
void cc_pisp_native_hdr(const struct cc_vc4_profile_data *p, double ratio,
    const struct pisp_fe_config *frontend, const struct pisp_statistics *statistics, struct pisp_be_config *b)
{
    uint32_t histogram[1024];
    memcpy(histogram,statistics->agc.histogram,sizeof(histogram));
    const struct cc_pisp_hdr_profile *h=&p->hdr;
    b->stitch=(struct pisp_be_stitch_config){
        .exposure_ratio=(uint16_t)(PISP_BE_STITCH_STREAMING_LONG|field(ratio,15,15)),
        .threshold_lo=h->threshold,.threshold_diff_power=h->diff_power,
        .motion_threshold_256=(uint8_t)field(h->motion_threshold,8,8)};
    b->global.bayer_enables|=PISP_BE_BAYER_ENABLE_STITCH|PISP_BE_BAYER_ENABLE_TONEMAP;
    spatial_gain(h,statistics,frontend,b);
    /* hdr.cpp's default MultiExposure quantile targets and contrast factors. */
    const double hi[][2]={{0.95,0.65},{0.5,0.28},{0.3,0.25}}, lo[][2]={{0.2,0.03},{1,0.15}};
    double gain=10, exponent=2;
    for (unsigned i=0; i<3; ++i) {
        double g=hi[i][1]/(histogram_mean(histogram,hi[i][0],1)+0.01);
        if (g<gain) gain=g;
    }
    gain=clamp(gain,1,1.6);
    for (unsigned i=0; i<2; ++i) {
        double value=clamp(histogram_mean(histogram,0,lo[i][0])*gain,0,1);
        double pwr=logarithm(lo[i][1]+1e-6)/logarithm(value+1e-6);
        if (pwr<exponent) exponent=pwr;
    }
    exponent=clamp(exponent,0.65,1);
    double points[9]={0};
    for (unsigned i=0; i<7; ++i) {
        points[i+1]=power(clamp((double)(512U<<i)*gain/65536,0,1),exponent)*65536;
        if (i<2) points[i+1]*=i ? 0.75:0.5;
    }
    points[8]=65535;
    b->tonemap=(struct pisp_be_tonemap_config){.detail_constant=h->detail_constant,
        .detail_slope=field(h->detail_slope,16,8),.iir_strength=field(h->iir_strength,12,4),
        .strength=field(h->strength,12,8)};
    uint32_t previous=0;
    for (unsigned i=0; i<64; ++i) {
        unsigned j=i+1, x=j<32 ? j*512:j<48 ? (j-32)*1024+16384:(j-48)*2048+32768;
        unsigned segment=0, left=0, right=512;
        while (segment<7 && x>right) { ++segment; left=right; right=segment==7 ? 65536:right*2; }
        double y=points[segment]+(points[segment+1]-points[segment])*(x-left)/(right-left);
        uint32_t next=(uint32_t)clamp(y,previous,65535), slope=next-previous;
        if (slope>16383) slope=16383;
        b->tonemap.lut[i]=previous|(slope<<16); previous+=slope;
    }
}
