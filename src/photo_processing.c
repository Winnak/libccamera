/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Manual gamma/CCM controls follow Raspberry Pi Ltd's native ISP policy. */
#include "photo_processing.h"

static double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
cc_photo_controls cc_photo_defaults(const cc_ipa_vc4_options *o)
{
    return (cc_photo_controls){.flags=CC_PHOTO_ALL, .contrast=1, .saturation=1, .sharpness=1,
        .white_balance=o->red_gain > 0 ? CC_PHOTO_WB_MANUAL:CC_PHOTO_WB_AUTO,
        .red_gain=o->red_gain, .blue_gain=o->blue_gain, .mains_hz=o->mains_hz,
        .denoise=CC_PHOTO_DENOISE_NORMAL};
}
cc_photo_controls cc_photo_resolve(const cc_ipa_vc4_options *o, const cc_photo_controls *c)
{
    cc_photo_controls p=cc_photo_defaults(o);
    if (!c) return p;
#define COPY(flag, field) if (c->flags & (flag)) p.field=c->field
    COPY(CC_PHOTO_EXPOSURE, exposure_ns); COPY(CC_PHOTO_GAIN, analogue_gain);
    COPY(CC_PHOTO_EV, exposure_value); COPY(CC_PHOTO_METERING, metering);
    COPY(CC_PHOTO_WB, white_balance); COPY(CC_PHOTO_WB, red_gain); COPY(CC_PHOTO_WB, blue_gain);
    COPY(CC_PHOTO_BRIGHTNESS, brightness); COPY(CC_PHOTO_CONTRAST, contrast);
    COPY(CC_PHOTO_SATURATION, saturation); COPY(CC_PHOTO_SHARPNESS, sharpness);
    COPY(CC_PHOTO_DENOISE, denoise); COPY(CC_PHOTO_FLICKER, mains_hz);
    COPY(CC_PHOTO_FRAME_DURATION, min_frame_duration_ns); COPY(CC_PHOTO_FRAME_DURATION, max_frame_duration_ns);
#undef COPY
    return p;
}
double cc_photo_ev_multiplier(double ev)
{
    /* exp2 on the bounded [-8,8] domain without a runtime libm dependency. */
    int whole=(int)ev;
    double x=(ev-whole)*0.69314718055994530942, sum=1, term=1;
    for (unsigned i=1; i<=16; ++i) { term*=x/i; sum+=term; }
    while (whole>0) { sum*=2; --whole; }
    while (whole<0) { sum*=0.5; ++whole; }
    return sum;
}
static double gamma_value(const cc_photo_controls *c, double y)
{
    return clamp((y-32768)*c->contrast+32768+c->brightness*65535,0,65535);
}
static void saturation(double matrix[9], double scale)
{
    const double luma[3]={0.299,0.587,0.114};
    double result[9]={0};
    for (unsigned row=0; row<3; ++row) for (unsigned col=0; col<3; ++col)
        for (unsigned k=0; k<3; ++k)
            result[row*3+col]+=((row==k ? scale:0)+(1-scale)*luma[k])*matrix[k*3+col];
    for (unsigned i=0; i<9; ++i) matrix[i]=result[i];
}
void cc_photo_apply_vc4(const cc_photo_controls *c, cc_vc4_tuning *t)
{
    if (c->brightness!=0 || c->contrast!=1)
        for (unsigned i=0; i<33; ++i) t->gamma.y[i]=(uint16_t)gamma_value(c,t->gamma.y[i]);
    if (c->saturation!=1) {
        double m[9];
        for (unsigned i=0; i<9; ++i) m[i]=(double)t->ccm.ccm.ccm[i/3][i%3].num/t->ccm.ccm.ccm[i/3][i%3].den;
        saturation(m,c->saturation);
        for (unsigned i=0; i<9; ++i)
            t->ccm.ccm.ccm[i/3][i%3]=(struct bcm2835_isp_rational){(int32_t)(m[i]*1000),1000};
    }
    t->sharpen.strength.num=(int32_t)(t->sharpen.strength.num*c->sharpness);
    t->sharpen.enabled=c->sharpness>0;
    if (c->denoise==CC_PHOTO_DENOISE_OFF) t->denoise.enabled=t->cdn.enabled=0;
}
void cc_photo_apply_pisp(const struct cc_vc4_profile_data *p, const cc_photo_controls *c,
    struct pisp_fe_config *f, struct pisp_be_config *b)
{
    const double *weights=p->metering[c->metering];
    for (unsigned i=0; i<128; ++i) f->agc_stats.weights[i]=0;
    for (unsigned y=0; y<15; ++y) for (unsigned x=0; x<15; ++x)
        f->agc_stats.weights[y*8+x/2] |= (uint8_t)((unsigned)weights[y*15+x] << (4*(x&1U)));
    if (c->saturation!=1) {
        double m[9];
        for (unsigned i=0; i<9; ++i) m[i]=(double)b->ccm.coeffs[i]/1024;
        saturation(m,c->saturation);
        for (unsigned i=0; i<9; ++i) {
            double v=m[i]*1024;
            b->ccm.coeffs[i]=(int16_t)clamp(v+(v<0 ? -0.5:0.5),-8192,8191);
        }
    }
    if (c->brightness!=0 || c->contrast!=1) {
        uint32_t prev=(uint32_t)gamma_value(c,b->gamma.lut[0]&65535U);
        for (unsigned i=0; i<64; ++i) {
            unsigned packed=b->gamma.lut[i];
            uint32_t next=(uint32_t)gamma_value(c,(packed&65535U)+(packed>>16));
            uint32_t slope=next>prev ? next-prev:0;
            if (slope>16383) slope=16383;
            b->gamma.lut[i]=prev | (slope<<16); prev+=slope;
        }
    }
    b->sharpen.positive_strength=(uint16_t)clamp(b->sharpen.positive_strength*c->sharpness,0,4095);
    b->sharpen.negative_strength=(uint16_t)clamp(b->sharpen.negative_strength*c->sharpness,0,4095);
    if (c->sharpness==0) b->global.rgb_enables &= ~(uint32_t)PISP_BE_RGB_ENABLE_SHARPEN;
    if (c->denoise==CC_PHOTO_DENOISE_OFF)
        b->global.bayer_enables &= ~(uint32_t)(PISP_BE_BAYER_ENABLE_SDN|PISP_BE_BAYER_ENABLE_CDN);
}
