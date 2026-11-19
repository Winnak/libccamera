/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Calibrated single-exposure mappings from Raspberry Pi Ltd libcamera 0.7.2
 * pisp.cpp, with libpisp v1.5.0 sharpening defaults. See data/ license notices. */
#include "vc4_profile.h"
#include <ccamera/pisp_tuning.h>
#include <errno.h>
#include <float.h>
#include <string.h>
#include "pisp_defaults.h"

static double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static uint32_t field(double x, unsigned bits, unsigned frac)
{
    return (uint32_t)clamp(x * (double)(1U << frac) + 0.5, 0, (double)((1U << bits) - 1U));
}
static double pwl(const double *v, size_t n, double x)
{
    if (x <= v[0]) return v[1];
    for (size_t i = 2; i < n; i += 2)
        if (x <= v[i]) return v[i-1] + (v[i+1]-v[i-1]) * (x-v[i-2]) / (v[i]-v[i-2]);
    return v[n-1];
}
const cc_rpi_profile_info *cc_pisp_get_profile_info(cc_rpi_profile profile)
{
    const struct cc_vc4_profile_data *p = cc_pisp_profile_data(profile);
    return p ? &p->info : NULL;
}
int cc_pisp_lookup_profile(const char *name, cc_rpi_profile *out)
{
    if (!name || !out)
        return -EINVAL;
    for (unsigned i = CC_VC4_PROFILE_OV5647; i <= CC_VC4_PROFILE_IMX477_16MM; ++i) {
        const cc_rpi_profile_info *p = cc_pisp_get_profile_info((cc_rpi_profile)i);
        if (p && !strcmp(name, p->name)) {
            *out = p->profile;
            return 0;
        }
    }
    return -ENOENT;
}
int cc_pisp_select_profile(const cc_sensor_model *sensor, cc_rpi_profile override, cc_rpi_profile *out)
{
    if (!sensor || !out)
        return -EINVAL;
    const cc_sensor_properties *s = cc_sensor_get_properties(sensor);
    if (!s)
        return -EINVAL;
    if (override == CC_VC4_PROFILE_AUTO)
        return cc_pisp_lookup_profile(s->model, out) == 0 ? 0 : -ENOTSUP;
    const cc_rpi_profile_info *p = cc_pisp_get_profile_info(override);
    if (!p)
        return -EINVAL;
    if (strcmp(s->model, p->sensor_model))
        return -ENOTSUP;
    *out = override;
    return 0;
}
static uint16_t grid_step(uint32_t extent, uint32_t size)
{
    uint32_t step = extent / size;
    return (uint16_t)(step > UINT16_MAX ? UINT16_MAX : step);
}
static void aberration(const struct cc_vc4_profile_data *p, const cc_sensor_geometry *g,
                       struct pisp_be_config *b)
{
    if (!p->cac)
        return;
    /* The reference tables are corner-sampled displacement vectors. Map
     * each vertex to the active area and convert shifts to mode pixels. */
    const unsigned cell = 1U << PISP_BE_CAC_STEP_PRECISION;
    const unsigned cells = PISP_BE_CAC_GRID_SIZE;
    uint32_t extent = cells * cell;
    b->cac.grid_step_x = grid_step(extent, g->width);
    b->cac.grid_step_y = grid_step(extent, g->height);
    double dx = (double)g->width / g->crop.width * (g->hflip ? -1 : 1);
    double dy = (double)g->height / g->crop.height * (g->vflip ? -1 : 1);
    const double scale[] = {dx, dy};
    for (unsigned y = 0; y <= cells; ++y)
        for (unsigned x = 0; x <= cells; ++x) {
            double u = clamp((double)x * cell / b->cac.grid_step_x / g->width, 0, 1);
            double v = clamp((double)y * cell / b->cac.grid_step_y / g->height, 0, 1);
            if (g->hflip)
                u = 1 - u;
            if (g->vflip)
                v = 1 - v;
            double sx = (g->crop.x - g->active.x + u * g->crop.width) / g->active.width * cells;
            double sy = (g->crop.y - g->active.y + v * g->crop.height) / g->active.height * cells;
            for (unsigned ch = 0; ch < 2; ++ch)
                for (unsigned axis = 0; axis < 2; ++axis) {
                    double shift = cc_rpi_sample_grid(p->cac[2 * ch + axis], cells + 1, cells + 1, sx, sy);
                    double code = shift * scale[axis] * p->cac_strength * 32;
                    b->cac.lut[y][x][ch][axis] = (int8_t)clamp(code + (code < 0 ? -0.5 : 0.5), -64, 63);
                }
        }
    b->global.bayer_enables |= PISP_BE_BAYER_ENABLE_CAC;
}
static void sharpen(const struct cc_vc4_profile_data *p, double mode_factor, struct pisp_be_config *b)
{
    b->sharpen = pisp_sharpen_base;
#define SCALE(name, value, bits) b->sharpen.name = (uint16_t)field(b->sharpen.name * (value), bits, 0)
    double scale = p->sharpen_threshold * mode_factor * 0.25;
#define FILTER(i) SCALE(threshold_offset##i, scale, 16); SCALE(threshold_slope##i, scale, 12)
    FILTER(0); FILTER(1); FILTER(2); FILTER(3); FILTER(4);
#undef FILTER
    double strength = p->sharpen_strength / mode_factor, limit = p->sharpen_limit / mode_factor;
    SCALE(positive_strength, strength, 12); SCALE(negative_strength, strength, 12);
    SCALE(positive_pre_limit, limit, 16); SCALE(negative_pre_limit, limit, 16);
    SCALE(positive_limit, limit, 16); SCALE(negative_limit, limit, 16);
#undef SCALE
    b->sh_fc_combine.y_factor = 192;
    b->ycbcr = (struct pisp_be_ccm_config){.coeffs={306,601,117,-173,-339,512,512,-429,-83},
        .offsets={0,33554432,33554432}};
    b->ycbcr_inverse = (struct pisp_be_ccm_config){.coeffs={1024,0,1436,1024,-352,-731,1024,1815,0},
        .offsets={-47043259,35509710,-59458469}};
}
int cc_pisp_build_tuning_with_shading(cc_rpi_profile profile, const cc_sensor_geometry *g, double ct,
    double gain, double red, double blue, const cc_rpi_lens_shading *shading,
    struct pisp_fe_config *fe, struct pisp_be_config *be)
{
    const struct cc_vc4_profile_data *p = cc_pisp_profile_data(profile);
    if (!p || !fe || !be || !cc_sensor_geometry_valid(g) || g->width < 64 || g->height < 64 ||
        (g->width | g->height) & 1U || !(ct >= 2000 && ct <= 13000) || !(gain >= 1 && gain <= 64) ||
        !(red >= 0.5 && red <= 4) || !(blue >= 0.5 && blue <= 4) ||
        !cc_rpi_lens_shading_valid(shading)) return -EINVAL;
    struct pisp_fe_config f = {0}; struct pisp_be_config b = {0};
    f.global.enables = PISP_FE_ENABLE_BLC | PISP_FE_ENABLE_RGBY | PISP_FE_ENABLE_AGC_STATS |
        PISP_FE_ENABLE_AWB_STATS | PISP_FE_ENABLE_CDAF_STATS;
    b.global.bayer_enables = PISP_BE_BAYER_ENABLE_BLC | PISP_BE_BAYER_ENABLE_DPC |
        PISP_BE_BAYER_ENABLE_GEQ | PISP_BE_BAYER_ENABLE_WBG | PISP_BE_BAYER_ENABLE_LSC |
        PISP_BE_BAYER_ENABLE_SDN | PISP_BE_BAYER_ENABLE_CDN;
    b.global.rgb_enables = PISP_BE_RGB_ENABLE_CCM | PISP_BE_RGB_ENABLE_GAMMA |
        PISP_BE_RGB_ENABLE_SHARPEN | PISP_BE_RGB_ENABLE_YCBCR | PISP_BE_RGB_ENABLE_YCBCR_INVERSE;
    f.blc = b.blc = (struct pisp_bla_config){.black_level_r=p->black, .black_level_gr=p->black,
        .black_level_gb=p->black, .black_level_b=p->black};
    double minimum = red < blue ? red : blue;
    if (minimum > 1) minimum = 1;
    b.wbg = (struct pisp_wbg_config){.gain_r=(uint16_t)field(red/minimum,14,10),
        .gain_g=(uint16_t)field(1/minimum,14,10), .gain_b=(uint16_t)field(blue/minimum,14,10)};
    f.rgby = (struct pisp_fe_rgby_config){.gain_r=(uint16_t)field(306*red/minimum,14,0),
        .gain_g=(uint16_t)field(601/minimum,14,0), .gain_b=(uint16_t)field(117*blue/minimum,14,0)};
    /* Reference 15x15 metering, with unused right/bottom native zones zero. */
    f.agc_stats.size_x = (uint16_t)((g->width/15) & ~1U);
    f.agc_stats.size_y = (uint16_t)((g->height/15) & ~1U);
    f.agc_stats.offset_x = (uint16_t)(((g->width-15U*f.agc_stats.size_x)/2) & ~1U);
    f.agc_stats.offset_y = (uint16_t)(((g->height-15U*f.agc_stats.size_y)/2) & ~1U);
    for (unsigned y=0; y<15; ++y) for (unsigned x=0; x<15; ++x)
        f.agc_stats.weights[y*8+x/2] |= (uint8_t)((unsigned)p->weights[y*15+x] << (4*(x&1U)));
    f.awb_stats.r_hi = f.awb_stats.g_hi = f.awb_stats.b_hi = UINT16_MAX;
    f.cdaf_stats.mode = 0x15;
    b.dpc = (struct pisp_be_dpc_config){.coeff_level=1,.coeff_range=8};
    b.demosaic = (struct pisp_be_demosaic_config){.sharper=8,.fc_mode=1};
    b.geq = (struct pisp_be_geq_config){.offset=(uint16_t)field(p->geq_offset*gain,16,0),
        .slope_sharper=(uint16_t)field(p->geq_slope*gain,10,10),.max=65535};
    double root=gain;
    for (unsigned i=0; i<16; ++i) root=(root+gain/root)*0.5;
    double mode_factor = cc_rpi_mode_noise_factor(g);
    root /= mode_factor;
    b.sdn = (struct pisp_be_sdn_config){.black_level=p->black,
        .leakage=(uint8_t)field(1-p->sdn_strength,8,8),
        .noise_constant=(uint16_t)field(p->noise_constant*root*p->sdn_deviation,16,0),
        .noise_constant2=(uint16_t)field(p->noise_constant*root*p->sdn_deviation,16,0),
        .noise_slope=(uint16_t)field(p->noise_slope*root*p->sdn_deviation,16,8),
        .noise_slope2=(uint16_t)field(p->noise_slope*root*p->sdn_deviation,16,8)};
    b.cdn = (struct pisp_be_cdn_config){.thresh=(uint16_t)field(p->cdn_deviation*p->noise_slope*root+
        p->noise_constant*root,16,0), .iir_strength=(uint8_t)field(p->cdn_strength,8,8)};
    unsigned ci=0;
    while (ci+2 < p->num_ccm && ct > p->ccm[ci+1][0]) ++ci;
    double mix=clamp((ct-p->ccm[ci][0])/(p->ccm[ci+1][0]-p->ccm[ci][0]),0,1);
    for (unsigned i=0; i<9; ++i) {
        double value=(p->ccm[ci][i+1]*(1-mix)+p->ccm[ci+1][i+1]*mix)*1024;
        b.ccm.coeffs[i]=(int16_t)clamp(value+(value<0 ? -0.5:0.5),-8192,8191);
    }
    uint32_t y=(uint32_t)pwl(p->gamma,p->num_gamma,0);
    for (unsigned i=0; i<64; ++i) {
        unsigned j=i+1, x=j<32 ? j*512 : j<48 ? (j-32)*1024+16384 : (j-48)*2048+32768;
        uint32_t next=(uint32_t)pwl(p->gamma,p->num_gamma,x), slope=next-y;
        if (slope>16383) slope=16383;
        b.gamma.lut[i]=y | (slope<<16); y+=slope;
    }
    unsigned li=0;
    while (li+2 < p->num_lsc && ct>p->lsc_ct[li+1]) ++li;
    mix=clamp((ct-p->lsc_ct[li])/(p->lsc_ct[li+1]-p->lsc_ct[li]),0,1);
    /* The factory chroma tables contain correction gains. */
    double full[3][1024], adjusted[3][1024];
    for (unsigned i=0; i<1024; ++i) {
        double lum=1+p->luminance_strength*(p->lsc_y[i]-1);
        full[0][i]=lum*(p->lsc_r[li*1024+i]*(1-mix)+p->lsc_r[(li+1)*1024+i]*mix);
        full[1][i]=lum;
        full[2][i]=lum*(p->lsc_b[li*1024+i]*(1-mix)+p->lsc_b[(li+1)*1024+i]*mix);
    }
    for (unsigned ch=0; ch<3; ++ch)
        if (shading && shading->enabled)
            cc_rpi_resample_shading(shading->gains[ch], 16, 12, g, adjusted[ch], 32, 32);
        else
            cc_rpi_resample_shading(full[ch], 32, 32, g, adjusted[ch], 32, 32);
    uint32_t extent=32U<<18;
    b.lsc.grid_step_x = grid_step(extent, g->width);
    b.lsc.grid_step_y = grid_step(extent, g->height);
    for (unsigned y0=0; y0<33; ++y0) for (unsigned x0=0; x0<33; ++x0) {
        double rgb[3], lo=DBL_MAX, hi=0;
        for (unsigned ch=0; ch<3; ++ch) {
            rgb[ch]=cc_rpi_sample_grid(adjusted[ch],32,32,(double)x0*(1U<<18)/b.lsc.grid_step_x/g->width*32-0.5,
                (double)y0*(1U<<18)/b.lsc.grid_step_y/g->height*32-0.5);
            if (rgb[ch]<lo) lo=rgb[ch];
            if (rgb[ch]>hi) hi=rgb[ch];
        }
        unsigned range=lo>=0.5 && hi<1.5 ? 0 : hi<2 ? 1 : hi<4 ? 2 : 3;
        double scale=range ? (double)(1024U>>range) : 1024, offset=range ? 0.5 : -511.5;
        b.lsc.lut_packed[y0][x0]=(range<<30) | field(offset+scale*rgb[0],10,0) |
            (field(offset+scale*rgb[1],10,0)<<10) | (field(offset+scale*rgb[2],10,0)<<20);
    }
    aberration(p,g,&b);
    sharpen(p,mode_factor,&b);
    *fe=f; *be=b;
    return 0;
}
int cc_pisp_build_tuning(cc_rpi_profile profile, const cc_sensor_geometry *g, double ct,
    double gain, double red, double blue, struct pisp_fe_config *fe, struct pisp_be_config *be)
{
    return cc_pisp_build_tuning_with_shading(profile, g, ct, gain, red, blue, NULL, fe, be);
}
