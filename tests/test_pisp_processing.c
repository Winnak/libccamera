/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* JPEG matrix vectors from libpisp v1.5.0 backend_default_config.json.
 * Copyright (C) 2021-2023 Raspberry Pi Ltd. See LICENSES/BSD-2-Clause.txt. */
#include <ccamera/pisp_be.h>

#include <errno.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)
#define OK(x) CHECK((x) == 0)

static cc_format image(uint32_t w, uint32_t h, uint32_t fourcc, uint32_t bytes)
{
    uint32_t stride = (w * bytes + 15U) & ~15U;
    return (cc_format) { .width = w, .height = h, .fourcc = fourcc, .num_planes = 1,
                         .planes = {{stride, stride * h}} };
}

static cc_pisp_be_configuration configuration(void)
{
    return (cc_pisp_be_configuration) { .input = image(1640, 1232, V4L2_PIX_FMT_SRGGB16, 2),
        .output = image(768, 576, V4L2_PIX_FMT_RGB24, 3), .crop = {{32, 16, 1536, 1152}},
        .output1 = image(320, 240, V4L2_PIX_FMT_RGB24, 3), .transform = {1, 2} };
}

static struct pisp_be_config tuning(void)
{
    struct pisp_be_config t = {0};
    t.global.bayer_enables = PISP_BE_BAYER_ENABLE_DPC | PISP_BE_BAYER_ENABLE_GEQ | PISP_BE_BAYER_ENABLE_SDN |
        PISP_BE_BAYER_ENABLE_BLC | PISP_BE_BAYER_ENABLE_WBG | PISP_BE_BAYER_ENABLE_CDN | PISP_BE_BAYER_ENABLE_LSC |
        PISP_BE_BAYER_ENABLE_CAC | PISP_BE_BAYER_ENABLE_DEBIN | PISP_BE_BAYER_ENABLE_TONEMAP;
    t.global.rgb_enables = PISP_BE_RGB_ENABLE_CCM | PISP_BE_RGB_ENABLE_SAT_CONTROL | PISP_BE_RGB_ENABLE_YCBCR |
        PISP_BE_RGB_ENABLE_YCBCR_INVERSE | PISP_BE_RGB_ENABLE_FALSE_COLOUR | PISP_BE_RGB_ENABLE_SHARPEN |
        PISP_BE_RGB_ENABLE_GAMMA;
    t.dpc = (struct pisp_be_dpc_config) {31, 31, 255, 1};
    t.geq = (struct pisp_be_geq_config) {65535, PISP_BE_GEQ_SHARPER | 1023, 0, 65535};
    memset(&t.sdn, 255, sizeof(t.sdn));
    memset(&t.cdn, 255, sizeof(t.cdn));
    t.debin = (struct pisp_be_debin_config) {{-7, 105, 35, -5}, 1, 1, {-1, -1}};
    t.tonemap.detail_constant = t.tonemap.detail_slope = 65535;
    t.tonemap.iir_strength = t.tonemap.strength = 4095;
    for (size_t i = 0; i < PISP_BE_TONEMAP_LUT_SIZE; ++i) t.tonemap.lut[i] = UINT32_MAX;
    for (size_t y = 0; y < PISP_BE_CAC_LUT_SIZE; ++y)
        for (size_t x = 0; x < PISP_BE_CAC_LUT_SIZE; ++x) {
            t.cac.lut[y][x][0][0] = -64; t.cac.lut[y][x][0][1] = 63;
            t.cac.lut[y][x][1][0] = -1; t.cac.lut[y][x][1][1] = (int8_t)(x + y);
        }
    t.demosaic = (struct pisp_be_demosaic_config) {255, 2, {255, 255}};
    t.ccm.coeffs[0] = t.ccm.coeffs[4] = t.ccm.coeffs[8] = 1024;
    t.ccm.offsets[0] = -67108864; t.ccm.offsets[1] = 67108863;
    t.sat_control = (struct pisp_be_sat_control_config) {0, 1, 2, 255};
    t.ycbcr = (struct pisp_be_ccm_config) {{306,601,117,-173,-339,512,512,-429,-83}, {255,255},
                                        {0,33554432,33554432}};
    t.ycbcr_inverse = (struct pisp_be_ccm_config) {{1024,0,1436,1024,-352,-731,1024,1815,0}, {255,255},
                                                {-47043259,35509710,-59458469}};
    t.false_colour = (struct pisp_be_false_colour_config) {2, {255,255,255}};
    memset(&t.sharpen, 255, sizeof(t.sharpen));
    int8_t *kernels[] = {t.sharpen.kernel0, t.sharpen.kernel1, t.sharpen.kernel2, t.sharpen.kernel3, t.sharpen.kernel4};
    for (size_t i = 0; i < 5; ++i) {
        memset(kernels[i], 0, 25);
        kernels[i][12] = 31; kernels[i][i] = -32;
    }
    t.sharpen.threshold_slope0 = t.sharpen.threshold_slope1 = t.sharpen.threshold_slope2 =
        t.sharpen.threshold_slope3 = t.sharpen.threshold_slope4 = 4095;
    t.sharpen.scale0 = t.sharpen.scale1 = t.sharpen.scale2 = t.sharpen.scale3 = t.sharpen.scale4 = 4095;
    t.sharpen.positive_strength = t.sharpen.negative_strength = 4095;
    t.sharpen.enables = t.sharpen.white = t.sharpen.black = 31;
    t.sh_fc_combine = (struct pisp_be_sh_fc_combine_config) {255,255,255,255};
    for (size_t i = 0; i < PISP_BE_GAMMA_LUT_SIZE; ++i) t.gamma.lut[i] = UINT32_C(0x3fffffff);
    /* The builder must generate geometry/addresses, regardless of these. */
    memset(&t.input_buffer, 255, sizeof(t.input_buffer));
    memset(t.output_buffer, 255, sizeof(t.output_buffer));
    memset(&t.hog_buffer, 255, sizeof(t.hog_buffer));
    return t;
}

static void zero(const void *bytes, size_t length)
{
    const unsigned char *p = bytes;
    for (size_t i = 0; i < length; ++i) CHECK(!p[i]);
}

static void verify(const cc_pisp_be_configuration *c, const struct pisp_be_tiles_config *p)
{
    struct pisp_be_config t = tuning();
    const struct pisp_be_config *n = &p->config;
    CHECK((n->global.bayer_enables & t.global.bayer_enables) == t.global.bayer_enables);
    CHECK((n->global.rgb_enables & t.global.rgb_enables) == t.global.rgb_enables);
    CHECK((n->dirty_flags_bayer & t.global.bayer_enables) == t.global.bayer_enables);
    CHECK((n->dirty_flags_rgb & t.global.rgb_enables) == t.global.rgb_enables);
    CHECK(n->dirty_flags_extra & PISP_BE_DIRTY_SH_FC_COMBINE);
    CHECK(n->sdn.black_level == 65535 && n->sdn.leakage == 255 && n->sdn.noise_constant == 65535 &&
          n->sdn.noise_slope == 65535 && n->sdn.noise_constant2 == 65535 && n->sdn.noise_slope2 == 65535);
    CHECK(!memcmp(&n->cdn, &t.cdn, sizeof(t.cdn)));
    CHECK(!memcmp(&n->tonemap, &t.tonemap, sizeof(t.tonemap)));
    CHECK(!memcmp(n->cac.lut, t.cac.lut, sizeof(t.cac.lut)));
    CHECK(!memcmp(n->debin.coeffs, t.debin.coeffs, sizeof(t.debin.coeffs)));
    CHECK(n->debin.h_enable == 1 && n->debin.v_enable == 1);
    CHECK(!memcmp(n->ycbcr.coeffs, t.ycbcr.coeffs, sizeof(t.ycbcr.coeffs)));
    CHECK(!memcmp(n->ycbcr_inverse.offsets, t.ycbcr_inverse.offsets, sizeof(t.ycbcr_inverse.offsets)));
    CHECK(n->sat_control.shift_r == 0 && n->sat_control.shift_g == 1 && n->sat_control.shift_b == 2);
    CHECK(n->false_colour.distance == 2 && n->sh_fc_combine.y_factor == 255 && n->sh_fc_combine.c2_factor == 255);
    CHECK(n->sharpen.kernel4[12] == 31 && n->sharpen.kernel4[4] == -32 && n->sharpen.scale3 == 4095);
    CHECK(n->sharpen.positive_strength == 4095 && n->sharpen.positive_func[8] == 65535 && n->sharpen.grey == 255);
    CHECK(!n->sdn.pad && !n->sat_control.pad && !n->sh_fc_combine.pad);
    zero(n->debin.pad, sizeof(n->debin.pad)); zero(n->false_colour.pad, sizeof(n->false_colour.pad));
    zero(n->ycbcr.pad, sizeof(n->ycbcr.pad)); zero(n->ycbcr_inverse.pad, sizeof(n->ycbcr_inverse.pad));
    zero(n->sharpen.pad0, sizeof(n->sharpen.pad0)); zero(n->sharpen.pad1, sizeof(n->sharpen.pad1));
    zero(n->sharpen.pad2, sizeof(n->sharpen.pad2)); zero(n->sharpen.pad3, sizeof(n->sharpen.pad3));
    zero(n->sharpen.pad4, sizeof(n->sharpen.pad4));
    CHECK(!n->sharpen.pad5 && !n->sharpen.pad6 && !n->sharpen.pad7 && !n->sharpen.pad8 && !n->sharpen.pad9);
    zero(&n->input_buffer, sizeof(n->input_buffer)); zero(n->output_buffer, sizeof(n->output_buffer));
    zero(&n->hog_buffer, sizeof(n->hog_buffer));
    CHECK(p->num_tiles > 1);
    for (uint32_t i = 0; i < p->num_tiles; ++i) {
        const struct pisp_tile *tile = &p->tiles[i];
        uint32_t x = (uint32_t)tile->input_offset_x + n->cac_extra.offset_x;
        uint32_t y = (uint32_t)tile->input_offset_y + n->cac_extra.offset_y;
        CHECK(tile->cac_grid_offset_x == x * n->cac.grid_step_x);
        CHECK(tile->cac_grid_offset_y == y * n->cac.grid_step_y);
        CHECK((uint64_t)(x + tile->input_width - 1U) * n->cac.grid_step_x < UINT32_C(8388608));
        CHECK((uint64_t)(y + tile->input_height - 1U) * n->cac.grid_step_y < UINT32_C(8388608));
        CHECK((uint32_t)tile->input_offset_x + tile->input_width <= c->input.width);
    }
}

static void test_blocks(void)
{
    cc_pisp_be_configuration c = configuration();
    struct pisp_be_config t = tuning();
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_configuration(&c, &t, &p));
    CHECK(p.config.cac.grid_step_x == 5115 && p.config.cac.grid_step_y == 6808);
    verify(&c, &p);
    t.cac.grid_step_x = 5000; t.cac.grid_step_y = 6000;
    t.cac_extra.offset_x = 10; t.cac_extra.offset_y = 20;
    OK(cc_pisp_be_build_configuration(&c, &t, &p));
    CHECK(p.config.cac.grid_step_x == 5000 && p.config.cac_extra.offset_y == 20);
    verify(&c, &p);
    CHECK(p.tiles[0].cac_grid_offset_x >= 50000 && p.tiles[0].cac_grid_offset_y >= 120000);
    /* A later job disabling a block must not retain its grid or enables. */
    OK(cc_pisp_be_build_configuration(&c, NULL, &p));
    CHECK(!(p.config.global.bayer_enables & t.global.bayer_enables));
    CHECK(!(p.config.global.rgb_enables & t.global.rgb_enables));
    for (uint32_t i = 0; i < p.num_tiles; ++i) CHECK(!p.tiles[i].cac_grid_offset_x && !p.tiles[i].cac_grid_offset_y);
}

static void rejected(const cc_pisp_be_configuration *c, const struct pisp_be_config *t, int error)
{
    struct pisp_be_tiles_config p;
    memset(&p, 0xa5, sizeof(p));
    struct pisp_be_tiles_config before = p;
    CHECK(cc_pisp_be_build_configuration(c, t, &p) == -error);
    CHECK(!memcmp(&p, &before, sizeof(p)));
}

static void test_limits(void)
{
    cc_pisp_be_configuration c = configuration();
    for (int test = 0; test < 46; ++test) {
        struct pisp_be_config t = tuning();
        switch (test) {
        case 0: t.cac.lut[0][0][0][0] = -65; break;
        case 1: t.cac.lut[8][8][1][1] = 64; break;
        case 2: t.cac.grid_step_x = 65535; break;
        case 3: t.cac.grid_step_y = 65535; break;
        case 4: t.cac_extra.offset_x = 65535; break;
        case 5: t.cac_extra.offset_y = 65535; break;
        case 6: t.debin.h_enable = -1; break;
        case 7: t.debin.v_enable = 2; break;
        case 8: t.tonemap.iir_strength = 4096; break;
        case 9: t.tonemap.strength = 4096; break;
        case 10: t.sat_control.shift_r = 3; break;
        case 11: t.sat_control.shift_g = 3; break;
        case 12: t.sat_control.shift_b = 3; break;
        case 13: t.false_colour.distance = 0; break;
        case 14: t.false_colour.distance = 3; break;
        case 15: t.sharpen.kernel0[0] = -33; break;
        case 16: t.sharpen.kernel1[12] = 32; break;
        case 17: t.sharpen.kernel2[13] = 1; break; /* Positive sum 32. */
        case 18: t.sharpen.kernel3[13] = -1; break; /* Negative sum -33. */
        case 19: t.sharpen.kernel4[12] = -32; break; /* Negative sum -64. */
        case 20: t.sharpen.enables = 32; break;
        case 21: t.sharpen.white = 32; break;
        case 22: t.sharpen.black = 32; break;
        case 23: t.sharpen.positive_strength = 4096; break;
        case 24: t.sharpen.negative_strength = 4096; break;
        case 25: t.sharpen.threshold_slope0 = 4096; break;
        case 26: t.sharpen.threshold_slope1 = 4096; break;
        case 27: t.sharpen.threshold_slope2 = 4096; break;
        case 28: t.sharpen.threshold_slope3 = 4096; break;
        case 29: t.sharpen.threshold_slope4 = 4096; break;
        case 30: t.sharpen.scale0 = 4096; break;
        case 31: t.sharpen.scale1 = 4096; break;
        case 32: t.sharpen.scale2 = 4096; break;
        case 33: t.sharpen.scale3 = 4096; break;
        case 34: t.sharpen.scale4 = 4096; break;
        case 35: t.ycbcr.coeffs[0] = -8193; break;
        case 36: t.ycbcr_inverse.coeffs[8] = 8192; break;
        case 37: t.ycbcr.offsets[1] = 67108864; break;
        case 38: t.ycbcr_inverse.offsets[2] = -67108865; break;
        case 39: t.ccm.offsets[0] = -67108865; break;
        case 40: t.ccm.offsets[1] = 67108864; break;
        case 41: t.gamma.lut[63] = UINT32_C(0x40000000); break;
        case 42: t.global.bayer_enables |= PISP_BE_BAYER_ENABLE_TDN; break;
        case 43: t.global.rgb_enables &= ~(uint32_t)PISP_BE_RGB_ENABLE_YCBCR; break;
        case 44: t.global.rgb_enables &= ~(uint32_t)PISP_BE_RGB_ENABLE_YCBCR_INVERSE; break;
        case 45: t.global.rgb_enables &= ~(uint32_t)(PISP_BE_RGB_ENABLE_YCBCR | PISP_BE_RGB_ENABLE_YCBCR_INVERSE); break;
        }
        rejected(&c, &t, test >= 42 ? EINVAL : ERANGE);
    }
    /* Half of a matrix pair is also rejected when the filter blocks are off. */
    struct pisp_be_config t = {0};
    t.global.rgb_enables = PISP_BE_RGB_ENABLE_YCBCR;
    rejected(&c, &t, EINVAL);
    t.global.rgb_enables = PISP_BE_RGB_ENABLE_YCBCR_INVERSE;
    rejected(&c, &t, EINVAL);
    /* Auto-sized full CAC cells need more than 16 pixels per cell. A caller
     * may instead use a larger grid covering a small input subregion. */
    t = (struct pisp_be_config) {0}; t.global.bayer_enables = PISP_BE_BAYER_ENABLE_CAC;
    c = (cc_pisp_be_configuration) { .input = image(128, 128, V4L2_PIX_FMT_SRGGB16, 2),
                                    .output = image(128, 128, V4L2_PIX_FMT_RGB24, 3) };
    rejected(&c, &t, ERANGE);
    t.cac.grid_step_x = t.cac.grid_step_y = 65535;
    struct pisp_be_tiles_config p;
    OK(cc_pisp_be_build_configuration(&c, &t, &p));
}

static void test_revisions(void)
{
    cc_pisp_be_configuration c = configuration();
    struct pisp_be_config t = tuning();
    struct pisp_be_tiles_config p;
    c.hardware_version = CC_PISP_BE_VERSION_BCM2712_C0;
    OK(cc_pisp_be_build_configuration(&c, &t, &p));
    t.ccm.offsets[0] = -268435456; t.ycbcr.offsets[1] = 268435455; t.ycbcr_inverse.offsets[2] = -268435456;
    rejected(&c, &t, ERANGE);
    c.hardware_version = CC_PISP_BE_VERSION_BCM2712_D0;
    OK(cc_pisp_be_build_configuration(&c, &t, &p));
    CHECK(p.config.ccm.offsets[0] == -268435456 && p.config.ycbcr.offsets[1] == 268435455);
    t.ycbcr_inverse.offsets[2] = -268435457;
    rejected(&c, &t, ERANGE);
    t = tuning(); t.ccm.offsets[1] = 268435456;
    rejected(&c, &t, ERANGE);
    c.hardware_version = CC_PISP_BE_VERSION_BCM2712_D0 + 1U;
    rejected(&c, NULL, ENOTSUP);
}

int main(void)
{
    test_blocks(); test_limits(); test_revisions();
    puts("PiSP single-frame processing, CAC tile grids, register limits and C0/D0 revisions passed");
    return 0;
}
