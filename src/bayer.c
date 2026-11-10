/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"
#include <errno.h>
#include <linux/media-bus-format.h>

/* Columns follow cc_camera_cfa. Each memory row preserves depth and packing. */
static const uint32_t codes[5][5]={
#define CODES(bits) {MEDIA_BUS_FMT_SRGGB##bits##_1X##bits, MEDIA_BUS_FMT_SGRBG##bits##_1X##bits, \
    MEDIA_BUS_FMT_SGBRG##bits##_1X##bits, MEDIA_BUS_FMT_SBGGR##bits##_1X##bits, MEDIA_BUS_FMT_Y##bits##_1X##bits}
    CODES(8), CODES(10), CODES(12), CODES(14), CODES(16)
#undef CODES
};
static const uint32_t formats[8][4]={
#define FORMATS(suffix) {V4L2_PIX_FMT_SRGGB##suffix, V4L2_PIX_FMT_SGRBG##suffix, \
    V4L2_PIX_FMT_SGBRG##suffix, V4L2_PIX_FMT_SBGGR##suffix}
    FORMATS(8), FORMATS(10), FORMATS(10P), FORMATS(12), FORMATS(12P),
    FORMATS(14), FORMATS(14P), FORMATS(16)
#undef FORMATS
};
static const uint32_t depths[8]={8,10,10,12,12,14,14,16};
static const struct { uint32_t fourcc,depth; } mono[]={
    {V4L2_PIX_FMT_GREY,8}, {V4L2_PIX_FMT_Y10,10}, {V4L2_PIX_FMT_Y10P,10},
    {V4L2_PIX_FMT_Y12,12}, {CC_FOURCC('Y','1','2','P'),12},
    {V4L2_PIX_FMT_Y14,14}, {V4L2_PIX_FMT_Y16,16}
};

uint32_t cc_bayer_code(uint32_t depth, enum cc_camera_cfa cfa)
{
    if (depth<8 || depth>16 || depth%2 || (unsigned)cfa>CC_CAMERA_CFA_MONO) return 0;
    return codes[(depth-8)/2][cfa];
}

int cc_bayer_code_info(uint32_t code, uint32_t *depth, enum cc_camera_cfa *cfa)
{
    for (uint32_t d=0; d<5; ++d) for (uint32_t c=0; c<5; ++c)
        if (codes[d][c]==code) {
            if (depth) *depth=8+2*d;
            if (cfa) *cfa=(enum cc_camera_cfa)c;
            return 0;
        }
    return -ENOTSUP;
}

uint32_t cc_bayer_mbus(uint32_t fourcc)
{
    for (size_t f=0; f<8; ++f) for (uint32_t c=0; c<4; ++c)
        if (formats[f][c]==fourcc) return cc_bayer_code(depths[f],(enum cc_camera_cfa)c);
    for (size_t m=0; m<sizeof(mono)/sizeof(mono[0]); ++m)
        if (mono[m].fourcc==fourcc) return cc_bayer_code(mono[m].depth,CC_CAMERA_CFA_MONO);
    return 0;
}

uint32_t cc_bayer_fourcc(uint32_t code, bool packed)
{
    uint32_t depth; enum cc_camera_cfa cfa;
    if (cc_bayer_code_info(code,&depth,&cfa)) return 0;
    if (cfa==CC_CAMERA_CFA_MONO) {
        uint32_t selected=0;
        for (size_t i=0; i<sizeof(mono)/sizeof(mono[0]); ++i) if (mono[i].depth==depth) {
            if (!selected || packed) selected=mono[i].fourcc;
        }
        return selected;
    }
    size_t row=depth==8 ? 0:depth==10 ? (packed ? 2U:1U):depth==12 ? (packed ? 4U:3U):
        depth==14 ? (packed ? 6U:5U):7U;
    return formats[row][cfa];
}

/* Change Bayer order without changing sample depth or memory packing. Mono
 * stays mono; the compressed PiSP formats carry their order in the last byte. */
uint32_t cc_bayer_reorder(uint32_t fourcc, enum cc_camera_cfa cfa)
{
    if ((unsigned)cfa>CC_CAMERA_CFA_MONO) return 0;
    for (size_t f=0; f<8; ++f) for (size_t c=0; c<4; ++c)
        if (formats[f][c]==fourcc) return cfa==CC_CAMERA_CFA_MONO ? 0:formats[f][cfa];
    for (size_t m=0; m<sizeof(mono)/sizeof(mono[0]); ++m)
        if (mono[m].fourcc==fourcc) return cfa==CC_CAMERA_CFA_MONO ? fourcc:0;
    for (uint32_t mode='1'; mode<='3'; ++mode) {
        uint32_t prefix=CC_FOURCC('P','C',mode,0);
        if ((fourcc&0xffffffU)!=prefix) continue;
        static const char orders[]={'R','G','g','B','M'};
        uint32_t old=fourcc>>24;
        for (size_t c=0; c<sizeof(orders); ++c)
            if (old==(uint32_t)orders[c]) {
                if ((c==CC_CAMERA_CFA_MONO)!=(cfa==CC_CAMERA_CFA_MONO)) return 0;
                return prefix|((uint32_t)orders[cfa]<<24);
            }
    }
    return 0;
}
