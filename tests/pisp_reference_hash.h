/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_TEST_PISP_REFERENCE_HASH_H
#define CCAMERA_TEST_PISP_REFERENCE_HASH_H
#include <ccamera/pisp_be.h>

/* Hash logical field values in a canonical byte order, independent of host
 * endianness and C padding. Used with reference-generated tile fixtures. */
static uint64_t cc_pisp_hash_value(uint64_t hash, uint32_t value)
{
    for (uint32_t byte = 0; byte < 4; ++byte) {
        hash ^= (value >> (byte * 8)) & 255U;
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}
static uint64_t cc_pisp_tile_hash(const struct pisp_tile *tiles, uint32_t count)
{
    uint64_t hash = cc_pisp_hash_value(UINT64_C(14695981039346656037), count);
    for (uint32_t i = 0; i < count; ++i) {
        const struct pisp_tile *t = &tiles[i];
#define HASH(field) hash = cc_pisp_hash_value(hash, t->field)
        HASH(edge);
        for (uint32_t n = 0; n < 3; ++n) HASH(pad0[n]);
        HASH(input_addr_offset); HASH(input_addr_offset2);
        HASH(input_offset_x); HASH(input_offset_y); HASH(input_width); HASH(input_height);
        HASH(tdn_input_addr_offset); HASH(tdn_output_addr_offset);
        HASH(stitch_input_addr_offset); HASH(stitch_output_addr_offset);
        HASH(lsc_grid_offset_x); HASH(lsc_grid_offset_y); HASH(cac_grid_offset_x); HASH(cac_grid_offset_y);
        for (uint32_t n = 0; n < 2; ++n) {
            HASH(crop_x_start[n]); HASH(crop_x_end[n]); HASH(crop_y_start[n]); HASH(crop_y_end[n]);
            HASH(resample_in_width[n]); HASH(resample_in_height[n]);
            HASH(output_offset_x[n]); HASH(output_offset_y[n]); HASH(output_width[n]); HASH(output_height[n]);
            HASH(output_addr_offset[n]); HASH(output_addr_offset2[n]);
        }
        for (uint32_t n = 0; n < 6; ++n) {
            HASH(downscale_phase_x[n]); HASH(downscale_phase_y[n]); HASH(resample_phase_x[n]); HASH(resample_phase_y[n]);
        }
        HASH(output_hog_addr_offset);
#undef HASH
    }
    return hash;
}
#endif
