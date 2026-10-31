/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* SMIA and IMX708 formats derived from the libcamera RPi helpers.
 * Copyright (C) 2019-2022, Raspberry Pi Ltd
 * C17 port with bounded reads, per-frame validation and checked accumulation.
 */
#include <ccamera/embedded.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

static bool packing(uint32_t bpp)
{
    return bpp == 8 || bpp == 10 || bpp == 12 || bpp == 14;
}
static bool dummy(uint32_t bpp, size_t offset)
{
    return (bpp == 10 && offset % 5 == 4) || (bpp == 12 && offset % 3 == 2) ||
           (bpp == 14 && offset % 7 >= 4);
}

int cc_smia_parse(const cc_smia_layout *layout, const void *data, size_t size,
                   cc_smia_register *registers, size_t count)
{
    if (!layout || (!data && size) || !registers || !count || count > CC_SMIA_MAX_REGISTERS)
        return -EINVAL;
    if (!packing(layout->bits_per_pixel)) return -ENOTSUP;
    for (size_t i = 0; i < count; ++i)
        for (size_t j = 0; j < i; ++j)
            if (registers[i].address == registers[j].address) return -EINVAL;
    if (!size) return -ENODATA;
    const uint8_t *bytes = data;
    if (bytes[0] != 0x0a) return -EBADMSG;
    uint8_t values[CC_SMIA_MAX_REGISTERS] = {0};
    bool found[CC_SMIA_MAX_REGISTERS] = {false};
    size_t position = 1, start = 0, line = 1, done = 0;
    uint32_t address = 0;
    for (;;) {
        size_t limit = size;
        if (layout->line_length_bytes && layout->line_length_bytes < size - start)
            limit = start + layout->line_length_bytes;
        if (position >= limit) return -EBADMSG;
        uint8_t tag = bytes[position++];
        while (dummy(layout->bits_per_pixel, position - start)) {
            if (position >= limit || bytes[position++] != 0x55) return -EBADMSG;
        }
        if (position >= limit) return -EBADMSG;
        uint8_t value = bytes[position++];
        switch (tag) {
        case 0x07: /* Line end, then either fixed-pitch or searched line start. */
            if (value != 0x07) return -EBADMSG;
            if (layout->num_lines && line == layout->num_lines) return -ENODATA;
            if (layout->line_length_bytes) {
                if (layout->line_length_bytes >= size - start) return -ENODATA;
                position = start + layout->line_length_bytes;
                if (bytes[position] != 0x0a) return -EBADMSG;
            } else {
                while (position < size && bytes[position] != 0x0a) ++position;
                if (position == size) return -ENODATA;
            }
            start = position++;
            ++line;
            break;
        case 0xaa: address = (address & 0xffU) | (uint32_t)value << 8; break;
        case 0xa5: address = (address & 0xff00U) | value; break;
        case 0x55:
            if (address == UINT16_MAX) return -EBADMSG;
            ++address;
            break;
        case 0x5a:
            for (size_t i = 0; i < count; ++i) {
                if (address != registers[i].address) continue;
                if (!found[i]) { found[i] = true; ++done; }
                values[i] = value;
                if (done == count) {
                    for (size_t j = 0; j < count; ++j) registers[j].value = values[j];
                    return 0;
                }
                break;
            }
            if (address == UINT16_MAX) return -EBADMSG;
            ++address;
            break;
        default: return -EBADMSG;
        }
    }
}

static int model_feature(const cc_sensor_model *model, uint32_t feature)
{
    if (!model) return -EINVAL;
    const cc_sensor_properties *properties = cc_sensor_get_properties(model);
    return properties->features & feature ? 0 : -ENOTSUP;
}
int cc_sensor_parse_embedded_status(const cc_sensor_model *model, const cc_smia_layout *layout,
                                   const void *data, size_t size, cc_sensor_embedded_status *status)
{
    if (!status) return -EINVAL;
    int ret = model_feature(model, CC_SENSOR_EMBEDDED);
    if (ret < 0) return ret;
    const cc_sensor_properties *properties = cc_sensor_get_properties(model);
    bool temperature = strcmp(properties->model, "imx519") != 0;
    cc_smia_register registers[] = {{0x0202,0}, {0x0203,0}, {0x0204,0}, {0x0205,0},
        {0x0340,0}, {0x0341,0}, {0x0342,0}, {0x0343,0}, {0x013a,0}};
    ret = cc_smia_parse(layout, data, size, registers, temperature ? 9 : 8);
    if (ret < 0) return ret;
    cc_sensor_embedded_status result = {0};
    result.exposure_lines = (uint32_t)registers[0].value * 256U + registers[1].value;
    result.analogue_gain_code = (uint32_t)registers[2].value * 256U + registers[3].value;
    result.frame_length_lines = (uint32_t)registers[4].value * 256U + registers[5].value;
    result.line_length_pck = (uint32_t)registers[6].value * 256U + registers[7].value;
    if (!result.frame_length_lines || !result.line_length_pck) return -ERANGE;
    ret = cc_sensor_gain(model, result.analogue_gain_code, &result.analogue_gain);
    if (ret < 0) return ret;
    if (temperature) {
        int value = registers[8].value;
        if (value >= 128) value -= 256;
        result.flags = CC_SENSOR_STATUS_TEMPERATURE;
        result.temperature_c = value < -20 ? -20 : value > 80 ? 80 : value;
    }
    *status = result;
    return 0;
}

static int imx708_line(const cc_sensor_model *model, uint32_t feature, uint32_t bpp,
                      const void *data, size_t size, size_t entries)
{
    if (!data && size) return -EINVAL;
    int ret = model_feature(model, feature);
    if (ret < 0) return ret;
    if (bpp != 10 && bpp != 12 && bpp != 14) return -ENOTSUP;
    return size < entries * (bpp / 2U) ? -EBADMSG : 0;
}
int cc_sensor_parse_pdaf(const cc_sensor_model *model, uint32_t bpp, const void *data, size_t size,
                         cc_sensor_pdaf *pdaf)
{
    if (!pdaf) return -EINVAL;
    int ret = imx708_line(model, CC_SENSOR_PDAF, bpp, data, size, 194);
    if (ret < 0) return ret;
    const uint8_t *bytes = data;
    if (bytes[0] != 0 || bytes[1] >= 0x40) return -EBADMSG;
    size_t step = bpp / 2U;
    cc_sensor_pdaf result = {.rows = CC_SENSOR_PDAF_ROWS, .cols = CC_SENSOR_PDAF_COLS};
    bytes += 2 * step;
    for (size_t i = 0; i < CC_SENSOR_PDAF_REGIONS; ++i, bytes += step) {
        unsigned confidence = (unsigned)bytes[0] * 8U + (bytes[1] >> 5);
        int phase = (int)(bytes[1] & 0x1f) * 64 + (bytes[2] >> 2);
        if (phase >= 1024) phase -= 2048;
        result.regions[i] = (cc_af_pdaf) {(uint16_t)confidence, confidence ? (int16_t)phase : 0};
    }
    *pdaf = result;
    return 0;
}

static uint32_t histogram_count(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] * 16384U + (uint32_t)bytes[1] * 64U + (bytes[2] >> 2);
}
int cc_sensor_parse_hdr_statistics(const cc_sensor_model *model, uint32_t bpp,
                                   const void *data, size_t size, cc_sensor_hdr_statistics *statistics)
{
    if (!statistics) return -EINVAL;
    int ret = imx708_line(model, CC_SENSOR_HDR_STATISTICS, bpp, data, size, 144);
    if (ret < 0) return ret;
    const uint8_t *bytes = data;
    size_t step = bpp / 2U;
    cc_sensor_hdr_statistics result = {0};
    uint64_t sum = 0;
    for (size_t i = 0; i < CC_SENSOR_HDR_BINS; ++i, bytes += step) {
        if (bytes[3] != 0x55) return -EBADMSG;
        uint32_t count = histogram_count(bytes);
        result.histogram[i] = count >> 2;
        if (i) {
            result.counted += count;
            sum += (uint64_t)count * (i * 512U + 256U);
        }
    }
    for (unsigned i = 0; i < 9; ++i, bytes += step) {
        if (bytes[3] != 0x55) return -EBADMSG;
        uint32_t count = histogram_count(bytes);
        result.counted += count;
        sum += (uint64_t)count * (UINT32_C(196608) >> (17U - i));
    }
    uint32_t overlap = (uint32_t)bytes[0] * 4096U + (uint32_t)bytes[1] * 16U + (bytes[2] >> 4);
    if (overlap != result.histogram[1]) return -EBADMSG;
    if (!result.counted) return -ENODATA;
    result.average = (uint32_t)(sum / result.counted);
    *statistics = result;
    return 0;
}
