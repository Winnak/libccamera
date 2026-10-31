/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/embedded.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "embedded_reference_vectors.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
static uint64_t hash_value(uint64_t hash, uint32_t value)
{
    for (unsigned byte = 0; byte < 4; ++byte) {
        hash ^= (value >> (byte * 8)) & 255U; hash *= UINT64_C(1099511628211);
    }
    return hash;
}
static const cc_sensor_model *model(const char *name)
{
    const cc_sensor_model *m; OK(cc_sensor_lookup(name, &m)); return m;
}
/* Independent wire encoder: each CSI-2 group carries 4/2/4 full bytes plus
 * 1/1/3 dummy bytes for RAW10/12/14. The tagged logical stream is separate. */
static size_t packed_offset(uint32_t bpp, size_t logical)
{
    if (bpp == 10) return logical / 4 * 5 + logical % 4;
    if (bpp == 12) return logical / 2 * 3 + logical % 2;
    if (bpp == 14) return logical / 4 * 7 + logical % 4;
    return logical;
}
static size_t smia_line(uint8_t *out, uint32_t bpp, const cc_smia_register *registers, size_t count)
{
    uint8_t logical[1 + CC_SMIA_MAX_REGISTERS * 6 + 2];
    size_t n = 0; logical[n++] = 0x0a;
    for (size_t i = 0; i < count; ++i) {
        logical[n++] = 0xaa; logical[n++] = (uint8_t)(registers[i].address >> 8);
        logical[n++] = 0xa5; logical[n++] = (uint8_t)registers[i].address;
        logical[n++] = 0x5a; logical[n++] = registers[i].value;
    }
    logical[n++] = 0x07; logical[n++] = 0x07;
    size_t size = packed_offset(bpp, n - 1) + 1;
    memset(out, 0x55, size);
    for (size_t i = 0; i < n; ++i) out[packed_offset(bpp, i)] = logical[i];
    return size;
}
static void test_smia(void)
{
    uint64_t hash = UINT64_C(1469598103934665603);
    for (uint32_t bpp = 8; bpp <= 14; bpp += 2) {
        for (unsigned round = 0; round < 64; ++round) {
            cc_smia_register expected[64], got[64] = {{0,0}};
            size_t count = round + 1;
            for (size_t i = 0; i < count; ++i)
                expected[i] = (cc_smia_register){(uint16_t)(round * 967U + i * 43U), (uint8_t)(i * 17U + round)};
            uint8_t bytes[1600];
            size_t first_count = count / 2;
            size_t first = smia_line(bytes + 1, bpp, expected, first_count);
            size_t stride = first + 13;
            memset(bytes + 1 + first, 0, stride - first);
            size_t size = stride + smia_line(bytes + 1 + stride, bpp, expected + first_count, count - first_count);
            for (size_t i = 0; i < count; ++i) got[count - 1 - i] = (cc_smia_register){expected[i].address, 0xe7};
            cc_smia_layout layout = {bpp, (uint32_t)stride, 2};
            /* Fixed stride must accommodate each complete logical line. */
            if (size - stride > stride) {
                stride = size - stride + 13;
                first = smia_line(bytes + 1, bpp, expected, first_count);
                memset(bytes + 1 + first, 0, stride - first);
                size = stride + smia_line(bytes + 1 + stride, bpp, expected + first_count, count - first_count);
                layout.line_length_bytes = (uint32_t)stride;
            }
            OK(cc_smia_parse(&layout, bytes + 1, size, got, count));
            for (size_t i = 0; i < count; ++i) hash = hash_value(hash, got[i].value);
            for (size_t i = 0; i < count; ++i) CHECK(got[count - 1 - i].value == expected[i].value);
            layout.line_length_bytes = 0;
            for (size_t i = 0; i < count; ++i) got[i].value = 0xe7;
            OK(cc_smia_parse(&layout, bytes + 1, size, got, count));
            layout.num_lines = 1;
            cc_smia_register saved[64]; memcpy(saved, got, sizeof(saved));
            CHECK(cc_smia_parse(&layout, bytes + 1, size, got, count) == -ENODATA);
            CHECK(!memcmp(got, saved, sizeof(saved)));
        }
        cc_smia_layout layout = {bpp, 0, 0};
        cc_smia_register expected[] = {{0x1234, 17}, {0xabcd, 93}}, got[] = {{0x1234,0xee},{0xabcd,0xee}};
        uint8_t bytes[80]; size_t size = smia_line(bytes, bpp, expected, 2);
        size_t last_value = packed_offset(bpp, 12);
        for (size_t length = 0; length <= last_value; ++length) {
            uint8_t *short_data = malloc(length ? length : 1); CHECK(short_data);
            memcpy(short_data, bytes, length);
            CHECK(cc_smia_parse(&layout, short_data, length, got, 2) < 0);
            CHECK(got[0].value == 0xee && got[1].value == 0xee);
            free(short_data);
        }
        OK(cc_smia_parse(&layout, bytes, size, got, 2));
        for (size_t i = 0; i < last_value; ++i) {
            if (bytes[i] != 0x55) continue;
            bytes[i] = 0;
            CHECK(cc_smia_parse(&layout, bytes, size, got, 2) == -EBADMSG);
            bytes[i] = 0x55;
        }
        cc_smia_register duplicate[] = {{0x1234,17}, {0x1234,18}};
        size = smia_line(bytes, bpp, duplicate, 2);
        CHECK(cc_smia_parse(&layout, bytes, size, got, 2) == -ENODATA);
        cc_smia_register repeated[] = {{0x1234,17}, {0x1234,18}, {0xabcd,93}};
        size = smia_line(bytes, bpp, repeated, 3);
        OK(cc_smia_parse(&layout, bytes, size, got, 2));
        CHECK(got[0].value == 18 && got[1].value == 93);
        cc_smia_register missing = {0x8888,77};
        size = smia_line(bytes, bpp, expected, 2);
        OK(cc_smia_parse(&layout, bytes, size, got, 2));
        CHECK(cc_smia_parse(&layout, bytes, size, &missing, 1) == -ENODATA && missing.value == 77);
        bytes[0] = 0; CHECK(cc_smia_parse(&layout, bytes, size, got, 2) == -EBADMSG);
    }
    cc_smia_layout layout = {8,0,0};
    cc_smia_register got[] = {{1,7},{1,8}};
    CHECK(cc_smia_parse(&layout, NULL, 0, got, 2) == -EINVAL);
    CHECK(cc_smia_parse(&layout, NULL, 0, got, 1) == -ENODATA);
    CHECK(cc_smia_parse(NULL, NULL, 0, got, 1) == -EINVAL);
    CHECK(cc_smia_parse(&layout, NULL, 1, got, 1) == -EINVAL);
    CHECK(cc_smia_parse(&layout, NULL, 0, got, 65) == -EINVAL);
    layout.bits_per_pixel = 11; CHECK(cc_smia_parse(&layout, NULL, 0, got, 1) == -ENOTSUP);
    /* Register auto-increment and skip tags, including 16-bit wrap rejection. */
    const uint8_t increment[] = {0x0a,0xaa,0x12,0xa5,0x34,0x55,0,0x5a,17,0x5a,18};
    layout.bits_per_pixel = 8; got[0].address = 0x1235; got[1].address = 0x1236;
    OK(cc_smia_parse(&layout, increment, sizeof(increment), got, 2));
    CHECK(got[0].value == 17 && got[1].value == 18);
    uint8_t wrap[] = {0x0a,0xaa,0xff,0xa5,0xff,0x55,0,0x5a,19};
    CHECK(cc_smia_parse(&layout, wrap, sizeof(wrap), got, 1) == -EBADMSG);
    wrap[5] = 0x5a; got[0].address = 0xffff;
    OK(cc_smia_parse(&layout, wrap, sizeof(wrap), got, 1)); CHECK(got[0].value == 0);
    CHECK(hash == cc_embedded_smia_reference_hash);
}
static void test_status(void)
{
    const char *names[] = {"imx477", "imx519", "imx708_wide_noir"};
    for (size_t sensor = 0; sensor < 3; ++sensor) {
        const cc_sensor_model *m = model(names[sensor]);
        CHECK(cc_sensor_get_properties(m)->features & CC_SENSOR_EMBEDDED);
        for (uint32_t bpp = 8; bpp <= 14; bpp += 2) {
            for (unsigned frame = 0; frame < 5; ++frame) {
                cc_smia_register registers[] = {{0x0202,0x12},{0x0203,0x34},{0x0204,2},{0x0205,0},
                    {0x0340,0x45},{0x0341,0x67},{0x0342,0x23},{0x0343,0x45},{0x013a,0}};
                const uint8_t temperatures[] = {0,127,128,255,33};
                const int32_t expected[] = {0,80,-20,-1,33};
                registers[8].value = temperatures[frame];
                /* Reverse register order on alternate frames; no cached offsets. */
                if (frame & 1U) {
                    for (size_t i = 0; i < 4; ++i) {
                        cc_smia_register swap = registers[i]; registers[i] = registers[7-i]; registers[7-i] = swap;
                    }
                }
                uint8_t bytes[120]; size_t size = smia_line(bytes, bpp, registers, sensor == 1 ? 8 : 9);
                cc_smia_layout layout = {bpp,0,2}; cc_sensor_embedded_status status;
                OK(cc_sensor_parse_embedded_status(m, &layout, bytes, size, &status));
                CHECK(status.exposure_lines == 0x1234 && status.analogue_gain_code == 512 && status.analogue_gain == 2);
                CHECK(status.line_length_pck == 0x2345 && status.frame_length_lines == 0x4567);
                CHECK(status.flags == (sensor == 1 ? 0 : CC_SENSOR_STATUS_TEMPERATURE));
                CHECK(status.temperature_c == (sensor == 1 ? 0 : expected[frame]));
                cc_sensor_embedded_status saved = status;
                CHECK(cc_sensor_parse_embedded_status(m, &layout, bytes, 2, &status) < 0);
                CHECK(!memcmp(&status,&saved,sizeof(saved)));
            }
        }
    }
    cc_sensor_embedded_status status;
    cc_smia_layout layout = {10,0,2};
    CHECK(cc_sensor_parse_embedded_status(model("imx219"), &layout, NULL, 0, &status) == -ENOTSUP);
    CHECK(!(cc_sensor_get_properties(model("imx219"))->features & CC_SENSOR_EMBEDDED));
    CHECK(cc_sensor_parse_embedded_status(NULL, &layout, NULL, 0, &status) == -EINVAL);
    cc_smia_register registers[] = {{0x0202,0},{0x0203,1},{0x0204,4},{0x0205,0},
        {0x0340,0},{0x0341,1},{0x0342,0},{0x0343,1}};
    uint8_t bytes[100]; size_t size = smia_line(bytes, 10, registers, 8);
    memset(&status, 0xe7, sizeof(status)); cc_sensor_embedded_status saved = status;
    CHECK(cc_sensor_parse_embedded_status(model("imx519"), &layout, bytes, size, &status) == -ERANGE);
    CHECK(!memcmp(&status,&saved,sizeof(saved)));
    registers[2].value = 0; registers[7].value = 0;
    size = smia_line(bytes, 10, registers, 8);
    CHECK(cc_sensor_parse_embedded_status(model("imx519"), &layout, bytes, size, &status) == -ERANGE);
}
static void phase_entry(uint8_t *out, unsigned confidence, int phase)
{
    unsigned p = (unsigned)(phase < 0 ? phase + 2048 : phase);
    out[0] = (uint8_t)(confidence >> 3);
    out[1] = (uint8_t)((confidence % 8U) * 32U + p / 64U);
    out[2] = (uint8_t)((p % 64U) * 4U);
}
static void test_pdaf(void)
{
    uint64_t hash = UINT64_C(1469598103934665603);
    const cc_sensor_model *m = model("imx708");
    CHECK(cc_sensor_get_properties(m)->features & CC_SENSOR_PDAF);
    for (uint32_t bpp = 10; bpp <= 14; bpp += 2) {
        size_t step = bpp / 2U, size = 194 * step;
        uint8_t *allocation = malloc(size + 1); CHECK(allocation); uint8_t *bytes = allocation + 1;
        for (int base = -1024; base < 1024; base += 191) {
            memset(bytes, 0xe7, size); bytes[0] = 0; bytes[1] = 0x3f;
            for (size_t i = 0; i < CC_SENSOR_PDAF_REGIONS; ++i) {
                int phase = (base + (int)i + 1024) % 2048 - 1024;
                phase_entry(bytes + (i + 2) * step, (unsigned)(i * 11U) % 2048U, phase);
            }
            cc_sensor_pdaf result;
            OK(cc_sensor_parse_pdaf(m,bpp,bytes,size,&result));
            CHECK(result.rows == 12 && result.cols == 16);
            for (size_t i = 0; i < CC_SENSOR_PDAF_REGIONS; ++i) {
                CHECK(result.regions[i].confidence == (i * 11U) % 2048U);
                CHECK(result.regions[i].phase == (result.regions[i].confidence ? (base + (int)i + 1024) % 2048 - 1024 : 0));
                hash = hash_value(hash, result.regions[i].confidence);
                hash = hash_value(hash, (uint32_t)(int32_t)result.regions[i].phase);
            }
            cc_sensor_pdaf saved = result;
            CHECK(cc_sensor_parse_pdaf(m,bpp,bytes,size-1,&result) == -EBADMSG);
            CHECK(!memcmp(&result,&saved,sizeof(saved)));
            bytes[0] = 1; CHECK(cc_sensor_parse_pdaf(m,bpp,bytes,size,&result) == -EBADMSG); bytes[0] = 0;
            bytes[1] = 0x40; CHECK(cc_sensor_parse_pdaf(m,bpp,bytes,size,&result) == -EBADMSG);
        }
        free(allocation);
    }
    cc_sensor_pdaf result;
    CHECK(cc_sensor_parse_pdaf(m,11,NULL,0,&result) == -ENOTSUP);
    CHECK(cc_sensor_parse_pdaf(model("imx477"),10,NULL,0,&result) == -ENOTSUP);
    CHECK(cc_sensor_parse_pdaf(NULL,10,NULL,0,&result) == -EINVAL);
    CHECK(cc_sensor_parse_pdaf(m,10,NULL,1,&result) == -EINVAL);
    CHECK(hash == cc_embedded_pdaf_reference_hash);
}
static void histogram_entry(uint8_t *out, uint32_t count)
{
    out[0] = (uint8_t)(count / 16384U); out[1] = (uint8_t)(count / 64U);
    out[2] = (uint8_t)(count % 64U * 4U); out[3] = 0x55;
}
static void test_histogram(void)
{
    uint64_t hash = UINT64_C(1469598103934665603);
    const cc_sensor_model *m = model("imx708");
    CHECK(cc_sensor_get_properties(m)->features & CC_SENSOR_HDR_STATISTICS);
    for (uint32_t bpp = 10; bpp <= 14; bpp += 2) {
        size_t step = bpp / 2U, size = 144 * step;
        uint8_t *bytes = malloc(size); CHECK(bytes);
        for (unsigned scenario = 0; scenario < 4; ++scenario) {
            memset(bytes, 0, size);
            uint64_t total = 0, sum = 0; uint32_t expected[128];
            for (size_t i = 0; i < 137; ++i) {
                uint32_t count = scenario == 0 ? 0 : scenario == 1 ? (uint32_t)(i*23U) :
                    scenario == 2 ? UINT32_C(4194303) : i == 136 ? 1U : 0;
                histogram_entry(bytes + i * step, count);
                if (i < 128) expected[i] = count / 4U;
                if (!i) continue;
                uint64_t centre = i < 128 ? i * 512U + 256U : UINT64_C(196608) >> (145 - i);
                sum += count * centre; total += count;
            }
            uint32_t overlap = expected[1];
            bytes[137*step] = (uint8_t)(overlap / 4096U);
            bytes[137*step+1] = (uint8_t)(overlap / 16U);
            bytes[137*step+2] = (uint8_t)(overlap % 16U * 16U);
            cc_sensor_hdr_statistics result; memset(&result,0xe7,sizeof(result));
            cc_sensor_hdr_statistics saved = result;
            int ret = cc_sensor_parse_hdr_statistics(m,bpp,bytes,size,&result);
            if (!scenario) { CHECK(ret == -ENODATA && !memcmp(&result,&saved,sizeof(saved))); continue; }
            OK(ret); CHECK(!memcmp(result.histogram,expected,sizeof(expected)));
            CHECK(result.counted == total && result.average == sum / total && result.average < 65536);
            if (scenario != 2) {
                for (size_t i = 0; i < 128; ++i) hash = hash_value(hash, result.histogram[i]);
                hash = hash_value(hash, result.average);
            }
            saved = result;
            bytes[3] = 0; CHECK(cc_sensor_parse_hdr_statistics(m,bpp,bytes,size,&result) == -EBADMSG);
            CHECK(!memcmp(&result,&saved,sizeof(saved))); bytes[3] = 0x55;
            bytes[137*step+2] ^= 0x10;
            CHECK(cc_sensor_parse_hdr_statistics(m,bpp,bytes,size,&result) == -EBADMSG);
            CHECK(cc_sensor_parse_hdr_statistics(m,bpp,bytes,size-1,&result) == -EBADMSG);
        }
        free(bytes);
    }
    CHECK(hash == cc_embedded_hdr_reference_hash);
}
static void test_decoded_autofocus(void)
{
    for (uint32_t bpp = 10; bpp <= 14; bpp += 2) {
        cc_af_tuning tuning; cc_af_reference_tuning(&tuning);
        cc_af *decoded, *direct; OK(cc_af_create(&tuning,&decoded)); OK(cc_af_create(&tuning,&direct));
        cc_control_info lens = {.id=V4L2_CID_FOCUS_ABSOLUTE,.type=V4L2_CTRL_TYPE_INTEGER,
            .minimum=0,.maximum=1023,.step=1,.elems=1};
        cc_rectangle region = {0,0,4608,2592};
        OK(cc_af_configure(decoded,&region,&lens)); OK(cc_af_configure(direct,&region,&lens));
        cc_af_controls controls = {.flags=CC_AF_SET_MODE|CC_AF_TRIGGER,.mode=CC_AF_AUTO};
        OK(cc_af_set_controls(decoded,&controls)); OK(cc_af_set_controls(direct,&controls));
        double position = 0; bool focused = false;
        for (unsigned frame = 0; frame < 180; ++frame) {
            uint8_t bytes[194*7] = {0}; cc_af_pdaf original[CC_SENSOR_PDAF_REGIONS];
            int phase = (int)((position-4.3)*50);
            for (size_t i = 0; i < CC_SENSOR_PDAF_REGIONS; ++i) {
                phase_entry(bytes+(i+2)*(bpp/2U),128,phase);
                original[i] = (cc_af_pdaf){128,(int16_t)phase};
            }
            cc_sensor_pdaf pdaf; OK(cc_sensor_parse_pdaf(model("imx708"),bpp,bytes,194*(bpp/2U),&pdaf));
            cc_af_pdaf_grid decoded_grid = {pdaf.rows,pdaf.cols,pdaf.regions};
            cc_af_pdaf_grid direct_grid = {12,16,original};
            cc_af_status a,b; OK(cc_af_prepare(decoded,&decoded_grid,&a)); OK(cc_af_prepare(direct,&direct_grid,&b));
            CHECK(a.flags == b.flags && a.state == b.state && a.pause_state == b.pause_state &&
                a.lens_setting == b.lens_setting && a.lens_position == b.lens_position);
            if (a.flags & CC_AF_LENS_VALID) position = a.lens_position;
            double delta = position-4.3; uint64_t focus[12];
            for (size_t i = 0; i < 12; ++i) focus[i] = (uint64_t)(100000/(1+delta*delta));
            cc_af_statistics statistics = {.focus_rows=3,.focus_cols=4,.focus=focus};
            OK(cc_af_process(decoded,&statistics)); OK(cc_af_process(direct,&statistics));
            if (a.state == CC_AF_FOCUSED) {
                CHECK(position > 3.95 && position < 4.65); focused = true; break;
            }
        }
        CHECK(focused); cc_af_destroy(decoded); cc_af_destroy(direct);
    }
}
int main(void)
{
    test_smia(); test_status(); test_pdaf(); test_histogram(); test_decoded_autofocus();
    puts("Bounded SMIA register reports, IMX708 phase/HDR decoding and decoded-phase AF convergence passed");
    return 0;
}
