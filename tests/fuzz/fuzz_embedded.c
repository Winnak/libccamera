/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include <ccamera/embedded.h>
#include <stdbool.h>

static size_t wire_offset(unsigned bpp, size_t n)
{
    if (bpp == 10) return n / 4 * 5 + n % 4;
    if (bpp == 12) return n / 2 * 3 + n % 2;
    if (bpp == 14) return n / 4 * 7 + n % 4;
    return n;
}
static void atomic_raw(const cc_sensor_model *model, const cc_smia_layout *layout,
                       const uint8_t *data, size_t size, uint8_t count)
{
    cc_smia_register registers[64]; memset(registers,0xe7,sizeof(registers));
    for (size_t i=0;i<count;++i) registers[i].address=(uint16_t)(i*257U);
    cc_smia_register saved[64]; memcpy(saved,registers,sizeof(saved));
    int ret=cc_smia_parse(layout,data,size,registers,count);
    FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(!memcmp(registers,saved,sizeof(saved)));
    cc_sensor_embedded_status status; memset(&status,0xe7,sizeof(status));
    cc_sensor_embedded_status old_status=status;
    ret=cc_sensor_parse_embedded_status(model,layout,data,size,&status);
    FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(!memcmp(&status,&old_status,sizeof(status)));
    else FUZZ_CHECK(status.analogue_gain>=1 && status.line_length_pck && status.frame_length_lines &&
                    status.temperature_c>=-20 && status.temperature_c<=80);
    cc_sensor_pdaf phase; memset(&phase,0xe7,sizeof(phase)); cc_sensor_pdaf old_phase=phase;
    ret=cc_sensor_parse_pdaf(model,layout->bits_per_pixel,data,size,&phase);
    FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(!memcmp(&phase,&old_phase,sizeof(phase)));
    else {
        FUZZ_CHECK(phase.rows==12 && phase.cols==16);
        for (size_t i=0;i<CC_SENSOR_PDAF_REGIONS;++i)
            FUZZ_CHECK(phase.regions[i].confidence<2048 && phase.regions[i].phase>=-1024 &&
                phase.regions[i].phase<1024 && (phase.regions[i].confidence || !phase.regions[i].phase));
    }
    cc_sensor_hdr_statistics hdr; memset(&hdr,0xe7,sizeof(hdr)); cc_sensor_hdr_statistics old_hdr=hdr;
    ret=cc_sensor_parse_hdr_statistics(model,layout->bits_per_pixel,data,size,&hdr);
    FUZZ_CHECK(ret<=0);
    if (ret) FUZZ_CHECK(!memcmp(&hdr,&old_hdr,sizeof(hdr)));
    else FUZZ_CHECK(hdr.counted && hdr.average<65536);
}
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in={data,size};
    uint8_t mode=fuzz_u8(&in);
    const char *names[]={"imx708","imx477","imx519","imx219","imx296","ar0521"};
    const cc_sensor_model *model;
    FUZZ_CHECK(!cc_sensor_lookup(names[fuzz_u8(&in)%6],&model));
    const uint32_t bpps[]={8,10,12,14};
    cc_smia_layout layout={bpps[fuzz_u8(&in)%4],fuzz_u32(&in),fuzz_u32(&in)};
    if (!(mode&1U)) {
        if (mode&2U) layout.bits_per_pixel=fuzz_u32(&in);
        uint8_t count=(uint8_t)(fuzz_u8(&in)%64U+1U);
        atomic_raw(model,&layout,in.data,in.size,count);
        return 0;
    }
    cc_smia_register expected[9]={{0x0202,0},{0x0203,0},{0x0204,0},{0x0205,0},
        {0x0340,0},{0x0341,0},{0x0342,0},{0x0343,0},{0x013a,0}};
    uint8_t logical[57]; size_t n=0; logical[n++]=0x0a;
    for (size_t i=0;i<9;++i) {
        expected[i].value=fuzz_u8(&in);
        logical[n++]=0xaa; logical[n++]=(uint8_t)(expected[i].address>>8);
        logical[n++]=0xa5; logical[n++]=(uint8_t)expected[i].address;
        logical[n++]=0x5a; logical[n++]=expected[i].value;
    }
    logical[n++]=0x07; logical[n++]=0x07;
    uint8_t bytes[194*7]; memset(bytes,0x55,sizeof(bytes));
    size_t length=wire_offset(layout.bits_per_pixel,n-1)+1;
    for (size_t i=0;i<n;++i) bytes[wire_offset(layout.bits_per_pixel,i)]=logical[i];
    layout.line_length_bytes=0; layout.num_lines=1;
    cc_smia_register got[9]; memcpy(got,expected,sizeof(got));
    for (size_t i=0;i<9;++i) got[i].value=0;
    FUZZ_CHECK(!cc_smia_parse(&layout,bytes,length,got,9));
    for (size_t i=0;i<9;++i) FUZZ_CHECK(got[i].value==expected[i].value);
    if (mode&2U) fuzz_patch(&in,bytes,length);
    atomic_raw(model,&layout,bytes,length,9);

    FUZZ_CHECK(!cc_sensor_lookup("imx708",&model));
    unsigned bpp=10U+fuzz_u8(&in)%3U*2U;
    size_t step=bpp/2U; memset(bytes,0,sizeof(bytes));
    cc_af_pdaf phases[CC_SENSOR_PDAF_REGIONS];
    for (size_t i=0;i<CC_SENSOR_PDAF_REGIONS;++i) {
        unsigned confidence=fuzz_u16(&in)%2048U, p=fuzz_u16(&in)%2048U;
        phases[i]=(cc_af_pdaf){(uint16_t)confidence,confidence ? (int16_t)((int)p-1024):0};
        unsigned wire=(p+1024U)%2048U;
        uint8_t *entry=bytes+(i+2)*step;
        entry[0]=(uint8_t)(confidence/8U); entry[1]=(uint8_t)(confidence%8U*32U+wire/64U);
        entry[2]=(uint8_t)(wire%64U*4U);
    }
    cc_sensor_pdaf phase;
    FUZZ_CHECK(!cc_sensor_parse_pdaf(model,bpp,bytes,194*step,&phase));
    for (size_t i=0;i<CC_SENSOR_PDAF_REGIONS;++i)
        FUZZ_CHECK(phase.regions[i].confidence==phases[i].confidence && phase.regions[i].phase==phases[i].phase);
    if (mode&2U) fuzz_patch(&in,bytes,194*step);
    layout.bits_per_pixel=bpp; atomic_raw(model,&layout,bytes,194*step,9);

    memset(bytes,0,sizeof(bytes)); uint64_t total=0,sum=0; uint32_t bins[128];
    for (size_t i=0;i<137;++i) {
        uint32_t count=fuzz_u32(&in)&UINT32_C(0x3fffff);
        uint8_t *entry=bytes+i*step;
        entry[0]=(uint8_t)(count/16384U); entry[1]=(uint8_t)(count/64U);
        entry[2]=(uint8_t)(count%64U*4U); entry[3]=0x55;
        if (i<128) bins[i]=count/4U;
        if (i) {
            uint64_t centre=i<128 ? i*512U+256U:UINT64_C(196608)>>(145-i);
            total+=count; sum+=count*centre;
        }
    }
    bytes[137*step]=(uint8_t)(bins[1]/4096U); bytes[137*step+1]=(uint8_t)(bins[1]/16U);
    bytes[137*step+2]=(uint8_t)(bins[1]%16U*16U);
    cc_sensor_hdr_statistics hdr;
    int ret=cc_sensor_parse_hdr_statistics(model,bpp,bytes,144*step,&hdr);
    if (total) FUZZ_CHECK(!ret && hdr.counted==total && hdr.average==sum/total && !memcmp(hdr.histogram,bins,sizeof(bins)));
    else FUZZ_CHECK(ret<0);
    if (mode&2U) fuzz_patch(&in,bytes,144*step);
    atomic_raw(model,&layout,bytes,144*step,9);
    return 0;
}
