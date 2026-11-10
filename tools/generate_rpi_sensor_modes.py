#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Compile the archived Pi selector to regenerate hardware-free C fixtures.

Usage: python3 tools/generate_rpi_sensor_modes.py /path/to/reference.tar.xz
Only fixture regeneration needs Python, a C++17 compiler and the local archive.
The scoring/selection methods are extracted verbatim; stubs replace unrelated
pipeline types, format lookup and logging. No network access is used.
"""
import argparse
import hashlib
from pathlib import Path
import subprocess
import tarfile
import tempfile

SHA256 = "ce4d60131c5fd218153f99cf1271375271815cc183a9ac7b270f18d3588015d5"


def method(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    if hashlib.sha256(args.archive.read_bytes()).hexdigest() != SHA256:
        parser.error("archive does not match the documented reference SHA-256")
    with tarfile.open(args.archive, "r:xz") as archive:
        source = archive.extractfile(
            "libcamera/src/libcamera/pipeline/rpi/common/pipeline_base.cpp"
        ).read().decode()
    prelude = r'''
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <vector>
#include <linux/media-bus-format.h>
struct Size { unsigned width, height; };
struct PixelFormat { unsigned bitsPerPixel; };
struct PixelFormatInfo {
    unsigned bitsPerPixel;
    static PixelFormatInfo info(PixelFormat f) { return {f.bitsPerPixel}; }
};
struct BayerFormat { enum class Packing { None }; };
enum class ColorSpace { Raw };
struct V4L2SubdeviceFormat { ColorSpace colorSpace{}; unsigned code{}; Size size{}; };
struct CameraData {
    std::map<unsigned, std::vector<Size>> sensorFormats_;
    double scoreFormat(double desired, double actual) const;
    V4L2SubdeviceFormat findBestFormat(const Size &req, unsigned bitDepth) const;
};
struct NullLog { template<typename T> NullLog &operator<<(const T &) { return *this; } };
#define LOG(a,b) NullLog()
namespace utils { unsigned abs_diff(unsigned a, unsigned b) { return a > b ? a-b : b-a; } }
PixelFormat mbusCodeToPixelFormat(unsigned code, BayerFormat::Packing) {
    switch (code) {
#define DEPTH(bits) \
    case MEDIA_BUS_FMT_SRGGB##bits##_1X##bits: \
    case MEDIA_BUS_FMT_SBGGR##bits##_1X##bits: \
    case MEDIA_BUS_FMT_SGBRG##bits##_1X##bits: \
    case MEDIA_BUS_FMT_SGRBG##bits##_1X##bits: \
    case MEDIA_BUS_FMT_Y##bits##_1X##bits: return {bits}
    DEPTH(8); DEPTH(10); DEPTH(12); DEPTH(14); DEPTH(16);
#undef DEPTH
    default: return {0};
    }
}
'''
    harness = r'''
int main() {
    struct Mode { unsigned code, width, height; };
    const Mode modes[] = {
        {MEDIA_BUS_FMT_SRGGB8_1X8,640,480},
        {MEDIA_BUS_FMT_SRGGB10_1X10,640,480},
        {MEDIA_BUS_FMT_SRGGB10_1X10,1920,1080},
        {MEDIA_BUS_FMT_SRGGB10_1X10,2592,1944},
        {MEDIA_BUS_FMT_SRGGB10_1X10,1280,720},
        {MEDIA_BUS_FMT_SRGGB12_1X12,1280,960},
        {MEDIA_BUS_FMT_SRGGB12_1X12,1920,1080},
        {MEDIA_BUS_FMT_SRGGB12_1X12,3840,2160},
        {MEDIA_BUS_FMT_SBGGR10_1X10,800,600},
        {MEDIA_BUS_FMT_SBGGR10_1X10,2592,1944},
        {MEDIA_BUS_FMT_SGBRG14_1X14,1920,1080},
        {MEDIA_BUS_FMT_SGRBG16_1X16,3840,2160},
        {MEDIA_BUS_FMT_Y8_1X8,640,480},
        {MEDIA_BUS_FMT_Y10_1X10,1280,720},
        {MEDIA_BUS_FMT_Y12_1X12,1920,1080},
        {MEDIA_BUS_FMT_Y16_1X16,3840,2160}
    };
    CameraData data;
    std::puts("static const struct native_mode rpi_reference_modes[]={");
    for (auto m : modes) {
        data.sensorFormats_[m.code].push_back({m.width,m.height});
        std::printf("    {0x%x,%u,%u},\n",m.code,m.width,m.height);
    }
    std::puts("};");
    for (auto &[code,sizes] : data.sensorFormats_)
        std::sort(sizes.begin(),sizes.end(),[](Size a, Size b) {
            uint64_t aa=uint64_t(a.width)*a.height, ba=uint64_t(b.width)*b.height;
            return aa != ba ? aa < ba : a.width < b.width;
        });
    const Size requests[] = {
        {1,1},{640,480},{1000,750},{1280,720},{1920,1080},{1800,1200},
        {1600,1000},{2592,1944},{4000,3000},{640,481},{100,900},
        {2000,1000},{3840,2160},{3000,1000},{1000,3000},{UINT32_MAX,UINT32_MAX}
    };
    std::puts("static const struct {\n"
              "    uint32_t width,height,depth,code,out_width,out_height;\n"
              "} rpi_reference_vectors[]={");
    for (auto req : requests) for (unsigned depth : {8,10,12,14,16}) {
        auto out=data.findBestFormat(req,depth);
        std::printf("    {%u,%u,%u,0x%x,%u,%u},\n",req.width,req.height,depth,
                    out.code,out.size.width,out.size.height);
    }
    std::puts("};");
}
'''
    methods = "\n".join(method(source, signature) for signature in (
        "double CameraData::scoreFormat(",
        "V4L2SubdeviceFormat CameraData::findBestFormat("))
    with tempfile.TemporaryDirectory(prefix="ccamera-rpi-modes-") as directory:
        directory = Path(directory)
        cpp = directory / "reference.cpp"
        exe = directory / "reference"
        cpp.write_text(prelude + methods + harness)
        subprocess.run([args.cxx, "-std=c++17", "-O0", str(cpp), "-o", str(exe)], check=True)
        output = subprocess.check_output([str(exe)], text=True)
    header = ("/* SPDX-License-Identifier: LGPL-2.1-or-later */\n"
              "/* Generated by tools/generate_rpi_sensor_modes.py from unmodified\n"
              " * CameraData::scoreFormat()/findBestFormat(), archived libcamera 0.7.2.\n"
              f" * Reference archive SHA-256: {SHA256}. */\n")
    target = Path(__file__).resolve().parents[1] / "tests/rpi_sensor_mode_vectors.h"
    target.write_text(header + output)


if __name__ == "__main__":
    main()
