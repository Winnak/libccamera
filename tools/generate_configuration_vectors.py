#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""Compile archived size/UVC validation policies to create independent C fixtures."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile


def method(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


archive = Path(sys.argv[1])
assert hashlib.sha256(archive.read_bytes()).hexdigest() == "ce4d60131c5fd218153f99cf1271375271815cc183a9ac7b270f18d3588015d5"
with tarfile.open(archive) as tar:
    def read(name):
        return tar.extractfile("libcamera/" + name).read().decode()
    geometry = read("src/libcamera/geometry.cpp")
    stream = read("src/libcamera/stream.cpp")
    uvc = read("src/libcamera/pipeline/uvcvideo/uvcvideo.cpp")
    prelude = r'''
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>
#include <time.h>
#include <linux/videodev2.h>
#include <linux/drm_fourcc.h>
using PixelFormat=uint32_t;
struct Size {
    unsigned width=0,height=0;
    Size()=default; Size(unsigned w,unsigned h):width(w),height(h) {}
    bool operator==(Size b) const { return width==b.width && height==b.height; }
    bool operator!=(Size b) const { return !(*this==b); }
};
bool operator<(const Size &,const Size &);
bool operator>(const Size &a,const Size &b) { return b<a; }
struct SizeRange { Size min,max; unsigned hStep=1,vStep=1; bool contains(const Size &) const; };
struct StreamFormats {
    std::map<PixelFormat,std::vector<SizeRange>> formats_;
    std::vector<PixelFormat> pixelformats() const { std::vector<PixelFormat> r; for (const auto &v:formats_) r.push_back(v.first); return r; }
    std::vector<Size> sizes(const PixelFormat &) const;
};
enum class Orientation { Rotate0=1 };
struct StreamConfiguration {
    PixelFormat pixelFormat=0; Size size; unsigned bufferCount=0; StreamFormats formats_;
    const StreamFormats &formats() const { return formats_; }
};
struct CameraConfiguration { enum Status { Valid,Adjusted,Invalid }; };
struct UVCCameraConfiguration:CameraConfiguration {
    bool sensorConfig=false; Orientation orientation=Orientation::Rotate0;
    std::vector<StreamConfiguration> config_; Status validate();
};
struct NullLog { template<typename T> NullLog &operator<<(const T &) { return *this; } };
#define LOG(a,b) NullLog()
'''
    # Keep the original validation code through bufferCount. Device I/O and
    # colour/stride acknowledgement are exercised separately by native C tests.
    validate = method(uvc, "CameraConfiguration::Status UVCCameraConfiguration::validate()")
    validate = validate[:validate.index("\tV4L2DeviceFormat format;")] + "\treturn status;\n}"
    program = prelude + "\n" + method(geometry, "bool operator<(const Size &lhs, const Size &rhs)")
    program += "\n" + method(geometry, "bool SizeRange::contains(const Size &size) const")
    program += "\n" + method(stream, "std::vector<Size> StreamFormats::sizes(const PixelFormat &pixelformat) const")
    program += "\n" + validate + r'''
int main() {
    const Size requests[]={{1,1},{1000,700},{1000,400},{1280,720},{1900,1000},{4096,2160},{800,600},{640,481}};
    const uint32_t native[]={V4L2_PIX_FMT_RGB24,V4L2_PIX_FMT_BGR24,0xffffffffU};
    for (unsigned kind=0;kind<3;++kind) for (auto request:requests) for (unsigned fmt=0;fmt<3;++fmt)
    for (unsigned o: {1U,8U}) for (unsigned count: {1U,3U}) {
        StreamFormats formats;
        std::vector<SizeRange> sizes;
        if (!kind) for (Size s: {Size(1280,720),Size(640,480),Size(1920,1080),Size(1024,300),Size(800,600)}) sizes.push_back({s,s});
        else if (kind==1) sizes={{{160,128},{1280,960},32,16}};
        else sizes={{{640,480},{4096,2160},64,16}};
        formats.formats_[DRM_FORMAT_BGR888]=sizes; formats.formats_[DRM_FORMAT_RGB888]=sizes;
        StreamConfiguration cfg; cfg.pixelFormat=fmt==0 ? DRM_FORMAT_BGR888:fmt==1 ? DRM_FORMAT_RGB888:0xffffffffU;
        cfg.size=request; cfg.bufferCount=4; cfg.formats_=formats;
        UVCCameraConfiguration c; c.orientation=static_cast<Orientation>(o); c.config_.assign(count,cfg);
        auto status=c.validate(); const auto &out=c.config_[0];
        uint32_t selected=out.pixelFormat==DRM_FORMAT_BGR888 ? V4L2_PIX_FMT_RGB24:V4L2_PIX_FMT_BGR24;
        printf("    {%u,%u,%u,0x%08xU,%u,%u,%u,%u,0x%08xU,%u},\n",kind,request.width,request.height,native[fmt],o,count,
            out.size.width,out.size.height,selected,static_cast<unsigned>(status));
    }
}
'''
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "linux").mkdir()
        for header in ("videodev2.h", "drm_fourcc.h", "drm.h", "drm_mode.h"):
            (root / "linux" / header).write_text(read("include/linux/" + header))
        (root / "vectors.cpp").write_text(program)
        subprocess.run(["c++", "-std=c++17", "-I", str(root), str(root / "vectors.cpp"), "-o", str(root / "vectors")], check=True)
        output = subprocess.check_output([str(root / "vectors")], text=True)
path = Path(__file__).resolve().parents[1] / "tests/configuration_vectors.h"
path.write_text("/* SPDX-License-Identifier: LGPL-2.1-or-later */\n"
    "/* Generated by compiled, unmodified archived Size/StreamFormats policies\n"
    " * and the UVC validate prefix before device I/O. Reference SHA256:\n"
    " * ce4d60131c5fd218153f99cf1271375271815cc183a9ac7b270f18d3588015d5. */\n"
    "static const struct configuration_vector configuration_vectors[]={\n" + output + "};\n")
print(f"Wrote {len(output.splitlines())} fixtures to {path}")
