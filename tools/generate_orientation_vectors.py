#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Compile unmodified archived orientation algebra and sensor selection policy.

Usage: python3 tools/generate_orientation_vectors.py /path/to/reference.tar.xz
Only regeneration needs the archive and a C++17 compiler. Builds remain C-only.
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
    end, depth = opening + 1, 1
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
        parser.error("archive does not match the documented SHA-256")
    with tarfile.open(args.archive, "r:xz") as archive:
        files = {}
        for name in ("include/libcamera/orientation.h", "include/libcamera/transform.h",
                     "src/libcamera/orientation.cpp", "src/libcamera/transform.cpp",
                     "src/libcamera/sensor/camera_sensor_legacy.cpp"):
            files[name] = archive.extractfile("libcamera/" + name).read().decode()
    policy = method(files["src/libcamera/sensor/camera_sensor_legacy.cpp"],
                    "Transform CameraSensorLegacy::computeTransform(")
    harness = r'''
#include <libcamera/orientation.h>
#include <libcamera/transform.h>
#include <climits>
#include <cstdio>
namespace libcamera {
struct CameraSensorLegacy {
    bool supportFlips_;
    Orientation mountingOrientation_;
    Transform computeTransform(Orientation *) const;
};
POLICY
}
using namespace libcamera;
int main() {
    const int angles[]={INT_MIN,-1080,-720,-450,-360,-270,-180,-90,-1,0,1,45,90,180,270,360,450,720,1080,INT_MAX};
    std::puts("static const struct { int32_t angle; uint32_t valid,orientation; } rotation_vectors[]={");
    for (int angle : angles) {
        bool valid;
        auto o=orientationFromRotation(angle,&valid);
        std::printf("    {%d,%u,%u},\n",angle,unsigned(valid),unsigned(o));
    }
    std::puts("};\nstatic const uint32_t compose_vectors[8][8]={");
    for (unsigned first=0;first<8;++first) {
        std::printf("    {");
        for (unsigned second=0;second<8;++second)
            std::printf("%u,",unsigned(Transform(first)*Transform(second)));
        std::puts("},");
    }
    std::puts("};\nstatic const uint32_t inverse_vectors[8]={");
    for (unsigned t=0;t<8;++t) std::printf("%u,",unsigned(-Transform(t)));
    std::puts("\n};\nstatic const uint32_t apply_vectors[8][8]={");
    for (unsigned base=1;base<=8;++base) {
        std::printf("    {");
        for (unsigned t=0;t<8;++t) std::printf("%u,",unsigned(Orientation(base)*Transform(t)));
        std::puts("},");
    }
    std::puts("};\nstatic const uint32_t between_vectors[8][8]={");
    for (unsigned desired=1;desired<=8;++desired) {
        std::printf("    {");
        for (unsigned base=1;base<=8;++base) std::printf("%u,",unsigned(Orientation(desired)/Orientation(base)));
        std::puts("},");
    }
    std::puts("};\nstatic const struct {\n    int32_t rotation;\n"
              "    uint32_t requested,support,orientation,transform;\n} sensor_orientation_vectors[]={");
    for (int angle : {0,90,180,270,45,-90,450,INT_MIN,INT_MAX}) {
        CameraSensorLegacy sensor;
        sensor.mountingOrientation_=orientationFromRotation(angle);
        for (unsigned requested=1;requested<=8;++requested) for (unsigned support=0;support<2;++support) {
            sensor.supportFlips_=support;
            auto o=Orientation(requested);
            auto t=sensor.computeTransform(&o);
            std::printf("    {%d,%u,%u,%u,%u},\n",angle,requested,support,unsigned(o),unsigned(t));
        }
    }
    std::puts("};");
}
'''.replace("POLICY", policy)
    with tempfile.TemporaryDirectory(prefix="ccamera-orientation-") as directory:
        root = Path(directory)
        for name, source in files.items():
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(source)
        (root / "harness.cpp").write_text(harness)
        subprocess.run([args.cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(root / "include"), str(root / "harness.cpp"),
                        str(root / "src/libcamera/orientation.cpp"),
                        str(root / "src/libcamera/transform.cpp"), "-o", str(root / "reference")], check=True)
        vectors = subprocess.check_output([str(root / "reference")], text=True)
    target = Path(__file__).resolve().parents[1] / "tests/orientation_vectors.h"
    target.write_text("/* SPDX-License-Identifier: CC0-1.0 */\n"
                      "/* Generated from unmodified archived libcamera 0.7.2\n"
                      " * orientation.cpp, transform.cpp and computeTransform().\n"
                      f" * Archive SHA-256: {SHA256}. */\n" + vectors)


if __name__ == "__main__":
    main()
