#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Compile archived ID methods with simulated sysfs to generate C fixtures.

Usage: python3 tools/generate_camera_id_vectors.py /path/to/reference.tar.xz
The methods are extracted verbatim. File/stream/logging stubs replace their
environment; only regeneration requires Python, C++17 and the local archive.
"""
import argparse
import hashlib
import json
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


def fixtures():
    cases = []

    def sensor(name, device, model, links=None, files=None, kind=1, pipeline=1):
        cases.append(dict(name=name, device=device, model=model, links=links or {},
                          files=files or {}, kind=kind, pipeline=pipeline))

    for bus in (10, 27):
        path = f"/sys/devices/platform/axi/i2c-{bus}/{bus}-0010"
        sensor(f"dt-i2c-{bus}", path, f"imx219 {bus}-0010",
               {path + "/of_node": "/sys/firmware/devicetree/base/axi/i2c@1000/imx219@10"})
    path = "/sys/devices/pci0000:00/0000:00:15.0/i2c-17/i2c-OVTI01A0:00"
    sensor("acpi-sensor", path, "ov01a10 17-0036",
           files={path + "/firmware_node/path": "\\_SB_.PCI0.I2C0.CAM0\n"})
    sensor("external-dt-root", path, "sensor", {path + "/of_node": "/sys/other-tree/camera@36"})
    sensor("vimc", "/sys/devices/platform/vimc.0", "Sensor B", kind=2, pipeline=3)
    sensor("vimc-instance", "/sys/devices/platform/vimc.1", "Sensor B", kind=2, pipeline=3)
    sensor("virtual-i2c-model", "/sys/devices/platform/test-camera.0", "jt8ew9 pixel_array 0-0010", kind=2)
    sensor("virtual-unicode", "/sys/devices/platform/test-camera.1", "Sénsor B", kind=2)
    sensor("unsupported-sensor", "/sys/devices/pci0000:00/0000:00:15.0/i2c-3/3-0036", "sensor 3-0036", kind=0)

    def usb(name, bus=3, port="2.4", interface="1.0", acpi=False, nearest=False, missing=None):
        controller = "/sys/devices/pci0000:00/0000:00:14.0" if acpi else "/sys/devices/platform/axi/usb@2000"
        parent = f"{controller}/usb{bus}/{bus}-{port}"
        device = parent + f"/{bus}-{port}:{interface}"
        links, files = {}, {}
        if acpi:
            files[controller + "/firmware_node/path"] = "\\_SB_.PCI0.XHC0\n"
        else:
            links[controller + "/of_node"] = "/sys/firmware/devicetree/base/axi/usb@2000"
        if nearest:
            links[parent + "/of_node"] = "/sys/firmware/devicetree/base/usb-port@2"
        files[device + "/../idVendor"] = "046d\n"
        files[device + "/../idProduct"] = "0825\n"
        if missing == "controller":
            links.clear()
            files.pop(controller + "/firmware_node/path", None)
        if missing == "vendor":
            del files[device + "/../idVendor"]
        if missing == "product":
            del files[device + "/../idProduct"]
        cases.append(dict(name=name, device=device, model="USB camera", links=links,
                          files=files, kind=0 if missing else 3, pipeline=4))

    usb("usb-bus-3")
    usb("usb-bus-19", bus=19)
    usb("usb-other-port", port="2.5")
    usb("usb-other-interface", interface="1.1")
    usb("usb-acpi", acpi=True)
    usb("usb-nearest-firmware", nearest=True)
    usb("usb-missing-controller", missing="controller")
    usb("usb-missing-vendor", missing="vendor")
    usb("usb-missing-product", missing="product")
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    if hashlib.sha256(args.archive.read_bytes()).hexdigest() != SHA256:
        parser.error("archive does not match the documented SHA-256")
    with tarfile.open(args.archive, "r:xz") as archive:
        def read(name):
            return archive.extractfile("libcamera/" + name).read().decode()
        firmware = method(read("src/libcamera/sysfs.cpp"), "std::string firmwareNodePath(")
        sensor = method(read("src/libcamera/sensor/camera_sensor_legacy.cpp"), "int CameraSensorLegacy::generateId()")
        uvc = method(read("src/libcamera/pipeline/uvcvideo/uvcvideo.cpp"), "bool UVCCameraData::generateId()")
        model = method(read("src/libcamera/v4l2_subdevice.cpp"), "const std::string &V4L2Subdevice::model()")
    prelude = r'''
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
static std::map<std::string,std::string> links,files;
struct reference_stat {};
static int reference_stat(const char *path, struct reference_stat *) { return links.count(path) ? 0:-1; }
static char *reference_realpath(const char *path, void *) {
    auto found=links.find(path);
    return found==links.end() ? nullptr:strdup(found->second.c_str());
}
namespace streams {
using ::std::string; using ::std::regex; using ::std::smatch; using ::std::regex_search;
struct ifstream {
    string path;
    explicit ifstream(const string &p) : path(p) {}
    bool is_open() const { return files.count(path); }
    void close() {}
};
static void getline(ifstream &in,string &out) { out=files.at(in.path).substr(0,files.at(in.path).find('\n')); }
}
namespace libcamera {
namespace std=::streams;
struct NullLog { template<typename T> NullLog &operator<<(const T &) { return *this; } };
#define LOG(a,b) NullLog()
#define stat reference_stat
#define realpath reference_realpath
struct File { static bool exists(const std::string &path) { return files.count(path); } };
namespace sysfs {
@@FIRMWARE@@
}
struct Entity { std::string value; const std::string &name() const { return value; } };
struct V4L2Subdevice {
    Entity *entity_;
    std::string path,model_;
    const std::string &devicePath() const { return path; }
    const std::string &model();
};
@@MODEL@@
struct CameraSensorLegacy {
    V4L2Subdevice *subdev_;
    std::string id_;
    const std::string &model() { return subdev_->model(); }
    int generateId();
};
@@SENSOR@@
struct UVCCameraData { V4L2Subdevice *video_; std::string id_; bool generateId(); };
namespace utils { std::string basename(const char *p) { const char *s=strrchr(p,'/'); return s ? s+1:p; } }
@@UVC@@
}
'''
    source = prelude.replace("@@FIRMWARE@@", firmware).replace("@@MODEL@@", model).replace("@@SENSOR@@", sensor).replace("@@UVC@@", uvc)
    quote = lambda value: json.dumps(value, ensure_ascii=False)
    cases = fixtures()
    source += "\nint main() {\n"
    for case in cases:
        source += "links.clear(); files.clear();\n"
        for key, value in case["links"].items():
            source += f"links[{quote(key)}]={quote(value)};\n"
        for key, value in case["files"].items():
            source += f"files[{quote(key)}]={quote(value)};\n"
        source += ("{ libcamera::Entity entity{" + quote(case["model"]) + "};\n"
                   "libcamera::V4L2Subdevice video{&entity," + quote(case["device"]) + ",{}};\n")
        if case["pipeline"] == 4:
            source += "libcamera::UVCCameraData data{&video,{}}; bool ok=data.generateId();\n"
        else:
            source += "libcamera::CameraSensorLegacy data{&video,{}}; bool ok=data.generateId()==0;\n"
        source += r'::std::cout << ok << " " << ::std::quoted(data.id_) << "\n"; }' + "\n"
    source += "}\n"
    with tempfile.TemporaryDirectory(prefix="ccamera-id-") as directory:
        root = Path(directory)
        (root / "reference.cpp").write_text(source)
        subprocess.run([args.cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        str(root / "reference.cpp"), "-o", str(root / "reference")], check=True)
        results = subprocess.check_output([str(root / "reference")], text=True).splitlines()
    output = ["/* SPDX-License-Identifier: CC0-1.0 */",
              "/* Generated from unmodified archived libcamera 0.7.2 ID methods.",
              f" * Archive SHA-256: {SHA256}. */"]
    for i, (case, result) in enumerate(zip(cases, results, strict=True)):
        valid, value = result.split(" ", 1)
        expected = json.loads(value)
        if bool(int(valid)) != bool(case["kind"]):
            raise RuntimeError("unexpected reference status: " + case["name"])
        for kind in ("links", "files"):
            output.append(f"static const struct id_file id_{kind}_{i}[]={{")
            for key, text in case[kind].items():
                output.append("    {" + quote(key) + "," + quote(text) + "},")
            output.append("    {NULL,NULL}\n};")
        case["expected"] = expected
    output.append("static const struct id_fixture id_fixtures[]={")
    for i, case in enumerate(cases):
        output.append("    {" + ",".join([quote(case["name"]), quote(case["device"]), quote(case["model"]),
                      str(case["pipeline"]), str(case["kind"]), quote(case["expected"]),
                      f"id_links_{i}", f"id_files_{i}"]) + "},")
    output.append("};\n")
    target = Path(__file__).resolve().parents[1] / "tests/camera_id_vectors.h"
    target.write_text("\n".join(output))


if __name__ == "__main__":
    main()
