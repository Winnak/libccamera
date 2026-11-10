#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""Generate native-to-reference sort keys; no generator is needed by builds."""
from pathlib import Path
import hashlib
import re
import subprocess
import sys
import tarfile
import tempfile

archive = Path(sys.argv[1])
assert hashlib.sha256(archive.read_bytes()).hexdigest() == "ce4d60131c5fd218153f99cf1271375271815cc183a9ac7b270f18d3588015d5"
with tarfile.open(archive) as tar:
    def read(name):
        return tar.extractfile("libcamera/" + name).read().decode()
    native = read("src/libcamera/v4l2_pixelformat.cpp")
    info_source = read("src/libcamera/formats.cpp")
    info = {}
    for block in info_source.split("\t{ formats::")[1:]:
        name = block.split(",", 1)[0]
        encoding = re.search(r"\.colourEncoding = PixelFormatInfo::ColourEncoding(\w+)", block)
        depth = re.search(r"\.bitsPerPixel = (\d+)", block)
        if encoding and depth:
            info[name] = ({"RGB": 1, "YUV": 2, "RAW": 3}[encoding[1]], int(depth[1]))
    descriptions = read("src/libcamera/formats.yaml")
    formats = {}
    for match in re.finditer(r"^  - (\w+):\n((?:      .*\n)+)", descriptions, re.M):
        values = dict(re.findall(r"^      (\w+): (.*)$", match[2], re.M))
        formats[match[1]] = values
    pairs = re.findall(r"V4L2PixelFormat\((V4L2_PIX_FMT_\w+)\),\s*\{ formats::(\w+),", native)
    assert len(pairs) > 60 and all(name in formats for _, name in pairs)
    assert all(name in info for _, name in pairs)
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "linux").mkdir()
        for header in ("videodev2.h", "drm_fourcc.h", "drm.h", "drm_mode.h"):
            (root / "linux" / header).write_text(read("include/linux/" + header))
        program = '#include <time.h>\n#include <stdio.h>\n#include <stdint.h>\n#include <linux/videodev2.h>\n#include <linux/drm_fourcc.h>\nint main(void) {\n'
        for fourcc, name in pairs:
            f = formats[name]
            code = f["fourcc"] + (" | (1U << 31)" if f.get("big_endian") == "true" else "")
            modifier = f.get("mod", "0")
            encoding, depth = info[name]
            program += f'printf("    {{0x%08xU,0x%08xU,UINT64_C(0x%016llx),{encoding},{depth}}}, /* {fourcc}: {name} */\\n", (unsigned){fourcc}, (unsigned)({code}), (unsigned long long)({modifier}));\n'
        program += "return 0; }\n"
        (root / "keys.c").write_text(program)
        subprocess.run(["cc", "-std=c17", "-I", str(root), str(root / "keys.c"), "-o", str(root / "keys")], check=True)
        output = subprocess.check_output([str(root / "keys")], text=True)
output_path = Path(__file__).resolve().parents[1] / "src/configuration_formats.h"
output_path.write_text("/* SPDX-License-Identifier: LGPL-2.1-or-later */\n"
    "/* Generated from the archived 0.7.2 V4L2/DRM map and formats.yaml.\n"
    " * Reference: Copyright (C) 2020 Google Inc. Regenerate with\n"
    " * tools/generate_configuration_formats.py; no DRM headers at build/runtime. */\n"
    "static const struct format_order format_orders[]={\n" + output + "};\n")
print(f"Wrote {len(pairs)} format keys to {output_path}")
