#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Independent JSON arithmetic fixtures for VC4 mappings and static ALSC.

Uses Python interpolation/sqrt, with no production generator/code imports.
Builds use the checked-in vectors and do not need Python or the archive.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
names = ("ov5647", "ov5647_noir", "imx219", "imx219_noir", "imx477", "imx477_noir",
         "imx708", "imx708_noir", "imx708_wide", "imx708_wide_noir")
indices = (0, 15, 80, 95, 176, 191)
lines = ["/* SPDX-License-Identifier: BSD-2-Clause */",
         "/* Derived from Raspberry Pi Ltd calibration, archived libcamera 0.7.2.",
         " * Independent arithmetic: tools/generate_vc4_tuning_vectors.py */",
         "struct vc4_vector {",
         "    enum cc_vc4_profile profile; double ct, gain; unsigned crop, flips, black;",
         "    int ccm[9]; unsigned gamma[33], geq_offset; int geq_slope, noise_slope;",
         "    double lsc[3][6];",
         "};", "static const unsigned lsc_indices[] = {0, 15, 80, 95, 176, 191};",
         "static const struct vc4_vector vc4_vectors[] = {"]


def interpolate(points, x):
    if x <= points[0][0]:
        return points[0][1]
    for (a, b), (c, d) in zip(points, points[1:]):
        if x <= c:
            return b + (d - b) * (x - a) / (c - a)
    return points[-1][1]


def c_array(values):
    return "{" + ", ".join(c_array(v) if isinstance(v, list) else repr(v) for v in values) + "}"


for name in names:
    raw = (root / "data" / f"{name}-vc4.json").read_bytes()
    a = {k: v for item in json.loads(raw)["algorithms"] for k, v in item.items()}
    lines.append(f"    /* {name}: SHA256 {hashlib.sha256(raw).hexdigest()} */")
    for ct, gain, crop, flips in ((2000, 1, 0, 0), (3000, 4, 0, 3), (4500, 16, 1, 1),
                                  (6500, 64, 1, 2), (13000, 2, 0, 0)):
        ccm = [int(interpolate([(v["ct"], v["ccm"][i]) for v in a["rpi.ccm"]["ccms"]], ct) * 1000)
               for i in range(9)]
        # Match native truncation while avoiding roundoff at integer-valued knots.
        gamma = a["rpi.contrast"]["gamma_curve"]
        gamma = [int(interpolate(list(zip(gamma[::2], gamma[1::2])), x))
                 for x in list(range(0, 16384, 1024)) + list(range(16384, 32768, 2048)) +
                 list(range(32768, 65536, 4096)) + [65535]]
        alsc = a["rpi.alsc"]
        full = []
        for channel in ("calibrations_Cr", None, "calibrations_Cb"):
            full.append([(1 + alsc["luminance_strength"] * (alsc["luminance_lut"][i] - 1)) /
                         (interpolate([(v["ct"], v["table"][i]) for v in alsc[channel]], ct) if channel else 1)
                         for i in range(192)])
        shading = []
        for table in full:
            samples = []
            for y in range(12):
                for x in range(16):
                    u, v = (x + 0.5) / 16, (y + 0.5) / 12
                    if flips & 1:
                        u = 1 - u
                    if flips & 2:
                        v = 1 - v
                    if crop:
                        u, v = 0.25 + 0.5 * u, 0.25 + 0.5 * v
                    sx, sy = max(0, min(15, u * 16 - 0.5)), max(0, min(11, v * 12 - 0.5))
                    x0, y0 = math.floor(sx), math.floor(sy)
                    x1, y1 = min(x0 + 1, 15), min(y0 + 1, 11)
                    q, w = sx - x0, sy - y0
                    samples.append((table[y0*16+x0]*(1-q) + table[y0*16+x1]*q)*(1-w) +
                                   (table[y1*16+x0]*(1-q) + table[y1*16+x1]*q)*w)
            shading.append([samples[i] / min(samples) for i in indices])
        offset = int(min(65535, a["rpi.geq"]["offset"] * gain))
        slope = int(min(0.99999, a["rpi.geq"]["slope"] * gain) * 1000)
        noise = int(a["rpi.noise"]["reference_slope"] * math.sqrt(gain) * a["rpi.sdn"].get("deviation", 3.2) * 1000)
        lines.append(f"    {{CC_VC4_PROFILE_{name.upper()}, {ct}, {gain}, {crop}, {flips}, " +
                     f"{a['rpi.black_level']['black_level']}, {c_array(ccm)}, {c_array(gamma)}, " +
                     f"{offset}, {slope}, {noise}, {c_array(shading)}}},")
lines.append("};")
path = root / "tests/vc4_tuning_vectors.h"
output = "\n".join(lines) + "\n"
if "--check" in sys.argv:
    if path.read_text() != output:
        raise SystemExit("VC4 arithmetic vectors are stale")
else:
    path.write_text(output)
