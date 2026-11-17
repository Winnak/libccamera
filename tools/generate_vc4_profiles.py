#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Compile vendored VC4 calibration. Not needed to build or use the library.

The JSON is unchanged from the archived libcamera 0.7.2 reference. Fail on
unsupported shapes/policies instead of silently borrowing another sensor's data.
"""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
NAMES = ("ov5647", "ov5647_noir", "imx219", "imx219_noir", "imx477", "imx477_noir",
         "imx708", "imx708_noir", "imx708_wide", "imx708_wide_noir")


def generate(pipeline="vc4"):
    pisp = pipeline == "pisp"
    lines = ["/* SPDX-License-Identifier: BSD-2-Clause */",
             "/* Copyright (C) 2019-2026 Raspberry Pi Ltd",
             f" * Generated from archived libcamera 0.7.2 {pipeline} calibration.",
             " * Regenerate: python3 tools/generate_vc4_profiles.py */",
             '#include "vc4_profile.h"']
    entries = []
    for name in NAMES:
        raw = (ROOT / "data" / f"{name}-{pipeline}.json").read_bytes()
        a = {k: v for item in json.loads(raw)["algorithms"] for k, v in item.items()}
        digest = hashlib.sha256(raw).hexdigest()
        lines.append(f"/* {name}: JSON SHA256 {digest} */")

        def array(key, values):
            def c(value):
                return "{" + ", ".join(c(x) for x in value) + "}" if isinstance(value, list) else repr(float(value))
            shape, v = [], values
            while isinstance(v, list):
                assert v
                shape.append(len(v))
                v = v[0]
            symbol = f"{name}_{key}"
            # Put table rows on separate lines to keep generated data inspectable.
            value = c(values) if len(shape) == 1 else "{\n    " + ",\n    ".join(c(x) for x in values) + "\n}"
            lines.append(f"static const double {symbol}{''.join(f'[{n}]' for n in shape)} = {value};")
            return symbol

        gamma = a["rpi.contrast"]["gamma_curve"]
        assert len(gamma) == (102 if pisp else 66) and gamma[0:2] == [0, 0] and gamma[-2:] == [65535, 65535]
        ccm = a["rpi.ccm"]["ccms"]
        assert len(ccm) >= 2 and all(len(v["ccm"]) == 9 for v in ccm)
        lsc = a["rpi.alsc"]
        r, b = lsc["calibrations_Cr"], lsc["calibrations_Cb"]
        assert len(r) >= 2 and [v["ct"] for v in r] == [v["ct"] for v in b]
        samples = 1024 if pisp else 192
        assert all(len(v["table"]) == samples for v in r + b) and len(lsc["luminance_lut"]) == samples
        fields = {
            "gamma": array("gamma", gamma),
            "num_gamma": str(len(gamma)),
            "ccm": array("ccm", [[v["ct"]] + v["ccm"] for v in ccm]),
            "num_ccm": str(len(ccm)),
            "lsc_ct": array("lsc_ct", [v["ct"] for v in r]),
            "lsc_r": array("lsc_r", [v["table"] for v in r]),
            "lsc_b": array("lsc_b", [v["table"] for v in b]),
            "lsc_y": array("lsc_y", lsc["luminance_lut"]),
            "num_lsc": str(len(r)),
            "lsc_width": "32" if pisp else "16", "lsc_height": "32" if pisp else "12",
        }
        for key in ("lsc_r", "lsc_b"):
            fields[key] = "&" + fields[key] + "[0][0]"
        awb = a["rpi.awb"]
        bayes = awb.get("bayes", 1)
        if bayes:
            curve = awb["ct_curve"]
            assert len(curve) % 3 == 0 and len(curve) >= 6
            fields["ct"] = array("ct", [curve[i:i+3] for i in range(0, len(curve), 3)])
            fields["num_ct"] = str(len(curve) // 3)
            priors = []
            for i, v in enumerate(awb["priors"]):
                symbol = array(f"prior_{i}", v["prior"])
                priors.append(f"{{{v['lux']}, {symbol}, {len(v['prior'])}}}")
            lines.append(f"static const struct cc_vc4_prior {name}_priors[] = {{" + ", ".join(priors) + "};")
            fields["priors"], fields["num_priors"] = f"{name}_priors", str(len(priors))
        agc = a["rpi.agc"].get("channels", [a["rpi.agc"]])[0]
        normal = agc["exposure_modes"]["normal"]
        constraints = agc["constraint_modes"]["normal"]
        assert len(constraints) == 1
        constraint = constraints[0]
        assert constraint["bound"] == "LOWER" and constraint["q_lo"] == 0.98 and constraint["q_hi"] == 1.0
        assert constraint["y_target"][1] == constraint["y_target"][3]
        assert len(normal["shutter"]) == len(normal["gain"])
        assert len(agc["metering_modes"]["centre-weighted"]["weights"]) == (225 if pisp else 15)
        for key, values in (("shutter", normal["shutter"]), ("gain", normal["gain"]),
                            ("target", agc["y_target"]),
                            ("weights", agc["metering_modes"]["centre-weighted"]["weights"])):
            fields[key] = array(key, values)
        fields["num_exposure"], fields["num_target"] = str(len(normal["gain"])), str(len(agc["y_target"]))
        sdn = a["rpi.denoise"]["normal"]["sdn"] if pisp else a["rpi.sdn"]
        cdn = a["rpi.denoise"]["normal"]["cdn"] if pisp else {}
        scalars = {
            "black": a["rpi.black_level"]["black_level"],
            "luminance_strength": lsc["luminance_strength"],
            "noise_constant": a["rpi.noise"]["reference_constant"],
            "noise_slope": a["rpi.noise"]["reference_slope"],
            "geq_offset": a["rpi.geq"]["offset"], "geq_slope": a["rpi.geq"]["slope"],
            "sdn_deviation": sdn.get("deviation_no_tdn", sdn.get("deviation", 3.2)),
            "sdn_strength": sdn.get("strength_no_tdn", sdn.get("strength", 0.75)),
            "cdn_deviation": cdn.get("deviation_no_tdn", cdn.get("deviation", 0)),
            "cdn_strength": cdn.get("strength", 0),
            "sharpen_threshold": a["rpi.sharpen"]["threshold"],
            "sharpen_strength": a["rpi.sharpen"]["strength"],
            "sharpen_limit": a["rpi.sharpen"]["limit"],
            "bayes": bayes,
            "sensitivity_r": awb.get("sensitivity_r", 1) if bayes else 1,
            "sensitivity_b": awb.get("sensitivity_b", 1) if bayes else 1,
            "transverse_pos": awb.get("transverse_pos", 0),
            "transverse_neg": awb.get("transverse_neg", 0),
            "base_ev": agc.get("base_ev", 1), "speed": agc.get("speed", 0.2),
            "constraint_target": constraint["y_target"][1],
            "lux_shutter": a["rpi.lux"]["reference_shutter_speed"] * 1000,
            "lux_gain": a["rpi.lux"]["reference_gain"], "lux_y": a["rpi.lux"]["reference_Y"],
            "lux_reference": a["rpi.lux"]["reference_lux"],
        }
        fields.update({k: str(v) for k, v in scalars.items()})
        flags = (1 if "noir" in name else 0) | (2 if "wide" in name else 0) | (4 if name.startswith("imx477") else 0)
        entries.append("    {\n        .info = {CC_VC4_PROFILE_" + name.upper() +
                       f', "{name}", "{name.split("_")[0]}", "{name}-{pipeline}-{digest[:12]}-calibration-v1", {flags}' + "},\n" +
                       "\n".join(f"        .{key} = {value}," for key, value in fields.items()) + "\n    }")
    lines.append("static const struct cc_vc4_profile_data profiles[] = {\n" + ",\n".join(entries) + "\n};")
    lines.append("""const struct cc_vc4_profile_data *cc_PIPELINE_profile_data(enum cc_vc4_profile profile)
{
    for (size_t i = 0; i < sizeof(profiles) / sizeof(profiles[0]); ++i)
        if (profiles[i].info.profile == profile)
            return &profiles[i];
    return NULL;
}""".replace("PIPELINE", pipeline))
    path = ROOT / f"src/{pipeline}_profiles.c"
    output = "\n".join(lines) + "\n"
    if "--check" in sys.argv:
        if path.read_text() != output:
            raise SystemExit("VC4 compiled calibration is stale")
    else:
        path.write_text(output)


if __name__ == "__main__":
    generate("pisp" if "--pisp" in sys.argv else "vc4")
