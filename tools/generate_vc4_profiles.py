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
PISP_LENSES = ("imx477_6mm", "imx477_16mm")


def generate(pipeline="vc4"):
    pisp = pipeline == "pisp"
    lines = ["/* SPDX-License-Identifier: BSD-2-Clause */",
             "/* Copyright (C) 2019-2026 Raspberry Pi Ltd",
             f" * Generated from archived libcamera 0.7.2 {pipeline} calibration.",
             " * Regenerate: python3 tools/generate_vc4_profiles.py */",
             '#include "vc4_profile.h"']
    entries = []
    hq_arrays = {}
    for name in NAMES + (PISP_LENSES if pisp else ()):
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
            if pisp and (name == "imx477" or name in PISP_LENSES):
                # The lens profiles share most of the stock HQ calibration.
                previous = hq_arrays.setdefault((key, value), symbol)
                if previous != symbol:
                    return previous
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
            modes = [awb["modes"][m] for m in
                     ("auto", "incandescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy")]
            fields["wb_modes"] = "{" + ", ".join("{%s,%s}" % (m["lo"], m["hi"]) for m in modes) + "}"
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
        meters = [array("meter_" + m.replace("-", "_"), agc["metering_modes"][m]["weights"])
                  for m in ("centre-weighted", "spot", "matrix")]
        fields["metering"] = "{" + ", ".join(meters) + "}"
        if "rpi.af" in a:
            af = a["rpi.af"]
            ranges = [af["ranges"][m] for m in ("normal", "macro")]
            ranges += [dict(min=min(r["min"] for r in ranges), max=max(r["max"] for r in ranges),
                            default=ranges[0]["default"])]
            speeds = [af["speeds"][m] for m in ("normal", "fast")]
            keys = ("step_coarse", "step_fine", "contrast_ratio", "retrigger_ratio", "retrigger_delay",
                    "pdaf_gain", "pdaf_squelch", "max_slew", "pdaf_frames", "dropout_frames", "step_frames")
            rows = ",".join("{%s,%s,%s}" % (r["min"], r["max"], r["default"]) for r in ranges)
            speed_rows = ",".join("{" + ",".join(str(s[k]) for k in keys) + "}" for s in speeds)
            mapping = af["map"]
            points = ",".join("{%s,%s}" % tuple(mapping[i:i+2]) for i in range(0, len(mapping), 2))
            lines.append(f"static const cc_af_tuning {name}_af = {{.ranges={{{rows}}}, .speeds={{{speed_rows}}},"
                         f" .confidence_epsilon={af['conf_epsilon']}, .confidence_threshold={af['conf_thresh']},"
                         f" .confidence_clip={af['conf_clip']}, .skip_frames={af['skip_frames']},"
                         f" .check_for_ir={int(af['check_for_ir'])}, .num_map_points={len(mapping)//2}, .map={{{points}}}}};")
            fields["af"] = "&" + name + "_af"
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
            "ae_startup_frames": agc.get("startup_frames", 10),
            "constraint_target": constraint["y_target"][1],
            "lux_shutter": a["rpi.lux"]["reference_shutter_speed"] * 1000,
            "lux_gain": a["rpi.lux"]["reference_gain"], "lux_y": a["rpi.lux"]["reference_Y"],
            "lux_reference": a["rpi.lux"]["reference_lux"],
        }
        fields.update({k: str(v) for k, v in scalars.items()})
        cac = a.get("rpi.cac")
        if pisp:
            if cac:
                shifts = [cac[key] for key in ("lut_rx", "lut_ry", "lut_bx", "lut_by")]
                assert all(len(axis) == 81 for axis in shifts)
                fields["cac"] = array("cac", shifts)
                fields["cac_strength"] = str(cac.get("strength", 1))
            temporal = []
            for mode in ("normal", "hdr"):
                denoise = a["rpi.denoise"][mode]
                s, c, t = (denoise[k] for k in ("sdn", "cdn", "tdn"))
                values = [s.get("deviation", 3.2), s.get("deviation2", s.get("deviation", 3.2)),
                          s.get("strength", 0.25), s.get("backoff", 0.75),
                          c.get("deviation_with_tdn", c.get("deviation_no_tdn", c.get("deviation", 150)) / 3),
                          t.get("deviation", 0.5), t.get("threshold", 0.75)]
                temporal.append("{" + ",".join(str(v) for v in values) + "}")
            fields["temporal"] = "{" + ",".join(temporal) + "}"
            h = a["rpi.hdr"]["MultiExposure"]
            assert h["cadence"] == [1, 2] and h["stitch_enable"] == h["tonemap_enable"] == 1
            assert "spatial_gain_curve" not in h
            spatial_gain = h.get("spatial_gain", 1)
            # libcamera reads spatial_gain as a scalar, defaulting to 2 for
            # the sequence in the archived imx477_16mm profile.
            if isinstance(spatial_gain, list):
                spatial_gain = 2
            values = [h.get("threshold_lo", 50000), h.get("detail_constant", 0), h.get("diff_power", 13),
                      h.get("motion_threshold", 0.005), h.get("detail_slope", 0),
                      h.get("iir_strength", 8), h.get("strength", 1.5), spatial_gain,
                      min(h.get("diffusion", 3), 15)]
            fields["hdr"] = "{" + ",".join(str(v) for v in values) + "}"
        flags = ((1 if "noir" in name else 0) | (2 if "wide" in name else 0) |
                 (4 if name.startswith("imx477") else 0) | (8 if cac else 0))
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
