#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Compile libpisp's native sharpening defaults (no runtime JSON dependency)."""
import hashlib
import json
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
raw = (root / 'data/pisp-default.json').read_bytes()
a = json.loads(raw)['sharpen']
lines = ['/* SPDX-License-Identifier: BSD-2-Clause */',
         '/* Copyright (C) 2021-2023 Raspberry Pi Ltd. libpisp v1.5.0.',
         ' * Regenerate: python3 tools/generate_pisp_defaults.py',
         ' * Source SHA256: ' + hashlib.sha256(raw).hexdigest() + ' */',
         'static const struct pisp_be_sharpen_config pisp_sharpen_base = {']
def field(name, value):
    if isinstance(value, list):
        value = '{' + ','.join(str(v) for v in value) + '}'
    lines.append(f'    .{name} = {value},')
for i in range(5):
    f = a[f'filter{i}']
    for key, source in [('kernel', 'kernel'), ('threshold_offset', 'offset'),
                        ('threshold_slope', 'threshold_slope'), ('scale', 'scale')]:
        field(f'{key}{i}', f[source])
for side in ('positive', 'negative'):
    for key, source in [('strength', 'strength'), ('pre_limit', 'pre_limit'), ('func', 'function'), ('limit', 'limit')]:
        field(f'{side}_{key}', a[side][source])
for key in ('enables', 'white', 'black', 'grey'):
    field(key, a[key])
lines.append('};')
path = root / 'src/pisp_defaults.h'
output = '\n'.join(lines) + '\n'
if '--check' in sys.argv:
    if path.read_text() != output:
        raise SystemExit('PiSP defaults are stale')
else:
    path.write_text(output)
