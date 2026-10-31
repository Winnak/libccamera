#!/usr/bin/env python3
"""Regenerate the compiled profile; Python is not required by CMake or the library."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
raw=(root/'data/ov5647-vc4.json').read_bytes()
a={k:v for x in json.loads(raw)['algorithms'] for k,v in x.items()}
lines=['/* SPDX-License-Identifier: BSD-2-Clause */', '/* Generated from Raspberry Pi Ltd OV5647 VC4 calibration (2019-2026).',
       ' * Archived libcamera 0.7.2 reference ce4d60131c5f; JSON SHA256:', ' * '+hashlib.sha256(raw).hexdigest(), ' * Regenerate: python3 tools/generate_ov5647.py */', '#define CC_OV5647_PROFILE_ID "ov5647-vc4-'+hashlib.sha256(raw).hexdigest()[:12]+'-calibration-v1"']
def array(name,value):
 def c(x):
  return '{'+','.join(c(v) for v in x)+'}' if isinstance(x,list) else repr(float(x))
 shape=[];v=value
 while isinstance(v,list):shape.append(len(v));v=v[0]
 lines.append('static const double '+name+''.join('['+str(n)+']' for n in shape)+' = '+c(value)+';')
array('ov_gamma',a['rpi.contrast']['gamma_curve'])
array('ov_ct', [a['rpi.awb']['ct_curve'][i:i+3] for i in range(0,21,3)])
array('ov_ccm', [[v['ct']]+v['ccm'] for v in a['rpi.ccm']['ccms']])
array('ov_lsc_ct',[v['ct'] for v in a['rpi.alsc']['calibrations_Cr']])
array('ov_lsc_r',[v['table'] for v in a['rpi.alsc']['calibrations_Cr']])
array('ov_lsc_b',[v['table'] for v in a['rpi.alsc']['calibrations_Cb']])
array('ov_lsc_y',a['rpi.alsc']['luminance_lut'])
array('ov_prior_lux',[v['lux'] for v in a['rpi.awb']['priors']])
# Priors have unequal lengths, keep their PWL data separate.
for i,v in enumerate(a['rpi.awb']['priors']): array('ov_prior_'+str(i),v['prior'])
array('ov_shutter',a['rpi.agc']['channels'][0]['exposure_modes']['normal']['shutter'])
array('ov_gain',a['rpi.agc']['channels'][0]['exposure_modes']['normal']['gain'])
(root/'src/ov5647_profile.h').write_text('\n'.join(lines)+'\n')
