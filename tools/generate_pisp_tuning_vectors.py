#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Independent native arithmetic fixtures from the unchanged PiSP JSON."""
import hashlib
import json
import math
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
lines=['/* SPDX-License-Identifier: BSD-2-Clause */',
       '/* Raspberry Pi Ltd calibration; independent Python arithmetic fixtures.',
       ' * Regenerate: python3 tools/generate_pisp_tuning_vectors.py */',
       'struct pisp_vector { enum cc_vc4_profile profile; double ct, gain; unsigned crop, black, geq, slope, sdn, cdn;',
       '    int ccm[9]; unsigned gamma[64], shading[9]; double raw_red, raw_blue; };', 'static const struct pisp_vector pisp_vectors[] = {']
cac_vectors = []
def interp(points,x):
    if x<=points[0][0]: return points[0][1]
    for (a,b),(c,d) in zip(points,points[1:]):
        if x<=c: return b+(d-b)*(x-a)/(c-a)
    return points[-1][1]
def rnd(x): return math.floor(x+0.5) if x>=0 else math.ceil(x-0.5)
def array(v): return '{'+','.join(str(x) for x in v)+'}'
def sample(v,x,y):
    x=max(0,min(31,x));y=max(0,min(31,y));a=math.floor(x);b=math.floor(y);c=min(31,a+1);d=min(31,b+1)
    u=x-a;w=y-b
    return (v[b*32+a]*(1-u)+v[b*32+c]*u)*(1-w)+(v[d*32+a]*(1-u)+v[d*32+c]*u)*w
for path in sorted(root.glob('data/*-pisp.json')):
    name=path.name.removesuffix('-pisp.json')
    raw=path.read_bytes(); a={k:v for x in json.loads(raw)['algorithms'] for k,v in x.items()}
    lines.append('/* '+name+' SHA256 '+hashlib.sha256(raw).hexdigest()+' */')
    if 'rpi.cac' in a:
        cac = a['rpi.cac']
        axes = [cac[k] for k in ('lut_rx', 'lut_ry', 'lut_bx', 'lut_by')]
        values = [array(max(-64, min(63, rnd(shift * cac['strength'] * 32))) for shift in point)
                  for point in zip(*axes)]
        cac_vectors.append(f'    {{CC_VC4_PROFILE_{name.upper()}, {{\n        ' +
                           ',\n        '.join(', '.join(values[i:i+9]) for i in range(0, 81, 9)) + '\n    }},')
    for ct,gain in ((2000,1),(3000,4),(4500,16),(6500,64),(13000,2)):
        crop=ct in (3000,6500)
        ccm=[rnd(interp([(v['ct'],v['ccm'][i]) for v in a['rpi.ccm']['ccms']],ct)*1024) for i in range(9)]
        curve=a['rpi.contrast']['gamma_curve']; points=list(zip(curve[::2],curve[1::2])); gamma=[]; y=0
        for j in range(1,65):
            x=j*512 if j<32 else (j-32)*1024+16384 if j<48 else (j-48)*2048+32768
            nxt=int(interp(points,x)); slope=min(nxt-y,16383);gamma.append(y+(slope<<16));y+=slope
        denoise=a['rpi.denoise']['normal']; sdn=denoise['sdn']; cdn=denoise['cdn']
        # The fixture downsamples by 2x or 4x on both axes. The reference
        # models either as 2x2 binning for noise and sharpening.
        noise=a['rpi.noise']['reference_slope']*math.sqrt(gain)/2
        values=[a['rpi.black_level']['black_level'],rnd(min(65535,a['rpi.geq']['offset']*gain)),
                min(1023,rnd(a['rpi.geq']['slope']*gain*1024)),rnd(noise*sdn['deviation_no_tdn']*256),
                min(65535,rnd(noise*cdn['deviation']))]
        alsc=a['rpi.alsc']; tables=[]
        for ch in ('calibrations_Cr',None,'calibrations_Cb'):
            t=[(1+alsc['luminance_strength']*(alsc['luminance_lut'][i]-1))/
               (interp([(c['ct'],c['table'][i]) for c in alsc[ch]],ct) if ch else 1) for i in range(1024)]
            if crop:
                t=[sample(t,(0.25+(x+0.5)/64)*32-0.5,(0.25+(y+0.5)/64)*32-0.5)
                   for y in range(32) for x in range(32)]
            minimum=min(t)
            tables.append([v/minimum for v in t])
        shading=[]
        for y0 in (0,16,32):
            for x0 in (0,16,32):
                rgb=[sample(t,x0-0.5,y0*(1<<18)/((32<<18)//768)/768*32-0.5) for t in tables]
                lo=min(rgb);hi=max(rgb);r=0 if lo>=0.5 and hi<1.5 else 1 if hi<2 else 2 if hi<4 else 3
                scale=1024>>r; offset=-511.5 if r==0 else 0.5
                codes=[max(0,min(1023,rnd(offset+scale*c))) for c in rgb]
                shading.append((r<<30)|(codes[2]<<20)|(codes[1]<<10)|codes[0])
        awb=a['rpi.awb']; raw_red=0.5;raw_blue=2.0
        if awb.get('bayes',1):
            curve=awb['ct_curve']
            raw_red=interp(list(zip(curve[::3],curve[1::3])),6500)/awb.get('sensitivity_r',1)
            raw_blue=interp(list(zip(curve[::3],curve[2::3])),6500)/awb.get('sensitivity_b',1)
        lines.append('    {CC_VC4_PROFILE_'+name.upper()+f', {ct}, {gain}, {int(crop)}, '+','.join(str(v) for v in values)+
                     ', '+array(ccm)+', '+array(gamma)+', '+array(shading)+f', {raw_red}, {raw_blue}'+'},')
lines += ['};', 'struct cac_vector { enum cc_vc4_profile profile; int shifts[81][4]; };',
          'static const struct cac_vector cac_vectors[] = {', *cac_vectors, '};']
output='\n'.join(lines)+'\n'; path=root/'tests/pisp_tuning_vectors.h'
if '--check' in sys.argv:
    if path.read_text()!=output: raise SystemExit('PiSP vectors are stale')
else: path.write_text(output)
