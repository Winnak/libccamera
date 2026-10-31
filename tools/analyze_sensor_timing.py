#!/usr/bin/env python3
"""Infer candidate frame delays from ccamera-still --diagnostic JSONL.

Inference assumes a stationary, uniformly lit, unsaturated scene. It is never
an embedded sensor measurement and never changes library defaults or files.
"""
import argparse
import collections
import json
import statistics
from pathlib import Path


def analyze(records):
    config = next((r for r in records if r.get('type') == 'configuration'), {})
    if config.get('status', 0) or config.get('trace_dropped', 0):
        return {'valid': False, 'reason': 'Capture failed or trace records were lost.'}
    kind = config.get('diagnostic')
    if kind not in ('exposure', 'gain', 'vblank'):
        return {'valid': False, 'reason': 'Use an exposure, gain or vblank diagnostic run.'}
    frames = sorted((r for r in records if r.get('type') == 'frame'), key=lambda r: r['sequence'])
    if len(frames) < 24:
        return {'valid': False, 'reason': 'Too few captured frames.'}
    # Sequence gaps can hide the first affected frame. Reject, rather than guess.
    if any(b['sequence'] != a['sequence'] + 1 for a, b in zip(frames, frames[1:])):
        return {'valid': False, 'reason': 'Source sequence gaps prevent reliable first-frame inference.'}
    values = {}
    for i, r in enumerate(frames):
        if kind == 'vblank':
            # The SOE-to-next-SOE interval belongs to this source frame.
            if i + 1 < len(frames):
                values[r['sequence']] = frames[i + 1]['timestamp_ns'] - r['timestamp_ns']
        else:
            values[r['sequence']] = r['luminance']
    observations = []
    for w in (r for r in records if r.get('type') == 'write'):
        if w.get('error'):
            continue
        seq = w['sequence']
        before = [values[i] for i in range(seq - 4, seq) if i in values]
        after = [values[i] for i in range(seq + 8, seq + 13) if i in values]
        if len(before) < 3 or len(after) < 3:
            continue
        low, high = statistics.median(before), statistics.median(after)
        scale = max(abs(low), abs(high), 1e-12)
        noise = max([abs(v-low) for v in before] + [abs(v-high) for v in after])
        delta = high-low
        if abs(delta) < max(0.05*scale, 5*noise):
            continue
        threshold = (low+high)/2
        for offset in range(9):
            value = values.get(seq+offset)
            if value is not None and (value >= threshold if delta > 0 else value <= threshold):
                observations.append({'write_sequence': seq, 'target': w['target'], 'delay': offset,
                                     'before': low, 'after': high})
                break
    counts = collections.Counter(o['delay'] for o in observations)
    candidate, agreement = counts.most_common(1)[0] if counts else (None, 0)
    accepted = len(observations) >= 3 and agreement >= 0.8*len(observations)
    return {'valid': accepted, 'inferred_only': True, 'control': kind,
            'suggested_delay': candidate if accepted else None,
            'reason': 'Consistent transitions; verify the raw captures before using this override.' if accepted
                      else 'Need at least three clean transitions with 80% agreement.',
            'observations': observations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.log.read_text().splitlines() if line.strip()]
    result = analyze(records)
    print(json.dumps(result, indent=2))
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
