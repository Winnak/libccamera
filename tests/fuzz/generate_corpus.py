#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""Regenerate the small checked-in seeds; Python is not needed to run fuzzers."""
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parent / "corpus"


def pack(fmt, *values):
    return struct.pack("<" + fmt, *values)


def seed(target, name, data):
    directory = ROOT / target
    directory.mkdir(parents=True, exist_ok=True)
    (directory / name).write_bytes(data)


for size in [0, 1, 8, 65536, 65537, 0xFFFFFFFF]:
    for fds in [0, 1, 253, 254, 255]:
        seed("ipc", f"header-{size}-{fds}", pack("IBBBB", size, fds, 0, 0, 0))
for mode in [0, 1, 2, 3]:
    seed("ipc", f"ancillary-{mode}", bytes([mode]) + pack("IBBBBBH", 64, 1, 0, 0, 0, 0, 253) + bytes(1024))
seed("ipc", "pidfd-ancillary-64", pack("Qiii", 20, 1, 4, 3) + bytes(4))
seed("ipc", "pidfd-ancillary-32", pack("Iiii", 16, 1, 4, 3))


for offset in [0, 4, 16, 40, 128, 192, 256, 512, 536, 640, 768, 780, 1036, 1536, 2047]:
    seed("ipa_module", f"patch-{offset}", bytes([1]) + pack("IB", offset, 7) + bytes(8))
seed("ipa_module", "valid", bytes([0]))
seed("ipa_module", "truncated", bytes([8, 255, 3]))
for mode in [0, 1, 2, 3, 4, 5, 7, 8, 16, 32, 64, 65, 67]:
    seed("ipa_manager", f"mode-{mode}", bytes([mode]) + b":/priority::/second:")
seed("ipa_manager", "too-many-directories", bytes([1]) + b"/x:" * 65)

for flags in [0, 1, 2, 4, 8, 16, 32, 64]:
    data = bytes([flags, 3, 3]) + pack("II", 640, 480) * 4 + bytes([1, 1])
    for role in [0, 1, 2]:
        data += bytes([role, 0]) + pack("III", 1920, 1080, 4)
    seed("configuration", f"flags-{flags}", data)

for action in ["add", "remove", "change", "bind", "unbind"]:
    for subsystem, node in [("video4linux", "video7"), ("media", "media2")]:
        packet = f"{action}@/devices/test\0ACTION={action}\0DEVPATH=/devices/test\0SUBSYSTEM={subsystem}\0DEVNAME={node}\0"
        seed("monitor", f"{action}-{subsystem}", b"\0" + packet.encode())
for mask in [0x100, 0x200, 0x40, 0x80, 0x4, 0x4000, 0x8000]:
    seed("monitor", f"inotify-{mask:x}", b"\x01" + pack("iIII", 0, mask, 0, 8) + b"video7\0\0")
for node in [b"video0", b"v4l-subdev999", b"../video0", b"media", b"media2/child", b"media42"]:
    seed("monitor", f"node-{node.hex()}", b"\x02" + node)


for af_mode in [0, 1, 2, 4, 6]:
    seed("af", f"mode-{af_mode}", bytes([af_mode, 63]) + bytes(range(256)) * 3)

for packing in range(4):
    for mode in [0, 1, 2, 3]:
        seed("embedded", f"packing-{packing}-mode-{mode}",
             bytes([mode, 0, packing]) + bytes(8) + bytes(range(256)) * 12)

for mode in [0, 1, 2, 4, 8, 12, 16, 64, 66, 68]:
    seed("routing", f"mode-{mode}", pack("BI", mode, 37) + bytes(range(256)))


def pisp(name, width, height, kind=0, *, second=False, crop=False, scale=None, flip=0, tuning=None, d0=False,
         compression=0, order=0):
    mode = ((4 if second else 0) | (8 if crop else 0) | (16 if scale else 0) | (2 if tuning else 0) |
            (64 if d0 else 0) | (128 if compression else 0))
    raw = order + (4 * (compression - 1) if compression else 0)
    data = pack("BHHB", mode, (width - 16) // 2, (height - 16) // 2, raw)
    if compression:
        data += pack("H", 2048)
    for branch in range(2 if second else 1):
        if crop:
            data += pack("HH", 8, 16)  # even offsets, with the rest of the frame retained
        data += pack("B", (kind + branch) % 29)
        if scale:
            data += pack("HH", (scale[0] - 16) // 2, (scale[1] - 16) // 2)
        data += pack("B", flip)
    if tuning:
        data += pack("II", *tuning) + bytes(5)  # one zero-byte patch at offset zero
    seed("pisp", name, data)


for kind in range(29):
    pisp(f"layout-{kind:02d}", 1640, 1232, kind, second=True, flip=kind % 4,
         d0=23 <= kind <= 26 or 23 <= (kind + 1) % 29 <= 26)
for w, h in [(16, 16), (640, 480), (642, 3074), (4608, 2592), (16384, 16384)]:
    pisp(f"geometry-{w}-{h}", w, h)
pisp("crop-resize-flip", 1640, 1232, second=True, crop=True, scale=(1280, 720), flip=3)
pisp("strong-downscale", 4608, 2592, 8, second=True, scale=(320, 240))
for compression in [1, 2, 3]:
    for order in range(4):
        pisp(f"compressed-{compression}-order-{order}", 1642, 1232, second=True, crop=True,
             scale=(1280, 720), flip=order, compression=compression, order=order)
seed("pisp", "regression-inactive-branch", bytes.fromhex("1c2c036024028d080210131200524742caffff10ffff100001"))
# Exercise accepted block masks, including automatic LSC/CAC grid finalization.
for bit in range(32):
    pisp(f"bayer-enable-{bit:02d}", 640, 480, tuning=(1 << bit, 0))
    pisp(f"rgb-enable-{bit:02d}", 640, 480, tuning=(0, 1 << bit))

for w in [64, 640, 6144, 6146, 12288, 12290, 16384]:
    for raw in range(7):
        seed("cfe", f"width-{w}-raw-{raw}", pack("BHHB", 0, (w - 64) // 2, 208, raw))
for bit in range(32):
    seed("cfe", f"enable-{bit:02d}", pack("BHHBI", 2, 288, 208, 0, 1 << bit) + bytes(5))
for compression in [1, 2, 3]:
    for raw in range(7):
        seed("cfe", f"compressed-{compression}-raw-{raw}",
             pack("BHHBH", 4 if compression == 1 else 12 if compression == 2 else 36, 789, 584, raw, 2048))
        seed("cfe", f"crop-scale-dual-{compression}-raw-{raw}",
             pack("BHHBH", 20 if compression == 1 else 28 if compression == 2 else 52, 789, 584, raw, 2048))
for raw in range(4, 20):
    seed("pisp", f"lsb-mono-{raw}", pack("BHHBBB", 0, 812, 608, raw, 29, 3))
for raw in range(12, 24):
    seed("pisp", f"compressed-mono-{raw}", pack("BHHBHBB", 128, 812, 608, raw, 2048, 30, 3))
for raw in range(12):
    seed("pisp", f"tdn-stitch-compressed-{raw}", pack("BHHBHBB", 160, 813, 608, raw, 2048, 0, 0))
seed("pisp", "tdn-stitch-rgb-crop", pack("BHHBHHBB", 40, 813, 608, 0, 400, 300, 0, 0))

for model in range(11):
    for special, bits in [("gain", 0x4000000000000000), ("inf", 0x7FF0000000000000),
                          ("nan", 0x7FF8000000000001)]:
        data = pack("BBQIIIIQI", 2, model, 96000000, 1920, 1080, 280, 45, 10000000, 436)
        data += pack("QIQQQ", bits, 127, 16, 232, 3) + b"imx708_wide_noir 0-001a"
        seed("sensor", f"model-{model:02d}-{special}", data)
seed("sensor", "overflow", bytes([7, 0]) + b"\xff" * 100)
seed("sensor", "zero-rate", bytes(102))

for count in [1, 4, 32]:
    for wrap in [0, 1]:
        data = pack("BB", wrap, count - 1)
        for i in range(count):
            data += pack("BQ", i * 2, i)
        data += bytes([191])  # enough commits to expire the 128-frame history
        for step in range(192):
            data += pack("BBI", 0, step % 16, (1 << count) - 1 if step < 2 else 0)
            if step < 2:
                for i in range(count):
                    data += pack("Q", step * 100 + i)
            data += pack("Q", step + 1)
        seed("delayed", f"controls-{count}-wrap-{wrap}", data)

for stream_type in range(6):
    multi = stream_type in [2, 3]
    for planes in ([1, 3, 8] if multi else [1]):
        for mode in [0, 1, 2, 4, 8, 16, 32, 63]:
            data = bytes([mode, stream_type]) + (bytes([planes - 1]) if multi else b"")
            data += pack("H", 4095) * (2 * planes)
            if mode & 1:
                data += pack("IB", 8, 3) + bytes(4)  # invalid FOURCC
            data += bytes((5 if mode & 2 else 3) * planes if multi else (4 if mode & 2 else 2))
            if mode & 4:
                data += pack("I", 0xFFFFFFFF)
            data += pack("IIQQ", 123, 0, 0xFFFFFFFFFFFFFFFF if mode & 8 else 42, 1000000 if mode & 8 else 123456)
            if multi and mode & 16:
                data += pack("I", 255)
            for _ in range(planes):
                data += pack("II", 0xFFFFFFFF, 1) if mode & 32 else pack("HB", 2048, 16)
            seed("stream", f"type-{stream_type}-planes-{planes}-mode-{mode}", data)

for mode in [0, 1, 2, 4, 8, 16, 32, 64, 65, 66]:
    for kind in range(6):
        data = pack("BBBI", mode, 8, kind % 4, 0xFFFFFFFF)
        for i in range(8):
            flags = i % 4  # scalar/payload/layout-changing combinations
            data += pack("BB", kind, flags)
            data += pack("I", 5) if mode & 8 else bytes([2])
            data += pack("IIQIQ", 4, 1, 0xFFFFFFFFFFFFFFFF, 128, i)
            if flags & 1:
                data += bytes([63])
        seed("controls", f"mode-{mode}-kind-{kind}", data)
seed("controls", "empty-batch", bytes(256))

seed("buffers", "one-plane", bytes(range(16)) + bytes([0, 0, 1, 3, 0, 16, 0, 0, 0]) + bytes(900))
seed("buffers", "many-planes", bytes(range(16)) + bytes([1, 0, 8]) + bytes(range(64)) * 16)
seed("buffers", "alias-and-errors", bytes(16) + bytes(range(255)) * 4)
seed("fences", "active-fd-zero", bytes(1024))
seed("fences", "statuses-and-errors", bytes(range(256)) * 4)

for mode in [0, 1, 2, 4, 8, 16, 32, 64, 255]:
    for flags in [0, 8, 16, 127]:
        data = bytearray(160)
        data[0], data[1], data[51], data[56] = mode, flags, 7, 7
        seed("uvc", f"mode-{mode:03d}-flags-{flags:03d}", data)

for mode in [0, 1, 2, 4, 8, 16, 64, 128]:
    data = bytes([mode, 7, 12])  # eight entities, thirteen links
    if mode & 1:
        data += pack("I", 7)  # repeated/out-of-order entity ID
    if mode & 2:
        data += pack("H", 4097)  # pad-count limit
    for i in range(13):
        data += bytes([i % 8, (i + 1) % 8, i % 4])  # cycles, alternate paths, immutable links
        if mode & 8:
            data += pack("HH", 0xFFFF, 0xFFFF)
        if mode & 16:
            data += pack("I", 0xFFFFFFFF)  # dangling sink
    seed("media", f"graph-mode-{mode}", data)
seed("media", "single-entity", bytes(3))
for mode in [0, 1, 2, 3, 4, 5, 16, 32, 64]:
    seed("media", f"v2-mode-{mode}", bytes([32, mode, 7, 20]) + bytes(range(256)) * 3)

for mode in [0, 1, 2, 4, 8, 16, 24, 31]:
    seed("ipa", f"mode-{mode}", bytes([mode]) + bytes(128))
