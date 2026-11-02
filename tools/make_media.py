#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成符合 FPGA 音视频播放器要求的 BMP + WAV 素材对。

约束（与 RTL 的硬校验一致）：
  BMP: 24 位真彩色、未压缩(BI_RGB)、640x480 或 1280x960
  WAV: PCM、单声道、48000 Hz、16 bit

仅依赖 Python 标准库，不需要 Pillow / numpy。

用法：
  python tools/make_media.py --out ./sdcard --index 01
  python tools/make_media.py --out ./sdcard --index 02 --size 1280x960 --seconds 8 --freq 660
"""

import argparse
import math
import os
import struct
import sys


def write_bmp(path, width, height):
    """写一张 24 位未压缩 BMP，内容为双色对角渐变 + 网格参考线。"""
    row_pad = (4 - (width * 3) % 4) % 4
    row_size = width * 3 + row_pad
    pixel_size = row_size * height
    file_size = 14 + 40 + pixel_size

    file_header = b"BM" + struct.pack("<IHHI", file_size, 0, 0, 54)
    info_header = struct.pack(
        "<IiiHHIIiiII",
        40,            # biSize
        width,         # biWidth
        height,        # biHeight（正数 = 自下而上存储）
        1,             # biPlanes
        24,            # biBitCount
        0,             # biCompression = BI_RGB
        pixel_size,    # biSizeImage
        2835, 2835,    # 分辨率 72 DPI
        0, 0,          # 调色板
    )

    # 预生成一行模板，避免逐像素构造 bytes 过慢
    pad = b"\x00" * row_pad
    rows = []
    for y in range(height - 1, -1, -1):          # BMP 自下而上
        vy = y / max(height - 1, 1)
        row = bytearray()
        for x in range(width):
            vx = x / max(width - 1, 1)
            # 背景：深蓝 → 金黄 的对角渐变
            r = int(18 + 200 * (vx * 0.6 + vy * 0.4))
            g = int(26 + 150 * (vx * 0.3 + vy * 0.5))
            b = int(60 + 60 * (1.0 - vx))
            # 每 80 px 画一条浅色网格线，便于肉眼确认缩放与像素对齐
            if x % 80 < 2 or y % 80 < 2:
                r, g, b = min(r + 45, 255), min(g + 45, 255), min(b + 45, 255)
            row += bytes((b, g, r))              # BMP 存 BGR
        rows.append(bytes(row) + pad)

    with open(path, "wb") as f:
        f.write(file_header)
        f.write(info_header)
        for row in rows:
            f.write(row)


def write_wav(path, seconds, sample_rate=48000, freq=1000.0, amplitude=0.35):
    """写一个单声道 16 bit PCM 正弦波，带 30 ms 淡入淡出避免爆音。"""
    n_samples = int(seconds * sample_rate)
    fade = int(0.03 * sample_rate)
    frames = bytearray()

    for i in range(n_samples):
        t = i / sample_rate
        # 叠加二次谐波，听感更接近"提示音"而非纯正弦
        s = math.sin(2 * math.pi * freq * t) + 0.3 * math.sin(4 * math.pi * freq * t)
        s *= amplitude / 1.3
        if i < fade:
            s *= i / fade
        elif i > n_samples - fade:
            s *= max(0.0, (n_samples - i) / fade)
        frames += struct.pack("<h", max(-32768, min(32767, int(s * 32767))))

    data_size = len(frames)
    byte_rate = sample_rate * 2
    header = b"RIFF" + struct.pack("<I", 36 + data_size) + b"WAVE"
    header += b"fmt " + struct.pack("<IHHIIHH", 16, 1, 1, sample_rate, byte_rate, 2, 16)
    header += b"data" + struct.pack("<I", data_size)

    with open(path, "wb") as f:
        f.write(header)
        f.write(frames)


def parse_size(text):
    try:
        w, h = text.lower().split("x")
        return int(w), int(h)
    except Exception:
        raise argparse.ArgumentTypeError("尺寸格式应为 宽x高，例如 640x480")


def main():
    parser = argparse.ArgumentParser(
        description="生成符合 FPGA 音视频播放器要求的 BMP + WAV 素材对",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--out", default="./sdcard", help="输出目录（默认 ./sdcard）")
    parser.add_argument("--index", default="01", help="素材编号，决定文件名（默认 01）")
    parser.add_argument("--size", type=parse_size, default=(640, 480),
                        help="BMP 尺寸，640x480 或 1280x960（默认 640x480）")
    parser.add_argument("--seconds", type=float, default=5.0, help="WAV 时长秒数（默认 5）")
    parser.add_argument("--freq", type=float, default=1000.0, help="正弦频率 Hz（默认 1000）")
    args = parser.parse_args()

    w, h = args.size
    if (w, h) not in ((640, 480), (1280, 960)):
        print(f"[警告] {w}x{h} 不在固件支持的 640x480 / 1280x960 之内，装载会失败。",
              file=sys.stderr)

    os.makedirs(args.out, exist_ok=True)
    bmp_path = os.path.join(args.out, f"{args.index}.BMP")
    wav_path = os.path.join(args.out, f"{args.index}.WAV")

    write_bmp(bmp_path, w, h)
    write_wav(wav_path, args.seconds, freq=args.freq)

    for p in (bmp_path, wav_path):
        print(f"生成 {p}  ({os.path.getsize(p):,} 字节)")

    print("\n下一步：把这两个文件拷到 TF 卡【根目录】（必须是 FAT32），插卡上电。")


if __name__ == "__main__":
    main()
