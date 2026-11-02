# -*- coding: utf-8 -*-
"""
逐像素比对：RTL 仿真输出（osd_pixels.txt） vs Python 参考模型（render_osd.compose）

判据：307,200 个像素全部一致（0 mismatch）。
用法：python check_pixels.py <sel> <cfg> <setup> <banner>
"""
import sys, io, os

sys.path.insert(0, r"D:/FPGA/VedioCardV2/tools")
import render_osd as R

sel, cfg, setup, banner, hide = (int(a) for a in sys.argv[1:6])

# banner: 1 = 停止提示（7 字，3 秒）；2 = 自动轮播中提示（5 字，1 秒）
# +hide=1 时 RTL 的 hold_cnt 已被 force 到超时，等于不显示
bkind = banner if hide == 0 else 0

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "osd_pixels.txt")
if not os.path.exists(path):
    raise SystemExit("missing osd_pixels.txt — run vvp first")

n = 0
bad = 0
first = []
hist = {}

with io.open(path, "r") as f:
    for line in f:
        p = line.split()
        if len(p) != 4:
            continue
        x, y = int(p[0]), int(p[1])
        sin, sout = p[2], p[3]
        rgb_in = (int(sin[0:2], 16), int(sin[2:4], 16), int(sin[4:6], 16))
        got = (int(sout[0:2], 16), int(sout[2:4], 16), int(sout[4:6], 16))
        exp = R.compose(x, y, rgb_in, R.NEW, sel, cfg, setup != 0, bkind)
        n += 1
        if exp != got:
            bad += 1
            key = (tuple(exp), tuple(got))
            hist[key] = hist.get(key, 0) + 1
            if len(first) < 12:
                first.append((x, y, rgb_in, exp, got))

print("pixels   : %d" % n)
print("mismatch : %d" % bad)
if bad:
    print("\n--- first mismatches (x, y, in, expected, got) ---")
    for r in first:
        print("  (%3d,%3d) in=%02X%02X%02X  exp=%02X%02X%02X  got=%02X%02X%02X"
              % (r[0], r[1], r[2][0], r[2][1], r[2][2],
                 r[3][0], r[3][1], r[3][2], r[4][0], r[4][1], r[4][2]))
    print("\n--- mismatch classes (exp -> got : count) ---")
    for (e, g), c in sorted(hist.items(), key=lambda kv: -kv[1])[:10]:
        print("  %02X%02X%02X -> %02X%02X%02X : %d" % (e[0], e[1], e[2], g[0], g[1], g[2], c))
    sys.exit(1)
print("PASS: RTL 与参考模型逐像素一致")
