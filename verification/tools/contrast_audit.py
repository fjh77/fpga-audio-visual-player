# -*- coding: utf-8 -*-
"""
OSD 可读性审计：按 WCAG 2.1 相对亮度公式，算各前景色在最坏背景下的对比度。

为什么需要它：
  面板底是"背景图 * α + 暗色"，所以面板底的最亮值出现在【纯白图片】上。
  只要纯白图下每个文字色都达到 4.5:1（小字 AA 标准），
  那任何真实图片（几乎没有纯白）都自然达标。

用法：python contrast_audit.py
"""
import sys


def lin(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def lum(rgb):
    return 0.2126 * lin(rgb[0]) + 0.7152 * lin(rgb[1]) + 0.0722 * lin(rgb[2])


def ratio(fg, bg):
    a, b = lum(fg), lum(bg)
    if a < b:
        a, b = b, a
    return (a + 0.05) / (b + 0.05)


def mask_old(bg):
    """改前：逐通道减半（50% 遮罩）"""
    return (bg[0] >> 1, bg[1] >> 1, bg[2] >> 1)


BASE = (12, 15, 22)


def mask_new(bg):
    """改后：暗基色 + 背景/8（87.5% 遮罩）"""
    return (BASE[0] + (bg[0] >> 3),
            BASE[1] + (bg[1] >> 3),
            BASE[2] + (bg[2] >> 3))


# 最坏情况背景：纯白、纯亮黄（G 通道最亮）、中灰、纯黑（对照）
BGS = [("纯白 255,255,255", (255, 255, 255)),
       ("亮黄 255,255,0  ", (255, 255, 0)),
       ("中灰 128,128,128", (128, 128, 128)),
       ("纯黑 0,0,0      ", (0, 0, 0))]

OLD = [("正文  #FFFFFF", (255, 255, 255)),
       ("正文  #EBEEF5", (235, 238, 245)),
       ("次要  #788091", (120, 128, 145)),
       ("标题  #66CCFF", (102, 204, 255)),
       ("状态  #66FF99", (102, 255, 153)),
       ("边框  #C0C0C0", (192, 192, 192))]

NEW = [("正文  #EBEEF5", (235, 238, 245)),
       ("次要  #9AA3B4", (154, 163, 180)),
       ("标题  #F0C040", (240, 192, 64)),
       ("状态  #56C8BE", (86, 200, 190)),
       ("边框  #6E788C", (110, 120, 140)),
       ("外框  #0A0C12", (10, 12, 18))]


def table(title, colors, masker):
    print("\n=== %s ===" % title)
    print("%-16s %s" % ("前景色", "  ".join("%-18s" % b[0] for b in BGS)))
    for name, fg in colors:
        cells = []
        for _, bg in BGS:
            r = ratio(fg, masker(bg))
            mark = "OK " if r >= 4.5 else ("~  " if r >= 3.0 else "LOW")
            cells.append("%5.2f:1 %s" % (r, mark))
        print("%-16s %s" % (name, "  ".join("%-18s" % c for c in cells)))
    print("  判据：≥4.5 OK（小字 AA） | 3.0~4.5 ~（仅够大字/边框） | <3.0 LOW")


if __name__ == "__main__":
    print("遮罩后最亮的面板底：")
    print("  改前 (bg>>1)          : %s" % (mask_old((255, 255, 255)),))
    print("  改后 (BASE + bg>>3)   : %s" % (mask_new((255, 255, 255)),))
    table("改前：50% 遮罩 + 原配色", OLD, mask_old)
    table("改后：87.5% 遮罩 + 提亮次要色", NEW, mask_new)
