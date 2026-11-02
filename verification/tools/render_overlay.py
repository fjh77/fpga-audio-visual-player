# -*- coding: utf-8 -*-
"""
osd_overlay.v 的参考模型（亮度/音量调节面板）。

与 render_osd.py 同一套路：compose_panel() 严格对应 RTL 的 mix always 块，
用来和 iverilog 的整帧 dump 做逐像素比对。
"""
import io, os, re

W, H = 640, 480

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
FONT_V = os.path.join(_REPO, "hardware", "hdmi_media_player", "src",
                      "user_source", "hdl_source", "osd_font_rom.v")


def load_font():
    s = io.open(FONT_V, encoding="utf-8").read()
    rom = {}
    for m in re.finditer(r"rom\[\s*(\d+)\]\s*=\s*16'h([0-9A-Fa-f]{4})", s):
        rom[int(m.group(1))] = int(m.group(2), 16)
    return rom


ROM = load_font()

# 面板几何（与 osd_overlay.v 的 localparam 一致）
PX0, PY0, PW, PH = 180, 380, 280, 64
TXT_OFF, TXT_Y, VAL_X = 16, 10, 216
BAR_X, BAR_Y, BAR_W, BAR_H = 16, 36, 248, 16

C_BORDER = (0xC0, 0xC0, 0xC0)
C_TEXT = (0xFF, 0xFF, 0xFF)
C_OUTLINE = (0x0A, 0x0C, 0x12)
MASK_R, MASK_G, MASK_B = 12, 15, 22
MASK_SHIFT = 3

C_LIANG, C_DU, C_YIN, C_LIANG2 = 11, 12, 15, 16
C_SPACE = 10


def gb(code, row, col):
    return (ROM.get(code * 16 + row, 0) >> (15 - col)) & 1


def compose_panel(x, y, rgb, sel, val, show):
    if not show:
        return rgb
    if not (PX0 <= x < PX0 + PW and PY0 <= y < PY0 + PH):
        return rgb

    panel_bg = (MASK_R + (rgb[0] >> MASK_SHIFT),
                MASK_G + (rgb[1] >> MASK_SHIFT),
                MASK_B + (rgb[2] >> MASK_SHIFT))

    in_border = (x < PX0 + 2 or x >= PX0 + PW - 2 or
                 y < PY0 + 2 or y >= PY0 + PH - 2)
    panel_edge = (x == PX0 or x == PX0 + PW - 1 or
                  y == PY0 or y == PY0 + PH - 1)

    # ---------- 进度条 ----------
    in_bar = (PX0 + BAR_X <= x < PX0 + BAR_X + BAR_W and
              PY0 + BAR_Y <= y < PY0 + BAR_Y + BAR_H)
    bx = x - (PX0 + BAR_X)
    by = y - (PY0 + BAR_Y)
    bar_border = in_bar and (bx < 2 or bx >= BAR_W - 2 or
                             by < 2 or by >= BAR_H - 2)
    fill_len = (val * 156) >> 6
    bar_fill = in_bar and (not bar_border) and bx >= 2 and bx < fill_len + 2

    # ---------- 文字 ----------
    in_name = (PX0 + TXT_OFF <= x < PX0 + TXT_OFF + 32 and
               PY0 + TXT_Y <= y < PY0 + TXT_Y + 16)
    in_val = (PX0 + VAL_X <= x < PX0 + VAL_X + 48 and
              PY0 + TXT_Y <= y < PY0 + TXT_Y + 16)

    nx = x - (PX0 + TXT_OFF)
    ny = y - (PY0 + TXT_Y)
    vx = x - (PX0 + VAL_X)
    vy = y - (PY0 + TXT_Y)

    d2 = val // 100
    rem = val % 100
    d1 = rem // 10
    d0 = rem % 10
    c2 = C_SPACE if d2 == 0 else d2
    c1 = C_SPACE if (d2 == 0 and d1 == 0) else d1
    c0 = d0

    if in_val:
        vidx = (vx >> 4) & 3
        code = c2 if vidx == 0 else (c1 if vidx == 1 else c0)
        fpix = gb(code, vy & 15, vx & 15)
    elif in_name:
        # nx[4] = 1 → 第二个字
        code = (C_LIANG2 if sel else C_DU) if (nx & 16) else (C_YIN if sel else C_LIANG)
        fpix = gb(code, ny & 15, nx & 15)
    else:
        fpix = 0

    in_text = fpix and (in_name or in_val)

    if in_text:
        return C_TEXT
    if bar_fill:
        return C_TEXT
    if panel_edge:
        return C_OUTLINE
    if bar_border or in_border:
        return C_BORDER
    return panel_bg
