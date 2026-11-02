# -*- coding: utf-8 -*-
"""
OSD 效果模拟渲染器（参考模型）
从 osd_font_rom.v 解析真实字库，按 osd_menu.v 的布局参数与合成优先级
逐像素渲染出 640x480 PNG。

★ 本文件同时是 osd_menu.v 的【参考模型】★
   compose() 与 RTL 的 mix always 块一一对应，参数用同一套名字。
   所以 "渲染图 == RTL 输出" 是可以被 iverilog 仿真逐像素验证的，
   而不是靠肉眼看着像。

用法：
  python render_osd.py current      # 渲染当前实现（含已知重叠缺陷）
  python render_osd.py proposed     # 渲染美化方案（c34 目标）
"""
import io, os, re, struct, sys

W, H = 640, 480

# 全部按仓库相对布局解析，克隆到任意路径都能跑
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
FONT_V = os.path.join(_REPO, "hardware", "hdmi_media_player", "src",
                      "user_source", "hdl_source", "osd_font_rom.v")
OUT_DIR = os.path.join(_REPO, "docs", "images")

# ---------------- 字库 ----------------
def load_font():
    s = io.open(FONT_V, encoding="utf-8").read()
    rom = {}
    for m in re.finditer(r"rom\[\s*(\d+)\]\s*=\s*16'h([0-9A-Fa-f]{4})", s):
        rom[int(m.group(1))] = int(m.group(2), 16)
    return rom

ROM = load_font()

# 字形槽编号（与 osd_menu.v 的 localparam 一致）
C = dict(SPACE=10, N0=0, N1=1, N2=2, N3=3,
         JIANG=17, JIE=18, MO=19, SHI=20, KUAI=21, SU=22, LIU=23, LAN=24,
         MAN=25, GUAN=26, KAN=27, SHOU=28, DONG=29, YI=30, QUE=31,
         LUN=32, BO=33, SHE=34, ZHI=35, XUAN=36, ZE=37, AN=38, JIAN=39,
         TING=40, ZHI2=41, ZI=42, MIAO=43, KAI=46, ZHONG=54,
         KUOH0=60, KUOH1=61)

# 菜单 4 个选项的 8 字格内容（不足补空格）
ITEMS = [
    [C["JIANG"], C["JIE"], C["MO"], C["SHI"]],
    [C["KUAI"], C["SU"], C["LIU"], C["LAN"], C["KUOH0"], C["N3"], C["MIAO"], C["KUOH1"]],
    [C["MAN"], C["GUAN"], C["KAN"], C["KUOH0"], C["N1"], C["N0"], C["MIAO"], C["KUOH1"]],
    [C["SHOU"], C["DONG"], C["LIU"], C["LAN"]],
]
TITLE = [C["LUN"], C["BO"], C["SHE"], C["ZHI"]]
HINT = [C["N2"], C["XUAN"], C["ZE"], C["SPACE"], C["SPACE"], C["N3"], C["SHE"], C["ZHI"]]
CFG0 = [C["YI"], C["XUAN"]]

# 「已选XXX」后缀（mcol 2..5）
CFG_SUFFIX = {
    0: [C["JIANG"], C["JIE"], C["MO"], C["SHI"]],
    1: [C["KUAI"], C["SU"], C["LIU"], C["LAN"]],
    2: [C["MAN"], C["GUAN"], C["KAN"], C["SPACE"]],
    3: [C["SHOU"], C["DONG"], C["LIU"], C["LAN"]],
}

BANNER_CUR = [C["YI"], C["TING"], C["ZHI2"], C["ZI"], C["DONG"], C["LUN"], C["BO"]]
BANNER_NEW = [C["ZI"], C["DONG"], C["LUN"], C["BO"], C["YI"], C["TING"], C["ZHI2"]]
# 「自动轮播中」5 字 —— 自动轮播进行中的短提示（1 秒，短于最短轮播周期 3 秒）
BANNER_AUTO = [C["ZI"], C["DONG"], C["LUN"], C["BO"], C["ZHONG"]]


def gb(code, row, col):
    """取字形 (code,row) 的第 col 位；bit15 = 最左像素。空格槽读不到 → 0"""
    if code is None or row is None or col is None:
        return 0
    return (ROM.get(code * 16 + row, 0) >> (15 - col)) & 1


# ============================================================
# 参数集：current / proposed
#   键名与 osd_menu.v 的 localparam 对齐
# ============================================================
CUR = dict(
    name="current",
    MX0=40, MY0=120, MW=560, MH=236,
    TX=24, TITLE_Y=14, TITLE_LINE_Y=None, ITEM0_Y=56, ITEM_DY=32,
    CFG_Y=168, HINT_Y=196,
    SEL_PAD_Y=0, SEL_ROW_H=16, SEL_X0=24 - 8, SEL_X1=560 - 8, BAR_W=0,
    CFG_BG_TOP=None, CFG_BG_BOT=None, CFG_BG_X0=0, CFG_BG_X1=560,
    BX0=120, BY0=400, BW=400, BH=56, BTX=16, BTY=20,
    BANNER_BG="sel", BANNER=BANNER_CUR, BANNER_AUTO=BANNER_CUR, BTX5=16,
    HAS_BRD_LO=False, HAS_SEL_BAR=False, HAS_CFG_BG=False,
    HAS_TITLE_LINE=False, HAS_BANNER_BAR=False, DIM_UNSEL=False,
    HAS_EDGE=False, MASK_SHIFT=1, MASK_BASE=(0, 0, 0),
    C_BORDER=(0xC0, 0xC0, 0xC0),
    C_BORDER_LO=(0x34, 0x3A, 0x48),
    C_TEXT=(0xFF, 0xFF, 0xFF),
    C_SEL_BG=(0x2E, 0x6B, 0xD8),
    C_TITLE=(0x66, 0xCC, 0xFF),
    C_HINT=(0xB0, 0xB0, 0xB0),
    C_CFG=(0x66, 0xFF, 0x99),
    C_CFG_BG=(0x22, 0x28, 0x36),
    C_ACCENT=(0xF0, 0xC0, 0x40),
    C_DIM=(0x78, 0x80, 0x91),
    C_OUTLINE=(0x0A, 0x0C, 0x12),
)

NEW = dict(
    name="proposed",
    MX0=80, MY0=72, MW=480, MH=336,
    TX=32, TITLE_Y=24, TITLE_LINE_Y=48, ITEM0_Y=68, ITEM_DY=44,
    CFG_Y=252, HINT_Y=300,
    SEL_PAD_Y=8, SEL_ROW_H=32, SEL_X0=16, SEL_X1=480 - 16, BAR_W=4,
    CFG_BG_TOP=252 - 8, CFG_BG_BOT=252 + 24, CFG_BG_X0=16, CFG_BG_X1=480 - 16,
    BX0=160, BY0=424, BW=320, BH=44,
    BTX=(320 - 7 * 16) // 2, BTY=14,
    BANNER_BG="panel", BANNER=BANNER_NEW,
    BANNER_AUTO=BANNER_AUTO, BTX5=(320 - 5 * 16) // 2,
    HAS_BRD_LO=True, HAS_SEL_BAR=True, HAS_CFG_BG=True,
    HAS_TITLE_LINE=True, HAS_BANNER_BAR=True, DIM_UNSEL=True,
    # 遮罩：base + (bg >> shift) —— shift=3 即 87.5% 遮罩
    HAS_EDGE=True, MASK_SHIFT=3, MASK_BASE=(12, 15, 22),
    C_BORDER=(0x6E, 0x78, 0x8C),
    C_BORDER_LO=(0x45, 0x4E, 0x63),
    C_TEXT=(0xEB, 0xEE, 0xF5),
    C_SEL_BG=(0x26, 0x3E, 0x6E),
    C_TITLE=(0xF0, 0xC0, 0x40),
    C_HINT=(0x9A, 0xA3, 0xB4),
    C_CFG=(0x56, 0xC8, 0xBE),
    C_CFG_BG=(0x2E, 0x36, 0x48),
    C_ACCENT=(0xF0, 0xC0, 0x40),
    C_DIM=(0x9A, 0xA3, 0xB4),
    C_OUTLINE=(0x0A, 0x0C, 0x12),
)


# ============================================================
# 参考模型：与 osd_menu.v 的 mix always 块一一对应
# ============================================================
def compose(x, y, rgb, P, menu_sel, cfg_sel, setup, banner):
    """banner: 0 = 不显示；1 = 停止提示（7 字）；2 = 自动轮播中提示（5 字）"""
    b_show = (banner == 1) or (banner == 2)
    b_auto = (banner == 2)
    btx = P["BTX5"] if b_auto else P["BTX"]
    btw = 80 if b_auto else 112
    blist = P["BANNER_AUTO"] if b_auto else P["BANNER"]

    # 面板底 = 暗基色 + 背景右移 shift（模拟 RTL 的 panel_bg）
    #   shift=1 + base 0     → 改前的 50% 减半
    #   shift=3 + base 12/15/22 → 改后的 87.5% 暗玻璃
    _s = P["MASK_SHIFT"]
    _b = P["MASK_BASE"]
    panel_bg = (_b[0] + (rgb[0] >> _s),
                _b[1] + (rgb[1] >> _s),
                _b[2] + (rgb[2] >> _s))

    # ---------- 菜单几何 ----------
    in_menu = setup and (P["MX0"] <= x < P["MX0"] + P["MW"]) \
                     and (P["MY0"] <= y < P["MY0"] + P["MH"])
    mx = x - P["MX0"] if in_menu else -1
    my = y - P["MY0"] if in_menu else -1

    menu_border = in_menu and (mx < 2 or mx >= P["MW"] - 2 or
                               my < 2 or my >= P["MH"] - 2)
    in_border_lo = P["HAS_BRD_LO"] and in_menu and \
        (mx == 2 or mx == P["MW"] - 3 or my == 2 or my == P["MH"] - 3)

    in_txt_col = in_menu and (P["TX"] <= mx < P["TX"] + 128)

    # 行距比较链（与 RTL 的 item_row / item_base 一致，支持非 2 的幂行距）
    #   ★ RTL 里 item_off 是 12 位无符号：my < ITEM0_Y 时下溢成大数，
    #     比较结果自然落到"第 3 行之外"，这里必须模拟同样的语义 ★
    off = my - P["ITEM0_Y"] if in_menu else 0xFFFF
    off_u = off if off >= 0 else (0x1000 + off)
    d1, d2, d3 = P["ITEM_DY"], P["ITEM_DY"] * 2, P["ITEM_DY"] * 3
    d4 = P["ITEM_DY"] * 4
    item_base = 0 if off_u < d1 else d1 if off_u < d2 else d2 if off_u < d3 else d3
    item_row = 0 if off_u < d1 else 1 if off_u < d2 else 2 if off_u < d3 else 3
    item_sub = off_u - item_base

    in_title = in_txt_col and (P["TITLE_Y"] <= my < P["TITLE_Y"] + 16)
    in_title_line = P["HAS_TITLE_LINE"] and in_menu and (my == P["TITLE_LINE_Y"]) \
        and (P["TX"] <= mx < P["MW"] - P["TX"])
    in_item = in_txt_col and (off_u < d4) and (item_sub < 16)
    in_hint = in_txt_col and (P["HINT_Y"] <= my < P["HINT_Y"] + 16)
    in_cfg = in_txt_col and (P["CFG_Y"] <= my < P["CFG_Y"] + 16)

    sel_top = P["ITEM0_Y"] - P["SEL_PAD_Y"] + item_base
    in_sel_row = in_menu and (not menu_border) and (item_row == menu_sel) and \
        (sel_top <= my < sel_top + P["SEL_ROW_H"]) and \
        (P["SEL_X0"] <= mx < P["SEL_X1"])
    in_sel_bar = P["HAS_SEL_BAR"] and in_sel_row and (mx < P["SEL_X0"] + P["BAR_W"])

    in_cfg_bg = P["HAS_CFG_BG"] and in_menu and \
        (P["CFG_BG_TOP"] <= my < P["CFG_BG_BOT"]) and \
        (P["CFG_BG_X0"] <= mx < P["CFG_BG_X1"])

    # 最外 1px 暗轮廓（"外暗 + 内亮"双描边）
    menu_edge = P["HAS_EDGE"] and in_menu and \
        (mx == 0 or mx == P["MW"] - 1 or my == 0 or my == P["MH"] - 1)

    # ---------- 提示条几何 ----------
    in_bb = b_show and (P["BX0"] <= x < P["BX0"] + P["BW"]) \
                     and (P["BY0"] <= y < P["BY0"] + P["BH"])
    bb_border = in_bb and (x < P["BX0"] + 2 or x >= P["BX0"] + P["BW"] - 2 or
                           y < P["BY0"] + 2 or y >= P["BY0"] + P["BH"] - 2)
    bx = x - P["BX0"] if in_bb else -1
    by = y - P["BY0"] if in_bb else -1
    in_bb_bar = P["HAS_BANNER_BAR"] and in_bb and (bx < P["BAR_W"])
    banner_edge = P["HAS_EDGE"] and in_bb and \
        (bx == 0 or bx == P["BW"] - 1 or by == 0 or by == P["BH"] - 1)
    in_bb_txt = in_bb and (btx <= bx < btx + btw) and \
        (P["BTY"] <= by < P["BTY"] + 16)

    # ---------- 字形寻址（与 RTL 的 fcode/frow/fcol 优先级一致）----------
    if in_bb_txt:
        bc = bx - btx
        code = blist[bc >> 4] if (bc >> 4) < len(blist) else C["SPACE"]
        fpix = gb(code, by - P["BTY"], (bx - btx) & 15)
    else:
        bcol = (mx - P["TX"]) >> 4
        code = C["SPACE"]
        if in_title:
            code = TITLE[bcol] if bcol < len(TITLE) else C["SPACE"]
        elif in_item:
            code = ITEMS[item_row][bcol] if bcol < len(ITEMS[item_row]) else C["SPACE"]
        elif in_cfg:
            if bcol == 0:
                code = CFG0[0]
            elif bcol == 1:
                code = CFG0[1]
            elif 2 <= bcol <= 5:
                code = CFG_SUFFIX[cfg_sel][bcol - 2]
            else:
                code = C["SPACE"]
        elif in_hint:
            code = HINT[bcol] if bcol < len(HINT) else C["SPACE"]

        if in_title:
            row = (my - P["TITLE_Y"]) & 15
        elif in_item:
            row = item_sub & 15
        elif in_cfg:
            row = (my - P["CFG_Y"]) & 15
        else:
            row = (my - P["HINT_Y"]) & 15
        fpix = gb(code, row, (mx - P["TX"]) & 15)

    menu_txt_px = in_menu and (in_title or in_item or in_hint or in_cfg) and fpix
    banner_txt_px = in_bb_txt and fpix

    # ---------- 文字颜色 ----------
    if in_item and P["DIM_UNSEL"] and item_row != menu_sel:
        txt_col = P["C_DIM"]
    elif in_item:
        txt_col = P["C_TEXT"]
    elif in_title:
        txt_col = P["C_TITLE"]
    elif in_hint:
        txt_col = P["C_HINT"]
    elif in_cfg:
        txt_col = P["C_CFG"]
    else:
        txt_col = P["C_TEXT"]

    # ---------- 合成优先级（高 -> 低）----------
    if banner_txt_px:
        return P["C_TEXT"]
    if in_bb_bar:
        return P["C_ACCENT"]
    if banner_edge:
        return P["C_OUTLINE"]
    if in_bb and not bb_border:
        return P["C_SEL_BG"] if P["BANNER_BG"] == "sel" else panel_bg
    if bb_border:
        return P["C_BORDER"]
    if menu_txt_px:
        return txt_col
    if in_sel_bar:
        return P["C_ACCENT"]
    if in_sel_row:
        return P["C_SEL_BG"]
    if in_cfg_bg:
        return P["C_CFG_BG"]
    if in_title_line:
        return P["C_BORDER_LO"]
    if menu_edge:
        return P["C_OUTLINE"]
    if menu_border:
        return P["C_BORDER"]
    if in_border_lo:
        return P["C_BORDER_LO"]
    if in_menu:
        return panel_bg
    return rgb


def bg_gradient():
    """模拟一张图片背景（深色渐变 + 轻微纹理），让 OSD 有真实对比感"""
    out = []
    for y in range(H):
        row = []
        for x in range(W):
            b = 20 + int(28 * (y / H)) + int(12 * (x / W))
            row.append((b, b + 3, b + 8))
        out.append(row)
    return out


def render(P, path, menu_sel=0, cfg_sel=0, setup=True, banner=True):
    bg = bg_gradient()
    px = [[compose(x, y, bg[y][x], P, menu_sel, cfg_sel, setup, banner)
           for x in range(W)] for y in range(H)]
    save_png(path, px)


def save_png(path, px):
    import zlib as _z
    h = len(px)
    w = len(px[0])
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        for x in range(w):
            r, g, b = px[y][x]
            raw += bytes((r, g, b))
    comp = _z.compress(bytes(raw), 9)

    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data
                + struct.pack(">I", _z.crc32(typ + data) & 0xffffffff))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", comp)
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)
    print("  saved: %s (%d bytes)" % (path, len(png)))


def save_bmp(path, px):
    row = W * 3
    pad = (4 - row % 4) % 4
    data = bytearray()
    for y in range(H - 1, -1, -1):          # BMP 自下而上
        for x in range(W):
            r, g, b = px[y][x]
            data += bytes((b, g, r))
        data += b"\x00" * pad
    size = 54 + len(data)
    hdr = b"BM" + struct.pack("<IHHI", size, 0, 0, 54)
    dib = struct.pack("<IiiHHIIiiII", 40, W, H, 1, 24, 0, len(data), 2835, 2835, 0, 0)
    with open(path, "wb") as f:
        f.write(hdr + dib + data)
    print("  saved: %s (%d bytes)" % (path, size))


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "current"
    if mode == "current":
        render(CUR, OUT_DIR + r"/osd_preview_current.png")
    elif mode == "proposed":
        render(NEW, OUT_DIR + r"/osd_preview_proposed.png")
    else:
        print("usage: render_osd.py current|proposed")
