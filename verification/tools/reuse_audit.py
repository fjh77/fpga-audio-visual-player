#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reuse_audit.py —— 与赛题参考实例的同源度审计

用途：
    复现《复用与自研边界声明》里的全部相似度数据。
    判定口径：去注释、去空行、压缩连续空白后，按行做 difflib 序列比对。

用法：
    python tools/reuse_audit.py                 # 全量统计
    python tools/reuse_audit.py --detail        # 附带每个文件的改动量（新增/删除/改）
    python tools/reuse_audit.py --file SD/bmp_read.v   # 只看某个文件

输出：
    分类清单（逐字节相同 / 派生 / 全新）+ 每个派生文件的相似度与改动行数
"""
import argparse
import difflib
import io
import os
import re
import sys

# ---------------------------------------------------------------- 路径配置
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))  # 仓库根目录

REF_SUBPROJECTS = ("lab_ex4_tf", "lab_ex5_i2s")
# 参考实例不随本仓库分发；默认位置只是占位，实际请用 --ref-root 指定
DEFAULT_REF_ROOT = os.path.join(ROOT, "third_party", "reference")
OUR_ROOT = os.path.join(ROOT, "hardware", "hdmi_media_player",
                        "src", "user_source", "hdl_source")

SKIP_DIR_PARTS = (".backup", "_archive", ".backup_")


def normalize(path):
    """去注释、去空行、压缩空白，返回代码行列表"""
    try:
        s = io.open(path, encoding="utf-8", errors="ignore").read()
    except OSError:
        return []
    s = re.sub(r"//[^\n]*", "", s)
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    out = []
    for line in s.splitlines():
        t = re.sub(r"\s+", " ", line).strip()
        if t:
            out.append(t)
    return out


def collect(root):
    """收集 .v/.vh 文件，返回 {相对路径: 绝对路径}"""
    result = {}
    if not os.path.isdir(root):
        return result
    for dirpath, dirnames, filenames in os.walk(root):
        if any(p in dirpath for p in SKIP_DIR_PARTS):
            continue
        for name in filenames:
            if name.endswith((".v", ".vh")):
                full = os.path.join(dirpath, name)
                rel = os.path.relpath(full, root).replace("\\", "/")
                result[rel] = full
    return result


def diff_stat(ref_lines, our_lines):
    """统计把参考改成当前版本所需的 新增/删除/改动 行数"""
    sm = difflib.SequenceMatcher(None, ref_lines, our_lines)
    add = dele = chg = 0
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == "insert":
            add += j2 - j1
        elif tag == "delete":
            dele += i2 - i1
        elif tag == "replace":
            chg += max(i2 - i1, j2 - j1)
    return add, dele, chg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", action="store_true", help="输出改动行数明细")
    ap.add_argument("--file", default=None, help="只审计指定文件（相对 hdl_source）")
    ap.add_argument("--ref-root", default=DEFAULT_REF_ROOT,
                    help="参考实例根目录（含 lab_ex4_tf / lab_ex5_i2s），不随本仓库分发")
    args = ap.parse_args()

    REF_ROOT = args.ref_root
    refs = {}
    for sub in REF_SUBPROJECTS:
        base = os.path.join(REF_ROOT, sub, "src", "user_source", "hdl_source")
        for rel, full in collect(base).items():
            refs[("%s/%s" % (sub, rel))] = full

    ours = collect(OUR_ROOT)

    if not refs:
        print("[ERROR] 找不到参考实例目录：%s" % REF_ROOT)
        print("        参考实例（lab_ex4_tf / lab_ex5_i2s）不随本仓库分发，")
        print("        请自备后用 --ref-root 指定其根目录。")
        return 2
    if not ours:
        print("[ERROR] 找不到工程源码目录：%s" % OUR_ROOT)
        return 2

    if args.file:
        target = args.file.replace("\\", "/")
        if target not in ours:
            print("[ERROR] 工程里没有这个文件：%s" % target)
            return 2
        ours = {target: ours[target]}

    same, derived, brand_new = [], [], []

    for rel, full in sorted(ours.items()):
        base_name = os.path.basename(rel)
        candidates = [(k, v) for k, v in refs.items()
                      if os.path.basename(k) == base_name]
        if not candidates:
            brand_new.append(rel)
            continue

        cur = normalize(full)
        best_sim, best_path = 0.0, None
        for _, ref_full in candidates:
            ref_lines = normalize(ref_full)
            if not ref_lines or not cur:
                continue
            quick = difflib.SequenceMatcher(None, cur, ref_lines).quick_ratio()
            if quick < best_sim - 0.15:
                continue
            sim = difflib.SequenceMatcher(None, cur, ref_lines).ratio()
            if sim > best_sim:
                best_sim, best_path = sim, ref_full

        if best_path is None:
            brand_new.append(rel)
            continue

        if open(full, "rb").read() == open(best_path, "rb").read():
            same.append(rel)
        else:
            derived.append((rel, best_sim, best_path, cur))

    total = len(ours)
    print("=" * 78)
    print(" 同源度审计   工程文件总数 %d" % total)
    print(" 参考实例：%s" % REF_ROOT)
    print("=" * 78)

    print("\n[A] 与参考实例逐字节相同：%d 个" % len(same))
    for rel in same:
        print("      %s" % rel)

    print("\n[B] 派生（有改动）：%d 个" % len(derived))
    print("      %-40s %8s  %s" % ("文件", "相似度", "改动量"))
    for rel, sim, ref_path, cur in sorted(derived, key=lambda x: -x[1]):
        ref_lines = normalize(ref_path)
        a, d, c = diff_stat(ref_lines, cur)
        flag = "  <= 高风险" if sim >= 0.90 else ""
        print("      %-40s %7.1f%%  参考%4d行 新增%3d 删除%3d 改%3d%s"
              % (rel, sim * 100, len(ref_lines), a, d, c, flag))

    print("\n[C] 参考实例中不存在（全新）：%d 个" % len(brand_new))
    for rel in brand_new:
        print("      %s" % rel)

    print("\n" + "=" * 78)
    print(" 合计：相同 %d + 派生 %d + 全新 %d = %d"
          % (len(same), len(derived), len(brand_new), total))
    if len(same) + len(derived) + len(brand_new) != total:
        print(" [WARN] 分类数量与总数不符，请检查")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
