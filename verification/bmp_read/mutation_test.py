#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
变异测试 —— 验证 sim_bmp/tb_bmp_equiv.v 是否真的具备检测能力

做法：向"正确版本"里故意注入已知缺陷，看仿真是否报 FAIL。
如果某个变异仍然 PASS，说明这版 testbench 对该类缺陷是瞎的，
那么"通过"就不构成证据。

三个变异都取自本次改写中真实踩过的坑：
  M1  hdr_off 用 8 位        -> 扇区 512 字节会回绕，覆盖真实头
  M2  扫描满额分支不回 IDLE  -> 一直往后扫，装载请求被吞
  M3  离开数据段不重置相位   -> 上一张图的半个像素把下一张图挪位
"""
import io
import os
import subprocess
import sys

SRC = "bmp_read_new.v"
TB  = "tb_bmp_equiv.v"
REF = "bmp_read_ref.v"

MUTATIONS = [
    ("M1 hdr_off 改成 8 位（扇区回绕）",
     "reg [9:0]  hdr_off;                 // 本扇区已接收的字节序号（0~511，必须 10 位）",
     "reg [7:0]  hdr_off;                 // 本扇区已接收的字节序号（0~511，必须 10 位）"),

    ("M2 扫描满额时不回 IDLE",
     """                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            scan_sector      <= scan_next_hit;""",
     """                            scan_done        <= 1'b1;
                            scan_sector      <= scan_next_hit;"""),

    ("M3 离开数据段不重置像素相位",
     """        bmp_data_wr_en <= 1'b0;
        px_ph          <= 2'd0;
        px_sr          <= 24'd0;
    end""",
     """        bmp_data_wr_en <= 1'b0;
    end"""),
]


def run_one(tag, old, new, src_text):
    if old not in src_text:
        return None, "锚点未命中（变异无法注入）"
    mutated = src_text.replace(old, new, 1)
    path = "_mut.v"
    io.open(path, "w", encoding="utf-8", newline="\n").write(mutated)

    c = subprocess.run(["iverilog", "-o", "_mut.vvp", "-g2005", TB, REF, path],
                       capture_output=True, text=True)
    if c.returncode != 0:
        return None, "编译失败: " + c.stderr.strip()[:200]

    r = subprocess.run(["vvp", "_mut.vvp"], capture_output=True, text=True)
    out = r.stdout
    verdict = "PASS" if "结果：PASS" in out else "FAIL"
    # 提取不一致处数
    detail = ""
    for line in out.splitlines():
        if "结果：" in line:
            detail = line.strip()
            break
    return verdict, detail


def main():
    src_text = io.open(SRC, encoding="utf-8").read()

    print("=" * 66)
    print(" 变异测试：验证 testbench 的检测能力")
    print("=" * 66)
    print(" 基线（未变异）应 PASS：")
    v, d = run_one("baseline", "XXXNOMATCH", "", src_text)
    c = subprocess.run(["iverilog", "-o", "equiv.vvp", "-g2005", TB, REF, SRC],
                       capture_output=True, text=True)
    r = subprocess.run(["vvp", "equiv.vvp"], capture_output=True, text=True)
    base_pass = "结果：PASS" in r.stdout
    print("   %s  %s" % ("PASS ✓" if base_pass else "FAIL ✗", "原始正确版本"))
    print()

    print(" 注入已知缺陷后应全部 FAIL：")
    all_caught = True
    for name, old, new in MUTATIONS:
        verdict, detail = run_one(name, old, new, src_text)
        if verdict is None:
            print("   [跳过] %-34s %s" % (name, detail))
            all_caught = False
        elif verdict == "FAIL":
            print("   [已捕获] %-32s %s" % (name, detail))
        else:
            print("   [漏检!] %-33s 变异后仍报 PASS —— testbench 对该类缺陷无效" % name)
            all_caught = False

    print()
    print("=" * 66)
    if base_pass and all_caught:
        print(" 结论：基线 PASS + 三个变异全部被捕获 —— testbench 有效")
    elif not base_pass:
        print(" 结论：基线就不通过，先修实现")
    else:
        print(" 结论：存在漏检的缺陷类别，testbench 需加强")
    print("=" * 66)

    for f in ("_mut.v", "_mut.vvp"):
        if os.path.exists(f):
            os.remove(f)
    return 0 if (base_pass and all_caught) else 1


if __name__ == "__main__":
    sys.exit(main())
