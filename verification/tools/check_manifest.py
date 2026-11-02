"""按交付清单（每行「MD5 *路径」）校验一组文件的完整性。

注意：该清单由竞赛交付包生成，**不随本仓库分发**。要校验仓库自身，
请先按清单格式自建，或用 --manifest 指向已有的清单文件。

用法：
    python verification/tools/check_manifest.py
    python verification/tools/check_manifest.py --root <包根目录>
    python verification/tools/check_manifest.py --manifest <清单路径>

退出码：0 = 全部一致；1 = 有缺失 / 不一致。
"""
import hashlib
import os
import sys

MANIFEST = "交付清单_MD5.txt"
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
COLS = 84  # 清单里路径列的宽度


def md5(path, chunk=1 << 20):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(chunk), b""):
            h.update(b)
    return h.hexdigest()


def parse(path):
    """从清单里取出 (相对路径, 字节, md5)。

    不能按固定列宽切：长路径会把后面的列推右。改成**从右往左**取 ——
    每行末两段是「字节数」和「32 位 md5」，其余全部是路径。
    """
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if not line.strip() or line.startswith("-") or line.startswith("="):
                continue
            parts = line.rsplit(None, 2)
            if len(parts) != 3:
                continue
            rel, size, digest = parts
            rel = rel.strip()
            if len(digest) != 32 or not size.isdigit() or not rel:
                continue
            if rel in ("文件", "合计") or "个文件" in rel:
                continue
            rows.append((rel, int(size), digest))
    return rows


def main():
    root = _REPO
    if "--root" in sys.argv:
        root = sys.argv[sys.argv.index("--root") + 1]
    mf = os.path.join(root, MANIFEST)
    if "--manifest" in sys.argv:
        mf = sys.argv[sys.argv.index("--manifest") + 1]
    if not os.path.isfile(mf):
        print("找不到清单：%s" % mf)
        print("  该清单由交付包生成，本仓库不含；请用 --manifest 指定清单路径，")
        print("  或 --root 指向包含清单的包根目录。")
        return 1

    rows = parse(mf)
    if not rows:
        print("清单解析失败：0 条记录")
        return 1

    missing, bad, ok = [], [], 0
    for rel, size, digest in rows:
        full = os.path.join(root, rel.replace("/", os.sep))
        if not os.path.isfile(full):
            missing.append(rel)
            continue
        if os.path.getsize(full) != size:
            bad.append((rel, "大小不符"))
            continue
        if md5(full) != digest:
            bad.append((rel, "MD5 不符"))
            continue
        ok += 1

    print("清单共 %d 条记录" % len(rows))
    print("  一致   %d" % ok)
    print("  缺失   %d" % len(missing))
    print("  异常   %d" % len(bad))
    for r in missing[:20]:
        print("    [缺失] %s" % r)
    for r, why in bad[:20]:
        print("    [%s] %s" % (why, r))
    return 0 if not missing and not bad else 1


if __name__ == "__main__":
    sys.exit(main())
