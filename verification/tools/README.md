# 验证辅助脚本

本目录收录各文档「验证方法 / 复用边界」章节中引用的脚本，用于**复核**交付内容，
不属于工程构建的一部分。

全部脚本只依赖 **Python 3 标准库**（`argparse` / `difflib` / `io` / `os` / `re` / `struct` / `sys` / `zlib`），
**不需要 numpy、Pillow，也不需要 TD**。

| 脚本 | 用途 | 引用出处 |
|---|---|---|
| `check_manifest.py` | 按《交付清单_MD5.txt》逐条核对包内文件的大小与 MD5，输出「一致 / 缺失 / 异常」计数 | `交付清单_MD5.txt` 末尾「核对方法」 |
| `reuse_audit.py` | 去注释、去空白后按行做 `difflib` 比对，统计复用代码与自研代码的相似度 | `README/复用与自研边界声明.md` / `.docx` |
| `render_osd.py` | OSD 参考模型：按 RTL 的优先级链渲染整帧，生成黄金参考图 | `README/技术栈说明书.docx` §验证 |
| `render_overlay.py` | 叠加层（亮度 / 对比度面板）参考模型 | 同上 |
| `check_pixels.py` | 把 RTL 仿真 dump 的逐像素结果与参考模型比对，输出失配数与坐标 | 同上 |
| `contrast_audit.py` | OSD 文字配色的 WCAG 对比度静态审计 | 同上 |

## 用法

每个脚本的**文件头注释**里都写了「用法」，直接照抄即可。例如：

```
python reuse_audit.py --help
```

> `check_pixels.py` 依赖同目录的 `render_osd.py`（`import render_osd`），两者必须放在一起。

## 说明

- 这些脚本是**开发期用来抓错**的工具，不是运行时依赖；删掉它们不影响工程的打开与重建。
- 脚本里的路径多数按开发目录写成相对路径，换到别处运行时按需调整。
- 工程侧的可独立运行验证环境另有 `03_verify/bmp_read/` 与 `03_verify/fat32_tbl/`，
  那两份自带 testbench，可直接用 iverilog 跑。
