# bmp_read 改写验证环境

本目录用于复核 `hardware/hdmi_media_player/src/user_source/hdl_source/SD/bmp_read.v`
骨架改写的正确性，派生关系与相似度数据见 `../../THIRD-PARTY-NOTICES.md`。

## 文件

| 文件 | 说明 |
|---|---|
| `bmp_read_new.v` | 改写后的版本，与工程内 `SD/bmp_read.v` 的改动同步 |
| `bmp_read_ref.v` | 改写前的基线版本，仅把模块名改成 `bmp_read_ref`，逻辑一字未动 |
| `tb_bmp_equiv.v` | 差分 testbench：两个版本接同一套激励，逐拍比对全部输出 |
| `mutation_test.py` | 变异测试：向正确版本注入已知缺陷，验证 testbench 有检测能力 |

## 运行

需要 `iverilog`（Icarus Verilog）。

```bash
# 差分等价性验证，期望输出「结果：PASS」
iverilog -o equiv.vvp -g2005 tb_bmp_equiv.v bmp_read_ref.v bmp_read_new.v
vvp equiv.vvp

# testbench 有效性验证，期望「基线 PASS + 三个变异全部被捕获」
python mutation_test.py
```

## 判据说明

**逐拍比对**覆盖 11 个输出：`ready`、`state_code`、`scan_done`、`scan_found_valid`、`scan_found_sector`、`scan_found_total`、`write_req`、`sd_sec_read`、`sd_sec_read_addr`、`bmp_data_wr_en`、`bmp_data`。

其中 `bmp_data` 只在 `bmp_data_wr_en` 有效那拍比对 —— 写使能为 0 时该数据不被下游采样，属于无关值。为覆盖真正重要的内容，另设**像素流校验和**：每遇到一次写使能就把当时的数据累加，两个版本必须在整个仿真结束时得到相同的和值。

**testbench 模型**：每个被测模块各挂一个 SD 卡从设备（延迟 2 拍后连续输出 512 字节，再给一拍 `read_end`）和一个帧缓冲握手发生器（`write_req` 有效后 3 拍给一拍 `ack`）。两个从设备参数完全相同，因此任何输出差异只可能来自被测模块本身。

**覆盖场景**：多扇区装载（5 扇区 / 682 像素）、单扇区装载（1 扇区 / 4 像素）、尺寸不匹配放弃、装载中途看门狗中止、卡重新初始化、重新扫描、多图扫描命中。

## 实测结果

| 项 | 数值 |
|---|---|
| 比对拍数 | 33,198 |
| 输出不一致 | 0 |
| 像素写脉冲 | 693 / 693 |
| 像素流校验和 | `b5f3dcad` / `b5f3dcad` |

变异测试（注入缺陷应被检出）：

| 变异 | 结果 |
|---|---|
| 扇区字节计数器位宽不足（8 位，512 字节回绕） | 捕获，41,082 处不一致 |
| 扫描满额后未回空闲态 | 捕获，5,192 处不一致 |
| 离开数据段未重置像素相位 | 捕获，23 处不一致 |
