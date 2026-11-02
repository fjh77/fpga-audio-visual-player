# 第三方组件与来源说明

本文件说明本仓库中 **不属于 MIT 授权范围** 的内容，以及各模块与参考设计的来源关系。
[LICENSE](LICENSE) 中的 MIT 授权仅覆盖本项目原创部分；下列内容版权归原权利人，按其原始授权使用。

若你要把本项目用于商业用途，请先自行向原权利人确认下列内容的授权。

---

## 1. 明确的第三方内容

以下文件带有厂商版权头、由厂商工具生成，或由厂商以加密形式交付：

| 路径 | 性质 | 权利人 / 来源 |
|---|---|---|
| `hdl_source/ip/sys_pll.v`、`ip/video_pll.v` | Anlogic IP Generator 生成（含 `Copyright (c) 2012-2024 Anlogic Inc.` 头） | 上海安路信息科技股份有限公司 |
| `hdl_source/ip/afifo_16_32_256.v`、`ip/afifo_32_16_256.v` | 同上，异步 FIFO | 同上 |
| `td_project/al_ip/PLL.v`、`PLL.vhd`、`PLL.ipc` | TD 的 IP 配置文件 | 同上 |
| `hdl_source/include/sdr_as_ram.enc.v`、`sdr_init_ref.enc.v`、`sdr_wrrd.enc.v` | 加密 RTL（文件头 `Company: anlgoic`） | 安路 SDRAM 控制器参考设计 |
| `hdl_source/hdmi1.4b_transmitter_core/hdmi_1_4b_transmitter_core_wrapper.enc.v` | 加密 RTL（324 KB） | 安路 APUG092 HDMI 1.4b Transmitter 参考设计 |
| `hdl_source/SD/sd_card_cmd.v`、`SD/sd_card_sec_read_write.v` | SD 卡底层协议（SPI 命令层 / 扇区读写） | 竞赛参考实例 `lab_ex4_tf` |
| `hdl_source/lane_lvds_10_1.v` | 板级 HDMI 差分发送适配 | 竞赛参考实例 `lab_ex4_tf` |
| `hdl_source/SD/sdram.v` | 文件头 `Company: anlgoic / Author: xg` | 安路 SDRAM 参考设计 |

> 加密文件（`*.enc.v`）只能用安路 TD 综合，无法阅读、无法修改；其授权随 TD 安装包。

## 2. 与参考设计的关系（如实披露）

本项目的开发起点是安路 APUG092_HDMI1.4b_Transmitter 参考设计，以及随 2026 年全国大学生
嵌入式芯片与系统设计竞赛选题指南提供的两个基础实例（`lab_ex4_tf`、`lab_ex5_i2s`）。
按模块统计（口径：`hdl_source` 下 53 个参与构建的 `.v`，去注释去空白后按行比对；
`include/curl_lut.vh` 是头文件、`video_source_test.v` 未被例化，两者不计）：

| 类别 | 数量 | 典型模块 |
|---|---|---|
| 与参考实例逐字节相同 | 27 | 厂商 IP、板级适配、SD 卡底层协议、显示时序 |
| 参考实例中不存在（本项目新写） | 20 | FAT32 全套、WAV 播放、总线仲裁、OSD、转场、卷曲 |
| 派生（有改动） | 6 | `top.v`、`SD/sd_card_bmp.v`、`SD/bmp_read.v`、`SD/video_timing_data.v`、`SD/frame_fifo_read.v`、`SD/frame_read_write.v` |

其中三个模块与参考实例的相似度仍然较高，**明确不主张为原创**：

| 文件 | 相似度 | 改动 |
|---|---|---|
| `SD/frame_read_write.v` | 98.8% | 新增 4 行透传端口 |
| `SD/frame_fifo_read.v` | 96.1% | 新增 11 行、改 4 行 |
| `SD/video_timing_data.v` | 81.1% | 新增 6 行、改 20 行 |

因此，README 中「`SD/` 下自研模块」的表述仅指 `fat32_*`、`bmp_load_fat`、`wav_player`、
`sd_arbiter3`、`video_curl` 等确属新写的模块；上表三个文件与第 1 节列出的文件不在 MIT 范围内。

## 3. 可复核方法

相似度数据由 `verification/tools/reuse_audit.py` 计算（`difflib.SequenceMatcher`，
对去注释、去空白后的代码行比对）。该脚本需要参考实例源码，而参考实例**不随本仓库分发**，
请自备后通过 `--ref-root` 指定：

```bash
python verification/tools/reuse_audit.py --ref-root /path/to/HX4S20_Contest_202606/7_lab_ex_2026_nosoft
```

工具链侧的交叉证据（`bmp_read` 不参与综合）见 `reports/C77_00_synth.log`：
`grep -c bmp_read` 为 0，`grep -c "elaborate module bmp_load_fat"` 为 1。

## 4. 素材与文档

- `docs/images/*.png`、`docs/manual-zh.pdf`：本项目自制。
- `tools/make_media.py` 生成的 BMP / WAV：本项目自制，无第三方素材。
- 参考实例自带的说明文档（`代码说明.md`、`设计参考例程文档.md/.docx`）**不随本仓库分发**。
