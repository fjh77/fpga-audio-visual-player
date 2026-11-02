# 构建报告（C77 实测）

本目录是交付比特流 `01_bit/hdmi_media_player_C77.bit` 的**全部原始报告**，
用于在包内直接复核文档中引用的每一个数字。**无需运行工具链即可复算。**

构建时间 2026-10-30 23:18–23:24　工具链 Anlogic TD 6.2.1（Engineer 6.2.168.116）
器件 EG4S20BG256　流程 `syn_1`（综合）→ `phy_1`（实现 + bitgen）

| 文件 | 内容 | 来源 |
| --- | --- | --- |
| `C77_00_synth.log` | 综合阶段完整日志 | `Runs/syn_1/run.log` |
| `C77_01_synth_gate.qor` | 综合 QoR（资源 / gate 级时序） | `Runs/syn_1/gate.qor` |
| `C77_02_place.qor` | 布局 QoR（含拥塞与 place 级 hold） | `Runs/phy_1/place.qor` |
| `C77_03_route.qor` | **布线 QoR —— 签核指标都在这里** | `Runs/phy_1/route.qor` |
| `C77_04_area.rpt` | 实现后面积利用率（LUT / REG / 分模块） | `Runs/phy_1/hdmi_media_player_phy.area` |
| `C77_05_timing.rpt` | 逐路径时序报告（含各时钟域汇总） | `Runs/phy_1/hdmi_media_player_pr.timing` |
| `C77_06_build.log` | 实现阶段完整日志 | `Runs/phy_1/run.log` |

---

## 1. 签核指标（引自 `C77_03_route.qor`）

| 指标 | 值 |
| --- | --- |
| Setup WNS | **+160 ps** |
| Setup TNS | 0 |
| 违例 S-EP | **0** |
| Hold WNS | **+3 ps** |
| Hold TNS | 0 |
| 违例 H-EP | **0** |
| slices | **8,728 / 9,800 = 89.06%** |
| RAM（ERAM） | 31 / 64 |
| DSP | 18 / 29 |
| 线长 | 1,704,144 |

## 2. 面积（引自 `C77_04_area.rpt`）

| 指标 | 值 |
| --- | --- |
| LUT | **15,670 / 19,600 = 79.95%** |
| REG | **8,679 / 19,600 = 44.28%** |
| IO | 33 / 188 = 17.55% |

> **两个 LUT 数字不要混用**：`C77_01_synth_gate.qor` 的 `#luts = 9,400` 是**综合后**的门级 LUT 数；
> `C77_04_area.rpt` 的 `#lut = 15,670` 是**实现后**（已做 LUT 合并/复制、映射到物理 slice）的数。
> 对外报"资源占用"应引用**实现后**的 15,670 / 79.95%，与 slices 同源。

## 3. 综合阶段（引自 `C77_01_synth_gate.qor`）

| 指标 | 值 | 说明 |
| --- | --- | --- |
| LUT | 9,400 | 门级 |
| REG | 8,577 | 与实现后的 8,679 差 102，是布局期寄存器复制（fanout 优化） |
| slices | 2,693 (27.48%) | 综合阶段未做布局，此值无实际意义 |
| RAM / DSP | 31 / 18 | 与实现后一致，说明没有额外的 RAM/DSP 推断变化 |
| Setup WNS | −988 ps / 4 S-EP | **gate 级数值不可作为收敛结论**（无布线延迟） |

## 4. 布局阶段（引自 `C77_02_place.qor`）—— 说明 hold 是怎么被修好的

| 指标 | 值 |
| --- | --- |
| Setup WNS | +1905 ps |
| Hold WNS | **−11 ps / 17 违例端点** |
| slices | 8,728（89.06%，与布线后相同 → 布线未新增资源） |
| 线长 | 903,068（约为布线后的 53%） |
| 拥塞 | Top 1% tile 利用率 60.48%，**超过 100% 的 tile 数 = 0** |

布局后 hold 是 −11 ps / 17 端点，**布线后变成 +3 ps / 0 端点** —— 差额来自后置 `fix_hold`。
日志中 `hold fixing is skipped` **未出现**（slices 89.06% 尚未触线，阈值实测在 90.71%~94.07% 之间）。
这条链路是本工程 `flow_holdfix.tcl` 存在的理由：TD 自带的 `DefaultFlow.tcl` 把后置 hold 修复
绑在 `arr_filter` 上（本工程为 false），会导致这段修复从不执行。

## 5. 检查项（引自两份 run.log）

| 检查 | 命令 | 实测 |
| --- | --- | --- |
| 编译错误 | `grep -c "^ERROR" C77_00_synth.log C77_06_build.log` | **0 / 0** |
| 悬空 net | `grep -c HDL-5314 C77_00_synth.log` | **2**（均为已知无害，见下） |
| 未解析文件 | `grep -c PRJ-7001` | **0** |
| 双驱动 | `grep -c HDL-8007` | **0** |
| hold 修复被跳过 | `grep -c "hold fixing is skipped"` | **0** |

两条 `HDL-5314` 是基线噪声，**不是本版引入**：

1. `net 'rom' does not have a driver ... osd_font_rom.v(20)` —— 存储阵列只在 `initial` 赋初值，
   TD 认为"无驱动"，实际按初值推断成 ROM，OSD 显示正常。
2. `net 'sub_ink_cnt[15]' does not have a driver ... top.v(327)` —— 驱动源在
   `if (USE_SUBTITLE != 0)` 分支内，而该开关恒 0，整条是死逻辑。

## 6. 时钟域（引自 `C77_05_timing.rpt` 的 Clock Summary）

| 时钟 | 频率 | Fanout | 用途 |
| --- | --- | --- | --- |
| `sd_card_clk` | 62.5 MHz | 2,712 | SD SPI / FAT32 / 按键 / 轮播计时 |
| `video_clk` | 25 MHz | 2,010 | 像素流水、OSD、字幕、音频输出域 |
| `ext_mem_clk` | 125 MHz | 495 | SDRAM 控制器与帧缓存 |
| `clk` | 50 MHz | 54 | 板级输入、数码管扫描 |
| `hdmi_5x_clk` | 125 MHz | 30 | TMDS 串行 |
| `sdram_clk_out` | 125 MHz | 1 | SDRAM 输出时钟 |

## 7. 复现方法

本包的 `phy_1/settings.cfg` 已配好为"可复现 C77"：`parent = ../syn_1`、`start_step opt_place`。
**必须两条一起跑**：

```
① syn_1：start_step=read_design, end_step=opt_gate   （约 2 分钟）
② phy_1：start_step=opt_place,  end_step=bitgen      （约 4 分钟，复用 syn_1/gate.db）
```

实测两次运行的 `Bitstream CRC` 均为 `10100010101001101100011101101011`，
`cmp -l` 仅差 4 字节（全在 136~140 的 `# Date:` 时间戳内）→ **配置逐位一致**。

反例：若让 `phy_1` 自己从 `read_design` 起跑（`run_type phy` 下综合），REG 会变成 8,593/8,670
而不是 8,577，布局随之漂移，CRC 必然不同（实测 `00111000000111011001001100000100`）。
**复现历史版本走 syn_1→phy_1；只有改 RTL 验证新逻辑时才走 read_design 自综合。**
