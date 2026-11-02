# 常见问题（FAQ）

> 按"现象"检索。若这里没有你的问题，先看 [`tf-card.md`](tf-card.md) 的排障表与 [`build.md`](build.md) 的错误码表。

---

## A. 打开与构建

<details>
<summary><b>Q1. TD 打开工程后 <code>syn_1</code> / <code>phy_1</code> 显示"未运行"，是坏了吗？</b></summary>

不是。仓库有意**不包含 run 数据库**（`*.db` 共约 65 MB），它们可由源码在 6 分钟内重建，
且与 TD 版本强绑定，放进版本库只会让仓库膨胀十倍。

按 [build.md 第 3 节](build.md#3-两种构建方式) 依次跑 `syn_1` → `phy_1` 即可。

</details>

<details>
<summary><b>Q2. 改了 RTL，重新构建后现象完全没变？</b></summary>

TD 每轮流程结束会执行 `export_flow_status`，**把 `start_step` 自动改回 `opt_place`**，
下一次"重新构建"只跑布局布线，开头 `import_db ../syn_1/...gate.db`——源码改了也白改。

三步修复（缺一不可）：

1. `syn_1/settings.cfg` 的 `start_step` 改回 `read_design`
2. 删除 `syn_1/` 下 `.read_design.end.f`、`.opt_rtl.end.f`、`.opt_gate.end.f`
3. 跑完确认 `syn_1/gate.qor` 的 `#luts` 变了

判据：`grep -m1 -E "elaborate|import_db" syn_1/run.log`，看到 `elaborate -top top` 才是真综合。

</details>

<details>
<summary><b>Q3. 构建出现 <code>HIER-1013</code> / <code>HDL-8001</code>，工程坏了吗？</b></summary>

没坏。这两个错误只在"tcl 里单独 `open_project` + `elaborate -top top`、缺少 `-single_run`"
时出现，是 TD 缺 run 上下文时的固有行为。走完整 flow（`flow_holdfix.tcl`）不会触发。

</details>

<details>
<summary><b>Q4. 为什么不能用 TD 自带的 <code>DefaultFlow.tcl</code>？</b></summary>

它把"路由后单独跑 `fix_hold`"绑在 `arr_filter` 上（本工程为 `false`），导致后置 hold 修复**从不执行**；
而路由阶段内置的 `route fix_hold` 在 slices 超阈值时又会被跳过。

实测：布局后 hold 为 −11 ps / 17 端点，用 `flow_holdfix.tcl` 补跑后变为 **+3 ps / 0 端点**。

</details>

<details>
<summary><b>Q5. 构建日志里的 <code>HDL-5314</code> 要处理吗？</b></summary>

**固定 2 条，已知无害**，不要"顺手修掉"：

1. `osd_font_rom.v` 的 `rom` 只在 `initial` 赋初值，TD 视作"无驱动"，实际按初值推断成 ROM；
2. `top.v` 的 `sub_ink_cnt[15]`，驱动源在 `USE_SUBTITLE != 0` 分支内而该开关恒 0。

**判据：只有"新出现的"`HDL-5314` 才需要排查。** 这类告警曾掩盖过一个致命 bug——
字模 `shared_fbits` 被隐式声明成 generate 作用域内的新 net，导致上板只画底带不画字，
而当时 `ERROR` 数为 0。

</details>

---

## B. 烧录与固化

<details>
<summary><b>Q6. 烧到 100% 了，断电重启却不启动？</b></summary>

按可能性排序：

1. **`MemorySize` 没改成 `16 Mbits`**（板载只有 W25Q16 = 2 MB，默认 128 Mbits 会让地址算错）；
2. **没有完整断电**，只按了复位键——FPGA 只在重新上电时加载位流；
3. 模式选成了 `Program SRAM`（掉电即失）；
4. 烧录中途被 `taskkill` 打断。

</details>

<details>
<summary><b>Q7. `Merge DualBoot Bit` 要点吗？</b></summary>

**不要点。** 它会把两个位流合并成双镜像并改写 Flash 布局，本项目不需要，
误点后需要 `Erase Flash` 整片擦除重烧。

</details>

<details>
<summary><b>Q8. 每次重新构建，md5 都不一样，是不是不可复现？</b></summary>

不是。`.bit` 前约 140 字节是文本头，含生成时刻 `# Date:`，md5 必然变化。

判"同一配置"要比 **`Bitstream CRC`**，或用 `cmp -l a.bit b.bit | awk '$1>140'` 确认差异只落在头部。
本工程两次独立构建：CRC 相同，`cmp` 仅差 4 字节（全在时间戳内）。

</details>

---

## C. TF 卡与素材

<details>
<summary><b>Q9. 插了卡，数码管不显示任何数字？</b></summary>

说明 FAT32 目录扫描没完成。依次排查：

1. 卡是否为 **FAT32**（不是 exFAT / NTFS / FAT16）；
2. `sd_miso` / `sd_ncs` 是否有 10 kΩ 上拉到 3.3 V；
3. SPI 走线是否过长——先把 `SPI_HIGH_SPEED_DIV` 改成 `2`（≈10.4 MHz）降速验证；
4. 是否**热插拔**了——本设计不支持热插拔，必须断电插卡再上电。

</details>

<details>
<summary><b>Q10. 显示组数为 0，但卡里明明有图？</b></summary>

按命中率排序：

1. 文件放在了**子文件夹**里（只扫根目录）；
2. 文件名不是 **8.3** 格式（中文名、长名会被转成 `XXXXXX~1.BMP`，前两字符规则失效）；
3. BMP 与 WAV 的**前两字符**不同（`01.BMP` 必须配 `01.WAV`）；
4. 超过 **4 组**上限，后面的被忽略。

</details>

<details>
<summary><b>Q11. 有组数显示，但 HDMI 黑屏不出图？</b></summary>

BMP 头校验失败。固件硬性要求：

| 字段 | 要求 |
|---|---|
| `bit_count` | **必须是 24** |
| `compression` | **必须是 0（BI_RGB，未压缩）** |
| 尺寸 | 640×480，或 1280×960（自动 2:1 缩小） |

最常见的原因是"另存为 BMP"时被存成了 **32 位带 Alpha** 或 **RLE 压缩**。

</details>

<details>
<summary><b>Q12. 图正常，但没有声音？</b></summary>

WAV 头四项校验全部要通过：**PCM(1) / 单声道(1) / 48000 Hz / 16 bit**。

```bash
# 一行转成合规格式
ffmpeg -i in.mp3 -ac 1 -ar 48000 -sample_fmt s16 -acodec pcm_s16le 01.WAV
```

立体声、44.1 kHz、24 bit 一律不播。

</details>

<details>
<summary><b>Q13. 声音断续或播到一半停住？</b></summary>

1. **簇链碎片**：重新格式化卡后连续拷贝素材（不要反复删拷）；
2. **SPI 仲裁抢占**：换图装载期间 WAV 预取优先级最低，可能短暂欠载——这是设计行为；
3. 卡容量过大且簇很小，导致单文件簇链过长，扫描变慢——建议 32 KB 簇。

</details>

<details>
<summary><b>Q14. 支持多大的卡？需要什么簇大小？</b></summary>

建议 **≤32 GB**、**FAT32**、簇 **4 KB~32 KB**。

注意：Windows 图形界面在 >32 GB 的卡上不提供 FAT32 选项，需用 `diskpart` 或第三方工具；
大容量卡出厂多为 exFAT，必须重新格式化。

</details>

---

## D. 显示与交互

<details>
<summary><b>Q15. 字幕为什么不随重音变亮？</b></summary>

**已知缺陷 1，不是故障。** 判据 `c_text = (sp_pos > 28)` 恒假（`sp_pos` 被 `CAP=14` 钳住），
从设计之初就从未生效。上板看到的字色从头到尾是同一个金色。

修法：按 `dy` 查 4 级色阶，代价 +30 LUT。因会把 slices 推过 90% 红线而未采用。

</details>

<details>
<summary><b>Q16. 行波在最后两个字断开？</b></summary>

**已知缺陷 2。** `dly[spos[8:5] * LAG]` 的乘积位宽被取成 `max(4,3)=4` 位，
`16→0`、`18→2` 回绕，导致字 8 读 `dly[0]`、字 9 读 `dly[2]`。

修法：先钳位再移位，+8 LUT，但 slices 会涨到 90.71%。已明确放弃。

</details>

<details>
<summary><b>Q17. 峰值时字顶贴住条带上边框？</b></summary>

**已知缺陷 3。** 字顶 `432−14 = 418`，正好压住条带内容首行 418。
修法是输出侧 3/4 压缩（`dy = pos − pos>>2`），+8 LUT。同样因面积被放弃。

</details>

<details>
<summary><b>Q18. 静音时字幕完全不动，是不是律动坏了？</b></summary>

正常。C77 没有"怠速呼吸"设计——语音间隙字幕落回基线并静止。
若想确认律动链路正常，播放一段有明显节奏的素材即可看到起伏。

</details>

<details>
<summary><b>Q19. 3 秒档感觉图还没看清就切走了？</b></summary>

物理约束：921 KB 的图装载约 **1.4 秒**，3 秒周期里只剩约 1.6 秒展示。
现场演示建议用 **10 秒档或讲解档**。

</details>

---

## E. 二次开发

<details>
<summary><b>Q20. 想加新功能，但 slices 已经 89.06% 了？</b></summary>

这是本工程最硬的约束：距 90% 只剩 **0.94 个百分点**，超过阈值 TD 会跳过 hold 修复、留下违例端点。

在动功能之前，先考虑这几处可回收的面积：

| 优化点 | 可省 | 代价 |
|---|---|---|
| 合并两个 `fat32_file` 实例 | ≈1,000 LUT | 音频晚 3.2 s 起播，破坏音画同步 |
| 降低 OSD 字库槽位 | 数十 LUT | 可用字形变少（当前 64 槽已满） |
| 关掉彩条兜底图 `color_bar` | 少量 | 无卡时黑屏 |

</details>

<details>
<summary><b>Q21. 源码注释里写的是 100 MHz，实际时钟是多少？</b></summary>

**注释过期，逻辑正确。** c70 把 SD/FAT 域从 100 MHz 降到 **62.5 MHz**，
但约 105 处注释仍写 `100MHz`（`key_viz_ctrl.v`、`sd_card_bmp.v` 的参数默认值等）。

真实值由 `top.v` 例化处统一覆盖：

```verilog
.CLK_FREQ_HZ      (62_500_000),
.PERIOD_3S_TICKS  (32'd187_500_000),
```

`ip/sys_pll.ipc` 也仍是旧配置，真正的分频在 `ip/sys_pll.v` 里（`CLKC0_DIV=16` → 62.5 MHz）。
**以时钟表为准，不要照抄注释。**

</details>

<details>
<summary><b>Q22. 能不能把工程拆成 rtl/ sim/ 这种标准结构？</b></summary>

可以，但需要同步重建 TD 工程。`.al` / `.prj` / `settings.cfg` 里全是相对路径
（`../user_source/...`、`al_ip/PLL.ipc`），移动源文件后 TD 会找不到。

本仓库选择**保持原工程树不动**，代价是 `hardware/` 层次略深，换来"克隆下来双击就能打开"。

</details>

---

## F. 其他

<details>
<summary><b>Q23. 仓库里为什么没有 <code>*.db</code> 中间库？</b></summary>

三类原因：

1. **体积**：6 个 `.db` 约 65 MB，占原包 90% 以上；
2. **可重建**：源码 6 分钟即可完整重建；
3. **版本绑定**：TD 版本变化后旧库往往读不进去。

代价：**不能"原地续跑"**（只重跑布局），第一次构建必须从综合开始。

</details>

<details>
<summary><b>Q24. 板载 I2S 排针能用吗？</b></summary>

本设计的音频**随 HDMI 一并传输**（TMDS 数据岛 + ACR 重建 48 kHz 采样时钟），
`pin.adc` 中 I2S 相关约束已注释、未启用。

若你要改用外接 I2S DAC，需要：接出 BCLK / LRCK / DOUT（约束文件里注释掉的那三行），
并把 `audio_acr_gen` 的输出改为直接驱动 I2S 发送器。

</details>
