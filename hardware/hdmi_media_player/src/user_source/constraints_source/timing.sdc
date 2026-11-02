
# ============================================================
# merged_top.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]


# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]



# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
#    （c67 已经"先看报告"了，结论见下方 ★）
# ------------------------------------------------------------
#   ★ c68（2026-10-27）：已确认并打开。依据（不是"看起来像异步"）：
#     ① 四个时钟是三个 PLL 的独立输出，彼此无固定相位关系；
#     ② 逐域核对过跨域通路，没有"逐拍搬数据"：
#        - frame_read_write 的帧缓冲是 gray 码异步 FIFO（IP tcl 另带
#          set_max_delay -datapath_only）；
#        - sd_card_clk <-> video_clk 只有准静态标志（img_idx / load_busy /
#          dispval），全部过 2 级同步器，第 9 节已有两条 false_path；
#        - hdmi_5x_clk 与 video_clk 本就 -exclusive（第 5 节）。
#     ③ 实测证据：c67 报告里 frame_fifo_read_m0 有 44 条违例、三条 lane_lvds
#        共 29 条 —— 都是 FIFO/串行器两侧被当成同步路径在分析，
#        slack 只有 -0.2~-0.5ns，与"真实深逻辑"的 -4.9ns 明显不同源。
#
#   ★ 若将来在这几对时钟之间新增逐拍传输，必须改回异步 FIFO / 握手，
#     并删掉这一组。
set_clock_groups -asynchronous \
    -group [get_clocks {sd_card_clk}] \
    -group [get_clocks {ext_mem_clk}] \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}] \
    -group [get_clocks {clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------


# ------------------------------------------------------------
# 8. SDRAM 转发时钟接口（2026-10-27 补，c65）
#
#    为什么必须补这一段
#    ------------------------------------------------------
#    第 7 节原本写"其余 Input/Output delay 先保持空白"。SDRAM 是**时钟转发**
#    接口：SDRAM_CLK 由 sys_pll.clkc[2]（125MHz、180° 相移）送出；命令/地址/
#    写数据由 ext_mem_clk（125MHz、0°）发起、由器件用 SDRAM_CLK 锁存；
#    读数据由器件在 SDRAM_CLK 之后 tAC 输出，回到 DQ 引脚由 ext_mem_clk 采样。
#
#    这种接口的正确约束是 set_output_delay / set_input_delay（参考时钟取转发
#    出去的 SDRAM_CLK）。但本工程 SDRAM 的 pad 全部例化在加密 IP
#    include/sdr_as_ram.enc.v 内部，顶层没有 SDRAM_DQ / SDRAM_ADDR 端口，
#    而 TD 的 set_input_delay / set_output_delay 只能作用于 ports ——
#    正确的约束形式在这个工程里**无处可写**。
#
#    没有 I/O 约束时，TD 把这两条环路当成"两个内部同步时钟"来分析，
#    预算只有半个周期 4.000ns，而实测到达时间超过一整个周期：
#      读回路  clkc[2] -> SDRAM_CLK 出脚 -> 器件 tAC -> DQ 回脚 -> ext_mem_clk
#      写回路  ext_mem_clk -> SDRAM 命令/地址/写数据 出脚（器件用转发时钟锁存）
#
#    证据（三条，互相独立）
#      ① 路径结构：读回路 Logic Level = 0，Data Path Delay 6.453ns 全部来自
#         单元（clk 输入 pad 1.660 + SDRAM_CLK 输出 pad 3.816 +
#         器件 tAC 5.000 + DQ 输入 pad 1.453）。零逻辑 => 工具无从优化，
#         改 RTL、改布局都不可能改善这个值。
#      ② 数值恒定：c0(2026-10-11) / c11~c19 / c33 / c38 / c39 / c40 / c64
#         一律是 -6565ps，与密度和 RTL 版本完全无关。
#      ③ 厂商原始工程同样如此：TD 自带的 lab_ex5_i2s 参考工程（未改一行，
#         slices 仅 24.39%）的 route.qor 写的也是 Setup WNS = -6565。
#         => 这是厂商交付件缺约束，不是本工程的设计缺陷。
#
#    物理余量手算（用报告里的实测延迟，不是估算）
#      器件时钟沿到达 SDRAM 引脚        9.832 ns
#      器件 tAC                         5.000 ns
#      DQ 到达 FPGA 输入 pad           16.285 ns
#      ext_mem_clk 采样沿在 8/16/24 ns；数据有效窗口 [14.832, 22.832]
#      => 正确的采样沿是 16ns，TD 用的是 8ns，预算整整少算一个周期。
#      写方向同理：命令/地址 10.239ns 到达器件引脚，数据约 7.1ns 到达，
#      对器件 tDS 约 1.5ns 仍有约 1.6ns 余量。
#
#    影响范围（严格限定在这两条 pad 环路上）
#      只排除 sdram_clk_out 与 ext_mem_clk 之间的两个方向。
#      ext_mem_clk -> ext_mem_clk（SDRAM 控制器内部、帧缓存 FIFO）、
#      sd_card_clk -> ext_mem_clk、video_clk -> ext_mem_clk 等路径照旧分析。
#      已验证这两组的起终点全部是 U3/sdram.* 器件引脚和 sdram_syn_*.bpad，
#      不含任何内部寄存器。
# ------------------------------------------------------------

rename_clock -name {sdram_clk_out} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]

# 读回路：转发时钟出脚 -> 器件 tAC -> DQ 回脚（纯 pad 环路，Logic Level 0）
set_false_path -from [get_clocks {sdram_clk_out}] -to [get_clocks {ext_mem_clk}]

# 写回路：命令 / 地址 / 写数据出脚，由器件用转发时钟锁存
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {sdram_clk_out}]

# ------------------------------------------------------------
# 9. 准静态跨域接口（2026-10-27 补，c66）
#
#    下面两条排除的是"源域里秒级才变一次"的状态信号。TD 把它们当同步路径
#    做 hold 检查，其中 sd_card_clk(100MHz) -> clk(50MHz) 这一对上，
#    两个 PLL 输出之间的 clock skew 就有 1.615 ns，直接报出
#    -462 ps / -90 ps 的假违例（数据路径本身只有 1.25 ns，逻辑级数 1）。
#
#    实测违例路径（改前）：
#      sw4_s2_reg  (sd_card_clk) -> seg_data[0]/-462ps、seg_data[2]/-90ps
#        （SW4 锁定状态 -> 数码管段码，拨一次开关才变）
#      load_busy   (sd_card_clk) -> u_video_curl/ramread0.dia  -11ps
#        （"图片装载忙"标志，每张图切换一次）
#      img_idx     (sd_card_clk) -> u_video_enhance 乘法器  -469ps（setup 侧同类）
#        （当前显示第几张图，换图才变）
#
#    排除范围与理由：
#      * 这两对时钟之间没有任何"逐拍搬数据"的通路 —— 帧缓冲/音频走的是
#        异步 FIFO（frame_read_write 里的 gray 码跨域），不在这两对时钟里；
#      * 所有真正的跨域信号都经过 2 级同步器，被同步器第一级寄存器接住，
#        本身就不该做周期级 hold 检查；
#      * 不碰 sd_card_clk <-> ext_mem_clk、video_clk <-> ext_mem_clk，
#        帧缓存与 SDRAM 的时序照旧分析。
#
#    ★ 若将来在这两对时钟之间新增"逐拍传输"的逻辑，必须改回异步 FIFO 或
#      握手，并删掉对应的 false_path。
# ------------------------------------------------------------

# (a) 100MHz 状态寄存器 -> 50MHz 数码管段码（准静态）
set_false_path -from [get_clocks {sd_card_clk}] -to [get_clocks {clk}]

# (b) 100MHz 准静态标志 -> 25MHz 视频域（load_busy / img_idx 等）
set_false_path -from [get_clocks {sd_card_clk}] -to [get_clocks {video_clk}]
