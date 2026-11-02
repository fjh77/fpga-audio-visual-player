//=============================================================================
// top —— 音视频播放系统顶层
//
//   TF 卡里的 BMP 图片与 WAV 音频 -> 640x480 HDMI（视频 + 内嵌音频包）+ 数码管状态显示
//   说明：本版音频经 HDMI 音频包送出（ACR + audio sample packet），顶层无 I2S 输出端口
//
//   时钟域
//   频率以 sys_pll / video_pll 的实际配置为准（ip 目录下的 *.ipc）：
//     video_clk       25 MHz  video_pll.clkc0  像素时钟（800 x 525 @ 59.52 Hz）
//     hdmi_5x_clk    125 MHz  video_pll.clkc1  TMDS 串化时钟
//     sd_card_clk    62.5MHz  sys_pll.clkc0    SD 卡 SPI、FAT 解析、按键去抖（c70 由 100MHz 降频）
//     ext_mem_clk    125 MHz  sys_pll.clkc1    SDRAM 控制器与帧缓存读写
//     ext_mem_clk_sft 125 MHz sys_pll.clkc2    与 ext_mem_clk 同频、相移，用于接口采样
//   sd_card_bmp 的定时参数按 100 MHz 计算（CLK_FREQ_HZ = 100_000_000）
//
//   视频通路
//     TF 卡 -> bmp_load_fat（含 img_scale_2to1 2:1 缩小，仅 1280x960 源）
//           -> SDRAM 双缓冲 -> 按行读回 -> video_enhance（亮度）
//           -> video_curl（卷曲转场）-> banner_scroll（滚动标语，可关）
//           -> osd_overlay（参数调节条：亮度/对比度/音量）-> osd_menu（设置菜单）-> HDMI 发送
//     banner_scroll 内含两套互斥的条带内容：
//       · 平时 = 滚动字幕 + 随音频响度的垂直律动（弹簧 + 逐字行波）
//       · 装载中 = 从左向右填充的进度条（I_prog / I_load_busy）
//
//   音频通路
//     TF 卡 -> wav_player -> audio_volume -> HDMI 音频包（ACR 由 audio_acr_gen 生成）
//                 └-> osd_audio_env（三级低通取 120~480Hz 能量，每帧出 1 个 7 位响度）
//                        └-> banner_scroll.I_level（驱动字幕律动）
//=============================================================================

`timescale 1ns / 1ps

module top(
    input                       clk,
    input                       rst_n,
    //-------------------------------------------------------------------------
    // 按键（低有效，内部去抖）
    //   SW4 = 1：三个按键有效
    //     KEY_2  手动切下一张（自动轮播进行中被禁用，只弹提示不切图）
    //     KEY_3  启动 / 停止自动轮播
    //            ── 手动浏览档（轮播周期 = 手动）下改为"上一张"
    //     KEY_4  不切档；SW3=1 时作"当前参数 -"（亮度/对比度/音量）
    //            ── 手动浏览档且 SW3=0 时改为"播 / 停当前这张图配对的语音"
    //   SW4 = 0：三个按键全部失效（防误触），数码管显示 "Loc"
    //   SW1 = 1：进入设置菜单，此时 KEY_2 移动光标、KEY_3 确认
    //            轮播周期档位（讲解 / 3s / 10s / 手动）只能在这里改
    //-------------------------------------------------------------------------
    input                       key1,           // KEY_2(B2)
    input                       key2,           // KEY_3(B1)
    input                       key3,           // KEY_4(C1)

    // 拨码开关（上拨 = 1，下拨 = 0；硬件接 GND，内部上拉决定下拨电平）
    input                       sw1,            // SW1(C8) 1 = 设置菜单，0 = 正常画面
    input                       sw2,            // SW2(C7) 转场模式：1=卷曲，0=上下推移（手动翻页固定推移）
    input                       sw3,            // SW3(C6) 1 = 参数调节条（亮度/对比度/音量）
    input                       sw4,            // SW4(C5) 1 = 按键有效，0 = 锁定

	output [5:0]                seg_sel,
	output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;

//=============================================================================
// 编译期开关
//
//   四个开关都是常量折叠：改值后必须重新综合 + 重新下载才生效。
//   关闭时对应逻辑在综合阶段被整块消除，不占资源。
//=============================================================================

//-------------------------------------------------------------------------
// DIAG_MODE：数码管诊断显示（现场定位 FAT 解析问题用）
//   0 = 正常显示（默认，诊断逻辑被折叠成常量）
//   1 = 数码管显示 fat32_dir 内部值，按"页"查看：
//         位0      页号 0~B
//         位1~2    该页 16 位数值的高字节
//         位3      解析完成指示（亮 = 已结束，此时读数有效）
//         位4~5    该页 16 位数值的低字节
//       DIAG_AUTO_CYCLE=1 时每 1.5 s 自动翻页；想固定某页就置 0 并设 DIAG_PAGE。
//   注：位3 的段 b/c 在板子上损坏，全亮时看起来像"6"，只当指示灯用。
//-------------------------------------------------------------------------
parameter        DIAG_MODE       = 1'b0;              // 0 = 正常显示，1 = 诊断显示
parameter [3:0]  DIAG_PAGE       = 4'd0;              // 不轮换时固定显示哪一页（0~B）
parameter        DIAG_AUTO_CYCLE = 1'b1;              // 1 = 诊断模式下自动轮换
parameter [31:0] DIAG_HOLD_CYC   = 32'd75_000_000;    // 每页停留 1.5 s @50MHz

//-------------------------------------------------------------------------
// USE_SUBTITLE：图片配对的 0N.SUB 掩膜字幕链路
//
//   本工程永久关闭，保持 1'b0。
//   关闭时 sub_mask_ram 本体与 sub_overlay / sub_scroll 都不例化
//   （必须用 generate 条件例化；无条件例化时 TD 照样会为 RAM 分配 BRAM），
//   资源与关闭前完全一致。
//
//   三个模块 sub_overlay / sub_scroll / sub_mask_ram 已归档到
//   _archive/future_feature/subtitle/，并从工程文件里移除。
//   下面的 generate 分支与相关信号保留不删（常量折叠，零资源），只作恢复入口。
//
//   恢复步骤：
//     ① 三个 .v 拷回 hdl_source/
//     ② 用 tools/fix_project_files.py 加回 .al 与 .prj（漏加会报 module not found）
//     ③ 本参数改成 1'b1
//     ④ 重跑综合，确认 BRAM 约 +11 块，再上板验证
//-------------------------------------------------------------------------
parameter        USE_SUBTITLE    = 1'b0;              // 永久 0：1 = 启用掩膜字幕（需先恢复文件）

//-------------------------------------------------------------------------
// USE_BANNER：底部滚动标语「欢迎观看太阳神鸟金饰」
//   1 = 例化 banner_scroll（当前启用）
//   0 = 整个模块不例化，vout_banner 直接短接
//       （不是"例化了但关显示"——后者照样占 LUT）
//
//   面积（**C52 首次放下时的实测**）：banner_scroll 本体 173 LUT，
//   全片 slices 8,587 / 87.62%、Hold +4ps/0 违例。启用前曾两次因面积越过
//   90% 失败，最终靠"三份字库合一"省下 389 LUT 才放下
//   —— 详见 docs/滚动标语_实施报告.md。
//   ★ 上面是 C52 的数字。之后 C67 加律动、C72 加进度条、C77 重铺响度映射，
//     当前（C77）全片 slices 8,728 / 89.06%、Setup WNS +160ps。别把 87.62% 当现状。
//-------------------------------------------------------------------------
parameter        USE_BANNER      = 1'b1;              // 1 = 启用滚动标语

//-------------------------------------------------------------------------
// 已归档字幕链的滚动参数（仅 USE_SUBTITLE=1 时有效，当前不影响本设计）
//   注意：底部滚动标语由 banner_scroll.v 自己的 localparam STEP 控制，
//   不使用这两个参数，二者互不相干。
//-------------------------------------------------------------------------
parameter [11:0] SUB_PX_PER_FRAME = 12'd4;             // 每帧前进几像素
parameter        SUB_LOOP         = 1'b1;             // 1 = 循环，0 = 停在末尾

//-------------------------------------------------------------------------
// AUDIO_EN：HDMI 音频通路总开关
//   1 = 开启，音频来自 TF 卡里的 WAV 文件
//   0 = 关闭，HDMI 核收到恒定 0（wav_player / audio_volume 的输出无人使用，
//       综合器会把它们优化掉，不占资源；模块仍保留，改回 1 即可恢复）
//-------------------------------------------------------------------------
localparam AUDIO_EN = 1'b1;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

//=============================================================================
// 提前声明 rst_all / sw4_s1 / sw4_s2
//
//   它们原先声明在首次使用之后，综合日志会报 HDL-5373（used before declaration）。
//   模块级作用域里前向引用是合法的，功能不受影响；但同样的写法若出现在
//   generate 内部，Verilog 会创建该作用域的新隐式 net，与外层同名 wire
//   不是同一根线，表现为静默悬空（HDL-5314，不报错但功能失效）。
//   统一提前声明，杜绝这类隐患。
//=============================================================================
wire rst_all;
reg  sw4_s1, sw4_s2;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 相关
wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// ===================== 轮播周期档位（4 档）=====================
//   period_sel 的语义（由 sd_card_bmp 驱动，只能在 SW1=1 设置菜单里改）：
//       2'd0 = 讲解模式：播放讲解音频，播完 + 停顿后切图
//       2'd1 = 快速浏览 3 秒  ┐ 定时档位：不播放音频，到点切图
//       2'd2 = 慢观看 10 秒   ┘
//       2'd3 = 手动浏览：不自动切图，只由 KEY_2 翻页
// KEY_4 不参与切档 ：它只在 SW3=1 时作"参数 −"（见 osd_ctrl.v）
//
//   数码管（SW4=1 正常显示）：
//       位 5 / 位 4 = 档位：A / 3 / 10 / P
//         位 4 是个位（讲解档 A、3 秒档 3、10 秒档 0、手动档 P）
//         位 5 只在 10 秒档亮 "1"，其余档位熄灭
//   SW4=0 锁定：位 4 = "L"、位 1 = "C"，其余位熄灭 —— 整体读作 "Loc"（锁）
//       位 1 的 C 带 !key_audio_gate 条件：给该信号一个真实消费者，
//       避免综合器把产生它的逻辑当成死逻辑剪掉
wire [1:0]  period_sel;
wire        key_audio_gate;      // sd_card_bmp 报"当前允许音频起播"
wire        key_period_explain;  // sd_card_bmp 报"当前档位是讲解模式"
wire [1:0]  img_idx_disp;        // 当前显示中的图片序号（保留连接，见例化处说明）

// c72：装载进度（100MHz 域，gray 码）与装载中标志，给字幕带画进度条
wire [7:0]  bmp_prog_gray;
wire        bmp_load_busy;
// 位 4 / 位 5 / 位 1 的显示码由下面的 always @(*) 驱动，因此必须是 reg（不是 wire）
reg  [6:0]  seg_period;          // 位 4：档位个位（A / 3 / 0 / P），锁定时为 L
reg  [6:0]  seg_period_tens;     // 位 5：档位十位（仅 10 秒档为 1，其余熄灭）
reg  [6:0]  seg_lock_1;          // 位 1：锁定模式下显示 C，正常模式显示文件数/菜单光标

// ===================== 阶段 3a：FAT32 解析结果显示 =====================
//   位 1 = FAT 根目录里解析到的文件条数的十位（正常是 0；>=10 才非 0）
//   位 2 = 文件条数的个位
//   位 3 = FAT 解析的 err_code（十六进制一位，0 = 无错）
//
// 为什么用位 1/2 两位一起表示文件数，而不是一位
//   err_code 是 0~15 的十六进制数，一位数码管就能完整显示；而"文件数"
//   用一位会在 >=10 个文件时被钳掉、看不出真假。三位一起用（1/2/3）
//   刚好把"条数（十位+个位）+ 错误码"完整显示出来，也不需要再额外加
//   "数字溢出"标志位。用户卡上 5 个文件 -> 位 1 显示 0、位 2 显示 5。
//
// 这三位的原有含义是"熄灭"，改动后：
//       USE_FAT_SCAN=0                -> 仍然熄灭（旧行为，一模一样）
//       USE_FAT_SCAN=1 且解析未完成   -> 仍然熄灭
//       USE_FAT_SCAN=1 且解析已完成   -> 显示 条数十位 / 条数个位 / err_code
//   位 0（WAV 状态码）、位 4（图片序号）、位 5（TF 卡状态码）一个字没动。
wire [4:0]  fat_count;           // FAT 解析到的文件条数
wire [3:0]  fat_err;             // fat32_dir 错误码
wire        fat_cnt_ovf;         // 条数 >= 10（十位非 0）
wire [3:0]  fat_cnt_digit;       // 条数的个位（sd_card_bmp 已钳到 0~9）
wire        fat_done;            // 解析完成
wire [3:0]  fat_bin_tens;        // 条数十位
wire [3:0]  fat_bin_units;       // 条数个位
wire [3:0]  fat_bin_err;         // 错误码
wire [6:0]  fat_tens_seg;        // 十位 -> 七段
wire [6:0]  fat_units_seg;       // 个位 -> 七段
wire [6:0]  fat_err_seg;         // 错误码 -> 七段
wire [6:0]  fat_tens_seg_m;      // 未完成时强制熄灭
wire [6:0]  fat_units_seg_m;
wire [6:0]  fat_err_seg_m;

// ===================== 临时诊断：来自 fat32_dir 的内部关键值 =====================
//   全部是"只读观测"信号，只在 DIAG_MODE=1 时才会被用到。
wire [31:0] dbg_root_lba;        // 根目录首簇绝对首扇区（真卡期望 0x0000A000）
wire [31:0] dbg_part_lba;        // MBR 分区起始 LBA     （真卡期望 0x00002000）
wire [31:0] dbg_data_lba;        // 数据区起始绝对扇区     （真卡期望 0x0000A000）
wire [31:0] dbg_fat_lba;         // FAT1 起始绝对扇区      （真卡期望 0x00002974）
wire [15:0] dbg_secs;            // 成功收到的扇区数
wire [7:0]  dbg_part_type;       // 分区类型（0x0C = FAT32 LBA）
wire [7:0]  dbg_root_b0;         // 根目录首扇区第 1 字节（真卡期望 0xE5）
wire [7:0]  dbg_root_b1;         // 根目录首扇区第 2 字节（真卡期望 0x20）
wire [15:0] dbg_rx_bytes;        // 收到的数据字节总数（6 扇区 -> 0x0C00）

// ===================== 阶段 3b：装载通路诊断值 =====================
//   USE_FAT_LOAD=0（旧通路）时这三个全是 0，页 C~F 读出来就是 0，
//   不会误解（旧通路本来也没有"FAT 表挑到几张图"这个概念）。
wire [3:0]  fat3b_bmp_cnt;       // FAT 表里挑到几张 BMP（真卡期望 4）
wire [3:0]  fat3b_load_err;      // 装载器错误码（0=无错；1=头不匹配 2=表项非法 3=簇链出错）
wire [31:0] fat3b_load_bytes;    // 最近一次装载从文件里取到的字节数（期望 921654 = 0x000E1036）

// ===================== S2：滚动字幕掩膜条（0N.SUB）=====================
//   写侧在 sd_card_bmp 里（100MHz），RAM 本体与读侧在这里（读侧 25MHz）。
// 读出来的数据必须有真实消费者，否则 TD 会把整条通路静默剪掉
//     S2 的消费者 = 诊断数码管（页 2/3/4，见下面的诊断页说明）。
wire        sub_ram_we;          // 100MHz：一列一个 48bit 字的写使能
wire [10:0] sub_ram_addr;        // 100MHz：写到第几列（0..2047）
wire [47:0] sub_ram_wdata;       // 100MHz：该列的 48 条扫描线
wire [11:0] sub_cols;            // 最近一次字幕装载写进去多少列（完整=2048=0x800）
wire        sub_ok;              // 1 = 最近一次字幕装载完整成功
wire [2:0]  sub_found;           // 配到几条字幕（期望 4）
wire [3:0]  sub_load_err;        // 字幕装载错误码（0=无错）
wire [10:0] sub_mask_addr;       // 25MHz：叠加层给的读列地址（S3 恒 = 像素 x）
wire [47:0] sub_rdata;           // 25MHz：该列的 48bit 掩膜
wire [15:0] sub_ink_cnt;         // 25MHz：本帧被字幕点亮的像素数（帧起始清零）
wire [31:0] sub_ink_total;       // 诊断页 4 用的 32 位副本
wire        sub_busy;            // 100MHz：装载器正在写掩膜 RAM（S3fix 新增）
wire        sub_valid_raw;       // 100MHz：掩码有效（清完 RAM 且不在装载中）
wire        sub_valid;           // 25MHz ：同步两拍后的"掩码有效"门控

// ===================== S4：滚动字幕 =====================
//   sub_playing : wav_player 的 o_playing（clk_audio = video_clk 域，零跨域）
//   sub_img_req : 图片提交脉冲同步到 video 域（新图 -> 偏移归零）
// sub_scroll_x: 送给 sub_overlay 的 I_x_off（每帧只在 vs 边界变一次）
wire        sub_playing;         // 25MHz：音频"确实已出声"
wire        sub_valid_sw;        // 100MHz：sub_valid_raw AND sw1（SW1=0 -> 恒 0）
wire        sub_img_req;         // 25MHz：新图提交（偏移归零触发）
wire [11:0] sub_scroll_x;        // 25MHz：当前滚动偏移 0..(2048-WIDTH)
reg         sw1_s1, sw1_s2;      // SW1 同步到 100MHz 域（与 sub_valid_reg 同域相与）

// ===================== 轮播设置菜单 / 停止提示 =====================
//   SW1 从"字幕开关"改为"轮播设置模式开关"（USE_SUBTITLE=0 时字幕链路本就是
//   常量 0，所以 SW1 摘出来后不留下任何悬空消费者）。
//     SW1=1 -> 菜单出现、自动轮播停、音频停
//              KEY2（sd 的 key_next）→ 菜单光标上下移动（o_menu_sel）
//              KEY3（sd 的 key_auto）→ 确认并存档（sd_card_bmp 内 period_sel）
//     SW1=0 -> 回画面；KEY3 启动/停止该档轮播；KEY2 手动切图（自动轮播中禁用）
//             停止的那一拍 o_stop_banner 出单拍脉冲，菜单模块弹 3 秒提示条
//             启动的那一拍 / KEY2 被拒的那一拍 o_auto_hint 出单拍脉冲，
//             菜单模块弹 1 秒「自动轮播中」短提示（1s < 最短轮播周期 3s）
wire [1:0] sd_menu_sel;          // 100MHz：菜单光标 0..3
wire [1:0] sd_cfg_sel;           // 100MHz：已确认档位 0..3
wire       sd_stop_banner;       // 100MHz：停止自动轮播的单拍脉冲
wire       sd_manual_mode;       // 100MHz：当前档位 = 手动浏览（永不自动切图）
wire       sd_auto_hint;         // 100MHz：自动轮播中短提示的单拍脉冲

// SW1 同步到 video 域（菜单显示用；拨码是慢速输入，2 级同步足够）
reg        sw1_v1, sw1_v2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        sw1_v1 <= 1'b0;
        sw1_v2 <= 1'b0;
    end else begin
        sw1_v1 <= sw1_s2;            // 先在 100MHz 域同步好，再进 video 域
        sw1_v2 <= sw1_v1;
    end
end

// ===================== SW4 总开关：video 域同步 =====================
//   SW4=1 -> 总开关打开：正常出图/出声/接受按键
//   SW4=0 -> 总开关关闭：画面全黑（HDMI 同步保持不变，不会失锁）、音频静音、
//            自动轮播停（sd_card_bmp 里 key_locked_w 已把三键与音频门死）
//
// 为什么在顶层再加一道"黑屏"
//     只靠 sd_card_bmp 的门控，SDRAM 里最后一帧仍会被读出来显示 ——
//     用户要的是"SW4=0 什么都看不到"。所以在送 rgb_to_axis之前
//     把 24 位像素整体压 0；vs/de 一律不动，保证 HDMI 时序/ACR 不受影响。
// 同步链 sw4_s1/sw4_s2(100MHz) -> sw4_v1/sw4_v2(video 25MHz)
reg        sw4_v1, sw4_v2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        sw4_v1 <= 1'b0;
        sw4_v2 <= 1'b0;
    end else begin
        sw4_v1 <= sw4_s2;            // 先在 100MHz 域同步好，再进 video 域
        sw4_v2 <= sw4_v1;
    end
end

// 跨域：菜单光标 / 已确认档位（都是准静态信号，2 级同步直接采）
reg  [1:0] menu_sel_v1, menu_sel_v2;
reg  [1:0] cfg_sel_v1,  cfg_sel_v2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        menu_sel_v1 <= 2'd0;
        menu_sel_v2 <= 2'd0;
        cfg_sel_v1  <= 2'd0;
        cfg_sel_v2  <= 2'd0;
    end else begin
        menu_sel_v1 <= sd_menu_sel;
        menu_sel_v2 <= menu_sel_v1;
        cfg_sel_v1  <= sd_cfg_sel;
        cfg_sel_v2  <= cfg_sel_v1;
    end
end

// 跨域：停止提示脉冲（100MHz 单拍 -> video_clk 域展宽成 1 拍脉冲）
// 必须展宽 ：25MHz 采 100MHz 的单拍脉冲，直接打两拍会漏采（4 拍里只碰到 1 拍），
//     用脉冲保持 + 应答的方式成本又太高。这里改成"在 100MHz 域把它展宽到
//     8 个 100MHz 拍（= 2 个 video_clk 周期）"，video 域 2 级同步后一定采到。
reg  [2:0] banner_stretch;       // 100MHz
reg        banner_lvl;           // 100MHz：展宽后的电平
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        banner_stretch <= 3'd0;
        banner_lvl     <= 1'b0;
    end else begin
        if (sd_stop_banner)
            banner_stretch <= 3'd6;      // 让 video 域有 ≥2 个周期可采
        else if (banner_stretch != 3'd0)
            banner_stretch <= banner_stretch - 3'd1;
        banner_lvl <= (banner_stretch != 3'd0) | sd_stop_banner;
    end
end

reg banner_v1, banner_v2, banner_v3;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        banner_v1 <= 1'b0;
        banner_v2 <= 1'b0;
        banner_v3 <= 1'b0;
    end else begin
        banner_v1 <= banner_lvl;
        banner_v2 <= banner_v1;
        banner_v3 <= banner_v2;
    end
end
wire banner_pulse_v = banner_v2 & ~banner_v3;   // video 域单拍脉冲

// 跨域：自动轮播短提示脉冲（100MHz 单拍 -> video_clk 域）
//   与上面停止提示完全同构（展宽到 8 个 100MHz 拍再 2 级同步），
//   独立一套寄存器，不复用 —— 两个提示的触发源、时长都不同，
//   共用一套"展宽电平"会在两路同时到来时丢掉一个。
reg  [2:0] abanner_stretch;
reg        abanner_lvl;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        abanner_stretch <= 3'd0;
        abanner_lvl     <= 1'b0;
    end else begin
        if (sd_auto_hint)
            abanner_stretch <= 3'd6;
        else if (abanner_stretch != 3'd0)
            abanner_stretch <= abanner_stretch - 3'd1;
        abanner_lvl <= (abanner_stretch != 3'd0) | sd_auto_hint;
    end
end

reg abanner_v1, abanner_v2, abanner_v3;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        abanner_v1 <= 1'b0;
        abanner_v2 <= 1'b0;
        abanner_v3 <= 1'b0;
    end else begin
        abanner_v1 <= abanner_lvl;
        abanner_v2 <= abanner_v1;
        abanner_v3 <= abanner_v2;
    end
end
wire abanner_pulse_v = abanner_v2 & ~abanner_v3;

// HDMI 音频链路（自研：video_clk 域直接产生 PCM，不引入额外音频 PLL）
wire        audio_valid;         // 音频样本有效脉冲（48kHz）
wire [23:0] audio_left_data;     // 左声道 PCM
wire [23:0] audio_right_data;    // 右声道 PCM
wire        acr_valid;           // ACR 更新脉冲
wire [19:0] acr_cts;             // ACR CTS
wire [19:0] acr_n;               // ACR N

// TF 卡语音播放（wav_player 在 sd_card_bmp 内，
// 它的音频输出已是 video_clk 域，与上面 audio_* 同域，无需再跨时钟）
wire        wav_valid;           // WAV 音频样本有效脉冲（48kHz）
wire [23:0] wav_left;            // 左声道 PCM
wire [23:0] wav_right;           // 右声道 PCM
wire        wav_found;           // 已扫描到 WAV（用于切换音源）
wire [4:0]  wav_state;           // WAV 播放状态码：0~5
wire [6:0]  wav_seg;             // 状态码的七段译码结果

// ===================== OSD 参数调节 =====================
wire        sw3_sync;            // sw3 同步到 100MHz 域
wire        sw4_sync;            // sw4 同步到 100MHz 域（播放模式：1=自动 0=手动）
wire        sd_key1_press;       // 板载 KEY_2 消抖脉冲
wire        sd_key2_press;       // 板载 KEY_3
wire        sd_key3_press;       // 板载 KEY_4
wire [6:0]  osd_brightness;      // 0~100
wire [6:0]  osd_contrast;        // 0~100
wire [6:0]  osd_volume;          // 0~100（默认 70）
wire [7:0]  osd_brt_gain;        // 100MHz 域，组合增益 Q7 = 对比度 x 亮度
wire [7:0]  osd_venh_off;        // 100MHz 域，偏移（偏置 128）
wire [7:0]  osd_knee;            // 100MHz 域，高光肩起点
wire [7:0]  osd_vol_q7;          // 100MHz 域，音量 Q7
wire        osd_param_tgl;       // 参数更新翻转（跨域握手）
wire        osd_block_page;      // OSD 打开时屏蔽翻页
// 跨到 25MHz 视频/音频域后的参数副本
reg  [7:0]  venh_brt;            // 亮度乘法增益（已跨域到 video_clk 域）
reg  [7:0]  venh_vol;            // Q7 音量
wire [23:0] vout_enh;            // 亮度/对比度处理后的 RGB
wire        vs_enh, de_enh;      // 同步延迟后的 vs/de
wire [23:0] audio_raw_left;      // 音量处理前的 PCM
wire [23:0] audio_raw_right;
wire [23:0] audio_vol_left;      // 音量处理后的 PCM
wire [23:0] audio_vol_right;

// ===================== OSD 叠加 =====================
wire [1:0]  osd_param_sel;       // 100MHz 域：0=亮度 1=对比度 2=音量
reg  [1:0]  venh_sel;            // 视频域副本（供 OSD 显示）
reg  [6:0]  venh_bright_val;     // 视频域里的亮度值（供 OSD 显示）
reg  [6:0]  venh_ctr_val;        // 视频域里的对比度值
reg  [6:0]  venh_vol_val;        // 视频域里的音量值
reg  [7:0]  venh_off;            // 视频域里的偏移（偏置 128）
reg  [7:0]  venh_knee;           // 视频域里的高光肩起点
reg         sw3_v1, sw3_v2;      // SW3 同步到视频域（OSD 显示开关）
reg         sw3_v3;              // 再延一拍，用来取 SW3 的跳变沿
wire        osd_trigger;         // OSD 重新计时触发（按键 或 进入调节模式）
wire [23:0] vout_osd;            // OSD 叠加后的 RGB
wire        vs_osd, de_osd;      // 同步延迟后的 vs/de

// 轮播设置菜单 / 停止提示条叠加后的最终画面
wire [23:0] vout_menu;
wire        vs_menu, de_menu;

// ===================== 转场 =====================
wire        sd_trans_pending;    // 100MHz：新图已就绪，等待切换
wire        sd_trans_skip;       // 100MHz：本次是第一张图，跳过转场
wire        sd_trans_force_push; // 100MHz：本次切换固定用上下推移（手动翻页）
wire        trans_hold;          // video ：拉住换缓冲（遮罩期间）
wire [23:0] vout_tr;             // 转场处理后的 RGB
wire        vs_tr, de_tr;        // 同步延迟后的 vs/de
// trans_hold 从 video 域同步回 100MHz 域，给 sd_card_bmp 用
//（声明必须在使用它的 sd_card_bmp 例化之前，否则是前向引用）
reg         trans_hold_d1, trans_hold_d2;
// trans_busy 同样同步回 100MHz 域，用来门控轮播计时器
reg         trans_busy_d1, trans_busy_d2;
reg         trans_go_d1, trans_go_d2;      // 换图许可 -> 100MHz 域
wire        trans_busy;          // video：转场进行中（推移约 1.0s / 卷曲约 2.35s）
wire        trans_go;            // video：换图许可（默认 0，只有转场模块放行才 1）

// 统一复位：TF 图像链路 + HDMI 链路
wire sys_pll_lock;
wire video_pll_lock;

// 复位稳定延时参数化（2026-10-22）
//   原来直接写死 20'd1_000_000，换晶振频率就静默失效（延时按比例偏）。
//   语义：两个 PLL 都 lock 后，再连续稳定 RST_DELAY_CYC 个 clk 周期才释放复位。
//   默认值 1_000_000 @ 50MHz = 20ms，与原行为逐位相同。
//   换频率时按 RST_DELAY_CYC = f_clk(Hz) × 0.02 重新计算。
localparam [19:0] RST_DELAY_CYC = 20'd1_000_000;

reg [19:0] rst_cnt = 20'd0;
reg rst_all_reg = 1'b1;

always @(posedge clk) begin
    if (!rst_n) begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end else if (!rst_all_reg) begin
        // 一旦复位释放，忽略后续微小的 PLL 抖动，防止系统在使用时随机复位
        rst_all_reg <= 1'b0;
    end else if (sys_pll_lock && video_pll_lock) begin
        // 等待 PLL 连续稳定 RST_DELAY_CYC 拍（默认 20ms @ 50MHz）
        if (rst_cnt < RST_DELAY_CYC) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst_all_reg <= 1'b1;
        end else begin
            rst_all_reg <= 1'b0;
        end
    end else begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end
end

assign rst_all = rst_all_reg;

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (sys_pll_lock),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (video_pll_lock),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    // ---- c70：以下参数按 sd_card_clk 由 100MHz 降到 62.5MHz 同步缩放（×0.625）----
    //   这些都是"按拍数写死的时间"，漏改会导致：3 秒档变 4.8 秒、10 秒档变 16 秒、
    //   讲解档停顿翻倍、WAV 时长算成 1.6 倍。
    .CLK_FREQ_HZ            (62_500_000),
    .PERIOD_3S_TICKS        (32'd187_500_000),   // 3 秒
    .PERIOD_5S_TICKS        (32'd312_500_000),   // 5 秒
    .PERIOD_10S_TICKS       (32'd625_000_000),   // 10 秒
    .DWELL_AUDIO_DELAY_CYC  (33'd93_750_000),    // 1.5 秒
    .DWELL_AUDIO_EXTRA_CYC  (33'd156_250_000),   // 2.5 秒
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    // 阶段 3a 总开关
    //   1 = 启用 FAT32 根目录解析（结果 -> 数码管位 1/2）
    //   0 = 完全退回旧行为（FAT 逻辑被综合器折叠掉，数码管位 1/2 熄灭）
    //   想退回：把下面这一行改成 .USE_FAT_SCAN(1'b0)，或整行删掉
    //   （sd_card_bmp 里的参数默认值就是 0）。
    //   FAT_START_DLY_CYC 保持默认 = 10 秒（100MHz 下），
    //   给"扫图 + 首图显示"留足时间，FAT 解析只在之后用空闲时间。
    .USE_FAT_SCAN      (1'b1),
    // 阶段 3b 总开关
    //   1 = 用 FAT 文件表直接装载图片（启动约 2 秒，推荐）
    //   0 = 退回旧通路（bmp_read 扫扇区，约 24 秒），代码一行没删
    //   想退回：把下面这一行改成 .USE_FAT_LOAD(1'b0)，或整行删掉
    //   （sd_card_bmp 里这个参数的默认值就是 0）。
    // USE_FAT_LOAD=1 时 USE_FAT_SCAN 的 3a 扫描会自动让位（不再例化），
    // 两者不会同时抢卡。
    .USE_FAT_LOAD      (1'b1),
    // 阶段 4 总开关：图片停留时长 = 讲解音频长度 + 1 秒
    //   1 = 按 FAT 表里第一个 WAV 的字节数算停留时长：
    //         音频时长 + 1.5 秒（语音起播延迟）+ 1 秒（播完后的停顿）
    //         真卡 VOICE.WAV=791378 字节 -> 8.238 秒 + 2.5 秒 = 10.74 秒
    //   0 = 退回阶段 4 之前：固定 5 秒（与上板验证过的 3b 行为一致）
    // 一键切回固定 5 秒：把下面这一行的 1'b1 改成 1'b0
    // 只从卡上第一个WAV 取时长（本阶段卡上只有一个 VOICE.WAV）；
    //     阶段 3c 会改成"按序号配对 01.BMP <-> 01.WAV"，见 sd_card_bmp.v 里
    // "阶段 4：停留时长"那一段的说明。
    // USE_FAT_LOAD=0（旧通路）时本开关自动失效（没有文件表可用）
    .USE_DWELL_FROM_WAV(1'b1),
    // 阶段 3c 总开关：音频也走 FAT 簇链 + 每张图播自己配的那段讲解
    //   1 = (A) 音频改成按 FAT 簇链读（根治"呼呼呼的嘈杂声"：碎片化卡上
    //           顺序读会读到别的文件的数据；wav_player.v 一个字没改，
    //           只换了它的扇区数据来源 —— 见 sd_card_bmp.v 里 wav_src_fat）
    //       (B) 01.BMP <-> 01.WAV 按序号配对（见 fat32_boot.v 文件头）
    //       (C) 每张图出现时都重播它自己那段讲解（换图时重启音频那一路）
    //   0 = 一键退回阶段 4/3b 的行为：音频仍从扫描起点顺序往后读、
    //       停留时长仍按"表里第一个 WAV"算、只有第一张图有声音。
    //       （wav_src_fat 整个不例化，资源与上板验证过的阶段 4 相同）
    // 想退回：把下面这一行的 1'b1 改成 1'b0（或整行删掉）
    // 只有 USE_FAT_LOAD=1 时才有意义；USE_FAT_LOAD=0 时本开关
    // 会被 sd_card_bmp 里的 USE_FAT_WAV_EFF 自动折叠成 0
    .USE_FAT_WAV(1'b1),
    // S2 总开关：滚动字幕掩膜条（0N.SUB）装载
    //   由上面那个 USE_SUBTITLE 参数统一控制（一键回退见那里的说明）。
    // 只有 USE_FAT_LOAD=1 时才有意义（配对表来自 fat32_boot）
    .USE_SUBTITLE(USE_SUBTITLE),
    //=========================================================================
    // 本次新增：按键交互 + 轮播周期档位（四个一键回退开关）
    //   四个开关的完整说明见 sd_card_bmp.v 文件头的参数注释。
    //   回退方法：把对应那一行整个删掉（参数默认值就是"新行为"），
    //   或者把值改成下面的"退回值"：
    //     .USE_KEY_CTRL     (1'b1)  -> 改 1'b0 = 完全退回本次改动前
    //     .USE_PERIOD_STAGE (1'b1)  -> 改 1'b0 = 只有讲解档（KEY_4 失效）
    //     .KEY_LOCK_ON_SW4  (1'b1)  -> 改 1'b0 = SW4=0 不锁三键
    //     .AUTO_PLAY_ON_BOOT(1'b0)  -> 改 1'b1 = 上电就自动轮播
    // 这四行在这里显式写出来（而不是靠默认值）：
    //     将来用户想回退时，一眼能在顶层找到开关，不用翻到 sd_card_bmp 里。
    //=========================================================================
    .USE_KEY_CTRL      (1'b1),
    .USE_PERIOD_STAGE  (1'b1),
    .KEY_LOCK_ON_SW4   (1'b1),
    .AUTO_PLAY_ON_BOOT (1'b0)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key1),
    .key_auto          (key2),
    .key_period        (key3),
    // SW3=1 时 KEY_4 归 osd_ctrl 调当前参数（亮度/对比度/音量），手动档的"播/停当前图音频"让路
    .i_sw3             (sw3_sync),
    .state_code        (state_code),
    .period_sel        (period_sel),
    // 本次：位 4 改成显示"轮播周期档位"，不再显示图片序号
    //   图片序号这一路仍然接出来（img_idx_disp），只是当前没有消费者 ——
    // 为什么保留连接而不是写成 () 本项目踩过的坑：把输出悬空会让
    //     TD 把它上游的组合逻辑一起优化掉，新构建的资源数就没法与基线对照
    //     （实测：同时悬空 write_buf_idx/disp_buf_idx/o_img_idx 后
    //      gate #lut 少了约 1,100、#bram 少了 2 —— 全是"少接了几根线"造成的，
    //      不是改动本身省了资源）。保留连接 = 对照才有意义。
    //   想恢复显示图片序号：把下面 seg 那一段改回 seg_decoder + img_idx_disp 即可。
    .o_img_idx         (img_idx_disp),
    .o_bmp_prog_gray   (bmp_prog_gray),
    .o_bmp_busy        (bmp_load_busy),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso),

    // ===== TF 卡语音播放（新增）=====
    .clk_audio         (video_clk),      // 25MHz 音频域
    .o_audio_valid     (wav_valid),
    .o_audio_left      (wav_left),
    .o_audio_right     (wav_right),
    .o_wav_found       (wav_found),
    .o_wav_state       (wav_state),

    // ===== 供 OSD 菜单复用的消抖按键 + 屏蔽翻页 =====
    .o_key1_press      (sd_key1_press),
    .o_key2_press      (sd_key2_press),
    .o_key3_press      (sd_key3_press),
    .i_block_page      (osd_block_page),
    // ===== 转场握手 =====
    .i_trans_hold      (trans_hold_d2),
    .o_trans_pending   (sd_trans_pending),
    .o_trans_skip      (sd_trans_skip),
    .o_trans_force_push(sd_trans_force_push),
    .i_trans_busy      (trans_busy_d2),
    .i_auto_mode       (sw4_sync),
    .i_trans_go        (trans_go_d2),

    // ===== 阶段 3a：FAT32 根目录解析结果 -> 数码管位 1/2 =====
    .o_fat_count       (fat_count),
    .o_fat_err         (fat_err),
    .o_fat_cnt_ovf     (fat_cnt_ovf),
    .o_fat_cnt_digit   (fat_cnt_digit),
    .o_fat_done        (fat_done),

    // ===== 临时诊断：内部关键值（只在 DIAG_MODE=1 时被使用）=====
    .o_fat_dbg_root_lba  (dbg_root_lba),
    .o_fat_dbg_part_lba  (dbg_part_lba),
    .o_fat_dbg_data_lba  (dbg_data_lba),
    .o_fat_dbg_fat_lba   (dbg_fat_lba),
    .o_fat_dbg_secs      (dbg_secs),
    .o_fat_dbg_part_type (dbg_part_type),
    .o_fat_dbg_root_b0   (dbg_root_b0),
    .o_fat_dbg_root_b1   (dbg_root_b1),
    .o_fat_dbg_rx_bytes  (dbg_rx_bytes),

    // ===== 阶段 3b：装载通路诊断（只在 DIAG_MODE=1 时被使用） =====
    //   USE_FAT_LOAD=0 时这三个信号恒 0（sd_card_bmp 里是常量选择）。
    .o_fat3b_bmp_cnt     (fat3b_bmp_cnt),
    .o_fat3b_load_err    (fat3b_load_err),
    .o_fat3b_load_bytes  (fat3b_load_bytes),

    // ===== S2：滚动字幕掩膜条（0N.SUB） =====
    // 这 7 根必须接上，否则 TD 会把整块掩膜 RAM 静默剪掉
    //   实测（2026-10-10）：漏接这 7 根时，
    //     于是 sub_ram_we 是悬空网络 -> RAM 永远不被写 -> TD 的 opt_rtl
    //     把 sub_mask_ram 连同它的读侧消费者一起优化掉，
    //     而日志里一条警告都没有（#bram 20 没涨、SYN-1023 还是 2 BRAM）。
    //     iverilog 也不报（它只对"悬空的输入端口"报 dangling，对"没接的
    //     例化输出/悬空网络"不报）—— 只能靠"综合后看 #bram 涨没涨"来发现。
    //   USE_SUBTITLE=0 时 sd_card_bmp 里这 7 根输出恒 0（常量选择），
    //   掩膜 RAM 的写口就是常量 0，等价于"这条通路不存在"。
    .o_sub_ram_we        (sub_ram_we),
    .o_sub_ram_addr      (sub_ram_addr),
    .o_sub_ram_wdata     (sub_ram_wdata),
    .o_sub_cols          (sub_cols),
    .o_sub_ok            (sub_ok),
    .o_sub_found         (sub_found),
    .o_sub_load_err      (sub_load_err),
    .o_sub_busy          (sub_busy),         // S3fix：装载中 -> 门控拉低
    // S4 新增：wav_player 的 o_playing（clk_audio = video_clk 域）
    //   滚动字幕的同步触发源。它是"确实已经出声"而不是"允许起播"，
    //   而且与显示/掩膜读同一个时钟域 —— 零跨域成本。
    .o_wav_playing       (sub_playing),      // S4：音频起播 -> 偏移归零并开始滚

    // ===== 按键交互：音频闸门 + 档位指示（本次新增的两个输出） =====
    // 这两根必须接真实消费者，否则 TD 可能把相关逻辑当死逻辑剪掉
    //     key_audio_gate    -> 只在本文件里被"按键档位数码管"那一块读
    //                          （见下面 7 段显示那一段；它同时是真实的功能信号，
    //                           在 sd_card_bmp 里是 wav_play_go 的闸门）
    //     key_period_explain-> 同上
    // 故意不把它们接到 seg 输出上 ：数码管显示的是 period_sel
    //     的译码结果，与这两个信号同源，所以接不接都不改变可观测行为；
    //     但它们必须被"某处真实逻辑"读到 —— 下面那一段判"讲解档"的
    //     组合逻辑就是消费者。
    .o_audio_gate        (key_audio_gate),
    .o_period_is_explain (key_period_explain),

    // ===== 轮播设置模式接口 =====
    //   i_setup_mode：SW1（100MHz 域已同步）。置 1 时：
    //     - 自动轮播强制停（auto_play_en 清 0，自动切图分支被否决）
    //     - 音频闸门强制关（讲解模式也不出声）
    //     - KEY2(key_next) 走菜单光标，KEY3(key_auto) 确认并存档
    .i_setup_mode        (sw1_s2),
    .o_menu_sel          (sd_menu_sel),
    .o_cfg_sel           (sd_cfg_sel),
    .o_stop_banner       (sd_stop_banner),
    .o_manual_mode       (sd_manual_mode),
    .o_auto_hint         (sd_auto_hint)
);

// ===================== 拨码开关同步（100MHz 域）=====================
// DIP 是慢速异步输入，2 级同步即可，不需要消抖（拨码不会抖动）
reg sw3_s1, sw3_s2;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        sw1_s1 <= 1'b0;
        sw1_s2 <= 1'b0;
        sw3_s1 <= 1'b0;
        sw3_s2 <= 1'b0;
        sw4_s1 <= 1'b0;
        sw4_s2 <= 1'b0;
    end else begin
        sw1_s1 <= sw1;
        sw1_s2 <= sw1_s1;
        sw3_s1 <= sw3;
        sw3_s2 <= sw3_s1;
        sw4_s1 <= sw4;
        sw4_s2 <= sw4_s1;
    end
end
assign sw3_sync = sw3_s2;
assign sw4_sync = sw4_s2;

// ===================== OSD 参数控制 =====================
osd_ctrl u_osd_ctrl (
    .I_clk        (sd_card_clk),      // 100MHz
    .I_rst        (rst_all),
    .I_sw3        (sw3_sync),
    .I_key1_press (sd_key1_press),
    .I_key2_press (sd_key2_press),
    .I_key3_press (sd_key3_press),
    .O_brightness (osd_brightness),
    .O_contrast   (osd_contrast),
    .O_volume     (osd_volume),
    .O_brt_gain   (osd_brt_gain),
    .O_venh_off   (osd_venh_off),     // 对比度偏移（偏置 128）
    .O_knee       (osd_knee),         // 高光肩起点
    .O_vol_q7     (osd_vol_q7),
    .O_param_sel  (osd_param_sel),
    .O_param_tgl  (osd_param_tgl),
    .O_block_page (osd_block_page)
);

// ===================== 参数跨域到视频域（toggle 握手）=====================
// 多比特值不能逐位 2 级同步（变化瞬间会撕裂，比如 63->64 多位同时翻转）。
// 这里用"数据 + 翻转位"：翻转信号同步过去后，一次性把两个值都锁存。
// 因为参数在 100MHz 域已寄存，等到翻转信号传到 25MHz 域时（≥120ns 后）
// 数据早已稳定，采样是安全的。
reg tgl_s1, tgl_s2, tgl_s3;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        tgl_s1 <= 1'b0;
        tgl_s2 <= 1'b0;
        tgl_s3 <= 1'b0;
    end else begin
        tgl_s1 <= osd_param_tgl;
        tgl_s2 <= tgl_s1;
        tgl_s3 <= tgl_s2;
    end
end

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        venh_brt        <= 8'd128;       // 组合增益中性（1.0x）
        venh_off        <= 8'd128;       // 偏移中性（真值 0）
        venh_knee       <= 8'd255;       // 中性：不压高光
        venh_vol        <= 8'd128;       // 音量默认 100 档 = 0dB
        venh_bright_val <= 7'd50;        // 亮度默认 50（中性）
        venh_ctr_val    <= 7'd50;        // 对比度默认 50（1.0x）
        venh_vol_val    <= 7'd80;        // 音量默认 80
        venh_sel        <= 2'd0;         // 默认调亮度
    end else if (tgl_s2 != tgl_s3) begin
        // 全部参数一次性锁存（同一次翻转），保证不会出现新旧混搭
        venh_brt        <= osd_brt_gain;
        venh_off        <= osd_venh_off;
        venh_knee       <= osd_knee;
        venh_vol        <= osd_vol_q7;
        venh_bright_val <= osd_brightness;   // 供 OSD 显示用
        venh_ctr_val    <= osd_contrast;
        venh_vol_val    <= osd_volume;
        venh_sel        <= osd_param_sel;
    end
end

// SW3 同步到视频域（OSD 显示开关），单比特慢速电平 2 级同步即可
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        sw3_v1 <= 1'b0;
        sw3_v2 <= 1'b0;
        sw3_v3 <= 1'b0;
    end else begin
        sw3_v1 <= sw3_sync;
        sw3_v2 <= sw3_v1;
        sw3_v3 <= sw3_v2;
    end
end

// OSD 重新计时触发：
//   - 任何参数变化（跨域锁存事件，tgl_s2 != tgl_s3 只持续一拍）
//   - 刚拨到调节模式（SW3 上升沿）
// 这样"一进调节模式就出现，之后每次按键刷新，停手 3 秒自动隐藏"。
assign osd_trigger = (tgl_s2 != tgl_s3) | (sw3_v2 & ~sw3_v3);

// ===================== 转场信号跨域 =====================
//   o_trans_pending / o_trans_skip / o_trans_force_push : 100MHz -> video
//     （单比特慢速电平，2 级同步）
//   trans_hold                     : video  -> 100MHz（同上）
//   sw2                            : 引脚   -> video（转场模式选择）
reg  trans_pend_s1, trans_pend_s2;
reg  trans_skip_s1, trans_skip_s2;
reg  trans_fpush_s1, trans_fpush_s2;
reg  dispval_s1,    dispval_s2;     // display_valid -> video 域（判断是否已有图）
reg  sw2_v1, sw2_v2;

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        trans_pend_s1 <= 1'b0;
        trans_pend_s2 <= 1'b0;
        trans_skip_s1 <= 1'b0;
        trans_skip_s2 <= 1'b0;
        trans_fpush_s1 <= 1'b0;
        trans_fpush_s2 <= 1'b0;
        dispval_s1    <= 1'b0;
        dispval_s2    <= 1'b0;
        sw2_v1        <= 1'b0;
        sw2_v2        <= 1'b0;
    end else begin
        trans_pend_s1 <= sd_trans_pending;
        trans_pend_s2 <= trans_pend_s1;
        trans_skip_s1 <= sd_trans_skip;
        trans_skip_s2 <= trans_skip_s1;
        trans_fpush_s1 <= sd_trans_force_push;
        trans_fpush_s2 <= trans_fpush_s1;
        dispval_s1    <= display_valid;
        dispval_s2    <= dispval_s1;
        sw2_v1        <= sw2;
        sw2_v2        <= sw2_v1;
    end
end

// 转场类型掩码：手动翻页固定推移，自动轮播听 SW2
//   只在这里做一次掩码，video_transition 内部不用改 —— 它拿到什么就用什么，
//   也就不会在 6 个现存 TB（tb_push_fsm / tb_curl_pub / tb_push_eq / ...）
//   里多出一个必须驱动的端口。
wire sw2_tr_mode = sw2_v2 & ~trans_fpush_s2;   // 1=卷曲；手动翻页时强制 0（推移）

// trans_hold 从 video 域回到 100MHz 域（声明已在前面转场那一节）
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        trans_hold_d1 <= 1'b0;
        trans_hold_d2 <= 1'b0;
        trans_busy_d1 <= 1'b0;
        trans_busy_d2 <= 1'b0;
        trans_go_d1   <= 1'b0;
        trans_go_d2   <= 1'b0;
    end else begin
        trans_hold_d1 <= trans_hold;
        trans_hold_d2 <= trans_hold_d1;
        trans_busy_d1 <= trans_busy;
        trans_busy_d2 <= trans_busy_d1;
        trans_go_d1   <= trans_go;
        trans_go_d2   <= trans_go_d1;
    end
end

// ===================== S2：字幕掩膜 RAM（写 100MHz / 读 25MHz）=====================
// 这段写法是探针实测出来的，三个坑一个都不能踩
//     （原始证据：tools/_probe/探针报告_2026-10-10.md §1）
//     ① 读口那个 always 里只能有 rdata <= mem[raddr]，绝对不要出现 rst
//        （加复位立刻退化成 LUT RAM：11 块 BRAM -> 0、LUT 45 -> 13494）
//     ② 绝不要复位整个数组（会综合出 98352 个 DFF，器件只有 19600 个）
//     ③ 绝不要写成 48 个 1bit 数组（TD 会卡死）
//   实测：2048x48x1bpp = 11 x 9Kb BRAM、#bram32k 仍然 0、#lut 只涨 45、
//   #reg 只涨 1；双时钟（写 100MHz / 读 25MHz）同样是 11 块，
//   所以"装载"与"逐像素读"可以真并行，不需要为省 BRAM 去仲裁同一个口。
// RAM 本体写在文件末尾的 sub_mask_ram 模块里（与顶层同一个 .v，
// 不新增文件 -> 用户不需要在 TD 里 Add Source File）
// 一键回退必须把 RAM 本体也一起去掉
//     第一版把 RAM 无条件例化，结果 USE_SUBTITLE=0 时 TD 照样给 31 块 BRAM
//     （写口是常量 0，但读口还在被诊断消费者扫描 -> CD 认为这块 RAM 还是要的）。
//     所以这里用 generate 门住例化：关掉时连 RAM 一起消失，资源回到基线。
// S3fix3：清零循环/门控用到的中间网 —— 必须先声明后使用（规矩 2）
//   上一版把这几根声明写在例化之后，靠 Verilog 的隐式 1 位网络兜住 ——
//   隐含网络是 1 位的，装载地址/数据有被截成 1 位的风险，TD 也会报 HDL-5373。
//   这里挪到例化之前，与注释里说的做法一致。
wire        sub_ram_we_m;        // 写口最终写使能 = 装载器 OR 清零循环
wire [10:0] sub_ram_addr_m;      // 写口最终地址
wire [47:0] sub_ram_wdata_m;     // 写口最终数据
wire        sub_valid_raw_m;     // 门控（100MHz 域）
reg         sub_clr_we;          // 清零循环写使能（由下面那个 FSM 驱动）

generate
if (USE_SUBTITLE != 0) begin
    sub_mask_ram u_sub_mask_ram (
        .clk_wr (sd_card_clk),      // 装载域 100MHz
        .clk_rd (video_clk),        // 显示域  25MHz
        .we     (sub_ram_we_m),     // S3fix3：装载器 OR 清零循环（装载器绝对优先）
        .waddr  (sub_ram_addr_m),   // S3fix3：装载时 = 装载器，清零时 = 计数器
        .wdata  (sub_ram_wdata_m),  // S3fix3：装载时 = 真数据，清零时 = 48'd0
        .raddr  (sub_mask_addr),
        .rdata  (sub_rdata)
    );
end
endgenerate

// 关掉时把读数据折成常量 0（单条 assign + 编译期常量选择；
// 不许两个 generate 分支各写一句 assign 同一个 net —— MD-ASSIGN 会误判）
wire [47:0] sub_rdata_m = (USE_SUBTITLE != 0) ? sub_rdata : 48'd0;

// ============================================================================
// S3fix：掩码有效门控 + 装载前清 RAM
// ----------------------------------------------------------------------------
// 上板实测的 bug：卡上没有 .SUB -> 掩膜 RAM 从来没被写过 -> BRAM 上电初值
// 不一定为 0-> sub_overlay 的 mask_on 是随机值 -> 字幕条区域显示随机噪声。
//
// 修法（两条，互相独立）：
//   ① sub_valid：只有"清完 RAM 且不在装载中"才为 1；为 0 时叠加层完全透传。
//   ② 装载前先用一个小 FSM 逐拍写 0 清一遍 RAM（2048 拍 = 20.48us @100MHz）。
// 绝对不能用"复位时清整个数组"：本项目实测会综合出 98352 个 DFF
// 只复位 FSM 的计数器，数组本体一个初值都不写
//
// sub_valid 的状态转移（100MHz 域，sd_card_clk 与 sub_ram_we 同域）：
//   复位            -> 0
//   清 RAM 正在跑    -> 0
//   清 RAM 跑完(ok)  -> 1     （此时 RAM 全 0 = 干净的透明底）
//   装载 sub_busy 起 -> 0     （半截掩膜绝不允许上屏；写总线被装载器占用）
// 装载成功(sub_ok) -> 保留刚装好的掩膜，不清零 -> 1（S3fix2）
//   装载失败/中止     -> 再清一遍抹掉垃圾 -> 1                 （S3fix2）
//
// 为什么整段不放进 generate
//   放进 generate 就会出现"同一个 net 在两个分支各 assign 一句"，
//   tools/verilog_multidriver_check.py 判 MD-ASSIGN（实测 4 处错误）——
//   与本文件其它地方（sub_rdata_m / o_sub_* 那 7 根）的写法保持一致：
//   单条 assign + 编译期常量选择。
//   USE_SUBTITLE=0 时：sub_clr_go=0 -> 清零 FSM 一步不走 -> 它的输出只喂给
//   被折成常量的选择支 -> 没有消费者 -> TD 自己会把这些寄存器剪掉（不占资源）。
// ============================================================================
reg [1:0]  sub_clr_st;       // 0=等装载器 1=清零循环 2=等装载器释放
reg [11:0] sub_clr_cnt;      // 清零循环计数器 0..2047
reg        sub_busy_d;       // 装载忙的上一拍（慢速事件，1 级同步）
reg        sub_loaded;       // 清完之后的第一次装载还没来过
reg        sub_done;         // S3fix3：装载成功过 -> 永不再清零（粘滞，只被复位清）
reg        sub_valid_reg;    // 门控寄存器：只有它决定"能不能叠加"

wire sub_clr_go = (USE_SUBTITLE != 0);   // 编译期常量：关掉总开关时永不启动

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        sub_clr_st    <= 2'd0;
        sub_clr_cnt   <= 12'd0;
        sub_clr_we    <= 1'b0;
        sub_busy_d    <= 1'b0;
        sub_loaded    <= 1'b0;
        sub_done      <= 1'b0;
        sub_valid_reg <= 1'b0;           // 上电默认"无效"= 完全透传
    end else begin
        sub_busy_d <= sub_busy;
        if (!sub_clr_go) begin
            sub_clr_we    <= 1'b0;
            sub_clr_cnt   <= 12'd0;
            sub_clr_st    <= 2'd0;
            sub_valid_reg <= 1'b0;
        end else begin
            case (sub_clr_st)
                2'd0: begin
                    sub_clr_we  <= 1'b0;
                    sub_clr_cnt <= 12'd0;
                    if (sub_busy) begin
                        // S3fix3：装载器起来时，只有从没装成功过才把门控拉低。
                        // 装成功过 -> 掩膜留在 RAM 里，门控保持 1（不清零、不掉门控）
                        sub_valid_reg <= sub_done ? 1'b1 : 1'b0;
                        sub_clr_st    <= 2'd2;
                    end else if (!sub_loaded && !sub_done) begin
                        // S3fix3：清零只允许在第一次成功装载之前跑
                        sub_clr_we <= 1'b1;
                        sub_clr_st <= 2'd1;   // 这一次装载还没来过 -> 先清 RAM
                    end
                end
                2'd1: begin
                    // S3fix3：装载器对写口有绝对优先权（写口 mux 也按这个优先级接）
                    //   一旦装载器起来，清零立刻让路；半截清零无害 ——
                    //   完整的装载会重写全部 2048 列，失败的装载在状态 2 回头再清。
                    if (sub_busy) begin
                        sub_clr_we  <= 1'b0;
                        sub_clr_cnt <= 12'd0;
                        sub_clr_st  <= 2'd2;      // 交给状态 2 等装载结束
                    end else begin
                    sub_clr_we <= 1'b1;       // 每拍写一列 48'd0
                    if (sub_clr_cnt == 12'd2047) begin
                        sub_clr_we    <= 1'b0;
                        sub_clr_cnt   <= 12'd0;
                        sub_loaded    <= 1'b1;  // 这一次清零完成，接下来等装载
                        sub_valid_reg <= 1'b1;  // 清完 -> RAM 干净 -> 允许叠加
                        sub_clr_st    <= 2'd0;
                    end else begin
                        sub_clr_cnt <= sub_clr_cnt + 12'd1;
                    end
                    end
                end
                2'd2: begin
                    sub_clr_we <= 1'b0;
                    // S3fix3：装载结束后绝对不再清零
                    //   S3fix2 版在这里对"失败"一律 sub_loaded<=0 -> 回状态 0 -> 再清一遍。
                    //   那条路径就是"清零循环把刚装好的掩膜抹掉"的入口：
                    //   只要有一次装载没报 ok（超时/中止/半截），2048 拍清零就把
                    //   RAM 里那条完整的掩膜全写成 0 —— 而页3 sub_cols 仍是 2048、
                    // 页2 仍是 0040，看着像"装载完全成功但数据没了"
                    //   现在：成功过(sub_done) -> 粘滞，永不再清零；只有从没成功过才清零。
                    if (!sub_busy) begin
                        if (sub_ok) begin
                            // 完整成功（头/12304 字节/2048 列/簇链全对）
                            sub_loaded    <= 1'b1;
                            sub_done      <= 1'b1;   // 粘滞：此后再也不许清零
                            sub_valid_reg <= 1'b1;   // 掩膜可用 -> 允许叠加
                        end else if (sub_done) begin
                            // 已经装成功过：这次失败保留上一次的掩膜，不许清零
                            sub_valid_reg <= 1'b1;
                        end else begin
                            sub_loaded <= 1'b0;      // 一次都没成功过 -> 清一遍抹掉垃圾
                        end
                        sub_clr_st <= 2'd0;
                    end
                end
                default: sub_clr_st <= 2'd0;
            endcase
        end
    end
end

// ---- 掩膜 RAM 写口的最终来源（每条 net 只有这一句 assign）----
// S3fix3：装载器绝对优先
//     旧版是"清零有效就选清零的地址/数据"，于是只要清零循环和装载器同拍，
//     装载器那 2048 列真数据会被换成 48'd0 写到清零计数器的地址上
// —— 计数器照样走满 2048、sub_ok 照样 =1，但 RAM 里全是 0
//     现在写使能仍然相或（不丢拍），但地址/数据装载器优先：
// 清零永远不可能覆盖装载器当拍写进去的那一列
assign sub_ram_we_m    = (USE_SUBTITLE != 0) ? (sub_ram_we | sub_clr_we) : 1'b0;
assign sub_ram_addr_m  = ((USE_SUBTITLE != 0) && !sub_ram_we && sub_clr_we) ? sub_clr_cnt[10:0]
                          : ((USE_SUBTITLE != 0) ? sub_ram_addr : 11'd0);
assign sub_ram_wdata_m = ((USE_SUBTITLE != 0) && !sub_ram_we && sub_clr_we) ? 48'd0
                          : ((USE_SUBTITLE != 0) ? sub_ram_wdata : 48'd0);
assign sub_valid_raw_m = (USE_SUBTITLE != 0) ? sub_valid_reg : 1'b0;

// ============================================================================
// 字幕有效门控：sub_valid_sw = sub_valid_raw_m & sw1_s2
// ----------------------------------------------------------------------------
//   USE_SUBTITLE 恒为 0，整个表达式在综合期折成常量 0 —— 字幕链路是死逻辑。
//   门控并入这条已有 wire（而不是新写旁路）的原因：
//     sub_overlay 的 rgb/vs/de 三条路延迟完全相同，SW1 切换只改变"选哪一支"，
//     画面不会整体偏移 1 拍（差 1 拍会让整幅画面错位，是本项目踩过的坑）。
//   SW1=0 时掩膜仍然装载（清零 FSM 照跑、RAM 照写），这样启用字幕时"一拨即可见"，
//   而且装载器始终是活逻辑，不会被 TD 当成无用逻辑剪掉。
//   sw1 已在 100MHz 域同步，与 sub_valid_reg 同域相与，不跨域拼逻辑。
// ============================================================================
assign sub_valid_sw = (USE_SUBTITLE != 0) ? (sub_valid_raw_m & sw1_s2) : 1'b0;

// ---- sub_valid 跨到视频域（慢速电平，两级同步；不涉及时序对齐）----
reg sub_val_s1, sub_val_s2;

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        sub_val_s1 <= 1'b0;
        sub_val_s2 <= 1'b0;
    end else begin
        // S3fix2：这里原来是 sub_valid_raw —— 那根 wire 只声明、从未赋值
        //   悬空 net = 恒 0 -> sub_valid 恒 0 -> 叠加层永远走透传 ->
        //   上板表现：页4 墨迹恒 0000 / 字幕不出现 / 画面干净；
        //   而页2=0040（4 条配对、错误码 0）、页3=0800（2048 列）全都正常。
        // 真 TD 的独立佐证：sub_valid 是常量 0 -> ov 支被折掉 -> 掩膜 RAM 成了
        // 死逻辑被剪掉（#bram 31 -> 20、层级表里 u_sub_mask_ram 整行消失）
        //   真正算出来的门控是 sub_valid_raw_m（见上面那条 assign）。
        // S4：这里换成 sub_valid_sw（= sub_valid_raw_m AND SW1）
        sub_val_s1 <= sub_valid_sw;
        sub_val_s2 <= sub_val_s1;
    end
end

// ============================================================================
// S4：滚动字幕的全部新逻辑（都在 video_clk = 25MHz 域）
// ----------------------------------------------------------------------------
//   偏移驱动源：● 帧边界 ●（sub_scroll 内部的 vs_edge，模板 = video_transition.v
//   L82 `wire vs_edge = (I_vs != vs_d);` + L93-94 `if (vs_edge) ... <= 0;`）
// 绝不用"像素时钟分频"：那样偏移会在一帧之内变多次-> 字幕撕裂；
//     而且速度与帧率解耦，换个分辨率就跑偏。
//
//   音频同步：● wav_player.o_playing ●（clk_audio 就是 video_clk，零跨域）
//     起播（o_playing 上升沿）-> 偏移归零 + 本帧就开始滚
//     没在播         -> 保持当前位置不动（不会跳、不会消失）
//     播完（o_playing 掉）-> 停住（不是消失），下次起播再归零
//
//   起止行为：
//     起点 = 0（= 显示掩膜最左 640 列，与 S3 完全一致 -> 不破坏已上板验证的行为）
//     终点 = 2048-640 = 1408（饱和，停在末尾；SUB_LOOP=1 则回绕到 0 重新滚）
// 内容比屏幕宽 3 倍以上（01.SUB 墨迹列 172..1876，窗口只有 640），
//       所以"从 0 向 1408 走"才是顺着文字读的方向（左进右出？不是 ——
//       是窗口从左往右平移，字从右边进来）。等价于"字幕从右向左滚动"。
//
//   字幕比音频长 / 音频比字幕长：
//     音频先讲完 -> 停在末尾那一屏不动（画面静止，不会消失）
//     字幕先滚完 -> 停在末尾那一屏不动（等音频讲完，然后换图）
// 速度是固定的（SUB_PX_PER_FRAME），所以"讲完"与"滚完"不一定同拍。
//       想要"正好讲完就滚完"要按"音频时长/字幕宽度"自适应，那需要视频域做
//       除法/乘法（本工程没有现成除法器），S4 不做 —— 见报告"取舍"一节。
//
//   图片提交 / 装载完成的归零（两条都是"视频域的慢速电平取上升沿"）：
//     sub_busy_v2 上升沿 = 装载器刚起来（清完 RAM）-> 把偏移归零
//       （S3fix3 保证此时 RAM 里已是干净空白，不会漏出上一张的掩膜）
//     sub_playing 上升沿 = 音频起播 -> 再归零一次（保证从第一屏开始讲）
// 为什么不用新增端口把 img_commit 引过来 ：sd_card_bmp 已经有 o_sub_busy
//     这一根现成的输出（S3fix 加的），复用它就够，不新增任何 IO。
// ============================================================================
reg  sub_busy_v1, sub_busy_v2;      // 装载忙 -> video 域（2 级同步）
reg  sub_play_v1, sub_play_v2;      // o_playing -> 本域已经是 25MHz，仍锁 1 拍去毛刺
wire sub_busy_rise = sub_busy_v2 & ~sub_busy_v1;
wire sub_play_rise = sub_play_v2 & ~sub_play_v1;

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        sub_busy_v1 <= 1'b0;
        sub_busy_v2 <= 1'b0;
        sub_play_v1 <= 1'b0;
        sub_play_v2 <= 1'b0;
    end else begin
        sub_busy_v1 <= sub_busy;
        sub_busy_v2 <= sub_busy_v1;
        sub_play_v1 <= sub_playing;
        sub_play_v2 <= sub_play_v1;
    end
end

assign sub_img_req = sub_busy_rise;      // 归零触发（装载起步）

generate
if (USE_SUBTITLE != 0) begin
    sub_scroll #(
        .WIDTH        (12'd640),          // 屏幕一行有效像素数
        .MASK_W       (12'd2048),         // 掩膜总列数
        .ROWS         (12'd48),
        .BAND_Y0      (12'd416)
    ) u_sub_scroll (
        .I_clk        (video_clk),
        .I_rst        (rst_all),
        .I_vs         (vs_tr),            // 帧域驱动源（与叠加层同一个 vs）
        .I_playing    (sub_playing),      // wav_player.o_playing，零跨域
        .I_img_commit (sub_img_req),      // 新图装载起步 -> 偏移归零
        .I_px_per_frm (SUB_PX_PER_FRAME),
        .I_loop_en    (SUB_LOOP),
        .O_x_off      (sub_scroll_x)      // 只有一根输出，没有悬空端口
    );
end
endgenerate

// 一键回退：USE_SUBTITLE=0 时偏移折成常量 0（与 S3 逐位相同）
wire [11:0] sub_scroll_x_m = (USE_SUBTITLE != 0) ? sub_scroll_x : 12'd0;

assign sub_valid = sub_val_s2;

// ===================== S3：读侧的常驻消费者 = 字幕叠加层 =====================
// S2 的"自由扫描 + popcount"消费者已经删掉
//     原因：S2 那个消费者整块扫描、只为喂诊断数码管，DIAG_MODE=0 时没人用，
//     TD 有可能把 RAM 静默剪掉。S3 起这个叠加层就是真正的、常驻的消费者：
//     它每个有效像素都读掩膜、并按掩膜选色 —— 与 DIAG_MODE 完全无关。
// 地址不再是扫描计数器，而是叠加层给的"本像素的列号"（S3 恒 = x）
//     O_ink（本帧被点亮的字幕像素数）依旧喂诊断页 4，所以诊断页还能看。
// USE_SUBTITLE=0 时叠加层整个不例化（见下面的 generate）

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

//=============================================================================
// 位 5 / 位 4 的"轮播周期 & 锁"两位显示（本次修订）
//
//   为什么改成两位：用户要的四档里"慢观看 10S"需要两位数，
//   一位数码管放不下（原来用 '2' 当标记，操作者根本看不懂）。
//   现在是位 5 = 十位，位 4 = 个位相邻两位，读数直观：
//       讲解模式   ->  " A"       （左边空白，档位用字母 A 表示，区别于数字 0）
//       快速浏览3S ->  " 3"
// 慢观看10S -> "10" 两位一起读
//       手动浏览   ->  " P"       （P = 手动/pause，不再自动切图）
//   位 5 原来显示 SD 状态码（调试信息），本次让位给档位十位；
//   要看状态码仍可用 DIAG_MODE=1 的诊断页。
//
//   SW4=0（总开关关闭）-> 两位一起显示 "Loc"：
//       位 4 = 'L'、位 1 = 'C'、位 5/3/2/0 全灭。
//
// 为什么这里要读 key_audio_gate
//     它是 sd_card_bmp 输出的真实音频闸门，顶层给它一个真实消费者，
//     避免 TD 把产生它的逻辑当死逻辑剪掉（本项目踩过两次：wav_player 被剪成
//     13 LUT、掩膜 RAM 被整块剪掉，日志里 0 警告）。
//     判档位用 key_period_is_explain（不是 key_audio_gate）：
//       闸门在"刚切回讲解档、还没换图"时是 0，若拿它判档会错显示成 3。
//=============================================================================
localparam [6:0] SEG_BLANK = 7'b111_1111;   // 熄灭（段码高有效=灭）
localparam [6:0] SEG_0     = 7'b100_0000;   // '0'
localparam [6:0] SEG_1     = 7'b111_1001;   // '1'
localparam [6:0] SEG_2     = 7'b001_0010;   // '2'
localparam [6:0] SEG_3     = 7'b011_0000;   // '3'
localparam [6:0] SEG_A     = 7'b000_1000;   // 'A'（讲解档）
localparam [6:0] SEG_P     = 7'b000_1100;   // 'P'（手动浏览）
localparam [6:0] SEG_L     = 7'b100_0111;   // 'L'（Loc 的首字母）
localparam [6:0] SEG_C     = 7'b100_0110;   // 'C'（Loc 的末字母）

// 设置模式（SW1=1）下把菜单光标显示到数码管位 1
//   为什么加这个：OSD 菜单里的光标高亮只有 HDMI 上能看到，一旦现场觉得
//   "KEY_2 选择没反应"，就无法判断是按键没被采到、还是光标动了但看不清。
//   现在设置模式下：位 5/位 4 = 已确认的档位（A / 3 / 10 / P），
//                   位 1     = 光标档位（0 / 1 / 2 / 3）。
//   ⇒ 按 KEY_2 位 1 必须跳，按 KEY_3 位 5/4 必须变成光标那一档。
//     两者都能直接读出来，是这套按键交互的现场自检点。
reg  [6:0] seg_cursor_1;
always @(*) begin
    case (sd_menu_sel)
    2'd0:    seg_cursor_1 = SEG_0;
    2'd1:    seg_cursor_1 = SEG_1;
    2'd2:    seg_cursor_1 = SEG_2;
    default: seg_cursor_1 = SEG_3;
    endcase
end

always @(*) begin
    if (!sw4_sync) begin
        // 总开关关闭：L / C（"Loc"，与用户约定的"锁"一致）
        seg_period      = SEG_L;
        seg_period_tens = SEG_BLANK;
        // 位 1：锁定模式显示 C
        // key_audio_gate 参与在条件里 ：锁定必须确实把音频闸门关掉了
        //     才显示 C（闸门 = 讲解档 && 未锁 && 本张图已重启过音频）。
        //     锁定时它必然已落 0，所以正常情况下面这个条件恒成立。
        //     之所以写进条件而不是忽略：给 o_audio_gate 一个真实消费者，
        //     避免 TD 把产生它的逻辑当死逻辑剪掉（本项目踩过两次）。
        seg_lock_1 = !key_audio_gate ? SEG_C : SEG_BLANK;
    end else begin
        // 位 1：正常模式交给原来的 FAT 条数十位（阶段 3a 的显示一字未改）；
        // 设置模式（SW1=1）下改显示菜单光标 0..3 —— 现场自检用
        seg_lock_1 = sw1_s2 ? seg_cursor_1 : fat_tens_seg_m;
        // 位 5 / 位 4：轮播周期两位数
        //   0 = 讲解模式（音频档）   -> " A"
        //   1 = 快速浏览 3S          -> " 3"
        //   2 = 慢观看  10S          -> "10"
        //   3 = 手动浏览             -> " P"
        case (period_sel)
        2'd0:    begin seg_period_tens = SEG_BLANK; seg_period = SEG_A; end
        2'd1:    begin seg_period_tens = SEG_BLANK; seg_period = SEG_3; end
        2'd2:    begin seg_period_tens = SEG_1;     seg_period = SEG_0; end
        default: begin seg_period_tens = SEG_BLANK; seg_period = SEG_P; end
        endcase
        // sd_manual_mode 与 period_sel==3 同源，此处作为冗余一致性读数保留：
        //   若两者不一致（理论上不会），强制熄灭该位以示异常
        if (sd_manual_mode != (period_sel == 2'd3))
            seg_period = SEG_BLANK;
    end
end

// WAV 播放状态码 -> 数码管第 0 位
// bin_data 是 4 位（input[3:0]），不要写成 {3'b000,...} 补零扩展，那样是 7 位会报错
seg_decoder seg_decoder_wav(
    .bin_data          (wav_state[3:0]),
    .seg_data          (wav_seg)
);

// ===================== 阶段 3a：位 1 / 位 2 / 位 3 的 FAT 显示 =====================
//   位 1：条数十位（正常是 0，>=10 个文件时才非 0）
//   位 2：条数个位（sd_card_bmp 里已把 >9 的情况钳到 9，这一位一定是一位十进制数）
//   位 3：错误码    （0 = 无错，十六进制一位）
// 全部用纯组合、不额外占寄存器；也不做任何乘法/移位拼接，
// 避免"上下文宽度"那类静默截断（本项目踩过）。
assign fat_bin_tens  = (fat_count >= 5'd10) ? 4'd1 : 4'd0;
assign fat_bin_units = fat_cnt_digit;
assign fat_bin_err   = fat_err;

seg_decoder seg_decoder_fat_tens(
    .bin_data          (fat_bin_tens),
    .seg_data          (fat_tens_seg)
);

seg_decoder seg_decoder_fat_units(
    .bin_data          (fat_bin_units),
    .seg_data          (fat_units_seg)
);

seg_decoder seg_decoder_fat_err(
    .bin_data          (fat_bin_err),
    .seg_data          (fat_err_seg)
);

// 未完成时把七段码整段抹成"全灭"（bit6:0 = 全 1；位 0/4/5 的 bit7 一直是 1'b1）
assign fat_tens_seg_m  = fat_done ? fat_tens_seg  : 7'b111_1111;
assign fat_units_seg_m = fat_done ? fat_units_seg : 7'b111_1111;
assign fat_err_seg_m   = fat_done ? fat_err_seg   : 7'b111_1111;

// ===================== 临时诊断显示（DIAG_MODE=1 时才生效）=====================
// 默认 DIAG_MODE=1'b0 时，这一段里的所有信号都不影响 seg_out_*，
//     而 DIAG_MODE 是编译期常量参数 -> 综合器把整棵树折叠掉，
// 6 位数码管与改动前逐位完全相同。
//
// 为什么数值不用位3
//   板上实测位3 的段 b/c（右边两条竖线）不亮，字形残缺、极易误判。
//   所以数值只用 位0/位1/位2/位4/位5 共 5 位显示；位3 改作"完成指示灯"，
//   它只做全亮/全灭，看的是亮不亮、不是字形，残缺也不影响判读。
//
//   16 页（页号显示在位0，十六进制 0~F）：
// 页0 : root_lba[15:0] 根目录首扇区 LBA 低 16 位 期望 A000 最关键
//     页1 : root_lba[31:16]    根目录 LBA 高 16 位         期望 0000
// 页2 : S2 {9'd0, sub_found, sub_load_err} 字幕配对条数 + 装载错误码
// 页3 : S2 sub_cols 最近一次字幕装载写进去多少列 期望 0800 关键
// 页4 : S2 ink_total[15:0] 掩膜里墨迹像素总数（整块 RAM 扫描统计）
//     页5 : fat_lba[15:0]      FAT1 起始 LBA 低 16 位      期望 2974
// 页6 : {part_type, root_b0} 分区类型 + 根目录首字节 期望 0C E5 关键
// 页7 : {root_b1, root_b0} 根目录首扇区前两字节指纹 期望 20 E5 关键
// 页8 : rx_bytes 收到的数据字节总数 期望 0C00 关键
// 页9 : secs 成功收到的扇区数 期望 0006 关键
//     页A : fat_err            错误码（4 位）              期望 0000
//     页B : fat_count          解析到的文件条数            期望 0005
//     ---- 以下 4 页是阶段 3b 新增（只有 USE_FAT_LOAD=1 时才有意义）----
// 页C : fat3b_bmp_cnt FAT 表里挑到几张 BMP 期望 0004 关键
// 页D : fat3b_load_err 装载器错误码 期望 0000 关键
// 页E : load_bytes[15:0] 最近一次装载的字节数低 16 位 期望 1036 关键
// 页F : load_bytes[31:16] 最近一次装载的字节数高 16 位 期望 000E 关键
//         （921654 = 0x000E1036，也就是"4 张图每张都完整读到了 921654 字节"）
//
// S2 的改动（显著标出）：页 2 / 3 / 4 被改成字幕掩膜的三页
//     原来这三页是 part_lba[15:0] / part_lba[31:16] / data_lba[15:0]。
//     为什么敢动它们：这三页是阶段 3a"根目录 LBA 算错"那次现场定位用的，
//     而根目录 LBA 的两页（页 0/1）与分区几何的一页（页 6 的 part_type）
//     都还在；现在更需要看的是"字幕数据到底进没进 RAM"。
// 要把旧值找回来：把下面 case 里 4'd2/4'd3/4'd4 三行换回
// dbg_part_lba[15:0] / dbg_part_lba[31:16] / dbg_data_lba[15:0] 即可（一行一处）
//     S2 三页的读法（USE_SUBTITLE=1 时）：
// 页2 = {9'd0, sub_found[2:0], sub_load_err[3:0]} S3fix2 更正位域
//             sub_found 只有 3 位、sub_load_err 4 位 -> 它们在bit6:4 / bit3:0
//             所以 配到 4 条 + 错误码 0 的读数是 0040（不是 0020！）
//       页3 期望 0800 = 2048 列（一条完整的字幕就是 2048 列）
//       页4 = 当前那块掩膜里的墨迹像素总数（每次换图都会变；
//             具体期望值用 tools/fat32_sim/sub_sim 的独立解码器算）
//------------------------------------------------------------------------------
reg  [3:0]  diag_page_cnt;       // 当前页号 0~11
reg  [31:0] diag_hold_cnt;       // 本页已停留拍数
reg  [24:0] diag_blink_cnt;      // 0.5s 闪烁计数（50MHz）
reg         diag_blink;          // 解析未完成时的闪烁标志
reg  [15:0] diag_val;            // 当前页的 16 位数值
wire [3:0]  diag_page_sel;       // 实际显示的页号
wire [6:0]  diag_seg_p;          // 位0：页号
wire [6:0]  diag_seg_1;          // 位1：数值高字节的高 4 位
wire [6:0]  diag_seg_2;          // 位2：数值高字节的低 4 位
wire [6:0]  diag_seg_4;          // 位4：数值低字节的高 4 位
wire [6:0]  diag_seg_5;          // 位5：数值低字节的低 4 位
wire        diag_show;           // 1 = 解析已完成，数值有效
wire        diag_lamp;           // 位3 的亮灭（完成=常亮，未完成=闪烁）
wire [7:0]  seg_out_0;
wire [7:0]  seg_out_1;
wire [7:0]  seg_out_2;
wire [7:0]  seg_out_3;
wire [7:0]  seg_out_4;
wire [7:0]  seg_out_5;

// 页号：自动轮换用计数器，否则用参数固定
assign diag_page_sel = (DIAG_AUTO_CYCLE != 1'b0) ? diag_page_cnt : DIAG_PAGE;
// 只有解析完成之后才显示数值（否则全是 0，会被误读成"真的算出来是 0"）
assign diag_show     = fat_done;
// 位3：完成常亮；未完成 0.5s 闪烁（提示"还在跑，别读"）
assign diag_lamp     = fat_done | diag_blink;

// 当前页的 16 位数值（每个分支都是 16 位，无隐式截断）
always @(*) begin
    case (diag_page_sel)
    4'd0:    diag_val = dbg_root_lba[15:0];
    4'd1:    diag_val = dbg_root_lba[31:16];
    4'd2:    diag_val = {9'd0, sub_found, sub_load_err};
    4'd3:    diag_val = {4'd0, sub_cols};
    4'd4:    diag_val = sub_ink_total[15:0];
    4'd5:    diag_val = dbg_fat_lba[15:0];
    4'd6:    diag_val = {dbg_part_type, dbg_root_b0};
    4'd7:    diag_val = {dbg_root_b1, dbg_root_b0};
    4'd8:    diag_val = dbg_rx_bytes;
    4'd9:    diag_val = dbg_secs;
    4'd10:   diag_val = {12'h000, fat_err};
    4'd11:   diag_val = {11'd0, fat_count};
    // ---- 阶段 3b：装载通路 4 页 ----
    4'd12:   diag_val = {12'h000, fat3b_bmp_cnt};
    4'd13:   diag_val = {12'h000, fat3b_load_err};
    4'd14:   diag_val = fat3b_load_bytes[15:0];
    4'd15:   diag_val = fat3b_load_bytes[31:16];
    default: diag_val = 16'h0000;
    endcase
end

// 5 位十六进制译码（位0=页号，位1/2=高字节，位4/5=低字节；位3 不参与）
seg_decoder u_diag_seg_p ( .bin_data (diag_page_sel),   .seg_data (diag_seg_p) );
seg_decoder u_diag_seg_1 ( .bin_data (diag_val[15:12]), .seg_data (diag_seg_1) );
seg_decoder u_diag_seg_2 ( .bin_data (diag_val[11:8]),  .seg_data (diag_seg_2) );
seg_decoder u_diag_seg_4 ( .bin_data (diag_val[7:4]),   .seg_data (diag_seg_4) );
seg_decoder u_diag_seg_5 ( .bin_data (diag_val[3:0]),   .seg_data (diag_seg_5) );

// 6 位的最终选择：DIAG_MODE=1'b0 时右边那一支就是原来的式子，一个字没改
// 本次新增：锁定模式（SW4=0）时位 0/2/3/5 一律熄灭，只留位 1='C' 位 4='L'
//     -> 数码管显示 "锁"（Loc）。DIAG_MODE=1 时不受影响（诊断优先）。
// 实现方式：把右边那一支整体套一层三目，DIAG_MODE=0 且 SW4=1 时
//     表达式与改动前逐字一致（多一层常量选择，综合后不增加逻辑）。
assign seg_out_0 = DIAG_MODE ? (diag_show ? {1'b1, diag_seg_p} : 8'hFF)
                             : (sw4_sync ? {1'b1, wav_seg} : 8'hFF);
assign seg_out_1 = DIAG_MODE ? (diag_show ? {1'b1, diag_seg_1} : 8'hFF)
                             : {1'b1, seg_lock_1};
assign seg_out_2 = DIAG_MODE ? (diag_show ? {1'b1, diag_seg_2} : 8'hFF)
                             : (sw4_sync ? {1'b1, fat_units_seg_m} : 8'hFF);
// 位3：诊断模式下只做"亮/灭"，全段点亮时因 b/c 坏会看起来像个"6"
assign seg_out_3 = DIAG_MODE ? {1'b1, (diag_lamp ? 7'b000_0000 : 7'b111_1111)}
                             : (sw4_sync ? {1'b1, fat_err_seg_m} : 8'hFF);
assign seg_out_4 = DIAG_MODE ? (diag_show ? {1'b1, diag_seg_4} : 8'hFF)
                             : {1'b1, seg_period};
assign seg_out_5 = DIAG_MODE ? (diag_show ? {1'b1, diag_seg_5} : 8'hFF)
                             : (sw4_sync ? {1'b1, seg_period_tens} : 8'hFF);

// 诊断计数器（本块是这几个 reg 的唯一驱动）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        diag_page_cnt  <= 4'd0;
        diag_hold_cnt  <= 32'd0;
        diag_blink_cnt <= 25'd0;
        diag_blink     <= 1'b0;
    end else begin
        // 0.5s 闪烁：50MHz x 0.5s = 25_000_000 拍
        if (diag_blink_cnt >= 25'd25_000_000) begin
            diag_blink_cnt <= 25'd0;
            diag_blink     <= ~diag_blink;
        end else begin
            diag_blink_cnt <= diag_blink_cnt + 25'd1;
        end

        // 解析完成之前不翻页、也不累计停留时间（完成后一定从第 0 页开始）
        if (!fat_done) begin
            diag_hold_cnt <= 32'd0;
            diag_page_cnt <= 4'd0;
        end else if (diag_hold_cnt >= DIAG_HOLD_CYC) begin
            diag_hold_cnt <= 32'd0;
            // 阶段 3b：页数由 12 页扩到 16 页（0~F），所以回绕值改成 15
            //   页 C~F 是 3b 的装载诊断（bmp_cnt / load_err / load_bytes）
            diag_page_cnt <= (diag_page_cnt == 4'd15) ? 4'd0 : (diag_page_cnt + 4'd1);
        end else begin
            diag_hold_cnt <= diag_hold_cnt + 32'd1;
        end
    end
end

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        (seg_out_0),   // WAV 播放状态码 / 诊断页号
    .seg_data_1        (seg_out_1),   // 3a：FAT 条数十位 / 诊断高字节高位
    .seg_data_2        (seg_out_2),   // 3a：FAT 条数个位 / 诊断高字节低位
    .seg_data_3        (seg_out_3),   // 3a：FAT err_code / 诊断完成指示灯
    .seg_data_4        (seg_out_4),   // 轮播周期档位 / 诊断低字节高位
    .seg_data_5        (seg_out_5)    // TF 卡状态码 / 诊断低字节低位
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data = display_valid ? vout_data_raw : 24'd0;

// ===================== 亮度 / 对比度 =====================
// 1 拍流水延迟；vs/de 由模块内部同步延迟同样的拍数，
// 否则下游 video_rgb_to_axis 的时序会和像素错位。
video_enhance u_video_enhance (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_rgb      (vout_data),
    .I_vs       (vs),
    .I_de       (de),
    .I_gain     (venh_brt),         // 已跨域到 video_clk 域（Q7，128 = 1.0x）
    .I_offb     (venh_off),         // 偏移（偏置 128）
    .I_knee     (venh_knee),        // 高光肩起点（输出域）
    .O_rgb      (vout_enh),
    .O_vs       (vs_enh),
    .O_de       (de_enh)
);

// ===================== 转场（上下推移 / 页面卷曲）=====================
// 接在亮度之后、OSD 之前。SW2 选择模式：0 = 上下推移，1 = 页面卷曲。
// 自动轮播按 SW2 走；手动按 KEY_2 翻页固定用推移（无视 SW2）；
// 第一张图没有旧图可比，直接切换（skip）。
//
// ---- WPS 上下推移：帧读分段 ----
//   video_transition 只导出几何（push_on_v / push_split_px），
//   真正的"新旧图拼接"发生在 frame_fifo_read 的地址跳变里。
// 这两个信号在 video_transition 内已按 vs 边沿量化，帧内不变
wire        push_on_v;                       // video 域，推送转场进行中（含 ST_WAIT）
wire [18:0] push_split_px;                   // prog*1280，旧图起源偏移（0..307200）
wire [1:0]  disp_idx_lat_v;                  // 转场首帧 vs 下降沿锁存的 disp_buf_idx

//------------------------------------------------------------------------------
// 页面卷曲video_transition 导出的卷曲几何（见 video_transition.v O_curl_*）
//   curl_on_v       : 卷曲转场进行中（含 ST_WAIT），读通路要按 split 分段
//   curl_split_px_v : split*640 = 段1（旧图，位于画面上部）的像素数
//   curl_prog_v     : 当前帧的档位 0..69（每帧一档），送 video_curl 查几何表
//------------------------------------------------------------------------------
wire        curl_on_v;
wire [18:0] curl_split_px_v;
wire [6:0]  curl_prog_v;
wire [19:0] curl_s8_v;          // 折线 y（8.8 定点，半档插值后）→ video_curl
wire [12:0] curl_rad8_v;        // 卷边半径（半档插值后，CURL14）→ video_curl

// push 与 curl 互斥（同一个状态机），合并成"任一转场进行中"给读通路用
wire        trans_any_v = push_on_v | curl_on_v;

// 两个帧缓冲的绝对基址（乒乓：disp 与 write 必不同）
wire [23:0] rd_disp_base_live = disp_buf_idx[0] ? BUF1_ADDR : BUF0_ADDR;
wire [23:0] rd_new_base_live  = disp_buf_idx[0] ? BUF0_ADDR : BUF1_ADDR;  // 乒乓必为 !disp

// ---------------------------------------------------------------------------
// 推送期间必须用锁存的缓冲索引，不能用实时 disp_buf_idx
// ---------------------------------------------------------------------------
//   sd_card_bmp 在 ST_WAIT 的 i_trans_go 那一拍就把 disp_buf_idx 提交成新缓冲了
//   （sd_card_bmp.v:1006-1011），而 push_on_v 要等到下一个 vs_edge 才落 0。
//   如果 rd_new_base 实时取 disp_buf_idx[0] 的反，那么在
//       「提交发生」 → 「下一个 vs_edge」
//   这个窗口里 push_on_v 仍为 1、split_len=0，而 alt_base 已经翻回旧缓冲
//   ⇒ 那一帧整屏回放旧图（窗口 ≈ trans_pend_s2 两级同步 80~120ns；
//      每帧 16.8ms，命中概率约 0.7%/次转场，展馆机约每 2 小时闪一次）。
//   所以：在推送第一帧的 vs_edge（此时尚未提交）把 disp_buf_idx 锁存下来，
//   整个推送期间（push_on_v=1）两个基址全用锁存值；非推送期仍用实时值，
//   与改动前逐拍一致。
// ---------------------------------------------------------------------------
reg  [1:0]  push_disp_lat;

//（顶层不再自己判 vs_edge —— 锁存本体在 video_transition 内部，
//   用它的 vs_edge / push_active，避免两处判定不一致。）

// 锁存动作已移进 video_transition（I_disp_idx / O_disp_idx_lat）
//   原因：那是「转场几何」的一部分，只放在顶层就无法被 tb_push_fsm 覆盖，
//   而父代理要求的正例/负对照 MUT=7 正是针对它。放进 DUT 才能被测。
//   这里只把结果打一拍搬过来用（与 trans_hold 等同一 video 域，无需同步器）。
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)
        push_disp_lat <= 2'd0;
    else
        push_disp_lat <= disp_idx_lat_v;
end

// 转场期间（推送或卷曲）两个基址都用锁存值；非转场期用实时值
// 卷曲同样要锁存：curl 提交后 disp_buf_idx 立刻翻新，而 curl_on_v 要到
// 下一个 vs 下降沿才落 0，这个窗口里若用实时值会闪旧图（理由见推送）
wire [23:0] rd_disp_base = trans_any_v
                         ? (push_disp_lat[0] ? BUF1_ADDR : BUF0_ADDR)
                         : rd_disp_base_live;
wire [23:0] rd_new_base  = trans_any_v
                         ? (push_disp_lat[0] ? BUF0_ADDR : BUF1_ADDR)
                         : rd_new_base_live;

// 第一段（旧图）的绝对起始地址
//   推送：旧图从第 S 行开始显示，段1 从"旧图基址 + S*640"读
//   卷曲：段1 就是旧图的最上面 split 行，从旧图基址原样读，不加偏移
wire [23:0] rd_push_base = push_on_v
                         ? (rd_disp_base + {5'd0, push_split_px})
                         : rd_disp_base;
// 第一段长度；非转场时为 0（见下面 rd_alt_base 的注释）
//   推送：段1 = 旧图的下半部分 (480-S)*640
//   卷曲：段1 = 旧图的上半部分 split*640
//   注意：本信号必须是 256 像素的整数倍
//     frame_fifo_read 的换基址判据是 `read_cnt == read_split_len_latch`，而
//     read_cnt 只在每个突发结束时 += BURST_SIZE —— 这里的 BURST_SIZE 是
//     frame_read_write 的默认值 256（顶层没有覆盖），不是 frame_fifo_read
//     自己默认的 128。长度对不上 256 就永远等不到相等，地址一次都不跳，
//     整帧都从段1基址读 = 卷轴下方整片旧图（CURL16 及以前就栽在这里）。
// 推送：prog*1280，1280 = 5*256 天然对齐。
//     卷曲：split*640，640 = 2.5*256 ⇒ split 必须为偶数，已在
//           video_transition.v 的 csplit 处用 SPLIT_EVEN 掩码强制。
//     改动任一侧（BURST_SIZE / 行宽 / 接缝来源）都要回来复核这一条。
wire [23:0] rd_split_len = push_on_v
                         ? (FRAME_PIXELS - {5'd0, push_split_px})
                         : (curl_on_v ? {5'd0, curl_split_px_v} : 24'd0);

// 非转场时 alt_base 必须回落到当前显示基址
//   frame_fifo_read 在 split_len==0 时不走 read_addr_index 选择器，
//   而是直接装载 read_alt_base（这样推送最后一档 S=480 也能整帧读新图）。
//   所以普通帧（不分段）必须靠本信号把自己的 disp 基址送进去。
//
// 注意：两种转场的段2 偏移完全不同，不能套用同一套算法（下面分别推导）
//
//   读流的寻址规则（frame_fifo_read）：
//       i <  split_len  ->  read_addr[read_addr_index] + i
//       i >= split_len  ->  read_alt_base + (i - split_len)
//
//   推送（S 行已顶入，split_len = (480-S)*640）：
//       屏幕第 y 行  = 旧图第 (y+S) 行   (y+S < 480)
// ⇒ 段1 从旧图第 S 行起（read_addr_2 = disp_base + S*640）
//       屏幕第 y 行  = 新图第 (y+S-480) 行 (y+S >= 480)
//       ⇒ 段2 的 i-split_len 正好等于"新图行号*640+列"，所以
// alt_base = new_base 不加偏移
//
//   卷曲（split 行已换新，split_len = split*640）：
// 屏幕第 y 行 = 旧图第 y 行 (y < split) ⇒ 段1 从旧图第 0 行起
//       屏幕第 y 行  = 新图第 y 行   (y >= split)
//       ⇒ 段2 的 i-split_len = (y-split)*640 + x，而目标是 new_base + y*640 + x
//         所以 alt_base 必须 = new_base + split*640
// 漏掉这个偏移的后果：底段显示的是新图从第 0 行起的内容，
//         于是新页画面跟着卷带一起往上滑（而不是原地不动），
//         上板表现为"卷轴往上卷，但底下那张图也在跟着走"。
wire [23:0] curl_alt_off = {5'd0, curl_split_px_v};
wire [23:0] rd_alt_base  = trans_any_v
                         ? (curl_on_v ? (rd_new_base + curl_alt_off) : rd_new_base)
                         : rd_disp_base;

video_transition u_video_transition (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_rgb      (vout_enh),
    .I_vs       (vs_enh),
    .I_de       (de_enh),
    .O_rgb      (vout_tr),
    .O_vs       (vs_tr),
    .O_de       (de_tr),
    .I_mode     (sw2_tr_mode),          // SW2：转场模式（手动翻页时强制推移）
    .I_pending  (trans_pend_s2),
    .I_skip     (trans_skip_s2),
    .I_disp_valid (dispval_s2),         // 第一张图不做转场
    .O_hold     (trans_hold),
    .O_busy     (trans_busy),
    .O_go       (trans_go),
    .O_push_on      (push_on_v),        // 推送转场进行中（含 ST_WAIT，按帧量化）
    .O_split_pixels (push_split_px),    // 旧图起源偏移 prog*1280（按帧量化）
    .O_curl_on      (curl_on_v),        // 卷曲转场进行中（含 ST_WAIT，按帧量化）
    .O_curl_split_px(curl_split_px_v),  // 卷曲段1像素数 split*640（按帧量化）
    .O_curl_prog    (curl_prog_v),      // 卷曲档位 0..69（每帧一档）
    .O_curl_s8      (curl_s8_v),        // 折线 y（8.8 定点，半档插值，抗闪屏）
    .O_curl_rad8    (curl_rad8_v),      // 卷边半径（半档插值，CURL14）
    .I_disp_idx     (disp_buf_idx),     // 实时乒乓索引（锁存本体在 DUT 内）
    .O_disp_idx_lat (disp_idx_lat_v)    // 转场首帧 vs 下降沿锁存值
);

// ===================== 页面卷曲着色层（S5）=====================
// 位置：video_transition 之后、字幕层之前
//     video_transition 只算几何（不碰像素），真正的内容合成在这里做。
// 本层是纯延迟链 + 逐列几何 ROM，总延迟 10 拍，
// O_vs/O_de 由内部延迟链产生，与 O_rgb 严格同拍 ⇒ 外部不要再打拍
// I_active=0 时是纯透传（逐位相等），不引入任何着色
//------------------------------------------------------------------------------
wire [23:0] vout_curl;
wire        vs_curl, de_curl;

video_curl #(
    .WIDTH  (12'd640),
    .HEIGHT (12'd480)
) u_video_curl (
    .I_clk    (video_clk),
    .I_rst    (rst_all),
    .I_rgb    (vout_tr),
    .I_vs     (vs_tr),
    .I_de     (de_tr),
    .I_active (curl_on_v),
    .I_prog   (curl_prog_v),
    .I_s8     (curl_s8_v),          // 折线位置（半档插值，见 video_transition）
    .I_rad8   (curl_rad8_v),        // 卷边半径（半档插值，CURL14）
    .O_rgb    (vout_curl),
    .O_vs     (vs_curl),
    .O_de     (de_curl)
);

// ===================== S3：字幕叠加（常驻消费者）=====================
// 位置：video_transition 与 osd_overlay 之间
//     所以 OSD 面板天然盖住字幕（osd_overlay.v 的优先级链一个字没改）。
// 本层只加 1 级流水，O_vs/O_de 与 O_rgb 同拍
// 一键回退：USE_SUBTITLE=0 时这里整个不例化、输出折成原图常量
wire [23:0] vout_sub;            // 字幕叠加后的 RGB
wire        vs_sub, de_sub;      // 同步延迟后的 vs/de

generate
if (USE_SUBTITLE != 0) begin
    sub_overlay #(
        .WIDTH          (12'd640),
        .BAND_Y0        (12'd416),
        .ROWS           (12'd48),
        .DIM_SHIFT      (0),              // S3：不加底框（0 = 逐通道右移 0 位）
        .C_SUB          (24'hFFFFFF),     // S3：白字
        .SUB_VALID_GATE (1)               // S3fix：启用掩码有效门控
    ) u_sub_overlay (
        .I_clk       (video_clk),
        .I_rst       (rst_all),
        .I_rgb       (vout_tr),
        .I_vs        (vs_tr),
        .I_de        (de_tr),
        .O_rgb       (vout_sub),
        .O_vs        (vs_sub),
        .O_de        (de_sub),
        .O_mask_addr (sub_mask_addr),
        .I_mask_data (sub_rdata_m),
        .I_x_off     (sub_scroll_x_m),   // S4：接滚动偏移（USE_SUBTITLE=0 时折成 0）
        .I_sub_valid (sub_valid),    // S3fix：0 = 完全透传（与叠加路径同延迟）
        .O_ink       (sub_ink_cnt)
    );
end
endgenerate

// 回退时（USE_SUBTITLE=0）叠加层输入输出全部折到原图那一支：
//   单条 assign + 编译期常量选择，不写两个 generate 分支（避免 MD-ASSIGN 误判）
wire [23:0] vout_sub_m = (USE_SUBTITLE != 0) ? vout_sub : vout_curl;
wire        vs_sub_m   = (USE_SUBTITLE != 0) ? vs_sub   : vs_curl;
wire        de_sub_m   = (USE_SUBTITLE != 0) ? de_sub   : de_curl;
assign      sub_ink_total = {16'd0, sub_ink_cnt};

// ===================== 音频可视化（c64）=====================
//   位置：banner_scroll 之后、osd_overlay 之前
//     -> 参数调节条与设置菜单天然盖在它上面，既有优先级链不用改
//     -> 与滚动字幕是"二选一"：可视化解占底部条带时字幕让位
//   开关：KEY_4 短按（仅讲解档）；手动档常显；3s/10s 档不加
//   这些 wire 必须在使用它的 banner 门控之前声明（本工程踩过隐式 net 的坑）
// c67：可视化层已删除，viz_show_v 现在只表示"字随音频跳动使能"
wire [6:0]  audio_level_v;      // 音频包络 v[2]+v[3]，0..118（7 位，6 位会回绕）
wire        viz_show_v;
wire        viz_show_100m;
wire        viz_key_on_100m;
// 提前声明：可视化层要用它，而它的驱动在文件后段"音源选择"那里。
// 不提前声明会在使用点生成一个隐式 1bit net，与后面的实体不是同一根线
//   -> 综合报 HDL-5314 "does not have a driver"（实测踩过一次）
wire        audio_valid_sel;

// ===================== 装载进度跨域（c72）=====================
//   100MHz（sd_card_clk）-> 25MHz（video_clk）
//   进度是 8 位、单调递增，用 gray 码传：每拍只可能变 1 位，
//   2 级同步后采样不会采到"多位同时翻转"的中间值（否则进度条会跳）。
//   busy 是准静态电平，同一套同步。
reg  [7:0]  prog_g1, prog_g2;
reg         ldbusy_s1, ldbusy_s2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        prog_g1   <= 8'd0;
        prog_g2   <= 8'd0;
        ldbusy_s1 <= 1'b0;
        ldbusy_s2 <= 1'b0;
    end else begin
        prog_g1   <= bmp_prog_gray;
        prog_g2   <= prog_g1;
        ldbusy_s1 <= bmp_load_busy;
        ldbusy_s2 <= ldbusy_s1;
    end
end

// gray -> 二进制（异或链，8 级，约 7 LUT）
wire [7:0] prog_bin;
assign prog_bin[7] = prog_g2[7];
assign prog_bin[6] = prog_bin[7] ^ prog_g2[6];
assign prog_bin[5] = prog_bin[6] ^ prog_g2[5];
assign prog_bin[4] = prog_bin[5] ^ prog_g2[4];
assign prog_bin[3] = prog_bin[4] ^ prog_g2[3];
assign prog_bin[2] = prog_bin[3] ^ prog_g2[2];
assign prog_bin[1] = prog_bin[2] ^ prog_g2[1];
assign prog_bin[0] = prog_bin[1] ^ prog_g2[0];

// ===================== 滚动标语层（banner_scroll）=====================
//   位置：video_curl 之后、osd_overlay 之前
//     -> 亮度条 / 设置菜单 / 提示条 都叠在它上面，天然覆盖，无需额外仲裁
//
//   显示条件：已装载图片（dispval_s2）且亮度条未占用
//     四个档位（讲解 / 3s / 10s / 手动）都显示。
//     亮度条面板 y380..444 与字幕条带 y416..472 重叠 28 行（416~444），
//     同时出现会形成"被切开的字"，所以亮度条显示期间整体隐藏字幕。
//
//   滚动行为：
//     文字周期 = 屏宽 640 px（320 px 文字 + 320 px 空白；c67 起字模 2× 放大，
//     文字块从 160 变成 320），每帧前进 2 px，即文字 2.7 s 扫过、空白 2.7 s，
//     屏幕任何时刻只有一段文字。
//     ★ c72 起换图**会**重置偏移（装载结束时由 banner 内部的 busy_fall 清零）。
//       旧版"换图不重置"的前提是单一文案；改成每图文案之后，不重置会让观众看到
//       "半句话突然跳到另一句的对应位置"，比重置难看得多。
//     转场期间继续滚动：本层在 video_curl 之后，转场只改像素、不改 vs/de 时序。
//
//   定格条件（是"定格"不是"隐藏"）：
//     设置菜单打开（sw1_v2）/ 提示条显示中（banner_show_w）
//
//   字模来源：本层优先级最低，由顶层共享字库统一提供（见下方字库段）。
//------------------------------------------------------------------------------
wire [23:0] vout_banner;
wire        vs_banner, de_banner;

// 提示条显示中（由 osd_menu 输出）
wire banner_show_w;

// 滚动字幕的字库请求（关闭时恒 0，见下方 generate 的 else 分支）
wire [5:0]  banner_fcode;
wire [3:0]  banner_frow;
wire        banner_freq;

//=============================================================================
// 以下三个 wire 必须声明在 generate之前
//
//   若声明在 generate 之后，generate 作用域内引用时会创建新的隐式 net，
//   与外层同名 wire 不是同一根线 —— 表现为 HDL-5314（net has no driver），
//   而综合照样通过、照样出 bit，只有上板才会发现字模恒 0（不显示文字）。
//   逐像素仿真也抓不到：测试台自己例化字库并直连被测模块，走不到这条路径。
//=============================================================================
wire [15:0] shared_fbits;          // 共享字库返回的点阵行
wire        banner_font_grant;     // 字库分给滚动字幕
wire        ovl_show_v;            // 亮度条显示中



generate
if (USE_BANNER != 0) begin : g_banner
    // 显示门控：已装载图片 且 亮度条未占用 且 音频可视化未占用
    //   四个档位都显示；亮度条面板与字幕条带重叠 20 行，
    //   同时出现会形成"被切开的字"，故亮度条显示期间整体隐藏字幕。
    //   ★ v3.1（2026-10-27）：可视化占用条带时必须**整层隐藏**（I_on=0），
    //     不能只停滚动（I_run=0）。原来 gate 只加在 banner_run_v 上，
    //     I_on 仍为 1 -> 字幕只是"定格"在原地照样画出来，于是和柱子叠在一起
    //     （现场照片：底部条带里字幕文字与可视化边框、柱子同时可见）。
    // c67：可视化层删除后，字幕不再让位（v3.1 那个"字幕停在原地与柱子重叠"的坑
    //   从根上消失）。viz_show_v 只作为跳动使能进 banner。
    wire banner_on_v = dispval_s2 && !ovl_show_v;

    // 滚动门控：菜单打开或提示条显示时定格；转场期间照常滚动
    //   （转场只改像素坐标、不改 vs/de 时序，本层的行列计数始终有效）
    // banner_rst_v 恒 0：c72 起"换图重置"改由 banner 内部的 busy_fall 完成
    //   （I_load_busy 的下降沿）。这里不再给单拍清零，避免两处同时归零。
    wire banner_run_v = banner_on_v && !sw1_v2 && !banner_show_w;
    wire banner_rst_v = 1'b0;

    banner_scroll u_banner_scroll (
        .I_clk     (video_clk),
        .I_rst     (rst_all),
        .I_rgb     (vout_sub_m),
        .I_vs      (vs_sub_m),
        .I_de      (de_sub_m),
        .O_rgb     (vout_banner),
        .O_vs      (vs_banner),
        .O_de      (de_banner),
        .I_on      (banner_on_v),
        .I_run     (banner_run_v),
        .I_rst_off (banner_rst_v),
        .I_bounce  (viz_show_v),        // KEY_4：跳动开/关
        .I_level   (audio_level_v),     // 音频包络 -> 弹簧
        .I_prog      (prog_bin),        // c72：装载进度 0..255
        .I_load_busy (ldbusy_s2),       // c72：1 = 正在装载（显示进度条）
        // c46 共享字库（本模块优先级最低）
        .O_fcode      (banner_fcode),
        .O_frow       (banner_frow),
        .O_freq       (banner_freq),
        .I_fbits      (shared_fbits),
        .I_font_grant (banner_font_grant)
    );
end else begin : g_nobanner
    // 关闭时整个模块不例化，直接短接。不是"例化了但关显示"
    //   后者照样占 300+ LUT —— USE_SUBTITLE 的注释里有同样说明。
    assign vout_banner = vout_sub_m;
    assign vs_banner   = vs_sub_m;
    assign de_banner   = de_sub_m;
    // 字库请求恒 0：三级 mux 会自然回落到 osd_overlay / 菜单
    assign banner_fcode = 6'd10;      // 空格，永远不会被选中
    assign banner_frow  = 4'd0;
    assign banner_freq  = 1'b0;
end
endgenerate

// ===================== 音频可视化开关（KEY_4 短按）=====================
//   全程 sd_card_clk(100MHz) 域：key_press(=sd_key3_press) / period_sel /
//   sw1_s2 / sw3_sync / sw4_sync 全部同域，本模块内部零跨域。
//   c67：柱状图删除后不再有"二选一"，KEY_4 只决定"字跳不跳"，所有档位统一。
//   门控仍排除手动档 —— 那里的 KEY_4 要留给 sd_card_bmp 做"播/停当前图语音"，
//   而手动档正是靠它播音、播起来字才跟着跳。
key_viz_ctrl #(
    .KEY_ON_INIT   (1'b1)           // c67：上电即跳动（不再是额外图层，默认开）
) u_key_viz_ctrl (
    .clk        (sd_card_clk),
    .rst        (rst_all),
    .key_press  (sd_key3_press),
    .period_sel (period_sel),
    .sw1        (sw1_s2),
    .sw3        (sw3_sync),
    .sw4        (sw4_sync),
    .o_key_on   (viz_key_on_100m),
    .o_show     (viz_show_100m)
);

// ---- 单 bit 电平跨到 video_clk：2 级同步即可（不需要 toggle/格雷码）----
reg viz_s1, viz_s2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        viz_s1 <= 1'b0;
        viz_s2 <= 1'b0;
    end else begin
        viz_s1 <= viz_show_100m;
        viz_s2 <= viz_s1;
    end
end
assign viz_show_v = viz_s2;

// ===================== 音频包络前端（c67）=====================
//   取代 osd_audio_viz（柱状图 + 波形，1,036 le + 1 BRAM）。
//   字随音频跳动只需要"一个响度数字"，不需要频谱图和波形，所以整个渲染侧删除，
//   只保留段 2 / 段 3 的包络（120-480Hz，本工程素材 67% 能量所在）。
//
//   面积：原模块 1,036 le -> 约 180 le；波形 BRAM 一并消失。
//   数值语义与 tools/viz_model.py 的 bands_raw() 逐位一致（不含倾斜补偿）。
osd_audio_env u_osd_audio_env (
    .I_clk     (video_clk),
    .I_rst     (rst_all),
    // 音频：同域只读，不反压、不改 valid 时序
    .I_svalid  (audio_valid_sel),
    // ★ 取"音量之前"的样本（2026-10-30 修正）
    //   原来接 audio_vol_left（音量之后），后果是音量旋钮同时拧动了画面：
    //   音量 20 档约 -24dB（Q7≈8），包络幅度只剩 1/16，log6 的输出 v 直接掉
    //   16 个码值、O_level(=v2+v3) 掉 32，落到 banner_scroll 的 THR=14 门限
    //   附近 —— 上板表现就是"音量 20 以下字幕基本不跳"。
    //   可视化应该只看"有没有声音、有多响"，不该被输出音量缩放：
    //   展台场景恰恰是"喇叭关小、画面仍要有律动"。
    //   换成 audio_raw_left 后：音量 100（Q7=128，0dB）时与原来逐位相同
    //   （乘法器乘 1.0x），只有低音量时才恢复律动幅度。
    //   SW4 音频总开关仍然有效 —— 它门控的是 audio_raw_*，关掉即静音且不跳。
    .I_sample  (audio_raw_left),
    .I_vs      (vs_sub_m),              // 帧捕获触发（像素流场同步）
    .O_level   (audio_level_v),
    .O_v2      (),                      // 调试口，不接
    .O_v3      ()
);

// ===================== 全片共享字库（三级优先级）=====================
//   三个 OSD 图层（设置菜单 / 亮度条 / 滚动标语）原先各例化一份 osd_font_rom，
//   合计 572 LUT 查同一张表。现在顶层只保留一份：各层送出 (code,row)，
//   按优先级 mux 后查表，再把点阵行广播回各层。
//
//   优先级：设置菜单 > 亮度条 > 滚动标语
//   O_freq 只在"本像素真的落在该层文字区"时为 1，绝大多数像素三者皆为 0，
//   所以两级 mux 足够，不需要真正的仲裁器。
//
//   grant 的作用：重叠区里字库只能服务一个图层，必须告诉低优先级图层
//   "这次的数据不是给你的"（grant=0），否则它会拿别人的字形画自己的字 —— 串字。
//   grant=0 时该层只是不画文字，底带与进度条照常。
//------------------------------------------------------------------------------
wire [5:0]  menu_fcode, ovl_fcode;
wire [3:0]  menu_frow,  ovl_frow;
wire        menu_freq,  ovl_freq;

wire [5:0]  rom_fcode = menu_freq ? menu_fcode : (ovl_freq ? ovl_fcode : banner_fcode);
wire [3:0]  rom_frow  = menu_freq ? menu_frow  : (ovl_freq ? ovl_frow  : banner_frow);

osd_font_rom u_osd_font_shared (
    .I_code (rom_fcode),
    .I_row  (rom_frow),
    .O_bits (shared_fbits)
);

// ovl_show_v / shared_fbits / banner_font_grant 已在 generate 之前声明（见上方"必须声明在 generate 之前"）
wire ovl_font_grant = ~menu_freq;                           // 菜单优先
assign banner_font_grant = ~menu_freq && ~ovl_freq;         // 菜单与亮度条都没有才轮到字幕

// ===================== OSD 叠加 =====================
// 接在亮度处理之后、TMDS 输出之前。参数值来自跨域锁存的副本。
// 只在 SW3=1（进入调节模式）时显示，平时完全透传、不改变画面。
osd_overlay u_osd_overlay (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_rgb      (vout_banner),      // c67：可视化层已删除，直接接字幕层
    .I_vs       (vs_banner),
    .I_de       (de_banner),
    .O_rgb      (vout_osd),
    .O_vs       (vs_osd),
    .O_de       (de_osd),
    .I_en       (sw3_v2),           // SW3 同步到视频域
    .I_trigger  (osd_trigger),      // 按键/进入模式时重新计时（自动隐藏）
    .I_sel      (venh_sel),         // 0=亮度 1=对比度 2=音量（已跨域）
    .I_bright   (venh_bright_val),
    .I_contrast (venh_ctr_val),
    .I_volume   (venh_vol_val),
    // c46 共享字库
    .O_fcode      (ovl_fcode),
    .O_frow       (ovl_frow),
    .O_freq       (ovl_freq),
    .I_fbits      (shared_fbits),
    .I_font_grant (ovl_font_grant),
    .O_show       (ovl_show_v)
);

// ===================== 轮播设置菜单 / 停止提示条 =====================
// 串在 osd_overlay 之后、写帧缓存之前。
//   - 设置菜单：SW1=1 时常显（不自动消失），SW1=0 完全透传
//   - 停止提示条：KEY3 停止自动轮播时弹 3 秒后自动消失
// 两个面板都在 25MHz 视频域，与画面严格同拍；模块内部 1 拍延迟，
// 写帧缓存的一侧本来就对 vs/de/rgb 同步延迟，所以不会错位。
osd_menu u_osd_menu (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_rgb      (vout_osd),
    .I_vs       (vs_osd),
    .I_de       (de_osd),
    .O_rgb      (vout_menu),
    .O_vs       (vs_menu),
    .O_de       (de_menu),
    .I_setup    (sw1_v2),            // SW1 同步到视频域
    .I_menu_sel (menu_sel_v2),       // 光标 0..3（跨域已同步）
    .I_cfg_sel  (cfg_sel_v2),        // 已确认档位
    .I_banner   (banner_pulse_v),    // 停止提示单拍脉冲（3 秒）
    .I_banner_auto (abanner_pulse_v),// 自动轮播中短提示单拍脉冲（1 秒）
    .O_banner_show (banner_show_w),  // c41：提示条显示中 -> 滚动字幕定格
    // c46 共享字库
    .O_fcode      (menu_fcode),
    .O_frow       (menu_frow),
    .O_freq       (menu_freq),
    .I_fbits      (shared_fbits)
);

// ===================== SW4 总开关：最终黑屏闸门 =====================
//   SW4=0 -> 像素全 0（vs/de 一律不动，HDMI 时序/ACR 不受影响，显示器不会失锁）
//   放在 OSD/菜单之后，所以 SW4=0 时连设置菜单和停止提示条也一起黑掉——
//   这正是"总开关关掉 = 整机不出任何画面"的语义。
//   注意：这里只压 rgb，不碰 de/vs，否则 video_rgb_to_axis 会丢行/丢帧。
wire [23:0] vout_final = sw4_v2 ? vout_menu : 24'd0;

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    // read_addr_2 在转场期间承载段1的绝对起始地址：
    //   推送 = 旧图基址 + prog*1280（旧图从中段开始显示）
    //   卷曲 = 旧图基址（段1 就是旧图最上面 split 行，不加偏移）
    // 配合下面的 read_addr_index=2 使用；其余时刻该值无意义（不被选中）。
    .read_addr_2       (rd_push_base),
    .read_addr_3       (24'd0),
    // 转场期间强制选 index=2（推送与卷曲都要）
    //   非转场时仍按 disp_buf_idx 选 BUF0/BUF1，与改动前完全一致。
    //   注意 frame_fifo_read 内部在 split_len==0 时优先用 read_alt_base，
    //   所以这里选 index=2 与 split_len=0 两种路径不会打架。
    .read_addr_index   (trans_any_v ? 2'd2 : disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_split_len    (rd_split_len),
    .read_alt_base     (rd_alt_base),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vs_menu),
    .I_de          (de_menu),
    .I_rgb         (vout_final),     // SW4 总开关：SW4=0 时全黑
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== HDMI 音频源 =====================
// 全部在 video_clk(25MHz) 域内，不引入额外音频 PLL，
// 因此 rst_all 保持不变 —— 音频出问题不会连带 video 一起挂住。
//
// 当前音源只有 TF 卡里的 WAV（见下面"音源选择"段）。
// 下面例化的 audio_tone_pcm 是 750Hz 提示音发生器，保留但未接入：
// 输出没有任何消费者，综合时会被当死逻辑剪掉，不占资源。
// 想恢复提示音，把"音源选择"段的三行改回 wav_found ? wav_valid : audio_valid。
audio_tone_pcm #(
    .CLK_HZ      (25_000_000),
    .SAMPLE_RATE (48_000),
    .AMP         (24'd7_000_000),    // 约 -1.6 dBFS，便携屏小喇叭也够响
    .ON_SAMPLES  (24_000),           // 0.5s @48kHz
    .OFF_SAMPLES (24_000)            // 0.5s @48kHz
) u_audio_tone_pcm (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_valid (audio_valid),
    .O_left  (audio_left_data),
    .O_right (audio_right_data)
);

// ===================== 音源选择 =====================
// 现在只放 TF 卡语音：
//   - 未扫描到 WAV 时，wav_player 持续输出 valid+静音，
//     既没有哔声，又能让 ACR 的 CTS 节拍不断（接收端不会失锁）。
//   - 第一张图显示后延迟 1.5 秒才开始出声（延迟在 sd_card_bmp 里，
//     由 display_valid 触发；想改时长见那里的 AUDIO_DELAY_CYC）。
//
// 提示音 audio_tone_pcm 保留但不再接入：
//   综合会把它当死逻辑优化掉，不占资源。
//   若想恢复提示音，把下面三行改回：
//     wav_found ? wav_valid : audio_valid   （left/right 同理）
assign      audio_valid_sel = wav_valid;

// ===================== 音量 =====================
// 全组合、无寄存 —— 绝不能在样本数据和 valid 之间插入流水延迟：
// HDMI 核是按 valid 采样本的，数据晚一拍就会被采到上一个样本，
// 听感上是持续的噪声/变调（这个坑在 wav_player 里已经踩过一次）。
// SW4 总开关：SW4=0 时样本压 0（静音），但 valid不动
//   为什么不动 valid：HDMI 核按 valid 采样本，ACR 的 CTS 节拍靠它维持；
//   把 valid 掐掉会让接收端以为音频流中断，可能引起重同步/爆音。
//   所以"静音"只压数据。sd_card_bmp 内部另有 audio_gate 把播放器本身停掉，
//   两层叠加：SW4=0 -> 播放器不启动 + 数据被压 0，绝无漏音。
//   sw4_v2 与音频链同属 video_clk 域，无需再同步。
assign audio_raw_left  = sw4_v2 ? wav_left  : 24'd0;
assign audio_raw_right = sw4_v2 ? wav_right : 24'd0;

// ---- 音量缓变（防"咔"声）----
// 增益若在音频流中途突变，波形会出现台阶 -> 听感上是"咔"的一下。
// 这里每来一个音频样本才把增益挪 1 级（48kHz 下满量程约 3.8ms），
// 既听不出延迟，也彻底消除突变。
// 增益是无符号量（0..182），直接比大小即可，不需要二补码转换。
reg  [7:0] vol_cur;
wire [8:0] vol_dist = (venh_vol > vol_cur) ? (venh_vol - vol_cur)
                                           : (vol_cur - venh_vol);

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        vol_cur <= 8'd128;          // 与默认音量 100 档对应（0dB）
    end else if (audio_valid_sel) begin
        if (venh_vol > vol_cur)
            vol_cur <= (vol_dist <= 9'd1) ? venh_vol : (vol_cur + 8'd1);
        else if (venh_vol < vol_cur)
            vol_cur <= (vol_dist <= 9'd1) ? venh_vol : (vol_cur - 8'd1);
    end
end

audio_volume u_audio_volume (
    .I_left    (audio_raw_left),
    .I_right   (audio_raw_right),
    .I_gain_q7 (vol_cur),           // 缓变后的增益
    .O_left    (audio_vol_left),
    .O_right   (audio_vol_right)
);

wire [23:0] audio_left_sel  = audio_vol_left;
wire [23:0] audio_right_sel = audio_vol_right;

audio_acr_gen #(
    .ACR_N (20'd6144),
    .DIV   (48)                      // 每 48 个样本(1ms)测一次 CTS
) u_audio_acr_gen (
    .I_clk          (video_clk),
    .I_rst          (rst_all),
    .I_sample_valid (audio_valid_sel),   // 必须用"选中后"的音源：CTS 是数样本数出来的
    .O_acr_valid    (acr_valid),
    .O_cts          (acr_cts),
    .O_n            (acr_n)
);

// ===================== HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    // ↓↓↓ 音频受顶部 AUDIO_EN 开关控制（AUDIO_EN=0 时全部为恒定 0，与原例程一致）
    .I_audio_valid      (AUDIO_EN ? audio_valid_sel      : 1'b0),
    .I_audio_left_data  (AUDIO_EN ? audio_left_sel       : 24'd0),
    .I_audio_right_data (AUDIO_EN ? audio_right_sel      : 24'd0),
    .I_acr_valid        (AUDIO_EN ? acr_valid        : 1'b0),
    .I_acr_cts          (AUDIO_EN ? acr_cts          : 20'd0),
    .I_acr_n            (AUDIO_EN ? acr_n            : 20'd0),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
