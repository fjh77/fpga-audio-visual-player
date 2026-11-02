//=============================================================================
// sd_card_bmp.v —— 存储通路与播放调度的总控（本工程最大的单个模块）
//
// 职责
// ----
//   把"TF 卡上的素材"变成"可显示的帧缓存 + 可播放的音频流"，并决定什么时候换图。
//   它不是一个纯数据通路，而是整机的调度中心：
//     1) SD 卡初始化与扇区级读写（经 sd_arbiter3 仲裁，见下）
//     2) FAT32 根目录解析与素材配对（fat32_boot），得出图片名单与音频名单
//     3) 图片装载：fat32_file 读簇链 -> bmp_load_fat 解码 -> (可选 2:1 缩小) -> SDRAM
//     4) 音频装载：另一路 fat32_file 独立读 WAV 扇区流 -> wav_player
//     5) 轮播状态机：档位计时（讲解 / 3s / 10s / 手动）、按键响应、锁定与提示条
//
// 为什么图片与音频各用一路 fat32_file
// -----------------------------------
//   共用一路会让"读图"和"读音频"互相阻塞：讲解播到一半去读下一张图，
//   音频缓冲就会欠载。两路独立后，音频通路可以持续预取，图片通路慢一点也无妨，
//   这是"音画同步"在架构上的保证（详见《技术文档》1.5 节创新点 1）。
//
// 总线仲裁
// --------
//   两路 FAT 读取 + 空闲时的心跳都要用 SPI，统一由 sd_arbiter3 排优先级，
//   同一时刻只有一个请求方持有总线。仲裁策略见 sd_arbiter3.v 的注释。
//
// 时钟域
// ------
//   clk_sd     —— SD 卡 SPI、FAT 解析、按键去抖、轮播计时（c70 起为 62.5 MHz，
//                 本文件的 CLK_FREQ_HZ 仍按 100 MHz 写，改动前先看 top.v 的时钟段）
//   clk_video  —— 25 MHz 像素域，只用于把"当前显示的是第几张"等信息同步过去
//
// 主要参数
// --------
//   CLK_FREQ_HZ        —— 定时器换算基准，改时钟必须同步改
//   SCAN_START_SECTOR  —— 目录扫描起点（默认 0，即从 MBR 开始）
//   SCAN_MAX_SECTOR    —— 扫描上限，防止坏卡时无限扫描
//   SCAN_TARGET_COUNT  —— 期望找齐的素材组数（默认 4 组，对应 01~04）
//
// 上游/下游
// ---------
//   上游：TF 卡（SPI）、按键、拨码
//   下游：SDRAM 帧缓存（经 frame_read_write）、wav_player、osd 层的状态输入
//=============================================================================

`timescale 1ns / 1ps

module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,

    //=========================================================================
    // 阶段 3a 总开关：FAT32 根目录解析
    //
    //   0 = 默认，旧行为完全不启用 FAT 解析。
    //       fat32_scan 整个模块在综合时被折叠成死逻辑（不占任何资源），
    //       仲裁器只剩 bmp/wav 两路，与改动前逐拍等价。
    //       数码管位 1/2 熄灭（与改动前一致）。
    //   1 = 启用：上电 10 秒后（SD 初始化完 + 旧通路已工作）在空闲时间
    //       跑一遍 FAT32 根目录解析，把"找到几个文件"显示到数码管位 1，
    //       把 err_code 显示到位 2。旧通路（显示/语音/OSD/转场/SW2/SW4）
    //       行为完全不变，且始终对卡有绝对优先权。
    //
    // 一键退回旧行为：把这里改成 0（或删掉顶层例化里的
    // .USE_FAT_SCAN(1'b1) 那一行，用本默认值）
    //=========================================================================
    parameter integer USE_FAT_SCAN      = 0,
    // FAT 解析启动延迟（拍）。100MHz 下 1_000_000_000 拍 = 10 秒。
    //   这段时间里 bmp_read 的扫图 + 首图显示基本已经完成，
    //   FAT 解析再来用空闲时间，对用户可见行为零影响。
    // 只在 USE_FAT_LOAD=0（3a 通路）时有意义
    //     3b 通路（USE_FAT_LOAD=1）是"卡一初始化好就解析"，不用这个延迟。
    parameter [31:0]  FAT_START_DLY_CYC = 32'd1_000_000_000,

    //=========================================================================
    // 阶段 3b 总开关：用 FAT 文件表直接装载图片
    //
    //   0 = 默认，旧行为完全走旧通路：上电先让 bmp_read 从扇区 0 顺序
    //       扫扇区找 BM 标记（实测约 24 秒才出头一张图）。此时 3b 的两个
    //       模块（fat32_boot / bmp_load_fat）一个都不例化，被常量折叠掉，
    //       综合结果与 3a完全一致。
    //   1 = 启用 3b：上电 -> SD 初始化完成 -> 立刻解析 FAT 根目录
    //       （只读 MBR/DBR/根目录共几个扇区，约几十毫秒）-> 按文件表里
    //       的出现顺序把扩展名是 BMP 的文件依次装进 SDRAM。
    //       启动时间约 2 秒（SD 初始化 + FAT 解析 + 第一张图 0.85 秒）。
    //
    // 一键退回旧方式：把顶层例化里那一行 .USE_FAT_LOAD(1'b1)
    //     改成 .USE_FAT_LOAD(1'b0)，或整行删掉（默认值就是 0），
    // 然后重新综合下载。旧通路的代码一行都没删。
    //
    // 行为差异
    //   USE_FAT_LOAD=1 时，图片列表来自 FAT 文件表而不是"扫扇区"，
    //   所以轮播顺序 = 文件表里的顺序（00_1 -> 01_2 -> 02_3 -> 03_4），
    //   而不是按物理扇区顺序；名字也不再需要满足任何命名规律。
    //   语音那一路没走 FAT（仍是顺序扫扇区），但扫描起点会由 FAT 表里
    //   第一个 WAV 的首扇区给出（见下面 wav_scan_start 那一段），
    //   否则它会从扇区 0 一路扫过去、起播晚十几秒。
    //=========================================================================
    parameter integer USE_FAT_LOAD      = 0,

    //=========================================================================
    // 阶段 4 总开关：图片停留时长 = 讲解音频长度 + 停顿
    //
    //   1 = 默认停留时长由"卡上第一个 WAV 的字节数"算出来：
    //         停留 = 音频时长 + 1.5 秒（语音起播延迟）+ 1 秒（播完后的停顿）
    //       见下面"阶段 4：停留时长"那一段的推导。
    //   0 = 完全退回阶段 4 之前的行为：固定 5 秒（TICKS_5S），
    //       与上板验证过的 3b 行为逐拍一致。
    //
    // 一键切回固定 5 秒：把顶层例化里
    //       .USE_DWELL_FROM_WAV (1'b1)
    //     改成 (1'b0) 即可（一行，见 top.v 里 sd_card_bmp 的例化）。
    //     也可以直接用参数默认值之外的办法：把那一行整个删掉 -> 用本文件的
    //     默认值 1（也就是仍然启用阶段 4）。
    //
    // 只在 USE_FAT_LOAD=1（3b 通路）时才有意义
    //     旧通路（USE_FAT_LOAD=0）没有 FAT 文件表，拿不到 WAV 字节数，
    //     所以本开关即使为 1 也会自动退化成固定 5 秒。
    //=========================================================================
    parameter integer USE_DWELL_FROM_WAV = 1,

    //=========================================================================
    // 阶段 3c 总开关：音频也走 FAT 簇链 + 每张图播自己配的那段讲解
    //
    //   1 = 启用 3c：
    //       (A) 音频的扇区数据源从"起始扇区顺序往后读"换成
    //           "wav_src_fat 包装 fat32_file 按 FAT 簇链读"——
    //           这是"音频是嘈杂声"这个 bug 的根治（碎片化卡上顺序读
    //           读到的是别的文件的数据）。
    // wav_player.v 在这个方案里一个字符都没改
    //             改的只是它的三根输入接哪儿（见下面例化处的编译期 mux），
    //             它内部那套验证过的 ping-pong / P_LAT 取样本序列原样保留。
    //       (B) 按序号配对：01.BMP <-> 01.WAV（fat32_boot 走两遍文件表，
    //           用 8.3 名字的前两个字符配对）。配不到 WAV 的那张图
    //           用 5 秒兜底时长且不放语音。
    //       (C) 每张图出现时都重新起播它自己那段讲解（换图那一拍把
    //           wav_player 与 wav_src_fat 一起复位一次，再重新按配对结果
    //           从文件头读一遍），而不是"只有第一张有声音"。
    //   0 = 默认，逐拍退回阶段 4/3b 行为：
    //           wav_player 仍接仲裁器那一路、仍从扫描起点顺序往后读、
    //           停留时长仍按"表里第一个 WAV"算、音频只在第一次扫到 WAV
    //           后循环预取（旧卡上就是那个能正常出声的版本）。
    //           wav_src_fat 不例化（被常量折叠掉），
    //           fat32_boot 的 USE_PAIR_DWELL 也不打开。
    //
    // 一键切回旧行为：把顶层例化里那一行 .USE_FAT_WAV(1'b1)
    //     改成 (1'b0)，或整行删掉（默认就是 0），重新综合下载即可。
    // 只关音频这一路、图片仍走 FAT 直读是完全支持的（两个开关独立）
    //
    // 只有 USE_FAT_LOAD=1（有 FAT 文件表）时才有意义
    //     USE_FAT_LOAD=0 时本开关自动失效（下面 USE_FAT_WAV_EFF 会折叠成 0），
    //     旧通路（bmp_read 扫扇区）一行都没删，行为逐拍不变。
    //=========================================================================
    parameter integer USE_FAT_WAV = 0,
    //=========================================================================
    // S2 总开关：滚动字幕掩膜条（0N.SUB）装载
    //   1 = 每张图提交之后，顺手把这张图配对的那条 0N.SUB 读进来写进
    //       顶层那块 2048x48 的 1bpp 掩膜 RAM（复用 bmp 路 + 同一个
    //       bmp_load_fat/fat32_file 实例，串行做第二个活儿）。
    //   0 = 一键退回：bmp_load_fat 的 i_job_sub 恒 0、掩膜 RAM 写口恒 0、
    //       所有 S2 新增的输出恒 0 —— 资源与行为跟 S2 之前完全一样。
    //       （前提：顶层那一行 .USE_SUBTITLE(1'b1) 也要同时改回 1'b0，
    //        见 top.v 的 USE_SUBTITLE 参数注释）
    // 只在 USE_FAT_LOAD=1 时才有意义：配对表来自 fat32_boot
    //     所以下面 USE_SUB_EFF 会把它和 USE_FAT_LOAD 相与。
    //=========================================================================
    parameter integer USE_SUBTITLE = 0,

    //=========================================================================
    // 按键与轮播档位（三个开关都可一键回退）
    //
    // 物理按键：KEY_2(B2)=key_next、KEY_3(B1)=key_auto、KEY_4(C1)=key_period
    //
    //   SW4(i_auto_mode) = 1 正常工作
    //       SW1 = 0 看展
    //           KEY_2  手动切下一张（自动轮播进行中被禁用，只弹提示不切图）
    //           KEY_3  启动 / 停止自动轮播
    //           KEY_4  不切档；消抖后的脉冲送 osd_ctrl，只在 SW3=1 时做当前参数 -
    //       SW1 = 1 设置菜单
    //           KEY_2  移动光标（档位 0..3）
    //           KEY_3  确认，把光标档位写进 period_sel
    //           菜单期间自动轮播强制停止、音频强制静音
    //   SW4 = 0 锁定（防误触）
    //       自动轮播停、音频静音、三键全部失效；
    //       顶层把画面打黑（HDMI 保持同步不致失锁），数码管显示 "Loc"
    //
    //   USE_KEY_CTRL = 0：回到旧行为（自动轮播使能直接取自 SW4、KEY_3 不启停）
    //       注意旧行为本身有缺陷：SW4=1 时手动翻页被禁用，回退会把它一起带回来
    //
    //   USE_PERIOD_STAGE = 1'b1 默认：四个周期档位（只在 SW1=1 菜单里改）
    //       档位 0 = 讲解模式：停留 = 讲解音频时长 + 1.5s + 1s，且放音频
    //       档位 1 = 快速浏览  3s：到点切图，完全不启动音频
    //       档位 2 = 慢观看   10s：到点切图，完全不启动音频
    //       档位 3 = 手动浏览    ：不自动切图，只由 KEY_2 翻页
    //   USE_PERIOD_STAGE = 1'b0：一键退回"只有讲解模式"（音频行为与阶段 4 一致）
    //
    //   KEY_LOCK_ON_SW4 = 1'b1 默认：SW4=0 时三键全锁（含音频静音）
    //   KEY_LOCK_ON_SW4 = 1'b0：一键退回"SW4 只当模式指示、不锁键"
    //
    //   AUTO_PLAY_ON_BOOT = 1'b0 默认：上电后自动轮播处于停止，
    //       要按一下 KEY_3 才开始（展馆场景更可控，避免一上电就自己切图+讲解）
    //       改成 1'b1 = 上电即自动轮播（那时 KEY_3 当"暂停/继续"用）
    //
    // 上面三个开关与 USE_KEY_CTRL 的关系
    //     USE_KEY_CTRL=0 时，另外两个开关不起作用（等价于全部回退）。
    //     USE_KEY_CTRL=1 而 USE_PERIOD_STAGE=0 时：只剩档位 0（讲解）。
    //=========================================================================
    parameter integer USE_KEY_CTRL      = 1,
    parameter integer USE_PERIOD_STAGE  = 1,
    parameter integer KEY_LOCK_ON_SW4   = 1,
    parameter        AUTO_PLAY_ON_BOOT  = 1'b0,

    //=========================================================================
    // 下面 5 个"停留拍数"参数：只在仿真里改小用（生产必须保持默认）
    //   默认值 = 生产值，改成别的值不改变公式，只改数量级。
    //   为什么参数化：讲解档的停留时长是真卡音频时长（8.24 秒）+ 2.5 秒
    //   ≈ 10.7 秒 = 10.7 亿拍 @100MHz，行为仿真等不起。
    //   仿真把 5 个值同比例缩小（例如都 ×1/50），公式不变、时序关系不变。
    //   DWELL_AUDIO_EXTRA_CYC = 语音起播延迟 + 播完停顿（默认 1.5s + 1s = 2.5s）
    // 本次加宽 32 -> 33 位
    //     1.5s = 150_000_000 装得下 32 位，但这两个参数会与 33 位的
    //     音频拍数相加】（见下面 DWELL_EXTRA_CYC / dwell_wav_w），
    //     所以和计时链一起提到 33 位，避免"窄 + 宽"混算时上下文宽度不一致。
    //     下面三个 PERIOD_*_TICKS保持 32 位：它们只被零扩展到 33 位使用，
    //     值本身（3e8/5e8/8e8）远小于 2^32，改成 33 位反而要动顶层/仿真传参。
    //=========================================================================
    parameter [32:0] DWELL_AUDIO_DELAY_CYC = 33'd150_000_000,   // 1.5 秒 @100MHz
    parameter [32:0] DWELL_AUDIO_EXTRA_CYC = 33'd250_000_000,   // 2.5 秒 @100MHz
    parameter [31:0] PERIOD_3S_TICKS       = 32'd300_000_000,   // 3 秒 @100MHz
    parameter [31:0] PERIOD_5S_TICKS       = 32'd500_000_000,   // 5 秒 @100MHz
    parameter [31:0] PERIOD_10S_TICKS      = 32'd1000_000_000,  // 10 秒 @100MHz
    parameter [31:0] PERIOD_8S_TICKS       = 32'd800_000_000,   // 8 秒 @100MHz（旧，保留兼容）
    //=========================================================================
    // 2026-10-20 新增：SW1 = 轮播设置模式（setup mode）
    //   SETUP_MODE_EN=1 时启用下面这套新交互（SW1 由顶层同步后送 i_setup_mode）：
    //     SW1=1（设置中）：自动轮播强制停、音频强制停；
    //                      KEY_2 循环移动菜单光标（menu_sel 0..3）
    //                      KEY_3 确认 -> period_sel <= menu_sel（存下选择）
    //     SW1=0（看展中）：KEY_3 启动/停止自动轮播；KEY_2 手动切下一张
    //   SETUP_MODE_EN=0 一键回退：i_setup_mode 被忽略，行为与旧版一致。
    //=========================================================================
    parameter integer SETUP_MODE_EN = 1
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       key_period,       // 板载 KEY_4(C1)：手动档播音；SW3=1 时归 osd_ctrl 调参
    // SW3 = 1：KEY_4 作"当前参数 −"（亮度/对比度/音量，osd_ctrl 的既有功能）优先，
    //          手动档的"播/停当前图音频"让路，避免一次按键两个动作。
    input                       i_sw3,
    // 2026-10-20：SW1 = 轮播设置模式（1 = 设置中，0 = 看展中）
    //   同步到本时钟域后送入；SETUP_MODE_EN=0 时该输入被忽略。
    input                       i_setup_mode,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output      [3:0]           state_code,
    output      [1:0]           o_img_idx,        // 当前显示中的图片序号（0 起）

    // c72：装载进度，给字幕带画进度条
    //   o_bmp_prog_gray 用 gray 码跨到 25MHz 域（8 位进度每拍只变 1 位，采样无中间值）
    //   o_bmp_busy = 1 表示正在装载（读卡 + 写帧 + 转场，直到新图提交才落）
    output      [7:0]           o_bmp_prog_gray,
    output                      o_bmp_busy,
    // 轮播周期档位（0=讲解 1=3s 2=10s 3=手动）
    //   只在 SW1=1 的设置菜单里改（KEY_2 选、KEY_3 确认）；
    //   既决定 tick_target，也用于数码管显示。
    //   顶层约定：锁定模式（SW4=0）时顶层不用这个值，直接显示 "Loc"。
    output reg  [1:0]           period_sel,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO,

    // ===== TF 卡语音播放（新增）=====
    input                       clk_audio,       // 25MHz 音频域（与 video_clk 同源）
    output                      o_audio_valid,   // 以下三根在 clk_audio 域，48kHz
    output [23:0]               o_audio_left,
    output [23:0]               o_audio_right,
    output                      o_wav_found,     // 已找到 WAV（供音源切换）
    output [4:0]                o_wav_state,     // WAV 播放状态码（0~5）

    // ===== 供 OSD 菜单复用的消抖按键脉冲 =====
    output                      o_key1_press,    // 板载 KEY_2(B2)
    output                      o_key2_press,    // 板载 KEY_3(B1)
    output                      o_key3_press,    // 板载 KEY_4(C1)
    // OSD 打开时屏蔽翻页，避免调节参数时误翻页
    input                       i_block_page,

    // ===== 转场支持 =====
    // i_trans_hold  : 1 = 转场模块正在做遮罩，暂时不要切显示缓冲。
    //                 切换会被推迟到整屏全黑那一刻，天然无撕裂。
    // o_trans_pending: 新图已写好、正准备切换（可能正被 hold 拉住）。
    // o_trans_skip   : 1 = 第一张图不转场，立即切换。
    //                  （2026-10-19 起手动翻页不再走这条支，见 o_trans_force_push）
    // o_trans_force_push : 1 = 本次切换固定用上下推移，无视 SW2。
    //   为什么要它：用户 2026-10-19 明确要求「手动切换改为推移」（约 1.0 秒）。
    //   推移只有 1.0 秒，按键响应干脆；卷曲 2.35 秒，做手动翻页太拖。
    //   所以「用户按键触发的即时翻页」= 推移；「自动轮播」= SW2 选的转场。
    input                       i_trans_hold,
    output                      o_trans_pending,
    output reg                  o_trans_skip,
    output reg                  o_trans_force_push,
    // i_trans_busy: 1 = 转场进行中（video_transition 里 st != ST_IDLE）
    //   时长取决于转场类型：上下推移约 1.0 秒，页面卷曲约 2.35 秒。
    //   用来让轮播计时器在这段时间保持归零，否则周期从"放行换图"起算、
    //   转场还没结束就又到点，表现为"刚切完马上又切一张"。
    input                       i_trans_busy,
    // i_auto_mode: 工作模式，由 SW4 决定（已在顶层同步）
    //   1 = 正常工作（可显示 / 出声 / 操作按键）
    //   0 = 锁定（自动轮播停、音频静音、三键失效；顶层把画面打黑，数码管显示 "Loc"）
    input                       i_auto_mode,
    // 换图许可（来自 video_transition，已同步到本域）
    //   电平有效，默认 0。本模块必须等到它才能换图。
    //   原设计是自己判定"写完了就换"，再用 i_trans_hold 事后拉住，
    //   但 hold 要跨域同步 80ns 才回来，而 o_trans_pending 是组合逻辑，
    //   两者同一拍成立 -> 切换常常抢跑 -> 偶发地没有转场。
    //   改成许可式后，时序上不可能抢跑。
    input                       i_trans_go,

    // ===== 阶段 3a：FAT32 根目录解析结果（给数码管看）=====
    //   USE_FAT_SCAN=0 时 o_fat_done 恒 0，顶层据此把位 1/2 熄灭，
    //   所以看不到任何变化= 旧行为。
    output     [4:0]            o_fat_count,    // FAT 根目录里解析到的文件条数
    output     [3:0]            o_fat_err,      // fat32_dir 错误码（0 = 无错）
    output                      o_fat_cnt_ovf,  // 1 = 文件数 > 9（数码管只有一位）
    output     [3:0]            o_fat_cnt_digit,// 文件数的单个数码管位（>9 钳到 9）
    output                      o_fat_done,     // 1 = 解析已完成（可能是带错完成）

    // ===== 临时诊断：FAT32 解析内部关键值（给顶层数码管）=====
    //   USE_FAT_SCAN=0 时全部恒 0（与"没有这一路"等价）。
    //   这些都是纯观测信号，不参与任何功能判定。
    output     [31:0]           o_fat_dbg_root_lba,
    output     [31:0]           o_fat_dbg_part_lba,
    output     [31:0]           o_fat_dbg_data_lba,
    output     [31:0]           o_fat_dbg_fat_lba,
    output     [15:0]           o_fat_dbg_secs,
    output     [7:0]            o_fat_dbg_part_type,
    output     [7:0]            o_fat_dbg_root_b0,
    output     [7:0]            o_fat_dbg_root_b1,
    output     [15:0]           o_fat_dbg_rx_bytes,

    // ===== 阶段 3b：装载通路的现场诊断（USE_FAT_LOAD=1 时才有意义）=====
    //   USE_FAT_LOAD=0 时全部恒 0（单条 assign + 编译期常量选择，
    //   不是两个 generate 分支各写一句 assign —— 那样会被
    //   tools/verilog_multidriver_check.py 误判成多驱动，实测确认过）。
    output     [3:0]            o_fat3b_bmp_cnt,     // FAT 表里挑到几张 BMP
    output     [3:0]            o_fat3b_load_err,    // 装载器错误码（0=无错）
    output     [31:0]           o_fat3b_load_bytes,  // 最近一次装载取到的字节数

    // ===== S2：滚动字幕掩膜条（0N.SUB） =====
    //   USE_SUB_EFF=0（默认）时下面 7 根输出全部恒 0（单条 assign + 编译期
    //   常量选择，不是两个 generate 分支各写一句 —— 会被 MD-ASSIGN 误判）。
    //   掩膜 RAM 本体在顶层（跨 100MHz 写 / 25MHz 读），这里只把写口引出去。
    output                      o_sub_ram_we,        // 单拍写使能（一列一个 48bit 字）
    output     [10:0]           o_sub_ram_addr,      // 写到第几列
    output     [47:0]           o_sub_ram_wdata,     // 该列的 48 条扫描线
    output     [11:0]           o_sub_cols,          // 最近一次字幕装载写进去多少列
    output                      o_sub_ok,            // 1 = 最近一次字幕装载完整成功
    output     [2:0]            o_sub_found,         // 配到几条字幕（期望 4）
    output     [3:0]            o_sub_load_err,      // 字幕装载错误码（0=无错）
    // S3fix 新增：1 = 装载器正在往掩膜 RAM 写东西
    //   顶层用它把 sub_valid 拉低（装载中途 -> 不许叠加，只能透传）。
    //   USE_SUB_EFF=0 时恒 0（与上面 7 根同样的"单条 assign + 常量选择"写法）。
    output                      o_sub_busy,

    // ===== S4 新增：wav_player 的 o_playing（clk_audio = 25MHz 域） =====
    //   语义：1 = 音频确实已经开始出声（不是"允许起播"）。
    // 顶层拿它当滚动字幕的同步触发源：音频起播 -> 偏移归零并开始滚
    // 为什么必须是它、不能是 wav_play_go
    //     wav_play_go 是 100MHz 域、且只表示"允许起播"（图片提交就置起，
    //     还要等 1.5 秒才真出声）；o_playing 是 clk_audio 域 = video_clk 域，
    //     与显示/掩膜读同一个时钟域，零跨域成本、且时刻精确对准出声。
    // 本端口只是把已有的 wav_playing 内部 wire 引出去，不改任何行为
    //   （wav_player.v 一个字符都没改）
    output                      o_wav_playing,

    // ===== 按键交互 + 周期档位（本次新增） =====
    //   o_audio_gate     : 1 = 当前允许音频起播（讲解模式 且 未锁定）
    //                      0 = 音频被门住（定时档位 / 锁定模式 / 刚切回讲解档还没换图）
    // 它是 wav_play_go 的闸门，不是纯观测量 —— 有真实消费者
    //   o_period_is_explain : 1 = 当前档位是"讲解模式"（period_sel == 0）
    // 有真实消费者：它是 o_audio_gate 的因子
    //                      （顺带给顶层/仿真当观测点；不是只给诊断用）
    output                      o_audio_gate,
    output                      o_period_is_explain,

    // ===== 2026-10-20 新增：设置模式的菜单光标 / 确认脉冲 / 停止提醒 =====
    //   o_menu_sel   : 设置模式下的菜单光标（0=讲解 1=快速3s 2=慢看10s 3=手动浏览）
    //                  SW1=1 时由 KEY_2 循环移动；KEY_3 确认后它等于 period_sel。
    //   o_cfg_sel    : 已确认的档位（= period_sel 的影子，供菜单/数码管显示）
    //   o_stop_banner: 单拍脉冲，自动轮播被 KEY_3 停下时为 1（顶层用它在屏幕
    //                  上弹一条"已停止自动轮播"提示，代替原字幕提醒）
    //   o_manual_mode: 1 = 当前档位是"手动浏览"（手动档永不自动切图）
    output reg  [1:0]           o_menu_sel,
    output      [1:0]           o_cfg_sel,
    output                      o_stop_banner,
    output                      o_manual_mode,
    // ===== 2026-10-23 新增：自动轮播中的短提示（单拍脉冲，100MHz 域）=====
    //   两个时机都会拉高一拍，顶层把它跨到 video 域弹「自动轮播中」（1 秒）：
    //     ① KEY_3 刚启动自动轮播 —— 告诉操作者"现在是自动的"
    //     ② 轮播中按 KEY_2 被拒  —— 告诉操作者"不是按键坏了，要先停再切"
    output                      o_auto_hint
);

wire key_next_press;
wire key_auto_press;
wire key_period_press;

//=============================================================================
// 阶段 3c：总开关的"有效值"（把两个参数合成一个编译期常量）
//   为什么要有这一步：USE_FAT_WAV 单靠自己是站不住的 ——
//   它要的那张"每张图配哪段 WAV"的表是 fat32_boot 走文件表得出的，
//   而 fat32_boot 只在 USE_FAT_LOAD=1 时才例化。
//   所以"有效"= USE_FAT_LOAD 打开且USE_FAT_WAV 打开。
//   两个都是常量参数，综合时这一句直接折叠，不留任何逻辑。
//   下面所有 3c 的选择都用它，保证：
//     * USE_FAT_WAV=0        -> 与阶段 4 逐拍一致（旧行为）
//     * USE_FAT_LOAD=0 且
//       USE_FAT_WAV=1        -> 也自动退回旧行为，不会出现悬空/黑箱
//=============================================================================
localparam integer USE_FAT_WAV_EFF = ((USE_FAT_LOAD != 0) && (USE_FAT_WAV != 0)) ? 1 : 0;

//=============================================================================
// S2：字幕装载总开关的"有效值"（与上面 USE_FAT_WAV_EFF 同构）
//   USE_SUBTITLE 单靠自己也站不住：它要的"每张图配哪条 .SUB"的表
//   是 fat32_boot 走文件表得出的，而 fat32_boot 只在 USE_FAT_LOAD=1 时才例化。
//   所以"有效" = USE_FAT_LOAD 打开且USE_SUBTITLE 打开。
//   两个都是常量参数，综合时这一句直接折叠，不留任何逻辑。
//=============================================================================
localparam integer USE_SUB_EFF = ((USE_FAT_LOAD != 0) && (USE_SUBTITLE != 0)) ? 1 : 0;

// 把消抖后的按键脉冲引出去给 OSD 菜单复用（原来只在模块内部用）
assign o_key1_press = key_next_press;    // 板载 KEY_2
assign o_key2_press = key_auto_press;    // 板载 KEY_3
assign o_key3_press = key_period_press;  // 板载 KEY_4

// 注意：o_trans_pending 的 assign 放在后面 load_busy/source_done_seen/
// write_done_seen 声明之后，避免前向引用（TD 会报错）。

// ---- bmp_read <-> SD 扇区读仲裁器 ----
// 原来 sd_sec_* 由 bmp_read 直接驱动并直连 sd_card_top；
// 现在中间插仲裁器，所以 bmp 侧与"最终送到卡"的信号必须分开命名。
wire             bmp_sec_read;
wire [31:0]      bmp_sec_read_addr;
wire [7:0]       bmp_sec_read_data;
wire             bmp_sec_read_data_valid;
wire             bmp_sec_read_end;

// ---- wav_player <-> SD 扇区读仲裁器 ----
wire             wav_sec_read;
wire [31:0]      wav_sec_read_addr;
wire [7:0]       wav_sec_read_data;
wire             wav_sec_read_data_valid;
wire             wav_sec_read_end;

// ===================== 阶段 3c：音频走 FAT 簇链那一路 =====================
//   wav_src_fat 的端口（例化在下面 USE_FAT_LOAD 的 generate 分支里）。
//   这些 wire 在 USE_FAT_WAV_EFF=0 时没有驱动源，但那时所有使用它们的
//   选择支都是常量 0，综合结果里连一根线都不剩 —— 与 fat3b_* 三根线
//   完全同样的处理方式（见本文件"装载通路诊断"那一段的说明）。
wire             wsrc_wav_req_w;    // -> 仲裁器 wav_req（3c：数据扇区）
wire [31:0]      wsrc_sd_lba_w;     // -> 仲裁器 wav_lba / fat_lba（两种请求共用）
wire             wsrc_fat_req_w;    // -> 仲裁器 fat_req（3c：FAT 表扇区，与 fat32_boot 共用）
// 3c 关闭时把上面那根请求线折成常量 0（单条 assign + 编译期选择，
// 不许两个 generate 分支各写一句 —— 会被 MD-ASSIGN 误判成多驱动）
wire             wsrc_fat_req_m = (USE_FAT_WAV_EFF != 0) ? wsrc_fat_req_w : 1'b0;
// 阶段 3c：fat 那一路（最低优先级）有两个请求者：fat32_boot 与 wav_src_fat
//   两者在时间上不重叠：fat32_boot 只在 B_RUN 里请求、解析完就永远不再要；
//   而 wav_src_fat 只在"某张图提交"之后才开始要 FAT 表扇区。
//   所以这里一个简单的 OR + 地址二选一就够，不需要真正的仲裁。
// 回程数据是广播的：两边都挂着 i_data/i_data_valid；谁在等由它自己的
// 状态机决定（fat32_dir done 之后一律不看来的字节，见 fat32_boot 注释）
// 这两根线的 assign 写在下面"阶段 3a：FAT 解析那一路"声明之后
//     本工程在 TD 上踩过 HDL-5373（"使用先于声明"）：第一版把 assign 写在
//     这里（fat_sec_read 还没声明），真 TD 报了 3 条新告警。
wire             fat_arb_req_w;
wire [31:0]      fat_arb_lba_w;
// 仲裁器 wav 那一路"最终"接谁：3c 打开时接 wav_src_fat，否则接 wav_player
// 写成"单条 assign + 编译期常量选择"
//     两个 generate 分支各写一句 assign 会被 verilog_multidriver_check.py
//     的 MD-ASSIGN 规则误判成多驱动（工具不认识 generate 分支，实测确认过）。
wire             wav_arb_req_w = (USE_FAT_WAV_EFF != 0) ? wsrc_wav_req_w : wav_sec_read;
wire [31:0]      wav_arb_lba_w = (USE_FAT_WAV_EFF != 0) ? wsrc_sd_lba_w : wav_sec_read_addr;
// 每张图配对的那段 WAV 的"首簇/大小/首扇区/有没有"
wire [31:0]      wav_sel_clu_w;
wire [31:0]      wav_sel_size_w;
wire [31:0]      wav_sel_lba_w;
wire             wav_sel_has_w;
// 阶段 3c：每张图重启音频那一路用的复位脉冲（声明必须在使用它的例化之前，
// 否则是"使用先于声明"，TD 会报 HDL-5373）
//   驱动逻辑（含那个 40 拍计数器）在下面"语音扫描起点"那一节，
//   因为它要用主状态机产生的 img_commit_pulse。
reg  [5:0]       wav_rst_cnt;
wire             wav_reload;
wire             wav_player_rst;
wire             wav_src_rst;

// ---- SD 扇区读仲裁器 -> sd_card_top ----
wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;

// ===================== 阶段 3a：FAT 解析那一路 =====================
// fat32_scan -> 仲裁器第 3 路（最低优先级）
// 这些声明必须在使用它们的例化之前（TD 对前向引用报错）
wire             fat_sec_read;
wire [31:0]      fat_sec_read_addr;
wire [7:0]       fat_sec_read_data;
wire             fat_sec_read_data_valid;
wire             fat_sec_read_end;
wire             fat_old_busy;
// 阶段 3c：fat 那一路的"合流"（声明在上一段，assign 放这里 —— 见上面的说明）
assign fat_arb_req_w = fat_sec_read | wsrc_fat_req_m;
assign fat_arb_lba_w = fat_sec_read ? fat_sec_read_addr : wsrc_sd_lba_w;
wire             sd_xfer_active;
wire [1:0]       sd_xfer_owner;
// 2026-10 超时修复：仲裁器的三路"授权"指示
//   sd_grant_bmp -> bmp_load_fat（它的两次读卡都走 bmp 那一路）
//   sd_grant_fat -> fat32_boot / fat32_scan（FAT 解析那一路）
//   o_grant_wav  -> 不接：wav_player 内部没有扇区超时计数器
//                   （它只有在 S_IDLE 等 scan_go，没有"等数据"的计时逻辑），
//                   没有需要修的同类缺陷，所以没有必要给它加端口。
// 这三根线只影响"超时计时要不要跳过排队期"，不参与任何数据/仲裁判定
wire             sd_grant_bmp;
wire             sd_grant_fat;
// 阶段 3c：o_grant_wav 以前是空的（wav_player 内部没有扇区超时计数器，
//   "排队不计时"对它没意义）。现在它有了第二个用途 ——
//   wav_src_fat 用它做"回程字节只在卡确实归我时才收"的门控，
// 所以这根线必须真的接出来
wire             sd_grant_wav;
wire [4:0]       fat_scan_count;
wire [3:0]       fat_scan_err;
wire             fat_scan_done;
wire             fat_scan_busy;
wire [3:0]       fat_scan_cnt_digit;
wire             fat_scan_cnt_ovf;

// 临时诊断：fat32_dir 内部关键值（模块级声明，generate 两个分支都要赋值）
wire [31:0]      fat_dbg_root_lba;
wire [31:0]      fat_dbg_part_lba;
wire [31:0]      fat_dbg_data_lba;
wire [31:0]      fat_dbg_fat_lba;
wire [15:0]      fat_dbg_secs;
wire [7:0]       fat_dbg_part_type;
wire [7:0]       fat_dbg_root_b0;
wire [7:0]       fat_dbg_root_b1;
wire [15:0]      fat_dbg_rx_bytes;

// ===================== 阶段 3b：FAT 直读装载那一路 =====================
// 全部在模块级声明
//   理由与上面 fat_* 一样：generate 的两个分支都要给同一批信号赋确定值，
//   而 TD 对前向引用报错，所以声明必须在使用它们的例化之前。
//   命名统一用 fat_boot_*（FAT 引导：解析 + 名单）与 fat_load_*（装载器）。
wire [3:0]  fatld_tbl_idx;        // 装载器 -> FAT 引导 的"读哪一条表项"
// c72：装载进度与装载中标志（只在 USE_FAT_LOAD 分支有驱动，
//      旧通路用下面的单条 assign 选常量 0 —— 与 fat3b_* 同一套写法）
wire [7:0]  fatld_prog_w;
wire        fatld_busy_w;
wire [87:0] fat_boot_tbl_name;
wire [31:0] fat_boot_tbl_cluster;
wire [31:0] fat_boot_tbl_size;
wire [7:0]  fat_boot_tbl_attr;
wire [31:0] fat_boot_fat_lba;
wire [31:0] fat_boot_data_lba;
wire [7:0]  fat_boot_spc;
wire [4:0]  fat_boot_spc_lg2;
wire        fat_boot_ready;       // 名单已就绪 -> 当 bmp_ready 用
wire        fat_boot_done;        // FAT 解析完成
wire [3:0]  fat_boot_bmp_cnt;     // 挑到几张 BMP
wire        fat_boot_wav_found;   // 表里有 WAV
wire [31:0] fat_boot_wav_lba;     // 第一个 WAV 的首扇区（语音扫描起点）
wire [3:0]  fat_boot_phase;       // 调试：3b 引导阶段
wire [31:0] fat_boot_root_lba_dbg;
wire [31:0] fat_boot_part_lba_dbg;
wire [31:0] fat_boot_data_lba_dbg;
wire [31:0] fat_boot_fat_lba_dbg;
wire [15:0] fat_boot_secs_dbg;
wire [7:0]  fat_boot_part_type_dbg;
wire [7:0]  fat_boot_root_b0_dbg;
wire [7:0]  fat_boot_root_b1_dbg;
wire [15:0] fat_boot_rx_bytes_dbg;
wire        fat_load_ready;       // 装载器空闲
wire [3:0]  fat3b_bmp_cnt_w;      // 3b：FAT 表里挑到几张 BMP（例化驱动）
wire [3:0]  fat3b_load_err_w;     // 3b：装载器错误码（例化驱动）
wire [31:0] fat3b_load_bytes_w;   // 3b：最近一次装载取到的字节数（例化驱动）
// 阶段 4：fat32_boot 按"第一个 WAV 的字节数"算出的停留拍数（例化驱动）
//   USE_FAT_LOAD=0 时这两根线没有驱动源（悬空），但使用它们的选择支是常量 0，
//   合成结果里连一根线都不剩 —— 与上面 fat3b_* 三根线完全同样的处理方式
//   （见本文件后面"装载通路诊断"那一段的说明）。
wire [32:0] fat3b_wav_ticks_w;    // 音频时长(拍)；无效时是 5 秒兜底值（本次 32->33 位）
wire        fat3b_wav_ok_w;       // 1 = 上面那个值来自真 WAV 大小

// ---- S2：滚动字幕掩膜条（0N.SUB）那一路的线 ----
//   驱动源是下面 USE_FAT_LOAD 的 generate 分支里的例化输出。
//   USE_SUB_EFF=0 时这些线没有驱动源（悬空），但所有使用它们的选择支都是
//   常量 0，综合结果里连一根线都不剩 —— 与上面 fat3b_* 三根线同样的处理方式
//   （两个 generate 分支各写一句 assign 同一根 net 会被
//     tools/verilog_multidriver_check.py 的 MD-ASSIGN 规则误判成多驱动）。
wire        sub_ram_we_w;         // 掩膜 RAM 写使能（bmp_load_fat 驱动）
wire [10:0] sub_ram_addr_w;       // 写到第几列
wire [47:0] sub_ram_wdata_w;      // 该列的 48 条扫描线
wire [11:0] sub_cols_w;           // 最近一次字幕装载写进去多少列
wire        sub_ok_w;             // 1 = 最近一次字幕装载完整成功
wire [3:0]  sub_load_err_w;       // 字幕装载错误码（0=无错）
wire [2:0]  sub_found_w;          // 配到几条字幕（fat32_boot 驱动）
wire [3:0]  sub_sel_idx_w;        // 当前这张图配到的 .SUB 表下标
wire        sub_sel_has_w;        // 当前这张图配到 .SUB 了吗
wire        sub_busy_w;           // bmp_load_fat 报"正在做字幕装载"
// 单条 assign + 编译期常量选择（见上面说明）
wire        sub_sel_has_m = (USE_SUB_EFF != 0) ? sub_sel_has_w : 1'b0;
wire        sub_busy_m    = (USE_SUB_EFF != 0) ? sub_busy_w    : 1'b0;

// ---- wav_player 状态 ----
wire             wav_found_i;       // wav_player 报"已找到"
wire             wav_playing;       // clk_audio 域：已开始播放
wire [4:0]       wav_state_raw;     // 扫描/预取状态码（clk_sd 域）

// 对外状态码：正在播就显示 3，否则显示扫描/预取状态
assign o_wav_state = wav_playing ? 5'd3 : wav_state_raw;
assign o_wav_found = wav_found_i;
wire [3:0]       state_code_i;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;

// 装载器直接吐出的像素流（缩放前）。缩放器把它们变成 write_en/write_data 要的格式。
//   640×480 -> 直通；1280×960 -> 2:1 缩小。见 img_scale_2to1.v
wire             bmp_data_wr_en_src;
wire [23:0]      bmp_data_src;
wire             bmp_2to1_mode;      // 1 = 本次装载是 1280×960

// c72：进度选择 + gray 编码
//   写成"单条 assign + 编译期常量选择"，不是 generate 两分支各写一句
//   （后者会被 tools/verilog_multidriver_check.py 的 MD-ASSIGN 误判成多驱动，
//    与本文件 wav_ticks_sel / bmp_2to1_mode 同一套写法）
wire [7:0]       bmp_prog_bin = (USE_FAT_LOAD != 0) ? fatld_prog_w : 8'd0;
wire             bmp_busy_w   = (USE_FAT_LOAD != 0) ? fatld_busy_w : 1'b0;

// 二进制 -> gray：每拍只可能变 1 位，跨时钟域采样不会出现中间值
assign o_bmp_prog_gray = bmp_prog_bin ^ (bmp_prog_bin >> 1);
assign o_bmp_busy      = bmp_busy_w;
wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
// 阶段 3c：图片"提交"（真正显示出来）那一拍的单拍脉冲
//   用途：每张图出现时重启音频那一路（wav_player / wav_src_fat 复位 +
//   换成这张图配对的那段 WAV + 重新起算 1.5 秒起播延迟）。
//   它在主状态机里与 display_valid 的置位同拍产生（见下面的提交分支）。
reg              img_commit_pulse;
// S2：字幕装载的时序控制（本块是这三个 reg 的唯一驱动源）
//   sub_job_pulse ：与 load_start_pulse 同拍的单拍脉冲，告诉 bmp_load_fat
//                   "这一次走字幕通路"（它俩必须同拍到达，否则会走错通路）。
//   sub_busy_r    ：字幕装载占着 bmp 路的标志。它把三个"图片装载启动"分支
//                   门住 —— 否则手动按键恰好落在字幕装载那 11ms 里时，
//                   图片装载会被 bmp_load_fat 忽略，而主状态机已经置了
//                   load_busy，只能等 3 秒看门狗才恢复。
//   sub_arm       ：刚发出脉冲后的几拍"保护期"（那时装载器还在 L_IDLE，
//                   不能凭 ready=1 就以为活儿干完了）。
reg              sub_job_pulse;
reg              sub_busy_r;
reg [2:0]        sub_arm;
// ===================== 按键交互 + 周期档位：新寄存器（本次新增） =====================
//   全部在这一个always 块里驱动（见下面主状态机），满足"一个 reg 一个 always"。
//   audio_need_reload : "音频这一路需要在本张图提交时重启一次"的 sticky 标志。
//     置位：① 复位；② 从"定时档位/锁定"切回"讲解模式"那一刻（细节③：
//           当前这张图静静停着，等下一张图再开始播，不会半路冒出一句话）
//     清零：audio_reload 真的发出去了（= 某张图提交且闸门允许音频）
//   audio_need_u      : 占位保持标志，保证上面那个"切换那一拍"只有一拍宽
// 只有 USE_AUTO_LIMIT > 0 时才用（默认 0 = 无限张，与旧行为一致）
reg              audio_need_reload;
reg              audio_need_u;
// 轮播周期档位总数（编译期常量，综合时折叠）：
//   USE_PERIOD_STAGE=1 -> 4 档（讲解 / 3s / 10s / 手动）；=0 -> 只有 1 档（讲解）
localparam integer PERIOD_STAGE_N = (USE_PERIOD_STAGE != 0) ? 4 : 1;
// 最后一档的编号（= N-1，2bit）——用 localparam 而不是参数位选，
// 因为 Verilog 里 "参数[1:0]" 是非法语法（参数不是向量，不能位选）。
localparam [1:0]  PERIOD_SEL_MAX = (USE_PERIOD_STAGE != 0) ? 2'd3 : 2'd0;
// "锁定模式"有效值：USE_KEY_CTRL=0 时恒 0（= 不锁，退回旧行为）
// 2026-10-12 修正：极性反了
// 原来写成 ? i_auto_mode : 1'b0 → SW4=1 时锁、SW4=0 时可用（和需求正好相反）
// 需求是：SW4=1 正常（三键可用） SW4=0 锁定（三键失效）
// → 加一个取反 ~ 即可（注释 L148/L1062 本来写的就是"SW4=0 时锁"，是代码写反了）
wire             key_locked_w = ((USE_KEY_CTRL != 0) && (KEY_LOCK_ON_SW4 != 0)) ? ~i_auto_mode : 1'b0;
// 档位是不是"讲解模式"：有真实消费者（o_audio_gate 的因子 + o_period_is_explain 输出）
wire             period_is_explain_w = (PERIOD_STAGE_N > 1) ? (period_sel == 2'd0) : 1'b1;

//=============================================================================
// 2026-10-20 新增：SW1 设置模式（setup mode）
//   setup_w   : SETUP_MODE_EN=0 时恒 0（一键回退，i_setup_mode 被忽略）
//   manual_w  : 档位 3 = 手动浏览（永不自动切图，只能按 KEY_2 一张张翻）
//=============================================================================
wire             setup_w  = (SETUP_MODE_EN != 0) ? i_setup_mode : 1'b0;
wire             manual_w = (PERIOD_STAGE_N > 1) ? (period_sel == 2'd3) : 1'b0;
assign           o_manual_mode = manual_w;
assign           o_cfg_sel     = period_sel;
// 设置模式统一表示成"三键全锁"：KEY_2/KEY_3 在 setup 下改走菜单分支，
// 其余所有依赖 !key_locked_w 的通路（音频门、定时切图）一律被按住。
wire             key_locked_all_w = key_locked_w | setup_w;

//=============================================================================
// 本次新增：音频闸门（细节① 的载体）
//   gate = 1：当前允许音频起播 —— 讲解档位 + 未锁定 + 已经为这张图重启过
//   gate = 0：音频被门住，三种情况
//     ① 定时档位（3 秒 / 10 秒档）：细节① —— 根本不启动 wav_player，
//        不是"静音"也不是"音量 0"：wav_play_go 恒 0 ->
//        wav_player 的播放侧永远停在 P_IDLE 出静音（o_left/o_right = 0），
//        同时取数侧在定时档位下不重启（wav_reload 不发），
//        连 SD 卡的扇区读都不去要 —— 最干净、还省带宽。
//     ② 锁定模式（SW4=0）：防观众乱按，顺手把声音也停掉。
//     ③ 刚切回讲解档、但当前这张图提交时还不是讲解档（audio_need_reload=1）：
//        细节③ —— 当前这张图静静停着，等下一张图再开始播。
// 它是 wav_play_go 的闸门，不是观测量 -> 有真实消费者，TD 不会剪
//=============================================================================
// 声明位置说明
//     audio_gate_w 的两个因子（period_is_explain_w / audio_need_reload）
//     都在本段上面声明过了，所以能在这里直接写连续赋值；
//     而 audio_need_reload 的驱动 always 块在文件靠下（要用到 key_locked_w，
//     那又要用主状态机那段的条件）—— 声明与驱动分开是本项目的既定写法
//     （wav_reload / wav_player_rst 等也是这么处理的）。
// 2026-10-20：auto_play_en 的声明必须提到这里
//   下面 audio_gate_w 要引用它；若仍留在原处（audio_gate_w 之后）会变成
//   "先使用后声明"，iverilog 会当成隐式 net 再与 reg 声明冲突。
reg              auto_play_en;
//------------------------------------------------------------------------------
// 手动档音频通道（本次新增）
//
// 背景：原来 audio_gate_w 同时带了 auto_play_en 与 !manual_w，所以手动浏览档
//       （period_sel=3）下音频恒静音，KEY_4 也没有任何音频动作。
//       本次给手动档开一条独立通道：KEY_4 按一下播、再按停。
//
// 为什么不是直接把 !manual_w 去掉
//   手动档的定义就是"纯手动、不自动播"。去掉 !manual_w 会让自动通道在手动档
//   也成立，于是只要 auto_play_en 还残留 1 就会自动出声、自动切图 —— 这正是
//   2026-10-20 那次修订要避免的暗态。所以这里保持两条通道并联、互不干扰：
//     自动通道：讲解档 + 自动轮播开 + 非手动档
//     手动通道：手动档 + 用户按过 KEY_4
//------------------------------------------------------------------------------
reg              man_audio_en;         // 1 = 手动档音频"请求播放"

// wav_playing 在 clk_audio（25MHz）域，本模块是 100MHz 域。
// 这里只把它当电平用（不取边沿），但仍走两级同步，避免按键那一拍正好采到亚稳态。
reg  [1:0]       wav_playing_sy;
wire             wav_playing_s = wav_playing_sy[1];

wire             man_audio_key_w = manual_w && key_period_press && !i_block_page
                                   && !key_locked_all_w && wav_sel_has_w && !i_sw3;

// "真的在播" = 用户请求播放 且 播放器已经起播。
//   用它同时区分两件事：
//     ① 按下去该"播"还是该"停"
//     ② WAV 自然播完后 wav_playing_s 会落下，此时再按一次会被判成"没在播"
//        -> 直接从当前图重新起播，不需要先按一次"空停"（这是最初的实现漏掉的）
wire             man_audio_playing_w = man_audio_en && wav_playing_s;
// 只有"要从头起播"那一拍才需要重启 wav 这一路（重新指向当前图的 WAV 并从头放）
wire             man_wav_reload_w    = man_audio_key_w && !man_audio_playing_w;

wire             audio_auto_gate_w = period_is_explain_w && !key_locked_all_w
                                     && !audio_need_reload && auto_play_en && !manual_w;
// 手动通道刻意【不】带 !audio_need_reload：
//   audio_need_reload 的语义是"等下一张图提交时才重启音频"（细节③），
//   那是给自动通道用的。手动档是用户显式按键即时起播，本来就要立刻响，
//   而且手动路径自己会发 audio_reload 把该标志清掉。
//   若在这里也串上它，还要反过来改 audio_need_reload 的置位条件才不死锁，
//   白白多改一处、多一片组合逻辑 —— 当前 slices 已卡在 90% 边缘，不值得。
wire             audio_man_gate_w  = manual_w && man_audio_en && !key_locked_all_w
                                     && wav_sel_has_w;
wire             audio_gate_w = audio_auto_gate_w || audio_man_gate_w;
// 图片提交时"音频这一路要不要真的重启"
//   USE_KEY_CTRL=0（一键回退）时本式退化成原来的 img_commit_pulse 单条件，
//   与阶段 3c 逐拍一致。
// 2026-10-12 修正：原来用 audio_gate_w 会形成循环依赖
// need_reload -> audio_gate_w -> audio_reload -> need_reload
// 症状：audio_gate_w 恒 0 -> 任何模式下都不放音频
// 改法：这里直接用"档位 + 未锁"，不用 audio_gate_w-> 打破环
//（语义不变：audio_reload 的意图就是"讲解档且未锁时，图片提交要重启音频这一路"）
// 2026-10-20：把 key_locked_w 换成 key_locked_all_w —— 设置模式下不重启音频
// 2026-10-20 二次修订：audio_reload 的 assign 已下移到 start_now_w
//    声明之后（本文件靠下的"KEY_3 启动"那一段），原因是它现在还要用
//    start_now_w —— 而 start_now_w 又依赖 img_found_count / auto_play_en，
//    在 L680 这里都还没声明，直接写就是前向引用（iverilog 会当成隐式 net，
// 再与后面的 reg 声明冲突）。这里只留声明，驱动见下方。
wire             audio_reload;
assign           o_audio_gate = audio_gate_w;
// 本次加宽 32 -> 33 位
//   33 位 @100MHz 上限 = 2^33/1e8 = 85.899 秒；最坏情况 60 秒音频 + 1.5 + 1
// = 62.5 秒 < 85.9 秒，装得下（32 位上限只有 42.95 秒，装不下 42.42 秒的音频）
reg [32:0]       auto_cnt;
reg [32:0]       tick_target;          // 当前档位的计数目标（寄存器保存，见 auto_tick）
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              prev_req_pending;     // 手动档 KEY_3：上一张
reg              load_busy;
reg [31:0]       load_timeout_cnt;

// 手动翻页请求：下一张 / 上一张共用一条装载支路，方向由 prev_req_pending 决定。
// 这样比"再复制一条 else if 分支"省一套装载握手，也避免了两个分支同拍竞争。
// 声明必须在 next_req_pending / prev_req_pending 之后（TD 对前向引用报错）。
wire             step_req_w = next_req_pending | prev_req_pending;

reg              load_abort;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
reg              write_done_seen;

// 新图已写好、正准备切换（可能正被 i_trans_hold 拉住）。
// 转场模块用它做握手：看到 1 -> 开始遮罩 -> 遮满后放开 hold -> 等它落下。
//（必须放在上面三个寄存器声明之后，否则是前向引用，TD 会报错）
assign o_trans_pending = load_busy && source_done_seen && write_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_current;
wire [1:0] step_from_current;          // 本次手动翻页的目标序号（上一张或下一张）

// ===================== 装载侧缩放（c71）=====================
//   位置：bmp_load_fat / bmp_read 的像素出口 -> 这里 -> 帧缓冲写口
//   为什么放这儿：下游（帧缓冲/转场/卷曲/OSD/字幕）全部按 640×480 线性寻址，
//     在入口把尺寸统一掉，这条链一个字节都不用改。
//   为什么不需要背压：本模块每 4 个输入像素才产 1 个输出像素，输出速率永远是
//     输入的 1/4；且不产生信箱黑行（黑边在素材里，跟着被平均），不存在
//     CURL22-27 那种"无输入也连发几万像素"的通路。
img_scale_2to1 u_img_scale (
    .I_clk   (clk),
    .I_rst   (rst),
    .I_start (load_start_pulse),     // 每张图开始清地址（字幕装载触发也无害）
    .I_2to1  (bmp_2to1_mode),
    .I_pix   (bmp_data_src),
    .I_de    (bmp_data_wr_en_src),
    .O_pix   (bmp_data),
    .O_de    (bmp_data_wr_en)
);

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};

// ===================== 轮播周期档位：讲解 / 3s / 10s / 手动 =====================
//   period_sel 由 SW1=1 设置菜单写入（KEY_2 选、KEY_3 确认）：
//     0 讲解     ：停留 = 音频时长 + 起播延迟 + 停顿（会启动音频）
//     1 快速浏览 ：3 秒到点切图，不启动音频
//     2 慢观看   ：10 秒到点切图，不启动音频
//     3 手动     ：不自动切图，只由 KEY_2 翻页（tick 目标不参与）
//   定时值来自参数（默认 3s / 10s @100MHz）；
//   参数化只是为了仿真时把数量级改小，生产构建取默认值，公式不变。
//   这些是编译期常量选择 + 一个 2bit 选择器，不生成乘法器。
// 本次加宽：这些常量全部提到 33 位（bit_new 进 tick_target 是 33 位）
// 特别注意 TICKS_1S：100e6 x 1 = 1e8 装得下 32 位，但同一行族里的
//       CLK_FREQ_HZ * 5 = 5e8 只有 29 位；真正装不下的是允许的最坏情况
//       60 秒 + 1.5s + 1s = 62.5 秒 = 6.25e9 > 2^32】，所以与计时链同宽写 33 位。
//       算式本身用 64 位字面量（100_000_000 x 5 = 5e8 仍 < 2^31-1，
// 但这里统一写 64'd 是为了和 fat32_boot 的 39->60 改法保持一致）
localparam [32:0] TICKS_1S = 64'd1 * CLK_FREQ_HZ;
// 讲解档"音频之外还要等多久"的兜底值（真值走参数，见文件头 DWELL_AUDIO_*_CYC）
localparam [32:0] TICKS_5S = (PERIOD_5S_TICKS != 32'd0) ? {1'b0, PERIOD_5S_TICKS}
                                                        : (64'd5 * CLK_FREQ_HZ);

//=============================================================================
// 阶段 4：停留时长 = 讲解音频长度 + 停顿
//
// 计时起点（先把事实查清，再决定怎么算）
// ------------------------------------
//   auto_cnt 的计数条件（见下面主状态机）是：
//       scan_done && auto_play_en && display_valid && !load_busy
//                  && !i_trans_busy && first_image_committed && (img_found_count>1)
//   不满足就把 auto_cnt 清 0。于是它的"零点"是：
//       * 首图：提交那一拍（display_valid 拉高、无装载、无转场）——即"图片出现"
//       * 后续图：转场淡入结束那一拍（i_trans_busy 落下）——即"图片完全显现"
//   两种零点都是"图片看得见"的时刻，这就是停留计时的起点。
//
// 那语音是什么时候开始播的
// ------------------------
//   语音起播门控是 display_valid + AUDIO_DELAY_CYC（1.5 秒，见下面"语音起播
//   延迟"那一段）。首图提交与 display_valid 拉高是同一拍，所以：
//       音频起点 = 停留计时零点 + 1.5 秒
//       音频终点 = 停留计时零点 + 1.5 秒 + 音频时长
//   → 要想"讲完再切"，tick_target 必须把这 1.5 秒也算进去：
//       tick_target = AUDIO_DELAY_CYC + 音频时长 + 停顿
//   （如果只算"音频时长 + 停顿"，切换会比音频播完早 1.5 秒——
//     图在讲解最后一句时就被切走，等于没修。这一点是这次改动的关键。）
//
// 数值（真卡：VOICE.WAV = 791378 字节）
// ------------------------------------
//   音频时长 = (791378-44)/96000 = 8.243 秒；换算成拍 = 8.2378 秒（见下）
//   tick_target = 1.5 + 8.2378 + 1.0 = 10.7378 秒
//   音频播完时刻 = 1.5 + 8.2431 = 9.7431 秒
//   => 切换发生在音频播完后 0.9947 秒（约 1 秒停顿），符合设计目标。
//
// 精度
//   音频秒数 -> 拍数的换算在 fat32_boot.v 的 wav_dwell_calc 里，
//   那里把 CLK_FREQ_HZ/96000 = 1041.666 取整成 1041（少算 0.064%）。
//   8.24 秒 -> 8.2378 秒，差 5.3 毫秒，对"停留多久"无感。详见那里的文件头。
//
// 上限保护：为什么是 60 秒，以及为什么必须先加宽到 33 位
// 本次改动的起因（实测数据）
//     新素材里 01.WAV = 42.42 秒：
//       tick_target = 1.5(起播延迟) + 42.42(音频) + 1.0(停顿) = 44.92 秒
//     而 32 位 @100MHz 的上限 = 2^32/1e8 = 42.95 秒
// -> 44.92 > 42.95 -> auto_cnt / tick_target 溢出
// -> 停留时长和切图时刻全错
// 改法：把 auto_cnt / tick_target（以及 fat32_boot 的 o_wav_ticks、
//     wav_dwell_calc 的 o_ticks）一起加宽到 33 位：
//       33 位上限 = 2^33/1e8 = 85.899 秒
// 最坏情况 = 60(钳位上限) + 1.5 + 1 = 62.5 秒 < 85.9 秒 留有余量
// 时钟频率的约束（写清楚，免得以后有人乱调）
//     要求 CLK_FREQ_HZ <= 2^33 / 62.5 = 137.4 MHz（本工程 100MHz，余量充足）
// 钳位值本身也在 fat32_boot.v 的 wav_dwell_calc 里同步从 39 秒提到 60 秒
//     两处必须同时改：只加宽不抬钳位 -> 42.42 秒的音频仍被钳到 39 秒，
// 停留时长偏短 3.4 秒（用户要的 42.42 秒仍然放不完）
//
// tick_target 的装载时机
//     ① 每张图片提交那一拍 —— 按当前period_sel（tick_new_w）
//     ② 设置菜单 KEY_3 确认那一拍 —— 按光标目标档位（tick_menu_w）
//     ③ 复位兜底
//   ①② 必须分开算：确认那一拍 period_sel 还没更新，
//   若沿用 tick_new_w 会装成旧档位的周期（表现为"按了没反应"）。
//=============================================================================

// ---- 语音起播延迟（原声明在后面"语音起播延迟"一节，阶段 4 提到这里）----
// 为什么要提上来 下面的 tick_target 要把这 1.5 秒算进去，而 Verilog
//   不允许前向引用（本项目在 TD 上踩过：前向引用会直接报错）。
//   100MHz 下 1 拍 = 10ns：1.5 秒 = 33'd150_000_000
// 现在是参数（默认值就是 150_000_000）—— 参数化只为仿真改小用
//     生产构建里参数取默认值，所以本行与改动前完全等价。
// 本次 32 -> 33 位：它要进 33 位的 dwell_wav_w 加法器（见下面）
localparam [32:0] AUDIO_DELAY_CYC = DWELL_AUDIO_DELAY_CYC;

// 播完之后的停顿（阶段 4 新增）：1 秒
//   参数 DWELL_AUDIO_EXTRA_CYC 默认 = 1.5s + 1s = 2.5s，
//   这里把它拆回"停顿" = extra - delay（默认正好是 1 秒）。
// 写成一个减法而不是两个独立参数 ：这样"起播延迟 + 停顿"这个
//     和一定等于 DWELL_EXTRA_CYC，仿真把两个参数同比例缩小时
//     公式的结构与生产完全一致（不会出现"少算一段"的假通过）。
localparam [32:0] DWELL_PAUSE_CYC =
    (DWELL_AUDIO_EXTRA_CYC >= DWELL_AUDIO_DELAY_CYC)
    ? (DWELL_AUDIO_EXTRA_CYC - DWELL_AUDIO_DELAY_CYC)   // 正常运行：2.5s - 1.5s = 1s
    : 33'd0;                                             // 仿真把 extra 调得比 delay 小 -> 钳 0
// 除音频时长之外要加的部分（起播延迟 + 停顿）—— 先在编译期加起来，
// 为什么 运行期写成 wav + 延迟 + 停顿 会串两级加法器，
//   直接进 tick_target 的建立时间预算；合成一个常量后只剩一级加法器。
// 本次 32 -> 33 位（1.5s + 1s = 2.5e8 本身装得下 32 位，
//     但它是 33 位加法器的一个输入，同宽才不会引入上下文宽度不一致）
localparam [32:0] DWELL_EXTRA_CYC = AUDIO_DELAY_CYC + DWELL_PAUSE_CYC;   // 2.5 秒

// USE_FAT_LOAD=0（旧通路）时把这两根悬空线选成常量 0：
//   写成"单条 assign + 编译期常量选择"，而不是两个 generate 分支各写一句
//   （后者会被 tools/verilog_multidriver_check.py 误判成多驱动，实测确认过）。
// 本次：位宽 32 -> 33，与 fat32_boot 的 o_wav_ticks 保持同宽
wire [32:0] wav_ticks_sel = (USE_FAT_LOAD != 0) ? fat3b_wav_ticks_w : 33'd0;
wire        wav_ok_sel    = (USE_FAT_LOAD != 0) ? fat3b_wav_ok_w    : 1'b0;

// 音频时长 + 起播延迟 + 停顿 = 按讲解长度决定的停留时长
// 本次加宽的核心一行 42.42 秒音频 -> 这里等于 44.92 秒 = 4.492e9 拍
// > 2^32-1（32 位会截断） 33 位装得下
wire [32:0] dwell_wav_w = wav_ticks_sel + DWELL_EXTRA_CYC;
// 只有"阶段 4 开着 + 时长确实来自 WAV"才用它，否则退回固定 5 秒
wire        dwell_use_wav_w = (USE_DWELL_FROM_WAV != 0) && wav_ok_sel;
wire [32:0] dwell_target_w  = dwell_use_wav_w ? dwell_wav_w : TICKS_5S;

//=============================================================================
// 本次新增：按档位选"停留拍数"
//   周期=0 讲解 -> dwell_target_w（= 讲解音频时长 + 1.5s + 1s，阶段 4 原式）
//   周期=1 3 秒 -> PERIOD_3S_TICKS
//   周期=2 5 秒 -> PERIOD_5S_TICKS
//   周期=3 8 秒 -> PERIOD_8S_TICKS
// 写成一条 assign + 2bit 选择（不是两个 generate 分支各写一句 assign，
//     那会被 tools/verilog_multidriver_check.py 的 MD-ASSIGN 误判成多驱动）。
// 为什么它就在 dwell_target_w 旁边 ：S4 那种"先算加法器、
//     再进比较器"的结构没变，只在比较器前多一个 2bit 选择器，时序开销最小。
// 本次 32 -> 33 位 三个 PERIOD_*_TICKS 参数保持 32 位，
// 在这里被零扩展到 33 位（值都 < 2^32，零扩展不会改变数值）
//=============================================================================
wire [32:0] tick_new_w =
    (PERIOD_STAGE_N <= 1)       ? dwell_target_w :
    (period_sel == 2'd0)        ? dwell_target_w :        // 讲解模式
    (period_sel == 2'd1)        ? PERIOD_3S_TICKS :       // 快速浏览 3 秒
    (period_sel == 2'd2)        ? PERIOD_10S_TICKS :      // 慢观看 10 秒
                                  PERIOD_10S_TICKS;       // 3'd3 手动浏览：永不 tick

//=============================================================================
// 2026-10-20 新增：按将要写入的档位算目标 / 下一个档位
//   tick_new_w 用的是当前period_sel。切档那一拍 period_sel 还没更新，
//   所以必须另算一份"用目标档位"的版本，否则 tick_target 会装成旧档位的
//   周期（下一张图才纠正 = 用户按了没反应）。
//     tick_menu_w : 设置模式 KEY_3 确认 —— 目标档位 = o_menu_sel
//   （tick_next_w 是早期"KEY_4 快捷切档"的遗留别名，当前没有任何消费者；
//    保留声明只为不改动网表，面积由综合器自行裁剪。）
//   两者都是纯组合选择，综合后就是两级 33 位 2 选 1，没有额外时序负担。
//=============================================================================
wire [1:0]  period_next_w = (period_sel == PERIOD_SEL_MAX) ? 2'd0 : (period_sel + 2'd1);
wire [1:0]  period_tgt_w  = setup_w ? o_menu_sel : period_next_w;

wire [32:0] tick_tgt_w =
    (PERIOD_STAGE_N <= 1)               ? dwell_target_w :
    (period_tgt_w == 2'd0)              ? dwell_target_w :
    (period_tgt_w == 2'd1)              ? PERIOD_3S_TICKS :
    (period_tgt_w == 2'd2)              ? PERIOD_10S_TICKS :
                                          PERIOD_10S_TICKS;
// 两个别名只为可读性：设置模式确认用 tick_menu_w，快捷切档用 tick_next_w
wire [32:0] tick_menu_w = tick_tgt_w;
wire [32:0] tick_next_w = tick_tgt_w;

// 2026-10-20 新增：自动轮播"被停下"的单拍脉冲（给顶层弹提示用）
//   条件：KEY_3 在看展模式下按下（设置模式里 KEY_3 是确认键，不弹提示），
//         且当前正在自动轮播、未被锁、张数够 —— 即"这一按真的把轮播关掉了"。
wire stop_banner_w = (USE_KEY_CTRL != 0) && key_auto_press && !i_block_page
                     && !setup_w && !key_locked_w && auto_play_en
                     && (img_found_count > 3'd1);
assign o_stop_banner = stop_banner_w;

// 2026-10-20 新增：KEY_3 按下的这一拍是不是"启动自动轮播"
//   用途：让 auto_cnt 在启动那一拍清零，"从当前图片开始"完整停留一个周期。
//   判据与 stop_banner_w 恰好互补（同一个按键、同一组前置条件，
//   唯一区别是看 auto_play_en 的当前值：它为 0 表示这一按是"开"）。
// 为什么不写成 start_now_w = key_auto_press && ... && !auto_play_en && !manual_w
//     只多一个 !manual_w：手动浏览档下 auto_play_en 根本不会翻成 1
//     （见 auto_play_en 那块），所以若不门住 manual_w，手动档按 KEY_3
//     会被判成"启动"，把计时器清零 —— 虽然手动档不靠计时器切图，
//     但这会让数码管/状态观感混乱，且违背"手动档就是纯手动"的语义。
wire start_now_w = (USE_KEY_CTRL != 0) && key_auto_press && !i_block_page
                   && !setup_w && !key_locked_w && !auto_play_en && !manual_w
                   && (img_found_count > 3'd1);

//=============================================================================
// 2026-10-23 新增：自动轮播运行中禁用 KEY_2 手动切图
//
//   需求：自动轮播跑着的时候，手动切换要锁掉；但操作者必须还能退出来，
//   所以只锁 KEY_2，KEY_3（启停）与 KEY_4（OSD 调参）一律照常。
//   若连 KEY_3 一起锁，用户就永远出不了自动模式 —— 这是这条需求最容易踩的坑。
//
//   auto_running_w 的判据：auto_play_en && !manual_w
//     auto_play_en 已经隐含了 !key_locked_all_w（锁定时被强制清 0）
//     和 !setup_w（设置模式同样清 0），所以不必再重复列条件。
//     manual_w 是冗余保护：手动浏览档下 auto_play_en 本就是 0，
//     写上去是为了让"手动档 = 纯手动，KEY_2 必须能用"这个语义显式可见。
//=============================================================================
wire auto_running_w = auto_play_en && !manual_w;

// KEY_2 被拒绝的那一拍（用于弹提示，让操作者知道"不是按键坏了"）
wire next_deny_w = (USE_KEY_CTRL != 0) && key_next_press && !i_block_page
                   && !setup_w && !key_locked_w && scan_done
                   && auto_running_w && (img_found_count > 3'd0);

// 两个时机共用同一句短提示
assign o_auto_hint = start_now_w | next_deny_w;

//=============================================================================
// 2026-10-23 c41 决议：不在此处实现"转场前 1 秒"
//
//   曾试过：
//     wire trans_soon_w = ... && ((tick_target - auto_cnt) <= TICKS_1S);
//   实测代价：sd_card_bmp 的 LUT 从 6334 涨到 6762（**+428**），
//   把全片 slices 推过 90%（8898 / 90.80%），TD 随即跳过 hold 修复，
//   留下 Hold WNS −20ps / 7 个违例端点 —— 这是"收益 50 LUT、代价 428 LUT"
//   的买卖，不划算，已撤回（端口也一并删掉，避免无谓的边界变化）。
//
//   现在滚动字幕的暂停只靠 trans_busy（转场本身的 2.352 秒），零额外逻辑。
//   若以后确实需要"转场前就停"，正确做法是：
//     ① 把 tick_target - TICKS_1S 在 tick_target 的每个赋值点同步算好存成
//        寄存器 tick_soon_r（不要每次比较都做减法）
//     ② 判断退化成 auto_cnt >= tick_soon_r（33 位比较，约 33 LUT）
//     ③ 必须重跑综合确认 slices 没再越过 90%
//=============================================================================

//=============================================================================
// 2026-10-20 二次修订：音频那一路的重启条件（修"讲解档按 KEY_3 后没有音频"）
//
// 现象（用户反馈）：SW1=1 里选"讲解模式"→ KEY_3 确认 → SW1=0 → 按 KEY_3 启动
//   自动轮播，图片会动，但当前这张图完全没有声音；要等切到下一张图才出声。
//
// 根因（两层，都在这里闭环）：
//   ① 设置模式里 auto_play_en 被 setup_w 支强制清 0 ⇒ audio_need_reload 的
//      "不该放音频"判据成立 ⇒ 它被置 1，而且是sticky（只在 audio_reload
//      那一拍才清 0，见下方 always 块）。
//   ② 原来 audio_reload 只由 img_commit_pulse 触发 = "换图才重启音频"。
//      所以出设置模式后按 KEY_3 启动，当前这张图永远等不到 img_commit，
//      audio_need_reload 一直是 1 ⇒ audio_gate_w 恒 0 ⇒ 完全无声。
//
// 修法：把 KEY_3 的"启动那一拍"并进 audio_reload。
// · 它清掉 sticky 的 audio_need_reload ⇒ audio_gate_w 立刻抬起
//   · 它装载 wav_rst_cnt ⇒ wav_player / wav_src_fat 同步重启，重新按簇链读一遍
// WAV 头和数据（不重启的话 wav_player 还停在上一轮"放完"的状态）
//   · 它清 audio_dly_cnt ⇒ 图片早已在屏上，1.5s 后即起播（不会因为没有新
// display_valid 而永远等不到）
// 只在讲解档生效（period_is_explain_w），定时档/手动档照旧不出声
// USE_KEY_CTRL=0（一键回退）时这一项自动消失，行为与旧版一致
//=============================================================================
assign audio_reload = (USE_KEY_CTRL != 0)
                      ? (((img_commit_pulse || start_now_w) && (USE_FAT_WAV_EFF != 0)
                          && period_is_explain_w && !key_locked_all_w)
                         // 手动档：KEY_4 按下"开"那一拍也要重启这一路，
                         //   否则 wav_player 还停在上一张图的 WAV（甚至已播完停住），
                         //   表现为"按了没反应"或"放的是别的图的语音"。
                         || (man_wav_reload_w && (USE_FAT_WAV_EFF != 0) && !key_locked_all_w))
                      : (img_commit_pulse && (USE_FAT_WAV_EFF != 0));

// auto_tick 用寄存器 vs 寄存器的等值比较，组合深度可控。
assign auto_tick  = (auto_cnt == tick_target);
// 按键交互新增：这个档位是不是"讲解模式"（= 允许音频）
assign o_period_is_explain = period_is_explain_w;
assign next_from_current = next_index_limited(img_idx, img_found_count);
// 手动翻页方向选择：两个 pending 互斥（同一时刻只可能有一个被置起，见按键段），
// 所以这里直接用 prev 优先，不需要额外的仲裁。
assign step_from_current = prev_req_pending ? prev_index_limited(img_idx, img_found_count)
                                            : next_from_current;

//------------------------------------------------------------------------------
// 手动档音频：按一下播、再按一下停
//   停止条件（任何一条成立即回到"未请求"）：
//     · 退出手动档（切到讲解/3s/10s）—— 那几档有自己的自动通道
//     · SW4 锁定 / 卡重新初始化
//     · 翻页（step_req_w）—— 否则换到下一张时上一张的语音还在响
//   按键那一支用 man_audio_playing_w 判断方向：在播则停，没在播则播（含播完后重播）
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        wav_playing_sy <= 2'b00;
    end else begin
        wav_playing_sy <= {wav_playing_sy[0], wav_playing};
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        man_audio_en <= 1'b0;
    end else if (!manual_w || key_locked_all_w || !sd_init_done) begin
        man_audio_en <= 1'b0;
    end else if (man_audio_key_w) begin
        man_audio_en <= ~man_audio_playing_w;
    end else if (step_req_w) begin
        man_audio_en <= 1'b0;
    end
end

// 把当前显示中的图片序号引出去给数码管显示（必须放在 img_idx 声明之后）
assign o_img_idx = img_idx;
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign state_code = state_code_i;

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_period (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_period),
    .press_pulse(key_period_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

// 上一张：与 next_index_limited 镜像对称，按"实际找到的张数"决定回绕点
//   count=2 时 0 -> 1；count=3 时 0 -> 2；count=4 时 0 -> 3
// 写成"count-1"的算术形式而不是 4 个分支的 case：
//   两种写法综合出来的功能相同，但 case 形式在这里要多一片 3 输入译码，
//   当前 slices 卡在 90% 边缘（>90% TD 会跳过 hold 修复），能省则省。
// 只有 1 张（或 0 张）时恒返回 0 —— 调用侧另有 (img_found_count > 1) 门控，
// 这里再兜一层，避免万一被别处误用给出非法序号。
function [1:0] prev_index_limited;
    input [1:0] cur;
    input [2:0] count;
    reg   [1:0] last;
    begin
        last = count[1:0] - 2'd1;
        prev_index_limited = (cur == 2'd0) ? last : (cur - 2'd1);
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        period_sel            <= 2'd0;      // 本次：复位默认档位 = 0 = 讲解模式
        tick_target           <= TICKS_5S;  // 复位先装兜底 5 秒；首图提交那一拍按档位重装
        // 本次：audio_need_reload / audio_need_u 的复位不在这里
        //   它们由文件靠下那个专门的 always 块驱动（一个 reg 只能一个 always，
        //   这是 TD 的红线 HDL-8007）。复位值在那边给：need_reload=1、need_u=0。
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        prev_req_pending <= 1'b0;
        load_busy             <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        load_abort            <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
        img_commit_pulse      <= 1'b0;
        sub_job_pulse         <= 1'b0;
        sub_busy_r            <= 1'b0;
        sub_arm               <= 3'd0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        img_commit_pulse <= 1'b0;      // 阶段 3c：默认单拍脉冲
        sub_job_pulse    <= 1'b0;      // S2：默认单拍脉冲

        // S2：字幕装载占位标志（与上面那几条赋值同一个 always 块 ——
        // 一个 reg 只能由一个 always 驱动，这是 TD 的红线）
        if (sub_job_pulse)
            sub_arm <= 3'd4;
        else if (sub_arm != 3'd0)
            sub_arm <= sub_arm - 3'd1;

        if (sub_job_pulse)
            sub_busy_r <= 1'b1;
        else if (sub_busy_r && (sub_arm == 3'd0) && !sub_busy_m && fat_load_ready)
            sub_busy_r <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            // 本次：卡重新初始化 = 整条音频/轮播那一路作废重来
            //   档位 period_sel 故意不清 —— 那是用户的选择，与卡无关
            //   （audio_need_reload / audio_need_u 由下面的专用 always 块驱动，
            //     它靠 period_is_explain_w / key_locked_w 自己就能收敛，不用在这里清）
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            o_trans_skip          <= 1'b0;
            o_trans_force_push    <= 1'b0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            prev_req_pending <= 1'b0;
            load_busy             <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            load_abort            <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            display_valid         <= 1'b0;
            // S2：卡重新初始化时字幕装载一并作废
            sub_job_pulse         <= 1'b0;
            sub_busy_r            <= 1'b0;
            sub_arm               <= 3'd0;
        end else begin
            // 扫描阶段缓存前 4 张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住 bmp_read 已经把源图送完 FIFO，但还不能切显示，得等整帧写完
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // 记住写帧完成脉冲，避免“先写完后源结束”导致脉冲丢失而卡死
            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            // 只有“源图送完 + 整帧写完 + 转场模块放行”都满足，才提交新图
            // i_trans_go 是许可信号（默认 0）
            //   原设计用 !i_trans_hold，但 hold 跨域同步慢 80ns，
            //   而本条件与 o_trans_pending 同一拍成立 -> 常常抢跑，
            //   表现为自动轮播时偶发地没有转场。许可式握手彻底消除这个竞态。
            if (load_busy && source_done_seen && write_done_seen && i_trans_go) begin
                load_busy             <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                disp_buf_idx          <= pending_buf_idx;
                img_idx               <= load_idx;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
                // 阶段 3c：图片"出现"这一拍的单拍脉冲
                //   音频那一路（wav_player / wav_src_fat）靠它重启一次：
                //   换成这张图配对的那段 WAV、从文件头重新读、重新起算
                //   1.5 秒起播延迟。这是"每张图都播自己的讲解"的触发点。
                // 为什么用这一拍当"零点" 它与 display_valid 置位、
                //     与 auto_cnt 的计时零点是同一拍（见上面"阶段 4：停留时长"
                //     的推导），所以 tick_target 的 1.5 + 音频 + 1 秒三段
                //     与真实听到的完全对齐。
                img_commit_pulse      <= 1'b1;
                // 阶段 4 / 3c：装载"停留时长"
                //   为什么放在这里（而不是 bmp_ready 那一拍）：
                //   1) 换算模块（wav_dwell_calc）是两级流水，输出比它输入
                //      稳定晚 2 拍；bmp_ready 只比它晚 1 拍，那拍到手的还会是
                //      兜底值（5 秒），功能就悄悄失效了。
                //   2) 更要紧的是时序：WAV 的字节数（以及 3c 的"选哪一张图"）
                //      在"走完文件表"那一拍就写定了，而图片提交要等一整张图
                //      读完（约 0.85 秒 = 8.5e7 拍）。于是这条路径在硬件上
                //      天然是"源数据早已稳定"的多周期路径。
                //   3) 语义上也更对：停留时长是从"图片出现/音频起播"那一刻
                //      开始用的，在那之前 auto_cnt 被 first_image_committed
                //      门着，用不到它。
                // 阶段 3c 的改动：从"只装一次"变成"每次提交都装一次"
                //     3c 打开时 dwell_target_w 已经是"这张图配对的那段 WAV"
                //     算出来的（fat32_boot 的 USE_PAIR_DWELL=1），
                //     所以每张图的停留时长各不相同。
                //     3c 关闭时条件退化成 !first_image_committed，
                //     与阶段 4 上板验证过的行为逐拍一致。
                // 本次（按键交互）的改动
                //     装载源从 dwell_target_w 改成 tick_new_w：
                //       讲解档 -> 还是 dwell_target_w（阶段 4 原式，一字未改）
                //       3 / 10 秒档 -> 固定拍数（这些档位本来也不放音频）
                // 为什么用 tick_new_w 而不是在别处再判一次档位
                //       保证"档位"只有一个真相来源，加载点也只有这一处
                //       （切档时另有一处立即重装，见下面"轮播周期切档"）。
                if ((USE_FAT_WAV_EFF != 0) || (!first_image_committed))
                    tick_target       <= tick_new_w;
                // 本次：自动轮播"已自动切几张"的计数（本图刚提交，先清 0）
                //   只有 USE_AUTO_LIMIT > 0 时才被下面判定，默认配置下是死逻辑
            end else if (load_busy) begin
                // 装载看门狗：读坏图/卡抽风时强制释放 busy，恢复按键响应。
                //
                // 阈值必须明显大于"load_busy 为 1 的全部正常时间"
                // 注意：这个计数器数的不只是"读卡时间"
                //   load_busy 从发起装载一直保持到 i_trans_go（转场放行才提交），
                //   所以"转场动画本身"也被算在这段时间里（见下面的提交条件
                //   L1006：load_busy && source_done_seen && write_done_seen && i_trans_go）。
                //
                //   预算（sd_card_clk = 100 MHz，1 拍 = 10 ns）：
                //     ① 读卡：1 张 640x480 BMP = 921654 字节 = 1801 扇区，
                //        SPI 25MHz 下约 470us/扇区 -> ≈ 0.85 秒；
                //        加 FIFO 握手 + SDRAM 整帧写入 -> 顶到 0.9~1.0 秒
                //     ② 转场：页面卷曲 CURL_LAST=69 × CURL_FPS=2
                //        = 140 帧 × 16.8ms = 2.352 秒（上下推移约 1.008 秒）
                //     ③ 提交前还要等 vs_fall 发布 + 两级同步，≈ 2 帧 = 0.034 秒
                //   合计 ≈ 3.35 秒（卷曲最坏情况）。
                //
                // 阈值下限的由来（2026-10-18 实测）
                //     原阈值 100_000_000 拍 = 1.000 秒 < 读卡时间本身，必炸；
                //     后来放宽到 300_000_000 拍 = 3.000 秒，在"转场 1.176 秒"时
                //     合计 2.18 秒还能过，但把卷曲放慢到 2.352 秒后
                //     合计 3.35 秒 > 3.000 秒 -> 每张图都会在转场末尾被
                //     看门狗掐掉 load_abort，表现为"卷曲播到尾巴就卡住、图不换"。
                //     所以这里给到 5 秒（坏卡仍能在 5 秒内自恢复）。
//
//   ★ c71：500_000_000 -> 750_000_000（5 s -> 12 s @62.5MHz，实际是 8 s -> 12 s）
//     原因：c70 把 sd_card_clk 从 100MHz 降到 62.5MHz，这个计数器实际变成 8 秒；
//     而 1280×960 的图要读 4.1~5.4 秒，再加 2.35 秒卷曲，只剩 0.25~1.55 秒余量。
//     放宽到 12 秒后余量 >= 4 秒，代价只是坏卡自恢复变慢。
                if (load_timeout_cnt > 32'd750_000_000) begin
                    // 超时（例如读到坏图或截断的文件导致死锁），强制释放 busy 以恢复按键响应
                    load_busy             <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_abort            <= 1'b1;
                end else begin
                    load_timeout_cnt <= load_timeout_cnt + 32'd1;
                end
            end else begin
                load_timeout_cnt <= 32'd0;
            end

            // 上电后自动发起一次“扫描前 4 张 BMP”
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                // 本次：上电后自动轮播是"停"还是"跑"由参数决定
                //   默认 AUTO_PLAY_ON_BOOT=0 -> 停在第一张，等 KEY_3 启动
                auto_play_en          <= ((USE_KEY_CTRL != 0) && (AUTO_PLAY_ON_BOOT != 1'b0)) ? 1'b1 : 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                prev_req_pending <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
            end else begin
                //=================================================================
                // 按键交互（KEY_2 翻页 / KEY_3 启停 / KEY_4 走 OSD 调参）
                //
                //   SW4(i_auto_mode) = 1：正常工作，三键有效
                //   SW4 = 0：锁定，三键全部失效（统一由 key_locked_w 门控）
                //
                //   ① KEY_2 手动翻页
                //      自动轮播开着也能翻：手动分支优先级高于自动，翻页后计时器重算，
                //      不会"翻完立刻又被自动切走"。
                //      i_block_page / setup_w 打开时 KEY_2 另作他用（OSD 调参 / 菜单光标），
                //      必须门住，否则一次按键会同时移动光标和切图。
                //   ② KEY_3 启动 / 停止自动轮播（电平翻转，不是点动）
                //   ③ KEY_4 不参与翻页与切档；只在 SW3=1 时由 osd_ctrl 当"参数 -"
                //
                //   自动轮播运行中只拦 KEY_2（auto_running_w），KEY_3 照常，随时能停下；
                //   被拦时会弹「自动轮播中」提示（o_auto_hint），不是静默无反应。
                if (key_next_press && !i_block_page && !key_locked_w && !setup_w && scan_done
                    && !auto_running_w && (img_found_count > 3'd0))
                    next_req_pending <= 1'b1;

                // KEY_3 在【手动浏览档】下 = 上一张（本次新增）
                //   为什么带上 manual_w：其它档位 KEY_3 仍是自动轮播启停，
                //   那是唯一入口，不能抢；而手动档下 auto_play_en 恒 0、
                //   KEY_3 原本按下去什么都不做，正好拿来做反向翻页。
                //   与下面那一支通过 manual_w / !manual_w 天然互斥，不会同拍双触发。
                //   (img_found_count > 1)：只有一张图时"上一张"没有意义，直接不响应。
                if (manual_w && key_auto_press && !i_block_page && !key_locked_w && !setup_w
                    && scan_done && (img_found_count > 3'd1))
                    prev_req_pending <= 1'b1;

                // KEY_3：启动 / 停止自动轮播
                //   USE_KEY_CTRL=0 时本段折叠成 auto_play_en <= i_auto_mode
                //
                //   当前语义：按一下开、再按一下停。
                //     · 启动那一拍只清 auto_cnt —— 本张图从这一刻重新计时，不切图
                //       （"从当前图片开始轮播"，不是按下去就换掉当前这张）
                //     · 停止那一拍只停，不切图、不改 auto_cnt 的有效值
                //   设置菜单 / 锁定模式 / 手动档下强制 auto_play_en=0：
                //   这些状态下 KEY_3 不是启停键（菜单里是确认键）。
                if (USE_KEY_CTRL == 0) begin
                    auto_play_en <= i_auto_mode;
                end else if (setup_w) begin
                    // 2026-10-20：设置模式（SW1=1）强制停止自动轮播
                    //   此时 KEY_3 是"确认键"，不是"启停键"，绝不能翻 auto_play_en。
                    auto_play_en <= 1'b0;
                end else if (key_locked_w) begin
                    // 锁定模式：强制停止自动轮播（并让计时器清零）
                    auto_play_en <= 1'b0;
                end else if (manual_w) begin
                    // 2026-10-20 新增：手动浏览档强制停止自动轮播
                    //   场景：轮播正在跑（auto_play_en=1）时，在设置菜单里把档位改成手动浏览，
                    //   若不在这里清 0，auto_play_en 会一直是 1 —— 屏幕上是"手动"，
                    //   内部却是"自动开启"，切回其它档立刻开始切图，观感突兀。
                    //   这一支与 setup_w / key_locked_w 同一层级：都是"外部状态
                    //   直接否决自动轮播"，且优先级低于 KEY_3 点动。
                    auto_play_en <= 1'b0;
                end else if (key_auto_press && !i_block_page && !manual_w && (img_found_count > 3'd1)) begin
                    // 按一下开、再按一下停
                    // 2026-10-20：条件里加了 !manual_w
                    //   手动浏览档（period_sel=3）下，KEY_3 按下去什么都不做：
                    //     - 不启动自动轮播（这是"手动"的定义）
                    //     - 也不弹"已停止"提示（stop_banner_w 要求 auto_play_en=1，
                    //       而手动档下 auto_play_en 恒 0，自然不会弹）
                    //   如果不门住，auto_play_en 会被翻成 1，虽然
                    //   ① audio_gate_w 有 !manual_w 挡住声音
                    //   ② 自动切图分支有 !manual_w 挡住切图
                    //   但内部状态与观感都会混乱（例如切回非手动档后
                    //   轮播"莫名其妙"已经在跑）。手动档就是纯手动，不留这种暗态。
                    auto_play_en <= ~auto_play_en;
                    // 2026-10-20：这里不再清 auto_cnt
                    //   同一个 always 块里下面的自增支会把它覆盖掉（后赋值赢），
                    //   写在这里是死代码，只会误导后来的人。
                    //   真正生效的清零在下面 auto_cnt 块的 start_now_w 分支。
                end

                // 自动播放周期计数：只有当前没有写图任务时才计时
                if (USE_KEY_CTRL != 0) begin
                    // 锁定 / 张数不足 / 设置模式 -> 计时器必须归零，否则恢复时会"接着上次的数"
                    // 2026-10-20：把 setup_w 也加进来 —— 设置模式下计时器必须是 0，
                    //     否则 SW1 拨回 0 的那一刻 auto_cnt 还停在上次的值，
                    // 一按 KEY_3 没数几下就到点，不是"从当前图片开始"
                    if (key_locked_all_w || (img_found_count <= 3'd1))
                        auto_cnt <= 33'd0;
                    // 2026-10-20 新增：KEY_3 启动那一拍重新计时
                    //   用户要求"从当前图片开始自动轮播"。
                    // 为什么写在这里、而不是在上面 auto_play_en 那一块里写
                    //     这两条赋值在同一个 always 块里，Verilog 语义是
                    //     "后赋值的赢"。上面那块先写 auto_cnt <= 0，
                    //     接着下面这条 else if 又写 auto_cnt <= auto_cnt + 1
                    //     （条件是"图片正在显示"，按 KEY_3 时它必然成立）
                    // -> 清零被吃掉，本张的剩余时间毫无变化
                    //     所以清零必须写在这条自增的前面且优先级更高。
                    //   时机判据 start_now_w：KEY_3 按下 && 当前是"停"（所以本次是启动）
                    //     && 其它前置条件齐全（与自增支同组，保证确实能数起来）。
                    else if (start_now_w)
                        auto_cnt <= 33'd0;
                    // 2026-10-12 修正：递增条件里不再要求 auto_play_en
                    //   症状（用户实测）：讲解自然播完后才按 KEY_3 启动自动轮播，
                    // 还要再等一整个周期（01 号 44.9 秒）才切图
                    //   根因：原来"没启动自动 -> auto_play_en=0 -> 上面第一支把
                    //     auto_cnt 清 0"，所以讲解播完时计数器还是 0；
                    // 按下 KEY_3 才从 0 开始数 -> 必然等满一个 tick_target
                    //   现在：只要"图片正在显示"（本次计时的全部前置条件都成立）
                    // 就一直数，与"自动轮播有没有启动"无关
                    //   配合下面那个 !auto_tick：数到 tick_target 后停在那个值
                    //     （不是新逻辑 —— 原来靠"到了就清 0"停，现在靠"到了不再加"停，
                    // 都是停在 tick_target，没有多/少一拍）
                    // 效果：讲完再按 KEY_3 -> auto_tick 当拍就成立 -> 立刻切图
                    // 刚显示就按 KEY_3 -> 计数器接着数 -> 等剩余时间（合理）
                    // 其余条件（scan_done / display_valid / !load_busy /
                    // !i_trans_busy / first_image_committed / 张数）一个都没动
                    //   （张数条件挪到上面第一支里判，这里是 else if，等价）
                    else if (scan_done && display_valid && !load_busy && !i_trans_busy && first_image_committed && (img_found_count > 3'd1)) begin
                        // 停下来靠这一句 数到 tick_target 就不再加：
                        // 没启动自动时没人清它 -> auto_cnt 就停在 tick_target
                        //   （自动切图那一支到点会把它清 0，见下面 o_trans_skip<=1'b0 那段）
                        if (!auto_tick)
                            auto_cnt <= auto_cnt + 33'd1;
                    end else begin
                        auto_cnt <= 33'd0;
                    end
                end else if (scan_done && auto_play_en && display_valid && !load_busy && !i_trans_busy && first_image_committed && (img_found_count > 3'd1)) begin
                    // 回退支：与改动前逐字一致的旧条件
                    if (auto_tick)
                        auto_cnt <= 33'd0;
                    else
                        auto_cnt <= auto_cnt + 33'd1;
                end else begin
                    auto_cnt <= 33'd0;
                end

                // 首图自动加载到 buffer0
                // S2：!sub_busy_r 把三个启动分支都门住（理由见 sub_busy_r 注释）
                if (scan_done && !first_image_committed && bmp_ready && !load_busy && !sub_busy_r && (img_found_count != 3'd0)) begin
                    load_idx         <= 2'd0;
                    load_sector      <= img_sector0;
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                    // 必须显式赋值 o_trans_skip 是黏性寄存器，
                    //   任何一个装载分支漏设，它就会带着上一次的值跑，
                    //   表现为"这次自动轮播却用了手动的立即切换"。
                    o_trans_skip     <= 1'b1;   // 第一张图不转场
                    o_trans_force_push <= 1'b0;
                end
                // 手动下一张优先：写到“非当前显示”的另一块 buffer
                // 本次：条件里的 !i_auto_mode 已去掉 —— SW4=1（自动轮播）时手动也能翻页
                //   （next_req_pending 只可能由"未被锁住的 KEY_2"置起，见上面）
                else if (scan_done && bmp_ready && display_valid && !load_busy && !sub_busy_r && step_req_w && (img_found_count != 3'd0)) begin
                    // 本次：手动翻页（下一张 / 上一张）共用这一条支路，
                    //   方向由 step_from_current 决定，其余握手与原来完全一致。
                    load_idx         <= step_from_current;
                    load_sector      <= sector_lut(step_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                    o_trans_skip     <= 1'b0;   // 手动翻页：不再硬切
                    o_trans_force_push <= 1'b1; // 手动翻页固定用推移（约 1.0 s）
                end
                // 自动播放下一张：同样写到“非当前显示”的另一块 buffer
                // 本次：这里不加!next_req_pending
                //   理由：手动请求只把 next_req_pending 置一拍，而本次计时到点也
                //   只有一拍；若两个分支都加互斥，同拍两者都成立时谁都不切，
                //   表现为"偶尔少切一张"。保持"手动分支优先级更高"即可 ——
                //   同拍成立时上面的分支赢，本次自动切换被吃掉，下一拍
                //   auto_cnt 已被清 0，重新计时，行为确定。
                // 2026-10-20：自动切图分支加两个否决项
                //   ① setup_w   —— SW1=1 设置模式下绝不自动切图（KEY_3 是确认键）
                //   ② manual_w  —— 手动浏览档（period_sel=3）永不自动切图
                //   两者的判断都放在条件里（组合），不额外加流水级。
                else if (scan_done && bmp_ready && display_valid && !load_busy && !sub_busy_r && auto_play_en && !key_locked_w && !setup_w && !manual_w && auto_tick && (img_found_count > 3'd1)) begin
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    auto_cnt         <= 32'd0;
                    o_trans_skip     <= 1'b0;   // 自动轮播：走转场
                    o_trans_force_push <= 1'b0; // 自动轮播听 SW2（推移 / 卷曲）
                end
                //=================================================================
                // S2：字幕掩膜条装载（复用 bmp 路 + 同一个 bmp_load_fat）
                //
                //   触发时机 = 图片"提交"（真正显示出来）那一拍之后一拍。
                //   为什么这个时机是安全的（三件事同时成立）：
                //     ① 图片装载早已结束（提交是在整帧写完 + 转场放行之后）；
                //     ② 音频还没起播 —— wav_play_go 要等 AUDIO_DELAY_CYC
                //        = 1.5 秒（150,000,000 拍 @100MHz），而本条字幕只要
                //        25 个扇区 x 约 470us = 约 11.3ms，物理上不可能让
                //        声音断续（那时声音还没出声）；
                //     ③ 下一次图片装载最早也要等 dwell（>= 1.5 秒 + 音频时长）
                //        或手动按键，而且那三个分支都被 !sub_busy_r 门过。
                // 不设 load_busy 字幕不占图片双缓冲、不参与转场握手，
                //     所以不能用 load_busy（那会让转场模块以为有新图）。
                // 不新增第 4 路仲裁 走的就是 bmp 那一路。
                //=================================================================
                else if ((USE_SUB_EFF != 0) && img_commit_pulse && sub_sel_has_m && !load_busy) begin
                    load_sector      <= {28'd0, sub_sel_idx_w};   // 复用"表下标"这个语义
                    sub_job_pulse    <= 1'b1;
                    load_start_pulse <= 1'b1;
                end

                //=================================================================
                // 2026-10-20 新增：SW1 设置模式（setup）下的菜单操作
                //
                //   物理键：KEY_2 = key_next_press、KEY_3 = key_auto_press
                //   设置模式（SW1=1）下：
                //     KEY_2 -> menu_sel 循环 0->1->2->3->0（只动光标，不动 period_sel）
                //     KEY_3 -> 确认：period_sel <= menu_sel，并按新档位重装计时器
                //   离开设置模式（SW1 回落 0）时不做任何事：
                //     period_sel 已是确认过的值，等用户按 KEY_3 才启动轮播。
                // 光标与档位分开两个寄存器 ：菜单里来回翻不会改变实际轮播档位，
                //     只有按 KEY_3 才真正生效 —— 这就是"选择 / 确认"的语义。
                // 为什么这里门不到 KEY_2/KEY_3
                //     设置模式下 key_locked_all_w=1，主状态机的"手动翻页"与
                //     "KEY_3 启停"两支都被门住，所以不会与菜单操作冲突。
                //=================================================================
                if (USE_PERIOD_STAGE == 0) begin
                    o_menu_sel <= 2'd0;                       // 回退：只有讲解档
                end else if (setup_w) begin
                    if (key_next_press)
                        o_menu_sel <= (o_menu_sel == PERIOD_SEL_MAX)
                                      ? 2'd0
                                      : (o_menu_sel + 2'd1);
                end else begin
                    // 看展模式：光标跟随已确认档位，下次进设置模式从当前位置开始
                    o_menu_sel <= period_sel;
                end

                //=================================================================
                // 轮播周期：只允许在 SW1=1 设置模式里用 KEY_3 确认修改
                //
                // 2026-10-20 二次修订（用户要求）
                //     删掉了原来"KEY_4 快捷切档"那条支路 —— KEY_4 从此
                //     没有任何切换轮播周期的能力，SW1=0 时按 KEY_4 无动作。
                //     唯一的改档入口：SW1=1 -> KEY_2 移光标 -> KEY_3 确认。
                //     （KEY_4 的消抖实例与 o_key3_press 输出仍然保留，
                //       那是 osd_ctrl 在 SW3=1 时调音量/对比度用的老功能，不受影响）
                //
                //   确认那一拍必须同时做三件事：
                //     ① period_sel 更新（取光标位置 o_menu_sel）
                // ② tick_target 立即按新档位重装关键
                //        注意：必须用新值算目标（tick_menu_w），
                //        用 tick_new_w 会拿到旧档位的目标（老代码的隐性 bug）
                //     ③ auto_cnt 清零，让新档位的停留从切换那一刻重新计时
                //
                //   USE_PERIOD_STAGE=0 时本段折叠掉（PERIOD_STAGE_N=1，永远停在档位 0）
                //=================================================================
                if (USE_PERIOD_STAGE == 0) begin
                    // 一键回退：只有讲解一个档位
                    period_sel <= 2'd0;
                end else if (setup_w) begin
                    // 设置模式：只有 KEY_3 确认才真正改档位；KEY_4 不再切档
                    if (key_auto_press) begin
                        period_sel  <= o_menu_sel;
                        tick_target <= tick_menu_w;
                        auto_cnt    <= 33'd0;
                    end
                end

                //=================================================================
                // 手动浏览（自动轮播停止时）：翻页完全由 key_next_press 决定。
                // SW4=0（锁定）时 KEY_2 已被门住。
                //
                // 2026-10-12 修正注释 ：原来这里写的是
                // "auto_play_en=0 -> auto_cnt 恒 0" 已过时
                // 现在 auto_cnt 在"图片显示中"一直数（与 auto_play_en 无关），
                // 数到 tick_target 就停住（靠 !auto_tick 那一句）
                // 目的：讲解自然播完后按 KEY_3 能立刻切图
                // 手动翻页自身的路径不受影响 —— 它不读 auto_cnt
                //（手动分支只用 key_next_press + next_req_pending）
                //=================================================================
            end
        end
    end
end

// ===================== 图片装载源：二选一（3b 总开关）=====================
//   USE_FAT_LOAD=1 -> 3b：FAT 直读通路（fat32_boot + bmp_load_fat）
//   USE_FAT_LOAD=0 -> 旧通路（bmp_read 扫扇区），与 3a 逐字相同
//
// 两个分支里驱动同一批"结果线"的东西，全部是例化输出端口
//   这一点很重要，别为了省事改成 assign：
//     tools/verilog_multidriver_check.py 是纯文本扫描，它不认识
//     generate 分支，会把"两个分支各写一句 assign 同一个 net"
//     误判成多驱动（本工程实测确认过）。用例化输出端口就不会被误判；
//     而综合器本来也只展开条件成立的那一个分支，不存在真多驱动。
generate
if (USE_FAT_LOAD != 0) begin
    // ================= 3b：FAT 直读通路 =================
      fat32_boot #(
          // 文件表容量：10 个槽位（实际可收纳 9 个文件，见 fat32_dir 的表深度说明）。
          //   实际用法是 4 张 BMP + 4 段配对 WAV = 8 个，留 1 个余量。
          //   原来是 16（可收纳 15 个），表深度直接决定 fat32_dir 里 4 张表的
          //   组合读 mux 规模，降到 10 可省出一次布局优化所需的面积。
          .MAX_FILES    (10),
        .TO_LIMIT     (32'd2000000),
        .TO_LIMIT_ALL (32'd200000000),
        .MAX_BMPS  (4'd4),
        // 阶段 4：换算停留拍数要知道系统时钟频率（透传给内部 wav_dwell_calc）
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        // 阶段 3c：停留时长改用"这张图配对的那段 WAV"
        //   0 = 旧行为（表里第一个 WAV），1 = 配对（见 fat32_boot.v 文件头）
        .USE_PAIR_DWELL (USE_FAT_WAV_EFF)
    ) u_fat32_boot (
        .clk                (clk),
        .rst                (rst),
        .i_sd_init_done     (sd_init_done),
        .i_old_busy         (fat_old_busy),
        // 名单放行的时机：用 sd_card_bmp 原有的"首次扫描启动"脉冲。
        //   复用它的原因见 fat32_boot.v 文件头（必须先让主状态机清完
        // img_found_count / img_sector0..3，名单才不会被清掉）
        .i_list_go          (scan_start_pulse),
        .o_rd_req           (fat_sec_read),
        .o_rd_lba           (fat_sec_read_addr),
        .i_data             (fat_sec_read_data),
        .i_data_valid       (fat_sec_read_data_valid),
        .i_data_end         (fat_sec_read_end),
        // 超时修复：把"卡现在归 FAT"告诉它，排队期间不计超时
        .i_granted          (sd_grant_fat),
        // 文件表读口：下标由装载器给（走表阶段内部会自己切成 widx）
        .i_tbl_idx          (fatld_tbl_idx),
        .o_tbl_name         (fat_boot_tbl_name),
        .o_tbl_cluster      (fat_boot_tbl_cluster),
        .o_tbl_size         (fat_boot_tbl_size),
        .o_tbl_attr         (fat_boot_tbl_attr),
        .o_fat_lba          (fat_boot_fat_lba),
        .o_data_lba         (fat_boot_data_lba),
        .o_sec_per_clus     (fat_boot_spc),
        .o_spc_lg2          (fat_boot_spc_lg2),
        // 名单接口：值 = FAT 文件表下标（不是扇区号！见文件头说明）
        .o_scan_done        (scan_done),
        .o_scan_found_valid (scan_found_valid),
        .o_scan_found_sector(scan_found_sector),
        .o_scan_found_total (scan_found_total),
        // 名单就绪即当"图片源 ready"用（主状态机的首次启动靠它）
        .o_ready            (bmp_ready),
        // 解析结果：接回 3a 原来那几根线，数码管显示逻辑不用改
        .o_count            (fat_scan_count),
        .o_err_code         (fat_scan_err),
        .o_done             (fat_scan_done),
        .o_busy             (fat_scan_busy),
        .o_cnt_digit        (fat_scan_cnt_digit),
        .o_cnt_ovf          (fat_scan_cnt_ovf),
        .o_bmp_cnt          (fat3b_bmp_cnt_w),
        .o_wav_found        (fat_boot_wav_found),
        .o_wav_lba          (fat_boot_wav_lba),
        // 阶段 4：第一个 WAV 的时长（拍）与"是否来自真 WAV 大小"
        // 阶段 3c：USE_PAIR_DWELL=1 时这两个值就是"i_wav_sel 那张图
        // 配对到的 WAV"的时长 → 每张图的停留时长各不相同
        .o_wav_ticks        (fat3b_wav_ticks_w),
        .o_wav_dur_ok       (fat3b_wav_ok_w),
        // 阶段 3c：按序号配对的结果（给音频那一路读文件用）
        //   选择下标 = 正在提交的那张图（load_idx），与 tick_target 同源，
        //   保证"这张图的时长"和"这张图播的语音"永远是同一段 WAV。
        .i_wav_sel          (load_idx),
        .o_wav_sel_clu      (wav_sel_clu_w),
        .o_wav_sel_size     (wav_sel_size_w),
        .o_wav_sel_lba      (wav_sel_lba_w),
        .o_wav_sel_has      (wav_sel_has_w),
        // S2：字幕（0N.SUB）的配对结果 —— 与上面 WAV 共用同一个选择下标
        // i_wav_sel = load_idx（正在提交/装载的那张图）
        .o_sub_sel_idx      (sub_sel_idx_w),
        .o_sub_sel_has      (sub_sel_has_w),
        .o_sub_cnt          (sub_found_w),
        .o_phase            (),
        .o_dbg_root_lba     (fat_dbg_root_lba),
        .o_dbg_part_lba     (fat_dbg_part_lba),
        .o_dbg_data_lba     (fat_dbg_data_lba),
        .o_dbg_fat_lba      (fat_dbg_fat_lba),
        .o_dbg_secs         (fat_dbg_secs),
        .o_dbg_part_type    (fat_dbg_part_type),
        .o_dbg_root_b0      (fat_dbg_root_b0),
        .o_dbg_root_b1      (fat_dbg_root_b1),
        .o_dbg_rx_bytes     (fat_dbg_rx_bytes)
    );


    bmp_load_fat #(
        .TO_LIMIT       (32'd2000000),
        .TO_LIMIT_ALL   (32'd200000000),
        .CHAIN_HARD_MAX (32'd1048576)
    ) u_bmp_load_fat (
        .clk            (clk),
        .rst            (rst),
        .ready          (fat_load_ready),
        .state_code     (state_code_i),
        .sd_init_done   (sd_init_done),
        .bmp_width      (bmp_width),
        .bmp_height     (bmp_height),
        // 装载请求：与 bmp_read 同一套脉冲；load_sector 的低 4 位
        // 在这里是"FAT 文件表下标"（主状态机存的就是扫描名单给的值）
        .load_start     (load_start_pulse),
        .load_abort     (load_abort),
        .load_tbl_idx   (load_sector[3:0]),
        .o_tbl_idx      (fatld_tbl_idx),
        .i_tbl_name     (fat_boot_tbl_name),
        .i_tbl_cluster  (fat_boot_tbl_cluster),
        .i_tbl_size     (fat_boot_tbl_size),
        .i_tbl_attr     (fat_boot_tbl_attr),
        .i_fat_lba      (fat_boot_fat_lba),
        .i_data_lba     (fat_boot_data_lba),
        .i_sec_per_clus (fat_boot_spc),
        .i_spc_lg2      (fat_boot_spc_lg2),
        .write_req      (write_req),
        .write_req_ack  (write_req_ack),
        .sd_sec_read    (bmp_sec_read),
        .sd_sec_read_addr (bmp_sec_read_addr),
        .sd_sec_read_data (bmp_sec_read_data),
        .sd_sec_read_data_valid (bmp_sec_read_data_valid),
        .sd_sec_read_end (bmp_sec_read_end),
        // 3b：把仲裁器的"正在传输"标志给装载器，让它等卡空闲再发首扇区
        // 请求（理由见 bmp_load_fat.v 的 i_sd_busy 端口注释）
        .i_sd_busy      (sd_xfer_active),
        // 超时修复：本模块走 bmp 那一路，授权指示取 o_grant_bmp
        //   （它只把这一路透传给 fat32_file 做"排队不计时"用）
        .i_granted      (sd_grant_bmp),
        // S2：字幕通路 —— 与 load_start_pulse 同拍的单拍脉冲，
        // 告诉装载器"这一次装载的是 0N.SUB 掩膜条，不是 BMP"
        .i_job_sub      (sub_job_pulse),
        // S2：掩膜 RAM 写口（顶层那块 2048x48 1bpp RAM 的写端口）
        .o_mask_we      (sub_ram_we_w),
        .o_mask_addr    (sub_ram_addr_w),
        .o_mask_wdata   (sub_ram_wdata_w),
        // S2：字幕装载结果
        .o_sub_cols     (sub_cols_w),
        .o_sub_ok       (sub_ok_w),
        .o_sub_busy     (sub_busy_w),
        .bmp_data_wr_en (bmp_data_wr_en_src),
        .bmp_data       (bmp_data_src),
        .o_src_2to1     (bmp_2to1_mode),
        .o_prog         (fatld_prog_w),
        .o_busy         (fatld_busy_w),
        .o_err_code     (fat3b_load_err_w),
        .o_bytes_out    (fat3b_load_bytes_w),
        // 观察窗本模块内部用不到（仿真 tb 直接例化 DUT 才接）
        .o_dbg_fdat     (),
        .o_dbg_fdv      (),
        .o_dbg_ff_done  (),
        .o_dbg_ff_err   ()
    );
end else begin
// ============================================================================
// ↓↓↓ 以下到 generate 结束，是 3a 及以前的原通路：一个字符都没改 ↓↓↓
// ============================================================================
bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code_i),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (bmp_sec_read),
    .sd_sec_read_addr       (bmp_sec_read_addr),
    .sd_sec_read_data       (bmp_sec_read_data),
    .sd_sec_read_data_valid (bmp_sec_read_data_valid),
    .sd_sec_read_end        (bmp_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en_src),
    .bmp_data               (bmp_data_src)
);
end
endgenerate
// ============================================================================
// ↑↑↑ 原通路到此结束 ↑↑↑
// ============================================================================

// ===================== 旧通路"正在用卡"的判据（两种模式共用）=====================
// 3b 里把它从 generate 分支里提到外面的原因：
//     两个 generate 分支各写一句 assign 同一个 net 会被静态检查的
//     MD-ASSIGN 规则误判（见上面那段说明）。提到外面后全模块只有这一句，
//     而表达式与 3a 里的那一句完全相同 —— 行为零变化。
assign fat_old_busy = sd_xfer_active | load_busy;

// ===================== 阶段 3c：音频数据源（虚拟扇区服务器） =====================
// 单独一个一层generate，不嵌套进上面那个"图片装载源"的 generate
//     本工程只有"一层 if/else generate"被 TD 6.2.1 实际验证过；能不加嵌套
//     就不加。放在外面还有个好处：USE_FAT_WAV_EFF=0 时它整个不存在，
//     USE_FAT_WAV=0 的资源/时序与上板验证过的阶段 4 完全相同。
// 它为什么要在 USE_FAT_LOAD=1 的分支里才能工作
//     它要的"每张图配对哪段 WAV"来自 fat32_boot 走文件表的结果；
//     而 USE_FAT_WAV_EFF 已经保证"只有 USE_FAT_LOAD!=0 时才可能为 1"，
//     所以这里的引用不会是悬空的（USE_FAT_WAV_EFF=0 时本块被常量折叠）。
generate
if (USE_FAT_WAV_EFF != 0) begin
    wav_src_fat #(
        // 总等待兜底放宽到 8 秒
        //   本模块会故意把 fat32_file 的扇区请求压住（背压），
        //   最长的一次合法压制是"图片出现 -> 1.5 秒 -> 语音起播"，
        //   旧值 2 秒会让 fat32_file 在等待期里误报 ERR_SECTOR_TO 并结束。
        //   完整理由见 wav_src_fat 的文件头。
        .TO_LIMIT       (32'd2000000),
        .TO_LIMIT_ALL   (32'd800000000),
        .CHAIN_HARD_MAX (32'd1048576)
    ) u_wav_src_fat (
        .clk            (clk),
        .rst            (wav_src_rst),
        // 对 wav_player：只听它"要不要下一块"（数据本身不走本模块）
        .i_chunk_req    (wav_sec_read),        // = wav_player 的 sec_read
        // 对仲裁器：wav 那一路（数据扇区）
        .o_wav_req      (wsrc_wav_req_w),
        .o_sd_lba       (wsrc_sd_lba_w),
        .i_wav_dat      (wav_sec_read_data),
        .i_wav_dv       (wav_sec_read_data_valid),
        .i_wav_end      (wav_sec_read_end),
        .i_wav_gnt      (sd_grant_wav),
        // 对仲裁器：fat 那一路（FAT 表扇区，与 fat32_boot 共用）
        .o_fat_req      (wsrc_fat_req_w),
        .i_fat_dat      (fat_sec_read_data),
        .i_fat_dv       (fat_sec_read_data_valid),
        .i_fat_end      (fat_sec_read_end),
        .i_fat_gnt      (sd_grant_fat),
        .i_sd_busy      (sd_xfer_active),
        // 目标文件 + 卷几何
        .i_first_clu    (wav_sel_clu_w),
        .i_size         (wav_sel_size_w),
        .i_fat_lba      (fat_boot_fat_lba),
        .i_data_lba     (fat_boot_data_lba),
        .i_spc          (fat_boot_spc),
        // 诊断：本阶段没有引到数码管（要改顶层的显示逻辑，风险大于收益），
        //       仿真里由 tb 直接读这几个输出。
        .o_err          (),
        .o_bytes        (),
        .o_done         ()
    );
end
endgenerate

// ===================== SD 扇区读仲裁器 =====================
// 阶段 3a：原来这里是内联的二路仲裁（bmp 优先、wav 用空闲）。
//   现在原封不动地搬进独立模块 sd_arbiter3.v，并多接一路 FAT 解析
//   （最低优先级）。搬家的目的是：
//     1) 三路仲裁的优先级/锁存行为可以单独用 iverilog 仿真验证；
//     2) 现有两路的时序与原来逐拍等价（见 sd_arbiter3.v 文件头）。
//   行为要点（与原来完全一致）：
//     * bmp_read 优先（图像加载对延迟敏感），wav_player 用空闲时间；
//     * 一旦开始一次传输就锁住 owner 直到 sd_sec_read_end，
//       避免中途换地址把正在进行的传输打断（那会读到半个扇区）；
//     * 回程 end / data_valid 按 owner 分流，另一方不会收到不属于自己的脉冲。
//   新增的第 3 路 fat32_scan 优先级最低，且只在两个现有通路都不请求时才上卡。
//   USE_FAT_SCAN=0 时 fat_sec_read 恒 0，仲裁器退化成与原来完全相同的二路仲裁。
sd_arbiter3 u_sd_arbiter3 (
    .clk         (clk),
    .rst         (rst),

    .bmp_req     (bmp_sec_read),
    .bmp_lba     (bmp_sec_read_addr),
    .bmp_dat_vld (bmp_sec_read_data_valid),
    .bmp_dat     (bmp_sec_read_data),
    .bmp_end     (bmp_sec_read_end),

    // 阶段 3c：wav 那一路的"请求"改由 wav_arb_req_w 选
    //   3c 打开时提出请求的是 wav_src_fat（它替 fat32_file 要扇区），
    //   3c 关闭时还是 wav_player 自己。回程三根线两种情况下都是同一套
    //   （wav_sec_read_data / _valid / _end），所以只有请求侧要选。
    // 优先级顺序没动：bmp > wav > fat
    .wav_req     (wav_arb_req_w),
    .wav_lba     (wav_arb_lba_w),
    .wav_dat_vld (wav_sec_read_data_valid),
    .wav_dat     (wav_sec_read_data),
    .wav_end     (wav_sec_read_end),

    .fat_req     (fat_arb_req_w),
    .fat_lba     (fat_arb_lba_w),
    .fat_dat_vld (fat_sec_read_data_valid),
    .fat_dat     (fat_sec_read_data),
    .fat_end     (fat_sec_read_end),

    .sd_req      (sd_sec_read),
    .sd_lba      (sd_sec_read_addr),
    .sd_dat      (sd_sec_read_data),
    .sd_dat_vld  (sd_sec_read_data_valid),
    .sd_end      (sd_sec_read_end),

    .o_busy      (sd_xfer_active),
    .o_owner     (sd_xfer_owner),
    // 2026-10 超时修复：授权指示（纯组合，不改仲裁逻辑）
    //   完整语义见 sd_arbiter3.v 的端口注释。
    .o_grant_bmp (sd_grant_bmp),
    // 阶段 3c：这一路现在真的要用（wav_src_fat 的回程门控）
    .o_grant_wav (sd_grant_wav),
    .o_grant_fat (sd_grant_fat)
);

// ===================== 阶段 3a：FAT32 根目录解析（只在旧通路下成立）=====================
// 3b 改了这里的条件
//     原条件：if (USE_FAT_SCAN != 0) ... else ...（两个分支必居其一）
//     现条件：if (USE_FAT_LOAD==0 && USE_FAT_SCAN!=0) ... else if (USE_FAT_LOAD==0 && USE_FAT_SCAN==0) ...
//     也就是加了一层"只有走旧通路时才例化 3a 的 fat32_scan"：
//       * USE_FAT_LOAD=1（3b）   -> 两个分支都不成立，fat32_scan 整块不存在，
//                                  fat_sec_* / fat_scan_* / fat_dbg_* 由
//                                  fat32_boot 的例化输出端口驱动；
//       * USE_FAT_LOAD=0（旧）   -> 与 3a 逐字相同，行为与上板验证过的版本一致。
//     为什么不是嵌套 generate：本工程只有"一层 if/else generate"被 TD 6.2.1
//     实际验证过（3a 上板成功），能不加嵌套就不加。
// 另外：原来这里那句 assign fat_old_busy 被提到了 generate 外面
//     两个分支各写一句 assign 同一个 net 会被静态检查的 MD-ASSIGN 误判
//     （verilog_multidriver_check.py 不认识 generate 分支，实测确认），
//     而表达式完全一样，提到外面行为零变化。
generate
if ((USE_FAT_LOAD == 0) && (USE_FAT_SCAN != 0)) begin
    // 旧通路"正在用卡"的判据：仲裁器正在为 bmp/wav 传输中，
    // 或者 sd_card_bmp 自己正在装载一张图。
    // 用组合赋值，不额外打拍 因为 fat32_scan 看到它之后还要
    //   等 dly/FSM 一拍才会拉请求，不存在组合环。
    // 3b：这一句已经提到 generate 外面去了
    //     两个分支各写一句 assign 同一个 net 会被静态检查判成多驱动
    //     （MD-ASSIGN，工具不认识 generate 分支），而表达式一模一样。
    //     （即：这里原来是 assign fat_old_busy = sd_xfer_active | load_busy;）

      fat32_scan #(
          .START_DLY_CYC (FAT_START_DLY_CYC),
          .MAX_FILES     (10),
        .TO_LIMIT      (32'd2000000),
        .TO_LIMIT_ALL  (32'd200000000)
    ) u_fat32_scan (
        .clk            (clk),
        .rst            (rst),
        .i_sd_init_done (sd_init_done),
        .i_old_busy     (fat_old_busy),
        .o_rd_req       (fat_sec_read),
        .o_rd_lba       (fat_sec_read_addr),
        .i_data         (fat_sec_read_data),
        .i_data_valid   (fat_sec_read_data_valid),
        .i_data_end     (fat_sec_read_end),
        // 超时修复：3a 通路也接授权指示（3a 里 FAT 与 bmp 扫描并发，
        // 排队时间同样会被旧代码错算成超时）
        .i_granted      (sd_grant_fat),
        .o_count        (fat_scan_count),
        .o_err_code     (fat_scan_err),
        .o_done         (fat_scan_done),
        .o_busy         (fat_scan_busy),
        .o_cnt_digit    (fat_scan_cnt_digit),
        .o_cnt_ovf      (fat_scan_cnt_ovf),
        .o_err_digit    (),
        // 临时诊断：内部关键值直接引到模块级 wire
        .o_dbg_root_lba (fat_dbg_root_lba),
        .o_dbg_part_lba (fat_dbg_part_lba),
        .o_dbg_data_lba (fat_dbg_data_lba),
        .o_dbg_fat_lba  (fat_dbg_fat_lba),
        .o_dbg_secs     (fat_dbg_secs),
        .o_dbg_part_type(fat_dbg_part_type),
        .o_dbg_root_b0  (fat_dbg_root_b0),
        .o_dbg_root_b1  (fat_dbg_root_b1),
        .o_dbg_rx_bytes (fat_dbg_rx_bytes)
    );
end else if ((USE_FAT_LOAD == 0) && (USE_FAT_SCAN == 0)) begin
    // 旧行为：FAT 一路永远不请求，仲裁器退化回原来的二路仲裁
    assign fat_sec_read           = 1'b0;
    assign fat_sec_read_addr      = 32'd0;
    assign fat_scan_count         = 5'd0;
    assign fat_scan_err           = 4'd0;
    assign fat_scan_done          = 1'b0;
    assign fat_scan_busy          = 1'b0;
    assign fat_scan_cnt_digit     = 4'd0;
    assign fat_scan_cnt_ovf       = 1'b0;
    // 临时诊断：关掉 FAT 扫描时诊断值恒 0
    assign fat_dbg_root_lba       = 32'd0;
    assign fat_dbg_part_lba       = 32'd0;
    assign fat_dbg_data_lba       = 32'd0;
    assign fat_dbg_fat_lba        = 32'd0;
    assign fat_dbg_secs           = 16'd0;
    assign fat_dbg_part_type      = 8'd0;
    assign fat_dbg_root_b0        = 8'd0;
    assign fat_dbg_root_b1        = 8'd0;
    assign fat_dbg_rx_bytes       = 16'd0;
end
endgenerate

// 结果引出（两种开关下都有确定值；关掉时 done=0，顶层据此把数码管位 1/2 熄灭）
assign o_fat_count     = fat_scan_count;
assign o_fat_err       = fat_scan_err;
assign o_fat_cnt_ovf   = fat_scan_cnt_ovf;
assign o_fat_cnt_digit = fat_scan_cnt_digit;
assign o_fat_done      = fat_scan_done;

// 临时诊断输出（纯连线）
assign o_fat_dbg_root_lba  = fat_dbg_root_lba;
assign o_fat_dbg_part_lba  = fat_dbg_part_lba;
assign o_fat_dbg_data_lba  = fat_dbg_data_lba;
assign o_fat_dbg_fat_lba   = fat_dbg_fat_lba;
assign o_fat_dbg_secs      = fat_dbg_secs;
assign o_fat_dbg_part_type = fat_dbg_part_type;
assign o_fat_dbg_root_b0   = fat_dbg_root_b0;
assign o_fat_dbg_root_b1   = fat_dbg_root_b1;
assign o_fat_dbg_rx_bytes  = fat_dbg_rx_bytes;

// ===================== 阶段 3b：装载通路诊断（给顶层数码管）=====================
// 写成单条 assign + 编译期常量选择，而不是两个 generate 分支各写一句
//   后者会被 verilog_multidriver_check.py 的 MD-ASSIGN 规则误判成多驱动
//   （工具不认 generate 分支）。USE_FAT_LOAD 是常量参数，综合时这一句
//   直接折叠成端口常 0，不占任何资源。
// USE_FAT_LOAD=0 时 fat3b_*_w 三根线没有驱动源（悬空），但选择支是常量 0，
// 综合结果里它们连一根线都不剩，仿真也读不到（不参与判定）
assign o_fat3b_bmp_cnt    = (USE_FAT_LOAD != 0) ? fat3b_bmp_cnt_w    : 4'd0;
assign o_fat3b_load_err   = (USE_FAT_LOAD != 0) ? fat3b_load_err_w   : 4'd0;
assign o_fat3b_load_bytes = (USE_FAT_LOAD != 0) ? fat3b_load_bytes_w : 32'd0;

// ===================== S2：字幕掩膜条那 7 根对外输出 =====================
//   与上面 fat3b_* 三根线完全同样的处理方式：单条 assign + 编译期常量选择。
//   USE_SUB_EFF=0 时全部恒 0 —— 顶层的掩膜 RAM 写口永远是 0，
//   "关掉字幕"就等于"这条通路根本不存在"（资源与行为一并退回）。
assign o_sub_ram_we    = (USE_SUB_EFF != 0) ? sub_ram_we_w    : 1'b0;
assign o_sub_ram_addr  = (USE_SUB_EFF != 0) ? sub_ram_addr_w  : 11'd0;
assign o_sub_ram_wdata = (USE_SUB_EFF != 0) ? sub_ram_wdata_w : 48'd0;
assign o_sub_cols      = (USE_SUB_EFF != 0) ? sub_cols_w      : 12'd0;
assign o_sub_ok        = (USE_SUB_EFF != 0) ? sub_ok_w        : 1'b0;
assign o_sub_found     = (USE_SUB_EFF != 0) ? sub_found_w     : 3'd0;
assign o_sub_load_err  = (USE_SUB_EFF != 0) ? sub_load_err_w  : 4'd0;
assign o_sub_busy      = (USE_SUB_EFF != 0) ? sub_busy_w      : 1'b0;

// S4：wav_player 的 o_playing 直接引出（wav_player 内部一行没改）
// 这里不做USE_FAT_WAV 的常量选择 ：o_playing 是 clk_audio 域的电平，
//     与字幕开关无关；即使 USE_FAT_WAV=0（音频走旧通路）它照样有意义。
//     顶层只在 USE_SUBTITLE=0 时才用到它（那时例化被 generate 门掉）。
assign o_wav_playing   = wav_playing;

// ===================== 语音扫描起点 =====================
// 图片扫描完成时，bmp_read 的读地址正好停在"最后一张图片之后"。
// 卡上的语音文件通常紧跟图片存放，从这个位置起扫几乎立刻命中 ——
// 既快，又避免语音扫描和图片扫描平分 SPI 把图片显示拖慢近一倍。
// 用 scan_done 的上升沿锁存：之后 bmp_read 装载图片会改这个地址，不能再取。
//
// 3b 的行为改动（显著标出）
//   3b 通路下"图片扫描"这一步根本不存在（名单直接来自 FAT 表），
//   scan_done 那一刻 bmp_sec_read_addr 还是 0 —— 如果照抄旧写法，
//   语音会从扇区 0开始一个扇区一个扇区地扫，而 VOICE.WAV 在
//   数据区很靠后的位置（真卡上 LBA 48256），会白白晚十几秒才出声。
//   所以 3b 模式下改成"直接把 FAT 表里第一个 WAV 的首扇区当扫描起点"，
//   一次命中。返回值：把 .USE_FAT_LOAD(1'b1) 改成 1'b0（或删掉该行）即可 —— 
//   USE_FAT_LOAD 是常量参数，这里会折叠回原来的 bmp_sec_read_addr。
//   wav_player 内部一个字没改，它拿到的仍然只是一个"起始扇区号"。
//
// 阶段 3c 的行为改动（显著标出）
//   3c 打开后，"扫描起点"不再是全局一次性的：**每张图提交那一拍**都会
//   把它换成"这张图配对的那段 WAV 的首扇区"，同时音频那一路整体重启。
//   于是每张图播的都是它自己的讲解，而不是"只有第一张有声音"。
// 3c 打开时那个首扇区其实只是给 wav_player 当"放弃阈值/状态显示"用，
// 真正的数据是 wav_src_fat 按簇链读出来的（根本不看这个地址）。
reg  scan_done_d;
reg  wav_scan_go;
reg  [31:0] wav_scan_start;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_done_d    <= 1'b0;
        wav_scan_go    <= 1'b0;
        wav_scan_start <= 32'd0;
    end else begin
        scan_done_d <= scan_done;
        if ((USE_FAT_WAV_EFF != 0) && img_commit_pulse) begin
            // 阶段 3c：这张图刚显示出来 -> 换成它配对的那段 WAV
            //   优先级高于下面那个"扫描完成"分支，因为图片提交一定发生在
            //   名单扫描完成之后。
            // 本次（按键交互）不动这里
            //     这里只更新"这张图配的是哪段 WAV"（起点地址），
            //     不启动播放；真正决定"放不放"的是下面 wav_reload 与
            //     wav_play_go 上的 audio_gate_w 闸门。
            wav_scan_start <= wav_sel_lba_w;
            wav_scan_go    <= 1'b1;
        end else if (scan_done && !scan_done_d) begin
            // 3b：FAT 表里找到了 WAV 就用它的首扇区当起点（见上面那段说明）；
            // 否则（旧通路 / 表里没 WAV）保持 3a 的行为：用 bmp 侧最后的地址。
            // 3c 打开时这个值随后会被上面那个分支覆盖（每张图各一次）
            wav_scan_start <= ((USE_FAT_LOAD != 0) && fat_boot_wav_found)
                              ? fat_boot_wav_lba
                              : bmp_sec_read_addr;
            wav_scan_go    <= 1'b1;
        end
    end
end

// =============================================================================
// 阶段 3c：每张图提交时"重启音频那一路"的复位脉冲
//   为什么要复位（而不是加一个"重新开始"的端口）：
//     * wav_player 的取数侧（乒乓缓冲的写 toggle、cur_sector、sectors_left）
//       和播放侧（play_ptr/playing/rd_tgl/phase）都必须回到初值，
//       才能从"这段 WAV 的第 0 字节"重新播一遍；
//     * 这些都是它本来就有的、上板验证过的复位路径——
//       用复位重启，等于复用它原有的正确行为，不需要改动它内部的 FSM；
//     * 代价：复位期间 o_valid 会断 40 拍（400ns）。
//       40 拍远小于一个 48kHz 采样周期（520 拍），ACR 的 CTS 节拍
//       只是少 40 个脉冲（0.4 个采样的量级），不会让接收端失锁。
// 这一点无法在仿真里验证（ACR 在加密的 HDMI 核里），
// 只能上板看"每次换图后声音是否正常"，已在报告里如实标出。
//   为什么拉长到 40 拍而不是 1 拍：
//     clk_audio（25MHz）域的复位是异步的，1 拍（10ns）就足够复位寄存器，
//     但拉长一点可以避开"释放沿正好落在 clk_audio 时钟沿附近"的
//     亚稳态窗口，纯属稳妥。
// USE_FAT_WAV_EFF=0 时 wav_rst_cnt 恒为 0，wav_player_rst 就是 rst，
// 与阶段 4 逐拍一致（旧行为）。
// 这三个信号（wav_rst_cnt / wav_reload / wav_player_rst / wav_src_rst）
//     的声明在本文件靠上的"阶段 3c 音频那一路"那一段，
// 这里是它们的驱动 —— 因为要用下面主状态机产生的 img_commit_pulse。
//=============================================================================
// 本次新增：audio_need_reload（细节③ 的载体）
//   语义："音频这一路需要在本张图提交时重启一次"
//   置位：① 复位；② 从"定时档位 或 锁定模式"回到"讲解模式"的那一刻
//         —— 这时 audio_need_u=0 表示还没置过，所以只置一拍；
//            下一拍 audio_need_u 变 1，条件不再成立，不会反复置位。
//   清零：audio_reload 真的发出去了（= 某张图提交且闸门允许音频）
// 为什么需要它
//     细节③：从定时档切回讲解档时，当前这张图必须静静停着，
//     等下一张图再开始播。若不加这个 sticky 标志，audio_gate 一抬起
//     wav_play_go 就会在"当前这张图"上冒出来 -> 半路突然说一句话。
//=============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // 2026-10-12 修正：复位值原来是 1'b1，造成死锁
        // need_reload=1 -> audio_gate_w=0 -> audio_reload=0 -> need_reload 永远清不掉
        // -> audio_gate_w 恒 0 -> 任何模式下都不放音频
        // 改成 1'b0：上电就允许放音频
        // 只有"从定时档/锁定态回到讲解档"时才由下面的分支置 1（细节③ 仍成立）
        audio_need_reload <= 1'b0;
        audio_need_u      <= 1'b0;
    end else if (audio_reload) begin
        audio_need_reload <= 1'b0;      // 已经按新状态重启过一次了
        audio_need_u      <= 1'b0;
    end else if (!(period_is_explain_w && !key_locked_all_w && auto_play_en && !manual_w)) begin
        // 处在"不该放音频"的状态（定时档位 / 锁定模式 / SW1 设置模式 / 手动档 /
        //   或自动轮播未启动）
        //   -> 标记"下次回到讲解档时，要在下一张图提交那一拍才重启音频"
        //
        // 2026-10-20 修正：本条件原来只判 period_is_explain_w && !key_locked_w，
        //   与 audio_gate_w（:662）的五因子不一致，导致：
        //     SW1=1 设置模式 / 手动浏览档 / 自动轮播停止 这三种情形都会
        //     误置 audio_need_reload=1，用户回到讲解档后要等下一张图提交
        //     才出声 —— 表现就是"该出声时不出声"。
        //   现在两边用同一组因子（只差 audio_need_reload 自身，这仍是必须的：
        //   若这里也带上它就成了 need_reload -> gate -> reload 的环，
        //   和 :669 那条注释警告的是同一个坑）。
        if (!audio_need_u) begin
            audio_need_reload <= 1'b1;
            audio_need_u      <= 1'b1;
        end
    end else begin
        audio_need_u      <= 1'b0;      // 回到讲解档（且未锁）：允许下一次状态变化再触发
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wav_rst_cnt <= 6'd0;
    end else if (audio_reload) begin
        wav_rst_cnt <= 6'd40;
    end else if (wav_rst_cnt != 6'd0) begin
        wav_rst_cnt <= wav_rst_cnt - 6'd1;
    end
end

assign wav_reload     = (wav_rst_cnt != 6'd0);
assign wav_player_rst = rst | wav_reload;   // 给 wav_player
assign wav_src_rst    = rst | wav_reload;   // 给 wav_src_fat（两边必须同步重启）

// ===================== 语音起播延迟 =====================
// display_valid 拉高 = 第一张图已提交给显示缓冲，也就是"图片出现"的时刻。
// 从那一刻起再等 AUDIO_DELAY_CYC 拍才允许语音起播。
//
// 取数侧完全不受影响：wav_player 仍在后台预取，
// 所以起播时乒乓缓冲早已就绪，不会出现开头欠载。
//
// 想改延迟：改 AUDIO_DELAY_CYC 即可（100MHz 下 1 拍 = 10ns）
//      1.0 秒 = 32'd100_000_000
//      1.5 秒 = 32'd150_000_000   ← 当前
//      2.0 秒 = 32'd200_000_000
// 阶段 4：这个 localparam 已提到上面"阶段 4：停留时长"那一段声明
//     原因：tick_target 必须把这段延迟算进去（否则切换会比音频播完早 1.5 秒），
//     而 Verilog 不允许前向引用。改延迟请改那**一**处，不要在这里再加一个。
//
// 阶段 3c：延迟的起点从"第一张图出现"改成"每一张图出现"
//     每张图提交那一拍（img_commit_pulse / wav_reload）都会把计数清零并
//     让 play_go 落下，于是每张图都是"图片出现 -> 1.5 秒 -> 自己的讲解起播"。
//     3c 关闭时 wav_reload 恒 0，这段逻辑与阶段 4 逐拍一致。
//
// 本次新增：闸门 audio_gate_w（细节①②③ 全落在这里）
//     闸门落下（定时档位 / 锁定 / 刚切回讲解档还没换图）时：
//       * audio_dly_cnt 一律清 0
//       * wav_play_go 立即拉低 —— 这一条就是细节②：
//         "正在讲讲解时切到定时档 -> 立即静音"（不等这张图讲完）。
//         为什么立即静音是安全的：wav_player 收到 play_go=0 后在下一个
//         48kHz tick 就输出静音（o_left/o_right=0），但 o_valid 仍然
//         保持脉冲（ACR 的 CTS 节拍不断，HDMI 接收端不会失锁）——
//         这正是 wav_player 里"欠载时保持 valid+静音"那段既有机制。
//=============================================================================
// 本次 32 -> 33 位：它比较/加载的对象 AUDIO_DELAY_CYC 已是 33 位，
// 同宽比较才不会被工具链按"窄比宽"报宽度不一致（值本身只要 28 位）
reg [32:0] audio_dly_cnt;
reg        wav_play_go;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        audio_dly_cnt <= 33'd0;
        wav_play_go   <= 1'b0;
    end else if (!audio_gate_w) begin
        // 闸门落下：立即静音（细节②/①/③）
        audio_dly_cnt <= 33'd0;
        wav_play_go   <= 1'b0;
    end else if (wav_reload) begin
        // 阶段 3c：这张图刚开始（音频那一路正在重启）-> 重新起算 1.5 秒
        //   wav_reload 有 40 拍，所以实际延迟是 40 拍 + AUDIO_DELAY_CYC，
        //   多出 400ns，相对于 1.5 秒完全可以忽略（已在报告里写明）。
        audio_dly_cnt <= 33'd0;
        wav_play_go   <= 1'b0;
    end else if (!display_valid) begin
        // 首图还没出来：复位计数，保持静音
        audio_dly_cnt <= 33'd0;
        wav_play_go   <= 1'b0;
    end else if (audio_dly_cnt < AUDIO_DELAY_CYC) begin
        audio_dly_cnt <= audio_dly_cnt + 33'd1;
    end else begin
        wav_play_go   <= 1'b1;
    end
end

// ===================== TF 卡语音播放器 =====================
// 阶段 3c：wav_player.v 本文件一个字符都没改
//   3c 打开时只改三件事（都在这一处例化里，全部是编译期常量选择）：
//     1) rst        : 从全局 rst 换成 wav_player_rst（每张图重启一次）
//     2) 三根数据输入: 从"仲裁器的 wav 回程"换成"wav_src_fat 的虚拟扇区流"
//     3) sec_read/sec_read_addr 输出照旧引出（3c 时它们只当
//        "我要下一块"的请求信号给 wav_src_fat 用，地址被忽略）
//   它内部的乒乓缓冲、toggle 握手、P_LAT 三拍取样本序列、
//   欠载时保持 valid+静音的机制 —— 全部原样保留。
wav_player #(
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071)     // 前 64MiB，与图片扫描范围一致
) wav_player_m0 (
    .clk_sd                 (clk),
    .clk_audio              (clk_audio),
    .rst                    (wav_player_rst),

    .sec_read               (wav_sec_read),
    .sec_read_addr          (wav_sec_read_addr),
    // 阶段 3c：数据口直接接仲裁器的 wav 回程—— 本模块的逻辑一点都
    //   不插在中间。这是真 TD 综合实测下来唯一不会被前端优化掉的接法：
    //   把这三根改接 wav_src_fat 的输出，TD 的 optimize_rtl 会把
    //   wav_player 的取数侧 + audio_volume + HDMI 核里的音频 FIFO 一起当成
    //   死逻辑剪掉（area 报告里 wav_player 只剩 36 个触发器、audio_volume
    //   整个消失）—— 那样综合出来的比特流是没有声音的。
    // 见报告"真 TD 综合"一节里 V2 对照实验的原始证据。
    .sec_read_data          (wav_sec_read_data),
    .sec_read_data_valid    (wav_sec_read_data_valid),
    .sec_read_end           (wav_sec_read_end),
    .sd_init_done           (sd_init_done),
    .scan_start             (wav_scan_start),
    .scan_go                (wav_scan_go),
    .play_go                (wav_play_go),

    .o_valid                (o_audio_valid),
    .o_left                 (o_audio_left),
    .o_right                (o_audio_right),

    .o_found                (wav_found_i),
    .o_state                (wav_state_raw),
    .o_playing              (wav_playing)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

// DEBOUNCE_CYCLES 从 localparam 改成 parameter
//   为什么：仿真平台需要在不改被测逻辑的前提下把消抖时长改小
//   （生产是 50ms = 5,000,000 拍/次按键，一个用例要按十几次 —— 仿真等不起）。
//   改成 parameter 后默认值表达式一字未变，所以生产综合结果完全不变；
//   仿真用 defparam 覆盖它。这是"参数化夹具"，不是"改功能"。
module key_press_debounce #(
    parameter integer CLK_FREQ_HZ     = 100_000_000,
    parameter integer DEBOUNCE_MS     = 20,
    parameter integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);


reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule


//=============================================================================
// 模块名 : wav_src_fat（阶段 3c 新增 —— 修"音频是嘈杂声"的那个 bug）
//
// 功能概述
// ----------
//   它让 wav_player 觉得"卡上的这个 WAV 文件是物理连续的"：
//   wav_player 仍然像以前一样"要一个扇区（512 字节）-> 收满 -> 再要下一个"，
//   但本模块替它把每一次请求的扇区地址换成"这个文件按 FAT 簇链排出来的
//   第 k 个扇区"的绝对地址。
//
// 于是 wav_player.v 里一个字都不用改
//     它的数据口（sec_read_data / _valid / _end）**直接接仲裁器的 wav 回程**，
//     与改动前完全同一种接法；改的只是"谁来给地址、谁来发请求"。
//
// 为什么必须这样改（根因）
// ------------------------
//   旧设计是"拿到起始扇区后一个扇区一个扇区往后读"。
//   文件在卡上物理连续时这是对的（旧卡就是，所以当时听起来正常）；
//   一旦文件碎片化（新卡 tf_card_3c.img 上 8 个文件全部被故意打散，
//   例如 01.WAV 的簇链是 8297 -> 19396 -> 40820 -> 57215 -> …），
//   顺序读就会读到别的文件的数据 —— 听感正是"呼呼呼的嘈杂声"。
//   图片那条路没问题，因为它走 fat32_file（跟簇链）。音频也必须跟簇链。
//
// 怎么跟簇链（复用阶段 2 验证过的 fat32_file，一行不改）
// ----------------------------------------------------
//   fat32_file 会依次提出两种扇区请求：
//     * 数据扇区：文件内容（它同时把字节吐在 o_data 上）
//     * FAT 表扇区：只为取"下一个簇号"（这一段它不吐字节）
//   本模块把这两种请求分流到仲裁器的两路上：
//     * 数据扇区 -> wav 那一路（wav_player 就挂在这一路上，收到的是文件字节）
//     * FAT 表扇区 -> fat 那一路（最低优先级；wav_player 完全看不到）
//
// 为什么要分流（这是本模块最重要的设计决定，别改回去）
//   如果两种请求都走 wav 那一路，wav_player 就会把 FAT 表扇区的 512 字节
//   当成音频收下来 —— 每读一个簇就插进 5.3 毫秒的随机噪声。
//   分流之后 wav_player 收到的东西与"文件物理连续"时逐字节相同：
//   一次数据扇区传输 = 512 字节文件数据 + 一个 end = 它要的"一块"。
// 这个分流还有一个硬性好处：wav_player 的数据输入变成仲裁器的
//     纯组合回程线（没有任何本模块的逻辑插在中间）—— 这是本项目真 TD
// 综合实测下来唯一不会被前端优化掉的接法（见报告"TD 综合"一节）。
//
// 背压（谁决定"什么时候读下一个扇区"）
// -----------------------------------
//   wav_player 只在有空闲乒乓缓冲时才拉 sec_read 要下一块。
//   本模块的数据扇区请求 = `ff_rd_req & ~ff_is_fat & i_chunk_req`：
//     它不想要，请求就发不出去，fat32_file 停在 S_REQ 等
//     （等待期间它的 o_rd_lba 是稳定的，一放行还是同一个扇区）。
//   FAT 表扇区的请求不受sec_read 门控 —— 那正是想要的：
//   fat32_file 可以在 wav_player 还没要下一块的间隙里，先把簇链往前推一格。
//
// 一块 = 一个数据扇区（为什么块边界天然对齐）
// ------------------------------------------
//   wav_player 数满 512 个字节 + 一个 end 就认为"一块好了"，
//   而一次数据扇区传输正好是 512 字节 + 一个 end（仲裁器的 wav_end），
//   所以它数出来的一块就是一整个扇区，与文件里第 k 个扇区一一对应。
// 文件最后一个扇区通常不满 512 字节：卡仍然会送出整扇区，
//     多出来的那几十~几百字节是簇内填充（本工程打镜像时填 0）。
//     wav_player 会把它当"最后一块的尾巴"播出去 —— 最多 5.3 毫秒的静音，
// 听不出来。（这一点在报告"没解决/不确定"里也写了）
//
// 没有配对 WAV 的那张图
// ---------------------
//   fat32_boot 会把 i_size 给 0（没配到），fat32_file 立刻报错并 done、
//   一个扇区都不读 -> 本模块不发任何请求 -> wav_player 停在 S_SCAN 等，
//   它的播放侧照常输出"valid + 静音"，ACR 的 CTS 节拍不断（不会失锁）。
//
// 为什么 TO_LIMIT_ALL 要放宽到 8 秒（不是 bmp 那一路的 2 秒）
// ---------------------------------------------------------
//   fat32_file 的"总等待兜底"是从发出请求起算、不看授权的（防饥饿）。
//   而本模块会故意把数据扇区请求压住：图片出现到语音起播有 1.5 秒延迟，
//   那段时间 wav_player 的乒乓缓冲早就满了、不会要新块，fat32_file 就一直
//   停在 S_REQ 等 —— 用 2 秒的话它会在等待期里误报 ERR_SECTOR_TO 并结束，
//   语音只播两块就没了。所以本路用 8 秒（32 位计数器能装 42.9 秒），
//   远大于"1.5 秒延迟 + 一次 bmp 整图装载 0.85 秒"的最坏合法等待。
//
// 位宽（本项目踩过多次的坑，全部显式写死）
// ----------------------------------------
//   所有字面量都写明确位宽；扇区号一律 32 位，不做任何隐式截断。
//=============================================================================
module wav_src_fat #(
    parameter [31:0] TO_LIMIT       = 32'd2000000,
    parameter [31:0] TO_LIMIT_ALL   = 32'd800000000,   // 8 秒，理由见文件头
    parameter [31:0] CHAIN_HARD_MAX = 32'd1048576
)(
    input               clk,
    input               rst,             // 高有效异步复位（本工程约定）

    // ---- 对 wav_player：只听它"要不要下一块"（数据本身不走本模块）----
    input               i_chunk_req,     // = wav_player 的 sec_read

    // ---- 对 sd_arbiter3 的 wav 那一路（数据扇区）----
    output              o_wav_req,
    output     [31:0]   o_sd_lba,        // 两种请求共用（同一拍只有一路在要）
    input      [7:0]    i_wav_dat,
    input               i_wav_dv,
    input               i_wav_end,
    input               i_wav_gnt,       // = 仲裁器 o_grant_wav

    // ---- 对 sd_arbiter3 的 fat 那一路（FAT 表扇区，与 fat32_boot 共用）----
    output              o_fat_req,
    input      [7:0]    i_fat_dat,
    input               i_fat_dv,
    input               i_fat_end,
    input               i_fat_gnt,       // = 仲裁器 o_grant_fat

    input               i_sd_busy,       // = 仲裁器 o_busy（有一次传输正在进行）

    // ---- 目标文件 + 卷几何（来自 fat32_boot 的配对结果 / fat32_dir）----
    input      [31:0]   i_first_clu,     // 配对 WAV 的首簇
    input      [31:0]   i_size,          // 配对 WAV 的字节数
    input      [31:0]   i_fat_lba,
    input      [31:0]   i_data_lba,
    input      [7:0]    i_spc,

    // ---- 诊断（纯观测；本工程暂未引到数码管）----
    output     [3:0]    o_err,
    output     [31:0]   o_bytes,
    output              o_done
);

    //-------------------------------------------------------------------------
    // fat32_file 接口
    //-------------------------------------------------------------------------
    wire        ff_rd_req;
    wire [31:0] ff_rd_lba;
    wire [7:0]  ff_dat;
    wire        ff_dv;
    wire        ff_done;
    wire        ff_busy;
    wire [3:0]  ff_err;
    wire [31:0] ff_bytes;
    wire [31:0] ff_clusters;
    wire [31:0] ff_hash;
    wire [4:0]  ff_spc_lg2;
    wire [7:0]  ff_dsec;
    wire        ff_is_fat;
    wire [31:0] ff_rd_secs;

    // 控制寄存器（本模块只有这一段时序逻辑，含复位）
    reg         ff_kill;      // 1 = 把 fat32_file 按在复位（还没轮到它）
    reg         fst_start;    // 单拍：放开 fat32_file 那一拍给它一个 start

    //-------------------------------------------------------------------------
    // 组合：请求分流 + 回程分流
    //   ff_is_fat = 1 表示"这一次扇区读是读 FAT 表"（fat32_file 的 ph_fat）
    //-------------------------------------------------------------------------
    assign o_wav_req = ff_rd_req & (~ff_is_fat) & i_chunk_req;
    assign o_fat_req = ff_rd_req & ff_is_fat;
    assign o_sd_lba  = ff_rd_lba;

    // 回程：按当前这次请求的种类，从对应那一路收字节 / 收 end / 收授权
    wire [7:0] ff_in_dat  = ff_is_fat ? i_fat_dat : i_wav_dat;
    wire       ff_in_dv   = ff_is_fat ? i_fat_dv  : i_wav_dv;
    wire       ff_in_last = ff_is_fat ? i_fat_end : i_wav_end;
    wire       ff_in_gnt  = ff_is_fat ? i_fat_gnt : i_wav_gnt;

    assign o_err   = ff_err;
    assign o_bytes = ff_bytes;
    assign o_done  = ff_done;

    // fat32_file 的复位：全局复位 或 "还没轮到它"（低有效端口）
    wire fst_rst_n = ~(rst | ff_kill);

    //-------------------------------------------------------------------------
    // 起步：等"wav_player 要第一块"且"仲裁器空闲"才放开 fat32_file
    // 为什么要等 i_chunk_req
    //     wav_player 还没轮到这张图时（复位中 / S_IDLE）不该让 fat32_file
    //     去占卡；否则它会在空闲期里把"总等待兜底"跑完。
    // 为什么要等 !i_sd_busy
    //     换图/上电时仲裁器可能还在替上一次传输收尾，
    //     此刻放开就等于把别人那一次的字节当成自己的
    //     （bmp_load_fat.v 的 i_sd_busy 端口注释里记着同一类坑）。
    //-------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ff_kill   <= 1'b1;
            fst_start <= 1'b0;
        end else begin
            fst_start <= 1'b0;
            if (ff_kill) begin
                if (!i_sd_busy && i_chunk_req) begin
                    ff_kill   <= 1'b0;
                    fst_start <= 1'b1;
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // 被封装的主角：fat32_file（跟簇链 + 按文件大小吐字节，一个字都没改）
    //   它的 o_data/o_data_valid 本模块不再往外发 —— 数据是从卡直接到
    //   wav_player 的（见文件头"为什么要分流"那一段）。
    //   这里仍然把它接上，只是为了让它内部的"吐字节/计数"行为保持原样
    //   （bytes_left 一到 0 就正常结束，这正是"什么时候停止请求"的依据）。
    //-------------------------------------------------------------------------
    fat32_file #(
        .TO_LIMIT       (TO_LIMIT),
        .TO_LIMIT_ALL   (TO_LIMIT_ALL),
        .CHAIN_HARD_MAX (CHAIN_HARD_MAX)
    ) u_fat32_file (
        .clk            (clk),
        .rst_n          (fst_rst_n),

        .start          (fst_start),
        .i_first_clu    (i_first_clu),
        .i_size         (i_size),

        .i_fat_lba      (i_fat_lba),
        .i_data_lba     (i_data_lba),
        .i_spc          (i_spc),
        .i_max_clusters (32'd0),

        .o_rd_lba       (ff_rd_lba),
        .o_rd_req       (ff_rd_req),
        .i_data         (ff_in_dat),
        .i_data_valid   (ff_in_dv),
        .i_data_last    (ff_in_last),
        .i_granted      (ff_in_gnt),

        .o_data         (ff_dat),
        .o_data_valid   (ff_dv),

        .busy           (ff_busy),
        .done           (ff_done),
        .err_code       (ff_err),

        .o_bytes_out    (ff_bytes),
        .o_clusters     (ff_clusters),
        .o_lba_hash     (ff_hash),
        .dbg_spc_lg2    (ff_spc_lg2),
        .dbg_dsec       (ff_dsec),
        .dbg_is_fat     (ff_is_fat),
        .dbg_rd_secs    (ff_rd_secs)
    );

endmodule
