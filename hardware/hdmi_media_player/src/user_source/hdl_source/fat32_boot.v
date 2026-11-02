//=============================================================================
// 文件名  : fat32_boot.v
// 功能    : FAT 引导模块 —— 上电后先把 FAT32 根目录读出来，
//           把"哪些文件是图片（扩展名 BMP）"整理成一张名单，
//           再按文件表里的出现顺序把名单逐个交给 sd_card_bmp 去装载。
//
// 为什么需要它
// ---------------------------------------
//   旧通路 fat32_scan 是「先扫图，空闲时再顺手解析 FAT」，
//   本模块把顺序反过来：
//       上电 -> SD 初始化好 -> FAT 解析（几毫秒）-> 按文件表直接读图片
//   所以这里没有START_DLY_CYC 这类"等一段时间再跑"的参数：
//   sd_init_done 一起来就解析，解析完才允许下面的装载通路动。
//   （图片装载要等 o_ready/o_scan_done，见 sd_card_bmp 的 generate 分支）
//
// 本模块不做什么
// ----------------
//   * 不读文件数据、不跟簇链 —— 那是 fat32_file.v（由 bmp_load_fat.v 例化）
//   * 不改 fat32_dir.v 内部一个字符（它是逐字节 + 真硬件验证过的）
//   * 不碰 bmp_read / wav_player / OSD / 转场 / 双缓冲：那些模块一行没动
//
// 名单是怎么交给 sd_card_bmp 的（接口复用，不是新造一套握手）
//   sd_card_bmp 原本的流程是：
//       bmp_read 扫图 -> 每找到一张就给一个 scan_found_valid 脉冲，
//       scan_found_sector = 该图的起始扇区，最后 scan_done=1；
//       sd_card_bmp 把这 4 个"扇区号"存进 img_sector0..3，
//       轮到哪张就把 img_sectorN 当 load_sector 发给 bmp_read。
//   3b 复用完全相同的握手，只把"值"的含义换掉：
//       scan_found_sector = 该 BMP 在FAT 文件表里的下标(0~15)
//   于是 sd_card_bmp 的主状态机（轮播 / 双缓冲 / 转场握手）一个字都不用改，
//   装载时把 load_sector 当"表下标"再查一次文件表拿首簇和大小即可。
// 这里只替换了语义，接口沿用旧握手，因此主状态机无需改动
//
// 为什么 walk（走表）要单独一个阶段
//   fat32_dir 的文件表读口是组合读：给 i_tbl_idx 就拿回那一条的
//   名字/首簇/大小/属性。上电解析完（done=1）以后表内容就固定了，
//   所以这里用一个 1 拍 1 条的小状态机把 16 条走一遍，
//   顺手把 BMP 的下标、WAV 的首扇区挑出来 —— 不额外占 BRAM。
//
// 本文件末尾另有一个 module —— wav_dwell_calc
//   它把"WAV 的字节数"换算成"图片停留多少拍"。
//   放在本文件里（而不是新建 .v）是有意的：新 .v 必须手工加进工程文件，
//   而从外部改 .al/.prj
//   不可靠"（用户实测 TD 会用内存里的旧工程树覆盖回去）。同一个 .v 里放两个
//   module 对 iverilog / TD 都是合法的，于是 TD 侧零改动。
//
// 按序号把 WAV 与 BMP 配对（两遍走表）
//   背景：卡上有 4 张图 + 4 段讲解（01.BMP/01.WAV … 04.BMP/04.WAV），
//         而早期 RTL 只认"表里第一个 WAV"（wav_seen 只置位一次），
//         于是 4 张图的停留时长全按 01.WAV 算、音频也只有第一张有。
//   改法：走表从1 遍变成2 遍——
//         第 1 遍（B_WALK） ：照旧挑 BMP（下标 + 名字前两字节），
//                            顺手记下"表里第一个 WAV"（**旧通路的语义一个字没改**）；
//         第 2 遍（B_WALK2）：再走一遍表，把每个 WAV 按"名字前两字节"配到
//                            同序号的那张 BMP 名下（4 个槽位各存首簇/大小/有没有）。
//   为什么必须是两遍（而不是一遍顺手配）：
//         配对的两个文件在表里的先后顺序不保证（本卡是 BMP 在前，
//         但规范没有这条约束）。一遍走表时，先遇到 WAV 就还没记下 BMP 的名字。
//         多走一遍表只是 16 拍（100MHz 下 160ns），代价可以忽略。
//   判定依据（题目要求）：11 字节 8.3 名里扩展名字节 8~10 ——
//         BMP 认 "BMP"（大小写都认，沿用原有 ext_is_bmp），WAV 认 "WAV"；
//         序号 = 名字的前两个字符（8.3 名的字节 0、1），
//         大小写统一折叠成大写后比较（'a'~'z' -> 'A'~'Z'，数字不受影响）。
//   兜底：某张 BMP 没配到 WAV -> o_wav_sel_has=0 -> 那一槽的时长走
//         wav_dwell_calc 的 5 秒兜底，且语音侧该槽不播（见 sd_card_bmp.v）。
//
// 旧通路（USE_PAIR_DWELL=0）行为零变化
//     o_wav_lba / o_wav_ticks / o_wav_dur_ok 这三个"第一个 WAV"的输出
//     仍然由第 1 遍的 wav_seen / wav_size_r 决定，表达式一个字没改；
//     新增的 o_wav_sel_* 只有在 sd_card_bmp 打开阶段 3c 开关时才被用到。
//
// 位宽（本项目踩过多次的坑，全部显式写死）
// ----------------------------------------
//   簇号 -> 扇区：(簇号 - 2) << log2(每簇扇区数)，中间量一律 40 位接住，
//   确认高位没超（|xxx[39:32]）才截到 32 位，绝不静默截断成"另一个扇区号"。
//=============================================================================

`timescale 1ns / 1ps

module fat32_boot #(
    // 透传给 fat32_dir 的表容量 / 扇区超时 / 总等待兜底（见 fat32_dir.v 注释）
    parameter        MAX_FILES    = 10,
    parameter [31:0] TO_LIMIT     = 32'd2000000,
    parameter [31:0] TO_LIMIT_ALL = 32'd200000000,
    // 最多挑几张图（sd_card_bmp 只缓存 4 张：img_sector0..3）
    parameter [3:0]  MAX_BMPS  = 4'd4,
    // 阶段 4 新增：把 WAV 字节数换算成"停留拍数"必须知道系统时钟频率
    //   只透传给下面的 wav_dwell_calc，不改变本模块任何既有行为。
    parameter integer CLK_FREQ_HZ = 100_000_000,
    // 阶段 3c 新增：停留时长的"来源"选哪个
    //   0 = 旧行为（阶段 4）：用表里第一个 WAV的字节数 —— 与上板验证过的
    //       阶段 4 逐拍一致。
    //   1 = 阶段 3c：用i_wav_sel 指定的那张图所配对的那个 WAV的字节数。
    //   这是一个纯编译期常量，只决定 wav_dwell_calc 的输入从哪来（一个 mux），
    //   不新增任何逻辑。sd_card_bmp 把它接成"阶段 3c 总开关"。
    parameter integer USE_PAIR_DWELL = 0
)(
    input               clk,
    input               rst,              // 高有效异步复位（本工程约定）

    // ---- 启动条件 ----
    input               i_sd_init_done,   // SD 卡初始化完成
    input               i_old_busy,       // 旧通路（语音等）正在用卡 -> 礼让
    // ---- 名单放行：来自 sd_card_bmp 的 scan_start_pulse ----
    // 为什么要等这个脉冲，而不是解析完就自己播名单
    //   sd_card_bmp 的"首次扫描启动"那一拍会先把 img_found_count /
    //   img_sector0..3 清 0，紧接着才轮到 scan_found_valid 计数。
    //   名单必须在那次清零之后才开始播，否则会被清掉。
    //   所以顺序是：本模块解析完 -> o_ready=1 -> 触发 sd_card_bmp 的启动
    //   -> 拿到 scan_start_pulse -> 才开始播名单。
    input               i_list_go,

    // ---- 扇区读取握手（接 sd_arbiter3 的 fat_* 最低优先级那一路）----
    output              o_rd_req,
    output     [31:0]   o_rd_lba,
    input      [7:0]    i_data,
    input               i_data_valid,
    input               i_data_end,
    // ---- 2026-10 超时修复新增：本路"已被仲裁器授权用卡" ----
    //   直接来自 sd_arbiter3 的 o_grant_fat，透传给内部的 fat32_dir。
    //   3b 是 FAT 挂在最低优先级那一路上，正是"排队被算成超时"的重灾区：
    //   bmp 一次占卡 ≈0.85 秒（1801 扇区），而阈值只有 16384 拍（164µs），
    //   请求一发出去就注定超时。完整理由见 fat32_dir.v 文件头要点 (F)。
    // 没有仲裁器的场合接 1'b1（退化成修复前行为）
    input               i_granted,

    // ---- 文件表读口（组合读；给 bmp_load_fat 查"下标 -> 首簇/大小"）----
    input      [3:0]    i_tbl_idx,
    output     [87:0]   o_tbl_name,
    output     [31:0]   o_tbl_cluster,
    output     [31:0]   o_tbl_size,
    output     [7:0]    o_tbl_attr,

    // ---- 卷几何（给 bmp_load_fat 用，由 fat32_dir 算好，只有一处实现）----
    output     [31:0]   o_fat_lba,
    output     [31:0]   o_data_lba,
    output     [7:0]    o_sec_per_clus,
    output     [4:0]    o_spc_lg2,

    // ---- 模拟 bmp_read 的"扫描"接口（语义见文件头）----
    output reg          o_scan_done,
    output reg          o_scan_found_valid,
    output reg [31:0]   o_scan_found_sector,   // 这里是FAT 文件表下标
    output     [2:0]    o_scan_found_total,

    // ---- 状态 / 结果 ----
    output              o_ready,          // 1 = 名单已就绪（给顶层当 bmp_ready）
    output              o_busy,           // 1 = FAT 解析进行中
    output     [4:0]    o_count,          // 根目录里解析到的文件总数
    output     [3:0]    o_err_code,       // fat32_dir 的错误码，0 = 无错
    output              o_done,           // 1 = FAT 解析完成（电平保持）
    // 数位显示用（与 fat32_scan 的同名输出语义完全一致）
    output     [3:0]    o_cnt_digit,      // 文件数单个十进制位（>9 钳到 9）
    output              o_cnt_ovf,        // 1 = 文件数 > 9
    output     [3:0]    o_bmp_cnt,        // 挑到几张 BMP（0 ~ MAX_BMPS）
    output              o_wav_found,      // 表里有 WAV
    output     [31:0]   o_wav_lba,        // 第一个 WAV 的首扇区（给语音做扫描起点）
    // 阶段 4 新增：第一个 WAV 的时长信息（给 sd_card_bmp 定停留时长）
    //   o_wav_ticks 单看永远是"可以直接拿来当停留时长用"的拍数：
    //     有效时 = 音频时长(拍)；无效（没 WAV / 大小非法 / 太短）时 = 兜底 5 秒。
    //   o_wav_dur_ok = 1 表示上面那个值确实来自 WAV 大小，不是兜底。
    // 字节数的前提与精度见文件末尾 wav_dwell_calc 的文件头
    // 本次加宽 32 -> 33 位（60 秒 @100MHz = 60 亿拍 > 2^32-1）
    output     [32:0]   o_wav_ticks,
    output              o_wav_dur_ok,
    // ---- 阶段 3c 新增：按序号配对的结果（每张图配它自己的那段讲解） ----
    //   i_wav_sel  = 选哪一张图（0 ~ MAX_BMPS-1，与 BMP 名单的顺序一致）
    //   四个输出都用同一个下标组合选择，所以它们永远是同一张图的同一段 WAV。
    //   为什么要用"下标选一个"而不是开 4×4 根端口：端口少、连线短，而且
    //   选择逻辑集中在一个 case 里（见下面 wav_*_sel 三个 function）。
    // 没配到 WAV 的槽：o_wav_sel_has=0，clu/size/lba 保持 0
    input      [1:0]    i_wav_sel,
    output     [31:0]   o_wav_sel_clu,     // 配对 WAV 的首簇（给 wav_src_fat 读文件）
    output     [31:0]   o_wav_sel_size,    // 配对 WAV 的字节数
    output     [31:0]   o_wav_sel_lba,     // 配对 WAV 的首扇区（只给 wav_player 当扫描起点）
    output              o_wav_sel_has,     // 1 = 这张图确实配到了 WAV
    // ---- S2 新增：按序号配对的滚动字幕掩码条 0N.SUB ----
    //   与 o_wav_sel_* 共用同一个选择下标 i_wav_sel（= 正在装载/提交的那张图），
    //   于是"这张图的语音"与"这张图的字幕"永远是同一个序号上的两个文件。
    // 配对规则与 WAV 逐字同构：名字前两字节相同且扩展名 == SUB
    //     扩展名这个与项绝对不能省：01.SUB 的 nm2 也是 "01"，
    //     只比 nm2 的话它会命中 bmp_nm2_0，抢掉 01.WAV 的槽。
    //     （硬规则的出处：tools/_probe/探针报告_2026-10-10.md §2.2）
    output     [3:0]    o_sub_sel_idx,     // 该图配对到的 .SUB 在文件表里的下标
    output              o_sub_sel_has,     // 1 = 这张图确实配到了 .SUB
    output     [2:0]    o_sub_cnt,         // 一共配到几条 .SUB（诊断用，期望 4）
    output     [3:0]    o_phase,          // 调试：当前阶段

    // ---- 诊断（纯观测，与 fat32_scan 的同名输出一一对应）----
    output     [31:0]   o_dbg_root_lba,
    output     [31:0]   o_dbg_part_lba,
    output     [31:0]   o_dbg_data_lba,
    output     [31:0]   o_dbg_fat_lba,
    output     [15:0]   o_dbg_secs,
    output     [7:0]    o_dbg_part_type,
    output     [7:0]    o_dbg_root_b0,
    output     [7:0]    o_dbg_root_b1,
    output     [15:0]   o_dbg_rx_bytes
);

    //-------------------------------------------------------------------------
    // 阶段状态机
    //   B_IDLE  : 等 SD 初始化 + 旧通路空闲
    //   B_PARSE : 给 fat32_dir 一个 start 脉冲
    //   B_RUN   : 等 fat32_dir done
    //   B_WALK  : 逐条走文件表第 1 遍，挑 BMP（+ 记下第一个 WAV，兼容旧通路）
    //   B_WALK2 : 逐条走文件表第 2 遍，把 WAV 按序号配到 BMP 名下（阶段 3c）
    //   B_WAIT  : o_ready=1，等 sd_card_bmp 的 scan_start_pulse
    //   B_PLAY  : 逐个给出 scan_found_valid（值 = 表下标）
    //   B_DONE  : scan_done=1，保持
    // B_WALK2 = 3'd7：st 是 3 位，0~6 已被占用，第 7 个正好空着
    //-------------------------------------------------------------------------
    localparam [2:0] B_IDLE = 3'd0,
                     B_PARSE= 3'd1,
                     B_RUN  = 3'd2,
                     B_WALK = 3'd3,
                     B_WAIT = 3'd4,
                     B_PLAY = 3'd5,
                     B_DONE = 3'd6,
                     B_WALK2= 3'd7;

    reg [2:0]  st;

    // fat32_dir 侧握手
    reg        fd_start;
    wire       fd_busy;
    wire       fd_done;
    wire [3:0] fd_err;
    wire [31:0] fd_lba;
    wire       fd_req;
    wire [4:0] fd_count;
    wire [3:0] fd_tbl_idx;      // 送给 fat32_dir 的读表下标（walk 时用 widx）

    // fat32_dir 是低有效复位，这里是高有效，端口上取反适配
    wire rst_n_i = ~rst;

    // 送进 fat32_dir 的数据有效（与 3a 的 DATA_LEAD=1 同样的"严格更稳"一侧：
    // 只要已经起跑就把有效字节透下去，第一个字节不会因为状态刚切换而丢掉。
    // 不能只写 (st == B_RUN)：那样"第一个字节恰好落在起跑那一拍"会被丢，
    //     fat32_dir 每个扇区就会整体错位一字节（3a 踩过，见 fat32_scan 的
    //     DATA_LEAD 注释）。多认几拍不会多收数据 —— fat32_dir 自己按
    // 内部状态与 bcnt 计数，done 之后来的字节它一律不看。
    wire fd_dvalid = (st != B_IDLE) && i_data_valid;

    //-------------------------------------------------------------------------
    // 走表用寄存器
    //-------------------------------------------------------------------------
    reg [3:0]  widx;            // 当前走到的表下标
    reg [3:0]  bmp_cnt;         // 已挑到的 BMP 数
    // BMP 表下标用 4 个独立寄存器保存，而不是"可变下标读数组"：
    //   取用时用 case 选一个（bmp_idx_sel），综合出来就是几个 mux，
    //   不依赖工具对数组变量下标读的支持程度。
    reg [3:0]  bmp_idx0;
    reg [3:0]  bmp_idx1;
    reg [3:0]  bmp_idx2;
    reg [3:0]  bmp_idx3;
    reg [3:0]  pidx;            // 播名单用的下标
    reg        wav_seen;
    reg [31:0] wav_lba_r;
    // 阶段 4：第一个 WAV 的字节数（走表时从文件表里顺手抄下来）
    //   为什么不用去读 WAV 文件头：
    //     fat32_dir 解析根目录时已经把目录项偏移 28 处的 4 字节文件大小
    //     存进了文件表（tbl_size），走表时读到的 o_tbl_size 就是它。
    //     再读一次文件头等于上电阶段多占一次卡（那时卡正忙着装载首图）。
    // 阶段 3c：这一对仍然是"表里第一个 WAV"，旧通路（USE_PAIR_DWELL=0）
    // 就是靠它算时长的，语义与改动前逐拍一致。
    reg [31:0] wav_size_r;

    //-------------------------------------------------------------------------
    // 阶段 3c：按序号配对用的表（4 个槽位，与 BMP 名单一一对应）
    //   资源：4 x 16bit(名字前两字节) + 4 x 32bit(首簇) + 4 x 32bit(大小)
    //         + 4 x 1bit(配到没配到) = 324 个触发器 + 少量比较器/mux。
    //   为什么把"名字前两字节"单独存下来：
    //     第 2 遍走表时，当前被读的那一条是 WAV，而它要比对的 BMP 名字
    //     是另外几条目录项的内容。fat32_dir 的表读口是组合读，
    //     一次只给得出一条；所以第 1 遍先把 4 个前缀抄进寄存器。
    // 为什么只在名字前两字节上比，而不是比完整 8.3 名
    //     题目要求"BMP 和 WAV 的名字前两个字符相同即为一对"（01.BMP<->01.WAV），
    //     比前两字节正好就是这个规则，而且不依赖 BMP/WAV 之外的任何命名规律。
    //-------------------------------------------------------------------------
    reg [15:0] bmp_nm2_0;
    reg [15:0] bmp_nm2_1;
    reg [15:0] bmp_nm2_2;
    reg [15:0] bmp_nm2_3;
    reg [31:0] wav_clu_0;
    reg [31:0] wav_clu_1;
    reg [31:0] wav_clu_2;
    reg [31:0] wav_clu_3;
    reg [31:0] wav_sz_0;
    reg [31:0] wav_sz_1;
    reg [31:0] wav_sz_2;
    reg [31:0] wav_sz_3;
    reg        wav_has_0;
    reg        wav_has_1;
    reg        wav_has_2;
    reg        wav_has_3;

    //-------------------------------------------------------------------------
    // S2 新增：字幕（0N.SUB）配对表（与上面 WAV 的 4 个槽一一对应）
    //   只存"表下标"和"有没有"两样东西：
    //     首簇/大小不用在这里存 —— 装载时 bmp_load_fat 会拿这个下标
    //     再去读一次 fat32_dir 的组合读表口（与 BMP 装载完全同一条路径）。
    //   资源：4 x 4bit + 4 x 1bit = 20 个触发器。
    //-------------------------------------------------------------------------
    reg [3:0]  sub_idx_0;
    reg [3:0]  sub_idx_1;
    reg [3:0]  sub_idx_2;
    reg [3:0]  sub_idx_3;
    reg        sub_has_0;
    reg        sub_has_1;
    reg        sub_has_2;
    reg        sub_has_3;

    // 阶段 4：换算出来的停留拍数（组合，见文件末尾 wav_dwell_calc）
    // 本次加宽 32 -> 33 位，与 wav_dwell_calc 的 o_ticks 保持同宽
    wire [32:0] wav_ticks_w;
    wire        wav_dur_ok_w;

    //-------------------------------------------------------------------------
    // 名字/属性判定（组合）
    //   8.3 名字 88 位里：byte0 在最高 8 位 -> 扩展名 3 个字节是 [23:16]/[15:8]/[7:0]
    //   大小写都认（用户卡上是 00_1.BMP 这种名字，不假设任何命名规律，
    //   只认"扩展名 == BMP"，与题目要求一致）
    //-------------------------------------------------------------------------
    wire [7:0] ext_b = o_tbl_name[23:16];
    wire [7:0] ext_m = o_tbl_name[15:8];
    wire [7:0] ext_p = o_tbl_name[7:0];
    wire [7:0] ext_w = o_tbl_name[23:16];
    wire [7:0] ext_a = o_tbl_name[15:8];
    wire [7:0] ext_v = o_tbl_name[7:0];

    wire ext_is_bmp = ((ext_b == "B") || (ext_b == "b")) &&
                      ((ext_m == "M") || (ext_m == "m")) &&
                      ((ext_p == "P") || (ext_p == "p"));

    wire ext_is_wav = ((ext_w == "W") || (ext_w == "w")) &&
                      ((ext_a == "A") || (ext_a == "a")) &&
                      ((ext_v == "V") || (ext_v == "v"));

    // S2：字幕掩码条 0N.SUB 的扩展名分类器（与 ext_is_bmp / ext_is_wav 同构）
    wire [7:0] ext_s2 = o_tbl_name[23:16];
    wire [7:0] ext_u2 = o_tbl_name[15:8];
    wire [7:0] ext_b2 = o_tbl_name[7:0];

    wire ext_is_sub = ((ext_s2 == "S") || (ext_s2 == "s")) &&
                      ((ext_u2 == "U") || (ext_u2 == "u")) &&
                      ((ext_b2 == "B") || (ext_b2 == "b"));

    // 目录/卷标/LFN 在 fat32_dir 里已经滤掉了（rec_ok），这里再兜一层，
    // 顺带把"大小为 0"和"首簇非法"的条目剔掉 —— 它们进了名单只会让
    // 装载通路白等一场（表现为黑屏 + 3 秒看门狗）。
    wire ent_file = ((o_tbl_attr & 8'h18) == 8'h00) &&
                    (o_tbl_size != 32'd0) &&
                    (o_tbl_cluster >= 32'd2);

    wire ent_is_bmp = ent_file && ext_is_bmp;
    wire ent_is_wav = ent_file && ext_is_wav;
    // 硬规则：字幕条目也必须"文件属性 + 扩展名"两个判据相与，
    // 绝不能写成只比 nm2（见文件头 / 端口注释）
    wire ent_is_sub = ent_file && ext_is_sub;

    //-------------------------------------------------------------------------
    // 簇号 -> 绝对扇区（40 位中间量，高位没超才算合法）
    //   只用于给语音一个"扫描起点"，不参与图片装载（图片那边由
    //   bmp_load_fat 用同样的公式算首扇区，两处都按 fat32_dir 的几何来）
    //-------------------------------------------------------------------------
    reg [39:0] wav_off_x;
    reg [39:0] wav_lba_x;

    always @(*) begin
        wav_off_x = ({8'd0, o_tbl_cluster} - 40'd2) << o_spc_lg2;
        wav_lba_x = {8'd0, o_data_lba} + wav_off_x;
    end

    //-------------------------------------------------------------------------
    // 播名单时按下标取"第 k 张 BMP 的表下标"
    //-------------------------------------------------------------------------
    function [3:0] bmp_idx_sel;
        input [3:0] k;
        begin
            case (k)
            4'd0:    bmp_idx_sel = bmp_idx0;
            4'd1:    bmp_idx_sel = bmp_idx1;
            4'd2:    bmp_idx_sel = bmp_idx2;
            default: bmp_idx_sel = bmp_idx3;
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    // 阶段 3c：序号（名字前两个字符）的取法与大小写折叠
    //   8.3 名的 88 位里 byte0 = [87:80]，byte1 = [79:72]（上面扩展名那段
    //   已经确认了 byte8~10 = [23:0]）。序号就是这两个字节。
    //   折叠只影响字母（'a'~'z'），数字原样通过 —— 用户卡上的 01/02/03/04
    //   本来就是数字，折叠是为了兼容 "0a.BMP" 与 "0A.WAV" 这种混用大小写。
    //-------------------------------------------------------------------------
    function [7:0] fold_up;
        input [7:0] c;
        begin
            if ((c >= "a") && (c <= "z"))
                fold_up = c - 8'h20;
            else
                fold_up = c;
        end
    endfunction

    wire [15:0] nm2_cur = {fold_up(o_tbl_name[87:80]), fold_up(o_tbl_name[79:72])};

    //-------------------------------------------------------------------------
    // ★ 判定流水（2026-10-30，修布线后 Setup -0.381ns）
    //
    //   文件表读口是组合读 LUTRAM（o_tbl_* = tbl[i_tbl_idx]），B_WALK /
    //   B_WALK2 又拿本拍读出的名字/大小直接做配对判定和写使能。这条
    //   "widx → LUTRAM 译码 → 88 位名字 → fold_up → 4 路比较 → 写 CE"
    //   的链被 TD 布到了 10 级逻辑、16.03ns（预算 16ns），其中布线 11.96ns
    //   —— 综合器还把 HDMI 发送器里的一个 LUT 复用进这条 mux 链，net 被
    //   拉去横跨半个芯片。修法就是把判定整体延一拍：
    //
    //     拍 n：widx=n，表读出条目 n → 寄存（本块）
    //     拍 n+1：用寄存值判定并写入（状态机里）
    //
    //   遍历节奏只多 1 拍（widx 走到 fd_count 才退出），16 条目录从 16 拍
    //   变 17 拍，上电阶段完全无感。判定基准不变：m_hit 组合块每拍都算，
    //   看到的 wav_has_* 仍是"上一拍写入后"的值，与延拍前的语义逐拍一致。
    //
    //   越界屏蔽（★ 用"流水连续性"，不能用 widx_d < fd_count）：
    //     过渡拍（进/出走表状态的边界）表读口会扫到越界下标，寄存进 _d 的
    //     是 RAM 残留 —— TD 综合后不是 X，会误判。判"残留"最可靠的判据是
    //     流水是否连续：正常流水下 widx_d 永远等于 widx-1（模 16），边界拍
    //     不满足（B_WALK2 第一拍 widx=0 而 widx_d=退出时的 fd_count）。
    //     所以状态机里的写使能统一与 (widx_d + 1 == widx) 相与：
    //       正常拍恒真；边界拍恒假 —— 不依赖 fd_count 的任何边界语义。
    //     （不能用 widx_d < fd_count：fat32_dir 的 o_count 在表满时饱和在
    //       TBL_LAST，此时表里实际有 fd_count+1 条有效条目，"<"会漏最后一条。）
    //-------------------------------------------------------------------------
    reg [3:0]  widx_d;                          // 判定对应的表下标（延 1 拍）
    reg        is_bmp_d, is_wav_d, is_sub_d;    // 类型判定（延 1 拍）
    reg [15:0] nm2_d;                           // 名字前两字节（延 1 拍）
    reg [31:0] clu_d, size_d, lba_d;            // 簇号/大小/起扇区（延 1 拍）
    reg        m_hit_r, sub_hit_r;              // 配对判定（延 1 拍）
    reg [1:0]  m_k_r, sub_k_r;

    wire walk_now = (st == B_WALK) || (st == B_WALK2);
    // 流水连续性：正常拍 widx_d == widx-1（模 16）；边界拍不成立
    wire widx_cont = (widx_d + 4'd1) == widx;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            widx_d    <= 4'd0;
            is_bmp_d  <= 1'b0;
            is_wav_d  <= 1'b0;
            is_sub_d  <= 1'b0;
            nm2_d     <= 16'd0;
            clu_d     <= 32'd0;
            size_d    <= 32'd0;
            lba_d     <= 32'd0;
            m_hit_r   <= 1'b0;
            m_k_r     <= 2'd0;
            sub_hit_r <= 1'b0;
            sub_k_r   <= 2'd0;
        end else if (walk_now) begin
            widx_d    <= widx;
            is_bmp_d  <= ent_is_bmp;
            is_wav_d  <= ent_is_wav;
            is_sub_d  <= ent_is_sub;
            nm2_d     <= nm2_cur;
            clu_d     <= o_tbl_cluster;
            size_d    <= o_tbl_size;
            lba_d     <= wav_lba_x[31:0];
            m_hit_r   <= m_hit;
            m_k_r     <= m_k;
            sub_hit_r <= sub_m_hit;
            sub_k_r   <= sub_m_k;
        end else begin
            // 离开走表后全部失效：下一段 _d 不会带着上一遍的残留
            is_bmp_d  <= 1'b0;
            is_wav_d  <= 1'b0;
            is_sub_d  <= 1'b0;
            m_hit_r   <= 1'b0;
            sub_hit_r <= 1'b0;
        end
    end

    //-------------------------------------------------------------------------
    // 阶段 3c：第 2 遍走表时，"当前这一条 WAV 该配到第几个槽"
    //   条件：该槽里确实有 BMP（bmp_cnt > k）、名字前两字节相同、且该槽
    //         还没配过 WAV（保证"同名多段 WAV"时取表里最靠前的那一个）。
    //   4 个槽全不中 -> m_hit=0 -> 这条 WAV 被忽略（不配对，不报错）。
    //-------------------------------------------------------------------------
    reg [1:0] m_k;
    reg       m_hit;

    always @(*) begin
        m_hit = 1'b0;
        m_k   = 2'd0;
        if ((bmp_cnt > 4'd0) && (nm2_cur == bmp_nm2_0) && !wav_has_0) begin
            m_hit = 1'b1; m_k = 2'd0;
        end else if ((bmp_cnt > 4'd1) && (nm2_cur == bmp_nm2_1) && !wav_has_1) begin
            m_hit = 1'b1; m_k = 2'd1;
        end else if ((bmp_cnt > 4'd2) && (nm2_cur == bmp_nm2_2) && !wav_has_2) begin
            m_hit = 1'b1; m_k = 2'd2;
        end else if ((bmp_cnt > 4'd3) && (nm2_cur == bmp_nm2_3) && !wav_has_3) begin
            m_hit = 1'b1; m_k = 2'd3;
        end
    end

    //-------------------------------------------------------------------------
    // S2：第 2 遍走表时"当前这一条 .SUB 该配到第几个槽"
    //   条件与上面 m_hit 逐字同构（换成 sub_has_*），
    //   另外必须由 ent_is_sub 把关（在状态机里与 sub_m_hit 相与）。
    //-------------------------------------------------------------------------
    reg [1:0] sub_m_k;
    reg       sub_m_hit;

    always @(*) begin
        sub_m_hit = 1'b0;
        sub_m_k   = 2'd0;
        if ((bmp_cnt > 4'd0) && (nm2_cur == bmp_nm2_0) && !sub_has_0) begin
            sub_m_hit = 1'b1; sub_m_k = 2'd0;
        end else if ((bmp_cnt > 4'd1) && (nm2_cur == bmp_nm2_1) && !sub_has_1) begin
            sub_m_hit = 1'b1; sub_m_k = 2'd1;
        end else if ((bmp_cnt > 4'd2) && (nm2_cur == bmp_nm2_2) && !sub_has_2) begin
            sub_m_hit = 1'b1; sub_m_k = 2'd2;
        end else if ((bmp_cnt > 4'd3) && (nm2_cur == bmp_nm2_3) && !sub_has_3) begin
            sub_m_hit = 1'b1; sub_m_k = 2'd3;
        end
    end

    // S2：按下标取"这一张图配对的那条 .SUB 的表下标 / 有没有"
    // 为什么这里用显式三目 mux而不是像上面 WAV 那样的 function + case
    //     iverilog 对"连续赋值里带函数调用"的灵敏度处理有缺陷：函数体里读的
    //     那些 net 变了，它不会重新求值 —— 实测（tools/_card_build/_subfix/
    //     sel_sens_t.v）：只改函数体里读的 reg、不改选择器时，
    //         function 版停在旧值 0；三目版立刻变成 1。
    //     真硬件与 TD 综合出来两者完全相同（都是纯组合 mux），
    //     但为了让 S2 的验证平台能真读到配对结果（而不是复位期的陈旧值），
    //     新增的这条通路用三目 mux 写。
    // 原有 WAV 那几个 function 一个字没动，不在本次范围

    //-------------------------------------------------------------------------
    // 阶段 3c：按下标取"这一张图配对的那段 WAV"的三个字段
    // input 是 x/z 时 case 走 default（取槽 3）
    //     这一点是刻意的：老的仿真 tb（tb_dwell / tb_fat_load）没有连
    //     i_wav_sel 这一根新端口，而它们用的 USE_PAIR_DWELL=0 时下面那个
    //     输入 mux 根本不会选到这三个 function 的输出，所以不影响它们。
    //-------------------------------------------------------------------------
    function [31:0] wav_clu_sel;
        input [1:0] k;
        begin
            case (k)
            2'd0:    wav_clu_sel = wav_clu_0;
            2'd1:    wav_clu_sel = wav_clu_1;
            2'd2:    wav_clu_sel = wav_clu_2;
            default: wav_clu_sel = wav_clu_3;
            endcase
        end
    endfunction

    function [31:0] wav_sz_sel;
        input [1:0] k;
        begin
            case (k)
            2'd0:    wav_sz_sel = wav_sz_0;
            2'd1:    wav_sz_sel = wav_sz_1;
            2'd2:    wav_sz_sel = wav_sz_2;
            default: wav_sz_sel = wav_sz_3;
            endcase
        end
    endfunction

    function [0:0] wav_has_sel;
        input [1:0] k;
        begin
            case (k)
            2'd0:    wav_has_sel = wav_has_0;
            2'd1:    wav_has_sel = wav_has_1;
            2'd2:    wav_has_sel = wav_has_2;
            default: wav_has_sel = wav_has_3;
            endcase
        end
    endfunction

    wire [31:0] sel_clu  = wav_clu_sel(i_wav_sel);
    wire [31:0] sel_size = wav_sz_sel(i_wav_sel);
    wire        sel_has  = wav_has_sel(i_wav_sel);

    //-------------------------------------------------------------------------
    // 阶段 3c：选中那个 WAV 的首扇区（40 位中间量，与上面 wav_lba_x 同公式）
    //   只用来给 wav_player 一个"扫描起点"（决定它的放弃阈值与状态显示），
    //   真正读文件的是 wav_src_fat + fat32_file（按簇链，不用这个数）。
    //   高位超 32 位就报 0，绝不静默截断成"另一个扇区号"。
    //-------------------------------------------------------------------------
    reg [39:0] sel_lba_x;
    reg [31:0] sel_lba_w;

    always @(*) begin
        sel_lba_x = {8'd0, o_data_lba} + (({8'd0, sel_clu} - 40'd2) << o_spc_lg2);
        sel_lba_w = (|sel_lba_x[39:32]) ? 32'd0 : sel_lba_x[31:0];
    end

    //-------------------------------------------------------------------------
    // 对外：结果与几何
    //-------------------------------------------------------------------------
    assign o_rd_req        = (st == B_RUN) && fd_req;
    assign o_rd_lba        = fd_lba;
    assign o_count         = fd_count;
    assign o_err_code      = fd_err;
    assign o_done          = fd_done;
    assign o_busy          = fd_busy;
    assign o_ready         = (st == B_WAIT) || (st == B_PLAY) || (st == B_DONE);
    assign o_bmp_cnt       = bmp_cnt;
    // 数位显示：与 fat32_scan 里那两句完全相同（>9 钳到 9 并给溢出标志）
    assign o_cnt_ovf       = (fd_count > 5'd9);
    assign o_cnt_digit     = (fd_count > 5'd9) ? 4'd9 : fd_count[3:0];
    assign o_wav_found     = wav_seen;
    assign o_wav_lba       = wav_lba_r;
    assign o_wav_ticks     = wav_ticks_w;
    assign o_wav_dur_ok    = wav_dur_ok_w;
    assign o_phase         = {1'b0, st};
    assign o_scan_found_total = {1'b0, pidx[1:0]};
    // 阶段 3c：按序号配对的结果（没配到的槽给 0 + has=0）
    assign o_wav_sel_clu   = sel_clu;
    assign o_wav_sel_size  = sel_size;
    assign o_wav_sel_lba   = sel_lba_w;
    assign o_wav_sel_has   = sel_has;
    // S2：字幕（0N.SUB）的配对结果 —— 与 WAV 用同一个下标 i_wav_sel
    //   （三目 mux，理由见上面 sub_idx_sel 那一段注释）
    assign o_sub_sel_idx   = (i_wav_sel == 2'd0) ? sub_idx_0 :
                             (i_wav_sel == 2'd1) ? sub_idx_1 :
                             (i_wav_sel == 2'd2) ? sub_idx_2 : sub_idx_3;
    assign o_sub_sel_has   = (i_wav_sel == 2'd0) ? sub_has_0 :
                             (i_wav_sel == 2'd1) ? sub_has_1 :
                             (i_wav_sel == 2'd2) ? sub_has_2 : sub_has_3;
    // 一共配到几条（只用来在数码管上看"4 条字幕都配上了没有"）
    // 显式位宽，不用隐式扩展
    assign o_sub_cnt       = {2'b00, sub_has_0} + {2'b00, sub_has_1}
                           + {2'b00, sub_has_2} + {2'b00, sub_has_3};

    // 走表时读 widx（第 1 遍挑 BMP、第 2 遍配对），其余时候把读口让给 bmp_load_fat
    assign fd_tbl_idx = ((st == B_WALK) || (st == B_WALK2)) ? widx : i_tbl_idx;

    //-------------------------------------------------------------------------
    // 主状态机（本块是 st / widx / bmp_cnt / bmp_idxX / pidx / wav_* 的唯一驱动）
    //-------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st                 <= B_IDLE;
            fd_start           <= 1'b0;
            widx               <= 4'd0;
            bmp_cnt            <= 4'd0;
            bmp_idx0           <= 4'd0;
            bmp_idx1           <= 4'd0;
            bmp_idx2           <= 4'd0;
            bmp_idx3           <= 4'd0;
            pidx               <= 4'd0;
            wav_seen           <= 1'b0;
            wav_lba_r          <= 32'd0;
            wav_size_r         <= 32'd0;
            // 阶段 3c：配对表一并复位（4 个槽的序号/首簇/大小/有效位）
            bmp_nm2_0          <= 16'd0;
            bmp_nm2_1          <= 16'd0;
            bmp_nm2_2          <= 16'd0;
            bmp_nm2_3          <= 16'd0;
            wav_clu_0          <= 32'd0;
            wav_clu_1          <= 32'd0;
            wav_clu_2          <= 32'd0;
            wav_clu_3          <= 32'd0;
            wav_sz_0           <= 32'd0;
            wav_sz_1           <= 32'd0;
            wav_sz_2           <= 32'd0;
            wav_sz_3           <= 32'd0;
            wav_has_0          <= 1'b0;
            wav_has_1          <= 1'b0;
            wav_has_2          <= 1'b0;
            wav_has_3          <= 1'b0;
            // S2：字幕配对表一并复位
            sub_idx_0          <= 4'd0;
            sub_idx_1          <= 4'd0;
            sub_idx_2          <= 4'd0;
            sub_idx_3          <= 4'd0;
            sub_has_0          <= 1'b0;
            sub_has_1          <= 1'b0;
            sub_has_2          <= 1'b0;
            sub_has_3          <= 1'b0;
            o_scan_done        <= 1'b0;
            o_scan_found_valid <= 1'b0;
            o_scan_found_sector<= 32'd0;
        end else begin
            // 默认：start 与 found_valid 都是单拍脉冲
            fd_start           <= 1'b0;
            o_scan_found_valid <= 1'b0;

            case (st)
            //---------------------------------------------------------- 等启动
            B_IDLE: begin
                if (i_sd_init_done && !i_old_busy) begin
                    // 3b：卡一初始化好就解析，不再等 10 秒（那是 3a 的礼让策略）
                    o_scan_done  <= 1'b0;
                    bmp_cnt      <= 4'd0;
                    pidx         <= 4'd0;
                    widx         <= 4'd0;
                    st           <= B_PARSE;
                end
            end

            //---------------------------------------------------------- 起跑
            B_PARSE: begin
                fd_start <= 1'b1;      // 一拍脉冲；fat32_dir 在 S_IDLE 收到即启动
                st       <= B_RUN;
            end

            //---------------------------------------------------------- 等解析
            B_RUN: begin
                if (fd_done)
                    st <= B_WALK;
            end

            //---------------------------------------------------------- 走表（第 1 遍）
            B_WALK: begin
                // ★ 判定用上一拍的寄存值（见"判定流水"一节）。
                //   原来直接用本拍组合量（LUTRAM 读出 → 判定 → 写 CE），
                //   被布出 10 级 / -0.381ns 的路径；延一拍后每级都只剩 2~3 层。
                //   widx_d < fd_count 屏蔽进/出状态过渡拍读到的越界条目。
                if (is_bmp_d && (bmp_cnt < MAX_BMPS) && (widx_cont)) begin
                    // 阶段 3c：顺手把这张 BMP 的"序号"（名字前两字节）存下来，
                    // 第 2 遍配 WAV 时要用
                    case (bmp_cnt)
                    4'd0:    begin bmp_idx0 <= widx_d; bmp_nm2_0 <= nm2_d; end
                    4'd1:    begin bmp_idx1 <= widx_d; bmp_nm2_1 <= nm2_d; end
                    4'd2:    begin bmp_idx2 <= widx_d; bmp_nm2_2 <= nm2_d; end
                    default: begin bmp_idx3 <= widx_d; bmp_nm2_3 <= nm2_d; end
                    endcase
                    bmp_cnt <= bmp_cnt + 4'd1;
                end

                if (is_wav_d && !wav_seen && (widx_cont)) begin
                    wav_seen  <= 1'b1;
                    // 只取低 32 位（高位在 40 位里判过；这里再保险一次）
                    wav_lba_r <= lba_d;
                    // 阶段 4：顺手把字节数也记下来 —— 这就是"讲解时长"的唯一依据
                    //   原来本拍 fd_tbl_idx == widx，o_tbl_size 正是这一条的大小；
                    //   现在换成了延一拍的 size_d，语义相同。
                    // 前提：WAV 是规范 44 字节头（tools/make_tf_wav.py 生成的就是），
                    // 换算公式与前提写在文件末尾 wav_dwell_calc 的文件头里。
                    wav_size_r <= size_d;
                end

                // 结束条件：上一拍（widx_d）已经是表尾 —— 判定流水要求
                //   widx 多走 1 拍（到 fd_count），让最后一条的判定落地。
                //   fd_count 最多 = MAX_FILES = 16，widx 是 4 位：走到 16 会
                //   回绕成 0，但退出条件同时把 widx 置 0，两者结果一致。
                // 用 5 位比较，避免 widx 是 4 位在 count=16 时回绕成死循环
                // 阶段 3c：这里不再直接去 B_WAIT，而是回头走第 2 遍做配对
                if ((widx_cont) && (({1'b0, widx_d} + 5'd1) >= fd_count)) begin
                    widx <= 4'd0;
                    st   <= B_WALK2;
                end else
                    widx <= widx + 4'd1;
            end

            //------------------------------------------------ 走表（第 2 遍：配对）
            // 为什么要回头重走
            //     配对需要"WAV 的序号"和"BMP 的序号"同时在手，而表读口一次只
            //     给得出一条目录项。第 1 遍已经把 4 张 BMP 的序号抄进寄存器了，
            //     第 2 遍只须再逐条看 WAV。16 条 = 16 拍 = 160ns，代价可忽略。
            // ★ 判定同样延一拍（见"判定流水"），m_hit_r/m_k_r 与 is_*_d、
            //   clu_d/size_d 必须同拍 —— 它们在同一个流水寄存器块里一起打的拍。
            //   widx_d < fd_count 屏蔽过渡拍垃圾（进出状态那一拍）。
            B_WALK2: begin
                if (is_wav_d && m_hit_r && (widx_cont)) begin
                    case (m_k_r)
                    2'd0: begin wav_clu_0 <= clu_d; wav_sz_0 <= size_d; wav_has_0 <= 1'b1; end
                    2'd1: begin wav_clu_1 <= clu_d; wav_sz_1 <= size_d; wav_has_1 <= 1'b1; end
                    2'd2: begin wav_clu_2 <= clu_d; wav_sz_2 <= size_d; wav_has_2 <= 1'b1; end
                    default: begin wav_clu_3 <= clu_d; wav_sz_3 <= size_d; wav_has_3 <= 1'b1; end
                    endcase
                end

                // S2：同一条通道里顺手把 .SUB 也配到同序号的图上
                // ent_is_sub 与 sub_m_hit 的与项是硬规则，不许省
                //   省掉之后 01.SUB 的 nm2="01" 会命中 bmp_nm2_0，
                //   于是它被当成 01.BMP 的字幕、或抢掉 01.WAV 的槽。
                if (is_sub_d && sub_hit_r && (widx_cont)) begin
                    case (sub_k_r)
                    2'd0: begin sub_idx_0 <= widx_d; sub_has_0 <= 1'b1; end
                    2'd1: begin sub_idx_1 <= widx_d; sub_has_1 <= 1'b1; end
                    2'd2: begin sub_idx_2 <= widx_d; sub_has_2 <= 1'b1; end
                    default: begin sub_idx_3 <= widx_d; sub_has_3 <= 1'b1; end
                    endcase
                end

                // 退出条件与 B_WALK 同构：比较 widx_d，多停一拍让最后一拍判定落地
                if ((widx_cont) && (({1'b0, widx_d} + 5'd1) >= fd_count))
                    st <= B_WAIT;
                else
                    widx <= widx + 4'd1;
            end

            //---------------------------------------------------------- 等放行
            B_WAIT: begin
                // o_ready 已经是 1；等 sd_card_bmp 的清零那一拍过去
                if (i_list_go)
                    st <= B_PLAY;
            end

            //---------------------------------------------------------- 播名单
            B_PLAY: begin
                if (!o_scan_found_valid) begin
                    // 第 1 拍：把"下标"摆到总线上
                    o_scan_found_valid  <= 1'b1;
                    o_scan_found_sector <= {28'd0, bmp_idx_sel(pidx)};
                end else begin
                    // 第 2 拍：sd_card_bmp 已在本拍采到（非阻塞，值已稳定）
                    if (({1'b0, pidx} + 5'd1) >= {1'b0, bmp_cnt})
                        st <= B_DONE;
                    else
                        pidx <= pidx + 4'd1;
                end
            end

            //---------------------------------------------------------- 完成
            B_DONE: begin
                o_scan_done <= 1'b1;
                st          <= B_DONE;    // 只跑一次；想看新结果就复位重来
            end

            default: st <= B_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 阶段 4：把"WAV 字节数"换算成"图片停留拍数"
    //   两级流水（见下面 wav_dwell_calc 的说明），输入在 B_WALK 就稳定了，
    //   输出 2 拍后有效。使用者（sd_card_bmp）要到图片提交那一拍才采样，
    //   比这里晚约 0.85 秒（≈8.5e7 拍），所以不需要额外的 valid 握手。
    //
    // 阶段 3c：输入从哪来由 USE_PAIR_DWELL 这个编译期常量选
    //     =0（旧）：表里第一个 WAV 的字节数（wav_size_r / wav_seen）
    //     =1（3c）：i_wav_sel 指定的那张图配对到的 WAV 的字节数
    //   两种情况下 wav_dwell_calc 都只有一个实例，所以乘法器还是一个，
    //   资源不增；选择是一个常量 mux，综合时直接折叠。
    //
    // 为什么这里不需要"选择下标变化后等 2 拍"
    //     i_wav_sel 由 sd_card_bmp 接成"正在提交的那张图"（load_idx），
    //     它在装载开始那一拍就定了，到提交（≈0.85 秒后）中间不动，
    //     两级流水的延迟（2 拍）远在这之前就消化掉了。
    //-------------------------------------------------------------------------
    wire [31:0] dw_size_w  = (USE_PAIR_DWELL != 0) ? sel_size : wav_size_r;
    wire        dw_found_w = (USE_PAIR_DWELL != 0) ? sel_has  : wav_seen;

    wav_dwell_calc #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ)
    ) u_wav_dwell_calc (
        .clk         (clk),
        .rst         (rst),
        .i_wav_found (dw_found_w),
        .i_wav_size  (dw_size_w),
        .o_ticks     (wav_ticks_w),
        .o_valid     (wav_dur_ok_w)
    );

    //-------------------------------------------------------------------------
    // 被封装的主角：fat32_dir（一个字都没改）
    //   本模块额外用到它的文件表读口和卷几何输出，
    //   这正是 3b 与 3a 的区别（3a 只看了条数和错误码）。
    //-------------------------------------------------------------------------
    fat32_dir #(
        .MAX_FILES    (MAX_FILES),
        .TO_LIMIT     (TO_LIMIT),
        .TO_LIMIT_ALL (TO_LIMIT_ALL)
    ) u_fat32_dir (
        .clk           (clk),
        .rst_n         (rst_n_i),

        .start         (fd_start),
        .busy          (fd_busy),
        .done          (fd_done),
        .err_code      (fd_err),

        .o_rd_lba      (fd_lba),
        .o_rd_req      (fd_req),
        .i_data        (i_data),
        .i_data_valid  (fd_dvalid),
        .i_data_last   (i_data_end),
        // 授权指示原样透传（只影响超时计时，不影响数据通路）
        .i_granted     (i_granted),

        .o_count       (fd_count),
        .i_tbl_idx     (fd_tbl_idx),
        .o_tbl_name    (o_tbl_name),
        .o_tbl_cluster (o_tbl_cluster),
        .o_tbl_size    (o_tbl_size),
        .o_tbl_attr    (o_tbl_attr),

        .tgt_we        (1'b0),
        .tgt_name      (88'd0),

        .o_fat_lba      (o_fat_lba),
        .o_data_lba     (o_data_lba),
        .o_sec_per_clus (o_sec_per_clus),
        .o_spc_lg2      (o_spc_lg2),

        .dbg_root_lba  (o_dbg_root_lba),
        .dbg_part_lba  (o_dbg_part_lba),
        .dbg_data_lba  (o_dbg_data_lba),
        .dbg_fat_lba   (o_dbg_fat_lba),
        .dbg_secs      (o_dbg_secs),
        .dbg_part_type (o_dbg_part_type),
        .dbg_root_b0   (o_dbg_root_b0),
        .dbg_root_b1   (o_dbg_root_b1),
        .dbg_rx_bytes  (o_dbg_rx_bytes)
    );

endmodule


//=============================================================================
// 模块名  : wav_dwell_calc   （阶段 4 新增，故意和 fat32_boot 放在同一个 .v 里）
// 功能    : 把"讲解 WAV 的字节数"换算成"图片停留拍数"（含兜底与上限）
//
// 为什么放在 fat32_boot.v 里，而不是新建一个 .v
// ---------------------------------------------
//   本工程有一条硬约束：**新增 .v 就必须让用户在 TD 里手动 Add Source File**，
//   而"从外部改 .al/.prj"不可靠（用户实测 TD 打开时会用内存里的旧工程树
//   覆盖回去，白构建一次）。把换算逻辑放进已有的 fat32_boot.v，
//   TD 侧就是零改动 —— 一个 .v 里放两个 module 对 iverilog / TD 都合法。
//
// 公式（以及它成立的前提）
// ------------------------
//   本项目 tools/make_tf_wav.py 生成的 WAV 是规范布局，音频数据固定从偏移 44
//   开始：
//       音频数据字节数 payload = size - 44
//       音频格式 48000Hz / 16bit / 单声道 -> 每秒 96000 字节
//       音频时长(拍) = payload x CLK_FREQ_HZ / 96000
// 前提（必须写清）
//     size 必须是"规范 44 字节头的 WAV"的大小。若卡上放的是带 LIST/INFO 等
//     额外 chunk 的 WAV（数据起点 > 44），这里算出的时长会偏长，
//     偏差 = 额外 chunk 字节数 / 96000 秒。本模块不解析文件头是刻意的：
//     字节数已经在 FAT 文件表里（阶段 1 验证过），再读一次 WAV 头等于上电
//     阶段多占一次 SD 卡，而那时卡正忙着装载首图。
//
// 精度（唯一一处损失，写在这里备查）
// --------------------------------
//   TICKS_PER_BYTE = CLK_FREQ_HZ / 96000 取整数商（向下取整）：
//       100MHz : 100_000_000 / 96000 = 1041.666...  -> 1041
//       相对误差 = (1041.6667-1041)/1041.6667 = 0.064%
//       8.243 秒的讲解 -> 少算 5.3 毫秒（对"图片停留多久"完全无感）
//   若 CLK_FREQ_HZ < 96000（本工程不可能，但防参数被改坏）-> TICKS_PER_BYTE=0
//   -> 时长恒 0 -> 被下面"太短"的判据兜成 5 秒，绝不会算出 0 拍去疯狂切图。
//
// 兜底 / 上限
// ----------
//   * 表里没有 WAV（i_wav_found=0）        -> o_valid=0，o_ticks=兜底 5 秒
//   * size <= 44（空文件 / 连头都不够）    -> o_valid=0，兜底 5 秒
//     （size=44 正好 0 时长；size<44 连减法都会下溢，两者都归到这里）
//   * 音频不足 MIN_AUDIO_SEC(1 秒)         -> o_valid=0，兜底 5 秒
//   * 音频超过 MAX_AUDIO_SEC(60 秒)        -> 钳位到 60 秒，o_valid=1
// 为什么上限取 60 秒，以及为什么必须先加宽到 33 位
//     100MHz 下 60 秒 = 60 亿拍，而 2^32 = 42.95 秒 —— 32 位装不下 60 秒。
//     所以本次把停留计时链（本模块 o_ticks / wav_ticks_w，以及 sd_card_bmp
//     的 auto_cnt / tick_target）一起加宽到 33 位：
//       33 位 @100MHz 上限 = 2^33/1e8 = 85.899 秒
// 最坏情况 = 60 秒音频 + 起播延迟 1.5 秒 + 停顿 1 秒 = 62.5 秒 < 85.9 秒
// 加宽与钳位必须同时做 ：只把 39 改成 60 而不加宽，60 亿拍写进
//       32 位会被截成很小的值 —— 恰好退化成"疯狂切图"，与加保护的目的相反。
//
// 位宽（本项目踩过多次的坑，全部显式写死）
// ----------------------------------------
//   1) 输出/中间量一律 33 位：o_ticks、ticks_x、ticks_use
//   2) 先按字节数封顶：payload <= 96000 x 60 = 5_760_000，避免"先乘后溢出"
//   3) 乘积中间量用显式 48 位：payload(<=23 位) x TICKS_PER_BYTE(16 位)
//        5_760_000 x 1041 = 5_996_160_000 < 2^33 —— 33 位装得下，
//        但这里仍然用 48 位接住（多两三位的进位链，换任何时钟都不静默截断）
// 4) 绝不用 integer / 32 位字面量算 CLK_FREQ_HZ x 60
// 100_000_000 x 60 = 6.0e9 > 2^31-1（integer 是有符号 32 位）-> 溢出成负数
// 6.0e9 也 > 2^32-1（无符号 32 位装不下）
// → 所以常量算式全部改用 64 位字面量：64'd100_000_000 * 64'd60 = 6e9
//          再用位选截到实际需要的宽度（本模块 48 位中间量）
// 为什么不在 localparam 左边写 [32:0]：Verilog 是"先按表达式自身宽度
//          算完再截到目标宽度"，只要算式本身用 64 位就不会溢出；但为了让
//          verilog_width_check.py 也能一眼看懂，下面每个字面量都显式带 64'd。
//=============================================================================
module wav_dwell_calc #(
    parameter integer CLK_FREQ_HZ      = 100_000_000,
    // 规范 WAV 头长度（tools/make_tf_wav.py 生成的就是 44）
    parameter integer WAV_HDR_BYTES    = 44,
    // 48000Hz x 16bit x 单声道
    parameter integer WAV_BYTES_PER_SEC= 96000,
    // 兜底停留：与阶段 4 之前的固定周期一致
    parameter integer FALLBACK_SEC     = 5,
    // 时长下限：不足它就当"数据不可信"，走兜底
    parameter integer MIN_AUDIO_SEC    = 1,
    // 时长上限：见文件头"为什么是 60 秒"
    // 本次：39 -> 60 必须与 o_ticks / sd_card_bmp 的 33 位加宽同时生效
    parameter integer MAX_AUDIO_SEC    = 60
)(
    input               clk,
    input               rst,            // 高有效异步复位（本工程约定）
    input               i_wav_found,    // 1 = FAT 文件表里确实有 WAV
    input      [31:0]   i_wav_size,     // 该 WAV 的字节数（文件表里的 size）
    // 本次加宽 32 -> 33 位：60 秒 @100MHz = 60 亿拍 > 2^32-1（见文件头）
    output reg [32:0]   o_ticks,        // 停留拍数（无效时 = 兜底 5 秒）
    output reg          o_valid         // 1 = o_ticks 来自 WAV 大小，不是兜底
);

    //-------------------------------------------------------------------------
    // 编译期常量（全部显式位宽，避免 Verilog 的"上下文宽度"陷阱）
    // 本次：凡是"乘完会超过 2^31-1（甚至 2^32-1）"的算式，
    // 一律写成 64 位字面量（64'd...），算完再截到需要的宽度
    // 理由见文件头第 4) 条：100_000_000 x 60 = 6e9，用 32 位算会溢出
    //-------------------------------------------------------------------------
    localparam [15:0] TPB_16 = CLK_FREQ_HZ / WAV_BYTES_PER_SEC;   // 100MHz -> 1041
    localparam [63:0] CLK_64 = CLK_FREQ_HZ;                       // 见文件头 4)：必须 64 位
    localparam [63:0] BPS_64 = WAV_BYTES_PER_SEC;
    localparam [63:0] HDR_64 = WAV_HDR_BYTES;
    // 上限对应的字节数/ 拍数 / 兜底拍数（全部 64 位算，再截到使用宽度）
    localparam [63:0] MAX_PAYLOAD_64  = BPS_64 * MAX_AUDIO_SEC;   // 5_760_000
    localparam [63:0] MAX_SIZE_64     = MAX_PAYLOAD_64 + HDR_64;  // 5_760_044
    localparam [63:0] MAX_TICKS_64    = CLK_64 * MAX_AUDIO_SEC;   // 6_000_000_000
    localparam [63:0] FALLBACK_TICKS_64 = CLK_64 * FALLBACK_SEC;  //   500_000_000
    // 音频至少 MIN_AUDIO_SEC 秒 -> 文件至少这么多字节（"太短"的判据）
    localparam [63:0] MIN_SIZE_64     = BPS_64 * MIN_AUDIO_SEC + HDR_64;  // 96_044
    // ---- 截到本次实际使用的宽度（33 位计时链 / 48 位乘积）----
    // 字节数封顶比较用 32 位就够（5_760_044 < 2^32）
    localparam [31:0] HDR_32          = HDR_64[31:0];             // 44
    localparam [31:0] MAX_SIZE        = MAX_SIZE_64[31:0];
    localparam [31:0] MIN_SIZE        = MIN_SIZE_64[31:0];
    //   上限拍数 60 亿 > 2^32，所以必须 33 位（本模块唯一需要 33 位的地方之一）
    localparam [32:0] MAX_TICKS       = MAX_TICKS_64[32:0];
    localparam [32:0] FALLBACK_TICKS  = FALLBACK_TICKS_64[32:0];

    //-------------------------------------------------------------------------
    // 为什么这里是两级流水而不是一段纯组合
    //   第一版写成纯组合（钳位 -> 减法 -> 乘法 -> 比较 -> 兜底 mux），
    //   真 TD 综合报出这条路径 Setup Slack = -10.907ns（数据路径 20.791ns、
    //   逻辑级数 16、其中 12 级是 ADDER）—— 因为"和 3744000 / 39 亿这两个
    //   常量比较"被综合成很长的进位链，而 wav_size_r 到 tick_target 只隔 1 拍，
    //   那是实打实的建立时间违例（不是多周期路径）。
    //   拆成两级流水后每一级的组合深度都只有 2~3 层，静态时序轻松通过。
    //
    //   代价：输出比输入晚 2 拍。使用者（fat32_boot 的调用方 sd_card_bmp）
    //   在第一张图提交那一拍才采样，比 wav_size_r 稳定晚了约 0.85 秒
    //   （≈8.5e7 拍），余量极大，不需要额外的 valid 握手。
    //-------------------------------------------------------------------------

    // ---- 第 1 级：封顶（按字节数比，等价于原来的"payload 封顶"）+ 减头 ----
    //   等价性：min(size, MAX_SIZE) - 44 == min(size-44, MAX_PAYLOAD)
    //   （size>=44 时成立；size<44 时结果下溢成一个大数，但下面 ok_r 会挡住它）
    wire [31:0] size_use = (i_wav_size > MAX_SIZE) ? MAX_SIZE : i_wav_size;
    wire [31:0] payload_raw = size_use - HDR_32;
    // 合法判据：表里有 WAV、文件至少 MIN_SIZE 字节
    // 一个比较就同时覆盖了"size>44"和"音频不足 1 秒"两条
    //     （MIN_SIZE = 96044 > 44）
    // 为什么用字节数判"太短"而不是用算出来的拍数判
    //     否则"正好 1.000 秒"会因为 TICKS_PER_BYTE 向下取整变成 0.99936 秒
    //     而被误判成太短，逻辑上说不通。
    wire        size_ok = i_wav_found && (i_wav_size >= MIN_SIZE);

    reg  [31:0] payload_r;
    reg         ok_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            payload_r <= 32'd0;
            ok_r      <= 1'b0;
        end else begin
            payload_r <= payload_raw;
            ok_r      <= size_ok;
        end
    end

    // ---- 第 2 级：乘（显式 48 位中间量）-> 溢出判 -> 兜底 mux ----
    // 写法说明 写成 `{16'd0, payload_r} * TPB_16` 而不是
    //     `{16'd0,payload_r} * {32'd0,TPB_16}`：后者会让
    //     tools/verilog_width_check.py 的启发式把 32 位字面量也算进"乘积至少
    //     需要 32+48=80 位"，从而对本行报"上下文宽度"告警（它只看字面量的
    //     声明位宽，不看值）。两种写法综合结果完全一样（TPB_16 本来就是 16 位）。
    // 本次：40 -> 48 位 上限乘积从 3.897e9 涨到 5.996e9（> 2^32），
    // 48 位接住后低 33 位就是本次的停留拍数（5_996_160_000 < 2^33）
    wire [47:0] ticks_x = {16'd0, payload_r} * TPB_16;
    // 溢出用 16 位或非门判（比 48 位比较器浅得多）：正常情况下 payload 已封顶，
    // 乘积 <= 5.996e9 装得下 33 位；这一支是"万一有人把 CLK_FREQ_HZ 调很高"的兜底。
    wire        ticks_ovf   = |ticks_x[47:33];
    // 本次加宽 32 -> 33 位 这一行原来写 ticks_x[31:0]，
    // 在 60 秒上限下会把 60 亿拍的高位丢掉（= 截断成很小的值）
    wire [32:0] ticks_use   = ticks_ovf ? MAX_TICKS : ticks_x[32:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            o_ticks <= FALLBACK_TICKS;
            o_valid <= 1'b0;
        end else begin
            // 无效（没 WAV / 太小 / 太短）一律给兜底 5 秒，绝不给 0
            o_ticks <= ok_r ? ticks_use : FALLBACK_TICKS;
            o_valid <= ok_r;
        end
    end

endmodule
