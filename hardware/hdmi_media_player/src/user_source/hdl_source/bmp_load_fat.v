//=============================================================================
// 文件名  : bmp_load_fat.v
// 功能    : 阶段 3b 的「按 FAT 文件表装载一张 BMP」模块。
//           对外接口与 bmp_read.v逐端口对齐，便于 sd_card_bmp 做二选一：
//             ready / state_code / sd_init_done / bmp_width / bmp_height /
//             load_start / load_abort / write_req / write_req_ack /
//             sd_sec_read* / bmp_data_wr_en / bmp_data
//           区别只有一个：bmp_read 用起始扇区当装载目标，
//           本模块用FAT 文件表下标当装载目标（load_tbl_idx）。
//
// 数据流（为什么这样分两相）
// --------------------------
//   相 1（L_HDR）：直接读"该文件首簇的第 0 个扇区"（一个扇区而已），
//                  用与 bmp_read.v 完全相同的字段偏移解析 BMP 头
//                  （BM / 文件大小 / 像素偏移 / 宽 / 高 / 位深 / 压缩）。
//                  头合法 -> 拉 write_req，等帧缓冲的 ack。
// 相 2（L_DATA）： 这时才启动 fat32_file.v ，让它从文件第 0 字节开始
//                  跟簇链吐字节；本模块拿这些字节做与 bmp_read.v 的
//                  ST_LOAD_DATA 完全相同的"跳过头 + 3 字节拼像素"处理。
//
// 为什么不是"一遍流下来"（本模块的关键取舍）
//   bmp_read 的 ST_LOAD_DATA 是重新从文件第 0 字节读一遍来做像素的：
//   因为帧缓冲要等 write_req_ack（frame_fifo_write 在 ack 那一拍才复位 FIFO），
//   ack 之前写进去的像素会被丢掉。
//   如果只用一条流：解析完头（前 512 字节，其中 54 字节之后就是像素）时
//   ack 还没来，这 458 个像素字节就没地方放（想缓存就要 512 字节 BRAM，
//   想暂停 fat32_file 又会踩它的等扇区超时）。
//   所以这里照抄 bmp_read 的两相结构：头单独读一个扇区，像素从第 0 字节
//   重新流一遍。代价是每张图多读 1 个扇区（约 0.5ms/张），
//   换来的是"fat32_file 在 ack 期间根本没启动"，不存在任何暂停/超时问题。
//
// 与 bmp_read.v 保持逐拍一致的三处（不要"顺手简化"）
// --------------------------------------------------
//   (1) 像素门控：(字节偏移 >= pixel_offset) && (字节偏移 < file_len)
//       偏移从文件第 0 字节起算，与 bmp_read 的 bmp_len_cnt 同一语义。
//       file_len 取 BMP 头里的值（不是目录项大小），与 bmp_read 一致。
//   (2) 3 字节拼一个像素：byte_idx 只在像素字节上推进；第 3 个字节
//       那一拍才 bmp_data_wr_en=1，且 bmp_data 与使能同拍（错一拍整屏错位）。
//   (3) 头字段偏移与 header_match 的条件、以及"不匹配就放弃本次装载"的行为，
//       全部照抄 bmp_read.v（含 width/height/bit_count/compression 的检查）。
//
// 结束条件
// --------
//   off_cnt >= file_len（文件字节收齐）或 fat32_file done（链结束/出错）。
//   两者取先到者 —— 任何一条路都不会挂死（挂死还有 sd_card_bmp 的 3 秒看门狗）。
//
// 位宽（本项目踩过多次的坑，全部显式写死）
// ----------------------------------------
//   首扇区 = 数据区起始 + (首簇 - 2) << log2(每簇扇区数)：
//   中间量 40 位接住，确认 |xxx[39:32] == 0 才认为是合法扇区号。
//=============================================================================

`timescale 1ns / 1ps

module bmp_load_fat #(
    // 透传给 fat32_file 的等扇区超时（拍）/ 总等待兜底 / 簇链硬顶
    //   （2026-10 超时修复：TO_LIMIT 16384 -> 2_000_000，并新增 TO_LIMIT_ALL，
    //     完整理由见 fat32_dir.v 与 fat32_file.v 的同名参数注释）
    parameter [31:0] TO_LIMIT       = 32'd2000000,
    parameter [31:0] TO_LIMIT_ALL   = 32'd200000000,
    parameter [31:0] CHAIN_HARD_MAX = 32'd1048576
)(
    input               clk,
    input               rst,              // 高有效异步复位（本工程约定）

    // ---- 与 bmp_read 对齐的状态接口 ----
    output              ready,            // 1 = 空闲，可以开始新的装载
    output reg [3:0]    state_code,       // 与 bmp_read 同编码：1=IDLE 2=HDR 3=WAIT 4=DATA

    input               sd_init_done,
    input      [15:0]   bmp_width,        // 期望宽度（顶层给 640）
    input      [15:0]   bmp_height,       // 期望高度（顶层给 480）

    // ---- 装载请求 ----
    input               load_start,       // 单拍脉冲
    input               load_abort,       // 看门狗中止（单拍脉冲）
    input      [3:0]    load_tbl_idx,     // 3b：FAT 文件表下标（不是扇区号）

    // ---- FAT 文件表读口（组合读；下标 -> 首簇/大小/名字）----
    output     [3:0]    o_tbl_idx,
    input      [87:0]   i_tbl_name,
    input      [31:0]   i_tbl_cluster,
    input      [31:0]   i_tbl_size,
    input      [7:0]    i_tbl_attr,

    // ---- 卷几何（来自 fat32_dir，只有一处实现）----
    input      [31:0]   i_fat_lba,
    input      [31:0]   i_data_lba,
    input      [7:0]    i_sec_per_clus,
    input      [4:0]    i_spc_lg2,

    // ---- 帧缓冲写请求（与 bmp_read 完全一致）----
    output reg          write_req,
    input               write_req_ack,

    // ---- SD 扇区读（接 sd_arbiter3 的 bmp_* 最高优先级那一路）----
    output              sd_sec_read,
    output     [31:0]   sd_sec_read_addr,
    input      [7:0]    sd_sec_read_data,
    input               sd_sec_read_data_valid,
    input               sd_sec_read_end,
    // ---- 仲裁器"正在传输"标志（sd_arbiter3 的 o_busy）----
    // 为什么必须要有它（这是仿真里抓到的一个真 bug，别删）
    //   本模块在文件读完的那一刻就把 fat32_file 按回复位、请求随之落下，
    //   但卡侧那一次扇区传输还在继续—— 仲裁器要等它自己的 sd_end
    //   才释放（这是"许可式锁存"的必然结果）。于是紧接着的下一次装载
    //   如果立刻发首扇区请求，仲裁器会因为"还有一次传输没结束"而
    //   先不受理，而那一次传输剩下的字节仍然会从 bmp 口回来 ——
    //   本模块在 L_HDR 里会把这些不属于自己的字节当成 BMP 头收下，
    //   头字段全乱 -> header_match 失败 -> err=1、一个字节都读不出来。
    //   仿真里表现为"连着装载时偶数张成功、奇数张全失败"。
    //   修法：进 L_HDR 之前等仲裁器空闲，保证"我请求的那一次"才是
    //   我收到的第一次传输。（真板上两次装载之间隔了 5 秒，撞不上，
    //   但这是真实存在的隐患，必须修。）
    input               i_sd_busy,

    // ---- 2026-10 超时修复新增：本路"已被仲裁器授权用卡" ----
    //   本模块的两次扇区读取都走bmp 那一路（L_HDR 相自己拉 sd_sec_read；
    //   数据相由 fat32_file 拉），所以顶层给它接的是 sd_arbiter3 的
    //   o_grant_bmp。它内部只把这一路透传给 fat32_file 做超时计时用：
    //   排队等别人（wav 等）让出卡的时间不算传输超时。
    // 没有仲裁器的场合接 1'b1（退化成修复前行为）
    input               i_granted,

    // ---- S2 新增：本次装载是不是"滚动字幕掩码条 0N.SUB" ----
    //   1 = 走字幕通路：不碰帧缓冲、不产像素，只把 fat32_file 吐出来的字节
    //       按"6 字节 = 一列 48bit"拼好、写给顶层的掩膜 RAM。
    // 复用同一个 fat32_file 实例 + 同一条 bmp 仲裁通路
    //     于是"图片装载"与"字幕装载"天然时间互斥（S2 不新增第 4 路仲裁）。
    // 位序/字节序（与 tools/make_subtitle.py 的 .SUB 格式定义一一对应）
    //       第 k 列 = 文件偏移 16 + k*6 起的 6 个字节；
    //       payload[k*6 + j] = (b >> (8*j)) & 0xFF   （48bit 小端）
    //       b 的 bit y = 条内第 y 行的像素，bit0 = 最上面一行
    //     拼接写法（见 L_DATA）：w <= (w >> 8) | (byte << 40)
    //       —— "先收到的字节放低位"，第 6 个字节那一拍 w 正好是完整的 b。
    // 注意：这个式子与"w <= {w[39:0],8'h00} | byte"不等价
    //       后者把先收到的字节放到高位，等于把整列上下颠倒 + 字节序反了。
    input               i_job_sub,

    // ---- S2 新增：掩膜 RAM 写口（只在字幕通路上有效；其余时候 we 恒 0）----
    output reg          o_mask_we,        // 单拍写使能：一列一个 48bit 字
    output reg [10:0]   o_mask_addr,      // 写到第几列（0..2047）
    output reg [47:0]   o_mask_wdata,     // 该列的 48 条扫描线（bit y = 条内第 y 行）

    // ---- S2 新增：字幕装载结果（给顶层诊断 + 门控用）----
    output reg [11:0]   o_sub_cols,       // 本次写进去多少列（完整装载 = 2048）
    output reg          o_sub_ok,         // 1 = 本次字幕装载完整成功（头/字节数/列数都对）
    output              o_sub_busy,       // 1 = 正在做字幕装载（给 sd_card_bmp 门控图片装载）

    // ---- 像素输出（与 bmp_read 完全一致）----
    output reg          bmp_data_wr_en,
    output reg [23:0]   bmp_data,

    // ---- 结果 / 调试 ----
    // 本次装载的源尺寸类别：1 = 1280×960（走 2:1 缩小），0 = 640×480（直通）
    //   ★ 必须是寄存器输出且一直保持到下一次装载：下游缩放器在像素流全程用它选路径
    output reg          o_src_2to1,

    // 装载进度（0..255）与"装载中"标志：字幕带画进度条用
    //   o_prog 单调递增，只在 load_start 时清零；用 gray 码跨到 25MHz 域
    output reg  [7:0]   o_prog,
    output              o_busy,

    output     [3:0]    o_err_code,       // 0=无错 1=头不匹配 2=表项非法 3=簇链出错 15=被中止
    output     [31:0]   o_bytes_out,      // 本次从文件里取到的字节数（正常 = 文件大小）
    output     [7:0]    o_dbg_fdat,       // 观察窗：fat32_file 吐出的字节流
    output              o_dbg_fdv,        // 观察窗：上面那个字节的使能
    output              o_dbg_ff_done,    // 观察窗：fat32_file 是否结束
    output     [3:0]    o_dbg_ff_err      // 观察窗：fat32_file 的错误码
);

    //-------------------------------------------------------------------------
    // 错误码（0 = 成功）
    //-------------------------------------------------------------------------
    localparam [3:0] ERR_NONE  = 4'd0,
                     ERR_HDR   = 4'd1,   // BMP 头不匹配（与 bmp_read 的放弃行为一致）
                     ERR_TBL   = 4'd2,   // 文件表项非法（首簇/大小/扇区号算不出来）
                     ERR_FAT   = 4'd3,   // fat32_file 报错（坏链/超时/大小不符）
                     ERR_ABORT = 4'd15;  // 被看门狗中止

    //-------------------------------------------------------------------------
    // S2 新增：字幕掩膜条（0N.SUB）的编译期常量
    //   与 tools/make_subtitle.py 的格式定义一一对应（S1 报告 §3）：
    //     文件 = 16 字节头 + 2048 列 x 6 字节 = 12,304 字节
    //   本模块只认 FPGA v1 读得动的那一种（2048x48/1bpp）——
    //   头不合法、或目录项大小连 12,304 都不到，就按 ERR_TBL/ERR_HDR 放弃
    //   （fail-soft：不写一个字节进 RAM，播放/图片完全不受影响）。
    //-------------------------------------------------------------------------
    localparam [31:0] SUB_COLS       = 32'd2048;             // 掩膜总列数
    localparam [31:0] SUB_PAY_BYTES  = 32'd12288;            // 2048 x 6
    localparam [31:0] SUB_FILE_BYTES = 32'd12304;            // 16 + 12288
    localparam [11:0] SUB_COLS_12    = 12'd2048;             // 完整装载后的列数

    //-------------------------------------------------------------------------
    // 主状态机
    //   L_IDLE : 空闲（fat32_file 被按在复位里，不会去请求卡）
    //   L_RST  : lat32_file 复位一拍，把上一次的簇链状态清干净
    //   L_SET  : 参数落定一拍 + 合法性检查（见文件头"要点 B"那类坑）
    //   L_HDR  : 直读首扇区、解析 BMP 头
    //   L_ACK  : 等帧缓冲的 write_req_ack
    //   L_DATA : fat32_file 流式吐字节 -> 拼像素
    //   L_DONE : 收尾
    //-------------------------------------------------------------------------
    localparam [2:0] L_IDLE = 3'd0,
                     L_RST  = 3'd1,
                     L_SET  = 3'd2,
                     L_HDR  = 3'd3,
                     L_ACK  = 3'd4,
                     L_DATA = 3'd5,
                     L_DONE = 3'd6;

    reg [2:0]  st;

    // 参数锁存
    reg [3:0]  tbl_idx_r;
    reg [31:0] clu_r;
    reg [31:0] size_r;

    // BMP 头字段（与 bmp_read 同名同宽）
    reg [7:0]  header_0;
    reg [7:0]  header_1;
    reg [31:0] file_len;
    reg [31:0] pixel_offset;
    reg [31:0] width;
    reg [31:0] height;
    reg [15:0] bit_count;
    reg [31:0] compression;

    reg [9:0]  rd_cnt;          // 头那一相：扇区内字节偏移
    reg [31:0] off_cnt;         // 数据相：本字节在文件里的偏移（0 起）
    reg [1:0]  byte_idx;        // 像素内字节序号 0/1/2
    reg [31:0] bytes_out_r;     // 从文件里取到的字节数
    reg [3:0]  err_code_r;

    // fat32_file 控制
    reg        ff_kill;         // 1 = 把 fat32_file 按在复位
    reg        fst_start;       // 单拍脉冲：启动一次读文件

    assign o_tbl_idx  = tbl_idx_r;
    assign o_err_code = err_code_r;
    assign o_bytes_out = bytes_out_r;
    assign ready      = (st == L_IDLE);

    //-------------------------------------------------------------------------
    // S2：字幕通路的内部状态
    //   sub_mode_r 在 L_IDLE 采 load_start 那一拍锁存（i_job_sub 与 load_start
    //   是同一拍到达的两个单拍脉冲），整个装载期间不变 ——
    //   所以它既可以当"这次走字幕通路"的判据，也可以当 o_sub_busy 的依据。
    //-------------------------------------------------------------------------
    reg        sub_mode_r;
    reg        sub_hdr_ok;      // 16 字节头的 4 项校验（偏移 15 那一拍落定）
    reg [2:0]  sub_bphase;      // 一列内 6 个字节的相位 0..5
    reg [11:0] sub_col;         // 正在收的列号（收满 2048 列后 = 2048）
    reg [47:0] sub_w;           // 拼装中的 48bit（收满 6 字节就是完整的一列）
    reg [7:0]  sub_h0;          // magic 第 1 字节（期望 'S'）
    reg [7:0]  sub_h1;          // magic 第 2 字节（期望 'U'）
    reg [7:0]  sub_h2;          // magic 第 3 字节（期望 'B'）
    reg [7:0]  sub_h3;          // magic 第 4 字节（期望 '1'）
    reg [7:0]  sub_wl;          // width 低字节（期望 0x00）
    reg [7:0]  sub_wh;          // width 高字节（期望 0x08 -> 0x0800 = 2048）
    reg [7:0]  sub_hl;          // height 低字节（期望 0x30）
    reg [7:0]  sub_hh;          // height 高字节（期望 0x00 -> 0x0030 = 48）
    reg [7:0]  sub_bpp;         // bpp（期望 0x01）

    //-------------------------------------------------------------------------
    // 首簇 -> 首扇区（40 位中间量）
    //-------------------------------------------------------------------------
    reg [39:0] hdr_lba_x;

    always @(*) begin
        hdr_lba_x = {8'd0, i_data_lba} + (({8'd0, clu_r} - 40'd2) << i_spc_lg2);
    end

    // 表项非法：首簇 < 2 / 高位超 32 位 / 大小连一个 BMP 头都装不下
    wire hdr_bad = (|hdr_lba_x[39:32]) || (clu_r < 32'd2) || (size_r < 32'd54);

    //-------------------------------------------------------------------------
    // BMP 头匹配（与 bmp_read.v 的 header_match 逐项相同）
    //-------------------------------------------------------------------------
    // 支持两种源尺寸：640×480（1:1 直通）与 1280×960（2:1 缩小）
    //   ★ 只放行这两档，不做"任意尺寸"：本工程的缩放器是定点 2:1，
    //     放行任意尺寸会让非 2:1 的图被当成 1280 行宽去拼，整幅斜切。
    //     要支持任意比例，得换成通用面积平均缩放器（约 450 LUT）。
    //   ★ 1280×3 = 3840 字节，是 4 的倍数，所以这两种尺寸都不触发 BMP 行填充，
    //     装载器现有的"连续 3 字节拼像素"写法可以完全不动。
    localparam [15:0] SRC_W_2TO1 = 16'd1280;
    localparam [15:0] SRC_H_2TO1 = 16'd960;

    wire is_src_1to1 = (width[15:0] == bmp_width)   && (height[15:0] == bmp_height);
    wire is_src_2to1 = (width[15:0] == SRC_W_2TO1)  && (height[15:0] == SRC_H_2TO1);

    wire header_match = (header_0 == "B") &&
                        (header_1 == "M") &&
                        (is_src_1to1 || is_src_2to1) &&
                        (bit_count    == 16'd24) &&
                        (compression  == 32'd0);

    //-------------------------------------------------------------------------
    // fat32_file：簇链跟随 + 按文件大小吐字节（一行未改）
    // 只在 L_DATA 相里把它放出来（其余时候 rst_n 被拉低）
    //   这样它绝不可能在我们等 ack / 空闲的时候去请求卡（也就不会踩
    //   它内部的"等扇区超时"）。
    //-------------------------------------------------------------------------
    wire        ff_rd_req;
    wire [31:0] ff_rd_lba;
    wire        ff_dv;
    wire [7:0]  ff_dat;
    wire        ff_done;
    wire        ff_busy;
    wire [3:0]  ff_err;
    wire [31:0] ff_bytes_out;
    wire [31:0] ff_clusters;
    wire [31:0] ff_lba_hash;
    wire [4:0]  ff_spc_lg2_dbg;
    wire [7:0]  ff_dsec_dbg;
    wire        ff_is_fat_dbg;
    wire [31:0] ff_rd_secs_dbg;

    // 送给 fat32_file 的数据有效：只在数据相透传（L_HDR 那一相它还在复位里）
    wire ff_dvalid = sd_sec_read_data_valid && (st == L_DATA);

    wire fst_rst_n = ~(rst | ff_kill);

    // ---- 送给仲裁器 bmp_* 那一路的请求：头那一相由本模块自己发，
    //      数据那一相完全交给 fat32_file（它才是真正跟簇链的人）。
    // 这两句必须写在 ff_rd_req / ff_rd_lba / hdr_lba_x 声明之后
    //        （TD 对前向引用报错；本工程已经在别处踩过）----
    assign sd_sec_read      = (st == L_HDR) ? 1'b1 : ff_rd_req;
    assign sd_sec_read_addr = (st == L_HDR) ? hdr_lba_x[31:0] : ff_rd_lba;

    assign o_dbg_fdat    = ff_dat;
    assign o_dbg_fdv     = ff_dv;
    assign o_dbg_ff_done = ff_done;
    assign o_dbg_ff_err  = ff_err;

    fat32_file #(
        .TO_LIMIT       (TO_LIMIT),
        .TO_LIMIT_ALL   (TO_LIMIT_ALL),
        .CHAIN_HARD_MAX (CHAIN_HARD_MAX)
    ) u_fat32_file (
        .clk            (clk),
        .rst_n          (fst_rst_n),

        .start          (fst_start),
        .i_first_clu    (clu_r),
        .i_size         (size_r),

        .i_fat_lba      (i_fat_lba),
        .i_data_lba     (i_data_lba),
        .i_spc          (i_sec_per_clus),

        .i_max_clusters (32'd0),

        .o_rd_lba       (ff_rd_lba),
        .o_rd_req       (ff_rd_req),
        .i_data         (sd_sec_read_data),
        .i_data_valid   (ff_dvalid),
        .i_data_last    (sd_sec_read_end),
        // 授权指示原样透传（只影响超时计时，不影响数据通路）
        .i_granted      (i_granted),

        .o_data         (ff_dat),
        .o_data_valid   (ff_dv),

        .busy           (ff_busy),
        .done           (ff_done),
        .err_code       (ff_err),

        .o_bytes_out    (ff_bytes_out),
        .o_clusters     (ff_clusters),
        .o_lba_hash     (ff_lba_hash),
        .dbg_spc_lg2    (ff_spc_lg2_dbg),
        .dbg_dsec       (ff_dsec_dbg),
        .dbg_is_fat     (ff_is_fat_dbg),
        .dbg_rd_secs    (ff_rd_secs_dbg)
    );

    //-------------------------------------------------------------------------
    // S2：字幕通路用到的组合量（必须写在 ff_dat / size_r 声明之后
    // 本工程在 TD 上踩过 HDL-5373「使用先于声明」）
    //-------------------------------------------------------------------------
    // 一列 6 字节的拼装： 先收到的字节放低位
    //   第 6 个字节那一拍：sub_w = b0 | b1<<8 | ... | b5<<40 = 完整的一列 b
    //   （b 的 bit y = 条内第 y 行；bit0 = 最上面一行，见端口注释）
    wire [47:0] sub_w_nx = (sub_w >> 8) | ({40'd0, ff_dat} << 40);

    // 送给 fat32_file 的读取长度：字幕通路固定只要 12,304 字节
    //   （目录项大小只用来判断"够不够"，实际读多少由本模块说了算）
    wire [31:0] ff_size_w = sub_mode_r ? SUB_FILE_BYTES : size_r;

    // 正在做字幕装载（给 sd_card_bmp 门控图片装载，避免两个活儿撞进同一个装载器）
    assign o_sub_busy = sub_mode_r && (st != L_IDLE);

    //-------------------------------------------------------------------------
    // 主状态机（本块是上面所有寄存器 + write_req / bmp_data* 的唯一驱动）
    //-------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st             <= L_IDLE;
            state_code     <= 4'd1;
            tbl_idx_r      <= 4'd0;
            clu_r          <= 32'd0;
            size_r         <= 32'd0;
            header_0       <= 8'd0;
            header_1       <= 8'd0;
            file_len       <= 32'd0;
            pixel_offset   <= 32'd54;
            width          <= 32'd0;
            height         <= 32'd0;
            bit_count      <= 16'd0;
            compression    <= 32'd0;
            rd_cnt         <= 10'd0;
            off_cnt        <= 32'd0;
            byte_idx       <= 2'd0;
            bytes_out_r    <= 32'd0;
            err_code_r     <= ERR_NONE;
            ff_kill        <= 1'b1;      // 复位期间 fat32_file 一直被按住
            fst_start      <= 1'b0;
            write_req      <= 1'b0;
            bmp_data_wr_en <= 1'b0;
            bmp_data       <= 24'd0;
            // S2：字幕通路的寄存器一并复位（含 RAM 写口，复位期间绝不写）
            sub_mode_r     <= 1'b0;
            sub_hdr_ok     <= 1'b0;
            sub_bphase     <= 3'd0;
            sub_col        <= 12'd0;
            sub_w          <= 48'd0;
            sub_h0         <= 8'd0;
            sub_h1         <= 8'd0;
            sub_h2         <= 8'd0;
            sub_h3         <= 8'd0;
            sub_wl         <= 8'd0;
            sub_wh         <= 8'd0;
            sub_hl         <= 8'd0;
            sub_hh         <= 8'd0;
            sub_bpp        <= 8'd0;
            o_mask_we      <= 1'b0;
            o_mask_addr    <= 11'd0;
            o_mask_wdata   <= 48'd0;
            o_sub_cols     <= 12'd0;
            o_sub_ok       <= 1'b0;
            o_src_2to1     <= 1'b0;
        end else if (!sd_init_done) begin
            // 卡重新初始化：作废当前这次装载（与 bmp_read 的 !sd_init_done 分支一致）
            st             <= L_IDLE;
            state_code     <= 4'd1;
            ff_kill        <= 1'b1;
            fst_start      <= 1'b0;
            write_req      <= 1'b0;
            bmp_data_wr_en <= 1'b0;
            err_code_r     <= ERR_NONE;
            o_mask_we      <= 1'b0;
        end else if (load_abort) begin
            // 看门狗中止：只中止"当前这一次装载"，其它状态保持
            st             <= L_IDLE;
            state_code     <= 4'd1;
            ff_kill        <= 1'b1;
            fst_start      <= 1'b0;
            write_req      <= 1'b0;
            bmp_data_wr_en <= 1'b0;
            err_code_r     <= ERR_ABORT;
            o_mask_we      <= 1'b0;
        end else begin
            // 默认：start 是单拍脉冲
            fst_start <= 1'b0;

            case (st)
            //====================================================== 空闲
            L_IDLE: begin
                state_code     <= 4'd1;
                write_req      <= 1'b0;
                bmp_data_wr_en <= 1'b0;
                ff_kill        <= 1'b1;

                if (load_start) begin
                    // 只锁下标，不在同一拍读首簇/大小
                    //   表读下标 o_tbl_idx 就是 tbl_idx_r，它本拍才被写进去，
                    //   组合读到的还是上一条的首簇（fat32_dir 要点 B 的同类坑）。
                    //   真正的读表放在 L_RST（下标已稳定）。
                    tbl_idx_r   <= load_tbl_idx;
                    err_code_r  <= ERR_NONE;
                    bytes_out_r <= 32'd0;
                    off_cnt     <= 32'd0;
                    byte_idx    <= 2'd0;
                    rd_cnt      <= 10'd0;
                    header_0    <= 8'd0;
                    header_1    <= 8'd0;
                    file_len    <= 32'd0;
                    pixel_offset<= 32'd54;
                    width       <= 32'd0;
                    height      <= 32'd0;
                    bit_count   <= 16'd0;
                    compression <= 32'd0;
                    clu_r       <= 32'd0;
                    size_r      <= 32'd0;
                    // S2：锁存"这次是图片还是字幕"，并把字幕通路的状态清干净
                    //   i_job_sub 与 load_start 是同一拍的两个单拍脉冲。
                    sub_mode_r  <= i_job_sub;
                    sub_hdr_ok  <= 1'b0;
                    sub_bphase  <= 3'd0;
                    sub_col     <= 12'd0;
                    sub_w       <= 48'd0;
                    sub_h0      <= 8'd0;
                    sub_h1      <= 8'd0;
                    sub_h2      <= 8'd0;
                    sub_h3      <= 8'd0;
                    sub_wl      <= 8'd0;
                    sub_wh      <= 8'd0;
                    sub_hl      <= 8'd0;
                    sub_hh      <= 8'd0;
                    sub_bpp     <= 8'd0;
                    // 结果先清 0：装到一半失败时数码管上看到的是 0，不是上一次的旧值
                    o_sub_cols  <= 12'd0;
                    o_sub_ok    <= 1'b0;
                    st          <= L_RST;
                end
            end

            //====================================================== 复位 fat32_file
            L_RST: begin
                // fat32_file 的复位释放（同步释放；异步复位是它自己的端口约定）
                ff_kill <= 1'b0;
                // 下标已稳定 -> 读表拿首簇/大小（下一拍起有效）
                clu_r   <= i_tbl_cluster;
                size_r  <= i_tbl_size;
                st      <= L_SET;
            end

            //====================================================== 参数落定 + 检查
            L_SET: begin
                state_code <= 4'd1;
                if (sub_mode_r) begin
                    // S2：字幕通路 —— 首簇合法 + 目录项大小至少装得下 12,304 字节
                    //   为什么用目录项大小当门槛而不是先读头：万一表项指到别的
                    //   文件（或 .SUB 被截断），在这里就能 fail-soft 退出，
                    //   一个字节都不写进掩膜 RAM。
                    if ((clu_r < 32'd2) || (size_r < SUB_FILE_BYTES)) begin
                        err_code_r <= ERR_TBL;
                        st         <= L_DONE;
                    end else if (i_sd_busy) begin
                        // 与 BMP 通路同一个理由：等卡真的空下来再开始，
                        // 否则上一次传输剩下的字节会被我当成自己的第一个字节
                        st <= L_SET;
                    end else begin
                        // 字幕通路不需要帧缓冲：不拉 write_req、不等 ack，
                        // 直接启动 fat32_file 从文件第 0 字节流一遍
                        //   （BMP 要分两相是因为 ack 之前写进去的像素会被丢，
                        //     字幕没有帧缓冲这个约束，所以省掉整整一相）
                        fst_start <= 1'b1;
                        st        <= L_DATA;
                    end
                end else if (hdr_bad) begin
                    err_code_r <= ERR_TBL;
                    st         <= L_DONE;
                end else if (i_sd_busy) begin
                    // 仲裁器还在为上一次（或别人）的传输忙：等它空下来再请求，
                    // 否则会把那一次传输剩下的字节当成自己的 BMP 头（见端口说明）
                    st <= L_SET;
                end else begin
                    st <= L_HDR;
                end
            end

            //====================================================== 读首扇区 + 解析头
            L_HDR: begin
                state_code <= 4'd2;

                if (sd_sec_read_data_valid) begin
                    if (rd_cnt <= 10'd511)
                        rd_cnt <= rd_cnt + 10'd1;
                    // 字段偏移与 bmp_read.v 的 case(rd_cnt) 逐条相同
                    case (rd_cnt)
                    10'd0 : header_0 <= sd_sec_read_data;
                    10'd1 : header_1 <= sd_sec_read_data;

                    10'd2 : file_len[7:0]   <= sd_sec_read_data;
                    10'd3 : file_len[15:8]  <= sd_sec_read_data;
                    10'd4 : file_len[23:16] <= sd_sec_read_data;
                    10'd5 : file_len[31:24] <= sd_sec_read_data;

                    10'd10: pixel_offset[7:0]   <= sd_sec_read_data;
                    10'd11: pixel_offset[15:8]  <= sd_sec_read_data;
                    10'd12: pixel_offset[23:16] <= sd_sec_read_data;
                    10'd13: pixel_offset[31:24] <= sd_sec_read_data;

                    10'd18: width[7:0]   <= sd_sec_read_data;
                    10'd19: width[15:8]  <= sd_sec_read_data;
                    10'd20: width[23:16] <= sd_sec_read_data;
                    10'd21: width[31:24] <= sd_sec_read_data;

                    10'd22: height[7:0]   <= sd_sec_read_data;
                    10'd23: height[15:8]  <= sd_sec_read_data;
                    10'd24: height[23:16] <= sd_sec_read_data;
                    10'd25: height[31:24] <= sd_sec_read_data;

                    10'd28: bit_count[7:0]  <= sd_sec_read_data;
                    10'd29: bit_count[15:8] <= sd_sec_read_data;

                    10'd30: compression[7:0]   <= sd_sec_read_data;
                    10'd31: compression[15:8]  <= sd_sec_read_data;
                    10'd32: compression[23:16] <= sd_sec_read_data;
                    10'd33: compression[31:24] <= sd_sec_read_data;
                    default: ;
                    endcase
                end else if (sd_sec_read_end) begin
                    // 头字段在上一拍已全部落定（非阻塞赋值），这里判最稳
                    if (header_match) begin
                        o_src_2to1 <= is_src_2to1;   // 全程保持，供缩放器选路
                        write_req  <= 1'b1;
                        st         <= L_ACK;
                    end else begin
                        err_code_r <= ERR_HDR;
                        st         <= L_DONE;
                    end
                end
            end

            //====================================================== 等帧缓冲 ack
            L_ACK: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    // 这一拍才启动 fat32_file（见文件头"为什么分两相"）
                    //   它要在 S_IDLE 看到 start；下一拍进 L_DATA，
                    //   第一拍 ff_dv 必然是 0（它还没开始读卡），不会丢字节。
                    fst_start <= 1'b1;
                    st        <= L_DATA;
                end
            end

            //====================================================== 流式取数据
            // S2：本状态在两种通路上含义不同
            //     图片通路（sub_mode_r=0）：字节 -> 3 字节拼一个像素（原逻辑，一字未改）
            //     字幕通路（sub_mode_r=1）：字节 -> 16 字节头校验 + 6 字节拼一列写 RAM
            L_DATA: begin
                state_code <= 4'd4;
                o_mask_we  <= 1'b0;      // 默认不写：只有收满一列那一拍才置 1

                if (sub_mode_r) begin
                    //---------------------------------------------------------
                    // S2：字幕掩膜通路
                    //---------------------------------------------------------
                    if (ff_dv) begin
                        if (off_cnt < 32'd16) begin
                            // ---- 头 16 字节 ----
                            // 只认 magic(0..3) / width(4,5) / height(6,7) / bpp(8)；
                            // 偏移 9~15（flags/speed/reserved）当前不影响装载，
                            // 不参与判据（flags.bit1 = 空掩码，载荷本来就全 0）。
                            case (off_cnt)
                            32'd0: sub_h0  <= ff_dat;
                            32'd1: sub_h1  <= ff_dat;
                            32'd2: sub_h2  <= ff_dat;
                            32'd3: sub_h3  <= ff_dat;
                            32'd4: sub_wl  <= ff_dat;
                            32'd5: sub_wh  <= ff_dat;
                            32'd6: sub_hl  <= ff_dat;
                            32'd7: sub_hh  <= ff_dat;
                            32'd8: sub_bpp <= ff_dat;
                            default: ;
                            endcase
                            if (off_cnt == 32'd15) begin
                                // 前 8 个字节在这一拍都已落定（它们至少早 7 拍）
                                sub_hdr_ok <= (sub_h0 == "S") && (sub_h1 == "U") &&
                                              (sub_h2 == "B") && (sub_h3 == "1") &&
                                              (sub_wl == 8'h00) && (sub_wh == 8'h08) &&
                                              (sub_hl == 8'h30) && (sub_hh == 8'h00) &&
                                              (sub_bpp == 8'h01);
                            end
                        end else if (off_cnt >= SUB_FILE_BYTES) begin
                            // 载荷已收满（16 + 12288 字节），不再写
                            o_mask_we <= 1'b0;
                        end else if (!sub_hdr_ok) begin
                            // 头不合法：立刻收手，一个字节都不写进掩膜 RAM
                            o_mask_we  <= 1'b0;
                            err_code_r <= ERR_HDR;
                            st         <= L_DONE;
                        end else begin
                            // ---- 载荷：6 字节一列， 先收到的字节放低位 ----
                            sub_w <= sub_w_nx;
                            if (sub_bphase == 3'd5) begin
                                // 第 6 个字节这一拍：sub_w_nx 就是完整的一列 b
                                o_mask_we    <= 1'b1;
                                o_mask_addr  <= sub_col[10:0];
                                o_mask_wdata <= sub_w_nx;
                                sub_bphase   <= 3'd0;
                                sub_col      <= sub_col + 12'd1;
                            end else begin
                                sub_bphase <= sub_bphase + 3'd1;
                            end
                        end

                        off_cnt     <= off_cnt + 32'd1;
                        bytes_out_r <= bytes_out_r + 32'd1;
                    end

                    // 结束：字节收齐（12,304）或 簇链走完（正常/出错）
                    if (ff_done || (off_cnt >= SUB_FILE_BYTES)) begin
                        if ((ff_err != 4'd0) || !sub_hdr_ok ||
                            (sub_col < SUB_COLS_12) || (off_cnt < SUB_FILE_BYTES)) begin
                            // 不完整：列数照实报，ok=0（RAM 里可能是半条掩膜，
                            // 顶层用 o_sub_ok 当"这半条有效吗"的判据）
                            o_sub_cols <= sub_col;
                            o_sub_ok   <= 1'b0;
                            if (err_code_r == ERR_NONE)
                                err_code_r <= (sub_hdr_ok ? ERR_FAT : ERR_HDR);
                        end else begin
                            o_sub_cols <= SUB_COLS_12;
                            o_sub_ok   <= 1'b1;
                        end
                        st <= L_DONE;
                    end
                end else begin
                if (ff_dv) begin
                    if ((off_cnt >= pixel_offset) && (off_cnt < file_len)) begin
                        // 3 字节拼一个像素；第 3 个字节那一拍才给写使能
                        case (byte_idx)
                        2'd0: begin
                            bmp_data_wr_en <= 1'b0;
                            bmp_data[7:0]  <= ff_dat;
                        end
                        2'd1: begin
                            bmp_data_wr_en <= 1'b0;
                            bmp_data[15:8] <= ff_dat;
                        end
                        2'd2: begin
                            bmp_data_wr_en  <= 1'b1;
                            bmp_data[23:16] <= ff_dat;
                        end
                        default: begin
                            bmp_data_wr_en <= 1'b0;
                        end
                        endcase
                        byte_idx <= (byte_idx == 2'd2) ? 2'd0 : (byte_idx + 2'd1);
                    end else begin
                        // 偏移 < pixel_offset（BMP 头那几十个字节）：只计数不写
                        bmp_data_wr_en <= 1'b0;
                    end

                    off_cnt     <= off_cnt + 32'd1;
                    bytes_out_r <= bytes_out_r + 32'd1;
                end else begin
                    bmp_data_wr_en <= 1'b0;
                end

                // 结束：字节收齐 或 簇链走完（正常/出错）都停在这里
                if (ff_done || (off_cnt >= file_len)) begin
                    if (ff_done && (ff_err != 4'd0))
                        err_code_r <= ERR_FAT;
                    st <= L_DONE;
                end
                end
            end

            //====================================================== 收尾
            L_DONE: begin
                // 把 fat32_file 按回复位（防止它继续向卡发请求）
                ff_kill        <= 1'b1;
                write_req      <= 1'b0;
                bmp_data_wr_en <= 1'b0;
                o_mask_we      <= 1'b0;
                state_code     <= 4'd1;
                st             <= L_IDLE;
            end

            default: st <= L_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 装载进度（c72）：0..255
    //   每写 N 个输出像素，进度 +1。N 按源尺寸取：
    //     640×480  -> 307200/256 = 1200
    //     1280×960 -> 1228800/256 = 4800
    //   ★ 用"每 N 个累加一次"代替除法：307200 恰好是 256 的 1200 倍，
    //     1228800 恰好是 4800 倍，所以进度条能精确走满 255，零除法器。
    //   ★ 计数源用 bmp_data_wr_en（源像素写使能，在缩放器之前），
    //     所以 1280×960 时会数 1,228,800 次、640×480 时数 307,200 次。
    //-------------------------------------------------------------------------
    reg [12:0] prog_acc;
    wire [12:0] prog_step = is_src_2to1 ? 13'd4800 : 13'd1200;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prog_acc <= 13'd0;
            o_prog   <= 8'd0;
        end else if (load_start) begin
            prog_acc <= 13'd0;
            o_prog   <= 8'd0;
        end else if (bmp_data_wr_en) begin
            if (prog_acc + 13'd1 >= prog_step) begin
                prog_acc <= 13'd0;
                if (o_prog != 8'd255)
                    o_prog <= o_prog + 8'd1;
            end else begin
                prog_acc <= prog_acc + 13'd1;
            end
        end
    end

    // 装载中 = 状态机不在 IDLE（覆盖读卡 + 写帧，转场期间保持 1）
    assign o_busy = (st != L_IDLE);

endmodule
