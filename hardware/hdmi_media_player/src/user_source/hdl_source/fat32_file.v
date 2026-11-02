//=============================================================================
// 文件名  : fat32_file.v
// 功能    : FAT32 阶段 2 —— 跟文件的簇链，并按文件大小把数据逐字节流出
//
//   输入：文件首簇号(32bit) + 文件大小(32bit) + 卷几何（FAT1 起始绝对扇区 /
//         数据区起始绝对扇区 / 每簇扇区数），这些都是 fat32_dir.v（阶段 1）
//         解析 MBR/BPB 时已经算出来的东西。
//   输出：start 之后逐拍吐出该文件的字节（o_data + o_data_valid），
//         读完（或出错）后 done 拉高，err_code 给出结论。
//
//   这样上层（BMP 解码 / WAV 播放）只要认 o_data/o_data_valid 这一条流，
//   不必关心 FAT、簇、扇区。
//
// 对外只有一层握手（与阶段 1 完全一致，便于共用同一条"取扇区"通路）
// ---------------------------------------------------------------------------
//   o_rd_req 拉高且 o_rd_lba 给出绝对扇区号 -> 外部按序还我 512 个字节
//   （i_data / i_data_valid，一个字节一拍也可以，中间断流也可以）。
//   o_rd_lba 是与 o_rd_req 同拍的纯组合输出：外部就在"请求有效的那一拍"
//   采扇区号，理由与踩过的坑见 fat32_dir.v 文件头要点 (A)。
//
// 簇链怎么跟（规范 + 本模块的取舍）
// ---------------------------------------------------------------------------
//   * FAT[N] 位于 FAT1 起始 + N*4 字节处，32 位 little-endian，只取低 28 位
//       0x0FFFFFF8 ~ 0x0FFFFFFF -> EOC（链正常结束）
//       0x00000000             -> 空闲簇：当坏链报错停下
//       0x00000001             -> 保留（不是合法簇号）-> 报错
//       0x0FFFFFF7             -> 坏簇标记 -> 报错
//       0x0FFFFFF0~0x0FFFFFF6  -> 保留值   -> 报错
//       其余 2 ~ 0x0FFFFFEF    -> 下一个簇
//   * 自指：FAT[N] == N 直接报 ERR_SELF_LOOP（一眼可判的坏链，不必等上限）
//   * 上限：chain_lim 绝对硬顶（见下）
//
// 为什么上限必须是与文件大小无关的硬顶（这段分析很关键，别改回去）
// ---------------------------------------------------------------------------
//   一开始想按"需要的簇数 = ceil(大小 / 每簇字节数)，上限 = 需要 + 2"来兜底。
//   但这个上限在结构上永远不可能触发：本模块一吐满 size 个字节就立刻结束，
//   而每个簇最多贡献"每簇字节数"个字节，所以走满 ceil(size/簇字节) 个簇时
//   字节一定已经吐完了。换句话说，"循环链"在没有硬顶的时候不会挂死，但也不会
//   报错 —— 它会像"重复读同一个簇"一样把 size 个字节吐完，然后"成功"结束，
//   这是假阳性（坏卡被判成好卡），比报错更危险。
//   真正致命的场景是目录项里的 size 被写坏成 0xFFFFFFFF 这种巨大值：此时
//   "按大小推导的上限"本身就是 50 万簇，等于没有上限。
//   所以这里用一个与大小无关的绝对硬顶：
//       chain_lim = (i_max_clusters != 0 且比硬顶更小) ? i_max_clusters : CHAIN_HARD_MAX
//   上层可以用 i_max_clusters 把上限再收紧（例如按自己的"文件预算"限流）；
//   传 0 就只用内部硬顶。超过上限报 ERR_CHAIN_LEN 并安全停下。
//
// 三个"必须这样写"的要点（都是本项目踩过坑的地方）
// ---------------------------------------------------------------------------
// (A) o_data 与 o_data_valid 必须同一拍、同一个分支一起赋值
//     分开两拍写会让消费者整体错位一个字节：921654 字节的 BMP 会整体移位，
//     画面直接错行，而且 sha256 只能告诉你"不一致"，很难看出是错位。
//
// (B) "本拍到底吐不吐字节"必须用赋值前的 bytes_left 判断（组合 will_emit）
//     若在同一拍先 bytes_left <= bytes_left - 1，再用新值判断，非阻塞赋值下
//     判的是旧值，会少吐 1 个字节（或把最后 1 个字节判成"已经读完"）。
//     正确写法：组合算出 will_emit，时序块里只做"减法"和"输出"。
//
// (C) 长度上限的判定放在"接受下一个簇之前"，不是"访问之后"
//     这样超限时停在簇边界上，已吐出的字节数一定是"整簇"的整数倍，
//     外部能算出确定的前缀长度（本项目的靶子镜像就按这个规则做逐字节比对）。
//
// (D) 2026-10 新增：等扇区超时必须区分"排队"与"传输"
//     本模块在 sd_card_bmp 里是挂在bmp 最高优先级那一路上的；即便如此，
//     当 wav/别的通路正在占卡时，它的请求也要排队。旧代码从发出请求就开始
//     计时，排队时间被算成传输超时 → 误报 ERR_SECTOR_TO(9)。
//     现在：i_granted=1（仲裁器把卡交给我）才开始算传输超时 tmo_cnt；
//           另有一个不看授权的总等待兜底 tmo_all_cnt（TO_LIMIT_ALL=2 秒）。
//     两层的完整理由见 fat32_dir.v 文件头要点 (F) 与 TO_LIMIT 参数注释。
//
// 位宽（本项目踩过多次的坑，全部显式写死）
// ---------------------------------------------------------------------------
//   簇偏移 = (簇号 - 2) << log2(每簇扇区数)：中间结果一律用 40 位接住，
//     确认高位没超才截到 32 位（绝不静默截断成"另一个扇区号"）。
//   FAT 表项偏移 = 簇号 * 4：38 位装得下（簇号最大 0x0FFFFFFE）。
//   log2(每簇扇区数) 只能用 case 从 SecPerClus 解出来，而且是它的唯一驱动
//     （同 fat32_dir.v 要点 E：本项目的 spc_lg2 曾经只声明不复位、恒为 0）。
//
// 关于"FAT 表项可能跨扇区边界"
// ---------------------------------------------------------------------------
//   规范上 FAT 表项的 4 个字节确实可能被扇区边界切成两半，所以这里的捕获逻辑
//   是通用的：按"表项在 FAT 内的绝对字节偏移"捕获 4 个字节，一个扇区没
//   捕满就继续读下一个 FAT 扇区，直到 4 个字节都到手（见 fat_sec_ofs）。
// 但必须诚实说明：BytsPerSec 恒为 512（fat32_dir 会把 !=512 的卡直接判
//   ERR_BPS），而 512 / 4 = 128 整除 —— 所以 4 字节表项永远整条落在一个
//   扇区内】，跨扇区分支在真实几何下数学上不可达。它作为防御性代码保留；
//   真正能被上面这套镜像验到的是它的边界：表项偏移 508（簇号 ≡ 127 mod 128），
//   第 4 个字节正好是本扇区的最后一个字节 —— 窗口末端少算一个字节的实现
//   （例如把结束条件写成 fat_in+3）会立刻在 FRAG.BIN 上暴露（见 README 负对照）。
//=============================================================================

module fat32_file #(
    //-------------------------------------------------------------------------
    // 等扇区数据的超时（拍）—— 2026-10 修复：语义与取值都改了
    //   语义：只在已被仲裁器授权用卡(i_granted=1) 期间，
    //         "连续 TO_LIMIT 拍没收到数据字节"才算超时；计数器每收到一个
    //         字节就清零，所以它约束的是字节间隔 / 授权到首字节的等待，
    //         不包含排队等卡的时间。
    //   取值：16384 拍 @100MHz = 163.84µs = 恰好是 25MHz SPI 传 512 字节
    //         纯数据的时间，连命令+响应+数据令牌的余量都没有；
    //         现在改成 2_000_000 拍 = 20ms（≈40 个扇区，见 fat32_dir.v
    //         里同一段注释的完整理由）。本模块和 fat32_dir 是同一个 bug，
    //         一起改，保持一致。
    parameter [31:0] TO_LIMIT       = 32'd2000000,
    //-------------------------------------------------------------------------
    // TO_LIMIT_ALL：从"发出请求"起算的总等待上限（含排队），防饥饿兜底
    //   没有它的话，把排队时间排除出超时之后，"永远排不上队"就永远不报错 ——
    //   新的挂死风险（读卡任务永远不 done）。2 秒 = 2×10^8 拍 @100MHz，
    //   明显大于一次 bmp 整图装载（≈0.85 秒），与 fat32_dir 取同一个值。
    parameter [31:0] TO_LIMIT_ALL   = 32'd200000000,
    // 簇链绝对硬顶（与文件大小无关的兜底），超过就报 ERR_CHAIN_LEN 停下
    parameter [31:0] CHAIN_HARD_MAX = 32'd1048576
)(
    input               clk,            // 系统时钟，25MHz，与读卡通路同一时钟域
    input               rst_n,          // 异步复位，低有效

    // ---- 启动与目标文件 ----
    input               start,          // 单拍脉冲启动一次；完成后要复位才能再启动
    input      [31:0]   i_first_clu,    // 文件首簇号（来自 fat32_dir 文件表）
    input      [31:0]   i_size,         // 文件大小（字节，来自 fat32_dir 文件表）

    // ---- 卷几何（来自 fat32_dir）----
    input      [31:0]   i_fat_lba,      // FAT1 起始绝对扇区
    input      [31:0]   i_data_lba,     // 数据区起始绝对扇区
    input      [7:0]    i_spc,          // 每簇扇区数（2 的幂，1~64）

    // ---- 仍然是"给我扇区 N -> 还我 512 字节"那一层握手 ----
    input      [31:0]   i_max_clusters, // 上层可选收紧的簇数上限；0 = 只用内部硬顶

    output reg [31:0]   o_rd_lba,       // 请求读哪个绝对扇区
    output reg          o_rd_req,       // 1 = 正在请求（保持到第一个字节回来）
    input      [7:0]    i_data,         // 扇区数据字节流
    input               i_data_valid,   // 1 = 本拍 i_data 有效
    input               i_data_last,    // 1 = 本拍是本扇区最后一个字节（本模块不依赖）

    // ---- 文件数据字节流（给 BMP 解码 / WAV 播放用）----
    output reg [7:0]    o_data,         // 数据字节，与 o_data_valid 同拍
    output reg          o_data_valid,   // 1 = 本拍 o_data 有效

    // ---- 状态 ----
    output              busy,           // 1 = 读文件中
    output              done,           // 1 = 结束（正常或出错，电平保持）
    output reg [3:0]    err_code,       // 0 = 无错；含义见 localparam

    // ---- 统计 / 调试 ----
    output reg [31:0]   o_bytes_out,    // 一共吐出了多少字节（正常时应等于 i_size）
    output reg [31:0]   o_clusters,     // 一共访问了多少个簇
    output reg [31:0]   o_lba_hash,     // 访问过的"簇起始绝对扇区"序列指纹：
                                        //   h = rotl32(h,1) ^ 簇起始扇区（见 README）
    output reg [4:0]    dbg_spc_lg2,    // 由 i_spc 解出来的 log2（调试/自检用）
    output     [7:0]    dbg_dsec,       // 当前在簇内读第几个扇区
    output              dbg_is_fat,     // 1 = 当前这次扇区读取是读 FAT 表
    output     [31:0]   dbg_rd_secs,    // 一共读了多少个扇区（含 FAT 扇区）

    //-------------------------------------------------------------------------
    // 2026-10 超时修复新增：本路"已被仲裁器授权用卡"
    //   1 = 卡现在确实归我（sd_arbiter3 已批准本次请求），0 = 还在排队。
    // 只参与超时计时，不参与任何数据/状态判定
    // 没有仲裁器的场合（单元仿真 tb）接 1'b1：退化成修复前行为
    //   本模块在 sd_card_bmp 里是挂在bmp那一路上的（bmp_load_fat 的
    //   L_HDR 相与数据相都走 bmp_sec_read），所以顶层给它接的是
    //   sd_arbiter3 的 o_grant_bmp，不是 o_grant_fat。
    input               i_granted
);

    // 下面三个调试量直接引内部寄存器（不再单独造一批寄存器）：
    //   * 用 assign 引出去，外部看到的一定就是内部真实状态，不可能"忘了赋值"
    //（本项目在这里真踩过：三个 dbg_* 声明成 output reg 却从头到尾没赋值，
    // 仿真里读到 x，连"簇起始扇区日志"的判据都因此恒为假、一条都没记下来）
    //   * 它们只是观察窗口，不参与任何逻辑
    // 2026-10（阶段 3b）：这三句 assign 原来写在被引用的寄存器声明之前，
    //     iverilog 不吭声，TD 却在第一次真正综合这个文件时报
    //       HDL-5373 WARNING: identifier 'dsec' is used before its declaration
    //     （3a 里本文件被 AutoExcluded、没进综合，所以一直没暴露）。
    //     把三句挪到 dsec/ph_fat/rd_secs 声明之后（见"流程寄存器"那一段），
    //     行为完全不变（同样的 assign、同样的位宽），只是满足"先声明后使用"。

    //-------------------------------------------------------------------------
    // 错误码（0 = 成功）。每个码对应一个明确的失败点，便于数码管/LED 显示。
    //-------------------------------------------------------------------------
    localparam [3:0] ERR_NONE       = 4'd0,   // 无错
                     ERR_FIRST_CLU  = 4'd1,   // 首簇号非法（<2 或 >= EOC 区）
                     ERR_SPC        = 4'd2,   // SecPerClus 不是 2 的幂 / 越界
                     ERR_CLU_ZERO   = 4'd3,   // FAT[N] == 0（空闲簇）：坏链
                     ERR_CLU_LOW    = 4'd4,   // FAT[N] < 2：非法簇号
                     ERR_CLU_BAD    = 4'd5,   // FAT[N] == 0x0FFFFFF7：坏簇标记
                     ERR_CLU_RESV   = 4'd6,   // FAT[N] 落在保留值 0x0FFFFFF0~6
                     ERR_CHAIN_LEN  = 4'd7,   // 超过簇数上限（坏链/巨大小防死循环）
                     ERR_SIZE_SHORT = 4'd8,   // 链提前 EOC，但文件大小还没读完
                     // 等扇区数据超时。2026-10 修复后语义分两种，都报这个码：
                     //   ① 已被授权后连续 TO_LIMIT 拍（20ms）没有数据字节
                     //   ② 从发出请求算起总等待超过 TO_LIMIT_ALL（2 秒，防饥饿）
                     ERR_SECTOR_TO  = 4'd9,
                     ERR_LBA_RANGE  = 4'd10,  // 算出来的扇区号超过 32 位
                     ERR_FAT_WIN    = 4'd11,  // 两个 FAT 扇区都没捕满 4 字节（防御）
                     ERR_SELF_LOOP  = 4'd12;  // FAT[N] == N：簇链自指

    //-------------------------------------------------------------------------
    // 主状态机
    //   S_IDLE : 等 start
    //   S_INIT : 参数落定后的第一拍：启动校验 + 长度上限落地
    //   S_SET  : 让刚写进去的簇号/扇区序号落定一拍，再发请求
    //（见 fat32_dir.v 要点 B：o_rd_lba 是组合量，参数换档后必须
    //              隔一拍，否则第一拍的 o_rd_lba 还是旧值）
    //   S_REQ  : o_rd_req 拉高，等第一个数据字节
    //   S_RD   : 收剩下的 511 个字节（数据阶段边收边吐）
    //   S_EVAL : 本扇区收完 -> 决定继续本簇 / 去读 FAT / 结束
    //   S_DONE : 收尾保持
    //-------------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0,
                     S_INIT = 3'd1,
                     S_SET  = 3'd2,
                     S_REQ  = 3'd3,
                     S_RD   = 3'd4,
                     S_EVAL = 3'd5,
                     S_DONE = 3'd6,
                     S_PRE  = 3'd7;   // c68：首簇链路第一级

    reg [2:0] st;

    assign busy = (st != S_IDLE) && (st != S_DONE);
    assign done = (st == S_DONE);

    //-------------------------------------------------------------------------
    // 参数锁存（start 那一拍从输入端口抓进来，之后整趟都只用锁存值）
    //-------------------------------------------------------------------------
    reg [31:0] size_r;          // 文件大小
    reg [31:0] fat_lba_r;       // FAT1 起始绝对扇区
    reg [31:0] data_lba_r;      // 数据区起始绝对扇区
    reg [7:0]  spc_r;           // 每簇扇区数
    reg [31:0] max_clu_r;       // 上层给的簇数上限（0 = 只用硬顶）

    //-------------------------------------------------------------------------
    // 流程寄存器
    //-------------------------------------------------------------------------
    reg [31:0] cur_clu;         // 当前簇号
    reg [31:0] bytes_left;      // 还没吐出的字节数
    reg [31:0] chain_cnt;       // 已访问的簇数
    reg [31:0] chain_lim;       // 本趟的簇数上限
    reg [7:0]  dsec;            // 簇内扇区序号 0 ~ spc-1
    reg        ph_fat;          // 1 = 当前这次扇区读取是读 FAT 表；0 = 读文件数据
    reg [9:0]  bcnt;            // 当前字节在扇区内的偏移 0~511
    // 两个超时计数器（2026-10 修复）
    //   tmo_cnt     : 只统计已授权之后的断流拍数 —— 真正的传输超时
    //   tmo_all_cnt : 不看授权、只要在等数据就累加 —— 防饥饿的总等待兜底
    //   两者都只在本模块唯一的那个主状态机 always 块里驱动（不许多驱动）。
    reg [31:0] tmo_cnt;
    reg [31:0] tmo_all_cnt;
    reg [31:0] rd_secs;         // 读过的扇区数

    // 三个调试量的引出（必须放在 dsec/ph_fat/rd_secs 声明之后
    //   原来写在这三行之外的文件上方，TD 报 HDL-5373"使用先于声明"）
    assign dbg_dsec    = dsec;
    assign dbg_is_fat  = ph_fat;
    assign dbg_rd_secs = rd_secs;

    // FAT 表项捕获（按"表项在 FAT 内的绝对字节偏移"抓 4 个字节）
    reg [1:0]  fat_sec_ofs;     // 表项跨扇区时要多读的扇区序号（0 或 1）
    reg [3:0]  fat_cap_cnt;     // 已经捕获到几个字节
    reg [31:0] fat_buf;         // 捕获到的 4 个字节（little-endian）

    //-------------------------------------------------------------------------
    // 组合：SecPerClus 合法性 + log2(SecPerClus)
    // 这一段是 dbg_spc_lg2 的唯一驱动（复位分支里没有它），
    //     否则同一个 reg 会被两个 always 块赋值（verilog_lint.py 会报，
    // 综合器也不认）。
    //   128 被 spc_r[7]==0 这一条排除（512 字节扇区下簇最大 64 个扇区），
    //   所以 case 里不列 128。
    //-------------------------------------------------------------------------
    reg spc_ok;

    always @(*) begin
        // 2 的幂且 <= 64：spc[7]==0 排除 128 及以上的越界值
        spc_ok = (spc_r != 8'd0) && ((spc_r & (spc_r - 8'd1)) == 8'd0) &&
                 (spc_r[7] == 1'b0);

        case (spc_r)
        8'd1:    dbg_spc_lg2 = 5'd0;
        8'd2:    dbg_spc_lg2 = 5'd1;
        8'd4:    dbg_spc_lg2 = 5'd2;
        8'd8:    dbg_spc_lg2 = 5'd3;
        8'd16:   dbg_spc_lg2 = 5'd4;
        8'd32:   dbg_spc_lg2 = 5'd5;
        8'd64:   dbg_spc_lg2 = 5'd6;
        default: dbg_spc_lg2 = 5'd0;
        endcase
    end

    //-------------------------------------------------------------------------
    // 组合：本趟的簇数上限
    //   与文件大小无关的绝对硬顶；上层给的 i_max_clusters 只能收紧不能放宽。
    //   理由见文件头"为什么上限必须是与文件大小无关的硬顶"。
    //-------------------------------------------------------------------------
    reg [31:0] lim_c;

    always @(*) begin
        if ((max_clu_r != 32'd0) && (max_clu_r < CHAIN_HARD_MAX))
            lim_c = max_clu_r;
        else
            lim_c = CHAIN_HARD_MAX;
    end

    //-------------------------------------------------------------------------
    // 组合：簇号 -> 扇区号（三个中间量全部 40 位，确认高位没超才截 32 位）
    //   簇 N 起始绝对扇区 = 数据区起始 + (N - 2) * SecPerClus
    //   FAT 表项所在扇区 = FAT1 起始 + (N*4) / 512  （+ 跨扇区时的第二个扇区）
    //   FAT 表项在扇区内的字节偏移 = (N*4) % 512 —— 恒为 4 的倍数，最大 508
    //-------------------------------------------------------------------------
    reg [39:0] clu_off_x;       // (cur_clu - 2) << spc_lg2
    reg [39:0] data_sec_x;      // 当前数据扇区绝对号
    reg [39:0] fat_off_x;       // cur_clu * 4
    reg [39:0] fat_sec_x;       // 当前 FAT 扇区绝对号
    reg [8:0]  fat_in;          // 表项首字节在本 FAT 扇区内的偏移
    reg        lba_bad_x;       // 扇区号超过 32 位
    reg        tmo_hit;         // 等数据超时

    always @(*) begin
        clu_off_x  = ({8'd0, cur_clu} - 40'd2) << dbg_spc_lg2;
        data_sec_x = {8'd0, data_lba_r} + clu_off_x + {32'd0, dsec};
        fat_off_x  = {6'd0, cur_clu} << 2;                      // 38 位装得下
        fat_sec_x  = {8'd0, fat_lba_r} + (fat_off_x >> 9) + {38'd0, fat_sec_ofs};
        fat_in     = fat_off_x[8:0];

        lba_bad_x = (|data_sec_x[39:32]) || (|fat_sec_x[39:32]);

        // o_rd_lba 必须是纯组合、与 o_rd_req 同拍（fat32_dir.v 要点 A）
        if (ph_fat)
            o_rd_lba = fat_sec_x[31:0];
        else
            o_rd_lba = data_sec_x[31:0];

        o_rd_req = (st == S_REQ);
        // 两种超时任一命中就报错（语义见 TO_LIMIT / TO_LIMIT_ALL 注释）
        tmo_hit  = (tmo_cnt >= TO_LIMIT) || (tmo_all_cnt >= TO_LIMIT_ALL);
    end

    //-------------------------------------------------------------------------
    // 组合：FAT 表项的字节捕获窗口
    //   win_off = 当前这个字节在"表项起始扇区起算"的绝对偏移（11 位够：512+512）
    //   捕获条件：win_off 落在 [fat_in, fat_in+4) 内
    //   fat_k_c = win_off - fat_in，就是"这是表项的第几个字节"（0~3）
    //   fat_in 恒为 4 的倍数，所以 fat_k_c[1:0] 直接可以当字节序号用。
    //-------------------------------------------------------------------------
    reg [10:0] win_off_c;
    reg [10:0] win_beg_c;
    reg [10:0] win_end_c;
    reg [10:0] fat_k_c;
    reg        fat_cap_hit;
    reg [9:0]  off_cur;         // 本拍正在处理第几个扇区内偏移

    always @(*) begin
        off_cur     = (st == S_REQ) ? 10'd0 : bcnt;
        win_off_c   = ({9'd0, fat_sec_ofs} << 9) + {1'b0, off_cur};
        win_beg_c   = {2'd0, fat_in};
        win_end_c   = {2'd0, fat_in} + 11'd4;
        fat_k_c     = win_off_c - win_beg_c;
        fat_cap_hit = (win_off_c >= win_beg_c) && (win_off_c < win_end_c);
    end

    //-------------------------------------------------------------------------
    // 组合：本拍要不要真的吐一个数据字节
    // 用赋值前的 bytes_left 判断（要点 B），并且只在数据阶段成立
    //   ph_fat==1 时即使 bytes_left 还有也不吐（那是在读 FAT 表）。
    //-------------------------------------------------------------------------
    reg rx_now;                 // 本拍确实在收一个有效字节
    reg will_emit;              // 本拍要吐一个数据字节

    always @(*) begin
        rx_now    = i_data_valid && ((st == S_REQ) ||
                                     ((st == S_RD) && (bcnt <= 10'd511)));
        will_emit = rx_now && (ph_fat == 1'b0) && (bytes_left != 32'd0);
    end

    //-------------------------------------------------------------------------
    // 组合：首簇号合法性 + 首簇起始扇区
    //   用输入端口 i_first_clu 判（S_INIT 那一拍 cur_clu 还没写进去）
    //-------------------------------------------------------------------------
    // ★ c68：首簇链路拆成三级。
    //   改前最差路径（c67 final_timing.rpt）：
    //     Begin: sd_card_bmp_m0/u_fat32_boot/wav_clu_3_reg[12]
    //     End  : sd_card_bmp_m0/u_wav_src_fat/u_fat32_file/cur_clu_reg[16].ce
    //     Slack -4.945ns, Delay 14.665ns (cell 39% / net 61%), Logic Level 11
    //   根因：i_first_clu 来自另一个模块（fat32_boot），进来就要连做
    //         40 位减法 -> 变量左移（桶形移位器 6 级）-> 40 位加法 -> 越界比较,
    //         最后还要穿过状态转移落到 cur_clu 的使能上。
    //   现在：i_first_clu 只到 fclu_r（1 级 FF，布线短）；
    //         fclu_r -> first_lba_x -> lba_r（全在模块内，布线短）；
    //         lba_r  -> 越界判断 -> 状态（1~2 级）。
    reg [31:0] fclu_r;          // 首簇号锁存（S_IDLE 抓）
    reg [39:0] lba_r;           // 首簇起始扇区（S_PRE 落）
    reg        ok_r;            // 首簇号合法性（不含 LBA 越界）
    reg [3:0]  err_r;

    // 组合：首簇起始扇区（输入是锁存后的 fclu_r，不再直接吃端口）
    wire [39:0] first_lba_x = {8'd0, data_lba_r} +
                              (({8'd0, fclu_r} - 40'd2) << dbg_spc_lg2);

    // 组合：首簇号合法性
    //   LBA 越界那一项不放在这里 —— 它依赖 first_lba_x（40 位移位结果），
    //   放进来的话这一级的终点又会拖长。改成在 S_INIT 用寄存后的 lba_r 判。
    reg        ok_comb;
    reg [3:0]  err_comb;
    always @(*) begin
        ok_comb  = 1'b1;
        err_comb = ERR_NONE;

        if (fclu_r < 32'd2) begin
            ok_comb  = 1'b0;
            err_comb = ERR_FIRST_CLU;
        end else if (fclu_r == 32'h0FFFFFF7) begin
            ok_comb  = 1'b0;
            err_comb = ERR_CLU_BAD;
        end else if (fclu_r >= 32'h0FFFFFF8) begin
            ok_comb  = 1'b0;
            err_comb = ERR_FIRST_CLU;
        end
    end

    //-------------------------------------------------------------------------
    // 组合：FAT 表项读回来之后的判定
    //   fat_next_c : 低 28 位就是下一个簇号
    //   nxt_ok/nxt_err/nxt_eoc : 合法性 + 是不是 EOC
    //   nxt_lba_x  : 下一个簇的起始绝对扇区（自检 32 位范围 + 参与序列指纹）
    //-------------------------------------------------------------------------
    reg [31:0] fat_next_c;
    reg        nxt_ok;
    reg [3:0]  nxt_err;
    reg        nxt_eoc;
    reg [39:0] nxt_lba_x;

    always @(*) begin
        fat_next_c = fat_buf & 32'h0FFFFFFF;
        nxt_ok     = 1'b1;
        nxt_err    = ERR_NONE;
        nxt_eoc    = 1'b0;

        if (fat_next_c == 32'd0) begin
            // 空闲簇：说明这条链是坏的。任务要求明确当成"坏链终止并报错"。
            nxt_ok  = 1'b0; nxt_err = ERR_CLU_ZERO;
        end else if (fat_next_c < 32'd2) begin
            nxt_ok  = 1'b0; nxt_err = ERR_CLU_LOW;
        end else if (fat_next_c == 32'h0FFFFFF7) begin
            nxt_ok  = 1'b0; nxt_err = ERR_CLU_BAD;
        end else if (fat_next_c >= 32'h0FFFFFF0) begin
            if (fat_next_c >= 32'h0FFFFFF8)
                nxt_eoc = 1'b1;                       // EOC：链正常结束
            else begin
                nxt_ok  = 1'b0; nxt_err = ERR_CLU_RESV;
            end
        end

        nxt_lba_x = {8'd0, data_lba_r} +
                    (({8'd0, fat_next_c} - 40'd2) << dbg_spc_lg2);
    end

    //-------------------------------------------------------------------------
    // 时序：收字节时"吐数据 / 捕获 FAT 表项"
    //   写成 task 是为了让 S_REQ（偏移 0）与 S_RD（偏移 1~511）共用同一段逻辑
    //   —— 早期两处分头写，偏移 0 被处理两次或漏一次（fat32_dir.v 的 task
    //   注释里记着同类事故）。task 里只有寄存器赋值，没有延时/事件控制，
    //   TD 与 iverilog 都能正常内联，是可综合的写法。
    //-------------------------------------------------------------------------
    task rx_byte;
        input [9:0] off;                 // 本字节在扇区内的偏移（本模块只用到它算窗口）
        begin
            if (ph_fat == 1'b1) begin
                // ---- 读 FAT 表：按窗口捕获 4 个字节（可能跨扇区，见文件头）----
                if (fat_cap_hit) begin
                    // 用 case 而不是位选下标算术：位选下标带乘/移位正是
                    // "上下文宽度陷阱"的温床，本项目在这上面踩过三次。
                    case (fat_k_c[1:0])
                    2'd0: fat_buf[7:0]   <= i_data;
                    2'd1: fat_buf[15:8]  <= i_data;
                    2'd2: fat_buf[23:16] <= i_data;
                    2'd3: fat_buf[31:24] <= i_data;
                    default: ;
                    endcase
                    fat_cap_cnt <= fat_cap_cnt + 4'd1;
                end
            end else if (will_emit) begin
                // ---- 读文件数据：吐字节（o_data 与 o_data_valid 同一拍，要点 A）----
                o_data       <= i_data;
                o_data_valid <= 1'b1;
                bytes_left   <= bytes_left - 32'd1;
                o_bytes_out  <= o_bytes_out + 32'd1;
            end
            // off 在本模块里不参与寻址（窗口已经由 win_off_c 算好），
            // 保留形参是为了与 S_REQ/S_RD 两种调用共用同一段代码。
        end
    endtask

    //-------------------------------------------------------------------------
    // 主状态机
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st           <= S_IDLE;
            size_r       <= 32'd0;
            fat_lba_r    <= 32'd0;
            data_lba_r   <= 32'd0;
            spc_r        <= 8'd0;
            max_clu_r    <= 32'd0;
            fclu_r       <= 32'd0;
            lba_r        <= 40'd0;
            ok_r         <= 1'b1;
            err_r        <= ERR_NONE;
            cur_clu      <= 32'd0;
            bytes_left   <= 32'd0;
            chain_cnt    <= 32'd0;
            chain_lim    <= 32'd0;
            dsec         <= 8'd0;
            ph_fat       <= 1'b0;
            bcnt         <= 10'd0;
            tmo_cnt      <= 32'd0;
            tmo_all_cnt  <= 32'd0;
            rd_secs      <= 32'd0;
            fat_sec_ofs  <= 2'd0;
            fat_cap_cnt  <= 4'd0;
            fat_buf      <= 32'd0;
            err_code     <= ERR_NONE;
            o_bytes_out  <= 32'd0;
            o_clusters   <= 32'd0;
            o_lba_hash   <= 32'd0;
            o_data       <= 8'd0;
            o_data_valid <= 1'b0;
            // 注意：dbg_spc_lg2 不在这里复位 —— 它由上面那个组合 case 块独占驱动
            //   （同一个 reg 出现在两个 always 块里会被 verilog_lint.py 判为多驱动，
            //    综合器也不接受；spc_r 复位成 0 时 case 走 default = 0，效果一样）。
        end else begin
            // ---- 超时计数（2026-10 修复：排队时间不计入传输超时） ----
            //   与 fat32_dir.v 里同一段逻辑、同一个理由（那边有完整说明）：
            //     收到数据字节 -> 两个计数器都清零；
            //     没收到数据   -> tmo_cnt 只在已被授权(i_granted=1) 时加，
            //                     tmo_all_cnt 不看授权一直加（防饥饿兜底）。
            //   本块是这两个 reg 的唯一驱动源（含复位）。
            if ((st == S_REQ) || (st == S_RD)) begin
                if (i_data_valid) begin
                    tmo_cnt     <= 32'd0;
                    tmo_all_cnt <= 32'd0;
                end else begin
                    if (i_granted && (tmo_cnt < TO_LIMIT))
                        tmo_cnt <= tmo_cnt + 32'd1;
                    if (tmo_all_cnt < TO_LIMIT_ALL)
                        tmo_all_cnt <= tmo_all_cnt + 32'd1;
                end
            end else begin
                tmo_cnt     <= 32'd0;
                tmo_all_cnt <= 32'd0;
            end

            // ---- 默认不吐数据；S_REQ/S_RD 里真吐的时候由 rx_byte 再置起 ----
            //   同一个 always 块里对同一寄存器多次赋非阻塞值，最后一次生效，
            //   所以"先清后置"的顺序不能颠倒。
            o_data_valid <= 1'b0;

            case (st)
            //============================================================ 空闲
            S_IDLE: begin
                dsec        <= 8'd0;
                ph_fat      <= 1'b0;
                bcnt        <= 10'd0;
                fat_sec_ofs <= 2'd0;
                fat_cap_cnt <= 4'd0;
                err_code    <= ERR_NONE;
                rd_secs     <= 32'd0;
                o_bytes_out <= 32'd0;
                o_clusters  <= 32'd0;
                o_lba_hash  <= 32'd0;
                if (start) begin
                    size_r     <= i_size;
                    fat_lba_r  <= i_fat_lba;
                    data_lba_r <= i_data_lba;
                    spc_r      <= i_spc;
                    max_clu_r  <= i_max_clusters;
                    fclu_r     <= i_first_clu;   // c68：首簇号在此锁存
                    st         <= S_PRE;
                end
            end

            //============================================================ 首簇链路第一级（c68）
            //   只做一件事：把"锁存好的首簇号"算成起始扇区并寄存。
            //   40 位减法 + 桶形移位 + 40 位加法都在这一拍里，但它们的前后都是
            //   本模块寄存器，布线短；改前这段逻辑的 61% 延迟来自跨模块绕线。
            S_PRE: begin
                lba_r <= first_lba_x;
                ok_r  <= ok_comb;
                err_r <= err_comb;
                st    <= S_INIT;
            end

            //============================================================ 启动校验
            //   参数（spc_r/size_r/...）是上一拍锁存的，这一拍全部可见。
            //   注意这里不能用 cur_clu 判首簇：cur_clu 是本拍才写进去的，
            //   组合量读到的还是旧值（fat32_dir.v 要点 B 记的就是这类错位）。
            S_INIT: begin
                if (!spc_ok) begin
                    err_code <= ERR_SPC;
                    st       <= S_DONE;
                end else if (size_r == 32'd0) begin
                    // 0 字节文件：一个扇区都不读，直接成功结束（合法情况）
                    st <= S_DONE;
                end else if (!ok_r) begin
                    err_code <= err_r;
                    st       <= S_DONE;
                end else if (|lba_r[39:32]) begin
                    // LBA 越过 32 位：lba_r 已经是寄存器输出，这一级路径很短
                    err_code <= ERR_LBA_RANGE;
                    st       <= S_DONE;
                end else begin
                    cur_clu    <= fclu_r;
                    bytes_left <= size_r;
                    chain_lim  <= lim_c;
                    chain_cnt  <= 32'd1;          // 首簇算作已访问的第 1 个簇
                    dsec       <= 8'd0;
                    ph_fat     <= 1'b0;
                    fat_sec_ofs<= 2'd0;
                    fat_cap_cnt<= 4'd0;
                    o_bytes_out<= 32'd0;
                    o_clusters <= 32'd1;
                    // 序列指纹的初值：rotl(0,1) ^ 首簇起始扇区 = 首簇起始扇区
                    o_lba_hash <= lba_r[31:0];
                    st         <= S_SET;
                end
            end

            //============================================================ 等参数落定一拍
            // 这一拍是必需的（fat32_dir.v 要点 B）
            //   o_rd_lba 是纯组合：它读 cur_clu/dsec/ph_fat，而这三个寄存器可能是
            //   上一拍（S_INIT 或 S_EVAL）才写进去的。隔一拍再发请求，保证
            //   "请求有效那一拍的扇区号"一定是新值。
            S_SET: begin
                if (lba_bad_x) begin
                    err_code <= ERR_LBA_RANGE;
                    st       <= S_DONE;
                end else begin
                    st <= S_REQ;
                end
            end

            //============================================================ 请求 / 收第一个字节
            S_REQ: begin
                if (i_data_valid) begin
                    rx_byte(10'd0);
                    rd_secs <= rd_secs + 32'd1;
                    bcnt    <= 10'd1;
                    st      <= S_RD;
                end else if (tmo_hit) begin
                    err_code <= ERR_SECTOR_TO;
                    st       <= S_DONE;
                end
            end

            //============================================================ 收剩下的字节
            S_RD: begin
                if (i_data_valid) begin
                    rx_byte(bcnt);
                    if (bcnt == 10'd511) begin
                        bcnt <= 10'd0;
                        st   <= S_EVAL;
                    end else begin
                        bcnt <= bcnt + 10'd1;
                    end
                end else if (tmo_hit) begin
                    // 收到一半断了：同样报超时，不要静默等下去
                    err_code <= ERR_SECTOR_TO;
                    st       <= S_DONE;
                end
            end

            //============================================================ 一个扇区收完
            S_EVAL: begin
                if (ph_fat == 1'b1) begin
                    // ---- 刚读完一个 FAT 扇区 ----
                    if (fat_cap_cnt >= 4'd4) begin
                        // 表项 4 个字节到手：判下一个簇
                        if (!nxt_ok) begin
                            err_code <= nxt_err;
                            st       <= S_DONE;
                        end else if (nxt_eoc) begin
                            // 链到头了，但字节还没吐完 -> 目录项里的 size 不可信
                            err_code <= ERR_SIZE_SHORT;
                            st       <= S_DONE;
                        end else if (fat_next_c == cur_clu) begin
                            // 自指 A->A：一眼可判的坏链，不用等上限
                            err_code <= ERR_SELF_LOOP;
                            st       <= S_DONE;
                        end else if (chain_cnt >= chain_lim) begin
                            // 长度上限：在接受下一个簇之前判，这样停下时
                            // 吐出的字节一定是整簇的整数倍（见文件头要点 C）
                            err_code <= ERR_CHAIN_LEN;
                            st       <= S_DONE;
                        end else if (|nxt_lba_x[39:32]) begin
                            err_code <= ERR_LBA_RANGE;
                            st       <= S_DONE;
                        end else begin
                            cur_clu     <= fat_next_c;
                            chain_cnt   <= chain_cnt + 32'd1;
                            o_clusters  <= o_clusters + 32'd1;
                            // 序列指纹：h = rotl32(h,1) ^ 簇起始绝对扇区
                            o_lba_hash  <= {o_lba_hash[30:0], o_lba_hash[31]} ^
                                           nxt_lba_x[31:0];
                            ph_fat      <= 1'b0;
                            dsec        <= 8'd0;
                            fat_sec_ofs <= 2'd0;
                            fat_cap_cnt <= 4'd0;
                            st          <= S_SET;
                        end
                    end else if (fat_sec_ofs == 2'd0) begin
                        // 表项被扇区边界切成两半（512 字节扇区下不可达，防御性代码）：
                        // 接着读下一个 FAT 扇区，窗口会继续往后捕获剩下的字节。
                        fat_sec_ofs <= 2'd1;
                        st          <= S_SET;
                    end else begin
                        // 连读两个扇区都没捕满 4 字节：几何/窗口逻辑有硬伤，别硬撑
                        err_code <= ERR_FAT_WIN;
                        st       <= S_DONE;
                    end
                end else begin
                    // ---- 刚读完一个文件数据扇区 ----
                    if (bytes_left == 32'd0) begin
                        // 文件字节已经吐完：正常结束。
                        //   注意这里不去读最后一个簇的 FAT 表项 —— 大小到了就
                        //   结束，既省一次读，也避免把"最后簇后面还挂着别的簇"
                        //   （预分配/碎片）误判成错误。
                        st <= S_DONE;
                    end else if ((dsec + 8'd1) >= spc_r) begin
                        // 本簇的最后一个扇区也读完了，但字节还没吐完 -> 去读 FAT
                        ph_fat      <= 1'b1;
                        dsec        <= 8'd0;
                        fat_sec_ofs <= 2'd0;
                        fat_cap_cnt <= 4'd0;
                        fat_buf     <= 32'd0;      // 清掉上一次的残留（防漏捕时留旧字节）
                        st          <= S_SET;
                    end else begin
                        dsec <= dsec + 8'd1;
                        st   <= S_SET;
                    end
                end
            end

            //============================================================ 收尾
            S_DONE: begin
                st <= S_DONE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
