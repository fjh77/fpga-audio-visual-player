//=============================================================================
// 文件名  : fat32_scan.v
// 功能    : 把已验证的 fat32_dir.v（FAT32 根目录解析）接到 FPGA 工程里的
//           薄封装——只负责三件事：
//             1) 上电后、SD 卡初始化完成且旧通路（扫图/语音）已经开始工作之后，
//                自动发一次 start 脉冲，让 FAT 解析在空闲时跑一遍；
//             2) 把 fat32_dir 的"请求扇区"握手接到 sd_arbiter3 的第三路
//                （最低优先级）上；
//             3) 把解析结果（文件数 o_count / 错误码 err_code）整理成
//                直接能喂数码管译码器的 4 位数字。
//
// 本模块不做什么
// ----------------
//   * 不改 fat32_dir.v 内部一个字符（它是已逐字节验证过的，动它有风险）
//   * 不读文件数据、不跟簇链 —— 那是阶段 3b（fat32_file.v）
//   * 不碰 bmp_read / wav_player / sd_card_top 的任何行为
//
// 为什么要有这一层，而不是把 fat32_dir 直接放进 sd_card_bmp
// ---------------------------------------------------------
//   1) 启动时序（等够一段延迟）和结果整理是"集成"的事，不是解析算法的事。
//      混在一起会让已验证的解析器变得难回退。
//   2) 顶层 sd_card_bmp 里加一个 parameter 就能整块旁路掉（见那里的
//      USE_FAT_SCAN），旁路时本模块整个是死逻辑，综合器会优化掉，不占资源。
//
// 复位极性
//   本工程绝大多数模块（sd_card_bmp / bmp_read / wav_player / sd_card_top）
//   用的是高有效异步复位 rst；而 fat32_dir.v 的端口是低有效 rst_n。
//   为了一行都不动 fat32_dir，这里在端口上收高有效 rst，内部取名 rst_n_i
//   并显式取反后再喂给 fat32_dir。这个取反是本模块唯一的"适配层"。
//
// 启动延迟（START_DLY_CYC）为什么需要
//   上电后 bmp_read 会从扇区 0 开始顺序扫描找前 4 张 BMP，
//   这是最耗时也最要紧的一件事（用户看到的第一张图就靠它）。
//   如果 FAT 解析这时去抢卡，虽然仲裁器保证 bmp 绝对优先、不会被拖慢，
//   但为了让"板子上的表现和改动前一模一样"，这里再主动等一段更宽裕的时间：
//   等扫描基本结束、图也显示出来了，才让 FAT 解析去用空闲时间。
//   本模块永远不会占用卡超过几十毫秒（只读 MBR+DBR+根目录共几个扇区）。
//
// 参数估算（100MHz 域，clk = sd_card_clk）
//   START_DLY_CYC = 100_000_000 -> 1 秒
//   默认 32'd1_000_000_000 = 10 秒（连上电那一下的卡初始化都留足了余量）
//=============================================================================

`timescale 1ns / 1ps

module fat32_scan #(
    // 上电后等待多少拍才开始 FAT 解析（见文件头说明）
    parameter [31:0] START_DLY_CYC = 32'd1_000_000_000,
    // 透传给 fat32_dir 的表容量 / 扇区超时 / 总等待兜底（见 fat32_dir.v 注释）
    parameter        MAX_FILES     = 10,
    parameter [31:0] TO_LIMIT      = 32'd2000000,
    parameter [31:0] TO_LIMIT_ALL  = 32'd200000000,
    // 数据前导许可（0 = 严格只认 S2_RUN，1 = 多认 S1_INIT 那一拍）
    //
    //   为什么需要有这个参数：
    //     o_rd_req = (st == S2_RUN) && fd_req，也就是"请求"要到 S2_RUN 那一拍
    //     才出现在仲裁器上；而扇区数据的第一个字节有可能在同一拍就回来
    //     （数据源零延迟时）。fat32_dir 内部的 S_REQ 状态本来就是"收到第一个
    //     字节才算进 S_READ"，所以 S1_INIT 那一拍若把 i_data_valid 丢掉，
    //     第一个字节就永远补不回来了 —— 表现为"每个扇区的数据整体错位一字节"，
    //     512 字节收完时签名对不上，报 ERR_MBR_SIG(1)。
    //
    //   真实板子上 SD 卡每读一个扇区要等几十微秒（跨时钟、SPI 位传输），
    //   第一个字节不可能落在 S1_INIT 那一拍，所以 0/1 在硬件上行为相同。
    //   本参数的存在是为了：
    //     * 仿真里能用同一个 DUT 复现这个坑（tb 用 0 复现、用 1 验证修复）；
    //     * 万一将来换成"BRAM 里的假卡"这种零延迟数据源，也不会再错位。
    //   综合时它是常量，不占任何资源。
    // 默认 1 ：这是严格更稳的一侧（多认一拍不会多收数据，只会防止丢首字节）。
    parameter        DATA_LEAD     = 1
)(
    input             clk,
    input             rst,              // 高有效异步复位（本模块端口约定）

    // ---- 启动条件：SD 卡初始化完成 + 旧通路已经开始工作 ----
    //   i_old_busy : 旧通路（扫图/装载/语音）正在占用卡。
    //                1 时本模块不发起请求，等它空下来再说。
    //                仲裁器本来就保证现有通路绝对优先，这里只是"礼让"，
    //                让卡在旧通路忙的时候完全看不到 FAT 的存在。
    input             i_sd_init_done,
    input             i_old_busy,

    // ---- 扇区读取握手（接 sd_arbiter3 的 fat_* 那一路）----
    output            o_rd_req,
    output     [31:0] o_rd_lba,
    input      [7:0]  i_data,
    input             i_data_valid,
    input             i_data_end,
    // ---- 2026-10 超时修复新增：本路"已被仲裁器授权用卡" ----
    //   直接来自 sd_arbiter3 的 o_grant_fat。透传给 fat32_dir：
    //   排队等卡的时间不算传输超时（完整理由见 fat32_dir.v 要点 (F)）。
    // 没有仲裁器的场合接 1'b1（退化成修复前行为）
    input             i_granted,

    // ---- 结果（给数码管）----
    //   o_count     : FAT 根目录里解析到的文件条数（真实值，0~MAX_FILES）
    //   o_err_code  : fat32_dir 的错误码，0 = 无错
    //   o_done      : 解析完成（电平保持）
    //   o_busy      : 解析进行中
    output     [4:0]  o_count,
    output     [3:0]  o_err_code,
    output            o_done,
    output            o_busy,
    //   数位显示用（已经做过去重/截断保护）：
    //     o_cnt_digit : 文件数的单个十进制位。>9 时钳到 9，
    //                   并把 o_cnt_ovf 拉高，便于分辨"真的是 9 个"和"超过 9 个"。
    //     o_err_digit : 错误码的单个十六进制位（0~15）。
    output     [3:0]  o_cnt_digit,
    output            o_cnt_ovf,
    output     [3:0]  o_err_digit,

    // ---- 临时诊断：fat32_dir 内部的关键值，原样引到顶层数码管 ----
    //   本层只做端口到端口的直连，不加任何寄存器/逻辑，
    //   所以既不会引入多驱动，也不可能改变原有行为。
    //   （这些信号以前是悬空的，本次只是把它们接出来）
    output     [31:0] o_dbg_root_lba,   // 根目录首簇的绝对首扇区（真卡期望 0x0000A000）
    output     [31:0] o_dbg_part_lba,   // MBR 分区起始 LBA      （真卡期望 0x00002000）
    output     [31:0] o_dbg_data_lba,   // 数据区起始绝对扇区      （真卡期望 0x0000A000）
    output     [31:0] o_dbg_fat_lba,    // FAT1 起始绝对扇区       （真卡期望 0x00002974）
    output     [15:0] o_dbg_secs,       // 成功收到的扇区数
    output     [7:0]  o_dbg_part_type,  // 分区类型字节（0x0C = FAT32 LBA）
    output     [7:0]  o_dbg_root_b0,    // 根目录首扇区第 1 字节（真卡期望 0xE5）
    output     [7:0]  o_dbg_root_b1,    // 根目录首扇区第 2 字节（真卡期望 0x20）
    output     [15:0] o_dbg_rx_bytes    // 收到的数据字节总数（6 扇区 -> 0x0C00）
);

    //-------------------------------------------------------------------------
    // 启动 FSM
    //   S0_IDLE : 等 SD 初始化 + 等够 START_DLY_CYC 拍
    //   S1_INIT : start 置 1（fat32_dir 在 S_IDLE 看到即启动）
    //   S2_RUN  : 撤 start，之后完全看 busy/done
    //   S3_DONE : 保持（只跑一次，不再自动重来）
    //-------------------------------------------------------------------------
    localparam [1:0] S0_IDLE = 2'd0,
                     S1_INIT = 2'd1,
                     S2_RUN  = 2'd2,
                     S3_DONE = 2'd3;

    reg [1:0]  st;
    reg [31:0] dly_cnt;

    // fat32_dir 侧握手信号
    reg        fd_start;
    wire       fd_busy;
    wire       fd_done;
    wire [3:0] fd_err;
    wire [31:0] fd_lba;
    wire       fd_req;
    wire [4:0] fd_count;

    // fat32_dir 是低有效复位，这里是高有效，端口上直接取反适配
    wire rst_n_i = ~rst;

    // 送进 fat32_dir 的两个握手信号（见 DATA_LEAD 说明）
    wire fd_dvalid;

    assign o_busy     = fd_busy;
    assign o_done     = fd_done;
    assign o_count    = fd_count;
    assign o_err_code = fd_err;

    // 送给仲裁器：只有在"等子模块要扇区"的状态里才把请求透出去
    assign o_rd_req = (st == S2_RUN) && fd_req;
    assign o_rd_lba = fd_lba;

    // 送给子模块的数据有效：见 DATA_LEAD 的说明
    //   DATA_LEAD=0：只有 S2_RUN（与最初的写法一致）
    //   DATA_LEAD=1：S1_INIT 也认（第一个字节恰好落在起跑那一拍时不会丢）
    assign fd_dvalid = (DATA_LEAD != 0) ? ((st == S2_RUN) || (st == S1_INIT)) && i_data_valid
                                        : (st == S2_RUN) && i_data_valid;

    //-------------------------------------------------------------------------
    // 结果整理：给数码管用的两个单字位
    // 用组合 assign，不额外占寄存器，也不可能与 o_count 不一致
    //-------------------------------------------------------------------------
    assign o_cnt_ovf   = (fd_count > 5'd9);
    assign o_cnt_digit = (fd_count > 5'd9) ? 4'd9 : fd_count[3:0];
    assign o_err_digit = fd_err;

    //-------------------------------------------------------------------------
    // 启动延迟计数 + FSM
    //-------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st       <= S0_IDLE;
            dly_cnt  <= 32'd0;
            fd_start <= 1'b0;
        end else begin
            // 默认：start 是单拍脉冲
            fd_start <= 1'b0;

            case (st)
            //------------------------------------------------------------ 等待
            S0_IDLE: begin
                if (!i_sd_init_done) begin
                    // 卡还没起来：计数器清零，从头算（上电卡初始化可能要几百 ms）
                    dly_cnt <= 32'd0;
                end else if (dly_cnt < START_DLY_CYC) begin
                    dly_cnt <= dly_cnt + 32'd1;
                end else if (!i_old_busy) begin
                    // 延迟到点、且旧通路当前空闲 -> 起跑
                    st <= S1_INIT;
                end
            end

            //------------------------------------------------------------ 起跑
            S1_INIT: begin
                fd_start <= 1'b1;      // 一拍脉冲；fat32_dir 在 S_IDLE 收到即启动
                st       <= S2_RUN;
            end

            //------------------------------------------------------------ 跑
            S2_RUN: begin
                if (fd_done)
                    st <= S3_DONE;
            end

            //------------------------------------------------------------ 完成
            S3_DONE: begin
                st <= S3_DONE;         // 只跑一次；想看新的结果就复位重来
            end

            default: st <= S0_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 被封装的主角：fat32_dir（一个字都没改）
    //   文件表读口本阶段用不到（阶段 3a 只看条数），把下标固定为 0，
    //   四条表输出留空不接 —— 它们对综合没有影响。
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
        .i_tbl_idx     (4'd0),
        .o_tbl_name    (),
        .o_tbl_cluster (),
        .o_tbl_size    (),
        .o_tbl_attr    (),

        .tgt_we        (1'b0),
        .tgt_name      (88'd0),

        // 临时诊断：内部关键值直连出去（纯连线，零逻辑）
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
