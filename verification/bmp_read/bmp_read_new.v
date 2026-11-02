//=============================================================================
// 【验证副本】本文件是 hardware/.../hdl_source/SD/bmp_read.v 的逐字拷贝，
//   仅把模块名改成 bmp_read_new 以便与 bmp_read_ref 做差分仿真。
//   修改 RTL 后请同步本文件（cp + 改模块名）。
//=============================================================================
// 文件名  : bmp_read.v
// 功能    : 传统"顺序扫扇区"通路的 BMP 扫描 / 装载器
//
// 本模块属于哪条通路
// ------------------
//   本工程的图片装载有两条二选一的通路（见 sd_card_bmp.v 的 USE_FAT_LOAD）：
//     USE_FAT_LOAD=1（默认）-> bmp_load_fat.v：FAT32 簇链直读，文件名不限
//     USE_FAT_LOAD=0         -> 本模块：从起始扇区起顺序扫，靠 BM 魔数认图
//   默认构建里本模块不被例化（综合日志中不出现），只在把顶层
//   .USE_FAT_LOAD 改成 1'b0 时才启用。保留它是为了留一条"卡上没有
//   合法 FAT 表也能出图"的兜底路径。
//
// 内部结构
// --------
//   扇区字节流 ──┬─→ [头缓冲 hdr[0:53]] ──固定下标切片──→ 各字段（组合）
//                └─→ [像素相位 px_ph + 24bit 移位装配] ──→ bmp_data / wr_en
//
//   三个实现要点（与逐字节位置判断的传统写法相比）：
//     1) 头解析 = 先把前 54 字节整块存进寄存器组，再用固定下标切片取值。
//        字段偏移是 BMP 格式规定的常量，切成 wire 之后每个字段都是直连，
//        不需要逐字节判断"现在收到第几个字节、该写哪个字段的哪几位"。
//     2) 像素装配 = 24bit 移位寄存器"右移、新字节进最高位"。BMP 像素
//        在文件里的顺序是 B,G,R，右移三次之后寄存器正好是 {R,G,B}，
//        输出顺序由移位方向自然得到，不需要额外的相位重排。
//     3) 数据段结束判断 = 装载前先算好"还要读几个扇区"的预算并递减，
//        不在数据段里逐字节和 file_len 比较。
//
// 时序约定（改动时不要破坏）
// ------------------------
//   * bmp_data_wr_en 必须与 bmp_data 同一拍有效（第 3 个像素字节到达那一拍
//     同时置起），错一拍整屏横向错位。
//   * sd_sec_read 在每个扇区读完那一拍拉低一拍，再拉高读下一个扇区。
//     这一拍的低电平是仲裁器重新裁决的窗口，不能省。
//   * state_code 的编码被主程序用于数码管显示：1=IDLE/空闲，2=读头/扫描，
//     3=等帧缓冲让出，4=送像素。改编码会让现场显示对不上号。
//=============================================================================

`timescale 1ns / 1ps

module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

//-----------------------------------------------------------------------------
// 常量
//-----------------------------------------------------------------------------
localparam [2:0] ST_IDLE      = 3'd0;
localparam [2:0] ST_SCAN      = 3'd1;
localparam [2:0] ST_LOAD_HDR  = 3'd2;
localparam [2:0] ST_LOAD_WAIT = 3'd3;
localparam [2:0] ST_LOAD_DATA = 3'd4;

// BMP 头长度（BITMAPINFOHEADER 的 14 字节文件头 + 40 字节信息头）
localparam [7:0] HDR_LEN      = 8'd54;

// 魔数 'B' 'M' 按小端拼成 16 位后的值
localparam [15:0] BMP_MAGIC    = 16'h4D42;

//-----------------------------------------------------------------------------
// 状态与通用寄存器
//-----------------------------------------------------------------------------
reg [2:0]  state;
reg [9:0]  hdr_off;                 // 本扇区已接收的字节序号（0~511，必须 10 位）
reg [31:0] scan_sector;             // 扫描游标
reg [31:0] load_sector_latched;     // 本次装载的起始扇区
reg [31:0] byte_off;                // 数据段已接收的字节序号（文件第 0 字节起算）
reg [31:0] sect_budget;             // 数据段还要读几个扇区
reg [1:0]  px_ph;                   // 像素相位：0=等 B，1=等 G，2=等 R
reg [23:0] px_sr;                   // 像素移位装配

//-----------------------------------------------------------------------------
// 头缓冲：把每个扇区的前 54 字节整块存下来
//   hdr_cap 只在扫描 / 读头两个状态有效，数据段不会污染这里，
//   所以字段切片在数据段期间是稳定的，可以直接当寄存器用。
//-----------------------------------------------------------------------------
reg [7:0] hdr [0:53];

wire hdr_stage = (state == ST_SCAN) || (state == ST_LOAD_HDR);
wire hdr_cap   = sd_sec_read_data_valid && hdr_stage && (hdr_off < HDR_LEN);

integer hdr_i;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (hdr_i = 0; hdr_i < 54; hdr_i = hdr_i + 1)
            hdr[hdr_i] <= 8'd0;
    end else if (hdr_cap) begin
        hdr[hdr_off] <= sd_sec_read_data;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        hdr_off <= 10'd0;
    end else if (hdr_stage) begin
        if (sd_sec_read_data_valid)
            hdr_off <= hdr_off + 10'd1;
        else if (sd_sec_read_end)
            hdr_off <= 10'd0;
    end else begin
        hdr_off <= 10'd0;
    end
end

//-----------------------------------------------------------------------------
// 头字段切片（BMP 头字段一律小端，最低字节在最小偏移）
//-----------------------------------------------------------------------------
wire [15:0] f_magic = {hdr[1],  hdr[0]};                            // "BM"
wire [31:0] f_flen  = {hdr[5],  hdr[4],  hdr[3],  hdr[2]};          // 文件总字节数
wire [31:0] f_pxoff = {hdr[13], hdr[12], hdr[11], hdr[10]};         // 像素数据起始偏移
wire [31:0] f_wid   = {hdr[21], hdr[20], hdr[19], hdr[18]};         // 宽
wire [31:0] f_hgt   = {hdr[25], hdr[24], hdr[23], hdr[22]};         // 高
wire [15:0] f_bpp   = {hdr[29], hdr[28]};                           // 位深
wire [31:0] f_cmp   = {hdr[33], hdr[32], hdr[31], hdr[30]};         // 压缩方式

// 认图条件：BM 魔数 + 尺寸与顶层期望一致 + 24 位 + 不压缩
wire header_match = (f_magic == BMP_MAGIC) &&
                    (f_wid[15:0] == bmp_width) &&
                    (f_hgt[15:0] == bmp_height) &&
                    (f_bpp == 16'd24) &&
                    (f_cmp == 32'd0);

// 整份文件占几个扇区（文件长为 0 时按 1 个算，避免预算为 0 而不读）
wire [31:0] file_sectors = (f_flen == 32'd0) ? 32'd1
                                             : ((f_flen + 32'd511) >> 9);

// 扫描推进量：命中就跳过整份文件，未命中只前进一个扇区
wire [31:0] scan_next_hit  = scan_sector + file_sectors;
wire [31:0] scan_next_miss = scan_sector + 32'd1;

//-----------------------------------------------------------------------------
// 数据段：字节游标 + 像素窗口
//-----------------------------------------------------------------------------
wire data_stage = (state == ST_LOAD_DATA);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        byte_off <= 32'd0;
    end else if (data_stage) begin
        if (sd_sec_read_data_valid)
            byte_off <= byte_off + 32'd1;
    end else begin
        byte_off <= 32'd0;
    end
end

// 只有落在 [像素偏移, 文件长) 区间内的字节才是像素
wire px_byte_valid = sd_sec_read_data_valid &&
                     (byte_off >= f_pxoff) &&
                     (byte_off <  f_flen);

//-----------------------------------------------------------------------------
// 像素装配：右移、新字节进最高位 -> 收满三个字节后寄存器即 {R,G,B}
//-----------------------------------------------------------------------------
wire [23:0] px_sr_nxt = {sd_sec_read_data, px_sr[23:8]};

always @(posedge clk or posedge rst) begin
    if (rst) begin
        px_sr          <= 24'd0;
        px_ph          <= 2'd0;
        bmp_data       <= 24'd0;
        bmp_data_wr_en <= 1'b0;
    end else if (data_stage) begin
        if (px_byte_valid) begin
            px_sr          <= px_sr_nxt;
            bmp_data       <= px_sr_nxt;
            bmp_data_wr_en <= (px_ph == 2'd2);
            px_ph          <= (px_ph == 2'd2) ? 2'd0 : (px_ph + 2'd1);
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        // 离开数据段就把相位归零：每张图必须从"等 B 字节"开始，
        // 否则上一张图剩下的半个像素会把下一张图的像素边界整体挪位。
        bmp_data_wr_en <= 1'b0;
        px_ph          <= 2'd0;
        px_sr          <= 24'd0;
    end
end

//-----------------------------------------------------------------------------
// 主状态机
//-----------------------------------------------------------------------------
assign ready = (state == ST_IDLE);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state               <= ST_IDLE;
        state_code          <= 4'd0;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        scan_found_total    <= 3'd0;
        scan_sector         <= 32'd0;
        load_sector_latched <= 32'd0;
        sect_budget         <= 32'd0;
    end else if (!sd_init_done) begin
        // 卡在重新初始化：扫描结果与在途装载全部作废
        state               <= ST_IDLE;
        state_code          <= 4'd0;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        scan_found_total    <= 3'd0;
        scan_sector         <= 32'd0;
        load_sector_latched <= 32'd0;
        sect_budget         <= 32'd0;
    end else if (load_abort) begin
        // 中止当前这一次装载，但不作废扫描结果。
        //
        //   load_abort 来自 sd_card_bmp 的装载看门狗（3 秒）。扫描只在上电时
        //   kick 一次，scan_done 一旦被清就再也回不到 1，而所有按键动作
        //   （下一张 / 轮播 / 切档）都门控在 scan_done 上，结果就是"切几次图
        //   之后整个系统失去响应"。所以这里只清在途的装载状态，
        //   scan_done / scan_found_total / scan_found_sector / scan_sector
        //   必须保留 —— 中止后图片列表依然有效，按键应当继续可用。
        state               <= ST_IDLE;
        state_code          <= 4'd0;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_found_valid    <= 1'b0;
        load_sector_latched <= 32'd0;
        sect_budget         <= 32'd0;
    end else begin
        // scan_found_valid 是单拍脉冲，默认每拍拉低
        scan_found_valid <= 1'b0;

        case (state)
            //-----------------------------------------------------------------
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            //-----------------------------------------------------------------
            // 扫描：一个扇区一个扇区往前试，命中的跳过整份文件
            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        // 张数够了、或下一张已越过扫描上界 -> 扫描结束，回空闲
                        if ((scan_found_total + 3'd1 >= scan_target_count) ||
                            (scan_next_hit > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            scan_sector      <= scan_next_hit;
                            sd_sec_read_addr <= scan_next_hit;
                        end else begin
                            scan_sector      <= scan_next_hit;
                            sd_sec_read_addr <= scan_next_hit;
                        end
                    end else if (scan_sector >= scan_max_sector) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        scan_sector      <= scan_next_miss;
                        sd_sec_read_addr <= scan_next_miss;
                    end
                end
            end

            //-----------------------------------------------------------------
            // 读头：读第一个扇区拿字段，合格就申请帧缓冲写权限
            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        write_req        <= 1'b1;
                        sect_budget      <= file_sectors;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        // 不是我们要的图，放弃本次装载
                        state <= ST_IDLE;
                    end
                end
            end

            //-----------------------------------------------------------------
            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            //-----------------------------------------------------------------
            // 数据段：从文件第 0 字节重新流一遍，跳过头之后拼像素
            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    sect_budget <= sect_budget - 32'd1;

                    if (sect_budget <= 32'd1) begin
                        state <= ST_IDLE;          // 预算用完，整份文件读完
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            //-----------------------------------------------------------------
            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
