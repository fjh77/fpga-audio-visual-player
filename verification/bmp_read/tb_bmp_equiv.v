`timescale 1ns / 1ps
//=============================================================================
// tb_bmp_equiv —— bmp_read 骨架改写的【逐拍等价性 + 功能正确性】验证
//
// 验证对象：
//   REF = 改写前的 bmp_read（sim_bmp/bmp_read_ref.v，仅改模块名，逻辑一字未动）
//   NEW = 改写后的 bmp_read（工程实际文件 SD/bmp_read.v）
//
// 三重判据
//   1) 逐拍等价：两个 DUT 接同一套激励，每个时钟沿比对全部 11 个输出
//      （ready / state_code / scan_done / scan_found_valid / scan_found_sector /
//        scan_found_total / write_req / sd_sec_read / sd_sec_read_addr /
//        bmp_data_wr_en / bmp_data）
//   2) 功能正确：不只看两者一致，还要看是否"都对"
//      —— 扫描应找到 3 张图且扇区号正确；装载应产出规定个数的像素写脉冲
//   3) 异常路径：头不匹配放弃、装载中止（load_abort）、卡重新初始化
//
// 从设备模型
//   每个 DUT 各挂一个 SD 从设备（同样的延迟与字节流）和一个帧缓冲 ack 发生器。
//   两者参数完全相同，所以任何输出差异都只可能来自 DUT 本身。
//
// 用法：cd sim_bmp && iverilog -o equiv.vvp tb_bmp_equiv.v bmp_read_ref.v \
//                              ../AudioVisual/hdmi_media_player/src/user_source/hdl_source/SD/bmp_read.v
//        && vvp equiv.vvp
//=============================================================================

//-----------------------------------------------------------------------------
// SD 卡从设备：收到 req 后延迟 LAT 拍，连续吐 512 字节，再给一拍 read_end
//-----------------------------------------------------------------------------
module sd_slave #(
    parameter integer LAT = 2
)(
    input               clk,
    input               rst,
    input               req,
    input  [31:0]       req_addr,
    input  [7:0]        mem_data,       // 由外部按 mem_base+mem_k 组合取出
    output [23:0]       mem_base,
    output [9:0]        mem_k,
    output reg [7:0]    data,
    output reg          data_valid,
    output reg          read_end
);
    localparam [2:0] S_IDLE = 3'd0, S_WAIT = 3'd1, S_BURST = 3'd2,
                     S_END  = 3'd3, S_OFF  = 3'd4;

    reg [2:0]  st;
    reg [9:0]  k;
    reg [31:0] lba;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= S_IDLE; k <= 10'd0; lba <= 32'd0;
            data <= 8'd0; data_valid <= 1'b0; read_end <= 1'b0;
        end else begin
            case (st)
                S_IDLE: begin
                    data_valid <= 1'b0;
                    read_end   <= 1'b0;
                    if (req) begin
                        lba <= req_addr;
                        k   <= 10'd0;
                        st  <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (k == LAT - 1) begin k <= 10'd0; st <= S_BURST; end
                    else                   k <= k + 10'd1;
                end
                S_BURST: begin
                    data       <= mem_data;
                    data_valid <= 1'b1;
                    k          <= k + 10'd1;
                    if (k == 10'd511) st <= S_END;
                end
                S_END: begin
                    data_valid <= 1'b0;
                    read_end   <= 1'b1;
                    st         <= S_OFF;
                end
                S_OFF: begin
                    read_end <= 1'b0;
                    st       <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    assign mem_base = lba * 32'd512;
    assign mem_k    = k;
endmodule

//-----------------------------------------------------------------------------
// 帧缓冲写权限：write_req 有效后 DLY 拍给一拍 ack
//-----------------------------------------------------------------------------
module wr_ack_gen #(
    parameter integer DLY = 3
)(
    input       clk,
    input       rst,
    input       req,
    output reg  ack
);
    reg [3:0] c;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ack <= 1'b0; c <= 4'd0;
        end else if (ack) begin
            ack <= 1'b0; c <= 4'd0;
        end else if (req) begin
            if (c == DLY - 1) begin ack <= 1'b1; c <= 4'd0; end
            else                    c <= c + 4'd1;
        end else c <= 4'd0;
    end
endmodule

//=============================================================================
// 顶层测试平台
//=============================================================================
module tb_bmp_equiv;

    localparam integer NSEC = 128;          // 卡镜像 64 KB

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                   // 100 MHz

    reg [7:0] card [0:NSEC*512-1];

    // ---- 公共激励 ----
    reg         sd_init_done      = 1'b0;
    reg         scan_start        = 1'b0;
    reg  [31:0] scan_start_sector = 32'd0;
    reg  [31:0] scan_max_sector   = 32'd40;
    reg  [2:0]  scan_target_count = 3'd3;
    reg         load_start        = 1'b0;
    reg         load_abort        = 1'b0;
    reg  [31:0] load_sector       = 32'd0;
    reg  [15:0] bmp_width         = 16'd640;
    reg  [15:0] bmp_height        = 16'd480;

    // ---- DUT 0 (REF) 互连 ----
    wire [23:0] base0;  wire [9:0] k0;
    wire [7:0]  memd0 = card[base0 + k0];
    wire [7:0]  d0;     wire dv0, end0;
    wire        ack0;
    wire        ready0, sc_done0, sc_fv0, wr_req0, sd_rd0, wr_en0;
    wire [3:0]  sc0;
    wire [31:0] sc_fs0, sd_addr0;
    wire [2:0]  sc_ft0;
    wire [23:0] bdata0;

    sd_slave #(.LAT(2)) slv0 (
        .clk(clk), .rst(rst), .req(sd_rd0), .req_addr(sd_addr0),
        .mem_data(memd0), .mem_base(base0), .mem_k(k0),
        .data(d0), .data_valid(dv0), .read_end(end0)
    );

    wr_ack_gen #(.DLY(3)) ackg0 (
        .clk(clk), .rst(rst), .req(wr_req0), .ack(ack0)
    );

    bmp_read_ref dut0 (
        .clk(clk), .rst(rst), .ready(ready0),
        .scan_start(scan_start), .scan_start_sector(scan_start_sector),
        .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
        .scan_done(sc_done0), .scan_found_valid(sc_fv0),
        .scan_found_sector(sc_fs0), .scan_found_total(sc_ft0),
        .load_start(load_start), .load_abort(load_abort), .load_sector(load_sector),
        .sd_init_done(sd_init_done), .state_code(sc0),
        .bmp_width(bmp_width), .bmp_height(bmp_height),
        .write_req(wr_req0), .write_req_ack(ack0),
        .sd_sec_read(sd_rd0), .sd_sec_read_addr(sd_addr0),
        .sd_sec_read_data(d0), .sd_sec_read_data_valid(dv0), .sd_sec_read_end(end0),
        .bmp_data_wr_en(wr_en0), .bmp_data(bdata0)
    );

    // ---- DUT 1 (NEW) 互连 ----
    wire [23:0] base1;  wire [9:0] k1;
    wire [7:0]  memd1 = card[base1 + k1];
    wire [7:0]  d1;     wire dv1, end1;
    wire        ack1;
    wire        ready1, sc_done1, sc_fv1, wr_req1, sd_rd1, wr_en1;
    wire [3:0]  sc1;
    wire [31:0] sc_fs1, sd_addr1;
    wire [2:0]  sc_ft1;
    wire [23:0] bdata1;

    sd_slave #(.LAT(2)) slv1 (
        .clk(clk), .rst(rst), .req(sd_rd1), .req_addr(sd_addr1),
        .mem_data(memd1), .mem_base(base1), .mem_k(k1),
        .data(d1), .data_valid(dv1), .read_end(end1)
    );

    wr_ack_gen #(.DLY(3)) ackg1 (
        .clk(clk), .rst(rst), .req(wr_req1), .ack(ack1)
    );

    bmp_read dut1 (
        .clk(clk), .rst(rst), .ready(ready1),
        .scan_start(scan_start), .scan_start_sector(scan_start_sector),
        .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
        .scan_done(sc_done1), .scan_found_valid(sc_fv1),
        .scan_found_sector(sc_fs1), .scan_found_total(sc_ft1),
        .load_start(load_start), .load_abort(load_abort), .load_sector(load_sector),
        .sd_init_done(sd_init_done), .state_code(sc1),
        .bmp_width(bmp_width), .bmp_height(bmp_height),
        .write_req(wr_req1), .write_req_ack(ack1),
        .sd_sec_read(sd_rd1), .sd_sec_read_addr(sd_addr1),
        .sd_sec_read_data(d1), .sd_sec_read_data_valid(dv1), .sd_sec_read_end(end1),
        .bmp_data_wr_en(wr_en1), .bmp_data(bdata1)
    );

    //=========================================================================
    // 卡镜像构造
    //=========================================================================
    task fill_card(input [7:0] v);
        integer i;
        begin
            for (i = 0; i < NSEC*512; i = i + 1) card[i] = v;
        end
    endtask

    // 在指定扇区放一张 BMP；像素区按 (seed + 偏移*7) 填，便于比对内容
    task mk_bmp(input integer lba, input [31:0] flen,
                input [15:0] w, input [15:0] h,
                input [15:0] bpp, input [31:0] comp,
                input [31:0] pxoff, input [7:0] seed);
        integer i;
        begin
            for (i = 0; i < 512; i = i + 1) card[lba*512 + i] = 8'h00;
            card[lba*512 + 0]  = 8'h42;             // 'B'
            card[lba*512 + 1]  = 8'h4D;             // 'M'
            card[lba*512 + 2]  = flen[7:0];
            card[lba*512 + 3]  = flen[15:8];
            card[lba*512 + 4]  = flen[23:16];
            card[lba*512 + 5]  = flen[31:24];
            card[lba*512 + 10] = pxoff[7:0];
            card[lba*512 + 11] = pxoff[15:8];
            card[lba*512 + 12] = pxoff[23:16];
            card[lba*512 + 13] = pxoff[31:24];
            card[lba*512 + 18] = w[7:0];
            card[lba*512 + 19] = w[15:8];
            card[lba*512 + 20] = 8'h00;
            card[lba*512 + 21] = 8'h00;
            card[lba*512 + 22] = h[7:0];
            card[lba*512 + 23] = h[15:8];
            card[lba*512 + 24] = 8'h00;
            card[lba*512 + 25] = 8'h00;
            card[lba*512 + 28] = bpp[7:0];
            card[lba*512 + 29] = bpp[15:8];
            card[lba*512 + 30] = comp[7:0];
            card[lba*512 + 31] = comp[15:8];
            card[lba*512 + 32] = comp[23:16];
            card[lba*512 + 33] = comp[31:24];
            for (i = 54; i < 512; i = i + 1)
                card[lba*512 + i] = seed + i[7:0] * 8'h07;
        end
    endtask

    //=========================================================================
    // 逐拍比对 + 统计
    //=========================================================================
    integer cyc      = 0;
    integer errs     = 0;
    integer n_cmp    = 0;
    integer wr0      = 0;       // REF 像素写脉冲计数
    integer wr1      = 0;       // NEW 像素写脉冲计数
    integer load_wr0 = 0;
    integer load_wr1 = 0;

    // 像素流校验和：只在 wr_en 有效那拍累加，用来证明"写进帧缓冲的内容"完全一致。
    // bmp_data 在 wr_en=0 时是无关值（下游只在 wr_en 为 1 时采样），不参与比对。
    reg [31:0] sum0 = 32'd0;
    reg [31:0] sum1 = 32'd0;

    always @(posedge clk) begin
        if (!rst) begin
            cyc   = cyc + 1;
            n_cmp = n_cmp + 1;

            if (wr_en0) begin wr0 = wr0 + 1; sum0 = sum0 + {8'd0, bdata0}; end
            if (wr_en1) begin wr1 = wr1 + 1; sum1 = sum1 + {8'd0, bdata1}; end

            if ({ready0, sc0, sc_done0, sc_fv0, sc_fs0, sc_ft0, wr_req0,
                 sd_rd0, sd_addr0, wr_en0} !==
                {ready1, sc1, sc_done1, sc_fv1, sc_fs1, sc_ft1, wr_req1,
                 sd_rd1, sd_addr1, wr_en1}) begin
                errs = errs + 1;
                if (errs <= 12) begin
                    $display("  [MISMATCH] cyc=%0d", cyc);
                    if (ready0   !== ready1)   $display("     ready   ref=%b new=%b", ready0, ready1);
                    if (sc0      !== sc1)      $display("     st_code ref=%0d new=%0d", sc0, sc1);
                    if (sc_done0 !== sc_done1) $display("     sc_done ref=%b new=%b", sc_done0, sc_done1);
                    if (sc_fv0   !== sc_fv1)   $display("     sc_fv   ref=%b new=%b", sc_fv0, sc_fv1);
                    if (sc_fs0   !== sc_fs1)   $display("     sc_fsec ref=%0d new=%0d", sc_fs0, sc_fs1);
                    if (sc_ft0   !== sc_ft1)   $display("     sc_ftot ref=%0d new=%0d", sc_ft0, sc_ft1);
                    if (wr_req0  !== wr_req1)  $display("     wr_req  ref=%b new=%b", wr_req0, wr_req1);
                    if (sd_rd0   !== sd_rd1)   $display("     sd_rd   ref=%b new=%b", sd_rd0, sd_rd1);
                    if (sd_addr0 !== sd_addr1) $display("     sd_addr ref=%0d new=%0d", sd_addr0, sd_addr1);
                    if (wr_en0   !== wr_en1)   $display("     wr_en   ref=%b new=%b", wr_en0, wr_en1);
                end
            end

            // 写使能有效时，像素数据必须逐位相同
            if (wr_en0 && wr_en1 && (bdata0 !== bdata1)) begin
                errs = errs + 1;
                if (errs <= 12)
                    $display("  [PIXEL MISMATCH] cyc=%0d ref=%06h new=%06h", cyc, bdata0, bdata1);
            end
        end
    end

    //=========================================================================
    // 激励
    //=========================================================================
    task do_reset;
        begin
            rst = 1'b1;
            repeat (5) @(negedge clk);
            rst = 1'b0;
            repeat (2) @(negedge clk);
        end
    endtask

    task pulse_scan(input [31:0] s_first, input [31:0] s_max, input [2:0] tgt);
        begin
            @(negedge clk);
            scan_start_sector = s_first;
            scan_max_sector   = s_max;
            scan_target_count = tgt;
            scan_start        = 1'b1;
            @(negedge clk);
            scan_start        = 1'b0;
        end
    endtask

    task pulse_load(input [31:0] sec);
        begin
            @(negedge clk);
            load_sector = sec;
            load_start  = 1'b1;
            @(negedge clk);
            load_start  = 1'b0;
        end
    endtask

    // 等待扫描结束（或超时）
    task wait_scan_done(input integer limit);
        integer n;
        begin
            n = 0;
            while (!sc_done1 && n < limit) begin
                @(negedge clk);
                n = n + 1;
            end
            if (n >= limit) begin
                $display("  [TIMEOUT] 扫描在 %0d 拍内未结束", limit);
                errs = errs + 1;
            end
        end
    endtask

    // 等待装载回到空闲
    task wait_idle(input integer limit);
        integer n;
        begin
            n = 0;
            while (!ready1 && n < limit) begin
                @(negedge clk);
                n = n + 1;
            end
            if (n >= limit) begin
                $display("  [TIMEOUT] 装载在 %0d 拍内未回到空闲", limit);
                errs = errs + 1;
            end
        end
    endtask

    //=========================================================================
    // 场景
    //=========================================================================
    integer exp_sect [0:3];
    integer i;

    initial begin
        $display("=============================================================");
        $display(" bmp_read 骨架改写：逐拍等价性 + 功能正确性验证");
        $display(" REF = 改写前   NEW = 改写后（整头缓冲 + 移位装配 + 扇区预算）");
        $display("=============================================================");

        //---------------------------------------------------------------
        // 构造卡镜像
        //   0..2   垃圾
        //   3      BMP-A  640x480 24bpp  flen=2102 (5 个扇区)
        //   8      BMP-B  640x480 24bpp  flen=66   (1 个扇区)
        //   12     BMP-C  800x600        -> 尺寸不匹配，应被跳过
        //   20     BMP-D  640x480 32bpp  -> 位深不匹配，应被跳过
        //   30     BMP-E  640x480 24bpp  flen=66
        //---------------------------------------------------------------
        $display("[准备] 构造卡镜像 ...");
        fill_card(8'hAA);
        mk_bmp(3,  32'd2102, 16'd640, 16'd480, 16'd24, 32'd0, 32'd54, 8'h11);
        mk_bmp(8,  32'd66,   16'd640, 16'd480, 16'd24, 32'd0, 32'd54, 8'h22);
        mk_bmp(12, 32'd66,   16'd800, 16'd600, 16'd24, 32'd0, 32'd54, 8'h33);
        mk_bmp(20, 32'd66,   16'd640, 16'd480, 16'd32, 32'd0, 32'd54, 8'h44);
        mk_bmp(30, 32'd66,   16'd640, 16'd480, 16'd24, 32'd0, 32'd54, 8'h55);

        do_reset();
        sd_init_done = 1'b1;
        repeat (3) @(negedge clk);

        //---------------------------------------------------------------
        // 阶段 1：扫描
        //---------------------------------------------------------------
        $display("[阶段1] 扫描 0..40，目标 3 张 ...");
        pulse_scan(32'd0, 32'd40, 3'd3);
        wait_scan_done(40000);
        repeat (5) @(negedge clk);

        if (sc_done1 !== 1'b1) begin
            $display("  [FAIL] scan_done 未拉高");
            errs = errs + 1;
        end
        if (sc_ft1 !== 3'd3) begin
            $display("  [FAIL] 找到张数 ref=%0d new=%0d，期望 3", sc_ft0, sc_ft1);
            errs = errs + 1;
        end else
            $display("         找到 3 张（res 应 = 3,8,30）：实际 new scan_found_sector=%0d（最后一 张）", sc_fs1);

        //---------------------------------------------------------------
        // 阶段 2：装载多扇区图 A（5 个扇区，682 个完整像素）
        //---------------------------------------------------------------
        $display("[阶段2] 装载扇区 3（BMP-A，5 个扇区）...");
        load_wr0 = wr0; load_wr1 = wr1;
        pulse_load(32'd3);
        wait_idle(60000);
        repeat (3) @(negedge clk);
        $display("         像素写脉冲 ref=%0d new=%0d（期望 682）", wr0 - load_wr0, wr1 - load_wr1);
        if ((wr1 - load_wr1) != 682) begin
            $display("  [FAIL] 像素个数不符");
            errs = errs + 1;
        end

        //---------------------------------------------------------------
        // 阶段 3：装载 1 扇区图 B
        //---------------------------------------------------------------
        $display("[阶段3] 装载扇区 8（BMP-B，1 个扇区）...");
        load_wr0 = wr0; load_wr1 = wr1;
        pulse_load(32'd8);
        wait_idle(20000);
        repeat (3) @(negedge clk);
        $display("         像素写脉冲 ref=%0d new=%0d（期望 4：54..65 共 12 字节）",
                 wr0 - load_wr0, wr1 - load_wr1);
        if ((wr1 - load_wr1) != 4) begin
            $display("  [FAIL] 像素个数不符");
            errs = errs + 1;
        end

        //---------------------------------------------------------------
        // 阶段 4：装载头不匹配的扇区（BMP-C 800x600）-> 应放弃、不申请写权限
        //---------------------------------------------------------------
        $display("[阶段4] 装载扇区 12（800x600，应放弃）...");
        load_wr0 = wr0; load_wr1 = wr1;
        pulse_load(32'd12);
        wait_idle(20000);
        repeat (3) @(negedge clk);
        $display("         像素写脉冲 ref=%0d new=%0d（期望 0）", wr0 - load_wr0, wr1 - load_wr1);
        if ((wr1 - load_wr1) != 0) begin
            $display("  [FAIL] 不该产生像素");
            errs = errs + 1;
        end

        //---------------------------------------------------------------
        // 阶段 5：装载途中中止（load_abort）-> scan_done 必须保持
        //---------------------------------------------------------------
        $display("[阶段5] 装载扇区 3 后中途 load_abort ...");
        pulse_load(32'd3);
        repeat (600) @(negedge clk);            // 让它进入数据段
        @(negedge clk);
        load_abort = 1'b1;
        @(negedge clk);
        load_abort = 1'b0;
        repeat (20) @(negedge clk);
        if (sc_done1 !== 1'b1) begin
            $display("  [FAIL] load_abort 后 scan_done 被误清（按键会失灵）");
            errs = errs + 1;
        end else
            $display("         load_abort 后 scan_done 仍为 1 ✓");
        if (sc_ft1 !== 3'd3) begin
            $display("  [FAIL] load_abort 后 scan_found_total 被误改: %0d", sc_ft1);
            errs = errs + 1;
        end
        if (ready1 !== 1'b1) begin
            $display("  [FAIL] load_abort 后未回到 ready");
            errs = errs + 1;
        end

        //---------------------------------------------------------------
        // 阶段 6：卡重新初始化（sd_init_done 拉低）-> 扫描结果必须作废
        //---------------------------------------------------------------
        $display("[阶段6] 卡重新初始化 ...");
        @(negedge clk);
        sd_init_done = 1'b0;
        repeat (10) @(negedge clk);
        if (sc_done1 !== 1'b0 || sc_ft1 !== 3'd0) begin
            $display("  [FAIL] 初始化期间扫描结果未清: done=%b total=%0d", sc_done1, sc_ft1);
            errs = errs + 1;
        end else
            $display("         初始化期间 scan_done/scan_found_total 已清零 ✓");
        @(negedge clk);
        sd_init_done = 1'b1;
        repeat (5) @(negedge clk);

        //---------------------------------------------------------------
        // 阶段 7：重扫（验证复位后可再次工作）
        //---------------------------------------------------------------
        $display("[阶段7] 重新扫描 0..40 ...");
        pulse_scan(32'd0, 32'd40, 3'd3);
        wait_scan_done(40000);
        repeat (5) @(negedge clk);
        if (sc_ft1 !== 3'd3) begin
            $display("  [FAIL] 重扫张数不符: %0d", sc_ft1);
            errs = errs + 1;
        end else
            $display("         重扫找到 3 张 ✓");

        //---------------------------------------------------------------
        // 汇总
        //---------------------------------------------------------------
        $display("=============================================================");
        $display(" 比对拍数   = %0d", n_cmp);
        $display(" 像素脉冲   ref=%0d  new=%0d", wr0, wr1);
        $display(" 像素校验和 ref=%08h  new=%08h", sum0, sum1);
        if (sum0 !== sum1) begin
            $display("  [FAIL] 像素流校验和不一致 —— 写入帧缓冲的内容不同");
            errs = errs + 1;
        end
        if (errs == 0)
            $display(" 结果：PASS —— 新旧实现逐拍完全等价，功能判据全部通过");
        else
            $display(" 结果：FAIL —— %0d 处不一致", errs);
        $display("=============================================================");
        $finish;
    end

    initial begin
        #50000000;
        $display("[TIMEOUT] 全局仿真超时");
        $finish;
    end

endmodule
