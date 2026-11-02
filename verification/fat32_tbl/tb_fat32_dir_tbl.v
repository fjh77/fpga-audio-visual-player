//=============================================================================
// tb_fat32_dir_tbl.v
//
// 目的：验证 fat32_dir 的文件表容量从 16 降到 10 之后，
//       ① 8 个文件（4 BMP + 4 WAV，即实际卡上的用法）仍被完整解析、逐条可读回；
//       ② 表满时按原语义报 ERR_OVF，并把 o_count 饱和在最后一个可用下标；
//       ③ 少量文件（2 个）不误报错。
//
// 为什么要单独做这个仿真
//   表深度是 reg 数组的范围参数。改小之后若某个下标越界，综合器不会报错，
//   只会读到 X 或别的槽位 —— 上板表现为"文件个数对、内容错"，极难定位。
//   所以必须在改动落地的同一轮里用仿真把读回内容逐字节核一遍。
//
// 镜像构造（与 fat32_dir 的几何计算一致）
//   MBR   : LBA 0   分区起始 32、类型 0x0C、扇区数 10000、签名 55AA
//   DBR   : LBA 32  bps=512 spc=1 rsvd=1 nfat=1 root_ent=0
//                   fatsz16=0 fatsz32=64 root_clus=2 签名 55AA
//   几何  : fat_lba = 32 + 1 = 33
//           data_lba = 32 + 1 + 1*64 = 97
//           root_lba = 97 + (2-2)<<0 = 97
//   目录  : LBA 97   N 个 32 字节目录项 + 一条 00 结束项
//=============================================================================
`timescale 1ns/1ps

module tb_fat32_dir_tbl;

    //-------------------------------------------------------------------------
    // 时钟 / 复位
    //-------------------------------------------------------------------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;              // 100 MHz，模块本身对频率无要求

    //-------------------------------------------------------------------------
    // DUT 端口
    //-------------------------------------------------------------------------
    reg         start        = 1'b0;
    wire        busy;
    wire        done;
    wire [3:0]  err_code;

    wire [31:0] o_rd_lba;
    wire        o_rd_req;
    reg  [7:0]  i_data       = 8'h00;
    reg         i_data_valid = 1'b0;
    reg         i_data_last  = 1'b0;

    wire [4:0]  o_count;
    reg  [3:0]  i_tbl_idx    = 4'd0;
    wire [87:0] o_tbl_name;
    wire [31:0] o_tbl_cluster;
    wire [31:0] o_tbl_size;
    wire [7:0]  o_tbl_attr;

    reg         tgt_we       = 1'b0;
    reg  [87:0] tgt_name     = 88'd0;

    wire [31:0] o_fat_lba;
    wire [31:0] o_data_lba;
    wire [7:0]  o_sec_per_clus;
    wire [4:0]  o_spc_lg2;

    wire [31:0] dbg_part_lba;
    wire [31:0] dbg_data_lba;
    wire [31:0] dbg_root_lba;
    wire [31:0] dbg_fat_lba;
    wire [15:0] dbg_secs;
    wire [7:0]  dbg_part_type;
    wire [7:0]  dbg_root_b0;
    wire [7:0]  dbg_root_b1;
    wire [15:0] dbg_rx_bytes;

    // 表容量：默认 10（工程例化值）。用 iverilog -P 可覆盖成 16 做对照：
    //   iverilog -Ptb_fat32_dir_tbl.MAX_FILES=16 ...
    parameter integer MAX_FILES = 10;

    fat32_dir #(
        .MAX_FILES (MAX_FILES)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .busy           (busy),
        .done           (done),
        .err_code       (err_code),
        .o_rd_lba       (o_rd_lba),
        .o_rd_req       (o_rd_req),
        .i_data         (i_data),
        .i_data_valid   (i_data_valid),
        .i_data_last    (i_data_last),
        .o_count        (o_count),
        .i_tbl_idx      (i_tbl_idx),
        .o_tbl_name     (o_tbl_name),
        .o_tbl_cluster  (o_tbl_cluster),
        .o_tbl_size     (o_tbl_size),
        .o_tbl_attr     (o_tbl_attr),
        .tgt_we         (tgt_we),
        .tgt_name       (tgt_name),
        .o_fat_lba      (o_fat_lba),
        .o_data_lba     (o_data_lba),
        .o_sec_per_clus (o_sec_per_clus),
        .o_spc_lg2      (o_spc_lg2),
        .dbg_part_lba   (dbg_part_lba),
        .dbg_data_lba   (dbg_data_lba),
        .dbg_root_lba   (dbg_root_lba),
        .dbg_fat_lba    (dbg_fat_lba),
        .dbg_secs       (dbg_secs),
        .dbg_part_type  (dbg_part_type),
        .dbg_root_b0    (dbg_root_b0),
        .dbg_root_b1    (dbg_root_b1),
        .dbg_rx_bytes   (dbg_rx_bytes),
        .i_granted      (1'b1)          // 单元仿真：无仲裁器，恒授权
    );

    //-------------------------------------------------------------------------
    // 镜像几何与本轮放几个文件
    //-------------------------------------------------------------------------
    localparam [31:0] LBA_MBR  = 32'd0;
    localparam [31:0] LBA_DBR  = 32'd32;
    localparam [31:0] LBA_ROOT = 32'd97;

    integer n_files_cfg = 8;

    //-------------------------------------------------------------------------
    // 按 (LBA, 扇区内偏移) 组合生成一个字节 —— 不用数组，避免与 task 时序打架
    //-------------------------------------------------------------------------
    // 目录项 idx 的首簇（每条不同，便于核对）
    function [31:0] ent_clus;
        input integer idx;
        begin
            ent_clus = 32'h10 + idx;
        end
    endfunction

    // 目录项 idx 的文件大小（每条不同）
    function [31:0] ent_size;
        input integer idx;
        begin
            ent_size = idx * 100 + 54;
        end
    endfunction

    // 目录项 idx 的 11 字节名字（8 字节名 + 3 字节扩展名，空格填充）
    //   0~3 -> 0N.BMP   4~7 -> 0(N-4).WAV   8 以上 -> X(N-8).DAT
    function [87:0] ent_name;
        input integer idx;
        reg [7:0] n0, n1, n2, n8, n9, n10;
        begin
            n0 = 8'h30;                    // '0'
            n1 = 8'h30;
            n2 = 8'h20;
            n8 = 8'h20; n9 = 8'h20; n10 = 8'h20;
            if (idx < 4) begin
                n1 = 8'h30 + idx[7:0];
                n8 = 8'h42; n9 = 8'h4D; n10 = 8'h50;   // "BMP"
            end else if (idx < 8) begin
                n1 = 8'h30 + (idx - 4);
                n8 = 8'h57; n9 = 8'h41; n10 = 8'h56;   // "WAV"
            end else begin
                n0 = 8'h58;                            // 'X'
                n1 = 8'h30 + (idx - 8);
                n8 = 8'h44; n9 = 8'h41; n10 = 8'h54;   // "DAT"
            end
            ent_name = {n0, n1, n2, 8'h20, 8'h20, 8'h20,
                        8'h20, 8'h20, n8, n9, n10};
        end
    endfunction

    // 生成扇区字节
    //   off 为扇区内偏移 0~511；返回该位置的字节值
    function [7:0] sec_byte;
        input [31:0] lba;
        input integer off;
        integer ent;          // 目录项序号
        integer off_in_ent;
        reg [87:0] nm;
        reg [31:0] c_tmp;     // 本项首簇
        reg [31:0] s_tmp;     // 本项大小
        begin
            sec_byte = 8'h00;

            if (lba == LBA_MBR) begin
                // MBR：分区表项 + 签名
                if (off >= 450 && off <= 453) sec_byte = (off == 450) ? 8'h0C : 8'h00;
                else if (off >= 454 && off <= 457) sec_byte = (off == 454) ? 8'd32 : 8'h00;
                else if (off >= 458 && off <= 461) sec_byte = (off == 458) ? 8'h10 : 8'h27;  // 10000 = 0x2710
                else if (off == 510) sec_byte = 8'h55;
                else if (off == 511) sec_byte = 8'hAA;
            end
            else if (lba == LBA_DBR) begin
                if (off == 11) sec_byte = 8'h00;
                else if (off == 12) sec_byte = 8'h02;          // 512
                else if (off == 13) sec_byte = 8'h01;          // spc = 1
                else if (off == 14) sec_byte = 8'h01;          // rsvd = 1
                else if (off == 15) sec_byte = 8'h00;
                else if (off == 16) sec_byte = 8'h01;          // nfat = 1
                else if (off == 17) sec_byte = 8'h00;          // root_ent = 0
                else if (off == 18) sec_byte = 8'h00;
                else if (off == 28) sec_byte = 8'h00;          // fatsz16 = 0
                else if (off == 29) sec_byte = 8'h00;
                else if (off >= 36 && off <= 39) sec_byte = (off == 36) ? 8'd64 : 8'h00;  // fatsz32 = 64
                else if (off >= 44 && off <= 47) sec_byte = (off == 44) ? 8'h02 : 8'h00;  // root_clus = 2
                else if (off == 510) sec_byte = 8'h55;
                else if (off == 511) sec_byte = 8'hAA;
            end
            else if (lba == LBA_ROOT) begin
                ent = off / 32;
                if (ent < n_files_cfg) begin
                    off_in_ent = off % 32;
                    nm    = ent_name(ent);
                    c_tmp = ent_clus(ent);
                    s_tmp = ent_size(ent);
                    case (off_in_ent)
                        // 名字 11 字节（byte0 在最高 8 位）
                        0:  sec_byte = nm[87:80];
                        1:  sec_byte = nm[79:72];
                        2:  sec_byte = nm[71:64];
                        3:  sec_byte = nm[63:56];
                        4:  sec_byte = nm[55:48];
                        5:  sec_byte = nm[47:40];
                        6:  sec_byte = nm[39:32];
                        7:  sec_byte = nm[31:24];
                        8:  sec_byte = nm[23:16];
                        9:  sec_byte = nm[15:8];
                        10: sec_byte = nm[7:0];
                        11: sec_byte = 8'h20;                  // 属性：普通文件
                        20, 21: sec_byte = 8'h00;              // FstClusHI
                        // 注意：Verilog-2001 不允许对函数返回值做位选，
                        //       所以先落到临时变量 c_tmp / s_tmp 再取字节
                        26: sec_byte = c_tmp[7:0];              // FstClusLO
                        27: sec_byte = c_tmp[15:8];
                        28: sec_byte = s_tmp[7:0];              // 文件大小
                        29: sec_byte = s_tmp[15:8];
                        30: sec_byte = s_tmp[23:16];
                        31: sec_byte = s_tmp[31:24];
                        default: sec_byte = 8'h00;
                    endcase
                end else begin
                    sec_byte = 8'h00;                          // 结束项
                end
            end
        end
    endfunction

    //-------------------------------------------------------------------------
    // SD 卡从设备模型
    //   IDLE -> 见到 o_rd_req 锁存 LBA -> PREP 一拍 -> SEND 逐字节送 512 个
    //-------------------------------------------------------------------------
    localparam [1:0] M_IDLE = 2'd0, M_PREP = 2'd1, M_SEND = 2'd2;

    reg [1:0]  m_st      = M_IDLE;
    reg [31:0] m_lba     = 32'd0;
    integer    m_i       = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            m_st         <= M_IDLE;
            i_data_valid <= 1'b0;
            i_data       <= 8'h00;
            m_i          <= 0;
        end else begin
            case (m_st)
            M_IDLE: begin
                i_data_valid <= 1'b0;
                if (o_rd_req) begin
                    m_lba <= o_rd_lba;      // 本轮要送的扇区
                    m_i   <= 0;
                    m_st  <= M_PREP;
                end
            end
            M_PREP: begin                   // 空一拍，让 m_lba 稳定
                m_st <= M_SEND;
            end
            M_SEND: begin
                if (m_i < 512) begin
                    i_data       <= sec_byte(m_lba, m_i);
                    i_data_valid <= 1'b1;
                    m_i          <= m_i + 1;
                end else begin
                    i_data_valid <= 1'b0;
                    m_st         <= M_IDLE;
                end
            end
            default: m_st <= M_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 跑一次解析
    //-------------------------------------------------------------------------
    integer errs = 0;

    // 探针：err_code 只要出现过非 0 就打印，用来分辨"从未置起"与"置起后被覆盖"
    reg [3:0] err_dbg = 4'd0;
    reg       probe_en = 1'b0;       // 只在 TC2 打开，避免刷屏
    always @(posedge clk) begin
        if (rst_n && (err_code !== 8'hxx) && (err_code !== err_dbg)) begin
            $display("    [probe] err_code %0d -> %0d @ %0t", err_dbg, err_code, $time);
            err_dbg <= err_code;
        end
        if (!rst_n) err_dbg <= 4'd0;

        if (probe_en && rst_n && dut.rec_commit)
            $display("    [probe] commit: bcnt=%0d rec_idx=%0d pend=%b cnt=%0d",
                     dut.bcnt, dut.rec_idx, dut.pend_rec, dut.o_count);
    end

    task run_parse;
        input integer n;
        integer guard;
        begin
            n_files_cfg = n;
            // 模块约定：done 之后必须复位才能再次 start。
            //   所以每次跑之前先打一次复位，否则第二次 start 会被忽略，
            //   读回的仍是上一轮的结果（第一版 TB 就踩了这个，表现为
            //   "TC2/TC3 的 o_count 与 TC1 相同"）。
            rst_n = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            repeat (3) @(posedge clk);

            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            guard = 0;
            while ((done !== 1'b1) && (guard < 300000)) begin
                @(posedge clk);
                guard = guard + 1;
            end
            if (done !== 1'b1) begin
                $display("  [FAIL] 解析超时未完成（n=%0d）", n);
                errs = errs + 1;
            end
            repeat (6) @(posedge clk);
        end
    endtask

    // 读表并比对一条记录
    task chk_entry;
        input integer idx;
        begin
            i_tbl_idx = idx;         // 只用到低 4 位
            #1;
            if (o_tbl_name !== ent_name(idx)) begin
                $display("  [FAIL] idx=%0d 名字错: %02h%02h%02h%02h%02h%02h%02h%02h%02h%02h%02h",
                         idx,
                         o_tbl_name[87:80], o_tbl_name[79:72], o_tbl_name[71:64],
                         o_tbl_name[63:56], o_tbl_name[55:48], o_tbl_name[47:40],
                         o_tbl_name[39:32], o_tbl_name[31:24], o_tbl_name[23:16],
                         o_tbl_name[15:8],  o_tbl_name[7:0]);
                errs = errs + 1;
            end
            if (o_tbl_cluster !== ent_clus(idx)) begin
                $display("  [FAIL] idx=%0d 首簇错: 实际 %0d 期望 %0d",
                         idx, o_tbl_cluster, ent_clus(idx));
                errs = errs + 1;
            end
            if (o_tbl_size !== ent_size(idx)) begin
                $display("  [FAIL] idx=%0d 大小错: 实际 %0d 期望 %0d",
                         idx, o_tbl_size, ent_size(idx));
                errs = errs + 1;
            end
            if (o_tbl_attr !== 8'h20) begin
                $display("  [FAIL] idx=%0d 属性错: 实际 %02h", idx, o_tbl_attr);
                errs = errs + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    integer i;
    integer e0;

    initial begin
        $display("=============================================================");
        $display(" fat32_dir 文件表容量 = %0d 的验证", MAX_FILES);
        $display("=============================================================");

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        //---------------------------------------------------------------------
        // TC1：8 个文件（4 BMP + 4 WAV）—— 实际卡上的用法
        //---------------------------------------------------------------------
        $display("");
        $display("[TC1] 8 个文件（工程实际用法：4 BMP + 4 WAV）");
        run_parse(8);
        e0 = errs;
        if (err_code !== 4'd0) begin
            $display("  [FAIL] err_code = %0d，期望 0", err_code);
            errs = errs + 1;
        end
        if (o_count !== 5'd8) begin
            $display("  [FAIL] o_count = %0d，期望 8", o_count);
            errs = errs + 1;
        end else begin
            $display("  [OK] o_count = 8，err_code = 0");
        end
        for (i = 0; i < 8; i = i + 1) chk_entry(i);
        if (errs == e0) $display("  [OK] 索引 0~7 的名字/首簇/大小/属性全部匹配");

        //---------------------------------------------------------------------
        // TC2：11 个文件 —— 是否超出容量取决于 MAX_FILES
        //   容量语义：可收纳 MAX_FILES-1 个条目，再多报 ERR_OVF
        //---------------------------------------------------------------------
        $display("");
        if (11 > (MAX_FILES - 1)) begin
            $display("[TC2] 11 个文件 —— 超出容量(%0d)，应报 ERR_OVF", MAX_FILES);
            probe_en = 1'b1;
            run_parse(11);
            if (err_code !== 4'd9) begin
                $display("  [FAIL] err_code = %0d，期望 9 (ERR_OVF)", err_code);
                errs = errs + 1;
            end else begin
                $display("  [OK] err_code = 9 (ERR_OVF)，表满被正确检出");
            end
            if (o_count !== (MAX_FILES - 1)) begin
                $display("  [FAIL] o_count = %0d，期望饱和在 %0d（MAX_FILES-1）",
                         o_count, MAX_FILES - 1);
                errs = errs + 1;
            end else begin
                $display("  [OK] o_count 饱和在 %0d（MAX_FILES-1）", MAX_FILES - 1);
            end
            for (i = 0; i < (MAX_FILES - 1); i = i + 1) chk_entry(i);
        end else begin
            $display("[TC2] 11 个文件 —— 容量(%0d)足够，应正常解析", MAX_FILES);
            run_parse(11);
            if (err_code !== 4'd0) begin
                $display("  [FAIL] err_code = %0d，期望 0", err_code);
                errs = errs + 1;
            end
            if (o_count !== 5'd11) begin
                $display("  [FAIL] o_count = %0d，期望 11", o_count);
                errs = errs + 1;
            end else begin
                $display("  [OK] o_count = 11，err_code = 0（容量足够，不误报）");
            end
            for (i = 0; i < 11; i = i + 1) chk_entry(i);
        end

        //---------------------------------------------------------------------
        // TC3：2 个文件 —— 少量文件不应误报
        //---------------------------------------------------------------------
        $display("");
        $display("[TC3] 2 个文件 —— 不应误报");
        run_parse(2);
        if (err_code !== 4'd0) begin
            $display("  [FAIL] err_code = %0d，期望 0", err_code);
            errs = errs + 1;
        end
        if (o_count !== 5'd2) begin
            $display("  [FAIL] o_count = %0d，期望 2", o_count);
            errs = errs + 1;
        end else begin
            $display("  [OK] o_count = 2，err_code = 0");
        end
        for (i = 0; i < 2; i = i + 1) chk_entry(i);

        //---------------------------------------------------------------------
        $display("");
        $display("=============================================================");
        if (errs == 0)
            $display(" 结果：PASS —— 表容量 %0d 下解析与读回全部正确", MAX_FILES);
        else
            $display(" 结果：FAIL —— %0d 处不符", errs);
        $display("=============================================================");
        $finish;
    end

    initial begin
        #50000000;
        $display("  [FATAL] 仿真超时");
        $finish;
    end

endmodule
