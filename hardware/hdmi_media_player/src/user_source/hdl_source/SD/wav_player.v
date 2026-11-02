//=============================================================================
// 文件名  : wav_player.v
// 功能    : 从 TF 卡扫描并流式播放规范 WAV（展品语音讲解）
//
// 设计要点
// --------
// 1) 双时钟：
//      clk_sd    (100MHz) —— 扇区读取 FSM + 写缓冲
//      clk_audio ( 25MHz) —— 播放 FSM + 读缓冲
//    缓冲用推断的双时钟 BRAM做跨时钟域点（本工程 frame_fifo 已是同模式）。
//
// 2) 跨时钟域握手用toggle 握手而非多比特标志：
//      每个 toggle 位只有一个域的写者，天然无竞争、无死锁。
//      wr_tgl[i] : clk_sd 域，填满缓冲 i 后翻转
//      rd_tgl[i] : clk_audio 域，播完缓冲 i 后翻转
//      clk_sd   域：缓冲 i 可填 <=> wr_tgl[i] == rd_tgl_sync[i]
//      clk_audio域：缓冲 i 可播 <=> wr_tgl_sync[i] != rd_tgl[i]
//
// 3) 乒乓缓冲：一个扇区 512 字节 = 48kHz/16bit/单声道下 5.33ms 音频，
//    而读一个扇区约 0.47ms，余量 11 倍，预取绰绰有余。
//
// 4) 欠载（underrun）时保持最后一个样本并继续给 valid 脉冲，
//    保证 ACR 的 CTS 节拍不断（宁可重复几十微秒，也不能让音频时钟失锁）。
//
// WAV 格式要求（由 tools/make_tf_wav.py 归一化保证）
// -------------------------------------------------
//      偏移 0  : "RIFF"      偏移 8  : "WAVE"
//      偏移 12 : "fmt "      偏移 20 : PCM=1, 单声道=1
//      偏移 24 : 48000       偏移 34 : 16bit
//      偏移 36 : "data"      偏移 40 : 数据长度
//      偏移 44 : PCM 数据起点（固定）
//
// 状态码（送数码管显示，便于无逻辑分析仪时定位）
// ---------------------------------------------
//      5'd0 等 SD 初始化      5'd1 扫描中
//      5'd2 已找到,预取中      5'd3 播放中
//      5'd4 找到但不是规范格式 5'd5 扫描完没找到
//=============================================================================

module wav_player #(
    parameter [31:0] SCAN_START_SECTOR = 32'd0,
    parameter [31:0] SCAN_MAX_SECTOR   = 32'd131071,   // 扫描前 64MiB
    // 48000Hz @ 25MHz :  48000 * 2^32 / 25_000_000 = 8246337.2
    parameter [31:0] SAMPLE_INC        = 32'd8246337,
    // 循环播放开关 —— 这是当初为了从容调音量/观察亮度开的调试开关
    //   1 = 播完后自动从头再放一遍（调试用：不用反复复位就能听一遍又一遍）
    //   0 = 只播一遍（生产必须为 0）
    //
    // 为什么生产必须是 0（阶段 4 起更关键）
    //     阶段 4 的停留时长是"讲解播完 + 停顿 1 秒才切下一张图"。
    //     如果这里是 1，音频永远不会"播完"（它会一直循环下去），
    //     那个设计前提就不成立：你听到的是讲解被无限重复，
    //     而且"播完"这个事件永远不发生。
    //     所以这一位是阶段 4 的硬前提，不是可选项。
    // 单比特参数、顶层没有覆盖（只在本文件这一处定义），
    // 全工程只此一处引用（见下面 S_FILL 里的 LOOP_PLAY）
    parameter        LOOP_PLAY         = 1'b0
)(
    // ---- 时钟与复位 ----
    input             clk_sd,               // 100MHz，扇区读取域
    input             clk_audio,            //  25MHz，音频输出域
    input             rst,                  // 两个域共用（同步复位）

    // ---- 扇区读请求（接 SD 仲裁器） ----
    output reg        sec_read,
    output reg [31:0] sec_read_addr,
    input      [7:0]  sec_read_data,
    input             sec_read_data_valid,
    input             sec_read_end,
    input             sd_init_done,

    // ---- 扫描起点提示（来自图片扫描的结果） ----
    // 图片扫描结束时已经走过所有图片，而语音文件通常紧跟其后。
    // 从那个位置起扫可以几乎立刻命中，同时避免和图片抢 SPI——
    // 否则两边平分总线，图片显示会被拖慢近一倍。
    input      [31:0] scan_start,
    input             scan_go,       // 电平：图片扫描完成后置起

    // ---- 起播门控 ----
    // 取数侧照常预取（这样起播时缓冲已就绪，不会有欠载）；
    // 播放侧等 play_go 才真正出声，用来做"图片显示后延迟 1~2 秒再讲解"。
    // 在 play_go 之前会持续输出 valid + 静音，保持 ACR 的 CTS 节拍不断，
    // 否则接收端失锁，真正起播时可能被吞掉开头。
    input             play_go,

    // ---- 音频输出（clk_audio 域，48kHz） ----
    output reg        o_valid,
    output reg [23:0] o_left,
    output reg [23:0] o_right,

    // ---- 状态 ----
    output reg        o_found,
    output reg [4:0]  o_state,
    output            o_playing      // clk_audio 域：播放已开始（供状态码显示 3）
);

// 播放是否已经开始：一旦进入取样本流程就置 1 并保持。
// 之所以不在 o_state 里直接写 3：o_state 属 clk_sd 域，
// 而"是否在播"是 clk_audio 域的事实，跨域去改会引入同步问题。
reg play_started;
assign o_playing = play_started;

    //=====================================================================
    // 乒乓缓冲：2 x 512 字节，推断为双时钟 BRAM
    //=====================================================================
    // 2026-10-22 资源优化：两页乒乓合并为单个 1024x8 阵列
    //
    //   改前：2 个独立阵列 + "if (sel) buf1[] else buf0[]" 条件选阵列写/读。
    //         TD 6.2.1 只能推断成 2 块分布式 RAM，综合日志实测：
    //           SYN-1027 : Infer Logic DRAM(...wav_player_m0/ramread0_syn_4) read 512x8
    //           SYN-1027 : Infer Logic DRAM(...wav_player_m0/ramread1_syn_4) read 512x8
    //         合计约 128 个 LUT（每块 512x8 用 64 个 LUT6 拼）。
    //
    //   改后：单阵列 + {sel, addr} 高位选页 = 标准"单写口 + 单读口 + 双时钟"
    //         简单双口（SDP）模板，可稳定映射 1 块 BRAM9K；
    //         1024x8 = 8192bit 恰好填满一块 9Kb BRAM，利用率 100%。
    //
    //   语义完全等价：sel=0 -> 地址 0..511（原 buf0），sel=1 -> 512..1023（原 buf1）。
    //
    //   读写不会撞同一地址：写侧受 buf_fillable_0/1 门控（只写确实空闲的页，
    //   见下方 L332 起的注释），读侧 playing 指向正在播的另一页。
    //
    //   写法依据：top.v 中 sub_mask_ram 的实测坑记录 ①~④
    //   （读口不得带复位、不得复位数组、不得拆成多维 1bit 数组、属性会被静默忽略）。
    //
    //   验收判据：综合日志 SYN-1023 应从
    //     "Infer 2 Logic DRAMs, 3 Logic BRAMs"  ->  "Infer 0 Logic DRAMs, 4 Logic BRAMs"
    reg [7:0] pbuf [0:1023];

    // ---- clk_sd 域写端口 ----
    reg        wr_en;
    reg        wr_sel;              // 0 -> 低 512B 页(原 buf0), 1 -> 高 512B 页(原 buf1)
    reg [8:0]  wr_addr;
    reg [7:0]  wr_data;

    always @(posedge clk_sd) begin
        if (wr_en) begin
            pbuf[{wr_sel, wr_addr[8:0]}] <= wr_data;
        end
    end

    // ---- clk_audio 域读端口 ----
    reg        rd_en;
    reg        rd_sel;
    reg [8:0]  rd_addr;
    reg [7:0]  rd_data;

    always @(posedge clk_audio) begin
        if (rd_en) begin
            rd_data <= pbuf[{rd_sel, rd_addr[8:0]}];
        end
    end

    //=====================================================================
    // 跨时钟域 toggle 握手
    //=====================================================================
    reg  [1:0] wr_tgl;              // clk_sd 域写
    reg  [1:0] rd_tgl;              // clk_audio 域写
    reg  [1:0] wr_tgl_s1, wr_tgl_s2;// clk_sd -> clk_audio
    reg  [1:0] rd_tgl_s1, rd_tgl_s2;// clk_audio -> clk_sd

    always @(posedge clk_audio) begin
        wr_tgl_s1 <= wr_tgl;
        wr_tgl_s2 <= wr_tgl_s1;
    end

    always @(posedge clk_sd) begin
        rd_tgl_s1 <= rd_tgl;
        rd_tgl_s2 <= rd_tgl_s1;
    end

    // clk_sd 域判据：缓冲 i 可填
    wire buf_fillable_0 = (wr_tgl[0] == rd_tgl_s2[0]);
    wire buf_fillable_1 = (wr_tgl[1] == rd_tgl_s2[1]);
    // clk_audio 域判据：缓冲 i 可播
    wire buf_playable_0 = (wr_tgl_s2[0] != rd_tgl[0]);
    wire buf_playable_1 = (wr_tgl_s2[1] != rd_tgl[1]);

    //=====================================================================
    // clk_sd 域：扫描 + 预取 FSM
    //=====================================================================
    localparam [2:0] S_IDLE   = 3'd0,
                     S_SCAN   = 3'd1,   // 发请求 + 收 512 字节
                     S_CHK    = 3'd2,   // 扇区收完，判是否 WAV
                     S_FILL   = 3'd3,   // 预取后续扇区
                     S_ERRFMT = 3'd4,   // 找到但不是规范格式
                     S_NOTFND = 3'd5;

    reg [2:0]  sd_st;
    reg [31:0] cur_sector;
    reg [8:0]  rx_cnt;              // 本扇区已收字节数
    reg [7:0]  hdr [0:43];          // 前 44 字节头
    reg [5:0]  hdr_cnt;             // 已存头字节数
    reg [31:0] data_len;            // data 块长度
    reg [31:0] sectors_left;        // 还剩多少扇区要读
    reg        first_sector_ready;  // 头扇区已填好（决定播放起点）
    // ---- 循环播放 ----
    reg [31:0] wav_first_sector;    // WAV 头所在扇区，循环时回到这里
    reg [31:0] wav_total_sec;       // 一轮要读的扇区总数，循环时重装

    // 头字段判定
    wire hdr_is_riff = (hdr[0]==8'h52 && hdr[1]==8'h49 && hdr[2]==8'h46 && hdr[3]==8'h46);
    wire hdr_is_wave = (hdr[8]==8'h57 && hdr[9]==8'h41 && hdr[10]==8'h56 && hdr[11]==8'h45);
    wire hdr_is_fmt  = (hdr[12]==8'h66 && hdr[13]==8'h6D && hdr[14]==8'h74 && hdr[15]==8'h20);
    wire hdr_is_data = (hdr[36]==8'h64 && hdr[37]==8'h61 && hdr[38]==8'h74 && hdr[39]==8'h61);
    // 小端字段
    wire [15:0] hdr_af   = {hdr[21], hdr[20]};             // 编码，应为 1
    wire [15:0] hdr_ch   = {hdr[23], hdr[22]};             // 声道，应为 1
    wire [31:0] hdr_rate = {hdr[27],hdr[26],hdr[25],hdr[24]}; // 采样率，应为 48000
    wire [15:0] hdr_bits = {hdr[35], hdr[34]};             // 位深，应为 16
    wire [31:0] hdr_dlen = {hdr[43],hdr[42],hdr[41],hdr[40]}; // data 长度

    wire hdr_canonical = hdr_is_riff && hdr_is_wave && hdr_is_fmt && hdr_is_data &&
                         (hdr_af == 16'd1) && (hdr_ch == 16'd1) &&
                         (hdr_rate == 32'd48000) && (hdr_bits == 16'd16);

    always @(posedge clk_sd or posedge rst) begin
        if (rst) begin
            sd_st              <= S_IDLE;
            sec_read           <= 1'b0;
            sec_read_addr      <= 32'd0;
            cur_sector         <= SCAN_START_SECTOR;
            rx_cnt             <= 9'd0;
            hdr_cnt            <= 6'd0;
            data_len           <= 32'd0;
            sectors_left       <= 32'd0;
            wr_en              <= 1'b0;
            wr_sel             <= 1'b0;
            wr_addr            <= 9'd0;
            wr_data            <= 8'd0;
            wr_tgl             <= 2'b00;
            o_found            <= 1'b0;
            o_state            <= 5'd0;
            first_sector_ready <= 1'b0;
            wav_first_sector   <= 32'd0;
            wav_total_sec      <= 32'd0;
        end else begin
            wr_en <= 1'b0;

            case (sd_st)
            //------------------------------------------------- 等初始化
            S_IDLE: begin
                o_state <= 5'd0;
                // 只等 scan_go，不要加短超时
                //
                //   我第一版加了 10 秒兜底，结果它比要防的问题更早发生：
                //   图片扫描本身要 20 秒以上（数据区从扇区 32768 才开始，
                //   前面 3 万多个扇区全是保留区+FAT 表，扫到第一张图很慢），
                //   于是 10 秒超时必然先触发 -> 语音退回从扇区 0 开始扫 ->
                //   平白多走 4 万个扇区，起播晚了十几秒。
                //
                //   scan_done 由 bmp_read 保证会给出（它自己有 scan_max_sector 上界），
                //   所以这里安心等即可。宁可等，也不能抢跑。
                if (sd_init_done && scan_go) begin
                    // 图片扫描已走完所有图片，从它结束的位置起扫。
                    // 卡上语音文件紧跟在图片后面，通常几十个扇区内就命中。
                    cur_sector <= scan_start;
                    sd_st      <= S_SCAN;
                    o_state    <= 5'd1;
                end
            end
            //------------------------------------------ 扫描：发请求并收数据
            S_SCAN: begin
                o_state <= 5'd1;
                if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= cur_sector;
                    rx_cnt        <= 9'd0;
                    hdr_cnt       <= 6'd0;
                    wr_addr       <= 9'd0;
                end
                if (sec_read_data_valid) begin
                    // 同时写入乒乓缓冲与头捕获
                    wr_en   <= 1'b1;
                    wr_sel  <= 1'b0;            // 扫描阶段固定用 buf0
                    wr_addr <= rx_cnt;
                    wr_data <= sec_read_data;
                    if (hdr_cnt < 6'd44) begin
                        hdr[hdr_cnt] <= sec_read_data;
                        hdr_cnt      <= hdr_cnt + 6'd1;
                    end
                    rx_cnt <= rx_cnt + 9'd1;
                end
                if (sec_read_end) begin
                    sec_read <= 1'b0;
                    sd_st    <= S_CHK;
                end
            end
            //------------------------------------------------- 判定扇区
            S_CHK: begin
                if (hdr_canonical) begin
                    o_found            <= 1'b1;
                    data_len           <= hdr_dlen;
                    first_sector_ready <= 1'b1;
                    wr_tgl[0]          <= ~wr_tgl[0];   // buf0 已填好，交给播放侧
                    // 需要读的后续扇区数
                    // CURL22 修复：原来写 (hdr_dlen+511)>>9，把首扇区重复算了一遍
                    //     首扇区（S_SCAN 读的那个）里前 44 字节是规范头，
                    //     播放侧 play_ptr 从偏移 44 起播（见 :426），
                    //     所以首扇区已经贡献了 512-44 = 468 字节音频。
                    //     正确的"后续扇区数"= ceil((dlen-468)/512)：
                    //       01.wav  dlen=4,074,824  -> 7958（原式给 7960，多 2）
                    //       02.wav  dlen=3,582,190  -> 6996（原式给 6997，多 1）
                    //     多算的后果：fat32_file 按文件大小读完后 done，
                    //     多出来的 1~2 个请求永远等不到 sec_read_end，
                    //     wav_player 卡在 S_FILL（数码管 bit0 停在 2 = 预取中），
                    //     音频播完后播放器不回空闲态，也不再有数据补充。
                    sectors_left <= (hdr_dlen > 32'd468)
                                    ? ((hdr_dlen - 32'd468 + 32'd511) >> 9)
                                    : 32'd0;
                    // 记下循环起点和一轮总扇区数（LOOP_PLAY 用，公式与上面一致）
                    wav_first_sector <= cur_sector;
                    wav_total_sec    <= (hdr_dlen > 32'd468)
                                        ? ((hdr_dlen - 32'd468 + 32'd511) >> 9)
                                        : 32'd0;
                    sd_st        <= S_FILL;
                    o_state      <= 5'd2;
                end else if (hdr_is_riff && hdr_is_wave) begin
                    // 是 WAV 但不是我们要求的规范格式 —— 明确报错而不是静默跳过
                    sd_st   <= S_ERRFMT;
                    o_state <= 5'd4;
                end else if (cur_sector >= SCAN_MAX_SECTOR) begin
                    sd_st   <= S_NOTFND;
                    o_state <= 5'd5;
                end else begin
                    cur_sector <= cur_sector + 32'd1;
                    sd_st      <= S_SCAN;
                end
            end
            //------------------------------------------------- 持续预取
            S_FILL: begin
                o_state <= 5'd2;
                if (sectors_left == 32'd0) begin
                    // 数据已全部送进乒乓缓冲
                    sec_read <= 1'b0;
                    // 循环播放
                    //   不能"一读完就回头" —— 此刻播放侧还在放最后一块缓冲，
                    //   立刻重填正在播放的那块会把数据覆盖掉
                    //   （正是之前那个"噪声"坑）。
                    //   安全判据：目标缓冲 fillable（wr_tgl[i] == rd_tgl_sync[i]）。
                    // 用 || 而不是 &&
                    //     只要有一块空闲就立刻开始回填，另一块还在播，
                    //     回填正好利用这段时间 -> 循环几乎无缝。
                    //     若用 &&（等两块都空），播放侧已经静音了才回填，
                    //     会多出约 5ms 的断音。
                    if (LOOP_PLAY && (buf_fillable_0 || buf_fillable_1)) begin
                        cur_sector   <= wav_first_sector;
                        sectors_left <= wav_total_sec;
                    end
                end else if (!sec_read) begin
                    // 必须先确认目标缓冲确实空闲才发请求
                    //
                    //   我第一版写成 "if (buf_fillable_1 && !buf_fillable_0) sel=1;
                    //   else sel=0;" 并在后面无条件发请求。两个缓冲都满时
                    //   就会走 else 分支，把数据写进 buf0 —— 而 buf0 很可能
                    //   正在被播放，于是：
                    //     ① 播放中的数据被覆盖  -> 听到的是噪声
                    //     ② cur_sector/sectors_left 照常递减 -> 整个 WAV 被瞬间烧完
                    //     ③ sectors_left 归零后停止取数 -> 播放器永久欠载 -> 静音
                    //   正确做法：两个都不空就什么都不做，等播放侧释放。
                    if (buf_fillable_0) begin
                        wr_sel        <= 1'b0;
                        sec_read      <= 1'b1;
                        sec_read_addr <= cur_sector;
                        rx_cnt        <= 9'd0;
                        wr_addr       <= 9'd0;
                    end else if (buf_fillable_1) begin
                        wr_sel        <= 1'b1;
                        sec_read      <= 1'b1;
                        sec_read_addr <= cur_sector;
                        rx_cnt        <= 9'd0;
                        wr_addr       <= 9'd0;
                    end
                    // 两个缓冲都满：不发请求，下一拍再看
                end
                if (sec_read_data_valid) begin
                    wr_en   <= 1'b1;
                    wr_addr <= rx_cnt;
                    wr_data <= sec_read_data;
                    rx_cnt  <= rx_cnt + 9'd1;
                end
                if (sec_read_end) begin
                    sec_read     <= 1'b0;
                    cur_sector   <= cur_sector + 32'd1;
                    sectors_left <= sectors_left - 32'd1;
                    // 翻转对应缓冲的写 toggle，通知播放侧"这块好了"
                    if (wr_sel) wr_tgl[1] <= ~wr_tgl[1];
                    else        wr_tgl[0] <= ~wr_tgl[0];
                end
            end
            //------------------------------------------------- 异常终态
            S_ERRFMT: begin
                o_state  <= 5'd4;
                sec_read <= 1'b0;
            end
            S_NOTFND: begin
                o_state  <= 5'd5;
                sec_read <= 1'b0;
            end
            default: sd_st <= S_IDLE;
            endcase
        end
    end

    //=====================================================================
    // clk_audio 域：播放 FSM（48kHz）
    //
    // 关键：BRAM 读有 1 拍延迟，取一个 16bit 样本需要3 拍：
    //   拍1 发低字节读地址  ->  拍2 rd_data 是低字节，锁存并发高字节读地址
    //                        ->  拍3 rd_data 是高字节，与锁存的低字节拼成样本
    // 48kHz tick 间隔 520 个时钟，3 拍绰绰有余。
    //=====================================================================
    // 状态编码必须够宽：修正后需要 5 个状态（含 IDLE）
    //
    //   第一版只有 P_IDLE/P_LO/P_HI/P_OUT 四个状态，写出：
    //       P_LO  : 发低字节读地址
    //       P_HI  : sample_lo <= rd_data      <-- 错在这里！
    //       P_OUT : o_left <= {rd_data, sample_lo, 0}
    //   BRAM 是寄存器输出，t0 发的地址要到 t0+2 数据才有效；
    //   而 P_HI 在 t0+1 就锁存 rd_data，拿到的是上一次读的残留字节。
    //   结果 sample_lo 永远不是当前样本的低字节：
    //     对纯音扫频 -> 只是失真，听起来还像上升音（所以测试"通过了"）
    //     对语音     -> 低字节全错 -> 彻底变成噪音
    //   必须多插一级 P_LAT，让锁存发生在数据真正到位的那一拍。
    localparam [2:0] P_IDLE = 3'd0,   // 等下一个 tick
                     P_LO   = 3'd1,   // 发低字节读地址
                     P_HI   = 3'd2,   // 发高字节读地址
                     P_LAT  = 3'd3,   // 锁存低字节（这一拍 rd_data 正是低字节）
                     P_OUT  = 3'd4;   // rd_data 已是高字节，拼成样本输出

    // 33bit 加法，用 bit32 判回绕（32bit 相加会丢掉进位）
    wire [32:0] phase_next = {1'b0, phase} + {1'b0, SAMPLE_INC};
    wire        sample_tick = phase_next[32];

    reg [2:0]  pb_st;
    reg [7:0]  sample_lo;
    reg        playing;             // 正在播的缓冲号
    reg [8:0]  play_ptr;            // 缓冲内字节指针（偶数=低字节）
    reg [31:0] phase;               // 48kHz 小数分频累加器（回绕=出一个样本）

    // play_go 从 clk_sd 域同步到 clk_audio 域（2 级同步器）
    reg play_go_s1, play_go_s2;
    always @(posedge clk_audio or posedge rst) begin
        if (rst) begin
            play_go_s1 <= 1'b0;
            play_go_s2 <= 1'b0;
        end else begin
            play_go_s1 <= play_go;
            play_go_s2 <= play_go_s1;
        end
    end

    wire playable_now = playing ? buf_playable_1 : buf_playable_0;

    always @(posedge clk_audio or posedge rst) begin
        if (rst) begin
            phase        <= 32'd0;
            pb_st        <= P_IDLE;
            playing      <= 1'b0;
            play_ptr     <= 9'd44;      // 首块从偏移 44 开始（跳过 44 字节规范头）
            sample_lo    <= 8'd0;
            play_started <= 1'b0;
            rd_tgl       <= 2'b00;      // 显式复位，不依赖上电初值
            rd_en        <= 1'b0;
            rd_sel       <= 1'b0;
            rd_addr      <= 9'd0;
            o_valid      <= 1'b0;
            o_left       <= 24'd0;
            o_right      <= 24'd0;
        end else begin
            o_valid <= 1'b0;
            rd_en   <= 1'b0;
            phase   <= phase_next[31:0];

            // 关键：3 拍取样本序列必须用时钟每拍推进，不能每个 tick 才走一步。
            //
            //   我第一版写成 "if (sample_tick) case(pb_st) ..."，
            //   结果是：P_LO/P_HI/P_OUT 各要等一个 tick → 出一个样本花 3 个 tick
            //   → 实际输出速率 48000/3 = 16000 Hz。
            //   而 ACR 是数 I_sample_valid 脉冲推算 CTS 的，CTS 直接差 3 倍，
            //   HDMI 接收端解不出正确的音频采样时钟，于是整路静音。
            //
            //   正确做法：tick 只负责"启动"序列；序列本身 3 拍走完
            //   （3 拍远短于 520 拍的 tick 间隔，不会撞上下一个 tick）。
            case (pb_st)
            //----------------------------------------- 等下一个 tick
            P_IDLE: begin
                if (sample_tick) begin
                    if (play_go_s2 && playable_now) begin
                        play_started <= 1'b1;      // 一旦有块可播就标记"已开始"
                        pb_st        <= P_LO;
                    end else begin
                        // 未到起播时刻 / 欠载 / 播完：出静音，但保持 valid 脉冲，
                        // 否则 ACR 的 CTS 节拍会断，接收端可能失锁
                        o_left  <= 24'd0;
                        o_right <= 24'd0;
                        o_valid <= 1'b1;
                    end
                end
            end
            //----------------------------------------- 发低字节读
            P_LO: begin
                rd_en   <= 1'b1;
                rd_sel  <= playing;
                rd_addr <= play_ptr;
                pb_st   <= P_HI;
            end
            //----------------------------------------- 发高字节读
            // 本拍 rd_addr 仍是 play_ptr，所以本拍结束 BRAM 采到的是低字节；
            // 低字节数据将在下一拍(P_LAT)出现在 rd_data 上
            P_HI: begin
                rd_en   <= 1'b1;
                rd_sel  <= playing;
                rd_addr <= play_ptr + 9'd1;
                pb_st   <= P_LAT;
            end
            //------------------------------- 锁低字节（数据真正到位的那一拍）
            P_LAT: begin
                sample_lo <= rd_data;       // 此刻 rd_data 正是低字节
                pb_st     <= P_OUT;
            end
            //----------------------------------------- 输出样本
            P_OUT: begin
                // 16bit 有符号 -> 24bit 左对齐（低 8 位补 0）
                o_left  <= {rd_data, sample_lo, 8'd0};
                o_right <= {rd_data, sample_lo, 8'd0};
                o_valid <= 1'b1;

                if (play_ptr >= 9'd510) begin
                    // 本块播完：释放它，切到另一块
                    rd_tgl[playing] <= ~rd_tgl[playing];
                    playing  <= ~playing;
                    play_ptr <= 9'd0;
                end else begin
                    play_ptr <= play_ptr + 9'd2;
                end
                pb_st <= P_IDLE;        // 回到等下一个 tick
            end
            default: pb_st <= P_IDLE;
            endcase
        end
    end

endmodule
