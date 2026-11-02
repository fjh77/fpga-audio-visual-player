//=============================================================================
// 文件名  : video_curl.v
// 功能    : 页面卷曲（page curl）转场着色
//
//   SW2=1 的图片切换由本模块在 video_transition 之后着色：上沿卷起 +
//   纸背明暗 + 折边高光 + 下沿投影。SW2=0（上下推移）在这里恒为透传。
//
// 唯一权威
//   tools/curl_sim/gen_curl_hw.py 的 hw_render() 是逐像素的黄金模型。
//   本模块必须与它逐像素相等，容差 0。所有定点取整顺序与四舍五入位置
//   都照抄该函数；任何"看起来等价"的改写都可能差 1 级（规格 §8 的坑）。
//
// 源像素不做非线性变形
//   hw_render() 里 `src = old[y] if y < split else new[y]` —— 取的是源像素，
//   x/y 不经过卷曲变换。卷曲是"着色"（VS 式），不是"像素位移"（PS 式）。
//   新旧图的拼接由读通路分段完成（frame_fifo_read），进到本模块的 I_rgb
//   已经是拼好的源画面。所以这里只按当前扫描位置 x/y 算覆盖率与光照，
//   RGB 走一条纯延迟链用于对齐。曾误写成"按 y+lift 取源像素"，
//   那会变成完全不同的效果（且与黄金模型差满屏）。
//
// 常数表
//   `include 必须放在 module 内部：curl_lut.vh 里是 10 个 Verilog-2001
//   function，放到模块外 iverilog 直接报 "function declarations must be
//   contained within a module"。tools/curl_sim/verify_lut_vh.py 的 TB 也是
//   这个写法。路径与 SD/sdram.v 同构（TD 以 src/td_project 为工作目录
//   解析相对路径，sdram.v 里 "../user_source/hdl_source/include/..." 是
// 现成证据）。 禁止在本文件里另抄一份表
//
// 几何表为什么先进 RAM
//   逐列几何 YT/YB/CEN 每帧只算一次（640 列），存进 1 个 54bit x 640 的
//   单写单读 RAM；像素路径上只做 1 次读。§2.1 明确要求不要在像素路径上
//   做逐像素乘法（综合后 LUT 余量有限，放不下逐像素乘法器）。等厚直筒之后三列几何其实
//   逐列相同，仍走"建表 + 查表"是为了保留一条"改公式即可恢复弧形卷边"
//   的通路，且不引入新的流水分支（见下面建表段的注释）。
//
// 流水线（10 级，总延迟 10 拍）
//   t0   输入（x_cnt/y_cnt 与 I_rgb 同拍，几何 RAM 发起读）
//   t+1  s1  geom_q 有效：YT/YB/CEN、ycen        -> cov/ao_ok/dr_ok/gi/(ycen-CEN)
//   t+2  s2  cov_2 / d_2 / e_2 / ai_2 / gi_2
//   t+3  s3  g8_3 / tc8_3 / |ycen-CEN|*inv_r8
//   t+4  s4  ao_mul_4 / uu8_4 / iu_4
//   t+5  s5  shade_5 / bright_5
//   t+6  s6  c6（施加 shade）/ pm_6
//   t+7  s7  drop_mul（直接由 tc8 查 curl_dr，不再有 smoothstep 一级）
//   t+8  s8  纸背混合（④）
//   t+9  s9  折边高光（⑤）
//   t+10 输出寄存器（⑥ 下沿投影 + O_rgb/O_vs/O_de）
//   每级最多 1 个 9x9 乘法（s6 有 3 个并列），video_clk=25MHz（40ns）。
//
// 不用 signed
//   工程工具链对 signed 支持不好（PROJECT_RULES §4）。负数一律用
//   "加常数偏移" 或 "取符号位再取绝对值"：
//     · YT/YB/CEN 存进 RAM 前加 BIAS=32768，18 位无符号。
//       实测值域 YT[-28902,122880] YB[-17638,123568] CEN[-23270,122880]，
//       ⊕32768 后落在 [3866,156648]，18 位（<262144）够。
//     · 表里两个量加了同一个偏移，相减时偏移抵消 —— 所以 YT-ycen、
//       YB-ytop 这类差值天然就是真值，不需要再减偏移。
//     · 差值用 19 位二补码算，靠 bit18 判负；|差| < 2^18 已核实
//       （最大 151654）。
//
// 每个 act_sr 下标的由来
//   act_sr[k] 在时刻 t 的值是 act_now(t-1-k)。第 n 级（t = t0+n）要用
//   像素自己的 act，所以取 k = n-1。下文每个 gate 都标注了它的 t。
//=============================================================================

module video_curl #(
    parameter [11:0] WIDTH  = 12'd640,      // 一行有效像素数
    parameter [11:0] HEIGHT = 12'd480       // 一帧有效行数
)(
    input               I_clk,              // video_clk 25MHz
    input               I_rst,              // 高有效（rst_all）

    // ---- 视频（来自 video_transition 的输出）----
    input      [23:0]   I_rgb,
    input               I_vs,
    input               I_de,

    // ---- 转场控制 ----
    input               I_active,           // 1 = 卷曲转场进行中（含 ST_WAIT）
    input      [6:0]    I_prog,             // 0..69，每帧一档（整数档，用于半径/滚花）

    // 2026-10-20 新增：折线位置（8.8 定点，与 curl_s8() 同格式）
    //   原来本模块内部直接算 s8_r <= curl_s8(I_prog)，只能取整数档；
    //   现在由 video_transition 给出半档插值后的 s8（20 位补码），
    //   让卷带的上下沿每帧都动一点点，而不是每 2 帧跳一大步。
    //   画面接缝（frame_fifo_read 的分段地址）仍在整数档上 ——
    //   因为接缝只能落在 128 像素（=1/5 行）的整数倍上，半档做不到；
    //   而整行数 × 640 一定是 128 的整数倍，所以半档只要取整行中点即可，
    //   video_transition 里就是这样取整的。
    input      [19:0]   I_s8,               // 折线 y（8.8 定点，负值表示已卷出屏顶）

    // 2026-10-21 新增（CURL14，修"卷曲转场仍然闪屏"）
    //   卷边半径（13 位，与 curl_rad8() 同格式），同样由 video_transition
    //   给出半档插值后的值。原来本模块内部按 I_prog 的整数档取
    //   curl_rad8(bp_1) —— 见下面建表触发处的说明。
    input      [12:0]   I_rad8,             // 卷边半径（半档插值后）

    // ---- 视频输出（比输入晚 10 拍）----
    output reg [23:0]   O_rgb,
    output reg          O_vs,
    output reg          O_de
);

    //=========================================================================
    // 常数表（必须 include 在 module 内部，见文件头）
    //=========================================================================
    `include "../user_source/hdl_source/include/curl_lut.vh"

    localparam [17:0] BIAS18       = 18'd32768;   // 几何量偏移（把负数搬成无符号）
    localparam [8:0]  R_MIN_ACTIVE = 9'd90;       // 与黄金模型 R_MIN_ACTIVE 一致

    //-------------------------------------------------------------------------
    // 像素坐标计数（与 I_rgb 同拍）
    //-------------------------------------------------------------------------
    reg  [9:0] x_cnt;
    reg  [9:0] y_cnt;
    reg        vs_d, de_d;

    wire vs_fall = vs_d & ~I_vs;   // 每帧恰好一次（color_bar 的 vs 每帧 1->0->1）
    wire de_fall = de_d & ~I_de;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            x_cnt <= 10'd0;
            y_cnt <= 10'd0;
            vs_d  <= 1'b0;
            de_d  <= 1'b0;
        end else begin
            vs_d <= I_vs;
            de_d <= I_de;
            if (!I_de)
                x_cnt <= 10'd0;
            else if (x_cnt != (WIDTH[9:0] - 10'd1))
                x_cnt <= x_cnt + 10'd1;
            if (vs_fall)
                y_cnt <= 10'd0;
            else if (de_fall)
                y_cnt <= y_cnt + 10'd1;
        end
    end

    // 行号 -> 像素中心 Q8 行坐标（y*256+128，最大 122752，17 位够）
    wire [16:0] y_cen = {y_cnt, 8'd0} + 17'd128;

    //=========================================================================
    // 逐列几何：YT / YB / CEN  ->  RAM（54bit x 640）
    //
    //   写口：prog 或 s8 变化后用 642 拍重建整张表（CURL14：改为逐帧跟随）。垂直消隐在 vs 下降沿之后
    //         还有 36 行 = 28800 拍，建表只占 2.2%，余量充足。
    // 读口：像素路径每像素 1 次， 读口里没有 rst（否则退化成 LUT RAM）。
    //
    //   frame_geom() 逐字对应等厚直筒：
    //       yt  = s8 - 2*rad8
    //       yb  = curl_s8(prog)              <- 卷带下沿 = s，就压在接缝正上方
    //       cen = yt + rad8
    //   存表前一律 +BIAS18。
    //
    // 为什么不再乘 sin 抬升折边（原 yb = yt + 2*rad8）
    //   读通路（frame_fifo_read）只能整行二选一，接缝必然是水平直线
    //   split_row(s)。只要卷带下沿随列号摆动（摆幅 = 抬升量 ≤ 1.95r
    //   ≈ 42.9 行 ≈ 9% 屏高），就会出现一条斜的错位带：
    //       y ≥ split 但 cov > 0  -> 新图内容被画成纸卷（新图被旧卷盖住）
    //       y <  split 但 cov = 0  -> 旧图裸露在新图该出现的位置
    //   第 35 档实测 36,230 个错图像素、下沿与接缝最大错位 43.5 行 ——
    //   这就是用户报的"两张图片衔接处有缝隙 / 卷轴下方有缝"。
    //   下沿钉住接缝后，接缝永远被卷筒下沿压住，不可能露出裸切口。
    //=========================================================================
    reg [53:0] geom_mem [0:639];
    reg [53:0] geom_q;

    reg [9:0]  bx;              // 建表列号
    reg        build_run;
    reg [6:0]  bprog;           // 建表用的 prog
    reg [9:0]  ax1, ax2;        // 列号流水（2 级，与数据严格同级）
    reg        v1, v2;          // 有效流水
    reg [6:0]  bp_1;            // prog 流水
    reg [19:0] s8_r;
    reg [12:0] rad8_r;

    reg  [6:0]  prog_q;
    wire        prog_chg = (I_prog != prog_q);

    // CURL14：建表触发从"prog 变化"改成"发布的几何变化"
    //   原来只在 prog_chg 重建（CURL_FPS=2 时 = 每 2 帧一次），而 CURL11 的
    //   半档插值让 I_s8 每帧都变 ⇒ 隔一帧接缝（读通路 split）往前挪半档、
    //   卷边着色带却还停在上一档的位置，接缝处露出一条不被卷边遮住的新图
    //   内容，隔帧闪一下 = 用户报的"sw2=1 还是闪屏"。
    //   现在 I_s8（含半档）一变就重建，带宽几何与接缝严格同帧。
    //   建表 642 拍 ≈ 1 行，垂直消隐 36 行，余量充足。
    reg  [19:0] s8_q;
    wire        s8_chg = (I_s8 != s8_q);

    // 建表流水（第 1 拍给列号，第 4 拍写 RAM）
    wire [20:0] s8_ext  = {{1{s8_r[19]}}, s8_r};            // 20 位补码 -> 21 位
    wire [19:0] two_r   = {rad8_r, 1'b0};                   // 2*rad8
    wire [20:0] yb_tc   = s8_ext;                           // yb = s8
    wire [20:0] yt_tc   = s8_ext - {1'b0, two_r};           // yt = s8 - 2*rad8
    wire [20:0] cen_tc  = yt_tc + {8'd0, rad8_r};           // + rad8
    wire [17:0] yt_bw   = yt_tc[17:0]  + BIAS18;
    wire [17:0] yb_bw   = yb_tc[17:0]  + BIAS18;
    wire [17:0] cen_bw  = cen_tc[17:0] + BIAS18;
    wire [53:0] geom_wd = {cen_bw, yb_bw, yt_bw};

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            bx        <= 10'd0;
            build_run <= 1'b1;      // 复位后就建一次表（覆盖 prog=0 的情形）
            bprog     <= 7'd0;
        end else if (prog_chg || s8_chg) begin
            // prog / s8 每帧最多变一次（在 vs 边沿由 video_transition 发布）
            bx        <= 10'd0;
            build_run <= 1'b1;
            bprog     <= I_prog;
        end else if (build_run) begin
            if (bx == (WIDTH[9:0] - 10'd1))
                build_run <= 1'b0;
            else
                bx <= bx + 10'd1;
        end
    end

    // 列号有效值与 LUT 数据必须严格同级
    //   逐级跟踪（cycle n 表示第 n 个时钟上升沿之后）：
    //     cycle n   : bx = X
    //     cycle n+1 : ax1 = X, bp_1 = prog
    //     cycle n+2 : ax2 = X, s8_r/rad8_r/yb_r = f(prog)
    //                 geom_wd 组合 = f(s8_r, rad8_r, yb_r)  <- 就是 X 列的几何
    //     写口在 n+2 -> n+3 的边沿用 ax2(cycle n+2)=X 写入 geom_wd(cycle n+2)
    //   ⇒ 必须是2 级(ax1 -> ax2)，数据与地址都停在同一级。
    // 我一度写成 3 级（ax3），整张表整体右移一列，症状正是
    // "内容与逐列几何各差 1 像素"。这里是唯一正确的对齐点。
    //
    // 等厚直筒后三列几何完全相同，建表本身已无必要，但保留
    //   保留原因：① 读口 geom_q 是逐像素现查，整列一致就等价于常数，
    //   不必再引入一条"整帧常数"的旁路（那要多一个跨 3 级流水的对齐点，
    //   正是上面这条注释里出过错的地方）；② 后续若要恢复弧形卷边，
    //   只要把这里改成逐列公式，其余流水/对齐/时序全部不动。
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            ax1    <= 10'd0;
            v1     <= 1'b0;
            bp_1   <= 7'd0;
        end else begin
            ax1    <= bx;
            v1     <= build_run;
            bp_1   <= bprog;
        end
    end

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            s8_r   <= 20'd0;
            rad8_r <= 13'd0;
            ax2    <= 10'd0;
            v2     <= 1'b0;
        end else begin
            // CURL14：s8 与 rad8 都来自端口（都已含半档插值）
            //   建表触发点见上面的 s8_chg。
            s8_r   <= I_s8;
            s8_q   <= I_s8;
            rad8_r <= I_rad8;
            ax2    <= ax1;
            v2     <= v1;
        end
    end

    // 写口：只有这一句（不复位数组，PROJECT_RULES §5）
    always @(posedge I_clk) begin
        if (v2)
            geom_mem[ax2] <= geom_wd;
    end

    // 读口：只有这一句， 里面没有 rst
    always @(posedge I_clk) begin
        geom_q <= geom_mem[x_cnt];
    end

    //-------------------------------------------------------------------------
    // 每帧常数：active（rad8 >= 90）与 inv_r8
    //   与建表同源（同一个 I_prog），保证"像素看到的表"和 active 一致。
    //-------------------------------------------------------------------------
    reg        active_r;
    reg [17:0] inv_r8_r;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            prog_q   <= 7'd0;
            active_r <= 1'b0;
            inv_r8_r <= 18'd0;
        end else begin
            prog_q <= I_prog;
            if (prog_chg) begin
                active_r <= (curl_rad8(I_prog) >= R_MIN_ACTIVE);
                inv_r8_r <= curl_inv_r(I_prog);
            end
        end
    end

    // 转场关闭时（I_active=0）必须精确透传。prog=0 时 active_r 本来就是 0，
    // 这里是二重保险（也覆盖 I_active 比 prog 晚一拍落下的边界帧）。
    wire act_now = I_active & active_r;

    //=========================================================================
    // 延迟链（只做对齐，不做坐标变换 —— 见文件头）
    //   rgb_sr 6 槽 x 24 位；s6 在 t0+5 采样，需要 I_rgb(t0) 在槽 4。
    //   de_sr/vs_sr/act_sr 9 级；输出级取 [8]。
    //
    // 为什么是 [8] 而不是 [9]
    //   约定 sr[k](t) = in(t-1-k)（先移位再被读）。
    //   O_de 在 t 采样 sr[k](t-1) = in(t-2-k)，要等于 in(t-10) ⇒ k = 8。
    //   O_rgb 的延迟也是 10（rgb_sr[4](t)=I_rgb(t-5)，c6r(t)=I_rgb(t-6)，
    //   逐级到 O_rgb(t)=I_rgb(t-10)）。两者必须严格相等，否则整帧错位。
    // 早先取 [9] 时 O_de 比 O_rgb 多 1 拍，测试图整整错开 1 个像素，
    // 表现为"每个像素都差"，极易被误判成算术错。
    //=========================================================================
    reg [143:0] rgb_sr;
    reg [8:0]   de_sr, vs_sr, act_sr;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            rgb_sr <= 144'd0;
            de_sr  <= 9'd0;
            vs_sr  <= 9'd0;
            act_sr <= 9'd0;
        end else begin
            rgb_sr <= {rgb_sr[119:0], I_rgb};
            de_sr  <= {de_sr[7:0],  I_de};
            vs_sr  <= {vs_sr[7:0],  I_vs};
            act_sr <= {act_sr[7:0], act_now};
        end
    end

    //=========================================================================
    // s1（t0+1）：几何 RAM 读出 + 像素中心行坐标
    //=========================================================================
    reg [16:0] ycen_1;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            ycen_1 <= 17'd0;
        end else begin
            ycen_1 <= y_cen;
        end
    end

    wire [17:0] yt_1  = geom_q[17:0];       // YT + BIAS
    wire [17:0] yb_1  = geom_q[35:18];      // YB + BIAS
    wire [17:0] cen_1 = geom_q[53:36];      // CEN + BIAS

    wire [16:0] ytop_1 = ycen_1 - 17'd128;  // y<<8
    wire [16:0] ybot_1 = ycen_1 + 17'd128;  // (y+1)<<8

    //=========================================================================
    // ① 覆盖率 cov / ② AO 判据 / ③ 下沿判据 / ⑤ 高斯索引 / ④ 的分子
    //   全是加减+比较，没有乘法。差值用 19 位二补码，bit18=1 即负。
    //=========================================================================
    // a = clip(ybot - YT, 0, 256)
    wire [18:0] a_v = {1'b0, ybot_1} + 19'd32768 - {1'b0, yt_1};
    wire [8:0]  a_c = a_v[18] ? 9'd0 : (|a_v[17:8] ? 9'd256 : a_v[8:0]);
    // b = clip(YB - ytop, 0, 256)
    // 2026-10-20 CURL8：下沿覆盖改成二值
    // 面积覆盖在下沿是"卷带与新图连接处透亮"的机械成因：
    //   yb 是连续值，最后一行 cov 从 1 滑到 0，是部分覆盖行；
    //   而下沿投影从 ycen >= YB 才开始 ⇒ 恰好有 1 行"几乎全露新图 + 还没投影"。
    // 实测（浮点模型，prog=35，s=109.61，YB=109.61）：
    //   y=107 84.8 → y=108 93.9 → y=109 114.2 → y=110 92.2
    //   y=109 是带内唯一局部极大，比带内倒数第二行高 +20.3、比其下新图高 +22.0。
    // 换二值后（像素中心 ycen < YB 即全有），该亮线在纯色页/哨兵图上精确归零，
    // 真实素材上由 27.1 级降到 5.7 级（残余是素材自身纹理）。
    // 上沿 a_c 必须保持面积覆盖 ：折边高光 EDGE_HI 会被整块压进第一行，
    // 实测 topLM 6.8→10.0、topStep −9.6→−14.4，反而更刺眼。
    // 等价式推导：ytop_1 = ycen_1 - 17'd128，故原式 = clamp(YB - (ycen-128),0,256)
    //   的两端值（>0 即 256、<=0 即 0），二值化后正是 (ycen_1 < yb_1) ? 256 : 0；
    //   两者都带同一 BIAS，比较保序。
    // 换算回同一坐标系
    //   ycen_1 是未加 BIAS的 Q8 中心行坐标（0..122752），
    //   yb_1 是YB + 32768。直接比会因 BIAS 恒偏 32768 而几乎恒真
    //   （实测多发 640×128 像素、最大差 160 级）。先把 ycen 抬进 BIAS 域。
    // ycen_1 + 32768 ≤ 122752+32768 = 155520 < 2^18
    wire [17:0] ycen_bi = {1'b0, ycen_1} + 18'd32768;
    wire [8:0]  b_c = (ycen_bi < yb_1) ? 9'd256 : 9'd0;

    wire        cov_gt = (yb_1 > yt_1);                 // YB > YT（加同一偏移，保序）
    wire [8:0]  cov_x  = cov_gt ? ((a_c < b_c) ? a_c : b_c) : 9'd0;

    // d = YT - ycen ；d <= 0（含 0）时 ao_mul = 256
    wire [18:0] d_x   = {1'b0, yt_1} - ({1'b0, ycen_1} + 19'd32768);
    wire        ao_ok = d_x[18] | (d_x == 19'd0);

    // d2 = ycen - YB ；d2 < 0 时 drop_mul = 256
    wire [18:0] e_x   = ({1'b0, ycen_1} + 19'd32768) - {1'b0, yb_1};
    wire        dr_ok = ~e_x[18];                        // ycen >= YB -> 要算投影

    // gi = clip(((ycen - YT) + 640) >> 4, 0, 103)
    wire [18:0] g_0  = ({1'b0, ycen_1} + 19'd32768) - {1'b0, yt_1};
    wire [18:0] g_1  = g_0 + 19'd640;
    wire [6:0]  gi_x = g_1[18] ? 7'd0 :
                       ((g_1[18:4] >= 15'd104) ? 7'd103 : g_1[10:4]);

    // ④ 的分子：ycen - CEN
    wire [18:0] ai_x = ({1'b0, ycen_1} + 19'd32768) - {1'b0, cen_1};

    reg [8:0]  cov_2;
    reg        ao_ok_2, dr_ok_2;
    reg [18:0] d_2, e_2, ai_2;
    reg [6:0]  gi_2;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            cov_2 <= 9'd0; ao_ok_2 <= 1'b0; dr_ok_2 <= 1'b0;
            d_2 <= 19'd0; e_2 <= 19'd0; ai_2 <= 19'd0; gi_2 <= 7'd0;
        end else begin
            cov_2   <= cov_x;
            ao_ok_2 <= ao_ok;
            dr_ok_2 <= dr_ok;
            d_2     <= d_x;
            e_2     <= e_x;
            ai_2    <= ai_x;
            gi_2    <= gi_x;
        end
    end

    //=========================================================================
    // s2 -> s3（t0+3）：g8（d*26>>8）/ tc8（d2*585>>13）/ |ycen-CEN|*inv_r8
    //   26 = 16+8+2 ； 585 = 512+64+8+1
    // 2026-10-19：tc8 的乘数从 315 改成 585
    //   315 = round(8192/26)，是按26 行投影长度算的定点倒数。
    //   投影改成 14 行（短而浓）后必须同步为 585 = round(8192/14)，
    //   否则 dr 索引只走到一半就截断，剖面形状完全错（写死过一次）。
    //   e_2 最大 140518（ycen 122880 + |YB| 17638），585*e = 8.2e7 < 2^27。
    //   只有 d>0 才用 d26，只有 d2>=0 才用 e585，所以 d_2/e_2 在用时非负。
    //=========================================================================
    wire [21:0] d26   = ({3'd0, d_2} << 4) + ({3'd0, d_2} << 3) + ({3'd0, d_2} << 1);
    wire [13:0] d26hi = d26[21:8];
    wire [8:0]  g8_x  = (d26hi >= 14'd256) ? 9'd0 : (9'd256 - d26hi[8:0]);

    wire [27:0] e_e    = {9'd0, e_2};
    wire [27:0] e585   = (e_e << 9) + (e_e << 6) + (e_e << 3) + e_e;
    wire [27:0] e585r  = e585 + 28'd4096;
    wire [14:0] e585hi = e585r[27:13];
    wire [8:0]  tc8_x  = (e585hi >= 15'd256) ? 9'd256 : e585hi[8:0];

    // |ycen-CEN| <= 146150 < 2^18 ；inv_r8 <= 3144 < 2^12
    // 乘积 <= 4.60e8 < 2^29 ⇒ 18x12 乘法器就够（不用 19x18，省 LUT）
    wire [17:0] ai_abs = ai_2[18] ? ((~ai_2[17:0]) + 18'd1) : ai_2[17:0];

    reg [8:0]   g8_3, tc8_3;
    reg [28:0]  iu_p_3;
    reg         iu_neg_3, ao_ok_3, dr_ok_3;
    reg [8:0]   cov_3;
    reg [6:0]   gi_3;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            g8_3 <= 9'd0; tc8_3 <= 9'd0; iu_p_3 <= 29'd0;
            iu_neg_3 <= 1'b0; ao_ok_3 <= 1'b0; dr_ok_3 <= 1'b0;
            cov_3 <= 9'd0; gi_3 <= 7'd0;
        end else begin
            g8_3     <= g8_x;
            tc8_3    <= tc8_x;
            iu_p_3   <= ai_abs * inv_r8_r[11:0];   // 18 x 12 -> 30 位，真值 < 2^29
            iu_neg_3 <= ai_2[18];
            ao_ok_3  <= ao_ok_2;
            dr_ok_3  <= dr_ok_2;
            cov_3    <= cov_2;
            gi_3     <= gi_2;
        end
    end

    //=========================================================================
    // s3 -> s4（t0+4）：② ao_mul（AO 查表）
    //                 ④ iu = clip(128 + (((ycen-CEN)*inv_r8 + 32768) >> 16), 0, 256)
    //   32768 这个四舍五入常数不能省（规格 §8 的坑）。Python 的 >> 对负数是
    //   算术右移（floor），这里用 32 位二补码 + bit31 的手工算术右移还原。
    // 2026-10-19：tc8->uu8->uu2->st8 的 smoothstep 链已整段删除
    //   投影改为「接触线最浓、向外单调淡出」（见 s7 段注释），
    //   不再需要 uu8/uu2/st8 三级连乘，省 1 个 9x9 乘法器与 3 个 9 位寄存器。
    //=========================================================================
    wire [8:0] ao_mul_x = ao_ok_3 ? 9'd256 : (9'd256 - {1'b0, curl_ao(g8_3[8:2])});

    // 32 位二补码（真值 |x| < 2^29）
    wire [31:0] iu_s32 = iu_neg_3 ? ({3'b111, ~iu_p_3} + 32'd1) : {3'b000, iu_p_3};
    wire [31:0] iu_sh  = iu_s32 + 32'd32768;
    wire [16:0] iu_sr  = {iu_sh[31], iu_sh[31:16]};        // 算术 >>16
    wire [17:0] iu_s18 = {iu_sr[16], iu_sr} + 18'd128;
    wire [8:0]  iu_x   = iu_s18[17] ? 9'd0 : (|iu_s18[16:8] ? 9'd256 : iu_s18[8:0]);

    //=========================================================================
    // s3 -> s4（t0+4）：② ao_mul（AO 查表）
    //                 ④ iu = clip(128 + (((ycen-CEN)*inv_r8 + 32768) >> 16), 0, 256)
    //   32768 这个四舍五入常数不能省（规格 §8 的坑）。Python 的 >> 对负数是
    //   算术右移（floor），这里用 32 位二补码 + bit31 的手工算术右移还原。
    //=========================================================================
    reg [8:0] ao_mul_4, iu_4, tc8_4;
    reg       dr_ok_4;
    reg [8:0] cov_4;
    reg [6:0] gi_4;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            ao_mul_4 <= 9'd256; iu_4 <= 9'd0; tc8_4 <= 9'd0;
            dr_ok_4 <= 1'b0; cov_4 <= 9'd0; gi_4 <= 7'd0;
        end else begin
            ao_mul_4 <= ao_mul_x;
            iu_4     <= iu_x;
            tc8_4    <= tc8_3;
            dr_ok_4  <= dr_ok_3;
            cov_4    <= cov_3;
            gi_4     <= gi_3;
        end
    end

    //=========================================================================
    // ② shade = 256 - ((256-ao_mul)*(256-cov))>>8   （在 s4 组合，s5 采样）
    // ④ bright = curl_roll(iu)
    // ① 与 ② 的先后不能换：shade 用 cov，cov 必须已经算出来（规格 §2.2）
    // act_sr 下标：s5 在 t0+5 采样，act_sr[3] = act_now(t0)
    //=========================================================================
    wire [8:0]  om_ao   = 9'd256 - ao_mul_4;
    wire [8:0]  om_cov  = 9'd256 - cov_4;
    wire [17:0] shade_p = om_ao * om_cov;                 // 真值 <= 65536
    wire [8:0]  shade_x = act_sr[3] ? (9'd256 - shade_p[16:8]) : 9'd256;

    reg [8:0] shade_5, bright_5;
    reg       dr_ok_5;
    reg [8:0] cov_5, tc8_5;
    reg [6:0] gi_5;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            shade_5 <= 9'd256; bright_5 <= 9'd0;
            dr_ok_5 <= 1'b0;
            cov_5 <= 9'd0; tc8_5 <= 9'd0; gi_5 <= 7'd0;
        end else begin
            shade_5  <= shade_x;
            bright_5 <= curl_roll(iu_4);
            dr_ok_5  <= dr_ok_4;
            cov_5    <= cov_4;
            tc8_5    <= tc8_4;
            gi_5     <= gi_4;
        end
    end

    //=========================================================================
    // s5 -> s6（t0+6）：② 施加 shade（三通道）
    //                  ④ pm  = (bright*cov) >> 8
    //   源像素取 rgb_sr 槽 4（t0+5 时刻槽 4 = I_rgb(t0)）
    //=========================================================================
    wire [7:0]  s6r = rgb_sr[119:112];
    wire [7:0]  s6g = rgb_sr[111:104];
    wire [7:0]  s6b = rgb_sr[103:96];

    wire [16:0] c6r_p = s6r * shade_5;
    wire [16:0] c6g_p = s6g * shade_5;
    wire [16:0] c6b_p = s6b * shade_5;

    wire [17:0] pm_p  = bright_5 * cov_5;
    wire [7:0]  pm_x  = pm_p[15:8];                        // 真值 <= 256

    reg [7:0]  c6r, c6g, c6b, pm_6;
    reg        dr_ok_6, pdf_6;
    reg [8:0]  cov_6, tc8_6;
    reg [6:0]  gi_6;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            c6r <= 8'd0; c6g <= 8'd0; c6b <= 8'd0; pm_6 <= 8'd0;
            dr_ok_6 <= 1'b0; pdf_6 <= 1'b0;
            cov_6 <= 9'd0; tc8_6 <= 9'd0; gi_6 <= 7'd0;
        end else begin
            c6r     <= c6r_p[15:8];
            c6g     <= c6g_p[15:8];
            c6b     <= c6b_p[15:8];
            pm_6    <= pm_x;
            dr_ok_6 <= dr_ok_5;
            // ④ 的进入条件：active && cov > 0（t0+6 采样，act 用 act_sr[4]）
            pdf_6   <= act_sr[4] && (cov_5 != 9'd0);
            cov_6   <= cov_5;
            tc8_6   <= tc8_5;
            gi_6    <= gi_5;
        end
    end

    //=========================================================================
    // s6 -> s7（t0+7）：③ drop_mul = 256 - ((DROP_K*curl_dr + 128) >> 8)
    //   当前 DROP_K = 127（= DROP_STRENGTH 127/256 = 0.4961 * 256）。
    // 2026-10-19 改：投影在接触线 t2=0 处最浓、向外单调淡出
    //   旧实现是 256 - ((66*st8*curl_dr + 32768)>>16)，其中
    //   st8 = smoothstep(t2/0.18) 是渐入——接触线处强度为 0，
    //   要走到 t2≈0.18（≈4.7 行）才最浓。于是在紧贴纸边的一行留下
    //   「无阴影的亮行」，其下几行越来越暗，剖面出现局部极大 ——
    //   用户报的"卷轴下方露缝/漏光亮线"就是它（m06186 实测确认形态）。
    //   渐变当初是给弓形几何（yb 逐列摆动 42 行）压锯齿用的；
    //   等厚直筒后 yb ≡ s8 是常数，成因消失，渐入只剩副作用。
    //   物理上接触线也最浓：纸边贴着下页，缝隙最小、遮挡最多。
    //   66*256 = 16896 < 2^15，>>8 后 0..66 ⇒ 最浓压暗 66/256 = 25.8%
    //   （= DROP_STRENGTH），与浮点模型 render() 第⑤层逐点一致。
    // 2026-10-19 二次改（用户 m07362 定案 Q_deep）：强度 66 -> 20
    //   66/256 = 25.8% 太重。它把紧贴纸边的 26 行整体压暗四分之一，
    //   与深灰纸背叠加后形成一条"暗带"，用户读成"割裂"。
    // 2026-10-20 四次改（CURL8，配下沿二值）：强度 115 -> 127
    //   下沿覆盖率改成二值后，纸边不再被透光提亮，投影成为唯一能把
    //   "纸边下方第一行"压到与纸边齐平的手段，必需吃满可用范围。
    //   127 = 128-1 = (v<<7) - v，比 115 的五项移位加法更省。
    // 127*256 = 32512 < 2^15（这是 15 位 sd_66 的硬上限：
    //   DROP_K ≤ 127 ⇔ DROP_STRENGTH ≤ 0.4961，再大就溢出）
    //   gen_curl_hw.py 里有对应断言：DROP_K*Q8 < 32768。
    //   同时 DROP_POWER 1.6 -> 1.0：幂次 1.6 让近处太浅
    //   （t2=0.1 时只压 0.496*0.158 = 7.8%），线性后近处才有足够斜率；
    //   该变化已体现在重新生成的 curl_dr 表里，本行不变。
    // act_sr[5] = act_now(t0)（s7 在 t0+7 采样）
    //=========================================================================
    wire [8:0]  dr_ix   = 9'd256 - tc8_6;
    wire [8:0]  dr_v    = curl_dr(dr_ix[8:2]);
    wire [14:0] sd_66   = ({6'd0, dr_v} << 7) - {6'd0, dr_v};
    wire [14:0] sd_r    = sd_66 + 15'd128;
    wire [8:0]  dr_rem  = sd_r[14:8];
    wire [8:0]  dropm_x = dr_ok_6 ? (9'd256 - dr_rem) : 9'd256;

    reg [7:0] c7r, c7g, c7b, pm_7;
    reg [8:0] dropm_7;
    reg       drop_en_7, pdf_7;
    reg [6:0] gi_7;

    // cov_7 必须存在
    //   ④ 的 c*icov 里 icov 与 c7 必须是同一个像素。
    //   c7r 承载像素 t-7，而 cov_6 是像素 t-6（差 1 拍）。
    //   直接把 cov_6 送进 s8 会让纸背混合的覆盖率整体错一级 ——
    //   在折边附近（cov 从 0 跳到 256 的那几行）表现成一条错位的亮边。
    reg [8:0] cov_7;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            c7r <= 8'd0; c7g <= 8'd0; c7b <= 8'd0; pm_7 <= 8'd0;
            dropm_7 <= 9'd256; drop_en_7 <= 1'b0; pdf_7 <= 1'b0; gi_7 <= 7'd0;
            cov_7 <= 9'd0;
        end else begin
            c7r  <= c6r;
            c7g  <= c6g;
            c7b  <= c6b;
            pm_7 <= pm_6;
            cov_7   <= cov_6;
            dropm_7 <= dropm_x;
            // ⑥ 的进入条件：active && drop_mul != 256
            // 2026-10-19：这里原来多一个 `cov_6 == 0` 条件，已删除
            // 要求 cov 恰为 0 会漏掉跨过纸边的那一行：像素中心已在
            // 下沿之外（d2>0）但仍被纸背覆盖几十分之一，于是既不投影、
            // 又几乎全是新图 ⇒ 紧贴纸边多出一条"无阴影的亮行"。
            // 实测 prog=42（s=63.01 split=62）y=63：cov=3/256、d2=0.49 行，
            // 正确应压暗 25%（249→187），旧条件下完全不压 ⇒ 差 62 级。
            // 带内像素（cov==256）必然 d2<0 ⇒ drop_mul 恒 256，不会误伤，
            // 所以 `cov == 0` 本身多余且有害。
            drop_en_7 <= act_sr[5] && (dropm_x != 9'd256);
            pdf_7   <= pdf_6;
            gi_7    <= gi_6;
        end
    end

    //=========================================================================
    // s7 -> s8（t0+8）：④ 卷带纸背  c = (c*icov + ipb*pm) >> 8
    // 2026-10-19 改（用户 m07362 定案 Q_deep 哑光深灰纸）
    //   旧 PAPER_BACK = (232,226,214)：卷带中心冲到 220 级，是画面最亮的东西，
    //   观众读成"卷轴在发光/漏光"；且带内落差 148 级（220 -> 70）。
    //   新 PAPER_BACK = (170,165,155)：中心 152 级，带内落差 42 级，
    //   带底 109~130 与新页 125~147 基本接平。
    //   170 = 128+32+8+2 = (<<7)+(<<5)+(<<3)+(<<1)
    //   165 = 128+32+4+1 = (<<7)+(<<5)+(<<2)+pm
    //   155 = 128+16+8+2+1 = (<<7)+(<<4)+(<<3)+(<<1)+pm
    // 必须移位加法（规格 §3），不要引乘法器
    //
    // 2026-10-19 三次改：纸背混入透光（show-through）
    //   纯色纸背让卷带640 列完全同色（实测跨列相邻列差恒为 0.00，
    //   带内 std 只剩圆柱剖面带来的逐行变化）。真实纸张永远不会这样，
    //   眼睛立刻把它读成"贴上去的横条"—— 这就是用户反复说的「割裂感」，
    //   既不是几何错位，也不是亮度深浅，所以调了多少轮亮度都没用。
    //   物理上纸是半透的：背面印刷内容会透过来一点。卷带正盖在旧页上
    //   （split ≈ yb），所以把本像素底下那层内容 c7（已过 AO 着色）
    //   按 ST8/256 = 72/256 = 28% 混进来。
    // 顺序：先做纸背合成、再做透光混合
    //     p8_ch = (c7*icov + PAPER_ch*pm) >> 8      // 纸背（常量，移位加法）
    //     c8_ch = (p8_ch*(256-ST8) + c7*ST8) >> 8   // 透光
    //   为什么不把透光放进纸背那一项：那样 ipb 是变量，
    //   ipb*pm 就变成 8x9 变量乘法 —— 三个通道三个乘法器，白烧 DSP。
    //   放在后面两步都只是乘常数（72 = 64+8，184 = 128+32+16+8），全是移位。
    //   位宽核算：
    // 255*170 = 43350 < 2^16（17 位够，不变宽）
    // p8 <= 255，p8*184 <= 46920，c7*72 <= 18360，和 <= 65280 < 2^17
    //=========================================================================
    wire [8:0]  icov_8 = 9'd256 - cov_7;

    wire [16:0] p8r_m = c7r * icov_8;
    wire [16:0] p8g_m = c7g * icov_8;
    wire [16:0] p8b_m = c7b * icov_8;

    wire [16:0] pm_9  = {9'd0, pm_7};

    // 2026-10-20 CURL9：PAPER_BACK 158/152/142 → 150/100/56（牛皮纸）
    // 用户连续两次说「卷轴颜色还是太淡」。上一版 158/152/142 的亮度是 152.5，
    // 但 R-B 只有 16 —— 等于中性灰。展陈素材两页的 R-B 都只有 30（01 页
    // RGB 88/71/58，02 页 103/87/73），一整屏都是暖调，一堆灰自然被读成
    // "没有性格的灰雾"，调多少亮度都没用。真正的抓手是把 R>>G>>B 拉开：
    //   150/100/56 ⇒ 亮度 110，R-B = 94（两页的 2.5 倍）。
    // 实测带内 R-B 稳定在 71~75，与背景的可分辨度来自色相而不是明度。
    // 分解：150 = 128+16+4+2 ；100 = 64+32+4 ；56 = 32+16+8
    // 位宽：255*150 = 38250 < 2^16
    wire [16:0] pb_r = (pm_9 << 7) + (pm_9 << 4) + (pm_9 << 2) + (pm_9 << 1);
    wire [16:0] pb_g = (pm_9 << 6) + (pm_9 << 5) + (pm_9 << 2);
    wire [16:0] pb_b = (pm_9 << 5) + (pm_9 << 4) + (pm_9 << 3);

    wire [16:0] p8r_s = p8r_m + pb_r;
    wire [16:0] p8g_s = p8g_m + pb_g;
    wire [16:0] p8b_s = p8b_m + pb_b;

    // ---- 透光混合（在纸背合成之后，见上面注释）----
    //   c8 = (p8*224 + c7*32) >> 8        ← CURL8：ST8 72 → 32（=0.125）
    //   224 = 128+64+32 ； 32 = 32
    //   p8 <= 255 ⇒ 255*224 = 57120 ； c7 <= 255 ⇒ 255*32 = 8160
    //   和 <= 65280 < 2^16（16 位刚好，用 17 位留余量）
    // 为什么从 0.28 降到 0.125
    //   ① 带内亮度与底图内容的相关系数 corr 高达 0.25~0.54 ⇒ 带子读成"半透雾"
    //      （用户说的「卷轴过于淡」）；
    //   ② 内容接缝 split 几乎总落在卷带最下 1 行内，而新页普遍比旧页亮
    //      （02.BMP 行 60–199 均 141.7 vs 01.BMP 均 98.3）⇒ 透光把新页亮度
    //      渗进纸边，造出一条横向亮线（用户说的「连接处附近有透亮」）。
    //   0.125 时 corr 0.09、带内跨列 std 5.6，仍保留底影；
    //   完全关到 0 会让跨列 std 归零、读成死板的纯色横条。
    wire [7:0]  p8r = p8r_s[15:8];
    wire [7:0]  p8g = p8g_s[15:8];
    wire [7:0]  p8b = p8b_s[15:8];

    wire [16:0] st_r = ({10'd0, p8r} << 7) + ({10'd0, p8r} << 6) + ({10'd0, p8r} << 5)
                     + ({10'd0, c7r} << 5);
    wire [16:0] st_g = ({10'd0, p8g} << 7) + ({10'd0, p8g} << 6) + ({10'd0, p8g} << 5)
                     + ({10'd0, c7g} << 5);
    wire [16:0] st_b = ({10'd0, p8b} << 7) + ({10'd0, p8b} << 6) + ({10'd0, p8b} << 5)
                     + ({10'd0, c7b} << 5);

    reg [7:0] c8r, c8g, c8b;
    reg [8:0] dropm_8;
    reg       drop_en_8;
    reg [6:0] gi_8;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            c8r <= 8'd0; c8g <= 8'd0; c8b <= 8'd0;
            dropm_8 <= 9'd256; drop_en_8 <= 1'b0; gi_8 <= 7'd0;
        end else begin
            c8r <= pdf_7 ? st_r[15:8] : c7r;
            c8g <= pdf_7 ? st_g[15:8] : c7g;
            c8b <= pdf_7 ? st_b[15:8] : c7b;
            dropm_8   <= dropm_7;
            drop_en_8 <= drop_en_7;
            gi_8      <= gi_7;
        end
    end

    //=========================================================================
    // s8 -> s9（t0+9）：⑤ 折边高斯高光  c = (c*(256-hi) + EDGE_ch*hi) >> 8
    // EDGE_LIGHT = (252,249,242)； 整段必须被 active 门控（Python 里是
    // `if active:` 包住整块，hi==0 只是内层）
    //   252 = 256-4 ；249 = 256-8+1 ；242 = 256-16+2
    // act_sr[7] = act_now(t0)（s9 在 t0+9 采样）
    //=========================================================================
    wire [6:0]  hi_x  = curl_gauss(gi_8);
    wire        hi_on = act_sr[7] & (hi_x != 7'd0);
    wire [8:0]  ih_9  = 9'd256 - {2'd0, hi_x};

    wire [16:0] e9r_m = c8r * ih_9;
    wire [16:0] e9g_m = c8g * ih_9;
    wire [16:0] e9b_m = c8b * ih_9;

    wire [16:0] hi_9  = {10'd0, hi_x};

    wire [16:0] el_r = (hi_9 << 8) - (hi_9 << 2);
    wire [16:0] el_g = (hi_9 << 8) - (hi_9 << 3) + hi_9;
    wire [16:0] el_b = (hi_9 << 8) - (hi_9 << 4) + (hi_9 << 1);

    wire [16:0] e9r_s = e9r_m + el_r;
    wire [16:0] e9g_s = e9g_m + el_g;
    wire [16:0] e9b_s = e9b_m + el_b;

    reg [7:0] c9r, c9g, c9b;
    reg [8:0] dropm_9;
    reg       drop_en_9;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            c9r <= 8'd0; c9g <= 8'd0; c9b <= 8'd0;
            dropm_9 <= 9'd256; drop_en_9 <= 1'b0;
        end else begin
            c9r <= hi_on ? e9r_s[15:8] : c8r;
            c9g <= hi_on ? e9g_s[15:8] : c8g;
            c9b <= hi_on ? e9b_s[15:8] : c8b;
            dropm_9   <= dropm_8;
            drop_en_9 <= drop_en_8;
        end
    end

    //=========================================================================
    // s10（t0+10）：⑥ 下沿投影施加 c = (c*drop_mul) >> 8（仅带外）
    //   这一级的乘积直接进输出寄存器（不再多打一拍）
    //=========================================================================
    wire [16:0] d10r = c9r * dropm_9;
    wire [16:0] d10g = c9g * dropm_9;
    wire [16:0] d10b = c9b * dropm_9;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            O_rgb <= 24'd0;
            O_vs  <= 1'b0;
            O_de  <= 1'b0;
        end else begin
            O_rgb <= drop_en_9 ? {d10r[15:8], d10g[15:8], d10b[15:8]}
                               : {c9r, c9g, c9b};
            O_vs  <= vs_sr[8];
            O_de  <= de_sr[8];
        end
    end

endmodule
