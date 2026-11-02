//=============================================================================
// 文件名  : video_transition.v
// 功能    : 图片切换转场，SW2 选择
//             SW2 = 0 (mode 0) -> 上下推移旧图上移、新图从屏幕底部顶入（1.0 秒）
//             SW2 = 1 (mode 1) -> 页面卷曲从下往上卷，70 档 ≈ 1.176 秒
//
// mode 0 为什么不做遮罩（改成"推流"）
// -----------------------------------
//   上下推移的本质是同一屏上同时出现两张图：屏幕第 y 行要么是旧图第
//   (y+S) 行、要么是新图第 (y+S-480) 行。而 SDRAM 读通路一次只能线性流式
//   读一个缓冲，读出来的像素是按扫描顺序进 FIFO 的，视频侧没有行缓存去
//   重排 —— 所以"按行取两张图的局部"必须由读流的地址跳变来完成，
//   而不是靠像素运算。
//
//   本模块因此只负责：① 走推送时序（prog 0..240，S=2*prog 覆盖 0..480）；
//                      ② 把几何量导出去（O_push_on / O_split_pixels）。
//   真正的分段读发生在 frame_fifo_read.v：
//       段1：旧图基址 + S*640，长度 (480-S)*640
//       段2：新图基址，        长度 S*640
//   两段拼起来恰好是一帧，带宽不变。像素完全透传，
//   所以 mode 0 在这里不需要任何遮罩/乘法 —— 若仍然压暗，
//   画面会平白暗一大截并且和新图对不上。
//
// 为什么必须靠读通路分段、而不能重发 read_req 换基址
//     frame_fifo_read 的 S_ACK 分支里 fifo_aclr<=1'b1 会清空读 FIFO，
//     一帧内重发 read_req 必然花屏。分段只能利用 S_READ_BURST_END 里
//     "换基址后继续读"那条分支（它不动 fifo_aclr）。
//
// mode 1 = 页面卷曲（本次改动）
// ---------------------------
//   转场几何（每列 YT/YB/CEN）与逐像素着色全部搬到新模块 video_curl.v，
//   本模块只负责时序 + 进度发布：
//       prog 0..69，每帧一档（vs 下降沿），70 档 ≈ 1.176 s @59.5238Hz
//   与 push 同构：走满 CURL_LAST 就放行换图（O_hold 落 0），然后 ST_WAIT
// 等 pending 落下。 prog 在 ST_WAIT 期间不重置（重置会把卷曲表索引
//   打到 256 / 或让末帧退回首帧几何）。
//
//   原来的"右->左遮罩擦除"（ST_OUT/ST_IN 两相位 + side_k/wipe_pos/prof
//   整条压暗通路）已整体删除：卷曲不再需要遮罩，且那条通路每帧千次乘法
//   白占 LUT。
//
// 关键设计：转场期间拉住"换缓冲"
//   O_hold = 1 时 sd_card_bmp 不会切 disp_buf_idx。
//   两种模式都在最后一档、画面已经是新图的那一刻放行换图 ——
//   切换发生在看不见的时候，零视觉跳变。
//
// 与手动翻页的关系
// ----------------
// 2026-10-19 起：手动按 KEY_2 翻页也走转场，固定用 mode 0（上下推移）。
//     实现方式是顶层把 I_mode 预先掩码（手动 -> 0），本模块不需要知道
//     "这次是不是手动" —— 它拿到 mode 就照着做，所以本模块没有新端口。
//   I_skip 现在只剩一个含义：本次是第一张图，没有旧图可转，立即切换。
//     （sd_card_bmp 只在首图分支把它置 1）
//
// 时序（video_clk = 25MHz）
// ----------------------
//   mode 0：PUSH_STEP_CYC=103734，prog 0..240 共 241 档
//           241 x 103734 = 24,999,894 拍 ≈ 1.000 秒
//   mode 1：每 CURL_FPS 帧一档（vs 下降沿），prog 0..69 共 70 档
// CURL_FPS=2 -> 140 帧 / 59.5238Hz = 2.352 秒 当前
// 初版是每帧一档 = 1.176 秒，上板后用户反馈"太快、观感不明显"，
// 故每档停 2 帧。改 CURL_FPS 不改几何表（prog 仍只走 0..69）。
//
// 为什么帧同步用 vs_fall 而不是 vs_edge
//   color_bar 的 vs 每帧是 1 -> 0 -> 1，即每帧两个边沿。
//   用 vs_edge 做 step_tick 会每帧走两档 ⇒ 70 档 35 帧走完 ⇒ 只有 0.588s，
//   正好是要求的一半，而且不报任何错。
//   vs_fall = vs_d & ~I_vs 与 SD/video_timing_data.v:68 的
//   `video_vs_d0 & ~video_vs`（每帧发一次 read_req 的地方）完全同构。
//=============================================================================

module video_transition #(
    parameter [11:0] WIDTH  = 12'd640,          // 一行有效像素数（保留兼容）
    // ---- mode 0（上下推移）时序 ----
    // prog 0..240，S = 2*prog 走 0..480（480 = 整屏高度）
    // 241 x 103734 = 24,999,894 拍 ≈ 1.000 秒 @25MHz
    parameter [8:0]  PUSH_LAST     = 9'd240,
    parameter [16:0] PUSH_STEP_CYC = 17'd103734,
    // ---- mode 1（页面卷曲）时序 ----
    // 70 档（0..69）；每档停留 CURL_FPS 个显示帧：
    //   CURL_FPS=1 ->  70 帧 = 1.176 秒（初版，上板嫌太快）
    //   CURL_FPS=2 -> 140 帧 = 2.352 秒（当前）
    //   CURL_FPS=3 -> 210 帧 = 3.528 秒（若还嫌快，只改这一个参数）
    // CURL_LAST 必须与 curl_lut.vh 的 CURL_N(70) 一致
    // 加大 CURL_FPS 不动几何表：prog 仍只走 0..69，表仍按 prog 重建
    parameter [8:0]  CURL_LAST     = 9'd69,
    parameter [3:0]  CURL_FPS      = 4'd2
)(
    input               I_clk,          // video_clk 25MHz
    input               I_rst,

    // ---- 视频 ----
    input      [23:0]   I_rgb,
    input               I_vs,
    input               I_de,
    output reg [23:0]   O_rgb,
    output reg          O_vs,
    output reg          O_de,

    // ---- 控制 ----
    input               I_mode,         // SW2: 0 = 上下推移, 1 = 页面卷曲
    input               I_pending,      // 新图已写入缓冲，等待切换
    input               I_skip,         // 1 = 本次是第一张图，没有旧图可转，立即切换
    input               I_disp_valid,   // 1 = 屏幕已有图（第一张图不该做转场）
    output reg          O_hold,         // 1 = 拉住 sd_card_bmp 不要切缓冲
    output              O_busy,         // 1 = 转场进行中
    // 换图许可 电平有效，只有本模块明确放行时才置 1。
    output reg          O_go,

    // ---- 推送转场几何导出（到 top，再进 frame_read_write）----
    // 已按帧量化（在 vs 下降沿更新），详见下方注释
    output              O_push_on,      // 1 = 推送转场进行中（读通路要分段）
    output     [18:0]   O_split_pixels, // push: prog*1280 = 旧图起源偏移（像素）

    // ---- 页面卷曲几何导出（到 top，再进 frame_read_write）----
    output              O_curl_on,      // 1 = 卷曲转场进行中（读通路要分段）
    output     [18:0]   O_curl_split_px,// curl: split*640 = 段1（旧图）像素数
    output     [6:0]    O_curl_prog,    // curl: 当前帧的档位 0..69（给 video_curl）
    output     [19:0]   O_curl_s8,      // curl: 折线 y（8.8 定点，半档插值后）
    output     [12:0]   O_curl_rad8,    // curl: 卷边半径（半档插值后，CURL14）

    // ---- 转场期间要用的「锁存后的显示缓冲索引」----
    input      [1:0]    I_disp_idx,     // 实时 disp_buf_idx（来自 sd_card_bmp）
    output reg [1:0]    O_disp_idx_lat  // 转场第一帧 vs 下降沿锁存值，整个转场期不变
);

    //=========================================================================
    // 卷曲常数表（只要 curl_split 一个 function，其余会在 elaborate 时被丢）
    //
    //   split 必须和 video_curl 的几何出自同一份表，否则读通路的切点
    //   和着色用的折边位置会错位。禁止在这里另抄一份 70 项常数。
    //   include 必须放在 module 内部：curl_lut.vh 是 Verilog-2001 function，
    //   放到模块外 iverilog 会报 "function declarations must be contained
    //   within a module"。路径写法与 SD/sdram.v 一致（TD 以 src/td_project
    //   为工作目录解析相对路径）。
    //=========================================================================
    `include "../user_source/hdl_source/include/curl_lut.vh"


    //-------------------------------------------------------------------------
    // x 坐标计数 / 帧边沿
    //-------------------------------------------------------------------------
    reg  [11:0] x_cnt;
    reg         vs_d, de_d;

    wire vs_edge = (I_vs != vs_d);      // 上升+下降都算（x_cnt 清零用，多次无害）
    wire vs_fall = vs_d & ~I_vs;        // 每帧恰好一次
    wire de_fall = de_d & ~I_de;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            x_cnt <= 12'd0;
            vs_d  <= 1'b0;
            de_d  <= 1'b0;
        end else begin
            vs_d <= I_vs;
            de_d <= I_de;
            if (vs_edge)
                x_cnt <= 12'd0;
            else if (!I_de)
                x_cnt <= 12'd0;
            else
                x_cnt <= x_cnt + 12'd1;
        end
    end

    //-------------------------------------------------------------------------
    // 转场状态机
    //   ST_IDLE : 正常显示
    //   ST_PUSH : mode 0上下推移，prog 0->240，本机不做任何像素运算
    //   ST_CURL : mode 1页面卷曲，prog 0->69，每帧一档
    //   ST_WAIT : 已放行换图，等 pending 落下（确认切完了）
    //-------------------------------------------------------------------------
    localparam [2:0] ST_IDLE = 3'd0, ST_WAIT = 3'd2, ST_PUSH = 3'd4,
                     ST_CURL = 3'd5;

    reg [2:0]  st;
    reg [16:0] cyc;             // 相位内时钟计数（只有 push 用）
    reg [8:0]  prog;            // 进度：push 0~240；curl 0~69（复用同一个计数器）
    reg [3:0]  chold;           // curl 档内帧计数：每档停 CURL_FPS 帧

    // 转场启动那一拍锁存 I_mode。
    // 用户可能在转场中途拨 SW2；若数据通路直接看 I_mode，状态机会错乱。
    //   锁存后整个转场用 mode_lat。
    reg        mode_lat;

    // 转场进行中：任何非 IDLE 状态都算。
    // 必须放在 st 声明之后，否则是前向引用，TD 会报错
    assign O_busy = (st != ST_IDLE);

    // I_mode=0 -> 推送（上下推移）；I_mode=1 -> 卷曲
    wire push_mode = ~mode_lat;

    // 两种转场的"进行中"都含 ST_WAIT
    //   最后一档时全屏已经是新图；此时若立刻退回"按 disp_buf_idx 读整帧"，
    //   而提交还没生效（disp 仍是旧缓冲），会闪 1~2 帧旧图。继续拉住
    //   O_push_on / O_curl_on 让读流与几何恒为"末档"，与 disp 何时提交无关。
    wire push_active  = push_mode && (st == ST_PUSH || st == ST_WAIT);
    wire curl_active  = (~push_mode) && (st == ST_CURL || st == ST_WAIT);
    wire trans_active = push_active | curl_active;

    // push：103734 拍/档（241 档 ≈ 1.0s）
    // curl：每帧一档（vs 下降沿），70 档 ≈ 1.176s
    wire step_tick_push = (cyc == PUSH_STEP_CYC);
    wire step_tick_curl = vs_fall;
    wire step_tick      = push_mode ? step_tick_push : step_tick_curl;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            st    <= ST_IDLE;
            cyc   <= 17'd0;
            prog  <= 9'd0;
            chold <= 4'd0;
            mode_lat <= 1'b0;
            O_hold <= 1'b0;
        end else begin
            case (st)
            //------------------------------------------------ 空闲
            ST_IDLE: begin
                O_hold <= 1'b0;
                cyc    <= 17'd0;
                prog   <= 9'd0;
                chold  <= 4'd0;
                if (I_pending) begin
                    if (I_skip || !I_disp_valid) begin
                        // 第一张图：不遮罩、不拉 hold，立即切换
                        st <= ST_IDLE;
                    end else begin
                        O_hold   <= 1'b1;   // 拉住换缓冲
                        mode_lat <= I_mode; // 锁存本次转场类型
                        prog     <= 9'd0;
                        cyc      <= 17'd0;
                        st       <= I_mode ? ST_CURL : ST_PUSH;
                    end
                end
            end
            //------------------------------------------------ mode 0：上下推移
            //   本状态只走时序，不做任何像素运算。
            //   真正的新旧图拼接由 frame_fifo_read 的分段读完成。
            ST_PUSH: begin
                O_hold <= 1'b1;
                if (step_tick_push) begin
                    cyc <= 17'd0;
                    if (prog == PUSH_LAST) begin
                        // 走满 240 档（S=480，旧图已完全推出屏幕）才放行换缓冲
                        O_hold <= 1'b0;
                        st     <= ST_WAIT;
                    end else begin
                        prog <= prog + 9'd1;
                    end
                end else begin
                    cyc <= cyc + 17'd1;
                end
            end
            //------------------------------------------------ mode 1：页面卷曲
            //   与 ST_PUSH 同构，只是步进改成"每 CURL_FPS 帧一档"（vs 下降沿）。
            //   cyc 不用，恒 0；档内帧数由 chold 记（4 位，CURL_FPS<=15）。
            ST_CURL: begin
                O_hold <= 1'b1;
                cyc    <= 17'd0;
                if (step_tick_curl) begin
                    // 每档停 CURL_FPS 帧：不是每个 vs 下降沿都推进 prog
                    //   档内重复发同一个 prog，几何表不重建（prog_chg 只看值是否变），
                    //   所以加长只花时间，不花资源。
                    if (chold != (CURL_FPS - 4'd1)) begin
                        chold <= chold + 4'd1;
                    end else begin
                        chold <= 4'd0;
                        if (prog == CURL_LAST) begin
                            // 末档：全屏已是新图（split=0），放行换缓冲
                            O_hold <= 1'b0;
                            st     <= ST_WAIT;
                        end else begin
                            prog <= prog + 9'd1;
                        end
                    end
                end
            end
            //------------------------------------------------ 等换图完成
            ST_WAIT: begin
                O_hold <= 1'b0;
                cyc    <= 17'd0;
                // 两种模式都不要动 prog
                //   push：prog 必须保持 240，O_split_pixels 继续输出 307200，
                //         读流恒为"整帧新图"。
                //   curl：prog 必须保持 69，末档几何（split=0）保持不变，
                //         否则会退回首帧几何闪一下旧图。
                if (!I_pending) begin       // pending 落下 = 确实切完了
                    st <= ST_IDLE;
                end
            end
            default: st <= ST_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 换图许可 O_go（电平，保持到 pending 落下为止）
    //   第一张图（I_skip=1） ：立刻放行（没有旧图可转）
    //   自动轮播 / 手动翻页   ：等末档、进入 ST_WAIT 才放行
    //   其余时刻            ：0（sd_card_bmp 必须等）
    //-------------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            O_go <= 1'b0;
        else if (st == ST_WAIT)
            O_go <= 1'b1;
        else if ((st == ST_IDLE) && I_pending && (I_skip || !I_disp_valid))
            O_go <= 1'b1;
        else if (!I_pending)
            O_go <= 1'b0;
    end

    //-------------------------------------------------------------------------
    // 像素通路： 完全透传
    //
    //   mode 0（推送）的拼接在读通路做完，这里乘任何系数都只会把画面压暗；
    //   mode 1（卷曲）的着色在 video_curl.v 里做。
    //   所以本模块现在只剩 1 拍的 vs/de/rgb 对齐寄存，原来的
    //   "in * factor / 256 + 饱和" 整条通路（含 ease/smoothstep、边缘剖面
    //   prof、side_k/wipe_pos 几何）已整体删除 —— 每帧上千次乘法白占 LUT，
    //   且 mode 1 已不再使用遮罩。
    //-------------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            O_rgb <= 24'd0;
            O_vs  <= 1'b0;
            O_de  <= 1'b0;
        end else begin
            O_rgb <= I_rgb;
            O_vs  <= I_vs;
            O_de  <= I_de;
        end
    end

    //-------------------------------------------------------------------------
    // 转场几何导出：按帧量化
    //
    // 为什么必须量化
    //   O_push_on / O_split_pixels / O_curl_* 要进 frame_fifo_read
    //   （mem_clk 125MHz 域）。若直接送组合值，prog 在 step_tick 那拍会更新，
    //   split 值可能出现多位同时翻转（例如 prog 63->64：80640 -> 81920）。
    //   mem 侧在 S_ACK 采样的瞬间若正好撞上这次翻转，多位混合会得到一个
    //   完全非法的拆分点（既不是旧值也不是新值）→ 那一帧花屏。
    //   概率极低，但展馆机要连续跑几小时，必须掐掉。
    //
    //   做法：只在 vs_fall 那一拍发布新值。
    //   vs_fall 同时是 video_timing_data 产生 read_req 的时刻；mem 侧要到
    //   S_ACK 才采样这些信号，中间还有 3 级同步（read_req_d0/d1/d2）
    //   加 2 拍状态机，至少 >=48ns，那时总线早已稳定。
    //
    //   代价：转场开始/结束会延后到下一帧才生效 —— 这是设计意图
    //   （帧内决不改变读流），不是 bug。
    //
    // 位置要求 本块必须写在 vs_fall / prog / trans_active 之后，
    //   否则是前向引用，TD 会报错。O_hold 不进这个量化（它是跨域两级
    //   同步的慢速电平，原有机制不动）。
    //-------------------------------------------------------------------------
    reg        push_on_r;
    reg        curl_on_r;
    reg [18:0] split_px_r;          // push: prog*1280
    reg [18:0] curl_split_px_r;     // curl: split*640
    reg [6:0]  curl_prog_r;         // curl: 本帧档位
    reg [19:0] curl_s8_r;           // curl: 折线 y（8.8 定点，含半档插值）
    reg [12:0] curl_rad8_r;         // curl: 卷边半径（含半档插值，CURL14）

    wire trans_on_r = push_on_r | curl_on_r;

    // prog*1280 = (prog<<10) + (prog<<8)，纯移位加法，不用乘法器
    wire [18:0] split_px_now = ({10'd0, prog} << 10) + ({10'd0, prog} << 8);

    // 卷曲发布哪一档
    //   vs_fall 这一拍发布的值，作用于紧接着这一帧的整帧扫描
    //   （几何建表在行消隐里完成，见 video_curl.v）。
    //   ST_CURL 里 prog 是在同一拍才 +1，所以这里必须发加之前的值：
    //     第 1 个 vs_fall 发 0，第 2 个发 1 …… 第 70 个发 69
    //   共 70 帧，正好 70/59.5238Hz = 1.176 秒，与黄金模型 CURL_N=70 对齐。
    // 早先写成 prog+1（想"发下一帧的值"）会让首帧从 1 起步、
    // 并把末档连发两帧 ⇒ 实际 71 帧且首帧跳过 —— 已改回 prog
    //
    // 2026-10-20 新增：半档插值（修"sw2=1 转场闪屏"）
    //   病因（用户反馈）：CURL_FPS=2 时几何每2帧才动一次，
    //   而且一次要跳 2~18 整行；接缝/卷带/高光带在 33ms 内整块位移一大段，
    //   观感就是一闪一闪。总时长不能变（用户要求仍是 2.35s），
    //   所以把每个整数档再对半劈开：偶数帧取整数档，奇数帧取相邻两档的中点。
    //
    // 为什么中点只能取"整行中点"
    //     读通路的分段长度必须落在 128 像素（=1/5 行）的整数倍上，
    //     frame_fifo_read 靠 "read_cnt == read_split_len_latch" 精确相等来换基址，
    //     非整数倍会永远等不到相等 ⇒ 整帧只读旧图（现象是"转场卡住不动"）。
    // 中点取整行 ⇒ 长度 = 整数行 × 640 = 128 的整数倍 永远合法。
    //     （半行 = 320 像素 = 2.5 个突发，非法，所以做不到比"整行"更细。）
    //
    // s8（折线 y，8.8 定点）必须与 split 同步插值
    //     两者描述同一条接缝；只插一个会出现"卷带压不到接缝"的错位感。
    //     s8 走有符号中点：末段 s8 会跨过 0 变负（补码），
    //     若用无符号相加取平均会得到一个巨大的错误值 ⇒ 那一帧整屏几何乱掉
    //     （正好又是一次"闪"）。所以这里必须 $signed + 算术右移。
    //
    //   帧序列（CURL_FPS=2 时，chold 就是档内相位）：
    //     vs_fall#1: 发 split(0)            chold=0 -> prog 仍 0，随后 chold=1
    //     vs_fall#2: 发 mid(split(0),split(1))  chold=1 -> 随后 prog=1, chold=0
    //     vs_fall#3: 发 split(1) ...
    // ⇒ 每帧都动、步长减半，总帧数仍 140，总时长仍 2.352s
    // CURL_FPS=1 时 chold 恒 0，插值自动退化成"每帧一档"（不影响）
    wire [8:0]  csplit_a = curl_split(prog[6:0]);
    // 末档的下一档 = 70（表里是 0/负值），保险起见夹住
    wire [8:0]  csplit_b = (prog >= CURL_LAST) ? csplit_a
                                              : curl_split(prog[6:0] + 7'd1);
    // 整行中点（向上取整到整行）：两档都是整行，和不会超 480+480<1024
    wire [9:0]  csplit_sum = {1'b0, csplit_a} + {1'b0, csplit_b} + 10'd1;
    wire [8:0]  csplit_h   = csplit_sum[9:1];

    // CURL17：接缝行必须是偶数，否则那 32 帧整帧回放旧图
    //   读通路换基址的判据是 frame_fifo_read 里的
    //       read_cnt == read_split_len_latch
    //   而 read_cnt 只在每个突发结束时 += BURST_SIZE。BURST_SIZE 是
    //   frame_read_write 的默认参数 256传下去的（顶层只覆盖了
    //   WRITE_V_FLIP / FRAME_WIDTH / FRAME_HEIGHT，没碰 BURST_SIZE），
    //   不是 frame_fifo_read 自己默认的 128 —— 早先的注释就是按 128 推的，
    //   于是只保证"整行"，没有保证"256 像素整除"。
    //   一行 640 像素，640 = 2.5 x 256 ⇒
    //       split*640 % 256 == 0  ⇔  split 为偶数
    //   整数档表 curl_split() 全部是偶数（生成器保证），但半档中点
    //   (a+b+1)>>1 在 a+b ≡ 2 (mod 4) 时是奇数，140 档里命中 32 帧。
    //   那 32 帧 split_len 末尾余 128，read_cnt 永远等不到相等 ⇒ 地址从不跳到
    //   新图基址 ⇒ 整帧都从 read_addr_2（旧图）读 ⇒ 卷轴下方整片旧图，
    //   隔几帧闪一次（用户报的"一闪一闪"）。实测 sim_curl\c17\tb_split.v：
    //       split=439 行 (280960, 余128) -> 换址 0 次   ← 整帧旧图
    // split=438 行 (280320, 余0) -> 换址 26880 次
    //   这里统一对 csplit 向下取偶：接缝最多上移 1 行，仍在卷带下沿 YB 之上
    //   （被纸背盖住），不影响观感；同时 alt_base 的偏移用的是同一个
    //   curl_split_px_v，两者天然一致，新图不会错位。
    localparam [8:0] SPLIT_EVEN = 9'h1FE;      // 清 bit0 = 向下取偶

    // 折线 y 的整数档值与中点（有符号，8.8 定点）
    wire [19:0] s8_a   = curl_s8(prog[6:0]);
    wire [19:0] s8_b   = (prog >= CURL_LAST) ? s8_a
                                             : curl_s8(prog[6:0] + 7'd1);
    wire [19:0] s8_h   = ($signed(s8_a) + $signed(s8_b)) >>> 1;

    // 档内相位：chold==0 -> 整数档；chold==1 -> 中点（CURL_FPS=1 时恒为整数档）
    wire        half_ph = (CURL_FPS != 1) && chold[0];

    wire [8:0]  csplit  = (half_ph ? csplit_h : csplit_a) & SPLIT_EVEN;
    wire [19:0] s8_pub  = half_ph ? s8_h     : s8_a;

    // CURL14 新增（2026-10-21，修"卷曲转场仍然闪屏"）
    //   卷边半径 rad8 决定卷带着色带的厚度（2*rad8）。CURL11 只把 s8 做成了
    //   半档插值，rad8 还是按整数档取 —— 叠加 video_curl 原来"只在 prog 变化
    //   时重建几何表"，结果就是隔一帧接缝往前挪半档、着色带停在原位。
    //   这里让 rad8 与 s8 走同一套中点插值，两路几何永远同帧。
    wire [12:0] rad_a   = curl_rad8(prog[6:0]);
    wire [12:0] rad_b   = (prog >= CURL_LAST) ? rad_a
                                              : curl_rad8(prog[6:0] + 7'd1);
    wire [13:0] rad_sum = {1'b0, rad_a} + {1'b0, rad_b} + 14'd1;
    wire [12:0] rad_h   = rad_sum[13:1];
    wire [12:0] rad_pub = half_ph ? rad_h : rad_a;

    // split*640 = (s<<9) + (s<<7)，纯移位加法；最大 480*640 = 307200 < 2^19
    wire [18:0] curl_split_now = ({10'd0, csplit} << 9) + ({10'd0, csplit} << 7);

    // 2026-10-20 二次修订（修"sw2=1 卷曲转场新图一直闪屏"）
    //   病因：卷曲原来按 vs_fall 发布几何，而 frame_fifo_read 的 read_req
    //   也在 vs_fall 发出 —— 同一边沿上"发布"与"取数"抢跑：
    //     · 读侧在 vs_fall 之后才把几何锁进 read_split_len_latch，拿到的是新值G(n)
    //     · 但这一帧的像素是在这之前就按旧值G(n-1) 读出来的
    //     · 而 video_curl 的着色用寄存器当前值G(n) 去画那一帧的像素
    //   ⇒ 接缝/折线永远比读回的像素新一帧，接缝处错位十几行，每帧交替出现
    //     = 用户看到的"一直闪屏"。推送（SW2=0）从旧 RTL 起就用 vs_edge
    //     发布（早于 read_req），读侧与着色器拿到的是同一个值 —— 所以推送不闪。
    //   修法：卷曲也改到 vs_edge 发布（一帧一次，早于 vs_fall 的 read_req）。
    // 动画时长不受影响 prog 的自增仍在 ST_CURL 的 vs_fall 上（chold
    //     分频不变），发布只是把"这一档"提前半个帧边界交出去，总帧数仍是
    //     70×CURL_FPS = 140 帧 = 2.352 秒。
    wire pub_tick = vs_edge;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            push_on_r       <= 1'b0;
            curl_on_r       <= 1'b0;
            split_px_r      <= 19'd0;
            curl_split_px_r <= 19'd0;
            curl_prog_r     <= 7'd0;
            curl_s8_r       <= 20'd0;
            curl_rad8_r     <= 13'd0;
        end else if (pub_tick) begin
            // 只在帧边界发布新值
            push_on_r       <= push_active;
            split_px_r      <= push_active ? split_px_now   : 19'd0;
            curl_on_r       <= curl_active;
            curl_split_px_r <= curl_active ? curl_split_now : 19'd0;
            curl_prog_r     <= curl_active ? prog[6:0]      : 7'd0;
            curl_s8_r       <= curl_active ? s8_pub         : 20'd0;
            curl_rad8_r     <= curl_active ? rad_pub        : 13'd0;
        end
    end

    assign O_push_on       = push_on_r;
    assign O_split_pixels  = split_px_r;
    assign O_curl_on       = curl_on_r;
    assign O_curl_split_px = curl_split_px_r;
    assign O_curl_prog     = curl_prog_r;
    assign O_curl_s8       = curl_s8_r;
    assign O_curl_rad8     = curl_rad8_r;

    //-------------------------------------------------------------------------
    // 转场期间的显示缓冲索引锁存（修「整屏回放旧图」的边界 bug）
    //
    // 现象（父代理审出来的）
    // ----------------------
    //   sd_card_bmp 在 ST_WAIT 的 i_trans_go 那一拍就把 disp_buf_idx 提交成
    //   新缓冲了（sd_card_bmp.v:1006-1011），而 push_on_r / curl_on_r 要到
    //   下一个 vs_fall才落 0。这个窗口里：
    //        trans_on_r 仍为 1、split_len = 0
    //   若 alt_base 由实时 disp_buf_idx 取反得到，那么 alt_base 已经翻回
    //   旧缓冲 ⇒ 该窗口里发出的那一帧整屏回放旧图。
    //   窗口 ≈ trans_pend_s2 两级同步 + 提交延迟 ≈ 80~120ns；
    //   每帧 16.8ms，命中概率 ≈ 0.7%/次转场；展馆机约 90 次转场/小时
    //   ⇒ 约每 2 小时闪一次旧图。必须掐掉。
    //
    // 做法
    // ----
    //   在转场第一帧的 vs_fall把 disp_buf_idx 采样一次（那时提交尚未
    //   发生），整个转场期（含末尾 ST_WAIT 那个尾巴）都用锁存值。
    // 采样条件用组合的 trans_active，不能用 trans_on_r
    //     （trans_on_r 本身就是被 vs_fall 量化的，它置 1 时已经错过了那一拍）。
    // 采样也只做一次：条件是 !trans_on_r（本帧还没发布过）
    // curl 与 push 共用这一份锁存（同一个 I_disp_idx，不另开寄存器）
    //-------------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            O_disp_idx_lat <= 2'd0;
        else if (pub_tick && trans_active && !trans_on_r)
            O_disp_idx_lat <= I_disp_idx;   // 转场第一帧的发布沿，采样未提交的值
    end

endmodule
