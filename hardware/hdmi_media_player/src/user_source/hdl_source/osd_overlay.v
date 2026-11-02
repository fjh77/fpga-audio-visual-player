//=============================================================================
// 文件名  : osd_overlay.v
// 功能    : OSD 调节面板（底部居中面板 + 进度条 + 自动隐藏）
//
// 界面样式（仿电视/显示器 OSD）
// ---------------------------
//        ┌──────────────────────────────────┐
//        │  亮度                       60   │   名称 + 数值
//        │  ┌────────────────────────────┐  │
//        │  │██████████████████          │  │   进度条（一眼看出当前档位）
//        │  └────────────────────────────┘  │
//        └──────────────────────────────────┘
//                     底部居中
//
// 为什么这样设计（参考业界做法）
// ----------------------------
//   1) 进度条Xilinx OSD IP / 各类电视 OSD 都用进度条而不是纯数字：
//      纯数字要读，进度条一眼就能看出"当前在哪、还能往哪调"。
//   2) 自动隐藏OSD 空闲时一直挂在屏幕上会挡住画面内容，
//      这是被明确提出的可用性问题；正确做法是"操作时出现，停手后消失"。
//      本设计：任何一次按键后显示 3 秒（HOLD_CYCLES 可调）。
//   3) 底部居中不遮挡画面主体；带边框显得是一块独立面板。
//
// 着色
// ----
//   面板：暗玻璃（固定暗基色 + I_rgb>>3 = 87.5% 遮罩），保证任何底图上文字都清楚，
//         同时保留 1/8 背景亮度，能看出底下图像的明暗结构（不是死板色块）
//   边框：外圈 1px 深色 + 内圈 1px 浅灰双描边（亮图/暗图各有一条能勾边）
//   文字 / 进度条填充：纯白
//   全单色方案，不引入额外色彩处理，也不会受色彩空间影响。
//
// 坐标系
// ------
//   x/y 由 I_de / I_vs 自行计数，与 I_rgb 严格同拍
//     - 一行内 I_de 为低时 x 归零；为高时递增 -> 第一个有效像素时 x=0
//     - 每个 I_de 下降沿 y 加一
//     - I_vs 跳变沿 x/y 归零（帧起始）
//
// 流水线
// ------
//   1 拍延迟，vs/de 同步延迟。
//=============================================================================

module osd_overlay #(
    // 按键后保持显示的时钟数。video_clk = 25MHz
    //   3 秒 = 75_000_000
    parameter [26:0] HOLD_CYCLES = 27'd75_000_000
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

    // ---- OSD 控制 ----
    input               I_en,           // SW3：调节模式总开关
    input               I_trigger,      // 参数更新脉冲：每来一次重新计时
    input      [1:0]    I_sel,          // 0 = 亮度，1 = 对比度，2 = 音量
    input      [6:0]    I_bright,       // 亮度参数 0~100
    input      [6:0]    I_contrast,     // 对比度参数 0~100
    input      [6:0]    I_volume,       // 音量参数 0~100

    // ---- 字库外置接口（全片共享一份 osd_font_rom）----
    // 本模块优先级低于osd_menu
    //   除了 code/row 之外还要一个 grant：只有顶层把字库分给本模块时
    //   （grant=1）fpix 才有效。否则菜单与亮度条同时显示时，
    //   本模块会拿菜单的字形数据去画自己的文字 —— 串字。
    output      [5:0]   O_fcode,        // 本像素要查的字形编号
    output      [3:0]   O_frow,         // 字形内行号
    output              O_freq,         // 1 = 本像素落在本模块的文字区
    input       [15:0]  I_fbits,        // 共享字库返回的点阵行
    input               I_font_grant,   // 1 = 本次查表结果属于本模块

    // ---- 显示状态（给滚动字幕做门控）----
    //   本面板 y380..444 与滚动条带 y424..468 有 20 行重叠。
    //   本面板显示期间让滚动字幕整体隐藏，否则文字上半截会被
    //   本面板的底带盖掉、下半截还在 —— 出现"被切开"的字。
    output              O_show
);

    //-------------------------------------------------------------------------
    // 面板几何（底部居中）
    //-------------------------------------------------------------------------
    localparam [11:0] PX0 = 12'd180;        // 面板左上角 (640-280)/2
    localparam [11:0] PY0 = 12'd380;
    localparam [11:0] PW  = 12'd280;
    localparam [11:0] PH  = 12'd64;

    localparam [11:0] TXT_OFF = 12'd16;     // 文字左边距
    localparam [11:0] TXT_Y   = 12'd10;     // 文字上边距
    localparam [11:0] VAL_X   = 12'd216;    // 数值区左边距（3 个字格 = 48px，右对齐）

    localparam [11:0] BAR_X = 12'd16;       // 进度条相对面板的位置
    localparam [11:0] BAR_Y = 12'd36;
    localparam [11:0] BAR_W = 12'd248;      // 248 = PW - 2*16
    localparam [11:0] BAR_H = 12'd16;

    localparam [23:0] C_BORDER = 24'hC0C0C0;    // 边框浅灰
    localparam [23:0] C_TEXT   = 24'hFFFFFF;    // 文字/条填充纯白

    //-------------------------------------------------------------------------
    // 像素坐标计数（与 I_rgb 同拍）
    //-------------------------------------------------------------------------
    reg  [11:0] x_cnt, y_cnt;
    reg         vs_d, de_d;

    wire vs_edge = (I_vs != vs_d);
    wire de_fall = de_d & ~I_de;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            x_cnt <= 12'd0;
            y_cnt <= 12'd0;
            vs_d  <= 1'b0;
            de_d  <= 1'b0;
        end else begin
            vs_d <= I_vs;
            de_d <= I_de;
            if (vs_edge) begin
                x_cnt <= 12'd0;
                y_cnt <= 12'd0;
            end else if (!I_de) begin
                x_cnt <= 12'd0;
                if (de_fall)
                    y_cnt <= y_cnt + 12'd1;
            end else begin
                x_cnt <= x_cnt + 12'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 自动隐藏计时：按键（I_trigger）后显示 HOLD_CYCLES 个时钟
    //-------------------------------------------------------------------------
    reg [26:0] hold_cnt;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            hold_cnt <= 27'd0;
        else if (I_trigger)
            hold_cnt <= 27'd0;
        else if (hold_cnt < HOLD_CYCLES)
            hold_cnt <= hold_cnt + 27'd1;
    end

    wire show = I_en && (hold_cnt < HOLD_CYCLES);

    // 把显示状态引出去，让滚动字幕在亮度条显示期间整体隐藏
    assign O_show = show;

    //-------------------------------------------------------------------------
    // 数值 -> 三位十进制 + 进度条长度
    //-------------------------------------------------------------------------
    reg  [7:0] val;
    reg  [3:0] d2, d1, d0;
    reg  [7:0] t;
    integer    i;

    always @(*) begin
        val = {1'b0, (I_sel == 2'd0) ? I_bright :
                     (I_sel == 2'd1) ? I_contrast : I_volume};
        // 百位（最多减 3 次，覆盖到 255）
        d2 = 4'd0;
        t  = val;
        for (i = 0; i < 3; i = i + 1)
            if (t >= 8'd100) begin
                d2 = d2 + 4'd1;
                t  = t - 8'd100;
            end
        // 十位
        d1 = 4'd0;
        for (i = 0; i < 10; i = i + 1)
            if (t >= 8'd10) begin
                d1 = d1 + 4'd1;
                t  = t - 8'd10;
            end
        d0 = t[3:0];
    end

    // 前导零显示为空格，避免出现 "060" 这种观感
    localparam [4:0] C_SPACE = 5'd10;
    wire [4:0] c2 = (d2 == 4'd0) ? C_SPACE : {1'b0, d2};
    wire [4:0] c1 = (d2 == 4'd0 && d1 == 4'd0) ? C_SPACE : {1'b0, d1};
    wire [4:0] c0 = {1'b0, d0};

    // 进度条填充长度：0 ~ (BAR_W-4) = 244，满量程 100
    // 必须用显式宽的中间信号，否则乘积会被 12 位上下文截断
    //   val * 244/100 = val * 2.44 -> 用 156/64 = 2.4375 逼近
    wire [15:0] fill_p = val * 8'd156;
    wire [15:0] fill_x = fill_p >> 6;
    wire [11:0] fill_len = fill_x[11:0];

    //-------------------------------------------------------------------------
    // 面板 / 边框 / 进度条 判定
    //-------------------------------------------------------------------------
    wire in_panel = (x_cnt >= PX0) && (x_cnt < PX0 + PW) &&
                    (y_cnt >= PY0) && (y_cnt < PY0 + PH);

    wire in_border = in_panel && ((x_cnt < PX0 + 12'd2) ||
                                  (x_cnt >= PX0 + PW - 12'd2) ||
                                  (y_cnt < PY0 + 12'd2) ||
                                  (y_cnt >= PY0 + PH - 12'd2));

    // 最外 1px 暗轮廓：亮图片上靠它勾出面板边界（暗 vs 白 ≈ 18:1），
    // 暗图片上靠内侧的 C_BORDER 浅灰框勾边。单色必然在某个极端背景上"融化"。
    wire panel_edge = in_panel && ((x_cnt == PX0) || (x_cnt == PX0 + PW - 12'd1) ||
                                   (y_cnt == PY0) || (y_cnt == PY0 + PH - 12'd1));

    wire [11:0] bx = x_cnt - (PX0 + BAR_X);
    wire [11:0] by = y_cnt - (PY0 + BAR_Y);

    wire in_bar = in_panel &&
                  (x_cnt >= PX0 + BAR_X) && (x_cnt < PX0 + BAR_X + BAR_W) &&
                  (y_cnt >= PY0 + BAR_Y) && (y_cnt < PY0 + BAR_Y + BAR_H);

    wire bar_border = in_bar && ((bx < 12'd2) || (bx >= BAR_W - 12'd2) ||
                                 (by < 12'd2) || (by >= BAR_H - 12'd2));

    wire bar_fill = in_bar && !bar_border && (bx >= 12'd2) &&
                    (bx < fill_len + 12'd2);

    //-------------------------------------------------------------------------
    // 文字区：名称（2 汉字）+ 数值（3 字格，右对齐）
    //-------------------------------------------------------------------------
    wire [11:0] nx = x_cnt - (PX0 + TXT_OFF);
    wire [11:0] ny = y_cnt - (PY0 + TXT_Y);
    wire in_name = (x_cnt >= PX0 + TXT_OFF) && (x_cnt < PX0 + TXT_OFF + 12'd32) &&
                   (y_cnt >= PY0 + TXT_Y)  && (y_cnt < PY0 + TXT_Y + 12'd16);

    wire [11:0] vx = x_cnt - (PX0 + VAL_X);
    wire [11:0] vy = y_cnt - (PY0 + TXT_Y);
    wire in_val = (x_cnt >= PX0 + VAL_X) && (x_cnt < PX0 + VAL_X + 12'd48) &&
                  (y_cnt >= PY0 + TXT_Y) && (y_cnt < PY0 + TXT_Y + 12'd16);

    // 字形编号（必须和 tools/make_osd_font.py 的 GLYPHS 表一致）
    //   字库槽位 64，编码位宽 6
    localparam [5:0] C_LIANG = 6'd11;   // 亮
    localparam [5:0] C_DU    = 6'd12;   // 度
    localparam [5:0] C_DUI   = 6'd13;   // 对
    localparam [5:0] C_BI    = 6'd14;   // 比
    localparam [5:0] C_YIN   = 6'd15;   // 音
    localparam [5:0] C_LIANG2= 6'd16;   // 量

    // 三页：亮度 / 对比度 / 音量（字形槽 11~16 正好是 亮度对比音量）
    wire [5:0] ncode = nx[4] ? ((I_sel == 2'd0) ? C_DU  :
                                (I_sel == 2'd1) ? C_BI  : C_LIANG2)
                             : ((I_sel == 2'd0) ? C_LIANG :
                                (I_sel == 2'd1) ? C_DUI : C_YIN);

    wire [1:0] vidx = vx[5:4];
    wire [5:0] vcode = (vidx == 2'd0) ? c2 : ((vidx == 2'd1) ? c1 : c0);

    wire [3:0] frow = in_val ? (vy[3:0]) : (ny[3:0]);
    wire [3:0] fcol = in_val ? (vx[3:0]) : (nx[3:0]);
    wire [5:0] fcode = in_val ? vcode : ncode;

    //-------------------------------------------------------------------------
    // 不再内部例化 osd_font_rom，改为把 (code,row) 送出顶层共享查表
    //   grant=0 表示本次查表结果被更高优先级的 osd_menu 占用，
    //   此时 fpix 强制为 0 —— 底带和进度条不受影响，只是文字不画。
    //   这与原有行为一致：菜单本来就盖在亮度条上面。
    //-------------------------------------------------------------------------
    assign O_fcode = fcode;
    assign O_frow  = frow;
    assign O_freq  = in_panel && (in_name || in_val);

    wire fpix = I_fbits[4'd15 - fcol] & I_font_grant;

    //-------------------------------------------------------------------------
    // 合成（优先级：文字 > 条填充 > 边框 > 面板底）
    //-------------------------------------------------------------------------
    wire in_text = in_panel && fpix && (in_name || in_val);

    //-------------------------------------------------------------------------
    // 面板底色：暗色遮罩 = 固定暗基色 + 背景/8（87.5% 遮罩）
    //
    // 不能整体右移
    //       {1'b0, I_rgb[23:1]} 是整个 24 位右移 1 位，会跨字节串色：
    //       纯绿 (00,FF,00) 会变成 (00,7F,80) —— 蓝色凭空多出 0x80。
    //       表现为"OSD 面板叠在画面上时那块区域颜色不对"。
    //   必须每个通道分别取高位。
    //
    // 为什么不是 >>1
    //       >>1 是 50% 遮罩，纯白图片上面板底 = (127,127,127)，
    //       白字对比度只有 4.00:1，在明亮画面上偏飘。
    //       改 >>3 + 基色后最亮 (43,46,53)，白字升到 11.70:1；
    //       同时保留 1/8 背景亮度，面板仍是"暗玻璃"，底下图像明暗可辨。
    //       与 osd_menu.v 用同一套遮罩参数，两块 OSD 观感一致。
    //-------------------------------------------------------------------------
    localparam [7:0] MASK_R = 8'd12;    // 基色 (12,15,22) #0C0F16
    localparam [7:0] MASK_G = 8'd15;
    localparam [7:0] MASK_B = 8'd22;

    wire [7:0] bg_mr = { 5'd0, I_rgb[23:19] };   // R >> 3 → 0..31
    wire [7:0] bg_mg = { 5'd0, I_rgb[15:11] };   // G >> 3
    wire [7:0] bg_mb = { 5'd0, I_rgb[7:3]   };   // B >> 3

    wire [23:0] panel_bg = { bg_mr + MASK_R,
                             bg_mg + MASK_G,
                             bg_mb + MASK_B };

    reg [23:0] mix;
    always @(*) begin
        if (!show)
            mix = I_rgb;                                    // 隐藏：完全透传
        else if (in_text)
            mix = C_TEXT;
        else if (bar_fill)
            mix = C_TEXT;
        else if (panel_edge)
            mix = 24'h0A0C12;                               // 最外 1px 暗轮廓
        else if (bar_border || in_border)
            mix = C_BORDER;
        else if (in_panel)
            mix = panel_bg;                                 // 暗玻璃底（87.5% 遮罩）
        else
            mix = I_rgb;
    end

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            O_rgb <= 24'd0;
            O_vs  <= 1'b0;
            O_de  <= 1'b0;
        end else begin
            O_rgb <= mix;
            O_vs  <= I_vs;
            O_de  <= I_de;
        end
    end

endmodule
