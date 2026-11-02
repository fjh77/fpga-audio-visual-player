//=============================================================================
// 文件名  : osd_menu.v
// 功能    : 轮播设置菜单 + 停止提示条（HDMI 画面叠加，中文，16x16 点阵）
// 版本    : 随工程 c52（布局 / 配色 / 可读性已在 c38、c39 定稿）
//
// 两个面板共用一套坐标系与字库：
//
//   ① 设置菜单（SW1=1 时显示，常显，不自动消失）
//      ┌──────────────────────────────────────┐  480x336 @ (80,72)
//      │  轮 播 设 置                          │  金色标题
//      │ ────────────────────────────────────  │  1px 分隔线
//      │▌ 讲 解 模 式                          │  选中行：深蓝底 + 金色左条
//      │   快 速 浏 览 （ 3 秒 ）               │  未选中：灰色文字
//      │   慢 观 看 （ 1 0 秒 ）                │
//      │   手 动 浏 览                          │
//      │ ┌────────────────────────────────┐    │
//      │ │已 选 讲 解 模 式                │    │  状态行：独立底色块（青色）
//      │ └────────────────────────────────┘    │
//      │ 2 选 择    3 设 置                     │  底部提示
//      └──────────────────────────────────────┘
//
//   ② 停止提示条（KEY_3 停下自动轮播时弹 3 秒，自动消失）
//      ┌────────────────────────────┐  320x44 @ (160,424)
//      │▌      自 动 轮 播 已 停 止   │  深底 + 金色左条 + 居中白字
//      └────────────────────────────┘
//
// 设计要点
// --------
//   1) 设置菜单不自动消失：它就是"设置模式"的一部分，SW1 拨回 0 才消失。
//      若也做 3 秒自动隐藏，操作者抬头就找不到菜单了。
//   2) 停止提示条自动消失：一次性提醒，挂久了挡画面。
//   3) 光标行整行反白：展会光线杂、观看距离远，只靠三角符号不够。
// 反白条的横向范围用 in_sel_row（不含 in_txt_col），否则会被
// "8 个字格" 的宽度限制成 128px 的小蓝块，看不见"整行"。
//   4) 已选XXX 状态行：KEY_3 确认后立刻换字，给操作者可见反馈。
//      没有它的话，按下 KEY_3 屏幕上什么都不变，操作者会以为按键坏了。
//   5) 面板底 = 暗玻璃（固定暗基色 + 背景/8 = 87.5% 遮罩）：
//      既保证任何亮图/复杂图下文字都够对比度，又保留 1/8 背景亮度，
//      所以面板仍是"透"的（能看出底下图像的明暗结构），不是死板色块。
// 不能整体右移 {1'b0, I_rgb[23:1]} 是 24 位整体右移，会跨字节串色
//      （纯绿 00,FF,00 会变成 00,7F,80，蓝色凭空多 0x80）。必须逐通道取高位。
//
// 设计要点（历次迭代的结论，按当前实现整理）
// ------------------------------------------
//   布局  ：面板 480x336 居中 (80,72)，内边距 32，行距 44，状态行 y252
//           行距 44 不是 2 的幂 -> 行号用比较链，不能用位切片
//   配色  ：金标题 + 青状态 + 低饱和蓝选中底 + 灰次要文字
//   可读性：遮罩 87.5%（背景>>3 + 暗基色），白底上正文 11.7:1
//           次要文字 #9AA3B4；原来的 #788091 在白底只有 1.01:1，等于看不见
//           边框"外暗 1px + 内亮 1px"双描边，亮图暗图各有一条能勾出边界
//   提示条：两套文案共用同一面板
//           停止「自 动 轮 播 已 停 止」7 字 / 3 秒
//           轮播「自 动 轮 播 中」      5 字 / 1 秒
//           字数不同，字格起点各自居中（104 / 120）；统一左对齐会偏 8px
//           1 秒必须小于最短轮播周期（3 秒），否则会横跨到下一张图
//   e) 提示条文案「已停止自动轮播」→「自动轮播已停止」，7 字居中
// 居中后的字格起点 104 不是 16 的倍数，字格内列索引必须用
// (bx - B_TXT_X) 而不是 bx[3:0]，否则整条文字被横向切碎
//
// 坐标系 / 流水线
// ---------------
//   与 osd_overlay.v 同构：x/y 由 I_de / I_vs 自行计数，1 拍延迟，
//   vs/de 同步延迟，可与其串联在同一像素流上。
//
// 字库编码（必须与 tools/make_osd_font.py 的 GLYPHS 表一致，6 位 64 槽）
// -----------------------------------------------------------------------
//    0-9   数字        10 空格        11-16 亮度对比音量（osd_overlay 用）
//   17-31 讲解模式 / 快速浏览 / 慢观看 / 手动 / 已 / 确
//   32-63 轮播设置 选择 按键 停止自 秒 当前 开关 上一张下 音频 中共张 从头
// 64 槽已满，新增字形必须先腾出未引用槽
//=============================================================================

module osd_menu #(
    // 停止提示条保持时间：video_clk = 25MHz，3 秒 = 75_000_000
    parameter [26:0] BANNER_HOLD      = 27'd75_000_000,
    // 自动轮播短提示保持时间：1 秒 = 25_000_000
    // 必须严格小于最短轮播周期（快速浏览档 3 秒）
    //     否则提示会横跨到下一张图，从"提示"变成"常驻挡画面"，
    //     正是这条需求要避免的事。1 秒只占最短周期的 1/3。
    parameter [26:0] BANNER_HOLD_AUTO = 27'd25_000_000
)(
    input               I_clk,          // video_clk 25MHz
    input               I_rst,

    // ---- 视频（串在像素流中间）----
    input      [23:0]   I_rgb,
    input               I_vs,
    input               I_de,
    output reg [23:0]   O_rgb,
    output reg          O_vs,
    output reg          O_de,

    // ---- 设置菜单 ----
    input               I_setup,        // 1 = 显示设置菜单（SW1）
    input      [1:0]    I_menu_sel,     // 光标位置 0..3
    input      [1:0]    I_cfg_sel,      // 已确认档位 0..3（提示行显示）

    // ---- 停止提示条 ----
    input               I_banner,       // 单拍脉冲：停止提示（3 秒）

    // ---- 自动轮播短提示 ----
    input               I_banner_auto,  // 单拍脉冲：轮播中提示（1 秒）

    // ---- 状态输出（给滚动字幕做门控）----
    // 只加端口、不改任何内部逻辑
    //   提示条 (320x44 @160,424) 与滚动字幕条带 (全宽 @0,424) 完全重叠，
    //   提示条显示期间要让字幕定格，否则两层信息打架。
    output              O_banner_show,

    // ---- 字库外置接口（全片共享一份 osd_font_rom）----
    // 本模块是最高优先级用户，不需要 grant 信号
    //   顶层按 menu_freq ? menu : overlay 选择后查表，再把 bits 广播回来。
    //   同一像素只会命中一个图层的文字区，所以 code/row 不需要额外仲裁。
    output      [5:0]   O_fcode,        // 本像素要查的字形编号
    output      [3:0]   O_frow,         // 字形内行号
    output              O_freq,         // 1 = 本像素落在本模块的文字区
    input       [15:0]  I_fbits         // 共享字库返回的点阵行
);

    //-------------------------------------------------------------------------
    // 面板几何（屏幕居中：(640-480)/2 = 80，(480-336)/2 = 72）
    //-------------------------------------------------------------------------
    localparam [11:0] MX0 = 12'd80;
    localparam [11:0] MY0 = 12'd72;
    localparam [11:0] MW  = 12'd480;
    localparam [11:0] MH  = 12'd336;

    localparam [11:0] M_TXT_X   = 12'd32;   // 相对面板的文字左边距（字格起点）
    localparam [11:0] M_TITLE_Y = 12'd24;   // 标题行 y
    localparam [11:0] M_TLINE_Y = 12'd48;   // 标题分隔线 y（1px）
    localparam [11:0] M_ITEM0_Y = 12'd68;   // 第一项 y
    localparam [11:0] M_ITEM_DY = 12'd44;   // 行距（16px 字形 + 28 间隔）
    localparam [11:0] M_CFG_Y   = 12'd252;  // 「已选XXX」状态行 y
    localparam [11:0] M_HINT_Y  = 12'd300;  // 底部提示行 y

    // 行距 44 不是 2 的幂，必须用比较链求行号/行首（见下 item_row/item_base）
    localparam [11:0] IT_OFF1 = M_ITEM_DY;                     //  44
    localparam [11:0] IT_OFF2 = M_ITEM_DY + M_ITEM_DY;         //  88
    localparam [11:0] IT_OFF3 = IT_OFF2 + M_ITEM_DY;           // 132
    localparam [11:0] IT_OFF4 = IT_OFF3 + M_ITEM_DY;           // 176（4 行总跨度）

    localparam [11:0] M_INSET    = 12'd16;  // 选中行底/状态行底块的左右内缩
    localparam [11:0] M_SEL_PAD  = 12'd8;   // 选中行底相对字形上下各扩 8
    localparam [11:0] M_SEL_H    = 12'd32;  // 选中行底高度（16 字形 + 8 + 8）
    localparam [11:0] M_BAR_W    = 12'd4;   // 选中行金色左强调条宽

    localparam [11:0] M_CFG_BG_TOP = M_CFG_Y - 12'd8;   // 244
    localparam [11:0] M_CFG_BG_BOT = M_CFG_Y + 12'd24;  // 276

    // 停止提示条 320x44，左上角 (160,424)
    localparam [11:0] BX0 = 12'd160;
    localparam [11:0] BY0 = 12'd424;
    localparam [11:0] BW  = 12'd320;
    localparam [11:0] BH  = 12'd44;
    localparam [11:0] B_TXT_X  = 12'd104;    // 7 字（112px）居中：(320-112)/2
    localparam [11:0] B_TXT_X5 = 12'd120;    // 5 字（ 80px）居中：(320- 80)/2
    localparam [11:0] B_TXT_Y  = 12'd14;     // (44 - 16)/2 = 14，垂直居中
    localparam [11:0] B_BAR_W = 12'd4;      // 提示条金色左强调条宽

    //-------------------------------------------------------------------------
    // 配色
    //   两条约束同时满足才算数：
    //     ① 层级靠亮度差表达（正文 235 / 次要 154），不靠"把次要色压到很暗"
    //        —— 压太暗在亮图上就没了（#788091 在白底上只有 1.01:1）
    //     ② 每个前景色在纯白图片下都要 ≥4.5:1（WCAG AA 小字标准），
    //        因为白图是面板底最亮的极端情况，过得了这一关，任何真实图都过。
    //   实测：tools/contrast_audit.py —— 改前 1.01~4.00，改后 5.35~11.70。
    //-------------------------------------------------------------------------
    localparam [23:0] C_BORDER    = 24'h6E788C;   // 第 2px 亮框（暗底上勾边）
    localparam [23:0] C_OUTLINE   = 24'h0A0C12;   // 最外 1px 暗轮廓（亮图上勾边）
    localparam [23:0] C_BORDER_LO = 24'h454E63;   // 内框暗部 / 标题分隔线
    localparam [23:0] C_TEXT      = 24'hEBEEF5;   // 正文近白        vs 白底 11.70:1
    localparam [23:0] C_DIM       = 24'h9AA3B4;   // 未选中项文字    vs 白底  5.35:1
    localparam [23:0] C_SEL_BG    = 24'h263E6E;   // 选中行底（低饱和蓝）
    localparam [23:0] C_ACCENT    = 24'hF0C040;   // 强调条 / 标题（金）
    localparam [23:0] C_TITLE     = 24'hF0C040;   // 标题（与强调条同色）vs 白底 7.97:1
    localparam [23:0] C_HINT      = 24'h9AA3B4;   // 底部提示行      vs 白底  5.35:1
    localparam [23:0] C_CFG       = 24'h56C8BE;   // 状态行文字（青）vs 白底  6.73:1
    localparam [23:0] C_CFG_BG    = 24'h2E3648;   // 状态行底色块（须比最亮面板底更亮）

    //-------------------------------------------------------------------------
    // 像素坐标计数（与 I_rgb 同拍）—— 与 osd_overlay.v 同构
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
    // 提示条自动隐藏计时
    //   两路提示共用一套计数器，但各有时长上限：
    //     停止提示 3 秒（一次性告知，可以多留一会儿）
    // 轮播提示 1 秒（必须 < 最短轮播周期 3 秒，否则会横跨到下一张图）
    //   后到的提示直接覆盖前一条（清零 + 换类型），不会叠加显示。
    //-------------------------------------------------------------------------
    reg [26:0] hold_cnt;
    reg        hold_auto;      // 1 = 当前显示的是"自动轮播中"短提示

    wire [26:0] hold_lim = hold_auto ? BANNER_HOLD_AUTO : BANNER_HOLD;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            hold_cnt  <= 27'd0;
            hold_auto <= 1'b0;
        end else if (I_banner) begin
            hold_cnt  <= 27'd0;
            hold_auto <= 1'b0;                 // 停止提示：3 秒
        end else if (I_banner_auto) begin
            hold_cnt  <= 27'd0;
            hold_auto <= 1'b1;                 // 轮播提示：1 秒
        end else if (hold_cnt < hold_lim) begin
            hold_cnt <= hold_cnt + 27'd1;
        end
    end

    wire show_banner = (hold_cnt < hold_lim);
    wire show_menu   = I_setup;

    // 提示条显示状态引出（滚动字幕在它显示期间定格）
    assign O_banner_show = show_banner;

    //-------------------------------------------------------------------------
    // 字库编码（与 tools/make_osd_font.py 的 GLYPHS 表一致）
    //-------------------------------------------------------------------------
    localparam [5:0] C_SPACE = 6'd10;

    // 数字 / 标题
    localparam [5:0] C_0  = 6'd0;
    localparam [5:0] C_1  = 6'd1;
    localparam [5:0] C_2  = 6'd2;
    localparam [5:0] C_3  = 6'd3;
    localparam [5:0] C_LUN  = 6'd32;   // 轮
    localparam [5:0] C_BO   = 6'd33;   // 播
    localparam [5:0] C_SHE  = 6'd34;   // 设
    localparam [5:0] C_ZHI  = 6'd35;   // 置

    // 四个选项
    localparam [5:0] C_JIANG = 6'd17;  // 讲
    localparam [5:0] C_JIE   = 6'd18;  // 解
    localparam [5:0] C_MO    = 6'd19;  // 模
    localparam [5:0] C_SHI   = 6'd20;  // 式
    localparam [5:0] C_KUAI  = 6'd21;  // 快
    localparam [5:0] C_SU    = 6'd22;  // 速
    localparam [5:0] C_LIU   = 6'd23;  // 浏
    localparam [5:0] C_LAN   = 6'd24;  // 览
    localparam [5:0] C_MAN   = 6'd25;  // 慢
    localparam [5:0] C_GUAN  = 6'd26;  // 观
    localparam [5:0] C_KAN   = 6'd27;  // 看
    localparam [5:0] C_SHOU  = 6'd28;  // 手
    localparam [5:0] C_DONG  = 6'd29;  // 动

    // 提示行 / 提示条
    localparam [5:0] C_AN    = 6'd38;  // 按
    localparam [5:0] C_JIAN  = 6'd39;  // 键
    localparam [5:0] C_XUAN  = 6'd36;  // 选
    localparam [5:0] C_ZE    = 6'd37;  // 择
    localparam [5:0] C_QUE   = 6'd31;  // 确
    localparam [5:0] C_YI    = 6'd30;  // 已
    localparam [5:0] C_TING  = 6'd40;  // 停
    localparam [5:0] C_ZHI2  = 6'd41;  // 止
    localparam [5:0] C_ZI    = 6'd42;  // 自
    localparam [5:0] C_MIAO  = 6'd43;  // 秒
    localparam [5:0] C_ZHONG = 6'd54;  // 中（「自动轮播中」用，字库现成，无需新增字形）
    localparam [5:0] C_KUOH0 = 6'd60;  //（
    localparam [5:0] C_KUOH1 = 6'd61;  // ）

    //-------------------------------------------------------------------------
    // 设置菜单几何判定
    //-------------------------------------------------------------------------
    wire in_menu = show_menu &&
                   (x_cnt >= MX0) && (x_cnt < MX0 + MW) &&
                   (y_cnt >= MY0) && (y_cnt < MY0 + MH);

    wire menu_border = in_menu && ((x_cnt < MX0 + 12'd2) ||
                                   (x_cnt >= MX0 + MW - 12'd2) ||
                                   (y_cnt < MY0 + 12'd2) ||
                                   (y_cnt >= MY0 + MH - 12'd2));

    // 最外 1px 暗轮廓：边框做成"外暗 + 内亮"双描边
    //   亮图片上靠暗轮廓勾出面板边界（暗 vs 白 ≈ 18:1），
    //   暗图片上靠内侧 C_BORDER 亮框勾边（亮 vs 黑 ≈ 4.3:1）。
    //   单用一种颜色必然在某个极端背景上"融化"——这是两条都要的原因。
    wire [11:0] mx = x_cnt - MX0;       // 面板内相对 x（in_menu 为假时下溢，比较自动不成立）
    wire [11:0] my = y_cnt - MY0;

    wire menu_edge = in_menu && ((mx == 12'd0) || (mx == MW - 12'd1) ||
                                 (my == 12'd0) || (my == MH - 12'd1));

    // 内框暗部 1px：外框之内再收一圈，做出层次（原版只有单层灰框）
    wire in_border_lo = in_menu && ((mx == 12'd2) || (mx == MW - 12'd3) ||
                                    (my == 12'd2) || (my == MH - 12'd3));

    // 文字区横向范围：从 M_TXT_X 起 8 个字格（128px）
    wire in_txt_col = (mx >= M_TXT_X) && (mx < M_TXT_X + 12'd128);

    // 标题行
    wire in_title = in_menu && in_txt_col &&
                    (my >= M_TITLE_Y) && (my < M_TITLE_Y + 12'd16);

    // 标题分隔线（1px，从左边距到右边距对称位置）
    wire in_title_line = in_menu && (my == M_TLINE_Y) &&
                         (mx >= M_TXT_X) && (mx < MW - M_TXT_X);

    //-------------------------------------------------------------------------
    // 选项行：行距 44 不是 2 的幂，不能再用 item_off[6:5] / item_off[4:0]
    //   改为比较链求行号与行首：r0=0 r1=44 r2=88 r3=132，4 行跨度 176。
    //   代价约 30 LUT（3 个 12 位比较器 + 3 个减法器 + 两级 mux），可接受。
    // item_off < 176 的边界必须保留：否则超出 4 行后行号回绕，
    // 底部空白区会又冒出第一项。
    //-------------------------------------------------------------------------
    wire [11:0] item_off = my - M_ITEM0_Y;

    wire [1:0]  item_row = (item_off < IT_OFF1) ? 2'd0 :
                           (item_off < IT_OFF2) ? 2'd1 :
                           (item_off < IT_OFF3) ? 2'd2 : 2'd3;

    wire [11:0] item_base = (item_off < IT_OFF1) ? 12'd0 :
                            (item_off < IT_OFF2) ? IT_OFF1 :
                            (item_off < IT_OFF3) ? IT_OFF2 : IT_OFF3;

    wire [11:0] item_sub12 = item_off - item_base;   // 字形槽内 y 偏移 0..43

    // 字形本体（16px 高）：item_off < 176 已隐含 my >= M_ITEM0_Y
    //（my < M_ITEM0_Y 时 12 位减法下溢成大数，条件自然不成立）
    wire in_item = in_menu && in_txt_col &&
                   (item_off < IT_OFF4) && (item_sub12 < 12'd16);

    // 光标行整行反白：字形上下各扩 8，高 32；横向从 16 到 MW-16（不贴边框）
    // 用 sel_row_top 而不是 item_sub：反白条比字形高，且要跟着行号走
    wire [11:0] sel_row_top = M_ITEM0_Y - M_SEL_PAD + item_base;

    wire in_sel_row = in_menu && !menu_border &&
                      (item_row == I_menu_sel) &&
                      (my >= sel_row_top) && (my < sel_row_top + M_SEL_H) &&
                      (mx >= M_INSET) && (mx < MW - M_INSET);

    // 选中行左侧金色强调条（4px）
    wire in_sel_bar = in_sel_row && (mx < M_INSET + M_BAR_W);

    // 状态行底色块：上下各扩 8（244..275），横向与选中行对齐（16..MW-16）
    wire in_cfg_bg = in_menu &&
                     (my >= M_CFG_BG_TOP) && (my < M_CFG_BG_BOT) &&
                     (mx >= M_INSET) && (mx < MW - M_INSET);

    // 底部提示行
    wire in_hint = in_menu && in_txt_col &&
                   (my >= M_HINT_Y) && (my < M_HINT_Y + 12'd16);

    // 已确认档位状态行「已选XXX」（KEY_3 确认后的可见反馈）
    wire in_cfg = in_menu && in_txt_col &&
                  (my >= M_CFG_Y) && (my < M_CFG_Y + 12'd16);

    //-------------------------------------------------------------------------
    // 菜单文字编码（每行 8 个字格，mcol = 0..7）
    //   标题  「轮 播 设 置」
    //   选项0  「讲 解 模 式」
    //   选项1  「快 速 浏 览（ 3 秒 ）」   -> mcol 4=（ 5=3 6=秒 7=）
    //   选项2  「慢 观 看  （ 1 0 秒 ）」   -> mcol 3=（ 4=1 5=0 6=秒 7=）
    //   选项3  「手 动 浏 览」
    //   提示  「2 选 择     3 设 置」
    //          -> mcol 0=2 1=选 2=择 3=空 4=空 5=3 6=设 7=置
    // 每行最多 8 个字格（128px），与 in_txt_col 的宽度严格一致
    // 字库 64 槽已满、且没有"认"字，所以第二半用「3 设置」
    //     （= KEY_3 确认/生效）而不用「3 确认」。若要改成"确认"，
    // 必须先给 osd_font_rom 增加"认"的字形槽（当前无空槽）。
    //-------------------------------------------------------------------------
    wire [2:0] mcol = (mx - M_TXT_X) >> 4;

    // 「已选」后面的档位名：按已确认档位 I_cfg_sel 取字
    //   0=讲解模式  1=快速浏览  2=慢观看  3=手动浏览
    //   （8 个字格：mcol 0=已 1=选 2..5=档位名，其余补空格）
    reg [5:0] cfg_c2, cfg_c3, cfg_c4, cfg_c5;
    always @(*) begin
        case (I_cfg_sel)
            2'd0:    begin cfg_c2 = C_JIANG; cfg_c3 = C_JIE;  cfg_c4 = C_MO;  cfg_c5 = C_SHI;   end
            2'd1:    begin cfg_c2 = C_KUAI;  cfg_c3 = C_SU;   cfg_c4 = C_LIU; cfg_c5 = C_LAN;   end
            2'd2:    begin cfg_c2 = C_MAN;   cfg_c3 = C_GUAN; cfg_c4 = C_KAN; cfg_c5 = C_SPACE; end
            default: begin cfg_c2 = C_SHOU;  cfg_c3 = C_DONG; cfg_c4 = C_LIU; cfg_c5 = C_LAN;   end
        endcase
    end

    reg [5:0] mcode;
    always @(*) begin
        mcode = C_SPACE;
        if (in_title) begin
            case (mcol)
                3'd0: mcode = C_LUN;
                3'd1: mcode = C_BO;
                3'd2: mcode = C_SHE;
                3'd3: mcode = C_ZHI;
                default: mcode = C_SPACE;
            endcase
        end else if (in_item) begin
            case (item_row)
                2'd0: case (mcol)
                          3'd0: mcode = C_JIANG;
                          3'd1: mcode = C_JIE;
                          3'd2: mcode = C_MO;
                          3'd3: mcode = C_SHI;
                          default: mcode = C_SPACE;
                      endcase
                2'd1: case (mcol)
                          3'd0: mcode = C_KUAI;
                          3'd1: mcode = C_SU;
                          3'd2: mcode = C_LIU;
                          3'd3: mcode = C_LAN;
                          3'd4: mcode = C_KUOH0;
                          3'd5: mcode = C_3;
                          3'd6: mcode = C_MIAO;
                          3'd7: mcode = C_KUOH1;
                          default: mcode = C_SPACE;
                      endcase
                2'd2: case (mcol)
                          3'd0: mcode = C_MAN;
                          3'd1: mcode = C_GUAN;
                          3'd2: mcode = C_KAN;
                          3'd3: mcode = C_KUOH0;
                          3'd4: mcode = C_1;
                          3'd5: mcode = C_0;
                          3'd6: mcode = C_MIAO;
                          3'd7: mcode = C_KUOH1;
                          default: mcode = C_SPACE;
                      endcase
                default: case (mcol)
                          3'd0: mcode = C_SHOU;
                          3'd1: mcode = C_DONG;
                          3'd2: mcode = C_LIU;
                          3'd3: mcode = C_LAN;
                          default: mcode = C_SPACE;
                      endcase
            endcase
        end else if (in_hint) begin
            // 「2 选 择     3 设 置」：KEY_2 选择、KEY_3 设置（确认生效）
            case (mcol)
                3'd0: mcode = C_2;
                3'd1: mcode = C_XUAN;
                3'd2: mcode = C_ZE;
                3'd5: mcode = C_3;
                3'd6: mcode = C_SHE;
                3'd7: mcode = C_ZHI;
                default: mcode = C_SPACE;
            endcase
        end else if (in_cfg) begin
            // 「已 选 + 已确认档位名」：KEY_3 确认后立刻换字（可见反馈）
            case (mcol)
                3'd0: mcode = C_YI;
                3'd1: mcode = C_XUAN;
                3'd2: mcode = cfg_c2;
                3'd3: mcode = cfg_c3;
                3'd4: mcode = cfg_c4;
                3'd5: mcode = cfg_c5;
                default: mcode = C_SPACE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 乱码修复字形的行索引必须是"字形槽内偏移"，不能用面板内 y 的低 4 位
    //   病因：三个文字行 + 提示条的槽顶在"面板内 y"里都不是 16 的倍数
    //         （标题 24、选项 68/112/156/200、提示 300、提示条 14），
    //         而旧写法 my[3:0] / by[3:0] 取的是 (面板内 y) mod 16，
    //         于是每个 16x16 字形都被竖向环绕错位。
    //   正确写法：行 = (像素 y) - (该字形槽顶 y)，结果恒在 0..15。
    //   横向：列 = (像素 x) - (该字格左边界)，同样不能用 (像素 x) 的低 4 位。
    // 提示条文字居中后字格起点 104 不是 16 的倍数，这条尤其关键
    //-------------------------------------------------------------------------
    wire [3:0] trow  = (my - M_TITLE_Y) & 12'h00F;  // 标题：槽顶 M_TITLE_Y
    wire [3:0] irow  = item_sub12[3:0];             // 选项：in_item 已限定 item_sub12<16
    wire [3:0] hrow  = (my - M_HINT_Y)  & 12'h00F;  // 提示行：槽顶 M_HINT_Y
    wire [3:0] crow  = (my - M_CFG_Y)   & 12'h00F;  // 状态行：槽顶 M_CFG_Y

    wire [3:0] mcolp = (mx - M_TXT_X) & 12'h00F;    // 字格内 x 偏移

    //-------------------------------------------------------------------------
    // 停止提示条：两条文案共用同一个面板，按 hold_auto 选择
    //   停止：「自 动 轮 播 已 停 止」7 字，起点 104
    //   轮播：「自 动 轮 播 中」      5 字，起点 120（各自居中，不是统一左对齐）
    //-------------------------------------------------------------------------
    wire [11:0] b_tx = hold_auto ? B_TXT_X5 : B_TXT_X;
    wire [11:0] b_tw = hold_auto ? 12'd80   : 12'd112;

    wire in_banner_box = show_banner &&
                         (x_cnt >= BX0) && (x_cnt < BX0 + BW) &&
                         (y_cnt >= BY0) && (y_cnt < BY0 + BH);

    wire banner_border = in_banner_box && ((x_cnt < BX0 + 12'd2) ||
                                           (x_cnt >= BX0 + BW - 12'd2) ||
                                           (y_cnt < BY0 + 12'd2) ||
                                           (y_cnt >= BY0 + BH - 12'd2));

    wire [11:0] bx = x_cnt - BX0;
    wire [11:0] by = y_cnt - BY0;

    // 提示条左侧金色强调条（与菜单选中行同款，统一两个面板的视觉语言）
    wire in_banner_bar = in_banner_box && (bx < B_BAR_W);

    // 最外 1px 暗轮廓（与菜单同款双描边；左边那条被金条覆盖，属预期）
    wire banner_edge = in_banner_box && ((bx == 12'd0) || (bx == BW - 12'd1) ||
                                         (by == 12'd0) || (by == BH - 12'd1));

    // 提示条文字的行/列索引：槽顶 B_TXT_Y、格左 b_tx（见"乱码修复"）
    wire [3:0] brow  = (by - B_TXT_Y) & 12'h00F;
    wire [3:0] bcolp = (bx - b_tx) & 12'h00F;

    wire in_banner_txt = in_banner_box &&
                         (bx >= b_tx) && (bx < b_tx + b_tw) &&
                         (by >= B_TXT_Y) && (by < B_TXT_Y + 12'd16);

    wire [2:0] bcol = (bx - b_tx) >> 4;
    reg  [5:0] bcode;
    always @(*) begin
        if (hold_auto) begin
            // 「自 动 轮 播 中」：5 字（字库里"中"= code 54，现成的）
            case (bcol)
                3'd0: bcode = C_ZI;
                3'd1: bcode = C_DONG;
                3'd2: bcode = C_LUN;
                3'd3: bcode = C_BO;
                3'd4: bcode = C_ZHONG;
                default: bcode = C_SPACE;
            endcase
        end else begin
            // 「自 动 轮 播 已 停 止」：7 字
            case (bcol)
                3'd0: bcode = C_ZI;
                3'd1: bcode = C_DONG;
                3'd2: bcode = C_LUN;
                3'd3: bcode = C_BO;
                3'd4: bcode = C_YI;
                3'd5: bcode = C_TING;
                3'd6: bcode = C_ZHI2;
                default: bcode = C_SPACE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 字库 ROM（与 osd_overlay 各自独立实例；两面板位置不重叠，
    // 同一像素只可能命中一个，故共用一套 (code,row,col) 选择）
    //-------------------------------------------------------------------------
    wire [5:0] fcode = in_banner_txt ? bcode : mcode;
    wire [3:0] frow  = in_banner_txt ? brow :
                       (in_title ? trow : (in_item ? irow :
                       (in_cfg ? crow : hrow)));
    wire [3:0] fcol  = in_banner_txt ? bcolp : mcolp;

    //-------------------------------------------------------------------------
    // 不再内部例化 osd_font_rom，改为把 (code,row) 送出顶层共享查表
    //   原来是 u_menu_font，占 364 LUT；与 osd_overlay 的那份重复。
    //   共享后全片只保留一份字库，多层按优先级 mux 输入。
    //-------------------------------------------------------------------------
    assign O_fcode = fcode;
    assign O_frow  = frow;
    assign O_freq  = in_title || in_item || in_hint || in_cfg || in_banner_txt;

    wire fpix = I_fbits[4'd15 - fcol];

    wire menu_txt_px   = in_menu && (in_title || in_item || in_hint || in_cfg) && fpix;
    wire banner_txt_px = in_banner_txt && fpix;

    //-------------------------------------------------------------------------
    // 文字颜色（未选中项降灰：原版 4 项全白，只能靠底色区分选中态）
    //-------------------------------------------------------------------------
    reg [23:0] txt_col;
    always @(*) begin
        if (in_item && (item_row != I_menu_sel))
            txt_col = C_DIM;          // 未选中选项
        else if (in_item)
            txt_col = C_TEXT;         // 选中选项
        else if (in_title)
            txt_col = C_TITLE;
        else if (in_hint)
            txt_col = C_HINT;
        else if (in_cfg)
            txt_col = C_CFG;
        else
            txt_col = C_TEXT;
    end

    //-------------------------------------------------------------------------
    // 面板底：暗色遮罩 = 固定暗基色 + 背景/8
    //
    // 不能写成 {1'b0, I_rgb[23:1]}
    //      那是整个 24 位右移 1 位，会跨字节串色：
    //      纯绿 (00,FF,00) 会变成 (00,7F,80)，蓝色凭空多出 0x80。
    //      必须每个通道分别取高位。
    //
    // 为什么从 >>1（50%）改成 >>3 + 基色（87.5%）
    //      50% 遮罩下，纯白图片上面板底是 (127,127,127)，
    //      此时次要文字 #788091 对比度只有 1.01:1（等于看不见）、
    //      金色标题 2.22:1、正文 4.00:1 —— 全部不合格。
    //      87.5% 后最亮面板底 (43,46,53)，四个色分别升到 5.35 / 7.97 / 11.70。
    //      保留 1/8 的背景亮度，所以面板仍是"暗玻璃"而不是死板纯色块，
    //      底下图片的明暗结构依然可辨（不遮挡图像构图，只压暗）。
    //      实测数据：tools/contrast_audit.py
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
        if (banner_txt_px)
            mix = C_TEXT;
        else if (in_banner_bar)
            mix = C_ACCENT;                       // 提示条金色左条
        else if (banner_edge)
            mix = C_OUTLINE;                      // 提示条最外 1px 暗轮廓
        else if (in_banner_box && !banner_border)
            mix = panel_bg;                       // 提示条：与菜单同款暗玻璃底
        else if (banner_border)
            mix = C_BORDER;
        else if (menu_txt_px)
            mix = txt_col;
        else if (in_sel_bar)
            mix = C_ACCENT;                       // 选中行金色左条
        else if (in_sel_row)
            mix = C_SEL_BG;
        else if (in_cfg_bg)
            mix = C_CFG_BG;
        else if (in_title_line)
            mix = C_BORDER_LO;
        else if (menu_edge)
            mix = C_OUTLINE;                      // 菜单最外 1px 暗轮廓
        else if (menu_border)
            mix = C_BORDER;
        else if (in_border_lo)
            mix = C_BORDER_LO;
        else if (in_menu)
            mix = panel_bg;
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
