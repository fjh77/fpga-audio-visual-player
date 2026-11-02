`timescale 1ns / 1ps

//=============================================================================
// 文件名  : osd_audio_env.v
// 功能    : 音频包络前端 —— 只算 v[2]+v[3]，供滚动字幕的字形跳动使用
//
// 由来
// ----
//   取代 osd_audio_viz.v（柱状图 + 波形渲染，1,036 le）。字随音频跳动只需要
//   "一个响度数字"，不需要频谱图和波形，所以把整个渲染侧删掉，只保留前端。
//
// 为什么只剩 3 级低通
// -------------------
//   驱动信号取 v[2] + v[3]（120–480 Hz，本工程素材 67% 能量所在）：
//       段2 = lp2 − lp1        段3 = lp3 − lp2
//   只需要 lp1/lp2/lp3 三级。原版有 lp0..lp6 七级 + 8 段包络，
//   这里省掉 4 个 18 位低通寄存器（72 FF）与 5 个 15 位包络寄存器（75 FF）。
//
//   段 0/1（<120Hz）在本工程的语音素材里几乎没有能量，段 4~7（>480Hz）主要是
//   齿音，让字跟着齿音抖反而破坏律动感 —— 删掉它们不影响效果，只影响"能不能看频谱"。
//
// 时序（每样本 6 拍，48kHz 下 520 拍里只用 6 拍）
// -----------------------------------------------
//   拍 0..1  频段处理：段2、段3 用**更新前**的 lp 求差值 → 整流 → 包络
//   拍 2..4  低通更新：lp1 <<= (x−lp1)>>6、lp2 >>5、lp3 >>4
//   拍 5     收尾
//   ★ 必须"先算频段再更新低通"，与原版 / 参考模型逐位一致。
//
// 帧捕获（vs 上升沿后 2 拍）
// --------------------------
//   拍 0：v2 = log6(en2)
//   拍 1：O_level = v2 + log6(en3)
//   只有一份 log6 组合逻辑，2 拍复用（原版 8 段共用一份，这里更省）。
//
// 数值语义与 tools/viz_model.py 的 bands_raw() 逐位一致：
//   v = idx*4 + frac，idx 是包络的最高有效位序号，frac 是其下 2 位；v<32 时输出 0。
//   注意这里**不做倾斜补偿**（原版 8 段才需要把各段拉到同一量级，
//   驱动响度不需要；而且 tilt 会让 THR 门限失去意义）。
//=============================================================================

module osd_audio_env (
    input               I_clk,
    input               I_rst,

    input               I_svalid,       // 48kHz 单拍 = audio_valid_sel
    input      [23:0]   I_sample,       // 有符号 24 位 = audio_vol_left
    input               I_vs,           // video_clk 域场同步（帧捕获触发）

    output reg [6:0]    O_level,        // v[2]+v[3]，0..118（★ 必须 7 位，6 位会回绕）
    output reg [5:0]    O_v2,
    output reg [5:0]    O_v3
);

    //-------------------------------------------------------------------------
    // 运算函数（全模块各只有一份实例）
    //-------------------------------------------------------------------------
    // 18 位有符号算术右移
    //   ★ 不能用"逻辑右移 | 符号掩码"的手工写法：
    //     ~(18'h3FFFF >> s) 在 s=6 时得到 0x3FC00（只清低 10 位，应为 0x3F000），
    //     负数右移结果完全错误 —— 实测 lp 稳态偏离上千，包络随之系统性偏大。
    //     直接声明 signed 再用 >>> 即可，语义由工具保证。
    function [17:0] asr18;
        input [17:0] v;
        input integer s;
        reg signed [17:0] sv;
        begin
            sv = v;
            asr18 = sv >>> s;
        end
    endfunction

    // 18 位有符号 -> 15 位全波整流
    function [14:0] rect15;
        input [17:0] v;
        reg   [16:0] m;
        begin
            m = v[17] ? ((~{1'b0, v[16:0]}) + 17'd1) : {1'b0, v[16:0]};
            rect15 = (m > 17'd32767) ? 15'd32767 : m[14:0];
        end
    endfunction

    // 包络低通：上升按 1/512 逼近，下降至少 1
    //   ★ 衰减量必须 >= 1：原版写成 e - ((e-r)>>9)，当 e-r<512 时右移得 0，
    //     包络就停在原地永不下降（实测 2 秒内中位数=90 分位数，完全不动）。
    function [14:0] env_step;
        input [14:0] e;
        input [14:0] r;
        reg   [14:0] d;
        begin
            if (r > e)
                env_step = e + ((r - e) >> 9);
            else if (e == 15'd0)
                env_step = 15'd0;
            else begin
                d = (e - r) >> 9;
                env_step = (d == 15'd0) ? (e - 15'd1) : (e - d);
            end
        end
    endfunction

    // 15 位包络 -> 6 位对数：v = idx*4 + frac，idx = 最高有效位序号（>=5）
    function [5:0] log6;
        input [14:0] v;
        integer     i;
        reg   [3:0] ix;
        reg   [1:0] fr;
        begin
            ix = 4'd0;
            fr = 2'b00;
            for (i = 5; i <= 14; i = i + 1)
                if (v[i]) begin
                    ix = i[3:0];
                    fr = {v[i - 1], v[i - 2]};
                end
            log6 = {ix, fr};
        end
    endfunction

    //-------------------------------------------------------------------------
    // 输入
    //-------------------------------------------------------------------------
    wire [15:0] s16 = I_sample[23:8];
    wire [17:0] x18 = { {2{s16[15]}}, s16 };

    //-------------------------------------------------------------------------
    // 状态
    //-------------------------------------------------------------------------
    reg        busy;
    reg  [2:0] step;
    reg        a_pend;
    reg  [17:0] xs;

    reg [17:0] lp1, lp2, lp3;              // 三级低通（sh = 6 / 5 / 4）
    reg [14:0] en2, en3;                   // 两个频段的包络

    // 低通增量：拍 2 -> lp1（sh=6）、拍 3 -> lp2（sh=5）、拍 4 -> lp3（sh=4）
    //   ★ 不能用 step[1:0] 当级号：拍 2/3/4 时 step[1:0] 是 2/3/0，
    //     会让 lp2 永远不被更新（实测 lp2 恒 0、段 3 恒 0）。
    reg [17:0] lpsh;
    always @(*) begin
        if (step == 3'd2)      lpsh = asr18(xs - lp1, 6);
        else if (step == 3'd3) lpsh = asr18(xs - lp2, 5);
        else                   lpsh = asr18(xs - lp3, 4);
    end

    // 频段处理（拍 0 = 段2，拍 1 = 段3）
    //   段2 = lp2 − lp1，段3 = lp3 − lp2（都用**更新前**的值）
    wire [17:0] bp = (step[1:0] == 2'd0) ? (lp2 - lp1) : (lp3 - lp2);
    wire [14:0] r_a = rect15(bp);

    reg [14:0] en_cur;
    always @(*) begin
        en_cur = (step[1:0] == 2'd1) ? en3 : en2;
    end

    wire [14:0] en_new = env_step(en_cur, r_a);

    //-------------------------------------------------------------------------
    // 视频同步（只用于帧捕获触发）
    //-------------------------------------------------------------------------
    reg  vs_d;
    wire vs_rise = I_vs & ~vs_d;

    //-------------------------------------------------------------------------
    // 帧捕获：vs 上升沿后 2 拍，共用一份 log6
    //-------------------------------------------------------------------------
    reg [1:0] cap;
    reg       cap_run;
    reg [5:0] v2_r, v3_r;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            cap     <= 2'd0;
            cap_run <= 1'b0;
            v2_r    <= 6'd0;
            v3_r    <= 6'd0;
            O_v2    <= 6'd0;
            O_v3    <= 6'd0;
            O_level <= 6'd0;
        end else begin
            vs_d <= I_vs;
            if (vs_rise) begin
                cap_run <= 1'b1;
                cap     <= 2'd0;
            end else if (cap_run) begin
                cap <= cap + 2'd1;
                case (cap)
                2'd0: v2_r <= log6(en2);
                2'd1: v3_r <= log6(en3);
                2'd2: begin
                    O_v2    <= v2_r;
                    O_v3    <= v3_r;
                    O_level <= {1'b0, v2_r} + {1'b0, v3_r};
                    cap_run <= 1'b0;
                end
                default: cap_run <= 1'b0;
                endcase
            end
        end
    end

    //-------------------------------------------------------------------------
    // 时分复用引擎：每样本 6 拍
    //-------------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            busy   <= 1'b0;
            step   <= 3'd0;
            a_pend <= 1'b0;
            xs     <= 18'd0;
            lp1    <= 18'd0;
            lp2    <= 18'd0;
            lp3    <= 18'd0;
            en2    <= 15'd0;
            en3    <= 15'd0;
        end else begin
            if (I_svalid) begin
                xs     <= x18;
                a_pend <= 1'b1;
            end

            if (!busy) begin
                if (a_pend) begin
                    a_pend <= 1'b0;
                    busy   <= 1'b1;
                    step   <= 3'd0;
                end
            end else begin
                step <= step + 3'd1;
                if (step <= 3'd1) begin
                    // 拍 0..1：频段 -> 包络
                    if (step == 3'd0) en2 <= en_new;
                    else              en3 <= en_new;
                end else if (step <= 3'd4) begin
                    // 拍 2..4：低通更新（用 step 原值判断，不能用 step[1:0]）
                    if (step == 3'd2)      lp1 <= lp1 + lpsh;
                    else if (step == 3'd3) lp2 <= lp2 + lpsh;
                    else                   lp3 <= lp3 + lpsh;
                end else begin
                    // 拍 5：收尾
                    busy <= 1'b0;
                    step <= 3'd0;
                end
            end
        end
    end

endmodule
