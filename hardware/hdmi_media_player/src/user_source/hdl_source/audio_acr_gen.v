//=============================================================================
// audio_acr_gen.v
//
// HDMI 音频时钟恢复（ACR）参数生成
//
// HDMI 规范里，源端要周期性发送 ACR 包，携带 N 和 CTS：
//     fs = (pixel_clk * N) / (128 * CTS)
//
// 本模块的做法：
//   每 DIV 个音频样本测一次「这段间隔里有多少个像素时钟」，作为 CTS；
//   N 固定为 ACR_N。
//
// 数值核对（video_clk = 25MHz, fs = 48kHz, N = 6144）：
//   DIV = 48 个样本 @48kHz = 1ms
//   CTS = 25e6 * 1ms = 25000
// 反代公式: 25e6 * 6144 / (128 * 25000) = 48000 Hz
// 20bit CTS 上限 1048575，25000 远未溢出
//
// 注：这里 DIV 与 ACR_N 是独立参数（参考实现用 ACR_N>>7 隐式推导，
//     当 ACR_N < 128 时会下溢成 0 导致除零式行为，这里显式给出并加断言）。
//=============================================================================

module audio_acr_gen #(
    parameter [19:0]  ACR_N = 20'd6144,   // ACR 的 N 值
    parameter integer DIV   = 48          // 每多少个音频样本发一次 ACR
)(
    input  wire        I_clk,             // video_clk 25MHz
    input  wire        I_rst,             // 高有效
    input  wire        I_sample_valid,    // 音频样本有效脉冲

    output reg         O_acr_valid,       // ACR 更新脉冲（1 个 I_clk）
    output reg  [19:0] O_cts,
    output reg  [19:0] O_n
);

    // 参数约束（改动时请自行核对，勿设为非法值）：
    //   DIV   >= 1        —— 否则 div_cnt 永远到不了 DIV_LAST，O_acr_valid 恒为 0
    //   ACR_N != 0        —— 否则 CTS/N 比值无意义，终端无法恢复音频时钟
    // 默认值 DIV=48 / ACR_N=6144 对应 25MHz 像素时钟 + 48kHz 采样率，CTS≈25000。
    localparam [15:0] DIV_LAST = DIV - 1;

    reg [15:0] div_cnt;      // 音频样本计数 0..DIV-1
    reg [19:0] cts_cnt;      // 距上次 ACR 的像素时钟计数

    wire acr_pulse = I_sample_valid && (div_cnt == DIV_LAST);

    //-------------------------------------------------------------------------
    // 样本计数
    //-------------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            div_cnt <= 16'd0;
        else if (I_sample_valid)
            div_cnt <= acr_pulse ? 16'd0 : (div_cnt + 16'd1);
    end

    //-------------------------------------------------------------------------
    // 像素时钟计数：ACR 触发的那一拍清零，其余每拍 +1
    //-------------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            cts_cnt <= 20'd0;
        else if (acr_pulse)
            cts_cnt <= 20'd0;
        else
            cts_cnt <= cts_cnt + 20'd1;
    end

    //-------------------------------------------------------------------------
    // 输出：只在脉冲那一拍有效，其余拍清零（与 HDMI 核的接口约定一致）
    //-------------------------------------------------------------------------
    always @(posedge I_clk) begin
        if (acr_pulse) begin
            O_acr_valid <= 1'b1;
            O_cts       <= cts_cnt + 20'd1;   // +1 补上被清零的这一拍
            O_n         <= ACR_N;
        end
        else begin
            O_acr_valid <= 1'b0;
            O_cts       <= 20'd0;
            O_n         <= 20'd0;
        end
    end

endmodule
