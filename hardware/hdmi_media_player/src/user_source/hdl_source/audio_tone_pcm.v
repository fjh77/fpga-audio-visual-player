//=============================================================================
// audio_tone_pcm.v
//
// 直接 PCM 音调生成器（不使用额外的音频 PLL）
//
// 设计要点 —— 全部在 video_clk(25MHz) 域内完成：
//   1. 32 位定点累加器做 48000Hz 采样率分频
//      INC = round(48000 * 2^32 / 25e6) = 8246337
//      -> 实际采样率 47999.9988 Hz，误差 -0.025 ppm
//   2. DDS 相位累加器产生音调，取相位高 6 位查 64 点正弦表
//      音调选 750Hz：48000/750 = 64，每个采样点恰好走 LUT 一项，
//      TONE_INC = 750*2^32/48000 = 2^26 = 67108864（整数）
//      -> 数学上无相位量化误差，输出是精确正弦
//   3. 响/停 门控：默认响 0.5s、停 0.5s（1Hz 节拍），便于用耳朵确认
//
// 与参考例程 lab_ex5_i2s 的区别：
//   参考实现是「内部生成 I2S 方波 -> I2S_receiver 再解码回 PCM」，
//   本模块直接产生 PCM 样本，省掉 I2S 收发两级，也不引入音频 PLL。
//
// 上电即响：只受 I_rst 控制，不等图片扫描完成（扫描要约 25 秒）。
//
// 注：本模块刻意不使用 Verilog 的 signed 类型与负数字面量，
//     正弦负半周直接写成 24 位二进制补码值，与工程内其他模块风格一致。
//=============================================================================

module audio_tone_pcm #(
    parameter integer CLK_HZ      = 25_000_000,      // video_clk 频率
    parameter integer SAMPLE_RATE = 48_000,          // 目标采样率
    parameter [23:0]  AMP         = 24'd7_000_000,   // 约 -1.6 dBFS
    parameter integer ON_SAMPLES  = 24_000,          // 响 0.5s @48kHz
    parameter integer OFF_SAMPLES = 24_000           // 停 0.5s @48kHz
)(
    input  wire        I_clk,        // video_clk 25MHz
    input  wire        I_rst,        // 高有效

    output reg         O_valid,      // 每个音频样本脉冲一拍
    output reg  [23:0] O_left,       // 左声道 PCM（二进制补码）
    output reg  [23:0] O_right       // 右声道 PCM（二进制补码）
);

    // round(48000 * 2^32 / 25000000) = 8246337
    localparam [31:0] SAMPLE_INC = 32'd8246337;
    // 750Hz -> 2^26（整数，保证精确遍历 64 点 LUT）
    localparam [31:0] TONE_INC   = 32'd67108864;

    localparam [23:0] ON_CNT  = ON_SAMPLES  - 1;
    localparam [23:0] OFF_CNT = OFF_SAMPLES - 1;

    //-------------------------------------------------------------------------
    // 64 点正弦表：round(sin(2*pi*k/64) * AMP)，负值用 24 位补码表示
    //   例：k=48 时 sin=-1.0 -> -7000000 -> 补码 16777216-7000000 = 9777216
    //-------------------------------------------------------------------------
    function [23:0] sine_lut;
        input [5:0] idx;
        begin
            case (idx)
                6'd0 : sine_lut = 24'd0;
                6'd1 : sine_lut = 24'd686120;
                6'd2 : sine_lut = 24'd1365632;
                6'd3 : sine_lut = 24'd2031993;
                6'd4 : sine_lut = 24'd2678784;
                6'd5 : sine_lut = 24'd3299777;
                6'd6 : sine_lut = 24'd3888992;
                6'd7 : sine_lut = 24'd4440753;
                6'd8 : sine_lut = 24'd4949747;
                6'd9 : sine_lut = 24'd5411073;
                6'd10: sine_lut = 24'd5820287;
                6'd11: sine_lut = 24'd6173449;
                6'd12: sine_lut = 24'd6467157;
                6'd13: sine_lut = 24'd6698582;
                6'd14: sine_lut = 24'd6865497;
                6'd15: sine_lut = 24'd6966293;
                6'd16: sine_lut = 24'd7000000;
                6'd17: sine_lut = 24'd6966293;
                6'd18: sine_lut = 24'd6865497;
                6'd19: sine_lut = 24'd6698582;
                6'd20: sine_lut = 24'd6467157;
                6'd21: sine_lut = 24'd6173449;
                6'd22: sine_lut = 24'd5820287;
                6'd23: sine_lut = 24'd5411073;
                6'd24: sine_lut = 24'd4949747;
                6'd25: sine_lut = 24'd4440753;
                6'd26: sine_lut = 24'd3888992;
                6'd27: sine_lut = 24'd3299777;
                6'd28: sine_lut = 24'd2678784;
                6'd29: sine_lut = 24'd2031993;
                6'd30: sine_lut = 24'd1365632;
                6'd31: sine_lut = 24'd686120;
                6'd32: sine_lut = 24'd0;
                6'd33: sine_lut = 24'd16091096;
                6'd34: sine_lut = 24'd15411584;
                6'd35: sine_lut = 24'd14745223;
                6'd36: sine_lut = 24'd14098432;
                6'd37: sine_lut = 24'd13477439;
                6'd38: sine_lut = 24'd12888224;
                6'd39: sine_lut = 24'd12336463;
                6'd40: sine_lut = 24'd11827469;
                6'd41: sine_lut = 24'd11366143;
                6'd42: sine_lut = 24'd10956929;
                6'd43: sine_lut = 24'd10603767;
                6'd44: sine_lut = 24'd10310059;
                6'd45: sine_lut = 24'd10078634;
                6'd46: sine_lut = 24'd9911719;
                6'd47: sine_lut = 24'd9810923;
                6'd48: sine_lut = 24'd9777216;
                6'd49: sine_lut = 24'd9810923;
                6'd50: sine_lut = 24'd9911719;
                6'd51: sine_lut = 24'd10078634;
                6'd52: sine_lut = 24'd10310059;
                6'd53: sine_lut = 24'd10603767;
                6'd54: sine_lut = 24'd10956929;
                6'd55: sine_lut = 24'd11366143;
                6'd56: sine_lut = 24'd11827469;
                6'd57: sine_lut = 24'd12336463;
                6'd58: sine_lut = 24'd12888224;
                6'd59: sine_lut = 24'd13477439;
                6'd60: sine_lut = 24'd14098432;
                6'd61: sine_lut = 24'd14745223;
                6'd62: sine_lut = 24'd15411584;
                default: sine_lut = 24'd16091096;   // 6'd63
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    reg [31:0] sample_acc;   // 采样率累加器（进位即出一个样本）
    reg [31:0] phase_acc;    // DDS 相位累加器
    reg [23:0] hold_cnt;     // 响/停 计时（样本数）
    reg        gate_on;      // 1 = 正在发声

    // 33 位加法，用最高位判断溢出（不使用 > 比较，避免边界歧义）
    wire [32:0] sample_acc_next = {1'b0, sample_acc} + {1'b0, SAMPLE_INC};
    wire        sample_tick     = sample_acc_next[32];

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            sample_acc <= 32'd0;
            phase_acc  <= 32'd0;
            hold_cnt   <= 24'd0;
            gate_on    <= 1'b1;
            O_valid    <= 1'b0;
            O_left     <= 24'd0;
            O_right    <= 24'd0;
        end
        else begin
            O_valid    <= 1'b0;
            sample_acc <= sample_acc_next[31:0];

            if (sample_tick) begin
                O_valid   <= 1'b1;
                phase_acc <= phase_acc + TONE_INC;

                // 门控：停的时候输出静音（全 0），而不是继续发声
                if (gate_on) begin
                    O_left  <= sine_lut(phase_acc[31:26]);
                    O_right <= sine_lut(phase_acc[31:26]);
                end
                else begin
                    O_left  <= 24'd0;
                    O_right <= 24'd0;
                end

                // 响/停 节拍
                if (hold_cnt >= (gate_on ? ON_CNT : OFF_CNT)) begin
                    hold_cnt <= 24'd0;
                    gate_on  <= ~gate_on;
                end
                else begin
                    hold_cnt <= hold_cnt + 24'd1;
                end
            end
        end
    end

endmodule
