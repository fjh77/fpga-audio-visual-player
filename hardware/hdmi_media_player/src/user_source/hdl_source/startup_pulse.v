//=============================================================================
// startup_pulse.v —— 复位释放后的单拍启动脉冲
//
// 用途
// ----
//   复位是异步的，释放时刻各触发器并不同步；用本模块产生一个**干净的单拍**
//   作为"可以开始工作"的起点，避免多个模块各自猜"复位好了没有"。
//
// 时序
// ----
//   复位期间 S_cnt 清零；释放后开始计数，数满 CNT_MAX 拍时 O_pulse 拉高**一拍**，
//   随后 S_done 置位、计数器停止（S_cnt 停在终值，不再自增）。
//   O_pulse 由寄存器直接输出，无毛刺。
//
// 参数
// ----
//   CNT_MAX —— 延时拍数，默认 100,000。25 MHz 下约 4 ms，足够让 PLL 锁定、
//              SDRAM 上电时序走完、外部器件稳定。
//              ★ 改了时钟频率就要按比例改这个值，否则实际延时长度会变。
//
// 使用约束
// --------
//   只产生一次脉冲：一旦 S_done 置位，后续不会再触发。需要"可重复触发"的场合
//   请另写启动检测逻辑——本模块的语义就是"上电一次"。
//=============================================================================

module startup_pulse #(
    parameter [19:0] CNT_MAX = 20'd100000    // 延时拍数（25MHz 下约 4ms）
)(
    input  wire I_clk,
    input  wire I_rst,
    output reg  O_pulse                       // 单拍高电平，仅在计数满时出现一次
);

    reg [19:0] S_cnt;                         // 延时计数（数满后保持，不再自增）
    reg        S_done;                        // 1 = 已产生过脉冲，锁定不再触发

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_cnt   <= 20'd0;
            S_done  <= 1'b0;
            O_pulse <= 1'b0;
        end
        else begin
            O_pulse <= 1'b0;                  // 默认拉低，保证只有一拍

            if (!S_done) begin
                if (S_cnt == CNT_MAX - 1'b1) begin
                    S_cnt   <= S_cnt;         // 停在终值，省掉后续无用的翻转
                    S_done  <= 1'b1;
                    O_pulse <= 1'b1;
                end
                else begin
                    S_cnt <= S_cnt + 1'b1;
                end
            end
        end
    end

endmodule
