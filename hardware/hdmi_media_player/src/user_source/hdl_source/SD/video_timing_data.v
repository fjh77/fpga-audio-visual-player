module video_timing_data
#(
	parameter DATA_WIDTH = 16                       // Video data one clock data width
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output reg                  read_req,           // Start reading a frame of data     
	input                       read_req_ack,       // Read request response
	//output                      read_en,            // Read data enable
	//input[DATA_WIDTH - 1:0]     read_data,          // Read data
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	output                      de                  // video valid
	//output[DATA_WIDTH - 1:0]    vout_data           // video data
);
wire                   video_hs;
wire                   video_vs;
wire                   video_de;
//delay video_hs video_vs  video_de 2 clock cycles
reg                    video_hs_d0;
reg                    video_vs_d0;
reg                    video_de_d0;
reg                    video_hs_d1;
reg                    video_vs_d1;
reg                    video_de_d1;

//reg[DATA_WIDTH - 1:0]  vout_data_r;
//assign read_en = video_de;
assign hs = video_hs_d1;
assign vs = video_vs_d1;
assign de = video_de_d1;
//assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		video_hs_d0 <= 1'b0;
		video_vs_d0 <= 1'b0;
		video_de_d0 <= 1'b0;
	end
	else
	begin
		//delay video_hs video_vs  video_de 2 clock cycles
		video_hs_d0 <= video_hs;
		video_vs_d0 <= video_vs;
		video_de_d0 <= video_de;
		video_hs_d1 <= video_hs_d0;
		video_vs_d1 <= video_vs_d0;
		video_de_d1 <= video_de_d0;		
	end
end
/*
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(video_de_d0)
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
*/
// ---------------------------------------------------------------------------
// CURL16：读请求必须等 vs上升沿之后再发（修"卷轴下方一直闪上一张图"）
// ---------------------------------------------------------------------------
//   CURL15 把 read_req 从"vs 沿当拍"延后了 48 拍，理由是"几何发布要 25 拍"。
//   这个 25 拍的数字没错，但**盯错了沿**：video_transition 的 pub_tick = vs_edge，
//   两个沿都发布，而这两个沿发布的值不一样。
//
//   真实 color_bar（VIDEO_640_480，VS_POL=0，负极性同步脉冲）的竖向结构是
//   （v_cnt 以 h_cnt==H_FP-1 为行边界递增）：
//       v_cnt  0..8   前肩 V_FP=10
//       v_cnt  9..10  同步脉冲（vs=0）   ← 下降沿在 v=9，上升沿在 v=11
//       v_cnt 11..43  后肩 V_BP=33
//       v_cnt 44..523 有效行
//   ⇒ 每一帧的时序是  **下降沿 → 上升沿（隔 2 行 = 1600 拍）→ 有效行（隔 33 行）**
//
//   而 video_transition 的状态机是在下降沿那一拍才 prog/chold 步进，
//   发布块与它同拍采样（非阻塞赋值，采到的是步进前的值）⇒
//       · 下降沿发布 = 上一帧的几何
//       · 上升沿发布 = 步进后的几何 = 本帧真正扫描用的几何
//   video_curl 在有效行里用的是寄存器当前值，也就是上升沿那一份。
//
//   ⇒ 读侧只有在上升沿发布之后锁存，才能和着色器同源。
//     下降沿 + 48 拍根本到不了上升沿（还差 1600-48 拍），锁到的是上一帧，
//     curl_split 单调递减 ⇒ split_读 > split_着色，两者之间那一条
//     （起手最多 18 行、末尾 2 行）读的是旧图基址= 卷轴下方露出旧图。
//
//   CURL15 为什么会算成 48：sim_curl\c15\tb_align.v 把 vs 写成
//     `wire vs_raw = (v < VS_LEN);` —— 同步脉冲在帧首拉高，
//     等于把极性写反，变成"先上升沿、后下降沿"，于是任何一个晚于下降沿
//     发布的锁存点都显得"对齐"。按真实负极性重做的 tb_align2.v 输出：
//       D=0/24/48/80 : 错位 138/139 帧（D=48 正是 CURL15 的取值）
//       D=104/128    : 错位 0 帧
//     压缩行下 2 行 = 80 拍，80+48 = 128 就是"上升沿 + 48 拍"。
//
//   修法：把沿判据从下降沿换成上升沿，REQ_DLY_CYC 仍取 48。
//   上升沿之后到第一个有效行还有 33 行（约 1.05 ms），推迟 1.92us 无影响；
//   FIFO 清空与起读仍远早于显示取数。
// ---------------------------------------------------------------------------
localparam [7:0] REQ_DLY_CYC = 8'd48;
reg [7:0] req_dly_cnt;
reg       req_dly_run;

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		req_dly_cnt <= 8'd0;
		req_dly_run <= 1'b0;
		read_req    <= 1'b0;
	end
	else
	begin
		// CURL16：用上升沿（同步脉冲结束），此时几何已按本帧发布
		if(~video_vs_d0 & video_vs) //vertical synchronization rising edge
		begin
			req_dly_run <= 1'b1;
			req_dly_cnt <= 8'd0;
		end
		else if(req_dly_run)
		begin
			if(req_dly_cnt == REQ_DLY_CYC - 8'd1)
				req_dly_run <= 1'b0;
			else
				req_dly_cnt <= req_dly_cnt + 8'd1;
		end

		// 延迟到点才发请求；一直保持到 ack（与原握手语义完全一致）
		if(read_req_ack)
			read_req <= 1'b0;
		else if(req_dly_run && (req_dly_cnt == REQ_DLY_CYC - 8'd1))
			read_req <= 1'b1;
	end
end

color_bar color_bar_m0(
	.clk(video_clk),
	.rst(rst),
	.hs(video_hs),
	.vs(video_vs),
	.de(video_de),
	.rgb_r(),
	.rgb_g(),
	.rgb_b()
);
endmodule 