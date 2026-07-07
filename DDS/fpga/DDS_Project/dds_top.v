// ============================================================
// 文件名: dds_top.v
// 功能: DDS信号发生器 (所有模块集成在一个文件)
// 硬件: EP4CE6F17C8, 50MHz时钟, 14位并行DAC
// 修正: 修复向量越界错误, 简化FTW_STEP定义
// ============================================================
module dds_top (
    input           clk,            // 50MHz 系统时钟
    input           rst_n,          // 复位，低有效 (接按键)
    input  [1:0]    mode_sel,       // 模式: 00=正弦波,01=AM,10=FM,11=键控
    input           sub_mode,       // 键控子模式: 0=PSK, 1=ASK
    input           freq_inc,       // 频率增加 (按键，上升沿有效)
    input           freq_dec,       // 频率减小 (按键)
    input           fm_dev_sel,     // FM频偏选择: 0=5kHz, 1=10kHz
    output reg [13:0] dac_data,     // 14位数据总线 (连接DAC的 B1~B14)
    output          dac_clk         // DAC时钟 (直接输出系统时钟)
);

// ========== 参数定义 ==========
localparam CLK_FREQ = 50_000_000;
localparam PHASE_WIDTH = 32;
localparam ADDR_WIDTH = 10;
localparam ROM_DEPTH = 1024;

// 修正1: 使用近似值，每100Hz对应的FTW = 100 * 2^32 / 50MHz ≈ 8589.93，取整8590
localparam FTW_STEP = 32'd8590;     

// 频率控制
reg  [31:0] freq_code;              // 频率代码 (单位: 100Hz, 范围 1~100000)
wire [31:0] FTW;

// 相位累加器
reg  [PHASE_WIDTH-1:0] phase_acc;
wire [ADDR_WIDTH-1:0] rom_addr;

// 正弦波ROM (数组)
reg [13:0] sin_rom [0:ROM_DEPTH-1];
wire [13:0] sine_data;

// AM调制相关
wire [13:0] mod_sin;                // 1kHz 调制信号
wire [13:0] am_carrier;             // AM载波
reg  [ 3:0] ma_index;               // 调制度索引 (0~10)
wire [13:0] ma_coeff;
wire [27:0] am_product;
wire [13:0] am_out;

// FM调制相关
wire [13:0] fm_mod_sin;             // 同 mod_sin
wire [31:0] fc_ftw;
wire [31:0] delta_f_ftw;
wire [13:0] offset;
wire [31:0] ftw_fm;
wire [13:0] fm_out;

// PSK/ASK 相关
wire        baseband;               // 10kbps 基带序列
wire [13:0] psk_raw;
wire [13:0] psk_out;
wire [13:0] ask_out;

// 模式选择输出
reg  [13:0] dac_data_reg;

// ========== 正弦波ROM初始化 ==========
initial begin
    $readmemh("sine_14bit_1024.hex", sin_rom);
end
assign sine_data = sin_rom[rom_addr];

// ========== 频率控制字生成 ==========
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        freq_code <= 32'd1;
    else begin
        if (freq_inc && (freq_code < 32'd100000))
            freq_code <= freq_code + 1;
        else if (freq_dec && (freq_code > 32'd1))
            freq_code <= freq_code - 1;
    end
end
assign FTW = freq_code * FTW_STEP;

// ========== 相位累加器 ==========
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phase_acc <= 0;
    else
        phase_acc <= phase_acc + FTW;
end
assign rom_addr = phase_acc[PHASE_WIDTH-1 : PHASE_WIDTH-ADDR_WIDTH];

// ========== 1kHz 调制信号产生 (独立DDS) ==========
reg [31:0] phase_mod1k;
wire [ADDR_WIDTH-1:0] rom_addr_mod;
assign rom_addr_mod = phase_mod1k[31:22];
reg [13:0] mod_rom [0:1023];
initial $readmemh("sine_14bit_1024.hex", mod_rom);
assign mod_sin = mod_rom[rom_addr_mod];
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phase_mod1k <= 0;
    else
        phase_mod1k <= phase_mod1k + 32'd8590;   // 1kHz 对应FTW
end

// ========== AM调制 (载波1MHz, 可扩展) ==========
localparam FTW_1M = 32'd85899346;
reg [31:0] phase_carrier_am;
wire [13:0] am_carrier_data;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phase_carrier_am <= 0;
    else
        phase_carrier_am <= phase_carrier_am + FTW_1M;
end
assign am_carrier = sin_rom[phase_carrier_am[31:22]];

// 调制度 ma (10%~100%，步进10%)，这里固定为100%以便测试，可后续扩展按键
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ma_index <= 4'd10;   // 100%
    else begin
        // 可以增加 ma_inc/ma_dec 按键，暂不实现
        ma_index <= ma_index;  // 防止latch
    end
end
assign ma_coeff = (ma_index == 4'd0) ? 14'd0 : (ma_index * 14'd1638);
wire [27:0] scaled_mod = ma_coeff * mod_sin;
wire [27:0] bias_plus_mod = scaled_mod + 28'd8192;
wire [27:0] am_mult = bias_plus_mod[13:0] * am_carrier;
assign am_out = am_mult[27:14];

// ========== FM调制 (中心频率可调，频偏可切换) ==========
// 中心频率使用 freq_code 控制的 DDS 输出作为载波频率（范围100Hz~10MHz）
// 但 FM 要求载波 100kHz~10MHz，我们限制 freq_code 最小1000 (100kHz)
wire [31:0] fc_ftw_tmp = (freq_code < 32'd1000) ? 32'd858993 : freq_code * FTW_STEP;
assign fc_ftw = fc_ftw_tmp;
localparam FTW_5K = 32'd429500;
localparam FTW_10K = 32'd859000;
assign delta_f_ftw = fm_dev_sel ? FTW_10K : FTW_5K;
assign offset = mod_sin - 14'd8192;   // 有符号偏移

// 修正2: 将乘积结果改为32位，避免索引越界
wire [31:0] ftw_delta_product = offset * delta_f_ftw[13:0];
wire [31:0] ftw_delta = ftw_delta_product;   // 直接赋值
assign ftw_fm = fc_ftw + ftw_delta;

reg [31:0] phase_fm;
wire [13:0] fm_sine;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phase_fm <= 0;
    else
        phase_fm <= phase_fm + ftw_fm;
end
assign fm_out = sin_rom[phase_fm[31:22]];

// ========== PSK/ASK 基带序列产生 (10kbps) ==========
reg [15:0] bit_cnt;
reg [ 7:0] prbs;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_cnt <= 0;
        prbs <= 8'b10101010;
    end else begin
        if (bit_cnt == 16'd4999) begin   // 100us = 5000个时钟周期 @50MHz
            bit_cnt <= 0;
            prbs <= {prbs[6:0], prbs[7] ^ prbs[5]};
        end else begin
            bit_cnt <= bit_cnt + 1;
        end
    end
end
assign baseband = prbs[0];

// PSK载波固定100kHz
localparam FTW_100K = 32'd858993;
reg [31:0] phase_psk;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phase_psk <= 0;
    else
        phase_psk <= phase_psk + FTW_100K;
end
assign psk_raw = sin_rom[phase_psk[31:22]];
assign psk_out = baseband ? psk_raw : (~psk_raw + 1'b1);
assign ask_out = baseband ? psk_raw : 14'd0;

// ========== 模式选择 ==========
always @(*) begin
    case (mode_sel)
        2'b00:   dac_data_reg = sine_data;
        2'b01:   dac_data_reg = am_out;
        2'b10:   dac_data_reg = fm_out;
        2'b11:   dac_data_reg = (sub_mode == 0) ? psk_out : ask_out;
        default: dac_data_reg = sine_data;
    endcase
end

always @(posedge clk) begin
    dac_data <= dac_data_reg;
end

assign dac_clk = clk;

endmodule