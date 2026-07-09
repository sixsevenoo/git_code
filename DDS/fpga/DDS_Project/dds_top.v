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
    output          dac_clk,        // DAC时钟 (直接输出系统时钟)
    output          uart_txd,       // UART发送到PC (接USB转串口, 115200bps)
    input           uart_rxd        // 3519 UART接收 (控制命令输入)
);

// ========== 参数定义 ==========
localparam CLK_FREQ = 50_000_000;
localparam PHASE_WIDTH = 32;
localparam ADDR_WIDTH = 10;
localparam ROM_DEPTH = 1024;

// 修正1: 使用近似值，每100Hz对应的FTW = 100 * 2^32 / 50MHz ≈ 8589.93，取整8590
localparam FTW_STEP = 32'd8590;

// ========== UART RX 命令解码 (3519→FPGA) ==========
// 命令字节: 'I'(inc) 'D'(dec) 'M'(mode) 'S'(sub) 'F'(fm) 'R'(reset)

wire [7:0] uart_cmd_byte;
wire       uart_cmd_valid;

reg        uart_cmd_inc_pulse;
reg        uart_cmd_dec_pulse;
reg        uart_cmd_mode_pulse;
reg        uart_cmd_sub_pulse;
reg        uart_cmd_fm_pulse;
reg        uart_cmd_rst_pulse;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uart_cmd_inc_pulse  <= 1'b0;
        uart_cmd_dec_pulse  <= 1'b0;
        uart_cmd_mode_pulse <= 1'b0;
        uart_cmd_sub_pulse  <= 1'b0;
        uart_cmd_fm_pulse   <= 1'b0;
        uart_cmd_rst_pulse  <= 1'b0;
    end else begin
        uart_cmd_inc_pulse  <= 1'b0;
        uart_cmd_dec_pulse  <= 1'b0;
        uart_cmd_mode_pulse <= 1'b0;
        uart_cmd_sub_pulse  <= 1'b0;
        uart_cmd_fm_pulse   <= 1'b0;
        uart_cmd_rst_pulse  <= 1'b0;
        if (uart_cmd_valid) begin
            case (uart_cmd_byte)
                8'h49, 8'h69: uart_cmd_inc_pulse  <= 1'b1;
                8'h44, 8'h64: uart_cmd_dec_pulse  <= 1'b1;
                8'h4D, 8'h6D: uart_cmd_mode_pulse <= 1'b1;
                8'h53, 8'h73: uart_cmd_sub_pulse  <= 1'b1;
                8'h46, 8'h66: uart_cmd_fm_pulse   <= 1'b1;
                8'h52, 8'h72: uart_cmd_rst_pulse  <= 1'b1;
            endcase
        end
    end
end

// UART模式/子模式/FM内部寄存器 (由3519命令控制)
reg [1:0] uart_mode_reg;
reg       uart_sub_reg;
reg       uart_fm_reg;

// FPGA按键复位也会重置UART寄存器, 保证同步
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uart_mode_reg <= 2'b00;
        uart_sub_reg  <= 1'b0;
        uart_fm_reg   <= 1'b0;
    end else begin
        if (uart_cmd_rst_pulse) begin              // 'R'命令: 复位所有UART寄存器
            uart_mode_reg <= 2'b00;
            uart_sub_reg  <= 1'b0;
            uart_fm_reg   <= 1'b0;
        end else begin
            if (uart_cmd_mode_pulse)
                uart_mode_reg <= uart_mode_reg + 1'b1;   // 循环: 00→01→10→11→00
            if (uart_cmd_sub_pulse)
                uart_sub_reg <= ~uart_sub_reg;
            if (uart_cmd_fm_pulse)
                uart_fm_reg <= ~uart_fm_reg;
        end
    end
end

// 综合控制信号: 外部引脚优先, UART命令补充
// mode_sel: 外部非00时用外部, 否则用UART值
// sub_mode/fm_dev_sel: 外部为1时覆盖, 否则用UART值
wire [1:0] mode_sel_eff = (mode_sel != 2'b00) ? mode_sel : uart_mode_reg;
wire       sub_mode_eff = sub_mode | uart_sub_reg;
wire       fm_dev_eff   = fm_dev_sel | uart_fm_reg;

// 频率增/减: FPGA消抖边沿 OR UART命令脉冲
wire       inc_cmd = inc_rise | uart_cmd_inc_pulse;
wire       dec_cmd = dec_rise | uart_cmd_dec_pulse;

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

// ========== 按键同步、消抖与边沿检测 ==========
// 20ms @ 50MHz = 1,000,000 周期
localparam DEBOUNCE_MAX = 20'd999_999;

// 同步链 (2级DFF, 消除亚稳态)
reg [1:0] inc_sync, dec_sync;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        inc_sync <= 2'b0;
        dec_sync <= 2'b0;
    end else begin
        inc_sync <= {inc_sync[0], freq_inc};
        dec_sync <= {dec_sync[0], freq_dec};
    end
end

// 消抖计数器: 输入稳定保持约20ms才翻转
reg inc_stable, dec_stable;
reg [19:0] inc_db_cnt, dec_db_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        inc_stable <= 1'b0;
        inc_db_cnt <= 20'd0;
    end else begin
        if (inc_sync[1] != inc_stable) begin
            if (inc_db_cnt >= DEBOUNCE_MAX) begin
                inc_db_cnt <= 20'd0;
                inc_stable <= ~inc_stable;
            end else
                inc_db_cnt <= inc_db_cnt + 1'b1;
        end else
            inc_db_cnt <= 20'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dec_stable <= 1'b0;
        dec_db_cnt <= 20'd0;
    end else begin
        if (dec_sync[1] != dec_stable) begin
            if (dec_db_cnt >= DEBOUNCE_MAX) begin
                dec_db_cnt <= 20'd0;
                dec_stable <= ~dec_stable;
            end else
                dec_db_cnt <= dec_db_cnt + 1'b1;
        end else
            dec_db_cnt <= 20'd0;
    end
end

// 上升沿检测 (带启动锁定, 防止复位后消抖稳定时的误触发)
reg inc_prev, dec_prev;
wire inc_rise, dec_rise;

reg [21:0] startup_cnt;
wire       startup_hold = (startup_cnt < 22'd2_500_000);   // ~50ms @ 50MHz

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        inc_prev <= 1'b0;
        dec_prev <= 1'b0;
        startup_cnt <= 22'd0;
    end else begin
        inc_prev <= inc_stable;
        dec_prev <= dec_stable;
        if (startup_hold)
            startup_cnt <= startup_cnt + 1'b1;
    end
end

assign inc_rise = inc_stable & ~inc_prev & ~startup_hold;
assign dec_rise = dec_stable & ~dec_prev & ~startup_hold;

// ========== 频率控制字生成 ==========
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        freq_code <= 32'd1;             // 上电默认 100Hz
    else if (uart_cmd_rst_pulse)
        freq_code <= 32'd1;             // 'R'命令: 复位到100Hz
    else begin
        if (inc_cmd && (freq_code < 32'd100000)) begin
            if (uart_cmd_inc_pulse)
                freq_code <= freq_code + 32'd10;   // UART: 1kHz步进
            else
                freq_code <= freq_code + 32'd1;    // 按键: 100Hz步进
        end
        else if (dec_cmd && (freq_code > 32'd1)) begin
            if (uart_cmd_dec_pulse)
                freq_code <= freq_code - 32'd10;   // UART: 1kHz步进
            else
                freq_code <= freq_code - 32'd1;    // 按键: 100Hz步进
        end
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
        phase_mod1k <= phase_mod1k + 32'd85900;  // 1kHz = 10 x 100Hz = 10 x 8590
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
// BUGFIX: [13:0] → [27:14], 原来取了低14位(全是噪声)，应该取高14位(调制包络)
wire [27:0] am_mult = bias_plus_mod[27:14] * am_carrier;
assign am_out = am_mult[27:14];

// ========== FM调制 (中心频率可调，频偏可切换) ==========
// 中心频率使用 freq_code 控制的 DDS 输出作为载波频率（范围100Hz~10MHz）
// 但 FM 要求载波 100kHz~10MHz，我们限制 freq_code 最小1000 (100kHz)
wire [31:0] fc_ftw_tmp = (freq_code < 32'd1000) ? 32'd858993 : freq_code * FTW_STEP;
assign fc_ftw = fc_ftw_tmp;
localparam FTW_5K = 32'd429500;
localparam FTW_10K = 32'd859000;
assign delta_f_ftw = fm_dev_eff ? FTW_10K : FTW_5K;

// BUGFIX1: 有符号偏移, FM需要正负方向都能调
wire signed [13:0] offset_s = $signed(mod_sin) - $signed(14'd8192);

// BUGFIX2: 原来的 delta_f_ftw[13:0] 取了低14位丢弃了有效数据
// 正确: ftw_delta = offset * delta_f_ftw / 8192
// 直接做有符号乘法然后右移13位(=除以8192)
wire signed [45:0] ftw_delta_full = offset_s * $signed({1'b0, delta_f_ftw});
wire signed [31:0] ftw_delta = ftw_delta_full[44:13];

// fc_ftw转有符号再相加, 保证负向偏移也能正确减小频率
assign ftw_fm = $unsigned($signed({1'b0, fc_ftw}) + ftw_delta);

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

// ========== 模式选择 (外部引脚 + UART命令合并) ==========
always @(*) begin
    case (mode_sel_eff)
        2'b00:   dac_data_reg = sine_data;
        2'b01:   dac_data_reg = am_out;
        2'b10:   dac_data_reg = fm_out;
        2'b11:   dac_data_reg = (sub_mode_eff == 0) ? psk_out : ask_out;
        default: dac_data_reg = sine_data;
    endcase
end

always @(posedge clk) begin
    dac_data <= dac_data_reg;
end

// 将DAC时钟反相，使数据在DAC时钟上升沿前已有半个周期稳定时间
assign dac_clk = ~clk;

// ========== UART 发送模块 (VOFA+ 波形显示) ==========
// 格式: 十进制ASCII + CR+LF, 115200bps, 8N1
// DDS输出值连续发送, 发送频率由UART速率自动限制
// VOFA+选择 "RowData" (文本) 模式即可显示

localparam BAUD_115200 = 9'd434;   // 50000000 / 115200
// localparam BAUD_921600 = 9'd54; // 如需更高速率, 取消注释并注释上行

// UART发送控制信号
wire        uart_busy;
reg  [7:0]  uart_tx_byte;
reg         uart_tx_en;

// 采样/发送状态机
localparam  UART_IDLE    = 3'd0,
            UART_COMPUTE = 3'd1,
            UART_SEND_D  = 3'd2,
            UART_WAIT_D  = 3'd3,
            UART_CR      = 3'd4,
            UART_WAIT_CR = 3'd5,
            UART_LF      = 3'd6,
            UART_WAIT_LF = 3'd7;

reg [2:0] uart_state;
reg [13:0] uart_sample;
reg [4:0] uart_digits[0:4];
reg [2:0] uart_digit_idx;
reg       uart_sent_nonzero;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uart_state <= UART_IDLE;
        uart_tx_en <= 1'b0;
        uart_sample <= 14'd0;
        uart_digit_idx <= 3'd0;
        uart_sent_nonzero <= 1'b0;
    end else begin
        uart_tx_en <= 1'b0;                             // 默认: 脉冲只持续1周期
        case (uart_state)
            // ========================
            UART_IDLE: begin
                if (!uart_busy) begin
                    uart_sample <= dac_data;
                    uart_state   <= UART_COMPUTE;
                end
            end
            // ========================
            UART_COMPUTE: begin                          // 计算BCD码 (纯组合逻辑路径)
                uart_digits[0] <= uart_sample % 10;
                uart_digits[1] <= (uart_sample / 10) % 10;
                uart_digits[2] <= (uart_sample / 100) % 10;
                uart_digits[3] <= (uart_sample / 1000) % 10;
                uart_digits[4] <= (uart_sample / 10000) % 10;
                uart_digit_idx   <= 3'd4;                // 从最高位开始
                uart_sent_nonzero <= 1'b0;
                uart_state <= UART_SEND_D;
            end
            // ========================
            UART_SEND_D: begin                           // 逐位发送, 抑制前导零
                if (uart_digit_idx == 3'd0) begin
                    uart_tx_byte <= "0" + uart_digits[0]; // 最后1位一定发
                    uart_tx_en   <= 1'b1;
                    uart_state   <= UART_WAIT_D;
                    uart_digit_idx <= 3'd5;              // 标记数字发完
                end else if (uart_digits[uart_digit_idx] != 0 || uart_sent_nonzero) begin
                    uart_tx_byte <= "0" + uart_digits[uart_digit_idx];
                    uart_tx_en   <= 1'b1;
                    uart_state   <= UART_WAIT_D;
                    uart_sent_nonzero <= 1'b1;
                    uart_digit_idx <= uart_digit_idx - 1;
                end else begin
                    uart_digit_idx <= uart_digit_idx - 1; // 跳过前导零
                end
            end
            // ========================
            UART_WAIT_D: begin
                if (!uart_busy) begin                    // 等待字节发送完成
                    uart_state <= (uart_digit_idx == 3'd5) ? UART_CR : UART_SEND_D;
                end
            end
            // ========================
            UART_CR: begin                               // 发送回车 CR
                uart_tx_byte <= 8'h0D;
                uart_tx_en   <= 1'b1;
                uart_state   <= UART_WAIT_CR;
            end
            UART_WAIT_CR: begin
                if (!uart_busy)
                    uart_state <= UART_LF;
            end
            // ========================
            UART_LF: begin                               // 发送换行 LF
                uart_tx_byte <= 8'h0A;
                uart_tx_en   <= 1'b1;
                uart_state   <= UART_WAIT_LF;
            end
            UART_WAIT_LF: begin
                if (!uart_busy)
                    uart_state <= UART_IDLE;             // 回到IDLE, 立即发送下个采样
            end
        endcase
    end
end

// UART TX实例化 (发送波形到PC)
uart_tx #(.BAUD_CNT(BAUD_115200)) uart_inst (
    .clk    (clk),
    .rst_n  (rst_n),
    .tx_data(uart_tx_byte),
    .tx_en  (uart_tx_en),
    .tx_busy(uart_busy),
    .txd    (uart_txd)
);

// UART RX实例化 (接收3519控制命令)
uart_rx #(.BAUD_CNT(BAUD_115200)) uart_rx_inst (
    .clk    (clk),
    .rst_n  (rst_n),
    .rxd    (uart_rxd),
    .rx_data(uart_cmd_byte),
    .rx_done(uart_cmd_valid)
);

endmodule


// ============================================================
// UART 发送模块 (8N1, 115200bps)
// ============================================================
module uart_tx #(
    parameter BAUD_CNT = 434          // 50000000 / 115200 = 434
) (
    input           clk,
    input           rst_n,
    input   [7:0]   tx_data,
    input           tx_en,
    output reg      tx_busy,
    output reg      txd
);

    localparam  IDLE  = 2'd0,
                START = 2'd1,
                DATA  = 2'd2,
                STOP  = 2'd3;

    reg [1:0] state;
    reg [8:0] baud_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            txd       <= 1'b1;
            tx_busy   <= 1'b0;
            baud_cnt  <= 9'd0;
            shift_reg <= 8'd0;
            bit_idx   <= 3'd0;
        end else begin
            case (state)
                IDLE: begin
                    txd <= 1'b1;
                    if (tx_en && !tx_busy) begin
                        tx_busy   <= 1'b1;
                        shift_reg <= tx_data;
                        baud_cnt  <= 9'd0;
                        bit_idx   <= 3'd0;
                        state     <= START;
                    end
                end

                START: begin
                    txd <= 1'b0;                          // 起始位
                    if (baud_cnt >= BAUD_CNT - 1) begin
                        baud_cnt <= 9'd0;
                        state    <= DATA;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end

                DATA: begin
                    txd <= shift_reg[0];                  // LSB先发
                    if (baud_cnt >= BAUD_CNT - 1) begin
                        baud_cnt  <= 9'd0;
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        if (bit_idx >= 3'd7) begin
                            state <= STOP;
                        end else
                            bit_idx <= bit_idx + 1;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end

                STOP: begin
                    txd <= 1'b1;                          // 停止位
                    if (baud_cnt >= BAUD_CNT - 1) begin
                        baud_cnt <= 9'd0;
                        tx_busy  <= 1'b0;
                        state    <= IDLE;
                    end else
                        baud_cnt <= baud_cnt + 1;
                end
            endcase
        end
    end

endmodule


// ============================================================
// UART 接收模块 (8N1, 115200bps)
// 使用单计数器方案, 逻辑更清晰
// ============================================================
module uart_rx #(
    parameter BAUD_CNT = 434           // 50000000 / 115200 = 434
) (
    input           clk,
    input           rst_n,
    input           rxd,
    output reg [7:0] rx_data,
    output reg      rx_done
);

    localparam BAUD_HALF = BAUD_CNT >> 1;   // 217

    // 同步器 + 下降沿检测
    reg rxd_sync, rxd_prev;
    wire start_edge;

    always @(posedge clk) begin
        rxd_sync <= rxd;
        rxd_prev <= rxd_sync;
    end
    assign start_edge = rxd_prev & ~rxd_sync;   // 下降沿

    // 接收状态: 0=idle, 1=start, 2..9=data bits 0..7, 10=stop
    reg [3:0] state;
    reg [8:0] cnt;
    reg [7:0] shreg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= 4'd0;
            cnt     <= 9'd0;
            shreg   <= 8'd0;
            rx_data <= 8'd0;
            rx_done <= 1'b0;
        end else begin
            rx_done <= 1'b0;                   // rx_done默认脉冲1周期

            case (state)
                // ============ IDLE ============
                4'd0: begin
                    cnt <= 9'd0;
                    if (start_edge)
                        state <= 4'd1;         // 检测到起始位下降沿
                end

                // ============ START ============
                4'd1: begin
                    if (cnt >= BAUD_HALF) begin
                        cnt <= 9'd0;
                        if (!rxd_sync)          // 起始位中点仍为低 → 有效
                            state <= 4'd2;      // 开始接收bit0
                        else
                            state <= 4'd0;      // 毛刺, 回到IDLE
                    end else
                        cnt <= cnt + 1;
                end

                // ============ DATA bits 0..7 ============
                4'd2, 4'd3, 4'd4, 4'd5,
                4'd6, 4'd7, 4'd8, 4'd9: begin
                    if (cnt >= BAUD_CNT - 1) begin
                        cnt   <= 9'd0;
                        // LSB first: 新bit移入MSB
                        shreg <= {rxd_sync, shreg[7:1]};
                        state <= state + 1;
                    end else
                        cnt <= cnt + 1;
                end

                // ============ STOP ============
                4'd10: begin
                    if (cnt >= BAUD_HALF) begin
                        cnt   <= 9'd0;
                        state <= 4'd0;          // 回到IDLE
                        if (rxd_sync) begin      // 停止位为高 → 有效帧
                            rx_data <= shreg;
                            rx_done <= 1'b1;
                        end                     // 停止位为低 → 帧错误,静默丢弃
                    end else
                        cnt <= cnt + 1;
                end

                // ============ 安全兜底 ============
                default:
                    state <= 4'd0;
            endcase
        end
    end

endmodule