// ADC raw-code (0..1990) to pendulum angle and velocity (Q12.16).
// The UART frame receiver is separate; every valid frame starts one processing pass.
module angle_sensor_reader #(
    parameter integer INVERT_DIR = 0,
    parameter [11:0] TOP_RAW = 12'd1300,
    parameter [11:0] BOTTOM_RAW = 12'd400
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               raw_valid,
    input  wire [11:0]        raw_data,
    output reg  [11:0]        raw_adc_data,
    output reg  signed [31:0] theta_err_q16,
    output reg  signed [31:0] dtheta_q16,
    output reg                sample_done
);
    // -------------------------------------------------------------------------
    // 1. 角度绝对零点扣减与 360° 全圆周越界解卷绕 (Unwrapping)
    // -------------------------------------------------------------------------
    // ADC 刻度范围: [0, 1990]，共 1991 个刻度
    // 固定直立零点 TOP_RAW=1300、最低点 BOTTOM_RAW=400。
    // 从最低点到直立点递增 900 码；另一半圈经 1990/0 边界，共 1091 码。
    // 以最低点为 ±pi 分界，映射范围为 [-900, 1090] 码。
    // 这样摆杆在垂直向上 (theta = 0) 附近做微小左右摆动时，diff 严格围绕 0 线性连续变化！
    // -------------------------------------------------------------------------
    localparam signed [12:0] ADC_CODES_PER_REV = 13'sd1991;
    localparam signed [12:0] NEG_HALF_COUNTS = $signed({1'b0, TOP_RAW}) - $signed({1'b0, BOTTOM_RAW});
    localparam signed [12:0] POS_HALF_COUNTS = ADC_CODES_PER_REV - NEG_HALF_COUNTS;

    reg signed [12:0] diff_raw;
    reg signed [12:0] diff_unwrapped;

    always @(*) begin
        diff_raw = $signed({1'b0, raw_data}) - $signed({1'b0, TOP_RAW});
        if (diff_raw >= POS_HALF_COUNTS) begin
            diff_unwrapped = diff_raw - ADC_CODES_PER_REV;
        end else if (diff_raw < -NEG_HALF_COUNTS) begin
            diff_unwrapped = diff_raw + ADC_CODES_PER_REV;
        end else begin
            diff_unwrapped = diff_raw;
        end
    end

    // -------------------------------------------------------------------------
    // 2. 角度尺度变换 (刻度 -> Q12.16 弧度) 与差分角速度滤波
    // -------------------------------------------------------------------------
    // 两段码值各对应 pi rad，按实测两端点分别缩放：
    // 负侧 900 码对应 pi；正侧 1091 码对应 pi。
    // 以下除法只用于参数常数的编译期计算，运行时均乘常数再 >>> 12。
    //
    // 角速度计算:
    // 在 1ms 周期内: dtheta_raw = (theta_err_q16 - theta_err_prev) * 1000
    // 先一阶 IIR 低通，再对 IIR 输出做 5 点滑动窗口平均。
    // -------------------------------------------------------------------------
    localparam signed [31:0] TWO_PI_Q16_X4096 = 32'sd1686630400; // 411775 * 4096
    localparam signed [31:0] NEG_SCALE_Q12 = (TWO_PI_Q16_X4096 + NEG_HALF_COUNTS) / (NEG_HALF_COUNTS * 2);
    localparam signed [31:0] POS_SCALE_Q12 = (TWO_PI_Q16_X4096 + POS_HALF_COUNTS) / (POS_HALF_COUNTS * 2);
    localparam signed [31:0] FILTER_ALPHA_Q16 = 32'sd2938; // 滤波系数  * 65536 (消除除法器 E4)

    // 64 位中间乘法节点避免位宽截断与符号位混淆
    wire signed [31:0] scale_per_code_q12 = (diff_unwrapped < 0) ? NEG_SCALE_Q12 : POS_SCALE_Q12;
    wire signed [63:0] diff_scaled_64     = $signed({{51{diff_unwrapped[12]}}, diff_unwrapped}) * $signed(scale_per_code_q12);
    wire signed [63:0] diff_rad_shifted   = diff_scaled_64 >>> 12;
    wire signed [31:0] diff_rad_q16       = diff_rad_shifted[31:0];

    reg signed [31:0] theta_prev_q16;
    reg signed [31:0] diff_raw_r;       // 拍1 寄存: 未解卷绕的角度差值
    reg signed [31:0] raw_dtheta_q16;   // 拍2 寄存: 解卷绕并 ×1000 后的原始角速度
    reg signed [31:0] dtheta_iir_q16;   // IIR 内部状态；与最终滑窗输出分离
    reg signed [31:0] dtheta_iir_sample_r;
    reg               dtheta_avg_start;
    wire signed [31:0] dtheta_avg_q16;
    wire               dtheta_avg_done;
    reg        [1:0]  filter_step;      // 3 状态流水步进 (与 encoder_quad_reader 结构一致)

    // 拍 1 组合逻辑: 角度尺度变换结果与前一拍角度相减
    // diff_rad_q16 已含一次 64 位乘法 + >>>12，故本拍只做减法，不再串接后续乘法
    wire signed [31:0] current_theta_q16  = (INVERT_DIR == 0) ? diff_rad_q16 : (-diff_rad_q16);
    wire signed [31:0] diff_theta_raw     = current_theta_q16 - theta_prev_q16;

    // 拍 2 组合逻辑: 最短角位移解卷绕 (对标 controller.py#L208 的 %2pi 保护)
    // theta_err_q16 的定义域为 [-pi, +pi]（Q16: [-205887, +205887]）。当摆杆跨过
    // ±180° 边界时（起摆过程必然发生），直接相减会注入 ±2pi 的假跳变：
    //   例: +179° -> -179°（真实只动 2°）
    //       raw diff = -409488 Q16 -> 乘 1000 后 = -6248 rad/s 假尖峰
    //       真实角速度仅 34.9 rad/s，尖峰为其 179 倍
    // IIR (beta=0.35) 无法滤除该量级冲击，污染会持续 15~20ms，
    // 直接破坏起摆方向因子 theta_dot*cos(theta) 与能量判据。
    // 2pi 的 Q16 表示为 411775，pi 为 205887；该解绕在弧度域中与 ADC 刻度无关。
    //
    // 时序说明: 解卷绕的"比较+三选"与随后的 ×1000 乘法必须单独占一拍。
    // 若与拍 1 的 64 位乘法串联，关键路径会达到 19.9ns，Fmax 仅 50.194MHz
    // (裕量 0.39%)，工程上不可接受。
    wire signed [31:0] diff_theta_step    = (diff_raw_r >  32'sd205887) ? (diff_raw_r - 32'sd411775) :
                                           ((diff_raw_r < -32'sd205888) ? (diff_raw_r + 32'sd411775) :
                                                                          diff_raw_r);
    wire signed [47:0] raw_dtheta_mult    = $signed(diff_theta_step) * 32'sd1000;
    wire signed [31:0] raw_dtheta_calc    = raw_dtheta_mult[31:0];

    // 针对官方 FAQ 指出的 WDD35D4 电位器 345° 盲区 (约 15° 绝缘间隙) 的防御性限幅:
    // 摆杆在起摆旋转过程中越过 15° 盲区时, 滑片可能引起 ADC 码值跳变, 导致差分测速产生 90+ rad/s 虚假尖峰
    // 实际 90g 摆杆物理最大角速度约为 20~25 rad/s, 故设置 ±35 rad/s (Q16: 2293760) 物理安全门限
    // 既完全不影响正常起摆与自平衡 (实测最大仅 20.379 rad/s), 又能彻底遏制盲区跳变对动能计算与 IIR 滤波的冲击
    localparam signed [31:0] DTHETA_PHYS_LIMIT_Q16 = 32'sd2293760; // ±35 rad/s (Q16)
    wire signed [31:0] raw_dtheta_clamped = (raw_dtheta_calc > DTHETA_PHYS_LIMIT_Q16) ? DTHETA_PHYS_LIMIT_Q16 :
                                           ((raw_dtheta_calc < -DTHETA_PHYS_LIMIT_Q16) ? -DTHETA_PHYS_LIMIT_Q16 :
                                            raw_dtheta_calc);

    // Limit successive 1ms velocity samples before they enter the IIR state.
    localparam signed [31:0] MAX_DTHETA_STEP_Q16 = 32'sd655360; // 10 rad/s per 1ms
    wire signed [31:0] raw_dtheta_delta = raw_dtheta_clamped - raw_dtheta_q16;
    wire signed [31:0] raw_dtheta_limited =
        (raw_dtheta_delta > MAX_DTHETA_STEP_Q16) ? (raw_dtheta_q16 + MAX_DTHETA_STEP_Q16) :
        (raw_dtheta_delta < -MAX_DTHETA_STEP_Q16) ? (raw_dtheta_q16 - MAX_DTHETA_STEP_Q16) :
                                                    raw_dtheta_clamped;

    // 拍 3 组合逻辑: 先执行 IIR，随后将其输出送入 5 点滑动窗口。
    wire signed [47:0] filter_diff_mult   = $signed(raw_dtheta_q16 - dtheta_iir_q16) * $signed(FILTER_ALPHA_Q16);
    wire signed [47:0] filter_diff_shift  = filter_diff_mult >>> 16;
    wire signed [31:0] filter_diff_q16    = filter_diff_shift[31:0];
    wire signed [31:0] dtheta_iir_next    = dtheta_iir_q16 + filter_diff_q16;

    moving_average_5 u_dtheta_mavg (
        .clk(clk), .rst_n(rst_n), .start(dtheta_avg_start),
        .sample_in(dtheta_iir_sample_r), .average_out(dtheta_avg_q16), .done(dtheta_avg_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_adc_data     <= 12'd0;
            theta_err_q16    <= 32'sd0;
            theta_prev_q16   <= 32'sd0;
            diff_raw_r       <= 32'sd0;
            raw_dtheta_q16   <= 32'sd0;
            dtheta_iir_q16   <= 32'sd0;
            dtheta_iir_sample_r <= 32'sd0;
            dtheta_q16       <= 32'sd0;
            dtheta_avg_start <= 1'b0;
            sample_done      <= 1'b0;
            filter_step      <= 2'd0;
        end else if (filter_step == 2'd1) begin
            // 流水拍 2: 解卷绕、限幅滤波并换算为 rad/s (消除盲区虚假尖峰)
            raw_dtheta_q16 <= raw_dtheta_limited;
            dtheta_avg_start <= 1'b0;
            filter_step    <= 2'd2;
            sample_done    <= 1'b0;
        end else if (filter_step == 2'd2) begin
            // 流水拍 3: 寄存 IIR 输出，并启动多周期 5 点滑动窗口平均。
            dtheta_iir_q16 <= dtheta_iir_next;
            dtheta_iir_sample_r <= dtheta_iir_next;
            dtheta_avg_start <= 1'b1;
            filter_step    <= 2'd3;
            sample_done    <= 1'b0;
        end else if (filter_step == 2'd3) begin
            dtheta_avg_start <= 1'b0;
            if (dtheta_avg_done) begin
                dtheta_q16  <= dtheta_avg_q16;
                filter_step <= 2'd0;
                sample_done <= 1'b1;
            end else begin
                sample_done <= 1'b0;
            end
        end else if (raw_valid) begin
            // 流水拍 1: 锁存 ADC 码值、更新角度、寄存未解卷绕差值 (修复 B1: filter_step 优先推进, 防抢占丢采样)
            raw_adc_data   <= raw_data;
            theta_err_q16  <= current_theta_q16;
            theta_prev_q16 <= current_theta_q16;
            diff_raw_r     <= diff_theta_raw;
            dtheta_avg_start <= 1'b0;
            filter_step    <= 2'd1;
            sample_done    <= 1'b0;
        end else begin
            dtheta_avg_start <= 1'b0;
            sample_done    <= 1'b0;
        end
    end

endmodule

// 转臂专用的精确有符号 10 点滑动平均器。
module moving_average_10 (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [31:0] sample_in,
    output reg  signed [31:0] average_out,
    output reg                done
);
    reg signed [31:0] samples [0:9];
    reg signed [35:0] sum;
    reg        [3:0]  write_idx;
    reg               window_ready;
    reg               divide_busy;
    reg               quotient_negative;
    reg [35:0]        dividend_shift;
    reg [35:0]        quotient_shift;
    reg [3:0]         remainder;
    reg [5:0]         divide_count;
    integer i;

    wire signed [35:0] sample_ext = {{4{sample_in[31]}}, sample_in};
    wire signed [35:0] oldest_ext = {{4{samples[write_idx][31]}}, samples[write_idx]};
    wire signed [35:0] sum_after_replace = sum - oldest_ext + sample_ext;
    wire signed [35:0] sum_for_divide = window_ready ? sum_after_replace : ((sample_ext <<< 3) + (sample_ext <<< 1));
    wire [35:0] abs_sum_for_divide = sum_for_divide[35] ? -sum_for_divide : sum_for_divide;
    wire [4:0] remainder_shifted = {remainder, dividend_shift[35]};
    wire quotient_bit = (remainder_shifted >= 5'd10);
    wire [3:0] remainder_next = quotient_bit ? (remainder_shifted - 5'd10) : remainder_shifted[3:0];
    wire [35:0] quotient_next = {quotient_shift[34:0], quotient_bit};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum               <= 36'sd0;
            write_idx         <= 4'd0;
            window_ready      <= 1'b0;
            divide_busy       <= 1'b0;
            quotient_negative <= 1'b0;
            dividend_shift    <= 36'd0;
            quotient_shift    <= 36'd0;
            remainder         <= 4'd0;
            divide_count      <= 6'd0;
            average_out       <= 32'sd0;
            done              <= 1'b0;
            for (i = 0; i < 10; i = i + 1)
                samples[i] <= 32'sd0;
        end else begin
            done <= 1'b0;
            if (!divide_busy) begin
                if (start) begin
                    sum               <= sum_for_divide;
                    quotient_negative <= sum_for_divide[35];
                    dividend_shift    <= abs_sum_for_divide;
                    quotient_shift    <= 36'd0;
                    remainder         <= 4'd0;
                    divide_count      <= 6'd0;
                    divide_busy       <= 1'b1;
                    if (!window_ready) begin
                        window_ready <= 1'b1;
                        for (i = 0; i < 10; i = i + 1)
                            samples[i] <= sample_in;
                    end else begin
                        samples[write_idx] <= sample_in;
                        if (write_idx == 4'd9)
                            write_idx <= 4'd0;
                        else
                            write_idx <= write_idx + 1'b1;
                    end
                end
            end else begin
                dividend_shift <= {dividend_shift[34:0], 1'b0};
                quotient_shift <= quotient_next;
                remainder      <= remainder_next;
                if (divide_count == 6'd35) begin
                    average_out <= quotient_negative ? -$signed(quotient_next[31:0]) : $signed(quotient_next[31:0]);
                    done        <= 1'b1;
                    divide_busy <= 1'b0;
                end else begin
                    divide_count <= divide_count + 1'b1;
                end
            end
        end
    end
endmodule

// 精确有符号 5 点滑动平均器。用 36 个时钟完成逐位除以 5，避免将
// 除法器放入 50 MHz 的单周期组合路径，供摆杆测速使用。
module moving_average_5 (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [31:0] sample_in,
    output reg  signed [31:0] average_out,
    output reg                done
);
    reg signed [31:0] samples [0:4];
    reg signed [35:0] sum;
    reg        [3:0]  write_idx;
    reg               window_ready;
    reg               divide_busy;
    reg               quotient_negative;
    reg [35:0]        dividend_shift;
    reg [35:0]        quotient_shift;
    reg [4:0]         remainder;
    reg [5:0]         divide_count;
    integer i;

    wire signed [35:0] sample_ext = {{4{sample_in[31]}}, sample_in};
    wire signed [35:0] oldest_ext = {{4{samples[write_idx][31]}}, samples[write_idx]};
    wire signed [35:0] sum_after_replace = sum - oldest_ext + sample_ext;
    wire signed [35:0] sum_for_divide = window_ready ? sum_after_replace :
                                                       ((sample_ext <<< 2) + sample_ext);
    wire [35:0] abs_sum_for_divide = sum_for_divide[35] ? -sum_for_divide : sum_for_divide;
    wire [4:0] remainder_shifted = {remainder[3:0], dividend_shift[35]};
    wire quotient_bit = (remainder_shifted >= 5'd5);
    wire [4:0] remainder_next = quotient_bit ? (remainder_shifted - 5'd5) : remainder_shifted;
    wire [35:0] quotient_next = {quotient_shift[34:0], quotient_bit};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum               <= 36'sd0;
            write_idx         <= 4'd0;
            window_ready      <= 1'b0;
            divide_busy       <= 1'b0;
            quotient_negative <= 1'b0;
            dividend_shift    <= 36'd0;
            quotient_shift    <= 36'd0;
            remainder         <= 5'd0;
            divide_count      <= 6'd0;
            average_out       <= 32'sd0;
            done              <= 1'b0;
            for (i = 0; i < 5; i = i + 1)
                samples[i] <= 32'sd0;
        end else begin
            done <= 1'b0;
            if (!divide_busy) begin
                if (start) begin
                    sum               <= sum_for_divide;
                    quotient_negative <= sum_for_divide[35];
                    dividend_shift    <= abs_sum_for_divide;
                    quotient_shift    <= 36'd0;
                    remainder         <= 5'd0;
                    divide_count      <= 6'd0;
                    divide_busy       <= 1'b1;
                    if (!window_ready) begin
                        window_ready <= 1'b1;
                        for (i = 0; i < 5; i = i + 1)
                            samples[i] <= sample_in;
                    end else begin
                        samples[write_idx] <= sample_in;
                        if (write_idx == 4'd4)
                            write_idx <= 4'd0;
                        else
                            write_idx <= write_idx + 1'b1;
                    end
                end
            end else begin
                dividend_shift <= {dividend_shift[34:0], 1'b0};
                quotient_shift <= quotient_next;
                remainder      <= remainder_next;
                if (divide_count == 6'd35) begin
                    average_out <= quotient_negative ? -$signed(quotient_next[31:0]) :
                                                       $signed(quotient_next[31:0]);
                    done        <= 1'b1;
                    divide_busy <= 1'b0;
                end else begin
                    divide_count <= divide_count + 1'b1;
                end
            end
        end
    end
endmodule
