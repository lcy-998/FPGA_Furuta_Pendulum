// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: j280_hw_top
// 目标芯片: 高云 GW2A-LV18PG256C8/I7（逻辑派 FPGA-G1）
// 功能描述: 旋转倒立摆姿态控制顶层；摆角原码由板载 GD32 串口桥提供。
//           将电机驱动、编码器测速测角、摆杆角度传感器、LQI 姿态控制核互联
//           集成:
//           1. 由 GD32 1kHz 有效采样帧驱动的控制流水线
//           2. 固定摆角端点（直立 ADC=1287，最低点 ADC=400）；KEY1 控制运行/停机
//           3. 复合按键交互: 短按轮换控制模式(0°/±45°定点/0.2Hz正弦跟踪)，长按清零转臂位置
//           4. 起摆能量泵 (P1) 与 4 态控制状态机 (STATE_HANGING / SWINGUP / BALANCE / PROTECT)
//           5. 解决跌落保护结构性冲突 (P1/D2): 起摆态放行大角度，仅平衡态触发跌落自锁
//           6. 转臂多圈软限位保护 (D3) 防止电机电缆绞断
//           7. 无除法定点数积分分离与防饱和累加器 (D2 / E4)
//           8. 完备的状态 LED 指示与 TB6612FNG 能耗制动电机驱动 (D1)
// =============================================================================

module j280_hw_top #(
    parameter integer CLK_FREQ_HZ           = 50_000_000,          // 系统时钟频率
    parameter integer CALIB_DEBOUNCE_CYCLES = 1_000_000,           // KEY1 按键消抖阈值 (20ms)
    parameter signed [31:0] ARM_SOFT_LIMIT_Q16 = 32'sd823548       // 转臂软限位阈值 (默认 ±2 圈)
)(
    // 系统时钟与复位
    input  wire                   clk_50m,        // 板载 50MHz 有源晶振
    input  wire                   rst_n,          // 硬件复位按键 (低有效)

    // 用户交互控制开关与按键
    // input  wire                   sw_motor_en,    // 拨码开关: 电机使能 (1: 运行, 0: 安全停机)
    // input  wire                   sw_brake_mode,  // 拨码开关: 停机模式 (1: 动态能耗制动, 0: 滑行)
    input  wire                   key_zero_calib, // 按键: 每按一次切换运行/停机 (低有效)
    input  wire                   key_pos_clear,  // 复合按键: 短按切换轨迹模式(0/1/2/3)，长按清零转臂位置 (低有效)

    // 直流电机驱动接口引脚 (连接 H 桥驱动芯片，如 TB6612FNG / A4950)
    output wire                   motor_pwm,      // 单端 PWM 脉冲引脚
    // output wire                   motor_dir,      // 转向 DIR 引脚
    output wire                   motor_in1,      // H 桥 IN1 引脚
    output wire                   motor_in2,      // H 桥 IN2 引脚
    // output wire                   motor_stby,     // TB6612 STBY 待机引脚 (高电平使能)

    // 电机正交光电编码器接口引脚
    input  wire                   enc_a,          // 编码器 A 相输入
    input  wire                   enc_b,          // 编码器 B 相输入

    input  wire                   adc_rx,         // GD32 PA2 -> FPGA K11，1 Mbaud 原码帧
    input  wire                   uart_rx,        // 板载 USB-Serial RX -> FPGA F13
    output wire                   uart_tx,        // FPGA F12 -> 板载 USB-Serial TX

    // 板载状态指示 LED
    output wire                   led_balance,    // 倒立摆处于直立自平衡区指示灯 (低电平点亮/起摆闪烁)
    output wire                   led_calib_ok,   // 固定零位已配置指示灯 (低电平点亮)
    output wire                   led_motor_run   // 电机动力输出与报警指示灯 (低电平点亮/保护快闪)
);
    wire sw_motor_en = 1'b1; // 默认使能电机，便于调试
    wire sw_brake_mode = 1'b0; // 默认能耗制动，便于快速停机保护

    // 状态指示闪烁分频器
    reg [24:0] blink_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            blink_cnt <= 25'd0;
        else
            blink_cnt <= blink_cnt + 1'b1;
    end
    wire blink_4hz  = blink_cnt[22]; // ~6 Hz
    wire blink_10hz = blink_cnt[21]; // ~12 Hz

    // -------------------------------------------------------------------------
    // 2. KEY1: 固定零位，无需现场标定；每次按下切换运行/停机
    // -------------------------------------------------------------------------
    reg [23:0] calib_debounce_cnt;
    reg        calib_pressed;
    reg        start_requested;
    reg        adc_has_sampled;

    wire        sample_done;
    wire        adc_stale;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            adc_has_sampled <= 1'b0;
        end else if (adc_stale) begin
            adc_has_sampled <= 1'b0;
        end else if (sample_done) begin
            adc_has_sampled <= 1'b1;
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            calib_debounce_cnt <= 24'd0;
            calib_pressed      <= 1'b0;
            start_requested    <= 1'b0;
        end else begin
            // ADC 失联后需要重新按 KEY1 启动；固定零位无需重新标定。
            if (adc_stale)
                start_requested <= 1'b0;
            if (key_zero_calib == 1'b0) begin
                if (calib_debounce_cnt < CALIB_DEBOUNCE_CYCLES) begin
                    calib_debounce_cnt <= calib_debounce_cnt + 1'b1;
                end else if (!calib_pressed && adc_has_sampled && !adc_stale) begin
                    start_requested <= ~start_requested;
                    calib_pressed   <= 1'b1;
                end
            end else begin
                calib_debounce_cnt <= 24'd0;
                calib_pressed      <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. 复合按键 KEY2: 短按模式轮换，长按绝对位置清零
    // -------------------------------------------------------------------------
    reg [25:0] key2_cnt;
    reg        clear_pos_pulse;
    reg [1:0]  traj_mode; // 0: 0°定点, 1: +45°定点, 2: -45°定点, 3: 0.2Hz正弦跟踪

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            key2_cnt        <= 26'd0;
            clear_pos_pulse <= 1'b0;
            traj_mode       <= 2'b00;
        end else begin
            clear_pos_pulse <= 1'b0;
            if (key_pos_clear == 1'b0) begin
                if (key2_cnt < 26'd50_000_000) begin
                    key2_cnt <= key2_cnt + 1'b1;
                end
                // 长按超过阈值触发转臂位置归零
                if (key2_cnt == CALIB_DEBOUNCE_CYCLES * 35) begin
                    clear_pos_pulse <= 1'b1;
                end
            end else begin
                // 短按释放 (消抖时间 ~ 长按阈值) 切换轨迹模式
                if (key2_cnt >= CALIB_DEBOUNCE_CYCLES && key2_cnt < CALIB_DEBOUNCE_CYCLES * 35) begin
                    traj_mode <= traj_mode + 1'b1;
                end
                key2_cnt <= 26'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. GD32 原码帧解析 + 摆角解算。仅有效帧触发一次控制采样。
    // -------------------------------------------------------------------------
    wire signed [31:0] theta_err_q16;
    wire signed [31:0] dtheta_q16;

    wire [11:0] adc_code;
    wire adc_valid;
    wire adc_framing_err;
    adc_bridge #(
        .CLK_HZ(CLK_FREQ_HZ), .BAUD(1_000_000), .TIMEOUT_MS(5)
    ) u_adc_bridge (
        .clk(clk_50m), .rst_n(rst_n), .rx(adc_rx),
        .code(adc_code), .valid(adc_valid), .stale(adc_stale),
        .framing_err(adc_framing_err)
    );

    angle_sensor_reader #(.INVERT_DIR(0), .TOP_RAW(12'd1290), .BOTTOM_RAW(12'd400)) u_angle_sensor (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .raw_valid     (adc_valid),
        .raw_data      (adc_code),
        .raw_adc_data  (),
        .theta_err_q16 (theta_err_q16),
        .dtheta_q16    (dtheta_q16),
        .sample_done   (sample_done)
    );

    // -------------------------------------------------------------------------
    // 5. 例化: 电机编码器 4 倍频与数字滤波测速测角模块 (D4/D5/E4)
    // -------------------------------------------------------------------------
    wire signed [31:0] pulse_count;
    wire signed [31:0] alpha_rad_q16;
    wire signed [31:0] dalpha_rad_s_q16;

    encoder_quad_reader #(
        .CLK_FREQ_HZ   (50_000_000),
        .FILTER_CYCLES (8),
        .CPR           (4000),
        .USE_Z_INDEX   (0),
        .REVERSE_DIR   (1)
    ) u_encoder (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .enc_a_raw     (enc_a),
        .enc_b_raw     (enc_b),
        .enc_z_raw     (1'b0),
        .clear_pos      (clear_pos_pulse),
        .calc_en        (sample_done),       // 修复 F9: 统一由 sample_done 触发，严格与角度采样对齐
        .pulse_count    (pulse_count),
        .alpha_rad_q16  (alpha_rad_q16),
        .dalpha_rad_s_q16(dalpha_rad_s_q16),
        .speed_pps      (),
        .dir_flag       ()
    );

    // -------------------------------------------------------------------------
    // 6. 转臂软限位保护判定 (D3) 防止连续单向旋转绞断电机引线
    // -------------------------------------------------------------------------
    wire soft_limit_err = (alpha_rad_q16 > ARM_SOFT_LIMIT_Q16) || (alpha_rad_q16 < -ARM_SOFT_LIMIT_Q16);
    // -------------------------------------------------------------------------
    // 7. 例化: 定点与正弦轨迹发生器 (P2 / 拓展要求 1 & 2 / F8 平滑与同步)
    // -------------------------------------------------------------------------
    wire signed [31:0] alpha_ref_q16;
    wire signed [31:0] dalpha_ref_q16;
    wire               traj_sync_pulse;

    traj_gen #(
        .CLK_FREQ_HZ(50_000_000),
        .UPDATE_HZ  (1000)
    ) u_traj_gen (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        .calc_en      (sample_done),
        .sync_phase   (traj_sync_pulse),    // 平衡切入瞬间复位正弦相位
        .mode_sel     (traj_mode),
        .alpha_ref_q16(alpha_ref_q16),
        .dalpha_ref_q16(dalpha_ref_q16)
    );

    // 状态误差解算 (引入轨迹位置与速度前馈偏差)
    wire signed [31:0] alpha_err_q16  = alpha_rad_q16 - alpha_ref_q16;
    wire signed [31:0] dalpha_err_q16 = dalpha_rad_s_q16 - dalpha_ref_q16;

    wire signed [17:0] gain_k1, gain_k2, gain_k3, gain_k4, gain_k5;
    wire [7:0] pc_gain_seq, pc_gain_status;
    uart_control_link #(.CLK_HZ(CLK_FREQ_HZ), .BAUD(115200)) u_pc_link (
        .clk(clk_50m), .rst_n(rst_n), .rx(uart_rx), .tx(uart_tx), .sample_done(sample_done),
        .theta_q16(theta_err_q16), .dtheta_q16(dtheta_q16),
        .alpha_q16(alpha_rad_q16), .dalpha_q16(dalpha_rad_s_q16),
        .theta_ref_q16(32'sd0), .dtheta_ref_q16(32'sd0),
        .alpha_ref_q16(alpha_ref_q16), .dalpha_ref_q16(dalpha_ref_q16),
        .gain_k1(gain_k1), .gain_k2(gain_k2), .gain_k3(gain_k3), .gain_k4(gain_k4), .gain_k5(gain_k5),
        .last_command_seq(pc_gain_seq), .last_command_status(pc_gain_status)
    );

    // -------------------------------------------------------------------------
    // 8. 声明内部连线并例化子模块
    // -------------------------------------------------------------------------
    wire [1:0]         fsm_state;
    wire               lqr_en;
    wire               swing_en;
    wire               fsm_motor_active;
    wire               reset_integral_pulse;
    wire               is_in_balance_zone;
    wire signed [15:0] final_pwm_duty;

    wire signed [15:0] swing_pwm_duty;
    wire               swing_calc_done;
    wire               swing_energy_deficit;

    wire signed [15:0] lqr_pwm_duty;
    wire               lqr_calc_done;

    // -------------------------------------------------------------------------
    // 9. LQI 转臂位置积分器 (D2 积分分离与 E4 无除法防饱和累加)
    // -------------------------------------------------------------------------
    // 积分饱和限幅: [-0.25, +0.25] rad -> Q16: 16384
    localparam signed [31:0] INT_LIMIT_Q16      = 32'sd16384;
    // 32 位全精度累加器限幅: 16384 * 131072 / 131 ~= 16393000
    localparam signed [31:0] INT_ACCUM_LIMIT    = 32'sd16393000;
    // 积分分离窗口: 优化至 ±1.1° (0.0192 rad -> Q16: 1258)，仅在稳态微小误差带内累加消除静差
    // 彻底消除斜坡大动态与前冲接杆瞬态导致的积分饱和超调 (Windup)
    localparam signed [31:0] INT_ERR_THRESH_Q16 = 32'sd1258;
    reg signed [31:0] alpha_int_accum;

    wire int_accum_en = lqr_en && (alpha_err_q16 > -INT_ERR_THRESH_Q16) &&
                        (alpha_err_q16 < INT_ERR_THRESH_Q16);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            alpha_int_accum <= 32'sd0;
        end else if (!fsm_motor_active || !lqr_en || reset_integral_pulse || soft_limit_err) begin
            alpha_int_accum <= 32'sd0; // 停机、非平衡态、捕获切入瞬间或超限时清零积分 (D2)
        end else if (sample_done) begin
            if (int_accum_en) begin
                if (alpha_int_accum + alpha_err_q16 > INT_ACCUM_LIMIT)
                    alpha_int_accum <= INT_ACCUM_LIMIT;
                else if (alpha_int_accum + alpha_err_q16 < -INT_ACCUM_LIMIT)
                    alpha_int_accum <= -INT_ACCUM_LIMIT;
                else
                    alpha_int_accum <= alpha_int_accum + alpha_err_q16;
            end else begin
                // 积分分离/动态运动防饱和: 动态斜坡跟踪 (dalpha_ref != 0) 或 误差越界时冻结积分 (消除 Windup 导致的过冲)
                alpha_int_accum <= alpha_int_accum;
            end
        end
    end

    // 无乘法移位加法实现 alpha_int_accum * 131 / 131072 (误差仅 0.05%)，带对称四舍五入
    // 扩展至 34 位防止 16393000 * 131 + 65536 发生 32 位有符号正溢出
    wire signed [33:0] accum_ext = {{2{alpha_int_accum[31]}}, alpha_int_accum};
    wire signed [33:0] int_x131 = (accum_ext <<< 7) + (accum_ext <<< 1) + accum_ext;
    wire signed [33:0] int_rounded = int_x131 + 34'sd65536;
    wire signed [31:0] alpha_int_scaled = int_rounded >>> 17;
    wire signed [31:0] alpha_int_q16 = (alpha_int_scaled > INT_LIMIT_Q16)  ? INT_LIMIT_Q16 :
                                       (alpha_int_scaled < -INT_LIMIT_Q16) ? -INT_LIMIT_Q16 :
                                       alpha_int_scaled;

    // -------------------------------------------------------------------------
    // 10. 例化: 起摆能量泵控制器 (P1 / F1 / F2 / F3 / F4)
    // -------------------------------------------------------------------------
    swing_up_ctrl u_swing_ctrl (
        .clk             (clk_50m),
        .rst_n           (rst_n),
        .calc_en         (sample_done),
        .theta_rad_q16   (theta_err_q16),
        .dtheta_rad_s_q16(dtheta_q16),
        .alpha_rad_q16   (alpha_rad_q16),
        .dalpha_rad_s_q16(dalpha_rad_s_q16),
        .pwm_swing_duty  (swing_pwm_duty),
        .energy_deficit  (swing_energy_deficit), // 修复 F2: 引出机械能指示
        .calc_done       (swing_calc_done)
    );

    // -------------------------------------------------------------------------
    // 11. 例化: LQR / LQI 定点数硬件乘加流水线计算核 (F7: 常开计算消除捕获空窗)
    // -------------------------------------------------------------------------
    // 平衡捕获平滑切入: 起摆进入自平衡初期 (前 768ms)，转臂位置误差分阶渐进接入 (25% -> 50% -> 75% -> 100%)
    reg [9:0] catch_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            catch_cnt <= 10'd1023;
        end else if (reset_integral_pulse) begin
            catch_cnt <= 10'd0;
        end else if (sample_done && (catch_cnt < 10'd1023)) begin
            catch_cnt <= catch_cnt + 10'd1;
        end
    end

    reg signed [31:0] lqr_alpha_err_q16;
    always @(*) begin
        case (catch_cnt[9:8])
            2'b00:   lqr_alpha_err_q16 = alpha_err_q16 >>> 2;
            2'b01:   lqr_alpha_err_q16 = alpha_err_q16 >>> 1;
            2'b10:   lqr_alpha_err_q16 = (alpha_err_q16 >>> 1) + (alpha_err_q16 >>> 2);
            default: lqr_alpha_err_q16 = alpha_err_q16;
        endcase
    end

    furuta_lqr_ctrl u_lqr_core (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .calc_en    (sample_done),               // 修复 F7: 去掉 && lqr_en，常开流水，切入瞬间当拍即可输出有效占空比
        .theta_err  (theta_err_q16),
        .dtheta     (dtheta_q16),
        .alpha_err  (lqr_alpha_err_q16),
        .dalpha     (dalpha_err_q16),
        .alpha_int  (alpha_int_q16),
        .gain_k1    (gain_k1), .gain_k2(gain_k2), .gain_k3(gain_k3),
        .gain_k4    (gain_k4), .gain_k5(gain_k5),
        .pwm_duty   (lqr_pwm_duty),
        .calc_done  (lqr_calc_done)
    );

    // -------------------------------------------------------------------------
    // 12. 例化: 四态主控制状态机 (P1 / F5 / F6 / F8)
    // -------------------------------------------------------------------------
    ctrl_fsm u_ctrl_fsm (
        .clk                 (clk_50m),
        .rst_n               (rst_n),
        .calc_en             (sample_done),
        .motor_en_sw         (sw_motor_en && start_requested && !adc_stale),
        .calib_done          (1'b1),
        .soft_limit_err      (soft_limit_err),
        .theta_err_q16       (theta_err_q16),
        .dtheta_q16          (dtheta_q16),
        .pwm_swing_duty      (swing_pwm_duty),
        .pwm_lqr_duty        (lqr_pwm_duty),
        .lqr_calc_done       (lqr_calc_done),
        .swing_calc_done     (swing_calc_done),
        .current_state       (fsm_state),
        .final_pwm_duty      (final_pwm_duty),
        .reset_integral_pulse(reset_integral_pulse),
        .traj_sync_pulse     (traj_sync_pulse),
        .is_in_balance_zone  (is_in_balance_zone),
        .lqr_en              (lqr_en),
        .swing_en            (swing_en),
        .motor_active        (fsm_motor_active)
    );

    // -------------------------------------------------------------------------
    // 13. 例化: 电机 PWM 驱动发生器与 H 桥接口模块 (D1: 能耗制动 STBY 常保)
    // -------------------------------------------------------------------------
    pwm_driver #(
        .CLK_FREQ_HZ   (50_000_000),
        .PWM_FREQ_HZ   (20_000),
        .DUTY_MAX_VAL  (1000),
        .DEAD_BAND_VAL (50)              // 非零指令额外补偿 50 PWM，补偿后仍限制在 1000
    ) u_motor_driver (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .motor_en   (fsm_motor_active && !adc_stale),
        .brake_mode (sw_brake_mode && !adc_stale),
        .pwm_duty   (adc_stale ? 16'sd0 : final_pwm_duty),
        .pwm_out    (motor_pwm),
        .dir_out    (motor_dir),
        .in1_out    (motor_in1),
        .in2_out    (motor_in2),
        .stby_out   (motor_stby)
    );

    // -------------------------------------------------------------------------
    // 14. 板载状态指示 LED 输出 (低电平点亮)
    // -------------------------------------------------------------------------
    // led_balance: BALANCE 态常亮(0)，SWINGUP 态快闪，其它熄灭(1)
    assign led_balance   = (fsm_state == 2'd2) ? 1'b0 :
                           (fsm_state == 2'd1) ? blink_4hz : 1'b1;

    // led_calib_ok: 固定端点已配置，常亮(0)
    assign led_calib_ok  = 1'b0;

    // led_motor_run: 保护自锁态警报快闪(10Hz)，正常工作有动力输出常亮(0)，其余熄灭(1)
    assign led_motor_run = (fsm_state == 2'd3) ? blink_10hz :
                           (fsm_motor_active && (final_pwm_duty != 16'sd0)) ? 1'b0 : 1'b1;

    wire unused_adc_framing_err = adc_framing_err;

endmodule
