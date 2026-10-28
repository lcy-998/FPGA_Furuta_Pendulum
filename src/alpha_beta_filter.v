// Fixed-period alpha-beta angle/velocity estimator. Inputs and outputs are Q16.
// The sample interval is nominally 1 ms. Gains are supplied in Q16.
module alpha_beta_filter #(
    parameter integer WRAP_ANGLE = 0,
    parameter integer ALPHA_Q16 = 32768,
    parameter integer BETA_Q16 = 8192
)(
    input wire clk,
    input wire rst_n,
    input wire sample_en,
    input wire signed [31:0] measurement_q16,
    output reg signed [31:0] angle_q16,
    output reg signed [31:0] velocity_q16
);
    localparam signed [31:0] TWO_PI_Q16 = 32'sd411775;
    localparam signed [31:0] PI_Q16 = 32'sd205887;
    localparam integer BETA_DT_GAIN = (BETA_Q16 * 1000) / 65536;
    reg initialized;
    wire signed [31:0] predicted_q16 = angle_q16 + (velocity_q16 / 32'sd1000);
    wire signed [31:0] raw_residual = measurement_q16 - predicted_q16;
    wire signed [31:0] residual_q16 = (WRAP_ANGLE != 0) ?
        ((raw_residual > PI_Q16) ? (raw_residual - TWO_PI_Q16) :
         ((raw_residual < -PI_Q16) ? (raw_residual + TWO_PI_Q16) : raw_residual)) : raw_residual;
    wire signed [63:0] velocity_correction_64 = $signed(residual_q16) * BETA_DT_GAIN;
    wire signed [31:0] velocity_correction_q16 = velocity_correction_64[31:0];
    wire signed [63:0] angle_correction_64 = $signed(residual_q16) * ALPHA_Q16;
    wire signed [31:0] angle_correction_q16 = angle_correction_64 >>> 16;
    wire signed [31:0] corrected_angle_unwrapped = predicted_q16 + angle_correction_q16;
    wire signed [31:0] corrected_angle_wrapped =
        (corrected_angle_unwrapped > PI_Q16) ? (corrected_angle_unwrapped - TWO_PI_Q16) :
        ((corrected_angle_unwrapped < -PI_Q16) ? (corrected_angle_unwrapped + TWO_PI_Q16) : corrected_angle_unwrapped);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            initialized <= 1'b0;
            angle_q16 <= 32'sd0;
            velocity_q16 <= 32'sd0;
        end else if (sample_en) begin
            if (!initialized) begin
                initialized <= 1'b1;
                angle_q16 <= measurement_q16;
                velocity_q16 <= 32'sd0;
            end else begin
                angle_q16 <= (WRAP_ANGLE != 0) ? corrected_angle_wrapped : corrected_angle_unwrapped;
                velocity_q16 <= velocity_q16 + velocity_correction_q16;
            end
        end
    end
endmodule
