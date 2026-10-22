create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

# Asynchronous board inputs are synchronized or debounced in RTL.
set_false_path -from [get_ports {rst_n sw_motor_en sw_brake_mode key_zero_calib key_pos_clear}]
set_false_path -from [get_ports {enc_a enc_b adc_rx}]
