# SPDX-FileCopyrightText: Copyright (C) 2025 Arrow
# SPDX-License-Identifier: MIT-0

set_time_format -unit ns -decimal_places 3

# ---------- Clocks ----------
# 25 MHz input clock
create_clock -name CLK_25M -period 40.000 [get_ports {CLK_25M_C}]

# PLL output clocks auto-created by Agilex 5 IP (100/400/2 MHz)
# Use: i_niosv|iopll|iopll_outclk0/1/2

# Virtual clocks for unconstrained I/O
create_clock -name VCLK_UART        -period 100.000
create_clock -name VCLK_TEST_PIN    -period 100.000
create_clock -name VCLK_ANALOG_400M -period 2.500

# ---------- False Paths ----------
set_false_path -from [get_ports {USER_BTN}]
set_false_path -from [get_ports {UART_RX}]
set_false_path -to [get_ports {UART_TX}]
set_false_path -to [get_ports {TEST_PIN}]
set_false_path -from [get_ports {ANALOG_IN}]
set_false_path -to [get_ports {FEEDBACK_OUT}]

# ---------- I/O Constraints ----------
# UART (async serial interface)
set_input_delay -clock [get_clocks VCLK_UART] -max 0.0 -source_latency_included [get_ports {UART_RX}]
set_input_delay -clock [get_clocks VCLK_UART] -min 0.0 -source_latency_included [get_ports {UART_RX}]
set_output_delay -clock [get_clocks VCLK_UART] -max 0.0 -source_latency_included [get_ports {UART_TX}]
set_output_delay -clock [get_clocks VCLK_UART] -min 0.0 -source_latency_included [get_ports {UART_TX}]

# Debug outputs
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -max 0.0 -source_latency_included [get_ports {TEST_PIN}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -min 0.0 -source_latency_included [get_ports {TEST_PIN}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -max 0.0 -source_latency_included [get_ports {LED1}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -min 0.0 -source_latency_included [get_ports {LED1}]

# Power control (static)
set_output_delay -clock [get_clocks CLK_25M] -max 0.0 -source_latency_included [get_ports {VSEL_1V3}]
set_output_delay -clock [get_clocks CLK_25M] -min 0.0 -source_latency_included [get_ports {VSEL_1V3}]

# Analog feedback loop (delta-sigma ADC)
set_input_delay -clock [get_clocks VCLK_ANALOG_400M] -max 0.0 -source_latency_included [get_ports {ANALOG_IN}]
set_input_delay -clock [get_clocks VCLK_ANALOG_400M] -min 0.0 -source_latency_included [get_ports {ANALOG_IN}]
set_output_delay -clock [get_clocks VCLK_ANALOG_400M] -max 0.0 -source_latency_included [get_ports {FEEDBACK_OUT}]
set_output_delay -clock [get_clocks VCLK_ANALOG_400M] -min 0.0 -source_latency_included [get_ports {FEEDBACK_OUT}]

# ---------- Multicycle Paths ----------
# Boot calibration counter (startup only)
set_multicycle_path -from [get_registers "*v_boot_counter*"] -to [get_registers "*v_boot_counter*"] -setup -start 12
set_multicycle_path -from [get_registers "*v_boot_counter*"] -to [get_registers "*v_boot_counter*"] -hold -start 11

# TDC dual-lobe bias selection pipeline (magnitude comparison, 2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*adj*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -hold 1 -from [get_registers {*adj*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -setup 2 -from [get_registers {*mag*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -hold 1 -from [get_registers {*mag*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -setup 2 -from [get_registers {*fine_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -hold 1 -from [get_registers {*fine_s1b*}] -to [get_registers {*d_used_s1c*}]

# CIC decimator comb stages (decimated rate, 4 cycles @ 400 MHz)
set_multicycle_path -setup 4 -from [get_registers {*i_cic_multibit*decimated*}] -to [get_registers {*i_cic_multibit*comb1_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic_multibit*decimated*}] -to [get_registers {*i_cic_multibit*comb1_out*}]
set_multicycle_path -setup 4 -from [get_registers {*i_cic_multibit*comb1_d*}] -to [get_registers {*i_cic_multibit*comb1_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic_multibit*comb1_d*}] -to [get_registers {*i_cic_multibit*comb1_out*}]

# Comb stage 2: comb1_out -> comb2_out
set_multicycle_path -setup 4 -from [get_registers {*i_cic_multibit*comb1_out*}] -to [get_registers {*i_cic_multibit*comb2_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic_multibit*comb1_out*}] -to [get_registers {*i_cic_multibit*comb2_out*}]
set_multicycle_path -setup 4 -from [get_registers {*i_cic_multibit*comb2_d*}] -to [get_registers {*i_cic_multibit*comb2_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic_multibit*comb2_d*}] -to [get_registers {*i_cic_multibit*comb2_out*}]

# Comb stage 3: comb2_out -> comb3_out
set_multicycle_path -setup 4 -from [get_registers {*i_cic_multibit*comb2_out*}] -to [get_registers {*i_cic_multibit*comb3_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic_multibit*comb2_out*}] -to [get_registers {*i_cic_multibit*comb3_out*}]
set_multicycle_path -setup 4 -from [get_registers {*i_cic_multibit*comb3_d*}] -to [get_registers {*i_cic_multibit*comb3_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic_multibit*comb3_d*}] -to [get_registers {*i_cic_multibit*comb3_out*}]
# Integrators are gated by CE (ref_clock edge detect at 50MHz = 1/8 of 400MHz)
# Allow 2 cycles for integrator paths
set_multicycle_path -setup 2 -from [get_registers {*i_cic_multibit*int1*}] -to [get_registers {*i_cic_multibit*int2*}]
set_multicycle_path -hold 1 -from [get_registers {*i_cic_multibit*int1*}] -to [get_registers {*i_cic_multibit*int2*}]
set_multicycle_path -setup 2 -from [get_registers {*i_cic_multibit*int2*}] -to [get_registers {*i_cic_multibit*int3*}]
set_multicycle_path -hold 1 -from [get_registers {*i_cic_multibit*int2*}] -to [get_registers {*i_cic_multibit*int3*}]

# CIC input pipeline (TDC to integrators, 2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*cic_input_tdc*}] -to [get_registers {*i_cic_multibit*int1*}]
set_multicycle_path -hold 1 -from [get_registers {*cic_input_tdc*}] -to [get_registers {*i_cic_multibit*int1*}]

# ---------- CIC Scale Pipeline Stages ----------  
# The scale_pipe registers form a multi-stage pipeline for output scaling.
# Each stage operates at the decimated output rate.
set_multicycle_path -setup 8 -from [get_registers {*i_cic_multibit|scale_pipe1*}] -to [get_registers {*i_cic_multibit|scale_pipe*}]
set_multicycle_path -hold 7 -from [get_registers {*i_cic_multibit|scale_pipe1*}] -to [get_registers {*i_cic_multibit|scale_pipe*}]

# ---------- Reset Synchronizer Paths ----------
# Reset synchronizer outputs are held stable for many cycles during reset release.
# These paths are not timing-critical.
set_multicycle_path -setup 4 -from [get_registers {*i_reset_sync|sync3}] -to [get_registers {*i_cic_multibit|*}]
set_multicycle_path -hold 3 -from [get_registers {*i_reset_sync|sync3}] -to [get_registers {*i_cic_multibit|*}]
set_multicycle_path -setup 4 -from [get_registers {*i_reset_sync|sync3}] -to [get_registers {*tdl_cal_fine_acc*}]
set_multicycle_path -hold 3 -from [get_registers {*i_reset_sync|sync3}] -to [get_registers {*tdl_cal_fine_acc*}]
set_multicycle_path -setup 4 -from [get_registers {*i_tdc|i_reset_sync|sync3}] -to [get_registers {*coarse_bias_calibrated*}]
set_multicycle_path -hold 3 -from [get_registers {*i_tdc|i_reset_sync|sync3}] -to [get_registers {*coarse_bias_calibrated*}]
set_multicycle_path -setup 4 -from [get_registers {*i_tdc|i_reset_sync|sync3}] -to [get_registers {*calib_hist_read_addr*}]
set_multicycle_path -hold 3 -from [get_registers {*i_tdc|i_reset_sync|sync3}] -to [get_registers {*calib_hist_read_addr*}]

# Reset sync to DAC output (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*i_reset_sync|sync3}] -to [get_registers {*dac_out_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*i_reset_sync|sync3}] -to [get_registers {*dac_out_ff*}]

# TDC reset sync to calibration histogram (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*i_tdc|i_reset_sync|sync3}] -to [get_registers {*i_tdc|calib_histogram*}]
set_multicycle_path -hold 1 -from [get_registers {*i_tdc|i_reset_sync|sync3}] -to [get_registers {*i_tdc|calib_histogram*}]

# ---------- TDC Adaptive Center Tracking Accumulator ----------
# TDC center accumulator (gated by tdc_valid @ ~50 MHz, 8 cycles @ 400 MHz)
set_multicycle_path -setup 8 -from [get_registers {*tdc_center_acc*}] -to [get_registers {*tdc_center_acc*}]
set_multicycle_path -hold 7 -from [get_registers {*tdc_center_acc*}] -to [get_registers {*tdc_center_acc*}]

# TDC result to center accumulator (IIR filter, 8 cycles @ 400 MHz)
set_multicycle_path -setup 8 -from [get_registers {*i_tdc|tdc_result*}] -to [get_registers {*tdc_center_acc_0*}]
set_multicycle_path -hold 7 -from [get_registers {*i_tdc|tdc_result*}] -to [get_registers {*tdc_center_acc_0*}]
set_multicycle_path -setup 8 -from [get_registers {*i_tdc|tdc_result*}] -to [get_registers {*tdc_center_acc_1*}]
set_multicycle_path -hold 7 -from [get_registers {*i_tdc|tdc_result*}] -to [get_registers {*tdc_center_acc_1*}]

# Calibration counter paths (8 cycles @ 400 MHz)
set_multicycle_path -setup 8 -from [get_registers {*cal_sample_cnt*}] -to [get_registers {*tdc_center_acc*}]
set_multicycle_path -hold 7 -from [get_registers {*cal_sample_cnt*}] -to [get_registers {*tdc_center_acc*}]
set_multicycle_path -setup 8 -from [get_registers {*v_total_samples*}] -to [get_registers {*tdc_center_acc*}]
set_multicycle_path -hold 7 -from [get_registers {*v_total_samples*}] -to [get_registers {*tdc_center_acc*}]

# Path from closed_loop_en control signal to all destinations
# This is a slow control signal that gates mode transitions - allow 4 cycles to all endpoints
set_multicycle_path -setup 4 -from [get_registers {*closed_loop_en*}]
set_multicycle_path -hold 3 -from [get_registers {*closed_loop_en*}]

# DAC output mux path (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*dac_muxed_reg*}] -to [get_registers {*dac_out_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*dac_muxed_reg*}] -to [get_registers {*dac_out_ff*}]

# Boot counter to DAC boot (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*p_boot_dither.v_boot_counter*}] -to [get_registers {*dac_boot_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*p_boot_dither.v_boot_counter*}] -to [get_registers {*dac_boot_ff*}]

# Boot counter to cal_enable RTM (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*p_boot_dither.v_boot_counter*}] -to [get_registers {*cal_enable_sys~RTM*}]
set_multicycle_path -hold 1 -from [get_registers {*p_boot_dither.v_boot_counter*}] -to [get_registers {*cal_enable_sys~RTM*}]

# ---------- TDC Difference IIR Lowpass Filter ----------
# TDC diff IIR filter (y[n] = y[n-1] + (x[n] - y[n-1]) / 64, 2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*tdc_diff_pipe*}] -to [get_registers {*tdc_diff_filtered_reg*}]
set_multicycle_path -hold 1 -from [get_registers {*tdc_diff_pipe*}] -to [get_registers {*tdc_diff_filtered_reg*}]
set_multicycle_path -setup 2 -from [get_registers {*tdc_diff_filtered_reg*}] -to [get_registers {*tdc_diff_filtered_reg*}]
set_multicycle_path -hold 1 -from [get_registers {*tdc_diff_filtered_reg*}] -to [get_registers {*tdc_diff_filtered_reg*}]