# SPDX-FileCopyrightText: Copyright (C) 2025 Arrow
# SPDX-License-Identifier: MIT-0

# 2 MHz ref -> 400 MHz synchronizer (false path)
set_false_path -from [get_clocks {i_niosv|iopll|iopll_outclk2}] -to [get_registers {*|ref_sync*}]

# ---------- CIC CDC FIFO Synchronizer Constraints (MTBF Critical) ----------
# Gray-coded pointer synchronizers crossing 100 MHz <-> 400 MHz
# Tightened constraints to improve settling time and meet MTBF requirements

# Write pointer: 400 MHz -> 100 MHz
set_max_skew -from [get_keepers "*delayed_wrptr_g*"] -to [get_keepers "*rs_dgwp|dffpipe*|dffe8a*"] -get_skew_value_from_clock_period min_clock_period -skew_value_multiplier 0.5
set_net_delay -from [get_keepers "*delayed_wrptr_g*"] -to [get_keepers "*rs_dgwp|dffpipe*|dffe8a*"] -max -get_value_from_clock_period dst_clock_period -value_multiplier 0.5

# Read pointer: 100 MHz -> 400 MHz (critical for MTBF)
set_max_skew -from [get_keepers "*rdptr_g*"] -to [get_keepers "*ws_dgrp|dffpipe*|dffe11a*"] -get_skew_value_from_clock_period min_clock_period -skew_value_multiplier 0.5
set_net_delay -from [get_keepers "*rdptr_g*"] -to [get_keepers "*ws_dgrp|dffpipe*|dffe11a*"] -max -get_value_from_clock_period dst_clock_period -value_multiplier 0.5
set_max_delay -from [get_keepers "*ws_dgrp|dffpipe*|dffe11a*"] -to [get_keepers "*ws_dgrp|dffpipe*|dffe12a*"] 1.5

# ---------- TDC Calibration Paths (Startup Only) ----------
# Histogram RAM (256x16 @ 400 MHz, 2 cycles)
set_multicycle_path -setup 2 -from [get_registers {*calib_hist_idx_d3*}] -to [get_registers {*calib_histogram*}]
set_multicycle_path -hold 1 -from [get_registers {*calib_hist_idx_d3*}] -to [get_registers {*calib_histogram*}]
set_multicycle_path -setup 2 -from [get_registers {*calib_search_idx*}] -to [get_registers {*calib_hist_read*}]
set_multicycle_path -hold 1 -from [get_registers {*calib_search_idx*}] -to [get_registers {*calib_hist_read*}]
set_multicycle_path -setup 2 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_read_d1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_read_d1[*]}]

# Calibration control signals (startup only, 2-3 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*i_tdc|calib_done*}] -to [get_registers {*i_tdc|adjp_s1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*i_tdc|calib_done*}] -to [get_registers {*i_tdc|adjp_s1[*]}]

# ---------- Boot Dither DAC Timing ----------
# The boot dither FSM generates a calibration waveform during startup.
# This is only active for ~100µs during initialization, not during normal operation.
# Boot dither paths (startup only, 2 cycles @ 400 MHz)
# Calibration search result write (calib_search_idx -> coarse_bias_calibrated)
# When calibration search completes (once at startup), calib_mode_value is written to coarse_bias_calibrated.
# Boot FSM state to sweep duty cycle (boot_state -> sweep_cross_duty)
# During startup sweep/dither states, the FSM updates the sweep_cross_duty register.
# This is only active during ~100µs initialization, not timing-critical.
# Allow 2 cycles for FSM state decode to duty cycle register at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*boot_state*}] -to [get_registers {*sweep_cross_duty[*]}]
set_multicycle_path -hold 1 -from [get_registers {*boot_state*}] -to [get_registers {*sweep_cross_duty[*]}]
set_multicycle_path -setup 2 -from [get_registers {*p_boot_dither.v_boot_counter[*]}] -to [get_registers {*boot_state*}]
set_multicycle_path -hold 1 -from [get_registers {*p_boot_dither.v_boot_counter[*]}] -to [get_registers {*boot_state*}]

# Calibration histogram write (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*calib_hist_update_d3*}] -to [get_registers {*calib_histogram[*][*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_hist_update_d3*}] -to [get_registers {*calib_histogram[*][*]}]

# Histogram read address to value pipeline (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*calib_hist_read_addr[*]}] -to [get_registers {*calib_hist_value_d1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_hist_read_addr[*]}] -to [get_registers {*calib_hist_value_d1[*]}]
# TDL calibration sum to settle counter (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_settle_cnt[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_settle_cnt[*]}]

# Calibration control signals (3 cycles @ 400 MHz)
set_multicycle_path -setup 3 -from [get_registers {*|calib_done*}] -to [get_registers {*|calib_histogram[*][*]}]
set_multicycle_path -hold 2 -from [get_registers {*|calib_done*}] -to [get_registers {*|calib_histogram[*][*]}]
set_multicycle_path -setup 3 -from [get_registers {*|calib_ready*}] -to [get_registers {*|calib_histogram[*][*]}]
set_multicycle_path -hold 2 -from [get_registers {*|calib_ready*}] -to [get_registers {*|calib_histogram[*][*]}]

# Calib done to adjustment pipeline
set_multicycle_path -setup 3 -from [get_registers {*|calib_done*}] -to [get_registers {*|adjm_s1[*]}]
set_multicycle_path -hold 2 -from [get_registers {*|calib_done*}] -to [get_registers {*|adjm_s1[*]}]

# TDL calibration sum to fine accumulator (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_cal_fine_acc[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_cal_fine_acc[*]}]

# Sweep duty cycle to boot DAC (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*sweep_cross_duty[*]}] -to [get_registers {*dac_boot_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*sweep_cross_duty[*]}] -to [get_registers {*dac_boot_ff*}]

# Histogram to histogram value paths (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_value_d1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_value_d1[*]}]

# TDL calibration sum to coarse bias (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*coarse_bias_cal[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*coarse_bias_cal[*]}]

# Calibration search index to mode value (2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*i_tdc|calib_search_idx[*]}] -to [get_registers {*i_tdc|calib_mode_value[*]}]
set_multicycle_path -hold 1 -from [get_registers {*i_tdc|calib_search_idx[*]}] -to [get_registers {*i_tdc|calib_mode_value[*]}]
set_multicycle_path -setup 2 -from [get_registers {*i_tdc|calib_search_idx[*]}] -to [get_registers {*i_tdc|calib_max_count[*]}]
set_multicycle_path -hold 1 -from [get_registers {*i_tdc|calib_search_idx[*]}] -to [get_registers {*i_tdc|calib_max_count[*]}]

# TDC arithmetic pipeline (barrel shifter + negate, 2 cycles @ 400 MHz)
set_multicycle_path -setup 2 -from [get_registers {*i_adc|tdc_sub_pipe[*]}] -to [get_registers {*i_adc|tdc_diff_pipe[*]}]
set_multicycle_path -hold 1 -from [get_registers {*i_adc|tdc_sub_pipe[*]}] -to [get_registers {*i_adc|tdc_diff_pipe[*]}]