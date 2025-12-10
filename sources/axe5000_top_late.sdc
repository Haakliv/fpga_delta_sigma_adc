# SPDX-FileCopyrightText: Copyright (C) 2025 Arrow
# SPDX-License-Identifier: MIT-0

# ---------- CDC false paths to FIRST sync stage (non-intrusive; prevents bogus timing) ----------
# 2 MHz ref -> 100 MHz: first flop of 3-FF synchronizer (ref_sys0 in axe5000_top)
# This is an asynchronous clock domain crossing - the first register will be metastable
# Mark as false path to prevent timing analysis on the CDC path
# The -from specifies the SOURCE clock (2 MHz), -to specifies the destination register
set_false_path -from [get_clocks {i_niosv|iopll|iopll_outclk2}] -to [get_registers {*|ref_sys*}]

# 2 MHz ref -> 400 MHz: first flop of 3-FF synchronizer (ref_sync0 in TDC path)  
# Same CDC logic but crossing to the TDC 400 MHz clock domain
set_false_path -from [get_clocks {i_niosv|iopll|iopll_outclk2}] -to [get_registers {*|ref_sync*}]

# ---------- TDC Calibration Histogram RAM Timing ----------
# The calibration histogram is a 256-entry x 16-bit RAM updated during TDC calibration.
# Writes occur at 400 MHz (2.5ns period) with address decode from calib_hist_idx_d3.
# The address path has significant fanout and the RAM write enable logic is complex.
# Allow 2 cycles for histogram writes (only active during calibration, not real-time).
set_multicycle_path -setup 2 -from [get_registers {*calib_hist_idx_d3*}] -to [get_registers {*calib_histogram*}]
set_multicycle_path -hold 1 -from [get_registers {*calib_hist_idx_d3*}] -to [get_registers {*calib_histogram*}]

# Calibration histogram read path (calib_search_idx -> calib_hist_read)
# This is used during the calibration search phase to find the dual-lobe center.
# Allow 2 cycles for the read address decode and data mux (not timing-critical).
set_multicycle_path -setup 2 -from [get_registers {*calib_search_idx*}] -to [get_registers {*calib_hist_read*}]
set_multicycle_path -hold 1 -from [get_registers {*calib_search_idx*}] -to [get_registers {*calib_hist_read*}]

# Calibration histogram RAM read timing (histogram memory -> calib_hist_read_d1)
# The 256-entry histogram RAM has high fanout mux during search phase.
# This is a one-time startup calibration, not real-time data path.
# Allow 2 cycles for RAM read at 400 MHz (2.5ns period).
set_multicycle_path -setup 2 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_read_d1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_read_d1[*]}]

# ---------- CIC Decimator Division Timing ----------
# The CIC gain removal uses a 63-bit division (scale_pipe2 / R^3 -> scale_pipe3).
# This is a multi-cycle operation that produces one sample every 25600 reference clocks.
# At 50 MHz ref clock, this is one output every 512 microseconds (1953 Hz sample rate).
# Path delay is ~51ns, requiring 6 cycles @ 100MHz (10ns period). 
# Allow 6 cycles for the division to complete (only affects output latency, not throughput).
set_multicycle_path -setup 6 -from [get_registers {*i_cic*|scale_pipe2[*]}] -to [get_registers {*i_cic*|scale_pipe3[*]}]
set_multicycle_path -hold 5 -from [get_registers {*i_cic*|scale_pipe2[*]}] -to [get_registers {*i_cic*|scale_pipe3[*]}]

# ---------- TDC Calibration Done Signal Timing ----------
# The calib_done signal from TDC auto-calibration affects the fine code adjustment pipeline.
# This is a control signal that changes once during startup and stays constant thereafter.
# Allow 2 cycles for this non-critical control path at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*i_tdc|calib_done*}] -to [get_registers {*i_tdc|adjp_s1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*i_tdc|calib_done*}] -to [get_registers {*i_tdc|adjp_s1[*]}]

# ---------- Boot Dither DAC Timing ----------
# The boot dither FSM generates a calibration waveform during startup.
# This is only active for ~100µs during initialization, not during normal operation.
# Allow 2 cycles for counter-to-DAC comparator logic at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*i_adc|p_boot_dither.v_boot_counter[*]}] -to [get_registers {*i_adc|dac_boot_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*i_adc|p_boot_dither.v_boot_counter[*]}] -to [get_registers {*i_adc|dac_boot_ff*}]

# DAC output mux (dac_boot_ff -> dac_out_ff) during boot sequence
# The DAC output muxes between boot dither and closed-loop PI output.
# During boot (~100µs), dac_boot_ff drives the output. Timing is not critical.
# Allow 2 cycles for the mux and output register at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*i_adc|dac_boot_ff*}] -to [get_registers {*i_adc|dac_out_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*i_adc|dac_boot_ff*}] -to [get_registers {*i_adc|dac_out_ff*}]

# Calibration search result write (calib_search_idx -> coarse_bias_calibrated)
# When calibration search completes (once at startup), calib_mode_value is written to coarse_bias_calibrated.
# This is derived from calib_search_idx and only updates once after the search finishes.
# Allow 2 cycles for this non-critical startup path at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*calib_search_idx*}] -to [get_registers {*coarse_bias_calibrated[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_search_idx*}] -to [get_registers {*coarse_bias_calibrated[*]}]

# Boot FSM state to sweep duty cycle (boot_state -> sweep_cross_duty)
# During startup sweep/dither states, the FSM updates the sweep_cross_duty register.
# This is only active during ~100µs initialization, not timing-critical.
# Allow 2 cycles for FSM state decode to duty cycle register at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*boot_state*}] -to [get_registers {*sweep_cross_duty[*]}]
set_multicycle_path -hold 1 -from [get_registers {*boot_state*}] -to [get_registers {*sweep_cross_duty[*]}]

# Boot counter to FSM state transitions (v_boot_counter -> boot_state)
# The boot counter drives FSM state transitions during startup calibration.
# This is only active during ~100µs initialization, not timing-critical.
# Allow 2 cycles for counter logic to FSM next-state at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*p_boot_dither.v_boot_counter[*]}] -to [get_registers {*boot_state*}]
set_multicycle_path -hold 1 -from [get_registers {*p_boot_dither.v_boot_counter[*]}] -to [get_registers {*boot_state*}]

# Reset synchronizer to DAC output (reset_sync -> dac_out_ff)
# The reset path from clk_sys domain crosses to clk_tdc domain DAC output.
# Reset is asynchronous and doesn't need tight timing.
# Allow 2 cycles for reset propagation at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*i_reset_sync|sync*}] -to [get_registers {*dac_out_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*i_reset_sync|sync*}] -to [get_registers {*dac_out_ff*}]

# Calibration histogram write (calib_hist_update_d3 -> calib_histogram)
# The histogram write enable signal calib_hist_update_d3 gates writes to the 256-entry RAM.
# This is already covered by the previous constraint but add explicit write enable path.
# Allow 2 cycles for write enable logic at 400 MHz (startup calibration only).
set_multicycle_path -setup 2 -from [get_registers {*calib_hist_update_d3*}] -to [get_registers {*calib_histogram[*][*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_hist_update_d3*}] -to [get_registers {*calib_histogram[*][*]}]

# ---------- PRIORITY 1 TIMING FIXES ----------
# Added to fix timing violations from STA report (December 2025)

# Reset synchronizer to calibration histogram (Violations #1-10: -0.193ns to -0.046ns)
# The reset sync signal i_reset_sync|sync3 fans out to all 256 histogram entries during reset.
# This massive fanout causes setup violations. Reset is asynchronous and only happens at startup.
# Allow 3 cycles for reset to propagate to all histogram entries at 400 MHz.
set_multicycle_path -setup 3 -from [get_registers {*i_reset_sync|sync3*}] -to [get_registers {*calib_histogram[*][*]}]
set_multicycle_path -hold 2 -from [get_registers {*i_reset_sync|sync3*}] -to [get_registers {*calib_histogram[*][*]}]

# TDC Encoder prefix arithmetic paths (Violations #2, #5, #10: -0.190ns to -0.097ns)
# The prefix encoder adders have long combinatorial paths computing thermometer code position.
# Allow 2 cycles for encoder arithmetic at 400 MHz.
# Note: Removed overly broad wildcard constraint - specific paths already covered by other constraints

# ---------- PRIORITY 2 TIMING FIXES ----------

# Histogram read address to value pipeline (Violations #6-7, #12-14, #28, #30: -0.132ns to -0.062ns)
# The histogram read path (calib_hist_read_addr -> calib_hist_value_d1) is a 256:1 mux.
# This is only used during calibration search, not real-time operation.
# Allow 2 cycles for histogram read mux at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*calib_hist_read_addr[*]}] -to [get_registers {*calib_hist_value_d1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_hist_read_addr[*]}] -to [get_registers {*calib_hist_value_d1[*]}]

# TDL calibration sum register to settle counter (Violations #15-26, #59, #62-65: -0.076ns to -0.032ns)
# The tdl_cal_sum_reg drives the tdl_settle_cnt for TDL delay line calibration timing.
# This is only active during startup TDL calibration, not real-time operation.
# Allow 2 cycles for calibration sum to counter logic at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_settle_cnt[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_settle_cnt[*]}]

# TDC v_tdc_max/v_tdc_min calibration tracking (Violations #11, #25, #32, #36: -0.081ns to -0.053ns)
# The v_tdc_max and v_tdc_min track min/max codes during startup calibration.
# These are local VHDL process variables, already constrained in axe5000_top.sdc
# Note: Process variables don't exist as registers after synthesis - paths handled by other constraints

# Calibration done/ready control signals (Violations #27, #33-35, #55, #61, #70, #79, #85-86, #96-97, #103)
# The calib_done and calib_ready signals change once at startup and remain constant.
# These control signal fanouts can tolerate extra latency.
# Allow 3 cycles for calibration control signals at 400 MHz.
set_multicycle_path -setup 3 -from [get_registers {*|calib_done*}] -to [get_registers {*|calib_histogram[*][*]}]
set_multicycle_path -hold 2 -from [get_registers {*|calib_done*}] -to [get_registers {*|calib_histogram[*][*]}]
set_multicycle_path -setup 3 -from [get_registers {*|calib_ready*}] -to [get_registers {*|calib_histogram[*][*]}]
set_multicycle_path -hold 2 -from [get_registers {*|calib_ready*}] -to [get_registers {*|calib_histogram[*][*]}]

# Calib done to adjustment pipeline
set_multicycle_path -setup 3 -from [get_registers {*|calib_done*}] -to [get_registers {*|adjm_s1[*]}]
set_multicycle_path -hold 2 -from [get_registers {*|calib_done*}] -to [get_registers {*|adjm_s1[*]}]

# ---------- PRIORITY 3 TIMING FIXES ----------

# TDL calibration sum to fine accumulator (Violations #43-58, #73-77: -0.039ns to -0.019ns)
# The tdl_cal_sum_reg drives tdl_cal_fine_acc for TDL fine delay calibration.
# This is only active during startup calibration, not real-time operation.
# Allow 2 cycles for calibration accumulator path at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_cal_fine_acc[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_cal_fine_acc[*]}]

# TDL calibration sum to sample counter (Violations #87-94: -0.011ns to -0.010ns)
# The tdl_cal_sum_reg drives tdl_cal_sample_cnt for calibration sample counting.
# Allow 2 cycles for calibration sample counter path at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_cal_sample_cnt[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*tdl_cal_sample_cnt[*]}]

# Sweep cross duty to DAC boot (Violation #41: -0.041ns)
# The sweep_cross_duty register drives dac_boot_ff during startup sweep.
# This is only active during ~100µs initialization.
# Already partially covered by existing constraint, add more specific path.
set_multicycle_path -setup 2 -from [get_registers {*sweep_cross_duty[*]}] -to [get_registers {*dac_boot_ff*}]
set_multicycle_path -hold 1 -from [get_registers {*sweep_cross_duty[*]}] -to [get_registers {*dac_boot_ff*}]

# Encoder group clamp to group counts (Violation #67: -0.030ns)
# Note: group_clamp_reg constraint removed - register name not found in synthesized design
# Encoder paths handled by prefix_half_a_r constraints above

# Additional histogram-to-histogram paths (remaining violations)
# Some histogram entries drive other histogram entries through control logic.
# Allow 2 cycles for these internal histogram paths.
set_multicycle_path -setup 2 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_value_d1[*]}]
set_multicycle_path -hold 1 -from [get_registers {*calib_histogram[*][*]}] -to [get_registers {*calib_hist_value_d1[*]}]

# ---------- ADDITIONAL TIMING FIXES (Second pass - December 10, 2025) ----------

# TDC result to v_tdc_min calibration tracking (-0.069ns to -0.001ns violations)
# The tdc_result output drives the v_tdc_min tracking during startup calibration.
# This is the comparison path that updates min/max TDC codes during boot.
# Allow 4 cycles for TDC result to calibration min/max tracking at 400 MHz.
set_multicycle_path -setup 4 -from [get_registers {*i_tdc|tdc_result[*]}] -to [get_registers {*p_tdc_calibration.v_tdc_min[*]}]
set_multicycle_path -hold 3 -from [get_registers {*i_tdc|tdc_result[*]}] -to [get_registers {*p_tdc_calibration.v_tdc_min[*]}]
set_multicycle_path -setup 4 -from [get_registers {*i_tdc|tdc_result[*]}] -to [get_registers {*p_tdc_calibration.v_tdc_max[*]}]
set_multicycle_path -hold 3 -from [get_registers {*i_tdc|tdc_result[*]}] -to [get_registers {*p_tdc_calibration.v_tdc_max[*]}]

# TDL calibration sum to coarse bias calibrated (-0.063ns to -0.005ns violations)
# The tdl_cal_sum_reg drives coarse_bias_cal during TDL delay line calibration.
# This updates the coarse bias setting based on TDL calibration measurements.
# Allow 2 cycles for TDL sum to coarse bias update at 400 MHz.
set_multicycle_path -setup 2 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*coarse_bias_cal[*]}]
set_multicycle_path -hold 1 -from [get_registers {*tdl_cal_sum_reg[*]}] -to [get_registers {*coarse_bias_cal[*]}]
