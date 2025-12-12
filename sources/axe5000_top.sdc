# SPDX-FileCopyrightText: Copyright (C) 2025 Arrow
# SPDX-License-Identifier: MIT-0

# ---------- Time format ----------
set_time_format -unit ns -decimal_places 3

# ---------- Base board clock ----------
# 25 MHz input pin
create_clock -name CLK_25M -period 40.000 [get_ports {CLK_25M_C}]

# NOTE: The PLL output clocks already exist in TimeQuest with these names:
#   i_niosv|iopll|iopll_outclk0  (100 MHz)
#   i_niosv|iopll|iopll_outclk1  (400 MHz)
#   i_niosv|iopll|iopll_outclk2  (2 MHz)
# Use get_clocks on those names, do NOT recreate them. (Agilex 5 auto-creates.)


# ---------- Virtual/output interface clocks ----------
create_clock -name VCLK_UART     -period 100.000
create_clock -name VCLK_TEST_PIN -period 100.000

# ---------- JTAG ----------
# Note: JTAG interface (altera_reserved_*) is not user-accessible in this design
# JTAG timing is handled automatically by Intel FPGA infrastructure

# ---------- Clock domain relationships ----------
# NOTE: For Agilex 5, PLL clocks are auto-created by IP SDC files.
# Clock domain crossing constraints moved to axe5000_top_late.sdc to ensure
# they are processed AFTER the IP SDC creates the PLL output clocks.

# Optional aliases (handy in reports); comment out if you don't want extra names
# create_generated_clock -name SYS_100M  -source [get_clocks {CLK_25M}] -divide_by 0.25 [get_clocks {i_niosv|iopll|iopll_outclk0}]
# create_generated_clock -name TDC_400M  -source [get_clocks {CLK_25M}] -divide_by 0.0625 [get_clocks {i_niosv|iopll|iopll_outclk1}]
# create_generated_clock -name REF_2M    -source [get_clocks {CLK_25M}] -multiply_by 0.08  [get_clocks {i_niosv|iopll|iopll_outclk2}]

# ---------- Obvious false paths ----------
# Async user input
set_false_path -from [get_ports {USER_BTN}]

# External trigger input (optional hardware trigger for burst capture)
set_input_delay -clock [get_clocks CLK_25M] -max 0.0 -source_latency_included [get_ports {TRIGGER_IN}]
set_input_delay -clock [get_clocks CLK_25M] -min 0.0 -source_latency_included [get_ports {TRIGGER_IN}]
set_false_path -from [get_ports {TRIGGER_IN}]

# UART RX input (asynchronous serial interface)
# Set nominal constraint with virtual clock, then false path
set_input_delay -clock [get_clocks VCLK_UART] -max 0.0 -source_latency_included [get_ports {UART_RX}]
set_input_delay -clock [get_clocks VCLK_UART] -min 0.0 -source_latency_included [get_ports {UART_RX}]
set_false_path -from [get_ports {UART_RX}]

# Self-test signal (ONLY SELF-TEST MODE)
# The test_signal intentionally crosses async to TDL to simulate external analog
#set_false_path -from [get_registers {*|test_signal}] -to [get_registers {*|i_tdc|tdl_bank_current*}]

# NOTE: CDC false paths moved to axe5000_top_late.sdc (processed after IP SDC creates PLL clocks)

# ---------- Simple board-output constraints ----------
# UART TX – treat as false path but add zero delays so tool knows it's "constrained"
set_output_delay -clock [get_clocks VCLK_UART]     -max 0.0 -source_latency_included [get_ports {UART_TX}]
set_output_delay -clock [get_clocks VCLK_UART]     -min 0.0 -source_latency_included [get_ports {UART_TX}]
set_false_path -to [get_ports {UART_TX}]

# Test pin (debug)
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -max 0.0 -source_latency_included [get_ports {TEST_PIN}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -min 0.0 -source_latency_included [get_ports {TEST_PIN}]
set_false_path -to [get_ports {TEST_PIN}]

# Debug LED (TDC valid indicator)
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -max 0.0 -source_latency_included [get_ports {LED1}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -min 0.0 -source_latency_included [get_ports {LED1}]
set_false_path -to [get_ports {LED1}]

# VADJ power control (static configuration pin)
set_output_delay -clock [get_clocks CLK_25M] -max 0.0 -source_latency_included [get_ports {VSEL_1V3}]
set_output_delay -clock [get_clocks CLK_25M] -min 0.0 -source_latency_included [get_ports {VSEL_1V3}]
set_false_path -to [get_ports {VSEL_1V3}]

# ---------- Analog Feedback Loop I/O (ANALOG_IN / FEEDBACK_OUT) ----------
# These pins form a continuous-time analog feedback loop for the delta-sigma ADC.
# The TDC samples the differential comparator output asynchronously relative to
# the 400 MHz TDC clock. The DAC output (FEEDBACK_OUT) is registered at 400 MHz.
#
# Virtual clock approach: Create a 400 MHz virtual clock to represent the TDC clock
# domain, then set relaxed input/output delays since this is an analog interface.
create_clock -name VCLK_ANALOG_400M -period 2.500

# ANALOG_IN: Analog input + differential comparator input (P-pin)
# This is sampled by the TDC asynchronously - set as false path with nominal constraint
set_input_delay -clock [get_clocks VCLK_ANALOG_400M] -max 0.0 -source_latency_included [get_ports {ANALOG_IN}]
set_input_delay -clock [get_clocks VCLK_ANALOG_400M] -min 0.0 -source_latency_included [get_ports {ANALOG_IN}]

# FEEDBACK_OUT: DAC output (feeds external RC filter back to ANALOG_IN comparator N-pin)
# Output path: Registered at 400 MHz TDC clock
# The RC-filtered signal returns to ANALOG_IN as the differential comparator N-pin
# Set as false path with nominal constraint (timing determined by analog loop, not I/O)
set_output_delay -clock [get_clocks VCLK_ANALOG_400M] -max 0.0 -source_latency_included [get_ports {FEEDBACK_OUT}]
set_output_delay -clock [get_clocks VCLK_ANALOG_400M] -min 0.0 -source_latency_included [get_ports {FEEDBACK_OUT}]

# The analog loop timing is determined by the TDC architecture, not I/O timing
# Set these as false paths to avoid over-constraining the analog interface
set_false_path -from [get_ports {ANALOG_IN}]
set_false_path -to [get_ports {FEEDBACK_OUT}]

# ---------- Boot Calibration Counter (Non-Critical Startup Logic) ----------
set_multicycle_path -from [get_registers "*v_boot_counter*"] -to [get_registers "*v_boot_counter*"] -setup -start 12
set_multicycle_path -from [get_registers "*v_boot_counter*"] -to [get_registers "*v_boot_counter*"] -hold -start 11

# ---------- TDC Result RTM Register (Pipelined TDC Output) ----------
# The tdc_result register has a long combinatorial path from the TDC pipeline stages.
# This is the final output register of the TDC and doesn't need single-cycle timing.
# Allow 2 cycles for this path (setup = 2, hold = 1).
set_multicycle_path -from [get_registers "*tdc_result*"] -to [get_registers "*tdc_center_tdc*"] -setup -end 4
set_multicycle_path -from [get_registers "*tdc_result*"] -to [get_registers "*tdc_center_tdc*"] -hold -end 3

# ---------- TDL Calibration Sample Counter (Non-Critical Calibration Logic) ----------
# Note: fine_at_comp constraint removed - register not found in current design
# TDL calibration paths handled by other constraints in axe5000_top_late.sdc

# ---------- TDC Calibration Min/Max Tracking (Non-Critical Boot Calibration) ----------
# The v_tdc_min and v_tdc_max variables in p_tdc_calibration track min/max TDC codes
# during boot calibration. These are local variables that update asynchronously and
# feed into tdc_center_tdc calculation. This is startup calibration only, not real-time.
# Allow 4 cycles for the comparison and center calculation logic.
set_multicycle_path -setup 4 -from [get_registers {*v_tdc_max*}] -to [get_registers {*tdc_center_tdc*}]
set_multicycle_path -hold 3 -from [get_registers {*v_tdc_max*}] -to [get_registers {*tdc_center_tdc*}]
set_multicycle_path -setup 4 -from [get_registers {*v_tdc_min*}] -to [get_registers {*tdc_center_tdc*}]
set_multicycle_path -hold 3 -from [get_registers {*v_tdc_min*}] -to [get_registers {*tdc_center_tdc*}]

# ---------- TDC Dual-Lobe Bias Selection Pipeline (Stage-1b to Stage-1c) ----------
# The p_stage1b process computes magnitude (abs_s9) of three bias adjustment candidates.
# The p_stage1c process performs min-of-three selection with tie-breaking logic.
# This is a heavily pipelined path that doesn't need single-cycle timing.
# Allow 2 cycles for the magnitude comparison and selection logic.
set_multicycle_path -setup 2 -from [get_registers {*adj*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -hold 1 -from [get_registers {*adj*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -setup 2 -from [get_registers {*mag*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -hold 1 -from [get_registers {*mag*_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -setup 2 -from [get_registers {*fine_s1b*}] -to [get_registers {*d_used_s1c*}]
set_multicycle_path -hold 1 -from [get_registers {*fine_s1b*}] -to [get_registers {*d_used_s1c*}]

# ---------- CIC Multi-bit Decimator Constraints ----------
# Comb filter stages run at decimated rate (once per 8192 cycles)
# The comb state machine takes 3 states total - allow 4 cycles per stage

# Comb stage 1: decimated -> comb1_out
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

# ---------- CIC Integrator Constraints ----------
# Integrators are gated by CE (ref_clock edge detect at 50MHz = 1/8 of 400MHz)
# Allow 2 cycles for integrator paths
set_multicycle_path -setup 2 -from [get_registers {*i_cic_multibit*int1*}] -to [get_registers {*i_cic_multibit*int2*}]
set_multicycle_path -hold 1 -from [get_registers {*i_cic_multibit*int1*}] -to [get_registers {*i_cic_multibit*int2*}]
set_multicycle_path -setup 2 -from [get_registers {*i_cic_multibit*int2*}] -to [get_registers {*i_cic_multibit*int3*}]
set_multicycle_path -hold 1 -from [get_registers {*i_cic_multibit*int2*}] -to [get_registers {*i_cic_multibit*int3*}]

# ---------- CIC Input Pipeline (TDC Combination to Integrator) ----------
# The cic_input_tdc signal combines DAC and TDC contributions before feeding the integrators.
# This path has 2 pipeline stages to meet 400MHz timing. Allow 2 cycles.
set_multicycle_path -setup 2 -from [get_registers {*cic_input_tdc*}] -to [get_registers {*i_cic_multibit*int1*}]
set_multicycle_path -hold 1 -from [get_registers {*cic_input_tdc*}] -to [get_registers {*i_cic_multibit*int1*}]

# ---------- CIC DSP Multiplier to Scale Pipeline ----------
# The CIC output scaling uses DSP multipliers (mult_0) that feed into the scale_pipe
# pipeline registers. These paths only need to complete once per decimation period.
# With decimation=128, the output valid rate is 50MHz/128 = 390kS/s, giving 128 cycles.
# Conservative setting: 8 cycles (still far more than needed).
set_multicycle_path -setup 8 -from [get_registers {*i_cic_multibit|mult_0*}] -to [get_registers {*i_cic_multibit|scale_pipe*}]
set_multicycle_path -hold 7 -from [get_registers {*i_cic_multibit|mult_0*}] -to [get_registers {*i_cic_multibit|scale_pipe*}]

# ---------- CIC Scale Pipeline Stages ----------  
# The scale_pipe registers form a multi-stage pipeline for output scaling.
# Each stage operates at the decimated output rate.
set_multicycle_path -setup 8 -from [get_registers {*i_cic_multibit|scale_pipe1*}] -to [get_registers {*i_cic_multibit|scale_pipe*}]
set_multicycle_path -hold 7 -from [get_registers {*i_cic_multibit|scale_pipe1*}] -to [get_registers {*i_cic_multibit|scale_pipe*}]

# ---------- Reset Synchronizer to CIC/TDC (Async Reset Domain Crossing) ----------
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

# ---------- TDC Calibration Histogram Pipeline ----------
# Calibration histogram read path has multi-cycle pipeline
set_multicycle_path -setup 4 -from [get_registers {*calib_hist_read_d1*}] -to [get_registers {*calib_mode_value*}]
set_multicycle_path -hold 3 -from [get_registers {*calib_hist_read_d1*}] -to [get_registers {*calib_mode_value*}]

# ---------- TDC Calibration Min/Max Self-Paths ----------
# The v_tdc_max comparison logic has long paths back to itself
set_multicycle_path -setup 4 -from [get_registers {*v_tdc_max*}] -to [get_registers {*v_tdc_max*}]
set_multicycle_path -hold 3 -from [get_registers {*v_tdc_max*}] -to [get_registers {*v_tdc_max*}]

# ---------- Use Closed Loop TDC to DAC Output ----------
# The use_closed_loop_tdc flag controls DAC output mux - slow control signal
set_multicycle_path -setup 4 -from [get_registers {*use_closed_loop_tdc*}] -to [get_registers {*dac_out_ff*}]
set_multicycle_path -hold 3 -from [get_registers {*use_closed_loop_tdc*}] -to [get_registers {*dac_out_ff*}]