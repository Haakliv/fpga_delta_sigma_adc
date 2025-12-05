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

# ---------- JTAG TCK (so TimeQuest stops whining) ----------
# JTAG clock is asynchronous to all design clocks - constrain as false path
create_clock -name altera_reserved_tck -period 100.000 [get_ports {altera_reserved_tck}]
#set_false_path -from [get_clocks altera_reserved_tck] -to [get_clocks *] Doesnt work quartus doesnt find it
set_false_path -from [get_clocks *] -to [get_clocks altera_reserved_tck]

# JTAG data pins - set nominal delays and false paths (debug interface only)
set_input_delay -clock [get_clocks altera_reserved_tck] -max 0.0 [get_ports {altera_reserved_tdi}]
set_input_delay -clock [get_clocks altera_reserved_tck] -min 0.0 [get_ports {altera_reserved_tdi}]
set_input_delay -clock [get_clocks altera_reserved_tck] -max 0.0 [get_ports {altera_reserved_tms}]
set_input_delay -clock [get_clocks altera_reserved_tck] -min 0.0 [get_ports {altera_reserved_tms}]
set_output_delay -clock [get_clocks altera_reserved_tck] -max 0.0 [get_ports {altera_reserved_tdo}]
set_output_delay -clock [get_clocks altera_reserved_tck] -min 0.0 [get_ports {altera_reserved_tdo}]
set_false_path -from [get_ports {altera_reserved_tdi}]
set_false_path -from [get_ports {altera_reserved_tms}]
set_false_path -to [get_ports {altera_reserved_tdo}]

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

# JTAG is independent of all other clock domains
#set_false_path -from [get_clocks *] -to [get_clocks {altera_reserved_tck}]

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

# LED1 (debug - not timing-critical)
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -max 0.0 -source_latency_included [get_ports {LED1}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -min 0.0 -source_latency_included [get_ports {LED1}]
set_false_path -to [get_ports {LED1}]

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

# ---------- CIC Decimator Multi-Cycle Paths ----------
# The CIC filter only produces output every R (decimation) clock cycles.
# The scaling pipeline stages have R cycles to complete, not just 1.
# For R=384, this gives 384 clock cycles = 3.84us at 100MHz.
# We conservatively allow 4 cycles for the scaling stages.
#
# Note: These paths are between registered pipeline stages that only
# toggle every R cycles. The valid signals gate the pipeline progression.
set_multicycle_path -setup 4 -from [get_registers {*i_cic|scale_pipe1*}] -to [get_registers {*i_cic|scale_pipe2*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic|scale_pipe1*}] -to [get_registers {*i_cic|scale_pipe2*}]
set_multicycle_path -setup 4 -from [get_registers {*i_cic|scale_pipe2*}] -to [get_registers {*i_cic|y_out*}]
set_multicycle_path -hold 3 -from [get_registers {*i_cic|scale_pipe2*}] -to [get_registers {*i_cic|y_out*}]

# ---------- Boot Calibration Counter (Non-Critical Startup Logic) ----------
set_multicycle_path -from [get_registers "*v_boot_counter*"] -to [get_registers "*v_boot_counter*"] -setup -start 8
set_multicycle_path -from [get_registers "*v_boot_counter*"] -to [get_registers "*v_boot_counter*"] -hold -start 7