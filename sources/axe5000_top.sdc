# 
# SPDX-FileCopyrightText: Copyright (C) 2025 Arrow Electronics, Inc. 
# SPDX-License-Identifier: MIT-0 
#

#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3

#**************************************************************
# Create Clock
#**************************************************************

# Base oscillator (25 MHz pin from board)
create_clock -name {clk_25m} -period 40.000 [get_ports {CLK_25M_C}]

# For Agilex 5, derive_pll_clocks is not supported - PLLs are handled automatically
# derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# Virtual clocks for async outputs
create_clock -name VCLK_UART        -period 100.000
create_clock -name VCLK_DAC         -period 100.000
create_clock -name VCLK_TEST_PIN    -period 100.000

# JTAG TCK clock for debugging (typical JTAG frequency)
create_clock -name {altera_reserved_tck} -period 100.000 -waveform { 0.000 50.000 } [get_ports {altera_reserved_tck}]


# JTAG interface constraints
set_input_delay -clock [get_clocks {altera_reserved_tck}] -max 20.0 [get_ports {altera_reserved_tdi}]
set_input_delay -clock [get_clocks {altera_reserved_tck}] -min 10.0 [get_ports {altera_reserved_tdi}]
set_input_delay -clock [get_clocks {altera_reserved_tck}] -max 20.0 [get_ports {altera_reserved_tms}]
set_input_delay -clock [get_clocks {altera_reserved_tck}] -min 10.0 [get_ports {altera_reserved_tms}]
set_output_delay -clock [get_clocks {altera_reserved_tck}] -max 20.0 [get_ports {altera_reserved_tdo}]
set_output_delay -clock [get_clocks {altera_reserved_tck}] -min 10.0 [get_ports {altera_reserved_tdo}]

#**************************************************************
# Set False Path
#**************************************************************

# Input (button)
set_false_path -from [get_ports USER_BTN]

# Output (UART)
set_output_delay -clock [get_clocks VCLK_UART] -max 0.0 -source_latency_included [get_ports {UART_TX}]
set_output_delay -clock [get_clocks VCLK_UART] -min 0.0 -source_latency_included [get_ports {UART_TX}]
set_false_path -to   [get_ports UART_TX]

# Output (DAC)
set_output_delay -clock [get_clocks VCLK_DAC]  -max 0.0 -source_latency_included [get_ports {DAC_OUT}]
set_output_delay -clock [get_clocks VCLK_DAC]  -min 0.0 -source_latency_included [get_ports {DAC_OUT}]
set_false_path -to   [get_ports DAC_OUT]

# DIP switches (asynchronous user inputs)
#set_false_path -from [get_ports {DIP_SW[*]}]

# Delta-Sigma ADC analog input (asynchronous comparator output)
set_false_path -from [get_ports {ANALOG_IN}]

# NIOS-V JTAG debugging paths for NIOS-V processor (more specific wildcards)
set_false_path -from [get_registers "*jtag_master*"] -to [get_registers "*"]
set_false_path -from [get_registers "*"] -to [get_registers "*jtag_master*"]
#set_false_path -from [get_registers "*dbg_mod*"] -to [get_registers "*"]
#set_false_path -from [get_registers "*"] -to [get_registers "*dbg_mod*"]

# JTAG false paths to all other clock domains
#set_false_path -from [get_clocks {altera_reserved_tck}] -to [get_clocks *]
set_false_path -from [get_clocks *] -to [get_clocks {altera_reserved_tck}]

# Test pin (debug/test output, no timing requirements) - specific constraint
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -max 0.0 -source_latency_included [get_ports {TEST_PIN}]
set_output_delay -clock [get_clocks VCLK_TEST_PIN] -min 0.0 -source_latency_included [get_ports {TEST_PIN}]
set_false_path -to [get_ports {TEST_PIN}]

#**************************************************************
# Design Rule Waivers
#**************************************************************

# Waive multiple reset synchronizer warning for NIOS-V system
# This is expected behavior for complex Intel IP with multiple subsystems
# set_design_assistant_rule_status -rule RDC-50003 -status WAIVED

# Waive intra-clock false path synchronizer warning for NIOS-V JTAG debug infrastructure
# These are intentional design patterns in Intel IP for JTAG debugging functionality
# Timing is managed through handshaking protocols rather than strict timing constraints
# set_design_assistant_rule_status -rule CDC-50101 -status WAIVED

# Suppress invalid JTAG Atlantic constraints warnings
# These are auto-generated constraints from Intel IP that may not apply to this design
# set_design_assistant_rule_status -rule TMC-20025 -status WAIVED
