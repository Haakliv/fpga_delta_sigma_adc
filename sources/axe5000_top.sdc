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

create_clock -name {CLK_25M_C} -period 40.000 -waveform { 0.000 20.000 } [get_ports {CLK_25M_C}]

# For Agilex 5, derive_pll_clocks is not supported - PLLs are handled automatically
# derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

# JTAG TCK clock for debugging (typical JTAG frequency)
create_clock -name {altera_reserved_tck} -period 100.000 -waveform { 0.000 50.000 } [get_ports {altera_reserved_tck}]

# JTAG interface constraints
set_input_delay -clock [get_clocks {altera_reserved_tck}] -max 20.0 [get_ports {altera_reserved_tdi}]
set_input_delay -clock [get_clocks {altera_reserved_tck}] -min 10.0 [get_ports {altera_reserved_tdi}]
set_input_delay -clock [get_clocks {altera_reserved_tck}] -max 20.0 [get_ports {altera_reserved_tms}]
set_input_delay -clock [get_clocks {altera_reserved_tck}] -min 10.0 [get_ports {altera_reserved_tms}]
set_output_delay -clock [get_clocks {altera_reserved_tck}] -max 20.0 [get_ports {altera_reserved_tdo}]
set_output_delay -clock [get_clocks {altera_reserved_tck}] -min 10.0 [get_ports {altera_reserved_tdo}]

# Alternative: If JTAG is purely for debugging and timing is not critical
# set_false_path -to [get_ports {altera_reserved_tdo}]
# set_false_path -from [get_ports {altera_reserved_tdi altera_reserved_tms}]

# Clock domain crossing between JTAG and system clocks
#set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}] -group [get_clocks {CLK_25M_C}]

# JTAG false paths to all other clock domains
#set_false_path -from [get_clocks {altera_reserved_tck}] -to [get_clocks *]
set_false_path -from [get_clocks *] -to [get_clocks {altera_reserved_tck}]

#**************************************************************
# Create Generated Clock
#**************************************************************


#**************************************************************
# Set False Path
#**************************************************************

# UART signals (asynchronous serial communication via USB-to-UART bridge)
set_false_path -from [get_ports {UART_RX}]
set_false_path -to [get_ports {UART_TX}]

# DIP switches (asynchronous user inputs)
set_false_path -from [get_ports {DIP_SW[*]}]

# Delta-Sigma ADC analog input (asynchronous comparator output)
set_false_path -from [get_ports {ANALOG_IN}]

# Delta-Sigma DAC output - asynchronous 1-bit output with relaxed constraints
# Use main system clock for delay constraint, then set false path
set_output_delay -clock [get_clocks CLK_25M_C] -max 30.0 [get_ports DAC_OUT]
set_output_delay -clock [get_clocks CLK_25M_C] -min -10.0 [get_ports DAC_OUT]
set_false_path -to [get_ports {DAC_OUT}]

# NIOS-V JTAG debugging paths for NIOS-V processor (more specific wildcards)
set_false_path -from [get_registers "*jtag_master*"] -to [get_registers "*"]
set_false_path -from [get_registers "*"] -to [get_registers "*jtag_master*"]
set_false_path -from [get_registers "*dbg_mod*"] -to [get_registers "*"]
set_false_path -from [get_registers "*"] -to [get_registers "*dbg_mod*"]

# Test pin (debug/test output, no timing requirements) - specific constraint
#set_false_path -to [get_ports {TEST_PIN}]

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
