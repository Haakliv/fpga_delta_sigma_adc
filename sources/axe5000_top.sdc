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

create_clock -name {ClkIn} -period 40.000 -waveform { 0.000 20.000 } [get_ports {CLK_25M_C}]

# Create JTAG clock constraint for altera_reserved_tck
# JTAG TCK is typically 10-25 MHz, using 10 MHz (100ns period) for conservative timing
# altera_reserved_tck is an auto-detected node, so we constrain it directly
create_clock -name {altera_reserved_tck} -period 100.000 -waveform { 0.000 50.000 } [get_nodes {altera_reserved_tck}]

# Note: derive_pll_clocks not supported in Agilex 5
# derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

#**************************************************************
# Create Generated Clock
#**************************************************************


#**************************************************************
# Set False Path
#**************************************************************

# Reset is asynchronous - commented out since CPU_RESETn pin assignment is commented out
# set_false_path -from [get_ports {CPU_RESETn}]

#**************************************************************
# Input and Output Delay Constraints
#**************************************************************

# TEST_PIN is for debugging - less critical timing - commented out since pin is commented out
# set_output_delay -clock [get_clocks {ClkIn}] -max 10.0 [get_ports {TEST_PIN}]
# set_output_delay -clock [get_clocks {ClkIn}] -min 0.0 [get_ports {TEST_PIN}]

# Set false paths between unrelated clock domains
# JTAG clock domain is asynchronous to system clock - handled by clock groups below

#**************************************************************
# Set Clock Groups
#**************************************************************

# Define asynchronous clock groups
set_clock_groups -asynchronous -group [get_clocks {ClkIn}] -group [get_clocks {altera_reserved_tck}]

#**************************************************************
# JTAG Clock Domain Constraints
#**************************************************************

# Since we've defined the clocks as asynchronous groups, we don't need additional timing constraints
# The clock group assignment handles the relationship between these domains

#**************************************************************
# Additional Clock Domain Crossing Constraints
#**************************************************************

# Remove overridden constraints and replace with false paths for JTAG clock crossings
# Since we already defined clock groups as asynchronous, we don't need max_delay constraints

# Handle specific JTAG clock domain crossings more appropriately
# Set false paths for purely asynchronous paths between JTAG and system clock
set_false_path -from [get_registers {*|niosv_m_0|*|dbg_mod|dtm_inst|*_clk_xer|*|in_data_toggle}] -to [get_registers {*|niosv_m_0|*|dbg_mod|dtm_inst|*_clk_xer|*|in_to_out_synchronizer|din_s1}]
set_false_path -from [get_registers {*|jtag_master|*|jtag_phy_embedded_in_jtag_master|*|sink_crosser|in_data_toggle}] -to [get_registers {*|jtag_master|*|jtag_phy_embedded_in_jtag_master|*|sink_crosser|in_to_out_synchronizer|din_s1}]
set_false_path -from [get_registers {*|niosv_m_0|*|dbg_mod|dtm_inst|*_clk_xer|*|out_data_toggle_flopped_n}] -to [get_registers {*|niosv_m_0|*|dbg_mod|dtm_inst|*_clk_xer|*|out_to_in_synchronizer|din_s1}]
set_false_path -from [get_registers {*|jtag_master|*|jtag_phy_embedded_in_jtag_master|*|sink_crosser|in_data_buffer*}] -to [get_registers {*|jtag_master|*|jtag_phy_embedded_in_jtag_master|*|sink_crosser|out_data_buffer*}]
set_false_path -from [get_registers {*|niosv_m_0|*|dbg_mod|dtm_inst|*_clk_xer|*|in_data_buffer*}] -to [get_registers {*|niosv_m_0|*|dbg_mod|dtm_inst|*_clk_xer|*|out_data_buffer*}]


