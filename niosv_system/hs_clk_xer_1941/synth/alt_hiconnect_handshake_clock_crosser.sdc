# (C) 2001-2025 Altera Corporation. All rights reserved.
# Your use of Altera Corporation's design tools, logic functions and other 
# software and tools, and its AMPP partner logic functions, and any output 
# files from any of the foregoing (including device programming or simulation 
# files), and any associated documentation or information are expressly subject 
# to the terms and conditions of the Altera Program License Subscription 
# Agreement, Altera IP License Agreement, or other applicable 
# license agreement, including, without limitation, that your use is for the 
# sole purpose of programming logic devices manufactured by Altera and sold by 
# Altera or its authorized distributors.  Please refer to the applicable 
# agreement for further details.


# (C) 2001-2020 Intel Corporation. All rights reserved.
# Your use of Intel Corporation's design tools, logic functions and other 
# software and tools, and its AMPP partner logic functions, and any output 
# files from any of the foregoing (including device programming or simulation 
# files), and any associated documentation or information are expressly subject 
# to the terms and conditions of the Intel Program License Subscription 
# Agreement, Intel FPGA IP License Agreement, or other applicable 
# license agreement, including, without limitation, that your use is for the 
# sole purpose of programming logic devices manufactured by Intel and sold by 
# Intel or its authorized distributors.  Please refer to the applicable 
# agreement for further details.


# (C) 2001-2018 Intel Corporation. All rights reserved.
# Your use of Intel Corporation's design tools, logic functions and other 
# software and tools, and its AMPP partner logic functions, and any output 
# files from any of the foregoing (including device programming or simulation 
# files), and any associated documentation or information are expressly subject 
# to the terms and conditions of the Intel Program License Subscription 
# Agreement, Intel FPGA IP License Agreement, or other applicable 
# license agreement, including, without limitation, that your use is for the 
# sole purpose of programming logic devices manufactured by Intel and sold by 
# Intel or its authorized distributors.  Please refer to the applicable 
# agreement for further details.



# -----------------------------------------------------------------------------
# Altera timing constraints for Avalon clock domain crossing (CDC) paths.
# The purpose of these constraints is to remove the false paths and replace with timing bounded 
# requirements for compilation.
#
# ***Important note *** 
#
# The clocks involved in this transfer must be kept synchronous and no false path
# should be set on these paths for these constraints to apply correctly.
# -----------------------------------------------------------------------------

set crosser_entity "clock_xer"
set_max_delay -from [get_registers *${crosser_entity}|in_data_buffer[*] ] -to [get_registers *${crosser_entity}|out_data_buffer[*] ] 100
set_min_delay -from [get_registers *${crosser_entity}|in_data_buffer[*] ] -to [get_registers *${crosser_entity}|out_data_buffer[*] ] -100

set_max_delay -from [get_registers *${crosser_entity}|out_data_toggle_flopped_n ] -to [get_registers *${crosser_entity}|out_to_in_synchronizer|din_s1 ] 100
set_min_delay -from [get_registers *${crosser_entity}|out_data_toggle_flopped_n ] -to [get_registers *${crosser_entity}|out_to_in_synchronizer|din_s1 ] -100

set_max_delay -from [get_registers *${crosser_entity}|in_data_toggle ] -to          [get_registers *${crosser_entity}|in_to_out_synchronizer|din_s1 ] 100
set_min_delay -from [get_registers *${crosser_entity}|in_data_toggle ] -to          [get_registers *${crosser_entity}|in_to_out_synchronizer|din_s1 ] -100


#**************************************************************
# false path in clrn
#**************************************************************

set aclr_collection_wr [get_pins -compatibility_mode -nocase -nowarn  *${crosser_entity}*|clrn]
set clrn_counter_wr [get_collection_size $aclr_collection_wr]

if {$clrn_counter_wr > 0} {
set_false_path -to [get_pins -compatibility_mode -nocase  *${crosser_entity}*|clrn]
}



# -----------------------------------------------------------------------------
# This procedure constrains the skew between the token and data bits, and should
# be called from the top level SDC, once per instance of the clock crosser.
#
# The hierarchy path to the handshake clock crosser instance is required as an 
# argument.
#
# In practice, the token and data bits tend to be placed close together, making
# excessive skew less of an issue.
# -----------------------------------------------------------------------------
proc constrain_alt_handshake_clock_crosser_skew { path } {

    set in_regs  [ get_registers $path|*clock_xer|in_data_buffer* ] 
    set out_regs [ get_registers $path|*clock_xer|out_data_buffer* ] 

    set in_regs [ add_to_collection $in_regs  [ get_registers $path|*clock_xer|in_data_toggle ] ]
    set out_regs [ add_to_collection $out_regs [ get_registers $path|*clock_xer|in_to_out_synchronizer|din_s1 ] ]

    set_max_skew -from $in_regs -to $out_regs -get_skew_value_from_clock_period dst_clock_period -skew_value_multiplier 0.8
}

