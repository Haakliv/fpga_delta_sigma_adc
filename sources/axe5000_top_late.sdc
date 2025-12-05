# SPDX-FileCopyrightText: Copyright (C) 2025 Arrow
# SPDX-License-Identifier: MIT-0

# ---------- CDC false paths to FIRST sync stage (non-intrusive; prevents bogus timing) ----------
# 2 MHz ref -> 400 MHz: first flop of 3-FF synchronizer
# PLL outputs both positive and inverted clock edges - both are valid sources
# Use wildcard to catch both the main clock and inverted variant
set_false_path -from [get_pins {i_niosv|iopll|iopll|tennm_ph2_iopll|out_clk[2]}] -to [get_registers {i_adc|ref_sync0}]

# Time DAC control CDC: clk_tdc -> clk_sys via 3-FF synchronizer
# Source is loop_output registers (time_dac_ctrl_tdc is combinatorial)
set_false_path -from [get_registers {i_adc|loop_output[*]}] -to [get_registers {i_adc|time_dac_ctrl_sync0}]

# Reset CDC: clk_sys -> clk_tdc (reset synchronizer in tdc_quantizer)
# The reset_synchronizer entity handles this with a 4-FF chain
set_false_path -from [get_registers {i_niosv|rst_controller_001|alt_rst_sync_uq1|altera_reset_synchronizer_int_chain_out}] -to [get_registers {i_adc|*i_tdc*|i_reset_sync|sync0}]
