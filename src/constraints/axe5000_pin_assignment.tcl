# ************************************************************************
# Pin Assignment for AXE5000 CRUVI HS Delta-Sigma ADC
# Allocates LVDS pairs for RC integrator and future TDC development
# ************************************************************************

# Project settings
set_global_assignment -name FAMILY "Agilex 5"
set_global_assignment -name DEVICE "A5ED065BB32E6SR0"

# ========================================================================
# DELTA-SIGMA ADC WITH RC INTEGRATOR (Current Implementation)
# ========================================================================

# ADC Channel 0 - Primary RC integrator (differential LVDS)
set_location_assignment PIN_N2 -to lvds_adc0_p     ; # A0_P (CRUVI pin 14)
set_location_assignment PIN_N1 -to lvds_adc0_n     ; # A0_N (CRUVI pin 16)
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_adc0_p
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_adc0_n
# Enable on-chip differential termination and hysteresis
set_instance_assignment -name DIFFERENTIAL_TERMINATION ON -to lvds_adc0_p
set_instance_assignment -name INPUT_HYSTERESIS ON -to lvds_adc0_p

# DAC Output for ADC Channel 0 feedback
set_location_assignment PIN_R2 -to dac_out0        ; # A1_P (CRUVI pin 20) - Single-ended
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to dac_out0

# ADC Channel 1 - Secondary RC integrator (differential LVDS)
set_location_assignment PIN_T2 -to lvds_adc1_p     ; # A2_P (CRUVI pin 26)
set_location_assignment PIN_T1 -to lvds_adc1_n     ; # A2_N (CRUVI pin 28)
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_adc1_p
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_adc1_n
# Enable on-chip differential termination and hysteresis
set_instance_assignment -name DIFFERENTIAL_TERMINATION ON -to lvds_adc1_p
set_instance_assignment -name INPUT_HYSTERESIS ON -to lvds_adc1_p

# DAC Output for ADC Channel 1 feedback
set_location_assignment PIN_V1 -to dac_out1        ; # A3_P (CRUVI pin 32) - Single-ended
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to dac_out1

# ========================================================================
# FUTURE TDC DEVELOPMENT (Time-to-Digital Converter)
# ========================================================================

# TDC Channel 0 - High-speed LVDS for parasitic R/C measurement
set_location_assignment PIN_T3 -to lvds_tdc0_p     ; # A4_P (CRUVI pin 38)
set_location_assignment PIN_R4 -to lvds_tdc0_n     ; # A4_N (CRUVI pin 40)
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_tdc0_p
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_tdc0_n
# Enable fast differential input for TDC
set_instance_assignment -name DIFFERENTIAL_TERMINATION ON -to lvds_tdc0_p
set_instance_assignment -name INPUT_HYSTERESIS ON -to lvds_tdc0_p

# TDC Channel 1 - Secondary TDC channel
set_location_assignment PIN_Y1 -to lvds_tdc1_p     ; # A5_P (CRUVI pin 44)
set_location_assignment PIN_W2 -to lvds_tdc1_n     ; # A5_N (CRUVI pin 46)
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_tdc1_p
set_instance_assignment -name IO_STANDARD "LVDS" -to lvds_tdc1_n
# Enable fast differential input for TDC
set_instance_assignment -name DIFFERENTIAL_TERMINATION ON -to lvds_tdc1_p
set_instance_assignment -name INPUT_HYSTERESIS ON -to lvds_tdc1_p

# TDC Control signals (if needed)
set_location_assignment PIN_V6 -to tdc_trigger     ; # B0_P (CRUVI pin 15) - Single-ended
set_location_assignment PIN_W5 -to tdc_enable      ; # B0_N (CRUVI pin 17) - Single-ended
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to tdc_trigger
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to tdc_enable

# ========================================================================
# SYSTEM CLOCKS AND CONTROL
# ========================================================================

# Main 25 MHz oscillator (from AXE5000 board)
set_location_assignment PIN_A7 -to CLK_25M_C
set_instance_assignment -name IO_STANDARD "1.2-V" -to CLK_25M_C

# Reference clock input (CRUVI REFCLK - if using external reference)
set_location_assignment PIN_AJ28 -to C_REFCLK      ; # REFCLK (CRUVI pin 11)
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to C_REFCLK

# UART connections (from AXE5000 board)
set_location_assignment PIN_AG23 -to UART_RXD
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to UART_RXD
set_location_assignment PIN_AG24 -to UART_TXD
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to UART_TXD

# Status LEDs (from AXE5000 board)
set_location_assignment PIN_AG21 -to LED1          ; # Board LED1
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to LED1
set_location_assignment PIN_AH22 -to RLED          ; # RGB Red LED
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to RLED
set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to RLED
set_location_assignment PIN_AK21 -to GLED          ; # RGB Green LED
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to GLED
set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to GLED
set_location_assignment PIN_AK20 -to BLED          ; # RGB Blue LED
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to BLED
set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to BLED

# User controls (from AXE5000 board)
set_location_assignment PIN_A12 -to USER_BTN       ; # User button
set_instance_assignment -name IO_STANDARD "1.2-V" -to USER_BTN
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to USER_BTN
set_location_assignment PIN_A14 -to DIP_SW[0]      ; # DIP switch 0
set_instance_assignment -name IO_STANDARD "1.2-V" -to DIP_SW[0]
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to DIP_SW[0]
set_location_assignment PIN_A13 -to DIP_SW[1]      ; # DIP switch 1
set_instance_assignment -name IO_STANDARD "1.2-V" -to DIP_SW[1]
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to DIP_SW[1]

# ========================================================================
# SPARE PINS FOR EXPANSION
# ========================================================================

# Additional LVDS pairs for future expansion
set_location_assignment PIN_U5 -to spare_lvds0_p   ; # B2_P (CRUVI pin 27)
set_location_assignment PIN_AC6 -to spare_lvds0_n  ; # B2_N (CRUVI pin 29)
set_location_assignment PIN_V3 -to spare_lvds1_p   ; # B3_P (CRUVI pin 33)
set_location_assignment PIN_W3 -to spare_lvds1_n   ; # B3_N (CRUVI pin 35)

# ========================================================================
# TIMING CONSTRAINTS
# ========================================================================

# Create clocks
create_clock -name CLK_25M_C -period 40.000 [get_ports CLK_25M_C]
create_clock -name C_REFCLK -period 10.000 [get_ports C_REFCLK]

# Standard clock derivation
derive_pll_clocks
derive_clock_uncertainty

# LVDS input timing
set_input_delay -clock CLK_25M_C -max 2.0 [get_ports lvds_adc0_p]
set_input_delay -clock CLK_25M_C -min 0.5 [get_ports lvds_adc0_p]
set_input_delay -clock CLK_25M_C -max 2.0 [get_ports lvds_adc1_p]
set_input_delay -clock CLK_25M_C -min 0.5 [get_ports lvds_adc1_p]

# TDC high-speed timing (tighter constraints)
set_input_delay -clock CLK_25M_C -max 0.5 [get_ports lvds_tdc0_p]
set_input_delay -clock CLK_25M_C -min 0.1 [get_ports lvds_tdc0_p]
set_input_delay -clock CLK_25M_C -max 0.5 [get_ports lvds_tdc1_p]
set_input_delay -clock CLK_25M_C -min 0.1 [get_ports lvds_tdc1_p]

# DAC output timing
set_output_delay -clock CLK_25M_C -max 5.0 [get_ports dac_out0]
set_output_delay -clock CLK_25M_C -min 1.0 [get_ports dac_out0]
set_output_delay -clock CLK_25M_C -max 5.0 [get_ports dac_out1]
set_output_delay -clock CLK_25M_C -min 1.0 [get_ports dac_out1]
