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

# Delta-Sigma ADC analog input (differential LVDS)
set_location_assignment PIN_V6 -to ANALOG_IN       ; # CRUVI analog input
set_instance_assignment -name IO_STANDARD "1.2-V TRUE DIFFERENTIAL SIGNALING" -to ANALOG_IN

# DAC Feedback Output - HSO (HS Serial Out)
set_location_assignment PIN_N6 -to DAC_OUT         ; # HSO (CRUVI pin 6)
set_instance_assignment -name IO_STANDARD "ADJUSTABLE" -to DAC_OUT

# Debug test pin
set_location_assignment PIN_W5 -to TEST_PIN        ; # Test/debug output
set_instance_assignment -name IO_STANDARD "1.2-V" -to TEST_PIN

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

# CPU Reset (from AXE5000 board)
set_location_assignment PIN_A11 -to CPU_RESETn     ; # Board reset button
set_instance_assignment -name IO_STANDARD "1.2-V" -to CPU_RESETn
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to CPU_RESETn

# Reference clock input (CRUVI REFCLK - if using external reference)
set_location_assignment PIN_AJ28 -to C_REFCLK      ; # REFCLK (CRUVI pin 11)
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to C_REFCLK

# UART connections (from AXE5000 board)
set_location_assignment PIN_AG23 -to UART_RX
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to UART_RX
set_location_assignment PIN_AG24 -to UART_TX
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to UART_TX

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

# TDC high-speed timing (tighter constraints) - only for future TDC signals
# Note: TDC timing constraints moved to axe5000_top.sdc to avoid conflicts
# Note: All timing constraints for delta-sigma ADC signals are in axe5000_top.sdc
