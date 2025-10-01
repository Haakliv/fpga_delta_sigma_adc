# ************************************************************************
# Pin Assignment for AXE5000 CRUVI HS Delta-Sigma ADC
# Allocates LVDS pairs for RC integrator
# ************************************************************************
# ========================================================================
# DELTA-SIGMA ADC WITH RC INTEGRATOR (Current Implementation)
# ========================================================================

# Delta-Sigma ADC analog input (differential LVDS)
set_location_assignment PIN_V6 -to ANALOG_IN       ; # CRUVI analog input
set_instance_assignment -name IO_STANDARD "1.3-V TRUE DIFFERENTIAL SIGNALING" -to ANALOG_IN

# DAC Feedback Output - HSO (HS Serial Out)
# Critical ΣΔ loop path: pack register into IO cell for 1-cycle latency
set_location_assignment PIN_N6 -to DAC_OUT         ; # HSO (CRUVI pin 6)
set_instance_assignment -name IO_STANDARD "1.3-V" -to DAC_OUT
set_instance_assignment -name FAST_OUTPUT_REGISTER ON -to DAC_OUT
set_instance_assignment -name SLEW_RATE 0 -to DAC_OUT

# Debug test pin
set_location_assignment PIN_U4 -to TEST_PIN        ; # Test/debug output
set_instance_assignment -name IO_STANDARD "1.3-V" -to TEST_PIN
set_instance_assignment -name SLEW_RATE 0 -to TEST_PIN
# ========================================================================
# SYSTEM CLOCKS AND CONTROL
# ========================================================================

# Main 25 MHz oscillator (from AXE5000 board)
set_location_assignment PIN_A7 -to CLK_25M_C
set_instance_assignment -name IO_STANDARD "1.3-V" -to CLK_25M_C

# UART connections (from AXE5000 board), RX not used currently
#set_location_assignment PIN_AG23 -to UART_RX
#set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to UART_RX
set_location_assignment PIN_AG24 -to UART_TX
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to UART_TX
set_instance_assignment -name SLEW_RATE 2 -to UART_TX
set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to UART_TX

# Status LEDs (from AXE5000 board)
#set_location_assignment PIN_AG21 -to LED1          ; # Board LED1
#set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to LED1
#set_location_assignment PIN_AH22 -to RLED          ; # RGB Red LED
#set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to RLED
#set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to RLED
#set_location_assignment PIN_AK21 -to GLED          ; # RGB Green LED
#set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to GLED
#set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to GLED
#set_location_assignment PIN_AK20 -to BLED          ; # RGB Blue LED
#set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to BLED
#set_instance_assignment -name CURRENT_STRENGTH_NEW 9MA -to BLED

# User controls (from AXE5000 board)
set_location_assignment PIN_A12 -to USER_BTN       ; # User button
set_instance_assignment -name IO_STANDARD "1.2-V" -to USER_BTN
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to USER_BTN

#set_location_assignment PIN_A14 -to DIP_SW[0]      ; # DIP switch 0
#set_instance_assignment -name IO_STANDARD "1.2-V" -to DIP_SW[0]
#set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to DIP_SW[0]
#set_location_assignment PIN_A13 -to DIP_SW[1]      ; # DIP switch 1
#set_instance_assignment -name IO_STANDARD "1.2-V" -to DIP_SW[1]
#set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to DIP_SW[1]
