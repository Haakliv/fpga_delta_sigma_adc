# ************************************************************************
# Pin Assignment File for AXE5000 Delta-Sigma ADC
# Based on AXE5000 Development Kit pinout
# ************************************************************************

# Clock and Reset
set_location_assignment PIN_Y15 -to CLK_25M_C
set_location_assignment PIN_T12 -to CPU_RESETn

# UART (same as reference design)
set_location_assignment PIN_U9 -to UART_TX
set_location_assignment PIN_T9 -to UART_RX

# Delta-Sigma ADC connections
# These pins should connect to your analog front-end
set_location_assignment PIN_A17 -to ANALOG_IN    # Connect to comparator output
set_location_assignment PIN_B17 -to DAC_OUT      # Connect to integrator input

# Debug/Test pin
set_location_assignment PIN_C17 -to TEST_PIN

# I/O Standards
set_instance_assignment -name IO_STANDARD "1.8 V" -to CLK_25M_C
set_instance_assignment -name IO_STANDARD "1.8 V" -to CPU_RESETn
set_instance_assignment -name IO_STANDARD "1.8 V" -to UART_TX
set_instance_assignment -name IO_STANDARD "1.8 V" -to UART_RX
set_instance_assignment -name IO_STANDARD "1.8 V" -to ANALOG_IN
set_instance_assignment -name IO_STANDARD "1.8 V" -to DAC_OUT
set_instance_assignment -name IO_STANDARD "1.8 V" -to TEST_PIN

# Clock constraints
create_clock -name CLK_25M_C -period 40.000 [get_ports CLK_25M_C]
derive_pll_clocks
derive_clock_uncertainty
