#!/usr/bin/env python3
"""
Monitor ADC status register to check comparator bitstream activity
Run this while your signal generator is connected
"""

import sys
import time

# This is a template - you'll need to adapt for your actual hardware interface
# The key is to read the status register repeatedly and check bit 0

print("=" * 60)
print("ADC Bitstream Monitor")
print("=" * 60)
print("\nThis script should read your ADC status register.")
print("Status register bit 0 = live comparator output (synchronized)")
print("\nExpected with 650mV DC + 100mVpp:")
print("  - Should toggle frequently (not stuck at 0 or 1)")
print("  - Average around 50% (duty cycle)")
print("\nIf stuck at 0 or 1:")
print("  - Check DAC output pin connection to RC network")
print("  - Check RC integrator components")  
print("  - Verify comparator input pins")
print("\n" + "=" * 60)

# TODO: Add your actual hardware read function here
# For example, via JTAG, UART command, or memory-mapped read
# status_reg = read_adc_register(C_ADC_REG_STATUS)  
# comparator_bit = status_reg & 0x01

print("\nNOTE: You need to implement the hardware read function")
print("      to actually monitor the bitstream.")
print("\nSuggested checks:")
print("1. Measure DAC output pin with oscilloscope")
print("2. Measure RC integrator output (comp- input)")  
print("3. Verify comp+ sees your 650mV + 100mVpp signal")
print("4. Check that bitstream toggles (not DC)")
