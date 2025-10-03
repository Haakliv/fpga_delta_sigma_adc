#!/usr/bin/env python3
"""
Test script to decode ADC values at different DC offsets
Usage: python test_sweep.py
Just record the hex values from UART at different voltage offsets
"""

import sys

def decode_adc(hex_str):
    """Decode 4-digit hex to signed 16-bit value and approximate voltage"""
    try:
        # Parse hex string
        hex_val = int(hex_str, 16)
        
        # Convert to signed 16-bit
        if hex_val >= 0x8000:
            signed_val = hex_val - 0x10000
        else:
            signed_val = hex_val
        
        # Convert to approximate voltage (assuming full scale = ±1.3V for example)
        # Adjust this scale factor based on your actual calibration
        voltage = signed_val * (1.3 / 32768)
        
        return signed_val, voltage
    except ValueError:
        return None, None

if __name__ == "__main__":
    print("ADC DC Offset Sweep Test")
    print("=" * 60)
    print("\nInstructions:")
    print("1. Set signal generator to 100mVpp, 100Hz")
    print("2. Adjust DC offset from 300mV to 800mV in 50mV steps")
    print("3. Record the UART hex output at each offset")
    print("4. Enter the values below\n")
    print("Expected: Output should be near 0x0000 at the 'balance point'")
    print("          and increase/decrease on either side\n")
    print("=" * 60)
    print()
    
    # Example data entry
    offsets = [300, 350, 400, 420, 450, 500, 540, 600, 650, 700, 750, 800]
    
    print("DC Offset (mV) | Hex Value | Signed Dec | Approx Voltage")
    print("-" * 60)
    
    for offset in offsets:
        hex_input = input(f"{offset:4d} mV      | ").strip().upper()
        if hex_input:
            signed, voltage = decode_adc(hex_input)
            if signed is not None:
                print(f"             | 0x{hex_input:4s}    | {signed:6d}     | {voltage:+.4f}V")
            else:
                print(f"             | Invalid hex value")
        print("-" * 60)
