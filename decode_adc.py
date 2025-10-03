#!/usr/bin/env python3
"""
Quick decoder for ADC UART hex output
Converts hex strings to signed 16-bit decimal values
"""

import sys

def decode_hex_to_signed16(hex_str):
    """Convert 4-digit hex to signed 16-bit integer"""
    # Remove any whitespace/newlines
    hex_str = hex_str.strip()
    
    # Convert to unsigned int
    unsigned_val = int(hex_str, 16)
    
    # Convert to signed (two's complement)
    if unsigned_val >= 0x8000:
        signed_val = unsigned_val - 0x10000
    else:
        signed_val = unsigned_val
    
    return signed_val

def main():
    print("ADC Hex Decoder (Ctrl+C to exit)")
    print("Expected input format: XXXX (4 hex digits)")
    print("-" * 50)
    
    if len(sys.argv) > 1:
        # Decode arguments
        for hex_val in sys.argv[1:]:
            try:
                signed = decode_hex_to_signed16(hex_val)
                voltage_estimate = (signed / 32768.0) * 1.0  # Assuming ±1V full scale
                print(f"0x{hex_val.upper():>4} = {signed:>6} = {voltage_estimate:+.6f}V (approx)")
            except ValueError:
                print(f"Invalid hex: {hex_val}")
    else:
        # Interactive mode
        print("\nEnter hex values (one per line):")
        try:
            while True:
                line = input("> ").strip()
                if not line:
                    continue
                    
                try:
                    signed = decode_hex_to_signed16(line)
                    voltage_estimate = (signed / 32768.0) * 1.0  # Assuming ±1V full scale
                    print(f"  Signed decimal: {signed:>6}")
                    print(f"  Voltage (est):  {voltage_estimate:+.6f}V")
                except ValueError:
                    print(f"  Error: Invalid hex format")
                    
        except (KeyboardInterrupt, EOFError):
            print("\nExiting...")

if __name__ == "__main__":
    main()
