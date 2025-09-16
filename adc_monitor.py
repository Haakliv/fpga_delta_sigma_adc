#!/usr/bin/env python3
"""Minimal ADC UART Monitor - Reads UART and calculates real ADC values"""

import serial
import re
import time

# ADC Configuration (Based on FPGA Implementation)
ADC_BITS = 16   # 16-bit delta-sigma ADC output
VREF = 1.2      # 1.2V I/O standard (Agilex 5 differential signaling)
GAIN = 1.0      # ADC gain (no amplification)

def adc_to_voltage(adc_raw):
    """Convert raw ADC value to voltage"""
    max_val = (1 << ADC_BITS) - 1
    return (adc_raw / max_val) * VREF * GAIN

def find_uart_port():
    """Try to find JTAG UART port"""
    import serial.tools.list_ports
    for port in serial.tools.list_ports.comports():
        if 'UART' in port.description.upper() or 'JTAG' in port.description.upper():
            return port.device
    return 'COM3'  # Default fallback

def main():
    # Find and connect to UART
    port = find_uart_port()
    print(f"Connecting to {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print("Connected! Monitoring ADC data...")
        print("Format: [Time] Raw: 0x12345678 | Voltage: 1.234V")
        print("-" * 50)
        
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Look for ADC data pattern: "ADC Data: 0x12345678"
            match = re.search(r'ADC Data:\s*0x([0-9A-Fa-f]+)', line)
            if match:
                raw_hex = match.group(1)
                raw_value = int(raw_hex, 16)
                voltage = adc_to_voltage(raw_value)
                timestamp = time.strftime("%H:%M:%S")
                
                print(f"[{timestamp}] Raw: 0x{raw_hex.upper():>8} | Voltage: {voltage:.3f}V")
            
            # Also print other system messages
            elif line and not line.startswith('---'):
                print(f"System: {line}")
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()