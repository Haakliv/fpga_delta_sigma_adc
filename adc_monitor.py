#!/usr/bin/env python3
"""Simple UART Monitor - Displays ADC data from NIOS-V (voltage conversion done in C)"""

import serial
import time

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
        print("Note: Voltage conversion is handled by NIOS-V C code")
        print("-" * 60)
        
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line:  # Print all non-empty lines
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] {line}")
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()