#!/usr/bin/env python3
"""Helper script to send UART commands to the FPGA.

Commands:
  T - Enable TDC monitor mode (sends TDC packets)
  A - Enable ADC sample mode (sends ADC samples)
  C - Capture samples
  D - Dump captured samples
"""

import sys
import serial
import serial.tools.list_ports

def find_uart_port():
    """Find the UART port."""
    for port in serial.tools.list_ports.comports():
        description = port.description.upper()
        if "UART" in description or "JTAG" in description or "USB" in description:
            return port.device
    return "COM3"

def send_command(cmd):
    """Send a single-byte command to the FPGA."""
    port = find_uart_port()
    print(f"Connecting to {port}...")
    
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            ser.write(cmd.encode('ascii'))
            print(f"Sent command: '{cmd}'")
            
            if cmd.upper() == 'T':
                print("TDC monitor mode enabled - expecting binary packets with 0xAA 0x55 header")
                print("Run: python tdc_monitor.py")
            elif cmd.upper() == 'A':
                print("ADC sample mode enabled - expecting ASCII hex samples")
                print("Run: python adc_monitor.py")
            elif cmd.upper() == 'C':
                print("Capture started - samples being buffered")
            elif cmd.upper() == 'D':
                print("Dump started - captured samples will stream over UART")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1].upper() not in ['T', 'A', 'C', 'D']:
        print("Usage: python uart_cmd.py <command>")
        print("")
        print("Commands:")
        print("  T - TDC monitor mode")
        print("  A - ADC sample mode")
        print("  C - Capture samples")
        print("  D - Dump captured samples")
        sys.exit(1)
    
    send_command(sys.argv[1].upper())
