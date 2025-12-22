#!/usr/bin/env python3
"""Helper script to send UART commands to the FPGA.

Commands:
  T - Enable TDC monitor mode (sends TDC packets)
  A - Enable ADC sample mode (sends ADC samples)
  C - Capture samples
  D - Dump captured samples
  X - Toggle TDC contribution (on/off)
  E - Toggle EQ filter (on/off)
  L - Toggle LP filter (on/off)
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
            elif cmd.upper() == 'X':
                print("TDC contribution toggled (on<->off)")
                print("When OFF, ADC runs in CIC-only mode (no TDC fine correction)")
            elif cmd.upper() == 'E':
                print("EQ filter toggled (on<->off)")
                print("When OFF, CIC output bypasses equalizer (sinc^3 droop not compensated)")
            elif cmd.upper() == 'L':
                print("LP filter toggled (on<->off)")
                print("When OFF, signal bypasses lowpass filter (no anti-aliasing)")
            elif cmd.upper() == 'N':
                print("TDC sign toggled (negate on<->off)")
                print("When ON, TDC contribution sign is inverted (test sign correctness)")
            elif cmd.upper() == 'F':
                print("Fast dump mode toggled (4096 samples on<->off)")
                print("When ON, only last 4096 samples are dumped (~2.2s vs 69s)")
            elif cmd.upper() == 'T':
                print("Trigger mode toggled (edge<->level)")
                print("Edge: triggers on rising edge only")
                print("Level: triggers while input is high (use for DC testing)")
            elif cmd.upper() == 'B':
                print("Burst mode enabled")
                print("Captures to buffer, then dumps on command or auto after HW trigger")
            elif cmd.upper() == 'S':
                print("Stream mode enabled")
                print("Continuous output (may drop samples if UART busy)")
            elif cmd == '0':
                print("Full dump mode set (131072 samples)")
            elif cmd == '1':
                print("Fast dump mode set (4096 samples)")
            elif cmd == '2':
                print("TDC gain set to 1x (default)")
            elif cmd == '3':
                print("TDC gain set to 2x")
            elif cmd == '4':
                print("TDC gain set to 4x")
            elif cmd == '5':
                print("TDC gain set to 8x")
            elif cmd == '6':
                print("TDC gain set to 16x")
            elif cmd == '7':
                print("TDC gain set to 32x")
            elif cmd == '8':
                print("TDC gain set to 64x")
            elif cmd == '9':
                print("TDC gain set to 128x")
            elif cmd.upper() == 'M':
                print("TDC monitor mode enabled - expecting binary packets with 0xAA 0x55 header")
                print("Run: python tdc_monitor.py")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ['T', 'A', 'C', 'D', 'X', 'E', 'L', 'N', 'F', 'B', 'S', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'M']:
        print("Usage: python uart_cmd.py <command>")
        print("")
        print("Commands:")
        print("  C - Capture samples (software trigger)")
        print("  D - Dump captured samples")
        print("  B - Burst mode (capture then dump)")
        print("  S - Stream mode (continuous output)")
        print("  0 - Set full dump mode (131072 samples)")
        print("  1 - Set fast dump mode (4096 samples)")
        print("  T - Toggle trigger mode (edge vs level)")
        print("  X - Toggle TDC contribution (on/off)")
        print("  N - Toggle TDC sign (negate on/off)")
        print("  2 - Set TDC gain to 1x")
        print("  3 - Set TDC gain to 2x")
        print("  4 - Set TDC gain to 4x")
        print("  5 - Set TDC gain to 8x")
        print("  6 - Set TDC gain to 16x")
        print("  7 - Set TDC gain to 32x")
        print("  8 - Set TDC gain to 64x")
        print("  9 - Set TDC gain to 128x")
        print("  E - Toggle EQ filter (on/off)")
        print("  L - Toggle LP filter (on/off)")
        print("  M - Enable TDC monitor mode (sends TDC packets)")

        sys.exit(1)
    
    send_command(sys.argv[1])
