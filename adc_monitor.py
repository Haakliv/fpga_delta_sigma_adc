#!/usr/bin/env python3
"""UART monitor for streaming ADC samples from the FPGA design."""

from __future__ import annotations

import time
from typing import Optional

try:
    import serial
    from serial.serialutil import SerialException
except ImportError:
    print("Error: pyserial not installed correctly.")
    print("Please run: pip uninstall serial pyserial")
    print("Then: pip install pyserial")
    exit(1)

# ADC configuration (must match the FPGA design)
ADC_BITS = 16
ADC_MAX_VALUE = (1 << ADC_BITS) - 1

# Calibration constants (to be determined empirically)
# Based on user measurements with OLD firmware (shift=37):
#   110mV → -968 counts
#   10mV step → 4 count change
# With NEW firmware (shift=33), counts are multiplied by 2^4 = 16
# So we expect: 110mV → -15,488 counts theoretically
# But need actual calibration points from user

# For now, use a placeholder calibration
# USER: Please provide two calibration points:
#   1. What voltage gives what count?
#   2. Another voltage and its count?
CALIBRATION_ENABLED = False


def find_uart_port() -> str:
    """Attempt to locate a likely UART device automatically."""
    import serial.tools.list_ports

    for port in serial.tools.list_ports.comports():
        description = port.description.upper()
        if "UART" in description or "JTAG" in description or "USB" in description:
            return port.device
    return "COM3"  # Fallback for Windows users


def decode_sample(line: str) -> Optional[int]:
    """Decode an ASCII hex string into a signed integer sample value."""
    token = line.strip()
    if not token:
        return None

    if token.startswith(("0x", "0X")):
        token = token[2:]

    try:
        value = int(token, 16)
    except ValueError:
        return None

    if value < 0 or value > ADC_MAX_VALUE:
        return None

    # Convert to signed 16-bit
    if value >= (1 << (ADC_BITS - 1)):
        value -= (1 << ADC_BITS)

    return value


def sample_to_voltage(sample: int) -> float:
    """Convert a signed ADC sample into a voltage.
    
    Simple linear calibration: V = m × count + b
    USER: Adjust these constants based on your measurements!
    """
    # Two-point calibration (USER TO UPDATE):
    # Point 1: (count1, voltage1)
    # Point 2: (count2, voltage2)
    
    # Example using your data (NEEDS UPDATING for new firmware):
    #   110mV @ ??? counts (new firmware)
    #   600mV @ ??? counts (new firmware)
    
    # Temporary placeholder - just show raw counts scaled
    # 1 count ≈ some millivolts
    COUNTS_PER_VOLT = 10000  # Placeholder - adjust based on measurements
    OFFSET_VOLTS = 0.0      # Placeholder
    
    return (sample / COUNTS_PER_VOLT) + OFFSET_VOLTS


def send_mmio_write(ser: serial.Serial, addr: int, value: int) -> None:
    """Send MMIO write command: 'W <addr> <value>\n'"""
    cmd = f"W {addr} {value}\n"
    ser.write(cmd.encode('ascii'))
    print(f"Sent MMIO write: addr={addr} value=0x{value:08X}")


def main() -> None:
    port = find_uart_port()
    print(f"Connecting to {port} at 115200 baud...")
    print("=" * 70)
    print("MMIO Write Commands:")
    print("  Send: W <addr> <value>")
    print("  Example: W 28 1  (Set comparator polarity inversion)")
    print("  Addr 27: Coarse bias (0-15)")
    print("  Addr 28: Comparator invert (0=normal, 1=inverted)")
    print("=" * 70)

    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print("Connected! Waiting for hex samples from FPGA...")
            print(f"ADC configuration: {ADC_BITS}-bit signed output")
            print("NOTE: Voltage calibration needs updating - showing raw counts/10000")
            print("-" * 60)

            sample_count = 0
            while True:
                raw_line = ser.readline().decode("utf-8", errors="ignore")
                if not raw_line:
                    continue

                sample = decode_sample(raw_line)
                if sample is None:
                    # Ignore non-sample lines silently
                    continue

                voltage = sample_to_voltage(sample)
                timestamp = time.strftime("%H:%M:%S")
                # Show hex as unsigned, decimal as signed
                hex_val = sample if sample >= 0 else (1 << ADC_BITS) + sample
                print(
                    f"[{timestamp}] Sample {sample_count:06d}: 0x{hex_val:04X} "
                    f"({sample:6d}) -> {voltage:.6f} V"
                )
                sample_count += 1

    except KeyboardInterrupt:
        print("\nStopped by user")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
    except Exception as exc:  # noqa: BLE001 - show unexpected issues
        print(f"Unexpected error: {exc}")


if __name__ == "__main__":
    main()
