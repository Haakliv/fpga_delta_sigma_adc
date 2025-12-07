#!/usr/bin/env python3
"""UART monitor for streaming ADC samples from the FPGA design.

The FPGA outputs Q15 signed values: [-32768, +32767] maps to [-1.0, +1.0)
We convert to millivolts using: mV = (q_value * 650) / 32768 + 650
This gives a range of 0-1300mV.
"""

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

# Voltage scaling for LVDS DAC swing
# FPGA I/O bank voltage is 1.3V. Must match VHDL to_millivolts:
# Q15 bipolar: '0' → -1 (0% duty, 0V avg), '1' → +1 (100% duty, ~1.3V avg)
# Q15: -32768 → 0mV, 0 → 650mV (50% duty), +32767 → 1300mV (100% duty)
V_CENTER_MV = 650       # 50% duty → 650mV (comparator center with 1.3V I/O)
V_HALF_SCALE_MV = 650   # Full 0-1300mV range


def find_uart_port() -> str:
    """Attempt to locate a likely UART device automatically."""
    import serial.tools.list_ports

    for port in serial.tools.list_ports.comports():
        description = port.description.upper()
        if "UART" in description or "JTAG" in description or "USB" in description:
            return port.device
    return "COM3"  # Fallback for Windows users


def decode_sample(line: str) -> Optional[int]:
    """Decode an ASCII hex string into a signed integer sample value (Q15 format)."""
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

    # Convert to signed 16-bit (Q15 format)
    if value >= (1 << (ADC_BITS - 1)):
        value -= (1 << ADC_BITS)

    return value


def q15_to_millivolts(q_value: int) -> float:
    """Convert a Q15 signed value to millivolts.
    
    LVDS DAC swing: 0V to 1.3V typical (RC filter time constant: 10kΩ × 1nF = 10µs)
    Formula: mV = (q_value * V_HALF_SCALE_MV) / 32768 + V_CENTER_MV
    
    Q15 mapping: -32768 → 0mV (0% duty), 0 → 650mV (50% duty), +32767 → 1300mV (100% duty)
    Comparator center: 0.65V (explains nonlinearity near 0.6V input)
    """
    mv = (q_value * V_HALF_SCALE_MV) / 32768 + V_CENTER_MV
    # Saturate to valid range
    v_min = V_CENTER_MV - V_HALF_SCALE_MV
    v_max = V_CENTER_MV + V_HALF_SCALE_MV
    if mv < v_min:
        mv = v_min
    elif mv > v_max:
        mv = v_max
    return mv


def main() -> None:
    port = find_uart_port()
    print(f"Connecting to {port} at 115200 baud...")
    print("=" * 80)
    print("ADC Monitor - Q15 Format with mV Conversion")
    print("=" * 80)
    print("FPGA outputs Q15 signed values: [-32768, +32767] -> [-1.0, +1.0)")
    print(f"Conversion: mV = (Q15 * {V_HALF_SCALE_MV}) / 32768 + {V_CENTER_MV}")
    print(f"Range: {V_CENTER_MV - V_HALF_SCALE_MV}mV to {V_CENTER_MV + V_HALF_SCALE_MV}mV")
    print("-" * 80)

    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print("Connected! Waiting for hex samples from FPGA...")
            print(f"{'Time':>10} | {'Sample#':>8} | {'Hex':>8} | {'Q15':>8} | {'mV':>10} | {'V':>10}")
            print("-" * 80)

            sample_count = 0
            while True:
                raw_line = ser.readline().decode("utf-8", errors="ignore")
                if not raw_line:
                    continue

                q_value = decode_sample(raw_line)
                if q_value is None:
                    # Ignore non-sample lines silently
                    continue

                mv = q15_to_millivolts(q_value)
                voltage_v = mv / 1000.0
                timestamp = time.strftime("%H:%M:%S")
                
                # Show hex as unsigned
                hex_val = q_value if q_value >= 0 else (1 << ADC_BITS) + q_value
                
                print(
                    f"{timestamp:>10} | {sample_count:>8} | 0x{hex_val:04X} | {q_value:>8} | "
                    f"{mv:>10.2f} | {voltage_v:>10.6f}"
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
