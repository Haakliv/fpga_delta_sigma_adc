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

# Voltage reference and scaling
# V_REF is the reference voltage for the ADC (typically FPGA I/O bank voltage)
V_REF = 1.25  # Hardware reference voltage (1.3V / 1.045 gain error = 1.244V ≈ 1.25V)

# Voltage scaling for LVDS DAC swing
# Q15 bipolar: '0' -> -1 (0% duty, 0V avg), '1' -> +1 (100% duty, ~V_REF avg)
# Q15: -32768 -> 0mV, 0 -> V_REF/2 (50% duty), +32767 -> V_REF (100% duty)
V_CENTER_MV = (V_REF / 2) * 1000       # 50% duty -> V_REF/2 (comparator center)
V_HALF_SCALE_MV = (V_REF / 2) * 1000   # Full 0 to V_REF range

# Calibration coefficients from measured transfer curve
# Previous fit: Gain 1.005, Offset -23.0mV
# Data showed: 0.2V->0.196 (-4mV), 0.6V->0.595 (-5mV), 1.2V->1.204 (+4mV)
# Range expansion (1.0V input -> 1.008V output) indicates Gain needs to increase.
#
# New fit for 0.2-1.2V range:
# Gain 1.0135, Offset -15.5mV
# 0.2V -> 201.2mV (+1.2mV)
# 0.6V -> 597.3mV (-2.7mV)
# 1.2V -> 1201.2mV (+1.2mV)
GAIN_ERROR = 1.0135     # Increased gain correction to compress output range
OFFSET_ERROR_MV = -15.5 # Adjusted offset to center the error


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
    """Convert a Q15 signed value to millivolts (calibrated input voltage).
    
    The ADC measures the feedback voltage (DAC output averaged by RC filter).
    Calibration model: Measured = GAIN * (Input - OFFSET)
    Inverse: Input = Measured / GAIN + OFFSET
    
    Steps:
    1. Convert Q15 to uncalibrated voltage (assumes V_REF)
    2. Apply inverse gain correction (divide by 1.045)
    3. Apply offset correction (add +20mV)
    """
    # Step 1: Calculate raw measured voltage from Q15
    mv_measured = (q_value * V_HALF_SCALE_MV) / 32768 + V_CENTER_MV
    
    # Step 2 & 3: Apply calibration
    # Input = (Measured / GAIN) + OFFSET
    mv_calibrated = (mv_measured / GAIN_ERROR) + OFFSET_ERROR_MV
    
    # Saturate to valid range (0 to V_REF after calibration)
    if mv_calibrated < 0:
        mv_calibrated = 0
    elif mv_calibrated > V_REF * 1000:
        mv_calibrated = V_REF * 1000
    
    return mv_calibrated


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
            print(f"{'Time':>10} | {'Sample#':>8} | {'Hex':>8} | {'Q15':>8} | {'mV':>10} | {'V':>10} | {'Avg30':>10}")
            print("-" * 80)

            sample_count = 0
            mv_buffer = []  # Rolling buffer for last 30 samples
            buffer_size = 30
            
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
                
                # Update rolling average buffer
                mv_buffer.append(mv)
                if len(mv_buffer) > buffer_size:
                    mv_buffer.pop(0)
                avg_mv = sum(mv_buffer) / len(mv_buffer)
                avg_v = avg_mv / 1000.0
                
                # Show hex as unsigned
                hex_val = q_value if q_value >= 0 else (1 << ADC_BITS) + q_value
                
                print(
                    f"{timestamp:>10} | {sample_count:>8} | 0x{hex_val:04X} | {q_value:>8} | "
                    f"{mv:>10.2f} | {voltage_v:>10.6f} | {avg_v:>10.6f}"
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
