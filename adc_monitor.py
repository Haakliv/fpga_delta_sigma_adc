#!/usr/bin/env python3
"""UART monitor for streaming ADC samples from the FPGA design."""

from __future__ import annotations

import time
from typing import Optional

import serial

# ADC configuration (must match the FPGA design)
ADC_BITS = 16
ADC_MAX_VALUE = (1 << ADC_BITS) - 1
VREF = 1.2  # Reference voltage in volts


def find_uart_port() -> str:
    """Attempt to locate a likely UART device automatically."""
    import serial.tools.list_ports

    for port in serial.tools.list_ports.comports():
        description = port.description.upper()
        if "UART" in description or "JTAG" in description or "USB" in description:
            return port.device
    return "COM3"  # Fallback for Windows users


def decode_sample(line: str) -> Optional[int]:
    """Decode an ASCII hex string into an integer sample value."""
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

    return value


def sample_to_voltage(sample: int) -> float:
    """Convert a raw ADC sample into a voltage."""
    return (sample / ADC_MAX_VALUE) * VREF


def main() -> None:
    port = find_uart_port()
    print(f"Connecting to {port} at 115200 baud...")

    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print("Connected! Waiting for hex samples from FPGA...")
            print(f"ADC configuration: {ADC_BITS}-bit, Vref = {VREF:.3f} V")
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
                print(
                    f"[{timestamp}] Sample {sample_count:06d}: 0x{sample:04X} "
                    f"({sample:5d}) -> {voltage:.6f} V"
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
