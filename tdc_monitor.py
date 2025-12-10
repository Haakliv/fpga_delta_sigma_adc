#!/usr/bin/env python3
"""TDC Monitor - Captures raw TDC codes for sanity checking.

This script logs TDC measurements to verify:
1. TDC monotonicity: Does tdc_code increase/decrease with voltage?
2. TDC saturation: Does it hit min/max at voltage extremes?
3. Center calibration: Is tdc_center near the midpoint of observed codes?
4. DAC correlation: Does tdc_code correlate with dac_bit state?

UART Packet Format (from FPGA):
- Byte 0: 0xAA (sync header)
- Byte 1: 0x55 (sync header)
- Bytes 2-3: tdc_monitor_code (16-bit signed, big-endian)
- Bytes 4-5: tdc_monitor_center (16-bit signed, big-endian)
- Bytes 6-7: tdc_monitor_diff (16-bit signed, big-endian)
- Byte 8: tdc_monitor_dac (1 bit in LSB)
- Byte 9: combined_data_out[15:8] (upper 8 bits of ADC output for voltage estimate)
- Byte 10: combined_data_out[7:0] (lower 8 bits)
- Byte 11: Checksum (XOR of bytes 2-10)

Total: 12 bytes per packet at ~2MHz TDC sample rate (but can decimate by 100+)
"""

from __future__ import annotations

import time
import struct
from typing import Optional
from dataclasses import dataclass

try:
    import serial
    from serial.serialutil import SerialException
except ImportError:
    print("Error: pyserial not installed correctly.")
    print("Please run: pip uninstall serial pyserial")
    print("Then: pip install pyserial")
    exit(1)

# Voltage scaling for FPGA I/O bank (1.3V)
# Q15 bipolar: '0' -> -1 (0% duty, 0V avg), '1' -> +1 (100% duty, ~1.3V avg)
# Map Q15 to 0-1300 mV range
V_CENTER_MV = 650
V_HALF_SCALE_MV = 650

# Comparator offset calibration
# The LVDS comparator has inherent offset - feedback must be ~50mV higher than
# input for the comparator to see equality. Subtract this offset to show true input voltage.
COMPARATOR_OFFSET_MV = 50


@dataclass
class TDCMonitorSample:
    """Single TDC monitor sample."""
    tdc_code: int      # Raw TDC code (signed 16-bit)
    tdc_center: int    # Calibrated center (signed 16-bit)
    tdc_diff: int      # tdc_code - tdc_center (signed 16-bit)
    dac_bit: bool      # DAC bitstream state
    adc_out: int       # ADC output value (Q15 signed)
    voltage_mv: float  # Estimated voltage in mV


def q15_to_millivolts(q_value: int) -> float:
    """Convert Q15 signed value to millivolts (input voltage).
    
    The ADC measures feedback voltage; we subtract comparator offset to get input.
    Formula: mV = (value * 650) / 32768 + 650 - 50
    """
    mv = (q_value * V_HALF_SCALE_MV) / 32768 + V_CENTER_MV - COMPARATOR_OFFSET_MV
    return max(0, min(1250, mv))


def find_uart_port() -> str:
    """Attempt to locate a likely UART device automatically."""
    import serial.tools.list_ports

    for port in serial.tools.list_ports.comports():
        description = port.description.upper()
        if "UART" in description or "JTAG" in description or "USB" in description:
            return port.device
    return "COM3"  # Fallback


def parse_tdc_packet(data: bytes) -> Optional[TDCMonitorSample]:
    """Parse 12-byte TDC monitor packet."""
    if len(data) != 12:
        return None
    
    # Check sync header
    if data[0] != 0xAA or data[1] != 0x55:
        return None
    
    # Extract fields (big-endian signed 16-bit)
    tdc_code = struct.unpack('>h', data[2:4])[0]
    tdc_center = struct.unpack('>h', data[4:6])[0]
    tdc_diff = struct.unpack('>h', data[6:8])[0]
    dac_bit = bool(data[8] & 0x01)
    adc_out = struct.unpack('>h', data[9:11])[0]
    checksum_rx = data[11]
    
    # Verify checksum (XOR of bytes 2-10)
    checksum_calc = 0
    for byte in data[2:11]:
        checksum_calc ^= byte
    
    if checksum_rx != checksum_calc:
        print(f"Checksum error: expected {checksum_calc:02X}, got {checksum_rx:02X}")
        return None
    
    voltage_mv = q15_to_millivolts(adc_out)
    
    return TDCMonitorSample(
        tdc_code=tdc_code,
        tdc_center=tdc_center,
        tdc_diff=tdc_diff,
        dac_bit=dac_bit,
        adc_out=adc_out,
        voltage_mv=voltage_mv
    )


def analyze_tdc_sweep(samples: list[TDCMonitorSample]) -> None:
    """Analyze TDC behavior across voltage sweep."""
    if not samples:
        print("No samples to analyze")
        return
    
    # Group by voltage bins (100mV bins)
    voltage_bins = {}
    for sample in samples:
        v_bin = int(sample.voltage_mv / 100) * 100
        if v_bin not in voltage_bins:
            voltage_bins[v_bin] = []
        voltage_bins[v_bin].append(sample)
    
    print("\n" + "=" * 80)
    print("TDC Analysis by Voltage")
    print("=" * 80)
    print(f"{'Voltage (mV)':<15} {'TDC Min':<10} {'TDC Max':<10} {'TDC Avg':<10} {'Center':<10} {'Samples':<10}")
    print("-" * 80)
    
    for v_bin in sorted(voltage_bins.keys()):
        bin_samples = voltage_bins[v_bin]
        tdc_codes = [s.tdc_code for s in bin_samples]
        tdc_min = min(tdc_codes)
        tdc_max = max(tdc_codes)
        tdc_avg = sum(tdc_codes) / len(tdc_codes)
        tdc_center = bin_samples[0].tdc_center  # Assume constant during sweep
        
        print(f"{v_bin:<15} {tdc_min:<10} {tdc_max:<10} {tdc_avg:<10.1f} {tdc_center:<10} {len(bin_samples):<10}")
    
    # Check monotonicity
    sorted_voltages = sorted(voltage_bins.keys())
    if len(sorted_voltages) >= 2:
        avg_codes = [sum(s.tdc_code for s in voltage_bins[v]) / len(voltage_bins[v]) for v in sorted_voltages]
        monotonic = all(avg_codes[i] < avg_codes[i+1] for i in range(len(avg_codes)-1))
        print(f"\nMonotonicity check: {'PASS' if monotonic else 'FAIL'}")
    
    # Check center calibration
    all_codes = [s.tdc_code for s in samples]
    if not all_codes:
        return
        
    code_min = min(all_codes)
    code_max = max(all_codes)
    code_mid = (code_min + code_max) / 2
    center_avg = sum(s.tdc_center for s in samples) / len(samples)
    center_error = abs(center_avg - code_mid)
    
    print(f"\nTDC Code Range: [{code_min}, {code_max}]")
    print(f"Observed Midpoint: {code_mid:.1f}")
    print(f"Calibrated Center: {center_avg:.1f}")
    
    range_span = code_max - code_min
    if range_span > 0:
        print(f"Center Error: {center_error:.1f} codes ({100*center_error/range_span:.1f}% of range)")
    else:
        print(f"Center Error: {center_error:.1f} codes (Range is 0)")


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--limit", type=int, default=0, help="Exit after capturing N packets (0=run forever)")
    args = parser.parse_args()

    port = find_uart_port()
    print(f"Connecting to {port} at 115200 baud...")
    print("=" * 80)
    print("TDC Monitor - Raw Code Capture for Sanity Check")
    print("=" * 80)
    print("Waiting for TDC monitor packets (0xAA 0x55 header)...")
    if args.limit > 0:
        print(f"Capturing {args.limit} packets...")
    else:
        print("Press Ctrl+C to stop and analyze")
    print("-" * 80)
    
    samples = []
    packet_count = 0
    error_count = 0
    
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            buffer = bytearray()
            
            while True:
                # Check limit
                if args.limit > 0 and packet_count >= args.limit:
                    break

                # Read available data
                if ser.in_waiting > 0:
                    buffer.extend(ser.read(ser.in_waiting))
                
                # Search for packet sync
                while len(buffer) >= 12:
                    # Look for 0xAA 0x55 header
                    sync_idx = -1
                    for i in range(len(buffer) - 1):
                        if buffer[i] == 0xAA and buffer[i+1] == 0x55:
                            sync_idx = i
                            break
                    
                    if sync_idx == -1:
                        # No sync found, keep last byte in case it's 0xAA
                        buffer = buffer[-1:]
                        break
                    
                    # Remove bytes before sync
                    if sync_idx > 0:
                        buffer = buffer[sync_idx:]
                    
                    # Check if we have full packet
                    if len(buffer) < 12:
                        break
                    
                    # Parse packet
                    packet_data = bytes(buffer[:12])
                    buffer = buffer[12:]  # Remove parsed packet
                    
                    sample = parse_tdc_packet(packet_data)
                    if sample:
                        samples.append(sample)
                        packet_count += 1
                        
                        # Print every 100th sample
                        if packet_count % 100 == 0:
                            print(f"[{packet_count:5d}] V={sample.voltage_mv:6.1f}mV  "
                                  f"TDC={sample.tdc_code:6d}  Center={sample.tdc_center:6d}  "
                                  f"Diff={sample.tdc_diff:6d}  DAC={sample.dac_bit}")
                    else:
                        error_count += 1
                
                # Rate limit
                time.sleep(0.01)
    
    except KeyboardInterrupt:
        print(f"\n\nCaptured {packet_count} packets ({error_count} errors)")
        if samples:
            analyze_tdc_sweep(samples)
    
    except SerialException as e:
        print(f"Serial port error: {e}")

    # End of capture (limit reached or interrupted)
    if args.limit > 0 and packet_count >= args.limit:
        print(f"\n\nLimit reached. Captured {packet_count} packets ({error_count} errors)")
        if samples:
            analyze_tdc_sweep(samples)


if __name__ == "__main__":
    main()
