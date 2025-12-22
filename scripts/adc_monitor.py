#!/usr/bin/env python3
"""UART monitor for streaming ADC samples from the FPGA design.

The FPGA outputs Q15 signed values: [-32768, +32767] maps to [-1.0, +1.0)
We convert to millivolts using: mV = (q_value * 650) / 32768 + 650
This gives a range of 0-1300mV.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt

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

# Sample rate (50MHz / 512 decimation = 97.656 kHz)
SAMPLE_RATE_HZ = 50_000_000 / 512

# Voltage reference and scaling
# V_REF is the actual hardware reference voltage (VADJ rail)
# Nominal: 1.30V, Measured (from gain error): ~1.34V
# The 0.97 gain slope indicates VADJ is ~3% higher than assumed
V_REF_NOMINAL = 1.30  # Design target voltage
V_REF_ACTUAL = 1.34   # Measured effective reference (from slope calibration)

# Hardware offset correction
# Physical cause: Input bias current through 1kΩ feedback resistor
# Measured offset: +21mV (indicates ~21µA bias current or comparator V_OS)
# This is a real voltage error at the comparator input
HARDWARE_OFFSET_MV = -21.0  # Correction to compensate for bias current drop

# Voltage scaling for LVDS DAC swing
# Q15 bipolar: '0' -> -1 (0% duty, 0V avg), '1' -> +1 (100% duty, ~V_REF avg)
# Q15: -32768 -> 0mV, 0 -> V_REF/2 (50% duty), +32767 -> V_REF (100% duty)
V_CENTER_MV = (V_REF_ACTUAL / 2) * 1000       # 50% duty -> V_REF/2 (comparator center)
V_HALF_SCALE_MV = (V_REF_ACTUAL / 2) * 1000   # Full 0 to V_REF range


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
    Two hardware corrections are applied:
    1. V_REF adjustment: Compensates for actual VADJ voltage (~1.34V vs nominal 1.30V)
    2. Offset correction: Compensates for input bias current drop over 1kΩ resistor (~21mV)
    """
    # Step 1: Calculate raw measured voltage from Q15 using actual V_REF
    mv_raw = (q_value * V_HALF_SCALE_MV) / 32768 + V_CENTER_MV
    
    # Step 2: Apply hardware offset correction (bias current compensation)
    mv_calibrated = mv_raw + HARDWARE_OFFSET_MV
    
    # Saturate to valid range (0 to V_REF_ACTUAL)
    if mv_calibrated < 0:
        mv_calibrated = 0
    elif mv_calibrated > V_REF_ACTUAL * 1000:
        mv_calibrated = V_REF_ACTUAL * 1000
    
    return mv_calibrated


def trigger_signal_generator() -> None:
    """Placeholder: Trigger an external signal generator via GPIB/USB/etc.
    
    TODO: Implement signal generator control (SCPI commands, pyvisa, etc.)
    Example use cases:
    - Trigger waveform output for ADC linearity testing
    - Synchronize signal gen with burst captures
    - Set frequency/amplitude/waveform type
    """
    # PLACEHOLDER: Signal generator control would go here
    # Examples:
    # - GPIB: pyvisa library to send SCPI commands
    # - USB: Direct instrument API (e.g., Rigol SDK, Keysight IO Libraries)
    # - LAN: Socket connection with SCPI over TCP
    print("TODO: Implement signal generator trigger control")
    pass


def capture_burst(ser: serial.Serial, fast_mode: bool = False) -> list[float]:
    """Capture a burst of ADC samples from the FPGA.
    
    Args:
        ser: Serial port object
        fast_mode: If True, enable short dump mode (4096 samples) after capture
    
    Returns:
        List of calibrated voltage measurements in millivolts
    """
    # Set dump mode explicitly using '1' (fast) or '0' (full)
    # Note: This only affects how many samples are DUMPED, not how many are CAPTURED.
    # Capture always fills the full 131072 sample buffer.
    if fast_mode:
        expected_samples = 4096
        print("Setting fast dump mode (4096 samples)...")
        ser.write(b'1')  # Explicit set to fast mode
    else:
        expected_samples = 131072
        print("Setting full dump mode (131072 samples)...")
        ser.write(b'0')  # Explicit set to full mode
    ser.flush()
    time.sleep(0.1)
    
    # Clear any pending data
    ser.reset_input_buffer()
    
    # Send capture command to start filling buffer
    # Capture always fills 131072 samples regardless of dump mode
    print("Sending capture command (C)...")
    ser.write(b'C')
    ser.flush()
    
    # Wait for capture to complete
    # 131072 samples @ 97.66 ksps = ~1.34 seconds (always, regardless of dump mode)
    wait_time = 1.5
    print(f"Waiting {wait_time}s for capture to complete (131072 samples @ 97.66 ksps)...")
    time.sleep(wait_time)
    
    print(f"Will dump {expected_samples} samples...")
    
    # Send dump command to start reading data
    print("Sending dump command (D)...")
    ser.write(b'D')
    ser.flush()
    
    # FPGA dumps data after receiving D command
    print("Waiting for data dump...")
    
    samples_mv = []
    timeout_start = time.time()
    timeout_duration = 180.0  # 180 second timeout (131072 samples @ ~700-1000 samples/sec over UART)
    
    while len(samples_mv) < expected_samples:
        if time.time() - timeout_start > timeout_duration:
            print(f"Timeout: Only received {len(samples_mv)}/{expected_samples} samples")
            break
            
        raw_line = ser.readline().decode("utf-8", errors="ignore")
        if not raw_line:
            continue
            
        q_value = decode_sample(raw_line)
        if q_value is None:
            continue
            
        mv = q15_to_millivolts(q_value)
        samples_mv.append(mv)
        
        # Progress indicator every 4096 samples
        if len(samples_mv) % 4096 == 0:
            elapsed = time.time() - timeout_start
            rate = len(samples_mv) / elapsed if elapsed > 0 else 0
            eta = (expected_samples - len(samples_mv)) / rate if rate > 0 else 0
            print(f"  Received {len(samples_mv)}/{expected_samples} samples... ({rate:.0f} S/s, ETA: {eta:.0f}s)")
    
    print(f"Burst capture complete: {len(samples_mv)} samples")
    return samples_mv


def plot_burst(samples_mv: list[float], signal_freq_hz: float = 1000.0) -> None:
    """Plot burst capture data showing time domain and FFT.
    
    Args:
        samples_mv: List of voltage samples in millivolts
        signal_freq_hz: Expected signal frequency for time axis scaling (default 1kHz)
    """
    samples = np.array(samples_mv)
    n_samples = len(samples)
    
    # Calculate time axis
    dt = 1.0 / SAMPLE_RATE_HZ
    t = np.arange(n_samples) * dt * 1000  # Time in milliseconds
    
    # Calculate how many samples for 6 cycles of the signal
    samples_per_cycle = SAMPLE_RATE_HZ / signal_freq_hz
    samples_for_6_cycles = int(6 * samples_per_cycle)
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Time domain plot (first 6 cycles)
    plot_samples = min(samples_for_6_cycles, n_samples)
    ax1.plot(t[:plot_samples], samples[:plot_samples], 'b-', linewidth=0.5)
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('Voltage (mV)')
    ax1.set_title(f'ADC Burst Capture - Time Domain ({signal_freq_hz:.0f}Hz signal, 6 cycles)')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, t[plot_samples-1] if plot_samples > 0 else 1)
    
    # FFT plot
    # Remove DC offset for cleaner FFT
    samples_ac = samples - np.mean(samples)
    
    # Compute FFT
    fft_result = np.fft.rfft(samples_ac)
    fft_freq = np.fft.rfftfreq(n_samples, dt)
    fft_magnitude_db = 20 * np.log10(np.abs(fft_result) / n_samples + 1e-10)
    
    # Plot up to Nyquist/4 for clarity
    max_freq_idx = len(fft_freq) // 4
    ax2.plot(fft_freq[:max_freq_idx] / 1000, fft_magnitude_db[:max_freq_idx], 'b-', linewidth=0.5)
    ax2.set_xlabel('Frequency (kHz)')
    ax2.set_ylabel('Magnitude (dB)')
    ax2.set_title('FFT Spectrum')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(-120, 0)
    
    plt.tight_layout()
    plt.show()
    return samples_mv


def analyze_burst(samples_mv: list[float]) -> None:
    """Analyze and display statistics for a burst of samples."""
    if not samples_mv:
        print("No samples to analyze!")
        return
    
    avg_mv = sum(samples_mv) / len(samples_mv)
    min_mv = min(samples_mv)
    max_mv = max(samples_mv)
    pp_mv = max_mv - min_mv
    
    if len(samples_mv) > 1:
        variance = sum((x - avg_mv) ** 2 for x in samples_mv) / (len(samples_mv) - 1)
        std_dev_mv = variance ** 0.5
    else:
        std_dev_mv = 0.0
    
    print("=" * 80)
    print("BURST CAPTURE ANALYSIS")
    print("=" * 80)
    print(f"Sample Count: {len(samples_mv)}")
    print(f"Average:      {avg_mv:>10.3f} mV ({avg_mv/1000:>10.6f} V)")
    print(f"Minimum:      {min_mv:>10.3f} mV ({min_mv/1000:>10.6f} V)")
    print(f"Maximum:      {max_mv:>10.3f} mV ({max_mv/1000:>10.6f} V)")
    print(f"Peak-Peak:    {pp_mv:>10.3f} mV")
    print(f"StdDev:       {std_dev_mv:>10.3f} mV (RMS noise)")
    print(f"SNR:          {20*((avg_mv/std_dev_mv)**0.5 if std_dev_mv > 0 else 0):>10.1f} dB (approx)")
    print("=" * 80)


def stream_mode(ser: serial.Serial) -> None:
    """Continuous streaming mode with real-time display."""
    print("Entering stream mode (press Ctrl+C to stop)...")
    
    # Send stream mode command to FPGA
    ser.write(b'S')
    ser.flush()
    time.sleep(0.1)
    
    print(f"{'Time':>10} | {'Sample#':>8} | {'Hex':>8} | {'Q15':>8} | {'mV':>10} | {'V':>10} | {'Avg30':>10} | {'Noise(mV)':>10}")
    print("-" * 94)

    sample_count = 0
    mv_buffer = []  # Rolling buffer for last 30 samples
    buffer_size = 30
    
    while True:
        raw_line = ser.readline().decode("utf-8", errors="ignore")
        if not raw_line:
            continue

        q_value = decode_sample(raw_line)
        if q_value is None:
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
        
        # Calculate StdDev (Noise)
        if len(mv_buffer) > 1:
            variance = sum((x - avg_mv) ** 2 for x in mv_buffer) / (len(mv_buffer) - 1)
            std_dev_mv = variance ** 0.5
        else:
            std_dev_mv = 0.0
        
        # Show hex as unsigned
        hex_val = q_value if q_value >= 0 else (1 << ADC_BITS) + q_value
        
        print(
            f"{timestamp:>10} | {sample_count:>8} | 0x{hex_val:04X} | {q_value:>8} | "
            f"{mv:>10.2f} | {voltage_v:>10.6f} | {avg_v:>10.6f} | {std_dev_mv:>10.3f}"
        )
        sample_count += 1


def main() -> None:
    import sys
    
    port = find_uart_port()
    print(f"Connecting to {port} at 115200 baud...")
    print("=" * 80)
    print("ADC Monitor - Q15 Format with Hardware-Calibrated mV Conversion")
    print("=" * 80)
    print("FPGA outputs Q15 signed values: [-32768, +32767] -> [-1.0, +1.0)")
    print(f"V_REF (Actual): {V_REF_ACTUAL}V (Nominal: {V_REF_NOMINAL}V)")
    print(f"Hardware Offset Correction: {HARDWARE_OFFSET_MV:+.1f}mV (Bias current compensation)")
    print(f"Conversion: mV = (Q15 * {V_HALF_SCALE_MV}) / 32768 + {V_CENTER_MV} + {HARDWARE_OFFSET_MV:+.1f}")
    print(f"Range: {(V_CENTER_MV - V_HALF_SCALE_MV + HARDWARE_OFFSET_MV):.1f}mV to {(V_CENTER_MV + V_HALF_SCALE_MV + HARDWARE_OFFSET_MV):.1f}mV")
    print("-" * 80)
    print("\nUsage:")
    print("  python adc_monitor.py           - Stream mode (continuous real-time)")
    print("  python adc_monitor.py burst     - Burst mode (131072 samples with analysis)")
    print("  python adc_monitor.py burst -f  - Fast burst mode (4096 samples, ~2.2 sec)")
    print("\nUART Commands (via uart_cmd.py):")
    print("  0 = Full dump mode (131072 samples)")
    print("  1 = Fast dump mode (4096 samples)")
    print("  2 = TDC gain 1x (default)")
    print("  3 = TDC gain 2x")
    print("  4 = TDC gain 4x")
    print("  5 = TDC gain 8x")
    print("  C = Capture trigger (software)")
    print("  T = Toggle trigger mode (edge/level)")
    print("  B = Burst mode, S = Stream mode")
    print("-" * 80)
    
    # Check for burst mode argument
    burst_mode = len(sys.argv) > 1 and sys.argv[1].lower() == 'burst'
    fast_mode = len(sys.argv) > 2 and sys.argv[2].lower() in ['-f', '--fast']

    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print("Connected!")
            
            if burst_mode:
                # Burst capture mode
                trigger_signal_generator()  # Placeholder for external trigger
                samples_mv = capture_burst(ser, fast_mode=fast_mode)
                analyze_burst(samples_mv)
                
                # Save to file for analysis
                with open('burst_data.csv', 'w') as f:
                    f.write('Sample,Voltage_mV\n')
                    for i, mv in enumerate(samples_mv):
                        f.write(f'{i},{mv:.3f}\n')
                
            else:
                # Continuous stream mode
                stream_mode(ser)

    except KeyboardInterrupt:
        print("\nStopped by user")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
    except Exception as exc:  # noqa: BLE001 - show unexpected issues
        print(f"Unexpected error: {exc}")


if __name__ == "__main__":
    main()
