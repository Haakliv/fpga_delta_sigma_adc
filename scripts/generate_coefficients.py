"""
FIR Filter Coefficient Generator for TDC ADC

This script generates the FIR filter coefficients used in the TDC ADC design:

1. FIR Lowpass Filter:
   - 63-tap half-band lowpass filter
   - Design: Fs=97.656kHz, Fc=24.414kHz (Nyquist/2)
   - Remez equiripple design
   - Passband: 21.4kHz, Stopband: 27.4kHz
   - -3dB cutoff: ~23.8kHz
   - Half-band property: 50% of coefficients are zero

2. FIR Equalizer:
   - 31-tap sinc^3 droop compensator
   - Compensates for CIC decimator's sinc^3 frequency response rolloff
   - Design: Fs=97.656kHz, passband DC to 19.5kHz (80% of Nyquist/2)
   - -3dB cutoff: ~21kHz
   - Passband ripple: < 2dB

All coefficients are in Q1.15 fixed-point format (16-bit signed).
DC gain normalized to 1.0 (coefficient sum = 32768).

Usage:
    python generate_coefficients.py

Output:
    - Prints VHDL constant declarations for both filters
    - Generates filter_response.png with frequency response plots
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def generate_halfband_lowpass(num_taps, passband_edge, stopband_edge, fs):
    """
    Generate 63-tap half-band lowpass filter using Remez equiripple design.
    
    Design specifications:
      - Fs = 97.656 kHz (50MHz / 512)
      - Fc = 24.414 kHz (Nyquist/2 = Fs/4)
      - Passband edge: 21.4 kHz
      - Stopband edge: 27.4 kHz
      - Half-band property: passband + stopband edges symmetric about Nyquist/2
    
    The Remez (Parks-McClellan) algorithm produces an equiripple design that
    minimizes the maximum error in both passband and stopband.
    
    For a true half-band filter, passband_edge + stopband_edge = Nyquist exactly,
    which gives the property that every other coefficient (except center) is zero.
    
    Returns Q1.15 fixed-point coefficients (sum = 32768).
    """
    nyquist = fs / 2
    
    # For half-band property, bands must be exactly symmetric about Nyquist/2
    # i.e., fp_norm + fs_norm = 1.0 exactly
    transition_width = stopband_edge - passband_edge  # 6 kHz
    half_transition = transition_width / 2  # 3 kHz
    center_freq = nyquist / 2  # 24.414 kHz (Nyquist/2)
    
    # Force exact symmetry about center
    passband_symmetric = center_freq - half_transition
    stopband_symmetric = center_freq + half_transition
    
    # Normalize to [0, 1] where 1 = Nyquist
    fp_norm = passband_symmetric / nyquist
    fs_norm = stopband_symmetric / nyquist
    # Now fp_norm + fs_norm = 1.0 exactly (half-band condition)
    
    # Design using Remez (Parks-McClellan) algorithm
    # bands: [0, fp, fs, 1] where frequencies are normalized to Nyquist
    # desired: [1, 0] for passband gain=1, stopband gain=0
    bands = [0, fp_norm, fs_norm, 1.0]
    desired = [1, 0]
    
    # Use fs=2 so that 1.0 = Nyquist frequency
    coeffs_float = signal.remez(num_taps, bands, desired, fs=2)
    
    # Normalize to DC gain = 1.0
    dc_gain = np.sum(coeffs_float)
    coeffs_float = coeffs_float / dc_gain
    
    # Quantize to Q1.15 (scale by 32768)
    coeffs_fixed = np.round(coeffs_float * 32768).astype(np.int32)
    
    # Distribute rounding error across multiple large taps instead of just center
    # This gives better match to reference implementation
    current_sum = np.sum(coeffs_fixed)
    error = 32768 - current_sum
    
    # Find the largest taps and distribute error
    center = num_taps // 2
    if error != 0:
        # Adjust center tap by half the error, the two adjacent large taps by rest
        coeffs_fixed[center] += error
    
    return coeffs_fixed


def generate_equalizer(num_taps, fs, passband_edge):
    """
    Generate 31-tap sinc^3 droop compensator using firwin2 design.
    
    Design specifications:
      - Compensates CIC decimator's sinc^3 frequency response rolloff
      - Fs = 97.656 kHz
      - Passband: DC to 19.5 kHz (80% of Nyquist/2)
      - -3dB cutoff: ~21 kHz
      - Passband ripple: < 2dB
    
    The response approximates 1/sinc^3(pi*f/Fs) in the passband,
    with a smooth quadratic rolloff after the passband edge.
    
    Returns Q1.15 fixed-point coefficients (sum = 32768).
    """
    nyquist = fs / 2
    
    # Create frequency response specification
    n_pts = 100
    freqs = np.linspace(0, nyquist, n_pts)
    
    # CIC sinc^3 response: sinc(f/fs) where fs is the sample rate
    sinc_arg = freqs / fs
    sinc_resp = np.abs(np.sinc(sinc_arg)) ** 3
    
    # Inverse sinc^3 with floor to prevent excessive gain
    sinc_inv = 1.0 / np.maximum(sinc_resp, 0.1)
    
    # Build desired response with smooth rolloff
    gains = np.zeros_like(freqs)
    for i, f in enumerate(freqs):
        if f <= passband_edge:
            # Full inverse sinc^3 compensation in passband
            gains[i] = sinc_inv[i]
        elif f <= passband_edge * 1.3:
            # Quadratic rolloff in transition band
            t = (f - passband_edge) / (passband_edge * 0.3)
            gains[i] = sinc_inv[i] * (1 - t) ** 2
        else:
            # Stopband
            gains[i] = 0.0
    
    # Clip maximum gain to ~4 dB to prevent instability
    gains = np.clip(gains, 0, 10 ** (4.0 / 20))
    
    # Ensure DC gain is exactly 1.0
    gains[0] = 1.0
    
    # Normalize frequencies to [0, 1] for firwin2
    freqs_norm = freqs / nyquist
    
    # Design filter using firwin2
    coeffs_float = signal.firwin2(num_taps, freqs_norm, gains)
    
    # Normalize to DC gain = 1.0
    dc_gain = np.sum(coeffs_float)
    coeffs_float = coeffs_float / dc_gain
    
    # Quantize to Q1.15 (scale by 32768)
    coeffs_fixed = np.round(coeffs_float * 32768).astype(np.int32)
    
    # Fine-tune center tap to ensure exact DC gain of 32768
    center = num_taps // 2
    current_sum = np.sum(coeffs_fixed)
    coeffs_fixed[center] += (32768 - current_sum)
    
    return coeffs_fixed

def plot_response(fs, coeffs_dict):
    plt.figure(figsize=(10, 6))
    
    for name, coeffs in coeffs_dict.items():
        w, h = signal.freqz(coeffs, worN=8000)
        freq = w * fs / (2 * np.pi)
        # Normalize to DC gain = 0 dB
        h_db = 20 * np.log10(abs(h) / abs(h[0]))
        plt.plot(freq, h_db, label=name)
        
    plt.title('Filter Frequency Response')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain (dB)')
    plt.grid(True)
    plt.legend()
    # plt.show() # Commented out for non-interactive environments, save instead
    plt.savefig('filter_response.png')
    print("Saved plot to filter_response.png")

def print_vhdl_coeffs(name, coeffs, width=16):
    """Print coefficients in VHDL constant array format (symmetric, half stored)."""
    print(f"-- {name} Coefficients (Q1.{width-1})")
    print(f"-- Total taps: {len(coeffs)}")
    print(f"-- Sum: {np.sum(coeffs)} (DC gain = {np.sum(coeffs)/(2**(width-1)):.6f})")
    
    # For symmetric filters, only store first half
    half_len = (len(coeffs) + 1) // 2
    
    # Verify symmetry
    is_symmetric = True
    for i in range(len(coeffs) // 2):
        if coeffs[i] != coeffs[-(i+1)]:
            is_symmetric = False
            break
    
    if not is_symmetric:
        print("WARNING: Coefficients are not symmetric!")
    
    print(f"constant C_COEF : T_COEF_ARRAY := (")
    
    for i in range(half_len):
        val = coeffs[i]
        comma = "," if i < half_len - 1 else ""
        # Check symmetry for comment
        mirror_idx = len(coeffs) - 1 - i
        if i == mirror_idx:
            comment = f"-- h[{i}] (center tap)"
        else:
            comment = f"-- h[{i}] = h[{mirror_idx}]"
        
        # Format with proper alignment
        print(f"    {i:2d} => to_signed({val:6d}, {width}){comma:1s}  {comment}")
    print(");")
    print("")

if __name__ == "__main__":
    # Design parameters
    fs = 50e6 / 512  # 97.656 kHz (after CIC decimation by 512)
    nyquist = fs / 2  # 48.828 kHz
    
    print("=" * 80)
    print("FILTER COEFFICIENT GENERATION")
    print("=" * 80)
    print(f"Sample rate (Fs): {fs/1e3:.3f} kHz")
    print(f"Nyquist frequency: {nyquist/1e3:.3f} kHz")
    print()
    
    # ========================================================================
    # 1. Generate FIR Lowpass (63-tap half-band)
    # ========================================================================
    print("=" * 80)
    print("FIR LOWPASS FILTER (Half-Band)")
    print("=" * 80)
    
    lp_taps = 63
    lp_passband = 21.4e3  # Hz
    lp_stopband = 27.4e3  # Hz
    
    print(f"Design: {lp_taps}-tap half-band lowpass")
    print(f"Passband edge: {lp_passband/1e3:.1f} kHz")
    print(f"Stopband edge: {lp_stopband/1e3:.1f} kHz")
    print(f"Fc (nominal): {nyquist/2/1e3:.3f} kHz (Nyquist/2)")
    print()
    
    lp_coeffs_fixed = generate_halfband_lowpass(lp_taps, lp_passband, lp_stopband, fs)
    
    print(f"Generated {lp_taps} coefficients")
    print(f"Sum: {np.sum(lp_coeffs_fixed)} (DC gain = {np.sum(lp_coeffs_fixed)/32768:.6f})")
    print(f"Non-zero taps: {np.count_nonzero(lp_coeffs_fixed)} (half-band property)")
    print()
    
    print_vhdl_coeffs("FIR Lowpass", lp_coeffs_fixed)
    
    # ========================================================================
    # 2. Generate FIR Equalizer (31-tap sinc^3 compensator)
    # ========================================================================
    print("=" * 80)
    print("FIR EQUALIZER (Sinc^3 Compensator)")
    print("=" * 80)
    
    eq_taps = 31
    eq_passband = nyquist / 2 * 0.80  # 80% of Nyquist/2 = 19.53 kHz
    
    print(f"Design: {eq_taps}-tap sinc^3 droop compensator")
    print(f"Passband: DC to {eq_passband/1e3:.1f} kHz (80% of Nyquist/2)")
    print(f"Target: Flatten CIC decimator's sinc^3 rolloff")
    print()
    
    eq_coeffs_fixed = generate_equalizer(eq_taps, fs, eq_passband)
    
    print(f"Generated {eq_taps} coefficients")
    print(f"Sum: {np.sum(eq_coeffs_fixed)} (DC gain = {np.sum(eq_coeffs_fixed)/32768:.6f})")
    print()
    
    print_vhdl_coeffs("FIR Equalizer", eq_coeffs_fixed)
    
    # ========================================================================
    # 3. Plot frequency response
    # ========================================================================
    print("=" * 80)
    print("FREQUENCY RESPONSE ANALYSIS")
    print("=" * 80)
    
    coeffs_dict = {
        "Lowpass": lp_coeffs_fixed,
        "Equalizer": eq_coeffs_fixed,
        "Combined (EQ + LP)": signal.convolve(eq_coeffs_fixed, lp_coeffs_fixed)
    }
    plot_response(fs, coeffs_dict)
    
    print("\nFrequency response plot saved to filter_response.png")
