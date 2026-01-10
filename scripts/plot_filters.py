import numpy as np
import matplotlib.pyplot as plt
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Plot all filters and save to files
def plot_all_filters():
    # 1. CIC Sinc^3 Equalizer
    coef = np.array([
        27, 12, -49, -73, 84, 254, -43, -615,
        -270, 1122, 1180, -1574, -3385, 1238, 10654, 15644
    ], dtype=float)

    # Build full symmetric 31-tap impulse response
    h = np.zeros(31)
    for i in range(15):
        h[i] = coef[i]
        h[30 - i] = coef[i]
    h[15] = coef[15]

    h = h / (2**15)  # Q1.15 → real gain

    nfft = 4096
    H = np.fft.rfft(h, nfft)
    f = np.linspace(0, 0.5, len(H))  # 0..0.5 * Fs
    mag_db = 20 * np.log10(np.abs(H) + 1e-12)

    plt.figure(figsize=(10, 6))
    plt.plot(f, mag_db, color='C0', linewidth=1.5)
    plt.xlabel('Normalized frequency ($f/f_s$)', fontsize=12)
    plt.ylabel('Magnitude (dB)', fontsize=12)
    plt.title('CIC sinc3 Equalizer Magnitude Response', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.ylim(-60, 5)
    plt.xlim(0, 0.325)
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'equalizer_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: equalizer_response.png")

    # 2. CIC Decimator
    N = 3        # number of stages (sinc^3)
    R = 512      # decimation factor (50MHz/512 = 97.656kHz)
    M = 1        # differential delay

    # Normalized frequency 0..0.5 (relative to Fs_out)
    f = np.linspace(0, 0.5, 10000)
    eps = 1e-12  # avoid div-by-zero at f=0

    # CIC magnitude: |sin(pi f R M) / (R sin(pi f))|^N
    num = np.sin(np.pi * R * M * f)
    den = R * np.sin(np.pi * f + eps)
    H = (num / den) ** N

    mag_db = 20 * np.log10(np.abs(H) + 1e-12)

    plt.figure(figsize=(10, 6))
    plt.plot(f, mag_db, color='C0', linewidth=1.5)
    plt.xlabel('Normalized frequency ($f/f_s$)', fontsize=12)
    plt.ylabel('Magnitude (dB)', fontsize=12)
    plt.title('CIC sinc3 Decimator Magnitude Response (R=512)', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.ylim(-120, 5)
    plt.xlim(0.0001, 6/512)  # Show 6 complete lobes
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'cic_decimator_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: cic_decimator_response.png")

    # 3. 63-tap Low-pass FIR
    coef_half = np.array([
        -13, 0, 21, 0, -38, 0, 61, 0,
        -94, 0, 138, 0, -197, 0, 274, 0,
        -375, 0, 508, 0, -686, 0, 934, 0,
        -1309, 0, 1953, 0, -3396, 0, 10403, 16400
    ], dtype=float)

    # Build full 63-tap symmetric h[n]
    h = np.zeros(63)
    for i in range(31):
        h[i] = coef_half[i]
        h[62 - i] = coef_half[i]
    h[31] = coef_half[31]  # center tap

    h = h / (2**15)  # Q1.15 - real gain

    Fs = 97656.0  # Hz (50MHz / 512 = 97.656kHz)
    nfft = 8192
    H = np.fft.rfft(h, nfft)
    freqs = np.linspace(0, Fs/2, len(H))
    mag_db = 20 * np.log10(np.abs(H) + 1e-14)

    plt.figure(figsize=(10, 6))
    plt.plot(freqs, mag_db, color='C0', linewidth=1.5)
    plt.xlabel("Frequency (Hz)", fontsize=12)
    plt.ylabel("Magnitude (dB)", fontsize=12)
    plt.title("63-tap Low-pass FIR Magnitude Response", fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.ylim(-80, 5)
    plt.xlim(0, 48828)  # 0 to Fs/2 = 48.828 kHz
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'lowpass_fir_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: lowpass_fir_response.png")

    # 4. Combined Plot
    plt.figure(figsize=(12, 8))

    # Common frequency grid: 0 to Fs/2
    # Fs_out = 50MHz / 512 = 97656.25 Hz
    Fs_in = 50000000.0
    R = 512
    Fs_out = Fs_in / R
    f_hz = np.linspace(0, Fs_out / 2, 2048)
    # Avoid zero for log and division
    f_hz[0] = 1e-6

    # --- CIC Response ---
    # f_in_norm from 0 to 0.5/R
    f_out_norm = f_hz / Fs_out
    f_in_norm = f_out_norm / R
    M = 1
    N = 3

    # CIC magnitude
    num = np.sin(np.pi * R * M * f_in_norm)
    den = R * np.sin(np.pi * f_in_norm)
    H_cic = np.ones_like(f_hz)
    mask = f_in_norm > 1e-9
    H_cic[mask] = (num[mask] / den[mask]) ** N
    mag_cic_db = 20 * np.log10(np.abs(H_cic) + 1e-12)

    # --- Equalizer Response ---
    # Re-build coefficients
    eq_coef_half = np.array([
        27, 12, -49, -73, 84, 254, -43, -615,
        -270, 1122, 1180, -1574, -3385, 1238, 10654, 15644
    ], dtype=float)
    h_eq = np.zeros(31)
    for i in range(15):
        h_eq[i] = eq_coef_half[i]
        h_eq[30 - i] = eq_coef_half[i]
    h_eq[15] = eq_coef_half[15]
    h_eq = h_eq / (2**15)

    # Compute DTFT manually to match f_hz points
    w = 2 * np.pi * f_out_norm
    H_eq = np.zeros_like(f_hz, dtype=complex)
    for n in range(31):
        # Shift n so it correlates with time?
        # FIR phase is linear if symmetric. |H(w)| is what matters.
        # Just use standard sum h[n]e^{-jwn}
        H_eq += h_eq[n] * np.exp(-1j * w * n)
    mag_eq_db = 20 * np.log10(np.abs(H_eq) + 1e-12)

    # --- Low-pass FIR Response ---
    fir_coef_half = np.array([
        -13, 0, 21, 0, -38, 0, 61, 0,
        -94, 0, 138, 0, -197, 0, 274, 0,
        -375, 0, 508, 0, -686, 0, 934, 0,
        -1309, 0, 1953, 0, -3396, 0, 10403, 16400
    ], dtype=float)
    h_fir = np.zeros(63)
    for i in range(31):
        h_fir[i] = fir_coef_half[i]
        h_fir[62 - i] = fir_coef_half[i]
    h_fir[31] = fir_coef_half[31]
    h_fir = h_fir / (2**15)

    H_fir = np.zeros_like(f_hz, dtype=complex)
    for n in range(63):
        H_fir += h_fir[n] * np.exp(-1j * w * n)
    mag_fir_db = 20 * np.log10(np.abs(H_fir) + 1e-12)

    # --- Combinations ---
    mag_cic_eq_db = mag_cic_db + mag_eq_db
    mag_total_db = mag_cic_eq_db + mag_fir_db

    # Plot
    plt.plot(f_hz, mag_cic_db, label='CIC sinc$^3$ Decimator', linewidth=2)
    plt.plot(f_hz, mag_eq_db, label='Equalizer', linewidth=2)
    plt.plot(f_hz, mag_fir_db, label='Low-pass FIR', linewidth=2)
    plt.plot(f_hz, mag_total_db, label='Total Response', linestyle=':', linewidth=2, color='black')

    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude (dB)')
    plt.title('Combined Filter Responses')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.xlim(0, Fs_out / 2)
    plt.ylim(-60, 20)
    
    plt.savefig(os.path.join(script_dir, 'combined_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: combined_response.png")
    
    # Analysis output
    print(f"DC Gain (dB):")
    print(f"  CIC: {mag_cic_db[0]:.2f}")
    print(f"  Eq:  {mag_eq_db[0]:.2f}")
    print(f"  FIR: {mag_fir_db[0]:.2f}")
    
    idx_nyq = len(f_hz) - 1
    print(f"At Nyquist ({f_hz[idx_nyq]:.1f} Hz):")
    print(f"  CIC: {mag_cic_db[idx_nyq]:.2f} dB")
    print(f"  Eq:  {mag_eq_db[idx_nyq]:.2f} dB")
    print(f"  Sum (CIC+Eq): {mag_cic_eq_db[idx_nyq]:.2f} dB")

    # Detailed Analysis
    # Check "passband" flatness. Let's assume passband is 0 to 0.4 Fs.
    passband_idx = f_hz < (0.4 * Fs_out)
    pb_cic_eq = mag_cic_eq_db[passband_idx]
    
    print("\nAnalysis (0 to 0.4 Fs):")
    print(f"  Max Error (CIC+Eq): {np.max(np.abs(pb_cic_eq)):.4f} dB (deviation from 0dB)")
    print(f"  Range: {np.min(pb_cic_eq):.4f} dB to {np.max(pb_cic_eq):.4f} dB")
    
    # Check at specific points
    points = [0.1, 0.2, 0.3, 0.4, 0.5]
    print("\nSpecific Frequencies (normalized to Fs_out):")
    num_points = len(f_hz)
    for p in points:
        # Scale: p=0.5 -> idx = num_points-1. p=0 -> idx=0.
        # idx = p / 0.5 * (num_points-1)
        idx = int((p / 0.5) * (num_points - 1))
        if idx >= num_points: idx = num_points - 1
        print(f"  f={p:.2f}: CIC={mag_cic_db[idx]:.2f}dB, Eq={mag_eq_db[idx]:.2f}dB, Sum={mag_cic_eq_db[idx]:.2f}dB, Total={mag_total_db[idx]:.2f}dB")

if __name__ == '__main__':
    plot_all_filters()
