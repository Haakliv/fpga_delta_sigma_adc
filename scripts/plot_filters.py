import numpy as np
import matplotlib.pyplot as plt
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Plot all filters and save to files
def plot_all_filters():
    # 1. CIC Sinc^3 Equalizer
    coef = np.array([
        316, 408, -462, -989, 152, 1401, 143, -2303,
        -1512, 2427, 2830, -3584, -8500, -1130, 15223, 23928
    ], dtype=float)

    # Build full symmetric 31-tap impulse response
    h = np.zeros(31)
    for i in range(15):
        h[i] = coef[i]
        h[30 - i] = coef[i]
    h[15] = coef[15]

    h = h / (2**15)  # Q1.15 -> real gain

    nfft = 4096
    H = np.fft.rfft(h, nfft)
    f = np.linspace(0, 0.5, len(H))  # 0..0.5 * Fs
    mag_db = 20 * np.log10(np.abs(H) + 1e-12)

    plt.figure(figsize=(10, 6))
    plt.plot(f, mag_db, color='C0', linewidth=1.5)
    plt.xlabel('Normalized frequency (f/Fs)', fontsize=12)
    plt.ylabel('Magnitude (dB)', fontsize=12)
    plt.title('CIC sinc3 Equalizer Magnitude Response', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.ylim(-1, 7)
    plt.xlim(0, 0.25)
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'equalizer_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: equalizer_response.png")

    # 2. CIC Decimator
    N = 3        # number of stages
    R = 64       # decimation factor
    M = 1        # differential delay

    # Normalized frequency 0..0.5 (relative to Fs_out)
    f = np.linspace(0, 0.5, 2000)
    eps = 1e-12  # avoid div-by-zero at f=0

    # CIC magnitude: |sin(pi f R M) / (R sin(pi f))|^N
    num = np.sin(np.pi * R * M * f)
    den = R * np.sin(np.pi * f + eps)
    H = (num / den) ** N

    mag_db = 20 * np.log10(np.abs(H) + 1e-12)

    plt.figure(figsize=(10, 6))
    plt.plot(f, mag_db, color='C0', linewidth=1.5)
    plt.xlabel('Normalized frequency (f / Fs_out)', fontsize=12)
    plt.ylabel('Magnitude (dB)', fontsize=12)
    plt.title('CIC sinc3 Decimator Magnitude Response (R=64)', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.ylim(-120, 5)
    plt.xlim(0.001, 0.094)  # Start just after DC to avoid the sharp drop
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'cic_decimator_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: cic_decimator_response.png")

    # 3. 63-tap Low-pass FIR
    coef_half = np.array([
        1, 1, -5, 11, -14, 10, 6, -31,
        56, -65, 42, 17, -100, 174, -195, 128,
        34, -248, 433, -485, 326, 53, -561, 1014,
        -1179, 850, 69, -1509, 3230, -4870, 6049, 26284
    ], dtype=float)

    # Build full 63-tap symmetric h[n]
    h = np.zeros(63)
    for i in range(31):
        h[i] = coef_half[i]
        h[62 - i] = coef_half[i]
    h[31] = coef_half[31]  # center tap

    h = h / (2**15)  # Q1.15 - real gain

    Fs = 1745.0  # Hz
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
    plt.xlim(0, 900)
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'lowpass_fir_response.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved: lowpass_fir_response.png")

if __name__ == '__main__':
    plot_all_filters()
