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

if __name__ == '__main__':
    plot_all_filters()
