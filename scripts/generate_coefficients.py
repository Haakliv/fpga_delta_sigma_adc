import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

def to_fixed_point(coeffs, width=16, scale_factor=None):
    """
    Convert floating point coefficients to fixed point Q1.15 (or similar).
    If scale_factor is None, it scales so that the sum is 2**(width-1) (DC gain = 1.0).
    """
    if scale_factor is None:
        # Normalize so sum is 2**(width-1) (representing 1.0 in Q1.15)
        # Actually, usually we want the max value to fit, or the DC gain to be 1.
        # The VHDL comments say "Sum of all coefficients = 32768 (DC gain = 1.0 in Q1.15)"
        target_sum = 2**(width-1)
        current_sum = np.sum(coeffs)
        scale_factor = target_sum / current_sum
    
    quantized = np.round(coeffs * scale_factor).astype(int)
    
    # Adjust center tap to ensure sum is exactly target_sum
    current_sum = np.sum(quantized)
    diff = target_sum - current_sum
    if diff != 0:
        # Find center tap index
        center_idx = len(quantized) // 2
        quantized[center_idx] += diff
        
    return quantized, scale_factor

def generate_lowpass_coefficients(num_taps, cutoff_freq, fs, beta=6.98):
    """
    Generate FIR lowpass filter coefficients using a Kaiser window.
    
    Args:
        num_taps (int): Number of taps (coefficients).
        cutoff_freq (float): Cutoff frequency in Hz.
        fs (float): Sampling frequency in Hz.
        beta (float): Kaiser window beta parameter.
        
    Returns:
        np.array: Floating point coefficients.
    """
    nyquist = 0.5 * fs
    normalized_cutoff = cutoff_freq / nyquist
    
    # firwin with kaiser window
    taps = signal.firwin(num_taps, normalized_cutoff, window=('kaiser', beta))
    
    return taps

def sinc3_inverse(f, fs):
    """
    Inverse sinc^3 response function.
    H(f) = 1 / (sinc(pi * f / fs))^3
    Note: numpy.sinc(x) is sin(pi*x)/(pi*x).
    """
    # Avoid division by zero at DC
    with np.errstate(divide='ignore', invalid='ignore'):
        # The CIC response is usually modeled as sinc(pi*f/fs) where fs is the output rate
        # if we are compensating the droop in the first Nyquist zone.
        # np.sinc(x) computes sin(pi*x)/(pi*x).
        # We want sinc(f/fs) in numpy terms? 
        # The VHDL comment says "approximate 1/sinc^3(pi*f/Fs)".
        # If x = f/Fs, then np.sinc(x) = sin(pi*f/Fs)/(pi*f/Fs).
        val = np.abs(np.sinc(f/fs))**3
        resp = 1.0 / val
    resp[f == 0] = 1.0
    return resp

def generate_equalizer_coefficients(num_taps, fs, passband_edge_freq):
    """
    Generate FIR equalizer coefficients to compensate for CIC sinc^3 droop.
    Uses firls (least squares) to fit the inverse response.
    
    Args:
        num_taps (int): Number of taps.
        fs (float): Sampling frequency.
        passband_edge_freq (float): Edge of the passband to optimize for.
        
    Returns:
        np.array: Floating point coefficients.
    """
    nyquist = 0.5 * fs
    
    # Define bands for firls
    # We want to fit from 0 to passband_edge_freq
    # We can add a transition band or stop band if needed, but usually 
    # for droop compensation we care about the passband.
    # However, firls needs bands. Let's try to fit up to the edge.
    
    # Number of points for the grid
    num_points = 100
    freqs = np.linspace(0, passband_edge_freq, num_points)
    
    # Calculate desired response
    desired = sinc3_inverse(freqs, fs)
    
    # Normalize frequencies to Nyquist (0 to 1)
    bands = [0, passband_edge_freq]
    
    # We need to provide the desired gain at the band edges for firls?
    # No, firls takes bands and desired gains.
    # If we want a custom response curve, we might need 'remez' or construct the bands carefully.
    # Scipy firls: "bands: A monotonic sequence containing the band edges."
    # "desired: A sequence the same size as bands containing the desired gain at the start and end of each band."
    # This implies linear interpolation between points in the band?
    # If the response is curved, we need many small bands.
    
    bands = np.linspace(0, passband_edge_freq, num_points)
    desired = sinc3_inverse(bands, fs)
    
    # firls expects pairs of band edges.
    # To approximate a curve, we can just pass the points if we treat them as connected?
    # Actually, scipy.signal.firls(numtaps, bands, desired)
    # "bands: ... All elements must be non-negative and less than or equal to the Nyquist frequency."
    # "desired: ... If the last element of bands is less than the Nyquist frequency, a 0 gain is assumed for the rest."
    # Wait, firls documentation says:
    # "bands: A monotonic sequence ... The length of bands must be even? No."
    # Let's check docs or recall.
    # "bands: A monotonic sequence containing the band edges. All elements must be non-negative and less than or equal to the Nyquist frequency."
    # "desired: A sequence the same size as bands containing the desired gain at the start and end of each band."
    # This suggests it does linear interpolation.
    # So if I want a curve, I should provide many points.
    
    # Construct bands for firls (pairs of start, end)
    # Actually, let's use many small bands to approximate the curve.
    firls_bands = []
    firls_desired = []
    
    # We can just use the points directly if we duplicate them?
    # e.g. band1=[f0, f1], gain=[g0, g1]. band2=[f1, f2], gain=[g1, g2].
    # But firls takes a flat list of bands.
    # [f0, f1, f1, f2, f2, f3 ...]
    
    for i in range(len(bands)-1):
        firls_bands.extend([bands[i], bands[i+1]])
        firls_desired.extend([desired[i], desired[i+1]])
        
    # Normalize bands by Nyquist
    firls_bands = np.array(firls_bands) / nyquist
    
    taps = signal.firls(num_taps, firls_bands, firls_desired)
    
    return taps

def generate_equalizer_coefficients_with_transition(num_taps, fs, passband_edge, stopband_edge):
    """
    Generate FIR equalizer coefficients with explicit stopband.
    """
    nyquist = 0.5 * fs
    
    # Passband
    num_points = 50
    bands = np.linspace(0, passband_edge, num_points)
    desired = sinc3_inverse(bands, fs)
    
    firls_bands = []
    firls_desired = []
    
    for i in range(len(bands)-1):
        firls_bands.extend([bands[i], bands[i+1]])
        firls_desired.extend([desired[i], desired[i+1]])
        
    # Stopband
    firls_bands.extend([stopband_edge, nyquist])
    firls_desired.extend([0, 0])
    
    firls_bands = np.array(firls_bands) / nyquist
    taps = signal.firls(num_taps, firls_bands, firls_desired)
    return taps

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
    print(f"-- {name} Coefficients (Q1.{width-1})")
    print(f"-- Total taps: {len(coeffs)}")
    print(f"-- Sum: {np.sum(coeffs)}")
    print(f"constant C_COEF : T_COEF_ARRAY := (")
    
    # Assuming symmetric, print half
    half_len = (len(coeffs) + 1) // 2
    for i in range(half_len):
        val = coeffs[i]
        comma = "," if i < half_len - 1 else ""
        # Check symmetry for comment
        mirror_idx = len(coeffs) - 1 - i
        if i == mirror_idx:
            comment = f"-- h[{i}] (center tap)"
        else:
            comment = f"-- h[{i}] = h[{mirror_idx}]"
        print(f"  to_signed({val}, {width}){comma} \t {comment}")
    print(");")
    print("")

if __name__ == "__main__":
    # Original Coefficients for comparison
    orig_lp_coeffs = np.array([
        1, 1, -5, 11, -14, 10, 6, -31, 56, -65, 42, 17, -100, 174, -195, 128, 34, -248, 433, -485, 326, 53, -561, 1014, -1179, 850, 69, -1509, 3230, -4870, 6049, 26284, 
        6049, -4870, 3230, -1509, 69, 850, -1179, 1014, -561, 53, 326, -485, 433, -248, 34, 128, -195, 174, -100, 17, 42, -65, 56, -31, 6, 10, -14, 11, -5, 1, 1
    ])
    
    orig_eq_coeffs = np.array([
        316, 408, -462, -989, 152, 1401, 143, -2303, -1512, 2427, 2830, -3584, -8500, -1130, 15223, 23928, 
        15223, -1130, -8500, -3584, 2830, 2427, -1512, -2303, 143, 1401, 152, -989, -462, 408, 316
    ])

    # 1. FIR Lowpass
    # Specs from VHDL: 63 taps, Kaiser beta=6.98, Fc=700Hz, Fs=1745Hz
    # Optimization found beta=6.9734 to match exact coefficients.
    print("Generating FIR Lowpass Coefficients...")
    # Using beta=6.9734 gives an exact match to the VHDL coefficients
    lp_taps_float = generate_lowpass_coefficients(num_taps=63, cutoff_freq=700, fs=1745, beta=6.9734)
    lp_taps_fixed, _ = to_fixed_point(lp_taps_float, width=16)
    
    # Verify against original
    diff_lp = np.sum(np.abs(lp_taps_fixed - orig_lp_coeffs))
    print(f"Lowpass Difference from Original: {diff_lp} (Should be 0)")
    
    print_vhdl_coeffs("FIR Lowpass (Generated)", lp_taps_fixed)
    
    # 2. FIR Equalizer
    print("Generating FIR Equalizer Coefficients...")
    # The original equalizer coefficients are hard to reproduce exactly without knowing the 
    # exact design method (likely firls or remez with specific bands/weights).
    # However, we can generate a valid sinc^3 compensator.
    # We use a passband up to 0.25*Fs and a stopband starting at 0.5*Fs (Nyquist).
    
    # Tunable parameters:
    eq_num_taps = 31
    eq_pass_edge = 0.22 # Tuned to approximate original response shape
    eq_stop_edge = 0.5
    
    eq_taps_float = generate_equalizer_coefficients_with_transition(
        num_taps=eq_num_taps, 
        fs=1.0, 
        passband_edge=eq_pass_edge, 
        stopband_edge=eq_stop_edge
    )
    eq_taps_fixed, _ = to_fixed_point(eq_taps_float, width=16)
    
    # Verify against original (Expect some difference)
    diff_eq = np.sum(np.abs(eq_taps_fixed - orig_eq_coeffs))
    print(f"Equalizer Difference from Original: {diff_eq} (Exact match requires original design parameters)")
    
    print_vhdl_coeffs(f"FIR Equalizer (Generated {eq_pass_edge}Fs-{eq_stop_edge}Fs)", eq_taps_fixed)
    
    # Plot comparison
    coeffs_dict = {
        "Original Lowpass": orig_lp_coeffs,
        "Generated Lowpass": lp_taps_fixed,
        "Original Equalizer": orig_eq_coeffs,
        "Generated Equalizer": eq_taps_fixed
    }
    plot_response(1745, coeffs_dict)
    
