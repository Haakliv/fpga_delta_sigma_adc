#!/usr/bin/env python3
"""
VUnit run script for FPGA Delta Sigma ADC testbenches
"""

from pathlib import Path
from vunit import VUnit
import sys
from subprocess import run

ROOT = Path(__file__).parent

# Handle coverage flag
enable_cov = False
if "-c" in sys.argv:
    enable_cov = True
    sys.argv.remove("-c")  # remove before giving argv to VUnit

# Remove --clean flag if present to prevent automatic clean
if "--clean" in sys.argv:
    sys.argv.remove("--clean")

# Create VUnit instance
try:
    vu = VUnit.from_argv(compile_builtins=False)
    vu.add_vhdl_builtins()
except SystemExit:
    print("VUnit initialization failed. Make sure VUnit is installed:")
    print("pip install vunit_hdl")
    sys.exit(1)

# Add source files
lib = vu.add_library("fpga_lib")
sources_dir = ROOT / "sources"
for fname in [
    "clk_rst_pkg.vhd",           # Includes reset_synchronizer entity
    "clk_rst_tb_pkg.vhd",
    "dsp_utils_pkg.vhd",
    "cic_sinc3_decimator.vhd",
    "fir_equalizer.vhd",
    "fir_lowpass.vhd",
    "uart_sample_streamer.vhd",
    "tdc_quantizer.vhd",
    "tdc_adc_top.vhd",
]:
    f = sources_dir / fname
    if f.exists():
        lib.add_source_file(f)
        print(f"Added source file: {fname}")
    else:
        print(f"Warning: Source file not found: {fname}")

# Add Intel/Altera precompiled simulation libraries
# These are required for Agilex 5 GPIO IP primitives
print("Adding Intel/Altera precompiled simulation libraries...")

# Determine ModelSim installation path from environment or default
import os
model_tech = os.environ.get("MODEL_TECH", "C:/altera_pro/25.3/questa_fse/win64aloem")
intel_lib_base = Path(model_tech).parent / "intel"

# Add Altera primitive libraries (VHDL versions)
# Reference: modelsim.ini [Library] section
vu.add_external_library("altera_mf", intel_lib_base / "vhdl/altera_mf")
vu.add_external_library("altera", intel_lib_base / "vhdl/altera")
vu.add_external_library("altera_lnsim", intel_lib_base / "vhdl/altera_lnsim")
vu.add_external_library("lpm", intel_lib_base / "vhdl/220model")
vu.add_external_library("220model", intel_lib_base / "vhdl/220model")

# Agilex 5 (tennm) primitive libraries
vu.add_external_library("tennm", intel_lib_base / "vhdl/tennm")
vu.add_external_library("tennm_hssi", intel_lib_base / "vhdl/tennm_hssi_all")

# Add Verilog versions of libraries (required for GPIO IP SystemVerilog cores)
vu.add_external_library("altera_mf_ver", intel_lib_base / "verilog/altera_mf")
vu.add_external_library("altera_ver", intel_lib_base / "verilog/altera")
vu.add_external_library("altera_lnsim_ver", intel_lib_base / "verilog/altera_lnsim")
vu.add_external_library("lpm_ver", intel_lib_base / "verilog/220model")
vu.add_external_library("220model_ver", intel_lib_base / "verilog/220model")
vu.add_external_library("tennm_ver", intel_lib_base / "verilog/tennm")

print(f"Added Intel primitive libraries from: {intel_lib_base}")

# Add Platform Designer system (adc_system) for GPIO IP simulation
print("Adding Platform Designer adc_system GPIO IP files...")
adc_system_lib = vu.add_library("adc_system")

# Shared GPIO core library (used by comparator)
gpio_core_lib = vu.add_library("altera_gpio_core10_ph2_2210")
gpio_core_lib.add_source_file(ROOT / "ip/adc_system/gpio_comp/altera_gpio_core10_ph2_2210/synth/altera_gpio.sv")
print("Added GPIO core: altera_gpio.sv")

# Platform Designer IPs (required by adc_system.vhd wrapper)
# Note: We only add the top-level VHDL wrappers from synth/ directories
# The lower-level Verilog implementations will be found via the precompiled Intel libraries

# PLL IP - Not needed for testbench (clocks generated directly in TB)
# iopll_lib = vu.add_library("iopll")
# iopll_lib.add_source_file(ROOT / "ip/adc_system/iopll/sim/iopll.vhd")

# Clock bridge
clock_in_lib = vu.add_library("adc_system_clock_in")
clock_in_lib.add_source_file(ROOT / "ip/adc_system/adc_system_clock_in/synth/adc_system_clock_in.vhd")

# Reset bridges
reset_bridge_lib = vu.add_library("reset_bridge")
reset_bridge_lib.add_source_file(ROOT / "ip/adc_system/reset_bridge/synth/reset_bridge.vhd")

reset_tap_lib = vu.add_library("adc_system_reset_bridge_tap")
reset_tap_lib.add_source_file(ROOT / "ip/adc_system/adc_system_reset_bridge_tap/synth/adc_system_reset_bridge_tap.vhd")

reset_release_lib = vu.add_library("reset_release")
reset_release_lib.add_source_file(ROOT / "ip/adc_system/reset_release/synth/reset_release.vhd")

# System clock bridge
sysclk_bridge_lib = vu.add_library("sysclk_bridge")
sysclk_bridge_lib.add_source_file(ROOT / "ip/adc_system/sysclk_bridge/synth/sysclk_bridge.vhd")

# UART (RS-232) - COMMENTED OUT due to SCFIFO defparam issues in simulation
# uart_lib = vu.add_library("adc_system_rs232_0")
# uart_lib.add_source_file(ROOT / "ip/adc_system/adc_system_rs232_0/synth/adc_system_rs232_0.vhd")

# GPIO IPs - Only gpio_comp used (differential input)
# gpio_adc_io removed - not used in current design
# gpio_dac removed - now using direct DAC output to external RC filter

# gpio_comparator_lib = vu.add_library("gpio_comparator_in")
# gpio_comparator_lib.add_source_file(ROOT / "ip/adc_system/gpio_comparator_in/synth/gpio_comparator_in.vhd")

# GPIO comparator (differential input)
gpio_comp_lib = vu.add_library("gpio_comp")
gpio_comp_lib.add_source_file(ROOT / "ip/adc_system/gpio_comp/synth/gpio_comp.vhd")

# GPIO sub-IP libraries (Verilog implementations used by GPIO wrappers)
altera_gpio_lib = vu.add_library("altera_gpio_2300")
altera_gpio_lib.add_source_file(ROOT / "ip/adc_system/gpio_comp/altera_gpio_2300/synth/gpio_comp_altera_gpio_2300_tepjmha.v")

# PLL sub-IP - Not needed for testbench
# altera_iopll_lib = vu.add_library("altera_iopll_2100")
# altera_iopll_lib.add_source_file(ROOT / "ip/adc_system/iopll/altera_iopll_2100/synth/iopll_altera_iopll_2100_ygllpty.v")

# Reset release sub-IP (SystemVerilog)
rst_clkgate_lib = vu.add_library("intel_user_rst_clkgate_101")
rst_clkgate_lib.add_source_file(ROOT / "ip/adc_system/reset_release/intel_user_rst_clkgate_101/synth/intel_user_rst_clkgate.sv")

# UART sub-IP (Verilog implementation) - COMMENTED OUT due to SCFIFO defparam issues in simulation
# altera_rs232_lib = vu.add_library("altera_up_avalon_rs232_180")
# altera_rs232_lib.add_source_file(ROOT / "ip/adc_system/adc_system_rs232_0/altera_up_avalon_rs232_180/synth/adc_system_rs232_0_altera_up_avalon_rs232_180_vimtwfq.v")
# altera_rs232_lib.add_source_file(ROOT / "ip/adc_system/adc_system_rs232_0/altera_up_avalon_rs232_180/synth/altera_up_rs232_counters.v")
# altera_rs232_lib.add_source_file(ROOT / "ip/adc_system/adc_system_rs232_0/altera_up_avalon_rs232_180/synth/altera_up_rs232_in_deserializer.v")
# altera_rs232_lib.add_source_file(ROOT / "ip/adc_system/adc_system_rs232_0/altera_up_avalon_rs232_180/synth/altera_up_rs232_out_serializer.v")
# altera_rs232_lib.add_source_file(ROOT / "ip/adc_system/adc_system_rs232_0/altera_up_avalon_rs232_180/synth/altera_up_sync_fifo.v")

print("Added Platform Designer IP wrappers (PLL, clocks, resets, GPIO)")

# Platform Designer top-level system wrapper - Not needed for TDC testbench (generates own clocks)
# adc_system_lib.add_source_file(ROOT / "adc_system/synth/adc_system.vhd")
# print("Added Platform Designer top-level: adc_system.vhd")

# Platform Designer reset controller (required by adc_system)
reset_ctrl_lib = vu.add_library("altera_reset_controller_1924")
reset_ctrl_lib.add_source_file(ROOT / "adc_system/altera_reset_controller_1924/synth/altera_reset_controller.v")
reset_ctrl_lib.add_source_file(ROOT / "adc_system/altera_reset_controller_1924/synth/altera_reset_synchronizer.v")
print("Added reset controller files")

# Add testbenches
tb_files = [
    ROOT / "testbench/tdc_adc_top_tb.vhd",
    ROOT / "testbench/tdc_characterization_tb.vhd",  # Open-loop TDC characterization
]
for tb in tb_files:
    if tb.exists():
        lib.add_source_file(tb)
        print(f"Added testbench: {tb.name}")
    else:
        print(f"Warning: Testbench not found: {tb.name}")
if not tb_files:
    print("Warning: No testbench files found!")

# Compiler/simulator options
vu.set_compile_option("modelsim.vcom_flags", ["-2008", "-explicit"])
# Add -voptargs=+acc to show variables in GUI waveforms
# Add Altera primitive libraries for Agilex 5 simulation
vu.set_sim_option("modelsim.vsim_flags", [
    "-t", "ps", 
    "-voptargs=+acc",
    "-L", "altera_mf",
    "-L", "altera",
    "-L", "altera_lnsim",
    "-L", "lpm",
    "-L", "220model",
    "-L", "tennm",
    "-L", "tennm_ver",
    "-L", "altera_mf_ver",
    "-L", "altera_ver",
    "-L", "altera_lnsim_ver",
])

if enable_cov:
    lib.set_sim_option("enable_coverage", True)
    lib.set_compile_option("modelsim.vcom_flags", ["+cover=bs"])

def main():
    """Main test run"""
    print(f"Found {len(lib.get_test_benches())} test bench(es)")

    # Configure test cases for tdc_adc_top_tb with realistic signals
    # NOTE: tb_test_duration_ms is actually in MICROSECONDS (scaled by 1us in TB)
    # This prevents super-slow simulation at 400MHz TDC clock
    # Signal types: 0=sine, 1=DC, 2=ramp, 3=square
    tdc_tb = lib.test_bench("tdc_adc_top_tb")
    if tdc_tb:
        # Test 1: Sine wave at 1kHz
        tdc_tb.test("basic_test").add_config(
            name="sine_1khz",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0  # 50 microseconds
            )
        )
        
        # Test 2: DC level (positive) - 600mV baseline
        tdc_tb.test("basic_test").add_config(
            name="dc_positive",
            generics=dict(
                GC_TB_SIGNAL_TYPE=1,  # DC
                GC_TB_AMPLITUDE=0.0,
                GC_TB_FREQUENCY_HZ=0.0,
                GC_TB_DC_LEVEL=0.5,   # 600mV (0.5 * 1200mV)
                GC_TB_REF_PPM=0.0,
                GC_TB_JIT_RMS_PS=0.0
            )
        )
        
        # Test 2a: DC level at 400mV - verify ADC tracks lower voltage
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_400mv",
            generics=dict(
                GC_TB_SIGNAL_TYPE=1,  # DC
                GC_TB_AMPLITUDE=0.0,
                GC_TB_FREQUENCY_HZ=0.0,
                GC_TB_DC_LEVEL=0.333,  # 400mV (0.333 * 1200mV)
                GC_TB_REF_PPM=0.0,
                GC_TB_JIT_RMS_PS=0.0
            )
        )
        
        # Test 2b: DC level at 800mV - verify ADC tracks higher voltage
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_800mv",
            generics=dict(
                GC_TB_SIGNAL_TYPE=1,  # DC
                GC_TB_AMPLITUDE=0.0,
                GC_TB_FREQUENCY_HZ=0.0,
                GC_TB_DC_LEVEL=0.667,  # 800mV (0.667 * 1200mV)
                GC_TB_REF_PPM=0.0,
                GC_TB_JIT_RMS_PS=0.0
            )
        )
        
        # Test 2c: DC level at 1000mV - verify ADC tracks near-full-scale voltage
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_1000mv",
            generics=dict(
                GC_TB_SIGNAL_TYPE=1,  # DC
                GC_TB_AMPLITUDE=0.0,
                GC_TB_FREQUENCY_HZ=0.0,
                GC_TB_DC_LEVEL=0.833,  # 1000mV (0.833 * 1200mV)
                GC_TB_REF_PPM=0.0,
                GC_TB_JIT_RMS_PS=0.0
            )
        )
        
        # PI TUNING TESTS - Closed-loop step response for gain tuning
        # These run with GC_OPEN_LOOP=false (default) for closed-loop control
        tdc_tb.test("pi_step_response").add_config(
            name="pi_tune_midscale",
            generics=dict(
                GC_TB_SIGNAL_TYPE=1,  # DC
                GC_TB_AMPLITUDE=0.0,
                GC_TB_FREQUENCY_HZ=0.0,
                GC_TB_DC_LEVEL=0.5,   # 650mV (midscale)
                GC_TB_REF_PPM=0.0,
                GC_TB_JIT_RMS_PS=0.0,
                GC_OPEN_LOOP=False    # CRITICAL: Closed-loop for PI tuning
            )
        )
        
        tdc_tb.test("pi_step_response").add_config(
            name="pi_tune_800mv",
            generics=dict(
                GC_TB_SIGNAL_TYPE=1,  # DC
                GC_TB_AMPLITUDE=0.0,
                GC_TB_FREQUENCY_HZ=0.0,
                GC_TB_DC_LEVEL=0.667,  # 800mV
                GC_TB_REF_PPM=0.0,
                GC_TB_JIT_RMS_PS=0.0,
                GC_OPEN_LOOP=False    # CRITICAL: Closed-loop for PI tuning
            )
        )
        
        # Test 3: Higher frequency sine
        tdc_tb.test("basic_test").add_config(
            name="sine_10khz",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.4,
                tb_frequency_hz=10000.0,
                tb_dc_level=0.0,
                tb_noise_level=0.02,
                tb_test_duration_ms=50.0  # 50 microseconds
            )
        )
        
        # Test 5: Square wave
        tdc_tb.test("basic_test").add_config(
            name="square_500hz",
            generics=dict(
                tb_signal_type=3,  # Square
                tb_amplitude=0.3,
                tb_frequency_hz=500.0,
                tb_dc_level=0.0,
                tb_noise_level=0.01,
                tb_test_duration_ms=80.0  # 80 microseconds
            )
        )
        
        # Test 6: Ramp (sawtooth)
        tdc_tb.test("basic_test").add_config(
            name="ramp",
            generics=dict(
                tb_signal_type=2,  # Ramp
                tb_amplitude=0.5,
                tb_frequency_hz=0.0,
                tb_dc_level=0.0,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0  # 50 microseconds
            )
        )
        
        # ========================================================================
        # STRESS TESTS: Handoff validation with realistic clock asynchrony
        # These tests enable GC_TB_REF_PPM and GC_TB_JIT_RMS_PS to stress the
        # closed-loop handoff mechanism. Expected to expose timing bugs.
        # ========================================================================
        
        # Stress Test 1: Moderate jitter, no ppm drift
        tdc_tb.test("basic_test").add_config(
            name="stress_jitter50ps",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0,
                GC_TB_REF_PPM=0.0,      # No ppm drift
                GC_TB_JIT_RMS_PS=50.0   # 50ps RMS jitter
            )
        )
        
        # Stress Test 2: High jitter
        tdc_tb.test("basic_test").add_config(
            name="stress_jitter150ps",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0,
                GC_TB_REF_PPM=0.0,       # No ppm drift
                GC_TB_JIT_RMS_PS=150.0   # 150ps RMS jitter (aggressive)
            )
        )
        
        # Stress Test 3: PPM drift, no jitter
        tdc_tb.test("basic_test").add_config(
            name="stress_ppm20",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0,
                GC_TB_REF_PPM=20.0,     # 20 ppm offset
                GC_TB_JIT_RMS_PS=0.0    # No jitter
            )
        )
        
        # Stress Test 4: Large PPM drift
        tdc_tb.test("basic_test").add_config(
            name="stress_ppm100",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0,
                GC_TB_REF_PPM=100.0,    # 100 ppm offset (worst case)
                GC_TB_JIT_RMS_PS=0.0    # No jitter
            )
        )
        
        # Stress Test 5: Combined moderate stress
        tdc_tb.test("basic_test").add_config(
            name="stress_combined_moderate",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0,
                GC_TB_REF_PPM=20.0,     # 20 ppm
                GC_TB_JIT_RMS_PS=50.0   # 50ps jitter
            )
        )
        
        # Stress Test 6: Maximum combined stress
        tdc_tb.test("basic_test").add_config(
            name="stress_combined_max",
            generics=dict(
                tb_signal_type=0,  # Sine
                tb_amplitude=0.3,
                tb_frequency_hz=1000.0,
                tb_dc_level=0.1,
                tb_noise_level=0.01,
                tb_test_duration_ms=50.0,
                GC_TB_REF_PPM=100.0,     # 100 ppm
                GC_TB_JIT_RMS_PS=150.0   # 150ps jitter
            )
        )
        
        # ====================================================================
        # TDC CHARACTERIZATION TESTS (Open-Loop Mode)
        # ====================================================================
        # REMOVED: Old simple characterization tests
        # 
        # NEW: 2D Sweep to find usable TDC operating region
        # Step 1: Is there ANY (Vin, Vdac) pair where TDC is NOT saturated?
        #
        # Test Matrix:
        #   Vin:  400mV, 650mV, 900mV  (low/mid/high, normalized: 0.31, 0.50, 0.69)
        #   DAC:  0%, 10%, 20%, ..., 100% (11 levels)
        #   Total: 33 test points
        #
        # For each point, testbench will:
        #   1. Fix DAC to specified duty cycle
        #   2. Apply DC input at Vin
        #   3. Log TDC histogram (mean, min, max, stddev)
        #   4. Determine if TDC is saturated or has usable dynamic range
        #
        # The testbench will sweep DAC duties automatically within each test.
        # This simplifies test configuration (just specify Vin).
        
        # Three input voltages to characterize
        vin_levels = [
            (0.31, "400mv"),  # 400mV / 1300mV = 0.31
            (0.50, "650mv"),  # 650mV / 1300mV = 0.50
            (0.69, "900mv"),  # 900mV / 1300mV = 0.69
        ]
        
        for dc_level, name_suffix in vin_levels:
            tdc_tb.test("tdc_characterization").add_config(
                name=f"char2d_{name_suffix}",
                generics=dict(
                    GC_TB_SIGNAL_TYPE=1,  # DC input
                    GC_TB_DC_LEVEL=dc_level,
                    GC_TB_AMPLITUDE=0.0,
                    GC_TB_FREQUENCY_HZ=1000.0,
                    GC_OPEN_LOOP=True  # CRITICAL: Enable open-loop mode for DAC characterization
                )
            )
        
        print("Configured 2D TDC characterization tests (3 Vin × 11 DAC duties = 33 test points):")
        print("\nBASELINE TESTS (no stress):")
        print("  - sine_1khz: 1kHz sine wave with DC offset (50us)")
        print("  - dc_positive: Positive DC level at 600mV (200us) - baseline")
        print("  - dc_positive_400mv: DC at 400mV - verify ADC tracks lower voltage")
        print("  - dc_positive_800mv: DC at 800mV - verify ADC tracks higher voltage")
        print("  - dc_positive_1000mv: DC at 1000mV - verify ADC tracks near-full-scale")
        print("  - sine_10khz: 10kHz sine wave (50us)")
        print("  - square_500hz: 500Hz square wave (80us)")
        print("  - ramp: Sawtooth ramp signal (50us)")
        print("\nSTRESS TESTS (realistic clock asynchrony to expose handoff bugs):")
        print("  - stress_jitter50ps: 50ps RMS jitter")
        print("  - stress_jitter150ps: 150ps RMS jitter (aggressive)")
        print("  - stress_ppm20: 20 ppm frequency offset")
        print("  - stress_ppm100: 100 ppm frequency offset (worst case)")
        print("  - stress_combined_moderate: 20ppm + 50ps jitter")
        print("  - stress_combined_max: 100ppm + 150ps jitter (maximum stress)")
        print("\nCHARACTERIZATION TESTS (open-loop TDC mapping):")
        print("  - char_dc*: TDC output vs input voltage (300mV to 800mV)")

    def post_run(results):
        if not enable_cov:
            return
        results.merge_coverage(file_name="coverage_data.ucdb")
        ucdb = str(Path("coverage_data.ucdb").resolve())

        inst_args = [
            "-instance=/cic_sinc3_decimator_tb/i_dut",
            "-instance=/fir_equalizer_tb/i_dut",
            "-instance=/fir_lowpass_tb/i_dut",
            "-instance=/rc_adc_top_tb/i_dut",
            "-instance=/tdc_quantizer_tb/i_dut",
            "-instance=/tdc_adc_top_tb/i_dut",
        ]
        run(["vcover", "report", "-html", "-output", "cov_html", *inst_args, ucdb], check=False)
        run(["vcover", "report", "-details", *inst_args, ucdb], check=False)
        print("HTML coverage report (DUT-only): cov_html/index.html")

    vu.main(post_run=post_run)

if __name__ == "__main__":
    main()
