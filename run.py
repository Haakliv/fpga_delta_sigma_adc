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

# Handle fast mode flag (disables +acc for faster simulation)
fast_mode = False
if "-f" in sys.argv or "--fast" in sys.argv:
    fast_mode = True
    if "-f" in sys.argv:
        sys.argv.remove("-f")
    if "--fast" in sys.argv:
        sys.argv.remove("--fast")
    print("FAST MODE: Disabling +acc for faster simulation (no waveform visibility)")

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
source_files = []
for fname in [
    "clk_rst_pkg.vhd",           # Includes reset_synchronizer entity
    "dsp_utils_pkg.vhd",
    "cic_sinc3_decimator.vhd",
    "fir_equalizer.vhd",
    "fir_lowpass.vhd",
    "uart_sample_streamer.vhd",
    "adc_capture.vhd",           # Burst capture with inferred block RAM
    "tdc_quantizer.vhd",
    "tdc_adc_top.vhd",
    "rc_adc_top.vhd",            # RC delta-sigma ADC
]:
    f = sources_dir / fname
    if f.exists():
        sf = lib.add_source_file(f)
        source_files.append(sf)
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

# NOTE: In coverage mode, we DON'T use add_external_library because it adds -L flags
# to vsim which causes Questa Intel Starter FPGA Edition to crash (vsim-191).
# The altera_mf library is already defined in modelsim.ini so compilation works.
# In non-coverage mode, add_external_library is used for explicit library management.

if not enable_cov:
    # Full library set for normal simulation (add_external_library safe without coverage)
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
else:
    print(f"COVERAGE MODE: Using only altera_mf from: {intel_lib_base}")

# Add Platform Designer system (adc_system) for GPIO IP simulation
# NOTE: When coverage is enabled (-c flag), we skip the Intel Verilog/SV IP files
# to avoid a Questa vsim-191 crash. The testbenches use behavioral models instead.
if not enable_cov:
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
else:
    # Coverage mode: Skip Intel IP to avoid Questa vsim-191 crash
    # Testbenches use behavioral models instead of GPIO IP
    print("COVERAGE MODE: Skipping Intel IP files (using behavioral models)")

# Add testbenches
tb_files = [
    ROOT / "testbench/clk_rst_tb_pkg.vhd",      # Testbench package (clocks/resets)
    ROOT / "testbench/dsp_utils_pkg_tb.vhd",    # DSP utilities unit tests
    ROOT / "testbench/cic_sinc3_decimator_tb.vhd",  # CIC decimator test
    ROOT / "testbench/fir_equalizer_tb.vhd",    # FIR equalizer filter test
    ROOT / "testbench/fir_lowpass_tb.vhd",      # FIR lowpass filter test
    ROOT / "testbench/tdc_quantizer_tb.vhd",    # TDC quantizer unit test
    ROOT / "testbench/adc_top_tb.vhd",          # Unified ADC testbench (TDC and RC)
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

# Simulator flags configuration
# In coverage mode, skip ALL -L flags to avoid Questa crash
if enable_cov:
    # Coverage mode: No -L library flags at all
    vsim_flags = ["-t", "ps"]
else:
    # Normal mode: Include all Intel libraries for full functionality
    vsim_flags = [
        "-t", "ps", 
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
    ]
    if not fast_mode:
        vsim_flags.append("-voptargs=+acc")  # Enable waveform visibility

vu.set_sim_option("modelsim.vsim_flags", vsim_flags)

if enable_cov:
    # Apply coverage compile flags only to our VHDL source files (not Intel IP or testbenches)
    # Note: Intel IP Verilog/SV files are completely excluded when coverage is enabled
    for sf in source_files:
        sf.set_compile_option("modelsim.vcom_flags", ["-2008", "-explicit", "+cover=bs"])
    
    # Enable coverage globally - test Intel DCFIFO without -L flags
    lib.set_sim_option("enable_coverage", True)
    
    print("\nCOVERAGE MODE: Enabled (Intel DCFIFO, no -L library flags)\n")

def main():
    """Main test run"""
    print(f"Found {len(lib.get_test_benches())} test bench(es)")

    # Configure test cases for adc_top_tb (unified TDC and RC ADC testbench)
    # NOTE: tb_test_duration_ms is actually in MICROSECONDS (scaled by 1us in TB)
    # This prevents super-slow simulation at 400MHz TDC clock
    # Signal types: 0=sine, 1=DC, 2=ramp, 3=square
    # GC_ADC_TYPE: "tdc" for TDC ADC, "rc" for RC ADC
    
    tdc_tb = lib.test_bench("adc_top_tb")
    if tdc_tb:
        # Test 1: Sine wave at 1kHz
        tdc_tb.test("basic_test").add_config(
            name="sine_1khz",
            generics={
                "GC_ADC_TYPE": "tdc",
                "GC_TB_SIGNAL_TYPE": 0,  # Sine
                "GC_TB_AMPLITUDE": 0.3,
                "GC_TB_FREQUENCY_HZ": 1000.0,
                "GC_TB_DC_LEVEL": 0.5
            }
        )
        
        # Test 2: DC level (positive) - 650mV (midscale for 1.3V range)
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_650mv",
            generics={
                "GC_ADC_TYPE": "tdc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.5    # 650mV (0.5 * 1300mV)
            }
        )
        
        # Test 2a: DC level at 400mV - verify ADC tracks lower voltage
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_400mv",
            generics={
                "GC_ADC_TYPE": "tdc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.333  # 400mV (0.333 * 1200mV)
            }
        )
        
        # Test 2b: DC level at 800mV - verify ADC tracks higher voltage
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_800mv",
            generics={
                "GC_ADC_TYPE": "tdc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.667  # 800mV (0.667 * 1200mV)
            }
        )
        
        # Test 2c: DC level at 1000mV - verify ADC tracks near-full-scale voltage
        tdc_tb.test("basic_test").add_config(
            name="dc_positive_1000mv",
            generics={
                "GC_ADC_TYPE": "tdc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.833  # 1000mV (0.833 * 1200mV)
            }
        )
        
        # Test 3: Higher frequency sine

        
        # ====================================================================
        # TDC CHARACTERIZATION TESTS (Open-Loop Mode) - DISABLED
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
        
        # # Three input voltages to characterize
        # vin_levels = [
        #     (0.31, "400mv"),  # 400mV / 1300mV = 0.31
        #     (0.50, "650mv"),  # 650mV / 1300mV = 0.50
        #     (0.69, "900mv"),  # 900mV / 1300mV = 0.69
        # ]
        # 
        # for dc_level, name_suffix in vin_levels:
        #     tdc_tb.test("tdc_characterization").add_config(
        #         name=f"char2d_{name_suffix}",
        #         generics=dict(
        #             GC_ADC_TYPE="tdc",
        #             GC_TB_SIGNAL_TYPE=1,  # DC input
        #             GC_TB_DC_LEVEL=dc_level,
        #             GC_TB_AMPLITUDE=0.0,
        #             GC_TB_FREQUENCY_HZ=1000.0,
        #             GC_OPEN_LOOP=True  # CRITICAL: Enable open-loop mode for DAC characterization
        #         )
        #     )
        
        print("\nTDC TESTS:")
        print("  - sine_1khz: 1kHz sine wave with DC offset")
        print("  - dc_positive_650mv: DC level at 650mV - baseline")
        print("  - dc_positive_400mv: DC at 400mV")
        print("  - dc_positive_800mv: DC at 800mV")
        print("  - dc_positive_1000mv: DC at 1000mV")

        # ========================================================================
        # RC ADC TESTS (GC_ADC_TYPE="rc")
        # ========================================================================
        
        # RC ADC: DC tests at various voltages (1.25V range)
        tdc_tb.test("basic_test").add_config(
            name="rc_dc_positive_400mv",
            generics={
                "GC_ADC_TYPE": "rc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.333  # Match TDC: 416mV
            }
        )
        
        tdc_tb.test("basic_test").add_config(
            name="rc_dc_positive_650mv",
            generics={
                "GC_ADC_TYPE": "rc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.5    # Match TDC: 625mV
            }
        )
        
        tdc_tb.test("basic_test").add_config(
            name="rc_dc_positive_800mv",
            generics={
                "GC_ADC_TYPE": "rc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.667  # Match TDC: 834mV
            }
        )
        
        tdc_tb.test("basic_test").add_config(
            name="rc_dc_positive_1000mv",
            generics={
                "GC_ADC_TYPE": "rc",
                "GC_TB_SIGNAL_TYPE": 1,  # DC
                "GC_TB_AMPLITUDE": 0.0,
                "GC_TB_FREQUENCY_HZ": 0.0,
                "GC_TB_DC_LEVEL": 0.833  # Match TDC: 1041mV
            }
        )
        
        print("\nRC ADC TESTS:")
        print("  - rc_dc_positive_400mv: DC at 400mV")
        print("  - rc_dc_positive_650mv: DC at 650mV (midscale)")
        print("  - rc_dc_positive_800mv: DC at 800mV")
        print("  - rc_dc_positive_1000mv: DC at 1000mV")

    def post_run(results):
        if not enable_cov:
            return
        
        # Use VUnit's standard coverage merge
        results.merge_coverage(file_name="coverage_data.ucdb")
        ucdb = str(Path("coverage_data.ucdb").resolve())

        # Coverage instances for all DUTs
        # Note: dsp_utils_pkg_tb tests package functions, no DUT instance
        inst_args = [
            "-instance=/cic_sinc3_decimator_tb/i_dut",
            "-instance=/fir_equalizer_tb/i_dut",
            "-instance=/fir_lowpass_tb/i_dut",
            "-instance=/tdc_quantizer_tb/i_dut",
            "-instance=/adc_top_tb/g_tdc_adc/i_dut_tdc",
            "-instance=/adc_top_tb/g_rc_adc/i_dut_rc",
        ]
        
        # Generate HTML report (suppress terminal output)
        run(["vcover", "report", "-html", "-output", "cov_html", *inst_args, str(ucdb)], 
            check=False, capture_output=True)
        
        print("\n=== COVERAGE REPORT GENERATED ===")
        print(f"  HTML: {Path('cov_html/index.html').resolve()}")

    vu.main(post_run=post_run)

if __name__ == "__main__":
    main()
