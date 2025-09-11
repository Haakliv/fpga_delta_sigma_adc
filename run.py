#!/usr/bin/env python3
"""
VUnit run script for FPGA Delta Sigma ADC testbenches
"""

from pathlib import Path
from vunit import VUnit
import sys

# Create VUnit instance
try:
    vu = VUnit.from_argv(compile_builtins=False)
    vu.add_vhdl_builtins()
except SystemExit:
    print("VUnit initialization failed. Make sure VUnit is installed:")
    print("pip install vunit_hdl")
    sys.exit(1)

# Get the root directory
root = Path(__file__).parent

# Add source libraries
lib = vu.add_library("fpga_lib")

# Add package files first (dependencies)
try:
    # Add all source files
    source_files = [
        "clk_rst_pkg.vhd",
        "dac_1_bit.vhd", 
        "cic_sinc3_decimator.vhd",
        "fir_equalizer.vhd",
        "fir_lowpass.vhd",
        "lvds_comparator.vhd",
        "rc_adc_top.vhd",
        "tdc_quantizer.vhd"
    ]
    
    sources_dir = root / "sources"
    for source_file in source_files:
        source_path = sources_dir / source_file
        if source_path.exists():
            lib.add_source_file(source_path)
            print(f"Added source file: {source_file}")
        else:
            print(f"Warning: Source file not found: {source_file}")
            
except Exception as e:
    print(f"Warning: Could not add all source files: {e}")
    print("Make sure your VHDL source files are in the sources/ directory")

# Add all fixed testbench files
try:
    tb_files = list((root / "testbench").glob("*_tb.vhd"))
    for tb_file in tb_files:
        lib.add_source_file(tb_file)
        print(f"Added testbench: {tb_file.name}")
        
    if not tb_files:
        print("Warning: No testbench files found!")
        
except Exception as e:
    print(f"Error adding testbench files: {e}")
    sys.exit(1)

# Configure test runner for ModelSim/QuestaSim
vu.set_compile_option("modelsim.vcom_flags", ["-2008", "-explicit"])
vu.set_sim_option("modelsim.vsim_flags", ["-t", "ps"])

def main():
    """Main function to run tests"""
    try:
        print(f"Found {len(lib.get_test_benches())} test bench(es)")
    except:
        print("Test benches loaded successfully")
    vu.main()

if __name__ == "__main__":
    main()
