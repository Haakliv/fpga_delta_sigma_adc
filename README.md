# Delta-Sigma ADC with AXE5000 Development Kit

This project implements a complete delta-sigma ADC on the AXE5000 (Agilex 5 FPGA) development kit, streaming real-time ADC samples directly over UART.

## Project Structure

```
accel_temp_refdes/
├── sources/                          # VHDL source files
│   ├── axe5000_top.vhd              # Top-level entity (integrates all modules)
│   ├── rc_adc_top.vhd               # Delta-sigma ADC core with filter chain
│   ├── cic_sinc3_decimator.vhd      # CIC SINC3 decimator filter
│   ├── fir_equalizer.vhd            # SINC3 droop compensation filter
│   ├── fir_lowpass.vhd              # Anti-alias lowpass filter
│   ├── uart_sample_streamer.vhd     # Sample-to-hex UART formatter
│   ├── tdc_quantizer.vhd            # TDC quantizer (future development)
│   ├── clk_rst_pkg.vhd              # Synthesizable clock/reset types
│   ├── clk_rst_tb_pkg.vhd           # Testbench utilities (non-synthesizable)
│   ├── dsp_utils_pkg.vhd            # DSP utility functions (saturate, etc.)
│   ├── axe5000_pin_assignment.tcl   # Pin assignments for AXE5000
│   └── axe5000_top.sdc              # Timing constraints
├── testbench/                        # Simulation testbenches
│   ├── cic_sinc3_decimator_tb.vhd   # CIC decimator verification
│   └── ...                          # Other testbenches
├── ip/                               # Platform Designer IP components
├── adc_system/                       # Generated Qsys system
├── build/                            # Build artifacts
├── run.py                            # Python ADC monitor script
├── adc_monitor.py                    # ADC data acquisition and plotting
├── axe5000_top.qpf/.qsf             # Quartus project files
└── README.md                         # This file
```

## Hardware Architecture

### Delta-Sigma ADC Core (`rc_adc_top.vhd`)
- **1-bit comparator input**: LVDS differential analog comparator
- **1-bit DAC output**: Integrated feedback (simple register, no separate module)
- **Dual-path design**: 
  - Path A: Direct DAC feedback (1 clock cycle loop delay)
  - Path B: 2-FF synchronizer for decimator (metastability protection, no reset)
- **Decimation and filtering**: CIC → Equalizer → Lowpass

### Signal Processing Chain
1. **CIC SINC3 Decimator** (OSR = 57,344 = 7 × 2¹³)
   - 100 MHz input → 1,745 Hz output
   - 3 integrators + 3 comb filters
   - Exact gain scaling with rounding

2. **SINC3 Equalizer** (31-tap FIR)
   - Compensates CIC droop
   - Passband: DC-872 Hz (flat response)
   - Q1.15 fixed-point coefficients

3. **Anti-Alias Lowpass** (95-tap FIR)
   - Fc = 750 Hz, Hamming window
   - >70 dB stopband attenuation
   - Linear phase response

### UART Sample Streamer (`uart_sample_streamer.vhd`)
- Captures ADC samples and formats as hex ASCII
- Output: 4 hex digits + CR + LF (e.g., "A5F3\r\n")
- Handles sample buffering and flow control
- Self-contained state machine

## Setup Instructions

### 1. Hardware Requirements
- **AXE5000 Development Kit** (Agilex 5 A5ED065BB32A7)
- **External analog circuit**:
  - Differential comparator (LVDS output)
  - RC integrator network
  - Optional: anti-aliasing filter
- **USB UART connection** to PC

### 2. FPGA Build Process
1. Open Quartus project: `axe5000_top.qpf`
2. Verify top-level entity: `axe5000_top.vhd`
3. Compile design (Full Compilation)
4. Program FPGA with generated `.sof` file

### 3. Platform Designer System (`adc_system.qsys`)
The project includes a pre-configured Platform Designer system with:
- **NIOS-V Processor** (32-bit RISC-V)
- **100 MHz PLL** (from 25 MHz input)
- **UART** (115200 baud, 8N1)
- **On-chip memory** for firmware
- **Reset controller**

### 4. External Connections

**Analog Input Chain:**
```
Signal → Anti-alias Filter → Comparator (+) 
                              Comparator (-) ← DAC_OUT (via RC integrator)
                              Comparator Out → ANALOG_IN (LVDS)
```

**UART Connection:**
- Connect UART_TX to PC serial port (115200 baud, 8N1)
- Monitor with terminal or use provided Python script (`run.py`)

### 5. Software Monitoring
Use the included Python script for real-time ADC monitoring:
```bash
python run.py
```
or
```bash
python adc_monitor.py --port COM3 --plot
```

## Performance Specifications

- **Sample Rate**: 1,745 Hz (100 MHz / 57,344)
- **Resolution**: ~14-16 bits effective (SNR depends on analog frontend)
- **Bandwidth**: DC - 700 Hz (-3 dB)
- **Nyquist Frequency**: 872 Hz
- **Stopband Rejection**: >70 dB above Nyquist
- **Passband Ripple**: <0.5 dB
- **Group Delay**: ~94 samples (~54 ms total latency)
- **UART Data Rate**: 1,745 samples/sec = 10,470 bytes/sec (91% UART utilization)

## Signal Description

### Delta-Sigma Modulation
1. **Comparator**: Creates 1-bit stream at 100 MHz sampling rate
2. **1-bit DAC**: Single register feedback (minimized loop delay)
3. **RC Integrator**: External passive low-pass filter
4. **Dual-path architecture**: 
   - Direct DAC feedback (1 cycle latency)
   - Synchronized decimator input (metastability protection)

### Digital Filtering
**CIC Decimator**: 100 MHz → 1,745 Hz  
**Equalizer**: Compensates CIC droop, preserves bandwidth  
**Lowpass**: Final anti-alias filtering with steep rolloff

## Key Design Features

### Optimized Synchronizer Design
- **No synchronous reset** in 2-FF synchronizer (minimizes FF delay)
- **Altera synthesis attributes** ensure proper placement and recognition
- **Metastability protection** without functional reset dependency

### Modular Architecture
- **Separated concerns**: ADC core, UART streamer, top-level integration
- **Reusable components**: Each module can be used independently
- **Clean interfaces**: Standard ready/valid handshaking

### Non-Synthesizable Code Separation
- **clk_rst_pkg.vhd**: Only synthesizable types and constants
- **clk_rst_tb_pkg.vhd**: Testbench utilities (clock generators, etc.)
- **Prevents synthesis tool warnings** from non-synthesizable constructs

## Future Development

### TDC Quantizer (`tdc_quantizer.vhd`)
- **Time-to-Digital Converter** for enhanced resolution
- **Currently included** but not instantiated
- **Future enhancement** for multi-bit quantization

## Customization

### Changing Decimation Ratio (OSR)
Edit `GC_ADC_DECIMATION` generic in `axe5000_top.vhd`:
```vhdl
GC_ADC_DECIMATION : positive := 32768  -- 100 MHz / 32768 = 3,052 Hz
GC_ADC_DECIMATION : positive := 65536  -- 100 MHz / 65536 = 1,526 Hz
```
**Note**: Higher OSR = better resolution but lower sample rate

### Modifying Data Width
Change `C_ADC_DATA_WIDTH` in `axe5000_top.vhd`:
```vhdl
constant C_ADC_DATA_WIDTH : positive := 12;  -- 12-bit output
constant C_ADC_DATA_WIDTH : positive := 24;  -- 24-bit output
```

### Adding Features
- **Multiple ADC channels**: Duplicate ADC core with MUX
- **DMA transfers**: Replace UART with high-speed DMA
- **Memory-mapped interface**: Enable the Avalon-MM signals in `rc_adc_top`
- **Different filter designs**: Replace FIR filters with custom coefficients
- **Adaptive OSR**: Dynamic decimation ratio based on signal conditions

## Debug and Verification

### Test Points
- **TEST_PIN**: Outputs the raw LVDS comparator bit (useful for scope monitoring)
- **DAC_OUT**: Observe feedback signal quality
- **UART_TX**: Monitor hex-formatted sample stream

### Common Issues
1. **No UART output**: Check PLL lock, verify 100 MHz clock generation
2. **Garbled data**: Confirm 115200 baud, 8N1 settings
3. **Saturated readings**: Check analog input range and comparator thresholds
4. **Low SNR**: Verify RC integrator time constant, check comparator noise

### Simulation
Testbenches are provided for key components:
- `cic_sinc3_decimator_tb.vhd`: Verify decimation and gain
- Additional testbenches for FIR filters and full system

## References and Theory

### Delta-Sigma Modulation
- Oversampling shifts quantization noise to high frequencies
- Noise shaping pushes noise out of band of interest
- Digital filtering removes out-of-band noise

### CIC Filter Design
- Computationally efficient (no multipliers)
- Inherent sinc³ droop requires equalization
- Gain = OSR³ must be compensated

### Project Background
This design was developed for high-precision analog measurements on FPGA platforms, demonstrating modern delta-sigma ADC techniques with careful attention to metastability protection and modular design principles.