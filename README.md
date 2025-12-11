# AXE5000 Delta-Sigma ADC Reference Design

High-resolution Delta-Sigma ADC implementation for Intel Agilex 5 FPGA with TDC-enhanced architecture for precision measurement applications.

## 🎯 Project Overview

This reference design implements a Delta-Sigma ADC on the AXE5000 development board with two architecture variants:
- **TDC-Based ADC** (`tdc_adc_top`): High-resolution architecture using Time-to-Digital Converter for enhanced precision
- **RC-Based ADC** (`rc_adc_top`): Simplified architecture with RC integrator feedback topology

Both architectures feature CIC SINC3 decimation, FIR equalization, and low-pass filtering with real-time UART streaming.

### Key Specifications
- **Resolution**: 16-bit signed output (Q15 format)
- **Sample Rate**: 7.8 kS/s (configurable decimation)
- **Input Range**: 0-1.3V (VADJ reference)
- **Output Format**: ASCII hex over UART (115200 baud)
- **Accuracy**: ±5mV INL with hardware calibration

## 📁 Project Structure

```
sources/
├── axe5000_top.vhd              # Top-level FPGA design with UART control
├── tdc_adc_top.vhd              # TDC-based Delta-Sigma ADC architecture
├── rc_adc_top.vhd               # RC-based Delta-Sigma ADC architecture
├── cic_sinc3_decimator.vhd      # CIC SINC3 decimation filter
├── fir_equalizer.vhd            # 31-tap FIR equalizer (CIC droop compensation)
├── fir_lowpass.vhd              # 63-tap FIR lowpass filter
├── tdc_quantizer.vhd            # Time-to-Digital Converter quantizer
├── adc_capture.vhd              # Burst capture buffer (4096 samples)
└── *.sdc                        # Timing constraints

scripts/
├── adc_monitor.py               # Python UART monitor with calibration
├── tdc_monitor.py               # TDC characterization tool
├── generate_coefficients.py    # FIR filter coefficient generator
└── plot_filters.py              # Filter frequency response visualization

adc_system/                       # Platform Designer system integration
testbench/                        # Simulation testbenches
```

## 🔧 ADC Architectures

### TDC-Based ADC (`tdc_adc_top`)

**Use Case**: Precision measurements requiring high linearity and low noise

**Features**:
- Time-to-Digital Converter quantization at 400 MHz
- TDC center calibration for offset correction
- Multi-bit internal processing (20-bit DAC + TDC)
- Configurable TDC contribution (can disable for CIC-only mode)
- Runtime filter bypass options (EQ, LP)

**Architecture**:
```
Comparator → TDC Quantizer → Multi-bit DSM → CIC Decimator → FIR EQ → FIR LP → Output
                ↑                                                                    ↓
                └────────────────── DAC Feedback ←──────────────────────────────────┘
```

**Clocking**:
- System Clock: 100 MHz (decimator, filters)
- TDC Clock: 400 MHz (time quantization)
- Reference Clock: 50 MHz (TDC start edge)

**Generics**:
- `GC_DECIMATION`: CIC decimation ratio (default: 6400 → 7.8 kS/s)
- `GC_TDC_OUTPUT`: TDC output width (default: 16 bits)
- `GC_OPEN_LOOP`: Open-loop test mode (bypass feedback)
- `GC_SIM`: Enable TDC debug reports for simulation

**Calibration**:
- Automatic TDC center calibration (256 samples at startup)
- Coarse bias calibration for TDL centering (8 samples)
- Hardware offset correction in Python monitor

### RC-Based ADC (`rc_adc_top`)

**Use Case**: Simplified design with lower resource utilization

**Features**:
- RC integrator feedback topology (external RC network)
- Single-bit feedback DAC
- Direct LVDS comparator input (minimal loop delay)
- 2-FF synchronizer for decimator path (metastability protection)

**Architecture**:
```
        ┌─────────────────────────────────────┐
        │                                     │
LVDS ──>├──> DAC FF (1 cycle) ───────────────┤──> DAC Out → RC Filter → Comparator
   ↓    │                                     │
   └────┼──> 2-FF Sync → CIC → FIR EQ → FIR LP → Output
        └─────────────────────────────────────┘
```

**Clocking**:
- Single clock domain (50 MHz for R=64, or 100 MHz for R=6400)

**Generics**:
- `GC_DECIMATION`: CIC decimation ratio (configurable, default: 64)
- `GC_DATA_WIDTH`: Output word width (default: 16 bits)

**Key Differences from TDC ADC**:
- No TDC quantizer (1-bit feedback instead of multi-bit)
- Simpler calibration (no TDC center offset)
- Lower FPGA resource usage
- External RC network required for integration

## 🚀 Getting Started

### Hardware Requirements
- AXE5000 Development Board (Agilex 5 FPGA)
- CRUVI analog input connector
- USB-UART adapter (115200 baud)
- Power supply (1.3V VADJ via VSEL_1V3)

### Software Requirements
- Intel Quartus Prime Pro (Agilex 5 support)
- Python 3.7+ with `pyserial`

### Building the Design

1. **Open the project**:
   ```bash
   quartus_sh --project axe5000_top.qpf
   ```

2. **Select ADC architecture**:
   - Edit `axe5000_top.vhd` to instantiate either:
     - `tdc_adc_top` (default, lines 165-195)
     - `rc_adc_top` (alternative, comment/uncomment)

3. **Compile**:
   ```bash
   quartus_sh --flow compile axe5000_top
   ```

4. **Program FPGA**:
   ```bash
   quartus_pgm -m jtag -o "p;axe5000_top.sof@1"
   ```

### Running the Monitor

**Install dependencies**:
```bash
pip install pyserial
```

**Stream mode** (continuous real-time):
```bash
cd scripts
python adc_monitor.py
```

**Burst mode** (4096 samples with statistics):
```bash
python adc_monitor.py burst
```

### UART Commands

Send single-character commands via UART RX:

| Command | Function                                |
| ------- | --------------------------------------- |
| `B`     | Enter burst mode (capture 4096 samples) |
| `S`     | Enter stream mode (continuous)          |
| `C`     | Trigger single burst capture            |
| `D`     | Disable ADC (pause sampling)            |
| `E`     | Enable ADC (resume sampling)            |
| `X`     | External trigger mode                   |
| `L`     | LED toggle (debug)                      |

## 📊 Performance

### Measured Specifications
- **Resolution**: 14.5 ENOB (effective)
- **Noise Floor**: <3mV RMS (burst mode)
- **INL**: ±5mV (voltage-dependent bias current)
- **Gain Error**: <0.5% (with calibration)
- **Offset Error**: ±1mV (after hardware correction)

### Timing Closure
- **System Clock**: 100 MHz (met)
- **TDC Clock**: 400 MHz (met)
- **Critical Paths**: Pipeline stages added in CIC multiplier and output

### Resource Utilization (TDC ADC)
- **ALMs**: ~2500 (includes TDC, CIC, FIR filters)
- **M20K Blocks**: ~8 (FIR coefficient storage)
- **DSP Blocks**: ~12 (FIR multipliers)

## 🔬 Calibration

The Python monitor applies hardware-based calibration:

**Voltage Reference Calibration**:
```python
V_REF_ACTUAL = 1.34  # Measured VADJ (vs 1.30V nominal)
```
- **Cause**: VADJ regulator tolerance (typ ±3%)

**Offset Correction**:
```python
HARDWARE_OFFSET_MV = -21.0  # mV
```
- **Cause**: Input bias current (~21µA) through 1kΩ feedback resistor

**Q15 to Voltage Conversion**:
```python
# Q15: -32768 → 0mV, 0 → V_REF/2, +32767 → V_REF
mv = (q15_value * V_REF_ACTUAL * 500) / 32768 + V_REF_ACTUAL * 500 + OFFSET
```

## 🛠️ Design Details

### CIC SINC3 Decimator
- **Order**: 3 (SINC³)
- **Decimation**: 6400 (50MHz → 7.8 kS/s)
- **Bit Growth**: log2(R³) = 38 bits for R=6400
- **Normalization**: Fixed-point reciprocal multiply (62-bit scale)
- **Pipeline**: 6 stages for 400 MHz timing closure

### FIR Equalizer (31 taps)
- **Purpose**: Compensate for CIC SINC³ droop in passband
- **Type**: Inverse SINC response
- **Coefficients**: Generated by `generate_coefficients.py`

### FIR Lowpass (63 taps)
- **Purpose**: Final anti-aliasing and noise shaping
- **Cutoff**: ~3 kHz (below Nyquist at 7.8 kS/s / 2)
- **Type**: Windowed SINC (Hamming window)

### UART Streamer
- **Baud Rate**: 115200 (6 bytes/sample = 19.2 samples/sec max)
- **Format**: ASCII hex: `0xABCD\n`
- **Overrun Handling**: Graceful sample dropping (keeps newest)

### Burst Capture Buffer
- **Depth**: 4096 samples (configurable)
- **Trigger**: Software (`C` command) or hardware (`TRIGGER_IN`)
- **Readback**: Automatic UART transmission after capture

## 📈 Testing & Characterization

### Linearity Test
```bash
# Generate test ramp with signal generator
python adc_monitor.py burst
# Analyze captured data for INL/DNL
```

### Noise Test
```bash
# Apply constant DC input
python adc_monitor.py burst
# Statistics show RMS noise and peak-peak variation
```

### Frequency Response
```bash
# Sweep signal generator through ADC passband
python scripts/plot_filters.py
```

## 🐛 Troubleshooting

### Issue: High noise or unstable readings
- **Check**: VADJ voltage (should be 1.3V via `VSEL_1V3`)
- **Check**: TDC calibration completed (LED1 indicator)
- **Check**: Input signal within 0-1.3V range

### Issue: UART data garbled
- **Check**: Baud rate 115200, 8N1
- **Check**: COM port permissions (Linux: add user to `dialout` group)
- **Try**: `python -m serial.tools.miniterm /dev/ttyUSB0 115200`

### Issue: Timing violations
- **Check**: Clock constraints in `.sdc` files
- **Check**: Pipeline stages in CIC decimator (scale_pipe3_75, cic_out_pre)
- **Try**: Reduce clock frequencies or increase decimation

### Issue: Offset or gain error
- **Update**: Calibration constants in `adc_monitor.py`
- **Measure**: Actual VADJ voltage with multimeter
- **Check**: Feedback resistor value (should be 1kΩ ±1%)

## 📚 References

### FPGA Development
- [Intel Agilex 5 FPGA Device Family](https://www.intel.com/content/www/us/en/products/details/fpga/agilex/5.html)
- [Quartus Prime Pro User Guide](https://www.intel.com/content/www/us/en/programmable/documentation/mwh1409959708154.html)

### Delta-Sigma Theory
- [Understanding Delta-Sigma ADCs](https://www.analog.com/media/en/training-seminars/tutorials/MT-022.pdf)
- [CIC Filter Introduction](https://www.embedded.com/the-cic-filter-a-brief-introduction/)
- [TDC-Based ADCs](https://ieeexplore.ieee.org/document/8310263)

### Design Inspiration
- [davemuscle/sigma_delta_converters](https://github.com/davemuscle/sigma_delta_converters) - RC ADC topology reference

## 📄 License

Copyright © 2025. All rights reserved.

## 🤝 Contributing

For questions, issues, or contributions, please contact the project maintainer.

---

**Built with Intel Agilex 5 FPGA** | **Precision Analog on FPGA**
