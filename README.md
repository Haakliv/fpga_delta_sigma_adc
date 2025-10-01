# Delta-Sigma ADC Integration with AXE5000 Reference Design

This project integrates a minimal delta-sigma ADC with the NIOS-V processor on the AXE5000 development kit.

## Project Structure

```
fpga_delta_sigma_adc/
├── src/
│   ├── hdl/                              # VHDL source files
│   │   ├── axe5000_delta_sigma_top.vhd   # Top-level entity
│   │   ├── delta_sigma_adc_avalon.vhd    # Avalon-MM wrapper
│   │   ├── cic_sinc3_decimator.vhd       # CIC SINC3 filter
│   │   ├── dac_1_bit.vhd                 # 1-bit DAC
│   │   └── clk_rst_pkg.vhd               # Clock/reset package
│   └── constraints/
│       └── axe5000_pin_assignment.tcl    # Pin assignments
├── software/
│   └── delta_sigma_adc_reader.c          # NIOS-V C application
├── testbench/
│   ├── cic_sinc3_decimator_tb.vhd        # CIC decimator testbench  
│   └── dac_1_bit_tb.vhd                  # DAC testbench
└── README.md                             # This file
```

## Setup Instructions

### 1. Quartus Project Setup
1. Create new Quartus project targeting Agilex 5 FPGA (A5ED065BB32A7)
2. Set `src/hdl/axe5000_delta_sigma_top.vhd` as top-level entity
3. Add all VHDL files from `src/hdl/` to project
4. Source pin assignments: `source src/constraints/axe5000_pin_assignment.tcl`

### 2. Qsys System Creation
Create a NIOS-V system with the following components:

**Required Components:**
- NIOS-V Processor (32-bit, with caches)
- On-chip Memory (256KB SRAM)
- PLL (25MHz input → 100MHz output)  
- UART (115200 baud)
- Avalon-MM Bridge for ADC interface

**ADC Interface Configuration:**
- Address Width: 1 bit (2 registers)
- Data Width: 32 bits
- Base Address: 0x4000 (or similar)
- IRQ connection: Connect adc_avalon_irq to NIOS-V interrupt

### 3. Memory Map (Simplified)
```
Base + 0x00: ADC_DATA   - 16-bit conversion result (read clears data_ready)
Base + 0x04: STATUS     - Data ready flags and current ADC value
```

### 4. Hardware Connections

**External Analog Components Needed:**
- Analog input signal (0-3.3V)
- Comparator (comparing input to DAC feedback)
- RC integrator on DAC output
- Optional: Anti-aliasing filter

**Connections:**
```
ANALOG_IN ← Comparator output (1-bit digital)
DAC_OUT   → RC integrator → Comparator reference
```

### 5. Software Build
1. Create NIOS-V C/C++ project in Intel FPGA Monitor Program
2. Add `software/delta_sigma_adc_reader.c` to project
3. Update `ADC_AVALON_BASE` address in code to match Qsys assignment
4. Build and download to NIOS-V

### 6. Testing
1. Connect UART to PC (115200 baud, 8N1)
2. Apply analog signal to input
3. Monitor UART output for:
   - **Startup messages** and system status
   - **"Active" heartbeat** every second
   - **ADC sample data** when conversions are ready
4. Observe `TEST_PIN` for interrupt activity

## Signal Description

### ADC Operation
1. **Sigma-Delta Modulation**: External comparator creates 1-bit stream at 100 MHz
2. **CIC Decimation**: 3rd-order sinc decimator (OSR=57,344) → 1,745 Hz output
3. **Sinc³ Equalization**: 31-tap FIR compensates CIC droop (non-decimating)
4. **Anti-Alias Filtering**: 95-tap linear-phase lowpass (Hamming window, Fc=750Hz)
5. **Avalon Interface**: NIOS-V reads filtered data via memory-mapped registers
6. **UART Output**: ADC samples transmitted to PC at 1,745 Hz (91% UART bandwidth)

### Signal Processing Chain Details

**CIC Decimator (OSR = 57,344 = 7 × 2^13)**
- Input: 1-bit bitstream at 100 MHz
- Output: 16-bit samples at 1,745 Hz
- Architecture: 3 integrators + 3 comb filters
- Gain: OSR³ with exact scaling via bit shifts
- Rounding: Added before final scaling to minimize quantization noise

**Sinc³ Equalizer (31-tap FIR, non-decimating)**
- Purpose: Compensates CIC's sinc³(f/Fs) droop
- Passband: Flattens response from DC to 0.5 × Nyquist (872 Hz)
- Coefficients: Q1.15 format (16-bit signed fixed-point)
- Implementation: Symmetric structure exploits h[i] = h[30-i]
- Latency: 15 samples (8.6 ms at 1,745 Hz)

**Anti-Alias Lowpass (95-tap FIR, linear-phase)**
- Purpose: Attenuate frequencies above Nyquist before final output
- Design: Hamming window, Fc = 750 Hz (0.43 × Fs)
- Passband: DC-700 Hz with <0.5 dB ripple
- Stopband: >872 Hz (Nyquist) with >70 dB attenuation
- Coefficients: Q1.15 format with symmetric structure
- Latency: 47 samples (27 ms at 1,745 Hz)

### Expected Performance
- Sample Rate: **1,745 Hz** (100 MHz / 57,344)
- Resolution: ~14-16 bits effective (depends on analog input quality)
- Latency: ~94 samples total (CIC=28, Eq=15, LP=47, Avalon=4)
- UART Utilization: **91%** (1,745 Hz × 6 bytes = 10,470 baud of 115,200)
- Passband Flatness: <0.5 dB from DC to 700 Hz
- Stopband Rejection: >70 dB above 872 Hz (Nyquist frequency)

## Software Features

### UART Output
- **Startup banner** with system information
- **Heartbeat messages**: "Active (heartbeat #N)" every second
- **ADC data**: "Sample N: ADC = DDDD (0xHHHH)" when new data available
- **Status information** during initialization

### Timing
- **Heartbeat**: 1 second intervals for system health monitoring
- **ADC polling**: Optimized delays to prevent busy-waiting
- **Sample processing**: Immediate display when new data arrives

## Customization

### Changing Decimation Factor (OSR)
Edit `ADC_DECIMATION` generic in `src/hdl/axe5000_delta_sigma_top.vhd`:
```vhdl
generic map (ADC_DECIMATION => 128)  -- Higher resolution, lower sample rate
generic map (ADC_DECIMATION => 32)   -- Higher sample rate, lower resolution
```

### Modifying Heartbeat Rate
Edit `HEARTBEAT_INTERVAL_US` in `software/delta_sigma_adc_reader.c`:
```c
#define HEARTBEAT_INTERVAL_US   500000    // 0.5 seconds
#define HEARTBEAT_INTERVAL_US   2000000   // 2 seconds
```

### Adding Features
- Multiple ADC channels
- Data logging/buffering
- Hardware trigger/timing control
- DMA for high-speed data transfer
- Real-time signal processing

## Debug Tips
- Use `TEST_PIN` for scope measurements (shows ADC interrupt activity)
- Monitor UART for continuous heartbeat to verify system operation
- Check PLL lock and clock frequencies
- Verify ADC sample rate matches expected decimation
- Use heartbeat timing to diagnose system responsiveness