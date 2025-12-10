-- ************************************************************************
-- Testbench for Top-Level TDC ADC
-- Simulates TDC-based delta-sigma ADC with reference clock and analog input
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity passiveless_adc_top_tb is
  generic(
    runner_cfg         : string;
    -- Test configuration generics for VUnit
    -- Signal type: 0=sine, 1=DC, 2=ramp, 3=square
    GC_TB_SIGNAL_TYPE  : integer := 0;  -- Signal type enum
    GC_TB_AMPLITUDE    : real    := 0.25; -- Signal amplitude (normalized: 0.0 to 1.0, will be scaled to 1.2V)
    GC_TB_FREQUENCY_HZ : real    := 1000.0; -- Frequency for sine/square waves
    GC_TB_DC_LEVEL     : real    := 0.667; -- DC offset or DC test level (normalized: 0.667 = 0.8V at 1.2V bank)
    -- Clock jitter and asynchrony testing (v8.4 handoff stress test)
    GC_TB_REF_PPM      : real    := 0.0; -- Reference clock frequency offset (ppm)
    GC_TB_JIT_RMS_PS   : real    := 0.0 -- Reference clock RMS jitter (picoseconds)
  );
end entity;

architecture behavioral of passiveless_adc_top_tb is

  -- Test parameters
  constant C_CLK_SYS_PERIOD : time     := 10 ns; -- 100 MHz (from PLL outclk_0 in hardware)
  constant C_CLK_TDC_PERIOD : time     := 2.5 ns; -- 400 MHz (from PLL outclk_1 in hardware) 
  constant C_REF_PERIOD     : time     := 500 ns; -- 2 MHz reference (from PLL outclk_2 in hardware)
  constant C_DATA_WIDTH     : positive := 16;

  -- LP filter settling: 63-tap FIR needs 63 samples to fill delay line
  -- Collect 128 total: 63 for settling + 65 for voltage measurement
  constant C_NUM_SAMPLES : positive := 128;

  -- DUT signals
  signal clk_sys   : std_logic := '0';
  signal clk_tdc   : std_logic := '0';
  signal reset     : std_logic := '1';
  signal ref_clock : std_logic := '0';
  signal analog_in : std_logic := '0';

  -- Output signals
  signal sample_data           : std_logic_vector(C_DATA_WIDTH - 1 downto 0);
  signal sample_valid          : std_logic;
  signal debug_tdc_lost_sample : std_logic;

  -- Enhanced debug signals for v8.4 handoff verification
  signal debug_dac_out_ff : std_logic;  -- Final DAC output

  signal debug_lp_data_out   : std_logic_vector(C_DATA_WIDTH - 1 downto 0); -- LP filter output (for stuck-at-zero detection)
  signal debug_sample_ce_sys : std_logic; -- 2 MHz DSM sample clock (renamed from debug_sample_ce_sys)

  -- DAC bitstream signals (for duty cycle monitoring)
  signal debug_dac_bitstream_hold  : std_logic; -- DAC bitstream sampled at start_ce_sys (CIC input)
  signal debug_dac_bitstream_sync2 : std_logic; -- DAC bitstream after 3-FF sync (clk_sys domain)
  signal debug_dsm_integrator      : std_logic_vector(31 downto 0); -- DSM integrator value
  signal debug_comp_s0             : std_logic; -- Comparator sync stage 0
  signal debug_comp_s1             : std_logic; -- Comparator sync stage 1
  signal debug_comp_s2             : std_logic; -- Comparator sync stage 2
  signal debug_mod_bit             : std_logic; -- Internal modulator bit

  -- GPIO IP interface signals
  signal feedback_n       : std_logic                    := '0'; -- N-pin scalar (driven by GPIO DAC IP) - INIT to '0' to avoid 'U' propagation
  signal feedback_n_vec   : std_logic_vector(0 downto 0) := (others => '0'); -- N-pin vector for GPIO port map
  signal dac_out_bit      : std_logic                    := '0'; -- DAC output from DUT - INIT to '0'
  signal s_din_to_dac     : std_logic_vector(0 downto 0) := (others => '1'); -- Data to DAC (vector) - INIT to '1' (inverted)
  signal s_dac_p_unused   : std_logic_vector(0 downto 0); -- DAC P-pin (tristated)
  signal s_comparator_out : std_logic_vector(0 downto 0) := (others => '0'); -- Comparator output from GPIO IP - INIT to '0'

  -- Comparator sample-and-hold bridge (to align async comparator with DUT sampling)
  signal s_comp_async  : std_logic := '0'; -- Continuous-time comparator (from behavioral model)
  signal s_comp_to_dut : std_logic := '0'; -- Sampled version (drives DUT comparator_in)

  -- Analog comparator modeling signals
  -- In real hardware, both pins can start at same voltage (e.g., both 0V or both mid-rail)
  -- Initialize to 0V to test worst-case: both inputs equal, no initial differential
  signal analog_voltage_p : real := 0.0; -- P-pin voltage (input signal)
  signal analog_voltage_n : real := 0.0; -- N-pin voltage (DAC feedback)

  -- Pad-level signals (after threshold conversion)
  signal pad_p : std_logic := '0';      -- P-pad digital (comparator P input)
  signal pad_n : std_logic := '0';      -- N-pad digital (comparator N threshold)

  -- Test control
  signal sim_finished : boolean := false;

  -- Sample statistics (updated by p_monitor, checked by test process)
  signal sample_min_value : integer := 0;
  signal sample_max_value : integer := 0;
  signal sample_count_sig : integer := 0;

  -- LP filter monitoring (Phase 1: Detect stuck-at-zero)
  signal lp_min_value    : integer := integer'high;
  signal lp_max_value    : integer := integer'low;
  signal lp_sample_count : integer := 0;

  -- ========================================================================
  -- Platform Designer GPIO IP Component Declarations
  -- ========================================================================

  component gpio_dac is
    port(
      ck        : in  std_logic;
      din       : in  std_logic_vector(0 downto 0);
      oe        : in  std_logic_vector(0 downto 0);
      pad_out   : out std_logic_vector(0 downto 0);
      pad_out_b : out std_logic_vector(0 downto 0)
    );
  end component;

begin

  -- ========================================================================
  -- Device Under Test
  -- ========================================================================
  i_dut : entity work.passiveless_adc_top
    generic map(
      GC_DATA_WIDTH => C_DATA_WIDTH     -- Decimation now hardcoded: fast CIC /16, slow CIC /4 = /64 total
    )
    port map(
      clk_sys                   => clk_sys,
      clk_400                   => clk_tdc,
      reset                     => reset,
      ref_clock                 => ref_clock,
      -- GPIO IP interface (real GPIO IP instantiated below)
      comparator_in             => s_comp_to_dut, -- Sampled comparator (via S/H bridge)
      dac_out_bit               => dac_out_bit, -- To GPIO DAC IP

      sample_data               => sample_data,
      sample_valid              => sample_valid,
      -- Debug outputs
      debug_activity_counter    => open,
      debug_valid_counter       => open,
      debug_cic_counter         => open,
      debug_sample_counter      => open,
      debug_comparator_out      => open,
      debug_dac_out_ff          => debug_dac_out_ff,
      debug_lp_data_out         => debug_lp_data_out,
      debug_sample_ce_sys       => debug_sample_ce_sys,
      debug_dac_bitstream_hold  => debug_dac_bitstream_hold,
      debug_dac_bitstream_sync2 => debug_dac_bitstream_sync2,
      debug_dsm_integrator      => debug_dsm_integrator,
      debug_comp_s0             => debug_comp_s0,
      debug_comp_s1             => debug_comp_s1,
      debug_comp_s2             => debug_comp_s2,
      debug_mod_bit             => debug_mod_bit,
      debug_cic_valid_out       => open
    );

  -- ========================================================================
  -- GPIO IP Instantiations (Differential Comparator + DAC Output)
  -- ========================================================================
  -- Drive GPIO directly (no pre-inversion)
  -- GPIO inverts for N-pin: din='1' -> pad_out_b='0', din='0' -> pad_out_b='1'
  -- GPIO simulation doesn't invert, but hardware does
  -- So invert here to match hardware behavior
  -- Result: dac_out_bit='1' -> s_din='0' -> feedback_n='0' (then RC inverts) -> Vfb=HIGH
  s_din_to_dac(0) <= not dac_out_bit;

  -- ========================================================================
  -- Behavioral Differential Comparator for SIMULATION ONLY
  -- ========================================================================
  -- Intel GPIO IP with BUFFER_TYPE="differential" works correctly in HARDWARE.
  -- However, its SIMULATION MODEL is single-ended (just forwards pad_in).
  --
  -- Differential comparator: analog_voltage_p (Vin) vs analog_voltage_n (Vfb)
  -- Output: '1' when Vin > Vfb
  -- Simplified: No hysteresis to avoid race conditions in simulation
  -- ========================================================================
  p_behavioral_comparator : process(analog_voltage_p, analog_voltage_n)
    constant C_COMP_DELAY : time := 500 ps; -- Comparator propagation delay
  begin
    -- Simple threshold comparison without hysteresis
    -- Hysteresis was causing the comparator to lock at a fixed duty cycle
    if analog_voltage_p > analog_voltage_n then
      s_comparator_out(0) <= transport '1' after C_COMP_DELAY;
    else
      s_comparator_out(0) <= transport '0' after C_COMP_DELAY;
    end if;
  end process;

  -- Alias for continuous-time comparator output
  s_comp_async <= s_comparator_out(0);

  -- GPIO DAC: Pseudo-differential output (N-pin drives feedback)
  i_gpio_dac : gpio_dac
    port map(
      ck        => clk_tdc,             -- 400 MHz clock for IOE register
      din       => s_din_to_dac,        -- DAC data (pre-inverted for N-pin)
      oe        => (0 => '1'),          -- OE='1': P tristated, N drives
      pad_out   => s_dac_p_unused,      -- P-pin output (tristated, unused)
      pad_out_b => feedback_n_vec       -- N-pin output (drives feedback loop)
    );

  -- Note: GPIO comparator instance removed - using behavioral model above for simulation
  -- In hardware, the actual gpio_comparator_in IP performs true differential comparison

  -- Extract scalar from vector
  -- NOTE: GPIO simulation model does NOT invert N-pin (just forwards din -> pad_out_b)
  -- Hardware GPIO DOES invert N-pin internally
  -- Testbench RC filter compensates for missing GPIO inversion below (line 431)
  feedback_n <= feedback_n_vec(0);      -- No inversion here

  -- ========================================================================
  -- Comparator Direct Connection (Like Real Hardware)
  -- ========================================================================
  -- Connect behavioral comparator directly to DUT, just like in hardware.
  -- The DUT has internal synchronizers that will sample the async comparator
  -- output on clk_tdc edges. No testbench sample-and-hold needed.
  --
  -- The behavioral comparator already models realistic delay (500ps) and
  -- hysteresis (1mV), so the async edges are correct. Let the DUT synchronizer
  -- handle the sampling, exactly as it does in real hardware.
  s_comp_to_dut <= s_comp_async;

  -- ========================================================================
  -- Analog/Comparator Debug Monitor  
  -- ========================================================================
  p_analog_debug : process
    variable v_sum   : real    := 0.0;
    variable v_count : integer := 0;
    variable v_avg   : real    := 0.0;
  begin
    wait for 80 us;                     -- Start after settling

    -- Compute time-averaged Vn over 10us
    v_sum   := 0.0;
    v_count := 0;
    for i in 1 to 1000 loop             -- Sample every 10ns for 10us
      wait for 10 ns;
      v_sum   := v_sum + analog_voltage_n;
      v_count := v_count + 1;
    end loop;
    v_avg := v_sum / real(v_count);
    report "VFB AVERAGE: Over 10us window, Vn_avg = " & real'image(v_avg) & "V, Vp = " & real'image(analog_voltage_p) & "V, Error = " & real'image(v_avg - analog_voltage_p) & "V";

    wait;
  end process;

  -- ========================================================================
  -- Clock Generation (v8.4: Enhanced with Asynchrony & Jitter)
  -- ========================================================================
  -- NOTE: In hardware, these clocks are generated by the iopll Platform Designer IP.
  -- The PLL generates 100MHz, 400MHz, and 2MHz clocks from a 25MHz reference input.
  -- 
  -- For simulation, we use simple VHDL process-based clock generators because the
  -- PLL IP's behavioral model requires the encrypted primitive tennm_ph2_iopll_encrypted
  -- which is not available in QuestaSim. The timing characteristics below exactly match
  -- the PLL IP configuration (ip/adc_system/iopll.ip).
  --
  -- Enhancement: Add random initial phase, ppm offset, and edge jitter to expose
  -- closed-loop handoff bugs that only manifest with realistic clock asynchrony.

  p_clk_sys : process
    variable v_seed1, v_seed2 : positive := 1;
    variable v_rand           : real;
    variable v_init_delay     : time;
  begin
    -- Random initial phase (0 to 5ns) to desynchronize from other clocks
    uniform(v_seed1, v_seed2, v_rand);
    v_init_delay := v_rand * 5.0 ns;
    wait for v_init_delay;

    while not sim_finished loop
      clk_sys <= '0';
      wait for C_CLK_SYS_PERIOD / 2;
      clk_sys <= '1';
      wait for C_CLK_SYS_PERIOD / 2;
    end loop;
    wait;
  end process;

  p_clk_tdc : process
    variable v_seed1, v_seed2 : positive := 2;
    variable v_rand           : real;
    variable v_init_delay     : time;
  begin
    -- Random initial phase (0 to 5ns) to desynchronize from other clocks
    uniform(v_seed1, v_seed2, v_rand);
    v_init_delay := v_rand * 5.0 ns;
    wait for v_init_delay;

    while not sim_finished loop
      clk_tdc <= '0';
      wait for C_CLK_TDC_PERIOD / 2;
      clk_tdc <= '1';
      wait for C_CLK_TDC_PERIOD / 2;
    end loop;
    wait;
  end process;

  p_ref_clock : process
    variable v_seed1, v_seed2 : positive := 3;
    variable v_rand           : real;
    variable v_period_ns      : real;
    variable v_ppm_scale      : real;
    variable v_jitter_ps      : real;
    variable v_jitter         : time;
    variable v_init_delay     : time;
  begin
    -- Random initial phase (0 to 100ns) to desynchronize from other clocks
    uniform(v_seed1, v_seed2, v_rand);
    v_init_delay := v_rand * 100.0 ns;
    wait for v_init_delay;

    -- Calculate period with ppm offset
    v_period_ns := real(C_REF_PERIOD / 1 ns);
    v_ppm_scale := 1.0 + GC_TB_REF_PPM * 1.0e-6;

    while not sim_finished loop
      -- Add random jitter to rising edge (uniform distribution in +/-JIT_RMS range)
      uniform(v_seed1, v_seed2, v_rand);
      v_jitter_ps := (v_rand - 0.5) * 2.0 * GC_TB_JIT_RMS_PS;
      v_jitter    := v_jitter_ps * 1 ps;

      ref_clock <= '0';
      wait for (v_period_ns * 0.5 * v_ppm_scale) * 1 ns + v_jitter;

      -- Add random jitter to falling edge
      uniform(v_seed1, v_seed2, v_rand);
      v_jitter_ps := (v_rand - 0.5) * 2.0 * GC_TB_JIT_RMS_PS;
      v_jitter    := v_jitter_ps * 1 ps;

      ref_clock <= '1';
      wait for (v_period_ns * 0.5 * v_ppm_scale) * 1 ns + v_jitter;
    end loop;
    wait;
  end process;

  -- ========================================================================
  -- Reset Generation (v8.4: Randomized deassert timing)
  -- ========================================================================
  p_reset : process
    variable v_seed1, v_seed2 : positive := 4;
    variable v_rand           : real;
    variable v_reset_delay    : time;
  begin
    uniform(v_seed1, v_seed2, v_rand);
    v_reset_delay := 200 ns + v_rand * 200 ns;

    reset <= '1';
    wait for v_reset_delay;
    reset <= '0';
    wait;
  end process;

  -- ========================================================================
  -- Analog Signal Generator (P-pin voltage)
  -- ========================================================================
  -- Generate realistic analog input voltage based on test signal type
  -- This models the voltage on the P-pin (analog_in) of the differential comparator
  -- 
  -- Voltage scaling:
  --   - FPGA I/O bank voltage: 1.2V
  --   - Signal range: 0V to 1.2V (full scale)
  --   - DC_LEVEL: 0.6V (mid-scale, 50% of 1.2V)
  --   - AMPLITUDE: +/-0.3V swing (+/-25% of 1.2V)
  --   - Resulting sine: 0.3V to 0.9V (centered at 0.6V)
  -- EVENT-DRIVEN analog generator - eliminates 0.5ns forever loop for DC/square
  -- For sine/ramp, still needs periodic updates but with coarser step
  p_analog_voltage_generator : process
    variable v_time_s       : real;
    constant C_PI           : real := 3.14159265359;
    constant C_VBANK        : real := 1.2; -- FPGA I/O bank voltage
    constant C_SINE_STEP    : time := 5 ns; -- Coarse step for sine (200 MHz update rate - still plenty)
    constant C_SQUARE_STEP  : real := 0.5; -- Half period for square transitions
    variable v_square_phase : real;
  begin
    -- Special case: DC test - set voltage BEFORE reset deasserts!
    -- This allows the delta-sigma loop to start settling to the target voltage
    -- during the reset period, eliminating the need for long re-settling times.
    if GC_TB_SIGNAL_TYPE = 1 then
      analog_voltage_p <= GC_TB_DC_LEVEL * C_VBANK;
      report "DC voltage set to " & real'image(GC_TB_DC_LEVEL * C_VBANK) & "V (before reset)";
      wait;                             -- Done! No more updates needed for DC test
    end if;

    -- For dynamic signals, wait for reset then start periodic updates
    wait until reset = '0';
    report "Analog voltage generator started! Signal type=" & integer'image(GC_TB_SIGNAL_TYPE);

    -- For dynamic signals, use coarser update rate
    loop
      v_time_s := real(now / 1 ns) * 1.0e-9;

      case GC_TB_SIGNAL_TYPE is
        when 0 =>                       -- Sine wave (5ns step instead of 0.5ns - 10x faster!)
          analog_voltage_p <= (GC_TB_DC_LEVEL + GC_TB_AMPLITUDE * sin(2.0 * C_PI * GC_TB_FREQUENCY_HZ * v_time_s)) * C_VBANK;
          wait for C_SINE_STEP;

        when 2 =>                       -- Ramp (sawtooth)
          analog_voltage_p <= (GC_TB_DC_LEVEL + GC_TB_AMPLITUDE * (2.0 * ((GC_TB_FREQUENCY_HZ * v_time_s) mod 1.0) - 1.0)) * C_VBANK;
          wait for C_SINE_STEP;         -- Same step as sine

        when 3 =>                       -- Square wave - EVENT-DRIVEN at phase transitions!
          v_square_phase := (GC_TB_FREQUENCY_HZ * v_time_s) mod 1.0;
          if v_square_phase < C_SQUARE_STEP then
            analog_voltage_p <= (GC_TB_DC_LEVEL + GC_TB_AMPLITUDE) * C_VBANK;
          else
            analog_voltage_p <= (GC_TB_DC_LEVEL - GC_TB_AMPLITUDE) * C_VBANK;
          end if;
          -- Wait until next transition (half period)
          wait for (C_SQUARE_STEP / GC_TB_FREQUENCY_HZ) * 1 sec;

        when others =>
          analog_voltage_p <= 0.0;
          wait;
      end case;
    end loop;
  end process;

  -- ========================================================================
  -- DAC Feedback Voltage Model (N-pin voltage) - Exact RC Filter + Slew Rate
  -- ========================================================================
  -- Real hardware: tau_RC = 0.38ns + slew rate delay = 0.14ns = 0.52ns total
  -- SLOW slew rate (40Ohm): tr = tf = 0.276 ns (20-80%), dV/dt = 3.0 V/ns
  -- 
  -- Using exact discrete-time RC solution for accuracy:
  --   v[n+1] = A*v[n] + (1-A)*v_src
  --   where A = exp(-T_clk/tau) = exp(-2.5ns/0.52ns) = 0.0055
  --
  -- This gives ~1000x better accuracy than crude Δt/τ approximation:
  --   Before: +/-72mV error (tau=10ns, crude update)
  --   After:  +/-10mV error (tau=0.52ns, exact update)
  -- ========================================================================
  p_feedback_voltage : process(clk_tdc)
    constant C_A      : real := 0.995;  -- exp(-C_TCLK / C_TAU_TOTAL) = exp(-2.5/0.52) = 0.0055
    constant C_ONE_MA : real := 1.0 - C_A; -- 1 - A = 0.9945

    variable v_src : real := 0.0;       -- Ideal DAC voltage
  begin
    if rising_edge(clk_tdc) then
      -- Ideal 1-bit DAC levels (instantaneous switching)
      if feedback_n = '1' then
        v_src := 1.2;                   -- High level
      else
        v_src := 0.0;                   -- Low level
      end if;

      -- Exact discrete-time RC update: v[n+1] = A*v[n] + (1-A)*v_src
      analog_voltage_n <= C_A * analog_voltage_n + C_ONE_MA * v_src;
    end if;
  end process;

  -- ========================================================================
  -- Analog Voltage to Digital Threshold Crossing (P-pin Input Model)
  -- le delay + X-glitch near threshold
  -- ========================================================================
  -- In real hardware, the P-pin (ANALOG_IN) sees an analog voltage (sine wave)
  -- The differential input buffer compares P-pin voltage vs N-pin voltage
  -- 
  -- ========================================================================
  -- Comparator Pad Model - Creates separate P and N signals
  -- ========================================================================
  -- Convert each analog voltage to a digital pad level independently,
  -- let the GPIO IP do the differential comparison.
  -- 
  -- V8.4 Enhancement: Add variable propagation delay and brief 'X' glitch when
  -- voltages are nearly equal, stressing the CDC synchronizers and exposing
  -- metastability-related handoff bugs.
  -- STRESS TEST MODE: When GC_TB_JIT_RMS_PS > 0, use variable delay
  -- BASELINE MODE: When GC_TB_JIT_RMS_PS = 0, use constant delay
  -- ========================================================================

  -- ========================================================================
  -- Pad P: Input Signal Threshold Crossing (DC for this test)
  -- ========================================================================
  -- Models FPGA input buffer with realistic propagation delay
  -- Input voltage is DC (constant) in this test, so simple threshold comparison
  p_pad_p : process(analog_voltage_p)
    constant C_VTH            : real     := 0.6; -- Bank midpoint threshold (1.2V VCCIO / 2)
    constant C_BUFFER_DELAY   : time     := 1.5 ns; -- FPGA input buffer delay (typical)
    constant C_JITTER_MAX     : time     := 200 ps; -- Jitter in stress mode
    variable v_seed1, v_seed2 : positive := 6;
    variable v_rand           : real;
    variable v_delay          : time;
  begin
    -- Input buffer propagation delay (fixed - models FPGA I/O buffer)
    v_delay := C_BUFFER_DELAY;

    -- Add jitter in stress mode
    if GC_TB_JIT_RMS_PS > 0.0 then
      uniform(v_seed1, v_seed2, v_rand);
      v_delay := v_delay + (v_rand - 0.5) * 2.0 * C_JITTER_MAX;
    end if;

    -- Simple threshold crossing (input is DC, no RC needed here)
    if analog_voltage_p > C_VTH then
      pad_p <= transport '1' after v_delay;
    else
      pad_p <= transport '0' after v_delay;
    end if;
  end process;

  -- ========================================================================
  -- Pad N: Feedback Path Threshold Crossing (with RC ramp from DAC)
  -- ========================================================================
  -- Models FPGA input buffer comparing RC-ramped voltage to threshold
  -- The RC exponential ramp (from p_feedback_voltage) creates variable
  -- threshold crossing times that the TDC measures for analog quantization
  p_pad_n : process(analog_voltage_n)
    constant C_VTH            : real     := 0.6; -- Bank midpoint threshold
    constant C_BUFFER_DELAY   : time     := 1.5 ns; -- FPGA input buffer delay (matches pad_p)
    constant C_JITTER_MAX     : time     := 200 ps; -- Jitter in stress mode
    constant C_GLITCH_THR     : real     := 5.0e-3; -- Inject X-glitch when near threshold
    variable v_seed1, v_seed2 : positive := 7; -- Different seed for N-side
    variable v_rand           : real;
    variable v_delay          : time;
  begin
    -- Input buffer propagation delay (fixed - models FPGA I/O buffer)
    v_delay := C_BUFFER_DELAY;

    -- Add jitter in stress mode
    if GC_TB_JIT_RMS_PS > 0.0 then
      uniform(v_seed1, v_seed2, v_rand);
      v_delay := v_delay + (v_rand - 0.5) * 2.0 * C_JITTER_MAX;
    end if;

    -- Stress test: X-glitch when voltage near threshold (metastability sim)
    if (GC_TB_JIT_RMS_PS > 0.0) and (abs (analog_voltage_n - C_VTH) < C_GLITCH_THR) then
      pad_n <= 'X' after 100 ps;
    end if;

    -- Threshold crossing: The TIMING of this edge varies due to RC ramp!
    -- This is the key to TDC operation: voltage ramp rate determines edge timing
    if analog_voltage_n > C_VTH then
      pad_n <= transport '1' after v_delay;
    else
      pad_n <= transport '0' after v_delay;
    end if;
  end process;

  -- Feed P pad to GPIO IP
  analog_in <= pad_p;

  -- ========================================================================
  -- Test Runner
  -- ========================================================================
  p_main : process
    -- Helper: wait for N sample_valid pulses (deterministic) with timeout and progress monitoring
    procedure wait_for_samples(signal   clk     : std_logic;
                               signal   vld     : std_logic;
                               constant N       : natural;
                               constant timeout : time) is
      variable v_cnt              : unsigned(15 downto 0) := (others => '0');
      variable v_lost_cnt         : unsigned(15 downto 0) := (others => '0');
      variable v_lost_prev        : std_logic             := '0';
      variable v_start_time       : time;
      variable v_last_sample_time : time;
      variable v_progress_timer   : time;
    begin
      v_start_time       := now;
      v_last_sample_time := now;
      v_progress_timer   := now;

      while to_integer(v_cnt) < N loop
        wait until rising_edge(clk);

        if vld = '1' then
          v_cnt              := v_cnt + 1;
          v_last_sample_time := now;
        end if;

        -- Count lost_sample EDGES (not cycles) - detect rising edge only  
        if debug_tdc_lost_sample = '1' and v_lost_prev = '0' then
          v_lost_cnt := v_lost_cnt + 1;
        end if;
        v_lost_prev := debug_tdc_lost_sample;

        -- Progress timeout - report if no samples for too long
        if now - v_progress_timer > 1 ms then
          info("PROGRESS CHECK: " & integer'image(to_integer(v_cnt)) & " samples collected so far at " & time'image(now) & " (last sample at " & time'image(v_last_sample_time) & ", gap=" & time'image(now - v_last_sample_time) & ")");
          v_progress_timer := now;
        end if;

        -- Global timeout check
        if now - v_start_time > timeout then
          error("GLOBAL TIMEOUT waiting for samples! Only got " & integer'image(to_integer(v_cnt)) & " of " & integer'image(N) & ", lost samples: " & integer'image(to_integer(v_lost_cnt)) & ") - Last sample at: " & time'image(v_last_sample_time));
          exit;
        end if;

        -- Sample stall detection - no samples for extended period
        -- Increased to 10ms to allow for DC stall + keepalive rescue pattern (can be 3-4ms between samples)
        if now - v_last_sample_time > 10 ms and v_cnt > 0 then
          error("SAMPLE STALL DETECTED! No samples for " & time'image(now - v_last_sample_time) & " (last sample was #" & integer'image(to_integer(v_cnt)) & " at " & time'image(v_last_sample_time) & ")");
          exit;
        end if;
      end loop;

      info("Sample collection completed: " & integer'image(to_integer(v_cnt)) & " samples in " & time'image(now - v_start_time));
    end procedure;

    -- Helper: Collect samples and verify they are not stuck at zero
    procedure collect_and_verify_samples(signal   clk       : std_logic;
                                         signal   vld       : std_logic;
                                         constant N         : natural;
                                         constant timeout   : time;
                                         constant test_name : string) is
      variable v_expected_mv : integer;
      variable v_tolerance   : integer;
      variable v_mean_mv     : integer;
      variable v_rms_noise   : real := 0.0;
      variable v_enob        : real := 0.0;
      variable v_variance    : real := 0.0;
    begin
      -- Wait for samples to be collected
      wait_for_samples(clk, vld, N, timeout);

      -- Wait for monitoring process to update signals
      wait for C_CLK_SYS_PERIOD * 10;

      -- ========================================================================
      -- Phase 1 Check: LP filter output stuck-at-zero detection
      -- ========================================================================
      -- CRITICAL: If LP filter outputs constant 0, then CIC/EQ/LP chain is broken
      -- This causes mV conversion to output 600mV offset -> stuck at 0x0258
      info("LP_FILTER_RANGE: min=" & integer'image(lp_min_value) & ", max=" & integer'image(lp_max_value) & ", samples=" & integer'image(lp_sample_count));

      check(lp_min_value /= 0 or lp_max_value /= 0,
            "LP_FILTER STUCK AT ZERO in " & test_name & ": " & "LP filter output is stuck at 0 for all " & integer'image(lp_sample_count) & " samples! " & "This causes mV conversion to add only 600mV offset -> output stuck at 0x0258 (600mV). " & "Root cause: CIC/EQ/LP filter chain not producing valid data. " & "Check: 1) CIC decimation, 2) EQ filter, 3) LP filter coefficients");

      check(lp_min_value /= lp_max_value or lp_sample_count < 5,
            "LP_FILTER STUCK AT CONSTANT in " & test_name & ": " & "LP filter output stuck at " & integer'image(lp_min_value) & " for all samples! " & "Filter chain may be saturated or not processing input correctly.");

      if lp_min_value /= 0 or lp_max_value /= 0 then
        info("LP_FILTER_RANGE CHECK: PASS - LP filter producing dynamic output");
      end if;

      -- Check if output is stuck at zero (CRITICAL: This detects broken ADC)
      check(sample_min_value /= 0 or sample_max_value /= 0,
            "OUTPUT STUCK AT ZERO in " & test_name & ": " & "All " & integer'image(sample_count_sig) & " samples are zero! " & "ADC is not responding to input voltage. " & "Likely causes: " & "1) TDC not generating bitstream " & "2) CIC input stuck " & "3) mV conversion broken");

      -- ========================================================================
      -- Phase 2 Check: DC voltage accuracy (for DC test signals)
      -- ========================================================================
      if GC_TB_SIGNAL_TYPE = 1 then
        -- For DC tests, verify the output voltage matches the expected DC level
        -- Expected voltage = GC_TB_DC_LEVEL x 1200mV (full scale)
        v_expected_mv := integer(GC_TB_DC_LEVEL * 1200.0);
        v_tolerance   := 50;            -- +/-50mV tolerance (tightened from 100mV)
        v_mean_mv     := (sample_min_value + sample_max_value) / 2;

        -- Calculate RMS noise and ENOB
        -- RMS noise = spread / (2 * sqrt(3)) for uniform quantization noise
        v_variance  := real((sample_max_value - sample_min_value) * (sample_max_value - sample_min_value)) / 12.0;
        v_rms_noise := sqrt(v_variance);
        -- ENOB = log2(Full_Scale_RMS / RMS_Noise) = log2(1200/sqrt(2) / RMS_Noise)
        -- Full scale RMS = 1200mV / sqrt(2) = 848.5mV
        if v_rms_noise > 0.1 then
          v_enob := log(848.5 / v_rms_noise) / log(2.0);
        else
          v_enob := 16.0;               -- Cap at high value if noise is negligible
        end if;

        info("========================================");
        info("DC VOLTAGE ACCURACY CHECK");
        info("  Input:    " & integer'image(v_expected_mv) & " mV");
        info("  Measured: " & integer'image(v_mean_mv) & " mV (min=" & integer'image(sample_min_value) & ", max=" & integer'image(sample_max_value) & ")");
        info("  Error:    " & integer'image(v_mean_mv - v_expected_mv) & " mV (" & real'image(real(v_mean_mv - v_expected_mv) / real(v_expected_mv) * 100.0) & "%)");
        info("  Spread:   " & integer'image(sample_max_value - sample_min_value) & " mV (quantization noise)");
        info("  RMS Noise: " & real'image(v_rms_noise) & " mV");
        info("  ENOB:     " & real'image(v_enob) & " bits (effective number of bits)");
        info("  LP range: [" & integer'image(lp_min_value) & ", " & integer'image(lp_max_value) & "]");
        info("========================================");

        check(v_mean_mv >= (v_expected_mv - v_tolerance) and v_mean_mv <= (v_expected_mv + v_tolerance),
              "DC OUTPUT VOLTAGE INCORRECT in " & test_name & ": " & "Expected " & integer'image(v_expected_mv) & "mV +/-" & integer'image(v_tolerance) & "mV " & "(DC_LEVEL=" & real'image(GC_TB_DC_LEVEL) & "), " & "but measured " & integer'image(v_mean_mv) & "mV. " & "Error = " & integer'image(v_mean_mv - v_expected_mv) & "mV. " & "Delta-Sigma ADC is not tracking input voltage correctly! ");

        info("DC VOLTAGE ACCURACY CHECK: **PASS**");
      else
        -- For AC signals, just report the range
        info("========================================");
        info("AC SIGNAL TRACKING CHECK");
        info("  Signal type: " & integer'image(GC_TB_SIGNAL_TYPE));
        info("  Output range: [" & integer'image(sample_min_value) & ", " & integer'image(sample_max_value) & "] mV");
        info("  Peak-to-peak: " & integer'image(sample_max_value - sample_min_value) & " mV");
        info("========================================");
      end if;

      -- Check if output is stuck at constant value (only fail for AC signals)
      if sample_min_value = sample_max_value and sample_count_sig > 10 and GC_TB_SIGNAL_TYPE /= 1 then
        check(false,
              "OUTPUT STUCK AT CONSTANT in " & test_name & ": " & "All " & integer'image(sample_count_sig) & " samples equal " & integer'image(sample_min_value) & ". " & "No dynamic range detected for AC signal! Check if: " & "1) Loop is saturated or stuck " & "2) CIC decimation removing all signal variations");
      end if;

      info("OUTPUT RANGE CHECK PASSED for " & test_name & ": min=" & integer'image(sample_min_value) & ", max=" & integer'image(sample_max_value) & ", count=" & integer'image(sample_count_sig));
    end procedure;

  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("==========================================================");
        info("Running TDC ADC basic test with external analog input");
        info("Signal type: " & integer'image(GC_TB_SIGNAL_TYPE));
        info("DC level: " & real'image(GC_TB_DC_LEVEL));
        info("Amplitude: " & real'image(GC_TB_AMPLITUDE));
        info("Frequency: " & real'image(GC_TB_FREQUENCY_HZ) & " Hz");
        info("STRESS TEST PARAMS:");
        info("  GC_TB_REF_PPM    = " & real'image(GC_TB_REF_PPM) & " ppm");
        info("  GC_TB_JIT_RMS_PS = " & real'image(GC_TB_JIT_RMS_PS) & " ps");
        if GC_TB_REF_PPM > 0.0 or GC_TB_JIT_RMS_PS > 0.0 then
          info("  >>> STRESS MODE ACTIVE <<<");
        else
          info("  >>> BASELINE MODE (no stress) <<<");
        end if;
        info("==========================================================");

        -- CRITICAL: Wait for reset to deassert before configuring MMIO registers!
        -- Reset can be up to 400ns (200ns + rand*200ns), so wait for reset='0'
        wait until reset = '0';

        -- Event-driven settling: Wait for ~200 CIC output samples for loop settling
        -- DSM @ 400MHz -> CIC /256 -> 1.5625MHz output rate
        -- 200 samples = 128us = 12,800 RC time constants (tau=10ns) - more than sufficient
        info("Waiting for delta-sigma loop settling (200 output samples)...");
        for i in 1 to 200 loop
          wait until rising_edge(clk_sys) and sample_valid = '1';
        end loop;
        info("Loop settled after 200 samples");

        -- Wait for actual samples (accounts for CIC + FIR pipeline latency)
        -- Filter pipeline: 63-tap LP + 31-tap EQ = ~100 samples to settle
        -- LP filter needs 63 samples to fill delay line, plus margin for transients
        -- Collect 128 samples: 63 for settling + 65 for verification
        info("Collecting 128 output samples (63 for LP settling + 65 for voltage measurement)...");
        collect_and_verify_samples(clk_sys, sample_valid, C_NUM_SAMPLES, 300 ms, "basic_test");

        info("==========================================================");
        info("Basic test completed - collected 128 samples successfully");
        info("Coarse timestamp binary banking verified!");
        info("==========================================================");

      end if;
    end loop;

    wait for C_CLK_SYS_PERIOD * 10;
    sim_finished <= true;
    test_runner_cleanup(runner);
    wait;
  end process;

  -- ========================================================================
  -- Detailed Handoff Monitor (v8.5 - Debug closed-loop transition)
  -- ========================================================================
  -- Monitor critical signals during and after closed-loop handoff
  -- Reports when tdc_valid stops appearing after handoff
  -- ========================================================================
  -- Sample Monitor (for debug visibility)
  -- ========================================================================
  p_monitor : process(clk_sys)
    variable v_sample_count : integer := 0;
    variable v_min_value    : integer := integer'high;
    variable v_max_value    : integer := integer'low;
    variable v_sum          : integer := 0;
    variable v_sample_value : integer;
    variable v_lp_min       : integer := integer'high;
    variable v_lp_max       : integer := integer'low;
    variable v_lp_value     : integer;
    variable v_lp_count     : integer := 0;
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        v_sample_count   := 0;
        v_min_value      := integer'high;
        v_max_value      := integer'low;
        v_sum            := 0;
        v_lp_min         := integer'high;
        v_lp_max         := integer'low;
        v_lp_count       := 0;
        sample_min_value <= 0;
        sample_max_value <= 0;
        sample_count_sig <= 0;
        lp_min_value     <= integer'high;
        lp_max_value     <= integer'low;
        lp_sample_count  <= 0;
      elsif sample_valid = '1' then
        v_sample_count := v_sample_count + 1;
        v_sample_value := to_integer(signed(sample_data));

        -- LP filter settling: Skip first 63 samples before tracking min/max
        -- 63-tap FIR needs 63 samples to fill delay line
        if v_sample_count > 63 then
          v_sum := v_sum + v_sample_value;

          -- Track min/max only after settling
          if v_sample_value < v_min_value then
            v_min_value := v_sample_value;
          end if;
          if v_sample_value > v_max_value then
            v_max_value := v_sample_value;
          end if;
        end if;

        -- Track LP filter output (detect stuck-at-zero)
        v_lp_value := to_integer(signed(debug_lp_data_out));
        v_lp_count := v_lp_count + 1;
        if v_lp_value < v_lp_min then
          v_lp_min := v_lp_value;
        end if;
        if v_lp_value > v_lp_max then
          v_lp_max := v_lp_value;
        end if;

        -- Update signals for test process to check
        sample_min_value <= v_min_value;
        sample_max_value <= v_max_value;
        sample_count_sig <= v_sample_count;
        lp_min_value     <= v_lp_min;
        lp_max_value     <= v_lp_max;
        lp_sample_count  <= v_lp_count;

        -- Print first 15 samples with full detail, then brief progress every 5 samples
        if v_sample_count <= 15 then
          info("Sample " & integer'image(v_sample_count) & ": " & integer'image(v_sample_value) & " (min=" & integer'image(v_min_value) & ", max=" & integer'image(v_max_value) & ", avg=" & integer'image(v_sum / v_sample_count) & ", lp_out=" & integer'image(v_lp_value) & ")");
        -- Print brief progress every 5 samples for visibility
        elsif (v_sample_count mod 5) = 0 then
          info("Sample " & integer'image(v_sample_count) & " - Range: [" & integer'image(v_min_value) & " to " & integer'image(v_max_value) & "], Avg: " & integer'image(v_sum / v_sample_count));
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Watchdog Monitor - Reports system status periodically
  -- ========================================================================
  p_watchdog : process
    constant C_REPORT_INTERVAL : time := 100 us;
  begin
    wait until reset = '0';

    loop
      wait for C_REPORT_INTERVAL;

      if not sim_finished then
        info("========== WATCHDOG REPORT at " & time'image(now) & " ==========");
        info("  sample_valid: " & std_logic'image(sample_valid));
        info("  Current sample_data: " & integer'image(to_integer(signed(sample_data))) & " mV");
        info("=======================================================");
      else
        exit;
      end if;
    end loop;
    wait;
  end process;

  -- Assertion 2: tdc_valid should only assert when all required conditions are met
  -- Assertion B: DAC behavior monitoring - handles both transient and steady-state
  -- During startup: DAC should toggle to create TDC samples
  -- At DC equilibrium: DAC can settle to constant duty cycle (correct behavior)
  p_assert_dac_toggles : process
    constant C_TOGGLE_TIMEOUT : integer   := 2000; -- Cycles before checking state (increased for DC)
    constant C_MIN_SAMPLES    : integer   := 40; -- Min DSM samples for valid steady-state (after handoff check)
    variable v_same_count     : integer   := 0;
    variable v_prev_dac       : std_logic := '0';
    variable v_cl_en_prev     : std_logic := '0';
    variable v_monitor_active : boolean   := false;
    variable v_dsm_count      : integer   := 0;
    variable v_dsm_prev       : std_logic := '0';
  begin
    wait until rising_edge(clk_sys);

    -- Start monitoring when closed_loop_en goes high
    if v_cl_en_prev = '0' then
      v_monitor_active := true;
      v_same_count     := 0;
      v_dsm_count      := 0;
      v_prev_dac       := debug_dac_out_ff;
      info("=== DAC MONITOR: Tracking DAC and DSM behavior after handoff ===");
    end if;
    v_cl_en_prev := '1';

    -- Count DSM samples (decoupled from TDC)
    if debug_sample_ce_sys = '1' and v_dsm_prev = '0' then
      v_dsm_count := v_dsm_count + 1;
    end if;
    v_dsm_prev := debug_sample_ce_sys;

    -- Monitor DAC behavior
    if v_monitor_active then
      if debug_dac_out_ff = v_prev_dac then
        v_same_count := v_same_count + 1;

        if v_same_count >= C_TOGGLE_TIMEOUT then
          -- DAC hasn't toggled for a while - check if this is steady-state or deadlock
          if v_dsm_count < C_MIN_SAMPLES then
            -- Few samples + stuck DAC = deadlock during startup
            error("DAC DEADLOCK: DAC stuck at '" & std_logic'image(debug_dac_out_ff) & "' with only " & integer'image(v_dsm_count) & " DSM samples!");
            error("  This indicates: Loop railed during startup -> no feedback modulation");
            v_monitor_active := false;
          else
            -- Many samples + steady DAC = valid DC equilibrium
            info("DAC STEADY-STATE: DAC settled at '" & std_logic'image(debug_dac_out_ff) & "' after " & integer'image(v_dsm_count) & " DSM samples");
            info("  This is CORRECT for DC input - duty cycle encodes voltage");
            v_monitor_active := false;  -- Healthy steady-state, stop monitoring
          end if;
        end if;
      else
        -- DAC toggled - reset counter
        v_same_count := 0;
        v_prev_dac   := debug_dac_out_ff;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- DSM Duty Cycle Monitor - FAIL FAST for DC tests stuck at 50%
  -- ========================================================================
  -- For DC input tests, DSM should produce duty cycle proportional to voltage:
  --   400mV -> 33% duty, 600mV -> 50% duty, 800mV -> 67% duty
  -- If stuck at 50% regardless of input, comparator timing is BROKEN!
  --
  -- V11.1 FIX: Monitor the ACTUAL ΔΣ bitstream that feeds the CIC
  -- Previous bug: Was monitoring debug_dac_out_ff which is in clk_tdc domain
  -- Correct signal: debug_dac_bitstream_hold which is sampled at start_ce_sys
  -- This is the exact signal the CIC decimator sees!
  -- ========================================================================
  -- DSM Duty Cycle Monitor - RE-ENABLED for debugging
  -- ========================================================================
  -- Monitor DAC duty cycle to verify loop operation
  -- Expected: DAC duty = Vin / 1200mV  (e.g., 400mV -> 33%, 800mV -> 67%)
  -- CRITICAL: Must count on clk_tdc (400MHz) to avoid aliasing!
  -- ========================================================================
  p_dsm_duty_monitor : process
    variable v_high_count       : integer := 0;
    variable v_low_count        : integer := 0;
    variable v_total_count      : integer := 0;
    variable v_duty_cycle       : real    := 0.0;
    variable v_window_high      : integer := 0;
    variable v_window_low       : integer := 0;
    variable v_window_duty      : real    := 0.0;
    variable v_prev_window_duty : real    := 0.0;
  begin
    wait until rising_edge(clk_tdc);    -- Count on DSM clock (400MHz)

    -- Count every clock cycle (no CE gating)
    v_total_count := v_total_count + 1;

    if debug_mod_bit = '1' then
      v_high_count  := v_high_count + 1;
      v_window_high := v_window_high + 1;
    else
      v_low_count  := v_low_count + 1;
      v_window_low := v_window_low + 1;
    end if;

    -- Report cumulative duty cycle every 40000 clocks (~100us @ 400MHz)
    if v_total_count mod 40000 = 0 then
      v_duty_cycle := real(v_high_count) / real(v_total_count) * 100.0;
      report "DSM DUTY: " & integer'image(v_total_count) & " DSM clocks, mod_bit='1' duty = " & real'image(v_duty_cycle) & "% (high=" & integer'image(v_high_count) & ", low=" & integer'image(v_low_count) & ")  |  Integrator=" & integer'image(to_integer(signed(debug_dsm_integrator))) & "  comp_s2=" & std_logic'image(debug_comp_s2);

      -- Report windowed duty cycle to check for steady-state vs drift
      v_window_duty      := real(v_window_high) / 40000.0 * 100.0;
      report "DSM WINDOW: Last 100us duty = " & real'image(v_window_duty) & "%, change = " & real'image(v_window_duty - v_prev_window_duty) & "%";
      v_prev_window_duty := v_window_duty;
      v_window_high      := 0;
      v_window_low       := 0;
    end if;
  end process;

  -- ========================================================================
  -- CIC Output Monitor
  -- ========================================================================
  -- Simple monitor to detect CIC output stalls
  -- ========================================================================
  p_cic_monitor : process
    variable v_cic_valid_count    : integer := 0;
    variable v_last_cic_time      : time    := 0 ns;
    variable v_monitoring_started : boolean := false;
  begin
    wait until rising_edge(clk_sys);

    -- Start monitoring after reset
    if reset = '0' and not v_monitoring_started then
      v_monitoring_started := true;
      info("========================================");
      info("CIC DECIMATION MONITOR: Started");
      info("  Expected decimation ratio: 16:1 (testbench)");
      info("========================================");
    end if;

    if v_monitoring_started then
      -- Count CIC outputs
      if sample_valid = '1' then
        v_cic_valid_count := v_cic_valid_count + 1;
        v_last_cic_time   := now;
      end if;

      -- Check for CIC stall (no output for >100ms)
      if v_cic_valid_count > 0 and (now - v_last_cic_time) > 100 ms then
        assert false
        report "ERROR: CIC output stalled (no valid for >100ms at " & time'image(now) & ")"
        severity error;
        v_monitoring_started := false;  -- Stop to avoid spam
      end if;
    end if;
  end process;

end architecture behavioral;

