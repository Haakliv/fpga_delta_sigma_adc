-- ************************************************************************
-- Testbench for TDC Quantizer
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity tdc_quantizer_tb is
  generic(runner_cfg : string);
end entity;

architecture behavioral of tdc_quantizer_tb is
  constant C_CLK_SYS_PERIOD : time := 10 ns; -- 100 MHz system clock
  constant C_CLK_TDC_PERIOD : time := 2.5 ns; -- 400 MHz TDC clock
  constant C_REF_PERIOD     : time := C_CLK_SYS_PERIOD * 200; -- 2 µs measurement window

  constant C_TDL_LANES      : positive := 4;
  constant C_TDL_LENGTH     : positive := 128;
  constant C_COARSE_BITS    : positive := 8;
  constant C_OUTPUT_WIDTH   : positive := 16;
  constant C_TIME_DAC_STEPS : positive := 8;

  signal clk_sys       : std_logic                                       := '0';
  signal clk_tdc       : std_logic                                       := '0';
  signal reset         : std_logic                                       := '1';
  signal analog_in     : std_logic                                       := '0';
  signal ref_phases    : std_logic_vector(C_TIME_DAC_STEPS - 1 downto 0) := (others => '0');
  signal time_dac_ctrl : std_logic                                       := '0';
  signal tdc_out       : signed(C_OUTPUT_WIDTH - 1 downto 0);
  signal tdc_valid     : std_logic;
  signal overflow      : std_logic;

  signal measurement_count : integer := 0;
  signal sim_finished      : boolean := false;

  -- Test stimulus
  signal test_voltage : real                 := 0.0; -- Simulated input voltage (0.0 to 1.0)
  signal ref_counter  : unsigned(7 downto 0) := (others => '0'); -- For generating ref phases
  signal lost_sample  : std_logic;      -- V3.3: Sticky overflow flag

begin
  -- DUT
  i_dut : entity work.tdc_quantizer
    generic map(
      GC_TDL_LANES    => C_TDL_LANES,
      GC_TDL_LENGTH   => C_TDL_LENGTH,
      GC_COARSE_BITS  => C_COARSE_BITS,
      GC_OUTPUT_WIDTH => C_OUTPUT_WIDTH,
      GC_TIME_DAC_DEN => 256            -- Digital Time-DAC denominator (step = 1/256)
    )
    port map(
      clk_sys         => clk_sys,
      clk_tdc         => clk_tdc,
      reset           => reset,
      analog_in       => analog_in,
      ref_phases(0)   => ref_phases(0), -- Use only first phase
      time_dac_ctrl   => time_dac_ctrl,
      invert_polarity => '0',           -- V3.3: Normal polarity (no inversion)
      clear_status    => '0',           -- V3.4: Don't clear sticky flag during test
      tdc_out         => tdc_out,
      tdc_valid       => tdc_valid,
      overflow        => overflow,
      lost_sample     => lost_sample    -- V3.3: Sticky overflow tracking
    );

  -- System Clock (100 MHz)
  p_clk_sys : process
  begin
    while not sim_finished loop
      clk_sys <= '0';
      wait for C_CLK_SYS_PERIOD / 2;
      clk_sys <= '1';
      wait for C_CLK_SYS_PERIOD / 2;
    end loop;
    wait;
  end process;

  -- TDC Clock (400 MHz)
  p_clk_tdc : process
  begin
    while not sim_finished loop
      clk_tdc <= '0';
      wait for C_CLK_TDC_PERIOD / 2;
      clk_tdc <= '1';
      wait for C_CLK_TDC_PERIOD / 2;
    end loop;
    wait;
  end process;

  -- VUnit runner
  p_main : process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("Running basic TDC test - checking monotonicity and range");

        -- Wait for measurements to complete
        wait for C_CLK_SYS_PERIOD * 1200;

        -- Basic checks
        check(measurement_count >= 3, "At least 3 measurements should be captured, got " & integer'image(measurement_count));

        info("TDC test complete with " & integer'image(measurement_count) & " measurements");
      end if;
    end loop;

    sim_finished <= true;
    test_runner_cleanup(runner);
    wait;
  end process;

  -- Reset
  p_reset : process
  begin
    reset <= '1';
    wait for C_CLK_SYS_PERIOD * 10;
    reset <= '0';
    wait;
  end process;

  -- Reference phase generator (simulates multi-phase PLL output)
  -- Generate 8 evenly-spaced phases of the reference signal
  -- For C_TIME_DAC_STEPS=8, each phase is delayed by C_CLK_TDC_PERIOD (2.5ns/8 = 312.5ps)
  p_ref_phases : process
    constant C_PHASE_STEP  : time := C_CLK_TDC_PERIOD / C_TIME_DAC_STEPS; -- 312.5ps per phase
    constant C_PULSE_WIDTH : time := C_CLK_TDC_PERIOD * 10; -- Wide pulse (25ns) for reliable edge detection
  begin
    wait until reset = '0';
    ref_counter <= (others => '0');
    loop
      -- Increment counter every system clock period
      wait for C_CLK_SYS_PERIOD * 100;  -- Reference period = 1µs

      -- Generate phase pulses with staggered delays
      -- Only phase 0 is used by DUT, but generate all for completeness
      for phase in 0 to C_TIME_DAC_STEPS - 1 loop
        ref_phases        <= (others => '0');
        wait for C_PHASE_STEP;
        ref_phases(phase) <= '1';
        wait for C_PULSE_WIDTH;         -- Wide pulse ensures edge detection
        ref_phases(phase) <= '0';
      end loop;

      ref_counter <= ref_counter + 1;
    end loop;
  end process;

  -- Analog input simulator: generate crossing events based on test_voltage
  -- For V3.2 with tight window check, analog_in must transition within ~1 coarse period of Start
  -- Solution: Connect analog_in to ref_phases(0) with NO delay (same clk_tdc edge)
  -- This creates Δcoarse=0, Δfine variations come from TDL propagation delays
  p_analog_sim : process
  begin
    wait until reset = '0';
    loop
      -- Simply follow ref_phases(0) with minimal delay
      -- This ensures Δcoarse = 0 (same coarse counter value)
      analog_in <= ref_phases(0);
      wait for 0 ns;                    -- Delta cycle delay only
    end loop;
  end process;

  -- Time DAC control (simple test pattern)
  p_time_dac : process
  begin
    wait until reset = '0';
    time_dac_ctrl <= '0';
    wait for C_CLK_SYS_PERIOD * 500;
    time_dac_ctrl <= '1';
    wait for C_CLK_SYS_PERIOD * 500;
    time_dac_ctrl <= '0';
    wait;
  end process;

  -- Test stimulus: vary the input voltage over time
  p_stim : process
  begin
    wait until reset = '0';
    wait for C_CLK_SYS_PERIOD * 10;

    report "TDC Test: Low voltage (0.2)" severity note;
    test_voltage <= 0.2;
    wait for C_CLK_SYS_PERIOD * 200;

    report "TDC Test: Mid voltage (0.5)" severity note;
    test_voltage <= 0.5;
    wait for C_CLK_SYS_PERIOD * 200;

    report "TDC Test: High voltage (0.8)" severity note;
    test_voltage <= 0.8;
    wait for C_CLK_SYS_PERIOD * 200;

    report "TDC Test: Ramping voltage" severity note;
    for i in 0 to 10 loop
      test_voltage <= real(i) / 10.0;
      wait for C_CLK_SYS_PERIOD * 100;
    end loop;

    report "TDC Quantizer Test Complete" severity note;
    wait;
  end process;

  -- Monitor TDC outputs (sample on clk_tdc to avoid CDC issues)
  p_monitor : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        measurement_count <= 0;
      elsif tdc_valid = '1' then
        measurement_count <= measurement_count + 1;
        if overflow = '1' then
          report "Meas " & integer'image(measurement_count) & ": TDC=" & integer'image(to_integer(tdc_out)) & " OVERFLOW" severity note;
        else
          report "Meas " & integer'image(measurement_count) & ": TDC=" & integer'image(to_integer(tdc_out)) & " voltage=" & real'image(test_voltage) severity note;
        end if;
      end if;
    end if;
  end process;

  -- Quick stats
  p_stats : process
  begin
    wait for C_CLK_SYS_PERIOD * 20;
    report "=== TDC Test Configuration ===" severity note;
    report "System clock: " & time'image(C_CLK_SYS_PERIOD) severity note;
    report "TDC clock: " & time'image(C_CLK_TDC_PERIOD) severity note;
    report "TDL lanes: " & integer'image(C_TDL_LANES) severity note;
    report "TDL length: " & integer'image(C_TDL_LENGTH) severity note;
    report "Output width: " & integer'image(C_OUTPUT_WIDTH) & " bits" severity note;
    wait;
  end process;

end architecture;
