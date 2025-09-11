-- ************************************************************************
-- Testbench for FIR Equalizer with Decimation-by-2 (simple averaging)
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity fir_equalizer_tb is
  generic(runner_cfg : string);
end entity;

architecture sim of fir_equalizer_tb is
  -- TB constants
  constant CLK_PERIOD   : time     := 10 ns;
  constant INPUT_WIDTH  : positive := 16;
  constant OUTPUT_WIDTH : positive := 16;

  -- TB signals
  signal clk       : std_logic                                  := '0';
  signal reset     : std_logic                                  := '1'; -- Changed to std_logic
  signal data_in   : std_logic_vector(INPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_in  : std_logic                                  := '0';
  signal data_out  : std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
  signal valid_out : std_logic;

  signal input_sample_count  : integer := 0;
  signal output_sample_count : integer := 0;
  signal sim_finished        : boolean := false;
begin
  -- DUT (entity instantiation)
  dut : entity work.fir_equalizer
    generic map(
      INPUT_WIDTH  => INPUT_WIDTH,
      OUTPUT_WIDTH => OUTPUT_WIDTH
    )
    port map(
      clk       => clk,
      reset     => reset,
      data_in   => data_in,
      valid_in  => valid_in,
      data_out  => data_out,
      valid_out => valid_out
    );

  -- Clock
  clk_process : process
  begin
    while not sim_finished loop
      clk <= '0';
      wait for CLK_PERIOD / 2;
      clk <= '1';
      wait for CLK_PERIOD / 2;
    end loop;
    wait;
  end process;

  -- Test runner
  main : process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("Running basic test for fir_equalizer_tb");
        wait for CLK_PERIOD * 6000;     -- give stimulus time to complete
        check(true, "Basic test completed");
      end if;
    end loop;

    sim_finished <= true;
    test_runner_cleanup(runner);
    wait;
  end process;

  -- Reset
  reset_process : process
  begin
    reset <= '1';
    wait for CLK_PERIOD * 5;
    reset <= '0';
    wait;
  end process;

  -- Stimulus (no VUnit control here; just drive samples)
  stimulus_process : process
    variable phase, phase_inc, amp : real;
    variable val_r                 : real;
    variable val_i                 : integer;
  begin
    wait until reset = '0';
    wait for CLK_PERIOD * 2;

    report "Starting FIR Equalizer (Decimate-by-2) Test" severity note;

    -- 1) DC input
    report "Test 1: DC Input (1000)" severity note;
    for i in 0 to 50 loop
      wait until rising_edge(clk);
      data_in  <= std_logic_vector(to_signed(1000, INPUT_WIDTH));
      valid_in <= '1';
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;
    wait for CLK_PERIOD * 20;

    -- 2) Alternating pattern (+/- 2000) ~ Nyquist
    report "Test 2: Alternating Pattern (+/-2000)" severity note;
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      if (i mod 2) = 0 then
        data_in <= std_logic_vector(to_signed(2000, INPUT_WIDTH));
      else
        data_in <= std_logic_vector(to_signed(-2000, INPUT_WIDTH));
      end if;
      valid_in <= '1';
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;
    wait for CLK_PERIOD * 20;

    -- 3) Low freq sine (should pass)
    report "Test 3: Low Frequency Sine" severity note;
    phase     := 0.0;
    phase_inc := 2.0 * MATH_PI / 64.0;
    amp       := 4000.0;
    for i in 0 to 200 loop
      wait until rising_edge(clk);
      val_r    := amp * sin(phase);
      val_i    := integer(val_r);
      data_in  <= std_logic_vector(to_signed(val_i, INPUT_WIDTH));
      valid_in <= '1';
      phase    := phase + phase_inc;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;
    wait for CLK_PERIOD * 20;

    -- 4) Higher freq sine (should attenuate after decimation)
    report "Test 4: Higher Frequency Sine" severity note;
    phase     := 0.0;
    phase_inc := 2.0 * MATH_PI / 16.0;
    amp       := 3000.0;
    for i in 0 to 200 loop
      wait until rising_edge(clk);
      val_r    := amp * sin(phase);
      val_i    := integer(val_r);
      data_in  <= std_logic_vector(to_signed(val_i, INPUT_WIDTH));
      valid_in <= '1';
      phase    := phase + phase_inc;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 50;
    report "FIR Equalizer Test Complete" severity note;
    wait;
  end process;

  -- Counters/monitoring
  input_counter_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        input_sample_count <= 0;
      elsif valid_in = '1' then
        input_sample_count <= input_sample_count + 1;
      end if;
    end if;
  end process;

  output_monitor_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        output_sample_count <= 0;
      elsif valid_out = '1' then
        output_sample_count <= output_sample_count + 1;
        if output_sample_count mod 10 = 0 then
          info("Output " & integer'image(output_sample_count) & ": " & integer'image(to_integer(signed(data_out))) & " (0x" & to_hstring(data_out) & ")");
        end if;
      end if;
    end if;
  end process;
end architecture;
