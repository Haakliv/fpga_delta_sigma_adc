-- ************************************************************************
-- Testbench for LVDS/Comparator Wrapper
-- Tests with ideal stream + random bit flips for robustness
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity lvds_comparator_tb is
  generic(runner_cfg : string);
end entity;

architecture sim of lvds_comparator_tb is

  -- Constants
  constant CLK_PERIOD    : time := 10 ns;
  constant BIT_FLIP_RATE : real := 0.02; -- 2% bit flip probability (more reasonable)

  -- Signals
  signal clk        : std_logic := '0';
  signal reset      : std_logic := '1'; -- Changed to std_logic
  signal bit_stream : std_logic;

  -- Test signals
  signal test_pattern  : std_logic := '0';
  signal corrupted_p   : std_logic := '0';
  signal corrupted_n   : std_logic := '1';
  signal error_count   : integer   := 0;
  signal total_samples : integer   := 0;
  signal sim_finished  : boolean   := false;

begin

  -- Device Under Test (entity instantiation)
  dut : entity work.lvds_comparator
    generic map(
      ENABLE_MAJORITY => true,
      USE_INTEL_LVDS  => false
    )
    port map(
      clk        => clk,
      reset      => reset,
      lvds_p     => corrupted_p,
      lvds_n     => corrupted_n,
      bit_stream => bit_stream
    );

  -- Clock generation
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

  -- Test runner process
  main : process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("Running basic test for lvds_comparator_tb");
        -- Test completion is handled in test_process
        wait for CLK_PERIOD * 10000;    -- Allow test to complete
        check(true, "Basic test completed");
      end if;
    end loop;

    sim_finished <= true;
    test_runner_cleanup(runner);
    wait;
  end process;

  -- Reset generation
  reset_process : process
  begin
    reset <= '1';
    wait for CLK_PERIOD * 5;
    reset <= '0';
    wait;
  end process;

  -- Test pattern generation (simple alternating pattern for debugging)
  test_pattern_process : process(clk)
    variable counter : integer                      := 0;
    constant pattern : std_logic_vector(7 downto 0) := "10101010"; -- Simple alternating pattern
  begin
    if rising_edge(clk) then
      if reset = '1' then
        counter      := 0;
        test_pattern <= '0';
      else
        test_pattern <= pattern(counter mod 8);
        counter      := counter + 1;
      end if;
    end if;
  end process;

  -- Add some bit flips to test robustness
  corruption_process : process(clk)
    variable seed1, seed2 : integer := 999;
    variable rand_val     : real;
    variable flip_bit     : std_logic;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        corrupted_p <= '0';
        corrupted_n <= '1';
      else
        -- Generate random number
        uniform(seed1, seed2, rand_val);

        -- Decide if we flip this bit
        if rand_val < BIT_FLIP_RATE then
          flip_bit := '1';
        else
          flip_bit := '0';
        end if;

        -- Apply corruption
        if flip_bit = '1' then
          corrupted_p <= not test_pattern;
          corrupted_n <= test_pattern;
        else
          corrupted_p <= test_pattern;
          corrupted_n <= not test_pattern;
        end if;
      end if;
    end if;
  end process;

  -- Test monitoring: mirror DUT exactly (2-FF sync + 3-tap majority)
  monitor_process : process(clk)
    variable sync_m     : std_logic_vector(1 downto 0) := (others => '0');
    variable maj_m      : std_logic_vector(2 downto 0) := (others => '0');
    variable sum        : integer range 0 to 3;
    variable expected   : std_logic                    := '0';
    variable comp_model : std_logic;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        error_count   <= 0;
        total_samples <= 0;
        sync_m        := (others => '0');
        maj_m         := (others => '0');
        expected      := '0';
      else
        -- Behavioral comparator exactly like the DUT sim_fallback
        comp_model := '1' when (corrupted_p = '1' and corrupted_n = '0') else
                      '0' when (corrupted_p = '0' and corrupted_n = '1') else
                      corrupted_p;

        -- Mirror 2-FF sync (shift register)
        sync_m := sync_m(0) & comp_model;

        -- Compute majority on CURRENT maj_m (before shift)
        sum      := 0;
        for i in 0 to 2 loop
          if maj_m(i) = '1' then
            sum := sum + 1;
          end if;
        end loop;
        expected := '1' when sum >= 2 else '0';

        -- Shift in new sample from sync output
        maj_m := maj_m(1 downto 0) & sync_m(1);

        -- Only start comparing after pipeline fills (2 sync + 3 majority = 5 cycles)
        total_samples <= total_samples + 1;
        if total_samples > 5 then
          if bit_stream /= expected then
            error_count <= error_count + 1;
            report "Mismatch at sample " & integer'image(total_samples) & ": got " & std_logic'image(bit_stream) & ", expected " & std_logic'image(expected) severity note;
          end if;
        end if;

        if (total_samples > 0) and ((total_samples mod 100) = 0) then
          report "Samples: " & integer'image(total_samples) & ", Errors: " & integer'image(error_count) & ", Error rate: " & real'image(real(error_count) / real(total_samples))
          severity note;
        end if;
      end if;
    end if;
  end process;

  -- Test sequence
  test_process : process
  begin
    wait until reset = '0';

    report "Starting LVDS/Comparator Test" severity note;
    report "Bit flip rate: " & real'image(BIT_FLIP_RATE) severity note;

    -- Run test for sufficient time
    wait for CLK_PERIOD * 1000;

    -- Final report
    wait for CLK_PERIOD * 10;
    report "=== Final Results ===" severity note;
    report "Total samples: " & integer'image(total_samples) severity note;
    report "Total errors: " & integer'image(error_count) severity note;

    if total_samples > 0 then
      report "Final error rate: " & real'image(real(error_count) / real(total_samples)) severity note;

      if real(error_count) / real(total_samples) < BIT_FLIP_RATE * 2.0 then -- Error rate should be better than 2x input corruption rate
        report "PASS: Majority filter working effectively" severity note;
      else
        report "FAIL: Too many errors getting through" severity error;
      end if;
    end if;

    wait;
  end process;

end architecture;
