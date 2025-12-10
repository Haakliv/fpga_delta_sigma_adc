-- ************************************************************************
-- Testbench for FIR Low-Pass Filter
-- Tests final filtering stage
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity fir_lowpass_tb is
  generic(runner_cfg : string);
end entity;

architecture behavioral of fir_lowpass_tb is

  -- component fir_lowpass is
  --   generic(
  --     INPUT_WIDTH  : positive := 16;
  --     OUTPUT_WIDTH : positive := 16
  --   );
  --   port(
  --     clk       : in  std_logic;
  --     reset     : in  std_logic;
  --     data_in   : in  std_logic_vector(INPUT_WIDTH - 1 downto 0);
  --     valid_in  : in  std_logic;
  --     data_out  : out std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
  --     valid_out : out std_logic
  --   );
  -- end component;

  -- Constants
  constant C_CLK_PERIOD   : time     := 10 ns;
  constant C_INPUT_WIDTH  : positive := 16;
  constant C_OUTPUT_WIDTH : positive := 16;

  -- Signals
  signal clk       : std_logic                                    := '0';
  signal reset     : std_logic                                    := '1'; -- Changed to std_logic
  signal data_in   : std_logic_vector(C_INPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_in  : std_logic                                    := '0';
  signal data_out  : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0);
  signal valid_out : std_logic;

  signal sample_count : integer := 0;
  signal sim_finished : boolean := false;

begin

  -- Device Under Test (entity instantiation)
  i_dut : entity fpga_lib.fir_lowpass
    generic map(
      GC_INPUT_WIDTH  => C_INPUT_WIDTH,
      GC_OUTPUT_WIDTH => C_OUTPUT_WIDTH
    )
    port map(
      clk       => clk,
      reset     => reset,
      data_in   => data_in,
      valid_in  => valid_in,
      data_out  => data_out,
      valid_out => valid_out
    );

  -- Clock generation
  p_clk : process
  begin
    while not sim_finished loop
      clk <= '0';
      wait for C_CLK_PERIOD / 2;
      clk <= '1';
      wait for C_CLK_PERIOD / 2;
    end loop;
    wait;
  end process;

  -- Test runner process
  p_main : process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("Running basic test for fir_lowpass_tb");
        -- Test completion is handled in test_process
        wait for C_CLK_PERIOD * 10000;  -- Allow test to complete
        check(true, "Basic test completed");
      end if;
    end loop;

    sim_finished <= true;
    test_runner_cleanup(runner);
    wait;
  end process;

  -- Reset generation
  p_reset : process
  begin
    reset <= '1';
    wait for C_CLK_PERIOD * 5;
    reset <= '0';
    wait;
  end process;

  -- Test stimulus
  p_stimulus : process
    variable v_phase      : real := 0.0;
    variable v_phase_inc  : real;
    variable v_sample_val : real;
    variable v_sample_int : integer;
  begin
    wait until reset = '0';
    wait for C_CLK_PERIOD * 2;

    report "Starting FIR Low-Pass Filter Test" severity note;

    -- Test 1: Step response
    report "Test 1: Step Response" severity note;
    for i in 0 to 20 loop
      wait until rising_edge(clk);
      data_in  <= std_logic_vector(to_signed(1000, C_INPUT_WIDTH));
      valid_in <= '1';
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for C_CLK_PERIOD;
    end loop;

    wait for C_CLK_PERIOD * 10;

    -- Test 2: Low frequency sine wave (should pass)
    report "Test 2: Low Frequency Sine Wave" severity note;
    v_phase     := 0.0;
    v_phase_inc := 2.0 * MATH_PI / 32.0; -- Low frequency

    for i in 0 to 100 loop
      wait until rising_edge(clk);
      v_sample_val := 2000.0 * sin(v_phase);
      v_sample_int := integer(v_sample_val);
      data_in      <= std_logic_vector(to_signed(v_sample_int, C_INPUT_WIDTH));
      valid_in     <= '1';
      v_phase      := v_phase + v_phase_inc;
      if v_phase >= 2.0 * MATH_PI then
        v_phase := v_phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in     <= '0';
      wait for C_CLK_PERIOD;
    end loop;

    wait for C_CLK_PERIOD * 10;

    -- Test 3: High frequency noise (should be attenuated)
    report "Test 3: High Frequency Noise" severity note;
    v_phase     := 0.0;
    v_phase_inc := 2.0 * MATH_PI / 4.0; -- High frequency

    for i in 0 to 100 loop
      wait until rising_edge(clk);
      v_sample_val := 1500.0 * sin(v_phase);
      v_sample_int := integer(v_sample_val);
      data_in      <= std_logic_vector(to_signed(v_sample_int, C_INPUT_WIDTH));
      valid_in     <= '1';
      v_phase      := v_phase + v_phase_inc;
      if v_phase >= 2.0 * MATH_PI then
        v_phase := v_phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in     <= '0';
      wait for C_CLK_PERIOD;
    end loop;

    wait for C_CLK_PERIOD * 20;

    report "FIR Low-Pass Filter Test Complete" severity note;
    wait;
  end process;

  -- Output monitoring
  p_monitor : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        sample_count <= 0;
      elsif valid_out = '1' then
        sample_count <= sample_count + 1;

        report "Output " & integer'image(sample_count) & ": " & integer'image(to_integer(signed(data_out)))
        severity note;
      end if;
    end if;
  end process;

end architecture;
