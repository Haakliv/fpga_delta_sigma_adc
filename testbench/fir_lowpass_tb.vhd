-- ************************************************************************
-- Testbench for FIR Low-Pass Filter
-- Tests final filtering stage
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.clk_rst_pkg.all;

entity fir_lowpass_tb is
end entity;

architecture sim of fir_lowpass_tb is

  component fir_lowpass is
    generic (
      INPUT_WIDTH  : positive := 16;
      OUTPUT_WIDTH : positive := 16
    );
    port (
      clk       : in  std_logic;
      reset     : in  rst_t;
      data_in   : in  std_logic_vector(INPUT_WIDTH-1 downto 0);
      valid_in  : in  std_logic;
      data_out  : out std_logic_vector(OUTPUT_WIDTH-1 downto 0);
      valid_out : out std_logic
    );
  end component;

  -- Constants
  constant CLK_PERIOD : time := 10 ns;
  constant INPUT_WIDTH : positive := 16;
  constant OUTPUT_WIDTH : positive := 16;

  -- Signals
  signal clk       : std_logic := '0';
  signal reset     : rst_t := RST_ACTIVE;
  signal data_in   : std_logic_vector(INPUT_WIDTH-1 downto 0) := (others => '0');
  signal valid_in  : std_logic := '0';
  signal data_out  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
  signal valid_out : std_logic;

  signal test_complete : boolean := false;
  signal sample_count  : integer := 0;

begin

  -- Device Under Test
  dut : fir_lowpass
    generic map (
      INPUT_WIDTH  => INPUT_WIDTH,
      OUTPUT_WIDTH => OUTPUT_WIDTH
    )
    port map (
      clk       => clk,
      reset     => reset,
      data_in   => data_in,
      valid_in  => valid_in,
      data_out  => data_out,
      valid_out => valid_out
    );

  -- Clock generation
  clk_process : process
  begin
    while not test_complete loop
      clk_gen(clk, CLK_PERIOD);
    end loop;
    wait;
  end process;

  -- Reset generation
  reset_process : process
  begin
    reset <= RST_ACTIVE;
    wait for CLK_PERIOD * 5;
    reset <= not RST_ACTIVE;
    wait;
  end process;

  -- Test stimulus
  stimulus_process : process
    variable phase : real := 0.0;
    variable phase_inc : real;
    variable sample_val : real;
    variable sample_int : integer;
  begin
    wait until reset = not RST_ACTIVE;
    wait for CLK_PERIOD * 2;

    report "Starting FIR Low-Pass Filter Test" severity note;
    
    -- Test 1: Step response
    report "Test 1: Step Response" severity note;
    for i in 0 to 20 loop
      wait until rising_edge(clk);
      data_in <= std_logic_vector(to_signed(1000, INPUT_WIDTH));
      valid_in <= '1';
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 10;

    -- Test 2: Low frequency sine wave (should pass)
    report "Test 2: Low Frequency Sine Wave" severity note;
    phase := 0.0;
    phase_inc := 2.0 * MATH_PI / 32.0; -- Low frequency
    
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      sample_val := 2000.0 * sin(phase);
      sample_int := integer(sample_val);
      data_in <= std_logic_vector(to_signed(sample_int, INPUT_WIDTH));
      valid_in <= '1';
      phase := phase + phase_inc;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 10;

    -- Test 3: High frequency noise (should be attenuated)
    report "Test 3: High Frequency Noise" severity note;
    phase := 0.0;
    phase_inc := 2.0 * MATH_PI / 4.0; -- High frequency
    
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      sample_val := 1500.0 * sin(phase);
      sample_int := integer(sample_val);
      data_in <= std_logic_vector(to_signed(sample_int, INPUT_WIDTH));
      valid_in <= '1';
      phase := phase + phase_inc;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 20;
    
    report "FIR Low-Pass Filter Test Complete" severity note;
    test_complete <= true;
    wait;
  end process;

  -- Output monitoring
  monitor_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        sample_count <= 0;
      elsif valid_out = '1' then
        sample_count <= sample_count + 1;
        report "Output " & integer'image(sample_count) & 
               ": " & integer'image(to_integer(signed(data_out)))
               severity note;
      end if;
    end if;
  end process;

end architecture sim;
