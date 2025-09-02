-- ************************************************************************
-- Testbench for FIR Equalizer with Decimation
-- Tests frequency response compensation and decimation functionality
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.clk_rst_pkg.all;

entity fir_equalizer_tb is
end entity;

architecture sim of fir_equalizer_tb is

  -- Component declaration
  component fir_equalizer is
    generic (
      INPUT_WIDTH  : positive := 16;
      OUTPUT_WIDTH : positive := 16;
      COEFF_WIDTH  : positive := 16
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

  -- Testbench constants
  constant CLK_PERIOD : time := 10 ns;
  constant INPUT_WIDTH : positive := 16;
  constant OUTPUT_WIDTH : positive := 16;

  -- Testbench signals
  signal clk       : std_logic := '0';
  signal reset     : rst_t := RST_ACTIVE;
  signal data_in   : std_logic_vector(INPUT_WIDTH-1 downto 0) := (others => '0');
  signal valid_in  : std_logic := '0';
  signal data_out  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
  signal valid_out : std_logic;

  -- Test signals
  signal input_sample_count  : integer := 0;
  signal output_sample_count : integer := 0;
  signal test_complete       : boolean := false;

begin

  -- Device Under Test
  dut : fir_equalizer
    generic map (
      INPUT_WIDTH  => INPUT_WIDTH,
      OUTPUT_WIDTH => OUTPUT_WIDTH,
      COEFF_WIDTH  => 16
    )
    port map (
      clk       => clk,
      reset     => reset,
      data_in   => data_in,
      valid_in  => valid_in,
      data_out  => data_out,
      valid_out => valid_out
    );

  -- Clock generation using package procedure
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

  -- Test stimulus generation
  stimulus_process : process
    variable phase : real := 0.0;
    variable phase_increment : real;
    variable amplitude : real;
    variable sample_value : real;
    variable sample_int : integer;
  begin
    -- Wait for reset release
    wait until reset = not RST_ACTIVE;
    wait for CLK_PERIOD * 2;

    report "Starting FIR Equalizer Test" severity note;
    report "=============================" severity note;

    -- Test 1: DC input (impulse response verification)
    report "Test 1: DC Input (value = 1000)" severity note;
    for i in 0 to 50 loop
      wait until rising_edge(clk);
      data_in <= std_logic_vector(to_signed(1000, INPUT_WIDTH));
      valid_in <= '1';
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 20;

    -- Test 2: Alternating pattern (Nyquist frequency)
    report "Test 2: Alternating Pattern (Nyquist test)" severity note;
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      if i mod 2 = 0 then
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

    -- Test 3: Low frequency sine wave
    report "Test 3: Low Frequency Sine Wave" severity note;
    phase := 0.0;
    phase_increment := 2.0 * MATH_PI / 64.0; -- Low frequency
    amplitude := 4000.0;
    
    for i in 0 to 200 loop
      wait until rising_edge(clk);
      sample_value := amplitude * sin(phase);
      sample_int := integer(sample_value);
      data_in <= std_logic_vector(to_signed(sample_int, INPUT_WIDTH));
      valid_in <= '1';
      phase := phase + phase_increment;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 20;

    -- Test 4: Higher frequency sine wave
    report "Test 4: Higher Frequency Sine Wave" severity note;
    phase := 0.0;
    phase_increment := 2.0 * MATH_PI / 16.0; -- Higher frequency
    amplitude := 3000.0;
    
    for i in 0 to 200 loop
      wait until rising_edge(clk);
      sample_value := amplitude * sin(phase);
      sample_int := integer(sample_value);
      data_in <= std_logic_vector(to_signed(sample_int, INPUT_WIDTH));
      valid_in <= '1';
      phase := phase + phase_increment;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      wait until rising_edge(clk);
      valid_in <= '0';
      wait for CLK_PERIOD;
    end loop;

    wait for CLK_PERIOD * 50;

    report "Test completed successfully!" severity note;
    test_complete <= true;
    wait;
  end process;

  -- Input sample counter
  input_counter_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        input_sample_count <= 0;
      elsif valid_in = '1' then
        input_sample_count <= input_sample_count + 1;
      end if;
    end if;
  end process;

  -- Output monitoring
  output_monitor_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        output_sample_count <= 0;
      elsif valid_out = '1' then
        output_sample_count <= output_sample_count + 1;
        
        -- Report output values for verification
        report "Output " & integer'image(output_sample_count) & 
               ": " & integer'image(to_integer(signed(data_out))) &
               " (0x" & to_hstring(data_out) & ")" 
               severity note;
      end if;
    end if;
  end process;

  -- Decimation verification
  decimation_check_process : process
  begin
    wait until test_complete;
    wait for CLK_PERIOD * 10;
    
    report "=== Test Summary ===" severity note;
    report "Input samples: " & integer'image(input_sample_count) severity note;
    report "Output samples: " & integer'image(output_sample_count) severity note;
    
    if output_sample_count > 0 then
      report "Decimation ratio: " & integer'image(input_sample_count / output_sample_count) severity note;
      
      if (input_sample_count / output_sample_count) = 2 then
        report "PASS: Decimation by 2 working correctly" severity note;
      else
        report "FAIL: Decimation ratio incorrect" severity error;
      end if;
    end if;
    
    report "=== Test Complete ===" severity note;
    wait;
  end process;

end architecture sim;
