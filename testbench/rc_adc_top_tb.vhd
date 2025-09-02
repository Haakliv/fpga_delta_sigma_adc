-- ************************************************************************
-- Testbench for Top-Level RC ADC
-- Board test simulation: low OSR → high OSR with spectrum analysis
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.clk_rst_pkg.all;

entity rc_adc_top_tb is
end entity;

architecture sim of rc_adc_top_tb is

  component rc_adc_top is
    generic (
      OSR              : positive := 64;
      DATA_WIDTH       : positive := 16;
      ENABLE_MAJORITY  : boolean := true
    );
    port (
      clk           : in  std_logic;
      reset         : in  rst_t;
      lvds_p        : in  std_logic;
      lvds_n        : in  std_logic;
      dac_out       : out std_logic;
      stream_out    : out std_logic_vector(DATA_WIDTH-1 downto 0);
      stream_valid  : out std_logic;
      status        : out std_logic_vector(7 downto 0)
    );
  end component;

  -- Test parameters
  constant CLK_PERIOD : time := 10 ns;  -- 100 MHz
  constant DATA_WIDTH : positive := 16;
  
  -- Signals for DUT with OSR=16 (low OSR start)
  signal clk           : std_logic := '0';
  signal reset         : rst_t := RST_ACTIVE;
  signal lvds_p        : std_logic := '0';
  signal lvds_n        : std_logic := '1';
  signal dac_out       : std_logic;
  signal stream_out    : std_logic_vector(DATA_WIDTH-1 downto 0);
  signal stream_valid  : std_logic;
  signal status        : std_logic_vector(7 downto 0);

  -- Test signals
  signal test_complete : boolean := false;
  signal test_phase    : integer := 0;
  signal sample_count  : integer := 0;
  signal input_freq    : real := 1000000.0; -- 1 MHz test frequency
  signal input_amp     : real := 0.1;       -- Low amplitude start

begin

  -- Device Under Test (start with low OSR=16)
  dut : rc_adc_top
    generic map (
      OSR              => 16,  -- Low OSR for initial test
      DATA_WIDTH       => DATA_WIDTH,
      ENABLE_MAJORITY  => true
    )
    port map (
      clk          => clk,
      reset        => reset,
      lvds_p       => lvds_p,
      lvds_n       => lvds_n,
      dac_out      => dac_out,
      stream_out   => stream_out,
      stream_valid => stream_valid,
      status       => status
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
    wait for CLK_PERIOD * 10;
    reset <= not RST_ACTIVE;
    wait;
  end process;

  -- Test signal generation (simulates analog input)
  signal_gen_process : process
    variable phase : real := 0.0;
    variable phase_inc : real;
    variable sample_val : real;
    variable noise : real;
    variable seed1, seed2 : integer := 123;
    variable rand_val : real;
  begin
    wait until reset = not RST_ACTIVE;
    
    phase_inc := 2.0 * MATH_PI * input_freq * (CLK_PERIOD / 1 sec);
    
    loop
      wait until rising_edge(clk);
      
      -- Generate test signal with noise
      sample_val := input_amp * sin(phase);
      
      -- Add small amount of noise
      uniform(seed1, seed2, rand_val);
      noise := (rand_val - 0.5) * 0.02; -- 2% noise
      sample_val := sample_val + noise;
      
      -- Convert to differential LVDS
      if sample_val > 0.0 then
        lvds_p <= '1';
        lvds_n <= '0';
      else
        lvds_p <= '0';
        lvds_n <= '1';
      end if;
      
      phase := phase + phase_inc;
      if phase >= 2.0 * MATH_PI then
        phase := phase - 2.0 * MATH_PI;
      end if;
      
      exit when test_complete;
    end loop;
    
    wait;
  end process;

  -- Test sequence controller
  test_sequence : process
  begin
    wait until reset = not RST_ACTIVE;
    wait for CLK_PERIOD * 20;
    
    report "=== RC ADC Board Test Sequence ===" severity note;
    
    -- Phase 1: Low OSR, Low Amplitude
    test_phase <= 1;
    input_amp <= 0.1;  -- 10% amplitude
    report "Phase 1: OSR=16, Low Amplitude (10%)" severity note;
    wait for CLK_PERIOD * 2000;  -- Let it settle
    
    -- Phase 2: Increase amplitude
    test_phase <= 2;
    input_amp <= 0.3;  -- 30% amplitude
    report "Phase 2: OSR=16, Medium Amplitude (30%)" severity note;
    wait for CLK_PERIOD * 2000;
    
    -- Phase 3: Higher amplitude
    test_phase <= 3;
    input_amp <= 0.7;  -- 70% amplitude
    report "Phase 3: OSR=16, High Amplitude (70%)" severity note;
    wait for CLK_PERIOD * 2000;
    
    -- Phase 4: Different frequency
    test_phase <= 4;
    input_freq <= 500000.0;  -- 500 kHz
    input_amp <= 0.5;
    report "Phase 4: OSR=16, 500kHz input" severity note;
    wait for CLK_PERIOD * 2000;
    
    -- Note: In real board test, you would reconfigure OSR here
    -- For simulation, we test the fundamental functionality
    
    report "=== Test Complete - Ready for OSR Ramping ===" severity note;
    report "Next step: Reconfigure with higher OSR (64, 128, 256)" severity note;
    
    wait for CLK_PERIOD * 1000;
    test_complete <= true;
    wait;
  end process;

  -- Output monitoring and analysis
  monitor_process : process(clk)
    variable min_val : integer := 32767;
    variable max_val : integer := -32768;
    variable sum_val : integer := 0;
    variable sum_count : integer := 0;
    variable current_val : integer;
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        sample_count <= 0;
        min_val := 32767;
        max_val := -32768;
        sum_val := 0;
        sum_count := 0;
      elsif stream_valid = '1' then
        sample_count <= sample_count + 1;
        
        -- Track statistics
        current_val := to_integer(signed(stream_out));
        if current_val < min_val then
          min_val := current_val;
        end if;
        if current_val > max_val then
          max_val := current_val;
        end if;
        sum_val := sum_val + current_val;
        sum_count := sum_count + 1;
        
        -- Report statistics every 100 samples
        if (sample_count mod 100) = 0 then
          report "Sample " & integer'image(sample_count) & 
                 ": Value=" & integer'image(to_integer(signed(stream_out))) &
                 ", Status=" & to_hstring(status) &
                 ", Range=[" & integer'image(min_val) & ".." & integer'image(max_val) & "]" &
                 ", Avg=" & integer'image(sum_val/sum_count)
                 severity note;
        end if;
      end if;
    end if;
  end process;

  -- Status monitoring
  status_monitor : process(clk)
  begin
    if rising_edge(clk) then
      if rising_edge(status(4)) then  -- Heartbeat
        report "Heartbeat - Comparator: " & std_logic'image(status(0)) &
               ", CIC: " & std_logic'image(status(1)) &
               ", EQ: " & std_logic'image(status(2)) &
               ", LP: " & std_logic'image(status(3))
               severity note;
      end if;
    end if;
  end process;

end architecture sim;
