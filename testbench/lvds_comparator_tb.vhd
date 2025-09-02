-- ************************************************************************
-- Testbench for LVDS/Comparator Wrapper
-- Tests with ideal stream + random bit flips for robustness
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.clk_rst_pkg.all;

entity lvds_comparator_tb is
end entity;

architecture sim of lvds_comparator_tb is

  component lvds_comparator is
    generic (
      ENABLE_MAJORITY : boolean := true
    );
    port (
      clk           : in  std_logic;
      reset         : in  rst_t;
      lvds_p        : in  std_logic;
      lvds_n        : in  std_logic;
      bit_stream    : out std_logic
    );
  end component;

  -- Constants
  constant CLK_PERIOD : time := 10 ns;
  constant BIT_FLIP_RATE : real := 0.05; -- 5% bit flip probability

  -- Signals
  signal clk         : std_logic := '0';
  signal reset       : rst_t := RST_ACTIVE;
  signal lvds_p      : std_logic := '0';
  signal lvds_n      : std_logic := '1';
  signal bit_stream  : std_logic;
  
  -- Test signals
  signal ideal_stream   : std_logic := '0';
  signal corrupted_p    : std_logic := '0';
  signal corrupted_n    : std_logic := '1';
  signal test_complete  : boolean := false;
  signal error_count    : integer := 0;
  signal total_samples  : integer := 0;

begin

  -- Device Under Test
  dut : lvds_comparator
    generic map (
      ENABLE_MAJORITY => true
    )
    port map (
      clk        => clk,
      reset      => reset,
      lvds_p     => corrupted_p,
      lvds_n     => corrupted_n,
      bit_stream => bit_stream
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

  -- Ideal bit stream generation (delta-sigma like pattern)
  ideal_stream_process : process(clk)
    variable counter : integer := 0;
    variable pattern : std_logic_vector(7 downto 0) := "10110100"; -- Example pattern
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        counter := 0;
        ideal_stream <= '0';
      else
        ideal_stream <= pattern(counter mod 8);
        counter := counter + 1;
      end if;
    end if;
  end process;

  -- Add random bit flips to test robustness
  corruption_process : process(clk)
    variable seed1, seed2 : integer := 999;
    variable rand_val : real;
    variable flip_bit : std_logic;
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
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
          corrupted_p <= not ideal_stream;
          corrupted_n <= ideal_stream;
        else
          corrupted_p <= ideal_stream;
          corrupted_n <= not ideal_stream;
        end if;
      end if;
    end if;
  end process;

  -- Test monitoring
  monitor_process : process(clk)
    variable delayed_ideal : std_logic_vector(4 downto 0) := (others => '0');
    variable expected_out : std_logic;
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        error_count <= 0;
        total_samples <= 0;
        delayed_ideal := (others => '0');
      else
        -- Delay ideal stream to account for pipeline delay
        delayed_ideal := delayed_ideal(3 downto 0) & ideal_stream;
        expected_out := delayed_ideal(4); -- 5 clock delay for majority filter
        
        total_samples <= total_samples + 1;
        
        -- Check for errors (after initial settling)
        if total_samples > 10 then
          if bit_stream /= expected_out then
            error_count <= error_count + 1;
          end if;
        end if;
        
        -- Report progress occasionally
        if (total_samples mod 100) = 0 and total_samples > 0 then
          report "Samples: " & integer'image(total_samples) & 
                 ", Errors: " & integer'image(error_count) &
                 ", Error rate: " & real'image(real(error_count)/real(total_samples))
                 severity note;
        end if;
      end if;
    end if;
  end process;

  -- Test sequence
  test_process : process
  begin
    wait until reset = not RST_ACTIVE;
    
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
      report "Final error rate: " & real'image(real(error_count)/real(total_samples)) severity note;
      
      if real(error_count)/real(total_samples) < 0.01 then -- Less than 1% error
        report "PASS: Majority filter working effectively" severity note;
      else
        report "FAIL: Too many errors getting through" severity error;
      end if;
    end if;
    
    test_complete <= true;
    wait;
  end process;

end architecture sim;
