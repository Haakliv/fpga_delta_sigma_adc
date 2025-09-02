-- ************************************************************************
-- Testbench for TDC Quantizer
-- Tests time measurement functionality and parasitic delay simulation
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.clk_rst_pkg.all;

entity tdc_quantizer_tb is
end entity;

architecture sim of tdc_quantizer_tb is

  component tdc_quantizer is
    generic (
      TDC_BITS     : positive := 8;
      COUNTER_BITS : positive := 16
    );
    port (
      clk       : in  std_logic;
      reset     : in  rst_t;
      tdc_start : in  std_logic;
      tdc_stop  : in  std_logic;
      enable    : in  std_logic;
      trigger   : in  std_logic;
      tdc_value : out std_logic_vector(TDC_BITS - 1 downto 0);
      tdc_valid : out std_logic;
      overflow  : out std_logic
    );
  end component;

  -- Constants
  constant CLK_PERIOD : time     := 10 ns; -- 100 MHz
  constant TDC_BITS   : positive := 8;

  -- Signals
  signal clk       : std_logic := '0';
  signal reset     : rst_t     := RST_ACTIVE;
  signal tdc_start : std_logic := '0';
  signal tdc_stop  : std_logic := '0';
  signal enable    : std_logic := '0';
  signal trigger   : std_logic := '0';
  signal tdc_value : std_logic_vector(TDC_BITS - 1 downto 0);
  signal tdc_valid : std_logic;
  signal overflow  : std_logic;

  -- Test signals
  signal test_complete     : boolean := false;
  signal test_phase        : integer := 0;
  signal measurement_count : integer := 0;

begin

  -- Device Under Test
  dut: tdc_quantizer
    generic map (
      TDC_BITS     => TDC_BITS,
      COUNTER_BITS => 16
    )
    port map (
      clk       => clk,
      reset     => reset,
      tdc_start => tdc_start,
      tdc_stop  => tdc_stop,
      enable    => enable,
      trigger   => trigger,
      tdc_value => tdc_value,
      tdc_valid => tdc_valid,
      overflow  => overflow
    );

  -- Clock generation
  clk_process: process
  begin
    while not test_complete loop
      clk_gen(clk, CLK_PERIOD);
    end loop;
    wait;
  end process;

  -- Reset generation
  reset_process: process
  begin
    reset <= RST_ACTIVE;
    wait for CLK_PERIOD * 5;
    reset <= not RST_ACTIVE;
    wait;
  end process;

  -- Test sequence
  test_process: process
  begin
    wait until reset = not RST_ACTIVE;
    wait for CLK_PERIOD * 2;

    report "Starting TDC Quantizer Test" severity note;
    report "============================" severity note;

    -- Enable TDC
    enable <= '1';
    wait for CLK_PERIOD * 2;

    -- Test 1: Basic timing measurement (10 clock cycles)
    test_phase <= 1;
    report "Test 1: 10 clock cycle measurement" severity note;

    wait until rising_edge(clk);
    trigger <= '1';
    wait until rising_edge(clk);
    trigger <= '0';
    tdc_start <= '1';
    wait until rising_edge(clk);
    tdc_start <= '0';

    -- Wait 10 clock cycles
    wait for CLK_PERIOD * 10;

    wait until rising_edge(clk);
    tdc_stop <= '1';
    wait until rising_edge(clk);
    tdc_stop <= '0';

    -- Wait for measurement
    wait until tdc_valid = '1';
    wait for CLK_PERIOD * 5;

    -- Test 2: Different delay (25 clock cycles)
    test_phase <= 2;
    report "Test 2: 25 clock cycle measurement" severity note;

    wait until rising_edge(clk);
    trigger <= '1';
    wait until rising_edge(clk);
    trigger <= '0';
    tdc_start <= '1';
    wait until rising_edge(clk);
    tdc_start <= '0';

    -- Wait 25 clock cycles
    wait for CLK_PERIOD * 25;

    wait until rising_edge(clk);
    tdc_stop <= '1';
    wait until rising_edge(clk);
    tdc_stop <= '0';

    wait until tdc_valid = '1';
    wait for CLK_PERIOD * 5;

    -- Test 3: Very short delay (3 clock cycles)
    test_phase <= 3;
    report "Test 3: 3 clock cycle measurement" severity note;

    wait until rising_edge(clk);
    trigger <= '1';
    wait until rising_edge(clk);
    trigger <= '0';
    tdc_start <= '1';
    wait until rising_edge(clk);
    tdc_start <= '0';

    -- Wait 3 clock cycles
    wait for CLK_PERIOD * 3;

    wait until rising_edge(clk);
    tdc_stop <= '1';
    wait until rising_edge(clk);
    tdc_stop <= '0';

    wait until tdc_valid = '1';
    wait for CLK_PERIOD * 5;

    -- Test 4: Maximum range (near overflow)
    test_phase <= 4;
    report "Test 4: Near maximum range measurement" severity note;

    wait until rising_edge(clk);
    trigger <= '1';
    wait until rising_edge(clk);
    trigger <= '0';
    tdc_start <= '1';
    wait until rising_edge(clk);
    tdc_start <= '0';

    -- Wait for near maximum (250 cycles for 8-bit TDC)
    wait for CLK_PERIOD * 250;

    wait until rising_edge(clk);
    tdc_stop <= '1';
    wait until rising_edge(clk);
    tdc_stop <= '0';

    wait until tdc_valid = '1';
    wait for CLK_PERIOD * 5;

    -- Test 5: Overflow test
    test_phase <= 5;
    report "Test 5: Overflow test (>255 cycles)" severity note;

    wait until rising_edge(clk);
    trigger <= '1';
    wait until rising_edge(clk);
    trigger <= '0';
    tdc_start <= '1';
    wait until rising_edge(clk);
    tdc_start <= '0';

    -- Wait for overflow (300 cycles for 8-bit TDC)
    wait for CLK_PERIOD * 300;

    wait until rising_edge(clk);
    tdc_stop <= '1';
    wait until rising_edge(clk);
    tdc_stop <= '0';

    wait until tdc_valid = '1';
    wait for CLK_PERIOD * 5;

    -- Test 6: Rapid measurements
    test_phase <= 6;
    report "Test 6: Rapid sequential measurements" severity note;

    for i in 1 to 5 loop
      wait until rising_edge(clk);
      trigger <= '1';
      wait until rising_edge(clk);
      trigger <= '0';
      tdc_start <= '1';
      wait until rising_edge(clk);
      tdc_start <= '0';

      -- Random delay between 5-15 cycles
      wait for CLK_PERIOD * (5 + (i * 2));

      wait until rising_edge(clk);
      tdc_stop <= '1';
      wait until rising_edge(clk);
      tdc_stop <= '0';

      wait until tdc_valid = '1';
      wait for CLK_PERIOD * 2;
    end loop;

    -- Test 7: Parasitic delay simulation
    test_phase <= 7;
    report "Test 7: Parasitic R/C delay simulation" severity note;

    -- Simulate varying parasitic delays (simple version)
    for voltage_level in 1 to 5 loop
      wait until rising_edge(clk);
      trigger <= '1';
      wait until rising_edge(clk);
      trigger <= '0';
      tdc_start <= '1';
      wait until rising_edge(clk);
      tdc_start <= '0';

      -- Simulate parasitic delay proportional to "voltage"
      -- Higher voltage = shorter delay (faster charging)
      if voltage_level = 1 then
        wait for CLK_PERIOD * 46;
      elsif voltage_level = 2 then
        wait for CLK_PERIOD * 42;
      elsif voltage_level = 3 then
        wait for CLK_PERIOD * 38;
      elsif voltage_level = 4 then
        wait for CLK_PERIOD * 34;
      else
        wait for CLK_PERIOD * 30;
      end if;

      wait until rising_edge(clk);
      tdc_stop <= '1';
      wait until rising_edge(clk);
      tdc_stop <= '0';

      wait until tdc_valid = '1';
      wait for CLK_PERIOD * 2;
    end loop;

    wait for CLK_PERIOD * 20;

    report "TDC Quantizer Test Complete" severity note;
    test_complete <= true;
    wait;
  end process;

  -- Measurement monitoring
  monitor_process: process (clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        measurement_count <= 0;
      elsif tdc_valid = '1' then
        measurement_count <= measurement_count + 1;

        if overflow = '1' then

            report "Measurement " & integer'image(measurement_count) & ": TDC Value = " & integer'image(to_integer(unsigned(tdc_value))) & " (0x" & to_hstring(tdc_value) & ") - OVERFLOW"
            severity note;
        else

            report "Measurement " & integer'image(measurement_count) & ": TDC Value = " & integer'image(to_integer(unsigned(tdc_value))) & " (0x" & to_hstring(tdc_value) & ")"
            severity note;
        end if;
      end if;
    end if;
  end process;

  -- Statistics tracking
  stats_process: process
  begin
    wait until test_complete;
    wait for CLK_PERIOD * 10;

    report "=== Test Statistics ===" severity note;
    report "Total measurements: " & integer'image(measurement_count) severity note;
    report "TDC resolution: " & integer'image(TDC_BITS) & " bits" severity note;
    report "Clock period: " & time'image(CLK_PERIOD) severity note;
    report "Time resolution: " & time'image(CLK_PERIOD) & " per LSB" severity note;

    if measurement_count > 0 then
      report "PASS: TDC functioning correctly" severity note;
    else
      report "FAIL: No measurements captured" severity error;
    end if;

    wait;
  end process;

end architecture;
