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

architecture sim of tdc_quantizer_tb is
  component tdc_quantizer is
    generic(
      TDC_BITS     : positive := 8;
      COUNTER_BITS : positive := 16
    );
    port(
      clk       : in  std_logic;
      reset     : in  std_logic;
      tdc_start : in  std_logic;
      tdc_stop  : in  std_logic;
      enable    : in  std_logic;
      trigger   : in  std_logic;
      tdc_value : out std_logic_vector(TDC_BITS - 1 downto 0);
      tdc_valid : out std_logic;
      overflow  : out std_logic
    );
  end component;

  constant CLK_PERIOD : time     := 10 ns; -- 100 MHz
  constant TDC_BITS_C : positive := 8;

  signal clk       : std_logic := '0';
  signal reset     : std_logic := '1';
  signal tdc_start : std_logic := '0';
  signal tdc_stop  : std_logic := '0';
  signal enable    : std_logic := '0';
  signal trigger   : std_logic := '0';
  signal tdc_value : std_logic_vector(TDC_BITS_C - 1 downto 0);
  signal tdc_valid : std_logic;
  signal overflow  : std_logic;

  signal measurement_count : integer := 0;
  signal sim_finished      : boolean := false;
begin
  -- DUT
  dut : tdc_quantizer
    generic map(
      TDC_BITS     => TDC_BITS_C,
      COUNTER_BITS => 16
    )
    port map(
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

  -- VUnit runner
  main : process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("Running basic test for tdc_quantizer_tb");
        wait for CLK_PERIOD * 2000;     -- enough for all subtests
        check(measurement_count > 0, "At least one measurement should be captured");
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

  -- Stimulus
  stim : process
    procedure start_measure is
    begin
      wait until rising_edge(clk);
      trigger   <= '1';
      wait until rising_edge(clk);
      trigger   <= '0';
      tdc_start <= '1';
      wait until rising_edge(clk);
      tdc_start <= '0';
    end procedure;

    procedure stop_measure is
    begin
      wait until rising_edge(clk);
      tdc_stop <= '1';
      wait until rising_edge(clk);
      tdc_stop <= '0';
    end procedure;

  begin
    wait until reset = '0';
    wait for CLK_PERIOD * 2;

    report "Starting TDC Quantizer Test" severity note;
    enable <= '1';
    wait for CLK_PERIOD * 2;

    -- 1) 10 cycles
    report "Test 1: 10 clock cycles" severity note;
    start_measure;
    wait for CLK_PERIOD * 10;
    stop_measure;
    wait for CLK_PERIOD * 5;

    -- 2) 25 cycles
    report "Test 2: 25 clock cycles" severity note;
    start_measure;
    wait for CLK_PERIOD * 25;
    stop_measure;
    wait for CLK_PERIOD * 5;

    -- 3) 3 cycles
    report "Test 3: 3 clock cycles" severity note;
    start_measure;
    wait for CLK_PERIOD * 3;
    stop_measure;
    wait for CLK_PERIOD * 5;

    -- 4) Near max (250 cycles @ 8 bits)
    report "Test 4: ~250 cycles" severity note;
    start_measure;
    wait for CLK_PERIOD * 250;
    stop_measure;
    wait for CLK_PERIOD * 5;

    -- 5) Overflow (300 cycles)
    report "Test 5: overflow (~300 cycles)" severity note;
    start_measure;
    wait for CLK_PERIOD * 300;
    stop_measure;
    wait for CLK_PERIOD * 5;

    -- 6) Rapid sequence (5 shots, 5..13 cycles)
    report "Test 6: rapid sequence" severity note;
    for i in 1 to 5 loop
      start_measure;
      wait for CLK_PERIOD * (5 + 2 * i);
      stop_measure;
      wait for CLK_PERIOD * 2;
    end loop;

    -- 7) Simple parasitic sweep
    report "Test 7: parasitic sweep" severity note;
    for v in 1 to 5 loop
      start_measure;
      case v is
        when 1      => wait for CLK_PERIOD * 46;
        when 2      => wait for CLK_PERIOD * 42;
        when 3      => wait for CLK_PERIOD * 38;
        when 4      => wait for CLK_PERIOD * 34;
        when others => wait for CLK_PERIOD * 30;
      end case;
      stop_measure;
      wait for CLK_PERIOD * 2;
    end loop;

    report "TDC Quantizer Test Complete" severity note;
    wait;
  end process;

  -- Monitor (valid is 1-cycle pulse)
  monitor : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        measurement_count <= 0;
      elsif tdc_valid = '1' then
        measurement_count <= measurement_count + 1;
        if overflow = '1' then
          report "Meas " & integer'image(measurement_count) & ": TDC=" & integer'image(to_integer(unsigned(tdc_value))) & " (0x" & to_hstring(tdc_value) & ") OVERFLOW" severity note;
        else
          report "Meas " & integer'image(measurement_count) & ": TDC=" & integer'image(to_integer(unsigned(tdc_value))) & " (0x" & to_hstring(tdc_value) & ")" severity note;
        end if;
      end if;
    end if;
  end process;

  -- Quick stats
  stats : process
  begin
    wait for CLK_PERIOD * 10;
    report "=== Test Statistics ===" severity note;
    report "TDC bits: " & integer'image(TDC_BITS_C) severity note;
    report "Clk period: " & time'image(CLK_PERIOD) severity note;
    wait;
  end process;

end architecture;
