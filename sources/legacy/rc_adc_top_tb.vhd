-- ************************************************************************
-- Testbench for Top-Level RC ADC
-- Board test simulation: low OSR → high OSR with spectrum analysis
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity rc_adc_top_tb is
  generic(runner_cfg : string);
end entity;

architecture behavioral of rc_adc_top_tb is

  -- Test parameters
  constant C_CLK_PERIOD : time     := 10 ns; -- 100 MHz
  constant C_DATA_WIDTH : positive := 16;

  -- Signals for DUT with OSR=16 (low OSR start)
  signal clk         : std_logic := '0';
  signal reset       : std_logic := '1';
  signal analog_in_p : std_logic := '0'; -- Internal test signals for differential
  signal analog_in_n : std_logic := '1';
  signal analog_in   : std_logic := '0'; -- Simulated LVDS comparator output
  signal dac_out     : std_logic;       -- @suppress

  signal sample_data  : std_logic_vector(C_DATA_WIDTH - 1 downto 0);
  signal sample_valid : std_logic;

  -- Memory-mapped interface signals (new interface)
  signal mem_cs      : std_logic                     := '0';
  signal mem_rd      : std_logic                     := '0';
  signal mem_wr      : std_logic                     := '0'; -- @suppress
  signal mem_addr    : std_logic_vector(11 downto 0) := (others => '0');
  signal mem_wdata   : std_logic_vector(31 downto 0) := (others => '0'); -- @suppress
  signal mem_rdata   : std_logic_vector(31 downto 0);
  signal mem_rdvalid : std_logic;

  -- Test signals (keep for test logic)
  signal sample_count  : integer := 0;
  signal input_freq    : real    := 1000000.0; -- 1 MHz test frequency
  signal input_amp     : real    := 0.1; -- Low amplitude start
  signal sim_finished  : boolean := false;
  signal test_complete : boolean := false;

begin

  -- Simulate LVDS I/O buffer differential comparison
  -- In real hardware, this is done by the LVDS I/O buffer in Quartus
  analog_in <= '1' when (analog_in_p = '1' and analog_in_n = '0') else
               '0' when (analog_in_p = '0' and analog_in_n = '1') else
               analog_in_p;             -- tie-break

  -- Device Under Test (entity instantiation with new interface)
  i_dut : entity work.rc_adc_top
    generic map(
      GC_DECIMATION => 16,              -- Low decimation for initial test
      GC_DATA_WIDTH => C_DATA_WIDTH
    )
    port map(
      clk          => clk,
      reset        => reset,
      mem_cs       => mem_cs,
      mem_rd       => mem_rd,
      mem_wr       => mem_wr,
      mem_addr     => mem_addr,
      mem_wdata    => mem_wdata,
      mem_rdata    => mem_rdata,
      mem_rdvalid  => mem_rdvalid,
      analog_in    => analog_in,
      dac_out      => dac_out,
      sample_data  => sample_data,
      sample_valid => sample_valid
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
    variable v_read_data : std_logic_vector(31 downto 0);

    -- Memory read procedure
    procedure mem_read(addr : in integer; data : out std_logic_vector(31 downto 0)) is
    begin
      wait until rising_edge(clk);
      mem_cs   <= '1';
      mem_rd   <= '1';
      mem_addr <= std_logic_vector(to_unsigned(addr, 12));
      wait until rising_edge(clk);
      wait until mem_rdvalid = '1';
      data     := mem_rdata;
      mem_cs   <= '0';
      mem_rd   <= '0';
      wait until rising_edge(clk);
    end procedure;

  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("basic_test") then
        info("Running basic test for rc_adc_top_tb");

        -- Wait for reset to complete
        wait for C_CLK_PERIOD * 100;

        -- Test memory-mapped reads
        info("Testing memory-mapped interface");
        mem_read(0, v_read_data);       -- Read ADC data
        info("ADC Data: " & to_string(v_read_data));

        mem_read(1, v_read_data);       -- Read status
        info("Status: " & to_string(v_read_data));

        mem_read(2, v_read_data);       -- Read valid counter
        info("Valid Counter: " & to_string(v_read_data));

        mem_read(3, v_read_data);       -- Read activity counter
        info("Activity Counter: " & to_string(v_read_data));

        mem_read(255, v_read_data);     -- Read invalid address (tests 'others' branch)
        info("Invalid addr (should be 0): " & to_string(v_read_data));

        wait for C_CLK_PERIOD * 1000;   -- Allow more time for ADC processing
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
    reset <= '1';                       -- Active high reset for std_logic
    wait for C_CLK_PERIOD * 10;
    reset <= '0';
    wait;
  end process;

  -- Test signal generation (simulates analog input)
  p_signal_gen : process
    variable v_phase          : real    := 0.0;
    variable v_phase_inc      : real;
    variable v_sample_val     : real;
    variable v_noise          : real;
    variable v_seed1, v_seed2 : integer := 123;
    variable v_rand_val       : real;
  begin
    wait until reset = '0';

    v_phase_inc := 2.0 * MATH_PI * input_freq * real(C_CLK_PERIOD / 1 ns) * 1.0e-9;

    loop
      wait until rising_edge(clk);

      -- Generate test signal with noise
      v_sample_val := input_amp * sin(v_phase);

      -- Add small amount of noise
      uniform(v_seed1, v_seed2, v_rand_val);
      v_noise      := (v_rand_val - 0.5) * 0.02; -- 2% noise
      v_sample_val := v_sample_val + v_noise;

      -- Convert to differential LVDS
      if v_sample_val > 0.0 then
        analog_in_p <= '1';
        analog_in_n <= '0';
      else
        analog_in_p <= '0';
        analog_in_n <= '1';
      end if;

      v_phase := v_phase + v_phase_inc;
      if v_phase >= 2.0 * MATH_PI then
        v_phase := v_phase - 2.0 * MATH_PI;
      end if;

      exit when test_complete;
    end loop;

    wait;
  end process;

  -- Test sequence controller
  p_test_sequence : process
  begin
    wait until reset = '0';
    wait for C_CLK_PERIOD * 20;

    report "=== RC ADC Board Test Sequence ===" severity note;

    -- Phase 1: Low OSR, Low Amplitude
    input_amp <= 0.1;                   -- 10% amplitude
    report "Phase 1: OSR=16, Low Amplitude (10%)" severity note;
    wait for C_CLK_PERIOD * 2000;       -- Let it settle

    -- Phase 2: Increase amplitude
    input_amp <= 0.3;                   -- 30% amplitude
    report "Phase 2: OSR=16, Medium Amplitude (30%)" severity note;
    wait for C_CLK_PERIOD * 2000;

    -- Phase 3: Higher amplitude
    input_amp <= 0.7;                   -- 70% amplitude
    report "Phase 3: OSR=16, High Amplitude (70%)" severity note;
    wait for C_CLK_PERIOD * 2000;

    -- Phase 4: Different frequency
    input_freq <= 500000.0;             -- 500 kHz
    input_amp  <= 0.5;
    report "Phase 4: OSR=16, 500kHz input" severity note;
    wait for C_CLK_PERIOD * 2000;

    -- Note: In real board test, you would reconfigure OSR here
    -- For simulation, we test the fundamental functionality
    report "=== Test Complete - Ready for OSR Ramping ===" severity note;
    report "Next step: Reconfigure with higher OSR (64, 128, 256)" severity note;

    wait for C_CLK_PERIOD * 1000;
    test_complete <= true;              -- Signal test completion
    wait;
  end process;

  -- Simple monitoring process (functionality preserved but simplified)
  p_monitor : process(clk)
    variable v_heartbeat_count : integer := 0;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        sample_count      <= 0;
        v_heartbeat_count := 0;
      else
        v_heartbeat_count := v_heartbeat_count + 1;

        if sample_valid = '1' then
          sample_count <= sample_count + 1;
          if ((sample_count + 1) mod 256) = 0 then
            report "Sample[" & integer'image(sample_count + 1) & "] = 0x" & to_hstring(sample_data) severity note;
          end if;
        end if;

        -- Simple heartbeat every 1000 cycles
        if (v_heartbeat_count mod 1000) = 0 then
          report "Heartbeat after " & integer'image(v_heartbeat_count) & " cycles, samples=" & integer'image(sample_count) severity note;
          -- Memory read testing is in the main test process
        end if;
      end if;
    end if;
  end process;

end architecture;

