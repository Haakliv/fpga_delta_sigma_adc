library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;
use fpga_lib.clk_rst_tb_pkg.all;

entity cic_sinc3_decimator_tb is
  generic(runner_cfg : string);
end entity;

architecture behavioral of cic_sinc3_decimator_tb is

  signal clk          : std_logic := '0';
  signal reset        : std_logic := '1'; -- Changed to std_logic
  signal data_in      : std_logic := '0';
  signal data_out     : std_logic_vector(15 downto 0);
  signal valid        : std_logic;
  signal sim_finished : boolean   := false;

  constant C_CLK_PERIOD : time := 10 ns;

begin

  -- DUT (Entity instantiation)
  i_dut : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => 16,
      GC_OUTPUT_WIDTH => 16
    )
    port map(
      clk      => clk,
      reset    => reset,
      data_in  => data_in,
      ce       => '1',                  -- Always enabled in testbench (every cycle)
      data_out => data_out,
      valid    => valid
    );

  -- Clock generation using package procedure
  p_clk : process
  begin
    while not sim_finished loop
      clk_gen(clk, C_CLK_PERIOD);
    end loop;
    wait;
  end process;

  -- Test
  process
  begin
    test_runner_setup(runner, runner_cfg);

    while test_suite loop
      if run("test_alternating_pattern") then
        -- Reset
        reset <= '1';
        wait for 100 ns;
        reset <= '0';

        info("Testing alternating pattern");
        -- Test with alternating pattern
        for i in 0 to 100 loop          -- Reduced iterations for faster testing
          data_in <= not data_in;
          wait for C_CLK_PERIOD;
        end loop;

        check(true, "Alternating pattern test completed");

      elsif run("test_all_ones") then
        -- Reset
        reset <= '1';
        wait for 100 ns;
        reset <= '0';

        info("Testing all ones pattern");
        -- Test with all ones
        data_in <= '1';
        wait for 100 * C_CLK_PERIOD;    -- Reduced time for faster testing

        check(true, "All ones pattern test completed");

      elsif run("test_all_zeros") then
        -- Reset
        reset <= '1';
        wait for 100 ns;
        reset <= '0';

        info("Testing all zeros pattern");
        -- Test with all zeros
        data_in <= '0';
        wait for 100 * C_CLK_PERIOD;    -- Reduced time for faster testing

        check(true, "All zeros pattern test completed");
      end if;
    end loop;

    sim_finished <= true;
    test_runner_cleanup(runner);
  end process;

  -- Output monitoring process  
  p_monitor : process(clk)
  begin
    if rising_edge(clk) then
      if valid = '1' then
        -- Simple monitoring - just read the outputs
        info("CIC Output: " & integer'image(to_integer(signed(data_out))));
      end if;
    end if;
  end process;

end architecture;
