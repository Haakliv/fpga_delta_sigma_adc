library ieee;
use IEEE.std_logic_1164.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity dac_1_bit_tb is
  generic(GC_RUNNER_CFG : string);
end entity;

architecture behavioral of dac_1_bit_tb is
  signal clk          : std_logic := '0';
  signal reset        : std_logic := '0'; -- Changed to std_logic
  signal data_in      : std_logic := '0';
  signal dac_out      : std_logic;
  signal sim_finished : boolean   := false;

begin
  i_uut : entity work.dac_1_bit
    port map(
      clk     => clk,
      reset   => reset,
      data_in => data_in,
      dac_out => dac_out
    );

  p_clk : process
  begin
    while not sim_finished loop
      clk_gen(clk, 20 ns);              -- 20 ns period (same as before: 10ns low + 10ns high)
    end loop;
    wait;
  end process;

  p_stimulus : process
  begin
    test_runner_setup(runner, GC_RUNNER_CFG);

    while test_suite loop
      if run("test_dac_output") then
        reset <= '1';
        wait for 20 ns;
        reset <= '0';

        data_in <= '1';
        wait for 20 ns;
        check_equal(dac_out, '1', "DAC output should be '1' when data_in is '1'");

        data_in <= '0';
        wait for 20 ns;
        check_equal(dac_out, '0', "DAC output should be '0' when data_in is '0'");

        data_in <= '1';
        wait for 20 ns;
        check_equal(dac_out, '1', "DAC output should be '1' when data_in is '1' again");
      end if;
    end loop;

    test_runner_cleanup(runner);
    sim_finished <= true;
  end process;

end architecture;
