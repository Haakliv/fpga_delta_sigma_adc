library ieee;
  use IEEE.std_logic_1164.all;

library work;
  use work.clk_rst_pkg.all;

entity dac_1_bit_tb is
end entity;

architecture testbench of dac_1_bit_tb is
  signal clk     : std_logic := '0';
  signal reset   : rst_t     := not RST_ACTIVE;
  signal data_in : std_logic := '0';
  signal dac_out : std_logic;

  component dac_1_bit is
    port (clk     : in  std_logic;
          reset   : in  rst_t;
          data_in : in  std_logic;
          dac_out : out std_logic);
  end component;

begin
  uut: dac_1_bit
    port map (
      clk     => clk,
      reset   => reset,
      data_in => data_in,
      dac_out => dac_out
    );

  clk_process: process
  begin
    while true loop
      clk_gen(clk, 20 ns); -- 20 ns period (same as before: 10ns low + 10ns high)
    end loop;
  end process;

  stimulus: process
  begin
    reset <= RST_ACTIVE;
    wait for 20 ns;
    reset <= not RST_ACTIVE;

    data_in <= '1';
    wait for 20 ns;

    data_in <= '0';
    wait for 20 ns;

    data_in <= '1';
    wait for 20 ns;

    wait;
  end process;

end architecture;
