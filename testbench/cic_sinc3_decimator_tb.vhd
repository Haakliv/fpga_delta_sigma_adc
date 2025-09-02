library IEEE;
  use IEEE.STD_LOGIC_1164.all;
  use IEEE.NUMERIC_STD.all;

library work;
  use work.clk_rst_pkg.all;

entity cic_sinc3_decimator_tb is
end entity;

architecture Behavioral of cic_sinc3_decimator_tb is

  component cic_sinc3_decimator is
    generic (
      DECIMATION   : positive := 64;
      OUTPUT_WIDTH : positive := 16
    );
    port (
      clk      : in  std_logic;
      reset    : in  rst_t;
      data_in  : in  std_logic;
      data_out : out std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
      valid    : out std_logic
    );
  end component;

  signal clk      : std_logic := '0';
  signal reset    : rst_t     := RST_ACTIVE;
  signal data_in  : std_logic := '0';
  signal data_out : std_logic_vector(15 downto 0);
  signal valid    : std_logic;

  constant CLK_PERIOD : time := 10 ns;

begin

  -- DUT
  DUT: cic_sinc3_decimator
    generic map (
      DECIMATION   => 16,
      OUTPUT_WIDTH => 16
    )
    port map (
      clk      => clk,
      reset    => reset,
      data_in  => data_in,
      data_out => data_out,
      valid    => valid
    );

  -- Clock generation using package procedure
  clk_process: process
  begin
    while true loop
      clk_gen(clk, CLK_PERIOD);
    end loop;
  end process;

  -- Test
  process
  begin
    -- Reset
    reset <= RST_ACTIVE;
    wait for 100 ns;
    reset <= not RST_ACTIVE;

    -- Test with alternating pattern
    for i in 0 to 1000 loop
      data_in <= not data_in;
      wait for CLK_PERIOD;
    end loop;

    -- Test with all ones
    data_in <= '1';
    wait for 500 * CLK_PERIOD;

    -- Test with all zeros
    data_in <= '0';
    wait for 500 * CLK_PERIOD;

    wait;
  end process;

end architecture;
