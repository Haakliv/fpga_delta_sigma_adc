library ieee;
use ieee.std_logic_1164.all;

-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity dac_1_bit is
  port(clk     : in  std_logic;
       reset   : in  T_RST_T;
       data_in : in  std_logic;
       dac_out : out std_logic);
end entity;

architecture behavioral of dac_1_bit is
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        dac_out <= '0';
      else
        dac_out <= data_in;
      end if;
    end if;
  end process;
end architecture;
