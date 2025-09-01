library ieee;
  use ieee.std_logic_1164.all;

library work;
  use work.clk_rst_pkg.all;

entity dac_1_bit is
  port (clk     : in  std_logic;
        reset   : in  rst_t;
        data_in : in  std_logic;
        dac_out : out std_logic);
end entity;

architecture Behavioral of dac_1_bit is
begin
  process (clk, reset)
  begin
    if (reset = RST_ACTIVE) then
      dac_out <= '0';
    elsif (clk'event and clk = '1') then
      dac_out <= data_in;
    end if;
  end process;
end architecture;
