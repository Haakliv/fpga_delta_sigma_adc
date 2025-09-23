library ieee;
use ieee.std_logic_1164.all;

package clk_rst_pkg is                  -- Global active-high reset type
  subtype  T_RST_T      is std_logic;
  constant C_RST_ACTIVE : std_logic := '1';

  -- Clock generation procedure for testbenches
  procedure clk_gen(signal clk : out std_logic; constant period : time);

end package;

package body clk_rst_pkg is

  -- Clock generation procedure for testbenches
  procedure clk_gen(signal clk : out std_logic; constant period : time) is
  begin
    clk <= '0';
    wait for period / 2;
    clk <= '1';
    wait for period / 2;
  end procedure;

end package body;
