library ieee;
use ieee.std_logic_1164.all;

package clk_rst_tb_pkg is

    procedure clk_gen(signal clk : out std_logic; constant period : time);

end package;

package body clk_rst_tb_pkg is

    procedure clk_gen(signal clk : out std_logic; constant period : time) is
    begin
        clk <= '0';
        wait for period / 2;
        clk <= '1';
        wait for period / 2;
    end procedure;

end package body;
