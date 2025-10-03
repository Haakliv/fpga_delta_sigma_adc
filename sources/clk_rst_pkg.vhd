-- ************************************************************************
-- Clock/Reset Package
-- Synthesizable global types and constants for reset handling
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;

package clk_rst_pkg is
  -- Global active-high reset type
  subtype  T_RST_T      is std_logic;
  constant C_RST_ACTIVE : std_logic := '1';

end package;
