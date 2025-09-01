library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

package clk_rst_pkg is

  -- Global active-high reset type
  subtype rst_t is std_logic;
  constant RST_ACTIVE : std_logic := '1';

  -- Synchronizer depth
  constant SYNC_STAGES : natural := 2;

  -- Clock generation procedure for testbenches
  procedure clk_gen(signal clk : out std_logic; constant period : time);

  -- Simple synchronizer entity declaration
  component reset_sync is
    generic (
      STAGES : positive := SYNC_STAGES
    );
    port (
      clk  : in  std_logic;
      arst : in  std_logic; -- async reset in, active high
      srst : out std_logic  -- sync reset out, active high
    );
  end component;

end package;

package body clk_rst_pkg is

  -- Clock generation procedure for testbenches
  procedure clk_gen(signal clk : out std_logic; constant period : time) is
  begin
    clk <= '0';
    wait for period/2;
    clk <= '1';
    wait for period/2;
  end procedure;

end package body;
