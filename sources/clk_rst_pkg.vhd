library ieee;
use ieee.std_logic_1164.all;

package clk_rst_pkg is
  subtype  T_RST_T      is std_logic;
  constant C_RST_ACTIVE : std_logic := '1';

end package;

library ieee;
use ieee.std_logic_1164.all;

entity reset_synchronizer is
  generic(
    GC_ACTIVE_LOW : boolean := false
  );
  port(
    clk       : in  std_logic;
    async_rst : in  std_logic;
    sync_rst  : out std_logic
  );
end entity reset_synchronizer;

architecture rtl of reset_synchronizer is
  signal sync0, sync1, sync2, sync3 : std_logic := '0';
begin

  g_active_low : if GC_ACTIVE_LOW generate
    p_sync : process(clk)
    begin
      if rising_edge(clk) then
        -- Pure synchronous scalar shift register - NO reset clause
        -- Power-on initialization handles reset state
        sync0 <= async_rst;
        sync1 <= sync0;
        sync2 <= sync1;
        sync3 <= sync2;
      end if;
    end process;

    sync_rst <= sync3;                  -- Use last stage (4th FF) as output
  end generate;

  g_active_high : if not GC_ACTIVE_LOW generate
    p_sync : process(clk)
    begin
      if rising_edge(clk) then
        sync0 <= async_rst;
        sync1 <= sync0;
        sync2 <= sync1;
        sync3 <= sync2;
      end if;
    end process;

    sync_rst <= sync3;
  end generate;

end architecture rtl;
