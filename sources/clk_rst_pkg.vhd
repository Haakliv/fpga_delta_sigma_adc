-- ************************************************************************
-- Clock/Reset Package
-- Synthesizable global types and constants for reset handling
-- Includes reset synchronizer entity for preventing metastability
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;

package clk_rst_pkg is
  -- Global active-high reset type
  subtype  T_RST_T      is std_logic;
  constant C_RST_ACTIVE : std_logic := '1';

end package;

-- ************************************************************************
-- Reset Synchronizer Entity Implementation
-- Note: Component declaration not needed - use entity instantiation
-- Example: i_sync : entity work.reset_synchronizer
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;

entity reset_synchronizer is
  generic(
    GC_ACTIVE_LOW : boolean := false    -- true = active-low reset, false = active-high reset
  );
  port(
    clk       : in  std_logic;          -- Clock to synchronize reset to
    async_rst : in  std_logic;          -- Asynchronous reset input
    sync_rst  : out std_logic           -- Synchronized reset output (same polarity as input)
  );
end entity reset_synchronizer;

architecture rtl of reset_synchronizer is

  -- Synthesis attributes for Intel/Altera FPGAs
  attribute altera_attribute : string;

  -- Scalar synchronizer chain instead of vector (avoids CDC-50006 "bus CDC" warning)
  signal sync0, sync1, sync2, sync3 : std_logic := '0';

  -- Intel recommended attributes for each synchronizer stage
  attribute altera_attribute of sync0 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";
  attribute altera_attribute of sync1 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";
  attribute altera_attribute of sync2 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";
  attribute altera_attribute of sync3 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";

begin

  -- Active-low reset variant (NO reset in process - pure shift register)
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

  -- Active-high reset variant (NO reset in process - pure shift register)
  g_active_high : if not GC_ACTIVE_LOW generate
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

end architecture rtl;
