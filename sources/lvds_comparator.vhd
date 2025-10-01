-- ************************************************************************
-- LVDS Comparator Input Wrapper
-- For true differential LVDS: Quartus I/O buffer handles P vs N comparison
-- This module optionally adds majority voting for glitch suppression
-- ************************************************************************
library ieee;
use ieee.std_logic_1164.all;

use work.clk_rst_pkg.all;

entity lvds_comparator is
  generic(
    GC_ENABLE_MAJORITY : boolean := true -- Enable 3-tap majority filter (adds latency for noise immunity)
  );
  port(
    clk        : in  std_logic;
    reset      : in  T_RST_T;
    lvds_in    : in  std_logic;         -- Already-compared bit from LVDS I/O buffer
    bit_stream : out std_logic
  );
end entity;

architecture rtl of lvds_comparator is
  signal maj_reg      : std_logic_vector(2 downto 0) := (others => '0');
  signal majority_out : std_logic                    := '0';
begin

  -- Optional 3-tap majority filter on LVDS input
  -- Note: This adds latency - disable for minimum loop delay in sigma-delta feedback
  g_majority_gen : if GC_ENABLE_MAJORITY generate
    p_majority : process(clk)
      variable v_sum : integer range 0 to 3;
    begin
      if rising_edge(clk) then
        if reset = C_RST_ACTIVE then
          maj_reg      <= (others => '0');
          majority_out <= '0';
        else
          -- shift register of lvds_in
          maj_reg <= maj_reg(1 downto 0) & lvds_in;

          -- majority of previous 3 samples (maj_reg before update)
          v_sum        := 0;
          for i in 0 to 2 loop
            if maj_reg(i) = '1' then
              v_sum := v_sum + 1;
            end if;
          end loop;
          majority_out <= '1' when v_sum >= 2 else '0';
        end if;
      end if;
    end process;

    bit_stream <= majority_out;
  end generate;

  g_no_majority : if not GC_ENABLE_MAJORITY generate
    bit_stream <= lvds_in;
  end generate;

end architecture;
