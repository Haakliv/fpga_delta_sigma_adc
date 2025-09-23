-- ************************************************************************
-- LVDS/Comparator Input Wrapper (Intel LVDS + portable sim fallback)
-- ************************************************************************
library ieee;
use ieee.std_logic_1164.all;

use work.clk_rst_pkg.all;

library altera_mf;
use altera_mf.altera_mf_components.all;

entity lvds_comparator is
  generic(
    GC_ENABLE_MAJORITY : boolean := true; -- Enable 3-tap majority filter
    GC_USE_INTEL_LVDS  : boolean := true -- true: instantiate altlvds_rx, false: behavioral compare for sim
  );
  port(
    clk        : in  std_logic;
    reset      : in  T_RST_T;
    lvds_p     : in  std_logic;         -- LVDS positive
    lvds_n     : in  std_logic;         -- LVDS negative (paired at IO; unused in altlvds_rx inst)
    bit_stream : out std_logic
  );
end entity;

architecture rtl of lvds_comparator is
  -- Intel/Altera differential input buffer (note: no rx_inb on this entity). Warnings suppressed due to being a vendor primitive
  component altlvds_rx                  -- @suppress "Component declaration is not equal to its matching entity"
    generic(
      number_of_channels     : natural := 1; -- @suppress "Naming convention violation: generic name should match pattern '^GC_[A-Z]([A-Z0-9]|(_(?!_)))*'"
      deserialization_factor : natural := 1; -- @suppress "Naming convention violation: generic name should match pattern '^GC_[A-Z]([A-Z0-9]|(_(?!_)))*'"
      data_rate              : string  := "UNUSED"; -- @suppress "Naming convention violation: generic name should match pattern '^GC_[A-Z]([A-Z0-9]|(_(?!_)))*'"
      intended_device_family : string  := "Agilex 5" -- @suppress "Naming convention violation: generic name should match pattern '^GC_[A-Z]([A-Z0-9]|(_(?!_)))*'"
    );
    port(
      rx_in      : in  std_logic_vector(0 downto 0);
      rx_out     : out std_logic_vector(0 downto 0);
      rx_inclock : in  std_logic
    );
  end component;

  signal comp_out     : std_logic;
  signal lvds_rx_out  : std_logic_vector(0 downto 0);
  signal sync_reg     : std_logic_vector(1 downto 0) := (others => '0');
  signal maj_reg      : std_logic_vector(2 downto 0) := (others => '0');
  signal majority_out : std_logic                    := '0';
begin

  g_use_intel : if GC_USE_INTEL_LVDS generate
    i_lvds_rx : altlvds_rx
      generic map(
        number_of_channels     => 1,
        deserialization_factor => 1,
        data_rate              => "UNUSED",
        intended_device_family => "Agilex 5"
      )
      port map(
        rx_in(0)   => lvds_p,           -- Only 'p' connects here; the 'n' pin is paired by IO assignment
        rx_out     => lvds_rx_out,
        rx_inclock => clk
      );
    comp_out <= lvds_rx_out(0);
  end generate;

  g_sim_fallback : if not GC_USE_INTEL_LVDS generate
    -- Simple behavioral comparator for simulation
    -- Works with your TB driving lvds_p = ideal, lvds_n = not ideal
    comp_out <= '1' when (lvds_p = '1' and lvds_n = '0') else
                '0' when (lvds_p = '0' and lvds_n = '1') else
                lvds_p;                 -- tie-break
  end generate;

  -- 2-FF synchronizer
  p_sync : process(clk)
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        sync_reg <= (others => '0');
      else
        sync_reg <= sync_reg(0) & comp_out;
      end if;
    end if;
  end process;

  -- Optional 3-tap majority on synchronized data
  g_majority_gen : if GC_ENABLE_MAJORITY generate
    p_majority : process(clk)
      variable v_sum : integer range 0 to 3;
    begin
      if rising_edge(clk) then
        if reset = C_RST_ACTIVE then
          maj_reg      <= (others => '0');
          majority_out <= '0';
        else
          -- shift register of sync_reg(1)
          maj_reg <= maj_reg(1 downto 0) & sync_reg(1);

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
    bit_stream <= sync_reg(1);
  end generate;

end architecture;
