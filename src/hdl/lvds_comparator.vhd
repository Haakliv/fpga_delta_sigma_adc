-- ************************************************************************
-- LVDS/Comparator Input Wrapper
-- Minimal design: Input buffer → sync → optional majority filter → output
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

entity lvds_comparator is
  generic (
    ENABLE_MAJORITY : boolean := true  -- Enable 3-tap majority filter
  );
  port (
    clk           : in  std_logic;
    reset         : in  rst_t;
    lvds_p        : in  std_logic;  -- LVDS positive
    lvds_n        : in  std_logic;  -- LVDS negative  
    bit_stream    : out std_logic
  );
end entity;

architecture rtl of lvds_comparator is

  signal comp_out     : std_logic;
  signal sync_reg     : std_logic_vector(1 downto 0) := (others => '0');
  signal maj_reg      : std_logic_vector(2 downto 0) := (others => '0');
  signal majority_out : std_logic;

begin

  -- Simple differential comparator (in real FPGA use LVDS input buffer)
  comp_out <= lvds_p and not lvds_n;

  -- Synchronizer (2 flip-flops)
  sync_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        sync_reg <= (others => '0');
      else
        sync_reg <= sync_reg(0) & comp_out;
      end if;
    end if;
  end process;

  -- Optional 3-tap majority filter
  majority_gen : if ENABLE_MAJORITY generate
    majority_process : process(clk)
      variable sum : integer range 0 to 3;
    begin
      if rising_edge(clk) then
        if reset = RST_ACTIVE then
          maj_reg <= (others => '0');
        else
          maj_reg <= maj_reg(1 downto 0) & sync_reg(1);
          
          -- Count ones in 3-bit window
          sum := 0;
          for i in 0 to 2 loop
            if maj_reg(i) = '1' then
              sum := sum + 1;
            end if;
          end loop;
          
          -- Majority decision
          if sum >= 2 then
            majority_out <= '1';
          else
            majority_out <= '0';
          end if;
        end if;
      end if;
    end process;
    
    bit_stream <= majority_out;
    
  else generate
    bit_stream <= sync_reg(1);
  end generate;

end architecture rtl;
