-- ************************************************************************
-- LVDS/Comparator Input Wrapper
-- Uses Intel/Altera differential input buffers for proper LVDS reception
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

-- Intel/Altera primitive library
library altera_mf;
  use altera_mf.altera_mf_components.all;

entity lvds_comparator is
  generic (
    ENABLE_MAJORITY : boolean := true -- Enable 3-tap majority filter
  );
  port (
    clk        : in  std_logic;
    reset      : in  rst_t;
    lvds_p     : in  std_logic; -- LVDS positive
    lvds_n     : in  std_logic; -- LVDS negative  
    bit_stream : out std_logic
  );
end entity;

architecture rtl of lvds_comparator is

  -- Intel/Altera differential input buffer component
  component altlvds_rx
    generic (
      number_of_channels     : natural := 1;
      deserialization_factor : natural := 1;
      data_rate              : string := "UNDEFINED";
      intended_device_family : string := "Agilex 5"
    );
    port (
      rx_in     : in  std_logic_vector(0 downto 0);
      rx_inb    : in  std_logic_vector(0 downto 0);
      rx_out    : out std_logic_vector(0 downto 0);
      rx_inclock: in  std_logic
    );
  end component;

  signal comp_out        : std_logic;
  signal lvds_rx_out     : std_logic_vector(0 downto 0);
  signal sync_reg        : std_logic_vector(1 downto 0) := (others => '0');
  signal maj_reg         : std_logic_vector(2 downto 0) := (others => '0');
  signal majority_out    : std_logic;

begin

  -- Intel/Altera LVDS differential input buffer
  lvds_rx_inst : altlvds_rx
    generic map (
      number_of_channels     => 1,
      deserialization_factor => 1,
      data_rate              => "UNDEFINED",
      intended_device_family => "Agilex 5"
    )
    port map (
      rx_in(0)    => lvds_p,
      rx_inb(0)   => lvds_n,
      rx_out      => lvds_rx_out,
      rx_inclock  => clk
    );

  comp_out <= lvds_rx_out(0);

  -- Synchronizer (2 flip-flops)
  sync_process: process (clk)
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
  majority_gen: if ENABLE_MAJORITY generate
    majority_process: process (clk)
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

end architecture;
