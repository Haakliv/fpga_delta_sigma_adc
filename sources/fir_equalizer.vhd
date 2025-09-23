-- ************************************************************************
-- Simple Decimator by 2 
-- Just decimates by 2 with simple averaging filter
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity fir_equalizer is
  generic(
    GC_INPUT_WIDTH  : positive := 16;
    GC_OUTPUT_WIDTH : positive := 16
  );
  port(
    clk       : in  std_logic;
    reset     : in  T_RST_T;
    data_in   : in  std_logic_vector(GC_INPUT_WIDTH - 1 downto 0);
    valid_in  : in  std_logic;
    data_out  : out std_logic_vector(GC_OUTPUT_WIDTH - 1 downto 0);
    valid_out : out std_logic
  );
end entity;

architecture rtl of fir_equalizer is

  signal prev_sample    : signed(GC_INPUT_WIDTH - 1 downto 0)  := (others => '0');
  signal current_sample : signed(GC_INPUT_WIDTH - 1 downto 0)  := (others => '0');
  signal decimation_cnt : std_logic                            := '0';
  signal output_reg     : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_reg      : std_logic                            := '0';

begin

  process(clk)
    variable v_sum : signed(GC_INPUT_WIDTH downto 0);
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        prev_sample    <= (others => '0');
        current_sample <= (others => '0');
        decimation_cnt <= '0';
        output_reg     <= (others => '0');
        valid_reg      <= '0';
      elsif valid_in = '1' then
        -- Store samples
        prev_sample    <= current_sample;
        current_sample <= signed(data_in);
        decimation_cnt <= not decimation_cnt;

        -- Output average every other sample
        if decimation_cnt = '1' then
          v_sum      := resize(prev_sample, GC_INPUT_WIDTH + 1) + resize(current_sample, GC_INPUT_WIDTH + 1);
          output_reg <= resize(shift_right(v_sum, 1), GC_OUTPUT_WIDTH);
          valid_reg  <= '1';
        else
          valid_reg <= '0';
        end if;
      else
        valid_reg <= '0';
      end if;
    end if;
  end process;

  data_out  <= std_logic_vector(output_reg);
  valid_out <= valid_reg;

end architecture;
