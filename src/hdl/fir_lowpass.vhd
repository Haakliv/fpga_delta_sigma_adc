-- ************************************************************************
-- Simple FIR Low-Pass Filter
-- Final filtering stage after decimation
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

entity fir_lowpass is
  generic (
    INPUT_WIDTH  : positive := 16;
    OUTPUT_WIDTH : positive := 16
  );
  port (
    clk       : in  std_logic;
    reset     : in  rst_t;
    data_in   : in  std_logic_vector(INPUT_WIDTH - 1 downto 0);
    valid_in  : in  std_logic;
    data_out  : out std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
    valid_out : out std_logic
  );
end entity;

architecture rtl of fir_lowpass is

  -- Simple 5-tap symmetric FIR filter coefficients
  -- Normalized to avoid scaling issues: [1, 4, 6, 4, 1] / 16
  type delay_array_t is array (0 to 4) of signed(INPUT_WIDTH - 1 downto 0);
  signal delay_line : delay_array_t := (others => (others => '0'));

  signal output_reg : signed(OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_reg  : std_logic                         := '0';

begin

  process (clk)
    variable sum : signed(INPUT_WIDTH + 3 downto 0); -- Extra bits for sum
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        delay_line <= (others => (others => '0'));
        output_reg <= (others => '0');
        valid_reg <= '0';
      elsif valid_in = '1' then
        -- Shift delay line
        for i in 4 downto 1 loop
          delay_line(i) <= delay_line(i - 1);
        end loop;
        delay_line(0) <= signed(data_in);

        -- Simple convolution: [1, 4, 6, 4, 1] / 16
        sum := resize(delay_line(4), INPUT_WIDTH + 4) + -- x[n-4] * 1
          resize(delay_line(3), INPUT_WIDTH + 4) * 4 + -- x[n-3] * 4  
          resize(delay_line(2), INPUT_WIDTH + 4) * 6 + -- x[n-2] * 6
          resize(delay_line(1), INPUT_WIDTH + 4) * 4 + -- x[n-1] * 4
          resize(delay_line(0), INPUT_WIDTH + 4); -- x[n] * 1

        -- Divide by 16 (shift right by 4) and resize
        output_reg <= resize(shift_right(sum, 4), OUTPUT_WIDTH);
        valid_reg <= '1';
      else
        valid_reg <= '0';
      end if;
    end if;
  end process;

  data_out  <= std_logic_vector(output_reg);
  valid_out <= valid_reg;

end architecture;
