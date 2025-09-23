-- ************************************************************************
-- Simple FIR Low-Pass Filter
-- Final filtering stage after decimation
-- 5-tap symmetric FIR with coeffs [1, 4, 6, 4, 1] / 16
-- Width-safe: all intermediate math uses a single accumulator width.
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity fir_lowpass is
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

architecture rtl of fir_lowpass is
  -- Delay line for 5 taps
  type   T_DELAY_ARRAY is array (0 to 4) of signed(GC_INPUT_WIDTH - 1 downto 0);
  signal delay_line    : T_DELAY_ARRAY := (others => (others => '0'));

  -- Output regs
  signal output_reg : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_reg  : std_logic                            := '0';
begin

  process(clk)
    -- Sum of coeffs is 16, so add 4 guard bits before >> 4
    constant C_ACC_W : natural := GC_INPUT_WIDTH + 4;
    variable v_acc   : signed(C_ACC_W - 1 downto 0);
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        delay_line <= (others => (others => '0'));
        output_reg <= (others => '0');
        valid_reg  <= '0';

      elsif valid_in = '1' then
        -- Shift delay line
        for i in 4 downto 1 loop
          delay_line(i) <= delay_line(i - 1);
        end loop;
        delay_line(0) <= signed(data_in);

        -- Convolution with [1,4,6,4,1] using shifts to avoid width growth
        v_acc := resize(delay_line(4), C_ACC_W) -- 1*x[n-4]
                 + shift_left(resize(delay_line(3), C_ACC_W), 2) -- 4*x[n-3]
                 + (shift_left(resize(delay_line(2), C_ACC_W), 2) -- 6*x[n-2] = 4*x + 2*x
                    + shift_left(resize(delay_line(2), C_ACC_W), 1)) + shift_left(resize(delay_line(1), C_ACC_W), 2) -- 4*x[n-1]
                 + resize(delay_line(0), C_ACC_W); -- 1*x[n]

        -- Normalize by 16 (>> 4). For round-to-nearest, add +8 before shifting.
        output_reg <= resize(shift_right(v_acc, 4), GC_OUTPUT_WIDTH);
        valid_reg  <= '1';

      else
        valid_reg <= '0';
      end if;
    end if;
  end process;

  data_out  <= std_logic_vector(output_reg);
  valid_out <= valid_reg;

end architecture;
