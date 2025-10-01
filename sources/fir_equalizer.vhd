-- ************************************************************************
-- CIC Sinc³ Equalizer (Droop Compensator)
-- Non-decimating FIR that flattens the sinc³ passband response
-- 
-- Design specs:
--   - Compensates sinc³ droop from CIC decimator
--   - 31-tap symmetric FIR (linear phase)
--   - Passband: DC to ~0.25×Fs (where Fs = fclk/OSR)
--   - Quantized to 16-bit coefficients for efficient implementation
--
-- For OSR=65536, fclk=100MHz: Fs=1526Hz, passband DC-380Hz
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.clk_rst_pkg.all;
use work.dsp_utils_pkg.all;

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

  -- 31-tap symmetric sinc³ inverse compensator
  -- Coefficients designed via firls to flatten CIC's sinc³ droop
  -- Quantized to Q1.15 format, normalized to DC gain = 1.0
  -- Symmetric: use only 16 unique values (center tap + 15 pairs)
  type T_COEF_ARRAY is array (0 to 15) of signed(15 downto 0);

  -- Sinc³ equalizer coefficients (approximate 1/sinc³(πf/Fs) in passband)
  -- Designed for passband 0-0.4×Fs with flat response (<0.1dB ripple)
  -- Sum of all 31 coefficients = 32768 (DC gain = 1.0 in Q1.15)
  constant C_COEF : T_COEF_ARRAY := (
    to_signed(316, 16),                 -- h[0] = h[30]
    to_signed(408, 16),                 -- h[1] = h[29]
    to_signed(-462, 16),                -- h[2] = h[28]
    to_signed(-989, 16),                -- h[3] = h[27]
    to_signed(152, 16),                 -- h[4] = h[26]
    to_signed(1401, 16),                -- h[5] = h[25]
    to_signed(143, 16),                 -- h[6] = h[24]
    to_signed(-2303, 16),               -- h[7] = h[23]
    to_signed(-1512, 16),               -- h[8] = h[22]
    to_signed(2427, 16),                -- h[9] = h[21]
    to_signed(2830, 16),                -- h[10] = h[20]
    to_signed(-3584, 16),               -- h[11] = h[19]
    to_signed(-8500, 16),               -- h[12] = h[18]
    to_signed(-1130, 16),               -- h[13] = h[17]
    to_signed(15223, 16),               -- h[14] = h[16]
    to_signed(23928, 16)                -- h[15] (center tap)
  );

  -- Delay line for 31 taps
  type   T_DELAY_ARRAY is array (0 to 30) of signed(GC_INPUT_WIDTH - 1 downto 0);
  signal delay_line    : T_DELAY_ARRAY := (others => (others => '0'));

  -- Output registers
  signal output_reg : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_reg  : std_logic                            := '0';

begin

  process(clk)
    -- Accumulator width: input + coef + log2(31) guard bits
    constant C_ACC_WIDTH : natural := GC_INPUT_WIDTH + 16 + 5;
    variable v_acc       : signed(C_ACC_WIDTH - 1 downto 0);
    variable v_prod      : signed(GC_INPUT_WIDTH + 16 downto 0); -- Sum has +1 bit, mult adds 16
    variable v_sum_pair  : signed(GC_INPUT_WIDTH downto 0);
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        delay_line <= (others => (others => '0'));
        output_reg <= (others => '0');
        valid_reg  <= '0';

      elsif valid_in = '1' then
        -- Shift delay line
        for i in 30 downto 1 loop
          delay_line(i) <= delay_line(i - 1);
        end loop;
        delay_line(0) <= signed(data_in);

        -- Symmetric FIR: Exploit symmetry to halve multiplications
        -- Center tap (h[15])
        v_acc := resize(delay_line(15) * C_COEF(15), C_ACC_WIDTH);

        -- Symmetric pairs: (h[i] × (x[i] + x[30-i]))
        for i in 0 to 14 loop
          v_sum_pair := resize(delay_line(i), GC_INPUT_WIDTH + 1) + resize(delay_line(30 - i), GC_INPUT_WIDTH + 1);
          v_prod     := v_sum_pair * C_COEF(i);
          v_acc      := v_acc + resize(v_prod, C_ACC_WIDTH);
        end loop;

        -- Scale down by 2^15 (Q1.15 format) with rounding
        v_acc      := v_acc + to_signed(16384, C_ACC_WIDTH); -- Round: add 0.5 LSB
        output_reg <= saturate(shift_right(v_acc, 15), GC_OUTPUT_WIDTH);
        valid_reg  <= '1';

      else
        valid_reg <= '0';
      end if;
    end if;
  end process;

  data_out  <= std_logic_vector(output_reg);
  valid_out <= valid_reg;

end architecture;
