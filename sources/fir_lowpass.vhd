-- ************************************************************************
-- Anti-Aliasing FIR Low-Pass Filter
-- Final filtering stage after CIC decimation and sinc^3 equalization
-- 63-tap symmetric linear-phase FIR (Type I)
-- Coefficients: Kaiser window (β=6.98), Fc=12.5kHz, Fs=97.656kHz
-- Q1.15 format (16-bit signed), DC gain = 1.0
-- Passband: DC-12.5kHz, designed for burst mode (50MHz/512 = 97.656kHz)
--
-- Architecture:
--   - 5-stage pipeline for timing closure at 100 MHz
--   - Stage 1: Symmetric pre-addition (31 adders)
--   - Stage 2: Multiplication (32 multipliers: 31 pairs + 1 center)
--   - Stage 3a/3b: Balanced binary accumulation tree (registered levels)
--   - Stage 4: Final accumulation
--   - Stage 5: Q1.15 scaling and saturation
-- Total latency: 5 clock cycles
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dsp_utils_pkg.all;

entity fir_lowpass is
  generic(
    GC_INPUT_WIDTH  : positive := 16;
    GC_OUTPUT_WIDTH : positive := 16
  );
  port(
    clk       : in  std_logic;
    reset     : in  std_logic;
    data_in   : in  std_logic_vector(GC_INPUT_WIDTH - 1 downto 0);
    valid_in  : in  std_logic;
    data_out  : out std_logic_vector(GC_OUTPUT_WIDTH - 1 downto 0);
    valid_out : out std_logic
  );
end entity;

architecture rtl of fir_lowpass is

  -- Coefficient width (Q1.15 fixed-point format)
  constant C_COEF_WIDTH : positive := 16;

  -- Accumulator needs headroom: (16+1)-bit sum x 16-bit coeff = 33 bits
  -- Sum of 63 products needs log2(63) = 6 more bits -> 33+6 = 39 bits total
  constant C_ACC_WIDTH : natural := GC_INPUT_WIDTH + C_COEF_WIDTH + 1 + 6;

  -- Delay line for 63 taps
  type   T_DELAY_ARRAY is array (0 to 62) of signed(GC_INPUT_WIDTH - 1 downto 0);
  signal delay_line    : T_DELAY_ARRAY := (others => (others => '0'));

  -- Coefficient ROM (Q1.15 format, only first half due to symmetry)
  -- Sum = 32768 for DC gain = 1.0
  type T_COEFF_ARRAY is array (0 to 31) of signed(C_COEF_WIDTH - 1 downto 0);
  -- Coefficients designed for Fs=390.625kHz (50MHz/128), Fc=50kHz
  constant C_COEFFS : T_COEFF_ARRAY := (
    0  => to_signed(0, 16),             -- h[0] = h[62]
    1  => to_signed(-3, 16),            -- h[1] = h[61]
    2  => to_signed(-7, 16),            -- h[2] = h[60]
    3  => to_signed(-6, 16),            -- h[3] = h[59]
    4  => to_signed(4, 16),             -- h[4] = h[58]
    5  => to_signed(20, 16),            -- h[5] = h[57]
    6  => to_signed(30, 16),            -- h[6] = h[56]
    7  => to_signed(19, 16),            -- h[7] = h[55]
    8  => to_signed(-19, 16),           -- h[8] = h[54]
    9  => to_signed(-66, 16),           -- h[9] = h[53]
    10 => to_signed(-85, 16),           -- h[10] = h[52]
    11 => to_signed(-43, 16),           -- h[11] = h[51]
    12 => to_signed(60, 16),            -- h[12] = h[50]
    13 => to_signed(166, 16),           -- h[13] = h[49]
    14 => to_signed(192, 16),           -- h[14] = h[48]
    15 => to_signed(77, 16),            -- h[15] = h[47]
    16 => to_signed(-150, 16),          -- h[16] = h[46]
    17 => to_signed(-360, 16),          -- h[17] = h[45]
    18 => to_signed(-380, 16),          -- h[18] = h[44]
    19 => to_signed(-118, 16),          -- h[19] = h[43]
    20 => to_signed(341, 16),           -- h[20] = h[42]
    21 => to_signed(726, 16),           -- h[21] = h[41]
    22 => to_signed(717, 16),           -- h[22] = h[40]
    23 => to_signed(157, 16),           -- h[23] = h[39]
    24 => to_signed(-767, 16),          -- h[24] = h[38]
    25 => to_signed(-1529, 16),         -- h[25] = h[37]
    26 => to_signed(-1477, 16),         -- h[26] = h[36]
    27 => to_signed(-186, 16),          -- h[27] = h[35]
    28 => to_signed(2247, 16),          -- h[28] = h[34]
    29 => to_signed(5142, 16),          -- h[29] = h[33]
    30 => to_signed(7488, 16),          -- h[30] = h[32]
    31 => to_signed(8388, 16)           -- h[31] (center tap)
  );

  -- Pipeline stage 1: Pre-addition (symmetric pairs)
  type   T_PREADD_ARRAY is array (0 to 31) of signed(GC_INPUT_WIDTH downto 0);
  signal preadd_reg     : T_PREADD_ARRAY := (others => (others => '0'));
  signal valid_stage1   : std_logic      := '0';

  -- Pipeline stage 2: Multiplication (with DSP block inference)
  type   T_PROD_ARRAY is array (0 to 31) of signed(GC_INPUT_WIDTH + C_COEF_WIDTH downto 0);
  signal prod_reg     : T_PROD_ARRAY := (others => (others => '0'));
  signal valid_stage2 : std_logic    := '0';

  -- Pipeline stage 3: Partial accumulation (first two levels registered for timing)
  type   T_ACC_LEVEL1   is array (0 to 15) of signed(C_ACC_WIDTH - 1 downto 0);
  type   T_ACC_LEVEL2   is array (0 to 7) of signed(C_ACC_WIDTH - 1 downto 0);
  signal acc_level1_reg : T_ACC_LEVEL1 := (others => (others => '0'));
  signal acc_level2_reg : T_ACC_LEVEL2 := (others => (others => '0'));
  signal valid_stage3a  : std_logic    := '0';
  signal valid_stage3b  : std_logic    := '0';

  -- Pipeline stage 4: Final accumulation
  signal acc_reg      : signed(C_ACC_WIDTH - 1 downto 0) := (others => '0');
  signal valid_stage4 : std_logic                        := '0';

  -- Pipeline stage 5: Scaling + saturation
  signal output_reg : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_reg  : std_logic                            := '0';

begin

  -- ========================================================================
  -- Pipeline Stage 1: Delay line shift + Symmetric pre-addition
  -- ========================================================================
  p_stage1_preadd : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        delay_line   <= (others => (others => '0'));
        preadd_reg   <= (others => (others => '0'));
        valid_stage1 <= '0';
      elsif valid_in = '1' then
        -- Shift delay line
        for i in 62 downto 1 loop
          delay_line(i) <= delay_line(i - 1);
        end loop;
        delay_line(0) <= signed(data_in);

        -- Pre-addition for symmetric pairs (31 pairs)
        for i in 0 to 30 loop
          preadd_reg(i) <= resize(delay_line(i), GC_INPUT_WIDTH + 1) + resize(delay_line(62 - i), GC_INPUT_WIDTH + 1);
        end loop;

        -- Center tap (no pair, just pass through with sign extension)
        preadd_reg(31) <= resize(delay_line(31), GC_INPUT_WIDTH + 1);

        valid_stage1 <= '1';
      else
        valid_stage1 <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 2: Multiplication
  -- ========================================================================
  p_stage2_multiply : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        prod_reg     <= (others => (others => '0'));
        valid_stage2 <= '0';
      elsif valid_stage1 = '1' then
        -- Multiply each pre-added sum by its coefficient
        for i in 0 to 31 loop
          prod_reg(i) <= preadd_reg(i) * C_COEFFS(i);
        end loop;

        valid_stage2 <= '1';
      else
        valid_stage2 <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 3a: First level of accumulation tree (registered for timing)
  -- ========================================================================
  -- Level 1: 32 products -> 16 sums
  p_stage3a_accumulate : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        acc_level1_reg <= (others => (others => '0'));
        valid_stage3a  <= '0';
      elsif valid_stage2 = '1' then
        -- Level 1: 16 pairs (32->16)
        for i in 0 to 15 loop
          acc_level1_reg(i) <= resize(prod_reg(2 * i), C_ACC_WIDTH) + resize(prod_reg(2 * i + 1), C_ACC_WIDTH);
        end loop;

        valid_stage3a <= '1';
      else
        valid_stage3a <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 3b: Second level of accumulation tree (registered)
  -- ========================================================================
  p_stage3b_accumulate : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        acc_level2_reg <= (others => (others => '0'));
        valid_stage3b  <= '0';
      elsif valid_stage3a = '1' then
        -- Level 2: 8 pairs (16->8)
        for i in 0 to 7 loop
          acc_level2_reg(i) <= acc_level1_reg(2 * i) + acc_level1_reg(2 * i + 1);
        end loop;

        valid_stage3b <= '1';
      else
        valid_stage3b <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 4: Final accumulation (remaining 3 levels)
  -- ========================================================================
  -- Level 3: 8 sums -> 4 sums
  -- Level 4: 4 sums -> 2 sums
  -- Level 5: 2 sums -> 1 final sum
  p_stage4_accumulate : process(clk)
    type     T_LEVEL3 is array (0 to 3) of signed(C_ACC_WIDTH - 1 downto 0);
    variable v_level3 : T_LEVEL3;
    type     T_LEVEL4 is array (0 to 1) of signed(C_ACC_WIDTH - 1 downto 0);
    variable v_level4 : T_LEVEL4;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        acc_reg      <= (others => '0');
        valid_stage4 <= '0';
      elsif valid_stage3b = '1' then
        -- Level 3: 4 pairs (8->4)
        for i in 0 to 3 loop
          v_level3(i) := acc_level2_reg(2 * i) + acc_level2_reg(2 * i + 1);
        end loop;

        -- Level 4: 2 pairs (4->2)
        for i in 0 to 1 loop
          v_level4(i) := v_level3(2 * i) + v_level3(2 * i + 1);
        end loop;

        -- Level 5: Final sum (2->1)
        acc_reg      <= v_level4(0) + v_level4(1);
        valid_stage4 <= '1';
      else
        valid_stage4 <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 5: Q1.15 scaling and saturation
  -- ========================================================================
  p_stage5_scale : process(clk)
    constant C_Q15_SHIFT   : natural                          := 15; -- Q1.15 format shift
    constant C_ROUND_CONST : signed(C_ACC_WIDTH - 1 downto 0) := to_signed(2 ** (C_Q15_SHIFT - 1), C_ACC_WIDTH); -- 0.5 LSB
    variable v_scaled      : signed(C_ACC_WIDTH - 1 downto 0);
  begin
    if rising_edge(clk) then
      if reset = '1' then
        output_reg <= (others => '0');
        valid_reg  <= '0';
      elsif valid_stage4 = '1' then
        -- Scale down by 2^15 (Q1.15 format) with rounding
        v_scaled   := acc_reg + C_ROUND_CONST;
        output_reg <= saturate(shift_right(v_scaled, C_Q15_SHIFT), GC_OUTPUT_WIDTH);
        valid_reg  <= '1';
      else
        valid_reg <= '0';
      end if;
    end if;
  end process;

  data_out  <= std_logic_vector(output_reg);
  valid_out <= valid_reg;

end architecture;
