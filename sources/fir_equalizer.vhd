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
-- Architecture:
--   - 4-stage pipeline for timing closure at 100 MHz
--   - Stage 1: Symmetric pre-addition (15 adders)
--   - Stage 2: Multiplication (15 multipliers + center tap)
--   - Stage 3: Balanced binary accumulation tree (log2(16) = 4 levels)
--   - Stage 4: Q1.15 scaling and saturation
--
-- For OSR=65536, fclk=100MHz: Fs=1526Hz, passband DC-380Hz
-- Total latency: 4 clock cycles
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dsp_utils_pkg.all;

entity fir_equalizer is
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

architecture rtl of fir_equalizer is

  -- Coefficient width (Q1.15 fixed-point format)
  constant C_COEF_WIDTH : positive := 16;

  -- 31-tap symmetric sinc³ inverse compensator
  -- Coefficients designed via firls to flatten CIC's sinc³ droop
  -- Quantized to Q1.15 format, normalized to DC gain = 1.0
  -- Symmetric: use only 16 unique values (center tap + 15 pairs)
  type T_COEF_ARRAY is array (0 to 15) of signed(C_COEF_WIDTH - 1 downto 0);

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

  -- Pipeline stage 1: Pre-addition (symmetric pairs)
  type   T_PREADD_ARRAY is array (0 to 15) of signed(GC_INPUT_WIDTH downto 0);
  signal preadd_reg     : T_PREADD_ARRAY := (others => (others => '0'));
  signal valid_stage1   : std_logic      := '0';

  -- Pipeline stage 2: Multiplication
  type   T_PROD_ARRAY is array (0 to 15) of signed(GC_INPUT_WIDTH + C_COEF_WIDTH downto 0);
  signal prod_reg     : T_PROD_ARRAY := (others => (others => '0'));
  signal valid_stage2 : std_logic    := '0';

  -- Pipeline stage 3: Accumulation tree
  -- Accumulator width: input + coef + log2(31) guard bits
  constant C_ACC_WIDTH  : natural                          := GC_INPUT_WIDTH + C_COEF_WIDTH + 5;
  signal   acc_reg      : signed(C_ACC_WIDTH - 1 downto 0) := (others => '0');
  signal   valid_stage3 : std_logic                        := '0';

  -- Pipeline stage 4: Scaling + saturation
  signal output_reg : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal valid_reg  : std_logic                            := '0';

begin

  -- ========================================================================
  -- Pipeline Stage 1: Delay line shift + Symmetric pre-addition
  -- ========================================================================
  -- Exploits FIR symmetry: h[i] = h[30-i]
  -- Computes (x[i] + x[30-i]) for each symmetric pair
  -- This reduces multipliers from 31 to 16 (15 pairs + 1 center tap)
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
        for i in 30 downto 1 loop
          delay_line(i) <= delay_line(i - 1);
        end loop;
        delay_line(0) <= signed(data_in);

        -- Pre-addition for symmetric pairs (15 pairs)
        for i in 0 to 14 loop
          preadd_reg(i) <= resize(delay_line(i), GC_INPUT_WIDTH + 1) + resize(delay_line(30 - i), GC_INPUT_WIDTH + 1);
        end loop;

        -- Center tap (no pair, just pass through with sign extension)
        preadd_reg(15) <= resize(delay_line(15), GC_INPUT_WIDTH + 1);

        valid_stage1 <= '1';
      else
        valid_stage1 <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 2: Multiplication
  -- ========================================================================
  -- Each pre-added pair is multiplied by its coefficient
  -- Modern FPGAs have dedicated DSP blocks that can do 18x18 multiplication
  -- in a single clock cycle, so this stage uses 16 DSP blocks
  -- ========================================================================
  p_stage2_multiply : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        prod_reg     <= (others => (others => '0'));
        valid_stage2 <= '0';
      elsif valid_stage1 = '1' then
        -- Multiply each pre-added sum by its coefficient
        for i in 0 to 15 loop
          prod_reg(i) <= preadd_reg(i) * C_COEF(i);
        end loop;

        valid_stage2 <= '1';
      else
        valid_stage2 <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 3: Balanced binary accumulation tree
  -- ========================================================================
  -- Uses a balanced tree structure to minimize critical path
  -- Tree depth: log2(16) = 4 levels, much better than 15 sequential adds
  -- ========================================================================
  p_stage3_accumulate : process(clk)
    -- Level 1: 8 sums of 2 products
    type     T_LEVEL1 is array (0 to 7) of signed(C_ACC_WIDTH - 1 downto 0);
    variable v_level1 : T_LEVEL1;
    -- Level 2: 4 sums
    type     T_LEVEL2 is array (0 to 3) of signed(C_ACC_WIDTH - 1 downto 0);
    variable v_level2 : T_LEVEL2;
    -- Level 3: 2 sums
    type     T_LEVEL3 is array (0 to 1) of signed(C_ACC_WIDTH - 1 downto 0);
    variable v_level3 : T_LEVEL3;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        acc_reg      <= (others => '0');
        valid_stage3 <= '0';
      elsif valid_stage2 = '1' then
        -- Level 1: 8 pairs (16→8)
        for i in 0 to 7 loop
          v_level1(i) := resize(prod_reg(2 * i), C_ACC_WIDTH) + resize(prod_reg(2 * i + 1), C_ACC_WIDTH);
        end loop;

        -- Level 2: 4 pairs (8→4)
        for i in 0 to 3 loop
          v_level2(i) := v_level1(2 * i) + v_level1(2 * i + 1);
        end loop;

        -- Level 3: 2 pairs (4→2)
        for i in 0 to 1 loop
          v_level3(i) := v_level2(2 * i) + v_level2(2 * i + 1);
        end loop;

        -- Level 4: Final sum (2→1)
        acc_reg      <= v_level3(0) + v_level3(1);
        valid_stage3 <= '1';

      else
        valid_stage3 <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 4: Q1.15 scaling and saturation
  -- ========================================================================
  p_stage4_scale : process(clk)
    constant C_Q15_SHIFT   : natural                          := 15; -- Q1.15 format shift
    constant C_ROUND_CONST : signed(C_ACC_WIDTH - 1 downto 0) := to_signed(2 ** (C_Q15_SHIFT - 1), C_ACC_WIDTH); -- 0.5 LSB
    variable v_scaled      : signed(C_ACC_WIDTH - 1 downto 0);
  begin
    if rising_edge(clk) then
      if reset = '1' then
        output_reg <= (others => '0');
        valid_reg  <= '0';
      elsif valid_stage3 = '1' then
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
