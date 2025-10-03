library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.common_pkg.all;
use work.dsp_utils_pkg.all;

entity cic_sinc3_decimator is
  generic(
    GC_DECIMATION   : positive := 64;   -- decimation factor R
    GC_OUTPUT_WIDTH : positive := 16    -- width of output samples
  );
  port(
    clk      : in  std_logic;
    reset    : in  std_logic;           -- Use std_logic directly - no need for custom type
    data_in  : in  std_logic;           -- 1-bit delta-sigma stream
    data_out : out std_logic_vector(GC_OUTPUT_WIDTH - 1 downto 0);
    valid    : out std_logic            -- high one cycle per output
  );
end entity;

architecture rtl of cic_sinc3_decimator is

  -- CIC growth bits = N * log2(R), N=3
  constant C_GROWTH_BITS : natural := 3 * clog2(GC_DECIMATION);
  constant C_EXTRA_GUARD : natural := 4;
  constant C_ACC_WIDTH   : natural := 1 + C_GROWTH_BITS + C_EXTRA_GUARD;

  subtype T_ACC is signed(C_ACC_WIDTH - 1 downto 0);

  -- Scaling: Remove CIC gain to fit output in GC_OUTPUT_WIDTH
  -- For SINC3, gain G = R^3
  -- For R=57344: log2(G) = 3*log2(57344) ≈ 47.4 bits
  -- We have C_GROWTH_BITS = 48, plus 4 guard bits = 52 bits total
  -- To get 16-bit output: shift down by (C_ACC_WIDTH - GC_OUTPUT_WIDTH)
  constant C_SCALE_SHIFT : natural := C_ACC_WIDTH - GC_OUTPUT_WIDTH;

  -- Integrators
  signal int1, int2, int3 : T_ACC := (others => '0');

  -- Comb delay registers
  signal comb1_d, comb2_d, comb3_d : T_ACC := (others => '0');

  -- Decimation counter - using unsigned for better synthesis and flexibility
  signal dec_cnt   : unsigned(clog2(GC_DECIMATION) - 1 downto 0) := (others => '0');
  signal dec_pulse : std_logic                                   := '0';

  -- Decimated sample (from last integrator)
  signal decimated : T_ACC := (others => '0');

  -- Pipeline stage for comb output (before scaling)
  signal comb3_out   : T_ACC     := (others => '0');
  signal comb3_valid : std_logic := '0';

  -- Output register
  signal y_out   : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal y_valid : std_logic                            := '0';

begin

  data_out <= std_logic_vector(y_out);
  valid    <= y_valid;

  -- Integrators @ input rate (proper cascaded accumulators)
  -- Stage 1: int1 accumulates bipolar input (±1)
  -- Stage 2: int2 accumulates int1
  -- Stage 3: int3 accumulates int2
  -- This creates the SINC³ transfer function H(z) = ((1-z^-R)/(1-z^-1))^3
  process(clk)
    variable v_x : T_ACC;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        int1 <= (others => '0');
        int2 <= (others => '0');
        int3 <= (others => '0');
      else
        -- Map bitstream to bipolar: '1' → +1, '0' → -1
        v_x := map_bipolar(data_in, C_ACC_WIDTH);

        -- Cascade: each integrator accumulates the previous stage
        int1 <= int1 + v_x;
        int2 <= int2 + int1;
        int3 <= int3 + int2;
      end if;
    end if;
  end process;

  -- Decimation counter
  process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        dec_cnt   <= (others => '0');
        dec_pulse <= '0';
        decimated <= (others => '0');
      else
        if dec_cnt = to_unsigned(GC_DECIMATION - 1, dec_cnt'length) then
          dec_cnt   <= (others => '0');
          dec_pulse <= '1';
          decimated <= int3;
        else
          dec_cnt   <= dec_cnt + 1;
          dec_pulse <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 1: Comb filters @ decimated rate
  -- ========================================================================
  -- Performs the three differentiation stages
  -- This happens only when dec_pulse = '1' (every GC_DECIMATION cycles)
  -- ========================================================================
  p_comb_stage : process(clk)
    variable v_comb1, v_comb2, v_comb3 : T_ACC;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        comb1_d     <= (others => '0');
        comb2_d     <= (others => '0');
        comb3_d     <= (others => '0');
        comb3_out   <= (others => '0');
        comb3_valid <= '0';
      elsif dec_pulse = '1' then
        -- Comb stages (differentiation)
        v_comb1 := decimated - comb1_d;
        comb1_d <= decimated;

        v_comb2 := v_comb1 - comb2_d;
        comb2_d <= v_comb1;

        v_comb3 := v_comb2 - comb3_d;
        comb3_d <= v_comb2;

        -- Register output for next pipeline stage
        comb3_out   <= v_comb3;
        comb3_valid <= '1';
      else
        comb3_valid <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 2: Scaling and saturation
  -- ========================================================================
  -- Simple arithmetic shift to scale CIC output to GC_OUTPUT_WIDTH
  -- Shift amount = C_ACC_WIDTH - GC_OUTPUT_WIDTH (removes growth + guard bits)
  -- ========================================================================
  p_scale_stage : process(clk)
    variable v_shifted : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  begin
    if rising_edge(clk) then
      if reset = '1' then
        y_out   <= (others => '0');
        y_valid <= '0';
      elsif comb3_valid = '1' then
        -- Arithmetic right shift to remove CIC gain
        -- Takes MSBs of comb output (with sign extension)
        v_shifted := resize(shift_right(comb3_out, C_SCALE_SHIFT), GC_OUTPUT_WIDTH);

        y_out   <= v_shifted;
        y_valid <= '1';
      else
        y_valid <= '0';
      end if;
    end if;
  end process;

end architecture;
