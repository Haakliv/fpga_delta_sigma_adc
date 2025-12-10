library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

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
    ce       : in  std_logic;           -- Clock enable - sample data_in when high
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

  -- Scaling: Remove CIC gain to achieve unity DC gain
  -- For sinc3 with M=1, DC gain G = R^3
  -- For power-of-2 R, shift by 3*log2(R) is exact and fast
  -- For non-power-of-2 R, we need multi-cycle division (handled via pipelining)
  -- Note: For large R (e.g. 25600), R^3 exceeds VHDL integer range.
  -- We compute R^3 at runtime for division, not as a constant.
  
  -- Detect if decimation is power-of-2 for fast shift instead of division
  function is_power_of_2(n : positive) return boolean is
    variable v : positive := n;
  begin
    while v > 1 loop
      if (v mod 2) /= 0 then
        return false;
      end if;
      v := v / 2;
    end loop;
    return true;
  end function;
  
  constant C_IS_POW2    : boolean := is_power_of_2(GC_DECIMATION);
  constant C_SHIFT_BITS : natural := 3 * clog2(GC_DECIMATION); -- For power-of-2: shift right by 3*log2(R)

  -- Q-format: Preserve fractional bits in output
  -- Output is signed Q-format: [-1.0, 1.0) maps to [-2^(W-1), 2^(W-1)-1]
  -- C_QBITS fractional bits ensure sub-unity means survive division by R^3
  constant C_QBITS : natural := GC_OUTPUT_WIDTH - 1; -- fractional bits in output

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
  signal comb1_out   : T_ACC     := (others => '0'); -- Comb1 output register
  signal comb2_out   : T_ACC     := (others => '0'); -- Comb2 output register
  signal comb3_out   : T_ACC     := (others => '0');
  signal comb3_valid : std_logic := '0';

  -- Pipeline registers for scaling stage (4-stage pipeline to meet timing)
  -- Timing path is broken into: upscale -> round_add -> divide -> saturate
  constant C_WIDE_WIDTH : natural := C_ACC_WIDTH + C_QBITS;
  
  -- Stage 1: Upscale (shift left by C_QBITS)
  signal scale_pipe1       : signed(C_WIDE_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe1_valid : std_logic := '0';
  
  -- Stage 2: Add rounding offset
  signal scale_pipe2       : signed(C_WIDE_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe2_valid : std_logic := '0';
  
  -- Stage 3: Division by R^3
  signal scale_pipe3       : signed(C_WIDE_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe3_valid : std_logic := '0';

  -- Output register
  signal y_out   : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal y_valid : std_logic                            := '0';

begin

  data_out <= std_logic_vector(y_out);
  valid    <= y_valid;

  -- Integrators @ input rate (proper cascaded accumulators)
  -- Stage 1: int1 accumulates bipolar input (+-1)
  -- Stage 2: int2 accumulates int1
  -- Stage 3: int3 accumulates int2
  -- This creates the sinc3 transfer function H(z) = ((1-z^-R)/(1-z^-1))^3
  -- Clock enable gates integration to sample rate
  process(clk)
    variable v_x : T_ACC;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        int1 <= (others => '0');
        int2 <= (others => '0');
        int3 <= (others => '0');
      else
        if ce = '1' then                -- Only integrate when new sample arrives
          -- Map bitstream to bipolar: '1' -> +1, '0' -> -1
          v_x := map_bipolar(data_in, C_ACC_WIDTH);

          -- Cascade: each integrator accumulates the previous stage
          int1 <= int1 + v_x;
          int2 <= int2 + int1;
          int3 <= int3 + int2;
        end if;
      end if;
    end if;
  end process;

  -- Decimation counter (only counts when ce='1')
  process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        dec_cnt   <= (others => '0');
        dec_pulse <= '0';
        decimated <= (others => '0');
      else
        dec_pulse <= '0';               -- Default
        if ce = '1' then                -- Only count when new sample arrives
          if dec_cnt = to_unsigned(GC_DECIMATION - 1, dec_cnt'length) then
            dec_cnt   <= (others => '0');
            dec_pulse <= '1';
            decimated <= int3;
          else
            dec_cnt <= dec_cnt + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 1: Comb filters @ decimated rate (3-CYCLE PIPELINE)
  -- ========================================================================
  -- Performs the three differentiation stages across 3 clock cycles
  -- This happens only when dec_pulse = '1' (every GC_DECIMATION cycles)
  -- Cycle 1: comb1 = decimated - comb1_d
  -- Cycle 2: comb2 = comb1 - comb2_d
  -- Cycle 3: comb3 = comb2 - comb3_d
  -- Total latency: 3 cycles after dec_pulse (acceptable for decimated rate)
  -- ========================================================================
  p_comb_stage : process(clk)
    variable v_comb1, v_comb2, v_comb3 : T_ACC;
    type     T_PIPE_STATE              is (ST_IDLE, ST_STAGE1, ST_STAGE2);
    variable v_pipe_state              : T_PIPE_STATE := ST_IDLE;
  begin
    if rising_edge(clk) then
      if reset = '1' then
        comb1_d      <= (others => '0');
        comb2_d      <= (others => '0');
        comb3_d      <= (others => '0');
        comb1_out    <= (others => '0');
        comb2_out    <= (others => '0');
        comb3_out    <= (others => '0');
        comb3_valid  <= '0';
        v_pipe_state := ST_IDLE;
      else
        case v_pipe_state is
          when ST_IDLE =>
            -- Idle: waiting for dec_pulse
            comb3_valid <= '0';
            if dec_pulse = '1' then
              -- Cycle 1: First comb stage
              v_comb1      := decimated - comb1_d;
              comb1_d      <= decimated;
              comb1_out    <= v_comb1;  -- Register for next cycle
              v_pipe_state := ST_STAGE1;
            end if;

          when ST_STAGE1 =>
            -- Cycle 2: Second comb stage
            v_comb2      := comb1_out - comb2_d;
            comb2_d      <= comb1_out;
            comb2_out    <= v_comb2;    -- Register for next cycle
            v_pipe_state := ST_STAGE2;

          when ST_STAGE2 =>
            -- Cycle 3: Third comb stage
            v_comb3      := comb2_out - comb3_d;
            comb3_d      <= comb2_out;
            comb3_out    <= v_comb3;
            comb3_valid  <= '1';        -- Output valid on cycle 3
            v_pipe_state := ST_IDLE;    -- Return to idle
        end case;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Stage 2: Q-Format Scaling and Saturation (4-STAGE PIPELINE)
  -- ========================================================================
  -- Scale CIC output by R^3 while preserving fractional bits (Q-format)
  -- 
  -- TIMING FIX: Split into 4 pipeline stages to break critical path:
  --   Stage 1: Upscale by 2^Q (bit shift - fast)
  --   Stage 2: Add rounding offset (wide addition - was timing critical)
  --   Stage 3: Divide by R^3 (division/shift)
  --   Stage 4: Saturate to output width
  --
  -- With R=64, we have 64 clock cycles between samples, so 4 cycles latency
  -- is negligible. This breaks the path: upscale+round+divide+saturate into
  -- separate registered stages.
  --
  -- Result: Output is signed Q-format where full-scale ±1.0 → ±(2^(W-1)-1)
  -- ========================================================================
  p_scale_stage : process(clk)
    -- Compute R^3 at runtime (variables support larger values than constants)
    variable v_r_cubed      : signed(63 downto 0);
    variable v_round_offset : signed(C_WIDE_WIDTH - 1 downto 0);
    -- Division operands (constrained to 63-bit to satisfy LPM_WIDTHN/D <= 64)
    variable v_dividend     : signed(62 downto 0);
    variable v_divisor      : signed(62 downto 0);
    variable v_quotient     : signed(62 downto 0);
  begin
    if rising_edge(clk) then
      if reset = '1' then
        scale_pipe1       <= (others => '0');
        scale_pipe1_valid <= '0';
        scale_pipe2       <= (others => '0');
        scale_pipe2_valid <= '0';
        scale_pipe3       <= (others => '0');
        scale_pipe3_valid <= '0';
        y_out             <= (others => '0');
        y_valid           <= '0';
      else
        -- Compute R^3 as 64-bit signed (handles large decimation ratios like 25600)
        -- Perform multiplications with intermediate resizing to avoid 192-bit intermediate
        v_r_cubed := resize(to_signed(GC_DECIMATION, 32) * to_signed(GC_DECIMATION, 32), 64);
        v_r_cubed := resize(v_r_cubed * to_signed(GC_DECIMATION, 32), 64);
        v_round_offset := resize(shift_right(v_r_cubed, 1), C_WIDE_WIDTH);
        -- ====== Pipeline Stage 1: Upscale by 2^Q ======
        -- Shift left to preserve fractional bits in Q-format
        if comb3_valid = '1' then
          scale_pipe1       <= shift_left(resize(comb3_out, C_WIDE_WIDTH), C_QBITS);
          scale_pipe1_valid <= '1';
        else
          scale_pipe1_valid <= '0';
        end if;

        -- ====== Pipeline Stage 2: Add rounding offset ======
        -- This was the timing-critical path: wide addition on ~37-bit signal
        -- Now it's in its own registered stage
        if scale_pipe1_valid = '1' then
          if scale_pipe1 >= 0 then
            scale_pipe2 <= scale_pipe1 + v_round_offset;
          else
            scale_pipe2 <= scale_pipe1 - v_round_offset;
          end if;
          scale_pipe2_valid <= '1';
        else
          scale_pipe2_valid <= '0';
        end if;

        -- ====== Pipeline Stage 3: Divide by R^3 ======
        -- For power-of-2: bit shift (fast)
        -- For non-power-of-2: exact division (synthesis uses DSP)
        -- Constrain division to 63-bit operands (LPM_WIDTHN/D must be <= 64)
        if scale_pipe2_valid = '1' then
          if C_IS_POW2 then
            scale_pipe3 <= shift_right(scale_pipe2, C_SHIFT_BITS);
          else
            -- Use 63-bit operands to stay under LPM divider limit
            v_dividend := resize(scale_pipe2, 63);
            v_divisor := resize(v_r_cubed, 63);
            v_quotient := v_dividend / v_divisor;
            scale_pipe3 <= resize(v_quotient, scale_pipe3'length);
          end if;
          scale_pipe3_valid <= '1';
        else
          scale_pipe3_valid <= '0';
        end if;

        -- ====== Pipeline Stage 4: Saturate to output width ======
        if scale_pipe3_valid = '1' then
          y_out   <= saturate(scale_pipe3, GC_OUTPUT_WIDTH);
          y_valid <= '1';
        else
          y_valid <= '0';
        end if;
      end if;
    end if;
  end process;

end architecture;
