library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dsp_utils_pkg.all;

entity cic_sinc3_decimator is
  generic(
    GC_INPUT_WIDTH  : positive := 1;    -- Input width: 1 for 1-bit stream, >1 for multi-bit signed
    GC_DECIMATION   : positive := 64;   -- decimation factor R
    GC_OUTPUT_WIDTH : positive := 16    -- width of output samples
  );
  port(
    clk          : in  std_logic;
    reset        : in  std_logic;       -- Use std_logic directly - no need for custom type
    data_in      : in  std_logic;       -- 1-bit delta-sigma stream (used when GC_INPUT_WIDTH=1)
    data_in_wide : in  signed(GC_INPUT_WIDTH - 1 downto 0) := (others => '0'); -- Multi-bit signed input (used when GC_INPUT_WIDTH>1)
    ce           : in  std_logic;       -- Clock enable - sample data_in when high
    data_out     : out std_logic_vector(GC_OUTPUT_WIDTH - 1 downto 0);
    valid        : out std_logic        -- high one cycle per output
  );
end entity;

architecture rtl of cic_sinc3_decimator is

  -- CIC growth bits = N * log2(R), N=3
  -- For multi-bit input, add input width to growth calculation
  constant C_GROWTH_BITS : natural := 3 * clog2(GC_DECIMATION) + (GC_INPUT_WIDTH - 1);
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
    variable v_x : integer range 1 to integer'high := n;
  begin
    while v_x > 1 loop
      if (v_x mod 2) /= 0 then
        return false;
      end if;
      v_x := v_x / 2;
    end loop;
    return true;
  end function;

  constant C_IS_POW2    : boolean := is_power_of_2(GC_DECIMATION);
  constant C_SHIFT_BITS : natural := 3 * clog2(GC_DECIMATION); -- For power-of-2: shift right by 3*log2(R)

  -- Q-format: Preserve fractional bits in output
  -- Output is signed Q-format: [-1.0, 1.0) maps to [-2^(W-1), 2^(W-1)-1]
  -- C_QBITS fractional bits ensure sub-unity means survive division by R^3

  -- For multi-bit input, the signal is already scaled (e.g. ±21845 for DAC).
  -- After CIC R^3 normalization, output = input_avg (unity DC gain).
  -- 
  -- C_DAC_AMPLITUDE = 21845 gives input range of ±21845.
  -- To map ±21845 to full Q15 range ±32767, we need gain = 32767/21845 = 1.5 exactly.
  -- We implement this as multiply by 3, shift right by 1 (3/2 = 1.5).
  --
  -- For 1-bit input: map +/-1 to full scale (QBITS = OUTPUT_WIDTH - 1 = 15)
  function get_qbits(input_width : positive; output_width : positive) return natural is
  begin
    if input_width = 1 then
      return output_width - 1;          -- 1-bit input: map +/-1 to full scale
    else
      return 0;                         -- Multi-bit input: unity here, fractional gain applied separately
    end if;
  end function;

  constant C_QBITS : natural := get_qbits(GC_INPUT_WIDTH, GC_OUTPUT_WIDTH);

  -- Fractional gain multiplier for multi-bit mode: 3/2 = 1.5
  -- Maps C_DAC_AMPLITUDE (±21845) to full Q15 range (±32767)
  -- Gain = 32767/21845 = 1.5 exactly
  constant C_MULTIBIT_GAIN_SHIFT : natural := 1; -- log2(2) = 1

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

  -- Pipeline registers for scaling stage (5-stage pipeline to meet timing)
  -- Timing path is broken into: upscale -> sign_check -> round_add -> divide -> saturate
  constant C_WIDTH : natural := C_ACC_WIDTH + C_QBITS;

  -- Stage 1: Upscale (shift left by C_QBITS)
  signal scale_pipe1       : signed(C_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe1_valid : std_logic                    := '0';

  -- Stage 1.5: Register sign for conditional rounding
  signal scale_pipe1_5       : signed(C_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe1_5_valid : std_logic                    := '0';
  signal scale_pipe1_5_pos   : std_logic                    := '0'; -- '1' if positive

  -- Stage 2: Add rounding offset
  signal scale_pipe2       : signed(C_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe2_valid : std_logic                    := '0';

  -- Stage 3: Division by R^3 (multiply only)
  signal scale_pipe3       : signed(C_WIDTH + 64 - 1 downto 0) := (others => '0'); -- Full multiply result
  signal scale_pipe3_valid : std_logic                         := '0';

  -- Stage 3.8: After shift/downscale (breaks multiplier timing path)
  signal scale_pipe3_8       : signed(C_WIDTH - 1 downto 0) := (others => '0');
  signal scale_pipe3_8_valid : std_logic                    := '0';

  -- Stage 3.5: Fractional gain for multi-bit mode (multiply by 3, shift by 1)
  signal scale_pipe3_5       : signed(C_WIDTH + 5 - 1 downto 0) := (others => '0'); -- Extra bits for *3
  signal scale_pipe3_5_valid : std_logic                        := '0';

  -- Stage 3.75: Extra register to break multiply timing path
  signal scale_pipe3_75       : signed(C_WIDTH + 5 - 1 downto 0) := (others => '0');
  signal scale_pipe3_75_valid : std_logic                        := '0';

  -- Output register
  signal y_out   : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal y_valid : std_logic                            := '0';

begin

  data_out <= std_logic_vector(y_out);
  valid    <= y_valid;

  -- Integrators @ input rate (proper cascaded accumulators)
  -- Stage 1: int1 accumulates bipolar input (+-1) or multi-bit signed input
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
          -- Input mapping depends on GC_INPUT_WIDTH
          if GC_INPUT_WIDTH = 1 then
            -- Map 1-bit bitstream to bipolar: '1' -> +1, '0' -> -1
            v_x := map_bipolar(data_in, C_ACC_WIDTH);
          else
            -- Multi-bit signed input: resize directly
            v_x := resize(data_in_wide, C_ACC_WIDTH);
          end if;

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
  -- Comb filters @ decimated rate (3-CYCLE SEQUENTIAL STATE MACHINE)
  -- ========================================================================
  -- Performs the three differentiation stages sequentially across 3 clock cycles
  -- NOTE: This is NOT a pipeline - only one sample is processed at a time.
  -- This happens only when dec_pulse = '1' (every GC_DECIMATION cycles)
  -- Cycle 1: comb1 = decimated - comb1_d
  -- Cycle 2: comb2 = comb1 - comb2_d
  -- Cycle 3: comb3 = comb2 - comb3_d
  -- Total latency: 3 cycles after dec_pulse (acceptable for decimated rate)
  -- 
  -- TIMING NOTE: The wide subtractions may not meet 400MHz timing, but since
  -- this FSM only activates once every GC_DECIMATION cycles (e.g., 8192),
  -- multicycle constraints can be applied to relax the requirement.
  -- Example SDC: set_multicycle_path -from {*comb*_d*} -to {*comb*_out*} -setup 2
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
  -- Q-Format Scaling and Saturation (TRUE 5-STAGE PIPELINE)
  -- ========================================================================
  -- Scale CIC output by R^3 while preserving fractional bits (Q-format)
  -- 
  -- TIMING FIX: Split into 5 pipeline stages to break critical path.
  -- This is a TRUE pipeline - multiple samples can be in-flight simultaneously.
  --   Stage 1: Upscale by 2^Q (bit shift - fast)
  --   Stage 1.5: Register sign check (breaks timing path)
  --   Stage 2: Add rounding offset (conditional add now uses registered sign)
  --   Stage 3: Divide by R^3 (division/shift)
  --   Stage 4: Saturate to output width
  --
  -- With R=25600, we have 25600 clock cycles between samples, so 5+3=8 cycles
  -- total latency is negligible. This breaks the path: upscale+sign+round+divide+
  -- saturate into separate registered stages, allowing max clock frequency.
  --
  -- Result: Output is signed Q-format where full-scale +/-1.0 -> +/-(2^(W-1)-1)
  -- ========================================================================
  p_scale_stage : process(clk)
    -- Compute R^3 at runtime (variables support larger values than constants)
    variable v_r_cubed      : signed(63 downto 0);
    variable v_round_offset : signed(C_WIDTH - 1 downto 0);
    -- Division-by-constant optimization: multiply by reciprocal instead of divide
    -- Compute (1/R^3) * 2^62 as a scaled reciprocal, then multiply and shift
    -- Increased scale from 48 to 62 to maintain precision for large R (e.g. 25600 -> R^3 ~ 2^44)
    constant C_RECIP_SCALE  : positive := 62; -- Scale factor for reciprocal precision
    variable v_recip        : signed(63 downto 0); -- Reciprocal: (2^62 / R^3)
    variable v_product      : signed(C_WIDTH + 64 - 1 downto 0); -- Multiplication result
  begin
    if rising_edge(clk) then
      if reset = '1' then
        scale_pipe1          <= (others => '0');
        scale_pipe1_valid    <= '0';
        scale_pipe1_5        <= (others => '0');
        scale_pipe1_5_valid  <= '0';
        scale_pipe1_5_pos    <= '0';
        scale_pipe2          <= (others => '0');
        scale_pipe2_valid    <= '0';
        scale_pipe3          <= (others => '0');
        scale_pipe3_valid    <= '0';
        scale_pipe3_8        <= (others => '0');
        scale_pipe3_8_valid  <= '0';
        scale_pipe3_8        <= (others => '0');
        scale_pipe3_8_valid  <= '0';
        scale_pipe3_5        <= (others => '0');
        scale_pipe3_5_valid  <= '0';
        scale_pipe3_75       <= (others => '0');
        scale_pipe3_75_valid <= '0';
        y_out                <= (others => '0');
        y_valid              <= '0';
      else
        -- Compute R^3 as 64-bit signed (handles large decimation ratios like 25600)
        -- Perform multiplications with intermediate resizing to avoid 192-bit intermediate
        v_r_cubed      := resize(to_signed(GC_DECIMATION, 32) * to_signed(GC_DECIMATION, 32), 64);
        v_r_cubed      := resize(v_r_cubed * to_signed(GC_DECIMATION, 32), 64);
        v_round_offset := resize(shift_right(v_r_cubed, 1), C_WIDTH);

        -- ====== Pipeline Stage 1: Upscale by 2^Q ======
        -- Shift left to preserve fractional bits in Q-format
        if comb3_valid = '1' then
          scale_pipe1       <= shift_left(resize(comb3_out, C_WIDTH), C_QBITS);
          scale_pipe1_valid <= '1';
        else
          scale_pipe1_valid <= '0';
        end if;

        -- ====== Pipeline Stage 1.5: Register sign check ======
        -- Break timing path by registering sign check result
        if scale_pipe1_valid = '1' then
          scale_pipe1_5       <= scale_pipe1;
          scale_pipe1_5_pos   <= '1' when scale_pipe1 >= 0 else '0';
          scale_pipe1_5_valid <= '1';
        else
          scale_pipe1_5_valid <= '0';
        end if;

        -- ====== Pipeline Stage 2: Add rounding offset ======
        -- Use pre-registered sign check to reduce combinational depth
        if scale_pipe1_5_valid = '1' then
          if scale_pipe1_5_pos = '1' then
            scale_pipe2 <= scale_pipe1_5 + v_round_offset;
          else
            scale_pipe2 <= scale_pipe1_5 - v_round_offset;
          end if;
          scale_pipe2_valid <= '1';
        else
          scale_pipe2_valid <= '0';
        end if;

        -- ====== Pipeline Stage 3: Divide by R^3 (MULTIPLY ONLY) ======
        -- For power-of-2: bit shift (fast)
        -- For non-power-of-2: multiply by reciprocal (much faster than division!)
        --   Instead of: result = value / R^3
        --   We compute: result = (value * (2^48 / R^3)) >> 48
        -- This uses a DSP multiplier instead of a multi-cycle divider
        -- TIMING FIX: Only do multiply here, shift in next stage
        if scale_pipe2_valid = '1' then
          if C_IS_POW2 then
            -- For power-of-2, place value in upper bits (will shift down in next stage)
            -- MUST resize to full width BEFORE shift_left, or upper bits are lost!
            scale_pipe3 <= shift_left(resize(scale_pipe2, scale_pipe3'length), 64);
          else
            -- Compute reciprocal: (2^62) / R^3
            v_recip     := shift_left(to_signed(1, 64), C_RECIP_SCALE) / v_r_cubed;
            -- Multiply by reciprocal (hardware-efficient: single DSP block)
            -- Store raw multiply result (will shift in next stage)
            v_product   := scale_pipe2 * v_recip;
            scale_pipe3 <= v_product;
          end if;
          scale_pipe3_valid <= '1';
        else
          scale_pipe3_valid <= '0';
        end if;

        -- ====== Pipeline Stage 3.8: Shift and downscale ======
        -- Register the multiplier output, then apply shift
        if scale_pipe3_valid = '1' then
          if C_IS_POW2 then
            scale_pipe3_8 <= resize(shift_right(scale_pipe3, 64 + C_SHIFT_BITS), scale_pipe3_8'length);
          else
            -- Shift right to normalize (remove 2^62 scaling)
            scale_pipe3_8 <= resize(shift_right(scale_pipe3, C_RECIP_SCALE), scale_pipe3_8'length);
          end if;
          scale_pipe3_8_valid <= '1';
        else
          scale_pipe3_8_valid <= '0';
        end if;

        -- ====== Pipeline Stage 3.5: Fractional gain for multi-bit mode ======
        -- Apply 3/2 = 1.5 gain by computing x*3 = x + x*2 (no multiplier needed)
        -- For 1-bit mode, this is bypassed (unity gain)
        if scale_pipe3_8_valid = '1' then
          if GC_INPUT_WIDTH = 1 then
            -- 1-bit mode: pass through (already scaled by QBITS=15)
            scale_pipe3_5 <= resize(scale_pipe3_8, scale_pipe3_5'length);
          else
            -- Multi-bit mode: multiply by 3 using x + (x << 1), will shift by 1 in next stage
            -- This avoids a hardware multiplier and meets timing at 400MHz
            scale_pipe3_5 <= resize(scale_pipe3_8, scale_pipe3_5'length) + resize(shift_left(scale_pipe3_8, 1), scale_pipe3_5'length);
          end if;
          scale_pipe3_5_valid <= '1';
        else
          scale_pipe3_5_valid <= '0';
        end if;

        -- ====== Pipeline Stage 3.75: Extra register to break multiply timing ======
        -- Register multiply output before shift/saturate (breaks critical path)
        if scale_pipe3_5_valid = '1' then
          scale_pipe3_75       <= scale_pipe3_5;
          scale_pipe3_75_valid <= '1';
        else
          scale_pipe3_75_valid <= '0';
        end if;

        -- ====== Pipeline Stage 4: Final shift and saturate to output width ======
        if scale_pipe3_75_valid = '1' then
          if GC_INPUT_WIDTH = 1 then
            -- 1-bit mode: direct saturation (no additional shift)
            y_out <= saturate(resize(scale_pipe3_75, C_WIDTH), GC_OUTPUT_WIDTH);
          else
            -- Multi-bit mode: shift right by 1 to complete 1.5 gain (3/2), then saturate
            y_out <= saturate(resize(shift_right(scale_pipe3_75, C_MULTIBIT_GAIN_SHIFT), C_WIDTH), GC_OUTPUT_WIDTH);
          end if;
          y_valid <= '1';
        else
          y_valid <= '0';
        end if;
      end if;
    end if;
  end process;

end architecture;
