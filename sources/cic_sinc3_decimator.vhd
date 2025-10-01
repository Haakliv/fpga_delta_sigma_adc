library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity cic_sinc3_decimator is
  generic(
    GC_DECIMATION   : positive := 64;   -- decimation factor R
    GC_OUTPUT_WIDTH : positive := 16    -- width of output samples
  );
  port(
    clk      : in  std_logic;
    reset    : in  T_RST_T;
    data_in  : in  std_logic;           -- 1-bit delta-sigma stream
    data_out : out std_logic_vector(GC_OUTPUT_WIDTH - 1 downto 0);
    valid    : out std_logic            -- high one cycle per output
  );
end entity;

architecture rtl of cic_sinc3_decimator is

  -- clog2 helper
  function clog2(x : positive) return natural is
    variable v_bit_count : natural range 0 to 31      := 0; -- Constrained range
    variable v_remainder : natural range 0 to 2 ** 30 := x - 1; -- Avoid overflow
  begin
    while v_remainder > 0 loop
      v_remainder := v_remainder / 2;
      v_bit_count := v_bit_count + 1;
    end loop;
    return v_bit_count;
  end function;

  -- CIC growth bits = N * log2(R), N=3
  constant C_GROWTH_BITS : natural := 3 * clog2(GC_DECIMATION);
  constant C_EXTRA_GUARD : natural := 4;
  constant C_ACC_WIDTH   : natural := 1 + C_GROWTH_BITS + C_EXTRA_GUARD;

  subtype T_ACC is signed(C_ACC_WIDTH - 1 downto 0);

  -- Integrators
  signal int1, int2, int3 : T_ACC := (others => '0');

  -- Comb delay registers
  signal comb1_d, comb2_d, comb3_d : T_ACC := (others => '0');

  -- Decimation counter
  signal dec_cnt   : natural range 0 to GC_DECIMATION - 1 := 0;
  signal dec_pulse : std_logic                            := '0';

  -- Decimated sample (from last integrator)
  signal decimated : T_ACC := (others => '0');

  -- Output register
  signal y_out   : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal y_valid : std_logic                            := '0';

  -- Scale shift = log2(R^3). Exact for power-of-two R (e.g., R=65536 → shift=48).
  constant C_SCALE_SHIFT : natural := 3 * clog2(GC_DECIMATION);

  -- 1-bit map: '1' → +1, '0' → −1
  function map_bipolar(b : std_logic) return T_ACC is
    variable v_one : T_ACC := (others => '0');
  begin
    v_one(0) := '1';
    if b = '1' then
      return v_one;
    else
      return -v_one;
    end if;
  end function;

begin

  data_out <= std_logic_vector(y_out);
  valid    <= y_valid;

  -- Integrators @ input rate
  process(clk)
    variable v_x    : T_ACC;
    variable v_stg1 : T_ACC;
    variable v_stg2 : T_ACC;
    variable v_stg3 : T_ACC;
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        int1 <= (others => '0');
        int2 <= (others => '0');
        int3 <= (others => '0');
      else
        v_x    := map_bipolar(data_in);
        v_stg1 := int1 + v_x;
        v_stg2 := int2 + v_stg1;
        v_stg3 := int3 + v_stg2;
        int1   <= v_stg1;
        int2   <= v_stg2;
        int3   <= v_stg3;
      end if;
    end if;
  end process;

  -- Decimation counter
  process(clk)
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        dec_cnt   <= 0;
        dec_pulse <= '0';
        decimated <= (others => '0');
      else
        if dec_cnt = GC_DECIMATION - 1 then
          dec_cnt   <= 0;
          dec_pulse <= '1';
          decimated <= int3;
        else
          dec_cnt   <= dec_cnt + 1;
          dec_pulse <= '0';
        end if;
      end if;
    end if;
  end process;

  -- Combs @ decimated rate
  process(clk)
    variable v_comb1, v_comb2, v_comb3 : T_ACC;
    variable v_scaled                  : T_ACC;
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        comb1_d <= (others => '0');
        comb2_d <= (others => '0');
        comb3_d <= (others => '0');
        y_out   <= (others => '0');
        y_valid <= '0';
      elsif dec_pulse = '1' then
        -- Comb stages (differentiation)
        v_comb1 := decimated - comb1_d;
        comb1_d <= decimated;

        v_comb2 := v_comb1 - comb2_d;
        comb2_d <= v_comb1;

        v_comb3 := v_comb2 - comb3_d;
        comb3_d <= v_comb2;

        -- Scale down by R^3 with rounding (exact for power-of-two R)
        v_scaled := v_comb3;
        if C_SCALE_SHIFT > 0 then
          -- Add rounding: 0.5 LSB at the bit we'll cut (round half up)
          v_scaled := v_scaled + shift_left(to_signed(1, v_scaled'length), C_SCALE_SHIFT - 1);
          -- Arithmetic right shift using numeric_std
          v_scaled := shift_right(v_scaled, C_SCALE_SHIFT);
        end if;

        -- Resize to output width (saturates on overflow)
        y_out   <= resize(v_scaled, GC_OUTPUT_WIDTH);
        y_valid <= '1';
      else
        y_valid <= '0';
      end if;
    end if;
  end process;

end architecture;
