library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity cic_sinc3_decimator is
  generic(
    DECIMATION   : positive := 64;      -- decimation factor R
    OUTPUT_WIDTH : positive := 16       -- width of output samples
  );
  port(
    clk      : in  std_logic;
    reset    : in  rst_t;
    data_in  : in  std_logic;           -- 1-bit delta-sigma stream
    data_out : out std_logic_vector(OUTPUT_WIDTH - 1 downto 0);
    valid    : out std_logic            -- high one cycle per output
  );
end entity;

architecture rtl of cic_sinc3_decimator is

  -- clog2 helper
  function clog2(x : positive) return natural is
    variable v : natural range 0 to 31      := 0; -- Constrained range
    variable y : natural range 0 to 2 ** 30 := x - 1; -- Avoid overflow
  begin
    while y > 0 loop
      y := y / 2;
      v := v + 1;
    end loop;
    return v;
  end function;

  -- CIC growth bits = N * log2(R), N=3
  constant GROWTH_BITS : natural := 3 * clog2(DECIMATION);
  constant EXTRA_GUARD : natural := 2;
  constant ACC_WIDTH   : natural := 1 + GROWTH_BITS + EXTRA_GUARD;

  subtype acc_t is signed(ACC_WIDTH - 1 downto 0);

  -- Integrators
  signal int1, int2, int3 : acc_t := (others => '0');

  -- Comb delay registers
  signal comb1_d, comb2_d, comb3_d : acc_t := (others => '0');

  -- Decimation counter
  signal dec_cnt   : natural range 0 to DECIMATION - 1 := 0;
  signal dec_pulse : std_logic                         := '0';

  -- Decimated sample (from last integrator)
  signal decimated : acc_t := (others => '0');

  -- Output register
  signal y_out   : signed(OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal y_valid : std_logic                         := '0';

  -- scale shift ≈ log2(R^3)
  constant SCALE_SHIFT : natural := 3 * clog2(DECIMATION);

  -- 1-bit map: '1' → +1, '0' → −1
  function map_bipolar(b : std_logic) return acc_t is
    variable one : acc_t := (others => '0');
  begin
    one(0) := '1';
    if b = '1' then
      return one;
    else
      return -one;
    end if;
  end function;

  -- arithmetic right shift with saturation
  function arshift_sat(x : acc_t; sh : natural; Wout : natural)
  return signed is
    variable r    : acc_t                     := x;
    variable outv : signed(Wout - 1 downto 0);
    constant MAXV : signed(Wout - 1 downto 0) := to_signed(2 ** (Wout - 1) - 1, Wout);
    constant MINV : signed(Wout - 1 downto 0) := to_signed(-2 ** (Wout - 1), Wout);
  begin
    for i in 1 to sh loop
      r := r(r'high) & r(r'high downto 1); -- arithmetic >> 1
    end loop;

    if r'length <= Wout then
      return resize(r, Wout);
    else
      outv := r(Wout - 1 downto 0);
      if r'high - 1 >= Wout then
        if (r(r'high) = '0' and (not (r(r'high - 1 downto Wout) = (r'high - 1 downto Wout => '0')))) then
          return MAXV;
        elsif (r(r'high) = '1' and (not (r(r'high - 1 downto Wout) = (r'high - 1 downto Wout => '1')))) then
          return MINV;
        else
          return outv;
        end if;
      else
        return outv;
      end if;
    end if;
  end function;

begin

  data_out <= std_logic_vector(y_out);
  valid    <= y_valid;

  -- Integrators @ input rate
  process(clk)
    variable x    : acc_t;
    variable stg1 : acc_t;
    variable stg2 : acc_t;
    variable stg3 : acc_t;
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        int1 <= (others => '0');
        int2 <= (others => '0');
        int3 <= (others => '0');
      else
        x    := map_bipolar(data_in);
        stg1 := int1 + x;
        stg2 := int2 + stg1;
        stg3 := int3 + stg2;
        int1 <= stg1;
        int2 <= stg2;
        int3 <= stg3;
      end if;
    end if;
  end process;

  -- Decimation counter
  process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        dec_cnt   <= 0;
        dec_pulse <= '0';
        decimated <= (others => '0');
      else
        if dec_cnt = DECIMATION - 1 then
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
    variable comb1, comb2, comb3 : acc_t;
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        comb1_d <= (others => '0');
        comb2_d <= (others => '0');
        comb3_d <= (others => '0');
        y_out   <= (others => '0');
        y_valid <= '0';
      elsif dec_pulse = '1' then
        comb1   := decimated - comb1_d;
        comb1_d <= decimated;

        comb2   := comb1 - comb2_d;
        comb2_d <= comb1;

        comb3   := comb2 - comb3_d;
        comb3_d <= comb2;

        y_out   <= arshift_sat(comb3, SCALE_SHIFT, OUTPUT_WIDTH);
        y_valid <= '1';
      else
        y_valid <= '0';
      end if;
    end if;
  end process;

end architecture;
