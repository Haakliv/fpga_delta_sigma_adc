-- ************************************************************************
-- Top-Level Delta-Sigma ADC
-- Complete RC ADC with LVDS input and all processing stages
--
-- ARCHITECTURE:
--   ADC core (comparator, DAC feedback, CIC) runs in 'clk' domain (50 MHz)
--   FIR filters and output run in 'clk_sys' domain (100 MHz)
--   CDC via simple valid pulse synchronizer (decimated rate ~97.66 kHz)
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dsp_utils_pkg.all;

entity rc_adc_top is
  generic(
    GC_DECIMATION : positive := 64;     -- Oversampling ratio (configurable)
    GC_DATA_WIDTH : positive := 16      -- Output data width
  );
  port(
    -- Clock and reset
    clk            : in  std_logic;     -- 50 MHz ADC sampling clock
    clk_fast       : in  std_logic;     -- 400 MHz fast clock for low-latency loop
    clk_sys        : in  std_logic;     -- 100 MHz system clock
    reset          : in  std_logic;
    -- Physical ADC interface
    -- True differential LVDS input (Quartus manages differential pair)
    analog_in      : in  std_logic;     -- LVDS differential comparator output
    dac_out        : out std_logic;     -- DAC output for feedback
    -- Optional trigger input (when '1', sampling is enabled; when '0', sampling is disabled)
    trigger_enable : in  std_logic := '1'; -- Default '1' for continuous sampling

    -- Streaming sample output (clk_sys domain)
    sample_data    : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid   : out std_logic
  );
end entity;

architecture rtl of rc_adc_top is

  -- Multi-bit signal width (matches TDC ADC for identical CIC behavior)
  constant C_MULTIBIT_WIDTH : positive := 20;

  -- DAC amplitude (matches TDC exactly: ±21845 gives 1.5x gain through CIC to reach full Q15 scale)
  constant C_DAC_AMPLITUDE : signed(C_MULTIBIT_WIDTH - 1 downto 0) := to_signed(21845, C_MULTIBIT_WIDTH);

  -- ========================================================================
  -- CLK domain signals (50 MHz ADC sampling clock)
  -- ========================================================================
  -- Fast domain signals (400 MHz)
  signal comp_sync0_fast : std_logic := '0';
  signal comp_sync1_fast : std_logic := '0';
  signal ref_sync0       : std_logic := '0';
  signal ref_sync1       : std_logic := '0';
  signal ref_sync2       : std_logic := '0';
  signal ref_sync2_prev  : std_logic := '0';

  -- DAC feedback register (1-bit DAC)
  signal dac_feedback : std_logic := '0';

  -- Delayed DAC for CIC input alignment
  signal dac_reg     : std_logic := '0';
  signal dac_delayed : std_logic := '0';

  -- CIC input and output (clk domain)
  signal cic_input     : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');
  signal cic_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out : std_logic;

  -- ========================================================================
  -- CDC signals (clk -> clk_sys)
  -- ========================================================================
  -- Toggle-based CDC for valid pulse (robust for slow rate)
  signal cic_toggle_clk   : std_logic := '0'; -- Toggles on each valid in clk domain
  signal cic_toggle_sync0 : std_logic := '0'; -- Synchronizer stage 1
  signal cic_toggle_sync1 : std_logic := '0'; -- Synchronizer stage 2
  signal cic_toggle_sync2 : std_logic := '0'; -- Edge detection delay
  signal cic_valid_sys    : std_logic := '0'; -- Reconstructed valid pulse

  -- Data holding register (stable during CDC)
  signal cic_data_hold : std_logic_vector(GC_DATA_WIDTH - 1 downto 0) := (others => '0');

  -- ========================================================================
  -- CLK_SYS domain signals (100 MHz system clock)
  -- ========================================================================
  signal cic_out_sys : signed(GC_DATA_WIDTH - 1 downto 0) := (others => '0');

  signal eq_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out : std_logic;
  signal lp_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out : std_logic;

begin

  -- ========================================================================
  -- Fast Loop (clk_fast domain - 400 MHz)
  -- ========================================================================
  -- Synchronize comparator and update DAC on falling edge of clk (50 MHz)
  -- This minimizes loop delay and matches TDC ADC timing
  p_fast_loop : process(clk_fast)
    variable v_start_falling : std_logic;
  begin
    if rising_edge(clk_fast) then
      -- Sync comparator
      comp_sync0_fast <= analog_in;
      comp_sync1_fast <= comp_sync0_fast;

      -- Sync 50 MHz clock to detect edges
      ref_sync0 <= clk;
      ref_sync1 <= ref_sync0;
      ref_sync2 <= ref_sync1;

      v_start_falling := not ref_sync2 and ref_sync2_prev;
      ref_sync2_prev  <= ref_sync2;

      if reset = '1' then
        dac_feedback <= '0';
      else
        -- Update DAC on falling edge of 50 MHz clock
        if v_start_falling = '1' then
          dac_feedback <= comp_sync1_fast;
        end if;
      end if;
    end if;
  end process;

  dac_out <= dac_feedback;

  -- ========================================================================
  -- CIC Input Generation (clk domain - 50 MHz)
  -- ========================================================================
  -- Convert 1-bit DAC to bipolar multi-bit signal
  -- Use DELAYED DAC value to match TDC ADC behavior (integrate previous DAC)
  p_cic_input : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        cic_input   <= (others => '0');
        dac_reg     <= '0';
        dac_delayed <= '0';
      elsif trigger_enable = '1' then
        -- Capture current DAC state (updated on falling edge)
        dac_reg     <= dac_feedback;
        -- Delay by 1 cycle to match TDC ADC (uses DAC from previous period)
        dac_delayed <= dac_reg;

        -- Map Delayed DAC bit to bipolar amplitude
        if dac_delayed = '1' then
          cic_input <= C_DAC_AMPLITUDE;
        else
          cic_input <= -C_DAC_AMPLITUDE;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- CIC SINC3 Decimator (clk domain - 50 MHz)
  -- ========================================================================
  -- Runs at full 50 MHz clock rate with ce='1' (always enabled)
  -- Output valid pulses at 50 MHz / GC_DECIMATION = 97.66 kHz
  i_cic : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => GC_DECIMATION,
      GC_INPUT_WIDTH  => C_MULTIBIT_WIDTH, -- Multi-bit mode (matches TDC)
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk          => clk,              -- Run at 50 MHz ADC clock
      reset        => reset,
      data_in      => '0',              -- Not used in multi-bit mode
      data_in_wide => cic_input,        -- Multi-bit signed input
      ce           => trigger_enable,   -- Always enabled when trigger active
      data_out     => cic_data_out,
      valid        => cic_valid_out
    );

  -- ========================================================================
  -- CDC: CIC Output to clk_sys domain (toggle-based pulse synchronizer)
  -- ========================================================================
  -- At ~97.66 kHz output rate, we have ~1000 clk_sys cycles between samples.
  -- Toggle-based CDC is robust and simple for this slow rate.

  -- Hold data and toggle on valid (clk domain)
  p_cic_toggle : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        cic_toggle_clk <= '0';
        cic_data_hold  <= (others => '0');
      elsif cic_valid_out = '1' then
        cic_toggle_clk <= not cic_toggle_clk;
        cic_data_hold  <= cic_data_out;
      end if;
    end if;
  end process;

  -- Synchronize toggle to clk_sys domain
  p_toggle_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        cic_toggle_sync0 <= '0';
        cic_toggle_sync1 <= '0';
        cic_toggle_sync2 <= '0';
        cic_valid_sys    <= '0';
        cic_out_sys      <= (others => '0');
      else
        -- 2-FF synchronizer for toggle signal
        cic_toggle_sync0 <= cic_toggle_clk;
        cic_toggle_sync1 <= cic_toggle_sync0;
        cic_toggle_sync2 <= cic_toggle_sync1;

        -- Edge detect: XOR of sync1 and sync2 detects toggle
        cic_valid_sys <= cic_toggle_sync1 xor cic_toggle_sync2;

        -- Capture data when valid detected (data is stable by now)
        if (cic_toggle_sync1 xor cic_toggle_sync2) = '1' then
          cic_out_sys <= signed(cic_data_hold);
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- FIR Filters (clk_sys domain after CDC)
  -- ========================================================================
  i_eq : entity work.fir_equalizer
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk_sys,
      reset     => reset,
      data_in   => std_logic_vector(cic_out_sys),
      valid_in  => cic_valid_sys,
      data_out  => eq_data_out,
      valid_out => eq_valid_out
    );

  -- Final low-pass filter
  i_lp : entity work.fir_lowpass
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk_sys,
      reset     => reset,
      data_in   => eq_data_out,
      valid_in  => eq_valid_out,
      data_out  => lp_data_out,
      valid_out => lp_valid_out
    );

  -- Streaming output interface (matches TDC ADC)
  sample_data  <= lp_data_out;
  sample_valid <= lp_valid_out;

end architecture rtl;
