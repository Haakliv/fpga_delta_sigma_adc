-- ************************************************************************
-- Pure 1-Bit Delta-Sigma ADC
-- ************************************************************************
-- Architecture:
--   Comparator → Digital Integrator → 1-bit DAC feedback
--   CIC Sinc³ Decimator (unity DC gain via internal C_SCALE_SHIFT)
--   FIR Equalizer + Low-pass (DC gain = 1.0)
--   Q-format voltage conversion: V = (y*600)/2^15 + 600 mV
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
use work.dsp_utils_pkg.all;

entity passiveless_adc_top is
  generic(
    GC_DATA_WIDTH : positive := 16      -- Output data width
  );
  port(
    -- Clocks and reset
    clk_sys                   : in  std_logic; -- System clock (100 MHz)
    clk_400                   : in  std_logic; -- High-speed clock for delta-sigma loop (400 MHz)
    ref_clock                 : in  std_logic; -- Reference clock for sampling (2 MHz)
    reset                     : in  std_logic;
    -- GPIO IP interface (connects to adc_system GPIO exports at top level)
    comparator_in             : in  std_logic; -- From adc_system comp_out_export
    dac_out_bit               : out std_logic; -- To adc_system slope_din_export

    -- Streaming sample output
    sample_data               : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid              : out std_logic;
    -- Debug outputs
    -- Debug outputs (for SignalTap/analysis)
    debug_activity_counter    : out std_logic_vector(15 downto 0);
    debug_valid_counter       : out std_logic_vector(7 downto 0);
    debug_cic_counter         : out std_logic_vector(7 downto 0);
    debug_sample_counter      : out std_logic_vector(7 downto 0);
    debug_comparator_out      : out std_logic; -- Comparator level (comp_s2)
    debug_dac_out_ff          : out std_logic; -- DAC bitstream output
    debug_lp_data_out         : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0); -- LP filter output
    debug_sample_ce_sys       : out std_logic; -- 2 MHz delta-sigma sample clock
    debug_cic_valid_out       : out std_logic; -- CIC decimation valid (~31kHz)
    debug_dac_bitstream_hold  : out std_logic; -- DAC bitstream at CIC sample point
    debug_dac_bitstream_sync2 : out std_logic; -- DAC bitstream after CDC
    debug_dsm_integrator      : out std_logic_vector(31 downto 0); -- DSM integrator value
    debug_comp_s0             : out std_logic; -- Comparator sync stage 0
    debug_comp_s1             : out std_logic; -- Comparator sync stage 1
    debug_comp_s2             : out std_logic; -- Comparator sync stage 2 (to DSM)
    debug_mod_bit             : out std_logic; -- Internal modulator bit
    debug_comp_duty_ones      : out std_logic_vector(31 downto 0); -- Comparator '1' count in window
    debug_comp_duty_snap      : out std_logic_vector(31 downto 0); -- Snapshot at window end
    debug_comp_duty_valid     : out std_logic -- Pulse when snapshot updated
  );
end entity;

architecture rtl of passiveless_adc_top is

  -- Clock domain alias
  alias clk_dsm is clk_400;             -- High-speed clock (400 MHz) for delta-sigma loop

  -- CIC decimator outputs (in DSM domain before CDC)
  signal cic_valid_out_dsm : std_logic;
  signal cic_data_out_dsm  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);

  -- Toggle-based CDC for CIC output (clocks from same PLL - related clocks)
  -- Use toggle handshake: fast domain toggles on valid, slow domain detects edge
  signal cic_data_reg_dsm : std_logic_vector(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal cic_toggle_dsm   : std_logic                                    := '0'; -- Toggle in 400 MHz domain

  -- CIC outputs in clk_sys domain (post-CDC)
  signal cic_toggle_meta : std_logic := '0'; -- Metastability stage
  signal cic_toggle_sync : std_logic := '0'; -- Synchronized toggle
  signal cic_toggle_prev : std_logic := '0'; -- Previous toggle for edge detect
  signal cic_data_out    : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out   : std_logic := '0';

  -- Direct ref_clock synchronization for 2 MHz CE
  signal ref_sys_s0   : std_logic := '0';
  signal ref_sys_s1   : std_logic := '0';
  signal ref_sys_s2   : std_logic := '0';
  signal start_ce_sys : std_logic := '0'; -- 2 MHz CE (not used by ΔΣ, kept for debug)

  -- Equalizer signals
  signal eq_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out : std_logic;

  -- Low-pass filter signals
  signal lp_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out : std_logic;

  -- Status monitoring
  signal activity_counter : unsigned(15 downto 0) := (others => '0');
  signal valid_counter    : unsigned(7 downto 0)  := (others => '0');
  signal cic_counter      : unsigned(7 downto 0)  := (others => '0');
  signal sample_counter   : unsigned(7 downto 0)  := (others => '0');

  -- Component-less differential feedback: Internal signal for comparator output
  signal s_comparator_out_internal : std_logic; -- Output from differential GPIO comparator

  -- DSM Comparator Sampling (post-DAC-settling)
  -- 3-FF synchronizer for raw comparator to clk_dsm domain
  signal comp_s0 : std_logic := '0';    -- First synchronizer stage
  signal comp_s1 : std_logic := '0';    -- Second synchronizer stage
  signal comp_s2 : std_logic := '0';    -- Third synchronizer stage (clean synchronized level)

  -- DAC decision signals
  signal mod_bit              : std_logic           := '0'; -- Internal DSM bit (1=high Vfb, 0=low Vfb)
  signal dac_integrator_ff    : std_logic           := '0'; -- Same as mod_bit for CIC input
  signal dac_out_ff           : std_logic           := '0'; -- Final DAC output to pin
  signal dsm_integrator_value : signed(31 downto 0) := (others => '0'); -- For debug output

  -- DSM pipeline registers (for 400 MHz timing closure)
  signal integ_reg           : signed(31 downto 0) := (others => '0'); -- Saturated integrator
  signal integ_unclamped_reg : signed(31 downto 0) := (others => '0'); -- Unclamped next value

  -- Comparator duty cycle monitoring (exposed for SignalTap)
  signal   comp_ones  : unsigned(31 downto 0) := (others => '0'); -- Count of comp_s2='1' in window
  signal   comp_total : unsigned(31 downto 0) := (others => '0'); -- Total cycles in window
  signal   comp_snap  : unsigned(31 downto 0) := (others => '0'); -- Snapshot of comp_ones at window end
  signal   comp_valid : std_logic             := '0'; -- Pulse when snapshot updated
  constant C_WINDOW   : unsigned(31 downto 0) := to_unsigned(400000, 32); -- 1 us at 400 MHz

  -- Reference edge synchronization (3-FF scalar chain for MTBF at 400 MHz)
  signal ref_sync0 : std_logic := '0';  -- First synchronizer stage
  signal ref_sync1 : std_logic := '0';  -- Second synchronizer stage
  signal ref_sync2 : std_logic := '0';  -- Third synchronizer stage

  -- Synchronized reset for clk_dsm domain
  signal reset_dsm : std_logic;

  -- Attributes for CDC synchronizers (MTBF and proper recognition)
  -- Each scalar synchronizer stage gets marked individually
  attribute altera_attribute : string;

  -- ref_sync chain (ref_clock -> clk_dsm)
  attribute altera_attribute of ref_sync0 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";
  attribute altera_attribute of ref_sync1 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";
  attribute altera_attribute of ref_sync2 : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";

  -- CIC output FIFO handles CDC (no manual synchronizer attributes needed)

  -- ========================================================================
  -- Signals for Multi-bit Delta-Sigma Loop
  -- ========================================================================

  -- Millivolt conversion for UART output (LP filter → mV)
  signal mv_code : unsigned(15 downto 0) := (others => '0');
begin

  -- TODO: Dither?

  -- ========================================================================
  -- Reset Synchronizer for clk_dsm domain
  -- ========================================================================
  i_reset_sync : entity work.reset_synchronizer
    generic map(
      GC_ACTIVE_LOW => false
    )
    port map(
      clk       => clk_dsm,
      async_rst => reset,
      sync_rst  => reset_dsm
    );

  -- Drive GPIO IP feedback input (IP drives the physical FEEDBACK_N pin)
  dac_out_bit <= dac_out_ff;

  -- Use GPIO IP differential comparator output directly
  s_comparator_out_internal <= not comparator_in;

  -- ========================================================================
  -- Reference Clock Synchronization (Pure 3-FF scalar synchronizer for CDC)
  -- ========================================================================
  -- Synchronize reference clock edge to DSM domain (400 MHz)
  -- All flops clocked by clk_dsm only - NO reset to avoid mixed clock domains
  p_ref_sync : process(clk_dsm)
  begin
    if rising_edge(clk_dsm) then
      ref_sync0 <= ref_clock;
      ref_sync1 <= ref_sync0;
      ref_sync2 <= ref_sync1;
    end if;
  end process;

  -- ========================================================================
  -- TODO: For debug, will be removed - Comparator Duty Cycle Monitor
  -- ========================================================================
  -- Monitor comp_s2 duty cycle over a fixed window to verify correct operation
  -- This helps diagnose if the feedback loop is working properly
  -- comp_snap / C_WINDOW should match expected duty cycle (Vin/Vref)
  -- ========================================================================
  p_comp_monitor : process(clk_dsm)
  begin
    if rising_edge(clk_dsm) then
      if reset_dsm = '1' then
        comp_ones  <= (others => '0');
        comp_total <= (others => '0');
        comp_snap  <= (others => '0');
        comp_valid <= '0';
      else
        comp_total <= comp_total + 1;
        if comp_s2 = '1' then
          comp_ones <= comp_ones + 1;
        end if;

        if comp_total = C_WINDOW then
          comp_snap  <= comp_ones;      -- freeze 1-count for debug
          comp_ones  <= (others => '0');
          comp_total <= (others => '0');
          comp_valid <= '1';
        else
          comp_valid <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- DSM Comparator Synchronization (Pure 3-FF scalar synchronizer for CDC)
  -- ========================================================================
  -- Synchronize raw comparator output to clk_dsm domain for DSM feedback
  -- This ensures DSM samples the comparator AFTER the DAC has settled
  -- (unlike s_comp_level_at_start which is captured at TDC window start time)
  p_comp_sync : process(clk_dsm)
  begin
    if rising_edge(clk_dsm) then
      -- Pure 3-FF scalar synchronizer: NO reset clause to avoid CDC violation
      -- Power-on initialization handles reset state (default '0')
      comp_s0 <= s_comparator_out_internal;
      comp_s1 <= comp_s0;
      comp_s2 <= comp_s1;               -- comp_s2 is the clean synchronized comparator level
    end if;
  end process;

  -- ========================================================================
  -- CDC for 2 MHz Start Pulse (clk_dsm → clk_sys)
  -- ========================================================================
  -- Always-on 2 MHz CE generation from ref_clock (independent of TDC activity)
  -- This ensures CIC continues to run after TDC handoff
  p_ref_clock_ce : process(clk_sys)
    variable v_prev        : std_logic                 := '0';
    variable v_edge_count  : integer range 0 to 100000 := 0; -- Saturate instead of overflow
    variable v_cycle_count : integer range 0 to 100000 := 0; -- Saturate instead of overflow
  begin
    if rising_edge(clk_sys) then
      -- Saturate cycle counter to prevent overflow
      if v_cycle_count < 100000 then
        v_cycle_count := v_cycle_count + 1;
      end if;

      -- 3-FF synchronizer (no reset clause to avoid dependency on reset)
      ref_sys_s0 <= ref_clock;
      ref_sys_s1 <= ref_sys_s0;
      ref_sys_s2 <= ref_sys_s1;

      -- 1-cycle CE pulse on rising edge ONLY of ref_clock (not both edges!)
      start_ce_sys <= ref_sys_s2 and not v_prev;

      -- Debug: report first few edges and every 1000th edge
      if (ref_sys_s2 and not v_prev) = '1' then
        -- Saturate edge counter to prevent overflow
        if v_edge_count < 100000 then
          v_edge_count := v_edge_count + 1;
        end if;
      end if;

      v_prev := ref_sys_s2;
    end if;
  end process;

  -- ========================================================================
  -- True 1-Bit Delta-Sigma Loop (Comparator-Based)
  -- ========================================================================
  -- 
  -- Classic ΔΣ feedback path:
  --   Vin → [Comparator 1-bit] → [Digital Integrator] → [1-bit DAC] → Vfb → Comparator
  --                  ↑                                        |
  --                  └────────────────────────────────────────┘
  -- 
  -- The comparator output (s_comp_level_at_start) is the 1-bit quantizer:
  --   '1' = Vin > Vfb (error positive, need to increase duty cycle)
  --   '0' = Vin < Vfb (error negative, need to decrease duty cycle)

  p_sigma_delta : process(clk_dsm, reset_dsm)
    variable v_next_unclamped : signed(31 downto 0);
    constant C_INTEG_GAIN     : signed(31 downto 0) := to_signed(1, 32); -- K=1 optimal (K=10 gives same result)
    constant C_LIM            : signed(31 downto 0) := to_signed(2_000_000, 32);
  begin
    if reset_dsm = '1' then
      integ_reg            <= (others => '0');
      integ_unclamped_reg  <= (others => '0');
      mod_bit              <= '0';
      dsm_integrator_value <= (others => '0');
    elsif rising_edge(clk_dsm) then
      -- Stage 2: Saturate previous cycle's unclamped result
      if integ_unclamped_reg > C_LIM then
        integ_reg <= C_LIM;
      elsif integ_unclamped_reg < -C_LIM then
        integ_reg <= -C_LIM;
      else
        integ_reg <= integ_unclamped_reg;
      end if;

      -- 1-bit quantizer: integrator >= 0 means \"want HIGH Vfb\"
      if integ_reg >= 0 then
        mod_bit <= '1';                 -- HIGH Vfb desired
      else
        mod_bit <= '0';                 -- LOW Vfb desired
      end if;

      -- Stage 1: Compute unclamped sum for next cycle
      -- comp_s2 = 1 → Vin > Vfb → need higher Vfb → want mod_bit=1
      if comp_s2 = '1' then
        v_next_unclamped := integ_reg + C_INTEG_GAIN;
      else
        v_next_unclamped := integ_reg - C_INTEG_GAIN;
      end if;
      integ_unclamped_reg <= v_next_unclamped;

      -- Debug: expose saturated integrator value
      dsm_integrator_value <= integ_reg;
    end if;
  end process;

  -- Register mod_bit for timing and CIC input
  dac_integrator_ff <= mod_bit;
  dac_out_ff        <= mod_bit;

  -- The feedback loop automatically adjusts DAC duty cycle to match input voltage.
  -- CIC/EQ/LP measure this duty cycle (average) with filtering.

  -- Convert ADC output to millivolts (0..1200 mV range)
  -- With proper ΔΣ loop: duty cycle of feedback = Vin/VFS
  -- CIC/EQ/LP measure this duty cycle (average) with filtering
  -- Simple scaling: output_mV = (lp_data × scale_factor) + offset

  p_mv_from_lp : process(clk_sys)
    variable v_lp_signed : signed(GC_DATA_WIDTH - 1 downto 0);
    variable v_scaled_mv : signed(31 downto 0);
    variable v_mv_count  : integer range 0 to 100000 := 0;
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        mv_code    <= (others => '0');
        v_mv_count := 0;
      elsif lp_valid_out = '1' then
        v_mv_count := v_mv_count + 1;

        -- Convert signed Q-format [-1, +1) to millivolts [0, 1200]
        -- CIC already removes its DC gain (R³) internally via C_SCALE_SHIFT
        -- LP output is Q-format: m = y / 2^(W-1)
        -- Voltage mapping: V = 600*m + 600 (NOT 1200*m, which doubles the swing)
        -- Combined: V = (y*600)/2^(W-1) + 600
        v_lp_signed := signed(lp_data_out);

        -- Scale: shift by (GC_DATA_WIDTH - 1) to undo Q-format, NOT by C_CIC_SHIFT
        -- Multiply creates 64-bit result, shift, then resize to 32-bit
        v_scaled_mv := resize(shift_right(resize(v_lp_signed, 32) * to_signed(600, 32),
                                          GC_DATA_WIDTH - 1), 32);

        -- Add midscale offset
        v_scaled_mv := v_scaled_mv + to_signed(600, 32);

        -- Saturate to 0..1200 mV range
        if v_scaled_mv < 0 then
          mv_code <= (others => '0');
        elsif v_scaled_mv > to_signed(1200, 32) then
          mv_code <= to_unsigned(1200, 16);
        else
          mv_code <= unsigned(v_scaled_mv(15 downto 0));
        end if;
      end if;
    end if;
  end process;

  -- 3-FF scalar synchronizer for dac_out_ff (actual ΔΣ output bitstream)
  -- This is the CORRECT signal to filter - it's the 1-bit quantizer output
  -- that encodes the input signal via duty cycle modulation
  -- ========================================================================
  -- CIC Decimator at 400 MHz - samples TRUE bitstream without aliasing!
  -- ========================================================================
  -- Running CIC at DSM frequency preserves all bit-density information
  -- Decimates by 256: 400 MHz / 256 = 1.5625 MHz output rate (same as before)
  -- This eliminates the 4:1 subsampling aliasing from the old 100MHz approach
  i_cic : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => 256,           -- Decimate 400 MHz → 1.5625 MHz
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk      => clk_dsm,              -- 400 MHz DSM clock
      reset    => reset_dsm,            -- Reset in DSM domain
      data_in  => dac_integrator_ff,    -- Raw bitstream (no CDC!)
      ce       => '1',                  -- Sample EVERY DSM cycle
      data_out => cic_data_out_dsm,     -- Output in DSM domain
      valid    => cic_valid_out_dsm     -- Valid in DSM domain
    );

  -- ========================================================================
  -- Toggle-based CDC: CIC output 400 MHz → 100 MHz (related clocks)
  -- ========================================================================
  -- Both clocks are from same PLL, so they have fixed phase relationship.
  -- Use toggle handshake protocol:
  --   1) Fast domain (400 MHz): Toggle signal when valid data available
  --   2) Slow domain (100 MHz): Synchronize toggle, detect edge = new sample
  -- This ensures reliable capture without racing CE and valid signals.
  -- Add SDC constraint: set_clock_groups -physically_exclusive -group clk_dsm -group clk_sys
  -- Or: set_multicycle_path -from [clk_dsm] -to [clk_sys] -setup 4
  -- ========================================================================

  -- Fast domain (400 MHz): Register data and toggle on valid
  p_cic_toggle_gen : process(clk_dsm)
  begin
    if rising_edge(clk_dsm) then
      if reset_dsm = '1' then
        cic_data_reg_dsm <= (others => '0');
        cic_toggle_dsm   <= '0';
      elsif cic_valid_out_dsm = '1' then
        cic_data_reg_dsm <= cic_data_out_dsm;
        cic_toggle_dsm   <= not cic_toggle_dsm; -- Toggle on each valid sample
      end if;
    end if;
  end process;

  -- Slow domain (100 MHz): Synchronize toggle and detect edges
  p_cic_toggle_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        cic_toggle_meta <= '0';
        cic_toggle_sync <= '0';
        cic_toggle_prev <= '0';
        cic_data_out    <= (others => '0');
        cic_valid_out   <= '0';
      else
        -- 2-FF synchronizer for toggle (handles metastability)
        cic_toggle_meta <= cic_toggle_dsm;
        cic_toggle_sync <= cic_toggle_meta;

        -- Edge detection: toggle changed = new sample available
        if cic_toggle_sync /= cic_toggle_prev then
          cic_toggle_prev <= cic_toggle_sync;
          cic_data_out    <= cic_data_reg_dsm; -- Sample data (treat as related-clock path in SDC)
          cic_valid_out   <= '1';
        else
          cic_valid_out <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- FIR Equalizer (on clk_sys for timing closure)
  -- ========================================================================
  -- Takes CDC'd output from fast CIC (6.25 MHz multi-bit samples)
  i_eq : entity work.fir_equalizer
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk_sys,
      reset     => reset,
      data_in   => cic_data_out,        -- From CIC decimator
      valid_in  => cic_valid_out,       -- CIC valid signal
      data_out  => eq_data_out,
      valid_out => eq_valid_out
    );

  -- ========================================================================
  -- FIR Equalizer (on clk_sys for timing closure)
  -- ========================================================================
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

  -- Debug monitor for LP filter output with mV conversion
  p_lp_out_monitor : process(clk_sys)
    variable v_lp_out_count : integer range 0 to 10000 := 0;
  begin
    if rising_edge(clk_sys) then
      if lp_valid_out = '1' then
        v_lp_out_count := v_lp_out_count + 1;
      end if;
    end if;
  end process;

  -- Streaming output: Report actual measured voltage as millivolts
  -- The CIC/EQ/LP chain processes both DC and AC signals with unity gain
  -- Final scaling converts filtered output to 0-1200mV range
  sample_data  <= std_logic_vector(resize(mv_code, GC_DATA_WIDTH));
  sample_valid <= lp_valid_out;

  -- ========================================================================
  -- Status Monitoring
  -- ========================================================================
  p_status : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        activity_counter <= (others => '0');
        valid_counter    <= (others => '0');
        cic_counter      <= (others => '0');
        sample_counter   <= (others => '0');
      else
        activity_counter <= activity_counter + 1;

        if lp_valid_out = '1' then
          valid_counter <= valid_counter + 1;
        end if;

        if cic_valid_out = '1' then
          cic_counter <= cic_counter + 1;
        end if;

        if start_ce_sys = '1' then
          sample_counter <= sample_counter + 1;
        end if;

        -- Overflow counter removed (was TDC-specific)
      end if;
    end if;
  end process;

  -- Debug outputs
  -- Repurpose activity_counter for delta-sigma diagnostics
  debug_activity_counter(0)           <= ref_clock; -- Bit 0: ref_clock input (should toggle at 2 MHz)
  debug_activity_counter(1)           <= ref_sync0; -- Bit 1: First sync stage
  debug_activity_counter(2)           <= ref_sync1; -- Bit 2: Second sync stage
  debug_activity_counter(3)           <= ref_sync2; -- Bit 3: Third sync stage
  debug_activity_counter(4)           <= start_ce_sys; -- Bit 4: 2MHz CE for CIC
  debug_activity_counter(5)           <= dac_out_ff; -- Bit 5: DAC output bitstream
  debug_activity_counter(6)           <= cic_valid_out; -- Bit 6: CIC valid (~1.56 MHz)
  debug_activity_counter(7)           <= lp_valid_out; -- Bit 7: LP valid
  debug_activity_counter(15 downto 8) <= (others => '0'); -- Bits 15:8: Reserved (was boot FSM signals)
  debug_valid_counter                 <= std_logic_vector(valid_counter);
  debug_cic_counter                   <= std_logic_vector(cic_counter);
  debug_sample_counter                <= std_logic_vector(sample_counter);

  -- Additional debug outputs for SignalTap
  debug_comparator_out <= s_comparator_out_internal; -- Comparator output from GPIO IP
  debug_dac_out_ff     <= mod_bit;      -- Internal modulator bit for monitoring

  -- LP filter output debug (for stuck-at-zero detection)
  debug_lp_data_out <= lp_data_out;     -- Raw LP filter output before mV conversion

  -- DSM sample clock debug (for watchdog monitoring)
  debug_sample_ce_sys <= start_ce_sys;  -- Always-on 2 MHz CE from ref_clock (not used by ΔΣ loop)
  debug_cic_valid_out <= cic_valid_out; -- CIC decimation valid (~1.56 MHz)

  -- DAC bitstream debug outputs (for duty cycle monitoring in testbench)
  debug_dac_bitstream_hold  <= dac_integrator_ff; -- Raw ΔΣ bitstream at 400 MHz
  debug_dac_bitstream_sync2 <= dac_integrator_ff; -- No longer synced (CIC at 400MHz)

  -- DSM loop debug
  debug_dsm_integrator  <= std_logic_vector(dsm_integrator_value);
  debug_comp_s0         <= comp_s0;
  debug_comp_s1         <= comp_s1;
  debug_comp_s2         <= comp_s2;
  debug_mod_bit         <= mod_bit;
  debug_comp_duty_ones  <= std_logic_vector(comp_ones);
  debug_comp_duty_snap  <= std_logic_vector(comp_snap);
  debug_comp_duty_valid <= comp_valid;

end architecture;

