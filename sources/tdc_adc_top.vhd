-- TDC-Based Delta-Sigma ADC Top Level

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
use work.dsp_utils_pkg.all;

entity tdc_adc_top is
  generic(
    -- Decimation Factor (R)
    -- Increased to 8192 for 50MHz operation: 50MHz / 8192 = 6.1 kHz output rate
    -- Prevents UART overflow (was 384 @ 2MHz = 5.2kHz)
    GC_DECIMATION    : positive := 8192;
    GC_DATA_WIDTH    : positive := 16;  -- Output data width
    GC_TDC_OUTPUT    : positive := 16;  -- TDC output width
    GC_SIM           : boolean  := false; -- Simulation mode for TDC (enables debug reports)
    GC_FAST_SIM      : boolean  := false; -- Fast simulation mode (reduces boot timeouts)
    GC_OPEN_LOOP     : boolean  := false -- Open-loop test mode (bypass feedback)
  );
  port(
    -- Clocks and reset
    clk_sys             : in  std_logic; -- System clock (e.g., 100 MHz)
    clk_tdc             : in  std_logic; -- TDC fast clock (e.g., 400-600 MHz)
    reset               : in  std_logic;
    -- Reference clock for TDC start edge
    ref_clock           : in  std_logic;
    -- GPIO IP interface (connects to adc_system GPIO exports at top level)
    comparator_in       : in  std_logic; -- From adc_system comp_out_export
    dac_out_bit         : out std_logic; -- To adc_system slope_din_export
    -- Optional trigger input (when '1', sampling is enabled; when '0', sampling is disabled)
    trigger_enable      : in  std_logic := '1'; -- Default '1' for continuous sampling -- @suppress "Unused port: trigger_enable is not used in fpga_lib.tdc_adc_top(rtl)"
    -- Open-loop test mode (GC_OPEN_LOOP=true only)
    open_loop_dac_duty  : in  std_logic := '0'; -- Fixed DAC output in open-loop mode

    -- Streaming sample output
    sample_data         : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid        : out std_logic;
    -- Debug outputs for characterization
    debug_tdc_out       : out signed(GC_TDC_OUTPUT - 1 downto 0);
    debug_tdc_valid     : out std_logic;
    -- TDC Monitor outputs (for isolating TDC sanity check)
    tdc_monitor_code    : out signed(GC_TDC_OUTPUT - 1 downto 0); -- Raw TDC code
    tdc_monitor_center  : out signed(GC_TDC_OUTPUT - 1 downto 0); -- Calibrated center
    tdc_monitor_diff    : out signed(GC_TDC_OUTPUT - 1 downto 0); -- tdc_code - center
    tdc_monitor_dac     : out std_logic; -- DAC bit at sample
    tdc_monitor_valid   : out std_logic; -- Monitor data valid
    -- Runtime control
    disable_tdc_contrib : in  std_logic := '0'; -- When '1', disable TDC contribution (CIC-only mode)
    disable_eq_filter   : in  std_logic := '0'; -- When '1', bypass EQ filter (CIC direct to LP)
    disable_lp_filter   : in  std_logic := '0' -- When '1', bypass LP filter (EQ direct to output)
  );
end entity;

architecture rtl of tdc_adc_top is
  -- Note: TDC contribution disable is now controlled by disable_tdc_contrib port (runtime)

  -- Runtime filter bypass controls (driven by input ports)
  -- When disabled, filters are bypassed to isolate delta-sigma path from FIR processing

  -- Runtime selectable TDC-free mode
  -- When disable_tdc_contrib='1' we also bypass the TDC-generated CE path and
  -- feed the CIC with the ref_clock edge based CE plus a directly synchronized
  -- DAC bitstream. This removes the TDC toggle/CDC path from the signal chain,
  -- ensuring the pure 1-bit ∆Σ path can be characterized independently.
  signal use_tdc_free_debug_mode : std_logic;

  -- TDC Center Calibration
  signal   tdc_center_cal       : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal   tdc_center_tdc       : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  constant C_CENTER_TRACK_SHIFT : integer                            := 8;
  signal   tdc_center_runtime   : signed(31 downto 0)                := (others => '0');

  signal cal_sample_cnt : unsigned(7 downto 0) := (others => '0');
  signal cal_done       : std_logic            := '0';

  -- TDC Monitor Mode signals (for debugging TDC sanity)
  signal tdc_mon_code   : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_mon_center : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_mon_diff   : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_mon_dac    : std_logic                          := '0';
  signal tdc_mon_valid  : std_logic                          := '0';

  -- TDC signals
  signal ref_phases      : std_logic_vector(0 downto 0);
  signal tdc_out         : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_valid       : std_logic;
  signal tdc_overflow    : std_logic;
  signal tdc_lost_sample : std_logic;

  -- Coarse bias configuration
  -- Default value: 220 (0xDC) - Shifted from 180 to provide positive headroom
  -- Previous value (180) still saw peaks at +31872 (near +32767 limit)
  constant C_DEFAULT_COARSE_BIAS : unsigned(7 downto 0) := to_unsigned(220, 8);

  signal coarse_bias    : unsigned(7 downto 0) := C_DEFAULT_COARSE_BIAS;
  signal coarse_bias_s0 : unsigned(7 downto 0) := C_DEFAULT_COARSE_BIAS;

  signal coarse_bias_tdc : unsigned(7 downto 0) := C_DEFAULT_COARSE_BIAS;

  -- TDL Centering Calibration
  signal   coarse_bias_cal     : unsigned(7 downto 0)  := C_DEFAULT_COARSE_BIAS;
  signal   tdl_cal_use_cal     : std_logic             := '0';
  signal   fine_at_comp_out    : unsigned(15 downto 0) := (others => '0');
  signal   fine_valid_out      : std_logic             := '0';
  constant C_TDL_CAL_SAMPLES   : natural               := 8;
  signal   tdl_cal_done        : std_logic             := '0';
  signal   tdl_cal_sample_cnt  : unsigned(3 downto 0)  := (others => '0');
  signal   tdl_cal_fine_acc    : unsigned(19 downto 0) := (others => '0');
  signal   tdl_cal_timeout_cnt : unsigned(15 downto 0) := (others => '0');

  -- TDL calibration settle counter (moved from variable to signal for timing)
  constant C_TDL_CAL_SETTLE : natural              := 1000;
  signal   tdl_settle_cnt   : unsigned(9 downto 0) := (others => '0'); -- 10 bits for 1000
  signal   tdl_settling     : std_logic            := '0'; -- Registered flag to break timing path

  -- TDL calibration pipeline registers (break timing path from fine_at_comp to tdl_cal_fine_acc)
  -- Stage 1: Register raw fine value
  signal fine_at_comp_reg    : unsigned(15 downto 0) := (others => '0');
  signal fine_valid_reg      : std_logic             := '0';
  -- Stage 2: Pre-computed sum for comparison (breaks add+compare timing path)
  signal tdl_cal_sum_reg     : unsigned(19 downto 0) := (others => '0'); -- tdl_cal_fine_acc + fine
  signal tdl_cal_sum_valid   : std_logic             := '0'; -- Valid flag for sum
  signal tdl_cal_last_sample : std_logic             := '0'; -- Was this the 8th sample?

  -- CDC signals
  signal tdc_valid_sys        : std_logic := '0';
  signal dac_bitstream_hold   : std_logic := '0';
  signal dac_bitstream_sync0  : std_logic := '0';
  signal dac_bitstream_sync1  : std_logic := '0';
  signal dac_bitstream_sync2  : std_logic := '0';
  signal closed_loop_en_tdc   : std_logic := '0';
  signal closed_loop_en_sync0 : std_logic := '0';
  signal closed_loop_en_sync1 : std_logic := '0';
  signal closed_loop_en_sync2 : std_logic := '0';
  -- Note: closed_loop_drive sync chain removed (was never read)

  -- CIC decimator signals
  signal cic_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out : std_logic;

  -- TDC-free debug mode signals (ref_clock-based CE, direct DAC sync)
  -- CE driven by ref_clock edges (~2MHz), NOT by counter or TDC
  -- 2.66x mismatch fixed by:
  --   DAC Amplitude: 16384 -> 24576
  --   Decimation Shift: 9 -> 8
  --   TDC Gain: 1/4 -> 1/1
  -- Result: Coarse step = 192, Fine step = 192.
  signal ref_sys0          : std_logic := '0';
  signal ref_sys1          : std_logic := '0';
  signal ref_sys2          : std_logic := '0';
  signal ref_sys_prev      : std_logic := '0';
  signal ce_ref            : std_logic := '0'; -- Rising edge of ref_clock in clk_sys domain
  signal dac_sync0         : std_logic := '0';
  signal dac_sync1         : std_logic := '0';
  signal dac_dbg_hold      : std_logic := '0'; -- DAC bit captured at ce_ref for debug CIC
  -- Debug CIC outputs
  signal dbg_cic_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal dbg_cic_valid_out : std_logic;
  -- Final mux outputs (select between TDC path and debug path)
  signal final_cic_data    : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal final_cic_valid   : std_logic;

  -- Multi-Bit TDC Decimator
  constant C_TDC_ACC_WIDTH  : natural                              := 32;
  signal   tdc_accumulator  : signed(C_TDC_ACC_WIDTH - 1 downto 0) := (others => '0');
  -- Increased width to 14 bits to support GC_DECIMATION=8192 (requires 13 bits)
  signal   tdc_dec_counter  : unsigned(13 downto 0)                := (others => '0');
  signal   tdc_dec_data_out : signed(GC_DATA_WIDTH - 1 downto 0)   := (others => '0');
  signal   tdc_dec_valid    : std_logic                            := '0';

  -- Filter signals
  signal eq_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out : std_logic;
  signal lp_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out : std_logic;

  -- Bypass mux signals (selected by disable_eq_filter and disable_lp_filter ports)
  signal eq_mux_out   : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_mux_valid : std_logic;
  signal lp_mux_out   : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_mux_valid : std_logic;

  -- Combined output path (CIC/EQ/LP + TDC contribution)
  signal combined_data_out  : signed(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal combined_valid_out : std_logic                          := '0';

  -- TDC contribution accumulator (runs parallel to CIC, added after LP filter)
  signal tdc_contrib_acc     : signed(31 downto 0)                := (others => '0');
  signal tdc_contrib_out     : signed(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal tdc_contrib_valid   : std_logic                          := '0';
  signal tdc_contrib_counter : unsigned(15 downto 0)              := (others => '0');

  -- Filter priming state - FIR filters need N samples to prime their delay lines
  -- EQ has 31 taps, LP has 63 taps, so need ~64 LP valid outputs before stable
  constant C_FILTER_PRIME_COUNT : integer := 70; -- Extra margin for safety

  -- Boot timeout constants (selectable via GC_FAST_SIM for faster simulation)
  -- Post-sweep settling: 25µs normal (10000 cycles @ 2.5µs/cycle), 2.5µs fast sim
  constant C_SWEEP_SETTLING_NORMAL : integer              := 10000;
  constant C_SWEEP_SETTLING_FAST   : integer              := 1000;
  -- Dither timeout: 10ms normal (4000000 cycles), 1ms fast sim
  constant C_DITHER_TIMEOUT_NORMAL : integer              := 4000000;
  constant C_DITHER_TIMEOUT_FAST   : integer              := 400000;
  signal   filter_prime_counter    : unsigned(7 downto 0) := (others => '0');
  signal   filter_primed           : std_logic            := '0';

  -- Held TDC contribution for combining with delayed LP output
  signal tdc_contrib_held : signed(GC_DATA_WIDTH - 1 downto 0) := (others => '0');

  -- Comparator and analog input
  signal s_comparator_out_internal : std_logic;
  signal analog_in_mux             : std_logic;

  -- Boot state machine
  type   T_BOOT_STATE          is (ST_SWEEP, ST_DITHER, ST_WAIT_FOR_START, ST_CLOSED_LOOP);
  signal boot_state            : T_BOOT_STATE := ST_SWEEP;
  signal closed_loop_en        : std_logic    := '0';
  signal closed_loop_drive     : std_logic    := '0';
  signal cl_valid_seen         : std_logic    := '0';
  signal cl_switch_pend        : std_logic    := '0';
  signal use_closed_loop       : std_logic    := '0';
  signal use_closed_loop_sync1 : std_logic    := '0';
  signal use_closed_loop_tdc   : std_logic    := '0';
  signal dac_boot_ff           : std_logic    := '0';
  signal dac_out_ff            : std_logic    := '0';
  signal dac_integrator_ff     : std_logic    := '0';

  -- Calibration enable
  signal cal_enable_sys   : std_logic := '0';
  signal cal_enable_sync0 : std_logic := '0';
  signal cal_enable_sync1 : std_logic := '0';
  signal cal_enable_tdc   : std_logic := '0';

  -- Calibration done CDC (clk_tdc -> clk_sys)
  signal cal_done_sync0 : std_logic := '0';
  signal cal_done_sync1 : std_logic := '0';
  signal cal_done_sys   : std_logic := '0';

  -- Boot sweep signals
  signal sweep_duty_counter : unsigned(7 downto 0) := (others => '0');
  signal sweep_period_cnt   : unsigned(7 downto 0) := (others => '0');
  signal sweep_found_cross  : std_logic            := '0';
  signal sweep_cross_duty   : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal sweep_init_integ   : signed(31 downto 0)  := (others => '0');
  signal comp_sync0         : std_logic            := '0';
  signal comp_sync1         : std_logic            := '0';
  signal comp_prev          : std_logic            := '0';

  -- Watchdog signals
  signal cl_watch_cnt : unsigned(15 downto 0) := (others => '0');

  -- TDC sample latch
  signal tdc_sample_latch : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_sample_ready : std_logic                          := '0';

  -- Start pulse detection
  signal start_pulse_pi    : std_logic := '0';
  signal ref_sync2_prev_pi : std_logic := '0';

  -- TDC CDC signals
  signal tdc_out_hold       : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal dac_at_sample      : std_logic                          := '0';
  signal dac_at_sample_prev : std_logic                          := '0';
  signal dac_at_sample_sys  : std_logic                          := '0';
  signal tdc_toggle         : std_logic                          := '0';
  signal tdc_toggle_sync0   : std_logic                          := '0';
  signal tdc_toggle_sync1   : std_logic                          := '0';
  signal tdc_toggle_sync2   : std_logic                          := '0';
  signal tdc_out_sys        : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal new_sample_sys     : std_logic                          := '0';
  signal tdc_overflow_sync  : std_logic_vector(1 downto 0)       := (others => '0');
  signal tdc_lost_sync      : std_logic_vector(1 downto 0)       := (others => '0');

  -- Reference edge synchronization
  signal ref_sync0 : std_logic := '0';
  signal ref_sync1 : std_logic := '0';
  signal ref_sync2 : std_logic := '0';
  signal reset_tdc : std_logic;

  -- Millivolt output
  signal mv_code : unsigned(15 downto 0) := (others => '0');

begin

  -- Reset Synchronizer
  i_reset_sync : entity work.reset_synchronizer
    generic map(
      GC_ACTIVE_LOW => false
    )
    port map(
      clk       => clk_tdc,
      async_rst => reset,
      sync_rst  => reset_tdc
    );

  -- Open-loop test mode: bypass feedback and use fixed DAC duty
  dac_out_bit <= open_loop_dac_duty when GC_OPEN_LOOP else dac_out_ff;

  -- When TDC contribution is disabled, also bypass the TDC timing chain and
  -- run the CIC on the ref_clock CE path. This isolates the pure 1-bit path.
  use_tdc_free_debug_mode <= disable_tdc_contrib;

  -- Debug outputs for TDC characterization
  debug_tdc_out   <= tdc_out;
  debug_tdc_valid <= tdc_valid;

  -- Use GPIO IP differential comparator output directly
  s_comparator_out_internal <= comparator_in;

  -- Mux between external analog input and internal digital test signal
  analog_in_mux <= s_comparator_out_internal;

  -- Boot Sweep + Dither + Closed-Loop FSM
  p_boot_dither : process(clk_tdc)
    -- Use VARIABLES for immediate-update edge detection
    variable v_start_pulse    : std_logic;
    variable v_ref_sync2_prev : std_logic                  := '0'; -- MUST be a variable for correct edge detect
    variable v_boot_counter   : integer range 0 to 5000001 := 0; -- Counter for various timeouts (larger range for sim)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        boot_state         <= ST_SWEEP;
        closed_loop_en     <= '0';
        cal_enable_sys     <= '0';
        v_boot_counter     := 0;
        v_ref_sync2_prev   := '0';
        dac_boot_ff        <= '0';
        sweep_duty_counter <= (others => '0');
        sweep_period_cnt   <= (others => '0');
        sweep_found_cross  <= '0';
        sweep_cross_duty   <= to_unsigned(128, 8); -- Default 50% if no crossing found
        sweep_init_integ   <= (others => '0');
        comp_sync0         <= '0';
        comp_sync1         <= '0';
        comp_prev          <= '0';
      else

        -- === COMPARATOR SYNCHRONIZER (always running) ===
        comp_sync0 <= analog_in_mux;
        comp_sync1 <= comp_sync0;

        -- === START PULSE DETECTION (using variables) ===
        v_start_pulse    := ref_sync2 and not v_ref_sync2_prev;
        v_ref_sync2_prev := ref_sync2;

        case boot_state is
          -- ================================================================
          -- STATE 0: SWEEP - Ramp DAC duty from 0% to 100% to find crossing
          -- ================================================================
          -- Output DAC based on duty counter: high for (duty/256) of each period
          -- Each duty level held for 256 clk_tdc cycles (~640ns @ 400MHz)
          -- Full sweep takes 256 * 256 = 65536 cycles (~164µs)
          when ST_SWEEP =>
            closed_loop_en <= '0';
            cal_enable_sys <= '0';      -- Don't collect calibration samples during sweep

            -- PWM generation: compare period counter against duty
            -- Pipeline the comparison to meet 400MHz timing
            if sweep_period_cnt < sweep_duty_counter then
              dac_boot_ff <= '1';
            else
              dac_boot_ff <= '0';
            end if;

            -- Increment period counter (wraps at 255)
            sweep_period_cnt <= sweep_period_cnt + 1;

            -- When period completes, increment duty (if not found crossing yet)
            if sweep_period_cnt = 255 then
              if sweep_found_cross = '0' then
                -- Check for comparator crossing (transition from '1' to '0')
                -- Comparator output = '1' when Vp > Vn, '0' when Vp < Vn
                -- We're looking for Vp < Vn (DAC has ramped past input voltage)
                if comp_prev = '1' and comp_sync1 = '0' then
                  -- Found crossing! Record the duty cycle
                  sweep_found_cross <= '1';
                  sweep_cross_duty  <= sweep_duty_counter;

                  -- Calculate initial integrator value based on duty cycle
                  -- Integrator threshold is 0 for 50% duty
                  -- Scale: duty 0-255 maps to integrator that gives 0%-100% duty
                  -- For duty > 128, integrator should be positive (DAC high more)
                  -- For duty < 128, integrator should be negative (DAC low more)
                  -- Rough scaling: (duty - 128) * 2^20 gives reasonable range
                  sweep_init_integ <= shift_left(resize(signed('0' & sweep_duty_counter) - 128, 32), 20);

                  if GC_SIM then
                    report "BOOT_SWEEP: Found comparator crossing at duty=" & integer'image(to_integer(sweep_duty_counter)) & "/255 (" & integer'image((to_integer(sweep_duty_counter) * 100) / 256) & "%) at " & time'image(now);
                  end if;
                end if;

                -- Increment duty (will wrap from 255 to 0)
                sweep_duty_counter <= sweep_duty_counter + 1;

                -- If we've swept all 256 duty levels without finding crossing, use default
                if sweep_duty_counter = 255 then
                  if sweep_found_cross = '0' then
                    if GC_SIM then
                      report "BOOT_SWEEP: No crossing found in full sweep! Using default 50% at " & time'image(now);
                    end if;
                    sweep_found_cross <= '1';
                    sweep_cross_duty  <= to_unsigned(128, 8);
                    sweep_init_integ  <= (others => '0');
                  end if;
                end if;
              end if;

              -- Store previous comparator state for edge detection
              comp_prev <= comp_sync1;
            end if;

            -- When crossing is found, wait a bit then move to dither
            if sweep_found_cross = '1' then
              v_boot_counter := v_boot_counter + 1;
              -- Settling time: 25µs normal, 2.5µs in fast sim mode
              if (GC_FAST_SIM and v_boot_counter >= C_SWEEP_SETTLING_FAST) or (not GC_FAST_SIM and v_boot_counter >= C_SWEEP_SETTLING_NORMAL) then
                if GC_SIM then
                  report "BOOT_FSM: Transitioning ST_SWEEP -> ST_DITHER at " & time'image(now) & " with init_integ=" & integer'image(to_integer(sweep_init_integ));
                end if;
                boot_state     <= ST_DITHER;
                v_boot_counter := 0;
                -- Set dac_boot_ff to match the found duty cycle center
                dac_boot_ff    <= '0';  -- Will toggle during dither
              end if;
            end if;

          -- ================================================================
          -- STATE 1: DITHER - 50% toggle for TDC calibration
          -- ================================================================
          when ST_DITHER =>
            closed_loop_en <= '0';
            cal_enable_sys <= '1';      -- Enable TDC calibration sample collection during dither

            -- FIXED 50% DUTY for TDC center calibration
            -- The TDC center is a geometric property of the circuit timing, not
            -- the input voltage. Use 50% duty to ensure symmetric crossings
            -- and collect calibration samples efficiently at all input voltages.
            if v_start_pulse = '1' then
              -- Increment period counter (8-bit PWM period)
              sweep_period_cnt <= sweep_period_cnt + 1;

              -- Simple PWM: 50% duty with ±8 dither
              -- Target duty = 64 (50% of 128)
              if sweep_period_cnt(7) = '0' then
                -- Upper half of dither cycle: use (64 + 8) = 72
                if sweep_period_cnt(6 downto 0) < to_unsigned(72, 7) then
                  dac_boot_ff <= '1';
                else
                  dac_boot_ff <= '0';
                end if;
              else
                -- Lower half of dither cycle: use (64 - 8) = 56
                if sweep_period_cnt(6 downto 0) < to_unsigned(56, 7) then
                  dac_boot_ff <= '1';
                else
                  dac_boot_ff <= '0';
                end if;
              end if;

              if GC_SIM and (v_boot_counter mod 10000) = 0 then
                report "ST_DITHER_PWM: FIXED 50% duty cnt=" & integer'image(to_integer(sweep_period_cnt)) & " at " & time'image(now);
              end if;
            end if;

            -- Wait for calibration completion OR timeout (safety valve)
            -- Priority: cal_done takes precedence over timer
            -- Increased timeout to 4,000,000 cycles @ 400MHz = 10ms to ensure calibration completes
            -- at all duty cycles (low input voltage = low duty = fewer TDC samples)
            v_boot_counter := v_boot_counter + 1;
            if cal_done = '1' then
              if GC_SIM then
                report "BOOT_FSM: cal_done='1'! Transitioning ST_DITHER -> ST_WAIT_FOR_START at " & time'image(now) & " after " & integer'image(v_boot_counter) & " cycles";
              end if;
              boot_state     <= ST_WAIT_FOR_START;
              v_boot_counter := 0;
            -- Calibration timeout: 10ms normal, 1ms in fast sim mode
            elsif (GC_FAST_SIM and v_boot_counter >= C_DITHER_TIMEOUT_FAST) or (not GC_FAST_SIM and v_boot_counter >= C_DITHER_TIMEOUT_NORMAL) then
              if GC_SIM then
                report "BOOT_FSM: Counter timeout! Transitioning ST_DITHER -> ST_WAIT_FOR_START at " & time'image(now) & " (cal_done still '0', will use uncalibrated center!)";
              end if;
              boot_state     <= ST_WAIT_FOR_START;
              v_boot_counter := 0;
            end if;

          -- ================================================================
          -- STATE 2: WAIT_FOR_START - Synchronize handoff to start edge
          -- ================================================================
          when ST_WAIT_FOR_START =>
            -- Wait for a start edge to align closed-loop enable
            if v_start_pulse = '1' then
              if GC_SIM then
                report "BOOT_FSM: Transitioning ST_WAIT_FOR_START -> ST_CLOSED_LOOP at " & time'image(now) & " (cal_done=" & std_logic'image(cal_done) & ")";
              end if;
              boot_state <= ST_CLOSED_LOOP;
            end if;

          -- ================================================================
          -- STATE 3: CLOSED_LOOP - Enable PI controller feedback
          -- ================================================================
          when ST_CLOSED_LOOP =>        -- @suppress "Dead state 'ST_CLOSED_LOOP': state does not have outgoing transitions"
            cal_enable_sys <= '0';      -- Calibration phase is over
            if closed_loop_en = '0' then
              closed_loop_en <= '1';
              if GC_SIM then
                report "BOOT_FSM: Entered ST_CLOSED_LOOP, closed_loop_en='1' at " & time'image(now);
              end if;
            end if;

            -- Continue PWM dithering until handoff completes
            if closed_loop_drive = '0' then
              if v_start_pulse = '1' then
                sweep_period_cnt <= sweep_period_cnt + 1;
                if sweep_period_cnt(7) = '0' then
                  if sweep_period_cnt(6 downto 0) < resize(sweep_cross_duty + 8, 7) then
                    dac_boot_ff <= '1';
                  else
                    dac_boot_ff <= '0';
                  end if;
                else
                  if sweep_period_cnt(6 downto 0) < resize(sweep_cross_duty - 8, 7) then
                    dac_boot_ff <= '1';
                  else
                    dac_boot_ff <= '0';
                  end if;
                end if;
              end if;
            end if;

          -- Note: ST_CLOSED_LOOP is intentionally a terminal state
          -- The delta-sigma loop runs here indefinitely until power cycle

          when others =>
            -- Safety fallback: return to initial state
            boot_state <= ST_SWEEP;
        end case;
      end if;
    end if;
  end process;

  -- Reference Clock Synchronization (3-FF)
  p_ref_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      ref_sync0 <= ref_clock;
      ref_sync1 <= ref_sync0;
      ref_sync2 <= ref_sync1;
    end if;
  end process;

  -- Trigger TDC on FALLING edge of ref_clock to align with DAC update (which is also on falling edge)
  -- This ensures the comparator transition (which happens after DAC update) falls within the TDC's
  -- early measurement window, rather than being 250ns late.
  ref_phases(0) <= not ref_sync2;

  -- Sample Generation for Decimator
  p_tdc_cdc_src : process(clk_tdc)
    variable v_sample_count : integer range 0 to 100000 := 0;
    variable v_ref_prev     : std_logic                 := '0';
    variable v_start_pulse  : std_logic                 := '0';
  begin
    if rising_edge(clk_tdc) then
      -- Detect ref_clock rising edge
      v_start_pulse := ref_sync2 and not v_ref_prev;
      v_ref_prev    := ref_sync2;

      -- Generate sample on EVERY ref_clock edge
      if v_start_pulse = '1' and closed_loop_en_tdc = '1' then
        -- Capture the CURRENT DAC value - this is the DAC state for the sample
        -- being generated. The previous approach of using v_dac_prev was causing
        -- ~2% duty cycle undercount at high voltages, leading to 55mV error.

        -- CRITICAL FIX: Check tdc_valid FIRST (same-cycle priority)
        -- This fixes the race condition where tdc_valid and start_pulse coincide:
        -- - tdc_sample_ready is updated at END of delta cycle
        -- - So when we read it, it still shows the OLD value ('0')
        -- - By checking tdc_valid directly, we catch the fresh TDC value
        if tdc_valid = '1' then
          -- Fresh TDC crossing THIS cycle - use it directly
          tdc_out_hold <= tdc_out;
        elsif tdc_sample_ready = '1' then
          -- Latched TDC crossing from PREVIOUS cycle - use latched value
          tdc_out_hold <= tdc_sample_latch;
        else
          -- No crossing detected - use center (neutral contribution)
          tdc_out_hold <= resize(tdc_center_cal, GC_TDC_OUTPUT);
        end if;

        dac_at_sample      <= dac_out_ff; -- Use CURRENT DAC (not previous)
        dac_at_sample_prev <= dac_out_ff;
        tdc_toggle         <= not tdc_toggle;

        if GC_SIM then
          v_sample_count := v_sample_count + 1;
          if v_sample_count <= 10 or (v_sample_count mod 5000) = 0 then
            report "SAMPLE_GEN: ref_edge #" & integer'image(v_sample_count) & " DAC=" & std_logic'image(dac_out_ff) & " tdc_valid=" & std_logic'image(tdc_valid) & " tdc_ready=" & std_logic'image(tdc_sample_ready) & " at " & time'image(now);
          end if;
        end if;
      elsif tdc_valid = '1' then
        -- TDC valid but NOT on start_pulse: just update hold, don't toggle
        -- This ensures we capture async TDC crossings between start_pulses
        tdc_out_hold       <= tdc_out;
        dac_at_sample_prev <= dac_out_ff;
        dac_at_sample      <= dac_at_sample_prev;
        tdc_toggle         <= not tdc_toggle;
      end if;
    end if;
  end process;

  -- Toggle synchronizer and edge detect (clk_sys domain)
  p_tdc_cdc_dst : process(clk_sys)
    variable v_toggle_prev : std_logic := '0';
    variable v_pulse       : std_logic;
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        -- POWER-ON SAFETY: Clear edge detector state during reset
        -- Prevents corrupting CIC decimation counter with glitch on tdc_valid_sys
        new_sample_sys    <= '0';
        v_toggle_prev     := '0';
        tdc_toggle_sync0  <= '0';
        tdc_toggle_sync1  <= '0';
        tdc_toggle_sync2  <= '0';
        tdc_out_sys       <= (others => '0');
        dac_at_sample_sys <= '0';
      else
        -- 3-FF scalar synchronizer: NO reset clause (avoids multi-clock-domain driving synchronizer)
        tdc_toggle_sync0 <= tdc_toggle;
        tdc_toggle_sync1 <= tdc_toggle_sync0;
        tdc_toggle_sync2 <= tdc_toggle_sync1;

        -- VARIABLE-BASED edge detect: compute pulse immediately and use in same cycle
        -- Safer than signal-based: avoids one-cycle delay that can hide pulse-width bugs
        -- The variable v_pulse is used both to drive new_sample_sys signal AND gate data capture
        v_pulse        := tdc_toggle_sync2 xor v_toggle_prev;
        new_sample_sys <= v_pulse;      -- Signal update for downstream logic
        v_toggle_prev  := tdc_toggle_sync2; -- Update state after computing pulse

        -- Data capture gated by the SAME CYCLE pulse
        if v_pulse = '1' then
          tdc_out_sys       <= tdc_out_hold; -- Capture when toggle changes
          dac_at_sample_sys <= dac_at_sample; -- DAC state at sample time (no CDC needed - held stable)
        end if;
      end if;
    end if;
  end process;

  -- Status CDC (2-FF synchronizers)
  p_status_cdc : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        tdc_overflow_sync <= (others => '0');
        tdc_lost_sync     <= (others => '0');
      else
        tdc_overflow_sync <= tdc_overflow_sync(0) & tdc_overflow;
        tdc_lost_sync     <= tdc_lost_sync(0) & tdc_lost_sample;
      end if;
    end if;
  end process;

  -- TDC Quantizer
  i_tdc : entity work.tdc_quantizer
    generic map(
      GC_TDL_LANES    => 4,
      GC_TDL_LENGTH   => 128,
      GC_COARSE_BITS  => 8,
      GC_OUTPUT_WIDTH => GC_TDC_OUTPUT,
      GC_SIM          => GC_SIM
    )

    port map(
      clk_sys          => clk_sys,
      clk_tdc          => clk_tdc,
      reset            => reset,
      analog_in        => analog_in_mux, -- Use muxed signal (external or internal test)
      ref_phases       => ref_phases,
      coarse_bias      => coarse_bias_tdc, -- Synchronized to clk_tdc
      invert_polarity  => '0',
      tdc_out          => tdc_out,
      tdc_valid        => tdc_valid,
      overflow         => tdc_overflow,
      lost_sample      => tdc_lost_sample,
      fine_at_comp_out => fine_at_comp_out, -- TDL centering calibration output
      fine_valid_out   => fine_valid_out -- TDL centering calibration valid
    );

  -- Coarse bias CDC synchronizer
  p_coarse_bias_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      coarse_bias_s0 <= coarse_bias;
      -- Mux between sys-domain coarse_bias and tdc-domain TDL calibrated value
      if tdl_cal_use_cal = '1' then
        coarse_bias_tdc <= coarse_bias_cal; -- Use TDL calibrated value
      else
        coarse_bias_tdc <= coarse_bias_s0; -- Use sys-domain value (CDC synchronized)
      end if;
    end if;
  end process;

  -- TDC Auto-Calibration (min/max tracking)
  p_tdc_calibration : process(clk_tdc)
    constant C_CAL_SAMPLES : integer := 16;
    variable v_tdc_sample  : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_tdc_min     : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_tdc_max     : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_center      : signed(GC_TDC_OUTPUT - 1 downto 0);
    impure function is_valid_sample(s : signed) return boolean is
    begin
      -- pragma translate_off
      for i in s'range loop
        if s(i) /= '0' and s(i) /= '1' then
          return false;
        end if;
      end loop;
      -- pragma translate_on
      return true;
    end function;
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        cal_sample_cnt <= (others => '0');
        cal_done       <= '0';
        tdc_center_tdc <= (others => '0'); -- Start at zero until calibrated
        -- Initialize min/max to extreme values
        v_tdc_min      := to_signed(2 ** (GC_TDC_OUTPUT - 1) - 1, GC_TDC_OUTPUT); -- Max positive
        v_tdc_max      := to_signed(-2 ** (GC_TDC_OUTPUT - 1), GC_TDC_OUTPUT); -- Max negative

      elsif cal_done = '0' then
        -- Calibration phase: track min/max TDC values during boot dither
        if tdc_valid = '1' and cal_enable_tdc = '1' then
          v_tdc_sample := signed(tdc_out);

          -- Only process valid samples
          if is_valid_sample(v_tdc_sample) then
            -- Track min and max
            if v_tdc_sample < v_tdc_min then
              v_tdc_min := v_tdc_sample;
            end if;
            if v_tdc_sample > v_tdc_max then
              v_tdc_max := v_tdc_sample;
            end if;

            cal_sample_cnt <= cal_sample_cnt + 1;

            -- Debug output disabled for speed (only show completion)

            -- After enough samples, compute center as midpoint of min/max
            if cal_sample_cnt = to_unsigned(C_CAL_SAMPLES - 1, cal_sample_cnt'length) then
              -- Center = (min + max) / 2
              -- IMPORTANT: Extend width by 1 bit before adding to prevent overflow!
              -- Both values are ~25000, so their sum (~50000) exceeds 16-bit signed max (32767)
              v_center       := resize(shift_right(resize(v_tdc_min, GC_TDC_OUTPUT + 1) + resize(v_tdc_max, GC_TDC_OUTPUT + 1), 1), GC_TDC_OUTPUT);
              -- Store the actual center value (not scaled) for proper subtraction
              -- The multi-bit decimator will subtract this before scaling
              tdc_center_tdc <= v_center;
              cal_done       <= '1';

              if GC_SIM then
                report "TDC_CAL: COMPLETE! min=" & integer'image(to_integer(v_tdc_min)) & " max=" & integer'image(to_integer(v_tdc_max)) & " center=" & integer'image(to_integer(v_center)) & " from " & integer'image(C_CAL_SAMPLES) & " samples at " & time'image(now);
              end if;
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- TDC Sample Latch
  p_tdc_latch : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdc_sample_latch <= (others => '0');
        tdc_sample_ready <= '0';
      else
        -- Latch TDC output when valid, mark as ready for processing
        if tdc_valid = '1' then
          tdc_sample_latch <= tdc_out;
          tdc_sample_ready <= '1';
          -- Debug disabled for speed (was TDC_LATCH)
        end if;

        -- Clear ready flag when sample is consumed (at start_pulse)
        if start_pulse_pi = '1' and tdc_sample_ready = '1' then
          tdc_sample_ready <= '0';
          -- Debug disabled for speed
        end if;
      end if;
    end if;
  end process;

  -- Direct Comparator Feedback (delta-sigma modulator)
  p_sigma_delta : process(clk_tdc)
    variable v_start_falling : std_logic;
  begin
    if rising_edge(clk_tdc) then
      -- Start pulse edge detection (local to this process)
      -- Rising edge for TDC latch clearing (matches p_tdc_cdc_src read timing)
      start_pulse_pi <= ref_sync2 and not ref_sync2_prev_pi;

      -- Falling edge for DAC update (to avoid rising-edge coupling glitches)
      v_start_falling := not ref_sync2 and ref_sync2_prev_pi;

      ref_sync2_prev_pi <= ref_sync2;

      if reset_tdc = '1' then
        dac_integrator_ff <= '0';
        ref_sync2_prev_pi <= '0';
        start_pulse_pi    <= '0';
      else
        -- DIRECT FEEDBACK: DAC = comparator (like rc_adc_top)
        -- Update at falling edge of start pulse (~200MHz) for phase coherence with TDC
        -- Previous mod_cnt prescaler (50MHz) destroyed coherence, causing ±90mV noise
        if v_start_falling = '1' and closed_loop_en_tdc = '1' then
          -- Direct comparator-to-DAC: comp='1' → DAC='1', comp='0' → DAC='0'
          -- The external RC filter provides integration
          dac_integrator_ff <= comp_sync1;
        end if;
      end if;
    end if;
  end process;

  -- TDC center CDC (clk_tdc -> clk_sys)
  p_tdc_center_cdc : process(clk_sys)
    variable v_prev_center : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        tdc_center_cal <= (others => '0');
        v_prev_center  := (others => '0');
      else
        -- Simple CDC - tdc_center_tdc is quasi-static (set once during cal)
        tdc_center_cal <= tdc_center_tdc;
        -- Debug: report when center changes
        if GC_SIM and tdc_center_tdc /= v_prev_center then
          report "CDC_CENTER: tdc_center_tdc changed from " & integer'image(to_integer(v_prev_center)) & " to " & integer'image(to_integer(tdc_center_tdc)) & " at " & time'image(now);
          v_prev_center := tdc_center_tdc;
        end if;
      end if;
    end if;
  end process;

  -- CDC for cal_done (clk_tdc -> clk_sys)
  -- Required so decimator only starts after calibration is complete
  p_cal_done_cdc : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        cal_done_sync0 <= '0';
        cal_done_sync1 <= '0';
        cal_done_sys   <= '0';
      else
        -- 3-FF synchronizer for scalar signal
        cal_done_sync0 <= cal_done;
        cal_done_sync1 <= cal_done_sync0;
        cal_done_sys   <= cal_done_sync1;
      end if;
    end if;
  end process;

  -- TDC valid signal (reuse toggle-based CDC)
  tdc_valid_sys <= new_sample_sys;

  -- DAC bitstream synchronizer (3-FF)
  p_dac_bitstream_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      -- 3-FF scalar synchronizer: NO reset clause
      dac_bitstream_sync0 <= dac_out_ff;
      dac_bitstream_sync1 <= dac_bitstream_sync0;
      dac_bitstream_sync2 <= dac_bitstream_sync1;

      -- Latch DAC bitstream at CIC CE edge to avoid sampling moving target
      if tdc_valid_sys = '1' then
        dac_bitstream_hold <= dac_bitstream_sync2;
      end if;
    end if;
  end process;

  -- CDC: closed_loop_en (clk_sys -> clk_tdc)
  p_closed_loop_en_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- 3-FF scalar synchronizer: NO reset clause (per CDC best practices)
      closed_loop_en_sync0 <= closed_loop_en;
      closed_loop_en_sync1 <= closed_loop_en_sync0;
      closed_loop_en_sync2 <= closed_loop_en_sync1;

      -- Debug report when synchronized signal transitions (always enabled)
      if (closed_loop_en_sync2 = '1') and (closed_loop_en_tdc = '0') then
        report "CDC_ENABLE: closed_loop_en_tdc going HIGH at " & time'image(now);
      end if;
      closed_loop_en_tdc <= closed_loop_en_sync2;

      -- CDC for cal_enable (gates TDC calibration to DITHER phase only)
      cal_enable_sync0 <= cal_enable_sys;
      cal_enable_sync1 <= cal_enable_sync0;
      cal_enable_tdc   <= cal_enable_sync1;
    end if;
  end process;

  -- TDL Centering Calibration FSM (clk_tdc domain)
  -- TIMING FIX: 2-stage pipeline to break critical path:
  --   Stage 1: Register fine_at_comp_out, compute sum (fine_at_comp + acc)
  --   Stage 2: Use registered sum for comparisons and decisions
  p_tdl_centering_cal : process(clk_tdc)
    constant C_TDL_CAL_TIMEOUT : natural               := 100000;
    -- Pre-computed thresholds (8 samples × threshold value)
    constant C_THRESH_LOW      : unsigned(19 downto 0) := to_unsigned(24576 * 8, 20); -- 196608
    constant C_THRESH_HIGH     : unsigned(19 downto 0) := to_unsigned(40960 * 8, 20); -- 327680
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdl_cal_done        <= '0';
        tdl_cal_sample_cnt  <= (others => '0');
        tdl_cal_fine_acc    <= (others => '0');
        tdl_cal_timeout_cnt <= (others => '0');
        coarse_bias_cal     <= C_DEFAULT_COARSE_BIAS; -- Reset to default value (101)
        tdl_cal_use_cal     <= '0';     -- Don't use calibrated value until we start adjusting
        tdl_settle_cnt      <= (others => '0');
        tdl_settling        <= '0';
        fine_at_comp_reg    <= (others => '0');
        fine_valid_reg      <= '0';
        tdl_cal_sum_reg     <= (others => '0');
        tdl_cal_sum_valid   <= '0';
        tdl_cal_last_sample <= '0';

      elsif cal_enable_tdc = '1' and tdl_cal_done = '0' then
        -- TDL centering calibration active during dither phase
        tdl_cal_use_cal     <= '1';     -- Start using calibrated value once calibration starts
        tdl_cal_timeout_cnt <= tdl_cal_timeout_cnt + 1;

        -- =====================================================================
        -- PIPELINE STAGE 1: Register inputs and compute sum
        -- =====================================================================
        fine_valid_reg   <= fine_valid_out;
        fine_at_comp_reg <= fine_at_comp_out;

        -- Default: clear stage 2 valid flags
        tdl_cal_sum_valid   <= '0';
        tdl_cal_last_sample <= '0';

        -- Timeout check
        if tdl_cal_timeout_cnt >= C_TDL_CAL_TIMEOUT then
          -- Timeout: accept current coarse_bias_cal as best effort
          tdl_cal_done <= '1';
          report "TDL_CAL: Timeout after " & integer'image(to_integer(tdl_cal_timeout_cnt)) & " cycles, coarse_bias_cal=" & integer'image(to_integer(coarse_bias_cal)) severity note;
        end if;

        -- Settle counter: registered countdown and flag update
        if tdl_settling = '1' then
          if tdl_settle_cnt > 0 then
            tdl_settle_cnt <= tdl_settle_cnt - 1;
          else
            tdl_settling <= '0';        -- Settling done, flag clears on next cycle
          end if;
        else
          -- Not settling: accumulate REGISTERED fine value
          if fine_valid_reg = '1' then
            -- Compute sum and store in pipeline register (for stage 2)
            tdl_cal_sum_reg   <= tdl_cal_fine_acc + resize(fine_at_comp_reg, 20);
            tdl_cal_sum_valid <= '1';

            -- Update accumulator
            tdl_cal_fine_acc   <= tdl_cal_fine_acc + resize(fine_at_comp_reg, 20);
            tdl_cal_sample_cnt <= tdl_cal_sample_cnt + 1;

            -- Flag if this is the last sample (comparison happens in stage 2)
            if tdl_cal_sample_cnt = to_unsigned(C_TDL_CAL_SAMPLES - 1, 4) then
              tdl_cal_last_sample <= '1';
            end if;
          end if;
        end if;

        -- =====================================================================
        -- PIPELINE STAGE 2: Use registered sum for comparisons (1 cycle later)
        -- =====================================================================
        if tdl_cal_sum_valid = '1' and tdl_cal_last_sample = '1' then
          -- Check if centered (within tolerance: 24576 to 40960 = 32768 ± 8192)
          -- Using pre-computed thresholds scaled by 8
          if tdl_cal_sum_reg >= C_THRESH_LOW and tdl_cal_sum_reg <= C_THRESH_HIGH then
            -- Centered! Calibration complete
            tdl_cal_done <= '1';
            report "TDL_CAL: SUCCESS! coarse_bias_cal=" & integer'image(to_integer(coarse_bias_cal)) & " fine_avg=" & integer'image(to_integer(resize(shift_right(tdl_cal_sum_reg, 3), 16))) severity note;

          elsif tdl_cal_sum_reg < C_THRESH_LOW then
            -- Average too low (< 24576): fine timing near 0, need to shift reference
            if coarse_bias_cal < 254 then
              coarse_bias_cal <= coarse_bias_cal + 1;
              report "TDL_CAL: fine_avg LOW, increasing coarse_bias_cal to " & integer'image(to_integer(coarse_bias_cal) + 1) severity note;
            end if;
            -- Reset accumulator and start settle wait
            tdl_cal_fine_acc   <= (others => '0');
            tdl_cal_sample_cnt <= (others => '0');
            tdl_settle_cnt     <= to_unsigned(C_TDL_CAL_SETTLE - 1, 10);
            tdl_settling       <= '1';

          else
            -- Average too high (> 40960): fine timing near max, need to shift reference
            if coarse_bias_cal > 1 then
              coarse_bias_cal <= coarse_bias_cal - 1;
              report "TDL_CAL: fine_avg HIGH, decreasing coarse_bias_cal to " & integer'image(to_integer(coarse_bias_cal) - 1) severity note;
            end if;
            -- Reset accumulator and start settle wait
            tdl_cal_fine_acc   <= (others => '0');
            tdl_cal_sample_cnt <= (others => '0');
            tdl_settle_cnt     <= to_unsigned(C_TDL_CAL_SETTLE - 1, 10);
            tdl_settling       <= '1';
          end if;
        end if;

      elsif cal_enable_tdc = '0' then
        -- Calibration phase ended (moved to closed loop)
        null;                           -- Keep using calibrated value
      end if;
    end if;
  end process;

  -- Note: p_closed_loop_drive_sync process removed (result was never used)

  -- Sticky Handoff Latch
  p_sticky_handoff : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        use_closed_loop <= '0';
      elsif closed_loop_drive = '1' then
        use_closed_loop <= '1';         -- STICKY: once true, stays true
        if GC_SIM and use_closed_loop = '0' then
          report "STICKY_HANDOFF: use_closed_loop latched to '1' at " & time'image(now) severity note;
        end if;
      end if;
    end if;
  end process;

  -- DAC Duty Cycle Monitor (simulation only)
  p_dac_duty_monitor : process(clk_sys)
    variable v_dac_count    : integer range 0 to 4096    := 0;
    variable v_total_count  : integer range 0 to 4096    := 0;
    variable v_window_count : integer range 0 to 1000000 := 0;
    variable v_duty_pct     : integer range 0 to 100     := 0;
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        v_dac_count    := 0;
        v_total_count  := 0;
        v_window_count := 0;
      elsif tdc_valid_sys = '1' then
        -- Count samples
        v_total_count := v_total_count + 1;
        if dac_bitstream_hold = '1' then
          v_dac_count := v_dac_count + 1;
        end if;

        -- Report every 2048 samples, first 2 windows plus every 4th
        if v_total_count = 2048 then
          v_window_count := v_window_count + 1;
          if GC_SIM and (v_window_count <= 2 or (v_window_count mod 4) = 0) then
            v_duty_pct := (v_dac_count * 100) / v_total_count;
            report "DAC_DUTY[" & integer'image(v_window_count) & "]: " & integer'image(v_dac_count) & "/2048 = " & integer'image(v_duty_pct) & "%";
          end if;
          -- Reset for next window
          v_dac_count    := 0;
          v_total_count  := 0;
        end if;
      end if;
    end if;
  end process;

  -- Multi-Bit TDC Accumulator/Decimator
  p_tdc_multibit_decimator : process(clk_sys)
    variable v_dec_cnt       : integer range 0 to 100000  := 0;
    variable v_acc_sum       : signed(C_TDC_ACC_WIDTH - 1 downto 0);
    variable v_avg           : signed(C_TDC_ACC_WIDTH - 1 downto 0);
    variable v_multibit_q    : signed(19 downto 0); -- Widened from GC_DATA_WIDTH
    variable v_dac_contrib   : signed(19 downto 0); -- Widened
    variable v_tdc_contrib   : signed(19 downto 0); -- Widened
    variable v_center_dyn    : signed(GC_TDC_OUTPUT - 1 downto 0);
    constant C_DAC_AMPLITUDE : signed(19 downto 0)        := to_signed(21845, 20); -- Correct amplitude: 384 * 21845 / 256 = 32767 (full Q15 range)
    variable v_sample_cnt    : integer range 0 to 1000000 := 0; -- Per-sample counter for debug rate-limiting
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        tdc_accumulator    <= (others => '0');
        tdc_dec_counter    <= (others => '0');
        tdc_dec_data_out   <= (others => '0');
        tdc_dec_valid      <= '0';
        tdc_center_runtime <= (others => '0'); -- Reset runtime center tracker
        v_dec_cnt          := 0;
        v_sample_cnt       := 0;
      else
        tdc_dec_valid <= '0';           -- Default: no output this cycle

        -- Accumulate on each valid TDC sample (only after calibration complete!)
        -- Gate with cal_done_sys to ensure tdc_center_cal is valid before processing
        if new_sample_sys = '1' and cal_done_sys = '1' then
          -- Runtime center tracking (IIR filter)
          if tdc_center_runtime = 0 then
            tdc_center_runtime <= shift_left(resize(tdc_out_sys, 32), 16);
          else
            tdc_center_runtime <= tdc_center_runtime + shift_right(shift_left(resize(tdc_out_sys, 32), 16) - tdc_center_runtime, C_CENTER_TRACK_SHIFT);
          end if;

          -- Multi-bit quantizer construction
          if dac_bitstream_hold = '1' then
            v_dac_contrib := C_DAC_AMPLITUDE;
          else
            v_dac_contrib := -C_DAC_AMPLITUDE;
          end if;

          -- Use slow-tracked runtime center to reject long-term drift
          v_center_dyn := resize(shift_right(tdc_center_runtime, 16), GC_TDC_OUTPUT);

          -- TDC contribution: (TDC - cal_center)
          -- No shift right (gain 1 instead of 0.25) to match coarser DAC steps
          -- TDC contribution: (TDC - cal_center)
          -- No shift right (gain 1 instead of 0.25) to match coarser DAC steps
          if disable_tdc_contrib = '1' then
            v_tdc_contrib := (others => '0');
          else
            v_tdc_contrib := resize(tdc_out_sys - resize(v_center_dyn, GC_TDC_OUTPUT), 20);
          end if;

          -- Combine: multi-bit quantizer output (pure 1-bit path when TDC contrib disabled)
          v_multibit_q := v_dac_contrib + v_tdc_contrib;

          -- Increment sample counter for debug rate-limiting
          v_sample_cnt := v_sample_cnt + 1;

          -- DEBUG: Print first few samples and then every 10000th sample
          if GC_SIM and (v_sample_cnt <= 3 or (v_sample_cnt mod 10000) = 0) then
            report "MULTIBIT_CALC[" & integer'image(v_sample_cnt) & "]: dac=" & std_logic'image(dac_bitstream_hold) & " tdc_out=" & integer'image(to_integer(tdc_out_sys)) & " tdc_center=" & integer'image(to_integer(tdc_center_cal)) & " tdc_diff=" & integer'image(to_integer(tdc_out_sys - resize(tdc_center_cal, GC_TDC_OUTPUT))) & " tdc_contrib=" & integer'image(to_integer(v_tdc_contrib)) & " q=" & integer'image(to_integer(v_multibit_q));
          end if;

          -- Check if decimation period complete
          if tdc_dec_counter = to_unsigned(GC_DECIMATION - 1, tdc_dec_counter'length) then
            -- On final sample: compute total sum including current sample
            v_acc_sum := tdc_accumulator + resize(v_multibit_q, C_TDC_ACC_WIDTH);

            -- Debug: show accumulator BEFORE adding current sample
            -- DEC_OUTPUT debug disabled for speed (printed every decimation period)

            -- Divide by 256 (shift 8) for perfect matching with 384 decimation
            v_avg := shift_right(v_acc_sum, 8);

            -- Output the decimated result
            tdc_dec_data_out <= resize(v_avg, GC_DATA_WIDTH);

            tdc_dec_valid   <= '1';
            tdc_dec_counter <= (others => '0');
            tdc_accumulator <= (others => '0'); -- Reset for next period

            v_dec_cnt := v_dec_cnt + 1;
            if GC_SIM and (v_dec_cnt <= 5 or (v_dec_cnt mod 500) = 0) then
              report "TDC_MULTIBIT_DEC [" & integer'image(v_dec_cnt) & "]: " & "cnt=" & integer'image(to_integer(tdc_dec_counter)) & " dac_at_sample=" & std_logic'image(dac_at_sample_sys) & " tdc_code=" & integer'image(to_integer(tdc_out_sys)) & " q_multibit=" & integer'image(to_integer(v_multibit_q)) & " acc=" & integer'image(to_integer(v_acc_sum)) & " avg=" & integer'image(to_integer(v_avg)) & " out=" & integer'image(to_integer(resize(v_avg, GC_DATA_WIDTH)));
            end if;
          else
            -- Not final sample: accumulate and increment counter
            tdc_accumulator <= tdc_accumulator + resize(v_multibit_q, C_TDC_ACC_WIDTH);
            tdc_dec_counter <= tdc_dec_counter + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- TDC Monitor Mode: Capture TDC codes for sanity check
  -- Captures raw TDC code, calibrated center, and difference on each sample
  -- This runs in parallel with main ADC operation
  -- Use to verify: TDC monotonicity, saturation, center calibration quality
  -- ========================================================================
  p_tdc_monitor : process(clk_sys)
    variable v_mon_count : integer range 0 to 10000 := 0; -- Prescaler to prevent UART overflow
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        tdc_mon_code   <= (others => '0');
        tdc_mon_center <= (others => '0');
        tdc_mon_diff   <= (others => '0');
        tdc_mon_dac    <= '0';
        tdc_mon_valid  <= '0';
        v_mon_count    := 0;
      else
        tdc_mon_valid <= '0';           -- Default: no new data

        -- Capture TDC data on each new sample
        if new_sample_sys = '1' and cal_done_sys = '1' then
          -- Rate limit the output to ~5kHz (50MHz / 10000)
          if v_mon_count >= 10000 then
            v_mon_count    := 0;
            tdc_mon_code   <= tdc_out_sys;
            tdc_mon_center <= resize(shift_right(tdc_center_runtime, 16), GC_TDC_OUTPUT);
            tdc_mon_diff   <= tdc_out_sys - resize(shift_right(tdc_center_runtime, 16), GC_TDC_OUTPUT);
            tdc_mon_dac    <= dac_bitstream_hold;
            tdc_mon_valid  <= '1';
          else
            v_mon_count := v_mon_count + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- TDC-Free Debug Mode: ref_clock edge-based CE generator
  -- CE pulses at modulator rate (~2MHz from ref_clock edges)
  -- CIC internal decimation (GC_DECIMATION=384) gives ~5.2kS/s output
  -- Completely independent of TDC toggle/new_sample_sys path
  -- ========================================================================
  p_ref_sync_sys : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        ref_sys0     <= '0';
        ref_sys1     <= '0';
        ref_sys2     <= '0';
        ref_sys_prev <= '0';
        ce_ref       <= '0';
      else
        -- 3-FF synchronizer for ref_clock
        ref_sys0 <= ref_clock;
        ref_sys1 <= ref_sys0;
        ref_sys2 <= ref_sys1;

        -- Edge detect: 1 clk_sys pulse per ref_clock RISING edge (~2MHz rate)
        -- FIX: Sample in middle of DAC period (Rising Edge) to mimic valid TDC path timing
        -- Previous Falling Edge capture coincided with DAC transition (dac_out_ff update), causing TB failure
        ce_ref       <= ref_sys2 and not ref_sys_prev;
        ref_sys_prev <= ref_sys2;
      end if;
    end if;
  end process;

  -- DAC synchronizer for debug mode (separate from TDC path's dac_bitstream_sync)
  p_dac_sync_dbg : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        dac_sync0    <= '0';
        dac_sync1    <= '0';
        dac_dbg_hold <= '0';
      else
        -- 2-FF synchronizer for DAC bit (completely independent of TDC path)
        dac_sync0 <= dac_out_ff;
        dac_sync1 <= dac_sync0;

        -- Latch DAC bit on CE edge to remove sampling skew relative to ce_ref
        if ce_ref = '1' then
          dac_dbg_hold <= dac_sync1;
        end if;
      end if;
    end if;
  end process;

  -- Debug CIC: Uses ref_clock edge CE and directly synchronized DAC bit
  -- No dependence on TDC toggle, new_sample_sys, or dac_bitstream_hold
  -- CE at ~2MHz, CIC decimates by GC_DECIMATION internally → ~5.2kS/s output
  i_cic_debug : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => GC_DECIMATION, -- 384: CIC does internal decimation
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk      => clk_sys,
      reset    => reset,
      data_in  => dac_dbg_hold,         -- DAC bit captured at ce_ref
      ce       => ce_ref,               -- ref_clock edge, NOT TDC or counter
      data_out => dbg_cic_data_out,
      valid    => dbg_cic_valid_out
    );

  -- CIC SINC3 Decimator (original TDC-based path)
  i_cic : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => GC_DECIMATION,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk      => clk_sys,
      reset    => reset,
      -- FIX: Always use the known-good 'dac_dbg_hold' stream (synced to ce_ref)
      -- The previous 'dac_bitstream_hold' from TDC CDC was causing -0.09V error (aliasing/corruption)
      data_in  => dac_dbg_hold,
      -- FIX: Always use ce_ref logic (clean Reference Edge) for decimation pacing
      ce       => ce_ref,
      data_out => cic_data_out,
      valid    => cic_valid_out
    );

  -- Select between TDC path and debug path (runtime selectable)
  final_cic_data  <= dbg_cic_data_out when use_tdc_free_debug_mode = '1' else cic_data_out;
  final_cic_valid <= dbg_cic_valid_out when use_tdc_free_debug_mode = '1' else cic_valid_out;

  -- FIR Equalizer (compensates CIC sinc3 droop)
  -- Uses final_cic_* which comes from debug or normal path based on C_TDC_FREE_DEBUG_MODE
  i_equalizer : entity work.fir_equalizer
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk_sys,
      reset     => reset,
      data_in   => final_cic_data,
      valid_in  => final_cic_valid,
      data_out  => eq_data_out,
      valid_out => eq_valid_out
    );

  -- EQ Bypass Mux: Select between EQ output or raw CIC output (runtime selectable)
  eq_mux_out   <= final_cic_data when disable_eq_filter = '1' else eq_data_out;
  eq_mux_valid <= final_cic_valid when disable_eq_filter = '1' else eq_valid_out;

  -- FIR Lowpass Filter (anti-aliasing) - placed after Equalizer in CIC chain
  i_lowpass : entity work.fir_lowpass
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk_sys,
      reset     => reset,
      data_in   => eq_mux_out,
      valid_in  => eq_mux_valid,
      data_out  => lp_data_out,
      valid_out => lp_valid_out
    );

  -- LP Bypass Mux: Select between LP output or EQ mux output (runtime selectable)
  lp_mux_out   <= eq_mux_out when disable_lp_filter = '1' else lp_data_out;
  lp_mux_valid <= eq_mux_valid when disable_lp_filter = '1' else lp_valid_out;

  -- ========================================================================
  -- TDC Contribution Accumulator
  -- Accumulates TDC fine timing contributions in parallel with CIC/EQ/LP chain
  -- The TDC provides sub-LSB resolution that complements the 1-bit DAC stream
  -- ========================================================================
  p_tdc_contrib_acc : process(clk_sys)
    variable v_tdc_contrib      : signed(GC_DATA_WIDTH - 1 downto 0);
    variable v_center_dyn       : signed(GC_TDC_OUTPUT - 1 downto 0);
    -- scaling: Use fixed shift of 13 for Decimation 8192 (Power of 2)
    -- 8192 = 2^13. Division by 2^13 gives Unity Gain for simple accumulation.
    constant C_DEC_SHIFT        : natural := 13;
    -- TDC contribution per-sample shift:
    -- At 2MHz (T=500ns), shift 5 (Div 32) was appropriate.
    -- At 50MHz (T=20ns), sensitivity is 25x higher (1 code is larger % of period).
    -- Need to increase attenuation by ~25x (Shift +5 bits).
    -- TDC Contribution Gain:
    -- Period = 20ns (50MHz). TDC LSB = ~20ps.
    -- Ratio: 20ns / 20ps = 1000 codes per period.
    -- 1 Full DAC Step (2 * C_DAC_AMPLITUDE) = 43690 digital codes.
    -- Required Gain = 43690 / 1000 = 43.7.
    -- Adjusted to x48 to fix mid-scale droop (Linearly interpolated).
    -- Implementation: x48 = x32 + x16 = (x << 5) + (x << 4)
    constant C_TDC_GAIN_SHIFT_5 : natural := 5; -- x32
    constant C_TDC_GAIN_SHIFT_4 : natural := 4; -- x16
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        tdc_contrib_acc     <= (others => '0');
        tdc_contrib_out     <= (others => '0');
        tdc_contrib_valid   <= '0';
        tdc_contrib_counter <= (others => '0');
      else
        tdc_contrib_valid <= '0';       -- Default

        -- Accumulate TDC contribution on each valid sample (after calibration)
        if new_sample_sys = '1' and cal_done_sys = '1' then
          -- FIX: Use STATIC Calibrated Center instead of Runtime Tracking.
          -- Runtime tracking (v_center_dyn) acts as a High-Pass filter, removing 
          -- valid DC corrections from the TDC, causing 'droop' at mid-scale.
          -- Using calibrated center enables true DC linearity correction.
          v_center_dyn := tdc_center_cal;

          -- TDC contribution: (TDC - cal_center) / 32 (shift 5)
          -- Reduced gain to prevent mid-scale over-correction that was pulling ~0.3-0.5V low.
          if disable_tdc_contrib = '1' then
            v_tdc_contrib := (others => '0');
          else
            -- Apply Gain x48 (x32 + x16) to match DAC scale
            -- v_tdc_contrib := (TDC - Center) * 48
            v_tdc_contrib := resize(
              shift_left(tdc_out_sys - resize(v_center_dyn, GC_TDC_OUTPUT), C_TDC_GAIN_SHIFT_5) + shift_left(tdc_out_sys - resize(v_center_dyn, GC_TDC_OUTPUT), C_TDC_GAIN_SHIFT_4),
              GC_DATA_WIDTH);
          end if;

          -- Check if decimation period complete
          if tdc_contrib_counter = to_unsigned(GC_DECIMATION - 1, tdc_contrib_counter'length) then
            -- Output averaged TDC contribution
            tdc_contrib_out     <= resize(shift_right(tdc_contrib_acc + resize(v_tdc_contrib, 32), C_DEC_SHIFT), GC_DATA_WIDTH);
            tdc_contrib_valid   <= '1';
            tdc_contrib_acc     <= (others => '0');
            tdc_contrib_counter <= (others => '0');
          else
            -- Accumulate
            tdc_contrib_acc     <= tdc_contrib_acc + resize(v_tdc_contrib, 32);
            tdc_contrib_counter <= tdc_contrib_counter + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Filter Priming Counter and TDC Contribution Hold Register
  -- The FIR filters need ~64 samples to prime their delay lines.
  -- Until primed, we use the TDC-only decimator output.
  -- After primed, we switch to the combined CIC/EQ/LP + TDC path.
  -- ========================================================================
  p_filter_prime : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        filter_prime_counter <= (others => '0');
        filter_primed        <= '0';
        tdc_contrib_held     <= (others => '0');
      else
        -- Hold TDC contribution when valid (it's calculated before LP output)
        if tdc_contrib_valid = '1' then
          tdc_contrib_held <= tdc_contrib_out;
        end if;

        -- Count LP valid outputs until primed (use mux signal for bypass support)
        if lp_mux_valid = '1' and filter_primed = '0' then
          if filter_prime_counter >= to_unsigned(C_FILTER_PRIME_COUNT - 1, 8) then
            filter_primed <= '1';
            if GC_SIM then
              report "FILTER_PRIMED: CIC/EQ/LP chain now stable after " & integer'image(C_FILTER_PRIME_COUNT) & " samples at " & time'image(now);
            end if;
          else
            filter_prime_counter <= filter_prime_counter + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Combined Output Path with Bypass During Filter Priming
  -- Before filters are primed: Use TDC-only decimator (tdc_dec_data_out)
  -- After filters are primed: Use CIC/EQ/LP + TDC contribution
  -- ========================================================================
  p_combine_output : process(clk_sys)
    variable v_lp_signed     : signed(GC_DATA_WIDTH - 1 downto 0);
    variable v_combined      : signed(GC_DATA_WIDTH + 1 downto 0);
    variable v_tdc_only      : signed(GC_DATA_WIDTH - 1 downto 0);
    variable v_combined_cnt  : integer range 0 to 1000000 := 0; -- Counter for rate-limiting COMBINED_OUT debug
    variable v_mon_prescaler : unsigned(1 downto 0)       := (others => '0'); -- Prescaler for monitor output (4x decimation)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        combined_data_out  <= (others => '0');
        combined_valid_out <= '0';
        mv_code            <= (others => '0');
        v_combined_cnt     := 0;
        v_mon_prescaler    := (others => '0');
      else
        combined_valid_out <= '0';      -- Default

        -- BYPASS MODE: Before LP filter is primed, use TDC-only decimator
        if filter_primed = '0' then
          if tdc_dec_valid = '1' then
            -- Use TDC multi-bit decimator output directly
            v_tdc_only         := tdc_dec_data_out;
            combined_data_out  <= v_tdc_only;
            combined_valid_out <= '1';

            -- Only report first few and last few priming samples to avoid flooding
            if GC_SIM and (filter_prime_counter <= 2 or filter_prime_counter >= C_FILTER_PRIME_COUNT - 2) then
              report "BYPASS_OUT: tdc_dec=" & integer'image(to_integer(v_tdc_only)) & " mv=" & integer'image(to_integer(mv_code)) & " (filter priming, " & integer'image(to_integer(filter_prime_counter)) & "/" & integer'image(C_FILTER_PRIME_COUNT) & ")";
            end if;
          end if;
        else
          -- COMBINED MODE: After LP filter is primed, use CIC/EQ/LP + TDC
          -- When C_BYPASS_EQ_FILTER or C_BYPASS_LP_FILTER are true, lp_mux_* carries CIC output directly
          if lp_mux_valid = '1' then
            -- LP mux output is the CIC/EQ/LP processed 1-bit DAC stream (or bypassed CIC output)
            -- CIC with bipolar input (±1) and unity gain gives output in Q15 format
            -- Range: [-32768, +32767] maps to [-1.0, +1.0)
            v_lp_signed := signed(lp_mux_out);

            -- Add held TDC contribution for fine resolution
            -- The tdc_contrib_held was captured when tdc_contrib_valid fired
            -- DEBUG: When C_DISABLE_TDC_CONTRIB=true, skip TDC to isolate CIC path
            if disable_tdc_contrib = '1' then
              v_combined := resize(v_lp_signed, GC_DATA_WIDTH + 2);
            else
              -- SIGN FIX: Subtract TDC contribution!
              -- Higher Input -> Faster Integration -> Shorter Time -> Smaller TDC Code.
              -- Start pulse is fixed. Comparator trips earlier.
              -- TDC = Stop - Start. Smaller TDC means "Turned ON earlier".
              -- If it turns on earlier, it means input was HIGH.
              -- So Smaller TDC = Higher Input.
              -- Correlation is Inverted.
              -- We want Higher Input -> Higher Output.
              -- So we must Negate TDC?
              -- Wait. If TDC is small (Higher Input), we want to ADD positive value?
              -- Result = Base (Rough) + Correction.
              -- If Base is "Average".
              -- Let's stick to the Sawtooth evidence:
              -- 0.4V -> 0.6V (Too High) -> TDC was adding positive value.
              -- 0.5V -> 0.4V (Too Low) -> TDC was adding negative value (or small positive).
              -- Flipping the sign is the correct move for sawtooth inversion.
              v_combined := resize(v_lp_signed, GC_DATA_WIDTH + 2) - resize(tdc_contrib_held, GC_DATA_WIDTH + 2);
            end if;

            -- Saturate to output range
            if v_combined > to_signed(32767, GC_DATA_WIDTH + 2) then
              combined_data_out <= to_signed(32767, GC_DATA_WIDTH);
            elsif v_combined < to_signed(-32768, GC_DATA_WIDTH + 2) then
              combined_data_out <= to_signed(-32768, GC_DATA_WIDTH);
            else
              combined_data_out <= resize(v_combined, GC_DATA_WIDTH);
            end if;

            -- Monitor Output Decimation (Reduce rate to prevent UART Overflow)
            -- 260kHz -> 65kHz (4x decimation)
            v_mon_prescaler := v_mon_prescaler + 1;
            if v_mon_prescaler = 0 then
              combined_valid_out <= '1';
            else
              combined_valid_out <= '0';
            end if;

            -- Rate-limit COMBINED_OUT debug (first 3 + every 20th)
            v_combined_cnt := v_combined_cnt + 1;
            if GC_SIM and (v_combined_cnt <= 3 or (v_combined_cnt mod 20) = 0) then
              report "COMBINED_OUT[" & integer'image(v_combined_cnt) & "]: lp=" & integer'image(to_integer(v_lp_signed)) & " tdc_contrib=" & integer'image(to_integer(tdc_contrib_held)) & " combined=" & integer'image(to_integer(v_combined)) & " mv=" & integer'image(to_integer(mv_code));
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Streaming output: Use combined CIC/EQ/LP + TDC path
  -- This provides both droop-compensated AC response AND TDC fine resolution
  -- Output is raw Q15 format: signed [-32768, +32767] maps to [-1.0, +1.0)
  -- The testbench and Python monitor convert to mV for display
  sample_data  <= std_logic_vector(combined_data_out);
  sample_valid <= combined_valid_out;

  -- TDC Monitor outputs
  tdc_monitor_code   <= tdc_mon_code;
  tdc_monitor_center <= tdc_mon_center;
  tdc_monitor_diff   <= tdc_mon_diff;
  tdc_monitor_dac    <= tdc_mon_dac;
  tdc_monitor_valid  <= tdc_mon_valid;

  -- ========================================================================
  -- Closed-Loop Drive Arming (v8.4 hardware stall fix - Valid-Qualified)
  -- ========================================================================
  p_arm_closed_loop : process(clk_tdc)
    variable v_ref_sync2_prev : std_logic := '0';
    variable v_start_pulse    : std_logic;
  begin
    if rising_edge(clk_tdc) then
      -- Detect rising edge of ref_sync2 (Start pulse)
      v_start_pulse    := ref_sync2 and not v_ref_sync2_prev;
      v_ref_sync2_prev := ref_sync2;

      if reset_tdc = '1' then
        cl_valid_seen     <= '0';
        cl_switch_pend    <= '0';
        closed_loop_drive <= '0';
        cl_watch_cnt      <= (others => '0');
      else
        -- ======================================================================
        -- Step 1: Latch once we observe a closed-loop tdc_valid
        -- ======================================================================
        -- TIMEOUT FALLBACK: If no TDC valid after 100 start pulses (~50µs),
        -- assume comparator-based mode and proceed with handoff anyway.
        -- This enables operation when TDC doesn't fire (static comparator).
        if (closed_loop_en = '1') and (tdc_valid = '1') and (cl_valid_seen = '0') then
          cl_valid_seen <= '1';
          if GC_SIM then
            report "HANDOFF: cl_valid_seen='1' at time " & time'image(now) & " (first closed-loop tdc_valid)";
          end if;
        elsif (closed_loop_en = '1') and (cl_valid_seen = '0') and (cl_watch_cnt >= 100) then
          -- Timeout: no TDC valid, force handoff (comparator mode)
          cl_valid_seen <= '1';
          if GC_SIM then
            report "HANDOFF: cl_valid_seen='1' by TIMEOUT (no TDC valid after 100 starts) at " & time'image(now);
          end if;
        end if;

        -- ======================================================================
        -- Step 2: Arm the handoff (one-shot) after the first closed-loop valid
        -- The DAC output mux will keep using boot dither until sniff completes
        -- ======================================================================
        if (closed_loop_en = '1') and (cl_valid_seen = '1') and (closed_loop_drive = '0') and (cl_switch_pend = '0') then
          cl_switch_pend <= '1';
          if GC_SIM then
            report "HANDOFF: cl_switch_pend='1' (armed) at time " & time'image(now);
          end if;
        end if;

        -- ======================================================================
        -- Step 3: Execute the handoff on a Start edge (ensures phase alignment)
        -- ======================================================================
        if (cl_switch_pend = '1') and (v_start_pulse = '1') then
          closed_loop_drive <= '1';
          cl_switch_pend    <= '0';
          cl_watch_cnt      <= (others => '0'); -- Reset watchdog on handoff
          if GC_SIM then
            report "HANDOFF: cl_switch executed at " & time'image(now);
          end if;
        end if;

        -- ======================================================================
        -- Watchdog: Count Start edges while waiting for TDC samples
        -- ======================================================================
        -- During handoff wait (closed_loop_en=1 but closed_loop_drive=0):
        --   Count starts. If we hit 200 starts (~100µs @ 2MHz), flag sticky error.
        -- During closed-loop operation (closed_loop_drive=1):
        --   Count starts when no TDC samples arrive. Reset on each tdc_valid.
        --   Used for keepalive and auto-rescue.
        if closed_loop_en = '1' then
          if closed_loop_drive = '0' then
            -- Handoff phase: count starts while waiting to switch
            if v_start_pulse = '1' then
              cl_watch_cnt <= cl_watch_cnt + 1;
            end if;
          -- Note: cl_sticky_no_valid debug flag removed
          else
            -- Closed-loop active: count starts, reset on tdc_valid
            if tdc_valid = '1' then
              cl_watch_cnt <= (others => '0'); -- Reset on each TDC sample
            elsif v_start_pulse = '1' then
              cl_watch_cnt <= cl_watch_cnt + 1; -- Increment if no samples
            end if;
          end if;
        else
          cl_watch_cnt <= (others => '0');
        end if;
      end if;
    end if;
  end process;

  -- CDC: use_closed_loop (clk_sys -> clk_tdc)
  p_cdc_use_closed_loop : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        use_closed_loop_sync1 <= '0';
        use_closed_loop_tdc   <= '0';
      else
        -- Two-FF synchronizer for CDC
        use_closed_loop_sync1 <= use_closed_loop;
        use_closed_loop_tdc   <= use_closed_loop_sync1;
      end if;
    end if;
  end process;

  -- DAC Output Register
  p_dac_output : process(clk_tdc)
    variable v_last_use_cl_tdc : std_logic := '0';
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        dac_out_ff        <= '0';
        v_last_use_cl_tdc := '0';
      else
        -- Debug: Report when use_closed_loop_tdc transitions
        if use_closed_loop_tdc /= v_last_use_cl_tdc then
          report "DAC_MUX: use_closed_loop_tdc changed " & std_logic'image(v_last_use_cl_tdc) & " -> " & std_logic'image(use_closed_loop_tdc) & " at " & time'image(now);
          v_last_use_cl_tdc := use_closed_loop_tdc;
        end if;

        if use_closed_loop_tdc = '0' then
          -- Boot/Dither Phase: Drive with dither signal
          dac_out_ff <= dac_boot_ff;
        else
          -- Closed Loop: Direct comparator feedback
          dac_out_ff <= dac_integrator_ff;
        end if;
      end if;
    end if;
  end process;

  -- Default Configuration
  coarse_bias <= C_DEFAULT_COARSE_BIAS;

end architecture;
