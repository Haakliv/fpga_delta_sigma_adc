-- ************************************************************************
-- Top-Level TDC-Based Delta-Sigma ADC
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
use work.dsp_utils_pkg.all;

entity tdc_adc_top is
  generic(
    GC_DECIMATION : positive := 64;     -- Decimation ratio for CIC
    GC_DATA_WIDTH : positive := 16;     -- Output data width
    GC_TDC_OUTPUT : positive := 16;     -- TDC output width
    GC_SIM        : boolean  := false;  -- Simulation mode for TDC
    GC_OPEN_LOOP  : boolean  := false   -- Open-loop test mode (bypass feedback)
  );

  port(
    -- Clocks and reset
    clk_sys            : in  std_logic; -- System clock (e.g., 100 MHz)
    clk_tdc            : in  std_logic; -- TDC fast clock (e.g., 400-600 MHz)
    reset              : in  std_logic;
    -- Reference clock for TDC start edge
    ref_clock          : in  std_logic;
    -- GPIO IP interface (connects to adc_system GPIO exports at top level)
    comparator_in      : in  std_logic; -- From adc_system comp_out_export
    dac_out_bit        : out std_logic; -- To adc_system slope_din_export
    -- Optional trigger input (when '1', sampling is enabled; when '0', sampling is disabled)
    trigger_enable     : in  std_logic := '1'; -- Default '1' for continuous sampling
    -- Open-loop test mode (GC_OPEN_LOOP=true only)
    open_loop_dac_duty : in  std_logic := '0'; -- Fixed DAC output in open-loop mode

    -- Streaming sample output
    sample_data        : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid       : out std_logic;
    -- Debug outputs for characterization
    debug_tdc_out      : out signed(GC_TDC_OUTPUT - 1 downto 0);
    debug_tdc_valid    : out std_logic
  );
end entity;

architecture rtl of tdc_adc_top is
  -- Simulation performance control
  constant C_VERBOSE_DEBUG : boolean := true; -- Set to false to speed up simulation significantly

  -- ========================================================================
  -- TDC Center-Code Calibration
  -- ========================================================================
  -- Measured TDC code when Vin ≈ Vfb (equilibrium point)
  -- This is determined by open-loop characterization:
  --   1. Set GC_OPEN_LOOP=true
  --   2. Sweep Vin and DAC duty
  --   3. Find TDC code where Vin ≈ Vdac_avg
  --   4. Store that code here
  --
  -- For τ=4.7µs RC filter: TDC will see slow-varying Vdac (not PWM ripple)
  -- Center code is where the loop should operate (minimal TDC error)
  -- AUTO-CALIBRATED at runtime during boot dither phase
  -- The TDC center is measured when DAC is at 50% duty (dithering) and Vin ≈ Vfeedback
  -- This automatically adapts to hardware variations and eliminates manual tuning
  signal tdc_center_cal : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0'); -- Runtime calibrated center
  signal tdc_center_tdc : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0'); -- CDC copy in clk_tdc domain

  -- Calibration sample counter
  signal cal_sample_cnt : unsigned(7 downto 0) := (others => '0'); -- Number of samples collected
  signal cal_done       : std_logic            := '0'; -- Calibration complete flag

  -- Error clamping range: must cover actual TDC range for proportional control
  -- TDC swings ~1000 codes over Vn range (0V to 1.3V), so allow ±512 for full coverage
  constant C_ERROR_MAX : integer := 4096; -- ±4096 codes (~280mV with K_tdc=14740)

  -- TDC signals
  signal ref_phases      : std_logic_vector(0 downto 0);
  signal clear_status    : std_logic                          := '0';
  signal tdc_out         : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_valid       : std_logic;
  signal tdc_overflow    : std_logic;
  signal tdc_lost_sample : std_logic;

  -- Loop filter signals (SLOW LOOP for DC trim only)
  signal loop_integrator : signed(31 downto 0) := (others => '0'); -- Start at zero (mid-scale for signed)
  signal loop_output     : signed(15 downto 0);

  -- ========================================================================
  -- Edge Keepalive: Prevent Deadlock from Static Comparator Output
  -- ========================================================================
  -- Inject slow dither into DAC decision when edges starve (prevents deadlock)
  signal keepalive_en : std_logic := '0';
  signal start_div2   : std_logic := '0';

  -- Fast path: tdc_out bitstream for CIC
  signal tdc_out_bitstream : std_logic; -- 1-bit ds bitstream from TDC sign bit

  -- Loop filter configuration (configurable via MMIO - clk_sys domain)
  signal kp_shift         : unsigned(3 downto 0) := to_unsigned(5, 4); -- Proportional gain: /32 (with error clamping)
  signal ki_shift         : unsigned(3 downto 0) := to_unsigned(5, 4); -- Integral gain: /32 (with error clamping)
  signal loop_enable      : std_logic            := '1';
  signal integrator_limit : signed(31 downto 0)  := to_signed(4096, 32); -- Moderate limit for larger error range
  signal pi_cfg_commit    : std_logic            := '0'; -- Toggle to commit config changes to clk_tdc domain

  -- Loop filter configuration shadow copies (clk_tdc domain - CDC synchronized)
  -- PI LOOP TUNING (Step 2 - Add Ki for zero steady-state error):
  -- Kp = 1/32 (shift=5) - fast response, verified stable
  -- Ki = 1/256 (shift=8) - slow integrator to eliminate steady-state error
  -- With K_tdc ≈ 9525 codes/V, 100mV error → 953 codes → I += 953/256 ≈ 4/sample
  -- At 2MHz sample rate, integrator builds 8000/sec → settles in ~100ms for 100mV error
  signal kp_shift_tdc         : unsigned(3 downto 0) := to_unsigned(5, 4); -- Proportional (÷32)
  signal ki_shift_tdc         : unsigned(3 downto 0) := to_unsigned(8, 4); -- Integral (÷256) - slow integration
  signal loop_enable_tdc      : std_logic            := '1';
  signal integrator_limit_tdc : signed(31 downto 0)  := to_signed(8192, 32); -- Larger limit
  signal pi_cfg_toggle_s0     : std_logic            := '0'; -- First sync stage
  signal pi_cfg_toggle_s1     : std_logic            := '0'; -- Second sync stage

  -- TDC configuration (configurable via MMIO - clk_sys domain)
  -- Coarse bias compensates for systematic timing offset between Start and Stop

  -- Parent module (tdc_adc_top) provides ref_phases(0) via 3-FF synchronizer
  -- Stop path has additional 3-FF analog_sync in tdc_quantizer
  -- Both Start and Stop use coarse_bin_bank_prev for capture

  function get_default_coarse_bias return unsigned is
  begin
    -- BEHAVIORAL TESTBENCH: Timing-based comparator model
    -- Crossing happens at ~250ns (center of 500ns ref_clock period)
    -- 250ns / 2.474ns = 101.05 coarse periods, use 101
    -- NOTE: Hardware will measure different value and calibration will find it automatically.
    -- Production hardware uses 10us RC filter with different timing characteristics.
    return to_unsigned(101, 8);         -- 0.5 ref period delay: 101 coarse periods
  end function;

  signal coarse_bias : unsigned(7 downto 0) := get_default_coarse_bias;

  -- TDC configuration shadow copies (clk_tdc domain - CDC synchronized)
  signal coarse_bias_s0  : unsigned(7 downto 0) := get_default_coarse_bias;
  signal coarse_bias_tdc : unsigned(7 downto 0) := get_default_coarse_bias;

  -- ========================================================================
  -- TDL Centering Calibration (Align comparator crossing to TDL center)
  -- ========================================================================
  -- When fine_at_comp_out is stuck at 0 or 65535, the comparator crossing lands
  -- outside the TDL's measurable window. This calibration adjusts coarse_bias
  -- until fine_at_comp_out is centered (~32768), giving the TDC maximum headroom.
  --
  -- This is the "align to mid-TDL" step from Section 3.5.2 of the reference paper.
  -- At a known input voltage (during DITHER with 50% DAC duty), we adjust the
  -- coarse_bias until the TDC reports a centered fine code.

  -- TDL centering calibration output (clk_tdc domain, CDC'd to coarse_bias_tdc)
  signal coarse_bias_cal : unsigned(7 downto 0) := get_default_coarse_bias; -- Calibrated bias in clk_tdc domain
  signal tdl_cal_use_cal : std_logic            := '0'; -- '1' when TDL cal is active and adjusting

  -- TDL centering calibration signals (from tdc_quantizer)
  signal fine_at_comp_out : unsigned(15 downto 0) := (others => '0'); -- Fine code captured at comparator edge
  signal fine_valid_out   : std_logic             := '0'; -- Pulses when fine_at_comp_out is updated

  -- TDL centering calibration state machine (clk_tdc domain)
  constant C_TDL_CAL_SAMPLES : natural := 8; -- Number of samples to average before deciding

  signal tdl_cal_enabled     : std_logic             := '0'; -- '1' during TDL centering calibration phase
  signal tdl_cal_done        : std_logic             := '0'; -- '1' when TDL centering calibration complete
  signal tdl_cal_sample_cnt  : unsigned(3 downto 0)  := (others => '0'); -- Sample counter (0-15)
  signal tdl_cal_fine_acc    : unsigned(19 downto 0) := (others => '0'); -- Accumulator for averaging (20 bits for 8 samples of 16-bit values)
  signal tdl_cal_fine_avg    : unsigned(15 downto 0) := (others => '0'); -- Averaged fine code
  signal tdl_cal_sweep_dir   : std_logic             := '0'; -- Not used, but kept for future expansion
  signal tdl_cal_timeout_cnt : unsigned(15 downto 0) := (others => '0'); -- Timeout counter

  -- Integrator reset CDC (clk_sys -> clk_tdc) - 3-FF scalar chain for MTBF
  signal reset_integrator_req   : std_logic := '0'; -- clk_sys domain
  signal reset_integrator_sync0 : std_logic := '0'; -- First synchronizer stage
  signal reset_integrator_sync1 : std_logic := '0'; -- Second synchronizer stage
  signal reset_integrator_sync2 : std_logic := '0'; -- Third synchronizer stage
  signal reset_integrator_ack   : std_logic := '0'; -- Acknowledge edge detect

  -- CDC for time_dac_ctrl + tdc_out_bitstream (clk_tdc -> clk_sys) - 3-FF scalar chain
  signal time_dac_ctrl_tdc   : std_logic := '0'; -- clk_tdc domain (slow loop DC trim)
  signal tdc_bitstream_sys   : std_logic := '0'; -- clk_sys domain (fast path) - NOT USED, see dac_bitstream_sys
  signal tdc_valid_sys       : std_logic := '0'; -- clk_sys domain (derived from toggle)
  signal tdc_bitstream_sync0 : std_logic := '0'; -- First synchronizer stage
  signal tdc_bitstream_sync1 : std_logic := '0'; -- Second synchronizer stage
  signal tdc_bitstream_sync2 : std_logic := '0'; -- Third synchronizer stage

  -- CDC for dac_out_ff (actual ΔΣ bitstream to filter) - clk_tdc -> clk_sys
  signal dac_bitstream_sys   : std_logic := '0'; -- clk_sys domain (actual ds output)
  signal dac_bitstream_hold  : std_logic := '0'; -- Held version sampled at CIC CE edge (tdc_valid_sys)
  signal dac_bitstream_sync0 : std_logic := '0'; -- First synchronizer stage
  signal dac_bitstream_sync1 : std_logic := '0'; -- Second synchronizer stage
  signal dac_bitstream_sync2 : std_logic := '0'; -- Third synchronizer stage

  -- CDC for closed_loop_en (boot state machine enable) - clk_sys -> clk_tdc
  signal closed_loop_en_tdc   : std_logic := '0'; -- clk_tdc domain (synchronized)
  signal closed_loop_en_sync0 : std_logic := '0'; -- First synchronizer stage
  signal closed_loop_en_sync1 : std_logic := '0'; -- Second synchronizer stage
  signal closed_loop_en_sync2 : std_logic := '0'; -- Third synchronizer stage

  -- CDC for closed_loop_drive (handoff status) - clk_tdc -> clk_sys
  signal closed_loop_drive_sys   : std_logic := '0'; -- clk_sys domain (synchronized)
  signal closed_loop_drive_sync0 : std_logic := '0'; -- First synchronizer stage
  signal closed_loop_drive_sync1 : std_logic := '0'; -- Second synchronizer stage
  signal closed_loop_drive_sync2 : std_logic := '0'; -- Third synchronizer stage

  -- CIC decimator signals
  signal cic_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out : std_logic;

  -- Equalizer signals
  signal eq_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out : std_logic;

  -- Low-pass filter signals
  signal lp_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out : std_logic;

  -- Status monitoring
  signal activity_counter : unsigned(15 downto 0) := (others => '0');
  signal valid_counter    : unsigned(7 downto 0)  := (others => '0');
  signal overflow_counter : unsigned(7 downto 0)  := (others => '0');
  signal cic_counter      : unsigned(7 downto 0)  := (others => '0');
  signal sample_counter   : unsigned(7 downto 0)  := (others => '0');
  signal status_reg       : std_logic_vector(7 downto 0);

  -- Component-less differential feedback: Internal signal for comparator output
  signal s_comparator_out_internal : std_logic; -- Output from differential GPIO comparator
  signal analog_in_mux             : std_logic; -- Muxed input: test signal or comparator output

  -- Boot-up sweep to find input voltage range, then calibrate and close loop
  -- Sweep solves cold-start problem for ANY input voltage (not just mid-scale)
  -- States: SWEEP (find crossing) -> DITHER (calibrate TDC) -> CLOSED_LOOP
  type   T_BOOT_STATE          is (ST_SWEEP, ST_DITHER, ST_WAIT_FOR_START, ST_CLOSED_LOOP);
  signal boot_state            : T_BOOT_STATE := ST_SWEEP;
  signal closed_loop_en        : std_logic    := '0';
  signal closed_loop_drive     : std_logic    := '0';
  signal cl_valid_seen         : std_logic    := '0'; -- Latch: went true after first tdc_valid in closed-loop
  signal cl_switch_pend        : std_logic    := '0'; -- Request to switch on next start edge (one-shot)
  signal use_closed_loop       : std_logic    := '0'; -- STICKY handoff latch (clk_sys): once both conditions met, never falls back
  -- CDC synchronizer for use_closed_loop (clk_sys -> clk_tdc)
  signal use_closed_loop_sync1 : std_logic    := '0'; -- First FF stage
  signal use_closed_loop_tdc   : std_logic    := '0'; -- Second FF stage (safe to use in clk_tdc)
  signal dac_boot_ff           : std_logic    := '0'; -- Boot dither toggle at start rate
  signal dac_out_ff            : std_logic    := '0'; -- Final registered DAC output (goes to IO cell) TODO: Remove if possible, adds latency
  signal dac_integrator_ff     : std_logic    := '0'; -- DAC decision from integrator (registered to align with loop_integrator update)

  -- Calibration enable: only collect TDC samples during DITHER phase (not SWEEP)
  -- CDC synchronized from clk_sys (boot_state) to clk_tdc (calibration process)
  signal cal_enable_sys   : std_logic := '0'; -- Asserted in ST_DITHER only
  signal cal_enable_sync0 : std_logic := '0'; -- CDC synchronizer stage 0
  signal cal_enable_sync1 : std_logic := '0'; -- CDC synchronizer stage 1
  signal cal_enable_tdc   : std_logic := '0'; -- Safe to use in clk_tdc domain

  -- Boot sweep signals: ramp DAC duty to find comparator crossing point
  signal sweep_duty_counter : unsigned(7 downto 0) := (others => '0'); -- 0-255 duty cycle during sweep
  signal sweep_period_cnt   : unsigned(7 downto 0) := (others => '0'); -- Counter within each sweep period
  signal sweep_found_cross  : std_logic            := '0'; -- Latched when crossing detected
  signal sweep_cross_duty   : unsigned(7 downto 0) := to_unsigned(128, 8); -- Duty where crossing occurred (default 50%)
  signal sweep_init_integ   : signed(31 downto 0)  := (others => '0'); -- Initial integrator value from sweep
  signal comp_sync0         : std_logic            := '0'; -- Comparator synchronizer stage 0
  signal comp_sync1         : std_logic            := '0'; -- Comparator synchronizer stage 1
  signal comp_prev          : std_logic            := '0'; -- Previous comparator state (for edge detect)

  -- Watchdog: detect if closed-loop never produces a valid sample
  signal cl_watch_cnt                : unsigned(15 downto 0) := (others => '0'); -- Counts Start edges after CL enable
  signal cl_sticky_no_valid          : std_logic             := '0'; -- Sticky flag if watchdog expires
  signal reset_integrator_at_handoff : std_logic             := '0'; -- Pulse to reset integrator when handoff executes

  -- TDC center calibration CDC (clk_tdc -> clk_sys -> clk_tdc)
  signal cal_done_sync1 : std_logic := '0'; -- CDC for cal_done
  signal cal_done_tdc   : std_logic := '0'; -- cal_done in clk_tdc domain

  -- TDC sample latch: Hold TDC measurement until next ref_clock edge for PI loop
  -- This enables DAC to be synchronized to ref_clock for proper TDC operation
  signal tdc_sample_latch : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0'); -- Latched TDC value
  signal tdc_sample_ready : std_logic                          := '0'; -- Flag: new TDC sample ready for PI processing

  -- Start pulse detection in p_sigma_delta (local edge detect)
  signal start_pulse_pi    : std_logic := '0'; -- Single-cycle pulse on ref_sync2 rising edge
  signal ref_sync2_prev_pi : std_logic := '0'; -- Previous ref_sync2 for edge detection

  -- CDC for TDC output to clk_sys domain (3-FF scalar chain for MTBF)
  signal tdc_out_hold     : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_toggle       : std_logic                          := '0';
  signal tdc_toggle_sync0 : std_logic                          := '0'; -- First synchronizer stage
  signal tdc_toggle_sync1 : std_logic                          := '0'; -- Second synchronizer stage
  signal tdc_toggle_sync2 : std_logic                          := '0'; -- Third synchronizer stage
  signal tdc_out_sys      : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal new_sample_sys   : std_logic                          := '0'; -- Edge detect output (MUST init to prevent power-on glitch)

  -- CDC for single-bit status signals
  signal tdc_overflow_sync : std_logic_vector(1 downto 0) := (others => '0');
  signal tdc_lost_sync     : std_logic_vector(1 downto 0) := (others => '0');

  -- Reference edge synchronization (3-FF scalar chain for MTBF at 400 MHz)
  signal ref_sync0 : std_logic := '0';  -- First synchronizer stage
  signal ref_sync1 : std_logic := '0';  -- Second synchronizer stage
  signal ref_sync2 : std_logic := '0';  -- Third synchronizer stage

  -- Synchronized reset for clk_tdc domain
  signal reset_tdc : std_logic;

  -- ========================================================================
  -- Signals for Multi-bit Delta-Sigma Loop
  -- ========================================================================
  -- No additional signals needed - using existing loop_integrator (32-bit)
  -- Millivolt conversion for UART output (LP filter -> mV)

  signal mv_code : unsigned(15 downto 0) := (others => '0');

  -- ========================================================================
  -- Helper Functions
  -- ========================================================================
  -- XOR helper for combining polarity bits
  function xor2(a, b : std_logic) return std_logic is
  begin
    return (a xor b);
  end function;
begin

  -- ========================================================================
  -- Reset Synchronizer for clk_tdc domain
  -- ========================================================================
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

  -- Debug outputs for TDC characterization
  debug_tdc_out   <= tdc_out;
  debug_tdc_valid <= tdc_valid;

  -- Use GPIO IP differential comparator output directly
  s_comparator_out_internal <= comparator_in;

  -- Mux between external analog input and internal digital test signal
  -- This logic uses the s_comparator_out_internal signal
  -- which is driven by either the sim or synth block.

  analog_in_mux <= s_comparator_out_internal;

  -- ========================================================================
  -- Boot Sweep + Dither + Closed-Loop FSM
  -- ========================================================================
  -- NEW ARCHITECTURE: Sweep to find input voltage before calibration
  -- FSM States:
  --   ST_SWEEP:          Ramp DAC duty 0->100% to find comparator crossing point
  --   ST_DITHER:         Dither around found crossing point for TDC calibration
  --   ST_WAIT_FOR_START: Wait for start edge to synchronize handoff
  --   ST_CLOSED_LOOP:    Enable PI controller feedback
  --
  -- This solves the bootstrap problem for ANY input voltage (not just mid-scale)
  p_boot_dither : process(clk_tdc)
    -- Use VARIABLES for immediate-update edge detection
    variable v_start_pulse    : std_logic;
    variable v_ref_sync2_prev : std_logic                  := '0'; -- MUST be a variable for correct edge detect
    variable v_boot_counter   : integer range 0 to 5000001 := 0; -- Counter for various timeouts (larger range for sim)
    variable v_sweep_cycles   : integer range 0 to 511     := 0; -- Cycles within current duty period
    variable v_sweep_high_cnt : integer range 0 to 511     := 0; -- How many cycles DAC should be high
    variable v_comp_cross_cnt : integer range 0 to 15      := 0; -- Debounce counter for crossing detection
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        -- Start in SWEEP state to find input voltage
        boot_state         <= ST_SWEEP;
        closed_loop_en     <= '0';
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
            if sweep_period_cnt < sweep_duty_counter then
              dac_boot_ff <= '1';       -- High portion of duty cycle
            else
              dac_boot_ff <= '0';       -- Low portion of duty cycle
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
              if v_boot_counter >= 10000 then -- ~25µs settling time
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

            -- PWM around sweep_cross_duty (found crossing point) instead of 50%
            -- This ensures TDC calibration happens near the actual operating point
            -- Use sweep_period_cnt as PWM counter, compare against sweep_cross_duty
            if v_start_pulse = '1' then
              -- Increment period counter (8-bit PWM period)
              sweep_period_cnt <= sweep_period_cnt + 1;

              -- Simple PWM: output HIGH when counter < duty, LOW otherwise
              -- Add small dither (±8 duty levels) using period_cnt bit 7
              if sweep_period_cnt(7) = '0' then
                -- Upper half of dither cycle: use (duty + 8)
                if sweep_period_cnt(6 downto 0) < resize(sweep_cross_duty + 8, 7) then
                  dac_boot_ff <= '1';
                else
                  dac_boot_ff <= '0';
                end if;
              else
                -- Lower half of dither cycle: use (duty - 8)
                if sweep_period_cnt(6 downto 0) < resize(sweep_cross_duty - 8, 7) then
                  dac_boot_ff <= '1';
                else
                  dac_boot_ff <= '0';
                end if;
              end if;

              if GC_SIM and (v_boot_counter mod 10000) = 0 then
                report "ST_DITHER_PWM: duty=" & integer'image(to_integer(sweep_cross_duty)) & " cnt=" & integer'image(to_integer(sweep_period_cnt)) & " at " & time'image(now);
              end if;
            end if;

            -- Wait for calibration completion OR timeout (safety valve)
            -- Priority: cal_done takes precedence over timer
            -- Increased timeout to 2,000,000 cycles @ 400MHz = 5ms to ensure calibration completes
            v_boot_counter := v_boot_counter + 1;
            if cal_done = '1' then
              if GC_SIM then
                report "BOOT_FSM: cal_done='1'! Transitioning ST_DITHER -> ST_WAIT_FOR_START at " & time'image(now) & " after " & integer'image(v_boot_counter) & " cycles";
              end if;
              boot_state     <= ST_WAIT_FOR_START;
              v_boot_counter := 0;
            elsif v_boot_counter >= 2000000 then
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
            closed_loop_en <= '0';
            cal_enable_sys <= '0';      -- Calibration complete, stop collecting

            -- Continue PWM dithering around sweep_cross_duty
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

            -- On Start edge, move to closed-loop
            if v_start_pulse = '1' then
              if GC_SIM then
                report "BOOT_FSM: Transitioning ST_WAIT_FOR_START -> ST_CLOSED_LOOP at " & time'image(now) & " (cal_done=" & std_logic'image(cal_done) & ")";
              end if;
              boot_state <= ST_CLOSED_LOOP;
            end if;

          -- ================================================================
          -- STATE 3: CLOSED_LOOP - Enable PI controller feedback
          -- ================================================================
          when ST_CLOSED_LOOP =>
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
        end case;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Reference Clock Synchronization (Pure 3-FF scalar synchronizer for CDC)
  -- ========================================================================
  -- Synchronize reference clock edge to TDC domain (400 MHz)
  p_ref_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- Pure 3-FF scalar synchronizer: NO reset clause to avoid CDC violation
      -- Power-on initialization handles reset state (default '0')
      -- Direct async capture from ref_clock (2 MHz domain)
      ref_sync0 <= ref_clock;
      ref_sync1 <= ref_sync0;
      ref_sync2 <= ref_sync1;
    end if;
  end process;

  -- Feed quantizer with synchronized reference edge
  ref_phases(0) <= ref_sync2;

  -- ========================================================================
  -- TDC Output CDC to clk_sys Domain
  -- ========================================================================
  -- Toggle-based handshake for multi-bit TDC value
  -- Free-running (no reset) - toggle mechanism is self-starting
  p_tdc_cdc_src : process(clk_tdc)
    variable v_sample_count : integer range 0 to 100000 := 0;
  begin
    if rising_edge(clk_tdc) then
      if tdc_valid = '1' then
        tdc_out_hold <= tdc_out;
        tdc_toggle   <= not tdc_toggle;
        if GC_SIM and C_VERBOSE_DEBUG then
          v_sample_count := v_sample_count + 1;
          if v_sample_count <= 20 or v_sample_count = 100 or v_sample_count = 1000 or (v_sample_count mod 500) = 0 then
            report "TDC_TOP: tdc_valid='1' sample " & integer'image(v_sample_count) & " value=" & integer'image(to_integer(signed(tdc_out))) & " toggle=" & std_logic'image(not tdc_toggle);
          end if;
        end if;
      end if;
    end if;
  end process;

  -- Toggle synchronizer (3-FF scalar chain for MTBF) and edge detect in clk_sys domain
  p_tdc_cdc_dst : process(clk_sys)
    variable v_toggle_prev : std_logic := '0'; -- Previous sync2 value for edge detection
    variable v_pulse       : std_logic; -- Edge detect pulse (combinatorial)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        -- POWER-ON SAFETY: Clear edge detector state during reset
        -- Prevents corrupting CIC decimation counter with glitch on tdc_valid_sys
        new_sample_sys <= '0';
        v_toggle_prev  := '0';
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
          tdc_out_sys <= tdc_out_hold;  -- Capture when toggle changes
          if GC_SIM and C_VERBOSE_DEBUG then
            report "TDC_TOP: new_sample_sys pulse, captured value=" & integer'image(to_integer(signed(tdc_out_hold)));
          end if;
        end if;
      end if;
    end if;
  end process;

  -- 2-FF synchronizers for single-bit status
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

  -- ========================================================================
  -- TDC Quantizer (Analog Feedback Only)
  -- ========================================================================
  -- Pure analog feedback design via dac_out_ff -> GPIO pin -> comparator
  -- Digital time-DAC disabled to avoid interference and timing violations
  i_tdc : entity work.tdc_quantizer
    generic map(
      GC_TDL_LANES    => 4,
      GC_TDL_LENGTH   => 128,
      GC_COARSE_BITS  => 8,
      GC_OUTPUT_WIDTH => GC_TDC_OUTPUT,
      GC_TIME_DAC_DEN => 256,
      GC_SIM          => GC_SIM
    )

    port map(
      clk_sys          => clk_sys,
      clk_tdc          => clk_tdc,
      reset            => reset,
      analog_in        => analog_in_mux, -- Use muxed signal (external or internal test)
      ref_phases       => ref_phases,
      time_dac_ctrl    => '0',          -- Not used when digital TDAC disabled
      coarse_bias      => coarse_bias_tdc, -- Synchronized to clk_tdc
      fine_phase_adj   => (others => '0'), -- Not used, TDL centering via coarse_bias adjustment
      invert_polarity  => '0',
      tdc_out          => tdc_out,
      tdc_valid        => tdc_valid,
      overflow         => tdc_overflow,
      lost_sample      => tdc_lost_sample,
      fine_at_comp_out => fine_at_comp_out, -- TDL centering calibration output
      fine_valid_out   => fine_valid_out -- TDL centering calibration valid
    );

  -- ========================================================================
  -- Loop Filter (Proportional-Integral with Saturation)
  -- ========================================================================
  -- PI controller with configurable gains and anti-windup
  -- Output = Kp * error + Ki * integral(error)
  -- Configuration CDC synchronizers (clk_sys -> clk_tdc)
  -- PI configuration: toggle-based handshake for atomic multi-bit update

  p_pi_config_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- Synchronize the commit toggle (2-FF synchronizer)
      pi_cfg_toggle_s0 <= pi_cfg_commit;
      pi_cfg_toggle_s1 <= pi_cfg_toggle_s0;

      -- Detect edge and atomically capture all config registers
      if pi_cfg_toggle_s1 /= pi_cfg_toggle_s0 then
        kp_shift_tdc         <= kp_shift;
        ki_shift_tdc         <= ki_shift;
        loop_enable_tdc      <= loop_enable;
        integrator_limit_tdc <= integrator_limit;
      end if;
    end if;
  end process;

  -- Coarse bias CDC: simple 2-FF synchronizer (4-bit bus, quasi-static)
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

  -- Integrator reset CDC synchronizer (clk_sys -> clk_tdc) - 3-FF scalar chain
  -- Free-running synchronizer (no reset) for MTBF integrity
  p_integrator_reset_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- 3-FF scalar synchronizer: each stage explicitly written
      reset_integrator_sync0 <= reset_integrator_req;
      reset_integrator_sync1 <= reset_integrator_sync0;
      reset_integrator_sync2 <= reset_integrator_sync1;
      reset_integrator_ack   <= reset_integrator_sync2 and not reset_integrator_sync1; -- Edge detect
    end if;
  end process;

  -- ========================================================================
  -- Fast Path: Extract 1-bit Delta-Sigma Bitstream from TDC
  -- ========================================================================
  -- In digital time-DAC mode, tdc_out is already the ΔΣ bitstream (±1)
  -- Extract the sign bit to create a 1-bit stream for the CIC

  tdc_out_bitstream <= not tdc_out(tdc_out'high); -- Invert sign: +1→'1', -1→'0'

  -- ========================================================================
  -- TDC Auto-Calibration Process
  -- ========================================================================
  -- During boot dither phase, track MIN and MAX TDC values seen.
  -- The center is computed as (MIN + MAX) / 2, which gives the true midpoint
  -- between the two quantization levels regardless of duty cycle or drift.
  -- This is more robust than averaging, which fails if Vn drifts during cal.
  p_tdc_calibration : process(clk_tdc)
    constant C_CAL_SAMPLES : integer := 16; -- Collect enough samples to see both levels
    variable v_tdc_sample  : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_tdc_min     : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_tdc_max     : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_center      : signed(GC_TDC_OUTPUT - 1 downto 0);
    -- Function to check if a signed value contains metavalues (for simulation)
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

            if GC_SIM then
              report "TDC_CAL: Sample " & integer'image(to_integer(cal_sample_cnt)) & " value=" & integer'image(to_integer(v_tdc_sample)) & " min=" & integer'image(to_integer(v_tdc_min)) & " max=" & integer'image(to_integer(v_tdc_max)) & " at " & time'image(now);
            end if;

            -- After enough samples, compute center as midpoint of min/max
            if cal_sample_cnt = to_unsigned(C_CAL_SAMPLES - 1, cal_sample_cnt'length) then
              -- Center = (min + max) / 2  (arithmetic right shift for signed division)
              v_center       := shift_right(v_tdc_min + v_tdc_max, 1);
              tdc_center_tdc <= v_center;
              cal_done       <= '1';

              if GC_SIM then
                report "TDC_CAL: COMPLETE! min=" & integer'image(to_integer(v_tdc_min)) & " max=" & integer'image(to_integer(v_tdc_max)) & " center=" & integer'image(to_integer(v_center)) & " from " & integer'image(C_CAL_SAMPLES) & " samples at " & time'image(now);
              end if;
            end if;
          else
            if GC_SIM then
              report "TDC_CAL: Skipping metavalue sample at " & time'image(now);
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- TDC Sample Latch: Capture TDC measurement for use at next ref_clock edge
  -- ========================================================================
  p_tdc_latch : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdc_sample_latch <= (others => '0');
        tdc_sample_ready <= '0';
      else
        -- Latch TDC output when valid, mark as ready for PI loop
        if tdc_valid = '1' then
          tdc_sample_latch <= tdc_out;
          tdc_sample_ready <= '1';
          if GC_SIM then
            report "TDC_LATCH: Captured tdc_out=" & integer'image(to_integer(tdc_out)) & " at " & time'image(now);
          end if;
        end if;

        -- Clear ready flag when PI loop consumes the sample (at start_pulse)
        if start_pulse_pi = '1' and tdc_sample_ready = '1' then
          tdc_sample_ready <= '0';
          if GC_SIM then
            report "TDC_LATCH: Cleared ready flag at start_pulse at " & time'image(now);
          end if;
        end if;
      end if;
    end if;
  end process;

  p_sigma_delta : process(clk_tdc)
    variable v_tdc_error          : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_error_clamp        : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_proportional       : signed(31 downto 0);
    variable v_pi_sum             : signed(31 downto 0);
    variable v_new_integrator     : signed(31 downto 0);
    variable v_old_integrator     : signed(31 downto 0); -- Copy of loop_integrator for debugging
    variable v_error_contribution : signed(31 downto 0); -- Error term for integral
  begin
    if rising_edge(clk_tdc) then
      -- Start pulse edge detection (local to this process)
      start_pulse_pi    <= ref_sync2 and not ref_sync2_prev_pi;
      ref_sync2_prev_pi <= ref_sync2;

      if reset_tdc = '1' then
        -- Integrator initialization: start at 0 and let PI converge
        loop_integrator   <= (others => '0');
        dac_integrator_ff <= '0';
        ref_sync2_prev_pi <= '0';
        start_pulse_pi    <= '0';
        if GC_SIM then
          report "INTEGRATOR_INIT: Starting loop_integrator=0 at " & time'image(now);
        end if;
      else
        -- PI UPDATE: Runs on ref_clock rising edge (start_pulse) using latched TDC sample
        -- This synchronizes DAC updates to ref_clock for proper TDC timing measurement
        if start_pulse_pi = '1' and closed_loop_en_tdc = '1' and tdc_sample_ready = '1' then
          -- ========================================================================
          -- PI CONTROLLER: Direct TDC-based delta-sigma modulation
          -- ========================================================================
          -- The TDC output directly represents the timing difference between
          -- Start (ref_clock) and Stop (comparator crossing).
          -- 
          -- TDC TIMING MODEL (from testbench comparator):
          --   crossing_time = 250ns + GAIN * (Vp - Vn)
          --   - When Vp > Vn: crossing is LATE (> 250ns) → TDC output POSITIVE
          --   - When Vp < Vn: crossing is EARLY (< 250ns) → TDC output NEGATIVE
          --
          -- DELTA-SIGMA FEEDBACK:
          --   - Positive TDC (Vp > Vn, Vn too low): DAC='1' to charge Vn up toward Vp
          --   - Negative TDC (Vp < Vn, Vn too high): DAC='0' to discharge Vn toward Vp
          --
          -- SIGN CONVENTION: Use TDC directly as error (positive TDC → positive PI → DAC='1')
          -- This matches the testbench model where positive TDC means we need more DAC=1 time.
          v_tdc_error := signed(tdc_sample_latch); -- Use LATCHED TDC output (positive = need more DAC HIGH)

          -- ERROR CLAMPING: Prevent runaway from extreme TDC values when Vn saturates
          -- This provides soft limiting to keep the loop stable
          if v_tdc_error > C_ERROR_MAX then
            v_error_clamp := to_signed(C_ERROR_MAX, v_error_clamp'length);
          elsif v_tdc_error < -C_ERROR_MAX then
            v_error_clamp := to_signed(-C_ERROR_MAX, v_error_clamp'length);
          else
            v_error_clamp := v_tdc_error;
          end if;

          -- Proportional term: P = error >> kp_shift
          v_proportional := shift_right(resize(v_error_clamp, 32), to_integer(kp_shift_tdc));

          -- Integral term: I += error >> ki_shift (compute new value as variable for immediate use)
          -- FULL WORKAROUND: Use only variables, avoid signal reads in expressions
          v_old_integrator := loop_integrator;

          -- Compute error contribution as separate variable
          v_error_contribution := shift_right(resize(v_error_clamp, 32), to_integer(ki_shift_tdc));

          -- DEBUG: Break down the calculation step by step
          report "INTEG_CALC: loop_int=" & integer'image(to_integer(loop_integrator)) & " v_old=" & integer'image(to_integer(v_old_integrator)) & " err=" & integer'image(to_integer(v_error_clamp)) & " err_contrib=" & integer'image(to_integer(v_error_contribution)) & " ki_shift=" & integer'image(to_integer(ki_shift_tdc));

          -- DEBUG: Test if addition is the problem
          report "BEFORE_ADD: v_old=" & integer'image(to_integer(v_old_integrator)) & " v_err_contrib=" & integer'image(to_integer(v_error_contribution));

          -- WORKAROUND: Do addition using INTEGER arithmetic to avoid signed operator bug
          v_new_integrator := to_signed(to_integer(v_old_integrator) + to_integer(v_error_contribution), 32);

          -- Debug: IMMEDIATELY AFTER ADDITION, BEFORE SATURATION
          report "AFTER_ADD: v_new_integrator=" & integer'image(to_integer(v_new_integrator)) & " (expected=" & integer'image(to_integer(v_old_integrator) + to_integer(v_error_contribution)) & ")";

          -- Debug: Check integrator_limit_tdc value
          report "SAT_CHECK: limit=" & integer'image(to_integer(integrator_limit_tdc)) & " I_before=" & integer'image(to_integer(v_new_integrator));

          -- Apply integrator saturation limit
          report "SAT_LIMITS: upper=" & integer'image(to_integer(signed(resize(integrator_limit_tdc, 32)))) & " lower=" & integer'image(to_integer(-signed(resize(integrator_limit_tdc, 32)))) & " v_new=" & integer'image(to_integer(v_new_integrator));

          if v_new_integrator > signed(resize(integrator_limit_tdc, 32)) then
            report "SAT_UPPER: Clamping " & integer'image(to_integer(v_new_integrator)) & " to upper limit";
            v_new_integrator := signed(resize(integrator_limit_tdc, 32));
          elsif v_new_integrator < -signed(resize(integrator_limit_tdc, 32)) then
            report "SAT_LOWER: Clamping " & integer'image(to_integer(v_new_integrator)) & " to lower limit";
            v_new_integrator := -signed(resize(integrator_limit_tdc, 32));
          end if;

          -- Update integrator register with saturated value
          loop_integrator <= v_new_integrator;

          -- PI output: sum proportional + integral
          v_pi_sum := v_proportional + v_new_integrator;

          -- 1-bit quantizer: Use PI sum for DAC decision (proper delta-sigma modulation)
          -- The integrator accumulates error over time and provides the "memory" needed
          -- to produce the correct duty cycle for any input voltage.
          --
          -- Delta-sigma negative feedback topology:
          --   - Positive PI sum → need more DAC HIGH → DAC = '1'
          --   - Negative PI sum → need more DAC LOW  → DAC = '0'
          -- The integrator naturally finds the operating point where average error = 0.
          if v_pi_sum >= 0 then
            report "QUANTIZER: PI_sum=" & integer'image(to_integer(v_pi_sum)) & " >= 0, setting DAC='1' (charge up Vn) at " & time'image(now);
            dac_integrator_ff <= '1';   -- Positive PI sum → DAC HIGH (charge Vn up)
          else
            report "QUANTIZER: PI_sum=" & integer'image(to_integer(v_pi_sum)) & " < 0, setting DAC='0' (discharge Vn) at " & time'image(now);
            dac_integrator_ff <= '0';   -- Negative PI sum → DAC LOW (discharge Vn)
          end if;

          -- Enhanced debug report (always enabled in simulation) - MOVED AFTER DAC ASSIGNMENT
          report "PI_LOOP: tdc_latch=" & integer'image(to_integer(signed(tdc_sample_latch))) & " err=" & integer'image(to_integer(v_error_clamp)) & " P=" & integer'image(to_integer(v_proportional)) & " I_new=" & integer'image(to_integer(v_new_integrator)) & " I_old=" & integer'image(to_integer(loop_integrator)) & " PI_sum=" & integer'image(to_integer(v_pi_sum)) & " DAC=" & std_logic'image(dac_integrator_ff) & " gains(kp=" & integer'image(to_integer(kp_shift_tdc)) & " ki=" & integer'image(to_integer(ki_shift_tdc)) & ")";
        elsif start_pulse_pi = '1' and closed_loop_en_tdc = '1' then
          -- Start pulse but no TDC sample ready - hold previous DAC output
          if GC_SIM then
            report "PI_HOLD: start_pulse but tdc_sample_ready='" & std_logic'image(tdc_sample_ready) & "', holding DAC at " & time'image(now);
          end if;
        end if;                         -- end if start_pulse_pi
      end if;                           -- end else (not reset)

      -- ALWAYS report integrator value at end of this clock cycle for debugging
      if start_pulse_pi = '1' and closed_loop_en_tdc = '1' and tdc_sample_ready = '1' then
        report "CYCLE_END: loop_int_will_be=" & integer'image(to_integer(loop_integrator)) & " at " & time'image(now);
      end if;
      time_dac_ctrl_tdc <= '0';
    end if;
  end process;

  -- Convert ADC output to millivolts (0..1300 mV range)
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

        -- Convert signed Q-format [-1, +1) to millivolts [0, 1300]
        -- CIC already removes its DC gain (R³) internally via C_SCALE_SHIFT
        -- LP output is Q-format: m = y / 2^(W-1)
        -- Voltage mapping: V = 650*m + 650 (for 1.3V full scale)
        -- Combined: V = (y*650)/2^(W-1) + 650
        v_lp_signed := signed(lp_data_out);

        -- Scale: shift by (GC_DATA_WIDTH - 1) to undo Q-format
        -- Multiply creates 64-bit result, shift, then resize to 32-bit
        v_scaled_mv := resize(shift_right(resize(v_lp_signed, 32) * to_signed(650, 32),
                                          GC_DATA_WIDTH - 1), 32);

        -- Add midscale offset (650mV for 1.3V full scale)
        v_scaled_mv := v_scaled_mv + to_signed(650, 32);

        -- Saturate to 0..1300 mV range
        if v_scaled_mv < 0 then
          mv_code <= (others => '0');
          if GC_SIM then
            report "MV_CONVERSION [" & integer'image(v_mv_count) & "]: lp_data_out=" & integer'image(to_integer(v_lp_signed)) & ", scaled_mv=" & integer'image(to_integer(v_scaled_mv)) & " (SATURATED TO 0)" severity note;
          end if;
        elsif v_scaled_mv > to_signed(1300, 32) then
          mv_code <= to_unsigned(1300, 16);
          if GC_SIM then
            report "MV_CONVERSION [" & integer'image(v_mv_count) & "]: lp_data_out=" & integer'image(to_integer(v_lp_signed)) & ", scaled_mv=" & integer'image(to_integer(v_scaled_mv)) & " (SATURATED TO 1300)" severity note;
          end if;
        else
          mv_code <= unsigned(v_scaled_mv(15 downto 0));
          if GC_SIM then
            report "MV_CONVERSION [" & integer'image(v_mv_count) & "]: lp_data_out=" & integer'image(to_integer(v_lp_signed)) & ", scaled_mv=" & integer'image(to_integer(v_scaled_mv)) & ", mv_code=" & integer'image(to_integer(unsigned(v_scaled_mv(15 downto 0)))) severity note;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- CDC: time_dac_ctrl + tdc_valid (clk_tdc -> clk_sys)
  -- ========================================================================
  -- Reuse the robust toggle-based CDC for clock enable
  -- The toggle mechanism already works reliably for new_sample_sys
  tdc_valid_sys <= new_sample_sys;      -- One clk_sys cycle pulse when toggle changes

  -- 3-FF scalar synchronizer for tdc_out_bitstream (MTBF improvement) - Fast path for ds bitstream

  p_bitstream_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      -- 3-FF scalar synchronizer: NO reset clause (avoids multi-clock-domain driving synchronizer)
      tdc_bitstream_sync0 <= tdc_out_bitstream;
      tdc_bitstream_sync1 <= tdc_bitstream_sync0;
      tdc_bitstream_sync2 <= tdc_bitstream_sync1;

      -- Latch the synchronized value (3rd FF output) on the CE edge (when new sample arrives)
      if new_sample_sys = '1' then
        tdc_bitstream_sys <= tdc_bitstream_sync2;
      end if;
    end if;
  end process;

  -- 3-FF scalar synchronizer for dac_out_ff (actual ΔΣ output bitstream)
  -- This is the CORRECT signal to filter - it's the 1-bit quantizer output
  -- that encodes the input signal via duty cycle modulation
  p_dac_bitstream_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      -- 3-FF scalar synchronizer: NO reset clause
      dac_bitstream_sync0 <= dac_out_ff;
      dac_bitstream_sync1 <= dac_bitstream_sync0;
      dac_bitstream_sync2 <= dac_bitstream_sync1;

      -- Continuous update (no conditional latch) - CIC clock enable controls sampling
      dac_bitstream_sys <= dac_bitstream_sync2;
      -- Latch DAC bitstream at CIC CE edge to avoid sampling moving target
      -- This mirrors the pattern used for tdc_bitstream_sys
      if tdc_valid_sys = '1' then
        dac_bitstream_hold <= dac_bitstream_sync2;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- CDC: closed_loop_en (clk_sys -> clk_tdc)
  -- ========================================================================
  -- 3-FF scalar synchronizer for closed_loop_en (boot state machine enable)

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

  -- ========================================================================
  -- TDL Centering Calibration FSM (clk_tdc domain)
  -- ========================================================================
  -- During boot dither phase, adjust coarse_bias until fine_at_comp_out is
  -- centered (~32768). This aligns the comparator crossing to the TDL center.
  --
  -- Algorithm:
  -- 1. Wait for initial coarse_bias calibration to complete in tdc_quantizer
  -- 2. Accumulate C_TDL_CAL_SAMPLES of fine_at_comp_out
  -- 3. Compute average: tdl_cal_fine_avg = tdl_cal_fine_acc >> 3
  -- 4. If avg < (32768 - 8192): coarse_bias needs adjustment
  --    If avg > (32768 + 8192): coarse_bias needs adjustment
  -- 5. Adjust coarse_bias by ±1 and repeat until centered or timeout
  --
  -- Note: Changing coarse_bias shifts which clock edge we reference,
  -- which indirectly shifts where the fine timing lands within the TDL.

  p_tdl_centering_cal : process(clk_tdc)
    constant C_TDL_CAL_TIMEOUT : natural                             := 100000; -- Max cycles for centering (~250µs @ 400MHz)
    constant C_TDL_CAL_SETTLE  : natural                             := 1000; -- Cycles to wait after coarse_bias_cal change
    variable v_settle_cnt      : natural range 0 to C_TDL_CAL_SETTLE := 0;
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdl_cal_done        <= '0';
        tdl_cal_enabled     <= '0';
        tdl_cal_sample_cnt  <= (others => '0');
        tdl_cal_fine_acc    <= (others => '0');
        tdl_cal_fine_avg    <= (others => '0');
        tdl_cal_timeout_cnt <= (others => '0');
        tdl_cal_sweep_dir   <= '0';     -- Initial sweep direction: increase coarse_bias
        coarse_bias_cal     <= get_default_coarse_bias; -- Reset to default value
        tdl_cal_use_cal     <= '0';     -- Don't use calibrated value until we start adjusting
        v_settle_cnt        := 0;

      elsif cal_enable_tdc = '1' and tdl_cal_done = '0' then
        -- TDL centering calibration active during dither phase
        tdl_cal_enabled     <= '1';
        tdl_cal_use_cal     <= '1';     -- Start using calibrated value once calibration starts
        tdl_cal_timeout_cnt <= tdl_cal_timeout_cnt + 1;

        -- Timeout check
        if tdl_cal_timeout_cnt >= C_TDL_CAL_TIMEOUT then
          -- Timeout: accept current coarse_bias_cal as best effort
          tdl_cal_done <= '1';
          report "TDL_CAL: Timeout after " & integer'image(to_integer(tdl_cal_timeout_cnt)) & " cycles, coarse_bias_cal=" & integer'image(to_integer(coarse_bias_cal)) & " fine_avg=" & integer'image(to_integer(tdl_cal_fine_avg)) severity note;
        end if;

        -- Wait for settle time after coarse_bias_cal adjustment
        if v_settle_cnt > 0 then
          v_settle_cnt := v_settle_cnt - 1;
        else
          -- Accumulate fine_at_comp_out samples
          if fine_valid_out = '1' then
            tdl_cal_fine_acc   <= tdl_cal_fine_acc + resize(fine_at_comp_out, 20);
            tdl_cal_sample_cnt <= tdl_cal_sample_cnt + 1;

            -- After C_TDL_CAL_SAMPLES samples, compute average and adjust
            if tdl_cal_sample_cnt = to_unsigned(C_TDL_CAL_SAMPLES - 1, 4) then
              -- Compute average (divide by 8 = shift right 3)
              tdl_cal_fine_avg <= resize(shift_right(tdl_cal_fine_acc + resize(fine_at_comp_out, 20), 3), 16);

              -- Check if centered (within tolerance: 24576 to 40960 = 32768 ± 8192)
              if (tdl_cal_fine_acc + resize(fine_at_comp_out, 20)) >= shift_left(to_unsigned(24576, 20), 3) and (tdl_cal_fine_acc + resize(fine_at_comp_out, 20)) <= shift_left(to_unsigned(40960, 20), 3) then
                -- Centered! Calibration complete
                tdl_cal_done <= '1';
                report "TDL_CAL: SUCCESS! coarse_bias_cal=" & integer'image(to_integer(coarse_bias_cal)) & " fine_avg=" & integer'image(to_integer(resize(shift_right(tdl_cal_fine_acc + resize(fine_at_comp_out, 20), 3), 16))) severity note;

              elsif (tdl_cal_fine_acc + resize(fine_at_comp_out, 20)) < shift_left(to_unsigned(24576, 20), 3) then
                -- Average too low (< 24576): fine timing near 0, need to shift reference
                -- Increase coarse_bias_cal to shift which clock edge we reference
                if coarse_bias_cal < 254 then
                  coarse_bias_cal <= coarse_bias_cal + 1;
                  report "TDL_CAL: fine_avg LOW, increasing coarse_bias_cal to " & integer'image(to_integer(coarse_bias_cal) + 1) severity note;
                end if;
                -- Reset accumulator and wait for settle
                tdl_cal_fine_acc   <= (others => '0');
                tdl_cal_sample_cnt <= (others => '0');
                v_settle_cnt       := C_TDL_CAL_SETTLE;

              else
                -- Average too high (> 40960): fine timing near max, need to shift reference
                -- Decrease coarse_bias_cal to shift which clock edge we reference
                if coarse_bias_cal > 1 then
                  coarse_bias_cal <= coarse_bias_cal - 1;
                  report "TDL_CAL: fine_avg HIGH, decreasing coarse_bias_cal to " & integer'image(to_integer(coarse_bias_cal) - 1) severity note;
                end if;
                -- Reset accumulator and wait for settle
                tdl_cal_fine_acc   <= (others => '0');
                tdl_cal_sample_cnt <= (others => '0');
                v_settle_cnt       := C_TDL_CAL_SETTLE;
              end if;
            end if;
          end if;
        end if;                         -- v_settle_cnt = 0

      elsif cal_enable_tdc = '0' then
        -- Calibration phase ended (moved to closed loop)
        -- Keep using calibrated value
        tdl_cal_enabled <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- CDC: closed_loop_drive (clk_tdc -> clk_sys)
  -- ========================================================================
  -- 3-FF scalar synchronizer for closed_loop_drive (handoff status feedback)

  p_closed_loop_drive_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      -- 3-FF scalar synchronizer
      closed_loop_drive_sync0 <= closed_loop_drive;
      closed_loop_drive_sync1 <= closed_loop_drive_sync0;
      closed_loop_drive_sync2 <= closed_loop_drive_sync1;
      closed_loop_drive_sys   <= closed_loop_drive_sync2;
    end if;
  end process;

  -- ========================================================================
  -- Sticky Handoff Latch (clk_tdc domain)
  -- ========================================================================
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

  -- ========================================================================
  -- Debug: DAC Duty Cycle Monitor (simulation only)
  -- ========================================================================
  p_dac_duty_monitor : process(clk_sys)
    variable v_dac_count   : integer range 0 to 1000 := 0;
    variable v_total_count : integer range 0 to 1000 := 0;
    variable v_duty_pct    : integer range 0 to 100  := 0;
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        v_dac_count   := 0;
        v_total_count := 0;
      elsif tdc_valid_sys = '1' then
        -- Count samples
        v_total_count := v_total_count + 1;
        if dac_bitstream_hold = '1' then
          v_dac_count := v_dac_count + 1;
        end if;

        -- Report every 16 samples
        if v_total_count mod 16 = 0 then
          v_duty_pct    := (v_dac_count * 100) / v_total_count;
          report "DAC_DUTY: " & integer'image(v_dac_count) & "/" & integer'image(v_total_count) & " = " & integer'image(v_duty_pct) & "% (last 16 samples: " & integer'image((v_dac_count * 100) / 16) & "%)";
          -- Reset for next window
          v_dac_count   := 0;
          v_total_count := 0;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- CIC SINC3 Decimator
  -- ========================================================================

  -- Feed the 1-bit ds bitstream (from TDC) with clock enable (tdc_valid)
  i_cic : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => GC_DECIMATION,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )

    port map(
      clk      => clk_sys,              --TODO should be 400MHz
      reset    => reset,
      data_in  => dac_bitstream_hold,   -- 1-bit ds bitstream held at CE edge (avoids sampling moving target)
      ce       => tdc_valid_sys,        -- Sample strobe (CDC'd from clk_tdc)
      data_out => cic_data_out,
      valid    => cic_valid_out
    );

  -- ========================================================================
  -- FIR Equalizer (on clk_sys for timing closure) - BYPASSED FOR FAST TEST
  -- ========================================================================
  -- i_eq : entity work.fir_equalizer
  --   generic map(
  --     GC_INPUT_WIDTH  => GC_DATA_WIDTH,
  --     GC_OUTPUT_WIDTH => GC_DATA_WIDTH
  --   )
  --
  --   port map(
  --     clk       => clk_sys,
  --     reset     => reset,
  --     data_in   => cic_data_out,
  --     valid_in  => cic_valid_out,
  --     data_out  => eq_data_out,
  --     valid_out => eq_valid_out
  --   );

  -- ========================================================================
  -- FIR Low-Pass Filter (on clk_sys for timing closure) - BYPASSED FOR FAST TEST
  -- ========================================================================
  -- i_lp : entity work.fir_lowpass
  --   generic map(
  --     GC_INPUT_WIDTH  => GC_DATA_WIDTH,
  --     GC_OUTPUT_WIDTH => GC_DATA_WIDTH
  --   )
  --
  --   port map(
  --     clk       => clk_sys,
  --     reset     => reset,
  --     data_in   => eq_data_out,
  --     valid_in  => eq_valid_out,
  --     data_out  => lp_data_out,
  --     valid_out => lp_valid_out
  --   );

  -- FAST TEST BYPASS: Connect CIC directly to output (skip FIR filters)
  eq_data_out  <= cic_data_out;
  eq_valid_out <= cic_valid_out;
  lp_data_out  <= cic_data_out;
  lp_valid_out <= cic_valid_out;

  -- Streaming output: Report actual measured voltage as millivolts
  -- The CIC/EQ/LP chain processes both DC and AC signals with unity gain
  -- Final scaling converts filtered output to 0-1200mV range
  sample_data  <= std_logic_vector(resize(mv_code, GC_DATA_WIDTH));
  sample_valid <= lp_valid_out;

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
        cl_valid_seen      <= '0';
        cl_switch_pend     <= '0';
        closed_loop_drive  <= '0';
        cl_watch_cnt       <= (others => '0');
        cl_sticky_no_valid <= '0';
        start_div2         <= '0';
        keepalive_en       <= '0';
      else
        -- ======================================================================
        -- Edge Keepalive: Divide Start by 2 for Slow Dither
        -- ======================================================================
        if v_start_pulse = '1' then
          start_div2 <= not start_div2;
        end if;

        -- ======================================================================
        -- Step 1: Latch once we observe a closed-loop tdc_valid
        -- ======================================================================
        if (closed_loop_en = '1') and (tdc_valid = '1') and (cl_valid_seen = '0') then
          cl_valid_seen <= '1';
          if GC_SIM then
            report "HANDOFF: cl_valid_seen='1' at time " & time'image(now) & " (first closed-loop tdc_valid)";
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
          -- NOTE: Do NOT reset integrator at handoff - let it stay at 0 and converge
          -- The sweep init value can be wrong/stale and clobber the PI loop
          -- reset_integrator_at_handoff <= '1'; -- DISABLED
          if GC_SIM then
            report "HANDOFF: cl_switch executed (integrator NOT reset) at " & time'image(now);
          end if;
          -- else
          -- reset_integrator_at_handoff <= '0'; -- Clear pulse (not needed since always '0')
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
            if cl_watch_cnt = to_unsigned(200, cl_watch_cnt'length) then
              cl_sticky_no_valid <= '1'; -- Flag for SignalTap/debug
            end if;
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

        -- ======================================================================
        -- Edge Keepalive Enable: Activate When Edges Starve
        -- ======================================================================
        -- Enable comparator-tracking when no TDC samples for extended period
        -- This creates crossings to restart the loop
        -- Threshold: 20 ref_clock cycles (~10µs @ 2MHz) without TDC samples
        -- NOTE: Must be low enough to recover loop quickly, but not so low
        --       that it triggers during normal operation
        if (closed_loop_drive = '1') and (cl_watch_cnt > to_unsigned(20, cl_watch_cnt'length)) then
          keepalive_en <= '1';
          report "KEEPALIVE: Enabled at cl_watch_cnt=" & integer'image(to_integer(cl_watch_cnt)) & " at " & time'image(now);
        else
          if keepalive_en = '1' then
            report "KEEPALIVE: Disabled (was enabled) at " & time'image(now);
          end if;
          keepalive_en <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Clock Domain Crossing Synchronizer: use_closed_loop (clk_sys -> clk_tdc)
  -- ========================================================================
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

  -- DAC Output Register (Single registered path to IO cell)
  -- ========================================================================
  -- Drive the 1-bit feedback pin (N-pin of differential comparator)
  -- This is the ANALOG FEEDBACK.

  -- HANDOFF ARCHITECTURE:
  --   Boot Phase (closed_loop_en='0'): Drive with boot dither (dac_boot_ff)
  --                                     This "kick-starts" the TDC and primes the integrator
  --   Closed-Loop Phase (closed_loop_en='1'): Drive with PI controller output (dac_integrator_ff)
  --                                             The PI feedback loop takes over
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
          -- Boot/Dither Phase: Drive with the boot dither signal
          -- This creates the initial edges to start the TDC.
          -- Continues until BOTH handoff AND polarity detection are complete.
          dac_out_ff <= dac_boot_ff;
        else
          -- ================================================================
          -- CLOSED LOOP: Bang-Bang Control (like rc_adc_top)
          -- ================================================================
          -- The DAC directly follows the comparator output at clk_tdc rate.
          -- This creates rapid hunting around Vp, generating many crossings.
          -- The TDC measures crossing times for enhanced resolution.
          -- The PI integrator adjusts the DC bias to center the loop.
          --
          -- This hybrid approach:
          --   1. Bang-bang ensures crossings occur at any input voltage
          --   2. TDC measures crossing time for sub-LSB resolution
          --   3. PI loop fine-tunes the operating point
          --
          -- DAC = comparator XOR integrator_sign
          -- - When integrator positive: invert comparator (shift Vn up)
          -- - When integrator negative: follow comparator (shift Vn down)
          -- This modulates the bang-bang duty cycle based on PI output.
          dac_out_ff <= comp_sync1 xor dac_integrator_ff;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Default Configuration (no memory-mapped interface)
  -- ========================================================================
  -- Static configuration with default values
  -- PI LOOP TUNING: For stable delta-sigma, integral gain must be high enough
  -- to dominate the proportional term over a few samples.
  -- REDUCED GAINS to prevent oscillation when TDC error is large
  clear_status         <= '0';
  kp_shift             <= to_unsigned(7, 4); -- Proportional (divide by 128) - reduced to prevent overshoot
  ki_shift             <= to_unsigned(7, 4); -- Integral (divide by 128) - reduced for slower, stable response
  loop_enable          <= '1';          -- Always enabled
  integrator_limit     <= to_signed(8192, 32); -- Larger limit for more range
  reset_integrator_req <= '0';          -- No reset requests
  pi_cfg_commit        <= '0';          -- No CDC updates needed
  coarse_bias          <= get_default_coarse_bias;

  -- ========================================================================
  -- Status Monitoring
  -- ========================================================================
  p_status : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        activity_counter <= (others => '0');
        valid_counter    <= (others => '0');
        overflow_counter <= (others => '0');
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

        if new_sample_sys = '1' then
          sample_counter <= sample_counter + 1;
        end if;

        -- Count overflows using synchronized signal
        if tdc_overflow_sync(1) = '1' then
          overflow_counter <= overflow_counter + 1;
        end if;
      end if;
    end if;
  end process;

  -- Status register (use CDC-synchronized signals from clk_tdc domain)
  status_reg(0) <= new_sample_sys;      -- TDC sample arrived (clk_sys domain)
  status_reg(1) <= cic_valid_out;
  status_reg(2) <= eq_valid_out;
  status_reg(3) <= lp_valid_out;
  status_reg(4) <= tdc_overflow_sync(1);
  status_reg(5) <= tdc_lost_sync(1);
  status_reg(6) <= activity_counter(15);
  status_reg(7) <= '0';

end architecture;
