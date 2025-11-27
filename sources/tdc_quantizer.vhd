library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.clk_rst_pkg.all;

entity tdc_quantizer is
  generic(
    GC_TDL_LANES    : positive range 1 to 16 := 4; -- Must be power of 2 for averaging
    GC_TDL_LENGTH   : positive               := 128; -- Taps per TDL lane
    GC_COARSE_BITS  : positive               := 8; -- Coarse counter width
    GC_OUTPUT_WIDTH : positive               := 16; -- Total TDC output width
    GC_TIME_DAC_DEN : positive               := 256; -- Digital Time-DAC step denominator (step = 1/GC_TIME_DAC_DEN)
    GC_SIM          : boolean                := false -- Simulation mode: true = explicit delays, false = synthesis
  );
  port(
    -- Clocks
    clk_sys         : in  std_logic;    -- System clock (e.g., 100 MHz)
    clk_tdc         : in  std_logic;    -- TDC fast clock (e.g., 400-600 MHz)
    reset           : in  std_logic;
    analog_in       : in  std_logic;    -- LVDS comparator output (true differential) or digital test signal

    -- Reference timing (Start edge source - should be clk_tdc or synchronous)
    ref_phases      : in  std_logic_vector(0 downto 0); -- Single phase (ref_phases(0) = Start source)

    -- 1-bit Time DAC control (feedback from loop filter)
    time_dac_ctrl   : in  std_logic;    -- 1 = advance, 0 = retard reference

    -- Polarity control (allows P/N swap correction without rebuild)
    invert_polarity : in  std_logic;    -- 1 = invert sign, 0 = normal

    -- Coarse bias for window check adjustment
    -- Allows compensation for systematic timing offsets (e.g., digital self-test mode, routing delays)
    -- Applied as: dcoarse_adjusted = dcoarse - coarse_bias before window check
    coarse_bias     : in  unsigned(7 downto 0); -- Subtract from dcoarse before window check (full 8-bit range)

    -- Fine phase adjustment for TDL centering calibration
    -- Adjusts start_coarse capture timing to center fine_at_comp in TDL range
    -- Range: 0-255 sub-coarse steps (wraps within one coarse cycle)
    fine_phase_adj  : in  unsigned(7 downto 0) := (others => '0'); -- Fine phase offset for TDL centering

    -- TDC output
    tdc_out         : out signed(GC_OUTPUT_WIDTH - 1 downto 0);
    tdc_valid       : out std_logic;
    overflow        : out std_logic;
    lost_sample     : out std_logic;    -- Sticky flag: overflow occurred since reset

    -- TDL centering calibration output
    fine_at_comp_out : out unsigned(15 downto 0); -- fine_avg_fp captured at comp edge (for TDL centering calibration)
    fine_valid_out   : out std_logic              -- Pulses when fine_at_comp_out is updated
  );
end entity;

architecture rtl of tdc_quantizer is

  -- ========================================================================
  -- Attributes
  -- ========================================================================
  attribute KEEP : boolean;

  -- ========================================================================
  -- Helper Functions
  -- ========================================================================
  function clog2(x : positive) return natural is
    variable v_bit_count : unsigned(4 downto 0)  := (others => '0');
    variable v_remainder : unsigned(29 downto 0) := to_unsigned(x - 1, 30);
  begin
    while v_remainder > 0 loop
      v_remainder := shift_right(v_remainder, 1);
      v_bit_count := v_bit_count + 1;
    end loop;
    return to_integer(v_bit_count);
  end function;

  -- ========================================================================
  -- Constants
  -- ========================================================================
  constant C_TDL_BITS : natural := clog2(GC_TDL_LENGTH + 1);

  -- Fixed-point for fine time: Q0.F format (fraction of Tclk_tdc)
  constant C_FINE_FRAC_BITS : natural                             := 16;
  constant C_TIME_FP_BITS   : natural                             := GC_COARSE_BITS + C_FINE_FRAC_BITS;
  -- 0.5 coarse periods in Q(COARSE.FINE) format = 0.5 × 2^FINE = 2^(FINE-1)
  constant C_HALF_COARSE_FP : signed(C_TIME_FP_BITS - 1 downto 0) := to_signed(2 ** (C_FINE_FRAC_BITS - 1), C_TIME_FP_BITS);

  -- Time-DAC: Step size in Q0.F format (fraction of Tclk_tdc per step)
  -- Step = 1/GC_TIME_DAC_DEN (e.g., 1/256 for fine granularity)
  constant C_TIME_DAC_STEP_FP : unsigned(C_FINE_FRAC_BITS - 1 downto 0) := to_unsigned((2 ** C_FINE_FRAC_BITS) / GC_TIME_DAC_DEN, C_FINE_FRAC_BITS);

  -- Used in dual-lobe bias selection to find min(|adj0|, |adjm|, |adjp|)
  function abs_s9(x : signed(GC_COARSE_BITS downto 0)) return unsigned is
    variable v_ux : unsigned(GC_COARSE_BITS downto 0);
  begin
    if x(x'high) = '1' then
      v_ux := unsigned(-x);             -- Two's complement negation, same width
    else
      v_ux := unsigned(x);
    end if;
    return v_ux;
  end function;

  -- ========================================================================
  -- Types
  -- ========================================================================
  type T_TDL_THERM        is array (0 to GC_TDL_LENGTH - 1) of std_logic;
  type T_TDL_THERM_ARRAY  is array (0 to GC_TDL_LANES - 1) of T_TDL_THERM;
  type T_FINE_CODE_ARRAY  is array (0 to GC_TDL_LANES - 1) of unsigned(C_TDL_BITS - 1 downto 0);
  type T_FINE_FP_ARRAY    is array (0 to GC_TDL_LANES - 1) of unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  type T_CAL_LUT          is array (0 to GC_TDL_LENGTH) of unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  type T_LANE_SCALE_ARRAY is array (0 to GC_TDL_LANES - 1) of unsigned(15 downto 0); -- Q1.15 format

  -- Pipelined encoder types (16 groups of 8 bits each for 128-bit TDL)
  constant C_POPCOUNT_GROUPS   : positive := 16;
  constant C_GROUP_SIZE        : positive := GC_TDL_LENGTH / C_POPCOUNT_GROUPS; -- 8 bits per group
  type     T_GROUP_COUNT_ARRAY is array (0 to C_POPCOUNT_GROUPS - 1) of unsigned(3 downto 0); -- 0-8 count
  type     T_LANE_GROUP_COUNTS is array (0 to GC_TDL_LANES - 1) of T_GROUP_COUNT_ARRAY;

  -- Partial sums for 3-stage adder tree (timing closure at 400MHz)
  constant C_PARTIAL_GROUPS     : positive := 4; -- Split 16 groups into 4 partials
  constant C_GROUPS_PER_PARTIAL : positive := C_POPCOUNT_GROUPS / C_PARTIAL_GROUPS; -- 4 groups each
  type     T_PARTIAL_SUM_ARRAY  is array (0 to C_PARTIAL_GROUPS - 1) of unsigned(C_TDL_BITS - 1 downto 0);
  type     T_LANE_PARTIAL_SUMS  is array (0 to GC_TDL_LANES - 1) of T_PARTIAL_SUM_ARRAY;

  -- ========================================================================
  -- Signals
  -- ========================================================================
  -- Reset synchronization: reset comes from clk_sys but used in clk_tdc domain
  signal reset_tdc : std_logic;

  signal analog_crossing : std_logic;

  -- Start/Stop arming (one stop per reference period)
  signal start_pulse     : std_logic := '0'; -- Single-cycle pulse on ref_phases(0) rising edge
  signal ref_phase0_prev : std_logic := '0'; -- For edge detection on ref_phases(0)
  signal armed           : std_logic := '0'; -- Arms stop capture after Start, clears after first Stop

  -- Stop synchronization (3-FF to match ref path depth)
  signal analog_sync         : std_logic_vector(2 downto 0) := (others => '0');
  signal analog_stop_mark    : std_logic                    := '0';
  signal stop_level_at_start : std_logic                    := '0'; -- LVDS level at Start, for first-transition detection
  signal edge_inhibit        : unsigned(1 downto 0)         := (others => '0'); -- 2-cycle inhibit counter for robustness

  -- ========================================================================
  -- CDC Attributes for Synchronizers
  -- ========================================================================
  -- Mark CDC synchronizer chains for proper MTBF analysis and timing constraint recognition
  attribute altera_attribute : string;

  -- analog_sync chain: analog_in → clk_tdc domain (3-FF synchronizer for Stop)
  attribute altera_attribute of analog_sync : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";

  -- TDL
  signal tdl_chains       : T_TDL_THERM_ARRAY;
  signal tdl_bank_current : T_TDL_THERM_ARRAY;
  signal tdl_bank_prev    : T_TDL_THERM_ARRAY;
  signal tdl_bank_prev_q  : T_TDL_THERM_ARRAY; -- Registered prev to break same-edge capture ambiguity
  signal tdl_captured     : T_TDL_THERM_ARRAY;

  -- Gray counter (banked like TDL for alignment)
  signal coarse_counter_bin : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');

  -- Start/Stop timestamps
  -- Start coarse: HELD (not pipelined) to preserve actual start time for correct delta
  signal start_coarse_hold : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');

  signal stop_coarse_bin : unsigned(GC_COARSE_BITS - 1 downto 0)   := (others => '0');
  signal stop_fine_fp    : unsigned(C_FINE_FRAC_BITS - 1 downto 0) := (others => '0');

  signal start_coarse_at_mark : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');
  signal start_coarse_pipe_d1 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d2 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d3 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d4 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d5 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d6 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d7 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d8 : unsigned(GC_COARSE_BITS - 1 downto 0);

  -- Pipeline delay registers to align coarse+stop_mark with 8-stage fine processing
  signal stop_coarse_pipe_d1 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d2 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d3 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d4 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d5 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d6 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d7 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d8 : unsigned(GC_COARSE_BITS - 1 downto 0);

  signal analog_stop_mark_d1       : std_logic := '0';
  signal analog_stop_mark_d2       : std_logic := '0';
  signal analog_stop_mark_d3       : std_logic := '0';
  signal analog_stop_mark_d4       : std_logic := '0';
  signal analog_stop_mark_d5       : std_logic := '0';
  signal analog_stop_mark_d6       : std_logic := '0';
  signal analog_stop_mark_d7       : std_logic := '0';
  signal analog_stop_mark_d8       : std_logic := '0';
  signal analog_stop_mark_pipe     : std_logic := '0';
  signal analog_stop_mark_comp_pre : std_logic := '0'; -- Extra pipeline stage for dcoarse alignment
  signal analog_stop_mark_comp     : std_logic := '0';

  -- Digital Time-DAC command (applied in interval computation)
  -- 0.0 initial allows Stage II centering to work immediately, loop can start
  signal time_dac_cmd_fp : signed(C_FINE_FRAC_BITS downto 0) := to_signed(0, C_FINE_FRAC_BITS + 1);

  signal time_dac_cmd_ext_d1 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d2 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d3 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d4 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d5 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d6 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d7 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d8 : signed(C_TIME_FP_BITS - 1 downto 0);
  signal time_dac_cmd_ext_d9 : signed(C_TIME_FP_BITS - 1 downto 0);

  -- Registered, pipeline-aligned interval (before TDAC subtraction)
  signal s_delta_meas     : signed(C_TIME_FP_BITS - 1 downto 0) := (others => '0'); -- Intermediate: delta_t = (delta_coarse<<F) + delta_fine
  signal s_delta_meas_adj : signed(C_TIME_FP_BITS - 1 downto 0) := (others => '0'); -- Final: delta_t - TDAC

  -- Coarse delta computed as SIGNED from the start (prevents wraparound artifacts)
  signal dcoarse_signed_raw : signed(GC_COARSE_BITS downto 0); -- 9 bits for 8-bit coarse (signed)

  -- Auto-calibration: histogram of dcoarse_signed_raw for first 16 samples
  type   T_CALIB_HISTOGRAM      is array (0 to 255) of unsigned(3 downto 0); -- Count 0-15 for each bucket (256 bins for 8-bit coarse)
  signal calib_histogram        : T_CALIB_HISTOGRAM               := (others => (others => '0'));
  signal calib_sample_count     : unsigned(4 downto 0)            := (others => '0'); -- 0-31 (use first 16)
  signal calib_done             : std_logic                       := '0';
  signal coarse_bias_calibrated : unsigned(7 downto 0)            := (others => '0'); -- Calibrated bias value (8-bit)
  signal coarse_bias_effective  : unsigned(7 downto 0); -- Mux between input and calibrated (8-bit)
  signal dcoarse_for_calib      : signed(GC_COARSE_BITS downto 0) := (others => '0'); -- Registered dcoarse for calibration (breaks timing path)
  signal analog_stop_mark_calib : std_logic                       := '0'; -- Delayed stop_mark for calibration (breaks timing path)

  -- Startup delay counter for calibration
  -- Must wait for tdc_adc_top's boot dither + TB comparator to enter TDC mode (C_SWEEP_END_TIME=70µs in TB)
  -- Calibration must happen AFTER comparator is generating realistic TDC timing edges
  constant C_CALIB_DELAY   : natural                          := 40000; -- Wait 40000 cycles (~100µs @ 400MHz) for closed-loop TDC mode
  signal   calib_delay_cnt : integer range 0 to C_CALIB_DELAY := 0;
  signal   calib_ready     : std_logic                        := '0'; -- Single-bit flag: '1' when delay expires (breaks timing path)

  -- Multi-cycle mode-finding state machine (breaks timing path)
  signal calib_search_idx : integer range 0 to 256 := 0; -- Current bucket being checked (0-255 + 256 for pipeline flush)
  signal calib_max_count  : unsigned(3 downto 0)   := (others => '0'); -- Running max count
  signal calib_mode_value : integer range 0 to 255 := 0; -- Running mode (bucket with max count)
  signal calib_searching  : std_logic              := '0'; -- '1' when searching for mode

  -- Pipeline registers to break histogram read-modify-write timing path (3-stage pipeline for timing)
  signal calib_hist_idx_d1    : integer range 0 to 255 := 0; -- Stage 1: Captured index
  signal calib_hist_value_d1  : unsigned(3 downto 0)   := (others => '0'); -- Stage 1: Read value from histogram
  signal calib_hist_idx_d2    : integer range 0 to 255 := 0; -- Stage 2: Index
  signal calib_hist_value_d2  : unsigned(3 downto 0)   := (others => '0'); -- Stage 2: Value
  signal calib_hist_update_d2 : std_logic              := '0'; -- Stage 2: Update enable

  -- Pipeline registers to break search comparison timing path
  signal calib_hist_read_d1  : unsigned(3 downto 0)   := (others => '0'); -- Histogram value read for comparison
  signal calib_search_idx_d1 : integer range 0 to 255 := 0; -- Index for comparison result

  -- Pipeline/register helpers to break window-check long path
  signal s_d_used_stage  : signed(GC_COARSE_BITS downto 0)     := (others => '0'); -- Registered chosen coarse for window check
  signal s_dfine_u_stage : unsigned(C_FINE_FRAC_BITS downto 0) := (others => '0'); -- Registered fine ext (unsigned form)
  signal s_vld_pre       : std_logic                           := '0'; -- Pre-valid flag captured in Stage-1b, used by p_win_check to drive s_vld_pipe(0)

  -- Fine codes
  signal fine_codes_raw : T_FINE_CODE_ARRAY;
  signal fine_codes_fp  : T_FINE_FP_ARRAY;
  signal lane_offsets   : T_FINE_FP_ARRAY;
  signal lane_scales    : T_LANE_SCALE_ARRAY; -- Q1.15 per-lane gain
  signal fine_avg_fp    : unsigned(C_FINE_FRAC_BITS - 1 downto 0);

  -- Pipelined averaging tree signals (2-stage balanced tree for timing closure @ 400 MHz)
  signal sum01_r : unsigned(C_FINE_FRAC_BITS downto 0); -- Pair sum: lane 0+1 (with carry bit)
  signal sum23_r : unsigned(C_FINE_FRAC_BITS downto 0); -- Pair sum: lane 2+3 (with carry bit)

  -- Pipelined encoder intermediate signals
  signal tdl_captured_reg : T_TDL_THERM_ARRAY; -- Registered TDL capture for timing
  signal group_counts     : T_LANE_GROUP_COUNTS; -- Stage A output: popcount per group
  signal partial_sums     : T_LANE_PARTIAL_SUMS; -- Stage B1 output: partial sums (4 groups each)

  -- Interval computation pipeline stages (3-stage split for timing closure @ 400 MHz)
  -- Stage I: Window check
  signal s_win_ok1     : std_logic                            := '0';
  signal s_win_ok_pipe : std_logic_vector(2 downto 0)         := (others => '0'); -- Pipeline for window check result
  -- Stage II: Error computation, centering, range check
  signal s_centered    : signed(C_TIME_FP_BITS - 1 downto 0)  := (others => '0');
  signal s_ovf2        : std_logic                            := '0';
  -- Valid pipeline (6-bit shift register for additional timing stages)
  signal s_vld_pipe    : std_logic_vector(5 downto 0)         := (others => '0');
  -- Stage III-A: Rounding addition (separate cycle)
  signal s_round_sum   : signed(C_TIME_FP_BITS - 1 downto 0)  := (others => '0'); -- s_centered + round_const
  signal s_ovf_a       : std_logic                            := '0'; -- Overflow check result (pipelined)
  -- Stage III-B: Shift and resize (separate cycle)
  signal s_shifted     : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0'); -- After right-shift
  signal s_ovf_b       : std_logic                            := '0'; -- Overflow forwarded
  -- Stage III-C: Polarity application (separate cycle)
  signal s_rounded     : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0'); -- After polarity
  signal s_ovf_pipe    : std_logic                            := '0'; -- Pipeline overflow check result

  -- Window check intermediate signals (coarse bias adjustment)
  signal dcoarse_adjusted : signed(GC_COARSE_BITS downto 0) := (others => '0'); -- After subtracting bias

  -- ========================================================================
  -- Stage-1a/1b/1c/1d/1e Dual-Lobe Bias Selection Pipeline (synthesizer-friendly, no integer math)
  -- ========================================================================
  -- Stage-1a: Register inputs at comp edge, compute three candidate adjustments
  signal dcoarse_signed_at_comp : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- dcoarse_signed_raw frozen at comp edge (breaks timing path)
  signal fine_at_comp           : unsigned(C_FINE_FRAC_BITS - 1 downto 0) := (others => '0'); -- fine_avg_fp frozen at comp edge
  signal bias_at_comp           : unsigned(7 downto 0)                    := (others => '0'); -- coarse_bias frozen at comp edge (8-bit)
  signal adj0_s1                : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- raw - bias0
  signal adjm_s1                : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- raw - (bias-3)
  signal adjp_s1                : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- raw - (bias+3)
  signal s1a_valid              : std_logic                               := '0'; -- Handshake: Stage-1a→1b
  -- Stage-1b: Magnitude calculation
  signal mag0_s1b               : unsigned(GC_COARSE_BITS downto 0)       := (others => '0'); -- |adj0|
  signal magm_s1b               : unsigned(GC_COARSE_BITS downto 0)       := (others => '0'); -- |adjm|
  signal magp_s1b               : unsigned(GC_COARSE_BITS downto 0)       := (others => '0'); -- |adjp|
  signal adj0_s1b               : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- Copy of adj0 for next stage
  signal adjm_s1b               : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- Copy of adjm for next stage
  signal adjp_s1b               : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- Copy of adjp for next stage
  signal fine_s1b               : unsigned(C_FINE_FRAC_BITS downto 0)     := (others => '0'); -- Copy of fine for tie-breaking
  signal s1b_valid              : std_logic                               := '0'; -- Handshake: Stage-1b→1c
  -- Stage-1c: Min-of-three selection
  signal d_used_s1c             : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- Chosen adjusted coarse
  signal fine_s1c               : unsigned(C_FINE_FRAC_BITS downto 0)     := (others => '0'); -- Fine for dfine construction
  signal s1c_valid              : std_logic                               := '0'; -- Handshake: Stage-1c->1d
  -- Stage-1d: dfine construction + window check
  signal d_used_s1              : signed(GC_COARSE_BITS downto 0)         := (others => '0'); -- Chosen adjusted coarse (legacy name)
  signal dfine_ext_s1b          : signed(C_FINE_FRAC_BITS + 1 downto 0)   := (others => '0'); -- Fine with +-1.0 bias (registered for timing)
  signal s1d_valid              : std_logic                               := '0'; -- Handshake: Stage-1d->1e
  -- Stage-1e: 24-bit delta computation
  signal dfine_ext_s2           : signed(C_FINE_FRAC_BITS + 1 downto 0)   := (others => '0'); -- Fine with +-1.0 bias (legacy name, kept for Stage-II)
  -- ========================================================================

  -- Output (buffered for start-synchronous delivery)
  signal tdc_result_buf    : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  signal overflow_buf      : std_logic := '0';
  signal sample_ready      : std_logic := '0'; -- Internal flag: sample captured on Stop
  signal tdc_result        : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  signal tdc_valid_i       : std_logic := '0';
  signal overflow_i        : std_logic := '0';
  signal lost_sample_latch : std_logic := '0'; -- Sticky flag for overflow tracking

  -- Time-DAC enable control (prevents deadlock on startup)
  -- Time-DAC accumulator must stay at zero until first valid sample is produced,
  -- otherwise the window check rejects every measurement (Catch-22 deadlock)
  signal tdac_enable_after_lock : std_logic := '0';

  -- Window failure CDC (clk_tdc -> clk_sys) - toggle-based handshake
  signal win_fail_toggle       : std_logic := '0'; -- Toggles in clk_tdc when s_win_ok1='0'
  signal win_fail_toggle_sync0 : std_logic := '0'; -- 1st sync stage
  signal win_fail_toggle_sync1 : std_logic := '0'; -- 2nd sync stage
  signal win_fail_toggle_sync2 : std_logic := '0'; -- 3rd sync stage

  -- Calibration
  signal cal_lut : T_CAL_LUT;
begin

  -- ========================================================================
  -- Reset Synchronization (clk_sys -> clk_tdc)
  -- ========================================================================
  i_reset_sync : entity work.reset_synchronizer
    generic map(
      GC_ACTIVE_LOW => false            -- Active-high reset
    )
    port map(
      clk       => clk_tdc,
      async_rst => reset,
      sync_rst  => reset_tdc
    );

  -- ========================================================================
  -- LVDS Input (Quartus-managed differential pair with LVDS_E_3V standard)
  -- ========================================================================
  analog_crossing <= analog_in;

  -- ========================================================================
  -- Gray-Coded Free-Running Coarse Counter (with banking like TDL)
  -- ========================================================================
  p_coarse_counter : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- Increment binary counter (free-running, no reset)
      coarse_counter_bin <= coarse_counter_bin + 1;
    end if;
  end process;

  g_tdac_off : if false generate        -- Time DAC disabled, tie to zero
    -- Hard-tie everything to zero so every bit has a real driver and the nets are kept consistent.
    -- (Keeps timing/pipeline structure intact, but subtracts 0.)
    p_time_dac_cmd_tieoff : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        time_dac_cmd_fp <= (others => '0');
      end if;
    end process;
  end generate;

  -- ========================================================================
  -- Start Pulse Generation (Direct Edge Detection on ref_phases(0))
  -- ========================================================================
  p_start_edge_detect : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        ref_phase0_prev <= '0';
        start_pulse     <= '0';
      else
        ref_phase0_prev <= ref_phases(0);
        -- Rising edge detection: single-cycle pulse
        if ref_phases(0) = '1' and ref_phase0_prev = '0' then
          start_pulse <= '1';
        else
          start_pulse <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Arm/Disarm Logic (One Stop Per Reference Period)
  -- ========================================================================
  -- Armed set on Start pulse, cleared on first valid Stop edge
  -- Prevents late/extra LVDS crossings from corrupting interval
  -- Power-on default: disarmed (armed := '0'), first start_pulse will arm
  -- Captures LVDS level at Start for first-transition detection
  p_arm_control : process(clk_tdc)
    variable v_start_count : integer range 0 to 1000000 := 0;
  begin
    if rising_edge(clk_tdc) then
      -- Don't disarm if both start and stop occur in same cycle
      -- When start_pulse='1', always arm (don't check analog_stop_mark)
      -- This prevents race condition where same-cycle stop would prevent arming
      if start_pulse = '1' then
        armed               <= '1';     -- Arm on Start (takes priority over same-cycle stop)
        stop_level_at_start <= analog_sync(2); -- Remember LVDS level at Start for transition detection
        -- Debug: Report every 20th start pulse to show TDC is receiving reference clocks
        v_start_count       := v_start_count + 1;
        if GC_SIM and (v_start_count <= 15 or (v_start_count mod 100) = 0) then
          report "TDC_START: pulse #" & integer'image(v_start_count) & " analog_sync=" & std_logic'image(analog_sync(2)) & " sample_ready=" & std_logic'image(sample_ready) & " at " & time'image(now);
        end if;
      -- edge_inhibit cleared in p_stop_sync process to avoid multiple drivers
      elsif (analog_stop_mark = '1' and armed = '1') then
        armed <= '0';                   -- Disarm on Stop (only when start not active)
        if GC_SIM and (v_start_count <= 20 or (v_start_count mod 100) = 0) then
          report "TDC_STOP: analog_stop_mark detected, disarming at " & time'image(now);
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stop Mark Synchronization with Multi-Cycle Inhibit and Arming
  -- ========================================================================
  -- Edge detector with 2-cycle inhibit + armed gating
  -- Detects first transition from level captured at Start (either polarity)
  p_stop_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- 3-FF synchronizer for async LVDS input (matches ref path 3-FF depth)
      analog_sync <= analog_sync(1 downto 0) & analog_crossing;

      -- Clear inhibit on every Start (also done in p_arm_control, but safe to duplicate)
      if start_pulse = '1' then
        edge_inhibit <= (others => '0');
      end if;

      -- Edge detection: first transition from Start level (either rising or falling)
      -- Compare current level to level captured at Start, not to previous cycle
      -- Only trigger when: level changed from Start + armed + no inhibit
      if (analog_sync(2) /= stop_level_at_start) and (armed = '1') and (edge_inhibit = to_unsigned(0, edge_inhibit'length)) then
        analog_stop_mark <= '1';
        edge_inhibit     <= to_unsigned(2, edge_inhibit'length); -- Inhibit for 2 cycles
      else
        analog_stop_mark <= '0';
        -- Decrement inhibit counter (saturates at 0)
        if edge_inhibit > to_unsigned(0, edge_inhibit'length) then
          edge_inhibit <= edge_inhibit - 1;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- TDL Chains with Continuous Sampling
  -- ========================================================================
  g_tdl_lanes : for lane in 0 to GC_TDL_LANES - 1 generate
    signal    tdl_delay_chain         : std_logic_vector(0 to GC_TDL_LENGTH);
    attribute KEEP of tdl_delay_chain : signal is true;
  begin
    tdl_delay_chain(0) <= analog_crossing;

    -- Delay chain generation, pure buffer chain for both sim and synthesis
    g_taps : for tap in 0 to GC_TDL_LENGTH - 1 generate
      tdl_delay_chain(tap + 1) <= tdl_delay_chain(tap);

      -- Monitoring signal
      tdl_chains(lane)(tap) <= tdl_delay_chain(tap);
    end generate;

    p_tdl_sample : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        -- Shift bank chain: current → prev
        tdl_bank_prev(lane)    <= tdl_bank_current(lane);
        tdl_bank_current(lane) <= tdl_chains(lane);
        -- Register prev to break same-edge capture ambiguity
        tdl_bank_prev_q(lane)  <= tdl_bank_prev(lane);
      end if;
    end process;

    p_frame_select : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        if analog_stop_mark = '1' then
          tdl_captured(lane) <= tdl_bank_prev_q(lane);
        end if;
      end if;
    end process;

    -- ======================================================================
    -- Register TDL Captured Data for Timing Closure
    -- ======================================================================
    -- Additional pipeline stage to break the long combinational path from
    -- tdl_captured through bubble filter, monotone clamp, and popcount
    p_tdl_register : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        tdl_captured_reg(lane) <= tdl_captured(lane);
      end if;
    end process;

  end generate;

  -- ========================================================================
  -- Four-Stage Thermometer-to-Binary Encoder (clk_tdc)
  -- ========================================================================
  -- Stage 0: Register TDL capture (tdl_captured -> tdl_captured_reg)
  --          Breaks long combinational path for timing
  -- Stage 1: Bubble filter + monotone clamp + group popcount (16× parallel)
  -- Stage 2: Partial sums (4 groups of 4 counts each)
  -- Stage 3: Final sum (16 counts -> fine_codes_raw)
  --
  -- Total encoder pipeline: 4 cycles (Stage 0 + Stage 1 + Stage 2 + Stage 3)
  g_encoder : for lane in 0 to GC_TDL_LANES - 1 generate

    -- Intermediate signals for pipelined encoder
    signal therm_filt_s   : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal therm_filt_reg : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal therm_mono_s   : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal therm_mono_reg : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);

    -- Hierarchical prefix clamp (Level-A: intra-group, Level-B: inter-group)
    signal group_clamp     : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal group_any       : std_logic_vector(C_POPCOUNT_GROUPS - 1 downto 0);
    signal group_clamp_reg : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal group_any_reg   : std_logic_vector(C_POPCOUNT_GROUPS - 1 downto 0);
    signal group_prefix_on : std_logic_vector(C_POPCOUNT_GROUPS - 1 downto 0);

    -- Level-B prefix OR pipeline registers (timing closure @ 400 MHz)
    -- Split 16-deep chain into two 8-deep stages to meet timing
    signal prefix_half_a   : std_logic_vector(7 downto 0); -- Groups 0..7 prefix OR (combinatorial)
    signal prefix_half_a_r : std_logic_vector(7 downto 0); -- Registered first half
    signal prior_on_r      : std_logic; -- Registered flag: any of groups 0..7 had '1'

    -- B2 split registers (timing closure @ 400 MHz)
    signal sum_b2_01_r : unsigned(C_TDL_BITS - 1 downto 0);
    signal sum_b2_23_r : unsigned(C_TDL_BITS - 1 downto 0);

  begin

    -- ======================================================================
    -- Combinational: 3-tap Majority Bubble Filter
    -- ======================================================================
    p_bubble_filter : process(all)
    begin
      -- Edge taps pass through
      therm_filt_s(0)                 <= tdl_captured_reg(lane)(0);
      therm_filt_s(GC_TDL_LENGTH - 1) <= tdl_captured_reg(lane)(GC_TDL_LENGTH - 1);

      -- Middle taps: majority vote of 3 neighbors
      for i in 1 to GC_TDL_LENGTH - 2 loop
        if (tdl_captured_reg(lane)(i - 1) and tdl_captured_reg(lane)(i)) or (tdl_captured_reg(lane)(i) and tdl_captured_reg(lane)(i + 1)) or (tdl_captured_reg(lane)(i - 1) and tdl_captured_reg(lane)(i + 1)) then
          therm_filt_s(i) <= '1';
        else
          therm_filt_s(i) <= '0';
        end if;
      end loop;
    end process;

    -- ======================================================================
    -- Register: Bubble Filter Output
    -- ======================================================================
    -- Pipeline stage to break combinational path before clamp
    p_filt_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        therm_filt_reg <= therm_filt_s; -- Register bubble-filter output
      end if;
    end process;

    -- ======================================================================
    -- Hierarchical Monotone Clamp (2-Level: 8-bit intra, 16-bit inter)
    -- ======================================================================
    -- Level-A: Intra-group running OR (8-bit prefix per group)
    -- Level-B: Inter-group prefix OR (16 groups) + expansion
    -- This keeps both chains short: max 8-bit or 16-bit prefix depth

    -- Level-A: Within each 8-bit group, compute running OR
    g_group_clamp : for g in 0 to C_POPCOUNT_GROUPS - 1 generate
      constant C_L : integer := g * C_GROUP_SIZE;
      constant C_R : integer := g * C_GROUP_SIZE + C_GROUP_SIZE - 1;
    begin
      -- Running OR inside the group (8 deep only)
      group_clamp(C_L) <= therm_filt_reg(C_L);
      g_intra : for b in C_L + 1 to C_R generate
      begin
        group_clamp(b) <= group_clamp(b - 1) or therm_filt_reg(b);
      end generate;

      -- "any" bit for the group (last bit of the running OR)
      group_any(g) <= group_clamp(C_R);
    end generate;

    -- Register Level-A outputs
    p_group_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        group_clamp_reg <= group_clamp;
        group_any_reg   <= group_any;
      end if;
    end process;

    -- ======================================================================
    -- Level-B: Prefix OR over 16 groups (PIPELINED for 400 MHz timing)
    -- ======================================================================
    -- Split into two 8-deep stages to reduce critical path from 16 → 8 gates
    -- Stage B-prefix-1: Groups 0..7 (8-deep chain, combinatorial)
    prefix_half_a(0) <= group_any_reg(0);
    g_prefix_first_half : for g in 1 to 7 generate
    begin
      prefix_half_a(g) <= prefix_half_a(g - 1) or group_any_reg(g);
    end generate;

    -- Register first half prefix results
    p_prefix_half_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        prefix_half_a_r <= prefix_half_a; -- Register groups 0..7 prefix chain
        prior_on_r      <= prefix_half_a(7); -- Register summary: any of 0..7 had '1'
      end if;
    end process;

    -- Stage B-prefix-2: Groups 0..15 using registered first half
    -- Groups 0..7: Use registered prefix results
    g_prefix_passthrough : for g in 0 to 7 generate
    begin
      group_prefix_on(g) <= prefix_half_a_r(g);
    end generate;

    -- Groups 8..15: Extend with 8-deep chain using prior_on_r
    group_prefix_on(8) <= prior_on_r or group_any_reg(8);
    g_prefix_second_half : for g in 9 to C_POPCOUNT_GROUPS - 1 generate
    begin
      group_prefix_on(g) <= group_prefix_on(g - 1) or group_any_reg(g);
    end generate;

    -- Build final monotone vector with safe generate (no g-1 when g=0):
    --  - For the first group (g=0): use its registered intra-group running OR.
    --  - For subsequent groups (g>0): if any previous group had a '1', entire group is '1'.
    g_final_clamp : for g in 0 to C_POPCOUNT_GROUPS - 1 generate
      constant C_L : integer := g * C_GROUP_SIZE;
      constant C_R : integer := g * C_GROUP_SIZE + C_GROUP_SIZE - 1;
    begin
      -- Case g = 0: No previous groups
      g_group_zero : if g = 0 generate
        g_bits_first : for b in C_L to C_R generate
        begin
          -- First group: just use its registered intra-group running-OR
          therm_mono_s(b) <= group_clamp_reg(b);
        end generate;
      end generate;

      -- Case g > 0: Use prefix-OR from previous group
      g_group_nonzero : if g > 0 generate
        g_bits_later : for b in C_L to C_R generate
        begin
          -- If any previous group is 'on' -> this whole group is 1s
          -- else take this group's intra-group running-OR
          therm_mono_s(b) <= group_prefix_on(g - 1) or ((not group_prefix_on(g - 1)) and group_clamp_reg(b));
        end generate;
      end generate;
    end generate;

    -- Register: Final monotone clamp output
    p_mono_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        therm_mono_reg <= therm_mono_s;
      end if;
    end process;

    -- ======================================================================
    -- Stage A: Group Popcount (Registered)
    -- ======================================================================
    p_encode_stage_a : process(clk_tdc)
      variable v_popcount : integer range 0 to C_GROUP_SIZE;
    begin
      if rising_edge(clk_tdc) then
        -- Popcount each group of 8 bits from the registered monotone thermometer
        -- In a monotone thermometer, popcount of group N = number of '1's in that group
        -- Total edge position = sum of all group popcounts
        for g in 0 to C_POPCOUNT_GROUPS - 1 loop
          v_popcount            := 0;
          for b in 0 to C_GROUP_SIZE - 1 loop
            if therm_mono_reg(g * C_GROUP_SIZE + b) = '1' then
              v_popcount := v_popcount + 1;
            end if;
          end loop;
          group_counts(lane)(g) <= to_unsigned(v_popcount, 4);
        end loop;
      end if;
    end process;

    -- ======================================================================
    -- Stage B1: Partial Sums (Registered) - Timing Closure at 400 MHz
    -- ======================================================================
    -- Split 16 groups into 4 partial sums (groups 0-3, 4-7, 8-11, 12-15)
    -- This breaks the tall adder tree into two shallower stages
    p_encode_stage_b1 : process(clk_tdc)
      variable v_partial : unsigned(C_TDL_BITS - 1 downto 0);
    begin
      if rising_edge(clk_tdc) then
        -- Sum each set of 4 groups into a partial sum
        for p in 0 to C_PARTIAL_GROUPS - 1 loop
          v_partial             := (others => '0');
          for g in 0 to C_GROUPS_PER_PARTIAL - 1 loop
            v_partial := v_partial + resize(group_counts(lane)(p * C_GROUPS_PER_PARTIAL + g), C_TDL_BITS);
          end loop;
          partial_sums(lane)(p) <= v_partial;
        end loop;
      end if;
    end process;

    -- ======================================================================
    -- Stage B2a: Pairwise Add (Registered) - Timing Closure @ 400 MHz
    -- ======================================================================
    p_encode_stage_b2a : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        sum_b2_01_r <= partial_sums(lane)(0) + partial_sums(lane)(1);
        sum_b2_23_r <= partial_sums(lane)(2) + partial_sums(lane)(3);
      end if;
    end process;

    -- ======================================================================
    -- Stage B2b: Final Add to Fine Code (Registered)
    -- ======================================================================
    p_encode_stage_b2b : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        fine_codes_raw(lane) <= sum_b2_01_r + sum_b2_23_r;
      end if;
    end process;

  end generate;

  -- ========================================================================
  -- Calibration LUT (Normalized to Tclk_tdc, no ps constants)
  -- ========================================================================
  p_calibration : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        -- Initialize LUT with linear calibration in Q0.F format
        -- Stores normalized bin edges as fraction of one Tclk_tdc period
        -- LUT is independent of actual clock frequency
        -- Will be replaced by code-density calibration at startup
        for i in 0 to GC_TDL_LENGTH loop
          if i = GC_TDL_LENGTH then
            -- Avoid wrap at top bin: use 2^F - 1 instead of 2^F
            -- This yields 0xFFFF (=0.99998) and prevents dfine->0 wrap when fully saturated
            cal_lut(i) <= to_unsigned((2 ** C_FINE_FRAC_BITS) - 1, C_FINE_FRAC_BITS);
          else
            -- Linear assumption: bin i represents i/GC_TDL_LENGTH of one period
            -- In Q0.F: (i * 2^F) / GC_TDL_LENGTH
            cal_lut(i) <= to_unsigned((i * (2 ** C_FINE_FRAC_BITS)) / GC_TDL_LENGTH, C_FINE_FRAC_BITS);
          end if;
        end loop;
        -- Initialize per-lane calibration
        for lane in 0 to GC_TDL_LANES - 1 loop
          lane_offsets(lane) <= (others => '0'); -- Zero offset
          lane_scales(lane)  <= to_unsigned(2 ** 15, 16); -- Unity gain (1.0 in Q1.15)
        end loop;
      end if;
    end if;
  end process;

  g_calibrate : for lane in 0 to GC_TDL_LANES - 1 generate
    p_cal_apply : process(clk_tdc)
      variable v_idx    : integer range 0 to GC_TDL_LENGTH;
      variable v_cal_fp : unsigned(C_FINE_FRAC_BITS - 1 downto 0);
      variable v_scaled : unsigned(C_FINE_FRAC_BITS + 15 downto 0); -- Q1.15 * Q0.16 -> Q1.31
    begin
      if rising_edge(clk_tdc) then
        v_idx := to_integer(fine_codes_raw(lane));
        if v_idx <= GC_TDL_LENGTH then
          -- 1) Apply calibration LUT (normalized to Tclk_tdc fraction)
          v_cal_fp := cal_lut(v_idx);

          -- 2) Apply per-lane gain correction (DNL compensation)
          --    Multiply by Q1.15 scale factor, then shift right 15 bits
          v_scaled := v_cal_fp * lane_scales(lane); -- Q0.16 * Q1.15 -> Q1.31
          v_cal_fp := resize(shift_right(v_scaled, 15), C_FINE_FRAC_BITS); -- Q1.31 -> Q0.16

          -- 3) Apply per-lane offset correction with modulo wrap
          --    This handles edges near 0/1 boundary correctly (circular offset)
          if v_cal_fp >= lane_offsets(lane) then
            v_cal_fp := v_cal_fp - lane_offsets(lane);
          else
            -- Wrap around: (v_cal_fp + 1.0) - offset
            -- In Q0.16: add 2^16 then subtract offset
            v_cal_fp := (v_cal_fp + to_unsigned(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS)) - lane_offsets(lane);
          end if;

          -- 4) Final modulo wrap if result exceeds 1.0 (can happen after gain > 1.0)
          --    Keep circular statistics consistent for averaging
          if v_cal_fp >= to_unsigned(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS) then
            fine_codes_fp(lane) <= v_cal_fp - to_unsigned(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS);
          else
            fine_codes_fp(lane) <= v_cal_fp;
          end if;
        else
          fine_codes_fp(lane) <= (others => '1');
        end if;
      end if;
    end process;
  end generate;

  -- ========================================================================
  -- Pipelined Lane Averaging (2-Stage Balanced Tree for Timing Closure @ 400 MHz)
  -- ========================================================================
  -- Stage A: Pairwise addition (lanes 0+1, 2+3) with register
  -- Stage B: Final sum + divide-by-4 with register
  -- Total: 2 cycles (cuts critical carry length by 1/2 compared to single-stage sum)

  p_avg_stage_a : process(clk_tdc)
    variable v_s01, v_s23 : unsigned(C_FINE_FRAC_BITS downto 0);
  begin
    if rising_edge(clk_tdc) then
      v_s01   := ('0' & fine_codes_fp(0)) + ('0' & fine_codes_fp(1));
      v_s23   := ('0' & fine_codes_fp(2)) + ('0' & fine_codes_fp(3));
      sum01_r <= v_s01;
      sum23_r <= v_s23;
    end if;
  end process;

  p_avg_stage_b : process(clk_tdc)
    variable v_ssum      : unsigned(C_FINE_FRAC_BITS + 1 downto 0);
    variable v_avg       : unsigned(C_FINE_FRAC_BITS - 1 downto 0);
    -- synthesis translate_off
    variable v_fine_seen : integer range 0 to 100000 := 0;
    -- synthesis translate_on
  begin
    if rising_edge(clk_tdc) then
      v_ssum      := ('0' & sum01_r) + ('0' & sum23_r);
      -- Divide by 4 (GC_TDL_LANES=4) → shift right 2
      v_avg       := resize(shift_right(v_ssum, 2), C_FINE_FRAC_BITS);
      fine_avg_fp <= v_avg;

      -- Debug disabled for performance
      -- synthesis translate_off
      -- if GC_SIM then
      --   if v_fine_seen < 100000 then
      --     v_fine_seen := v_fine_seen + 1;
      --   end if;
      --   if (v_fine_seen <= 5) or (v_fine_seen = 100) or (v_fine_seen = 500) or ((v_fine_seen mod 1000) = 0) then
      --     report "TDC_QUANTIZER Fine [" & integer'image(v_fine_seen) & "]: fine_avg_fp=" & integer'image(to_integer(v_avg)) &
      --            " (range: 0..65535, center=32768)";
      --   end if;
      -- end if;
      -- synthesis translate_on
    end if;
  end process;

  -- ========================================================================
  -- Start Timestamp Capture (coordinated banking with Stop path)
  -- ========================================================================
  p_start_timestamp : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if start_pulse = '1' then
        -- Signed math and auto-calibration will handle alignment
        start_coarse_hold <= coarse_counter_bin;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stop Timestamp Capture (simple direct counter)
  -- ========================================================================
  -- p_stop_timestamp: Capture coarse+fine timestamps when Stop event occurs
  -- Coarse captured immediately (direct counter read)
  p_stop_timestamp : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- Capture coarse on analog_stop_mark (direct counter read)
      if analog_stop_mark = '1' then
        stop_coarse_bin <= coarse_counter_bin;

        start_coarse_at_mark <= start_coarse_hold;
      end if;

      if analog_stop_mark_comp = '1' then
        stop_fine_fp <= fine_avg_fp;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Pipeline Delay Compensation
  -- ========================================================================
  p_pipe_delay : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      -- Pipeline start snapshot in lockstep with stop coarse
      start_coarse_pipe_d1 <= start_coarse_at_mark; -- Delay 1
      start_coarse_pipe_d2 <= start_coarse_pipe_d1; -- Delay 2
      start_coarse_pipe_d3 <= start_coarse_pipe_d2; -- Delay 3
      start_coarse_pipe_d4 <= start_coarse_pipe_d3; -- Delay 4
      start_coarse_pipe_d5 <= start_coarse_pipe_d4; -- Delay 5
      start_coarse_pipe_d6 <= start_coarse_pipe_d5; -- Delay 6
      start_coarse_pipe_d7 <= start_coarse_pipe_d6; -- Delay 7
      start_coarse_pipe_d8 <= start_coarse_pipe_d7; -- Delay 8

      -- Pipeline stop coarse (existing)
      stop_coarse_pipe_d1 <= stop_coarse_bin; -- Delay 1
      stop_coarse_pipe_d2 <= stop_coarse_pipe_d1; -- Delay 2
      stop_coarse_pipe_d3 <= stop_coarse_pipe_d2; -- Delay 3
      stop_coarse_pipe_d4 <= stop_coarse_pipe_d3; -- Delay 4
      stop_coarse_pipe_d5 <= stop_coarse_pipe_d4; -- Delay 5
      stop_coarse_pipe_d6 <= stop_coarse_pipe_d5; -- Delay 6
      stop_coarse_pipe_d7 <= stop_coarse_pipe_d6; -- Delay 7
      stop_coarse_pipe_d8 <= stop_coarse_pipe_d7; -- Delay 8

      analog_stop_mark_d1   <= analog_stop_mark; -- Delay 1
      analog_stop_mark_d2   <= analog_stop_mark_d1; -- Delay 2
      analog_stop_mark_d3   <= analog_stop_mark_d2; -- Delay 3
      analog_stop_mark_d4   <= analog_stop_mark_d3; -- Delay 4
      analog_stop_mark_d5   <= analog_stop_mark_d4; -- Delay 5
      analog_stop_mark_d6   <= analog_stop_mark_d5; -- Delay 6
      analog_stop_mark_d7   <= analog_stop_mark_d6; -- Delay 7
      analog_stop_mark_d8   <= analog_stop_mark_d7; -- Delay 8
      analog_stop_mark_pipe <= analog_stop_mark_d8; -- Delay 8 (for coarse delta computation)
      -- analog_stop_mark_comp moved to p_dcoarse_register for proper alignment with dcoarse_signed_at_comp

      if analog_stop_mark_pipe = '1' then
        if signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8) < -128 then
          -- Wraparound detected: add 256 to get correct positive difference
          dcoarse_signed_raw <= signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8) + 256;
          -- Debug enabled
          -- synthesis translate_off
          if GC_SIM then
            report "TDC_QUANTIZER Coarse: dcoarse_raw=" & integer'image(to_integer(signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8) + 256)) & " (wraparound, stop=" & integer'image(to_integer(unsigned(stop_coarse_pipe_d8))) & " start=" & integer'image(to_integer(unsigned(start_coarse_pipe_d8))) & ")";
          end if;
        -- synthesis translate_on
        else
          -- Normal case: use raw difference
          dcoarse_signed_raw <= signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8);
          -- Debug enabled
          -- synthesis translate_off
          if GC_SIM then
            report "TDC_QUANTIZER Coarse: dcoarse_raw=" & integer'image(to_integer(signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8))) & " (normal, stop=" & integer'image(to_integer(unsigned(stop_coarse_pipe_d8))) & " start=" & integer'image(to_integer(unsigned(start_coarse_pipe_d8))) & ")";
          end if;
          -- synthesis translate_on
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Auto-Bootstrap Bias Calibration (First 16 samples after reset)
  -- ========================================================================
  -- Accumulates histogram of dcoarse_signed_raw for first 16 samples
  -- Finds the mode (most common value) and uses it as coarse_bias
  -- This allows the system to self-calibrate to any board's actual delay
  -- without requiring manual tuning or MMIO access
  p_auto_calibration : process(clk_tdc)
    variable v_dcoarse_idx : integer range 0 to 255; -- Full 8-bit range for 256-bin histogram
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        calib_histogram        <= (others => (others => '0'));
        calib_sample_count     <= (others => '0');
        calib_done             <= '0';
        coarse_bias_calibrated <= (others => '0');
        calib_searching        <= '0';
        calib_search_idx       <= 0;
        calib_max_count        <= (others => '0');
        calib_mode_value       <= 0;
        calib_delay_cnt        <= 0;    -- Reset delay counter
        calib_ready            <= '0';  -- Reset ready flag
        calib_hist_idx_d1      <= 0;
        calib_hist_value_d1    <= (others => '0');
        calib_hist_idx_d2      <= 0;
        calib_hist_value_d2    <= (others => '0');
        calib_hist_update_d2   <= '0';
        calib_hist_read_d1     <= (others => '0');
        calib_search_idx_d1    <= 0;

      else
        -- Increment delay counter and set ready flag when it expires (breaks timing path)
        if calib_delay_cnt < C_CALIB_DELAY then
          calib_delay_cnt <= calib_delay_cnt + 1;
        elsif calib_ready = '0' then
          calib_ready <= '1';           -- Set flag once and hold (single-bit comparison, not counter compare)
          report "CALIB: Ready flag set after delay" severity note;
        end if;

        -- Calibration logic: only active when ready=1 and not done
        if calib_ready = '1' and calib_done = '0' then

          -- PHASE 1: Accumulate histogram for first 16 valid samples (2-stage pipeline for timing)
          -- Stage 1: Read histogram value and register it
          if analog_stop_mark_calib = '1' and calib_sample_count < 16 then
            -- Convert registered dcoarse to array index (0-255) for full 8-bit calibration
            -- Clamp to valid range to handle out-of-bounds values
            if dcoarse_for_calib < 0 then
              v_dcoarse_idx := 0;
            elsif dcoarse_for_calib > 255 then
              v_dcoarse_idx := 255;
            else
              v_dcoarse_idx := to_integer(dcoarse_for_calib);
            end if;

            -- Pipeline stage 1: Read histogram
            calib_hist_idx_d1   <= v_dcoarse_idx;
            calib_hist_value_d1 <= calib_histogram(v_dcoarse_idx);

            calib_sample_count <= calib_sample_count + 1;
          end if;

          -- Pipeline stage 2: Register read value
          calib_hist_idx_d2   <= calib_hist_idx_d1;
          calib_hist_value_d2 <= calib_hist_value_d1;
          if calib_sample_count > 0 and calib_sample_count <= 16 then
            calib_hist_update_d2 <= '1';
          else
            calib_hist_update_d2 <= '0';
          end if;

          -- Pipeline stage 3: Increment and write back
          if calib_hist_update_d2 = '1' and calib_hist_value_d2 < 15 then
            calib_histogram(calib_hist_idx_d2) <= calib_hist_value_d2 + 1;
          end if;

          -- PHASE 2: After 16 samples, wait two cycles for pipeline flush, then start search
          if calib_sample_count = 18 and calib_searching = '0' then
            -- Initialize search
            report "CALIB: Starting histogram search (16 samples collected)" severity note;
            calib_searching  <= '1';
            calib_search_idx <= 0;
            calib_max_count  <= (others => '0');
            calib_mode_value <= 0;
          end if;

          -- PHASE 3: Search for mode one bucket per cycle (2-stage pipeline for timing)
          if calib_searching = '1' then
            -- Stage 1: Read histogram bucket and register it (only for valid indices)
            if calib_search_idx < 256 then
              calib_hist_read_d1  <= calib_histogram(calib_search_idx);
              calib_search_idx_d1 <= calib_search_idx;
            end if;

            -- Stage 2: Compare and update (use pipelined values from previous cycle)
            -- This compares the value read in the PREVIOUS cycle
            if calib_search_idx > 0 and calib_hist_read_d1 > calib_max_count then
              calib_max_count  <= calib_hist_read_d1;
              calib_mode_value <= calib_search_idx_d1;
            end if;

            -- Move to next bucket or finish
            if calib_search_idx = 255 then
              -- One more cycle for pipeline (idx=256 for final comparison, don't read histogram)
              calib_search_idx <= 256;
            elsif calib_search_idx = 256 then
              -- Search complete after pipeline flush - lock in result
              coarse_bias_calibrated <= to_unsigned(calib_mode_value, 8); -- 8-bit bias
              calib_done             <= '1';
              calib_searching        <= '0';
              -- Debug output
              report "CALIBRATION COMPLETE: coarse_bias=" & integer'image(calib_mode_value) & " max_count=" & integer'image(to_integer(calib_max_count)) & " (will now use calibrated value instead of input coarse_bias)" severity note;
            else
              calib_search_idx <= calib_search_idx + 1;
            end if;
          end if;

        end if;                         -- calib_ready = '1' and calib_done = '0'
      end if;                           -- reset_tdc
    end if;                             -- rising_edge
  end process;

  g_tdac_pipeline_off : if false generate -- Time DAC disabled, tie pipeline to zero
    -- Hard-tie pipeline to zero so every bit has a real driver
    p_time_dac_pipe_tieoff : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        time_dac_cmd_ext_d1 <= (others => '0');
        time_dac_cmd_ext_d2 <= (others => '0');
        time_dac_cmd_ext_d3 <= (others => '0');
        time_dac_cmd_ext_d4 <= (others => '0');
        time_dac_cmd_ext_d5 <= (others => '0');
        time_dac_cmd_ext_d6 <= (others => '0');
        time_dac_cmd_ext_d7 <= (others => '0');
        time_dac_cmd_ext_d8 <= (others => '0');
        time_dac_cmd_ext_d9 <= (others => '0');
      end if;
    end process;
  end generate;

  -- ========================================================================
  -- Stage I: Dual-Lobe Bias Selection (Split into 1a and 1b for Synthesis)
  -- ========================================================================
  -- Mux between input coarse_bias and auto-calibrated value
  coarse_bias_effective <= coarse_bias_calibrated when (calib_done = '1') else coarse_bias;

  -- Debug: report when effective bias changes
  p_bias_debug : process(clk_tdc)
    variable v_bias_prev : unsigned(7 downto 0) := (others => '0');
  begin
    if rising_edge(clk_tdc) then
      if coarse_bias_effective /= v_bias_prev then
        report "BIAS CHANGE: coarse_bias_effective=" & integer'image(to_integer(coarse_bias_effective)) & " (calib_done=" & std_logic'image(calib_done)(2) & " input_bias=" & integer'image(to_integer(coarse_bias)) & " calib_bias=" & integer'image(to_integer(coarse_bias_calibrated)) & ")" severity note;
        v_bias_prev := coarse_bias_effective;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Continuous register of dcoarse_signed_raw to break timing path
  -- ========================================================================
  -- This register captures dcoarse_signed_raw on EVERY cycle, so that when
  -- analog_stop_mark_comp arrives, we can use the already-registered value
  -- instead of the raw combinational dcoarse_signed_raw.
  -- 
  -- CRITICAL: Due to VHDL signal update semantics (delta delay), when both
  -- dcoarse_signed_raw and dcoarse_signed_at_comp are assigned on the same
  -- clock edge, dcoarse_signed_at_comp captures the OLD value of dcoarse_signed_raw.
  -- To align properly, analog_stop_mark_comp is delayed by one extra cycle
  -- relative to analog_stop_mark_pipe, matching the dcoarse_signed_at_comp delay.

  p_dcoarse_register : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      dcoarse_signed_at_comp <= dcoarse_signed_raw;
      dcoarse_for_calib      <= dcoarse_signed_raw; -- Separate register for calibration timing
      analog_stop_mark_calib <= analog_stop_mark_pipe; -- Delay stop mark for calibration (breaks timing)

      -- Add one extra cycle delay to analog_stop_mark_comp to align with registered dcoarse
      analog_stop_mark_comp_pre <= analog_stop_mark_pipe; -- First delay (same as dcoarse registration)
      analog_stop_mark_comp     <= analog_stop_mark_comp_pre; -- Second delay (aligns with dcoarse_signed_at_comp)
    end if;
  end process;

  -- ========================================================================
  -- Stage-1a: Capture Inputs and Compute Bias Adjustment Candidates
  -- ========================================================================
  -- At comp edge, freeze bias/fine and compute three candidate adjusted coarse values:
  --   adj0_s1: raw - bias0       (primary lobe)
  --   adjm_s1: raw - (bias0-3)   (lower lobe, -3 cycles)
  --   adjp_s1: raw - (bias0+3)   (upper lobe, +3 cycles)
  p_stage1a : process(clk_tdc)
    variable v_b0_s : signed(GC_COARSE_BITS downto 0);
    variable v_bm_s : signed(GC_COARSE_BITS downto 0);
    variable v_bp_s : signed(GC_COARSE_BITS downto 0);
  begin
    if rising_edge(clk_tdc) then
      s1a_valid <= '0';                 -- Default: no new data

      if analog_stop_mark_comp = '1' then
        -- Freeze bias and fine at comp edge
        bias_at_comp <= coarse_bias_effective;
        fine_at_comp <= fine_avg_fp;

        -- Build three bias candidates: {bias0, bias0-3, bias0+3}
        v_b0_s := signed(resize(coarse_bias_effective, GC_COARSE_BITS + 1));
        v_bm_s := v_b0_s - to_signed(3, v_b0_s'length);
        v_bp_s := v_b0_s + to_signed(3, v_b0_s'length);

        -- Compute three adjusted coarse candidates from REGISTERED dcoarse_signed_at_comp
        adj0_s1 <= dcoarse_signed_at_comp - v_b0_s; -- Primary lobe
        adjm_s1 <= dcoarse_signed_at_comp - v_bm_s; -- Lower lobe (bias-3)
        adjp_s1 <= dcoarse_signed_at_comp - v_bp_s; -- Upper lobe (bias+3)

        -- synthesis translate_off
        if GC_SIM then
          report "TDC_STAGE1A: dcoarse=" & integer'image(to_integer(dcoarse_signed_at_comp)) & " bias=" & integer'image(to_integer(coarse_bias_effective)) & " adj0=" & integer'image(to_integer(dcoarse_signed_at_comp - v_b0_s)) & " at " & time'image(now);
        end if;
        -- synthesis translate_on

        s1a_valid <= '1';               -- Signal to Stage-1b: fresh data available next cycle
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stage-1b: Magnitude Calculation
  -- ========================================================================
  p_stage1b : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if s1a_valid = '1' then
        -- Compute all three magnitudes in parallel
        mag0_s1b <= abs_s9(adj0_s1);
        magm_s1b <= abs_s9(adjm_s1);
        magp_s1b <= abs_s9(adjp_s1);

        -- Pass through the signed values for next stage
        adj0_s1b <= adj0_s1;
        adjm_s1b <= adjm_s1;
        adjp_s1b <= adjp_s1;

        -- Pass through fine value for tie-breaking in next stage
        fine_s1b <= ('0' & fine_at_comp);

        s1b_valid <= '1';
      else
        s1b_valid <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stage-1c: Min-of-Three Selection, Window Check, and dfine Construction
  -- ========================================================================
  -- This process implements the core of the dual-lobe bias selection fix.
  -- It selects the minimum-magnitude adjustment from {adj0, adjm, adjp},
  -- computes dfine based on the selected candidate, and validates the window.
  p_stage1c : process(clk_tdc)
    variable v_d_used     : signed(GC_COARSE_BITS downto 0);
    variable v_mag_winner : unsigned(GC_COARSE_BITS downto 0);
  begin
    if rising_edge(clk_tdc) then
      -- Stage-1c fires when Stage-1b fired LAST CYCLE (s1b_valid=1 delayed by 1 clk)
      if s1b_valid = '1' then
        v_d_used     := adj0_s1b;
        v_mag_winner := mag0_s1b;

        -- Compare magnitudes with polarity-aware tie-breaking
        -- Step 1: Compare adj0 vs adjm
        if magm_s1b < mag0_s1b then
          v_d_used     := adjm_s1b;
          v_mag_winner := magm_s1b;
        elsif magm_s1b = mag0_s1b then
          -- Tie: prefer candidate that passes fine polarity rule
          if (adjm_s1b = to_signed(1, adjm_s1b'length) and fine_s1b < to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) or (adjm_s1b = to_signed(-1, adjm_s1b'length) and fine_s1b >= to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) then
            v_d_used     := adjm_s1b;
            v_mag_winner := magm_s1b;
          end if;
        end if;

        -- Step 2: Compare current winner vs adjp (use pre-computed magnitude)
        if magp_s1b < v_mag_winner then
          v_d_used := adjp_s1b;
        elsif magp_s1b = v_mag_winner then
          -- Tie: prefer candidate that passes fine polarity rule
          if (adjp_s1b = to_signed(1, adjp_s1b'length) and fine_s1b < to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) or (adjp_s1b = to_signed(-1, adjp_s1b'length) and fine_s1b >= to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) then
            v_d_used := adjp_s1b;
          end if;
        end if;

        -- Register chosen adjusted coarse for Stage-1d
        d_used_s1c <= v_d_used;
        fine_s1c   <= fine_s1b;         -- Pass fine through for dfine construction

        -- Signal to Stage-1d: fresh data available next cycle
        s1c_valid <= '1';

      else
        -- No valid data from Stage-1b -> clear pipeline
        s1c_valid <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stage-1d: dfine Construction + Window Check
  -- ========================================================================
  p_stage1d : process(clk_tdc)
    variable v_dfine_ext : signed(C_FINE_FRAC_BITS + 1 downto 0);
    variable v_win_ok    : std_logic;
  begin
    if rising_edge(clk_tdc) then
      if s1c_valid = '1' then
        -- ======================================================================
        -- Compute dfine_ext Based on Chosen Adjusted Coarse
        -- ======================================================================
        if d_used_s1c = to_signed(-1, GC_COARSE_BITS + 1) then
          -- Previous cycle: stop_fine + 1.0
          v_dfine_ext := signed('0' & fine_s1c) + to_signed(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 2);
        elsif d_used_s1c = to_signed(0, GC_COARSE_BITS + 1) then
          -- Same cycle: fine time in [0, 1)
          v_dfine_ext := signed('0' & fine_s1c);
        elsif d_used_s1c = to_signed(1, GC_COARSE_BITS + 1) then
          -- Next cycle: stop_fine - 1.0
          v_dfine_ext := signed('0' & fine_s1c) - to_signed(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 2);
        else
          -- Out of window: formal value
          v_dfine_ext := signed('0' & fine_s1c);
        end if;

        dfine_ext_s1b <= v_dfine_ext;   -- Register for Stage-1e
        dfine_ext_s2  <= v_dfine_ext;   -- Also update legacy signal for Stage-II

        v_win_ok := '0';                -- Default to fail

        if d_used_s1c = to_signed(0, GC_COARSE_BITS + 1) then
          v_win_ok := '1';              -- Always valid (dfine >= 0 by construction)
        elsif d_used_s1c = to_signed(1, GC_COARSE_BITS + 1) then
          -- Valid only if dfine < 0
          if v_dfine_ext < to_signed(0, v_dfine_ext'length) then
            v_win_ok := '1';
          end if;
        elsif d_used_s1c = to_signed(-1, GC_COARSE_BITS + 1) then
          -- Valid only if dfine > 0
          if v_dfine_ext > to_signed(0, v_dfine_ext'length) then
            v_win_ok := '1';
          end if;
        end if;

        -- Register chosen coarse and debug output
        d_used_s1        <= d_used_s1c;
        dcoarse_adjusted <= d_used_s1c; -- Debug output

        -- Capture chosen coarse and fine into pipeline registers for window-check process
        s_d_used_stage  <= d_used_s1c;
        s_dfine_u_stage <= fine_s1c;
        -- mark pre-valid; p_win_check will use s_vld_pre to drive s_vld_pipe(0)
        s_vld_pre       <= '1';

        -- Signal to Stage-1e: fresh data available next cycle
        s1d_valid <= '1';

      else
        -- No valid data from Stage-1c → clear pipeline
        s_vld_pre <= '0';
        s1d_valid <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stage-1e: 24-Bit Delta Computation
  -- ========================================================================
  p_stage1e : process(clk_tdc)
    variable v_delta_meas : signed(C_TIME_FP_BITS - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if s1d_valid = '1' then
        -- ======================================================================
        -- 24-Bit Delta Computation
        -- ======================================================================
        v_delta_meas := shift_left(resize(d_used_s1, C_TIME_FP_BITS), C_FINE_FRAC_BITS) + resize(dfine_ext_s1b, C_TIME_FP_BITS);
        s_delta_meas <= v_delta_meas;   -- Register for TDAC subtraction stage
      end if;
    end if;
  end process;

  -- ==============================================================================
  -- TDAC Subtraction Process
  -- ==============================================================================
  p_tdac_subtract : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      s_delta_meas_adj <= s_delta_meas; -- Bypass
    end if;
  end process p_tdac_subtract;

  -- ======================================================================
  -- Window Check Process (registered) - computes v_win_ok from registered
  -- chosen coarse/fine values to break long path from adjm_s1 -> s_win_ok
  -- ======================================================================
  p_win_check : process(clk_tdc)
    variable v_win_ok_local    : std_logic := '0';
    variable v_dfine_ext_local : signed(C_FINE_FRAC_BITS + 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        s_win_ok1        <= '0';
        s_win_ok_pipe(0) <= '0';
        s_vld_pipe(0)    <= '0';
        win_fail_toggle  <= '0';
      else
        -- Default clear
        v_win_ok_local := '0';

        -- Reconstruct v_dfine_ext from registered unsigned fine (kept for reference)
        v_dfine_ext_local := signed('0' & s_dfine_u_stage);
        v_win_ok_local    := '1';       -- Always pass (multi-bit mode)

        -- Drive registered outputs; align valid with s_vld_pre (one-cycle delayed)
        s_win_ok1        <= v_win_ok_local;
        s_win_ok_pipe(0) <= v_win_ok_local;
        s_vld_pipe(0)    <= s_vld_pre;

        -- Window failure CDC toggle (registered)
        if v_win_ok_local = '0' then
          win_fail_toggle <= not win_fail_toggle;
        end if;
      end if;
    end if;
  end process p_win_check;

  -- ========================================================================
  -- Stage II: Error Computation, Centering, and Range Check
  -- ========================================================================
  -- This stage computes the centered error signal and validates it's within
  -- the expected range for ΣΔ operation.
  p_interval_stage2 : process(clk_tdc)
    variable v_centered : signed(C_TIME_FP_BITS - 1 downto 0);
    -- synthesis translate_off
    variable v_s2_seen  : integer range 0 to 100000 := 0; -- Warm-up counter for consistency check (saturates at max)
    -- synthesis translate_on
    constant C_TOL      : integer                   := 8192; -- Tolerance: +-(1/8) coarse period for lock check (simulation only)
  begin
    if rising_edge(clk_tdc) then
      -- Stage II: Check bit 0 (data from Stage-1b, after TDAC subtraction in same cycle)
      if s_vld_pipe(0) = '1' then
        v_centered := s_delta_meas_adj - C_HALF_COARSE_FP;
        s_centered <= v_centered;

        -- synthesis translate_off
        if GC_SIM then
          -- Debug: Report centering details every 100 samples
          v_s2_seen := v_s2_seen + 1;
          if (v_s2_seen <= 5) or (v_s2_seen = 100) or (v_s2_seen = 500) or ((v_s2_seen mod 1000) = 0) then
            report "TDC_QUANTIZER Stage2 [" & integer'image(v_s2_seen) & "]: " & "delta_meas_adj=" & integer'image(to_integer(s_delta_meas_adj)) & " centered=" & integer'image(to_integer(v_centered)) & " (half_coarse=32768)";
          end if;
        end if;
        -- synthesis translate_on

        -- Shift pipeline valids forward one more cycle for Stage III
        s_vld_pipe(1)    <= '1';
        s_win_ok_pipe(1) <= s_win_ok_pipe(0);
        if s_win_ok_pipe(0) = '0' then
          -- Window check failed -> immediate overflow (don't wait for range check)
          s_ovf2 <= '1';
        else
          -- Window passed, check range on next cycle in Stage III
          -- For now, clear overflow (Stage III will re-evaluate)
          s_ovf2 <= '0';
        end if;
      else
        -- No valid data -> clear pipeline bits
        s_vld_pipe(1)    <= '0';
        s_win_ok_pipe(1) <= '0';
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stage III: Overflow Check, Rounding, Resize, Polarity, and Buffer Update
  -- Split into sub-stages for timing closure at 400MHz (2.5ns period)
  -- ========================================================================
  p_interval_stage3 : process(clk_tdc)
    constant C_ROUND_BIT   : integer := C_TIME_FP_BITS - GC_OUTPUT_WIDTH - 1;
    constant C_ROUND_CONST : signed(C_TIME_FP_BITS - 1 downto 0) := to_signed(2 ** C_ROUND_BIT, C_TIME_FP_BITS);
    -- HARDWARE DEBUG: Increase tolerance to ±100 coarse cycles (was ±20)
    constant C_OVF_TOL     : integer := 100 * (2 ** C_FINE_FRAC_BITS); -- 6553600: ±100 coarse cycles for debugging
    variable v_ovf_check   : boolean;
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        -- Reset ONLY sticky control flags
        lost_sample_latch      <= '0';
        tdac_enable_after_lock <= '0';
        overflow_buf           <= '0';
        sample_ready           <= '0';
        tdc_valid_i            <= '0';
        tdc_result_buf         <= (others => '0');
        overflow_i             <= '0';
        tdc_result             <= (others => '0');
        s_round_sum            <= (others => '0');
        s_shifted              <= (others => '0');
        s_rounded              <= (others => '0');
        s_ovf_a                <= '0';
        s_ovf_b                <= '0';
        s_ovf_pipe             <= '0';
      else
        -- Default: no new output (unless start_pulse triggers delivery)
        tdc_valid_i <= '0';

        -- ================================================================
        -- Stage III-A: Overflow check + Rounding addition (cycle 1)
        -- ================================================================
        if s_vld_pipe(1) = '1' then
          -- Overflow check: compare magnitude against tolerance
          v_ovf_check := (s_win_ok_pipe(1) = '0') or (s_centered > C_OVF_TOL) or (s_centered < -C_OVF_TOL);
          s_ovf_a     <= '1' when v_ovf_check else '0';

          -- Rounding: just do the addition this cycle (shift in next cycle)
          if C_ROUND_BIT >= 0 then
            s_round_sum <= s_centered + C_ROUND_CONST;
          else
            s_round_sum <= s_centered;
          end if;

          s_vld_pipe(2) <= '1';
        else
          s_vld_pipe(2) <= '0';
        end if;

        -- ================================================================
        -- Stage III-B: Right-shift and resize (cycle 2)
        -- ================================================================
        if s_vld_pipe(2) = '1' then
          s_ovf_b   <= s_ovf_a;
          s_shifted <= resize(shift_right(s_round_sum, C_TIME_FP_BITS - GC_OUTPUT_WIDTH), GC_OUTPUT_WIDTH);
          s_vld_pipe(3) <= '1';
        else
          s_vld_pipe(3) <= '0';
        end if;

        -- ================================================================
        -- Stage III-C: Polarity inversion (cycle 3)
        -- ================================================================
        if s_vld_pipe(3) = '1' then
          s_ovf_pipe <= s_ovf_b;
          if invert_polarity = '1' then
            s_rounded <= -s_shifted;
          else
            s_rounded <= s_shifted;
          end if;
          s_vld_pipe(4) <= '1';
        else
          s_vld_pipe(4) <= '0';
        end if;

        -- ================================================================
        -- Stage III-D: Buffer update (cycle 4)
        -- ================================================================
        if s_vld_pipe(4) = '1' then
          if s_ovf_pipe = '1' then
            -- Overflow: don't deliver sample
            overflow_buf      <= '1';
            sample_ready      <= '0';
            lost_sample_latch <= '1';
          else
            -- GOOD SAMPLE: Auto-clear lost_sample_latch
            overflow_buf      <= '0';
            lost_sample_latch <= '0';
            tdc_result_buf    <= s_rounded;
            sample_ready      <= '1';
          end if;
        end if;

        -- Start-synchronous delivery: output buffered sample on next start_pulse
        -- This ensures loop filter updates at a deterministic start rate
        if start_pulse = '1' then
          if sample_ready = '1' then
            -- Deliver buffered sample
            tdc_result             <= tdc_result_buf;
            overflow_i             <= overflow_buf;
            tdc_valid_i            <= '1';
            sample_ready           <= '0';
            tdac_enable_after_lock <= '0';
          else
            -- No sample ready this start: normal pipeline timing
            tdc_valid_i <= '0';
          end if;
        end if;
      end if;
    end if;
  end process;

  tdc_out     <= tdc_result;
  tdc_valid   <= tdc_valid_i;
  overflow    <= overflow_i;
  lost_sample <= lost_sample_latch;

  -- ========================================================================
  -- TDL Centering Calibration Outputs
  -- ========================================================================
  -- Expose fine_at_comp (captured fine_avg_fp at comparator edge) for TDL centering calibration
  fine_at_comp_out <= fine_at_comp;
  fine_valid_out   <= s1a_valid;  -- Pulses when fine_at_comp is updated (same as Stage-1a valid)

  -- ========================================================================
  -- Window Failure CDC (clk_tdc -> clk_sys)
  -- ========================================================================
  -- 3-FF synchronizer for toggle-based handshake
  p_win_fail_cdc : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      -- 3-FF scalar synchronizer chain
      win_fail_toggle_sync0 <= win_fail_toggle;
      win_fail_toggle_sync1 <= win_fail_toggle_sync0;
      win_fail_toggle_sync2 <= win_fail_toggle_sync1;
    end if;
  end process;

end architecture;
