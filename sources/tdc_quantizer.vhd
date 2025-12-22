library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.clk_rst_pkg.all;

entity tdc_quantizer is
  generic(
    GC_TDL_LANES    : positive range 1 to 16 := 4;
    GC_TDL_LENGTH   : positive               := 128;
    GC_COARSE_BITS  : positive               := 8;
    GC_OUTPUT_WIDTH : positive               := 16
  );
  port(
    clk_sys          : in  std_logic;
    clk_tdc          : in  std_logic;
    reset            : in  std_logic;
    analog_in        : in  std_logic;
    ref_phases       : in  std_logic_vector(0 downto 0);
    invert_polarity  : in  std_logic;
    coarse_bias      : in  unsigned(7 downto 0);
    tdc_out          : out signed(GC_OUTPUT_WIDTH - 1 downto 0);
    tdc_valid        : out std_logic;
    overflow         : out std_logic;
    lost_sample      : out std_logic;
    fine_at_comp_out : out unsigned(15 downto 0);
    fine_valid_out   : out std_logic
  );
end entity;

architecture rtl of tdc_quantizer is

  attribute KEEP : boolean;

  -- Helper Functions
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

  constant C_TDL_BITS       : natural                             := clog2(GC_TDL_LENGTH + 1);
  constant C_FINE_FRAC_BITS : natural                             := 16;
  constant C_TIME_FP_BITS   : natural                             := GC_COARSE_BITS + C_FINE_FRAC_BITS;
  constant C_HALF_COARSE_FP : signed(C_TIME_FP_BITS - 1 downto 0) := to_signed(2 ** (C_FINE_FRAC_BITS - 1), C_TIME_FP_BITS);

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

  type T_TDL_THERM        is array (0 to GC_TDL_LENGTH - 1) of std_logic;
  type T_TDL_THERM_ARRAY  is array (0 to GC_TDL_LANES - 1) of T_TDL_THERM;
  type T_FINE_CODE_ARRAY  is array (0 to GC_TDL_LANES - 1) of unsigned(C_TDL_BITS - 1 downto 0);
  type T_FINE_FP_ARRAY    is array (0 to GC_TDL_LANES - 1) of unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  type T_CAL_LUT          is array (0 to GC_TDL_LENGTH) of unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  type T_LANE_SCALE_ARRAY is array (0 to GC_TDL_LANES - 1) of unsigned(15 downto 0);

  constant C_POPCOUNT_GROUPS    : positive := 16;
  constant C_GROUP_SIZE         : positive := GC_TDL_LENGTH / C_POPCOUNT_GROUPS;
  type     T_GROUP_COUNT_ARRAY  is array (0 to C_POPCOUNT_GROUPS - 1) of unsigned(3 downto 0);
  type     T_LANE_GROUP_COUNTS  is array (0 to GC_TDL_LANES - 1) of T_GROUP_COUNT_ARRAY;
  constant C_PARTIAL_GROUPS     : positive := 4;
  constant C_GROUPS_PER_PARTIAL : positive := C_POPCOUNT_GROUPS / C_PARTIAL_GROUPS;
  type     T_PARTIAL_SUM_ARRAY  is array (0 to C_PARTIAL_GROUPS - 1) of unsigned(C_TDL_BITS - 1 downto 0);
  type     T_LANE_PARTIAL_SUMS  is array (0 to GC_TDL_LANES - 1) of T_PARTIAL_SUM_ARRAY;

  signal reset_tdc       : std_logic;
  signal analog_crossing : std_logic;

  signal ref_phase0_prev      : std_logic            := '0';
  signal start_pulse          : std_logic            := '0';
  signal armed                : std_logic            := '0';
  signal analog_stop_mark     : std_logic            := '0';
  signal inhibit_just_ended   : std_logic            := '0';
  signal edge_inhibit         : unsigned(3 downto 0) := (others => '0');
  signal level_at_start_pulse : std_logic            := '0';
  signal stop_level_at_start  : std_logic            := '0';

  signal analog_sync : std_logic_vector(2 downto 0) := (others => '0');

  signal tdl_chains       : T_TDL_THERM_ARRAY;
  signal tdl_bank_current : T_TDL_THERM_ARRAY;
  signal tdl_bank_prev    : T_TDL_THERM_ARRAY;
  signal tdl_bank_prev_q  : T_TDL_THERM_ARRAY;
  signal tdl_captured     : T_TDL_THERM_ARRAY;

  signal coarse_counter_bin : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');
  signal start_coarse_hold  : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');
  signal stop_coarse_bin    : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');
  signal start_coarse_at_mark                                                                   : unsigned(GC_COARSE_BITS - 1 downto 0) := (others => '0');
  signal start_coarse_pipe_d1, start_coarse_pipe_d2, start_coarse_pipe_d3, start_coarse_pipe_d4 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_coarse_pipe_d5, start_coarse_pipe_d6, start_coarse_pipe_d7, start_coarse_pipe_d8 : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d1, stop_coarse_pipe_d2, stop_coarse_pipe_d3, stop_coarse_pipe_d4     : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_coarse_pipe_d5, stop_coarse_pipe_d6, stop_coarse_pipe_d7, stop_coarse_pipe_d8     : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal analog_stop_mark_d1, analog_stop_mark_d2, analog_stop_mark_d3, analog_stop_mark_d4     : std_logic                             := '0';
  signal analog_stop_mark_d5, analog_stop_mark_d6, analog_stop_mark_d7, analog_stop_mark_d8     : std_logic                             := '0';
  signal analog_stop_mark_pipe, analog_stop_mark_comp_pre, analog_stop_mark_comp                : std_logic                             := '0';

  -- Time DAC and interval signals (currently disabled)
  signal s_delta_meas       : signed(C_TIME_FP_BITS - 1 downto 0) := (others => '0');
  signal s_delta_meas_adj   : signed(C_TIME_FP_BITS - 1 downto 0) := (others => '0');
  signal dcoarse_signed_raw : signed(GC_COARSE_BITS downto 0);

  type   T_CALIB_HISTOGRAM      is array (0 to 255) of unsigned(3 downto 0);
  signal calib_histogram        : T_CALIB_HISTOGRAM               := (others => (others => '0'));
  signal calib_sample_count     : unsigned(4 downto 0)            := (others => '0');
  signal calib_done             : std_logic                       := '0';
  signal coarse_bias_calibrated : unsigned(7 downto 0)            := (others => '0');
  signal coarse_bias_effective  : unsigned(7 downto 0);
  signal dcoarse_for_calib      : signed(GC_COARSE_BITS downto 0) := (others => '0');
  signal analog_stop_mark_calib : std_logic                       := '0';

  -- Startup delay counter for calibration
  -- Must wait for tdc_adc_top's boot dither + TB comparator to enter TDC mode (C_SWEEP_END_TIME=70us in TB)
  -- Calibration must happen AFTER comparator is generating realistic TDC timing edges
  constant C_CALIB_DELAY   : natural                          := 40000; -- Wait 40000 cycles (~100us @ 400MHz) for closed-loop TDC mode
  signal   calib_delay_cnt : integer range 0 to C_CALIB_DELAY := 0;
  signal   calib_ready     : std_logic                        := '0'; -- Single-bit flag: '1' when delay expires (breaks timing path)

  -- Multi-cycle mode-finding state machine (breaks timing path)
  signal calib_search_idx : integer range 0 to 256 := 0; -- Current bucket being checked (0-255 + 256 for pipeline flush)
  signal calib_max_count  : unsigned(3 downto 0)   := (others => '0'); -- Running max count
  signal calib_mode_value : integer range 0 to 255 := 0; -- Running mode (bucket with max count)
  signal calib_searching  : std_logic              := '0'; -- '1' when searching for mode

  -- Pipeline registers to break histogram read-modify-write timing path (4-stage pipeline for timing)
  signal calib_hist_idx_d1    : integer range 0 to 255 := 0; -- Stage 1: Captured index
  signal calib_hist_value_d1  : unsigned(3 downto 0)   := (others => '0'); -- Stage 1: Read value from histogram
  signal calib_hist_idx_d2    : integer range 0 to 255 := 0; -- Stage 2: Index
  signal calib_hist_value_d2  : unsigned(3 downto 0)   := (others => '0'); -- Stage 2: Value
  signal calib_hist_update_d2 : std_logic              := '0'; -- Stage 2: Update enable
  signal calib_hist_idx_d3    : integer range 0 to 255 := 0; -- Stage 3: Index
  signal calib_hist_value_d3  : unsigned(3 downto 0)   := (others => '0'); -- Stage 3: Incremented value
  signal calib_hist_update_d3 : std_logic              := '0'; -- Stage 3: Update enable

  -- Pipeline registers to break search comparison timing path
  signal calib_hist_read_d1   : unsigned(3 downto 0)   := (others => '0'); -- Histogram value read for comparison
  signal calib_search_idx_d1  : integer range 0 to 255 := 0;
  signal calib_hist_read_addr : integer range 0 to 255 := 0; -- Address for histogram read during search

  -- Pipeline/register helpers for window check
  signal s_vld_pre : std_logic := '0';

  signal fine_codes_raw : T_FINE_CODE_ARRAY;
  signal fine_codes_fp  : T_FINE_FP_ARRAY;
  signal lane_offsets   : T_FINE_FP_ARRAY;
  signal lane_scales    : T_LANE_SCALE_ARRAY;
  signal fine_avg_fp    : unsigned(C_FINE_FRAC_BITS - 1 downto 0);

  signal sum01_r : unsigned(C_FINE_FRAC_BITS downto 0);
  signal sum23_r : unsigned(C_FINE_FRAC_BITS downto 0);

  signal tdl_captured_reg : T_TDL_THERM_ARRAY;
  signal group_counts     : T_LANE_GROUP_COUNTS;
  signal partial_sums     : T_LANE_PARTIAL_SUMS;

  -- Interval computation pipeline
  signal s_win_ok_pipe : std_logic_vector(2 downto 0)         := (others => '0');
  signal s_centered    : signed(C_TIME_FP_BITS - 1 downto 0)  := (others => '0');
  signal s_vld_pipe    : std_logic_vector(5 downto 0)         := (others => '0');
  signal s_round_sum   : signed(C_TIME_FP_BITS - 1 downto 0)  := (others => '0');
  signal s_ovf_a       : std_logic                            := '0';
  signal s_shifted     : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal s_ovf_b       : std_logic                            := '0';
  signal s_rounded     : signed(GC_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  signal s_ovf_pipe    : std_logic                            := '0';

  signal dcoarse_signed_at_comp : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal fine_at_comp           : unsigned(C_FINE_FRAC_BITS - 1 downto 0) := (others => '0');
  signal adj0_s1                : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal adjm_s1                : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal adjp_s1                : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal s1a_valid              : std_logic                               := '0';
  signal mag0_s1b               : unsigned(GC_COARSE_BITS downto 0)       := (others => '0');
  signal magm_s1b               : unsigned(GC_COARSE_BITS downto 0)       := (others => '0');
  signal magp_s1b               : unsigned(GC_COARSE_BITS downto 0)       := (others => '0');
  signal adj0_s1b               : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal adjm_s1b               : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal adjp_s1b               : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal fine_s1b               : unsigned(C_FINE_FRAC_BITS downto 0)     := (others => '0');
  signal s1b_valid              : std_logic                               := '0';
  signal d_used_s1c             : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal fine_s1c               : unsigned(C_FINE_FRAC_BITS downto 0)     := (others => '0');
  signal s1c_valid              : std_logic                               := '0';
  signal d_used_s1              : signed(GC_COARSE_BITS downto 0)         := (others => '0');
  signal dfine_ext_s1b          : signed(C_FINE_FRAC_BITS + 1 downto 0)   := (others => '0');
  signal s1d_valid              : std_logic                               := '0';

  signal tdc_result_buf    : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  signal overflow_buf      : std_logic := '0';
  signal sample_ready      : std_logic := '0';
  signal tdc_result        : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  signal tdc_valid_i       : std_logic := '0';
  signal overflow_i        : std_logic := '0';
  signal lost_sample_latch : std_logic := '0';

  signal win_fail_toggle : std_logic := '0';

  signal cal_lut : T_CAL_LUT;
begin

  i_reset_sync : entity work.reset_synchronizer
    generic map(
      GC_ACTIVE_LOW => false
    )
    port map(
      clk       => clk_tdc,
      async_rst => reset,
      sync_rst  => reset_tdc
    );

  analog_crossing <= analog_in;

  p_coarse_counter : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      coarse_counter_bin <= coarse_counter_bin + 1;
    end if;
  end process;

  p_start_edge_detect : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        ref_phase0_prev <= '0';
        start_pulse     <= '0';
      else
        ref_phase0_prev <= ref_phases(0);
        if ref_phases(0) = '1' and ref_phase0_prev = '0' then
          start_pulse <= '1';
        else
          start_pulse <= '0';
        end if;
      end if;
    end if;
  end process;

  p_arm_control : process(clk_tdc)
    variable v_start_count : integer range 0 to 1000000 := 0;
  begin
    if rising_edge(clk_tdc) then
      if start_pulse = '1' then
        armed         <= '1';
        v_start_count := v_start_count + 1;
      elsif (analog_stop_mark = '1' and armed = '1') then
        armed <= '0';
      end if;
    end if;
  end process;

  p_stop_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      analog_sync <= analog_sync(1 downto 0) & analog_crossing;

      inhibit_just_ended <= '0';

      if start_pulse = '1' then
        edge_inhibit         <= to_unsigned(2, edge_inhibit'length);
        level_at_start_pulse <= analog_sync(2);
      elsif edge_inhibit > to_unsigned(0, edge_inhibit'length) then
        edge_inhibit <= edge_inhibit - 1;
        if edge_inhibit = to_unsigned(1, edge_inhibit'length) then
          inhibit_just_ended <= '1';
        end if;
      end if;

      if inhibit_just_ended = '1' then
        if analog_sync(2) /= level_at_start_pulse then
          analog_stop_mark <= '1';
          edge_inhibit     <= to_unsigned(2, edge_inhibit'length);
        else
          stop_level_at_start <= analog_sync(2);
        end if;
      end if;

      if (analog_sync(2) /= stop_level_at_start) and (armed = '1') and (edge_inhibit = to_unsigned(0, edge_inhibit'length)) and (inhibit_just_ended = '0') then
        analog_stop_mark <= '1';
        edge_inhibit     <= to_unsigned(2, edge_inhibit'length);
      elsif inhibit_just_ended = '0' then
        analog_stop_mark <= '0';
      end if;
    end if;
  end process;

  g_tdl_lanes : for lane in 0 to GC_TDL_LANES - 1 generate
    signal    tdl_delay_chain         : std_logic_vector(0 to GC_TDL_LENGTH);
    attribute KEEP of tdl_delay_chain : signal is true;
  begin
    tdl_delay_chain(0) <= analog_crossing;

    g_taps : for tap in 0 to GC_TDL_LENGTH - 1 generate
      tdl_delay_chain(tap + 1) <= tdl_delay_chain(tap);

      tdl_chains(lane)(tap) <= tdl_delay_chain(tap);
    end generate;

    p_tdl_sample : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        tdl_bank_prev(lane)    <= tdl_bank_current(lane);
        tdl_bank_current(lane) <= tdl_chains(lane);
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

    p_tdl_register : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        tdl_captured_reg(lane) <= tdl_captured(lane);
      end if;
    end process;

  end generate;

  g_encoder : for lane in 0 to GC_TDL_LANES - 1 generate

    -- Intermediate signals for pipelined encoder
    signal therm_filt_s   : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal therm_filt_reg : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal therm_mono_s   : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal therm_mono_reg : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);

    signal group_clamp     : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal group_any       : std_logic_vector(C_POPCOUNT_GROUPS - 1 downto 0);
    signal group_clamp_reg : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
    signal group_any_reg   : std_logic_vector(C_POPCOUNT_GROUPS - 1 downto 0);
    signal group_prefix_on : std_logic_vector(C_POPCOUNT_GROUPS - 1 downto 0);
    signal prefix_half_a   : std_logic_vector(7 downto 0);
    signal prefix_half_a_r : std_logic_vector(7 downto 0);
    signal prior_on_r      : std_logic;
    signal sum_b2_01_r     : unsigned(C_TDL_BITS - 1 downto 0);
    signal sum_b2_23_r     : unsigned(C_TDL_BITS - 1 downto 0);
    signal sum_b2_01_r2    : unsigned(C_TDL_BITS - 1 downto 0);
    signal sum_b2_23_r2    : unsigned(C_TDL_BITS - 1 downto 0);

  begin

    p_bubble_filter : process(all)
    begin
      therm_filt_s(0)                 <= tdl_captured_reg(lane)(0);
      therm_filt_s(GC_TDL_LENGTH - 1) <= tdl_captured_reg(lane)(GC_TDL_LENGTH - 1);

      for i in 1 to GC_TDL_LENGTH - 2 loop
        if (tdl_captured_reg(lane)(i - 1) and tdl_captured_reg(lane)(i)) or (tdl_captured_reg(lane)(i) and tdl_captured_reg(lane)(i + 1)) or (tdl_captured_reg(lane)(i - 1) and tdl_captured_reg(lane)(i + 1)) then
          therm_filt_s(i) <= '1';
        else
          therm_filt_s(i) <= '0';
        end if;
      end loop;
    end process;

    p_filt_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        therm_filt_reg <= therm_filt_s;
      end if;
    end process;

    g_group_clamp : for g in 0 to C_POPCOUNT_GROUPS - 1 generate
      constant C_L : integer := g * C_GROUP_SIZE;
      constant C_R : integer := g * C_GROUP_SIZE + C_GROUP_SIZE - 1;
    begin
      group_clamp(C_L) <= therm_filt_reg(C_L);
      g_intra : for b in C_L + 1 to C_R generate
      begin
        group_clamp(b) <= group_clamp(b - 1) or therm_filt_reg(b);
      end generate;

      group_any(g) <= group_clamp(C_R);
    end generate;

    p_group_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        group_clamp_reg <= group_clamp;
        group_any_reg   <= group_any;
      end if;
    end process;

    prefix_half_a(0) <= group_any_reg(0);
    g_prefix_first_half : for g in 1 to 7 generate
    begin
      prefix_half_a(g) <= prefix_half_a(g - 1) or group_any_reg(g);
    end generate;

    p_prefix_half_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        prefix_half_a_r <= prefix_half_a;
        prior_on_r      <= prefix_half_a(7);
      end if;
    end process;

    g_prefix_passthrough : for g in 0 to 7 generate
    begin
      group_prefix_on(g) <= prefix_half_a_r(g);
    end generate;

    group_prefix_on(8) <= prior_on_r or group_any_reg(8);
    g_prefix_second_half : for g in 9 to C_POPCOUNT_GROUPS - 1 generate
    begin
      group_prefix_on(g) <= group_prefix_on(g - 1) or group_any_reg(g);
    end generate;

    g_final_clamp : for g in 0 to C_POPCOUNT_GROUPS - 1 generate
      constant C_L : integer := g * C_GROUP_SIZE;
      constant C_R : integer := g * C_GROUP_SIZE + C_GROUP_SIZE - 1;
    begin
      g_group_zero : if g = 0 generate
        g_bits_first : for b in C_L to C_R generate
        begin
          therm_mono_s(b) <= group_clamp_reg(b);
        end generate;
      end generate;

      g_group_nonzero : if g > 0 generate
        g_bits_later : for b in C_L to C_R generate
        begin
          therm_mono_s(b) <= group_prefix_on(g - 1) or ((not group_prefix_on(g - 1)) and group_clamp_reg(b));
        end generate;
      end generate;
    end generate;

    p_mono_reg : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        therm_mono_reg <= therm_mono_s;
      end if;
    end process;

    p_encode_stage_a : process(clk_tdc)
      variable v_popcount : integer range 0 to C_GROUP_SIZE;
    begin
      if rising_edge(clk_tdc) then
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

    p_encode_stage_b1 : process(clk_tdc)
      variable v_partial : unsigned(C_TDL_BITS - 1 downto 0);
    begin
      if rising_edge(clk_tdc) then
        for p in 0 to C_PARTIAL_GROUPS - 1 loop
          v_partial             := (others => '0');
          for g in 0 to C_GROUPS_PER_PARTIAL - 1 loop
            v_partial := v_partial + resize(group_counts(lane)(p * C_GROUPS_PER_PARTIAL + g), C_TDL_BITS);
          end loop;
          partial_sums(lane)(p) <= v_partial;
        end loop;
      end if;
    end process;

    p_encode_stage_b2a : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        sum_b2_01_r <= partial_sums(lane)(0) + partial_sums(lane)(1);
        sum_b2_23_r <= partial_sums(lane)(2) + partial_sums(lane)(3);
      end if;
    end process;

    p_encode_stage_b2a5 : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        sum_b2_01_r2 <= sum_b2_01_r;
        sum_b2_23_r2 <= sum_b2_23_r;
      end if;
    end process;

    p_encode_stage_b2b : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        fine_codes_raw(lane) <= sum_b2_01_r2 + sum_b2_23_r2;
      end if;
    end process;

  end generate;

  p_calibration : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        for i in 0 to GC_TDL_LENGTH loop
          if i = GC_TDL_LENGTH then
            cal_lut(i) <= to_unsigned((2 ** C_FINE_FRAC_BITS) - 1, C_FINE_FRAC_BITS);
          else
            cal_lut(i) <= to_unsigned((i * (2 ** C_FINE_FRAC_BITS)) / GC_TDL_LENGTH, C_FINE_FRAC_BITS);
          end if;
        end loop;
        for lane in 0 to GC_TDL_LANES - 1 loop
          lane_offsets(lane) <= (others => '0');
          lane_scales(lane)  <= to_unsigned(2 ** 15, 16);
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
          v_cal_fp := cal_lut(v_idx);

          v_scaled := v_cal_fp * lane_scales(lane);
          v_cal_fp := resize(shift_right(v_scaled, 15), C_FINE_FRAC_BITS);

          if v_cal_fp >= lane_offsets(lane) then
            v_cal_fp := v_cal_fp - lane_offsets(lane);
          else
            v_cal_fp := resize(('0' & v_cal_fp) + to_unsigned(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 1) - ('0' & lane_offsets(lane)), C_FINE_FRAC_BITS);
          end if;

          if ('0' & v_cal_fp) >= to_unsigned(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 1) then
            fine_codes_fp(lane) <= resize(('0' & v_cal_fp) - to_unsigned(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 1), C_FINE_FRAC_BITS);
          else
            fine_codes_fp(lane) <= v_cal_fp;
          end if;
        else
          fine_codes_fp(lane) <= (others => '1');
        end if;
      end if;
    end process;
  end generate;

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
    variable v_ssum : unsigned(C_FINE_FRAC_BITS + 1 downto 0);
    variable v_avg  : unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      v_ssum      := ('0' & sum01_r) + ('0' & sum23_r);
      v_avg       := resize(shift_right(v_ssum, 2), C_FINE_FRAC_BITS);
      fine_avg_fp <= v_avg;
    end if;
  end process;

  p_start_timestamp : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if start_pulse = '1' then
        start_coarse_hold <= coarse_counter_bin;
      end if;
    end if;
  end process;

  p_stop_timestamp : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if analog_stop_mark = '1' then
        stop_coarse_bin <= coarse_counter_bin;

        start_coarse_at_mark <= start_coarse_hold;
      end if;

    end if;
  end process;

  p_pipe_delay : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      start_coarse_pipe_d1 <= start_coarse_at_mark;
      start_coarse_pipe_d2 <= start_coarse_pipe_d1;
      start_coarse_pipe_d3 <= start_coarse_pipe_d2;
      start_coarse_pipe_d4 <= start_coarse_pipe_d3;
      start_coarse_pipe_d5 <= start_coarse_pipe_d4;
      start_coarse_pipe_d6 <= start_coarse_pipe_d5;
      start_coarse_pipe_d7 <= start_coarse_pipe_d6;
      start_coarse_pipe_d8 <= start_coarse_pipe_d7;

      stop_coarse_pipe_d1 <= stop_coarse_bin;
      stop_coarse_pipe_d2 <= stop_coarse_pipe_d1;
      stop_coarse_pipe_d3 <= stop_coarse_pipe_d2;
      stop_coarse_pipe_d4 <= stop_coarse_pipe_d3;
      stop_coarse_pipe_d5 <= stop_coarse_pipe_d4;
      stop_coarse_pipe_d6 <= stop_coarse_pipe_d5;
      stop_coarse_pipe_d7 <= stop_coarse_pipe_d6;
      stop_coarse_pipe_d8 <= stop_coarse_pipe_d7;

      analog_stop_mark_d1   <= analog_stop_mark;
      analog_stop_mark_d2   <= analog_stop_mark_d1;
      analog_stop_mark_d3   <= analog_stop_mark_d2;
      analog_stop_mark_d4   <= analog_stop_mark_d3;
      analog_stop_mark_d5   <= analog_stop_mark_d4;
      analog_stop_mark_d6   <= analog_stop_mark_d5;
      analog_stop_mark_d7   <= analog_stop_mark_d6;
      analog_stop_mark_d8   <= analog_stop_mark_d7;
      analog_stop_mark_pipe <= analog_stop_mark_d8;

      if analog_stop_mark_pipe = '1' then
        if signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8) < -128 then
          dcoarse_signed_raw <= resize(
            resize(signed('0' & stop_coarse_pipe_d8), GC_COARSE_BITS + 2) - resize(signed('0' & start_coarse_pipe_d8), GC_COARSE_BITS + 2) + to_signed(256, GC_COARSE_BITS + 2), GC_COARSE_BITS + 1);
        else
          dcoarse_signed_raw <= signed('0' & stop_coarse_pipe_d8) - signed('0' & start_coarse_pipe_d8);
        end if;
      end if;
    end if;
  end process;

  p_auto_calibration : process(clk_tdc)
    variable v_dcoarse_idx : integer range 0 to 255;
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
        calib_delay_cnt        <= 0;
        calib_ready            <= '0';
        calib_hist_idx_d1      <= 0;
        calib_hist_value_d1    <= (others => '0');
        calib_hist_idx_d2      <= 0;
        calib_hist_value_d2    <= (others => '0');
        calib_hist_update_d2   <= '0';
        calib_hist_update_d2   <= '0';
        calib_hist_read_d1     <= (others => '0');
        calib_search_idx_d1    <= 0;
        calib_hist_read_addr   <= 0;
        calib_hist_idx_d3      <= 0;
        calib_hist_update_d3   <= '0';
        calib_hist_value_d3    <= (others => '0');

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

            -- Pipeline stage 1: Read histogram using dedicated read index
            calib_hist_idx_d1    <= v_dcoarse_idx;
            calib_hist_read_addr <= v_dcoarse_idx;

            calib_sample_count <= calib_sample_count + 1;
          end if;                       -- analog_stop_mark_calib

          -- Pipeline stage 2: Perform the Read (synchronous to address change)
          -- This breaks the timing path from dcoarse logic to memory output
          calib_hist_value_d1 <= calib_histogram(calib_hist_read_addr);
          calib_hist_idx_d1   <= calib_hist_read_addr; -- Pass address along pipeline

          -- Pipeline stage 3: Register read value (d2)
          -- Note: shifted pipeline stages d2, d3...
          calib_hist_idx_d2   <= calib_hist_idx_d1;
          calib_hist_value_d2 <= calib_hist_value_d1;
          if calib_sample_count > 0 and calib_sample_count <= 16 then
            calib_hist_update_d2 <= '1';
          else
            calib_hist_update_d2 <= '0';
          end if;

          -- Pipeline stage 3: Increment
          calib_hist_idx_d3    <= calib_hist_idx_d2;
          calib_hist_update_d3 <= calib_hist_update_d2;
          if calib_hist_update_d2 = '1' and calib_hist_value_d2 < 15 then
            calib_hist_value_d3 <= calib_hist_value_d2 + 1;
          else
            calib_hist_value_d3 <= calib_hist_value_d2; -- Pass through unchanged
          end if;

          -- Pipeline stage 4: Write back
          if calib_hist_update_d3 = '1' then
            calib_histogram(calib_hist_idx_d3) <= calib_hist_value_d3;
          end if;

          -- PHASE 2: After 16 samples, wait two cycles for pipeline flush, then start search
          if calib_sample_count = 18 and calib_searching = '0' then
            -- Initialize search
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

  coarse_bias_effective <= coarse_bias_calibrated when (calib_done = '1') else coarse_bias;

  p_bias_debug : process(clk_tdc)
    variable v_bias_prev : unsigned(7 downto 0) := (others => '0');
  begin
    if rising_edge(clk_tdc) then
      if coarse_bias_effective /= v_bias_prev then
        report "BIAS CHANGE: coarse_bias_effective=" & integer'image(to_integer(coarse_bias_effective)) & " (calib_done=" & std_logic'image(calib_done) & " input_bias=" & integer'image(to_integer(coarse_bias)) & " calib_bias=" & integer'image(to_integer(coarse_bias_calibrated)) & ")" severity note;
        v_bias_prev := coarse_bias_effective;
      end if;
    end if;
  end process;

  p_dcoarse_register : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      dcoarse_signed_at_comp <= dcoarse_signed_raw;
      dcoarse_for_calib      <= dcoarse_signed_raw;
      analog_stop_mark_calib <= analog_stop_mark_pipe;

      analog_stop_mark_comp_pre <= analog_stop_mark_pipe;
      analog_stop_mark_comp     <= analog_stop_mark_comp_pre;
    end if;
  end process;

  p_stage1a : process(clk_tdc)
    variable v_b0_s : signed(GC_COARSE_BITS downto 0);
    variable v_bm_s : signed(GC_COARSE_BITS downto 0);
    variable v_bp_s : signed(GC_COARSE_BITS downto 0);
  begin
    if rising_edge(clk_tdc) then
      s1a_valid <= '0';

      if analog_stop_mark_comp = '1' then
        fine_at_comp <= fine_avg_fp;

        v_b0_s := signed(resize(coarse_bias_effective, GC_COARSE_BITS + 1));
        v_bm_s := v_b0_s - to_signed(3, v_b0_s'length);
        v_bp_s := v_b0_s + to_signed(3, v_b0_s'length);

        adj0_s1 <= dcoarse_signed_at_comp - v_b0_s;
        adjm_s1 <= dcoarse_signed_at_comp - v_bm_s;
        adjp_s1 <= dcoarse_signed_at_comp - v_bp_s;

        s1a_valid <= '1';
      end if;
    end if;
  end process;

  p_stage1b : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if s1a_valid = '1' then
        mag0_s1b <= abs_s9(adj0_s1);
        magm_s1b <= abs_s9(adjm_s1);
        magp_s1b <= abs_s9(adjp_s1);

        adj0_s1b <= adj0_s1;
        adjm_s1b <= adjm_s1;
        adjp_s1b <= adjp_s1;

        fine_s1b <= ('0' & fine_at_comp);

        s1b_valid <= '1';
      else
        s1b_valid <= '0';
      end if;
    end if;
  end process;

  p_stage1c : process(clk_tdc)
    variable v_d_used     : signed(GC_COARSE_BITS downto 0);
    variable v_mag_winner : unsigned(GC_COARSE_BITS downto 0);
  begin
    if rising_edge(clk_tdc) then
      if s1b_valid = '1' then
        v_d_used     := adj0_s1b;
        v_mag_winner := mag0_s1b;

        if magm_s1b < mag0_s1b then
          v_d_used     := adjm_s1b;
          v_mag_winner := magm_s1b;
        elsif magm_s1b = mag0_s1b then
          if (adjm_s1b = to_signed(1, adjm_s1b'length) and fine_s1b < to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) or (adjm_s1b = to_signed(-1, adjm_s1b'length) and fine_s1b >= to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) then
            v_d_used     := adjm_s1b;
            v_mag_winner := magm_s1b;
          end if;
        end if;

        if magp_s1b < v_mag_winner then
          v_d_used := adjp_s1b;
        elsif magp_s1b = v_mag_winner then
          if (adjp_s1b = to_signed(1, adjp_s1b'length) and fine_s1b < to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) or (adjp_s1b = to_signed(-1, adjp_s1b'length) and fine_s1b >= to_unsigned(2 ** (C_FINE_FRAC_BITS - 1), fine_s1b'length)) then
            v_d_used := adjp_s1b;
          end if;
        end if;

        d_used_s1c <= v_d_used;
        fine_s1c   <= fine_s1b;

        s1c_valid <= '1';

      else
        s1c_valid <= '0';
      end if;
    end if;
  end process;

  p_stage1d : process(clk_tdc)
    variable v_dfine_ext : signed(C_FINE_FRAC_BITS + 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if s1c_valid = '1' then
        if d_used_s1c = to_signed(-1, GC_COARSE_BITS + 1) then
          v_dfine_ext := signed('0' & fine_s1c) + to_signed(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 2);
        elsif d_used_s1c = to_signed(0, GC_COARSE_BITS + 1) then
          v_dfine_ext := signed('0' & fine_s1c);
        elsif d_used_s1c = to_signed(1, GC_COARSE_BITS + 1) then
          v_dfine_ext := signed('0' & fine_s1c) - to_signed(2 ** C_FINE_FRAC_BITS, C_FINE_FRAC_BITS + 2);
        else
          v_dfine_ext := signed('0' & fine_s1c);
        end if;

        dfine_ext_s1b <= v_dfine_ext;

        d_used_s1 <= d_used_s1c;

        s_vld_pre <= '1';

        s1d_valid <= '1';

      else
        s_vld_pre <= '0';
        s1d_valid <= '0';
      end if;
    end if;
  end process;

  p_stage1e : process(clk_tdc)
    variable v_delta_meas : signed(C_TIME_FP_BITS - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if s1d_valid = '1' then
        v_delta_meas := shift_left(resize(d_used_s1, C_TIME_FP_BITS), C_FINE_FRAC_BITS) + resize(dfine_ext_s1b, C_TIME_FP_BITS);
        s_delta_meas <= v_delta_meas;
      end if;
    end if;
  end process;

  p_tdac_subtract : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      s_delta_meas_adj <= s_delta_meas;
    end if;
  end process p_tdac_subtract;

  p_win_check : process(clk_tdc)
    variable v_win_ok_local : std_logic := '0';
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        s_win_ok_pipe(0) <= '0';
        s_vld_pipe(0)    <= '0';
        win_fail_toggle  <= '0';
      else
        v_win_ok_local := '0';
        v_win_ok_local := '1';

        s_win_ok_pipe(0) <= v_win_ok_local;
        s_vld_pipe(0)    <= s_vld_pre;

        if v_win_ok_local = '0' then
          win_fail_toggle <= not win_fail_toggle;
        end if;
      end if;
    end if;
  end process p_win_check;

  p_interval_stage2 : process(clk_tdc)
    variable v_centered : signed(C_TIME_FP_BITS - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        s_vld_pipe(1)    <= '0';
        s_win_ok_pipe(1) <= '0';
        s_centered       <= (others => '0');
      else
        if s_vld_pipe(0) = '1' then
          v_centered := s_delta_meas_adj - C_HALF_COARSE_FP;
          s_centered <= v_centered;

          s_vld_pipe(1)    <= '1';
          s_win_ok_pipe(1) <= s_win_ok_pipe(0);
        else
          s_vld_pipe(1)    <= '0';
          s_win_ok_pipe(1) <= '0';
        end if;
      end if;
    end if;
  end process;

  p_interval_stage3 : process(clk_tdc)
    constant C_ROUND_BIT   : integer                             := C_TIME_FP_BITS - GC_OUTPUT_WIDTH - 1;
    constant C_ROUND_CONST : signed(C_TIME_FP_BITS - 1 downto 0) := to_signed(2 ** C_ROUND_BIT, C_TIME_FP_BITS);
    constant C_OVF_TOL     : integer                             := 300 * (2 ** C_FINE_FRAC_BITS);
    variable v_ovf_check   : boolean;
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        lost_sample_latch <= '0';
        overflow_buf      <= '0';
        sample_ready      <= '0';
        tdc_valid_i       <= '0';
        tdc_result_buf    <= (others => '0');
        overflow_i        <= '0';
        tdc_result        <= (others => '0');
        s_round_sum       <= (others => '0');
        s_shifted         <= (others => '0');
        s_rounded         <= (others => '0');
        s_ovf_a           <= '0';
        s_ovf_b           <= '0';
        s_ovf_pipe        <= '0';
        s_vld_pipe(2)     <= '0';
        s_vld_pipe(3)     <= '0';
        s_vld_pipe(4)     <= '0';
      else
        tdc_valid_i <= '0';

        if s_vld_pipe(1) = '1' then
          v_ovf_check := (s_win_ok_pipe(1) = '0') or (s_centered > C_OVF_TOL) or (s_centered < -C_OVF_TOL);
          s_ovf_a     <= '1' when v_ovf_check else '0';

          if C_ROUND_BIT >= 0 then
            s_round_sum <= s_centered + C_ROUND_CONST;
          else
            s_round_sum <= s_centered;
          end if;

          s_vld_pipe(2) <= '1';
        else
          s_vld_pipe(2) <= '0';
        end if;

        if s_vld_pipe(2) = '1' then
          s_ovf_b       <= s_ovf_a;
          s_shifted     <= resize(shift_right(s_round_sum, C_TIME_FP_BITS - GC_OUTPUT_WIDTH), GC_OUTPUT_WIDTH);
          s_vld_pipe(3) <= '1';
        else
          s_vld_pipe(3) <= '0';
        end if;

        if s_vld_pipe(3) = '1' then
          s_ovf_pipe    <= s_ovf_b;
          if invert_polarity = '1' then
            s_rounded <= -s_shifted;
          else
            s_rounded <= s_shifted;
          end if;
          s_vld_pipe(4) <= '1';
        else
          s_vld_pipe(4) <= '0';
        end if;

        if s_vld_pipe(4) = '1' then
          if s_ovf_pipe = '1' then
            overflow_buf      <= '1';
            sample_ready      <= '0';
            lost_sample_latch <= '1';
          else
            overflow_buf      <= '0';
            lost_sample_latch <= '0';
            tdc_result_buf    <= s_rounded;
            sample_ready      <= '1';
          end if;
        end if;

        if start_pulse = '1' then
          if sample_ready = '1' then
            tdc_result   <= tdc_result_buf;
            overflow_i   <= overflow_buf;
            tdc_valid_i  <= '1';
            sample_ready <= '0';
          else
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

  fine_at_comp_out <= fine_at_comp;
  fine_valid_out   <= s1a_valid;

end architecture;
