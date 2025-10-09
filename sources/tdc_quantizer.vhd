library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.common_pkg.all;

entity tdc_quantizer is
  generic(
    GC_TDL_LANES    : positive range 1 to 16 := 4; -- Must be power of 2 for averaging
    GC_TDL_LENGTH   : positive               := 128; -- Taps per TDL lane
    GC_COARSE_BITS  : positive               := 8; -- Coarse counter width
    GC_OUTPUT_WIDTH : positive               := 16; -- Total TDC output width
    GC_TIME_DAC_DEN : positive               := 256 -- Digital Time-DAC step denominator (step = 1/GC_TIME_DAC_DEN)
  );
  port(
    -- Clocks
    clk_sys         : in  std_logic;    -- System clock (e.g., 100 MHz)
    clk_tdc         : in  std_logic;    -- TDC fast clock (e.g., 400-600 MHz)
    reset           : in  std_logic;
    -- LVDS differential input (Quartus manages differential pair with LVDS_E_3V standard)
    analog_in       : in  std_logic;    -- LVDS comparator output (true differential)

    -- Reference timing (Start edge source - should be clk_tdc or synchronous)
    ref_phases      : in  std_logic_vector(0 downto 0); -- Single phase (ref_phases(0) = Start source)

    -- 1-bit Time DAC control (feedback from loop filter)
    time_dac_ctrl   : in  std_logic;    -- 1 = advance, 0 = retard reference

    -- Polarity control (allows P/N swap correction without rebuild)
    invert_polarity : in  std_logic;    -- 1 = invert sign, 0 = normal

    -- Status control
    clear_status    : in  std_logic;    -- 1 = clear sticky lost_sample flag

    -- TDC output
    tdc_out         : out signed(GC_OUTPUT_WIDTH - 1 downto 0);
    tdc_valid       : out std_logic;
    overflow        : out std_logic;
    lost_sample     : out std_logic     -- Sticky flag: overflow occurred since reset
  );
end entity;

architecture rtl of tdc_quantizer is

  -- ========================================================================
  -- Constants
  -- ========================================================================
  constant C_TDL_BITS : natural := clog2(GC_TDL_LENGTH + 1);

  -- Fixed-point for fine time: Q0.F format (fraction of Tclk_tdc)
  constant C_FINE_FRAC_BITS : natural                             := 16;
  constant C_TIME_FP_BITS   : natural                             := GC_COARSE_BITS + C_FINE_FRAC_BITS;
  constant C_HALF_SCALE_FP  : signed(C_TIME_FP_BITS - 1 downto 0) := to_signed(2 ** (GC_COARSE_BITS - 1 + C_FINE_FRAC_BITS), C_TIME_FP_BITS);

  -- Window tolerance: 1 LSB of Q0.F for boundary edge cases
  constant C_WINDOW_TOLERANCE_FP : signed(C_FINE_FRAC_BITS downto 0) := to_signed(1, C_FINE_FRAC_BITS + 1);

  -- Time-DAC: Step size in Q0.F format (fraction of Tclk_tdc per step)
  -- Step = 1/GC_TIME_DAC_DEN (e.g., 1/256 for fine granularity)
  constant C_TIME_DAC_STEP_FP : unsigned(C_FINE_FRAC_BITS - 1 downto 0) := to_unsigned((2 ** C_FINE_FRAC_BITS) / GC_TIME_DAC_DEN, C_FINE_FRAC_BITS);

  -- ========================================================================
  -- Types
  -- ========================================================================
  type T_TDL_THERM        is array (0 to GC_TDL_LENGTH - 1) of std_logic;
  type T_TDL_THERM_ARRAY  is array (0 to GC_TDL_LANES - 1) of T_TDL_THERM;
  type T_FINE_CODE_ARRAY  is array (0 to GC_TDL_LANES - 1) of unsigned(C_TDL_BITS - 1 downto 0);
  type T_FINE_FP_ARRAY    is array (0 to GC_TDL_LANES - 1) of unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  type T_CAL_LUT          is array (0 to GC_TDL_LENGTH) of unsigned(C_FINE_FRAC_BITS - 1 downto 0);
  type T_LANE_SCALE_ARRAY is array (0 to GC_TDL_LANES - 1) of unsigned(15 downto 0); -- Q1.15 format

  -- ========================================================================
  -- Signals
  -- ========================================================================
  signal analog_crossing : std_logic;

  -- Start/Stop arming (one stop per reference period)
  signal start_pulse     : std_logic;   -- Single-cycle pulse on ref_phases(0) rising edge
  signal ref_phase0_prev : std_logic;   -- For edge detection
  signal armed           : std_logic;   -- Arms stop capture after Start, clears after first Stop

  -- Stop synchronization
  signal analog_sync      : std_logic_vector(1 downto 0);
  signal analog_stop_mark : std_logic;
  signal stop_mark_prev   : std_logic;
  signal edge_inhibit     : unsigned(1 downto 0); -- 2-cycle inhibit counter for robustness

  -- TDL
  signal tdl_chains       : T_TDL_THERM_ARRAY;
  signal tdl_bank_current : T_TDL_THERM_ARRAY;
  signal tdl_bank_prev    : T_TDL_THERM_ARRAY;
  signal tdl_captured     : T_TDL_THERM_ARRAY;

  -- Gray counter (banked like TDL for alignment)
  signal coarse_counter_gray   : std_logic_vector(GC_COARSE_BITS - 1 downto 0);
  signal coarse_counter_bin    : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal coarse_gray_bank_curr : std_logic_vector(GC_COARSE_BITS - 1 downto 0);
  signal coarse_gray_bank_prev : std_logic_vector(GC_COARSE_BITS - 1 downto 0);

  -- Start/Stop timestamps
  signal start_coarse_bin   : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal start_fine_fp_sync : unsigned(C_FINE_FRAC_BITS - 1 downto 0);

  signal stop_coarse_bin : unsigned(GC_COARSE_BITS - 1 downto 0);
  signal stop_fine_fp    : unsigned(C_FINE_FRAC_BITS - 1 downto 0);

  -- Digital Time-DAC command (applied in interval computation)
  signal time_dac_cmd_fp : signed(C_FINE_FRAC_BITS downto 0); -- Q0.F signed Δt command

  -- Fine codes
  signal fine_codes_raw : T_FINE_CODE_ARRAY;
  signal fine_codes_fp  : T_FINE_FP_ARRAY;
  signal lane_offsets   : T_FINE_FP_ARRAY;
  signal lane_scales    : T_LANE_SCALE_ARRAY; -- Q1.15 per-lane gain
  signal fine_avg_fp    : unsigned(C_FINE_FRAC_BITS - 1 downto 0);

  -- Output (buffered for start-synchronous delivery)
  signal tdc_result_buf    : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  signal overflow_buf      : std_logic;
  signal sample_ready      : std_logic; -- Internal flag: sample captured on Stop
  signal tdc_result        : signed(GC_OUTPUT_WIDTH - 1 downto 0);
  signal tdc_valid_i       : std_logic;
  signal overflow_i        : std_logic;
  signal lost_sample_latch : std_logic; -- Sticky flag for overflow tracking

  -- Calibration
  signal cal_lut : T_CAL_LUT;

  -- Attributes
  attribute KEEP                     : boolean;
  attribute DONT_TOUCH               : boolean;
  attribute DONT_MERGE               : boolean;
  attribute KEEP of tdl_chains       : signal is true;
  attribute DONT_TOUCH of tdl_chains : signal is true;
  attribute DONT_MERGE of tdl_chains : signal is true;

begin

  -- ========================================================================
  -- Design Constraints and Assertions
  -- ========================================================================
  -- Ensure GC_TDL_LANES is power of 2 for efficient averaging
  assert (2 ** clog2(GC_TDL_LANES) = GC_TDL_LANES)
  report "GC_TDL_LANES must be a power of 2 (1, 2, 4, 8, 16)" severity failure;

  -- ========================================================================
  -- LVDS Input (Quartus-managed differential pair with LVDS_E_3V standard)
  -- ========================================================================
  -- Pin assignment in .qsf assigns differential pair to analog_in
  -- LVDS I/O buffer is inferred by Quartus (no explicit primitive needed)
  analog_crossing <= analog_in;

  -- ========================================================================
  -- Gray-Coded Free-Running Coarse Counter (with banking like TDL)
  -- ========================================================================
  p_coarse_counter : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        coarse_counter_bin    <= (others => '0');
        coarse_counter_gray   <= (others => '0');
        coarse_gray_bank_curr <= (others => '0');
        coarse_gray_bank_prev <= (others => '0');
      else
        -- Increment binary counter
        coarse_counter_bin                      <= coarse_counter_bin + 1;
        -- Binary to Gray conversion
        coarse_counter_gray(GC_COARSE_BITS - 1) <= std_logic(coarse_counter_bin(GC_COARSE_BITS - 1));
        for i in 0 to GC_COARSE_BITS - 2 loop
          coarse_counter_gray(i) <= std_logic(coarse_counter_bin(i) xor coarse_counter_bin(i + 1));
        end loop;
        -- Bank the Gray counter (same as TDL banks)
        coarse_gray_bank_prev                   <= coarse_gray_bank_curr;
        coarse_gray_bank_curr                   <= coarse_counter_gray;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Digital Time-DAC (Replaces Clock Mux to Avoid Glitch Risks)
  -- ========================================================================
  -- Instead of physically muxing ref_phases (which requires ALTCLKCTRL and is limited to 2-4 phases),
  -- we use a SINGLE reference phase and apply the time offset DIGITALLY in the interval computation.
  -- This is robust, tool-friendly, and avoids global-clock switching hazards.
  -- 
  -- The commanded time offset is accumulated and applied as: e = Δt_meas - Δt_cmd
  -- where Δt_cmd = ±C_TIME_DAC_STEP_FP per feedback bit

  p_time_dac_digital : process(clk_tdc)
    variable v_ref0_prev : std_logic;
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        time_dac_cmd_fp <= (others => '0'); -- Start at zero offset
        v_ref0_prev     := '0';
      else
        -- Update time command once per reference period (on phase 0 rising edge)
        -- This prevents mid-cycle updates and maintains deterministic timing
        if ref_phases(0) = '1' and v_ref0_prev = '0' then
          if time_dac_ctrl = '1' then
            -- Advance: positive time offset (increase commanded delay)
            time_dac_cmd_fp <= time_dac_cmd_fp + signed('0' & C_TIME_DAC_STEP_FP);
          else
            -- Retard: negative time offset (decrease commanded delay)
            time_dac_cmd_fp <= time_dac_cmd_fp - signed('0' & C_TIME_DAC_STEP_FP);
          end if;
        end if;
        v_ref0_prev := ref_phases(0);
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Start Pulse Generation (Edge Detect on ref_phases(0) in clk_tdc Domain)
  -- ========================================================================
  -- CRITICAL: Keep everything in clk_tdc domain - no derived clock from ref_edge_delayed
  -- Generate single-cycle start_pulse on rising edge of ref_phases(0)
  p_start_edge_detect : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
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
  -- Start armed after reset to allow first measurement
  p_arm_control : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        armed <= '1';                   -- Start armed after reset to allow first Stop
      else
        if start_pulse = '1' then
          -- Arm on Start edge (allows next Stop to be captured)
          armed <= '1';
        elsif analog_stop_mark = '1' and armed = '1' then
          -- Disarm after first Stop edge (ignore subsequent edges until next Start)
          armed <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stop Mark Synchronization with Multi-Cycle Inhibit and Arming
  -- ========================================================================
  -- Edge detector with 2-cycle inhibit + armed gating
  -- Only accept stop edge when armed=1 (first edge after Start)
  -- Clear inhibit on every Start to prevent cross-frame carryover
  p_stop_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        analog_sync      <= (others => '0');
        analog_stop_mark <= '0';
        stop_mark_prev   <= '0';
        edge_inhibit     <= (others => '0');
      else
        -- 2-FF synchronizer for async LVDS input
        analog_sync    <= analog_sync(0) & analog_crossing;
        stop_mark_prev <= analog_sync(1);

        -- Clear inhibit on every Start to prevent carryover across frames
        if start_pulse = '1' then
          edge_inhibit <= (others => '0');
        end if;

        -- Edge detection with armed gating and 2-cycle inhibit
        -- Only trigger when: rising edge + armed + no inhibit
        if analog_sync(1) = '1' and stop_mark_prev = '0' and armed = '1' and edge_inhibit = 0 then
          analog_stop_mark <= '1';
          edge_inhibit     <= to_unsigned(2, 2); -- Inhibit for 2 cycles
        else
          analog_stop_mark <= '0';
          -- Decrement inhibit counter (saturates at 0)
          if edge_inhibit > 0 then
            edge_inhibit <= edge_inhibit - 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- TDL Chains with Continuous Sampling
  -- ========================================================================
  -- CRITICAL for Hardware: TDL inference and placement strategy
  --
  -- Generic delays won't infer carry chains in Quartus. For production hardware:
  --
  -- Option 1: Instantiate Intel carry primitives (CARRY_SUM) per tap
  --   - Direct control over carry-chain mapping
  --   - Device-specific (requires Agilex 5 primitive library)
  --
  -- Option 2: Use adder cascade pattern that Quartus maps to carry
  --   - signal adder_sum : std_logic_vector(0 to GC_TDL_LENGTH);
  --   - adder_sum(i+1) <= adder_sum(i) + constant_1;
  --   - Tap carry output for thermometer code
  --   - More portable across Intel FPGA families
  --
  -- Placement constraints (apply in .qsf or .sdc):
  --   1. LogicLock each lane into single LAB column
  --   2. set_instance_assignment -name PRESERVE_REGISTER ON -to tdl_chains[*]
  --   3. set_instance_assignment -name DONT_RETIME ON -to tdl_chains[*]
  --   4. set_instance_assignment -name DONT_DUPLICATE ON -to tdl_chains[*]
  --   5. set_false_path -from [get_ports analog_in] -to tdl_chains[*][*]
  --   6. set_max_delay -datapath_only <small_value> -from tdl_chains[*][i] -to tdl_chains[*][i+1]
  --      (keeps routing tight between adjacent taps and capture flops)
  --
  -- Time-DAC step sizing for production:
  --   After calibration, measure effective TDL LSB from histogram (Q0.F value)
  --   Example: If LSB ≈ 1/128 of clk_tdc period, set GC_TIME_DAC_DEN = 128-256
  --   Goal: Δt_step ≈ 1-2 calibrated fine LSBs for smooth loop dynamics
  --
  g_tdl_lanes : for lane in 0 to GC_TDL_LANES - 1 generate
    signal    tdl_delay_chain               : std_logic_vector(0 to GC_TDL_LENGTH);
    attribute KEEP of tdl_delay_chain       : signal is true;
    attribute DONT_TOUCH of tdl_delay_chain : signal is true;
  begin
    tdl_delay_chain(0) <= analog_crossing;

    -- pragma translate_off
    g_taps : for tap in 0 to GC_TDL_LENGTH - 1 generate
      tdl_delay_chain(tap + 1) <= tdl_delay_chain(tap) after 100 ps;
      tdl_chains(lane)(tap)    <= tdl_delay_chain(tap);
    end generate;
    -- pragma translate_on

    p_tdl_sample : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        if reset = '1' then
          tdl_bank_current(lane) <= (others => '0');
          tdl_bank_prev(lane)    <= (others => '0');
        else
          tdl_bank_prev(lane)    <= tdl_bank_current(lane);
          tdl_bank_current(lane) <= tdl_chains(lane);
        end if;
      end if;
    end process;

    p_frame_select : process(clk_tdc)
    begin
      if rising_edge(clk_tdc) then
        if reset = '1' then
          tdl_captured(lane) <= (others => '0');
        elsif analog_stop_mark = '1' then
          -- CRITICAL: Capture the PREVIOUS bank, not current
          -- analog_stop_mark is synced from the async edge that launched the TDL
          -- By the time stop_mark asserts, one more clk_tdc may have passed
          -- The previous bank reflects the sub-clock instant of the async edge
          tdl_captured(lane) <= tdl_bank_prev(lane);
        end if;
      end if;
    end process;
  end generate;

  -- ========================================================================
  -- TDL Encoder with Bubble Filtering and Monotone Clamping
  -- ========================================================================
  g_encoders : for lane in 0 to GC_TDL_LANES - 1 generate
    p_encode : process(clk_tdc)
      variable v_therm_vec  : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
      variable v_therm_filt : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
      variable v_therm_mono : std_logic_vector(GC_TDL_LENGTH - 1 downto 0);
      variable v_edge_pos   : integer range 0 to GC_TDL_LENGTH;
      variable v_found      : boolean;
    begin
      if rising_edge(clk_tdc) then
        if reset = '1' then
          fine_codes_raw(lane) <= (others => '0');
        else
          for i in 0 to GC_TDL_LENGTH - 1 loop
            v_therm_vec(i) := tdl_captured(lane)(i);
          end loop;

          -- 3-tap majority filter for bubble suppression
          v_therm_filt(0)                 := v_therm_vec(0);
          v_therm_filt(GC_TDL_LENGTH - 1) := v_therm_vec(GC_TDL_LENGTH - 1);
          for i in 1 to GC_TDL_LENGTH - 2 loop
            if (v_therm_vec(i - 1) and v_therm_vec(i)) or (v_therm_vec(i) and v_therm_vec(i + 1)) or (v_therm_vec(i - 1) and v_therm_vec(i + 1)) then
              v_therm_filt(i) := '1';
            else
              v_therm_filt(i) := '0';
            end if;
          end loop;

          -- Running OR to clamp thermometer to monotone (000...0111...1)
          -- Forward pass: once a '1' appears, all subsequent bits stay '1'
          v_therm_mono(0) := v_therm_filt(0);
          for i in 1 to GC_TDL_LENGTH - 1 loop
            v_therm_mono(i) := v_therm_mono(i - 1) or v_therm_filt(i);
          end loop;

          -- Find leading edge (first '1' in monotone thermometer)
          v_edge_pos := 0;
          v_found    := false;
          for i in 0 to GC_TDL_LENGTH - 1 loop
            if v_therm_mono(i) = '1' and not v_found then
              v_edge_pos := i;
              v_found    := true;
            end if;
          end loop;

          -- Handle edge cases
          if not v_found then
            if v_therm_mono(GC_TDL_LENGTH - 1) = '1' then
              v_edge_pos := GC_TDL_LENGTH; -- Saturated
            else
              v_edge_pos := 0;          -- No edge detected
            end if;
          end if;

          fine_codes_raw(lane) <= to_unsigned(v_edge_pos, C_TDL_BITS);
        end if;
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
          -- Linear assumption: bin i represents i/GC_TDL_LENGTH of one period
          -- In Q0.F: (i * 2^F) / GC_TDL_LENGTH
          cal_lut(i) <= to_unsigned((i * (2 ** C_FINE_FRAC_BITS)) / GC_TDL_LENGTH, C_FINE_FRAC_BITS);
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
      variable v_scaled : unsigned(C_FINE_FRAC_BITS + 15 downto 0); -- Q1.15 * Q0.16 → Q1.31
    begin
      if rising_edge(clk_tdc) then
        if reset = '1' then
          fine_codes_fp(lane) <= (others => '0');
        else
          v_idx := to_integer(fine_codes_raw(lane));
          if v_idx <= GC_TDL_LENGTH then
            -- 1) Apply calibration LUT (normalized to Tclk_tdc fraction)
            v_cal_fp := cal_lut(v_idx);

            -- 2) Apply per-lane gain correction (DNL compensation)
            --    Multiply by Q1.15 scale factor, then shift right 15 bits
            v_scaled := v_cal_fp * lane_scales(lane); -- Q0.16 * Q1.15 → Q1.31
            v_cal_fp := resize(shift_right(v_scaled, 15), C_FINE_FRAC_BITS); -- Q1.31 → Q0.16

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
      end if;
    end process;
  end generate;

  p_average : process(clk_tdc)
    variable v_sum : unsigned(C_FINE_FRAC_BITS + clog2(GC_TDL_LANES) - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        fine_avg_fp <= (others => '0');
      else
        v_sum       := (others => '0');
        for lane in 0 to GC_TDL_LANES - 1 loop
          v_sum := v_sum + resize(fine_codes_fp(lane), v_sum'length);
        end loop;
        fine_avg_fp <= resize(shift_right(v_sum, clog2(GC_TDL_LANES)), C_FINE_FRAC_BITS);
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Start Timestamp Capture (clocked by clk_tdc, gated by start_pulse)
  -- ========================================================================
  -- CRITICAL: Keep everything in clk_tdc domain (no derived clock from ref_edge_delayed)
  -- Latch Gray counter on start_pulse (rising edge of ref_phases(0))
  -- Fine time is ZERO (time offset applied digitally via time_dac_cmd_fp)
  p_start_timestamp : process(clk_tdc)
    variable v_temp_bin : unsigned(GC_COARSE_BITS - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        start_coarse_bin   <= (others => '0');
        start_fine_fp_sync <= (others => '0');
      else
        if start_pulse = '1' then
          -- Capture Gray counter at Start edge and convert to binary immediately
          -- No need for separate CDC - already in clk_tdc domain
          v_temp_bin                     := (others => '0');
          v_temp_bin(GC_COARSE_BITS - 1) := '0' when coarse_counter_gray(GC_COARSE_BITS - 1) = '0' else '1';
          for i in GC_COARSE_BITS - 2 downto 0 loop
            if coarse_counter_gray(i) = '1' then
              v_temp_bin(i) := not v_temp_bin(i + 1);
            else
              v_temp_bin(i) := v_temp_bin(i + 1);
            end if;
          end loop;
          start_coarse_bin               <= v_temp_bin;
          -- Start fine time is ZERO - time offset applied digitally in interval computation
          start_fine_fp_sync             <= (others => '0');
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Stop Timestamp Capture (use PREVIOUS banks for alignment)
  -- ========================================================================
  -- CRITICAL: Use coarse_gray_bank_prev to match tdl_bank_prev timing
  -- Both refer to the same "prev" cycle when analog_stop_mark='1'
  p_stop_timestamp : process(clk_tdc)
    variable v_gray_bin : unsigned(GC_COARSE_BITS - 1 downto 0);
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        stop_coarse_bin <= (others => '0');
        stop_fine_fp    <= (others => '0');
      else
        if analog_stop_mark = '1' then
          -- Convert PREVIOUS Gray bank to binary (aligned with tdl_bank_prev)
          v_gray_bin(GC_COARSE_BITS - 1) := '0' when coarse_gray_bank_prev(GC_COARSE_BITS - 1) = '0' else '1';
          for i in GC_COARSE_BITS - 2 downto 0 loop
            if coarse_gray_bank_prev(i) = '1' then
              v_gray_bin(i) := not v_gray_bin(i + 1);
            else
              v_gray_bin(i) := v_gray_bin(i + 1);
            end if;
          end loop;
          stop_coarse_bin                <= v_gray_bin;
          stop_fine_fp                   <= fine_avg_fp;
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Interval Computation with Digital Time-DAC and Tight Window Check
  -- ========================================================================
  -- Stop-synchronous: computes on analog_stop_mark, buffers result
  -- Start-synchronous output: tdc_valid fires on next start_pulse
  p_interval : process(clk_tdc)
    variable v_delta_coarse_raw : unsigned(GC_COARSE_BITS - 1 downto 0);
    variable v_delta_fine       : signed(C_FINE_FRAC_BITS downto 0);
    variable v_coarse_fp        : signed(C_TIME_FP_BITS - 1 downto 0);
    variable v_delta_t_meas     : signed(C_TIME_FP_BITS - 1 downto 0);
    variable v_error            : signed(C_TIME_FP_BITS - 1 downto 0);
    variable v_centered         : signed(C_TIME_FP_BITS - 1 downto 0);
    variable v_result_raw       : signed(GC_OUTPUT_WIDTH - 1 downto 0);
    constant C_ROUND_BIT        : integer := C_TIME_FP_BITS - GC_OUTPUT_WIDTH - 1;
    variable v_valid_window     : boolean;
  begin
    if rising_edge(clk_tdc) then
      if reset = '1' then
        tdc_result_buf    <= (others => '0');
        overflow_buf      <= '0';
        sample_ready      <= '0';
        lost_sample_latch <= '0';
        tdc_result        <= (others => '0');
        tdc_valid_i       <= '0';
        overflow_i        <= '0';
      else
        -- Default: no new output (unless start_pulse triggers delivery)
        tdc_valid_i <= '0';

        -- Clear sticky lost_sample flag on request (or it persists until reset)
        if clear_status = '1' then
          lost_sample_latch <= '0';
        end if;

        -- Stop-synchronous computation: capture interval on analog_stop_mark
        if analog_stop_mark = '1' then
          -- Raw coarse delta (unsigned subtraction, handles wrap via modulo)
          v_delta_coarse_raw := stop_coarse_bin - start_coarse_bin;
          -- Explicit leading '0' to avoid sign-extension issues across tools
          v_delta_fine       := signed('0' & stop_fine_fp) - signed('0' & start_fine_fp_sync);

          -- Tight window check with 1-LSB tolerance on boundary
          -- Per-period ΣΔ should have delta in [0, 1] coarse cycles
          -- AND fine time order must be consistent with coarse delta
          v_valid_window := false;
          if v_delta_coarse_raw = to_unsigned(0, GC_COARSE_BITS) then
            -- Same coarse cycle: require stop_fine >= start_fine
            -- (Allow one-LSB equality for jitter tolerance)
            if v_delta_fine >= 0 then
              v_valid_window := true;
            else
              v_valid_window := false;  -- Fine went backward in same coarse cycle → invalid
            end if;
          elsif v_delta_coarse_raw = to_unsigned(1, GC_COARSE_BITS) then
            -- One coarse cycle difference: require stop_fine < start_fine
            -- (Stop event happened early in next cycle, Start was late in previous)
            -- Add 1-LSB tolerance: accept Δfine ≤ -ε (not just strictly < 0)
            if v_delta_fine <= -C_WINDOW_TOLERANCE_FP then
              v_valid_window := true;
            else
              -- If fine increased or barely negative, interval > 1 period → overflow
              v_valid_window := false;
            end if;
          else
            -- Delta > 1 coarse cycle → overflow (missed sample or extreme wrap)
            v_valid_window := false;
          end if;

          -- Compute measured interval: Δt_meas = (Δcoarse << F) + Δfine
          -- Explicit resize to avoid sign-extension issues
          v_coarse_fp    := signed(resize(v_delta_coarse_raw, C_TIME_FP_BITS - C_FINE_FRAC_BITS)) & to_signed(0, C_FINE_FRAC_BITS);
          v_delta_t_meas := v_coarse_fp + resize(v_delta_fine, C_TIME_FP_BITS);

          -- Apply digital Time-DAC: error = Δt_meas - Δt_cmd
          -- Δt_cmd is accumulated offset from time_dac_ctrl feedback
          v_error := v_delta_t_meas - resize(time_dac_cmd_fp, C_TIME_FP_BITS);

          -- Center around zero by subtracting half scale (0.5 coarse periods)
          v_centered := v_error - C_HALF_SCALE_FP;

          -- Set overflow flag and squelch if window invalid or out of range
          -- CRITICAL: Don't emit valid samples when overflow - prevents downstream integration errors
          if not v_valid_window then
            overflow_buf      <= '1';
            sample_ready      <= '0';   -- Don't deliver invalid sample
            lost_sample_latch <= '1';   -- Sticky flag for loop supervisor
          elsif v_centered > C_HALF_SCALE_FP or v_centered < -C_HALF_SCALE_FP then
            overflow_buf      <= '1';
            sample_ready      <= '0';   -- Don't deliver out-of-range sample
            lost_sample_latch <= '1';   -- Sticky flag for loop supervisor
          else
            overflow_buf <= '0';
            -- Symmetric rounding: add 0.5 LSB before right-shift
            if C_ROUND_BIT >= 0 then
              v_result_raw := resize(shift_right(v_centered + to_signed(2 ** C_ROUND_BIT, C_TIME_FP_BITS),
                                                 C_TIME_FP_BITS - GC_OUTPUT_WIDTH), GC_OUTPUT_WIDTH);
            else
              v_result_raw := resize(v_centered, GC_OUTPUT_WIDTH);
            end if;

            -- Apply polarity inversion if requested (allows P/N swap correction)
            if invert_polarity = '1' then
              tdc_result_buf <= -v_result_raw;
            else
              tdc_result_buf <= v_result_raw;
            end if;

            sample_ready <= '1';        -- Mark sample ready for delivery
          end if;
        end if;

        -- Start-synchronous delivery: output buffered sample on next start_pulse
        -- This ensures loop filter updates at a deterministic start rate
        if start_pulse = '1' then
          if sample_ready = '1' then
            -- Deliver buffered sample
            tdc_result   <= tdc_result_buf;
            overflow_i   <= overflow_buf;
            tdc_valid_i  <= '1';
            sample_ready <= '0';        -- Clear ready flag after delivery
          else
            -- No sample ready (missed stop or overflow) - don't assert valid
            tdc_valid_i <= '0';
            overflow_i  <= '1';         -- Indicate lost sample
          end if;
        end if;
      end if;
    end if;
  end process;

  tdc_out     <= tdc_result;
  tdc_valid   <= tdc_valid_i;
  overflow    <= overflow_i;
  lost_sample <= lost_sample_latch;

end architecture;
