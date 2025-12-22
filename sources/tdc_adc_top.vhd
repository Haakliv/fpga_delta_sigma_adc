-- TDC-Based Delta-Sigma ADC Top Level

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
use work.dsp_utils_pkg.all;

-- DCFIFO component for CDC (Intel/Altera IP)
library altera_mf;
use altera_mf.altera_mf_components.all;

entity tdc_adc_top is
  generic(
    GC_DECIMATION : positive := 8192;
    GC_DATA_WIDTH : positive := 16;
    GC_TDC_OUTPUT : positive := 16;
    GC_SIM        : boolean  := false;
    GC_FAST_SIM   : boolean  := false;
    GC_OPEN_LOOP  : boolean  := false
  );
  port(
    clk_sys             : in  std_logic;
    clk_tdc             : in  std_logic;
    reset               : in  std_logic;
    ref_clock           : in  std_logic;
    comparator_in       : in  std_logic;
    dac_out_bit         : out std_logic;
    trigger_enable      : in  std_logic            := '1'; -- @suppress "Unused port: trigger_enable is not used in fpga_lib.tdc_adc_top(rtl)"
    open_loop_dac_duty  : in  std_logic            := '0';
    sample_data         : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid        : out std_logic;
    debug_tdc_out       : out signed(GC_TDC_OUTPUT - 1 downto 0);
    debug_tdc_valid     : out std_logic;
    tdc_monitor_code    : out signed(GC_TDC_OUTPUT - 1 downto 0);
    tdc_monitor_center  : out signed(GC_TDC_OUTPUT - 1 downto 0);
    tdc_monitor_diff    : out signed(GC_TDC_OUTPUT - 1 downto 0);
    tdc_monitor_dac     : out std_logic;
    tdc_monitor_valid   : out std_logic;
    disable_tdc_contrib : in  std_logic            := '0';
    negate_tdc_contrib  : in  std_logic            := '0';
    tdc_scale_shift     : in  unsigned(2 downto 0) := "000";
    disable_eq_filter   : in  std_logic            := '0';
    disable_lp_filter   : in  std_logic            := '0';
    adc_ready           : out std_logic
  );
end entity;

architecture rtl of tdc_adc_top is
  constant C_MULTIBIT_WIDTH : positive := 20;

  -- TDC Center Calibration: Dual centers eliminate bimodal distribution noise
  -- DAC=1 causes later comparator crossing (higher TDC), DAC=0 causes earlier (lower TDC)
  signal tdc_center_cal_0 : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_center_cal_1 : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_center_tdc_0 : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_center_tdc_1 : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');

  -- Adaptive TDC center tracking: IIR filter with alpha = 1/256
  signal tdc_center_acc_0  : signed(GC_TDC_OUTPUT + 8 - 1 downto 0) := (others => '0');
  signal tdc_center_acc_1  : signed(GC_TDC_OUTPUT + 8 - 1 downto 0) := (others => '0');
  signal tdc_center_init_0 : std_logic                              := '0';
  signal tdc_center_init_1 : std_logic                              := '0';

  signal cal_sample_cnt_0 : unsigned(8 downto 0) := (others => '0');
  signal cal_sample_cnt_1 : unsigned(8 downto 0) := (others => '0');
  signal cal_done         : std_logic            := '0';

  signal tdc_mon_code   : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_mon_center : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_mon_diff   : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_mon_dac    : std_logic                          := '0';
  signal tdc_mon_valid  : std_logic                          := '0';

  signal ref_phases      : std_logic_vector(0 downto 0);
  signal tdc_out         : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_valid       : std_logic;
  signal tdc_overflow    : std_logic;
  signal tdc_lost_sample : std_logic;

  constant C_DEFAULT_COARSE_BIAS : unsigned(7 downto 0) := to_unsigned(220, 8);

  signal coarse_bias    : unsigned(7 downto 0) := C_DEFAULT_COARSE_BIAS;
  signal coarse_bias_s0 : unsigned(7 downto 0) := C_DEFAULT_COARSE_BIAS;

  signal coarse_bias_tdc : unsigned(7 downto 0) := C_DEFAULT_COARSE_BIAS;

  signal   coarse_bias_cal     : unsigned(7 downto 0)  := C_DEFAULT_COARSE_BIAS;
  signal   tdl_cal_use_cal     : std_logic             := '0';
  signal   fine_at_comp_out    : unsigned(15 downto 0) := (others => '0');
  signal   fine_valid_out      : std_logic             := '0';
  constant C_TDL_CAL_SAMPLES   : natural               := 8;
  signal   tdl_cal_done        : std_logic             := '0';
  signal   tdl_cal_sample_cnt  : unsigned(3 downto 0)  := (others => '0');
  signal   tdl_cal_fine_acc    : unsigned(19 downto 0) := (others => '0');
  signal   tdl_cal_timeout_cnt : unsigned(15 downto 0) := (others => '0');

  constant C_TDL_CAL_SETTLE : natural              := 1000;
  signal   tdl_settle_cnt   : unsigned(9 downto 0) := (others => '0');
  signal   tdl_settling     : std_logic            := '0';

  -- TDL calibration pipeline: breaks timing path from fine_at_comp to tdl_cal_fine_acc
  signal fine_at_comp_reg    : unsigned(15 downto 0) := (others => '0');
  signal fine_valid_reg      : std_logic             := '0';
  signal tdl_cal_sum_reg     : unsigned(19 downto 0) := (others => '0');
  signal tdl_cal_sum_valid   : std_logic             := '0';
  signal tdl_cal_last_sample : std_logic             := '0';

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

  -- CDC: Runtime control (quasi-static, 2-FF synchronizer)
  signal disable_tdc_contrib_sync0 : std_logic            := '0';
  signal disable_tdc_contrib_tdc   : std_logic            := '0';
  signal negate_tdc_contrib_sync0  : std_logic            := '0';
  signal negate_tdc_contrib_tdc    : std_logic            := '0';
  signal tdc_scale_shift_sync0     : unsigned(2 downto 0) := "000";
  signal tdc_scale_shift_tdc       : unsigned(2 downto 0) := "000";

  -- TDC arithmetic pipeline: breaks 400MHz timing path into 3 stages
  signal cal_done_reg         : std_logic                             := '0';
  signal tdc_valid_reg        : std_logic                             := '0';
  signal tdc_sample_ready_reg : std_logic                             := '0';
  signal tdc_out_reg          : signed(GC_TDC_OUTPUT - 1 downto 0)    := (others => '0');
  signal tdc_sample_latch_reg : signed(GC_TDC_OUTPUT - 1 downto 0)    := (others => '0');
  signal tdc_sample_pipe      : signed(GC_TDC_OUTPUT - 1 downto 0)    := (others => '0');
  signal tdc_center_pipe      : signed(GC_TDC_OUTPUT - 1 downto 0)    := (others => '0');
  signal tdc_sub_pipe         : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');
  signal dac_contrib_pipe     : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');
  signal combine_valid_pipe   : std_logic                             := '0';
  signal tdc_diff_pipe        : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');
  signal dac_contrib_pipe2    : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');
  signal combine_valid_pipe2  : std_logic                             := '0';

  -- TDC lowpass filter: IIR y[n] = y[n-1] + (x[n] - y[n-1]) / 16
  signal tdc_diff_filtered_reg : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');

  signal cic_input_tdc     : signed(C_MULTIBIT_WIDTH - 1 downto 0) := (others => '0');
  signal cic_input_valid   : std_logic                             := '0';
  signal cic_data_out_tdc  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out_tdc : std_logic;

  -- CDC using DCFIFO (more robust than toggle handshake)
  signal cic_fifo_wrreq   : std_logic                                    := '0';
  signal cic_fifo_rdreq   : std_logic                                    := '0';
  signal cic_fifo_q       : std_logic_vector(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal cic_fifo_rdempty : std_logic                                    := '1';
  signal cic_fifo_wrfull  : std_logic                                    := '0';
  signal cic_out_sys      : signed(GC_DATA_WIDTH - 1 downto 0)           := (others => '0');
  signal cic_valid_sys    : std_logic                                    := '0';

  signal final_cic_data  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal final_cic_valid : std_logic;

  -- Filter signals
  signal eq_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out : std_logic;
  signal lp_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out : std_logic;

  signal eq_mux_out   : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_mux_valid : std_logic;
  signal lp_mux_out   : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_mux_valid : std_logic;

  signal combined_data_out  : signed(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal combined_valid_out : std_logic                          := '0';

  constant C_FILTER_PRIME_COUNT : integer := 70;

  constant C_SWEEP_SETTLING_NORMAL : integer              := 10000;
  constant C_SWEEP_SETTLING_FAST   : integer              := 1000;
  constant C_DITHER_TIMEOUT_NORMAL : integer              := 4000000;
  constant C_DITHER_TIMEOUT_FAST   : integer              := 400000;
  signal   filter_prime_counter    : unsigned(7 downto 0) := (others => '0');
  signal   filter_primed           : std_logic            := '0';

  signal s_comparator_out_internal : std_logic;
  signal analog_in_mux             : std_logic;

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
  signal dac_muxed_reg         : std_logic    := '0';

  signal cal_enable_sys   : std_logic := '0';
  signal cal_enable_sync0 : std_logic := '0';
  signal cal_enable_sync1 : std_logic := '0';
  signal cal_enable_tdc   : std_logic := '0';

  signal cal_done_sync0 : std_logic := '0';
  signal cal_done_sync1 : std_logic := '0';
  signal cal_done_sys   : std_logic := '0';

  -- Boot sweep signals
  signal sweep_duty_counter : unsigned(7 downto 0) := (others => '0');
  signal sweep_period_cnt   : unsigned(7 downto 0) := (others => '0');
  signal sweep_found_cross  : std_logic            := '0';
  signal sweep_cross_duty   : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal sweep_init_integ   : signed(31 downto 0)  := (others => '0');
  signal sweep_dac_next     : std_logic            := '0';
  signal comp_sync0         : std_logic            := '0';
  signal comp_sync1         : std_logic            := '0';
  signal comp_prev          : std_logic            := '0';

  signal cl_watch_cnt : unsigned(15 downto 0) := (others => '0');

  -- TDC sample latch
  signal tdc_sample_latch : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal tdc_sample_ready : std_logic                          := '0';

  signal start_pulse_pi    : std_logic := '0';
  signal ref_sync2_prev_pi : std_logic := '0';

  -- TDC CDC signals
  signal tdc_out_hold      : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal dac_at_sample     : std_logic                          := '0';
  signal tdc_toggle        : std_logic                          := '0';
  signal tdc_toggle_sync0  : std_logic                          := '0';
  signal tdc_toggle_sync1  : std_logic                          := '0';
  signal tdc_toggle_sync2  : std_logic                          := '0';
  signal tdc_out_sys       : signed(GC_TDC_OUTPUT - 1 downto 0) := (others => '0');
  signal new_sample_sys    : std_logic                          := '0';
  signal tdc_overflow_sync : std_logic_vector(1 downto 0)       := (others => '0');
  signal tdc_lost_sync     : std_logic_vector(1 downto 0)       := (others => '0');

  signal ref_sync0 : std_logic := '0';
  signal ref_sync1 : std_logic := '0';
  signal ref_sync2 : std_logic := '0';
  signal reset_tdc : std_logic;

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

  dac_out_bit <= open_loop_dac_duty when GC_OPEN_LOOP else dac_out_ff;

  debug_tdc_out   <= tdc_out;
  debug_tdc_valid <= tdc_valid;

  s_comparator_out_internal <= comparator_in;
  analog_in_mux             <= s_comparator_out_internal;

  p_boot_dither : process(clk_tdc)
    -- Variables for immediate-update edge detection (required for correct timing)
    variable v_start_pulse    : std_logic;
    variable v_ref_sync2_prev : std_logic                  := '0';
    variable v_boot_counter   : integer range 0 to 5000001 := 0;
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
        sweep_cross_duty   <= to_unsigned(128, 8);
        sweep_init_integ   <= (others => '0');
        comp_sync0         <= '0';
        comp_sync1         <= '0';
        comp_prev          <= '0';
        sweep_dac_next     <= '0';
      else

        comp_sync0 <= analog_in_mux;
        comp_sync1 <= comp_sync0;

        v_start_pulse    := ref_sync2 and not v_ref_sync2_prev;
        v_ref_sync2_prev := ref_sync2;

        case boot_state is
          when ST_SWEEP =>
            closed_loop_en <= '0';
            cal_enable_sys <= '0';

            dac_boot_ff <= sweep_dac_next;

            -- Pre-compute next cycle DAC value (breaks timing path)
            if (sweep_period_cnt + 1) < sweep_duty_counter then
              sweep_dac_next <= '1';
            else
              sweep_dac_next <= '0';
            end if;

            -- Increment period counter (wraps at 255)
            sweep_period_cnt <= sweep_period_cnt + 1;

            -- When period completes, increment duty (if not found crossing yet)
            if sweep_period_cnt = 255 then
              if sweep_found_cross = '0' then
                -- Look for comparator crossing (comp '1' -> '0')
                if comp_prev = '1' and comp_sync1 = '0' then
                  sweep_found_cross <= '1';
                  sweep_cross_duty  <= sweep_duty_counter;

                  -- Calculate initial integrator value: (duty - 128) * 2^20
                  sweep_init_integ <= shift_left(resize(signed('0' & sweep_duty_counter) - 128, 32), 20);

                  if GC_SIM then
                    report "BOOT_SWEEP: Found comparator crossing at duty=" & integer'image(to_integer(sweep_duty_counter)) & "/255 (" & integer'image((to_integer(sweep_duty_counter) * 100) / 256) & "%) at " & time'image(now);
                  end if;
                end if;

                sweep_duty_counter <= sweep_duty_counter + 1;

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

              comp_prev <= comp_sync1;
            end if;

            if sweep_found_cross = '1' then
              v_boot_counter := v_boot_counter + 1;
              if (GC_FAST_SIM and v_boot_counter >= C_SWEEP_SETTLING_FAST) or (not GC_FAST_SIM and v_boot_counter >= C_SWEEP_SETTLING_NORMAL) then
                if GC_SIM then
                  report "BOOT_FSM: Transitioning ST_SWEEP -> ST_WAIT_FOR_START at " & time'image(now) & " with init_integ=" & integer'image(to_integer(sweep_init_integ));
                end if;
                boot_state     <= ST_WAIT_FOR_START;
                v_boot_counter := 0;
                dac_boot_ff    <= '0';
              end if;
            end if;

          when ST_DITHER =>
            closed_loop_en <= '0';
            cal_enable_sys <= '1';

            -- Dither around found crossing duty to ensure comparator crossings
            if v_start_pulse = '1' then
              sweep_period_cnt <= sweep_period_cnt + 1;

              -- PWM with dither: sweep_cross_duty / 2 ± 8
              if sweep_period_cnt(7) = '0' then
                -- Upper half of dither cycle: duty + 8
                if sweep_period_cnt(6 downto 0) < resize(shift_right(sweep_cross_duty, 1) + 8, 7) then
                  dac_boot_ff <= '1';
                else
                  dac_boot_ff <= '0';
                end if;
              else
                -- Lower half of dither cycle: duty - 8
                if sweep_period_cnt(6 downto 0) < resize(shift_right(sweep_cross_duty, 1) - 8, 7) then
                  dac_boot_ff <= '1';
                else
                  dac_boot_ff <= '0';
                end if;
              end if;

              if GC_SIM and (v_boot_counter mod 10000) = 0 then
                report "ST_DITHER_PWM: Dither around duty=" & integer'image(to_integer(sweep_cross_duty)) & " cnt=" & integer'image(to_integer(sweep_period_cnt)) & " at " & time'image(now);
              end if;
            end if;

            v_boot_counter := v_boot_counter + 1;
            if cal_done = '1' then
              if GC_SIM then
                report "BOOT_FSM: cal_done='1'! Transitioning ST_DITHER -> ST_WAIT_FOR_START at " & time'image(now) & " after " & integer'image(v_boot_counter) & " cycles";
              end if;
              boot_state     <= ST_WAIT_FOR_START;
              v_boot_counter := 0;
            elsif (GC_FAST_SIM and v_boot_counter >= C_DITHER_TIMEOUT_FAST) or (not GC_FAST_SIM and v_boot_counter >= C_DITHER_TIMEOUT_NORMAL) then
              if GC_SIM then
                report "BOOT_FSM: Counter timeout! Transitioning ST_DITHER -> ST_WAIT_FOR_START at " & time'image(now) & " (cal_done still '0', will use uncalibrated center!)";
              end if;
              boot_state     <= ST_WAIT_FOR_START;
              v_boot_counter := 0;
            end if;

          when ST_WAIT_FOR_START =>
            if v_start_pulse = '1' then
              if GC_SIM then
                report "BOOT_FSM: Transitioning ST_WAIT_FOR_START -> ST_CLOSED_LOOP at " & time'image(now) & " (cal_done=" & std_logic'image(cal_done) & ")";
              end if;
              boot_state     <= ST_CLOSED_LOOP;
              v_boot_counter := 0;
            end if;

          when ST_CLOSED_LOOP =>        -- @suppress "Dead state 'ST_CLOSED_LOOP': state does not have outgoing transitions"
            if closed_loop_en = '0' then
              closed_loop_en <= '1';
              v_boot_counter := 0;
              if GC_SIM then
                report "BOOT_FSM: Entered ST_CLOSED_LOOP, closed_loop_en='1' at " & time'image(now);
              end if;
            end if;

            if cal_done = '0' then
              if v_boot_counter < 5000000 then
                v_boot_counter := v_boot_counter + 1;
              end if;

              if v_boot_counter > 10000 then
                cal_enable_sys <= '1';
              end if;
            else
              cal_enable_sys <= '0';
            end if;

            if closed_loop_drive = '0' then
              if v_start_pulse = '1' then
                sweep_period_cnt <= sweep_period_cnt + 1;
                if sweep_period_cnt(7) = '0' then
                  if sweep_period_cnt(6 downto 0) < resize(shift_right(sweep_cross_duty, 1) + 8, 7) then
                    dac_boot_ff <= '1';
                  else
                    dac_boot_ff <= '0';
                  end if;
                else
                  if sweep_period_cnt(6 downto 0) < resize(shift_right(sweep_cross_duty, 1) - 8, 7) then
                    dac_boot_ff <= '1';
                  else
                    dac_boot_ff <= '0';
                  end if;
                end if;
              end if;
            end if;

          when others =>
            boot_state <= ST_SWEEP;
        end case;
      end if;
    end if;
  end process;

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
  -- early measurement window
  ref_phases(0) <= not ref_sync2;

  -- Sample Generation for Decimator (clk_tdc domain) - 3-STAGE PIPELINE
  -- Stage 1: Select TDC source and register inputs (no arithmetic)
  -- Stage 2: Compute TDC difference (tdc_sample - tdc_center)
  -- Stage 3: Add DAC contribution (dac_contrib + tdc_diff)
  -- This breaks the critical timing path at 400MHz into manageable pieces
  -- Pre-register control signals to break timing paths from tdc_quantizer
  p_control_preregister : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        cal_done_reg         <= '0';
        tdc_valid_reg        <= '0';
        tdc_sample_ready_reg <= '0';
        tdc_out_reg          <= (others => '0');
        tdc_sample_latch_reg <= (others => '0');
        dac_muxed_reg        <= '0';
      else
        -- Register all control signals one cycle early
        cal_done_reg         <= cal_done;
        tdc_valid_reg        <= tdc_valid;
        tdc_sample_ready_reg <= tdc_sample_ready;
        tdc_out_reg          <= tdc_out;
        tdc_sample_latch_reg <= tdc_sample_latch;

        -- Mux early to break timing path to dac_out_ff
        if use_closed_loop_tdc = '1' then
          dac_muxed_reg <= dac_integrator_ff;
        else
          dac_muxed_reg <= dac_boot_ff;
        end if;
      end if;
    end if;
  end process;

  p_tdc_cdc_src : process(clk_tdc)
    variable v_sample_count    : integer range 0 to 100000             := 0;
    variable v_ref_prev        : std_logic                             := '0';
    variable v_start_pulse     : std_logic                             := '0';
    variable v_dac_for_contrib : std_logic                             := '0'; -- DAC value to use for THIS sample's contribution
    -- C_DAC_AMPLITUDE scaled so that CIC 3/2 gain maps exactly to full Q15 range
    -- With 3/2 gain: ±21845 × 1.5 = ±32767.5 (full Q15 scale)
    -- This is the mathematically exact value for unity gain through the system
    constant C_DAC_AMPLITUDE   : signed(C_MULTIBIT_WIDTH - 1 downto 0) := to_signed(21845, C_MULTIBIT_WIDTH);

  begin
    -- TDC Scaling: now controlled via tdc_scale_2x input port
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdc_out_hold          <= (others => '0');
        dac_at_sample         <= '0';
        tdc_toggle            <= '0';
        cic_input_tdc         <= (others => '0');
        cic_input_valid       <= '0';
        tdc_sample_pipe       <= (others => '0');
        tdc_center_pipe       <= (others => '0');
        tdc_sub_pipe          <= (others => '0');
        dac_contrib_pipe      <= (others => '0');
        combine_valid_pipe    <= '0';
        tdc_diff_pipe         <= (others => '0');
        tdc_diff_filtered_reg <= (others => '0');
        dac_contrib_pipe2     <= (others => '0');
        combine_valid_pipe2   <= '0';
        v_ref_prev            := '0';
        v_sample_count        := 0;
      else
        -- ===================================================================
        -- STAGE 1: Select TDC source and register (minimize mux-to-register path)
        -- ===================================================================
        -- Detect ref_clock rising edge
        v_start_pulse := ref_sync2 and not v_ref_prev;
        v_ref_prev    := ref_sync2;

        -- Default: clear pipeline valid flags
        combine_valid_pipe  <= '0';
        combine_valid_pipe2 <= '0';

        -- Generate CIC input on every ref_clock edge when in closed loop
        -- TDC contribution is only added after calibration completes
        if v_start_pulse = '1' and closed_loop_en_tdc = '1' then
          -- ===============================================================
          -- STAGE 1: Register inputs (no arithmetic, just mux and register)
          -- ===============================================================

          -- FIRST: Capture DAC state for this sample's contribution
          -- Must be done BEFORE using v_dac_for_contrib for center selection
          -- The TDC measurement corresponds to the DAC that was active during it
          -- dac_at_sample holds the DAC from the previous start pulse
          v_dac_for_contrib := dac_at_sample; -- Use previous cycle's DAC
          dac_at_sample     <= dac_out_ff; -- Update for next cycle

          -- Select TDC value for this sample (simple mux, no arithmetic)
          -- Only use TDC after calibration is done; before that, use center (zero contribution)
          -- Use PRE-REGISTERED control signals to meet 400MHz timing
          -- 
          -- CRITICAL FIX: When no new TDC sample is available, HOLD the previous
          -- tdc_sample_pipe value instead of falling back to center. Falling back
          -- to center caused half the samples to have zero TDC contribution,
          -- reducing ENOB by ~1 bit.
          if cal_done_reg = '1' then
            if tdc_valid_reg = '1' then
              -- New TDC measurement available - use it
              tdc_sample_pipe <= tdc_out_reg;
              tdc_out_hold    <= tdc_out_reg;
            elsif tdc_sample_ready_reg = '1' then
              -- Latched TDC measurement available - use it
              tdc_sample_pipe <= tdc_sample_latch_reg;
              tdc_out_hold    <= tdc_sample_latch_reg;
              -- else: HOLD previous tdc_sample_pipe value (implicit latch behavior)
              -- Do NOT fall back to center - that causes 50% of samples to have zero TDC contribution
            end if;
          else
            -- Before calibration: use zero TDC contribution (sample = center => diff = 0)
            -- Use whichever center matches the current DAC state
            if v_dac_for_contrib = '0' then
              tdc_sample_pipe <= tdc_center_tdc_0;
              tdc_out_hold    <= tdc_center_tdc_0;
            else
              tdc_sample_pipe <= tdc_center_tdc_1;
              tdc_out_hold    <= tdc_center_tdc_1;
            end if;
          end if;

          -- Register TDC center for next stage (breaks timing path)
          -- Select appropriate center based on DAC state for this sample
          if v_dac_for_contrib = '0' then
            tdc_center_pipe <= tdc_center_tdc_0;
          else
            tdc_center_pipe <= tdc_center_tdc_1;
          end if;

          -- Pre-compute TDC difference in Stage 1 at full width
          -- Resize to C_MULTIBIT_WIDTH here moves sign-extension out of Stage 2 critical path
          -- Stage 2 only needs shift_left + conditional negate (no resize)
          tdc_sub_pipe <= resize(tdc_sample_pipe, C_MULTIBIT_WIDTH) - resize(tdc_center_pipe, C_MULTIBIT_WIDTH);

          -- Compute DAC contribution using the same v_dac_for_contrib (now correctly assigned above)
          if v_dac_for_contrib = '1' then
            dac_contrib_pipe <= C_DAC_AMPLITUDE;
          else
            dac_contrib_pipe <= -C_DAC_AMPLITUDE;
          end if;

          -- Mark pipeline stage 1 as valid
          combine_valid_pipe <= '1';
          tdc_toggle         <= not tdc_toggle;

        end if;

        -- ===================================================================
        -- STAGE 2: Apply scaling and optional negation (subtraction done in Stage 1)
        -- ===================================================================
        if combine_valid_pipe = '1' then
          -- Apply variable scaling to pre-computed difference
          -- Subtraction was done in Stage 1 to reduce this path's timing
          -- Use SYNCHRONIZED control signals to avoid CDC timing violations
          -- 
          -- TIMING OPTIMIZATION: shift_left already returns C_MULTIBIT_WIDTH, so outer resize is removed
          -- Conditional logic simplified to reduce mux depth
          if disable_tdc_contrib_tdc = '1' then
            tdc_diff_pipe <= (others => '0');
          else
            -- tdc_sub_pipe is already C_MULTIBIT_WIDTH - no resize needed
            -- Barrel shifter + conditional negate is the only combinational logic
            if negate_tdc_contrib_tdc = '1' then
              tdc_diff_pipe <= -shift_left(tdc_sub_pipe, to_integer(tdc_scale_shift_tdc));
            else
              tdc_diff_pipe <= shift_left(tdc_sub_pipe, to_integer(tdc_scale_shift_tdc));
            end if;
          end if;

          -- Carry forward DAC contribution to stage 3
          dac_contrib_pipe2   <= dac_contrib_pipe;
          combine_valid_pipe2 <= '1';
        end if;

        -- STAGE 3: IIR lowpass filter on TDC diff (reduces sample-to-sample noise)
        cic_input_valid <= '0';

        if combine_valid_pipe2 = '1' then
          tdc_diff_filtered_reg <= tdc_diff_filtered_reg + shift_right(tdc_diff_pipe - tdc_diff_filtered_reg, 6);

          cic_input_tdc   <= dac_contrib_pipe2 + tdc_diff_filtered_reg;
          cic_input_valid <= '1';

          if GC_SIM then
            v_sample_count := v_sample_count + 1;
            if v_sample_count <= 10 or (v_sample_count mod 5000) = 0 then
              report "SAMPLE_GEN: ref_edge #" & integer'image(v_sample_count) & " DAC=" & std_logic'image(dac_at_sample) & " tdc_diff=" & integer'image(to_integer(tdc_diff_pipe)) & " tdc_filt=" & integer'image(to_integer(tdc_diff_filtered_reg)) & " at " & time'image(now);
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  p_tdc_cdc_dst : process(clk_sys)
    variable v_toggle_prev : std_logic := '0';
    variable v_pulse       : std_logic;
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        new_sample_sys   <= '0';
        v_toggle_prev    := '0';
        tdc_toggle_sync0 <= '0';
        tdc_toggle_sync1 <= '0';
        tdc_toggle_sync2 <= '0';
        tdc_out_sys      <= (others => '0');
      else
        tdc_toggle_sync0 <= tdc_toggle;
        tdc_toggle_sync1 <= tdc_toggle_sync0;
        tdc_toggle_sync2 <= tdc_toggle_sync1;

        -- Edge detect and data capture in same cycle
        v_pulse        := tdc_toggle_sync2 xor v_toggle_prev;
        new_sample_sys <= v_pulse;
        v_toggle_prev  := tdc_toggle_sync2;

        if v_pulse = '1' then
          tdc_out_sys <= tdc_out_hold;
        end if;
      end if;
    end if;
  end process;

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
      GC_OUTPUT_WIDTH => GC_TDC_OUTPUT
    )

    port map(
      clk_sys          => clk_sys,
      clk_tdc          => clk_tdc,
      reset            => reset,
      analog_in        => analog_in_mux,
      ref_phases       => ref_phases,
      coarse_bias      => coarse_bias_tdc,
      invert_polarity  => '0',
      tdc_out          => tdc_out,
      tdc_valid        => tdc_valid,
      overflow         => tdc_overflow,
      lost_sample      => tdc_lost_sample,
      fine_at_comp_out => fine_at_comp_out,
      fine_valid_out   => fine_valid_out
    );

  p_coarse_bias_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      coarse_bias_s0 <= coarse_bias;
      if tdl_cal_use_cal = '1' then
        coarse_bias_tdc <= coarse_bias_cal;
      else
        coarse_bias_tdc <= coarse_bias_s0;
      end if;
    end if;
  end process;

  -- Adaptive TDC center tracking: first-order IIR filter for TWO centers (DAC=0 and DAC=1)
  -- Eliminates bimodal noise caused by DAC-dependent TDC offset

  -- DAC state tracker: ensures we know which DAC was active for each TDC measurement
  p_tdc_calibration : process(clk_tdc)
    variable v_tdc_sample    : signed(GC_TDC_OUTPUT - 1 downto 0);
    variable v_error_0       : signed(GC_TDC_OUTPUT + 8 - 1 downto 0);
    variable v_error_1       : signed(GC_TDC_OUTPUT + 8 - 1 downto 0);
    variable v_center_next_0 : signed(GC_TDC_OUTPUT + 8 - 1 downto 0);
    variable v_center_next_1 : signed(GC_TDC_OUTPUT + 8 - 1 downto 0);
    variable v_dac_state     : std_logic; -- DAC state when this TDC sample was taken
    variable v_total_samples : unsigned(9 downto 0) := (others => '0'); -- Total sample count for timeout
    impure function is_valid_sample(s : signed) return boolean is
    begin
      if not GC_SIM then
        return true;                    -- In synthesis, all signals are valid
      end if;
      -- synthesis translate_off
      for i in s'range loop
        if s(i) /= '0' and s(i) /= '1' then
          return false;
        end if;
      end loop;
      -- synthesis translate_on
      return true;
    end function;
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        cal_sample_cnt_0  <= (others => '0');
        cal_sample_cnt_1  <= (others => '0');
        cal_done          <= '0';
        tdc_center_tdc_0  <= (others => '0');
        tdc_center_tdc_1  <= (others => '0');
        tdc_center_acc_0  <= (others => '0');
        tdc_center_acc_1  <= (others => '0');
        tdc_center_init_0 <= '0';
        tdc_center_init_1 <= '0';
        v_total_samples   := (others => '0');

      else
        if tdc_valid = '1' and closed_loop_en_tdc = '1' then
          v_tdc_sample := signed(tdc_out);
          v_dac_state  := dac_integrator_ff;

          if is_valid_sample(v_tdc_sample) then

            if v_total_samples < to_unsigned(511, v_total_samples'length) then
              v_total_samples := v_total_samples + 1;
            end if;

            if v_dac_state = '0' then
              if tdc_center_init_0 = '0' then
                tdc_center_acc_0  <= shift_left(resize(v_tdc_sample, tdc_center_acc_0'length), 8);
                tdc_center_tdc_0  <= v_tdc_sample;
                tdc_center_init_0 <= '1';
                cal_sample_cnt_0  <= (others => '0');
              else
                v_error_0        := resize(v_tdc_sample, tdc_center_acc_0'length) - shift_right(tdc_center_acc_0, 8);
                v_center_next_0  := tdc_center_acc_0 + v_error_0;
                tdc_center_acc_0 <= v_center_next_0;
                tdc_center_tdc_0 <= resize(shift_right(v_center_next_0, 8), GC_TDC_OUTPUT);

                if cal_sample_cnt_0 < to_unsigned(255, cal_sample_cnt_0'length) then
                  cal_sample_cnt_0 <= cal_sample_cnt_0 + 1;
                end if;
              end if;

            else
              if tdc_center_init_1 = '0' then
                tdc_center_acc_1  <= shift_left(resize(v_tdc_sample, tdc_center_acc_1'length), 8);
                tdc_center_tdc_1  <= v_tdc_sample;
                tdc_center_init_1 <= '1';
                cal_sample_cnt_1  <= (others => '0');
              else
                v_error_1        := resize(v_tdc_sample, tdc_center_acc_1'length) - shift_right(tdc_center_acc_1, 8);
                v_center_next_1  := tdc_center_acc_1 + v_error_1;
                tdc_center_acc_1 <= v_center_next_1;
                tdc_center_tdc_1 <= resize(shift_right(v_center_next_1, 8), GC_TDC_OUTPUT);

                if cal_sample_cnt_1 < to_unsigned(255, cal_sample_cnt_1'length) then
                  cal_sample_cnt_1 <= cal_sample_cnt_1 + 1;
                end if;
              end if;
            end if;

            if cal_done = '0' then
              if cal_sample_cnt_0 >= to_unsigned(255, cal_sample_cnt_0'length) and cal_sample_cnt_1 >= to_unsigned(255, cal_sample_cnt_1'length) then
                cal_done <= '1';
              elsif v_total_samples >= to_unsigned(511, v_total_samples'length) then
                -- Fallback: copy initialized center to uninitialized one
                if tdc_center_init_0 = '1' and tdc_center_init_1 = '0' then
                  tdc_center_tdc_1  <= tdc_center_tdc_0;
                  tdc_center_acc_1  <= tdc_center_acc_0;
                  tdc_center_init_1 <= '1';
                elsif tdc_center_init_1 = '1' and tdc_center_init_0 = '0' then
                  tdc_center_tdc_0  <= tdc_center_tdc_1;
                  tdc_center_acc_0  <= tdc_center_acc_1;
                  tdc_center_init_0 <= '1';
                end if;
                cal_done <= '1';
              end if;
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  p_tdc_latch : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdc_sample_latch <= (others => '0');
        tdc_sample_ready <= '0';
      else
        if tdc_valid = '1' then
          tdc_sample_latch <= tdc_out;
          tdc_sample_ready <= '1';
        end if;

        if start_pulse_pi = '1' and tdc_sample_ready = '1' then
          tdc_sample_ready <= '0';
        end if;
      end if;
    end if;
  end process;

  p_sigma_delta : process(clk_tdc)
    variable v_start_falling : std_logic;
  begin
    if rising_edge(clk_tdc) then
      start_pulse_pi    <= ref_sync2 and not ref_sync2_prev_pi;
      v_start_falling   := not ref_sync2 and ref_sync2_prev_pi;
      ref_sync2_prev_pi <= ref_sync2;

      if reset_tdc = '1' then
        dac_integrator_ff <= '0';
        ref_sync2_prev_pi <= '0';
        start_pulse_pi    <= '0';
      else
        -- Direct comparator feedback: update at falling edge for phase coherence with TDC
        if v_start_falling = '1' and closed_loop_en_tdc = '1' then
          dac_integrator_ff <= comp_sync1;
        end if;
      end if;
    end if;
  end process;

  p_tdc_center_cdc : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        tdc_center_cal_0 <= (others => '0');
        tdc_center_cal_1 <= (others => '0');
      else
        tdc_center_cal_0 <= tdc_center_tdc_0;
        tdc_center_cal_1 <= tdc_center_tdc_1;
      end if;
    end if;
  end process;

  p_cal_done_cdc : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        cal_done_sync0 <= '0';
        cal_done_sync1 <= '0';
        cal_done_sys   <= '0';
      else
        cal_done_sync0 <= cal_done;
        cal_done_sync1 <= cal_done_sync0;
        cal_done_sys   <= cal_done_sync1;
      end if;
    end if;
  end process;

  tdc_valid_sys <= new_sample_sys;

  p_dac_bitstream_sync : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      dac_bitstream_sync0 <= dac_out_ff;
      dac_bitstream_sync1 <= dac_bitstream_sync0;
      dac_bitstream_sync2 <= dac_bitstream_sync1;

      if tdc_valid_sys = '1' then
        dac_bitstream_hold <= dac_bitstream_sync2;
      end if;
    end if;
  end process;

  p_closed_loop_en_sync : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      closed_loop_en_sync0 <= closed_loop_en;
      closed_loop_en_sync1 <= closed_loop_en_sync0;
      closed_loop_en_sync2 <= closed_loop_en_sync1;

      closed_loop_en_tdc <= closed_loop_en_sync2;

      cal_enable_sync0 <= cal_enable_sys;
      cal_enable_sync1 <= cal_enable_sync0;
      cal_enable_tdc   <= cal_enable_sync1;

      disable_tdc_contrib_sync0 <= disable_tdc_contrib;
      disable_tdc_contrib_tdc   <= disable_tdc_contrib_sync0;

      negate_tdc_contrib_sync0 <= negate_tdc_contrib;
      negate_tdc_contrib_tdc   <= negate_tdc_contrib_sync0;

      tdc_scale_shift_sync0 <= tdc_scale_shift;
      tdc_scale_shift_tdc   <= tdc_scale_shift_sync0;
    end if;
  end process;

  p_tdl_centering_cal : process(clk_tdc)
    constant C_THRESH_LOW  : unsigned(19 downto 0) := to_unsigned(24576 * 8, 20);
    constant C_THRESH_HIGH : unsigned(19 downto 0) := to_unsigned(40960 * 8, 20);
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        tdl_cal_done        <= '0';
        tdl_cal_sample_cnt  <= (others => '0');
        tdl_cal_fine_acc    <= (others => '0');
        tdl_cal_timeout_cnt <= (others => '0');
        coarse_bias_cal     <= C_DEFAULT_COARSE_BIAS;
        tdl_cal_use_cal     <= '0';
        tdl_settle_cnt      <= (others => '0');
        tdl_settling        <= '0';
        fine_at_comp_reg    <= (others => '0');
        fine_valid_reg      <= '0';
        tdl_cal_sum_reg     <= (others => '0');
        tdl_cal_sum_valid   <= '0';
        tdl_cal_last_sample <= '0';

      elsif cal_enable_tdc = '1' and tdl_cal_done = '0' then
        tdl_cal_use_cal     <= '1';
        tdl_cal_timeout_cnt <= tdl_cal_timeout_cnt + 1;

        fine_valid_reg   <= fine_valid_out;
        fine_at_comp_reg <= fine_at_comp_out;

        tdl_cal_sum_valid   <= '0';
        tdl_cal_last_sample <= '0';

        if tdl_settling = '1' then
          if tdl_settle_cnt > 0 then
            tdl_settle_cnt <= tdl_settle_cnt - 1;
          else
            tdl_settling <= '0';
          end if;
        else
          if fine_valid_reg = '1' then
            tdl_cal_sum_reg   <= tdl_cal_fine_acc + resize(fine_at_comp_reg, 20);
            tdl_cal_sum_valid <= '1';

            tdl_cal_fine_acc   <= tdl_cal_fine_acc + resize(fine_at_comp_reg, 20);
            tdl_cal_sample_cnt <= tdl_cal_sample_cnt + 1;

            if tdl_cal_sample_cnt = to_unsigned(C_TDL_CAL_SAMPLES - 1, 4) then
              tdl_cal_last_sample <= '1';
            end if;
          end if;
        end if;

        -- Pipeline stage 2: check if centered
        if tdl_cal_sum_valid = '1' and tdl_cal_last_sample = '1' then
          if tdl_cal_sum_reg >= C_THRESH_LOW and tdl_cal_sum_reg <= C_THRESH_HIGH then
            tdl_cal_done <= '1';

          elsif tdl_cal_sum_reg < C_THRESH_LOW then
            if coarse_bias_cal < 254 then
              coarse_bias_cal <= coarse_bias_cal + 1;
            end if;
            tdl_cal_fine_acc   <= (others => '0');
            tdl_cal_sample_cnt <= (others => '0');
            tdl_settle_cnt     <= to_unsigned(C_TDL_CAL_SETTLE - 1, 10);
            tdl_settling       <= '1';

          else
            if coarse_bias_cal > 1 then
              coarse_bias_cal <= coarse_bias_cal - 1;
            end if;
            tdl_cal_fine_acc   <= (others => '0');
            tdl_cal_sample_cnt <= (others => '0');
            tdl_settle_cnt     <= to_unsigned(C_TDL_CAL_SETTLE - 1, 10);
            tdl_settling       <= '1';
          end if;
        end if;

      elsif cal_enable_tdc = '0' then
        null;
      end if;
    end if;
  end process;

  p_sticky_handoff : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        use_closed_loop <= '0';
      elsif closed_loop_drive = '1' then
        use_closed_loop <= '1';
      end if;
    end if;
  end process;

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
        v_total_count := v_total_count + 1;
        if dac_bitstream_hold = '1' then
          v_dac_count := v_dac_count + 1;
        end if;

        if v_total_count = 2048 then
          v_window_count := v_window_count + 1;
          if GC_SIM and (v_window_count <= 2 or (v_window_count mod 4) = 0) then
            v_duty_pct := (v_dac_count * 100) / v_total_count;
            report "DAC_DUTY[" & integer'image(v_window_count) & "]: " & integer'image(v_dac_count) & "/2048 = " & integer'image(v_duty_pct) & "%";
          end if;
          v_dac_count    := 0;
          v_total_count  := 0;
        end if;
      end if;
    end if;
  end process;

  -- TDC Monitor: Capture TDC codes for sanity checking (parallel with main operation)
  p_tdc_monitor : process(clk_sys)
    variable v_mon_count : integer range 0 to 10000 := 0;
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
        tdc_mon_valid <= '0';

        if new_sample_sys = '1' and cal_done_sys = '1' then
          if v_mon_count >= 10000 then
            v_mon_count   := 0;
            tdc_mon_code  <= tdc_out_sys;
            if dac_bitstream_hold = '0' then
              tdc_mon_center <= tdc_center_cal_0;
              tdc_mon_diff   <= tdc_out_sys - tdc_center_cal_0;
            else
              tdc_mon_center <= tdc_center_cal_1;
              tdc_mon_diff   <= tdc_out_sys - tdc_center_cal_1;
            end if;
            tdc_mon_dac   <= dac_bitstream_hold;
            tdc_mon_valid <= '1';
          else
            v_mon_count := v_mon_count + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- CIC SINC3 Decimator (combines DAC+TDC before filtering)
  i_cic_multibit : entity work.cic_sinc3_decimator
    generic map(
      GC_INPUT_WIDTH  => C_MULTIBIT_WIDTH,
      GC_DECIMATION   => GC_DECIMATION,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk          => clk_tdc,
      reset        => reset_tdc,
      data_in      => '0',
      data_in_wide => cic_input_tdc,
      ce           => cic_input_valid,
      data_out     => cic_data_out_tdc,
      valid        => cic_valid_out_tdc
    );

  -- CDC using DCFIFO (more robust than toggle handshake)
  cic_fifo_wrreq <= cic_valid_out_tdc and not cic_fifo_wrfull;

  i_cic_cdc_fifo : dcfifo
    generic map(                        -- @suppress "Generic map uses default values. Missing optional actuals: lpm_hint, delay_rdusedw, and 12 more"
      lpm_width              => GC_DATA_WIDTH,
      lpm_numwords           => 16,
      lpm_widthu             => 4,
      lpm_showahead          => "ON",
      overflow_checking      => "ON",
      underflow_checking     => "ON",
      use_eab                => "ON",
      intended_device_family => "Agilex 5"
    )
    port map(
      wrclk     => clk_tdc,
      wrreq     => cic_fifo_wrreq,
      data      => cic_data_out_tdc,
      rdclk     => clk_sys,
      rdreq     => cic_fifo_rdreq,
      q         => cic_fifo_q,
      rdempty   => cic_fifo_rdempty,
      wrfull    => cic_fifo_wrfull,
      aclr      => reset,
      eccstatus => open,
      rdfull    => open,
      rdusedw   => open,
      wrempty   => open,
      wrusedw   => open
    );

  cic_fifo_rdreq <= not cic_fifo_rdempty;

  p_cic_fifo_read : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        cic_out_sys   <= (others => '0');
        cic_valid_sys <= '0';
      else
        if cic_fifo_rdreq = '1' then
          cic_out_sys   <= signed(cic_fifo_q);
          cic_valid_sys <= '1';
        else
          cic_valid_sys <= '0';
        end if;
      end if;
    end if;
  end process;

  -- Connect to downstream filters (EQ/LP in clk_sys domain)
  final_cic_data  <= std_logic_vector(cic_out_sys);
  final_cic_valid <= cic_valid_sys;

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

  eq_mux_out   <= final_cic_data when disable_eq_filter = '1' else eq_data_out;
  eq_mux_valid <= final_cic_valid when disable_eq_filter = '1' else eq_valid_out;

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

  lp_mux_out   <= eq_mux_out when disable_lp_filter = '1' else lp_data_out;
  lp_mux_valid <= eq_mux_valid when disable_lp_filter = '1' else lp_valid_out;

  -- Filter priming: FIR filters need ~64 samples to prime delay lines
  p_filter_prime : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        filter_prime_counter <= (others => '0');
        filter_primed        <= '0';
      else
        if lp_mux_valid = '1' and filter_primed = '0' then
          if filter_prime_counter >= to_unsigned(C_FILTER_PRIME_COUNT - 1, 8) then
            filter_primed <= '1';
          else
            filter_prime_counter <= filter_prime_counter + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- Bypass unprimed filters, then use full CIC/EQ/LP chain
  p_combine_output : process(clk_sys)
    variable v_lp_signed : signed(GC_DATA_WIDTH - 1 downto 0);
  begin
    if rising_edge(clk_sys) then
      if reset = '1' then
        combined_data_out  <= (others => '0');
        combined_valid_out <= '0';
      else
        combined_valid_out <= '0';

        if filter_primed = '0' then
          if cic_valid_sys = '1' then
            combined_data_out  <= cic_out_sys;
            combined_valid_out <= '1';
          end if;
        else
          if lp_mux_valid = '1' then
            v_lp_signed        := signed(lp_mux_out);
            combined_data_out  <= v_lp_signed;
            combined_valid_out <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;

  sample_data  <= std_logic_vector(combined_data_out);
  sample_valid <= combined_valid_out;

  adc_ready <= closed_loop_en and filter_primed;

  -- TDC Monitor outputs
  tdc_monitor_code   <= tdc_mon_code;
  tdc_monitor_center <= tdc_mon_center;
  tdc_monitor_diff   <= tdc_mon_diff;
  tdc_monitor_dac    <= tdc_mon_dac;
  tdc_monitor_valid  <= tdc_mon_valid;

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
        -- TIMEOUT FALLBACK: If no TDC valid after 100 start pulses (~50us),
        -- assume comparator-based mode and proceed with handoff anyway.
        -- This enables operation when TDC doesn't fire (static comparator).
        if (closed_loop_en = '1') and (tdc_valid = '1') and (cl_valid_seen = '0') then
          cl_valid_seen <= '1';
        elsif (closed_loop_en = '1') and (cl_valid_seen = '0') and (cl_watch_cnt >= 100) then
          -- Timeout: no TDC valid, force handoff (comparator mode)
          cl_valid_seen <= '1';
        end if;

        -- ======================================================================
        -- Step 2: Arm the handoff (one-shot) after the first closed-loop valid
        -- The DAC output mux will keep using boot dither until sniff completes
        -- ======================================================================
        if (closed_loop_en = '1') and (cl_valid_seen = '1') and (closed_loop_drive = '0') and (cl_switch_pend = '0') then
          cl_switch_pend <= '1';
        end if;

        -- ======================================================================
        -- Step 3: Execute the handoff on a Start edge (ensures phase alignment)
        -- ======================================================================
        if (cl_switch_pend = '1') and (v_start_pulse = '1') then
          closed_loop_drive <= '1';
          cl_switch_pend    <= '0';
          cl_watch_cnt      <= (others => '0'); -- Reset watchdog on handoff
        end if;

        -- ======================================================================
        -- Watchdog: Count Start edges while waiting for TDC samples
        -- ======================================================================
        -- During handoff wait (closed_loop_en=1 but closed_loop_drive=0):
        --   Count starts. If we hit 200 starts (~100us @ 2MHz), flag sticky error.
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
              cl_watch_cnt <= cl_watch_cnt + 1; -- Increment if no samples
            end if;
          end if;
        else
          cl_watch_cnt <= (others => '0');
        end if;
      end if;
    end if;
  end process;

  p_cdc_use_closed_loop : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        use_closed_loop_sync1 <= '0';
        use_closed_loop_tdc   <= '0';
      else
        use_closed_loop_sync1 <= use_closed_loop;
        use_closed_loop_tdc   <= use_closed_loop_sync1;
      end if;
    end if;
  end process;

  p_dac_output : process(clk_tdc)
  begin
    if rising_edge(clk_tdc) then
      if reset_tdc = '1' then
        dac_out_ff <= '0';
      else
        dac_out_ff <= dac_muxed_reg;
      end if;
    end if;
  end process;

  coarse_bias <= C_DEFAULT_COARSE_BIAS;

end architecture;
