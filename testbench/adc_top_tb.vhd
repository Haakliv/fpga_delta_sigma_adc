library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;
use fpga_lib.clk_rst_pkg.all;

entity adc_top_tb is
    generic(
        runner_cfg         : string;
        GC_ADC_TYPE        : string  := "tdc"; -- "tdc" or "rc"
        GC_TB_SIGNAL_TYPE  : integer := 0; -- 0=sine, 1=DC, 2=ramp, 3=square
        GC_TB_AMPLITUDE    : real    := 0.25;
        GC_TB_FREQUENCY_HZ : real    := 1000.0;
        GC_TB_DC_LEVEL     : real    := 0.5;
        GC_OPEN_LOOP       : boolean := false
    );
end entity;

architecture behavioral of adc_top_tb is

    constant C_CLK_SYS_PERIOD : time     := 10 ns;
    constant C_CLK_TDC_PERIOD : time     := 2.474 ns;
    constant C_REF_PERIOD     : time     := 20 ns;
    constant C_DATA_WIDTH     : positive := 16;
    constant C_TDC_WIDTH      : positive := 16;
    signal clk_sys   : std_logic := '0';
    signal clk_tdc   : std_logic := '0';
    signal reset     : std_logic := '1';
    signal ref_clock : std_logic := '0';

    signal sample_data  : std_logic_vector(C_DATA_WIDTH - 1 downto 0);
    signal sample_valid : std_logic;

    signal dac_out_bit      : std_logic;
    signal s_comparator_out : std_logic_vector(0 downto 0);

    signal analog_voltage_p : real := 0.0;
    signal analog_voltage_n : real := 0.0;

    -- RC filter internal signals (for ref_clock-rate optimization)
    signal duty_count_reg   : integer range 0 to 255 := 0;
    signal duty_period_done : std_logic              := '0';

    -- Pad-level signals (after threshold conversion)
    signal pad_p : std_logic := '0';    -- P-pad digital (comparator P input)
    signal pad_n : std_logic := '0';    -- N-pad digital (comparator N threshold)

    -- RC ADC comparator signal - simple instantaneous analog comparison
    signal rc_comparator_out : std_logic := '0';

    -- Debug signals for TDC characterization
    signal debug_tdc_out   : signed(C_TDC_WIDTH - 1 downto 0);
    signal debug_tdc_valid : std_logic;

    -- Test control
    signal sim_finished    : boolean   := false;
    signal tdc_char_done   : boolean   := false; -- Early termination for TDC characterization

    -- Sample statistics (updated by p_monitor, checked by test process)
    signal sample_min_value : integer := 0;
    signal sample_max_value : integer := 0;
    signal sample_count_sig : integer := 0;
    signal sample_sum_value : integer := 0; -- Sum of all samples for averaging

begin

    g_tdc_adc : if GC_ADC_TYPE = "tdc" generate
        i_dut_tdc : entity work.tdc_adc_top
            generic map(
                GC_DECIMATION => 64,
                GC_DATA_WIDTH => C_DATA_WIDTH,
                GC_TDC_OUTPUT => C_TDC_WIDTH,
                GC_SIM        => false,
                GC_FAST_SIM   => true,
                GC_OPEN_LOOP  => GC_OPEN_LOOP
            )
            port map(
                clk_sys             => clk_sys,
                clk_tdc             => clk_tdc,
                reset               => reset,
                ref_clock           => ref_clock,
                comparator_in       => s_comparator_out(0),
                dac_out_bit         => dac_out_bit,
                trigger_enable      => '1',
                open_loop_dac_duty  => '0',  -- Closed-loop mode (default)

                sample_data         => sample_data,
                sample_valid        => sample_valid,
                debug_tdc_out       => debug_tdc_out,
                debug_tdc_valid     => debug_tdc_valid,
                tdc_monitor_code    => open,
                tdc_monitor_center  => open,
                tdc_monitor_diff    => open,
                tdc_monitor_dac     => open,
                tdc_monitor_valid   => open,
                disable_tdc_contrib => '0',
                disable_eq_filter   => '0',
                disable_lp_filter   => '0',
                negate_tdc_contrib  => '0',
                tdc_scale_shift     => "000",
                adc_ready           => open
            );
    end generate;

    g_rc_adc : if GC_ADC_TYPE = "rc" generate
        i_dut_rc : entity work.rc_adc_top
            generic map(
                GC_DECIMATION => 64,
                GC_DATA_WIDTH => C_DATA_WIDTH
            )
            port map(
                clk            => ref_clock,
                clk_fast       => clk_tdc,
                clk_sys        => clk_sys,
                reset          => reset,
                analog_in      => rc_comparator_out,
                dac_out        => dac_out_bit,
                trigger_enable => '1',
                sample_data    => sample_data,
                sample_valid   => sample_valid
            );
        debug_tdc_out   <= (others => '0');
        debug_tdc_valid <= '0';
    end generate;

    p_behavioral_comparator : process(pad_p, pad_n)
    begin
        if pad_p = '1' and pad_n = '0' then
            s_comparator_out(0) <= '1';
        elsif pad_p = '0' and pad_n = '1' then
            s_comparator_out(0) <= '0';
        else
            s_comparator_out(0) <= pad_p xor pad_n;
        end if;
    end process;

    p_clk_sys : process
        variable v_seed1, v_seed2 : positive := 1;
        variable v_rand           : real;
        variable v_init_delay     : time;
    begin
        -- Random initial phase (0 to 5ns) to desynchronize from other clocks
        uniform(v_seed1, v_seed2, v_rand);
        v_init_delay := v_rand * 5.0 ns;
        wait for v_init_delay;

        while not sim_finished loop
            clk_sys <= '0';
            wait for C_CLK_SYS_PERIOD / 2;
            clk_sys <= '1';
            wait for C_CLK_SYS_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_clk_tdc : process
        variable v_seed1, v_seed2 : positive := 2;
        variable v_rand           : real;
        variable v_init_delay     : time;
    begin
        -- Random initial phase (0 to 5ns) to desynchronize from other clocks
        uniform(v_seed1, v_seed2, v_rand);
        v_init_delay := v_rand * 5.0 ns;
        wait for v_init_delay;

        while not sim_finished loop
            clk_tdc <= '0';
            wait for C_CLK_TDC_PERIOD / 2;
            clk_tdc <= '1';
            wait for C_CLK_TDC_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_ref_clock : process
        variable v_seed1, v_seed2 : positive := 3;
        variable v_rand           : real;
        variable v_init_delay     : time;
    begin
        -- Random initial phase (0 to 100ns) to desynchronize from other clocks
        uniform(v_seed1, v_seed2, v_rand);
        v_init_delay := v_rand * 100.0 ns;
        wait for v_init_delay;

        while not sim_finished loop
            ref_clock <= '0';
            wait for C_REF_PERIOD / 2;

            ref_clock <= '1';
            wait for C_REF_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_reset : process
        variable v_seed1, v_seed2 : positive := 4;
        variable v_rand           : real;
        variable v_reset_delay    : time;
    begin
        uniform(v_seed1, v_seed2, v_rand);
        v_reset_delay := 200 ns + v_rand * 200 ns;

        reset <= '1';
        wait for v_reset_delay;
        reset <= '0';
        wait;
    end process;

    p_analog_voltage_generator : process
        variable v_time_s : real := 0.0;
        constant C_PI     : real := 3.14159265359;
        constant C_VBANK  : real := 1.25;
    begin
        wait until reset = '0';
        report "Analog voltage generator started! Signal type=" & integer'image(GC_TB_SIGNAL_TYPE);

        loop
            wait until rising_edge(ref_clock);
            v_time_s := v_time_s + real(C_REF_PERIOD / 1 ns) * 1.0e-9;

            case GC_TB_SIGNAL_TYPE is
                when 0 =>
                    analog_voltage_p <= (GC_TB_DC_LEVEL + GC_TB_AMPLITUDE * sin(2.0 * C_PI * GC_TB_FREQUENCY_HZ * v_time_s)) * C_VBANK;
                when 1 =>
                    analog_voltage_p <= GC_TB_DC_LEVEL * C_VBANK;
                when 2 =>
                    analog_voltage_p <= (GC_TB_DC_LEVEL + GC_TB_AMPLITUDE * (2.0 * ((GC_TB_FREQUENCY_HZ * v_time_s) mod 1.0) - 1.0)) * C_VBANK;
                when 3 =>
                    if ((GC_TB_FREQUENCY_HZ * v_time_s) mod 1.0) < 0.5 then
                        analog_voltage_p <= (GC_TB_DC_LEVEL + GC_TB_AMPLITUDE) * C_VBANK;
                    else
                        analog_voltage_p <= (GC_TB_DC_LEVEL - GC_TB_AMPLITUDE) * C_VBANK;
                    end if;
                when others =>
                    analog_voltage_p <= 0.0;
            end case;
        end loop;
    end process;

    p_duty_counter : process(clk_tdc, reset)
        constant C_TICKS_PER_PERIOD : integer               := 8;
        variable v_tick_count       : integer range 0 to 15 := 0;
        variable v_high_count       : integer range 0 to 15 := 0;
    begin
        if reset = '1' then
            v_tick_count     := 0;
            v_high_count     := 0;
            duty_count_reg   <= 0;
            duty_period_done <= '0';
        elsif rising_edge(clk_tdc) then
            if dac_out_bit = '1' then
                v_high_count := v_high_count + 1;
            end if;
            v_tick_count := v_tick_count + 1;

            if v_tick_count >= C_TICKS_PER_PERIOD then
                duty_count_reg   <= v_high_count;
                duty_period_done <= '1';
                v_tick_count     := 0;
                v_high_count     := 0;
            else
                duty_period_done <= '0';
            end if;
        end if;
    end process;

    p_rc_filter_duty_avg : process(clk_tdc, reset)
        constant C_DAC_HIGH_V       : real    := 1.25;
        constant C_TICKS_PER_PERIOD : integer := 8;
        constant C_ALPHA            : real    := 0.9802;
        variable v_vn               : real      := 0.625;
        variable v_vdac_avg         : real      := 0.0;
        variable v_prev_done        : std_logic := '0';
    begin
        if reset = '1' then
            v_vn             := 0.625;
            analog_voltage_n <= 0.625;
            v_prev_done      := '0';
        elsif rising_edge(clk_tdc) then
            if duty_period_done = '1' and v_prev_done = '0' then
                v_vdac_avg       := real(duty_count_reg) / real(C_TICKS_PER_PERIOD) * C_DAC_HIGH_V;
                v_vn             := C_ALPHA * v_vn + (1.0 - C_ALPHA) * v_vdac_avg;
                analog_voltage_n <= v_vn;
            end if;
            v_prev_done := duty_period_done;
        end if;
    end process;

    p_comparator_model : process
        constant C_SWEEP_END_TIME : time      := 70 us;
        variable v_verr           : real      := 0.0;
        variable v_cross_ns       : real      := 250.0;
        variable v_pulse_count    : integer   := 0;
        variable v_in_sweep       : boolean   := true;
        variable v_final          : std_logic := '0';
    begin
        pad_p <= '0';
        pad_n <= '1';
        wait until reset = '0';
        wait until rising_edge(ref_clock);

        loop
            v_verr        := analog_voltage_p - analog_voltage_n;
            v_pulse_count := v_pulse_count + 1;
            v_in_sweep    := now < C_SWEEP_END_TIME;

            if v_in_sweep then
                if v_verr > 0.0 then
                    pad_p <= '1';
                    pad_n <= '0';
                else
                    pad_p <= '0';
                    pad_n <= '1';
                end if;

                wait until rising_edge(ref_clock);
            else
                if v_verr > 0.0 then
                    v_final := '1';
                else
                    v_final := '0';
                end if;

                v_cross_ns := 8.0 + (3.0 * v_verr);

                if v_cross_ns < 2.0 then
                    v_cross_ns := 2.0;
                elsif v_cross_ns > 18.0 then
                    v_cross_ns := 18.0;
                end if;

                pad_p <= not v_final;
                pad_n <= v_final;

                pad_p <= transport v_final after v_cross_ns * 1 ns;
                pad_n <= transport not v_final after v_cross_ns * 1 ns;

                wait until rising_edge(ref_clock);
            end if;
        end loop;
    end process;

    -- RC ADC comparator behavioral model
    -- Models 1st-order delta-sigma with RC integrator
    -- Physical circuit: LVDS+ = Vin, LVDS- = RC integrator node
    -- The RC node integrates the difference between Vin and DAC output
    -- Real hardware: R=1kohm, C=1nF -> tau = 1us
    p_rc_comparator_model : process(clk_sys, reset)
        constant C_DAC_HIGH_V : real := 1.25;      -- DAC output voltage when dac_out_bit='1'
        constant C_RC_TAU     : real := 0.000001;  -- RC time constant: 1kohm * 1nF = 1us
        constant C_CLK_PERIOD : real := 0.00000001; -- clk_sys period in seconds (10ns = 100MHz)
        constant C_ALPHA      : real := C_CLK_PERIOD / C_RC_TAU; -- Integration rate
        variable v_rc_node    : real := 0.625;     -- RC integrator node voltage (starts at midscale)
        variable v_dac_voltage : real := 0.0;
    begin
        if reset = '1' then
            rc_comparator_out <= '0';
            v_rc_node         := 0.625;
        elsif rising_edge(clk_sys) then
            -- DAC drives through resistor to the RC node
            if dac_out_bit = '1' then
                v_dac_voltage := C_DAC_HIGH_V;
            else
                v_dac_voltage := 0.0;
            end if;

            -- RC node voltage: exponential approach toward DAC voltage
            -- dV/dt = (Vdac - Vrc) / tau
            -- Vrc(n+1) = Vrc(n) + (Vdac - Vrc) * dt/tau
            v_rc_node := v_rc_node + (v_dac_voltage - v_rc_node) * C_ALPHA;
            
            -- Comparator: LVDS+ (Vin) vs LVDS- (RC node)
            -- Output '1' when Vin > RC node voltage
            if analog_voltage_p > v_rc_node then
                rc_comparator_out <= '1';
            else
                rc_comparator_out <= '0';
            end if;
        end if;
    end process;

    p_main : process
        function voltage_to_q15(v_normalized : real) return integer is
            variable v_q15 : integer;
        begin
            -- v_normalized is 0.0 to 1.0, mapping to 0mV to 1250mV
            -- Center is 0.5 (625mV) which maps to Q15 = 0
            v_q15 := integer((v_normalized - 0.5) * 65536.0);
            -- Saturate to Q15 range
            if v_q15 > 32767 then
                return 32767;
            elsif v_q15 < -32768 then
                return -32768;
            else
                return v_q15;
            end if;
        end function;

        function q_to_mv(q_value : integer) return integer is
            variable v_scaled : integer;
        begin
            v_scaled := (q_value * 625) / 32768 + 625;
            if v_scaled < 0 then
                return 0;
            elsif v_scaled > 1250 then
                return 1250;
            else
                return v_scaled;
            end if;
        end function;

        procedure wait_for_samples(signal   clk     : std_logic;
                                   signal   vld     : std_logic;
                                   constant N       : natural;
                                   constant timeout : time) is
            variable v_cnt              : unsigned(15 downto 0) := (others => '0');
            variable v_start_time       : time;
            variable v_last_sample_time : time;
            variable v_progress_timer   : time;
        begin
            v_start_time       := now;
            v_last_sample_time := now;
            v_progress_timer   := now;

            while to_integer(v_cnt) < N loop
                wait until rising_edge(clk);

                if vld = '1' then
                    v_cnt              := v_cnt + 1;
                    v_last_sample_time := now;
                    if (to_integer(v_cnt) mod 10) = 0 or to_integer(v_cnt) = N then
                        info("Progress: Collected sample " & integer'image(to_integer(v_cnt)) & " of " & integer'image(N) & " at " & time'image(now));
                    end if;
                end if;

                if now - v_progress_timer > 500 us then
                    info("PROGRESS CHECK: " & integer'image(to_integer(v_cnt)) & " samples collected so far at " & time'image(now) & " (last sample at " & time'image(v_last_sample_time) & ", gap=" & time'image(now - v_last_sample_time) & ")");
                    v_progress_timer := now;
                end if;

                if now - v_start_time > timeout then
                    error("GLOBAL TIMEOUT waiting for samples! Only got " & integer'image(to_integer(v_cnt)) & " of " & integer'image(N) & " - Last sample at: " & time'image(v_last_sample_time));
                    exit;
                end if;
            end loop;

            info("Sample collection completed: " & integer'image(to_integer(v_cnt)) & " samples in " & time'image(now - v_start_time));
        end procedure;

        procedure collect_and_verify_samples(signal   clk       : std_logic;
                                             signal   vld       : std_logic;
                                             constant N         : natural;
                                             constant timeout   : time;
                                             constant test_name : string) is
            variable v_mean_mv      : integer;
            variable v_variation_mv : integer;
            variable v_expected_mv  : integer;
            variable v_error_mv     : integer;
            variable v_min_mv       : integer;
            variable v_max_mv       : integer;
            variable v_avg_q        : integer;
            variable v_expected_q   : integer;
            variable v_tolerance_q  : integer;
            variable v_error_q      : integer;
        begin
            wait_for_samples(clk, vld, N, timeout);
            wait for C_CLK_SYS_PERIOD * 10;
            check(sample_min_value /= 0 or sample_max_value /= 0,
                  "OUTPUT STUCK AT ZERO in " & test_name & ": " & "All " & integer'image(sample_count_sig) & " samples are zero! " & "ADC is not responding to input voltage. " & "Likely causes: " & "1) TDC not generating bitstream " & "2) CIC input stuck " & "3) Servo or mV conversion broken");

            -- Check if output is stuck at constant value (only fail for AC signals)
            -- For DC inputs (GC_TB_SIGNAL_TYPE=1), constant output is expected and correct
            if sample_min_value = sample_max_value and sample_count_sig > 10 and GC_TB_SIGNAL_TYPE /= 1 then
                check(false,
                      "OUTPUT STUCK AT CONSTANT in " & test_name & ": " & "All " & integer'image(sample_count_sig) & " samples equal " & integer'image(sample_min_value) & ". " & "No dynamic range detected for AC signal! Check if: " & "1) Loop is saturated or stuck " & "2) CIC decimation removing all signal variations");
            elsif GC_TB_SIGNAL_TYPE = 1 then
                -- DC INPUT TEST: Verify voltage accuracy in Q15 format
                v_avg_q := sample_sum_value / sample_count_sig; -- Average in Q15

                -- Calculate expected Q15 value from DC level
                -- voltage_to_q15 maps: 0.0 -> -32768, 0.5 -> 0, 1.0 -> +32767
                v_expected_q := voltage_to_q15(GC_TB_DC_LEVEL);

                -- Define tolerance in Q15 units
                -- TDC ADC: +/-20mV corresponds to +/-1006 Q15 units (20 * 32768 / 650)
                -- RC ADC: +/-25mV corresponds to +/-1260 Q15 units (25 * 32768 / 650)
                -- Note: Behavioral simulation has higher error at voltage extremes (near 0 or 1250mV)
                -- due to comparator/RC filter model limitations. Hardware is more accurate.
                if GC_ADC_TYPE = "rc" then
                    v_tolerance_q := 1260; -- +/-25mV equivalent in Q15
                else
                    v_tolerance_q := 1050; -- +/-20mV equivalent in Q15 (behavioral model has ~18mV error at extremes)
                end if;
                v_error_q := abs (v_avg_q - v_expected_q);

                -- Convert to mV for reporting only (not for comparison)
                v_min_mv       := q_to_mv(sample_min_value);
                v_max_mv       := q_to_mv(sample_max_value);
                v_mean_mv      := q_to_mv(v_avg_q);
                v_expected_mv  := integer(GC_TB_DC_LEVEL * 1250.0);
                v_variation_mv := v_max_mv - v_min_mv;
                v_error_mv     := abs (v_mean_mv - v_expected_mv);

                -- Report both Q-format and mV values
                info("DC INPUT TEST (Q15): Raw Q15 range " & integer'image(sample_min_value) & " to " & integer'image(sample_max_value) & " (avg=" & integer'image(v_avg_q) & " from " & integer'image(sample_count_sig) & " samples)");
                info("DC INPUT TEST (mV):  Converted range " & integer'image(v_min_mv) & "-" & integer'image(v_max_mv) & "mV (variation=" & integer'image(v_variation_mv) & "mV, avg=" & integer'image(v_mean_mv) & "mV)");
                info("DC ACCURACY (Q15): Expected=" & integer'image(v_expected_q) & ", Measured=" & integer'image(v_avg_q) & ", Error=" & integer'image(v_error_q) & " (tolerance=+/-" & integer'image(v_tolerance_q) & ")");
                info("DC ACCURACY (mV):  Expected=" & integer'image(v_expected_mv) & "mV, Measured=" & integer'image(v_mean_mv) & "mV, Error=" & integer'image(v_error_mv) & "mV");

                -- Check voltage accuracy in Q15 format
                check(v_error_q <= v_tolerance_q,
                      "DC VOLTAGE ERROR EXCEEDS TOLERANCE in " & test_name & ": " & "Expected Q15=" & integer'image(v_expected_q) & " (+/-" & integer'image(v_tolerance_q) & "), " & "Measured Q15=" & integer'image(v_avg_q) & ", " & "Error = " & integer'image(v_error_q) & ". " & "Input voltage: " & integer'image(integer(GC_TB_DC_LEVEL * 1250.0)) & "mV.");

                info("DC VOLTAGE ACCURACY CHECK PASSED: Error Q15=" & integer'image(v_error_q) & " within +/-" & integer'image(v_tolerance_q) & " tolerance");
            else
                -- For AC signals (sine, square, ramp), just verify dynamic range exists
                -- The ADC outputs absolute voltage (0-1200mV), not differential amplitude
                -- So for a 0.25 amplitude sine at 0.5 DC offset (300mV swing centered at 600mV),
                -- the output range will be close to full scale as the loop tracks the input
                v_min_mv       := q_to_mv(sample_min_value);
                v_max_mv       := q_to_mv(sample_max_value);
                v_variation_mv := v_max_mv - v_min_mv; -- Peak-to-peak in mV

                info("AC INPUT TEST (Q15): Raw Q15 range " & integer'image(sample_min_value) & " to " & integer'image(sample_max_value));
                info("AC INPUT TEST (mV):  Peak-to-peak range = " & integer'image(v_variation_mv) & " mV (" & integer'image(v_min_mv) & "-" & integer'image(v_max_mv) & "mV)");

                -- Just verify we have reasonable dynamic range (>30mV) to confirm loop is tracking
                -- Note: Behavioral simulation produces less swing than real hardware
                check(v_variation_mv > 30,
                      "AC OUTPUT HAS NO DYNAMIC RANGE in " & test_name & ": " & "Only " & integer'image(v_variation_mv) & "mV p-p detected " & "(AMPLITUDE=" & real'image(GC_TB_AMPLITUDE) & ", FREQUENCY=" & real'image(GC_TB_FREQUENCY_HZ) & "Hz). " & "Delta-Sigma loop may be stuck!");

                info("AC VOLTAGE TRACKING CHECK PASSED: Measured " & integer'image(v_variation_mv) & "mV p-p dynamic range");
            end if;

            info("OUTPUT RANGE CHECK PASSED for " & test_name & ": Q15 min=" & integer'image(sample_min_value) & ", max=" & integer'image(sample_max_value) & ", count=" & integer'image(sample_count_sig));
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("basic_test") then
                info("==========================================================");
                info("Running TDC ADC basic test with external analog input");
                info("Signal type: " & integer'image(GC_TB_SIGNAL_TYPE));
                info("DC level: " & real'image(GC_TB_DC_LEVEL));
                info("Amplitude: " & real'image(GC_TB_AMPLITUDE));
                info("Frequency: " & real'image(GC_TB_FREQUENCY_HZ) & " Hz");
                info("==========================================================");

                -- DYNAMIC READY DETECTION: Wait for first sample_valid pulse
                -- This indicates calibration is complete and system is producing valid output
                info("Waiting for system ready (first sample_valid)...");
                wait until rising_edge(clk_sys) and sample_valid = '1' for 500 us;
                if sample_valid /= '1' then
                    error("Timeout waiting for first sample_valid - calibration may have failed!");
                end if;
                info("System ready detected at " & time'image(now));

                if GC_TB_SIGNAL_TYPE = 1 then
                    if GC_ADC_TYPE = "rc" then
                        info("Collecting 125 output samples for RC ADC DC accuracy test (120 settling + 5 measurement)...");
                        collect_and_verify_samples(clk_sys, sample_valid, 125, 200 us, "basic_test");
                    else
                        info("Collecting 75 output samples for TDC ADC DC accuracy test (70 settling + 5 measurement)...");
                        collect_and_verify_samples(clk_sys, sample_valid, 75, 2500 us, "basic_test");
                    end if;
                else
                    if GC_ADC_TYPE = "rc" then
                        info("Collecting 220 output samples for RC ADC AC test (120 settling + 100 measurement)...");
                        collect_and_verify_samples(clk_sys, sample_valid, 220, 500 us, "basic_test");
                    else
                        info("Collecting 100 output samples for TDC ADC AC test (includes filter priming)...");
                        collect_and_verify_samples(clk_sys, sample_valid, 100, 25 ms, "basic_test");
                    end if;
                end if;

                info("==========================================================");
                info("Basic test completed successfully");
                info("==========================================================");

            end if;
        end loop;

        wait for C_CLK_SYS_PERIOD * 10;
        sim_finished <= true;
        test_runner_cleanup(runner);
        wait;
    end process;

    p_monitor : process(clk_sys)
        constant C_SETTLE_SAMPLES_TDC : integer := 70;
        constant C_SETTLE_SAMPLES_RC  : integer := 120;
        variable v_settle_limit       : integer;
        variable v_sample_count       : integer := 0;
        variable v_stats_count        : integer := 0;
        variable v_min_value          : integer := integer'high;
        variable v_max_value          : integer := integer'low;
        variable v_sum                : integer := 0;
        variable v_sample_value       : integer;
    begin
        if rising_edge(clk_sys) then
            if GC_ADC_TYPE = "rc" then
                v_settle_limit := C_SETTLE_SAMPLES_RC;
            else
                v_settle_limit := C_SETTLE_SAMPLES_TDC;
            end if;

            if reset = '1' then
                v_sample_count   := 0;
                v_stats_count    := 0;
                v_min_value      := integer'high;
                v_max_value      := integer'low;
                v_sum            := 0;
                sample_min_value <= 0;
                sample_max_value <= 0;
                sample_count_sig <= 0;
                sample_sum_value <= 0;
            elsif sample_valid = '1' then
                v_sample_count := v_sample_count + 1;
                v_sample_value := to_integer(signed(sample_data));

                if v_sample_count > v_settle_limit then
                    v_stats_count := v_stats_count + 1;
                    v_sum         := v_sum + v_sample_value;

                    if v_sample_value < v_min_value then
                        v_min_value := v_sample_value;
                    end if;
                    if v_sample_value > v_max_value then
                        v_max_value := v_sample_value;
                    end if;
                end if;

                if v_sample_count > v_settle_limit and v_stats_count > 0 then
                    sample_min_value <= v_min_value;
                    sample_max_value <= v_max_value;
                    sample_count_sig <= v_stats_count;
                    sample_sum_value <= v_sum;
                end if;

                if v_sample_count <= 5 then
                    info("Sample " & integer'image(v_sample_count) & " [SETTLING]: " & integer'image(v_sample_value));
                elsif v_sample_count = v_settle_limit + 1 then
                    info("=== SETTLING COMPLETE (" & integer'image(v_settle_limit) & " samples discarded) ===");
                end if;
            end if;
        end if;
    end process;

    g_tdc_monitor : if GC_OPEN_LOOP generate
        p_tdc_monitor : process(clk_tdc)
            variable v_tdc_count          : integer := 0;
            variable v_tdc_sum            : integer := 0;
            variable v_tdc_sum_sq         : integer := 0;
            variable v_tdc_min            : integer := integer'high;
            variable v_tdc_max            : integer := integer'low;
            variable v_tdc_sat_count      : integer := 0;
            variable v_tdc_value          : integer;
            variable v_mean               : integer;
            variable v_variance           : integer;
            variable v_sat_pct            : integer;
            constant C_CHAR_SAMPLE_TARGET : integer := 2000;
        begin
            if rising_edge(clk_tdc) then
                if reset = '1' then
                    v_tdc_count     := 0;
                    v_tdc_sum       := 0;
                    v_tdc_sum_sq    := 0;
                    v_tdc_min       := integer'high;
                    v_tdc_max       := integer'low;
                    v_tdc_sat_count := 0;
                    tdc_char_done   <= false;
                elsif debug_tdc_valid = '1' and not tdc_char_done then
                    v_tdc_value  := to_integer(debug_tdc_out);
                    v_tdc_count  := v_tdc_count + 1;
                    v_tdc_sum    := v_tdc_sum + v_tdc_value;
                    v_tdc_sum_sq := v_tdc_sum_sq + (v_tdc_value * v_tdc_value);

                    -- Track saturation: TDC saturates when coarse count hits limits
                    -- With 8-bit coarse counter and 8 fractional bits, saturation is:
                    --   Max positive: +(2^7 - 1) x 256 = +32512 codes
                    --   Max negative: -(2^7) x 256 = -32768 codes
                    -- Use conservative threshold of +/-30000 to detect approaching saturation
                    if abs (v_tdc_value) >= 30000 then
                        v_tdc_sat_count := v_tdc_sat_count + 1;
                    end if;

                    if v_tdc_value < v_tdc_min then
                        v_tdc_min := v_tdc_value;
                    end if;
                    if v_tdc_value > v_tdc_max then
                        v_tdc_max := v_tdc_value;
                    end if;

                    -- Log every 2000th sample with full statistics (reduced for speed)
                    if (v_tdc_count mod 2000) = 0 then
                        v_mean     := v_tdc_sum / v_tdc_count;
                        v_variance := (v_tdc_sum_sq / v_tdc_count) - (v_mean * v_mean);
                        v_sat_pct  := (v_tdc_sat_count * 100) / v_tdc_count;

                        info("TDC_STAT: n=" & integer'image(v_tdc_count) & " mean=" & integer'image(v_mean) & " var=" & integer'image(v_variance) & " range=[" & integer'image(v_tdc_min) & "," & integer'image(v_tdc_max) & "]" & " sat=" & integer'image(v_sat_pct) & "%" & " Vin=" & integer'image(integer(analog_voltage_p * 1000.0)) & "mV" & " Vdac=" & integer'image(integer(analog_voltage_n * 1000.0)) & "mV" & " dV=" & integer'image(integer((analog_voltage_p - analog_voltage_n) * 1000.0)) & "mV");
                    end if;

                    -- Early termination: signal done after reaching sample target
                    if v_tdc_count >= C_CHAR_SAMPLE_TARGET then
                        tdc_char_done <= true;
                    end if;
                end if;
            end if;
        end process;
    end generate g_tdc_monitor;

    p_watchdog : process
        constant C_REPORT_INTERVAL : time := 1 ms;
    begin
        wait until reset = '0';

        loop
            wait for C_REPORT_INTERVAL;

            if not sim_finished then
                info("WATCHDOG at " & time'image(now) & " sample_data=" & integer'image(to_integer(signed(sample_data))));
            else
                exit;
            end if;
        end loop;
        wait;
    end process;

end architecture behavioral;
