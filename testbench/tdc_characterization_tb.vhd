-- ************************************************************************
-- Minimal TDC Characterization Testbench
-- Open-loop test to map TDC output vs analog input voltage
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;

library fpga_lib;

entity tdc_characterization_tb is
    generic(
        runner_cfg : string
    );
end entity;

architecture behavioral of tdc_characterization_tb is

    -- Test parameters
    constant C_CLK_SYS_PERIOD : time     := 10 ns; -- 100 MHz
    constant C_CLK_TDC_PERIOD : time     := 2.5 ns; -- 400 MHz
    constant C_REF_PERIOD     : time     := 500 ns; -- 2 MHz
    constant C_DATA_WIDTH     : positive := 16;
    constant C_TDC_WIDTH      : positive := 16;

    -- DUT signals
    signal clk_sys   : std_logic := '0';
    signal clk_tdc   : std_logic := '0';
    signal reset     : std_logic := '1';
    signal ref_clock : std_logic := '0';

    -- Open-loop test signals
    signal open_loop_dac    : std_logic := '0';
    signal s_comparator_out : std_logic := '0';

    -- Debug signals
    signal debug_tdc_out   : signed(C_TDC_WIDTH - 1 downto 0);
    signal debug_tdc_valid : std_logic;

    -- Analog signals
    signal analog_voltage_p : real      := 0.0;
    signal analog_voltage_n : real      := 0.0;
    signal pad_p            : std_logic := '0';
    signal pad_n            : std_logic := '0';

    -- Test control
    signal sim_finished : boolean := false;

    -- File for logging results
    file output_file : text;

begin

    -- ========================================================================
    -- Device Under Test (Open-Loop Mode)
    -- ========================================================================
    i_dut : entity work.tdc_adc_top
        generic map(
            GC_DECIMATION => 16,
            GC_DATA_WIDTH => C_DATA_WIDTH,
            GC_TDC_OUTPUT => C_TDC_WIDTH,
            GC_SIM        => true,
            GC_OPEN_LOOP  => true       -- CRITICAL: Open-loop for characterization
        )
        port map(
            clk_sys            => clk_sys,
            clk_tdc            => clk_tdc,
            reset              => reset,
            ref_clock          => ref_clock,
            comparator_in      => s_comparator_out,
            dac_out_bit        => open,  -- Not monitored in characterization
            trigger_enable     => '1',
            open_loop_dac_duty => open_loop_dac,
            sample_data        => open,  -- Not monitored in characterization
            sample_valid       => open,  -- Not monitored in characterization
            debug_tdc_out      => debug_tdc_out,
            debug_tdc_valid    => debug_tdc_valid
        );

    -- ========================================================================
    -- Clock Generation
    -- ========================================================================
    p_clk_sys : process
    begin
        while not sim_finished loop
            clk_sys <= '0';
            wait for C_CLK_SYS_PERIOD / 2;
            clk_sys <= '1';
            wait for C_CLK_SYS_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_clk_tdc : process
    begin
        while not sim_finished loop
            clk_tdc <= '0';
            wait for C_CLK_TDC_PERIOD / 2;
            clk_tdc <= '1';
            wait for C_CLK_TDC_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_ref_clock : process
    begin
        while not sim_finished loop
            ref_clock <= '0';
            wait for C_REF_PERIOD / 2;
            ref_clock <= '1';
            wait for C_REF_PERIOD / 2;
        end loop;
        wait;
    end process;

    -- ========================================================================
    -- Reset Generation
    -- ========================================================================
    p_reset : process
    begin
        reset <= '1';
        wait for 500 ns;
        reset <= '0';
        wait;
    end process;

    -- ========================================================================
    -- Analog Input (Fixed DC Level)
    -- ========================================================================
    p_analog_voltage_p : process
        constant C_VBANK : real := 1.3; -- FPGA I/O bank voltage
        constant C_LEVEL : real := 0.5; -- Fixed mid-rail level for characterization
    begin
        wait until reset = '0';

        -- In characterization mode, we set this to a fixed level
        -- The test will control it via shared variable or external mechanism
        analog_voltage_p <= C_LEVEL * C_VBANK;

        wait for 10 ms;                 -- Hold for duration of test
    end process;

    -- ========================================================================
    -- RC Filter Model (DAC output to N-pin)
    -- ========================================================================
    p_rc_filter : process(open_loop_dac, reset)
        constant C_DAC_LOW_V  : real := 0.0;
        constant C_DAC_HIGH_V : real := 1.3;
        constant C_DELAY      : time := 80 ps;
    begin
        if reset = '1' then
            analog_voltage_n <= 0.0;
        else
            if open_loop_dac = '1' then
                analog_voltage_n <= transport C_DAC_HIGH_V after C_DELAY;
            else
                analog_voltage_n <= transport C_DAC_LOW_V after C_DELAY;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Differential Comparator Model
    -- ========================================================================
    p_pad_differential : process(analog_voltage_p, analog_voltage_n)
        constant C_DELAY : time := 2.0 ns;
        constant C_HYST  : real := 0.005;
    begin
        if analog_voltage_p > (analog_voltage_n + C_HYST) then
            pad_p <= transport '1' after C_DELAY;
            pad_n <= transport '0' after C_DELAY;
        elsif analog_voltage_p < (analog_voltage_n - C_HYST) then
            pad_p <= transport '0' after C_DELAY;
            pad_n <= transport '1' after C_DELAY;
        else
            if (analog_voltage_p - analog_voltage_n) > 0.0 then
                pad_p <= transport '1' after C_DELAY;
                pad_n <= transport '0' after C_DELAY;
            else
                pad_p <= transport '0' after C_DELAY;
                pad_n <= transport '1' after C_DELAY;
            end if;
        end if;
    end process;

    -- Behavioral comparator
    p_behavioral_comparator : process(pad_p, pad_n)
    begin
        if pad_p = '1' and pad_n = '0' then
            s_comparator_out <= '1';
        elsif pad_p = '0' and pad_n = '1' then
            s_comparator_out <= '0';
        else
            s_comparator_out <= pad_p xor pad_n;
        end if;
    end process;

    -- ========================================================================
    -- TDC Logger - Writes raw TDC values to file
    -- ========================================================================
    p_tdc_logger : process
        variable v_line        : line;
        variable v_count       : integer := 0;
        constant C_MAX_SAMPLES : integer := 1000; -- Collect 1000 TDC samples
    begin
        wait until reset = '0';
        wait until rising_edge(clk_tdc);

        file_open(output_file, "tdc_characterization_log.txt", write_mode);
        write(v_line, string'("# TDC Characterization Data"));
        writeline(output_file, v_line);
        write(v_line, string'("# time_ns,tdc_value,analog_p,analog_n,delta_v,dac_duty"));
        writeline(output_file, v_line);

        while v_count < C_MAX_SAMPLES loop
            wait until rising_edge(clk_tdc);

            if debug_tdc_valid = '1' then
                write(v_line, real'image(real(now / 1 ps) / 1000.0));
                write(v_line, string'(","));
                write(v_line, integer'image(to_integer(debug_tdc_out)));
                write(v_line, string'(","));
                write(v_line, real'image(analog_voltage_p));
                write(v_line, string'(","));
                write(v_line, real'image(analog_voltage_n));
                write(v_line, string'(","));
                write(v_line, real'image(analog_voltage_p - analog_voltage_n));
                write(v_line, string'(","));
                if open_loop_dac = '1' then
                    write(v_line, string'("1.0"));
                else
                    write(v_line, string'("0.0"));
                end if;
                writeline(output_file, v_line);

                v_count := v_count + 1;

                if v_count mod 100 = 0 then
                    info("Logged " & integer'image(v_count) & " TDC samples");
                end if;
            end if;
        end loop;

        file_close(output_file);
        info("TDC logging complete - " & integer'image(v_count) & " samples written");

        wait;
    end process;

    -- ========================================================================
    -- Test Runner
    -- ========================================================================
    p_main : process
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("characterize_dac_low") then
                info("==========================================================");
                info("Characterization Test: DAC=LOW (0V), Input=0.65V");
                info("==========================================================");

                open_loop_dac <= '0';   -- DAC outputs 0V

                -- Wait for logging to complete
                wait for 10 ms;

            elsif run("characterize_dac_high") then
                info("==========================================================");
                info("Characterization Test: DAC=HIGH (1.3V), Input=0.65V");
                info("==========================================================");

                open_loop_dac <= '1';   -- DAC outputs 1.3V

                -- Wait for logging to complete
                wait for 10 ms;

            end if;
        end loop;

        wait for C_CLK_SYS_PERIOD * 10;
        sim_finished <= true;
        test_runner_cleanup(runner);
        wait;
    end process;

end architecture behavioral;
