-- ************************************************************************
-- Component-less Comparator IO Wrapper
-- Direct instantiation of Intel IO primitives for differential input and
-- single-ended output on the same LVDS pair
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Intel Agilex 5 primitive library
library tennm_ph2;

entity componentless_comparator_io is
    port(
        -- Clocks
        clk_sys    : in    std_logic;   -- System clock for registering comparator output
        clk_io     : in    std_logic;   -- High-speed clock for IOE register (DAC output)
        reset_n    : in    std_logic;   -- Active-low reset

        -- Logic interface
        dac_data   : in    std_logic;   -- DAC output data (will drive FEEDBACK_N)
        dac_oe     : in    std_logic;   -- DAC output enable
        comp_out   : out   std_logic;   -- Comparator output (ANALOG_IN - FEEDBACK_N)

        -- Physical pins (inout for flexibility, but used as input/output)
        ANALOG_IN  : inout std_logic;   -- P-pin: analog input (not driven by FPGA)
        FEEDBACK_N : inout std_logic    -- N-pin: DAC feedback output
    );
end entity componentless_comparator_io;

architecture rtl of componentless_comparator_io is

    -- Component declarations for Intel primitives
    component tennm_ph2_io_ibuf
        generic(
            bus_hold        : string := "BUS_HOLD_OFF";
            buffer_usage    : string := "REGULAR";
            equalization    : string := "EQUALIZATION_OFF";
            io_standard     : string := "IO_STANDARD_IOSTD_OFF";
            rzq_id          : string := "RZQ_ID_RZQ0";
            schmitt_trigger : string := "SCHMITT_TRIGGER_OFF";
            termination     : string := "TERMINATION_RT_OFF";
            toggle_speed    : string := "TOGGLE_SPEED_SLOW";
            usage_mode      : string := "USAGE_MODE_GPIO";
            vref            : string := "VREF_OFF";
            weak_pull_down  : string := "WEAK_PULL_DOWN_OFF";
            weak_pull_up    : string := "WEAK_PULL_UP_OFF"
        );
        port(
            i    : in  std_logic;
            ibar : in  std_logic;
            o    : out std_logic
        );
    end component;

    component tennm_ph2_io_obuf
        generic(
            open_drain              : string  := "OPEN_DRAIN_OFF";
            buffer_usage            : string  := "REGULAR";
            dynamic_pull_up_enabled : boolean := false;
            equalization            : string  := "EQUALIZATION_OFF";
            io_standard             : string  := "IO_STANDARD_IOSTD_OFF";
            rzq_id                  : string  := "RZQ_ID_RZQ0";
            slew_rate               : string  := "SLEW_RATE_FAST";
            termination             : string  := "TERMINATION_SERIES_OFF";
            toggle_speed            : string  := "TOGGLE_SPEED_FAST";
            usage_mode              : string  := "USAGE_MODE_GPIO"
        );
        port(
            i  : in  std_logic;
            oe : in  std_logic;
            o  : out std_logic
        );
    end component;

    -- Internal signals
    signal comp_raw : std_logic;        -- Raw differential comparator output

    -- Synthesis attributes to prevent optimization
    attribute preserve             : boolean;
    attribute preserve of comp_raw : signal is true;

begin

    -- ========================================================================
    -- DIFFERENTIAL INPUT BUFFER (Comparator)
    -- Reads ANALOG_IN (P) - FEEDBACK_N (N) using hardware differential buffer
    -- ========================================================================
    i_diff_ibuf : tennm_ph2_io_ibuf
        generic map(
            bus_hold        => "BUS_HOLD_OFF",
            buffer_usage    => "REGULAR",
            equalization    => "EQUALIZATION_OFF",
            io_standard     => "IO_STANDARD_IOSTD_OFF",
            rzq_id          => "RZQ_ID_RZQ0",
            schmitt_trigger => "SCHMITT_TRIGGER_OFF",
            termination     => "TERMINATION_RT_OFF",
            toggle_speed    => "TOGGLE_SPEED_SLOW",
            usage_mode      => "USAGE_MODE_GPIO",
            vref            => "VREF_OFF",
            weak_pull_down  => "WEAK_PULL_DOWN_OFF",
            weak_pull_up    => "WEAK_PULL_UP_OFF"
        )
        port map(
            i    => ANALOG_IN,          -- P-pin input
            ibar => FEEDBACK_N,         -- N-pin input (for differential comparison)
            o    => comp_raw            -- Differential output: (ANALOG_IN - FEEDBACK_N)
        );

    -- Register comparator output in system clock domain
    process(clk_sys, reset_n)
    begin
        if reset_n = '0' then
            comp_out <= '0';
        elsif rising_edge(clk_sys) then
            comp_out <= comp_raw;
        end if;
    end process;

    --Error(20196): Location(s) already occupied and the components cannot be merged. (1 location affected) 
    -- Error(14566): The Fitter cannot place 1 periphery component(s) due to conflicts with existing constraints (1 I/O pad(s)). Fix the errors described in the submessages, and then rerun the Fitter. The Intel FPGA Knowledge Database may also contain articles with information on how to resolve this periphery placement failure. Review the errors and then visit the Knowledge Database at https://www.intel.com/content/www/us/en/support/programmable/kdb-filter.html and search for this specific error message number. 
    -- 	Error(175019): Illegal constraint of I/O pad to the location PIN_W3 
    -- 		Error(16234): No legal location could be found out of 1 considered location(s).  Reasons why each location could not be used are summarized below: 
    -- 			Error(20196): Location(s) already occupied and the components cannot be merged. (1 location affected) 
    -- Error(175019): Illegal constraint of I/O pad to the location PIN_W3 
    -- 	Error(16234): No legal location could be found out of 1 considered location(s).  Reasons why each location could not be used are summarized below: 
    -- 		Error(20196): Location(s) already occupied and the components cannot be merged. (1 location affected) 
    -- Error(16234): No legal location could be found out of 1 considered location(s).  Reasons why each location could not be used are summarized below: 
    -- 	Error(20196): Location(s) already occupied and the components cannot be merged. (1 location affected) 

    -- ========================================================================
    -- SINGLE-ENDED OUTPUT BUFFER (DAC - N-pin only)
    -- Only drives FEEDBACK_N, ANALOG_IN remains undriven by FPGA
    -- ========================================================================

    -- Register data and OE in IOE at high speed clock
    -- Two single-ended output buffers approach (from paper):
    -- P-pin buffer: Always tristated (high-Z) - allows external analog input
    -- N-pin buffer: DAC output with OE control

    -- N-pin output buffer - actual DAC output
    i_dac_obuf_n : tennm_ph2_io_obuf
        generic map(
            open_drain              => "OPEN_DRAIN_OFF",
            buffer_usage            => "REGULAR",
            dynamic_pull_up_enabled => false,
            equalization            => "EQUALIZATION_OFF",
            io_standard             => "IO_STANDARD_IOSTD_OFF",
            rzq_id                  => "RZQ_ID_RZQ0",
            slew_rate               => "SLEW_RATE_FAST",
            termination             => "TERMINATION_SERIES_OFF",
            toggle_speed            => "TOGGLE_SPEED_FAST",
            usage_mode              => "USAGE_MODE_GPIO"
        )
        port map(
            i  => dac_data,             -- DAC data input
            oe => dac_oe,               -- Output enable
            o  => FEEDBACK_N            -- N-pin output
        );

    -- Note: This matches the approach from the paper where both P and N pins
    -- have output buffers, but P is kept permanently tristated

end architecture rtl;
