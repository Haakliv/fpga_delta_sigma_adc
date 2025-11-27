-- ************************************************************************
-- DSP Utilities Package
-- Common functions for digital signal processing blocks
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package dsp_utils_pkg is
    -- Ceiling log2 function
    -- Returns the number of bits needed to represent values from 0 to x-1
    -- Example: clog2(16) = 4, clog2(17) = 5
    function clog2(x : positive) return natural;

    -- Saturating resize function
    -- Properly saturates signed values instead of wrapping
    -- Output width is inferred from the return value's range
    function saturate(x : signed; result_width : positive) return signed;

    -- Map 1-bit input to bipolar signed value
    -- '1' → +1, '0' → -1
    -- Generic width parameter allows use with different accumulator sizes
    function map_bipolar(b : std_logic; width : positive) return signed;

end package;

package body dsp_utils_pkg is
    -- Ceiling log2 function body
    function clog2(x : positive) return natural is
        variable v_temp : natural := x - 1;
        variable v_n    : natural := 0;
    begin
        while v_temp > 0 loop
            v_temp := v_temp / 2;
            v_n    := v_n + 1;
        end loop;
        return v_n;
    end function;

    function saturate(x : signed; result_width : positive) return signed is
        variable v_result     : signed(result_width - 1 downto 0);
        variable v_msb_result : std_logic; -- MSB of the result
        variable v_overflow   : boolean := false;
    begin
        -- Only check for overflow if input is wider than output
        if x'length > result_width then
            -- The MSB of the result will be x(x'low + result_width - 1)
            -- All bits above this must match it for no overflow
            v_msb_result := x(x'low + result_width - 1);

            -- Check if all bits from x'left downto (x'low + result_width) match v_msb_result
            for i in x'left downto (x'low + result_width) loop
                if x(i) /= v_msb_result then
                    v_overflow := true;
                    exit;
                end if;
            end loop;

            if v_overflow then
                -- Determine saturation direction based on input sign bit
                if x(x'left) = '0' then
                    -- Positive overflow: saturate to max positive
                    v_result                   := (others => '1');
                    v_result(result_width - 1) := '0';
                else
                    -- Negative underflow: saturate to max negative
                    v_result                   := (others => '0');
                    v_result(result_width - 1) := '1';
                end if;
                return v_result;
            end if;
        end if;

        -- No overflow/underflow, just resize
        v_result := resize(x, result_width);
        return v_result;
    end function;

    -- Map 1-bit input to bipolar signed value
    -- '1' → +1, '0' → -1
    function map_bipolar(b : std_logic; width : positive) return signed is
        variable v_one : signed(width - 1 downto 0) := (others => '0');
    begin
        v_one(0) := '1';
        if b = '1' then
            return v_one;
        else
            return -v_one;
        end if;
    end function;

end package body;
