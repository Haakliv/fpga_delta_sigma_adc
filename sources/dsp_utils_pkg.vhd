-- ************************************************************************
-- DSP Utilities Package
-- Common functions for digital signal processing blocks
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package dsp_utils_pkg is

    -- Saturating resize function
    -- Properly saturates signed values instead of wrapping
    function saturate(x : signed; width : positive) return signed;

end package;

package body dsp_utils_pkg is

    function saturate(x : signed; width : positive) return signed is
        variable v_result     : signed(width - 1 downto 0);
        variable v_msb_result : std_logic; -- MSB of the result
        variable v_overflow   : boolean := false;
    begin
        -- Only check for overflow if input is wider than output
        if x'length > width then
            -- The MSB of the result will be x(width-1)
            -- All bits above this must match it for no overflow
            v_msb_result := x(width - 1);

            -- Check if all bits from x'left downto width match v_msb_result
            for i in x'left downto width loop
                if x(i) /= v_msb_result then
                    v_overflow := true;
                    exit;
                end if;
            end loop;

            if v_overflow then
                -- Determine saturation direction based on input sign bit
                if x(x'left) = '0' then
                    -- Positive overflow: saturate to max positive
                    v_result            := (others => '1');
                    v_result(width - 1) := '0';
                else
                    -- Negative underflow: saturate to max negative
                    v_result            := (others => '0');
                    v_result(width - 1) := '1';
                end if;
                return v_result;
            end if;
        end if;

        -- No overflow/underflow, just resize
        v_result := resize(x, width);
        return v_result;
    end function;

end package body;
