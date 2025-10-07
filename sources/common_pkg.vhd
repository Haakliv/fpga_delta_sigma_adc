-- ************************************************************************
-- Common Utilities Package
-- Reusable helper functions for FPGA design
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package common_pkg is

    -- Ceiling log2 function
    -- Returns the number of bits needed to represent values from 0 to x-1
    -- Example: clog2(16) = 4, clog2(17) = 5
    function clog2(x : positive) return natural;

    -- ========================================================================
    -- ADC Register Map (for rc_adc_top memory-mapped interface)
    -- ========================================================================
    constant C_ADC_REG_DATA     : natural := 0; -- ADC data output (16-bit)
    constant C_ADC_REG_STATUS   : natural := 1; -- Status register (8-bit)
    constant C_ADC_REG_VALID    : natural := 2; -- Valid counter (8-bit)
    constant C_ADC_REG_ACTIVITY : natural := 3; -- Activity counter (16-bit)

end package;

package body common_pkg is

    function clog2(x : positive) return natural is
        variable v_bit_count : natural range 0 to 31      := 0; -- Constrained range
        variable v_remainder : natural range 0 to 2 ** 30 := x - 1; -- Avoid overflow
    begin
        while v_remainder > 0 loop
            v_remainder := v_remainder / 2;
            v_bit_count := v_bit_count + 1;
        end loop;
        return v_bit_count;
    end function;

end package body;
