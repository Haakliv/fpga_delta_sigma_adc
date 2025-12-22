library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package dsp_utils_pkg is
    function clog2(x : positive) return natural;

    function saturate(x : signed; result_width : positive) return signed;

    function map_bipolar(b : std_logic; width : positive) return signed;

end package;

package body dsp_utils_pkg is
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
        variable v_msb_result : std_logic;
        variable v_overflow   : boolean := false;
    begin
        if x'length > result_width then
            v_msb_result := x(x'low + result_width - 1);

            for i in x'left downto (x'low + result_width) loop
                if x(i) /= v_msb_result then
                    v_overflow := true;
                    exit;
                end if;
            end loop;

            if v_overflow then
                if x(x'left) = '0' then
                    v_result                   := (others => '1');
                    v_result(result_width - 1) := '0';
                else
                    v_result                   := (others => '0');
                    v_result(result_width - 1) := '1';
                end if;
                return v_result;
            end if;
        end if;

        v_result := resize(x, result_width);
        return v_result;
    end function;

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
