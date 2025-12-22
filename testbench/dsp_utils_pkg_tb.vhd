library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library vunit_lib;
context vunit_lib.vunit_context;

use work.dsp_utils_pkg.all;

entity dsp_utils_pkg_tb is
    generic(runner_cfg : string);
end entity;

architecture tb of dsp_utils_pkg_tb is
begin

    p_main : process
        variable v_val_18bit : signed(17 downto 0);
        variable v_val_16bit : signed(15 downto 0);

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop

            if run("test_saturate") then
                v_val_18bit := to_signed(16#1234#, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(16#1234#, 16), "Positive value should pass through");

                v_val_18bit := to_signed(-4660, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(-4660, 16), "Negative value should pass through");

                v_val_18bit := to_signed(65536, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(32767, 16), "Should saturate to max positive");

                v_val_18bit := to_signed(-131072, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(-32768, 16), "Should saturate to max negative");

                v_val_18bit := to_signed(32767, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(32767, 16), "Max positive should not saturate");

                v_val_18bit := to_signed(32768, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(32767, 16), "Just over max should saturate");

                v_val_18bit := to_signed(-32768, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(-32768, 16), "Max negative should not saturate");

                v_val_18bit := to_signed(-32769, 18);
                v_val_16bit := saturate(v_val_18bit, 16);
                check_equal(v_val_16bit, to_signed(-32768, 16), "Just under min should saturate");

            end if;
        end loop;

        test_runner_cleanup(runner);
    end process;

    test_runner_watchdog(runner, 10 ms);

end architecture;
