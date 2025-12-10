-------------------------------------------------------------------------------
-- Testbench for GPIO pseudo-differential output verification
-- Tests OE control, output drive, and polarity of the modified Intel GPIO IP
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library vunit_lib;
context vunit_lib.vunit_context;

entity gpio_pseudo_diff_tb is
    generic(
        runner_cfg : string
    );
end entity;

architecture tb of gpio_pseudo_diff_tb is

    -- Clock: 400 MHz like clk_tdc
    constant C_CLK_PERIOD : time      := 2.5 ns;
    signal   clk          : std_logic := '0';

    -- DUT signals
    signal oe_in     : std_logic_vector(0 downto 0) := "0";
    signal din_in    : std_logic_vector(0 downto 0) := "0";
    signal pad_p_vec : std_logic_vector(0 downto 0);
    signal pad_n_vec : std_logic_vector(0 downto 0);
    signal pad_p     : std_logic;
    signal pad_n     : std_logic;

    -- Component declaration for GPIO IP
    component gpio_dac is
        port(
            ck        : in  std_logic;
            din       : in  std_logic_vector(0 downto 0);
            oe        : in  std_logic_vector(0 downto 0);
            pad_out   : out std_logic_vector(0 downto 0);
            pad_out_b : out std_logic_vector(0 downto 0)
        );
    end component;

    -- Helper function to convert std_logic to string
    function to_string(value : std_logic) return string is
    begin
        case value is
            when '0' => return "0";
            when '1' => return "1";
            when 'Z' => return "Z";
            when 'X' => return "X";
            when 'U' => return "U";
            when 'W' => return "W";
            when 'L' => return "L";
            when 'H' => return "H";
            when '-' => return "-";
        end case;
    end function;

    -- Check task
    procedure check_drive(
        constant tag        : in string;
        constant expect_inv : in boolean;
        signal   clk        : in std_logic;
        signal   din_in     : in std_logic_vector;
        signal   pad_p      : in std_logic;
        signal   pad_n      : in std_logic
    ) is
        variable v_expect_n : std_logic;
    begin
        -- Wait a few cycles for SDR register path
        for i in 1 to 4 loop
            wait until rising_edge(clk);
        end loop;

        -- Check P is Hi-Z
        check(pad_p = 'Z',
              "[" & tag & "] P leg is NOT Hi-Z while OE enables N (pad_p=" & to_string(pad_p) & ")");

        -- Check N is not X
        check(pad_n /= 'X',
              "[" & tag & "] N leg is X (dependent on P internal path?)");

        -- Polarity check: With Intel pseudo-diff, N = ~din (through obar)
        if expect_inv then
            v_expect_n := not din_in(0);
        else
            v_expect_n := din_in(0);
        end if;

        if pad_n /= v_expect_n then
            error("[" & tag & "] Polarity mismatch: din=" & to_string(din_in(0)) & " pad_n=" & to_string(pad_n) & " expected=" & to_string(v_expect_n));
        else
            info("[" & tag & "] OK: din=" & to_string(din_in(0)) & " -> pad_n=" & to_string(pad_n) & ", pad_p=" & to_string(pad_p));
        end if;
    end procedure;

begin

    -- Clock generation
    clk <= not clk after C_CLK_PERIOD / 2;

    -- Extract scalars from vectors
    pad_p <= pad_p_vec(0);
    pad_n <= pad_n_vec(0);

    -- DUT instantiation
    i_dut : gpio_dac
        port map(
            ck        => clk,
            din       => din_in,
            oe        => oe_in,
            pad_out   => pad_p_vec,
            pad_out_b => pad_n_vec
        );

    -- Main test process
    p_main : process
    begin
        test_runner_setup(runner, runner_cfg);

        info("=== GPIO Pseudo-Differential Testbench ===");
        info("Testing OE control, output drive, and polarity");
        info("");

        while test_suite loop
            if run("idle_tristate") then
                -- Case 0: tri-state everything (sampling phase) -> both Z
                info("Test: Both pads should be Hi-Z when OE=0");
                oe_in  <= "0";
                din_in <= "0";

                for i in 1 to 4 loop
                    wait until rising_edge(clk);
                end loop;

                if not (pad_p = 'Z' and pad_n = 'Z') then
                    error("[idle] Both legs should be Z when OE=0 (pad_p=" & to_string(pad_p) & " pad_n=" & to_string(pad_n) & ")");
                else
                    info("[idle] Both legs are Hi-Z as expected");
                end if;

            elsif run("drive_output_0") then
                -- Case 1a: drive N with din=0, P must be Hi-Z
                info("Test: Drive N with din=0 while P is Hi-Z");
                oe_in  <= "1";
                din_in <= "0";
                check_drive("drive0", true, clk, din_in, pad_p, pad_n);

            elsif run("drive_output_1") then
                -- Case 1b: drive N with din=1, P must be Hi-Z
                info("Test: Drive N with din=1 while P is Hi-Z");
                oe_in  <= "1";
                din_in <= "1";
                check_drive("drive1", true, clk, din_in, pad_p, pad_n);

            elsif run("stress_toggle") then
                -- Stress test: toggle din rapidly and watch for X
                info("Test: Stress toggle din while OE=1");
                oe_in <= "1";

                for i in 1 to 16 loop
                    if (i mod 2) = 0 then
                        din_in <= "0";
                    else
                        din_in <= "1";
                    end if;

                    wait until rising_edge(clk);

                    check(pad_n /= 'X',
                          "[stress] pad_n turned X while P is Hi-Z (coupling suspicion)");

                    info("[stress] din=" & to_string(din_in(0)) & " -> pad_n=" & to_string(pad_n) & " (OK)");
                end loop;

            end if;
        end loop;

        info("");
        info("=== TB done - All checks passed ===");
        test_runner_cleanup(runner);
    end process;

    -- Watchdog
    test_runner_watchdog(runner, 1 ms);

end architecture;
