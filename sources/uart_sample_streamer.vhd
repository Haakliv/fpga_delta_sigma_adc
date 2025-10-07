-- ************************************************************************
-- UART Sample Streamer
-- Captures ADC samples and streams them as hex values over UART
-- Format: 4 hex digits + CR + LF (e.g., "A5F3\r\n")
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_sample_streamer is
    generic(
        GC_DATA_WIDTH : positive := 16  -- Sample data width
    );
    port(
        -- Clock and reset
        clk           : in  std_logic;
        rst           : in  std_logic;
        -- Sample input from ADC
        sample_data   : in  std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
        sample_valid  : in  std_logic;
        -- UART transmit interface
        uart_tx_data  : out std_logic_vector(7 downto 0);
        uart_tx_valid : out std_logic;
        uart_tx_ready : in  std_logic
    );
end entity;

architecture rtl of uart_sample_streamer is

    signal sample_capture : std_logic_vector(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal sample_staged  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal sample_latched : std_logic                                    := '0';
    signal sample_take    : std_logic                                    := '0';

    type   T_UART_STATE is (
        ST_UART_IDLE,
        ST_UART_SEND_N3,
        ST_UART_SEND_N2,
        ST_UART_SEND_N1,
        ST_UART_SEND_N0,
        ST_UART_SEND_CR,
        ST_UART_SEND_LF
    );
    signal uart_state   : T_UART_STATE := ST_UART_IDLE;

    constant C_UART_CR : std_logic_vector(7 downto 0) := x"0D";
    constant C_UART_LF : std_logic_vector(7 downto 0) := x"0A";

    -- Convert 4-bit nibble to ASCII hex character
    function to_hex_ascii(nibble : std_logic_vector(3 downto 0)) return std_logic_vector is
        variable v_value : integer range 0 to 15;
        variable v_ascii : std_logic_vector(7 downto 0);
    begin
        v_value := to_integer(unsigned(nibble));
        if v_value < 10 then
            v_ascii := std_logic_vector(to_unsigned(v_value + 48, 8)); -- '0'-'9'
        else
            v_ascii := std_logic_vector(to_unsigned(v_value - 10 + 65, 8)); -- 'A'-'F'
        end if;
        return v_ascii;
    end function;

begin

    -- Sample capture process
    -- Captures incoming samples and latches them until UART state machine is ready
    p_sample_capture : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                sample_capture <= (others => '0');
                sample_latched <= '0';
            else
                if sample_take = '1' then
                    sample_latched <= '0';
                end if;

                if sample_valid = '1' then
                    sample_capture <= sample_data;
                    sample_latched <= '1';
                end if;
            end if;
        end if;
    end process;

    -- UART state machine
    -- Sends captured sample as 4 hex digits followed by CR+LF
    p_uart_sm : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                uart_state    <= ST_UART_IDLE;
                uart_tx_data  <= (others => '0');
                uart_tx_valid <= '0';
                sample_staged <= (others => '0');
                sample_take   <= '0';
            else
                sample_take <= '0';

                if uart_tx_valid = '1' then
                    if uart_tx_ready = '1' then
                        uart_tx_valid <= '0';

                        case uart_state is
                            when ST_UART_SEND_N3 =>
                                uart_state <= ST_UART_SEND_N2;
                            when ST_UART_SEND_N2 =>
                                uart_state <= ST_UART_SEND_N1;
                            when ST_UART_SEND_N1 =>
                                uart_state <= ST_UART_SEND_N0;
                            when ST_UART_SEND_N0 =>
                                uart_state <= ST_UART_SEND_CR;
                            when ST_UART_SEND_CR =>
                                uart_state <= ST_UART_SEND_LF;
                            when ST_UART_SEND_LF =>
                                uart_state <= ST_UART_IDLE;
                            when others =>
                                uart_state <= ST_UART_IDLE;
                        end case;
                    end if;
                else
                    case uart_state is
                        when ST_UART_IDLE =>
                            if sample_latched = '1' then
                                sample_staged <= sample_capture;
                                sample_take   <= '1';
                                uart_state    <= ST_UART_SEND_N3;
                            end if;

                        when ST_UART_SEND_N3 =>
                            uart_tx_data  <= to_hex_ascii(sample_staged(15 downto 12));
                            uart_tx_valid <= '1';

                        when ST_UART_SEND_N2 =>
                            uart_tx_data  <= to_hex_ascii(sample_staged(11 downto 8));
                            uart_tx_valid <= '1';

                        when ST_UART_SEND_N1 =>
                            uart_tx_data  <= to_hex_ascii(sample_staged(7 downto 4));
                            uart_tx_valid <= '1';

                        when ST_UART_SEND_N0 =>
                            uart_tx_data  <= to_hex_ascii(sample_staged(3 downto 0));
                            uart_tx_valid <= '1';

                        when ST_UART_SEND_CR =>
                            uart_tx_data  <= C_UART_CR;
                            uart_tx_valid <= '1';

                        when ST_UART_SEND_LF =>
                            uart_tx_data  <= C_UART_LF;
                            uart_tx_valid <= '1';
                    end case;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
