-- ************************************************************************
-- TDC Monitor UART Streamer
-- Packetizes TDC monitor data for transmission over UART
-- 
-- Packet Format (12 bytes):
--   Byte 0-1:   0xAA 0x55 (sync header)
--   Byte 2-3:   tdc_monitor_code (16-bit signed, big-endian)
--   Byte 4-5:   tdc_monitor_center (16-bit signed, big-endian)
--   Byte 6-7:   tdc_monitor_diff (16-bit signed, big-endian)
--   Byte 8:     tdc_monitor_dac (LSB = DAC bit)
--   Byte 9-10:  combined_data_out (16-bit signed ADC output, big-endian)
--   Byte 11:    Checksum (XOR of bytes 2-10)
-- 
-- Decimation: Monitor samples at ~2MHz, but UART at 115200 baud can only
-- send ~10kB/s. With 12-byte packets, that's ~850 packets/s max.
-- We decimate by 256 to send ~7.8k packets/s safely.
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tdc_monitor_uart_streamer is
    generic(
        GC_TDC_WIDTH    : positive := 16;
        GC_ADC_WIDTH    : positive := 16;
        GC_DECIMATION   : positive := 256  -- Decimate monitor samples for UART bandwidth
    );
    port(
        clk   : in std_logic;
        reset : in std_logic;

        -- TDC monitor inputs
        tdc_monitor_code   : in signed(GC_TDC_WIDTH - 1 downto 0);
        tdc_monitor_center : in signed(GC_TDC_WIDTH - 1 downto 0);
        tdc_monitor_diff   : in signed(GC_TDC_WIDTH - 1 downto 0);
        tdc_monitor_dac    : in std_logic;
        tdc_monitor_valid  : in std_logic;

        -- ADC output (for voltage estimation)
        adc_data_out   : in signed(GC_ADC_WIDTH - 1 downto 0);
        adc_data_valid : in std_logic;

        -- UART transmit interface
        uart_tx_data  : out std_logic_vector(7 downto 0);
        uart_tx_valid : out std_logic;
        uart_tx_ready : in  std_logic
    );
end entity;

architecture rtl of tdc_monitor_uart_streamer is

    -- FSM for packet transmission
    type t_state is (
        IDLE,
        SEND_SYNC0,     -- 0xAA
        SEND_SYNC1,     -- 0x55
        SEND_TDC_MSB,   -- tdc_code[15:8]
        SEND_TDC_LSB,   -- tdc_code[7:0]
        SEND_CTR_MSB,   -- tdc_center[15:8]
        SEND_CTR_LSB,   -- tdc_center[7:0]
        SEND_DIFF_MSB,  -- tdc_diff[15:8]
        SEND_DIFF_LSB,  -- tdc_diff[7:0]
        SEND_DAC,       -- dac_bit
        SEND_ADC_MSB,   -- adc_out[15:8]
        SEND_ADC_LSB,   -- adc_out[7:0]
        SEND_CHECKSUM   -- XOR checksum
    );

    signal state : t_state := IDLE;

    -- Decimation counter
    signal decim_counter : unsigned(15 downto 0) := (others => '0');

    -- Captured packet data
    signal tdc_code_cap   : signed(GC_TDC_WIDTH - 1 downto 0) := (others => '0');
    signal tdc_center_cap : signed(GC_TDC_WIDTH - 1 downto 0) := (others => '0');
    signal tdc_diff_cap   : signed(GC_TDC_WIDTH - 1 downto 0) := (others => '0');
    signal dac_bit_cap    : std_logic                         := '0';
    signal adc_out_cap    : signed(GC_ADC_WIDTH - 1 downto 0) := (others => '0');

    -- Checksum accumulator (XOR of data bytes)
    signal checksum : std_logic_vector(7 downto 0) := (others => '0');

    -- UART output registers
    signal uart_data  : std_logic_vector(7 downto 0) := (others => '0');
    signal uart_valid : std_logic                    := '0';

begin

    p_fsm : process(clk)
        variable v_checksum : std_logic_vector(7 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                state         <= IDLE;
                decim_counter <= (others => '0');
                tdc_code_cap   <= (others => '0');
                tdc_center_cap <= (others => '0');
                tdc_diff_cap   <= (others => '0');
                dac_bit_cap    <= '0';
                adc_out_cap    <= (others => '0');
                checksum      <= (others => '0');
                uart_data     <= (others => '0');
                uart_valid    <= '0';
            else
                case state is
                    when IDLE =>
                        uart_valid <= '0';

                        -- Wait for TDC monitor sample
                        if tdc_monitor_valid = '1' then
                            if decim_counter = to_unsigned(GC_DECIMATION - 1, decim_counter'length) then
                                -- Capture data for packet
                                tdc_code_cap   <= tdc_monitor_code;
                                tdc_center_cap <= tdc_monitor_center;
                                tdc_diff_cap   <= tdc_monitor_diff;
                                dac_bit_cap    <= tdc_monitor_dac;

                                -- Capture most recent ADC output (may be older than TDC sample)
                                adc_out_cap <= adc_data_out;

                                -- Reset checksum
                                checksum <= (others => '0');

                                -- Start packet transmission
                                state         <= SEND_SYNC0;
                                decim_counter <= (others => '0');
                            else
                                decim_counter <= decim_counter + 1;
                            end if;
                        end if;

                    when SEND_SYNC0 =>
                        if uart_tx_ready = '1' then
                            uart_data  <= x"AA";
                            uart_valid <= '1';
                            state      <= SEND_SYNC1;
                        end if;

                    when SEND_SYNC1 =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data  <= x"55";
                            uart_valid <= '1';
                            state      <= SEND_TDC_MSB;
                        end if;

                    when SEND_TDC_MSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(tdc_code_cap(15 downto 8));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(tdc_code_cap(15 downto 8));
                            checksum     <= v_checksum;
                            state        <= SEND_TDC_LSB;
                        end if;

                    when SEND_TDC_LSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(tdc_code_cap(7 downto 0));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(tdc_code_cap(7 downto 0));
                            checksum     <= v_checksum;
                            state        <= SEND_CTR_MSB;
                        end if;

                    when SEND_CTR_MSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(tdc_center_cap(15 downto 8));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(tdc_center_cap(15 downto 8));
                            checksum     <= v_checksum;
                            state        <= SEND_CTR_LSB;
                        end if;

                    when SEND_CTR_LSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(tdc_center_cap(7 downto 0));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(tdc_center_cap(7 downto 0));
                            checksum     <= v_checksum;
                            state        <= SEND_DIFF_MSB;
                        end if;

                    when SEND_DIFF_MSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(tdc_diff_cap(15 downto 8));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(tdc_diff_cap(15 downto 8));
                            checksum     <= v_checksum;
                            state        <= SEND_DIFF_LSB;
                        end if;

                    when SEND_DIFF_LSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(tdc_diff_cap(7 downto 0));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(tdc_diff_cap(7 downto 0));
                            checksum     <= v_checksum;
                            state        <= SEND_DAC;
                        end if;

                    when SEND_DAC =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            if dac_bit_cap = '1' then
                                uart_data <= x"01";
                            else
                                uart_data <= x"00";
                            end if;
                            uart_valid <= '1';
                            if dac_bit_cap = '1' then
                                v_checksum := checksum xor x"01";
                            else
                                v_checksum := checksum xor x"00";
                            end if;
                            checksum <= v_checksum;
                            state    <= SEND_ADC_MSB;
                        end if;

                    when SEND_ADC_MSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(adc_out_cap(15 downto 8));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(adc_out_cap(15 downto 8));
                            checksum     <= v_checksum;
                            state        <= SEND_ADC_LSB;
                        end if;

                    when SEND_ADC_LSB =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data    <= std_logic_vector(adc_out_cap(7 downto 0));
                            uart_valid   <= '1';
                            v_checksum   := checksum xor std_logic_vector(adc_out_cap(7 downto 0));
                            checksum     <= v_checksum;
                            state        <= SEND_CHECKSUM;
                        end if;

                    when SEND_CHECKSUM =>
                        if uart_valid = '1' and uart_tx_ready = '1' then
                            uart_valid <= '0';
                        end if;
                        if uart_valid = '0' or uart_tx_ready = '1' then
                            uart_data  <= checksum;
                            uart_valid <= '1';
                            state      <= IDLE;
                        end if;

                end case;
            end if;
        end if;
    end process;

    -- Output assignments
    uart_tx_data  <= uart_data;
    uart_tx_valid <= uart_valid;

end architecture;
