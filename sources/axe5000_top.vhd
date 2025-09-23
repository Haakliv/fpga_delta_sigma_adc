-- ************************************************************************
-- Delta-Sigma ADC Top Level for AXE5000
-- Streams filtered samples directly over RS-232 UART (Intel IP)
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axe5000_top is
  generic(
    GC_ADC_DECIMATION : positive := 64  -- Configurable OSR/decimation factor
  );
  port(
    -- Clock and Reset
    CLK_25M_C : in  std_logic;
    -- UART
    UART_TX   : out std_logic;
    -- DIP Switches
    -- Delta-Sigma ADC (differential LVDS input)
    ANALOG_IN : in  std_logic;          -- From comparator (differential pair handled at I/O level)
    DAC_OUT   : out std_logic;          -- To integrator/filter

    -- Debug
    TEST_PIN  : out std_logic;
    USER_BTN  : in  std_logic           -- Active high reset
  );
end entity;

architecture rtl of axe5000_top is

  constant C_ADC_DATA_WIDTH : positive := 16;

  signal sysclk_pd     : std_logic;
  signal rst           : std_logic;
  signal rst_n_from_pd : std_logic;

  signal adc_sample_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal adc_sample_valid : std_logic;

  signal sample_capture : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal sample_staged  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal sample_latched : std_logic                                       := '0';
  signal sample_take    : std_logic                                       := '0';

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

  signal uart_tx_data  : std_logic_vector(7 downto 0) := (others => '0');
  signal uart_tx_valid : std_logic                    := '0';
  signal uart_tx_ready : std_logic;

  constant C_UART_CR : std_logic_vector(7 downto 0) := x"0D";
  constant C_UART_LF : std_logic_vector(7 downto 0) := x"0A";

  function to_hex_ascii(nibble : std_logic_vector(3 downto 0)) return std_logic_vector is
    variable v_value : integer range 0 to 15;
    variable v_ascii : std_logic_vector(7 downto 0);
  begin
    v_value := to_integer(unsigned(nibble));
    if v_value < 10 then
      v_ascii := std_logic_vector(to_unsigned(v_value + 48, 8));
    else
      v_ascii := std_logic_vector(to_unsigned(v_value - 10 + 65, 8));
    end if;
    return v_ascii;
  end function;

  component adc_system is               -- @suppress
    port(
      clk_25m_clk                    : in  std_logic                    := 'X'; -- clk
      dip_sw_export                  : in  std_logic_vector(1 downto 0) := (others => 'X'); -- export
      pb_export                      : in  std_logic                    := 'X'; -- export
      reset_n_reset_n                : in  std_logic                    := 'X'; -- reset_n
      sys_reset_reset_n              : out std_logic; -- reset_n
      data_receive_ready             : in  std_logic                    := 'X'; -- ready
      data_receive_data              : out std_logic_vector(7 downto 0); -- data
      data_receive_error             : out std_logic; -- error
      data_receive_valid             : out std_logic; -- valid
      data_transmit_data             : in  std_logic_vector(7 downto 0) := (others => 'X'); -- data
      data_transmit_error            : in  std_logic                    := 'X'; -- error
      data_transmit_valid            : in  std_logic                    := 'X'; -- valid
      data_transmit_ready            : out std_logic; -- ready
      rs232_0_external_interface_RXD : in  std_logic                    := 'X'; -- RXD -- @suppress "Naming convention violation: port name should match pattern '^(?:[a-z](?:[a-z0-9]|_(?!_))*|[A-Z](?:[A-Z0-9]|_(?!_))*)(?:_n)?$'"
      rs232_0_external_interface_TXD : out std_logic; -- TXD -- @suppress "Naming convention violation: port name should match pattern '^(?:[a-z](?:[a-z0-9]|_(?!_))*|[A-Z](?:[A-Z0-9]|_(?!_))*)(?:_n)?$'"
      sysclk_clk                     : out std_logic -- clk
    );
  end component adc_system;

begin

  TEST_PIN <= ANALOG_IN;

  i_niosv : adc_system
    port map(
      clk_25m_clk                    => CLK_25M_C,
      reset_n_reset_n                => USER_BTN,
      sys_reset_reset_n              => rst_n_from_pd,
      dip_sw_export                  => (others => '0'),
      pb_export                      => '0',
      data_receive_ready             => '1',
      data_receive_data              => open,
      data_receive_error             => open,
      data_receive_valid             => open,
      data_transmit_data             => uart_tx_data,
      data_transmit_error            => '0',
      data_transmit_valid            => uart_tx_valid,
      data_transmit_ready            => uart_tx_ready,
      rs232_0_external_interface_RXD => open,
      rs232_0_external_interface_TXD => UART_TX,
      sysclk_clk                     => sysclk_pd
    );

  i_adc : entity work.rc_adc_top
    generic map(
      GC_DECIMATION      => GC_ADC_DECIMATION,
      GC_DATA_WIDTH      => C_ADC_DATA_WIDTH,
      GC_ENABLE_MAJORITY => false
    )
    port map(
      clk          => sysclk_pd,
      reset        => rst,
      mem_cs       => '0',
      mem_rd       => '0',
      mem_wr       => '0',
      mem_addr     => (others => '0'),
      mem_wdata    => (others => '0'),
      mem_rdata    => open,
      mem_rdvalid  => open,
      analog_in_p  => ANALOG_IN,
      analog_in_n  => '0',
      dac_out      => DAC_OUT,
      sample_data  => adc_sample_data,
      sample_valid => adc_sample_valid
    );

  rst <= not rst_n_from_pd;

  p_sample_capture : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      if rst = '1' then
        sample_capture <= (others => '0');
        sample_latched <= '0';
      else
        if sample_take = '1' then
          sample_latched <= '0';
        end if;

        if adc_sample_valid = '1' then
          sample_capture <= adc_sample_data;
          sample_latched <= '1';
        end if;
      end if;
    end if;
  end process;

  p_uart_sm : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
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
