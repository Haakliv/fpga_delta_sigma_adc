-- ************************************************************************
-- Delta-Sigma ADC Top Level for AXE5000
-- Streams filtered samples directly over RS-232 UART
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axe5000_top is
  generic(
    -- OSR calculation for 100 MHz clock, 115200 baud UART:
    -- UART capacity: 115200 / 10 bits = 11,520 bytes/sec
    -- Bytes per sample: 6 (4 hex + CR + LF)
    -- Max sample rate: 11,520 / 6 = 1,920 samples/sec
    -- Optimized OSR: 57344 gives 1,745 Hz sample rate (91% UART utilization)
    -- 
    -- Signal chain: CIC (1745 Hz) → Sinc³ Equalizer (1745 Hz) → Lowpass (1745 Hz)
    -- Non-decimating filters preserve full bandwidth (Nyquist = 872 Hz)
    GC_ADC_DECIMATION : positive := 57344 -- 7 × 2^13 for optimal UART bandwidth
  );
  port(
    -- Clock and Reset
    CLK_25M_C : in  std_logic;
    -- UART
    UART_TX   : out std_logic;
    -- DIP Switches
    -- Delta-Sigma ADC (true differential LVDS input)
    -- Quartus automatically creates ANALOG_IN(n) when LVDS I/O standard is assigned
    ANALOG_IN : in  std_logic;          -- Differential pair P-pin (Quartus manages N-pin)
    DAC_OUT   : out std_logic;          -- To integrator/filter

    -- Debug
    TEST_PIN  : out std_logic;
    USER_BTN  : in  std_logic           -- Active high reset
  );
end entity;

architecture rtl of axe5000_top is

  constant C_ADC_DATA_WIDTH : positive := 16;

  signal sysclk_pd     : std_logic;     -- 100 MHz system clock from PLL
  signal rst           : std_logic;
  signal rst_n_from_pd : std_logic;

  signal adc_sample_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal adc_sample_valid : std_logic;

  signal uart_tx_data  : std_logic_vector(7 downto 0);
  signal uart_tx_valid : std_logic;
  signal uart_tx_ready : std_logic;

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

  TEST_PIN <= DAC_OUT;
  

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

  rst <= not rst_n_from_pd;

  -- ADC core with delta-sigma modulator and filter chain
  i_adc : entity work.rc_adc_top
    generic map(
      GC_DECIMATION => GC_ADC_DECIMATION,
      GC_DATA_WIDTH => C_ADC_DATA_WIDTH
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
      analog_in    => ANALOG_IN,
      dac_out      => DAC_OUT,
      sample_data  => adc_sample_data,
      sample_valid => adc_sample_valid
    );

  -- UART streamer for ADC samples
  i_uart_streamer : entity work.uart_sample_streamer
    generic map(
      GC_DATA_WIDTH => C_ADC_DATA_WIDTH
    )
    port map(
      clk           => sysclk_pd,
      rst           => rst,
      sample_data   => adc_sample_data,
      sample_valid  => adc_sample_valid,
      uart_tx_data  => uart_tx_data,
      uart_tx_valid => uart_tx_valid,
      uart_tx_ready => uart_tx_ready
    );

end architecture rtl;
