-- ************************************************************************
-- Delta-Sigma ADC Top Level for AXE5000
-- Streams filtered samples directly over RS-232 UART
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axe5000_top is
  generic(
    -- OSR calculation for ~2 MHz tdc_valid rate, 115200 baud UART:
    -- UART capacity: 115200 / 10 bits = 11,520 bytes/sec
    -- Bytes per sample: 6 (4 hex + CR + LF)
    -- Max sample rate: 11,520 / 6 = 1,920 samples/sec
    -- Optimized OSR: 1152 (9×128) for ~1.74 kHz sample rate (91% UART utilization)
    -- 
    -- Input rate: tdc_valid_sys ≈ 2 MHz (one valid per Start pulse @ 2 MHz ref_clock)
    -- Decimation: 2,000,000 / 1152 ≈ 1,736 samples/sec
    -- 
    -- Signal chain: CIC (1.74 kHz) → Sinc³ Equalizer (1.74 kHz) → Lowpass (1.74 kHz)
    -- Non-decimating filters preserve full bandwidth (Nyquist = 868 Hz)
    GC_ADC_DECIMATION : positive := 1152 -- 9 × 128 for CIC-friendly decimation
  );
  port(
    -- Clock and Reset
    CLK_25M_C    : in  std_logic;
    -- UART
    UART_TX      : out std_logic;
    -- Differential analog input (Quartus auto-assigns N-pin for differential)
    ANALOG_IN    : in  std_logic;       -- LVDS differential input (P-pin, N-pin auto-assigned)
    -- Feedback output
    FEEDBACK_OUT : out std_logic;       -- Feedback DAC output

    -- Debug
    TEST_PIN     : out std_logic;
    LED1         : out std_logic;       -- Debug LED (TDC valid indicator)
    USER_BTN     : in  std_logic;       -- Active low reset
    -- Optional trigger (can be tied to '1' if not used)
    TRIGGER_IN   : in  std_logic := '1' -- Active high trigger (default '1' for continuous sampling)
  );
end entity;

architecture rtl of axe5000_top is

  constant C_ADC_DATA_WIDTH : positive := 16;

  signal sysclk_pd     : std_logic;     -- 100 MHz system clock from PLL
  signal clk_tdc_400m  : std_logic;     -- 400 MHz TDC clock from PLL
  signal clk_ref_2m    : std_logic;     -- 2 MHz reference clock from PLL
  signal rst_n_from_pd : std_logic;     -- Active-low reset from Platform Designer reset bridge
  signal rst           : std_logic;     -- Active-high reset for RTL modules

  signal adc_sample_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal adc_sample_valid : std_logic;

  signal uart_tx_data  : std_logic_vector(7 downto 0);
  signal uart_tx_valid : std_logic;
  signal uart_tx_ready : std_logic;

  -- GPIO IP signals
  signal s_comparator_out : std_logic_vector(0 downto 0); -- Differential comparator output from GPIO IP
  signal w_dac_bit        : std_logic;  -- DAC bit output to FEEDBACK_OUT

  component adc_system is               -- @suppress
    port(
      clk_25m_clk                    : in  std_logic                    := 'X'; -- clk
      -- Comparator GPIO IP (gpio_comp) - Bidirectional with separate P/N OE control
      gpio_comp_dout_export          : out std_logic_vector(0 downto 0); -- Differential comparator output
      gpio_comp_pad_in_export        : in  std_logic_vector(0 downto 0) := (others => 'X');
      gpio_comp_pad_in_b_export      : in  std_logic_vector(0 downto 0) := (others => 'X');
      -- Clock and Reset
      iopll_outclk_400m_clk          : out std_logic; -- clk
      iopll_outclk_2m_clk            : out std_logic; -- clk
      reset_n_reset_n                : in  std_logic                    := 'X'; -- reset_n
      sysrst_reset_n                 : out std_logic; -- reset_n
      -- UART Interface
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

  -- ========================================================================
  -- TDC ADC Configuration
  -- ========================================================================
  -- ANALOG_IN and ANALOG_IN_N: Differential comparator input
  -- FEEDBACK_OUT: 1-bit DAC output (external 1k + 1nF RC filter, shorted to ANALOG_IN_N)

  -- Debug and DAC outputs
  TEST_PIN     <= adc_sample_valid;     -- Sample valid pulse
  LED1         <= clk_tdc_400m;         -- LED shows 400MHz clock activity
  FEEDBACK_OUT <= w_dac_bit;            -- DAC output with external RC filter

  i_niosv : adc_system
    port map(
      clk_25m_clk                    => CLK_25M_C,
      reset_n_reset_n                => USER_BTN, -- Feed async reset directly to PD reset bridge
      sysrst_reset_n                 => rst_n_from_pd, -- Use PD's synchronized reset output

      -- Comparator GPIO (differential input - N-pin auto-assigned by Quartus)
      gpio_comp_dout_export          => s_comparator_out,
      gpio_comp_pad_in_export(0)     => ANALOG_IN,
      gpio_comp_pad_in_b_export(0)   => '0', -- Not used, Quartus assigns N-pin automatically
      -- PLL Clocks
      iopll_outclk_400m_clk          => clk_tdc_400m,
      iopll_outclk_2m_clk            => clk_ref_2m,
      sysclk_clk                     => sysclk_pd,
      -- UART
      data_receive_ready             => '1',
      data_receive_data              => open,
      data_receive_error             => open,
      data_receive_valid             => open,
      data_transmit_data             => uart_tx_data,
      data_transmit_error            => '0',
      data_transmit_valid            => uart_tx_valid,
      data_transmit_ready            => uart_tx_ready,
      rs232_0_external_interface_RXD => open,
      rs232_0_external_interface_TXD => UART_TX
    );

  -- Convert Platform Designer's active-low reset to active-high for RTL modules
  rst <= not rst_n_from_pd;

  -- TDC ADC instantiation
  i_adc : entity work.tdc_adc_top
    generic map(
      GC_DECIMATION => GC_ADC_DECIMATION,
      GC_DATA_WIDTH => C_ADC_DATA_WIDTH,
      GC_TDC_OUTPUT => 16,
      GC_SIM        => false
    )
    port map(
      clk_sys        => sysclk_pd,
      clk_tdc        => clk_tdc_400m,
      reset          => rst,
      -- Reference clock
      ref_clock      => clk_ref_2m,
      -- Comparator input from differential GPIO
      comparator_in  => s_comparator_out(0),
      -- DAC output to external pin (1k + 1nF RC filter, shorted to ANALOG_IN_N)
      dac_out_bit    => w_dac_bit,
      -- Optional trigger input (temporarily disabled - always enabled)
      trigger_enable => '1',
      -- Sample output
      sample_data    => adc_sample_data,
      sample_valid   => adc_sample_valid
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
