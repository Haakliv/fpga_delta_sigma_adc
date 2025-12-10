-- ************************************************************************
-- Delta-Sigma ADC Top Level for AXE5000
-- Streams filtered samples directly over RS-232 UART
-- Supports both live streaming and burst capture modes
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axe5000_top is
  generic(
    -- ADC Decimation: 50MHz / 25600 ≈ 1953 S/s
    -- ASCII mode: 115200/10/6 = 1920 S/s max, so use higher decimation
    GC_ADC_DECIMATION  : positive := 25600;  -- Decimation for ~1953 S/s (fits ASCII mode)
    GC_UART_BINARY     : boolean  := false; -- false=ASCII hex (human readable)
    GC_CAPTURE_DEPTH   : positive := 4096;  -- Burst capture buffer depth
    GC_CAPTURE_ENABLED : boolean  := false  -- Disable burst capture - just stream continuously
  );
  port(
    -- Clock and Reset
    CLK_25M_C    : in  std_logic;
    -- UART
    UART_TX      : out std_logic;
    UART_RX      : in  std_logic := '1';    -- UART receive for commands
    -- Differential analog input (Quartus auto-assigns N-pin for differential)
    ANALOG_IN    : in  std_logic;       -- LVDS differential input (P-pin, N-pin auto-assigned)
    -- Feedback output
    FEEDBACK_OUT : out std_logic;       -- Feedback DAC output

    -- Debug
    TEST_PIN     : out std_logic;
    LED1         : out std_logic;       -- Debug LED (TDC valid indicator)
    USER_BTN     : in  std_logic;       -- Active low reset
    -- Optional trigger (directly triggers capture when GC_CAPTURE_ENABLED)
    TRIGGER_IN   : in  std_logic := '0' -- Active high trigger for capture start
  );
end entity;

architecture rtl of axe5000_top is

  constant C_ADC_DATA_WIDTH : positive := 16;
  constant C_CAPTURE_ADDR_W : positive := 12;  -- log2(4096)

  signal sysclk_pd     : std_logic;     -- 100 MHz system clock from PLL
  signal clk_tdc_400m  : std_logic;     -- 400 MHz TDC clock from PLL
  signal clk_ref_2m    : std_logic;     -- 50 MHz reference clock from PLL
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

  -- Capture module signals
  signal capture_start    : std_logic := '0';
  signal capture_dump     : std_logic := '0';
  signal capture_active   : std_logic;
  signal capture_done     : std_logic;
  signal dump_active      : std_logic;
  signal dump_data        : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal dump_valid       : std_logic;
  signal dump_ready       : std_logic;
  
  -- UART mux signals (live vs dump mode)
  signal streamer_data    : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal streamer_valid   : std_logic;
  
  -- Runtime control
  signal disable_tdc_contrib : std_logic := '0';  -- 0=TDC enabled, 1=TDC disabled (CIC-only)
  signal disable_eq_filter   : std_logic := '0';  -- 0=EQ enabled, 1=EQ bypassed
  signal disable_lp_filter   : std_logic := '0';  -- 0=LP enabled, 1=LP bypassed
  
  -- UART RX command decoder
  signal uart_rx_data     : std_logic_vector(7 downto 0);
  signal uart_rx_valid    : std_logic;
  signal trigger_sync     : std_logic_vector(2 downto 0) := (others => '0');
  signal trigger_edge     : std_logic;

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
  LED1         <= capture_active or dump_active; -- LED shows capture/dump activity
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
      data_receive_data              => uart_rx_data,
      data_receive_error             => open,
      data_receive_valid             => uart_rx_valid,
      data_transmit_data             => uart_tx_data,
      data_transmit_error            => '0',
      data_transmit_valid            => uart_tx_valid,
      data_transmit_ready            => uart_tx_ready,
      rs232_0_external_interface_RXD => UART_RX,
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
      GC_SIM        => false,
      GC_OPEN_LOOP  => false  -- Normal closed-loop operation
    )
    port map(
      clk_sys            => sysclk_pd,
      clk_tdc            => clk_tdc_400m,
      reset              => rst,
      -- Reference clock
      ref_clock          => clk_ref_2m,
      -- Comparator input from differential GPIO
      comparator_in      => s_comparator_out(0),
      -- DAC output to external pin (1k + 1nF RC filter, shorted to ANALOG_IN_N)
      dac_out_bit        => w_dac_bit,
      -- Optional trigger input (always enabled for ADC sampling)
      trigger_enable     => '1',
      -- Open-loop test mode (not used in production)
      open_loop_dac_duty => '0',
      -- Sample output
      sample_data        => adc_sample_data,
      sample_valid       => adc_sample_valid,
      -- Debug outputs (not monitored in production)
      debug_tdc_out      => open,
      debug_tdc_valid    => open,
      -- TDC Monitor outputs (not used)
      tdc_monitor_code   => open,
      tdc_monitor_center => open,
      tdc_monitor_diff   => open,
      tdc_monitor_dac    => open,
      tdc_monitor_valid  => open,
      -- Runtime control
      disable_tdc_contrib => disable_tdc_contrib,
      disable_eq_filter   => disable_eq_filter,
      disable_lp_filter   => disable_lp_filter
    );

  -- ========================================================================
  -- Trigger Edge Detection (for external trigger input)
  -- ========================================================================
  p_trigger_sync : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      if rst = '1' then
        trigger_sync <= (others => '0');
      else
        trigger_sync <= trigger_sync(1 downto 0) & TRIGGER_IN;
      end if;
    end if;
  end process;
  trigger_edge <= trigger_sync(1) and not trigger_sync(2);  -- Rising edge detect

  -- ========================================================================
  -- UART Command Decoder
  -- Commands: 'C' = start capture, 'D' = dump buffer
  -- ========================================================================
  p_uart_cmd : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      if rst = '1' then
        capture_start <= '0';
        capture_dump  <= '0';
      else
        capture_start <= '0';  -- Default: single-cycle pulses
        capture_dump  <= '0';
        
        -- External trigger starts capture
        if trigger_edge = '1' then
          capture_start <= '1';
        end if;
        
        -- UART commands
        if uart_rx_valid = '1' then
          case uart_rx_data is
            when x"43" | x"63" =>  -- 'C' or 'c' = Capture
              capture_start <= '1';
            when x"44" | x"64" =>  -- 'D' or 'd' = Dump
              capture_dump <= '1';
            when x"58" | x"78" =>  -- 'X' or 'x' = Toggle TDC contribution
              disable_tdc_contrib <= not disable_tdc_contrib;
            when x"45" | x"65" =>  -- 'E' or 'e' = Toggle EQ filter
              disable_eq_filter <= not disable_eq_filter;
            when x"4C" | x"6C" =>  -- 'L' or 'l' = Toggle LP filter
              disable_lp_filter <= not disable_lp_filter;
            when others =>
              null;
          end case;
        end if;
        
        -- Auto-dump after capture complete (optional behavior)
        if capture_done = '1' and dump_active = '0' then
          capture_dump <= '1';
        end if;
      end if;
    end if;
  end process;

  -- ========================================================================
  -- Capture Module (optional - controlled by generic)
  -- ========================================================================
  g_capture : if GC_CAPTURE_ENABLED generate
    i_capture : entity work.adc_capture
      generic map(
        GC_DATA_WIDTH => C_ADC_DATA_WIDTH,
        GC_DEPTH      => GC_CAPTURE_DEPTH,
        GC_ADDR_WIDTH => C_CAPTURE_ADDR_W
      )
      port map(
        clk           => sysclk_pd,
        rst           => rst,
        sample_data   => adc_sample_data,
        sample_valid  => adc_sample_valid,
        start_capture => capture_start,
        start_dump    => capture_dump,
        capturing     => capture_active,
        capture_done  => capture_done,
        dumping       => dump_active,
        dump_done     => open,  -- Status signal available but not currently monitored
        sample_count  => open,
        dump_data     => dump_data,
        dump_valid    => dump_valid,
        dump_ready    => dump_ready
      );
    
    -- Mux: During dump, send captured data; otherwise send live samples
    streamer_data  <= dump_data  when dump_active = '1' else adc_sample_data;
    streamer_valid <= dump_valid when dump_active = '1' else adc_sample_valid;
    dump_ready     <= uart_tx_ready when dump_active = '1' else '0';
  end generate;

  g_no_capture : if not GC_CAPTURE_ENABLED generate
    -- No capture module - direct connection
    streamer_data  <= adc_sample_data;
    streamer_valid <= adc_sample_valid;
    capture_active <= '0';
    capture_done   <= '0';
    dump_active    <= '0';
  end generate;

  -- UART streamer for ADC samples (live or dump mode)
  i_uart_streamer : entity work.uart_sample_streamer
    generic map(
      GC_DATA_WIDTH  => C_ADC_DATA_WIDTH,
      GC_BINARY_MODE => GC_UART_BINARY
    )
    port map(
      clk           => sysclk_pd,
      rst           => rst,
      sample_data   => streamer_data,
      sample_valid  => streamer_valid,
      uart_tx_data  => uart_tx_data,
      uart_tx_valid => uart_tx_valid,
      uart_tx_ready => uart_tx_ready
    );

end architecture rtl;
