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
    -- ADC Decimation: 50MHz / 128 ≈ 390 kS/s for burst capture
    -- In burst mode, samples stored in RAM at full rate (~97.7 kS/s)
    -- During dump, transmitted at UART-limited rate (~1.9 kS/s)
    -- Stream mode not recommended at this rate (UART will drop samples)
    GC_ADC_DECIMATION  : positive := 512; -- Decimation for burst mode (97.7 kS/s, ~1mV noise)
    GC_UART_BINARY     : boolean  := false; -- false=ASCII hex (human readable)
    GC_CAPTURE_DEPTH   : positive := 131072; -- Burst capture buffer depth (128K samples)
    GC_CAPTURE_ENABLED : boolean  := true -- Enable burst capture mode
  );
  port(
    -- Clock and Reset
    CLK_25M_C    : in  std_logic;
    -- UART
    UART_TX      : out std_logic;
    UART_RX      : in  std_logic := '1'; -- UART receive for commands
    -- Differential analog input (Quartus auto-assigns N-pin for differential)
    ANALOG_IN    : in  std_logic;       -- LVDS differential input (P-pin, N-pin auto-assigned)
    -- Feedback output
    FEEDBACK_OUT : out std_logic;       -- Feedback DAC output

    -- Debug
    TEST_PIN     : out std_logic;
    LED1         : out std_logic;       -- Debug LED (TDC valid indicator)
    USER_BTN     : in  std_logic;       -- Active low reset
    -- Optional trigger (directly triggers capture when GC_CAPTURE_ENABLED)
    TRIGGER_IN   : in  std_logic := '0'; -- Active high trigger for capture start
    -- Power Control
    VSEL_1V3     : out std_logic        -- Select 1.3V VADJ for CRUVI
  );
end entity;

architecture rtl of axe5000_top is

  constant C_ADC_DATA_WIDTH : positive := 16;
  constant C_CAPTURE_ADDR_W : positive := 17; -- log2(131072)

  signal sysclk_pd     : std_logic;     -- 100 MHz system clock from PLL
  signal clk_tdc_400m  : std_logic;     -- 400 MHz TDC clock from PLL
  signal clk_ref_2m    : std_logic;     -- 50 MHz reference clock from PLL
  signal rst_n_from_pd : std_logic;     -- Active-low reset from Platform Designer reset bridge
  signal rst           : std_logic;     -- Active-high reset for RTL modules

  signal adc_sample_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal adc_sample_valid : std_logic;
  signal adc_ready        : std_logic;  -- ADC is in closed-loop and producing valid samples

  signal uart_tx_data  : std_logic_vector(7 downto 0);
  signal uart_tx_valid : std_logic;
  signal uart_tx_ready : std_logic;

  -- GPIO IP signals
  signal s_comparator_out : std_logic_vector(0 downto 0); -- Differential comparator output from GPIO IP
  signal w_dac_bit        : std_logic;  -- DAC bit output to FEEDBACK_OUT

  -- Capture module signals
  signal capture_start  : std_logic := '0';
  signal capture_dump   : std_logic := '0';
  signal capture_active : std_logic;
  signal capture_done   : std_logic;
  signal dump_active    : std_logic;
  signal dump_data      : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal dump_valid     : std_logic;
  signal dump_ready     : std_logic;

  -- UART mux signals (live vs dump mode)
  signal streamer_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal streamer_valid : std_logic;
  signal streamer_ready : std_logic;

  -- Burst done extra newline signals
  signal dump_done        : std_logic;
  signal send_extra_lf    : std_logic := '0';
  signal extra_lf_state   : unsigned(1 downto 0) := "00"; -- 0=idle, 1=send LF, 2=wait
  
  -- UART output mux (streamer vs extra LF)
  signal streamer_tx_data  : std_logic_vector(7 downto 0);
  signal streamer_tx_valid : std_logic;
  signal streamer_tx_ready : std_logic;

  -- TDC Monitor signals
  signal tdc_monitor_code   : signed(15 downto 0);
  signal tdc_monitor_center : signed(15 downto 0);
  signal tdc_monitor_diff   : signed(15 downto 0);
  signal tdc_monitor_dac    : std_logic;
  signal tdc_monitor_valid  : std_logic;
  signal tdc_monitor_enable : std_logic := '0'; -- Enable TDC monitor mode (toggled via 'M' command)
  signal tdc_mon_tx_data    : std_logic_vector(7 downto 0);
  signal tdc_mon_tx_valid   : std_logic;
  signal tdc_mon_tx_ready   : std_logic;

  -- Runtime control
  signal disable_tdc_contrib : std_logic            := '0'; -- 0=TDC enabled, 1=TDC disabled (CIC-only)
  signal negate_tdc_contrib  : std_logic            := '0'; -- 0=normal TDC sign, 1=inverted TDC sign
  signal tdc_scale_shift     : unsigned(2 downto 0) := "000"; -- TDC gain shift: 000=1x to 111=128x
  signal disable_eq_filter   : std_logic            := '0'; -- 0=EQ enabled, 1=EQ bypassed
  signal disable_lp_filter   : std_logic            := '0'; -- 0=LP enabled, 1=LP bypassed
  signal short_dump_mode     : std_logic            := '0'; -- 0=full buffer (131K), 1=short (4K samples)
  signal level_trigger_mode  : std_logic            := '0'; -- 0=edge trigger, 1=level trigger

  -- Mode control: Burst vs Stream
  signal burst_mode      : std_logic := '1'; -- 1=burst mode (default), 0=stream mode
  signal burst_mode_prev : std_logic := '1'; -- For edge detection
  signal mode_changed    : std_logic := '0'; -- Pulse when mode changes

  -- UART RX command decoder
  signal uart_rx_data      : std_logic_vector(7 downto 0);
  signal uart_rx_valid     : std_logic;
  signal trigger_sync      : std_logic_vector(3 downto 0) := (others => '0'); -- 4-stage for async input
  signal trigger_edge      : std_logic;
  signal trigger_pending   : std_logic                    := '0'; -- Sticky latch for missed triggers
  signal capture_done_prev : std_logic                    := '0'; -- For edge detection
  signal capture_done_edge : std_logic                    := '0'; -- Rising edge of capture_done

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
  LED1         <= trigger_pending;      -- LED shows trigger latch state (goes high when trigger detected, clears when capture starts)
  FEEDBACK_OUT <= w_dac_bit;            -- DAC output with external RC filter
  VSEL_1V3     <= '1';                  -- Select 1.3V VADJ for CRUVI

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
      GC_FAST_SIM   => false,
      GC_OPEN_LOOP  => false            -- Normal closed-loop operation
    )
    port map(
      clk_sys             => sysclk_pd,
      clk_tdc             => clk_tdc_400m,
      reset               => rst,
      -- Reference clock
      ref_clock           => clk_ref_2m,
      -- Comparator input from differential GPIO
      comparator_in       => s_comparator_out(0),
      -- DAC output to external pin (1k + 1nF RC filter, shorted to ANALOG_IN_N)
      dac_out_bit         => w_dac_bit,
      -- Optional trigger input (always enabled for ADC sampling)
      trigger_enable      => '1',
      -- Open-loop test mode (not used in production)
      open_loop_dac_duty  => '0',
      -- Sample output
      sample_data         => adc_sample_data,
      sample_valid        => adc_sample_valid,
      -- Debug outputs (not monitored in production)
      debug_tdc_out       => open,
      debug_tdc_valid     => open,
      -- TDC Monitor outputs (for TDC sanity check/debug)
      tdc_monitor_code    => tdc_monitor_code,
      tdc_monitor_center  => tdc_monitor_center,
      tdc_monitor_diff    => tdc_monitor_diff,
      tdc_monitor_dac     => tdc_monitor_dac,
      tdc_monitor_valid   => tdc_monitor_valid,
      -- Runtime control
      disable_tdc_contrib => disable_tdc_contrib,
      negate_tdc_contrib  => negate_tdc_contrib,
      tdc_scale_shift     => tdc_scale_shift,
      disable_eq_filter   => disable_eq_filter,
      disable_lp_filter   => disable_lp_filter,
      -- Status
      adc_ready           => adc_ready
    );

  -- ========================================================================
  -- Trigger Edge Detection (for external trigger input)
  -- ========================================================================
  p_trigger_sync : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      if rst = '1' then
        trigger_sync    <= (others => '0');
        trigger_pending <= '0';
        burst_mode_prev <= '1';
        mode_changed    <= '0';
      else
        -- 4-stage synchronizer for async external trigger
        trigger_sync <= trigger_sync(2 downto 0) & TRIGGER_IN;

        -- Detect mode changes (for state reset)
        burst_mode_prev <= burst_mode;
        mode_changed    <= burst_mode xor burst_mode_prev;

        -- Sticky trigger latch: set on edge, clear when capture starts or mode changes
        -- Only latch trigger edges when in burst mode
        if mode_changed = '1' then
          -- Clear pending trigger when switching modes to avoid stale triggers
          trigger_pending <= '0';
        elsif trigger_edge = '1' and burst_mode = '1' then
          -- Only latch trigger in burst mode
          trigger_pending <= '1';
        elsif capture_active = '1' then
          -- Clear pending once capture is actually running
          trigger_pending <= '0';
        end if;
      end if;
    end if;
  end process;
  -- Trigger detection: edge or level mode
  -- Edge mode: rising edge detect on synchronized signal (use stages 2 and 3 for clean edge)
  -- Level mode: just use synchronized signal directly
  trigger_edge <= trigger_sync(3) when level_trigger_mode = '1' else
                  (trigger_sync(2) and not trigger_sync(3));

  -- ========================================================================
  -- UART Command Decoder
  -- Commands:
  --   'C' = start Capture (burst mode)
  --   'D' = Dump buffer
  --   'B' = switch to Burst mode (capture-then-dump)
  --   'S' = switch to Stream mode (continuous output, drops samples if UART busy)
  --   'X' = toggle TDC contribution
  --   'N' = toggle TDC sign (negate)
  --   'E' = toggle EQ filter
  --   'L' = toggle LP filter
  -- ========================================================================
  p_uart_cmd : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      if rst = '1' then
        capture_start       <= '0';
        capture_dump        <= '0';
        disable_tdc_contrib <= '0';
        negate_tdc_contrib  <= '0';
        tdc_scale_shift     <= "000";   -- Default: 1x TDC gain
        disable_eq_filter   <= '0';
        disable_lp_filter   <= '0';
        short_dump_mode     <= '0';     -- Default: full buffer dump
        level_trigger_mode  <= '0';     -- Default: edge trigger
        tdc_monitor_enable  <= '0';     -- Default: TDC monitor disabled
        burst_mode          <= '1';     -- Default: burst mode
        capture_done_prev   <= '0';
        capture_done_edge   <= '0';
      else
        capture_start <= '0';           -- Default: single-cycle pulses
        capture_dump  <= '0';

        -- External trigger starts capture (use pending latch for reliability)
        -- Start capture when: in burst mode AND pending trigger AND not currently capturing/dumping
        if burst_mode = '1' and (trigger_edge = '1' or trigger_pending = '1') and capture_active = '0' and dump_active = '0' then
          capture_start <= '1';
        end if;

        -- UART commands
        if uart_rx_valid = '1' then
          case uart_rx_data is
            when x"43" | x"63" =>       -- 'C' or 'c' = Capture
              capture_start <= '1';
            when x"44" | x"64" =>       -- 'D' or 'd' = Dump
              capture_dump <= '1';
            when x"42" | x"62" =>       -- 'B' or 'b' = Burst mode
              burst_mode <= '1';
            when x"53" | x"73" =>       -- 'S' or 's' = Stream mode
              burst_mode <= '0';
            when x"58" | x"78" =>       -- 'X' or 'x' = Toggle TDC contribution
              disable_tdc_contrib <= not disable_tdc_contrib;
            when x"4E" | x"6E" =>       -- 'N' or 'n' = Toggle TDC sign (negate)
              negate_tdc_contrib <= not negate_tdc_contrib;
            when x"45" | x"65" =>       -- 'E' or 'e' = Toggle EQ filter
              disable_eq_filter <= not disable_eq_filter;
            when x"4C" | x"6C" =>       -- 'L' or 'l' = Toggle LP filter
              disable_lp_filter <= not disable_lp_filter;
            when x"30" =>               -- '0' = Disable fast dump (full 131K)
              short_dump_mode <= '0';
            when x"31" =>               -- '1' = Enable fast dump (4K only)
              short_dump_mode <= '1';
            when x"32" =>               -- '2' = Set TDC gain to 1x
              tdc_scale_shift <= "000";
            when x"33" =>               -- '3' = Set TDC gain to 2x
              tdc_scale_shift <= "001";
            when x"34" =>               -- '4' = Set TDC gain to 4x
              tdc_scale_shift <= "010";
            when x"35" =>               -- '5' = Set TDC gain to 8x
              tdc_scale_shift <= "011";
            when x"36" =>               -- '6' = Set TDC gain to 16x
              tdc_scale_shift <= "100";
            when x"37" =>               -- '7' = Set TDC gain to 32x
              tdc_scale_shift <= "101";
            when x"38" =>               -- '8' = Set TDC gain to 64x
              tdc_scale_shift <= "110";
            when x"39" =>               -- '9' = Set TDC gain to 128x
              tdc_scale_shift <= "111";
            when x"54" | x"74" =>       -- 'T' or 't' = Toggle Trigger mode (edge vs level)
              level_trigger_mode <= not level_trigger_mode;
            when x"4D" | x"6D" =>       -- 'M' or 'm' = Toggle TDC Monitor mode
              tdc_monitor_enable <= not tdc_monitor_enable;
            when others =>
              null;
          end case;
        end if;

        -- Auto-dump after capture complete (burst mode only)
        -- Use edge detection to only trigger once per capture
        capture_done_prev <= capture_done;
        capture_done_edge <= capture_done and not capture_done_prev;

        if burst_mode = '1' and capture_done_edge = '1' and dump_active = '0' then
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
        capture_reset => mode_changed,  -- Clear stale state on mode switch
        short_dump    => short_dump_mode, -- '1' = dump last 4K only
        capturing     => capture_active,
        capture_done  => capture_done,
        dumping       => dump_active,
        dump_done     => dump_done,     -- Used to send extra newline when dump completes
        sample_count  => open,
        dump_data     => dump_data,
        dump_valid    => dump_valid,
        dump_ready    => dump_ready
      );

    -- Mux: During dump, send captured data
    -- In burst mode: nothing sent until dump command
    -- In stream mode: send live samples continuously
    streamer_data  <= dump_data when dump_active = '1' else adc_sample_data;
    streamer_valid <= dump_valid when dump_active = '1' else
                      adc_sample_valid when burst_mode = '0' else
                      '0';              -- Burst mode: suppress output until dump
    dump_ready     <= streamer_ready when dump_active = '1' else '0';
  end generate;

  -- ========================================================================
  -- TDC Monitor UART Streamer
  -- When tdc_monitor_enable is active, streams TDC diagnostic packets
  -- ========================================================================
  i_tdc_monitor_streamer : entity work.tdc_monitor_uart_streamer
    generic map(
      GC_TDC_WIDTH  => 16,
      GC_ADC_WIDTH  => C_ADC_DATA_WIDTH,
      GC_DECIMATION => 256  -- ~380 packets/s at 97kHz sample rate
    )
    port map(
      clk                => sysclk_pd,
      reset              => rst,
      tdc_monitor_code   => tdc_monitor_code,
      tdc_monitor_center => tdc_monitor_center,
      tdc_monitor_diff   => tdc_monitor_diff,
      tdc_monitor_dac    => tdc_monitor_dac,
      tdc_monitor_valid  => tdc_monitor_valid and tdc_monitor_enable,
      adc_data_out       => signed(adc_sample_data),
      uart_tx_data       => tdc_mon_tx_data,
      uart_tx_valid      => tdc_mon_tx_valid,
      uart_tx_ready      => tdc_mon_tx_ready
    );
  
  -- TDC monitor ready: only when enabled and UART ready
  tdc_mon_tx_ready <= uart_tx_ready when tdc_monitor_enable = '1' else '0';

  -- ========================================================================
  -- Extra Newline After Burst Dump
  -- When dump completes, send an extra LF to mark end of burst
  -- This helps Python scripts detect burst completion reliably
  -- ========================================================================
  p_extra_newline : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      if rst = '1' then
        send_extra_lf  <= '0';
        extra_lf_state <= "00";
      else
        send_extra_lf <= '0';  -- Default: single cycle pulse
        
        case extra_lf_state is
          when "00" =>  -- Idle: wait for dump_done
            if dump_done = '1' then
              extra_lf_state <= "01";
            end if;
            
          when "01" =>  -- Send LF: wait for UART ready
            if uart_tx_ready = '1' then
              send_extra_lf  <= '1';
              extra_lf_state <= "10";
            end if;
            
          when "10" =>  -- Wait for UART to accept LF
            if uart_tx_ready = '1' then
              extra_lf_state <= "00";
            end if;
            
          when others =>
            extra_lf_state <= "00";
        end case;
      end if;
    end if;
  end process;

  g_no_capture : if not GC_CAPTURE_ENABLED generate
    -- No capture module - direct connection
    streamer_data  <= adc_sample_data;
    streamer_valid <= adc_sample_valid;
    capture_active <= '0';
    capture_done   <= '0';
    dump_active    <= '0';
    dump_done      <= '0';
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
      uart_tx_data  => streamer_tx_data,
      uart_tx_valid => streamer_tx_valid,
      uart_tx_ready => streamer_tx_ready,
      ready         => streamer_ready
    );

  -- ========================================================================
  -- UART Output Mux: Streamer vs TDC Monitor vs Extra LF
  -- Priority: Extra LF > TDC Monitor > Sample Streamer
  -- ========================================================================
  uart_tx_data  <= x"0A" when send_extra_lf = '1' else
                   tdc_mon_tx_data when tdc_monitor_enable = '1' else
                   streamer_tx_data;
  uart_tx_valid <= send_extra_lf or 
                   (tdc_mon_tx_valid and tdc_monitor_enable) or 
                   (streamer_tx_valid and not tdc_monitor_enable);
  streamer_tx_ready <= uart_tx_ready when (send_extra_lf = '0' and tdc_monitor_enable = '0') else '0';

end architecture rtl;
