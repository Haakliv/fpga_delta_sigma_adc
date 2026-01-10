library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axe5000_top is
  generic(
    GC_ADC_DECIMATION : positive := 512;
    GC_UART_BINARY    : boolean  := false
  );
  port(
    CLK_25M_C    : in  std_logic;
    UART_TX      : out std_logic;
    UART_RX      : in  std_logic := '1';
    ANALOG_IN    : in  std_logic;
    FEEDBACK_OUT : out std_logic;
    TEST_PIN     : out std_logic;
    LED1         : out std_logic;
    USER_BTN     : in  std_logic;
    VSEL_1V3     : out std_logic
  );
end entity;

architecture rtl of axe5000_top is

  constant C_ADC_DATA_WIDTH  : positive := 16;
  constant C_CAPTURE_ENABLED : boolean  := false;
  constant C_CAPTURE_DEPTH   : positive := 131072;
  constant C_CAPTURE_ADDR_W  : positive := 17;

  signal sysclk_pd     : std_logic;
  signal clk_dsm       : std_logic;
  signal clk_tdc       : std_logic;
  signal rst_n_from_pd : std_logic;
  signal rst           : std_logic;

  signal adc_sample_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal adc_sample_valid : std_logic;

  signal uart_tx_data  : std_logic_vector(7 downto 0);
  signal uart_tx_valid : std_logic;
  signal uart_tx_ready : std_logic;

  signal s_comparator_out : std_logic_vector(0 downto 0);
  signal w_dac_bit        : std_logic;

  -- Capture module signals
  -- signal capture_start   : std_logic := '0';
  -- signal capture_dump    : std_logic := '0';
  -- signal capture_active  : std_logic;
  -- signal capture_done    : std_logic;
  -- signal dump_active     : std_logic;
  -- signal dump_data       : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  -- signal dump_valid      : std_logic;
  -- signal dump_ready      : std_logic;
  -- signal short_dump_mode : std_logic := '0';

  -- UART mux signals (live vs dump mode)
  signal streamer_data  : std_logic_vector(C_ADC_DATA_WIDTH - 1 downto 0);
  signal streamer_valid : std_logic;
  signal streamer_ready : std_logic;

  -- Burst done extra newline signals
  -- signal dump_done      : std_logic;
  -- signal send_extra_lf  : std_logic            := '0';
  -- signal extra_lf_state : unsigned(1 downto 0) := "00"; -- 0=idle, 1=send LF, 2=wait

  -- UART output mux (streamer vs extra LF)
  signal streamer_tx_data  : std_logic_vector(7 downto 0);
  signal streamer_tx_valid : std_logic;
  signal streamer_tx_ready : std_logic;

  -- Runtime control
  signal disable_tdc_contrib : std_logic := '0';
  signal negate_tdc_contrib  : std_logic := '0';
  signal disable_eq_filter   : std_logic := '0';
  signal disable_lp_filter   : std_logic := '0';
  signal level_trigger_mode  : std_logic := '0';

  -- Mode control: Burst vs Stream
  -- signal burst_mode      : std_logic := '1'; -- 1=burst mode (default), 0=stream mode
  -- signal burst_mode_prev : std_logic := '1'; -- For edge detection
  -- signal mode_changed    : std_logic := '0'; -- Pulse when mode changes

  -- UART RX command decoder
  signal uart_rx_data  : std_logic_vector(7 downto 0);
  signal uart_rx_valid : std_logic;

  component adc_system is               -- @suppress
    port(
      clk_25m_clk                    : in  std_logic                    := 'X';
      gpio_comp_dout_export          : out std_logic_vector(0 downto 0);
      gpio_comp_pad_in_export        : in  std_logic_vector(0 downto 0) := (others => 'X');
      gpio_comp_pad_in_b_export      : in  std_logic_vector(0 downto 0) := (others => 'X');
      iopll_outclk_tdc_clk           : out std_logic;
      iopll_outclk_dsm_clk           : out std_logic;
      reset_n_reset_n                : in  std_logic                    := 'X';
      sysrst_reset_n                 : out std_logic;
      data_receive_ready             : in  std_logic                    := 'X';
      data_receive_data              : out std_logic_vector(7 downto 0);
      data_receive_error             : out std_logic;
      data_receive_valid             : out std_logic;
      data_transmit_data             : in  std_logic_vector(7 downto 0) := (others => 'X');
      data_transmit_error            : in  std_logic                    := 'X';
      data_transmit_valid            : in  std_logic                    := 'X';
      data_transmit_ready            : out std_logic;
      rs232_0_external_interface_RXD : in  std_logic                    := 'X'; -- @suppress
      rs232_0_external_interface_TXD : out std_logic; -- @suppress
      sysclk_clk                     : out std_logic
    );
  end component adc_system;

begin

  TEST_PIN     <= adc_sample_valid xor (xor adc_sample_data);
  LED1         <= '0';
  FEEDBACK_OUT <= w_dac_bit;
  VSEL_1V3     <= '1';

  i_niosv : adc_system
    port map(
      clk_25m_clk                    => CLK_25M_C,
      reset_n_reset_n                => USER_BTN,
      sysrst_reset_n                 => rst_n_from_pd,
      gpio_comp_dout_export          => s_comparator_out,
      gpio_comp_pad_in_export(0)     => ANALOG_IN,
      gpio_comp_pad_in_b_export(0)   => '0',
      iopll_outclk_tdc_clk           => clk_tdc, -- 400 MHz clock for TDC
      iopll_outclk_dsm_clk           => clk_dsm, -- 50 MHz clock for ADC sampling
      sysclk_clk                     => sysclk_pd,
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

  rst <= not rst_n_from_pd;

  -- i_adc : entity work.tdc_adc_top
  --   generic map(
  --     GC_DECIMATION => GC_ADC_DECIMATION,
  --     GC_DATA_WIDTH => C_ADC_DATA_WIDTH,
  --     GC_TDC_OUTPUT => 16,
  --     GC_SIM        => false,
  --     GC_FAST_SIM   => false,
  --     GC_OPEN_LOOP  => false            -- Normal closed-loop operation
  --   )
  --   port map(
  --     clk_sys             => sysclk_pd,
  --     clk_tdc             => clk_tdc,
  --     reset               => rst,
  --     -- Reference clock
  --     ref_clock           => clk_dsm,
  --     -- Comparator input from differential GPIO
  --     comparator_in       => s_comparator_out(0),
  --     -- DAC output to external pin (1k + 1nF RC filter, shorted to ANALOG_IN_N)
  --     dac_out_bit         => w_dac_bit,
  --     -- Optional trigger input (always enabled for ADC sampling)
  --     trigger_enable      => '1',
  --     -- Open-loop test mode (not used in production)
  --     open_loop_dac_duty  => '0',
  --     -- Sample output
  --     sample_data         => adc_sample_data,
  --     sample_valid        => adc_sample_valid,
  --     -- Debug outputs (not monitored in production)
  --     debug_tdc_out       => open,
  --     debug_tdc_valid     => open,
  --     -- TDC Monitor outputs (for TDC sanity check/debug)
  --     tdc_monitor_code    => open,
  --     tdc_monitor_center  => open,
  --     tdc_monitor_diff    => open,
  --     tdc_monitor_dac     => open,
  --     tdc_monitor_valid   => open,
  --     -- Runtime control
  --     disable_tdc_contrib => disable_tdc_contrib,
  --     negate_tdc_contrib  => negate_tdc_contrib,
  --     tdc_scale_shift     => "000",
  --     disable_eq_filter   => disable_eq_filter,
  --     disable_lp_filter   => disable_lp_filter,
  --     -- Status
  --     adc_ready           => open
  --   );

  i_adc : entity work.rc_adc_top
    generic map(
      GC_DECIMATION => GC_ADC_DECIMATION,
      GC_DATA_WIDTH => C_ADC_DATA_WIDTH
    )
    port map(
      clk            => clk_dsm,        -- 50 MHz sampling clock
      clk_fast       => clk_tdc,        -- 400 MHz fast clock
      clk_sys        => sysclk_pd,      -- 100 MHz system clock
      reset          => rst,
      analog_in      => s_comparator_out(0),
      dac_out        => w_dac_bit,
      trigger_enable => '1',
      sample_data    => adc_sample_data,
      sample_valid   => adc_sample_valid
    );

  -- p_trigger_sync : process(sysclk_pd)
  -- begin
  --   if rising_edge(sysclk_pd) then
  --     if rst = '1' then
  --       trigger_sync    <= (others => '0');
  --       trigger_pending <= '0';
  --       burst_mode_prev <= '1';
  --       mode_changed    <= '0';
  --     else
  --       -- 4-stage synchronizer for async external trigger
  --       trigger_sync <= trigger_sync(2 downto 0) & TRIGGER_IN;
  --
  --       -- Detect mode changes (for state reset)
  --       burst_mode_prev <= burst_mode;
  --       mode_changed    <= burst_mode xor burst_mode_prev;
  --
  --       -- Sticky trigger latch: set on edge, clear when capture starts or mode changes
  --       -- Only latch trigger edges when in burst mode
  --       if mode_changed = '1' then
  --         -- Clear pending trigger when switching modes to avoid stale triggers
  --         trigger_pending <= '0';
  --       elsif trigger_edge = '1' and burst_mode = '1' then
  --         -- Only latch trigger in burst mode
  --         trigger_pending <= '1';
  --       elsif capture_active = '1' then
  --         -- Clear pending once capture is actually running
  --         trigger_pending <= '0';
  --       end if;
  --     end if;
  --   end if;
  -- end process;
  -- -- Trigger detection: edge or level mode
  -- -- Edge mode: rising edge detect on synchronized signal (use stages 2 and 3 for clean edge)
  -- -- Level mode: just use synchronized signal directly
  -- trigger_edge <= trigger_sync(3) when level_trigger_mode = '1' else
  --                 (trigger_sync(2) and not trigger_sync(3));

  p_uart_cmd : process(sysclk_pd)
  begin
    if rising_edge(sysclk_pd) then
      -- Default values for pulsed signals
      -- capture_start <= '0';
      -- capture_dump  <= '0';
      -- mode_changed  <= '0';

      if rst = '1' then
        disable_tdc_contrib <= '0';
        negate_tdc_contrib  <= '0';
        disable_eq_filter   <= '0';
        disable_lp_filter   <= '0';
        level_trigger_mode  <= '0';
      -- burst_mode          <= '1';
      -- short_dump_mode     <= '0';
      else

        if uart_rx_valid = '1' then
          case uart_rx_data is
            when x"58" | x"78" =>
              disable_tdc_contrib <= not disable_tdc_contrib;
            when x"4E" | x"6E" =>
              negate_tdc_contrib <= not negate_tdc_contrib;
            when x"45" | x"65" =>
              disable_eq_filter <= not disable_eq_filter;
            when x"4C" | x"6C" =>
              disable_lp_filter <= not disable_lp_filter;
            when x"54" | x"74" =>
              level_trigger_mode <= not level_trigger_mode;

            -- New commands
            -- when x"42" | x"62" =>       -- B/b
            --   burst_mode   <= '1';      -- Explicit Burst Mode
            --   mode_changed <= '1';
            -- when x"53" | x"73" =>       -- S/s
            --   burst_mode   <= '0';      -- Explicit Stream Mode
            --   mode_changed <= '1';

            -- when x"30" =>               -- '0'
            --   short_dump_mode <= '0';   -- Full dump (128k)
            -- when x"31" =>               -- '1'
            --   short_dump_mode <= '1';   -- Fast dump (4k)

            -- when x"43" | x"63" =>       -- C/c
            --   if burst_mode = '1' then
            --     capture_start <= '1';
            --   end if;
            -- when x"44" | x"64" =>       -- D/d
            --   if burst_mode = '1' then
            --     capture_dump <= '1';
            --   end if;

            when others =>
              null;
          end case;
        end if;
      end if;
    end if;
  end process;

  -- g_capture : if GC_CAPTURE_ENABLED generate
  --   i_capture : entity work.adc_capture
  --     generic map(
  --       GC_DATA_WIDTH => C_ADC_DATA_WIDTH,
  --       GC_DEPTH      => GC_CAPTURE_DEPTH,
  --       GC_ADDR_WIDTH => C_CAPTURE_ADDR_W
  --     )
  --     port map(
  --       clk           => sysclk_pd,
  --       rst           => rst,
  --       sample_data   => adc_sample_data,
  --       sample_valid  => adc_sample_valid,
  --       start_capture => capture_start,
  --       start_dump    => capture_dump,
  --       capture_reset => mode_changed,  -- Clear stale state on mode switch
  --       short_dump    => short_dump_mode, -- '1' = dump last 4K only
  --       capturing     => capture_active,
  --       capture_done  => capture_done,
  --       dumping       => dump_active,
  --       dump_done     => dump_done,     -- Used to send extra newline when dump completes
  --       sample_count  => open,
  --       dump_data     => dump_data,
  --       dump_valid    => dump_valid,
  --       dump_ready    => dump_ready
  --     );

  --   -- Mux: During dump, send captured data
  --   -- In burst mode: nothing sent until dump command
  --   -- In stream mode: send live samples continuously
  --   streamer_data  <= dump_data when dump_active = '1' else adc_sample_data;
  --   streamer_valid <= dump_valid when dump_active = '1' else
  --                     adc_sample_valid when burst_mode = '0' else
  --                     '0';              -- Burst mode: suppress output until dump
  --   dump_ready     <= streamer_ready when dump_active = '1' else '0';
  -- end generate;

  streamer_data  <= adc_sample_data;
  streamer_valid <= adc_sample_valid;

  -- p_extra_newline : process(sysclk_pd)
  -- begin
  --   if rising_edge(sysclk_pd) then
  --     if rst = '1' then
  --       send_extra_lf  <= '0';
  --       extra_lf_state <= "00";
  --     else
  --       send_extra_lf <= '0';           -- Default: single cycle pulse

  --       case extra_lf_state is
  --         when "00" =>                  -- Idle: wait for dump_done
  --           if dump_done = '1' then
  --             extra_lf_state <= "01";
  --           end if;

  --         when "01" =>                  -- Send LF: wait for UART ready
  --           if uart_tx_ready = '1' then
  --             send_extra_lf  <= '1';
  --             extra_lf_state <= "10";
  --           end if;

  --         when "10" =>                  -- Wait for UART to accept LF
  --           if uart_tx_ready = '1' then
  --             extra_lf_state <= "00";
  --           end if;

  --         when others =>
  --           extra_lf_state <= "00";
  --       end case;
  --     end if;
  --   end if;
  -- end process;

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

  uart_tx_data      <= streamer_tx_data;
  uart_tx_valid     <= streamer_tx_valid;
  streamer_tx_ready <= uart_tx_ready;

  -- uart_tx_data      <= x"0A" when send_extra_lf = '1' else streamer_tx_data;
  -- uart_tx_valid     <= '1' when send_extra_lf = '1' else streamer_tx_valid;
  -- streamer_tx_ready <= uart_tx_ready when send_extra_lf = '0' else '0';

  -- uart_tx_data  <= (others => '0');
  -- uart_tx_valid <= '0';

end architecture rtl;
