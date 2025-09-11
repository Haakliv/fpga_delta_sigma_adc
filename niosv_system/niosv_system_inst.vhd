	component niosv_system is
		port (
			clk_25m_clk               : in  std_logic                     := 'X';             -- clk
			dip_sw_export             : in  std_logic_vector(1 downto 0)  := (others => 'X'); -- export
			uart_rxd                  : in  std_logic                     := 'X';             -- rxd
			uart_txd                  : out std_logic;                                        -- txd
			mm_ccb_0_m0_clk_clk       : in  std_logic                     := 'X';             -- clk
			mm_ccb_0_m0_reset_reset   : in  std_logic                     := 'X';             -- reset
			mm_ccb_0_m0_waitrequest   : in  std_logic                     := 'X';             -- waitrequest
			mm_ccb_0_m0_readdata      : in  std_logic_vector(31 downto 0) := (others => 'X'); -- readdata
			mm_ccb_0_m0_readdatavalid : in  std_logic                     := 'X';             -- readdatavalid
			mm_ccb_0_m0_burstcount    : out std_logic_vector(0 downto 0);                     -- burstcount
			mm_ccb_0_m0_writedata     : out std_logic_vector(31 downto 0);                    -- writedata
			mm_ccb_0_m0_address       : out std_logic_vector(15 downto 0);                    -- address
			mm_ccb_0_m0_write         : out std_logic;                                        -- write
			mm_ccb_0_m0_read          : out std_logic;                                        -- read
			mm_ccb_0_m0_byteenable    : out std_logic_vector(3 downto 0);                     -- byteenable
			mm_ccb_0_m0_debugaccess   : out std_logic;                                        -- debugaccess
			pwm_out_conduit           : out std_logic_vector(2 downto 0);                     -- conduit
			reset_n_reset_n           : in  std_logic                     := 'X';             -- reset_n
			sysclk_clk                : out std_logic                                         -- clk
		);
	end component niosv_system;

	u0 : component niosv_system
		port map (
			clk_25m_clk               => CONNECTED_TO_clk_25m_clk,               --           clk_25m.clk
			dip_sw_export             => CONNECTED_TO_dip_sw_export,             --            dip_sw.export
			uart_rxd                  => CONNECTED_TO_uart_rxd,                  --              uart.rxd
			uart_txd                  => CONNECTED_TO_uart_txd,                  --                  .txd
			mm_ccb_0_m0_clk_clk       => CONNECTED_TO_mm_ccb_0_m0_clk_clk,       --   mm_ccb_0_m0_clk.clk
			mm_ccb_0_m0_reset_reset   => CONNECTED_TO_mm_ccb_0_m0_reset_reset,   -- mm_ccb_0_m0_reset.reset
			mm_ccb_0_m0_waitrequest   => CONNECTED_TO_mm_ccb_0_m0_waitrequest,   --       mm_ccb_0_m0.waitrequest
			mm_ccb_0_m0_readdata      => CONNECTED_TO_mm_ccb_0_m0_readdata,      --                  .readdata
			mm_ccb_0_m0_readdatavalid => CONNECTED_TO_mm_ccb_0_m0_readdatavalid, --                  .readdatavalid
			mm_ccb_0_m0_burstcount    => CONNECTED_TO_mm_ccb_0_m0_burstcount,    --                  .burstcount
			mm_ccb_0_m0_writedata     => CONNECTED_TO_mm_ccb_0_m0_writedata,     --                  .writedata
			mm_ccb_0_m0_address       => CONNECTED_TO_mm_ccb_0_m0_address,       --                  .address
			mm_ccb_0_m0_write         => CONNECTED_TO_mm_ccb_0_m0_write,         --                  .write
			mm_ccb_0_m0_read          => CONNECTED_TO_mm_ccb_0_m0_read,          --                  .read
			mm_ccb_0_m0_byteenable    => CONNECTED_TO_mm_ccb_0_m0_byteenable,    --                  .byteenable
			mm_ccb_0_m0_debugaccess   => CONNECTED_TO_mm_ccb_0_m0_debugaccess,   --                  .debugaccess
			pwm_out_conduit           => CONNECTED_TO_pwm_out_conduit,           --           pwm_out.conduit
			reset_n_reset_n           => CONNECTED_TO_reset_n_reset_n,           --           reset_n.reset_n
			sysclk_clk                => CONNECTED_TO_sysclk_clk                 --            sysclk.clk
		);

