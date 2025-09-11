	niosv_system u0 (
		.clk_25m_clk               (_connected_to_clk_25m_clk_),               //   input,   width = 1,           clk_25m.clk
		.dip_sw_export             (_connected_to_dip_sw_export_),             //   input,   width = 2,            dip_sw.export
		.uart_rxd                  (_connected_to_uart_rxd_),                  //   input,   width = 1,              uart.rxd
		.uart_txd                  (_connected_to_uart_txd_),                  //  output,   width = 1,                  .txd
		.mm_ccb_0_m0_clk_clk       (_connected_to_mm_ccb_0_m0_clk_clk_),       //   input,   width = 1,   mm_ccb_0_m0_clk.clk
		.mm_ccb_0_m0_reset_reset   (_connected_to_mm_ccb_0_m0_reset_reset_),   //   input,   width = 1, mm_ccb_0_m0_reset.reset
		.mm_ccb_0_m0_waitrequest   (_connected_to_mm_ccb_0_m0_waitrequest_),   //   input,   width = 1,       mm_ccb_0_m0.waitrequest
		.mm_ccb_0_m0_readdata      (_connected_to_mm_ccb_0_m0_readdata_),      //   input,  width = 32,                  .readdata
		.mm_ccb_0_m0_readdatavalid (_connected_to_mm_ccb_0_m0_readdatavalid_), //   input,   width = 1,                  .readdatavalid
		.mm_ccb_0_m0_burstcount    (_connected_to_mm_ccb_0_m0_burstcount_),    //  output,   width = 1,                  .burstcount
		.mm_ccb_0_m0_writedata     (_connected_to_mm_ccb_0_m0_writedata_),     //  output,  width = 32,                  .writedata
		.mm_ccb_0_m0_address       (_connected_to_mm_ccb_0_m0_address_),       //  output,  width = 16,                  .address
		.mm_ccb_0_m0_write         (_connected_to_mm_ccb_0_m0_write_),         //  output,   width = 1,                  .write
		.mm_ccb_0_m0_read          (_connected_to_mm_ccb_0_m0_read_),          //  output,   width = 1,                  .read
		.mm_ccb_0_m0_byteenable    (_connected_to_mm_ccb_0_m0_byteenable_),    //  output,   width = 4,                  .byteenable
		.mm_ccb_0_m0_debugaccess   (_connected_to_mm_ccb_0_m0_debugaccess_),   //  output,   width = 1,                  .debugaccess
		.pwm_out_conduit           (_connected_to_pwm_out_conduit_),           //  output,   width = 3,           pwm_out.conduit
		.reset_n_reset_n           (_connected_to_reset_n_reset_n_),           //   input,   width = 1,           reset_n.reset_n
		.sysclk_clk                (_connected_to_sysclk_clk_)                 //  output,   width = 1,            sysclk.clk
	);

