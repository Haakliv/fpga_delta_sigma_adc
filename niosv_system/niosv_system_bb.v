module niosv_system (
		input  wire        clk_25m_clk,               //           clk_25m.clk
		input  wire [1:0]  dip_sw_export,             //            dip_sw.export
		input  wire        uart_rxd,                  //              uart.rxd
		output wire        uart_txd,                  //                  .txd
		input  wire        mm_ccb_0_m0_clk_clk,       //   mm_ccb_0_m0_clk.clk
		input  wire        mm_ccb_0_m0_reset_reset,   // mm_ccb_0_m0_reset.reset
		input  wire        mm_ccb_0_m0_waitrequest,   //       mm_ccb_0_m0.waitrequest
		input  wire [31:0] mm_ccb_0_m0_readdata,      //                  .readdata
		input  wire        mm_ccb_0_m0_readdatavalid, //                  .readdatavalid
		output wire [0:0]  mm_ccb_0_m0_burstcount,    //                  .burstcount
		output wire [31:0] mm_ccb_0_m0_writedata,     //                  .writedata
		output wire [15:0] mm_ccb_0_m0_address,       //                  .address
		output wire        mm_ccb_0_m0_write,         //                  .write
		output wire        mm_ccb_0_m0_read,          //                  .read
		output wire [3:0]  mm_ccb_0_m0_byteenable,    //                  .byteenable
		output wire        mm_ccb_0_m0_debugaccess,   //                  .debugaccess
		output wire [2:0]  pwm_out_conduit,           //           pwm_out.conduit
		input  wire        reset_n_reset_n,           //           reset_n.reset_n
		output wire        sysclk_clk                 //            sysclk.clk
	);
endmodule

