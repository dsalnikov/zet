module soc(
	//////////// ADC //////////
	output		          		ADC_CONVST,
	output		          		ADC_SCK,
	output		          		ADC_SDI,
	input 		          		ADC_SDO,

	//////////// ARDUINO //////////
	inout 		    [15:0]		ARDUINO_IO,
	inout 		          		ARDUINO_RESET_N,

	//////////// CLOCK //////////
	input 		          		FPGA_CLK1_50,
	input 		          		FPGA_CLK2_50,
	input 		          		FPGA_CLK3_50,

	//////////// HDMI //////////
	inout 		          		HDMI_I2C_SCL,
	inout 		          		HDMI_I2C_SDA,
	inout 		          		HDMI_I2S,
	inout 		          		HDMI_LRCLK,
	inout 		          		HDMI_MCLK,
	inout 		          		HDMI_SCLK,
	output		          		HDMI_TX_CLK,
	output		          		HDMI_TX_DE,
	output		    [23:0]		HDMI_TX_D,
	output		          		HDMI_TX_HS,
	input 		          		HDMI_TX_INT,
	output		          		HDMI_TX_VS,

	//////////// HPS //////////
	output		    [14:0]		HPS_DDR3_ADDR,
	output		     [2:0]		HPS_DDR3_BA,
	output		          		HPS_DDR3_CAS_N,
	output		          		HPS_DDR3_CKE,
	output		          		HPS_DDR3_CK_N,
	output		          		HPS_DDR3_CK_P,
	output		          		HPS_DDR3_CS_N,
	output		     [3:0]		HPS_DDR3_DM,
	inout 		    [31:0]		HPS_DDR3_DQ,
	inout 		     [3:0]		HPS_DDR3_DQS_N,
	inout 		     [3:0]		HPS_DDR3_DQS_P,
	output		          		HPS_DDR3_ODT,
	output		          		HPS_DDR3_RAS_N,
	output		          		HPS_DDR3_RESET_N,
	input 		          		HPS_DDR3_RZQ,
	output		          		HPS_DDR3_WE_N,


	//////////// KEY //////////
	input 		     [1:0]		KEY,

	//////////// LED //////////
	output		     [7:0]		LED,

	//////////// SW //////////
	input 		     [3:0]		SW,
	
	
	//loan io
	//sd card io
	inout hps_io_hps_io_gpio_inst_LOANIO36, //MOSI
	inout hps_io_hps_io_gpio_inst_LOANIO38, //MISO
	inout hps_io_hps_io_gpio_inst_LOANIO45, //CLK
	inout hps_io_hps_io_gpio_inst_LOANIO47, //CS
	//uart io
	inout hps_io_hps_io_gpio_inst_LOANIO49,
	inout hps_io_hps_io_gpio_inst_LOANIO50
	
);





wire sdram_clk;
wire [29:0] sdram0_data_address;
wire [7:0] sdram0_data_burstcount;
wire sdram0_data_waitrequest;
wire [31:0] sdram0_data_readdata;
wire sdram0_data_readdatavalid;
wire sdram0_data_read;
wire [31:0] sdram0_data_writedata;
wire [3:0] sdram0_data_byteenable;
wire sdram0_data_write;


wire [66:0] loan_io_in;
wire [66:0] loan_io_out;
wire [66:0] loan_io_oe;


//uart
assign loan_io_oe[50] = 1'b1;

wire uart_tx = loan_io_out[50];
wire uart_rx = loan_io_in[49];

assign loan_io_out[50] = uart_rx;



    soc_system u0 (
        .clk_clk                                            (FPGA_CLK1_50),   //                      clk.clk
        
		  .hps_0_f2h_boot_from_fpga_boot_from_fpga_ready      (1'b1), // hps_0_f2h_boot_from_fpga.boot_from_fpga_ready
        .hps_0_f2h_boot_from_fpga_boot_from_fpga_on_failure (1'b1), //                         .boot_from_fpga_on_failure
        
		  .hps_0_f2h_sdram0_clock_clk                         (sdram_clk),                         //   hps_0_f2h_sdram0_clock.clk
        .hps_0_f2h_sdram0_data_address                      (sdram0_data_address),                      //    hps_0_f2h_sdram0_data.address
        .hps_0_f2h_sdram0_data_burstcount                   (sdram0_data_burstcount),                   //                         .burstcount
        .hps_0_f2h_sdram0_data_waitrequest                  (sdram0_data_waitrequest),                  //                         .waitrequest
        .hps_0_f2h_sdram0_data_readdata                     (sdram0_data_readdata),                     //                         .readdata
        .hps_0_f2h_sdram0_data_readdatavalid                (sdram0_data_readdatavalid),                //                         .readdatavalid
        .hps_0_f2h_sdram0_data_read                         (sdram0_data_read),                         //                         .read
        .hps_0_f2h_sdram0_data_writedata                    (sdram0_data_writedata),                    //                         .writedata
        .hps_0_f2h_sdram0_data_byteenable                   (sdram0_data_byteenable),                   //                         .byteenable
        .hps_0_f2h_sdram0_data_write                        (sdram0_data_write),                        //                         .write
        
		  .hps_0_h2f_loan_io_in                               (loan_io_in),                               //        hps_0_h2f_loan_io.in
        .hps_0_h2f_loan_io_out                              (loan_io_out),                              //                         .out
        .hps_0_h2f_loan_io_oe                               (loan_io_oe),                               //                         .oe
        
		  .hps_io_hps_io_gpio_inst_LOANIO36                   (hps_io_hps_io_gpio_inst_LOANIO36),                   //                   hps_io.hps_io_gpio_inst_LOANIO36
        .hps_io_hps_io_gpio_inst_LOANIO38                   (hps_io_hps_io_gpio_inst_LOANIO38),                   //                         .hps_io_gpio_inst_LOANIO38
        .hps_io_hps_io_gpio_inst_LOANIO45                   (hps_io_hps_io_gpio_inst_LOANIO45),                   //                         .hps_io_gpio_inst_LOANIO45
        .hps_io_hps_io_gpio_inst_LOANIO47							(hps_io_hps_io_gpio_inst_LOANIO47),                   //                         .hps_io_gpio_inst_LOANIO47
		  
		  .hps_io_hps_io_gpio_inst_LOANIO49                   (hps_io_hps_io_gpio_inst_LOANIO49),                   //                         .hps_io_gpio_inst_LOANIO49
        .hps_io_hps_io_gpio_inst_LOANIO50                   (hps_io_hps_io_gpio_inst_LOANIO50),                   //                         .hps_io_gpio_inst_LOANIO50
        
		  .memory_mem_a                                       (HPS_DDR3_ADDR),                                       //                   memory.mem_a
        .memory_mem_ba                                      (HPS_DDR3_BA),                                      //                         .mem_ba
        .memory_mem_ck                                      (HPS_DDR3_CK_P),                                      //                         .mem_ck
        .memory_mem_ck_n                                    (HPS_DDR3_CK_N),                                    //                         .mem_ck_n
        .memory_mem_cke                                     (HPS_DDR3_CKE),                                     //                         .mem_cke
        .memory_mem_cs_n                                    (HPS_DDR3_CS_N),                                    //                         .mem_cs_n
        .memory_mem_ras_n                                   (HPS_DDR3_RAS_N),                                   //                         .mem_ras_n
        .memory_mem_cas_n                                   (HPS_DDR3_CAS_N),                                   //                         .mem_cas_n
        .memory_mem_we_n                                    (HPS_DDR3_WE_N),                                    //                         .mem_we_n
        .memory_mem_reset_n                                 (HPS_DDR3_RESET_N),                                 //                         .mem_reset_n
        .memory_mem_dq                                      (HPS_DDR3_DQ),                                      //                         .mem_dq
        .memory_mem_dqs                                     (HPS_DDR3_DQS_P),                                     //                         .mem_dqs
        .memory_mem_dqs_n                                   (HPS_DDR3_DQS_N),                                   //                         .mem_dqs_n
        .memory_mem_odt                                     (HPS_DDR3_ODT),                                     //                         .mem_odt
        .memory_mem_dm                                      (HPS_DDR3_DM),                                      //                         .mem_dm
        .memory_oct_rzqin                                   (HPS_DDR3_RZQ)                                    //                         .oct_rzqin
    );
	 
	 
	 
	 
	 /////////////////////////////////////////////////////////
wire [31:0] dat_i;
wire [31:0] dat_o;
wire [30:1] adr;
wire        we;
wire        tga;
wire [ 1:0] sel;
wire        stb;
wire        cyc;
wire        ack;
	 
	 // bridge to sdram
	 wb_to_avalon_bridge #(
		.DW(32),					// Data width
		.AW(30),					// Address width
		.BURST_SUPPORT(0)
	) wb2avl (
		
		// Wishbone Slave Input
		.wb_adr_i               (adr),
		.wb_dat_i               (dat_i),
		.wb_sel_i               (sel),
		.wb_we_i                (we),
		.wb_cyc_i               (cyc),
		.wb_stb_i               (stb),
		.wb_cti_i               (3'b000),	//CLASSIC
		.wb_bte_i               (2'b00),		//LINEAR_BURST
		.wb_dat_o               (dat_o),
		.wb_ack_o               (ack),
		//.wb_err_o               (),
		//.wb_rty_o               (),
	
		// Avalon Master Output
		.m_av_address_o			(sdram0_data_address),
		.m_av_byteenable_o		(sdram0_data_byteenable),
		.m_av_read_o				(sdram0_data_read),
		.m_av_readdata_i			(sdram0_data_readdata),
		.m_av_burstcount_o		(sdram0_data_burstcount),
		.m_av_write_o				(sdram0_data_write),
		.m_av_writedata_o			(sdram0_data_writedata),
		.m_av_waitrequest_i		(sdram0_data_waitrequest),
		.m_av_readdatavalid_i	(sdram0_data_readdatavalid)
	);


	wire clk;
	wire rst;
	
	wire [15:0] bios_dat_i;
	wire [15:0] bios_dat_o;
	wire [19:1] bios_adr;
	wire        bios_we;
	wire        bios_tga;
	wire [ 1:0] bios_sel;
	wire        bios_stb;
	wire        bios_cyc;
	wire        bios_ack;
	 
	 
	bios_rom bios(
		.wb_clk_i(clk),
		.wb_rst_i(rst),
		
		.wb_dat_i(bios_dat_i),
		.wb_dat_o(bios_dat_o),
		.wb_we_i (bios_we),
		.wb_adr_i(bios_adr[1]),
		.wb_sel_i(bios_sel),
		.wb_stb_i(bios_stb),
		.wb_cyc_i(bios_cyc),
		.wb_ack_o(bios_ack)
	);
	
	
	 
endmodule