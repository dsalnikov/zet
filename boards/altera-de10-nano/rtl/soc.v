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
	inout hps_io_hps_io_gpio_inst_LOANIO36,
	inout hps_io_hps_io_gpio_inst_LOANIO38,
	inout hps_io_hps_io_gpio_inst_LOANIO45,
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

	 
endmodule