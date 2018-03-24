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
	inout hps_io_hps_io_gpio_inst_LOANIO50,
	
	output [19:0] pc
);


wire [5:0] leds_h;


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

//sd card io
assign loan_io_oe[45] = 1'b1;
assign loan_io_oe[36] = 1'b1;
assign loan_io_oe[47] = 1'b1;

//wire uart_tx = loan_io_out[50];
//wire uart_rx = loan_io_in[49];

//assign loan_io_out[50] = uart_rx;



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
	 
	 
	 
	 
	//PLL
	wire clk;
	wire vga_clk;
	wire lock;
	wire        rst_lck;

	pll pll(
		.refclk(FPGA_CLK1_50),   // refclk.clk
		.rst(SW[0]),      		 // reset.reset
		.outclk_0(sdram_clk),    // 100 Mhz
		.outclk_1(vga_clk),      // 25 Mhz
		.outclk_2(clk),          // 12.5 Mhz
		.locked(lock)            // locked.export
	);

	wire reset_n = lock;

	clk_gen #(
		.res   (21),
		.phase (21'd100091)
	) timerclk (
		.clk_i (vga_clk),    // 25 MHz
		.rst_i (rst),
		.clk_o (timer_clk)   // 1.193178 MHz (required 1.193182 MHz)
	);
	
	
`ifndef SIMULATION
  /*
   * Debounce it (counter holds reset for 10.49ms),
   * and generate power-on reset.
   */
  initial rst_debounce <= 17'h1FFFF;
  reg rst;
  initial rst <= 1'b1;
  always @(posedge clk) begin
    if(~rst_lck) /* reset is active low */
      rst_debounce <= 17'h1FFFF;
    else if(rst_debounce != 17'd0)
      rst_debounce <= rst_debounce - 17'd1;
    rst <= rst_debounce != 17'd0;
  end
`else
  wire rst;
  assign rst = !rst_lck;
`endif
	
	 //HDMI I2C
	I2C_HDMI_Config u_I2C_HDMI_Config (
		.iCLK(FPGA_CLK1_50),
		.iRST_N(reset_n),
		.I2C_SCLK(HDMI_I2C_SCL),
		.I2C_SDAT(HDMI_I2C_SDA),
		.HDMI_TX_INT(HDMI_TX_INT)
	 );
	 

	// wires to vga controller
	wire [15:0] vga_dat_o;
	wire [15:0] vga_dat_i;
	wire        vga_tga_i;
	wire [19:1] vga_adr_i;
	wire [ 1:0] vga_sel_i;
	wire        vga_we_i;
	wire        vga_cyc_i;
	wire        vga_stb_i;
	wire        vga_ack_o;
	 
	// VGA signals
	wire [ 3:0] vga_r;
	wire [ 3:0] vga_g;
	wire [ 3:0] vga_b;
	wire        vga_hsync;
	wire        vga_vsync;
	
	//HDMI_TX_D[23:16] r
	//HDMI_TX_D[15:8] g
	//HDMI_TX_D[7:0] b
	
	assign HDMI_TX_D[7:0] = {3'b000, vga_b};
	assign HDMI_TX_D[15:8] = {3'b000, vga_g};
	assign HDMI_TX_D[23:16] = {3'b000, vga_r};

	assign HDMI_TX_HS = vga_hsync;
	assign HDMI_TX_VS = vga_vsync;
	
	//external sram wires
	wire [17:1] csrm_adr_o;
	wire [ 1:0] csrm_sel_o;
	wire        csrm_we_o;
	wire [15:0] csrm_dat_o;
	wire [15:0] csrm_dat_i;

	vga vga (
		.wb_rst_i (rst),

		// Wishbone slave interface
		.wb_clk_i (vga_clk),   // 25MHz VGA clock
		.wb_dat_i (vga_dat_i),
		.wb_dat_o (vga_dat_o),
		.wb_adr_i (vga_adr_i[16:1]),  // 128K
		.wb_we_i  (vga_we_i),
		.wb_tga_i (vga_tga_i),
		.wb_sel_i (vga_sel_i),
		.wb_stb_i (vga_stb_i),
		.wb_cyc_i (vga_cyc_i),
		.wb_ack_o (vga_ack_o),

		// VGA pad signals
		.vga_red_o   (vga_r),
		.vga_green_o (vga_g),
		.vga_blue_o  (vga_b),
		.horiz_sync  (vga_hsync),
		.vert_sync   (vga_vsync),

		// CSR SRAM master interface
		.csrm_adr_o (csrm_adr_o),
		.csrm_sel_o (csrm_sel_o),
		.csrm_we_o  (csrm_we_o),
		.csrm_dat_o (csrm_dat_o),
		.csrm_dat_i (csrm_dat_i)
	);
 
	 
	 
	 /////////////////////////////////////////////////////////
	wire [31:0] sdram_dat_i;
	wire [31:0] sdram_dat_o;
	wire [30:1] sdram_adr;
	wire        sdram_we;
	wire        sdram_tga;
	wire [ 1:0] sdram_sel;
	wire        sdram_stb;
	wire        sdram_cyc;
	wire        sdram_ack;
		 
		 
  // wires to SDRAM controller
  wire [19:1] fmlbrg_adr_s;
  wire [15:0] fmlbrg_dat_w_s;
  wire [15:0] fmlbrg_dat_r_s;
  wire [ 1:0] fmlbrg_sel_s;
  wire        fmlbrg_cyc_s;
  wire        fmlbrg_stb_s;
  wire        fmlbrg_tga_s;
  wire        fmlbrg_we_s;
  wire        fmlbrg_ack_s;
  
	wb_abrgr wb_fmlbrg (
    .sys_rst (rst),

    // Wishbone slave interface
    .wbs_clk_i (clk),
    .wbs_adr_i (fmlbrg_adr_s),
    .wbs_dat_i (fmlbrg_dat_w_s),
    .wbs_dat_o (fmlbrg_dat_r_s),
    .wbs_sel_i (fmlbrg_sel_s),
    .wbs_tga_i (fmlbrg_tga_s),
    .wbs_stb_i (fmlbrg_stb_s),
    .wbs_cyc_i (fmlbrg_cyc_s),
    .wbs_we_i  (fmlbrg_we_s),
    .wbs_ack_o (fmlbrg_ack_s),

    // Wishbone master interface
    .wbm_clk_i (sdram_clk),
    .wbm_adr_o (sdram_adr[19:1]),
    .wbm_dat_o (sdram_dat_i[15:0]),
    .wbm_dat_i (sdram_dat_o[15:0]),
    .wbm_sel_o (sdram_sel),
    .wbm_tga_o (sdram_tga),
    .wbm_stb_o (sdram_stb),
    .wbm_cyc_o (sdram_cyc),
    .wbm_we_o  (sdram_we),
    .wbm_ack_i (sdram_ack)
  );
  
	 // bridge to sdram
	 wb_to_avalon_bridge #(
		.DW(32),					// Data width
		.AW(30),					// Address width
		.BURST_SUPPORT(0)
	) wb2avl (
		.wb_clk_i					(sdram_clk),
		.wb_rst_i					(rst),
		
		// Wishbone Slave Input
		.wb_adr_i               (sdram_adr),
		.wb_dat_i               (sdram_dat_i),
		.wb_sel_i               (sdram_sel),
		.wb_we_i                (sdram_we),
		.wb_cyc_i               (sdram_cyc),
		.wb_stb_i               (sdram_stb),
		.wb_cti_i               (3'b000),	//CLASSIC
		.wb_bte_i               (2'b00),		//LINEAR_BURST
		.wb_dat_o               (sdram_dat_o),
		.wb_ack_o               (sdram_ack),
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

	
	
	
  //Zet cpu system

	
  //wire [19:0] pc;
  
  wire [15:0] dat_o;
  wire [15:0] dat_i;
  wire [19:1] adr;
  wire        we;
  wire        tga;
  wire [ 1:0] sel;
  wire        stb;
  wire        cyc;
  wire        ack;
	
	  // wires to BIOS ROM
  wire [15:0] rom_dat_o;
  wire [15:0] rom_dat_i;
  wire        rom_tga_i;
  wire [19:1] rom_adr_i;
  wire [ 1:0] rom_sel_i;
  wire        rom_we_i;
  wire        rom_cyc_i;
  wire        rom_stb_i;
  wire        rom_ack_o;

  // wires to flash controller
  wire [15:0] fl_dat_o;
  wire [15:0] fl_dat_i;
  wire        fl_tga_i;
  wire [19:1] fl_adr_i;
  wire [ 1:0] fl_sel_i;
  wire        fl_we_i;
  wire        fl_cyc_i;
  wire        fl_stb_i;
  wire        fl_ack_o;

	  // wires to uart controller
  wire [15:0] uart_dat_o;
  wire [15:0] uart_dat_i;
  wire        uart_tga_i;
  wire [19:1] uart_adr_i;
  wire [ 1:0] uart_sel_i;
  wire        uart_we_i;
  wire        uart_cyc_i;
  wire        uart_stb_i;
  wire        uart_ack_o;

  // wires to keyboard controller
  wire [15:0] keyb_dat_o;
  wire [15:0] keyb_dat_i;
  wire        keyb_tga_i;
  wire [19:1] keyb_adr_i;
  wire [ 1:0] keyb_sel_i;
  wire        keyb_we_i;
  wire        keyb_cyc_i;
  wire        keyb_stb_i;
  wire        keyb_ack_o;

  // wires to timer controller
  wire [15:0] timer_dat_o;
  wire [15:0] timer_dat_i;
  wire        timer_tga_i;
  wire [19:1] timer_adr_i;
  wire [ 1:0] timer_sel_i;
  wire        timer_we_i;
  wire        timer_cyc_i;
  wire        timer_stb_i;
  wire        timer_ack_o;

  // wires to sd controller
  wire [19:1] sd_adr_i;
  wire [ 7:0] sd_dat_o;
  wire [15:0] sd_dat_i;
  wire        sd_tga_i;
  wire [ 1:0] sd_sel_i;
  wire        sd_we_i;
  wire        sd_cyc_i;
  wire        sd_stb_i;
  wire        sd_ack_o;

    // wires to sd bridge
  wire [19:1] sd_adr_i_s;
  wire [15:0] sd_dat_o_s;
  wire [15:0] sd_dat_i_s;
  wire        sd_tga_i_s;
  wire [ 1:0] sd_sel_i_s;
  wire        sd_we_i_s;
  wire        sd_cyc_i_s;
  wire        sd_stb_i_s;
  wire        sd_ack_o_s;

  // wires to gpio controller
  wire [15:0] gpio_dat_o;
  wire [15:0] gpio_dat_i;
  wire        gpio_tga_i;
  wire [19:1] gpio_adr_i;
  wire [ 1:0] gpio_sel_i;
  wire        gpio_we_i;
  wire        gpio_cyc_i;
  wire        gpio_stb_i;
  wire        gpio_ack_o;

    // wires to default stb/ack
  wire        def_cyc_i;
  wire        def_stb_i;
  wire [15:0] sw_dat_o;

  
  wire [ 7:0] intv;
  wire [ 2:0] iid;
  wire        intr;
  wire        inta;

  wire        nmi_pb;
  wire        nmi;
  wire        nmia;

  reg  [16:0] rst_debounce;

  wire        timer_clk;
  wire        timer2_o;

  // Audio only signals
  wire [ 7:0] aud_dat_o;
  wire        aud_cyc_i;
  wire        aud_ack_o;
  wire        aud_sel_cond;

  // Keyboard-audio shared signals
  wire [ 7:0] kaud_dat_o;
  wire        kaud_cyc_i;
  wire        kaud_ack_o;

  wire        sb_cyc_i;
  wire        sb_stb_i;

  wire        csr_cyc_i;
  wire        csr_stb_i;
  
	zet zet (
    .pc (pc),

    // Wishbone master interface
    .wb_clk_i (clk),
    .wb_rst_i (rst),
    .wb_dat_i (dat_i),
    .wb_dat_o (dat_o),
    .wb_adr_o (adr),
    .wb_we_o  (we),
    .wb_tga_o (tga),
    .wb_sel_o (sel),
    .wb_stb_o (stb),
    .wb_cyc_o (cyc),
    .wb_ack_i (ack),
    .wb_tgc_i (intr),
    .wb_tgc_o (inta),
    .nmi      (nmi),
    .nmia     (nmia)
  );

  bootrom bootrom (
    .clk (clk),            // Wishbone slave interface
    .rst (rst),
    .wb_dat_i (rom_dat_i),
    .wb_dat_o (rom_dat_o),
    .wb_adr_i (rom_adr_i),
    .wb_we_i  (rom_we_i ),
    .wb_tga_i (rom_tga_i),
    .wb_stb_i (rom_stb_i),
    .wb_cyc_i (rom_cyc_i),
    .wb_sel_i (rom_sel_i),
    .wb_ack_o (rom_ack_o)
  );
  
  bios_rom bios_rom(
    .wb_clk_i (clk),            // Main Clock
    .wb_rst_i (rst),            // Reset Line
    .wb_adr_i (fl_adr_i[1]),    // Address lines
    .wb_sel_i (fl_sel_i),       // Select lines
    .wb_dat_i (fl_dat_i),       // Command to send
    .wb_dat_o (fl_dat_o),       // Received data
    .wb_cyc_i (fl_cyc_i),       // Cycle
    .wb_stb_i (fl_stb_i),       // Strobe
    .wb_we_i  (fl_we_i),        // Write enable
    .wb_ack_o (fl_ack_o)        // Normal bus termination
  );
	 
   // RS232 COM1 Port
  serial com1 (
    .wb_clk_i (clk),              // Main Clock
    .wb_rst_i (rst),              // Reset Line
    .wb_adr_i (uart_adr_i[2:1]),  // Address lines
    .wb_sel_i (uart_sel_i),       // Select lines
    .wb_dat_i (uart_dat_i),       // Command to send
    .wb_dat_o (uart_dat_o),
    .wb_we_i  (uart_we_i),        // Write enable
    .wb_stb_i (uart_stb_i),
    .wb_cyc_i (uart_cyc_i),
    .wb_ack_o (uart_ack_o),
    .wb_tgc_o (intv[4]),          // Interrupt request

    .rs232_tx (loan_io_out[50]),        // UART signals
    .rs232_rx (loan_io_in[49])         // serial input/output
  );

   ps2 ps2 (
    .wb_clk_i (clk),             // Main Clock
    .wb_rst_i (rst),             // Reset Line
    .wb_adr_i (keyb_adr_i[2:1]), // Address lines
    .wb_sel_i (keyb_sel_i),      // Select lines
    .wb_dat_i (keyb_dat_i),      // Command to send to Ethernet
    .wb_dat_o (keyb_dat_o),
    .wb_we_i  (keyb_we_i),       // Write enable
    .wb_stb_i (keyb_stb_i),
    .wb_cyc_i (keyb_cyc_i),
    .wb_ack_o (keyb_ack_o),
    .wb_tgk_o (intv[1]),         // Keyboard Interrupt request
    .wb_tgm_o (intv[3]),         // Mouse Interrupt request

    .ps2_kbd_clk_ (ps2_kclk_),
    .ps2_kbd_dat_ (ps2_kdat_),
    .ps2_mse_clk_ (ps2_mclk_),
    .ps2_mse_dat_ (ps2_mdat_)
  );

   timer timer (
    .wb_clk_i (clk),
    .wb_rst_i (rst),
    .wb_adr_i (timer_adr_i[1]),
    .wb_sel_i (timer_sel_i),
    .wb_dat_i (timer_dat_i),
    .wb_dat_o (timer_dat_o),
    .wb_stb_i (timer_stb_i),
    .wb_cyc_i (timer_cyc_i),
    .wb_we_i  (timer_we_i),
    .wb_ack_o (timer_ack_o),
    .wb_tgc_o (intv[0]),
    .tclk_i   (timer_clk),     // 1.193182 MHz = (14.31818/12) MHz
    .gate2_i  (aud_dat_o[0]),
    .out2_o   (timer2_o)
  );

  simple_pic pic0 (
    .clk  (clk),
    .rst  (rst),
    .intv (intv),
    .inta (inta),
    .intr (intr),
    .iid  (iid)
  );

    wb_abrgr sd_brg (
    .sys_rst (rst),

    // Wishbone slave interface
    .wbs_clk_i (clk),
    .wbs_adr_i (sd_adr_i_s),
    .wbs_dat_i (sd_dat_i_s),
    .wbs_dat_o (sd_dat_o_s),
    .wbs_sel_i (sd_sel_i_s),
    .wbs_tga_i (sd_tga_i_s),
    .wbs_stb_i (sd_stb_i_s),
    .wbs_cyc_i (sd_cyc_i_s),
    .wbs_we_i  (sd_we_i_s),
    .wbs_ack_o (sd_ack_o_s),

    // Wishbone master interface
    .wbm_clk_i (sdram_clk),
    .wbm_adr_o (sd_adr_i),
    .wbm_dat_o (sd_dat_i),
    .wbm_dat_i ({8'h0,sd_dat_o}),
    .wbm_tga_o (sd_tga_i),
    .wbm_sel_o (sd_sel_i),
    .wbm_stb_o (sd_stb_i),
    .wbm_cyc_o (sd_cyc_i),
    .wbm_we_o  (sd_we_i),
    .wbm_ack_i (sd_ack_o)
  );

   sdspi sdspi (
    // Serial pad signal
    .sclk (loan_io_out[45]),
    .miso (loan_io_in[38]),
    .mosi (loan_io_out[36]),
    .ss   (loan_io_out[47]),

    // Wishbone slave interface
    .wb_clk_i (sdram_clk),
    .wb_rst_i (rst),
    .wb_dat_i (sd_dat_i[8:0]),
    .wb_dat_o (sd_dat_o),
    .wb_we_i  (sd_we_i),
    .wb_sel_i (sd_sel_i),
    .wb_stb_i (sd_stb_i),
    .wb_cyc_i (sd_cyc_i),
    .wb_ack_o (sd_ack_o)
  );

  // Switches and leds
  sw_leds sw_leds (
    .wb_clk_i (clk),
    .wb_rst_i (rst),

    // Wishbone slave interface
    .wb_adr_i (gpio_adr_i[1]),
    .wb_dat_o (gpio_dat_o),
    .wb_dat_i (gpio_dat_i),
    .wb_sel_i (gpio_sel_i),
    .wb_we_i  (gpio_we_i),
    .wb_stb_i (gpio_stb_i),
    .wb_cyc_i (gpio_cyc_i),
    .wb_ack_o (gpio_ack_o),

    // GPIO inputs/outputs
    .leds_  (),
    .sw_    (),
    .pb_    (KEY[0]),
    .tick   (intv[0]),
    .nmi_pb (nmi_pb) // NMI from pushbutton
  );
 
  
  wb_switch #(
    .s0_addr_1 (20'b0_1111_1111_1111_0000_000), // bios boot mem 0xfff00 - 0xfffff
    .s0_mask_1 (20'b1_1111_1111_1111_0000_000), // bios boot ROM Memory

    .s1_addr_1 (20'b0_1010_0000_0000_0000_000), // mem 0xa0000 - 0xbffff
    .s1_mask_1 (20'b1_1110_0000_0000_0000_000), // VGA

    .s1_addr_2 (20'b1_0000_0000_0011_1100_000), // io 0x3c0 - 0x3df
    .s1_mask_2 (20'b1_0000_1111_1111_1110_000), // VGA IO

    .s2_addr_1 (20'b1_0000_0000_0011_1111_100), // io 0x3f8 - 0x3ff
    .s2_mask_1 (20'b1_0000_1111_1111_1111_100), // RS232 IO

    .s3_addr_1 (20'b1_0000_0000_0000_0110_000), // io 0x60, 0x64
    .s3_mask_1 (20'b1_0000_1111_1111_1111_101), // Keyboard / Mouse IO

    .s4_addr_1 (20'b1_0000_0000_0001_0000_000), // io 0x100 - 0x101
    .s4_mask_1 (20'b1_0000_1111_1111_1111_111), // SD Card IO

    .s5_addr_1 (20'b1_0000_1111_0001_0000_000), // io 0xf100 - 0xf103
    .s5_mask_1 (20'b1_0000_1111_1111_1111_110), // GPIO

    .s6_addr_1 (20'b1_0000_1111_0010_0000_000), // io 0xf200 - 0xf20f
    .s6_mask_1 (20'b1_0000_1111_1111_1111_000), // CSR Bridge SDRAM Control

    .s7_addr_1 (20'b1_0000_0000_0000_0100_000), // io 0x40 - 0x43
    .s7_mask_1 (20'b1_0000_1111_1111_1111_110), // Timer control port

    .s8_addr_1 (20'b1_0000_0000_0010_0011_100), // io 0x0238 - 0x023b
    .s8_mask_1 (20'b1_0000_1111_1111_1111_110), // Flash IO port

    .s9_addr_1 (20'b1_0000_0000_0010_0001_000), // io 0x0210 - 0x021F
    .s9_mask_1 (20'b1_0000_1111_1111_1111_000), // Sound Blaster

    .sA_addr_1 (20'b1_0000_1111_0011_0000_000), // io 0xf300 - 0xf3ff
    .sA_mask_1 (20'b1_0000_1111_1111_0000_000), // SDRAM Control
    .sA_addr_2 (20'b0_0000_0000_0000_0000_000), // mem 0x00000 - 0xfffff
    .sA_mask_2 (20'b1_0000_0000_0000_0000_000)  // Base RAM

    ) wbs (

    // Master interface
    .m_dat_i (dat_o),
    .m_dat_o (sw_dat_o),
    .m_adr_i ({tga,adr}),
    .m_sel_i (sel),
    .m_we_i  (we),
    .m_cyc_i (cyc),
    .m_stb_i (stb),
    .m_ack_o (ack),

    // Slave 0 interface - bios rom
    .s0_dat_i (rom_dat_o),
    .s0_dat_o (rom_dat_i),
    .s0_adr_o ({rom_tga_i,rom_adr_i}),
    .s0_sel_o (rom_sel_i),
    .s0_we_o  (rom_we_i),
    .s0_cyc_o (rom_cyc_i),
    .s0_stb_o (rom_stb_i),
    .s0_ack_i (rom_ack_o),

     // Slave 1 interface - vga
    .s1_dat_i (vga_dat_o),
    .s1_dat_o (vga_dat_i),
    .s1_adr_o ({vga_tga_i,vga_adr_i}),
    .s1_sel_o (vga_sel_i),
    .s1_we_o  (vga_we_i),
    .s1_cyc_o (vga_cyc_i),
    .s1_stb_o (vga_stb_i),
    .s1_ack_i (vga_ack_o),

    // Slave 2 interface - uart
    .s2_dat_i (uart_dat_o),
    .s2_dat_o (uart_dat_i),
    .s2_adr_o ({uart_tga_i,uart_adr_i}),
    .s2_sel_o (uart_sel_i),
    .s2_we_o  (uart_we_i),
    .s2_cyc_o (uart_cyc_i),
    .s2_stb_o (uart_stb_i),
    .s2_ack_i (uart_ack_o),

    // Slave 3 interface - keyb
    .s3_dat_i ({kaud_dat_o,keyb_dat_o[7:0]}),
    .s3_dat_o (keyb_dat_i),
    .s3_adr_o ({keyb_tga_i,keyb_adr_i}),
    .s3_sel_o (keyb_sel_i),
    .s3_we_o  (keyb_we_i),
    .s3_cyc_o (kaud_cyc_i),
    .s3_stb_o (keyb_stb_i),
    .s3_ack_i (kaud_ack_o),

    // Slave 4 interface - sd
    .s4_dat_i (sd_dat_o_s),
    .s4_dat_o (sd_dat_i_s),
    .s4_adr_o ({sd_tga_i_s,sd_adr_i_s}),
    .s4_sel_o (sd_sel_i_s),
    .s4_we_o  (sd_we_i_s),
    .s4_cyc_o (sd_cyc_i_s),
    .s4_stb_o (sd_stb_i_s),
    .s4_ack_i (sd_ack_o_s),

    // Slave 5 interface - gpio
    .s5_dat_i (gpio_dat_o),
    .s5_dat_o (gpio_dat_i),
    .s5_adr_o ({gpio_tga_i,gpio_adr_i}),
    .s5_sel_o (gpio_sel_i),
    .s5_we_o  (gpio_we_i),
    .s5_cyc_o (gpio_cyc_i),
    .s5_stb_o (gpio_stb_i),
    .s5_ack_i (gpio_ack_o),

    // Slave 6 interface - csr bridge
    .s6_dat_i (),
    .s6_dat_o (),
    .s6_adr_o (),
    .s6_sel_o (),
    .s6_we_o  (),
    .s6_cyc_o (csr_cyc_i),
    .s6_stb_o (csr_stb_i),
    .s6_ack_i (csr_cyc_i && csr_stb_i),

    // Slave 7 interface - timer
    .s7_dat_i (timer_dat_o),
    .s7_dat_o (timer_dat_i),
    .s7_adr_o ({timer_tga_i,timer_adr_i}),
    .s7_sel_o (timer_sel_i),
    .s7_we_o  (timer_we_i),
    .s7_cyc_o (timer_cyc_i),
    .s7_stb_o (timer_stb_i),
    .s7_ack_i (timer_ack_o),

    // Slave 8 interface - flash
    .s8_dat_i (fl_dat_o),
    .s8_dat_o (fl_dat_i),
    .s8_adr_o ({fl_tga_i,fl_adr_i}),
    .s8_sel_o (fl_sel_i),
    .s8_we_o  (fl_we_i),
    .s8_cyc_o (fl_cyc_i),
    .s8_stb_o (fl_stb_i),
    .s8_ack_i (fl_ack_o),

    // Slave 9 interface - not connected
    .s9_dat_i (),
    .s9_dat_o (),
    .s9_adr_o (),
    .s9_sel_o (),
    .s9_we_o  (),
    .s9_cyc_o (sb_cyc_i),
    .s9_stb_o (sb_stb_i),
    .s9_ack_i (sb_cyc_i && sb_stb_i),

    // Slave A interface - sdram
    .sA_dat_i (fmlbrg_dat_r_s),
    .sA_dat_o (fmlbrg_dat_w_s),
    .sA_adr_o ({fmlbrg_tga_s,fmlbrg_adr_s}),
    .sA_sel_o (fmlbrg_sel_s),
    .sA_we_o  (fmlbrg_we_s),
    .sA_cyc_o (fmlbrg_cyc_s),
    .sA_stb_o (fmlbrg_stb_s),
    .sA_ack_i (fmlbrg_ack_s),
	 
	 
    // Slave B interface - default
    .sB_dat_i (16'h0000),
    .sB_dat_o (),
    .sB_adr_o (),
    .sB_sel_o (),
    .sB_we_o  (),
    .sB_cyc_o (def_cyc_i),
    .sB_stb_o (def_stb_i),
    .sB_ack_i (def_cyc_i & def_stb_i)
  );
	
	
	// Continuous assignments
    assign rst_lck    = !SW[0] & lock;

	assign nmi = nmi_pb;
	assign dat_i = nmia ? 16'h0002 :
				(inta ? { 13'b0000_0000_0000_1, iid } :
				sw_dat_o);

	
	 
endmodule