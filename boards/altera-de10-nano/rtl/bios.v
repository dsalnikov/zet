module bios_rom (
     // Wishbone slave interface
    input           wb_clk_i,
    input           wb_rst_i,
    input  [15:0]   wb_dat_i,
    output [15:0]   wb_dat_o,
    input           wb_we_i,
    input           wb_adr_i,
    input  [ 1:0]   wb_sel_i,
    input           wb_stb_i,
    input           wb_cyc_i,
    output          wb_ack_o
  );

  //BIOS ROM
  reg  [15:0] rom[0:65535];  

  // Registers and nets
  reg  [21:0] address;

  // Combinatorial logic
  assign wb_ack_o = wb_stb_i & wb_cyc_i;
  assign wb_dat_o = rom[address];

  // --------------------------------------------------------------------
  // Register addresses and defaults
  // --------------------------------------------------------------------
  `define FLASH_ALO   1'h0    // Lower 16 bits of address lines
  `define FLASH_AHI   1'h1    // Upper  6 bits of address lines
  always @(posedge wb_clk_i)  // Synchrounous
    if(wb_rst_i)
      address <= 22'h000000;  // Interupt Enable default
    else
      if(wb_stb_i & wb_cyc_i & wb_we_i)          // If a write was requested
        case(wb_adr_i)        // Determine which register was writen to
            `FLASH_ALO: address[15: 0] <= wb_dat_i;
            `FLASH_AHI: address[21:16] <= wb_dat_i[5:0];
            default:    ;     // Default
        endcase               // End of case

        
        
        
  // Instantiate the ROM
  initial $readmemh("bios/bios_rom.hex", rom);

endmodule
