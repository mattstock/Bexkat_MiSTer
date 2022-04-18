//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
 //Master input clock
 input 	       CLK_50M,
 
 //Async reset from top-level module.
 //Can be used as initial reset.
 input 	       RESET,
 
 //Must be passed to hps_io module
 inout [48:0]  HPS_BUS,
 
 //Base video clock. Usually equals to CLK_SYS.
 output        CLK_VIDEO,
 
 //Multiple resolutions are supported using different CE_PIXEL rates.
 //Must be based on CLK_VIDEO
 output        CE_PIXEL,
 
 //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
 //if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
 output [12:0] VIDEO_ARX,
 output [12:0] VIDEO_ARY,
 
 output [7:0]  VGA_R,
 output [7:0]  VGA_G,
 output [7:0]  VGA_B,
 output        VGA_HS,
 output        VGA_VS,
 output        VGA_DE, // = ~(VBlank | HBlank)
 output        VGA_F1,
 output [1:0]  VGA_SL,
 output        VGA_SCALER, // Force VGA scaler

 input [11:0]  HDMI_WIDTH,
 input [11:0]  HDMI_HEIGHT,
 output        HDMI_FREEZE,
 
`ifdef MISTER_FB
 // Use framebuffer in DDRAM (USE_FB=1 in qsf)
 // FB_FORMAT:
 //    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
 //    [3]   : 0=16bits 565 1=16bits 1555
 //    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
 //
 // FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
 output        FB_EN,
 output [4:0]  FB_FORMAT,
 output [11:0] FB_WIDTH,
 output [11:0] FB_HEIGHT,
 output [31:0] FB_BASE,
 output [13:0] FB_STRIDE,
 input 	       FB_VBL,
 input 	       FB_LL,
 output        FB_FORCE_BLANK,
 
 `ifdef MISTER_FB_PALETTE
 // Palette control for 8bit modes.
 // Ignored for other video modes.
 output        FB_PAL_CLK,
 output [7:0]  FB_PAL_ADDR,
 output [23:0] FB_PAL_DOUT,
 input [23:0]  FB_PAL_DIN,
 output        FB_PAL_WR,
 `endif
`endif
 
 output        LED_USER, // 1 - ON, 0 - OFF.
 
 // b[1]: 0 - LED status is system status OR'd with b[0]
 //       1 - LED status is controled solely by b[0]
 // hint: supply 2'b00 to let the system control the LED.
 output [1:0]  LED_POWER,
 output [1:0]  LED_DISK,
 
 // I/O board button press simulation (active high)
 // b[1]: user button
 // b[0]: osd button
 output [1:0]  BUTTONS,
 
 input 	       CLK_AUDIO, // 24.576 MHz
 output [15:0] AUDIO_L,
 output [15:0] AUDIO_R,
 output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
 output [1:0]  AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)
 
 //ADC
 inout [3:0]   ADC_BUS,
 
 //SD-SPI
 output        SD_SCK,
 output        SD_MOSI,
 input 	       SD_MISO,
 output        SD_CS,
 input 	       SD_CD,

 //High latency DDR3 RAM interface
 //Use for non-critical time purposes
 output        DDRAM_CLK,
 input 	       DDRAM_BUSY,
 output [7:0]  DDRAM_BURSTCNT,
 output [28:0] DDRAM_ADDR,
 input [63:0]  DDRAM_DOUT,
 input 	       DDRAM_DOUT_READY,
 output        DDRAM_RD,
 output [63:0] DDRAM_DIN,
 output [7:0]  DDRAM_BE,
 output        DDRAM_WE,

 //SDRAM interface with lower latency
 output        SDRAM_CLK,
 output        SDRAM_CKE,
 output [12:0] SDRAM_A,
 output [1:0]  SDRAM_BA,
 inout [15:0]  SDRAM_DQ,
 output        SDRAM_DQML,
 output        SDRAM_DQMH,
 output        SDRAM_nCS,
 output        SDRAM_nCAS,
 output        SDRAM_nRAS,
 output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
 //Secondary SDRAM
 //Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
 input 	       SDRAM2_EN,
 output        SDRAM2_CLK,
 output [12:0] SDRAM2_A,
 output [1:0]  SDRAM2_BA,
 inout [15:0]  SDRAM2_DQ,
 output        SDRAM2_nCS,
 output        SDRAM2_nCAS,
 output        SDRAM2_nRAS,
 output        SDRAM2_nWE,
`endif
 
 input 	       UART_CTS,
 output        UART_RTS,
 input 	       UART_RXD,
 output        UART_TXD,
 output        UART_DTR,
 input 	       UART_DSR,
 
 // Open-drain User port.
 // 0 - D+/RX
 // 1 - D-/TX
 // 2..6 - USR2..USR6
 // Set USER_OUT to 1 to read from USER_IN.
 input [6:0]   USER_IN,
 output [6:0]  USER_OUT,
 
 input 	       OSD_STATUS);
  
  ///////// Default values for ports not used in this core /////////
  
  assign ADC_BUS  = 'Z;
  assign USER_OUT = '1;
  assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
  assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
  assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;

  assign CE_PIXEL = 0;
  
  assign VGA_SL = 0;
  assign VGA_F1 = 0;
  assign VGA_SCALER = 0;
  assign HDMI_FREEZE = 0;
  
  assign AUDIO_S = 0;
  assign AUDIO_L = 0;
  assign AUDIO_R = 0;
  assign AUDIO_MIX = 0;
  
  assign LED_DISK = 0;
  assign LED_POWER = 0;
  assign BUTTONS = 0;
  
  //////////////////////////////////////////////////////////////////
    
  wire [1:0]   ar = status[9:8];
  
  assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
  assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;
  
`include "build_id.v"
  localparam CONF_STR = {
			 "Bexkat;UART115200;",
			 "-;",
			 "O89,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
			 "O2,TV Mode,NTSC,PAL;",
			 "O34,Noise,White,Red,Green,Blue;",
			 "-;",
			 "-;",
			 "T0,Reset;",
			 "R0,Reset and close OSD;",
			 "V,v",`BUILD_DATE
			 };
  
  wire 	       forced_scandoubler;
  wire [1:0]   buttons;
  wire [31:0]  status;
  wire [10:0]  ps2_key;
  wire [64:0]  RTC;
  wire [32:0]  START_TIME;
	      
  hps_io #(.CONF_STR(CONF_STR)) hps_io(.clk_sys(clk_sys),
				       .HPS_BUS(HPS_BUS),
				       .RTC(RTC),
				       .TIMESTAMP(START_TIME),
				       .EXT_BUS(),
				       .gamma_bus(),
				       
				       .forced_scandoubler(forced_scandoubler),
				       
				       .buttons(buttons),
				       .status(status),
				       .status_menumask({status[5]}),
				       
				       .ps2_key(ps2_key));
  
  ///////////////////////   CLOCKS   ///////////////////////////////

  parameter clkfreq = 20000000;
  
  logic 	       clk_sys, clk_vga;
  
  pll pll(.refclk(CLK_50M),
	  .rst(0),
	  .outclk_0(clk_vga),  // 40MHz
	  .outclk_1(clk_sys)); // 20MHz
  
  wire 	       reset = RESET | status[0] | buttons[1];
  
  ////////////////////////// BUSSES ///////////////////////////////////

  if_wb cpu_ibus(), cpu_dbus();
  if_wb ram0_ibus(), ram0_dbus();
  if_wb ram1_ibus(), ram1_dbus();
  if_wb io_dbus(), io_uart(), io_timer();
  if_wb vga_dbus(), vga_fb0(), vga_fb1();  

  mmu mmu_bus0(.clk_i(clk_sys),
	       .rst_i(reset),
	       .mbus(cpu_ibus.slave),
	       .p5(ram0_ibus.master),
	       .p7(ram1_ibus.master));
  
  mmu mmu_bus1(.clk_i(clk_sys),
	       .rst_i(reset),
	       .mbus(cpu_dbus.slave),
	       .p5(ram0_dbus.master),
	       .p3(io_dbus.master),
	       .p7(ram1_dbus.master),
	       .p8(vga_dbus.master),
	       .pc(vga_fb0.master));

  mmu #(.BASE(12)) mmu_bus2(.clk_i(clk_sys),
			    .rst_i(reset),
			    .mbus(io_dbus.slave),
			    .p2(io_uart.master));

  ////////////////////////////////////////////////////////////////////

  logic        cpu_halt;
  logic        cpu_inter_en;
  
  bexkat2 cpu0(.clk_i(clk_sys),
	       .rst_i(reset),
	       .ins_bus(cpu_ibus.master),
	       .dat_bus(cpu_dbus.master),
	       .halt(cpu_halt),
	       .int_en(cpu_inter_en),
	       .inter(4'b0));

  // 128kB
  dualram #(.AWIDTH(15),
	    .INIT_FILE("rom.mif")) ram1(.clk_i(clk_sys),
					.rst_i(reset),
					.wren(1'b0),
					.bus0(ram1_ibus.slave),
					.bus1(ram1_dbus.slave));
  
  // 16kB
  dualram #(.AWIDTH(12)) ram0(.clk_i(clk_sys),
			      .rst_i(reset),
			      .wren(1'b1),
			      .bus0(ram0_ibus.slave),
			      .bus1(ram0_dbus.slave));
  
  ///////////////////////// PERIPHERALS //////////////////////////////////
  
  assign UART_DTR = 1'b1;
  logic [1:0]  serial0_interrupts;
  
  uart #(.CLKFREQ(clkfreq),
	 .BAUD(115200)) uart0(.clk_i(clk_sys),
			      .rst_i(reset),
			      .bus(io_uart.slave),
			      .tx(UART_TXD),
			      .rx(UART_RXD),
			      .cts(UART_CTS),
			      .rts(UART_RTS),
			      .interrupt(serial0_interrupts));

  assign LED_USER = cpu_ibus.stb;

  /////////////////////////// VIDEO  //////////////////////////////////////

  dualram
    #(.AWIDTH(15)) vgamem0(.clk_i(clk_sys),
			   .rst_i(reset),
			   .wren(1'b0),
			   .bus0(vga_fb0.slave),
			   .bus1(vga_fb1.slave));

  assign CLK_VIDEO = clk_vga;
  
  bexkat_vga vga0(.clk_i(clk_sys),
		  .clk_vga_i(clk_vga),
		  .rst_i(reset),
		  .blank_n(VGA_DE),
		  .r(VGA_R),
		  .g(VGA_G),
		  .b(VGA_B),
		  .vs(VGA_VS),
		  .hs(VGA_HS),
		  .inbus(vga_dbus.slave),
		  .outbus(vga_fb1.master));
  
endmodule
