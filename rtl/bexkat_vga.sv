`include "../../fpgalib/wb.vh"

module bexkat_vga
  #(BPP = 8)
  (
   input 	    clk_i,
   input 	    rst_i,
   input 	    clk_vga_i,
   if_wb.slave      inbus,
   if_wb.master     outbus,
   output 	    vs,
   output 	    hs,
   output [BPP-1:0] r,
   output [BPP-1:0] g,
   output [BPP-1:0] b,
   output 	    blank_n);

  // Configuration registers
  // 0xc00 - video memory base address
  // 0xc01 - video mode, palette select

  logic [31:0] 	    inbus_dat_o, inbus_dat_i;
 
`ifdef NO_MODPORT_EXPRESSIONS
  assign inbus_dat_i = inbus.dat_m;
  assign inbus.dat_s = inbus_dat_o;
`else
  assign inbus_dat_i = inbus.dat_i;
  assign inbus.dat_o = inbus_dat_o;
`endif
  
  logic 	    sstate, sstate_next;
  logic [31:0] 	    setupreg, setupreg_next,
		    cursorpos, cursorpos_next;
  logic [31:0] 	    inbus_dat_o_next;
  logic [23:0] 	    cursorcolor, cursorcolor_next;
  
  assign inbus.ack = sstate;
  assign inbus.stall = 1'h0;

  // Slave state machine
  always_ff @(posedge clk_i or posedge rst_i)
    begin
      if (rst_i)
	begin
	  setupreg <= 32'h05;
	  inbus_dat_o <= 32'h0;
	  cursorpos <= 32'h0;
	  cursorcolor <= 24'ha0a0a0;
	  sstate <= 0;
	end
      else
	begin
	  setupreg <= setupreg_next;
	  inbus_dat_o <= inbus_dat_o_next;
	  cursorpos <= cursorpos_next;
	  cursorcolor <= cursorcolor_next;
	  sstate <= sstate_next;
	end
    end
  
  always_comb
    begin
      sstate_next = sstate;
      setupreg_next = setupreg;
      cursorpos_next = cursorpos;
      cursorcolor_next = cursorcolor;
      inbus_dat_o_next = inbus_dat_o;
      
      case (sstate)
	0:
	  if (inbus.cyc & inbus.stb)
	    begin
	      case (inbus.adr[9:2])
		8'h1:
		  begin // c01 - graphics mode / cursor mode
		    if (inbus.we)
		      begin
			if (inbus.sel[3])
			  setupreg_next[31:24] = inbus_dat_i[31:24];
			if (inbus.sel[2])
			  setupreg_next[23:16] = inbus_dat_i[23:16];
			if (inbus.sel[1])
			  setupreg_next[15:8] = inbus_dat_i[15:8];
			if (inbus.sel[0])
			  setupreg_next[7:0] = inbus_dat_i[7:0];
		      end
		    else
		      inbus_dat_o_next = setupreg;
		  end
		8'h2:
		  begin // c02 - cursor position
		    if (inbus.we)
		      begin
			if (inbus.sel[3])
			  cursorpos_next[31:24] = inbus_dat_i[31:24];
			if (inbus.sel[2])
			  cursorpos_next[23:16] = inbus_dat_i[23:16];
			if (inbus.sel[1])
			  cursorpos_next[15:8] = inbus_dat_i[15:8];
			if (inbus.sel[0])
			  cursorpos_next[7:0] = inbus_dat_i[7:0];
		      end
		    else
		      inbus_dat_o_next = cursorpos;
		  end // case: 8'h2
		8'h3:
		  begin // c03 - cursor color
		    if (inbus.we)
		      begin
			if (inbus.sel[2])
			  cursorcolor_next[23:16] = inbus_dat_i[23:16];
			if (inbus.sel[1])
			  cursorcolor_next[15:8] = inbus_dat_i[15:8];
			if (inbus.sel[0])
			  cursorcolor_next[7:0] = inbus_dat_i[7:0];
		      end
		    else
		      inbus_dat_o_next = cursorcolor;
		  end
		default:
		  begin
		    if (~inbus.we)
		      inbus_dat_o_next = 32'h0;
		  end
	      endcase
	      sstate_next = 1;
	    end
	1: sstate_next = 0;
      endcase
    end

  textdrv #(.BPP(BPP)) textdriver0(.clk_i(clk_i),
				   .rst_i(rst_i),
				   .blank_n(blank_n),
				   .red(r),
				   .green(g),
				   .blue(b),
				   .clk_vga_i(clk_vga_i),
				   .vs(vs),
				   .hs(hs),
				   .cursorpos(cursorpos),
				   .cursormode(setupreg[7:4]),
				   .cursorcolor(cursorcolor),
				   .bus(outbus.master));

endmodule
