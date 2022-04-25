`include "../../../fpgalib/wb.vh"

module io_misc
  (input        clk_i,
   input 	rst_i,
   if_wb.slave  bus,
   output reg 	led,
   input [32:0] boot_time);

  typedef enum bit[3:0] { S_IDLE,
			  S_BUSY,
			  S_READ,
			  S_DONE
			  } state_t;

  logic [31:0] dat_o, dat_i;

`ifdef NO_MODPORT_EXPRESSIONS
  assign dat_i = bus.dat_m;
  assign bus.dat_s = dat_o;
`else
  assign dat_i = bus.dat_i;
  assign bus.dat_o = dat_o;
`endif
  
  state_t      bstate, bstate_next;
  logic [31:0] result, result_next;

  logic        led_next;
  
  assign dat_o = result;
  assign bus.stall = 1'h0;
  assign bus.ack = (bstate == S_DONE);

  always_ff @(posedge clk_i)
    begin
      if (rst_i)
	begin
	  bstate <= S_IDLE;
	  result <= 32'h0;
	  led <= 1'h0;
	end
      else
	begin
	  bstate <= bstate_next;
	  result <= result_next;
	  led <= led_next;
	end
    end

  always_comb
    begin
      bstate_next = bstate;
      result_next = result;
      led_next = led;
      
      case (bstate)
	S_IDLE:
	  begin
	    if (bus.cyc && bus.stb)
	      bstate_next = S_BUSY;
	  end
	S_BUSY:
	  begin
	    case (bus.adr[3:2])
	      2'b00:
		begin
		  if (bus.we)
		    begin
		      result_next = 32'h0;
		      led_next = dat_i[0];
		    end
		  else
		    begin
		      result_next = { 31'h0, led };
		    end
		end // case: 2'h00
	      2'b01:
		begin
		  result_next = 32'h0;
		end
	      2'b10:
		begin
		  if (bus.we)
		    begin
		      result_next = 32'h0;
		    end
		  else
		    begin
		      result_next = { 31'h0, boot_time[32] };
		    end
		end // case: 2'h10
	      2'b11:
		begin
		  if (bus.we)
		    begin
		      result_next = 32'h0;
		    end
		  else
		    begin
		      result_next = { 31'h0, boot_time[32] };
		    end
		end // case: 2'h11
	    endcase // case (bus.adr[3:2])
	    bstate_next = S_DONE;
	  end // case: S_BUSY
	S_DONE:
	  begin
	    bstate_next = S_IDLE;
	  end
      endcase // case (bstate)
    end

endmodule
