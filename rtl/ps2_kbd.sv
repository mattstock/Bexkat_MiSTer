`include "../../../fpgalib/wb.vh"

module ps2_kbd
  (input        clk_i,
   input 	rst_i,
   if_wb.slave  bus,
   input [10:0] ps2_key);

  typedef enum bit[3:0] { S_IDLE,
			  S_BUSY,
			  S_READ,
			  S_DONE
			  } state_t;

  logic [31:0] dat_o;

`ifdef NO_MODPORT_EXPRESSIONS
//  assign dat_i = bus.dat_m;
  assign bus.dat_s = dat_o;
`else
//  assign dat_i = bus.dat_i;
  assign bus.dat_o = dat_o;
`endif
  
  state_t      bstate, bstate_next;
  logic [31:0] result, result_next;
  logic        fifo_pop, fifo_push, fifo_empty, fifo_full;
  logic [9:0]  fifo_out;
  logic [3:0]  count, count_next;
  logic [9:0]  fifo_in, fifo_in_next;
  logic [7:0]  event_data;
  logic        event_ready, ps2_clock_falling, ps2_clock_rising;
  logic [1:0]  keychange;
  
  assign dat_o = result;
  assign bus.stall = 1'h0;
  assign bus.ack = (bstate == S_DONE);

  assign fifo_pop = (bstate == S_READ);

  always_ff @(posedge clk_i)
    begin
      if (rst_i)
	begin
	  bstate <= S_IDLE;
	  result <= 32'h0;
	end
      else
	begin
	  bstate <= bstate_next;
	  result <= result_next;
	end
    end

  always_comb
    begin
      bstate_next = bstate;
      result_next = result;

      case (bstate)
	S_IDLE:
	  begin
	    if (bus.cyc && bus.stb)
	      bstate_next = S_BUSY;
	  end
	S_BUSY:
	  begin
	    case (bus.adr[2])
	      1'h0:
		begin
		  if (fifo_empty || bus.we)
		    begin
		      result_next = 32'h0;
		      bstate_next = S_DONE;
		    end
		  else
		    begin
		      bstate_next = S_READ;
		    end
		end
	      1'h1:
		begin
		  result_next = { 30'h0, fifo_full, fifo_empty };
		  bstate_next = S_DONE;
		end
	    endcase // case (adr_i[2])
	  end // case: S_BUSY
	S_READ:
	  begin
	    result_next = { 22'h0, fifo_out };
	    bstate_next = S_DONE;
	  end
	S_DONE:
	  begin
	    bstate_next = S_IDLE;
	  end
	endcase
    end

  always_ff @(posedge clk_i)
    begin
      if (rst_i)
	begin
	  keychange <= 2'h0;
	end
      else
	begin
	  keychange <= { keychange[0], ps2_key[10] };
	end
      
    end

  assign fifo_push = (keychange[1] != keychange[0]);
  
  fifo #(.DWIDTH(10)) ps2_fifo0(.clk_i(clk_i),
				.rst_i(rst_i),
				.full(fifo_full),
				.empty(fifo_empty),
				.out(fifo_out),
				.pop(fifo_pop),
				.push(fifo_push),
				.in(ps2_key[9:0]));
  
endmodule
