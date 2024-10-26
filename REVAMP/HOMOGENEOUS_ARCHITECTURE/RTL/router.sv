`ifdef HETERO_NETWORK

module router(
    clk,
    reset,
    i__sram_xbar_sel,
    i__flit_in,
    i__alu_out,
    i__treg,
    o__flit_out,
    o__flit_out_local,
    regbypass,
    regWEN
);

import SMARTPkg::*;

parameter DATA_WIDTH			= 16;
parameter NUM_VCS                       = 4;

localparam NUM_INPUT_PORTS              = 6;
localparam NUM_OUTPUT_PORTS             = 7;
localparam LOG_NUM_VCS                  = $clog2(NUM_VCS);


input  logic                            clk;
input  logic                            reset;

input  logic [NUM_INPUT_PORTS-1:0]	i__sram_xbar_sel	        [NUM_OUTPUT_PORTS-1:0];
input  FlitFixed         i__flit_in                      [NUM_INPUT_PORTS-1-2:0];
input  FlitFixed1         i__alu_out;
input  FlitFixed1         i__treg;
output FlitFixed         o__flit_out                     [NUM_OUTPUT_PORTS-1-3:0];
output FlitFixed1         o__flit_out_local                     [2:0];

input  [3:0] 				regbypass;
input  [3:0] 				regWEN;

FlitFixed                w__flit_xbar_flit_out                   [NUM_OUTPUT_PORTS-1-3:0];
FlitFixed                w__flit_xbar_flit_out_local                   [2:0];

    logic [NUM_INPUT_PORTS-1:0]         this_flit_xbar_sel              [NUM_OUTPUT_PORTS-1:0];
    //FlitFixed                           this_flit_out_local_lsb             [NUM_INPUT_PORTS-1-2:0];
    FlitFixed                           this_flit_out_local             [NUM_INPUT_PORTS-1-2:0];
    FlitFixed                           this_flit_in                    [NUM_INPUT_PORTS-1-2:0];
    FlitFixed                           this_flit_in_lsb                    [NUM_INPUT_PORTS-1:0];
    FlitFixed                           this_flit_in_msb                    [NUM_INPUT_PORTS-1:0];
    FlitFixed                           this_flit_xbar_flit_out_lsb         [NUM_OUTPUT_PORTS-1:0];
    FlitFixed                           this_flit_xbar_flit_out_msb         [NUM_OUTPUT_PORTS-2:0];



    always_comb
    begin
        for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
        begin
	    this_flit_in[j] 		= i__flit_in[j];
        end
	    this_flit_in_lsb[ALU_T] 	= {i__alu_out[16],i__alu_out[7:0]};
	    this_flit_in_msb[ALU_T]     = {i__alu_out[16],i__alu_out[15:8]};
	    this_flit_in_lsb[TREG]          = {i__treg[16],i__treg[7:0]};
	    this_flit_in_msb[TREG]          = {i__treg[16],i__treg[15:8]};
	

        for(int j = 0; j < NUM_OUTPUT_PORTS; j++)
        begin
            for(int k = 0; k < NUM_INPUT_PORTS; k++)
            begin
                this_flit_xbar_sel[j][k] = i__sram_xbar_sel[j][k];
            end
        end
    end

always @(posedge clk)
if (reset)
begin
   for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
   begin
	this_flit_out_local[j] <= {DATA_WIDTH+1{1'b0}};
   end
end
else 
begin
   for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
   begin
	    if (regWEN[j] == 1'b1)  
	    begin
		this_flit_out_local[j] <= i__flit_in[j];
	    end
   end
end
    always_comb
    begin
        for(int j = 0; j < NUM_OUTPUT_PORTS-3; j++)
        begin
            w__flit_xbar_flit_out[j] = this_flit_xbar_flit_out_lsb[j];
        end
	w__flit_xbar_flit_out_local[0] = {(this_flit_xbar_flit_out_lsb[4][16] & this_flit_xbar_flit_out_msb[4][16]),this_flit_xbar_flit_out_msb[4][7:0],this_flit_xbar_flit_out_lsb[4][7:0]};
	w__flit_xbar_flit_out_local[1] = {(this_flit_xbar_flit_out_lsb[5][16] & this_flit_xbar_flit_out_msb[5][16]),this_flit_xbar_flit_out_msb[5][7:0],this_flit_xbar_flit_out_lsb[5][7:0]};
	w__flit_xbar_flit_out_local[2] = {this_flit_xbar_flit_out_lsb[5][16] ,{8{1'b0}},this_flit_xbar_flit_out_lsb[6][7:0]};	
    end

    xbar_bypass_lsb  
        #(
            .DATA_WIDTH                 ($bits(FlitFixed))
        )
        xbar_bypass_lsb(
            .i__sel                     (this_flit_xbar_sel),
            .i__data_in_local           (this_flit_out_local),
            .i__data_in_remote          (this_flit_in_lsb),
	    .regbypass			(regbypass),
            .o__data_out                (this_flit_xbar_flit_out_lsb)
        );
   xbar_bypass_msb
        #(
            .DATA_WIDTH                 ($bits(FlitFixed))
        )
        xbar_bypass_msb(
            .i__sel                     (this_flit_xbar_sel[5:0]),
            .i__data_in_local           (this_flit_out_local),
            .i__data_in_remote          (this_flit_in_msb),
            .regbypass                  (regbypass),
            .o__data_out                (this_flit_xbar_flit_out_msb)
        );


always_comb
begin
    for(int i = 0; i < (NUM_OUTPUT_PORTS-3); i++)
    begin
        o__flit_out[i] = w__flit_xbar_flit_out[i];
    end

    for(int i = 0; i < 3; i++)
    begin
        o__flit_out_local[i] = w__flit_xbar_flit_out_local[i];
    end

end

endmodule

`else
module router(
    clk,
    reset,
    i__sram_xbar_sel,
    i__flit_in,
    i__alu_out,
    i__treg,
    o__flit_out,
    regbypass,
    regWEN
);

import SMARTPkg::*;

parameter DATA_WIDTH			= 32;
parameter NUM_VCS                       = 4;

localparam NUM_INPUT_PORTS              = 6;
localparam NUM_OUTPUT_PORTS             = 7;
localparam LOG_NUM_VCS                  = $clog2(NUM_VCS);


input  logic                            clk;
input  logic                            reset;

input  logic [NUM_INPUT_PORTS-1:0]	i__sram_xbar_sel	        [NUM_OUTPUT_PORTS-1:0];
input  FlitFixed         i__flit_in                      [NUM_INPUT_PORTS-1-2:0];
input  FlitFixed         i__alu_out;
input  FlitFixed         i__treg;
output FlitFixed         o__flit_out                     [NUM_OUTPUT_PORTS-1:0];

input  [3:0] 				regbypass;
input  [3:0] 				regWEN;

FlitFixed                w__flit_xbar_flit_out                   [NUM_OUTPUT_PORTS-1:0];

    logic [NUM_INPUT_PORTS-1:0]         this_flit_xbar_sel              [NUM_OUTPUT_PORTS-1:0];
    FlitFixed                           this_flit_out_local             [NUM_INPUT_PORTS-1-2:0];
    FlitFixed                           this_flit_in                    [NUM_INPUT_PORTS-1:0];
    FlitFixed                           this_flit_xbar_flit_out         [NUM_OUTPUT_PORTS-1:0];

    always_comb
    begin
        for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
        begin
	    this_flit_in[j] 		= i__flit_in[j];
        end
	    this_flit_in[ALU_T] 	= i__alu_out;
	    this_flit_in[TREG]  	= i__treg;

        for(int j = 0; j < NUM_OUTPUT_PORTS; j++)
        begin
            for(int k = 0; k < NUM_INPUT_PORTS; k++)
            begin
                this_flit_xbar_sel[j][k] = i__sram_xbar_sel[j][k];
            end
        end
    end

always @(posedge clk)
if (reset)
begin
   for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
   begin
	this_flit_out_local[j] <= {DATA_WIDTH+1{1'b0}};
   end
end
else 
begin
   for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
   begin
	    if (regWEN[j] == 1'b1)  
	    begin
		this_flit_out_local[j] <= i__flit_in[j];
	    end
   end
end
    always_comb
    begin
        for(int j = 0; j < NUM_OUTPUT_PORTS; j++)
        begin
            w__flit_xbar_flit_out[j] = this_flit_xbar_flit_out[j];
        end
    end

    xbar_bypass  
        #(
            .DATA_WIDTH                 ($bits(FlitFixed))
        )
        xbar_bypass(
            .i__sel                     (this_flit_xbar_sel),
            .i__data_in_local           (this_flit_out_local),
            .i__data_in_remote          (this_flit_in),
	    .regbypass			(regbypass),
            .o__data_out                (this_flit_xbar_flit_out)
        );


always_comb
begin
    for(int i = 0; i < (NUM_OUTPUT_PORTS); i++)
    begin
        o__flit_out[i] = w__flit_xbar_flit_out[i];
    end
end

endmodule

`endif
