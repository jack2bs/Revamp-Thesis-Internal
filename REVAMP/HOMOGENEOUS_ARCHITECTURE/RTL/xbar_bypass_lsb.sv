module xbar_bypass_lsb(
    i__sel,
    i__data_in_local,
    i__data_in_remote,
    regbypass,
    o__data_out
);


import TopPkg::MuxType;
import SMARTPkg::*;

parameter  DATA_WIDTH                   = 16;

localparam NUM_INPUT_PORTS              = 6;
localparam NUM_OUTPUT_PORTS             = 7;
localparam MuxType MUX_TYPE             = TopPkg::MUX_ONEHOT;
localparam SEL_WIDTH                    = NUM_INPUT_PORTS;

input  logic [SEL_WIDTH-1:0]            i__sel                  [NUM_OUTPUT_PORTS-1:0];
input  FlitFixed                        i__data_in_local        [NUM_INPUT_PORTS-3:0];
input  FlitFixed                        i__data_in_remote       [NUM_INPUT_PORTS-1:0];
input  [3:0]				regbypass;
output FlitFixed                        o__data_out             [NUM_OUTPUT_PORTS-1:0];


logic [NUM_INPUT_PORTS-1:0]             w__sel_input        [NUM_OUTPUT_PORTS-1:0];
logic [NUM_INPUT_PORTS-1:0]             w__sel_onehot       [NUM_OUTPUT_PORTS-1:0]; 

logic [2:0]                             w__sel__next        [NUM_OUTPUT_PORTS-1:0];
logic [2:0]                             r__sel__pff         [NUM_OUTPUT_PORTS-1:0];
logic [2:0]                             w__sel_local__next;
logic [2:0]                             r__sel_local__pff;

FlitFixed                               w__flit_in          [NUM_OUTPUT_PORTS-1:0][5:0]; 


generate
genvar i;
for(i = 0; i < NUM_OUTPUT_PORTS; i++)
begin: mux
    encoder_onehot
        #(
            .NUM_BITS                   (6)
        )
        gen(
	    .i__onehot                  (w__sel_onehot[i]),
            .o__valid                   (),
            .o__encode                  (w__sel__next[i])
        );
end
endgenerate


always_comb
begin
    for(int i = 0; i < NUM_OUTPUT_PORTS; i++)   
    begin
	if (w__sel__next[i] == 3'b111)
	o__data_out[i] = {17{1'b0}};
	else
        o__data_out[i] = w__flit_in[i][w__sel__next[i]];   
    end

end


always_comb
begin
    for(int i = 0; i < NUM_OUTPUT_PORTS; i++)
    begin
        w__sel_input[i] = i__sel[i];     
    end
end


always_comb
begin
    
if (regbypass[0] == 1'b1)   
begin
        w__sel_onehot[EAST][0]  = (w__sel_input[EAST][EAST] == 1'b1);
        w__flit_in[EAST][0]     = i__data_in_local[EAST];
end
else begin                  
        w__sel_onehot[EAST][0]  = (w__sel_input[EAST][EAST] == 1'b1);
        w__flit_in[EAST][0]     = i__data_in_remote[EAST];
end
if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[EAST][1]  = (w__sel_input[EAST][SOUTH] == 1'b1);
        w__flit_in[EAST][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[EAST][1]  = (w__sel_input[EAST][SOUTH] == 1'b1);
        w__flit_in[EAST][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[EAST][2]  = (w__sel_input[EAST][WEST] == 1'b1);
        w__flit_in[EAST][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[EAST][2]  = (w__sel_input[EAST][WEST] == 1'b1);
        w__flit_in[EAST][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[EAST][3]  = (w__sel_input[EAST][NORTH] == 1'b1);
        w__flit_in[EAST][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[EAST][3]  = (w__sel_input[EAST][NORTH] == 1'b1);
        w__flit_in[EAST][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[EAST][4]  = (w__sel_input[EAST][ALU_T] == 1'b1);
    w__sel_onehot[EAST][5]  = (w__sel_input[EAST][TREG] == 1'b1);

    w__flit_in[EAST][4]     = i__data_in_remote[ALU_T];
    w__flit_in[EAST][5]     = i__data_in_remote[TREG];


  
if (regbypass[0] == 1'b1)
begin
        w__sel_onehot[SOUTH][0]  = (w__sel_input[SOUTH][EAST] == 1'b1);
        w__flit_in[SOUTH][0]     = i__data_in_local[EAST];
end
else begin
        w__sel_onehot[SOUTH][0]  = (w__sel_input[SOUTH][EAST] == 1'b1);
        w__flit_in[SOUTH][0]     = i__data_in_remote[EAST];
end

if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[SOUTH][1]  = (w__sel_input[SOUTH][SOUTH] == 1'b1);
        w__flit_in[SOUTH][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[SOUTH][1]  = (w__sel_input[SOUTH][SOUTH] == 1'b1);
        w__flit_in[SOUTH][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[SOUTH][2]  = (w__sel_input[SOUTH][WEST] == 1'b1);
        w__flit_in[SOUTH][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[SOUTH][2]  = (w__sel_input[SOUTH][WEST] == 1'b1);
        w__flit_in[SOUTH][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[SOUTH][3]  = (w__sel_input[SOUTH][NORTH] == 1'b1);
        w__flit_in[SOUTH][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[SOUTH][3]  = (w__sel_input[SOUTH][NORTH] == 1'b1);
        w__flit_in[SOUTH][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[SOUTH][4] = (w__sel_input[SOUTH][ALU_T] == 1'b1);
    w__sel_onehot[SOUTH][5] = (w__sel_input[SOUTH][TREG] == 1'b1);

    w__flit_in[SOUTH][4]    = i__data_in_remote[ALU_T];
    w__flit_in[SOUTH][5]    = i__data_in_remote[TREG];


    
if (regbypass[0] == 1'b1)
begin
        w__sel_onehot[WEST][0]  = (w__sel_input[WEST][EAST] == 1'b1);
        w__flit_in[WEST][0]     = i__data_in_local[EAST];
end
else begin
        w__sel_onehot[WEST][0]  = (w__sel_input[WEST][EAST] == 1'b1);
        w__flit_in[WEST][0]     = i__data_in_remote[EAST];
end

if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[WEST][1]  = (w__sel_input[WEST][SOUTH] == 1'b1);
        w__flit_in[WEST][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[WEST][1]  = (w__sel_input[WEST][SOUTH] == 1'b1);
        w__flit_in[WEST][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[WEST][2]  = (w__sel_input[WEST][WEST] == 1'b1);
        w__flit_in[WEST][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[WEST][2]  = (w__sel_input[WEST][WEST] == 1'b1);
        w__flit_in[WEST][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[WEST][3]  = (w__sel_input[WEST][NORTH] == 1'b1);
        w__flit_in[WEST][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[WEST][3]  = (w__sel_input[WEST][NORTH] == 1'b1);
        w__flit_in[WEST][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[WEST][4]  = (w__sel_input[WEST][ALU_T] == 1'b1);
    w__sel_onehot[WEST][5]  = (w__sel_input[WEST][TREG] == 1'b1);

    w__flit_in[WEST][4]     = i__data_in_remote[ALU_T];
    w__flit_in[WEST][5]     = i__data_in_remote[TREG];

    
if (regbypass[0] == 1'b1)
begin
        w__sel_onehot[NORTH][0]  = (w__sel_input[NORTH][EAST] == 1'b1);
        w__flit_in[NORTH][0]     = i__data_in_local[EAST];
end
else begin
        w__sel_onehot[NORTH][0]  = (w__sel_input[NORTH][EAST] == 1'b1);
        w__flit_in[NORTH][0]     = i__data_in_remote[EAST];
end

if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[NORTH][1]  = (w__sel_input[NORTH][SOUTH] == 1'b1);
        w__flit_in[NORTH][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[NORTH][1]  = (w__sel_input[NORTH][SOUTH] == 1'b1);
        w__flit_in[NORTH][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[NORTH][2]  = (w__sel_input[NORTH][WEST] == 1'b1);
        w__flit_in[NORTH][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[NORTH][2]  = (w__sel_input[NORTH][WEST] == 1'b1);
        w__flit_in[NORTH][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[NORTH][3]  = (w__sel_input[NORTH][NORTH] == 1'b1);
        w__flit_in[NORTH][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[NORTH][3]  = (w__sel_input[NORTH][NORTH] == 1'b1);
        w__flit_in[NORTH][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[NORTH][4] = (w__sel_input[NORTH][ALU_T] == 1'b1);
    w__sel_onehot[NORTH][5] = (w__sel_input[NORTH][TREG] == 1'b1);

    w__flit_in[NORTH][4]    = i__data_in_remote[ALU_T];
    w__flit_in[NORTH][5]    = i__data_in_remote[TREG];

    
if (regbypass[0] == 1'b1)
begin
        w__sel_onehot[4][0]  = (w__sel_input[4][EAST] == 1'b1);
        w__flit_in[4][0]     = i__data_in_local[EAST];
end
else begin
        w__sel_onehot[4][0]  = (w__sel_input[4][EAST] == 1'b1);
        w__flit_in[4][0]     = i__data_in_remote[EAST];
end

if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[4][1]  = (w__sel_input[4][SOUTH] == 1'b1);
        w__flit_in[4][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[4][1]  = (w__sel_input[4][SOUTH] == 1'b1);
        w__flit_in[4][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[4][2]  = (w__sel_input[4][WEST] == 1'b1);
        w__flit_in[4][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[4][2]  = (w__sel_input[4][WEST] == 1'b1);
        w__flit_in[4][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[4][3]  = (w__sel_input[4][NORTH] == 1'b1);
        w__flit_in[4][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[4][3]  = (w__sel_input[4][NORTH] == 1'b1);
        w__flit_in[4][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[4][4] = (w__sel_input[4][ALU_T] == 1'b1);
    w__sel_onehot[4][5] = (w__sel_input[4][TREG] == 1'b1);

    w__flit_in[4][4]    = i__data_in_remote[ALU_T];
    w__flit_in[4][5]    = i__data_in_remote[TREG];


    
if (regbypass[0] == 1'b1)
begin
        w__sel_onehot[5][0]  = (w__sel_input[5][EAST] == 1'b1);
        w__flit_in[5][0]     = i__data_in_local[EAST];
end
else begin
        w__sel_onehot[5][0]  = (w__sel_input[5][EAST] == 1'b1);
        w__flit_in[5][0]     = i__data_in_remote[EAST];
end

if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[5][1]  = (w__sel_input[5][SOUTH] == 1'b1);
        w__flit_in[5][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[5][1]  = (w__sel_input[5][SOUTH] == 1'b1);
        w__flit_in[5][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[5][2]  = (w__sel_input[5][WEST] == 1'b1);
        w__flit_in[5][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[5][2]  = (w__sel_input[5][WEST] == 1'b1);
        w__flit_in[5][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[5][3]  = (w__sel_input[5][NORTH] == 1'b1);
        w__flit_in[5][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[5][3]  = (w__sel_input[5][NORTH] == 1'b1);
        w__flit_in[5][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[5][4] = (w__sel_input[5][ALU_T] == 1'b1);
    w__sel_onehot[5][5] = (w__sel_input[5][TREG] == 1'b1);

    w__flit_in[5][4]    = i__data_in_remote[ALU_T];
    w__flit_in[5][5]    = i__data_in_remote[TREG];

    
if (regbypass[0] == 1'b1)
begin
        w__sel_onehot[6][0]  = (w__sel_input[6][EAST] == 1'b1);
        w__flit_in[6][0]     = i__data_in_local[EAST];
end
else begin
        w__sel_onehot[6][0]  = (w__sel_input[6][EAST] == 1'b1);
        w__flit_in[6][0]     = i__data_in_remote[EAST];
end

if (regbypass[3] == 1'b1)
begin
        w__sel_onehot[6][1]  = (w__sel_input[6][SOUTH] == 1'b1);
        w__flit_in[6][1]     = i__data_in_local[SOUTH];
end
else begin
        w__sel_onehot[6][1]  = (w__sel_input[6][SOUTH] == 1'b1);
        w__flit_in[6][1]     = i__data_in_remote[SOUTH];
end

if (regbypass[1] == 1'b1)
begin
        w__sel_onehot[6][2]  = (w__sel_input[6][WEST] == 1'b1);
        w__flit_in[6][2]     = i__data_in_local[WEST];
end
else begin
        w__sel_onehot[6][2]  = (w__sel_input[6][WEST] == 1'b1);
        w__flit_in[6][2]     = i__data_in_remote[WEST];
end

if (regbypass[2] == 1'b1)
begin
        w__sel_onehot[6][3]  = (w__sel_input[6][NORTH] == 1'b1);
        w__flit_in[6][3]     = i__data_in_local[NORTH];
end
else begin
        w__sel_onehot[6][3]  = (w__sel_input[6][NORTH] == 1'b1);
        w__flit_in[6][3]     = i__data_in_remote[NORTH];
end

    w__sel_onehot[6][4] = (w__sel_input[6][ALU_T] == 1'b1);
    w__sel_onehot[6][5] = (w__sel_input[6][TREG] == 1'b1);

    w__flit_in[6][4]    = i__data_in_remote[ALU_T];
    w__flit_in[6][5]    = i__data_in_remote[TREG];

end
endmodule

