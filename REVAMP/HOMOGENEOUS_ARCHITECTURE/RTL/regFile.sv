/*
----------------------------------------------------------------------------------
-- MODULE: Register file (using flip-flops), single read port & single write port.
-- Two implementations for write enable bits: BIT-enabled or BYTE-enabled.
-- Uncomment selected implementation.

To instantiate, paste the following:

reg_file #(.WIDTH(), .DEPTH(), .DEPTH_BITS()) REG_FILE_NAME
(
	.clk(),
    .wr_addr(),
    .wr_data(),
    .wr_bit_en(),
    .wr_en(),
    .rd_addr(),
    .rd_data(),
    .rd_en()
);
----------------------------------------------------------------------------------
*/

module reg_file(
    clk,
    wr_addr,
    wr_data,
    wr_bit_en,
    wr_en,
    rd_addr,
    rd_data,
    rd_en
);

//------------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------
parameter WIDTH = 64;
parameter DEPTH = 24;
parameter DEPTH_BITS = 5; // Number of address bits.

//------------------------------------------------------------------------------
// IO
//------------------------------------------------------------------------------
input clk;
input [DEPTH_BITS-1:0] wr_addr;
input [WIDTH-1:0] wr_data;
input [WIDTH-1:0] wr_bit_en;
input wr_en;
input [DEPTH_BITS-1:0] rd_addr;
output logic [WIDTH-1:0] rd_data;
input rd_en;

//------------------------------------------------------------------------------
// Module
//------------------------------------------------------------------------------

reg [WIDTH-1:0] memory [0:DEPTH-1];

// // Initial block for simulation only. Initializes memory to 0.
// generate
// 	genvar j;
// 	for(j = 0; j < DEPTH; j++) begin
// 		initial begin
// 			memory[j] = 64'b0;
// 		end
// 	end
// endgenerate

assign rd_data = (!rd_en)? memory[rd_addr] : {WIDTH{1'b0}};

// IMPL 1: each bit written controlled by wr_bit_en. Mirrors previous memory module from library, but may be unnecessary. 
generate
	genvar i;
	for(i = 0; i < WIDTH; i++) begin
		always @(posedge clk) begin
			if (~wr_en) begin				
				if(~wr_bit_en[i])
					memory[wr_addr][i] <= wr_data[i];
			end
		end
	end
endgenerate

// // IMPL 2: byte addressable. Mirrors hycube cm_bit_en patterns (64 bits). 
// //
// // Only writes to byte if ALL bits of the selected byte in wr_bit_en are 0. 
// // Different bytes of word can be written simultaneously.
// always @(posedge clk) begin
// 	if (~wr_en) begin				
// 		if(wr_bit_en[7:0] == 0)
// 			memory[wr_addr][7:0] <= wr_data[7:0];
// 		if(wr_bit_en[15:8] == 0)
// 			memory[wr_addr][15:8] <= wr_data[15:8];
// 		if(wr_bit_en[23:16] == 0)
// 			memory[wr_addr][23:16] <= wr_data[23:16];
// 		if(wr_bit_en[31:24] == 0)
// 			memory[wr_addr][31:24] <= wr_data[31:24];
// 		if(wr_bit_en[39:32] == 0)
// 			memory[wr_addr][39:32] <= wr_data[39:32];
// 		if(wr_bit_en[47:40] == 0)
// 			memory[wr_addr][47:40] <= wr_data[47:40];
// 		if(wr_bit_en[55:48] == 0)
// 			memory[wr_addr][55:48] <= wr_data[55:48];
// 		if(wr_bit_en[63:56] == 0)
// 			memory[wr_addr][63:56] <= wr_data[63:56];
// 	end
// end

endmodule: reg_file




















































