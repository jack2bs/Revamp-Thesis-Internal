`timescale 1ns/1ps
module cgra(
    clk,
    reset,
    chip_en,
    data_in,
    address_in,
    data_out,
    scan_data_or_addr,
    read_write,
    data_out_valid,
    data_addr_valid,
    scan_start_exec,
    bist_en,
    exec_end
);

input logic                             reset;
input logic                             clk;
input logic                             chip_en;

input  logic                            scan_start_exec;

output  logic                           data_out_valid;
input  [1:0]                            data_addr_valid;

input [31:0]                            data_in;
input [19:0]                            address_in;
output [31:0]                           data_out;
input  logic                            scan_data_or_addr;
input  logic                            read_write;
output logic                            exec_end;
input logic                             bist_en;

import TopPkg::*;
import SMARTPkg::*;

`ifdef HETERO
`include "hetero_config.v"
`endif

parameter DATA_WIDTH 			= 32;
`ifdef HETERO_CONFIG
parameter CM_WIDTH			= 68;
`else
parameter CM_WIDTH			= 64;
`endif
parameter DATA_DEPTH			= 9;

parameter CM_DEPTH			= 32;
parameter CM_DEPTH_BITS                 = $clog2(CM_DEPTH);


parameter DM_DEPTH			= 512;
parameter DM_DEPTH_BITS                 = $clog2(DM_DEPTH);

parameter NUM_RESET_SYNC_STAGES         = 2;


//parameter ENABLE =32; 
//parameter int ENABLE[36] = {32,32,32,32,32,32,32,16,16,16,16,32,32,32,32,32,32,32,16,16,16,16,32,32,32,32,32,32,32,32,32,32,32,32,16,16};
//parameter ROUTER_CONFIG =8; 
//parameter OPCODE_CONFIG=8;
//parameter LOCAL_CONFIG=8;
//parameter CONST_CONFIG=8;
//parameter ALU_ADD=0;	
//parameter ALU_CMP=0;
//parameter ALU_LOG=1;
//parameter ALU_MUL=1;
//parameter ALU_MEM=1;	

localparam TILE_NUM_INPUT_PORTS         = 6;
localparam TILE_NUM_OUTPUT_PORTS        = 7;
localparam NUM_CLUSTERS			= 4;

logic start_exec;
reg [31:0] data_out;
reg [63:0] look_up_table_reg;
reg [CM_WIDTH-1:0] cm_data;
reg [DATA_WIDTH-1:0] dm_data;
reg [DATA_WIDTH-1:0] dm_bit_en;
reg [CM_WIDTH-1:0] cm_bit_en;
reg bist_success;
reg [1:0] data_addr_valid_reg;
//reg data_or_addr_reg;
wire chip_en_dm0;
wire chip_en_dm8;
wire chip_en_dm16;
wire chip_en_dm24;
wire chip_en_dm32;
wire chip_en_dm40;
wire chip_en_dm48;
wire chip_en_dm56;

/*always @ (posedge clk)
begin
if (reset)
begin
	data_addr_valid_reg <=2'b00;
	data_or_addr_reg <= 1'b0;
end
else if (chip_en) begin
	data_addr_valid_reg <=data_addr_valid;
	data_or_addr_reg <=scan_data_or_addr;
end 
end */

//always @ (posedge clk)
always_comb
begin
if (reset) begin
	cm_data = {{CM_WIDTH{1'b0}}};
	cm_bit_en = {CM_WIDTH{1'b1}};
end
else begin
if (address_in[18]==1'b0 && address_in[19]==1'b0) begin
        if(address_in[2:1]==2'b00) begin
                if (data_addr_valid==2'b01) begin
                        cm_data = {{56{1'b0}},data_in[7:0]};
			cm_bit_en = {{56{1'b0}},{8{1'b1}}};
                end
                else if (data_addr_valid==2'b10) begin
                        cm_data = {{48{1'b0}},data_in[15:8],{8{1'b0}}};
			cm_bit_en = {{48{1'b0}},{8{1'b1}},{8{1'b0}}};
                end
                else if (data_addr_valid==2'b11) begin
                        cm_data = {{48{1'b0}},data_in[15:0]};
			cm_bit_en = {{48{1'b0}},{16{1'b1}}};
                end
                else if (data_addr_valid==2'b00) begin
                        cm_data = {{48{1'b0}},data_in[15:0]};
			cm_bit_en = {{48{1'b0}},{8{1'b0}},{8{1'b0}}};
                end
        end
        else if(address_in[2:1]==2'b01) begin
                if (data_addr_valid==2'b01) begin
                        cm_data = {{40{1'b0}},data_in[7:0],{16{1'b0}}};
			cm_bit_en = {{40{1'b0}},{8{1'b1}},{16{1'b0}}};
                end
                else if (data_addr_valid==2'b10) begin
                        cm_data = {{32{1'b0}},data_in[15:8],{8{1'b0}},{16{1'b0}}};
			cm_bit_en = {{32{1'b0}},{8{1'b1}},{24{1'b0}}};
                end
                else if (data_addr_valid==2'b11) begin
                        cm_data = {{32{1'b0}},data_in[15:0],{16{1'b0}}};
			cm_bit_en = {{32{1'b0}},{16{1'b1}},{16{1'b0}}};
                end
                else if (data_addr_valid==2'b00) begin
                        cm_data = {{32{1'b0}},data_in[15:0],{16{1'b0}}};
			cm_bit_en = {{32{1'b0}},{16{1'b0}},{16{1'b0}}};
                end
        end
        else if(address_in[2:1]==2'b10) begin
                if (data_addr_valid==2'b01) begin
                        cm_data = {{24{1'b0}},data_in[7:0],{32{1'b0}}};
			cm_bit_en = {{24{1'b0}},{8{1'b1}},{32{1'b0}}};
                end
                else if (data_addr_valid==2'b10) begin
                        cm_data = {{16{1'b0}},data_in[15:8],{8{1'b0}},{32{1'b0}}};
			cm_bit_en = {{16{1'b0}},{8{1'b1}},{40{1'b0}}};
                end
                else if (data_addr_valid==2'b11) begin
                        cm_data = {{16{1'b0}},data_in[15:0],{32{1'b0}}};
			cm_bit_en = {{16{1'b0}},{16{1'b1}},{32{1'b0}}};
                end
                else if (data_addr_valid==2'b00) begin
                        cm_data = {{16{1'b0}},data_in[15:0],{32{1'b0}}};
			cm_bit_en = {{16{1'b0}},{16{1'b0}},{32{1'b0}}};
                end
        end
        else if(address_in[2:1]==2'b11) begin 
                if (data_addr_valid==2'b01) begin
                        cm_data = {{8{1'b0}},data_in[7:0],{48{1'b0}}};
			cm_bit_en = {{8{1'b0}},{8{1'b1}},{48{1'b0}}}; 
                end
                else if (data_addr_valid==2'b10) begin
                        cm_data = {data_in[15:8],{8{1'b0}},{48{1'b0}}};
			cm_bit_en = {{8{1'b1}},{8{1'b0}},{48{1'b1}}}; 
                end
                else if (data_addr_valid==2'b11) begin
                        cm_data = {data_in[15:0],{48{1'b0}}};
			cm_bit_en = {{8{1'b1}},{8{1'b1}},{48{1'b0}}}; 
                end
                else if (data_addr_valid==2'b00) begin
                        cm_data = {data_in[15:0],{48{1'b0}}};
			cm_bit_en = {{8{1'b0}},{8{1'b0}},{48{1'b0}}}; 
                end
        end
	else begin
		cm_data = {{CM_WIDTH{1'b0}}};
		cm_bit_en = {{CM_WIDTH{1'b0}}};
	end
end 
else begin
	
	cm_data = {{CM_WIDTH{1'b0}}};
	cm_bit_en = {{CM_WIDTH{1'b0}}};
end
end
end 

reg [35:0] cm_en;
reg [CM_WIDTH-1:0] cm_bit_en0;
reg [CM_WIDTH-1:0] cm_bit_en1;
reg [CM_WIDTH-1:0] cm_bit_en2;
reg [CM_WIDTH-1:0] cm_bit_en3;
reg [CM_WIDTH-1:0] cm_bit_en4;
reg [CM_WIDTH-1:0] cm_bit_en5;
reg [CM_WIDTH-1:0] cm_bit_en6;
reg [CM_WIDTH-1:0] cm_bit_en7;
reg [CM_WIDTH-1:0] cm_bit_en8;
reg [CM_WIDTH-1:0] cm_bit_en9;
reg [CM_WIDTH-1:0] cm_bit_en10;
reg [CM_WIDTH-1:0] cm_bit_en11;
reg [CM_WIDTH-1:0] cm_bit_en12;
reg [CM_WIDTH-1:0] cm_bit_en13;
reg [CM_WIDTH-1:0] cm_bit_en14;
reg [CM_WIDTH-1:0] cm_bit_en15;
reg [CM_WIDTH-1:0] cm_bit_en16;
reg [CM_WIDTH-1:0] cm_bit_en17;
reg [CM_WIDTH-1:0] cm_bit_en18;
reg [CM_WIDTH-1:0] cm_bit_en19;
reg [CM_WIDTH-1:0] cm_bit_en20;
reg [CM_WIDTH-1:0] cm_bit_en21;
reg [CM_WIDTH-1:0] cm_bit_en22;
reg [CM_WIDTH-1:0] cm_bit_en23;
reg [CM_WIDTH-1:0] cm_bit_en24;
reg [CM_WIDTH-1:0] cm_bit_en25;
reg [CM_WIDTH-1:0] cm_bit_en26;
reg [CM_WIDTH-1:0] cm_bit_en27;
reg [CM_WIDTH-1:0] cm_bit_en28;
reg [CM_WIDTH-1:0] cm_bit_en29;
reg [CM_WIDTH-1:0] cm_bit_en30;
reg [CM_WIDTH-1:0] cm_bit_en31;
reg [CM_WIDTH-1:0] cm_bit_en32;
reg [CM_WIDTH-1:0] cm_bit_en33;
reg [CM_WIDTH-1:0] cm_bit_en34;
reg [CM_WIDTH-1:0] cm_bit_en35;

//always @ (posedge clk)
always_comb
begin
if (reset) begin
	cm_en = 36'b000000000000000000000000000000000000;
end
else begin
        if(data_addr_valid==2'b00 || scan_data_or_addr==1'b1 || (address_in[18]==1'b1) || (address_in[19]==1'b1)) begin
                cm_en = 36'b000000000000000000000000000000000000;
	end
        else begin
                case(address_in[12:7])
                6'b000000: cm_en = 36'b000000000000000000000000000000000001;
                6'b000001: cm_en = 36'b000000000000000000000000000000000010;
                6'b000010: cm_en = 36'b000000000000000000000000000000000100;
                6'b000011: cm_en = 36'b000000000000000000000000000000001000;
                6'b000100: cm_en = 36'b000000000000000000000000000000010000;
                6'b000101: cm_en = 36'b000000000000000000000000000000100000;
                6'b000110: cm_en = 36'b000000000000000000000000000001000000;
                6'b000111: cm_en = 36'b000000000000000000000000000010000000;
                6'b001000: cm_en = 36'b000000000000000000000000000100000000;
                6'b001001: cm_en = 36'b000000000000000000000000001000000000;
                6'b001010: cm_en = 36'b000000000000000000000000010000000000;
                6'b001011: cm_en = 36'b000000000000000000000000100000000000;
                6'b001100: cm_en = 36'b000000000000000000000001000000000000;
                6'b001101: cm_en = 36'b000000000000000000000010000000000000;
                6'b001110: cm_en = 36'b000000000000000000000100000000000000;
                6'b001111: cm_en = 36'b000000000000000000001000000000000000;
                6'b010000: cm_en = 36'b000000000000000000010000000000000000;
                6'b010001: cm_en = 36'b000000000000000000100000000000000000;
                6'b010010: cm_en = 36'b000000000000000001000000000000000000;
                6'b010011: cm_en = 36'b000000000000000010000000000000000000;
                6'b010100: cm_en = 36'b000000000000000100000000000000000000;
                6'b010101: cm_en = 36'b000000000000001000000000000000000000;
                6'b010110: cm_en = 36'b000000000000010000000000000000000000;
                6'b010111: cm_en = 36'b000000000000100000000000000000000000;
                6'b011000: cm_en = 36'b000000000001000000000000000000000000;
                6'b011001: cm_en = 36'b000000000010000000000000000000000000;
                6'b011010: cm_en = 36'b000000000100000000000000000000000000;
                6'b011011: cm_en = 36'b000000001000000000000000000000000000;
                6'b011100: cm_en = 36'b000000010000000000000000000000000000;
                6'b011101: cm_en = 36'b000000100000000000000000000000000000;
                6'b011110: cm_en = 36'b000001000000000000000000000000000000;
                6'b011111: cm_en = 36'b000010000000000000000000000000000000;
                6'b100000: cm_en = 36'b000100000000000000000000000000000000;
                6'b100001: cm_en = 36'b001000000000000000000000000000000000;
                6'b100010: cm_en = 36'b010000000000000000000000000000000000;
                6'b100011: cm_en = 36'b100000000000000000000000000000000000;
		default  : cm_en = 36'b000000000000000000000000000000000000;
                endcase 
        end
end
end

assign cm_bit_en0  = (cm_en == 36'b000000000000000000000000000000000001) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en1  = (cm_en == 36'b000000000000000000000000000000000010) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en2  = (cm_en == 36'b000000000000000000000000000000000100) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en3  = (cm_en == 36'b000000000000000000000000000000001000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en4  = (cm_en == 36'b000000000000000000000000000000010000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en5  = (cm_en == 36'b000000000000000000000000000000100000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en6  = (cm_en == 36'b000000000000000000000000000001000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en7  = (cm_en == 36'b000000000000000000000000000010000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en8  = (cm_en == 36'b000000000000000000000000000100000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en9  = (cm_en == 36'b000000000000000000000000001000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en10 = (cm_en == 36'b000000000000000000000000010000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en11 = (cm_en == 36'b000000000000000000000000100000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en12 = (cm_en == 36'b000000000000000000000001000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en13 = (cm_en == 36'b000000000000000000000010000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en14 = (cm_en == 36'b000000000000000000000100000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en15 = (cm_en == 36'b000000000000000000001000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en16 = (cm_en == 36'b000000000000000000010000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en17 = (cm_en == 36'b000000000000000000100000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en18 = (cm_en == 36'b000000000000000001000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en19 = (cm_en == 36'b000000000000000010000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en20 = (cm_en == 36'b000000000000000100000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en21 = (cm_en == 36'b000000000000001000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en22 = (cm_en == 36'b000000000000010000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en23 = (cm_en == 36'b000000000000100000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en24 = (cm_en == 36'b000000000001000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en25 = (cm_en == 36'b000000000010000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en26 = (cm_en == 36'b000000000100000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en27 = (cm_en == 36'b000000001000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en28 = (cm_en == 36'b000000010000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en29 = (cm_en == 36'b000000100000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en30 = (cm_en == 36'b000001000000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en31 = (cm_en == 36'b000010000000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en32 = (cm_en == 36'b000100000000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en33 = (cm_en == 36'b001000000000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en34 = (cm_en == 36'b010000000000000000000000000000000000) ? cm_bit_en : {64{1'b0}};
assign cm_bit_en35 = (cm_en == 36'b100000000000000000000000000000000000) ? cm_bit_en : {64{1'b0}};

always @ (posedge clk)
begin
if (reset)
begin
        start_exec <= 1'b0;
end
else if (chip_en)
begin
                start_exec <= scan_start_exec;
end
end

logic [NUM_RESET_SYNC_STAGES-1:0]       r__reset__pff;

FlitFixed                w__flit_in                          [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_INPUT_PORTS-1-2:0];
FlitFixed                w__flit_out                         [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_OUTPUT_PORTS-1-3:0];
logic scan_chain_out_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];

logic [CM_WIDTH-1:0] data_out_cm_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic rd_en_shifted_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic [CM_DEPTH_BITS-1:0] addr_shifted_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic [DM_DEPTH_BITS-1:0] addr_dm_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic [DATA_WIDTH-1:0] bit_en_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic [DATA_WIDTH-1:0] data_in_dm_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic rd_en_dm_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic wr_en_dm_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
wire [3:0] addr;

generate
genvar i, j, k;

for(i = 0; i < NUM_TILES_Y; i++)
begin: y
    for(j = 0; j < NUM_TILES_X; j++)
    begin: x
        TileXYId                        my_xy_id;

        FlitFixed        my_flit_in                      [TILE_NUM_INPUT_PORTS-1-2:0];
        FlitFixed        my_flit_out                     [TILE_NUM_OUTPUT_PORTS-1-3:0];


wire [CM_DEPTH_BITS-1:0] loop_start;
wire [CM_DEPTH_BITS-1:0] loop_end;
wire [CM_WIDTH-1:0] data_in;
wire wr_en;

wire [4:0] addr;
wire [DM_DEPTH_BITS-1:0] addr_dm;
wire [DATA_WIDTH-1:0] bit_en;
wire [DATA_WIDTH-1:0] data_in_dm;
wire rd_en_dm;
wire wr_en_dm;
wire is_dm_tile;

reg wr_en_shifted;
reg rd_en_shifted;

wire scan_chain_out;
        assign my_flit_in[EAST]     = w__flit_in[i][j][EAST];
        assign my_flit_in[SOUTH]    = w__flit_in[i][j][SOUTH];
        assign my_flit_in[WEST]     = w__flit_in[i][j][WEST];
        assign my_flit_in[NORTH]    = w__flit_in[i][j][NORTH];

        assign w__flit_out[i][j][EAST]      = my_flit_out[EAST];
        assign w__flit_out[i][j][SOUTH]     = my_flit_out[SOUTH];
        assign w__flit_out[i][j][WEST]      = my_flit_out[WEST];
        assign w__flit_out[i][j][NORTH]     = my_flit_out[NORTH];
 
wire [DATA_WIDTH-1:0] data_out_dm;


assign is_dm_tile = (i==0 && j==0) ? 1'b1 : (i==0 && j==1) ? 1'b1 : (i==0 && j==2) ? 1'b1 : (i==0 && j==3) ? 1'b1 : (i==0 && j==4) ? 1'b1 : (i==0 && j==5) ? 1'b1 : (i==5 && j==0) ? 1'b1 : (i==5 && j==1) ? 1'b1 : (i==5 && j==2) ? 1'b1 : (i==5 && j==3) ? 1'b1: (i==5 && j==4) ? 1'b1 : (i==5 && j==5) ? 1'b1 : 1'b0 ;

wire [CM_WIDTH-1:0] data_out;
reg [CM_DEPTH_BITS-1:0] addr_shifted;
reg [CM_WIDTH-1:0] control_reg_data;

always @(posedge clk) begin
if (reset) begin
	wr_en_shifted <= 1'b1;
	rd_en_shifted <= 1'b1;
end
else if(chip_en) begin
	if (scan_start_exec) begin
		rd_en_shifted <= 1'b0;
		wr_en_shifted <= 1'b1;
	end
	else begin
		rd_en_shifted <= ~read_write;
		wr_en_shifted <= read_write;
	end
end
end
wire [3:0] loop_end_dash;

assign loop_end_dash = (loop_end < loop_start) ? (loop_end + 4'b1000) : loop_end; 


assign addr_dm_out[i][j] = addr_dm;
assign bit_en_out[i][j] = bit_en;
assign data_in_dm_out[i][j] = data_in_dm;
assign rd_en_dm_out[i][j] = rd_en_dm;
assign wr_en_dm_out[i][j] = wr_en_dm;
assign rd_en_shifted_out[i][j] = rd_en_shifted;
assign data_out_cm_out[i][j] = data_out;

reg [4:0] jumpl_reg;
always @(posedge clk)
begin
if (reset)
	jumpl_reg <= 5'b00000;
else if(chip_en) begin
	jumpl_reg <= control_reg_data[34:30];
end
end

always @(posedge clk)
begin
if (reset)
	addr_shifted <= 4'b0000;
else if (chip_en) begin
	if (start_exec) begin
		if (control_reg_data[34:30]==5'b11110 && jumpl_reg != 5'b11110) begin
			addr_shifted <= {1'b0,control_reg_data[45:43]};
		end
		else if ((addr_shifted >= loop_end_dash) || (addr_shifted < loop_start)) begin
			addr_shifted <= loop_start;
		end
		else begin
			addr_shifted <= addr_shifted + 1'b1;
		end
	end
	else begin 
		addr_shifted <= address_in[6:3];
	end
end
end

assign addr_shifted_out[i][j] = addr_shifted;
reg [CM_WIDTH-1:0] cm_bit_en_shifted;
logic cm_en_shifted;
/*
`ifdef HETERO
if (i==0 && j==0)begin
	defparam genblk1.tile.ENABLE =ENABLE_0_0 ;
end

`endif
*/

always_comb begin
if (reset)
        cm_bit_en_shifted <= {CM_WIDTH{1'b1}};
else if (chip_en && !start_exec) begin
        if (i==0 && j==0)begin
                cm_bit_en_shifted <= cm_bit_en0;
                cm_en_shifted = cm_en[0];
	//	defparam tile.ENABLE = 0;
	//	defparam ROUTER_CONFIG = ROUTER_CONFIG_0_0;
	//	defparam OPCODE_CONFIG = OPCODE_CONFIG_0_0;
	//	defparam LOCAL_CONFIG  = LOCAL_CONFIG_0_0;
	//	defparam CONST_CONFIG  = CONST_CONFIG_0_0;
	//	defparam ALU_ADD = ALU_ADD_0_0;
	//	defparam ALU_MUL = ALU_MUL_0_0;
	//	defparam ALU_LOG = ALU_LOG_0_0;
	//	defparam ALU_CMP = ALU_CMP_0_0;
        end
        else if (i==0 && j==1)begin
                cm_bit_en_shifted <= cm_bit_en1;
                cm_en_shifted = cm_en[1];
        end
        else if (i==0 && j==2)begin
                cm_bit_en_shifted <= cm_bit_en2;
                cm_en_shifted = cm_en[2];
        end
        else if (i==0 && j==3)begin
                cm_bit_en_shifted <= cm_bit_en3;
                cm_en_shifted = cm_en[3];
        end
        else if (i==0 && j==4)begin
                cm_bit_en_shifted <= cm_bit_en4;
                cm_en_shifted = cm_en[4];
        end
        else if (i==0 && j==5)begin
                cm_bit_en_shifted <= cm_bit_en5;
                cm_en_shifted = cm_en[5];
        end
        else if (i==1 && j==0)begin
                cm_bit_en_shifted <= cm_bit_en6;
                cm_en_shifted = cm_en[6];
        end
        else if (i==1 && j==1)begin
                cm_bit_en_shifted <= cm_bit_en7;
                cm_en_shifted = cm_en[7];
        end
        else if (i==1 && j==2)begin
                cm_bit_en_shifted <= cm_bit_en8;
                cm_en_shifted = cm_en[8];
        end
        else if (i==1 && j==3)begin
                cm_bit_en_shifted <= cm_bit_en9;
                cm_en_shifted = cm_en[9];
        end
        else if (i==1 && j==4)begin
                cm_bit_en_shifted <= cm_bit_en10;
                cm_en_shifted = cm_en[10];
        end
        else if (i==1 && j==5)begin
                cm_bit_en_shifted <= cm_bit_en11;
                cm_en_shifted = cm_en[11];
        end
        else if (i==2 && j==0)begin
                cm_bit_en_shifted <= cm_bit_en12;
                cm_en_shifted = cm_en[12];
        end
        else if (i==2 && j==1)begin
                cm_bit_en_shifted <= cm_bit_en13;
                cm_en_shifted = cm_en[13];
        end
        else if (i==2 && j==2)begin
                cm_bit_en_shifted <= cm_bit_en14;
                cm_en_shifted = cm_en[14];
        end
        else if (i==2 && j==3)begin
                cm_bit_en_shifted <= cm_bit_en15;
                cm_en_shifted = cm_en[15];
        end
        else if (i==2 && j==4)begin
                cm_bit_en_shifted <= cm_bit_en16;
                cm_en_shifted = cm_en[16];
        end
        else if (i==2 && j==5)begin
                cm_bit_en_shifted <= cm_bit_en17;
                cm_en_shifted = cm_en[17];
        end
        else if (i==3 && j==0)begin
                cm_bit_en_shifted <= cm_bit_en18;
                cm_en_shifted = cm_en[18];
        end
        else if (i==3 && j==1)begin
                cm_bit_en_shifted <= cm_bit_en19;
                cm_en_shifted = cm_en[19];
        end
        else if (i==3 && j==2)begin
                cm_bit_en_shifted <= cm_bit_en20;
                cm_en_shifted = cm_en[20];
        end
        else if (i==3 && j==3)begin
                cm_bit_en_shifted <= cm_bit_en21;
                cm_en_shifted = cm_en[21];
        end
        else if (i==3 && j==4)begin
                cm_bit_en_shifted <= cm_bit_en22;
                cm_en_shifted = cm_en[22];
        end
	else if (i==3 && j==5)begin
                cm_bit_en_shifted <= cm_bit_en23;
                cm_en_shifted = cm_en[23];
        end
        else if (i==4 && j==0)begin
                cm_bit_en_shifted <= cm_bit_en24;
                cm_en_shifted = cm_en[24];
        end
        else if (i==4 && j==1)begin
                cm_bit_en_shifted <= cm_bit_en25;
                cm_en_shifted = cm_en[25];
        end
        else if (i==4 && j==2)begin
                cm_bit_en_shifted <= cm_bit_en26;
                cm_en_shifted = cm_en[26];
        end
        else if (i==4 && j==3)begin
                cm_bit_en_shifted <= cm_bit_en27;
                cm_en_shifted = cm_en[27];
        end
        else if (i==4 && j==4)begin
                cm_bit_en_shifted <= cm_bit_en28;
                cm_en_shifted = cm_en[28];
        end
        else if (i==4 && j==5)begin
                cm_bit_en_shifted <= cm_bit_en29;
                cm_en_shifted = cm_en[29];
        end
        else if (i==5 && j==0)begin
                cm_bit_en_shifted <= cm_bit_en30;
                cm_en_shifted = cm_en[30];
        end
        else if (i==5 && j==1)begin
                cm_bit_en_shifted <= cm_bit_en31;
                cm_en_shifted = cm_en[31];
        end
        else if (i==5 && j==2)begin
                cm_bit_en_shifted <= cm_bit_en32;
                cm_en_shifted = cm_en[32];
        end
        else if (i==5 && j==3)begin
                cm_bit_en_shifted <= cm_bit_en33;
                cm_en_shifted = cm_en[33];
        end
        else if (i==5 && j==4)begin
                cm_bit_en_shifted <= cm_bit_en34;
                cm_en_shifted = cm_en[34];
        end
        else if (i==5 && j==5)begin
                cm_bit_en_shifted <= cm_bit_en35;
                cm_en_shifted = cm_en[35];
        end
else begin
                cm_bit_en_shifted <= {CM_WIDTH{1'b1}};
                cm_en_shifted = 1'b1;
        end
end
else if(start_exec)
        cm_en_shifted = 1'b0;
end



reg [CM_WIDTH-1:0] cm_data_shifted;

always_comb begin
if (reset) begin
	cm_data_shifted <= {CM_WIDTH{1'b0}};
end
else if (chip_en) begin
	cm_data_shifted <= cm_data;
end
end
assign clk_shifted=~clk;

        tile 
`ifdef HETERO
	#( 
		.ENABLE(ENABLE[i*NUM_TILES_Y+j]), 
		.ROUTER_CONFIG(ROUTER_CONFIG[i*NUM_TILES_Y+j]), 
		.OPCODE_CONFIG(OPCODE_CONFIG[i*NUM_TILES_Y+j]),
		.LOCAL_CONFIG(LOCAL_CONFIG[i*NUM_TILES_Y+j]),
		.CONST_CONFIG(CONST_CONFIG[i*NUM_TILES_Y+j]),
		.ALU_ADD(ALU_ADD[i*NUM_TILES_Y+j]),	
		.ALU_CMP(ALU_CMP[i*NUM_TILES_Y+j]),
		.ALU_LOG(ALU_LOG[i*NUM_TILES_Y+j]),
		.ALU_MUL(ALU_MUL[i*NUM_TILES_Y+j]),
		.ALU_MEM(ALU_MEM[i*NUM_TILES_Y+j]))
`endif
            tile(
                .clk                                (clk_shifted),
                .reset                              (r__reset__pff[NUM_RESET_SYNC_STAGES-1]),
                .i__flit_in                         (my_flit_in),
                .o__flit_out                        (my_flit_out),
		.control_mem_data_in                (cm_data_shifted),
                .control_mem_en                     (cm_en_shifted),
                .control_mem_bit_en                 (cm_bit_en_shifted),
                .control_mem_addr                   (addr_shifted),
                .control_mem_wr_en                  (wr_en_shifted),
		.is_dm_tile			    (is_dm_tile),
		.start_exec			    (start_exec),
		.loop_start		    	    (loop_start),
		.loop_end		    	    (loop_end),
		.data_in_dm			    (data_in_dm),
		.data_out_dm			    (data_out_dm),
		.addr_dm			    (addr_dm),
		.bit_en				    (bit_en),
		.rd_en_dm			    (rd_en_dm),
		.wr_en_dm			    (wr_en_dm)
            );
    end
end

for(i = 0; i < NUM_TILES_Y; i++)
begin: y_connection
    for(j = 0; j < NUM_TILES_X; j++)
    begin: x_connection

        // East
        if(j < (NUM_TILES_X-2))
        begin: flit_east
            assign w__flit_in[i][j][EAST]   = w__flit_out[i][j + 2][WEST];
        end
        else if(j == (NUM_TILES_X-2))
        begin
            assign w__flit_in[i][j][EAST]   = w__flit_out[i][j + 1][EAST];
        end
	else
	begin
            assign w__flit_in[i][j][EAST]   = w__flit_out[i][j - 1][EAST];
        end

        // South
        if(i >= 2)
        begin: flit_south
            assign w__flit_in[i][j][NORTH]      = w__flit_out[i - 2][j][SOUTH];
        end
	else if(i >= 1)
	begin 
            assign w__flit_in[i][j][NORTH]      = w__flit_out[i - 1][j][NORTH];
        end
        else
        begin 
            assign w__flit_in[i][j][NORTH]      = w__flit_out[i + 1][j][NORTH];
        end

        // West
        if(j >= 2)
        begin: flit_west
            assign w__flit_in[i][j][WEST]   = w__flit_out[i][j - 2][EAST];
        end
	else if(j >= 1)
	begin 
            assign w__flit_in[i][j][WEST]   = w__flit_out[i][j - 1][WEST];
        end
        else
        begin 
            assign w__flit_in[i][j][WEST]   = w__flit_out[i][j + 1][WEST];
        end

        // North
        if(i < (NUM_TILES_Y-2))
        begin: flit_north
            assign w__flit_in[i][j][SOUTH]      = w__flit_out[i + 2][j][NORTH];
        end
	else if(i == (NUM_TILES_Y-2))
	begin 
            assign w__flit_in[i][j][SOUTH]      = w__flit_out[i + 1][j][SOUTH];
        end
        else
        begin 
            assign w__flit_in[i][j][SOUTH]      = w__flit_out[i - 1][j][SOUTH];
        end
 
    end
end

endgenerate

always_ff @ (posedge clk)
begin
    if(reset == 1'b1)
    begin
        r__reset__pff <= '1;
    end
    else if (chip_en) 
    begin
        r__reset__pff <= {r__reset__pff[NUM_RESET_SYNC_STAGES-2:0], 1'b0};
    end
end



endmodule

