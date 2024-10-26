
module tile(
    clk,
    reset,
    i__flit_in,
    o__flit_out,
    control_mem_data_in,
    control_mem_en,
    control_mem_bit_en,
    control_mem_addr,
    control_mem_wr_en,
    is_dm_tile,
    start_exec,
    loop_start,
    loop_end,
    look_up_table,
    data_in_dm,
    data_out_dm,
    addr_dm,    
    bit_en,    
    rd_en_dm,
    wr_en_dm
);

import TopPkg::ScanControl;
import SMARTPkg::*;

parameter DATA_WIDTH			= 32;
parameter CM_WIDTH 			= 64;
parameter CM_DEPTH			= 32;
parameter CM_DEPTH_BITS                 = $clog2(CM_DEPTH);
parameter DM_DEPTH			= 512;
parameter DM_DEPTH_BITS                 = $clog2(DM_DEPTH);
parameter NIC_OUTPUT_FIFO_DEPTH         = 8;
parameter ROUTER_NUM_VCS                = 8;

localparam NUM_INPUT_PORTS              = 6;
localparam NUM_OUTPUT_PORTS             = 7;
localparam ROUTER_NUM_INPUT_PORTS       = NUM_INPUT_PORTS;
localparam ROUTER_NUM_OUTPUT_PORTS      = NUM_OUTPUT_PORTS;

input  logic                            clk;
input  logic                            reset;

input  FlitFixed         i__flit_in                      [NUM_INPUT_PORTS-1-2:0];

output FlitFixed         o__flit_out                     [NUM_OUTPUT_PORTS-1-3:0];

input logic  [CM_WIDTH-1:0]             control_mem_data_in;
input logic                             control_mem_en;
input logic  [CM_WIDTH-1:0]             control_mem_bit_en;
input logic  [CM_DEPTH_BITS-1:0]        control_mem_addr;
input logic                             control_mem_wr_en;


input  [63:0]			        look_up_table;
input 	is_dm_tile;
input 	start_exec;
output [CM_DEPTH_BITS-1:0] loop_start;
output [CM_DEPTH_BITS-1:0] loop_end;
output [DM_DEPTH_BITS-1:0] addr_dm;
output [DATA_WIDTH-1:0] bit_en;
input [DATA_WIDTH-1:0] data_out_dm;
output [DATA_WIDTH-1:0] data_in_dm;
output rd_en_dm;
output wr_en_dm;


logic                                   r__reset__pff;



logic [ROUTER_NUM_INPUT_PORTS-1:0]	i__sram_xbar_sel	[ROUTER_NUM_OUTPUT_PORTS-1:0];

`ifdef HETERO_NETWORK
FlitFixed         o__flit_out_wire                     [NUM_OUTPUT_PORTS-1-3:0];
FlitFixed1         o__flit_out_wire_local                     [2:0];
`else
FlitFixed         o__flit_out_wire                     [NUM_OUTPUT_PORTS-1:0];
`endif
FlitFixed         i__flit_in_wire                     [NUM_INPUT_PORTS-1:0];

logic [DATA_WIDTH-1:0] data_out_dm;
logic [DATA_WIDTH-1:0] data_out_dm_reg;
logic [DM_DEPTH_BITS-1:0] addr_dm;
logic [DATA_WIDTH-1:0] bit_en;
logic [DATA_WIDTH-1:0] data_in_dm;
logic rd_en_dm;
logic wr_en_dm;
logic [CM_WIDTH-1:0] control_reg_data;
logic [63:0] look_up_table;

logic [DATA_WIDTH-1:0] lsu_alu_out;
logic not_to_execute_reg;



logic Cout;
logic [DATA_WIDTH-1:0] alu_out;
logic [DATA_WIDTH-1:0] alu_out_dm_tiles;
logic [DATA_WIDTH:0] tile_out;
logic [DATA_WIDTH:0] treg;
logic [DATA_WIDTH:0] op_rhs;
logic [DATA_WIDTH:0] op_lhs;
logic [DATA_WIDTH:0] op_predicate_reg;
logic [DATA_WIDTH:0] op_predicate;
logic [DATA_WIDTH:0] op_shift;

logic [5:0] operation;
logic [3:0] regbypass;
logic [3:0] regWEN;

logic [CM_DEPTH_BITS-1:0] current_loop;

logic current_loop_prev_cycle;
logic [CM_DEPTH_BITS-1:0] loop_start;
logic [CM_DEPTH_BITS-1:0] loop_end;

logic [CM_DEPTH_BITS-1:0] ls_lut;
logic [CM_DEPTH_BITS-1:0] le_lut;

logic valid_lut;
logic [8:0] prev_cycle_p_i2_i1;
logic not_to_execute;
logic not_to_execute_all;
logic not_to_execute_select;

logic [DATA_WIDTH-1:0] data_out_ldst;
logic [DATA_WIDTH-1:0] data_in_ldst;
logic [DM_DEPTH_BITS:0] ldst_addr_in;
logic control_mem_en_t;


always @(posedge clk)
begin
if (reset)
	prev_cycle_p_i2_i1 <= 9'b111111111;
else if (start_exec)
	prev_cycle_p_i2_i1 <= control_reg_data[20:12];
end

always @(posedge clk)
begin
if (reset || control_reg_data[34:30]==5'b11110)
begin
	op_rhs <= {DATA_WIDTH+1{1'b0}};
	op_lhs <= {DATA_WIDTH+1{1'b0}};
	op_predicate_reg <= {DATA_WIDTH+1{1'b0}};
end
else
if (start_exec) 
begin
	if (control_reg_data[14:12]!=3'b111)
`ifdef HETERO_NETWORK
		op_lhs <= o__flit_out_wire_local[4][DATA_WIDTH:0];
`else
		op_rhs <= o__flit_out_wire[4][DATA_WIDTH:0];
`endif
	else if (operation[4:0] != 5'b00000)
		op_rhs <= {DATA_WIDTH+1{1'b0}};
	if (control_reg_data[17:15]!=3'b111)
`ifdef HETERO_NETWORK
		op_lhs <= o__flit_out_wire_local[5][DATA_WIDTH:0];
`else
		op_lhs <= o__flit_out_wire[5][DATA_WIDTH:0];
`endif
	else if (operation[4:0] != 5'b00000)
		op_lhs <= {DATA_WIDTH+1{1'b0}};
	if (control_reg_data[20:18]!=3'b111)
`ifdef HETERO_NETWORK
		op_predicate_reg <= o__flit_out_wire_local[6][DATA_WIDTH:0];
`else
		op_predicate_reg <= o__flit_out_wire[6][DATA_WIDTH:0];
`endif
	else if (operation[4:0] != 5'b00000)
		op_predicate_reg <= {DATA_WIDTH+1{1'b0}};
end
end

assign op_predicate = (control_reg_data[CM_WIDTH-1]) ? {op_predicate_reg[DATA_WIDTH:1], ~op_predicate_reg[0]} : op_predicate_reg[DATA_WIDTH:0];

assign op_shift = {1'b1,{5{1'b0}},control_reg_data[61:35]};
assign operation = {control_reg_data[CM_WIDTH-2],control_reg_data[34:30]};
assign regbypass = control_reg_data[24:21];
assign regWEN = control_reg_data[29:26];
assign not_to_execute_all = ((prev_cycle_p_i2_i1[2:0] != 3'b111 && op_rhs[DATA_WIDTH]==1'b0) || (control_reg_data[CM_WIDTH-2] != 1'b1 && prev_cycle_p_i2_i1[5:3] != 3'b111 && op_lhs[DATA_WIDTH]==1'b0) || (prev_cycle_p_i2_i1[8:6] != 3'b111 && (op_predicate[DATA_WIDTH]==1'b0 ||op_predicate[DATA_WIDTH-1:0]=={DATA_WIDTH{1'b0}}) )) ? 1'b1 : 1'b0;
assign not_to_execute_select = ((op_rhs[DATA_WIDTH]==1'b0 && op_lhs[DATA_WIDTH]==1'b0) || (prev_cycle_p_i2_i1[8:6] != 3'b111 && (op_predicate[DATA_WIDTH]==1'b0 || op_predicate[DATA_WIDTH-1:0]=={DATA_WIDTH{1'b0}}))) ? 1'b1 : 1'b0;
assign not_to_execute = (control_reg_data[34:30]==5'b10000) ? not_to_execute_select :  not_to_execute_all;	

assign o__flit_out[0] = o__flit_out_wire[0];
assign o__flit_out[1] = o__flit_out_wire[1];
assign o__flit_out[2] = o__flit_out_wire[2];
assign o__flit_out[3] = o__flit_out_wire[3];


simple_alu 
	#(
	.DATA_WIDTH		(DATA_WIDTH)
	)
	a25_simple_alu(
	.op_predicate	(op_predicate[DATA_WIDTH-1:0]), 
	.op_LHS		(op_lhs), 
	.op_RHS		(op_rhs), 
	.op_SHIFT	(op_shift), 
	.operation	(operation), 
	.result		(alu_out)
);


always @(posedge clk)
begin
if (reset)
begin	
	not_to_execute_reg <= 1'b0;
	alu_out_dm_tiles <= {DATA_WIDTH{1'b0}};
end
else if (start_exec)
begin
	not_to_execute_reg <= not_to_execute;
	alu_out_dm_tiles <= alu_out;
end
end

assign tile_out [DATA_WIDTH-1:0] = (is_dm_tile) ? lsu_alu_out : alu_out;
assign tile_out [DATA_WIDTH] = (is_dm_tile) ? ~not_to_execute_reg : ~not_to_execute;

always @(posedge clk)
begin
if (reset)
begin
		current_loop <= {CM_DEPTH_BITS{1'b0}};
		current_loop_prev_cycle <= 1'b0;
end
else if (start_exec)
begin
	if(operation[4:0]==5'b10110 || operation[4:0]==5'b10111)
	begin
		current_loop <= tile_out[CM_DEPTH_BITS-1:0];
		current_loop_prev_cycle <= 1'b1;
	end
	else 
	begin
		current_loop_prev_cycle <= 1'b0;
	end
end
end

always_comb begin
case (current_loop)
look_up_table[4:1] : begin
	if (look_up_table[0]==1'b1) begin
		ls_lut <= look_up_table[8:5];
		le_lut <= look_up_table[12:9];
	end
	else begin
		ls_lut <= {CM_DEPTH_BITS{1'b0}};
		le_lut <= {CM_DEPTH_BITS{1'b0}};
	end
end
look_up_table[17:14] : begin
	if (look_up_table[13]==1'b1) begin
		ls_lut <= look_up_table[21:18];
		le_lut <= look_up_table[25:22];
	end
	else begin
		ls_lut <= look_up_table[8:5];
		le_lut <= look_up_table[12:9];
	end
end
look_up_table[30:27] : begin
	if (look_up_table[26]==1'b1) begin
		ls_lut <= look_up_table[34:31];
		le_lut <= look_up_table[38:35];
	end
	else begin
		ls_lut <= look_up_table[8:5];
		le_lut <= look_up_table[12:9];
	end
end
look_up_table[43:40] : begin
	if (look_up_table[39]==1'b1) begin
		ls_lut <= look_up_table[47:44];
		le_lut <= look_up_table[51:48];
	end
	else begin
		ls_lut <= look_up_table[8:5];
		le_lut <= look_up_table[12:9];
	end
end
default : begin
		ls_lut <= look_up_table[8:5];
		le_lut <= look_up_table[12:9];
end
endcase
end
//
always @(posedge clk)
begin
if (reset)
	loop_start <= {CM_DEPTH_BITS{1'b0}};
else if (start_exec)
begin
if (operation[4:0]==5'b11110 || current_loop_prev_cycle==1'b1)
begin
	if (operation[4:0]==5'b11110)
	loop_start <= control_reg_data[38:35];
	else
	loop_start <= ls_lut;
end
end
end

always @(posedge clk)
begin
if (reset)
	loop_end <= {CM_DEPTH_BITS{1'b0}};
else if (start_exec)
begin
if (operation[4:0]==5'b11110 || current_loop_prev_cycle==1'b1)
begin
	if (operation[4:0]==5'b11110)
	loop_end <= control_reg_data[42:39];
	else
	loop_end <= le_lut;
end
end
end

always @(posedge clk)   //TREG written or not
begin
if (reset)
	treg <= {DATA_WIDTH+1{1'b0}};
else if (start_exec)
begin
	if (control_reg_data[25]==1'b1) 
	begin
		treg <= tile_out;
	end
end
end



wire [5:0] ldst;
assign ldst = operation; 

reg [5:0] ldst_prev_cycle;;
reg [1:0] op_shift_prev_cycle;
reg [1:0] op_lhs_prev_cycle;

assign lsu_alu_out = (ldst_prev_cycle[4:0] == 5'b11000 || ldst_prev_cycle[4:0] == 5'b11001 || ldst_prev_cycle[4:0] == 5'b11010) ? data_out_dm_reg : alu_out_dm_tiles[DATA_WIDTH-1:0];
assign data_in_ldst = (ldst[4:0] == 5'b11011 || ldst[4:0] == 5'b11100 || ldst[4:0] == 5'b11101) ? op_rhs : {DATA_WIDTH{1'b0}};
assign ldst_addr_in = (control_reg_data[61]) ? op_shift[DM_DEPTH_BITS-1:0] : op_lhs[DM_DEPTH_BITS-1];

always @(posedge clk)
begin
if (reset)
begin
	ldst_prev_cycle <= 6'b000000;
	op_shift_prev_cycle <= 2'b00;
	op_lhs_prev_cycle <= 2'b00;
end
else
begin
	ldst_prev_cycle <= ldst;
	op_shift_prev_cycle <= op_shift[1:0];
	op_lhs_prev_cycle <= op_lhs[1:0];
end
end

reg [5:0] xbar_input0;
reg [5:0] xbar_input1;
reg [5:0] xbar_input2;
reg [5:0] xbar_input3;
reg [5:0] xbar_input4;
reg [5:0] xbar_input5;
reg [5:0] xbar_input6;


always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[2:0])
3'b000: begin
		xbar_input0 <= 6'b000001;
		i__sram_xbar_sel[0] <= 6'b000001;
	end
3'b001: begin
		xbar_input0 <= 6'b000010;
		i__sram_xbar_sel[0] <= 6'b000010;
	end
3'b010: begin
		xbar_input0 <= 6'b000100;
		i__sram_xbar_sel[0] <= 6'b000100;
	end
3'b011: begin
		xbar_input0 <= 6'b001000;
		i__sram_xbar_sel[0] <= 6'b001000;
	end
3'b100: begin
		xbar_input0 <= 6'b010000;
		i__sram_xbar_sel[0] <= 6'b010000;
	end
3'b101: begin
		xbar_input0 <= 6'b100000;
		i__sram_xbar_sel[0] <= 6'b100000;
	end
default: begin
		xbar_input0 <= 6'b000000;
		i__sram_xbar_sel[0] <= 6'b000000;
	end
endcase
end

always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[5:3])
3'b000: begin   
                xbar_input1 <= 6'b000001;
                i__sram_xbar_sel[1] <= 6'b000001;
        end     
3'b001: begin   
                xbar_input1 <= 6'b000010;
                i__sram_xbar_sel[1] <= 6'b000010;
        end     
3'b010: begin   
                xbar_input1 <= 6'b000100;
                i__sram_xbar_sel[1] <= 6'b000100;
        end     
3'b011: begin
                xbar_input1 <= 6'b001000;
                i__sram_xbar_sel[1] <= 6'b001000;
        end     
3'b100: begin   
                xbar_input1 <= 6'b010000;
                i__sram_xbar_sel[1] <= 6'b010000;
        end     
3'b101: begin   
                xbar_input1 <= 6'b100000;
                i__sram_xbar_sel[1] <= 6'b100000;
        end     
default: begin  
                xbar_input1 <= 6'b000000;
                i__sram_xbar_sel[1] <= 6'b000000;
        end
endcase
end

always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[8:6])
3'b000: begin   
                xbar_input2 <= 6'b000001;
                i__sram_xbar_sel[2] <= 6'b000001;
        end     
3'b001: begin   
                xbar_input2 <= 6'b000010;
                i__sram_xbar_sel[2] <= 6'b000010;
        end     
3'b010: begin   
                xbar_input2 <= 6'b000100;
                i__sram_xbar_sel[2] <= 6'b000100;
        end     
3'b011: begin
                xbar_input2 <= 6'b001000;
                i__sram_xbar_sel[2] <= 6'b001000;
        end     
3'b100: begin   
                xbar_input2 <= 6'b010000;
                i__sram_xbar_sel[2] <= 6'b010000;
        end     
3'b101: begin   
                xbar_input2 <= 6'b100000;
                i__sram_xbar_sel[2] <= 6'b100000;
        end     
default: begin  
                xbar_input2 <= 6'b000000;
                i__sram_xbar_sel[2] <= 6'b000000;
        end
endcase
end

always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[11:9])
3'b000: begin   
                xbar_input3 <= 6'b000001;
                i__sram_xbar_sel[3] <= 6'b000001;
        end     
3'b001: begin   
                xbar_input3 <= 6'b000010;
                i__sram_xbar_sel[3] <= 6'b000010;
        end     
3'b010: begin   
                xbar_input3 <= 6'b000100;
                i__sram_xbar_sel[3] <= 6'b000100;
        end     
3'b011: begin
                xbar_input3 <= 6'b001000;
                i__sram_xbar_sel[3] <= 6'b001000;
        end     
3'b100: begin   
                xbar_input3 <= 6'b010000;
                i__sram_xbar_sel[3] <= 6'b010000;
        end     
3'b101: begin   
                xbar_input3 <= 6'b100000;
                i__sram_xbar_sel[3] <= 6'b100000;
        end     
default: begin  
                xbar_input3 <= 6'b000000;
                i__sram_xbar_sel[3] <= 6'b000000;
        end
endcase
end

always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[14:12])
3'b000: begin   
                xbar_input4 <= 6'b000001;
                i__sram_xbar_sel[4] <= 6'b000001;
        end     
3'b001: begin   
                xbar_input4 <= 6'b000010;
                i__sram_xbar_sel[4] <= 6'b000010;
        end     
3'b010: begin   
                xbar_input4 <= 6'b000100;
                i__sram_xbar_sel[4] <= 6'b000100;
        end     
3'b011: begin
                xbar_input4 <= 6'b001000;
                i__sram_xbar_sel[4] <= 6'b001000;
        end     
3'b100: begin   
                xbar_input4 <= 6'b010000;
                i__sram_xbar_sel[4] <= 6'b010000;
        end     
3'b101: begin   
                xbar_input4 <= 6'b100000;
                i__sram_xbar_sel[4] <= 6'b100000;
        end     
default: begin  
                xbar_input4 <= 6'b000000;
                i__sram_xbar_sel[4] <= 6'b000000;
        end
endcase
end

always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[17:15])
3'b000: begin   
                xbar_input5 <= 6'b000001;
                i__sram_xbar_sel[5] <= 6'b000001;
        end     
3'b001: begin   
                xbar_input5 <= 6'b000010;
                i__sram_xbar_sel[5] <= 6'b000010;
        end     
3'b010: begin   
                xbar_input5 <= 6'b000100;
                i__sram_xbar_sel[5] <= 6'b000100;
        end     
3'b011: begin
                xbar_input5 <= 6'b001000;
                i__sram_xbar_sel[5] <= 6'b001000;
        end     
3'b100: begin   
                xbar_input5 <= 6'b010000;
                i__sram_xbar_sel[5] <= 6'b010000;
        end     
3'b101: begin   
                xbar_input5 <= 6'b100000;
                i__sram_xbar_sel[5] <= 6'b100000;
        end     
default: begin  
                xbar_input5 <= 6'b000000;
                i__sram_xbar_sel[5] <= 6'b000000;
        end
endcase
end

always @ (control_reg_data[20:18] or control_reg_data[17:15] or control_reg_data[14:12] or control_reg_data[11:9] or control_reg_data[8:6] or control_reg_data[5:3] or control_reg_data[2:0])
begin
case (control_reg_data[20:18])
3'b000: begin   
                xbar_input6 <= 6'b000001;
                i__sram_xbar_sel[6] <= 6'b000001;
        end     
3'b001: begin   
                xbar_input6 <= 6'b000010;
                i__sram_xbar_sel[6] <= 6'b000010;
        end     
3'b010: begin   
                xbar_input6 <= 6'b000100;
                i__sram_xbar_sel[6] <= 6'b000100;
        end     
3'b011: begin
                xbar_input6 <= 6'b001000;
                i__sram_xbar_sel[6] <= 6'b001000;
        end     
3'b100: begin   
                xbar_input6 <= 6'b010000;
                i__sram_xbar_sel[6] <= 6'b010000;
        end     
3'b101: begin   
                xbar_input6 <= 6'b100000;
                i__sram_xbar_sel[6] <= 6'b100000;
        end     
default: begin  
                xbar_input6 <= 6'b000000;
                i__sram_xbar_sel[6] <= 6'b000000;
        end
endcase
end


IN22FDX_R1PV_NFVG_W00032B064M02C256 control_mem(
.CLK(clk),
.CEN(control_mem_en_t),
.RDWEN(control_mem_wr_en),
.DEEPSLEEP(1'b0),
.POWERGATE(1'b0),
.AW(control_mem_addr[3:0]),
.AC(control_mem_addr[4]),
.D(control_mem_data_in),
.BW(control_mem_bit_en),
.T_LOGIC(1'b0),
.MA_SAWL(1'b0),
.MA_WL(1'b0),
.MA_WRAS(1'b0),
.MA_WRASD(1'b0),
.Q(control_reg_data),
.OBSV_CTL()
);

load_store
        #(
                .DATA_WIDTH             (DATA_WIDTH)
        )
        ldst_unit(
        .clk            (clk),
        .reset          (reset),
        .not_to_execute (not_to_execute_all),
        .start_exec     (start_exec),
        .data_in_ldst   (data_in_ldst),
        .data_out_dm    (data_out_dm),
        .op_addr        (ldst_addr_in),
        .operation      (operation),
        .data_out_ldst  (data_out_ldst),
        .data_in_dm     (data_in_dm),
        .addr_dm        (addr_dm),
        .rd_en_dm       (rd_en_dm),
        .wr_en_dm       (wr_en_dm),
        .bit_en         (bit_en)
);

router
    #(
        .NUM_VCS                        (ROUTER_NUM_VCS),
	.DATA_WIDTH			(DATA_WIDTH)
        `ifdef FLEXIBLE_FLIT_ENABLE
        ,
        .NUM_SSRS                       (NUM_SSRS)
        `endif
    )
    router(
        .clk                            (clk),
        .reset                          (r__reset__pff),
        .i__sram_xbar_sel               (i__sram_xbar_sel),
        .i__flit_in                     (i__flit_in),
        .i__alu_out                     (tile_out),
        .i__treg                        (treg),
        .o__flit_out                    (o__flit_out_wire),
        .regbypass			(regbypass),
        .regWEN				(regWEN)
    );
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Reset synchronizor
//------------------------------------------------------------------------------
always_ff @ (posedge clk)
begin
    r__reset__pff <= reset;
end
//------------------------------------------------------------------------------



endmodule
