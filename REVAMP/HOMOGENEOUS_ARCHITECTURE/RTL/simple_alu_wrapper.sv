module simple_alu(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result);
parameter DATA_WIDTH = 32;
parameter ALU_ADD		= 1;
parameter ALU_CMP		= 1;
parameter ALU_LOG		= 1;
parameter ALU_MUL		= 1;	

input [DATA_WIDTH:0] op_RHS;
input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH:0] op_SHIFT;
input [DATA_WIDTH-1:0] op_predicate;

input [5:0] operation;

output [DATA_WIDTH-1:0] result;

logic [DATA_WIDTH-1:0] result;

wire [DATA_WIDTH:0] op_2;
assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

`ifdef HETERO_COMPUTE
logic [DATA_WIDTH-1:0] result_adder, result_multiplier, result_comparator, result_logic;
always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b00001: result = result_adder; //add
	5'b00010: result = result_adder; //sub
	5'b00011: result = result_multiplier ; //mult
	5'b01000: result = result_logic; // ls
	5'b01001: result = result_logic; // rs
	5'b01010: result = result_logic; // ars
	5'b01011: result = result_logic; //bitwise and
	5'b01100: result = result_logic; //bitwise or
	5'b01101: result = result_logic; //bitwise xor
	5'b10000: result = result_comparator;
	5'b10001: begin
			if (operation[5]==1'b0) 
				result = op_RHS[DATA_WIDTH-1:0]; //cmerge
			else
				result = op_SHIFT[DATA_WIDTH-1:0]; //cmerge
		end	
	5'b10010: result = result_comparator; //cmp
	5'b10011: result = result_comparator; //clt
	5'b10100: result = result_comparator; //br
	5'b10101: result = result_comparator; //cgt
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

if(ALU_ADD==1)
	simple_alu_add adder(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result_adder);
else
	assign  result_adder ={DATA_WIDTH{1'b0}};


if(ALU_MUL==1)
	simple_alu_mul multiplier(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result_multiplier);
else
	assign  result_multiplier ={DATA_WIDTH{1'b0}};


if(ALU_CMP==1)
	simple_alu_cmp comparator(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result_comparator);
else
	assign  result_comparator ={DATA_WIDTH{1'b0}};


if(ALU_LOG==1)
	simple_alu_log log(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result_logic);
else
	assign  result_logic ={DATA_WIDTH{1'b0}};


`else
always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b00001: result = (op_RHS[DATA_WIDTH-1:0] + op_2[DATA_WIDTH-1:0]); //add
	5'b00010: result = (op_RHS[DATA_WIDTH-1:0] - op_2[DATA_WIDTH-1:0]) ; //sub
	5'b00011: result = ($signed(op_RHS[DATA_WIDTH-1:0]) * $signed(op_2[DATA_WIDTH-1:0])) ; //mult
	5'b01000: result = (op_RHS[DATA_WIDTH-1:0] << op_2[DATA_WIDTH-1:0]); // ls
	5'b01001: result = (op_RHS[DATA_WIDTH-1:0] >> op_2[DATA_WIDTH-1:0]); // rs
	5'b01010: result = (op_RHS[DATA_WIDTH-1:0] >>> op_2[DATA_WIDTH-1:0]); // ars
	5'b01011: result = (op_RHS[DATA_WIDTH-1:0] & op_2[DATA_WIDTH-1:0]); //bitwise and
	5'b01100: result = (op_RHS[DATA_WIDTH-1:0] | op_2[DATA_WIDTH-1:0]); //bitwise or
	5'b01101: result = (op_RHS[DATA_WIDTH-1:0] ^ op_2[DATA_WIDTH-1:0]); //bitwise xor
	5'b10000: begin
			if (operation[5]==1'b0) begin
				if (op_LHS[DATA_WIDTH] == 1'b1) 
					result = op_LHS[DATA_WIDTH-1:0]; //select
				else if (op_RHS[DATA_WIDTH] == 1'b1) 
					result = op_RHS[DATA_WIDTH-1:0]; //select
				else 
					result = {DATA_WIDTH{1'b0}};
			end
			else begin
				result = op_SHIFT[DATA_WIDTH-1:0];
			end
	end
	5'b10001: begin
			if (operation[5]==1'b0) 
				result = op_RHS[DATA_WIDTH-1:0]; //cmerge
			else
				result = op_SHIFT[DATA_WIDTH-1:0]; //cmerge
		end	
	5'b10010: result = {{DATA_WIDTH-1{1'b0}},(op_RHS[DATA_WIDTH-1:0] == op_2[DATA_WIDTH-1:0])}; //cmp
	5'b10011: result = {{DATA_WIDTH-1{1'b0}},($signed (op_RHS[DATA_WIDTH-1:0]) < $signed(op_2[DATA_WIDTH-1:0]))}; //clt
	5'b10100: result = (op_predicate[DATA_WIDTH-1:0] | op_RHS[DATA_WIDTH-1:0] | op_2[DATA_WIDTH-1:0]); //br
	5'b10101: result = {{DATA_WIDTH-1{1'b0}},($signed(op_2[DATA_WIDTH-1:0]) < $signed(op_RHS[DATA_WIDTH-1:0]))}; //cgt
	5'b11111: result = op_2[DATA_WIDTH-1:0]; //movc	
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

`endif



endmodule


module simple_alu_add(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result);

parameter DATA_WIDTH = 32;

input [DATA_WIDTH:0] op_RHS;
input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH:0] op_SHIFT;
input [DATA_WIDTH-1:0] op_predicate;

input [5:0] operation;

output [DATA_WIDTH-1:0] result;

reg [DATA_WIDTH-1:0] result;

wire [DATA_WIDTH:0] op_2;
assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b00001: result = (op_RHS[DATA_WIDTH-1:0] + op_2[DATA_WIDTH-1:0]); //add
	5'b00010: result = (op_RHS[DATA_WIDTH-1:0] - op_2[DATA_WIDTH-1:0]) ; //sub
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

endmodule


module simple_alu_mul(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result);

parameter DATA_WIDTH = 32;

input [DATA_WIDTH:0] op_RHS;
input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH:0] op_SHIFT;
input [DATA_WIDTH-1:0] op_predicate;

input [5:0] operation;

output [DATA_WIDTH-1:0] result;

reg [DATA_WIDTH-1:0] result;

wire [DATA_WIDTH:0] op_2;
assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b00011: result = ($signed(op_RHS[DATA_WIDTH-1:0]) * $signed(op_2[DATA_WIDTH-1:0])) ; //mult
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

endmodule

module simple_alu_cmp(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result);

parameter DATA_WIDTH = 32;

input [DATA_WIDTH:0] op_RHS;
input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH:0] op_SHIFT;
input [DATA_WIDTH-1:0] op_predicate;

input [5:0] operation;

output [DATA_WIDTH-1:0] result;

reg [DATA_WIDTH-1:0] result;

wire [DATA_WIDTH:0] op_2;
assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b10000: begin
			if (operation[5]==1'b0) begin
				if (op_LHS[DATA_WIDTH] == 1'b1) 
					result = op_LHS[DATA_WIDTH-1:0]; //select
				else if (op_RHS[DATA_WIDTH] == 1'b1) 
					result = op_RHS[DATA_WIDTH-1:0]; //select
				else 
					result = {DATA_WIDTH{1'b0}};
			end
			else begin
				result = op_SHIFT[DATA_WIDTH-1:0];
			end
	end
	5'b10010: result = {{DATA_WIDTH-1{1'b0}},(op_RHS[DATA_WIDTH-1:0] == op_2[DATA_WIDTH-1:0])}; //cmp
	5'b10011: result = {{DATA_WIDTH-1{1'b0}},($signed (op_RHS[DATA_WIDTH-1:0]) < $signed(op_2[DATA_WIDTH-1:0]))}; //clt
	5'b10100: result = (op_predicate[DATA_WIDTH-1:0] | op_RHS[DATA_WIDTH-1:0] | op_2[DATA_WIDTH-1:0]); //br
	5'b10101: result = {{DATA_WIDTH-1{1'b0}},($signed(op_2[DATA_WIDTH-1:0]) < $signed(op_RHS[DATA_WIDTH-1:0]))}; //cgt
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

endmodule

module simple_alu_log(op_predicate, op_LHS, op_RHS, op_SHIFT, operation, result);

parameter DATA_WIDTH = 32;

input [DATA_WIDTH:0] op_RHS;
input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH:0] op_SHIFT;
input [DATA_WIDTH-1:0] op_predicate;

input [5:0] operation;

output [DATA_WIDTH-1:0] result;

reg [DATA_WIDTH-1:0] result;

wire [DATA_WIDTH:0] op_2;
assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b01000: result = (op_RHS[DATA_WIDTH-1:0] << op_2[DATA_WIDTH-1:0]); // ls
	5'b01001: result = (op_RHS[DATA_WIDTH-1:0] >> op_2[DATA_WIDTH-1:0]); // rs
	5'b01010: result = (op_RHS[DATA_WIDTH-1:0] >>> op_2[DATA_WIDTH-1:0]); // ars
	5'b01011: result = (op_RHS[DATA_WIDTH-1:0] & op_2[DATA_WIDTH-1:0]); //bitwise and
	5'b01100: result = (op_RHS[DATA_WIDTH-1:0] | op_2[DATA_WIDTH-1:0]); //bitwise or
	5'b01101: result = (op_RHS[DATA_WIDTH-1:0] ^ op_2[DATA_WIDTH-1:0]); //bitwise xor
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

endmodule
