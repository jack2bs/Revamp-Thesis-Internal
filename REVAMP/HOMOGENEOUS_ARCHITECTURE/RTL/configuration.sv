module configuration(
	clk,
	reset,
	start_exec,
	operation,
	control_mem_addr,
	control_mem_data_in,
	control_mem_bit_en,
	control_mem_wr_en,
	control_reg_data_out,
	control_mem_en
);


parameter ENABLE		= 32; 
parameter ROUTER_CONFIG 	= 8;
parameter OPCODE_CONFIG 	= 4;
parameter LOCAL_CONFIG 		= 4;
parameter CONST_CONFIG 		= 4;
parameter ENABLE_BITS		= $clog2(ENABLE);
parameter ROUTER_CONFIG_BITS    = $clog2(ROUTER_CONFIG);
parameter OPCODE_CONFIG_BITS    = $clog2(OPCODE_CONFIG);
parameter LOCAL_CONFIG_BITS   	= $clog2(LOCAL_CONFIG);
parameter CONST_CONFIG_BITS    	= $clog2(CONST_CONFIG);


input logic 				clk;
input logic 				reset;
input logic 				start_exec;
input logic [5:0]				operation;
input logic                             control_mem_wr_en;
input logic                             control_mem_en;
output logic  [63:0]            		control_reg_data_out;


`ifdef HETERO_CONFIG
input logic  [ENABLE_BITS-1:0]        control_mem_addr;
input logic  [67:0]             control_mem_data_in;
input logic  [67:0]             control_mem_bit_en;

logic [ROUTER_CONFIG_BITS-1:0] end_router;
logic [OPCODE_CONFIG_BITS-1:0] end_opcode;
logic [LOCAL_CONFIG_BITS-1:0] end_local;
logic [CONST_CONFIG_BITS-1:0] end_const;

logic [63:0]            		control_reg_data;


always @(posedge clk)
begin
if (reset)
begin
        end_router <= {ROUTER_CONFIG_BITS{1'b0}};
	end_opcode <= {OPCODE_CONFIG_BITS{1'b0}};
	end_local <= {LOCAL_CONFIG_BITS{1'b0}};
	end_const <= {CONST_CONFIG_BITS{1'b0}};
end
else if (start_exec)
begin
if (operation[4:0]==5'b11110)
begin
        end_router <= control_reg_data[42+ROUTER_CONFIG_BITS-1:42];
	end_opcode <= control_reg_data[42+ROUTER_CONFIG_BITS+OPCODE_CONFIG_BITS-1:42+ROUTER_CONFIG_BITS];
	end_local <= control_reg_data[42+ROUTER_CONFIG_BITS+OPCODE_CONFIG_BITS+LOCAL_CONFIG_BITS-1:42+ROUTER_CONFIG_BITS+OPCODE_CONFIG_BITS];
	end_const <= control_reg_data[42+ROUTER_CONFIG_BITS+OPCODE_CONFIG_BITS+LOCAL_CONFIG_BITS+CONST_CONFIG_BITS-1:42+ROUTER_CONFIG_BITS+OPCODE_CONFIG_BITS+LOCAL_CONFIG_BITS];
end
end
end


logic [ROUTER_CONFIG_BITS-1:0] addr_router;
logic [OPCODE_CONFIG_BITS-1:0] addr_opcode;
logic [LOCAL_CONFIG_BITS-1:0] addr_local;
logic [CONST_CONFIG_BITS-1:0] addr_const;

logic clk_gated_router;
logic clk_gated_opcode;
logic clk_gated_local;
logic clk_gated_const;

logic [9:0] control_reg_en;
logic [20:0] control_out_router;
logic [4:0] control_out_opcode;
logic [26:0] control_out_const;
logic [8:0] control_out_local;

logic control_mem_wr_en_router;
logic control_mem_wr_en_opcode;
logic control_mem_wr_en_local;
logic control_mem_wr_en_const;


always @(posedge clk_gated_router)
begin
	if(reset)
		addr_router ={ROUTER_CONFIG_BITS{1'b0}};
	else
	begin
		if(addr_router == end_router)
		begin
			addr_router = {ROUTER_CONFIG_BITS{1'b0}}; 
		end
		else
		begin
			addr_router = addr_router +1;
		end
	end
end

always @(posedge clk_gated_opcode)
begin
        if(reset)
                addr_opcode ={OPCODE_CONFIG_BITS{1'b0}};
        else
        begin
                if(addr_opcode == end_opcode)
                begin
                        addr_opcode = {OPCODE_CONFIG_BITS{1'b0}};
                end
                else
                begin
                        addr_opcode = addr_opcode +1;
                end
        end
end

always @(posedge clk_gated_const)
begin
        if(reset)
                addr_const = {LOCAL_CONFIG_BITS{1'b0}};
        else
        begin
                if(addr_const == end_const)
                begin
                        addr_const = {LOCAL_CONFIG_BITS{1'b0}};
                end
                else
                begin
                        addr_const = addr_const +1;
                end
        end
end

always @(posedge clk_gated_local)
begin
        if(reset)
                addr_local = {CONST_CONFIG_BITS{1'b0}};
        else
        begin
                if(addr_local == end_local)
                begin
                        addr_local = {CONST_CONFIG_BITS{1'b0}};
                end
                else
                begin
                        addr_local = addr_local +1;
                end
        end
end

assign control_reg_data_out = control_reg_data;

assign control_reg_data[20:0] = (control_reg_en[1])?{21{1'b1}}:control_out_router;
assign control_reg_data[34:30] = (control_reg_en[3])?{5{1'b0}}:control_out_opcode;
assign control_reg_data[61:35] = control_out_const;
assign control_reg_data[28:21] = control_out_local;
assign control_reg_data[29] = control_reg_en[8];
assign control_reg_data[62] = control_reg_en[5];
assign control_reg_data[63] = control_reg_en[9];

assign control_mem_wr_en_router = (start_exec)? (control_mem_wr_en && control_reg_en[0]):(control_mem_wr_en && control_mem_data_in[58]);
assign control_mem_wr_en_opcode = (start_exec)? (control_mem_wr_en && control_reg_en[2]):(control_mem_wr_en && control_mem_data_in[60]);
assign control_mem_wr_en_const  = (start_exec)? (control_mem_wr_en && control_reg_en[4]):(control_mem_wr_en && control_mem_data_in[62]);
assign control_mem_wr_en_local  = (start_exec)? (control_mem_wr_en && control_reg_en[7]):(control_mem_wr_en && control_mem_data_in[65]);

assign clk_gated_router = (start_exec)? (clk && control_reg_en[0]):(clk && control_mem_data_in[58]);
assign clk_gated_opcode = (start_exec)? (clk && control_reg_en[2]):(clk && control_mem_data_in[60]);
assign clk_gated_const  = (start_exec)? (clk && control_reg_en[4]):(clk && control_mem_data_in[62]);
assign clk_gated_local  = (start_exec)? (clk && control_reg_en[7]):(clk && control_mem_data_in[65]);



	reg_file #(.WIDTH(21), .DEPTH(ROUTER_CONFIG), .DEPTH_BITS(ROUTER_CONFIG_BITS))
 	control_router
	(
    		.clk(clk_gated_router),
    		.wr_addr(addr_router),
    		.wr_data(control_mem_data_in[20:0]),
    		.wr_bit_en(control_mem_bit_en[20:0]),
    		.wr_en(control_mem_wr_en_router),
    		.rd_addr(addr_router),
    		.rd_data(control_out_router),
    		.rd_en(control_mem_en & ~control_mem_wr_en_router)
	);

	reg_file #(.WIDTH(5), .DEPTH(OPCODE_CONFIG), .DEPTH_BITS(OPCODE_CONFIG_BITS))
 	control_opcode
	(
    		.clk(clk_gated_opcode),
    		.wr_addr(addr_opcode),
    		.wr_data(control_mem_data_in[34:30]),
    		.wr_bit_en(control_mem_bit_en[34:30]),
    		.wr_en(control_mem_wr_en_opcode),
    		.rd_addr(addr_opcode),
    		.rd_data(control_out_opcode),
    		.rd_en(control_mem_en & ~control_mem_wr_en_opcode)
	);

	reg_file #(.WIDTH(10), .DEPTH(ENABLE), .DEPTH_BITS(ENABLE_BITS))
 	control_en
	(
    		.clk(clk),
    		.wr_addr(control_mem_addr),
    		.wr_data(control_mem_data_in[67:58]),
    		.wr_bit_en(control_mem_bit_en[67:58]),
    		.wr_en(control_mem_wr_en),
    		.rd_addr(control_mem_addr),
    		.rd_data(control_reg_en),
    		.rd_en(control_mem_en & ~control_mem_wr_en)
	);

	reg_file #(.WIDTH(9), .DEPTH(LOCAL_CONFIG), .DEPTH_BITS(LOCAL_CONFIG_BITS))
 	control_local
	(
    		.clk(clk_gated_local),
    		.wr_addr(addr_local),
    		.wr_data(control_mem_data_in[20:12]),
    		.wr_bit_en(control_mem_bit_en[20:12]),
    		.wr_en(control_mem_wr_en_local),
    		.rd_addr(addr_local),
    		.rd_data(control_out_local),
    		.rd_en(control_mem_en & ~control_mem_wr_en_local)
	);

	reg_file #(.WIDTH(27), .DEPTH(CONST_CONFIG), .DEPTH_BITS(CONST_CONFIG_BITS))
 	control_const
	(
    		.clk(clk_gated_const),
    		.wr_addr(addr_const),
    		.wr_data(control_mem_data_in[61:35]),
    		.wr_bit_en(control_mem_bit_en[61:35]),
    		.wr_en(control_mem_wr_en_const),
    		.rd_addr(addr_const),
    		.rd_data(control_out_const),
    		.rd_en(control_mem_en & ~control_mem_wr_en_const)
	);

`else
input logic  [4:0]        control_mem_addr;
input logic  [63:0]             control_mem_data_in;
input logic  [63:0]             control_mem_bit_en;


	reg_file #(.WIDTH(64), .DEPTH(32), .DEPTH_BITS(5))
	 control_mem
	(
   		.clk(clk),
   		.wr_addr(control_mem_addr),
   		.wr_data(control_mem_data_in),
   		.wr_bit_en(control_mem_bit_en),
    		.wr_en(control_mem_wr_en),
    		.rd_addr(control_mem_addr),
    		.rd_data(control_reg_data_out),
    		.rd_en(control_mem_en & ~control_mem_wr_en)
);
`endif

endmodule
