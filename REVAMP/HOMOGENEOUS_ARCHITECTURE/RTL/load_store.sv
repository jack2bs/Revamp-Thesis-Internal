module load_store(
	clk,
	reset,
	not_to_execute,
	start_exec,
	data_in_ldst,
	data_out_dm,
	op_addr,
	operation,
	data_out_ldst,
	data_in_dm,
	addr_dm,
	rd_en_dm,
	wr_en_dm,
	bit_en	
);

parameter DATA_WIDTH = 32;
localparam DM_DEPTH_BITS = $clog2(512);


input logic clk;
input logic reset;
input logic not_to_execute;
input logic start_exec;
input [DATA_WIDTH-1:0] data_in_ldst;
input [DATA_WIDTH-1:0] data_out_dm;
input logic [DM_DEPTH_BITS:0] op_addr;
input [5:0] operation;
output logic [DATA_WIDTH-1:0] data_out_ldst;
output logic [DATA_WIDTH-1:0] data_in_dm;
output logic [DM_DEPTH_BITS-1:0] addr_dm;
output logic rd_en_dm;
output logic wr_en_dm;
output logic [DATA_WIDTH-1:0] bit_en;

logic data_out_dm_reg;

always @(posedge clk)
begin
if (reset) 
begin
                data_out_dm_reg <= {DATA_WIDTH{1'b0}};
end
else if (start_exec)
begin
casex(operation)
6'b111000:
        begin
		data_out_dm_reg <= data_out_dm;
        end
6'b011000:
        begin
		data_out_dm_reg <= data_out_dm;
        end
6'b110110:
        begin
		data_out_dm_reg <= data_out_dm;
        end
6'b010110:
        begin
		data_out_dm_reg <= data_out_dm;
        end
6'b111001:
        begin
		data_out_dm_reg <= data_out_dm;
        end
6'b011001:
        begin
		data_out_dm_reg <= data_out_dm;

        end
6'b111010:
        begin
      	        if (op_addr[0]== 1'b0)
                begin
                	data_out_dm_reg <= {{DATA_WIDTH-8{1'b0}},data_out_dm[7:0]};
                end
		else
		begin
			data_out_dm_reg <= {{DATA_WIDTH-8{1'b0}},data_out_dm[15:8]};
		end
      end
6'b011010:
        begin
         	if (op_addr[0]== 1'b0)
                begin

                        data_out_dm_reg <= {{DATA_WIDTH-8{1'b0}},data_out_dm[7:0]};
                end
                else 
                begin
                        data_out_dm_reg <= {{DATA_WIDTH-8{1'b0}},data_out_dm[15:8]};
                end
        end
default:
	begin
		data_out_dm_reg <= {DATA_WIDTH{1'b0}};
	end
endcase
end
end


always_comb
begin
if (~not_to_execute) begin

casex(operation)
6'b111000:
        begin
		//$display("LOAD CONST INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b011000:
        begin
		//$display("LOAD INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b110110:
        begin
		//$display("LOADCL CONST INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b010110:
        begin
		//$display("LOADCL INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
                addr_dm <= op_addr[DM_DEPTH_BITS:1];
		data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b111001: // Do we still need this instruction?
        begin
		//$display("LOAD CONST 2bytes INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
                addr_dm <= op_addr[DM_DEPTH_BITS:1];
		data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b011001: //Do we still need this instruction?
        begin
		//$display("LOAD 2bytes INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
                addr_dm <= op_addr[DM_DEPTH_BITS:1];
		data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b111010:
        begin
		//$display("LOAD CONST 1byte INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b011010:
        begin
		//$display("LOAD 1byte INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b1;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b1}};
        end
6'b111011:
        begin
		//$display("STORE CONST INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b0;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= data_in_ldst;
		bit_en <= {DATA_WIDTH{1'b1}};
	end
6'b011011:
        begin
		//$display("STORE INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b0;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
                data_in_dm <= data_in_ldst;
		bit_en <= {DATA_WIDTH{1'b1}};
	end
6'b111100: // DO we still need this instruction
        begin
		//$display("STORE 2byte CONST INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b0;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
		data_in_dm <= data_in_ldst;
		bit_en <= {DATA_WIDTH{1'b1}};
	end
6'b011100: //Do we still need this instruction
        begin
		//$display("STORE 2byte INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b0;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
		data_in_dm <= data_in_ldst;
		bit_en <= {DATA_WIDTH{1'b1}};
	end
6'b111101:
        begin
		//$display("STORE 1byte CONST INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b0;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
		if (op_addr[0] == 1'b0) begin
                      data_in_dm <= {{DATA_WIDTH-8{1'b0}},data_in_ldst[7:0]};
                      bit_en <= {{DATA_WIDTH-8{1'b0}},{8{1'b1}}};
                end
		//CHANGE: if bit size changes
                else if (op_addr[0] == 1'b1) begin
                        data_in_dm <= {data_in_ldst[7:0],{8{1'b0}}};
                        bit_en <= {{8{1'b1}},{8{1'b0}}};
                end
	end
6'b011101:
        begin
		//$display("STORE 1byte INST");
                rd_en_dm <= 1'b0;
		wr_en_dm <= 1'b0;
		addr_dm <= op_addr[DM_DEPTH_BITS:1];
		if (op_addr[0] == 1'b0) begin
                      data_in_dm <= {{DATA_WIDTH-8{1'b0}},data_in_ldst[7:0]};
                      bit_en <= {{DATA_WIDTH-8{1'b0}},{8{1'b1}}};
                end
                //CHANGE: if bit size changes
                else if (op_addr[0] == 1'b1) begin
                        data_in_dm <= {data_in_ldst[7:0],{8{1'b0}}};
                        bit_en <= {{8{1'b1}},{8{1'b0}}};
                end
	end
default:
	begin
		//$display("NOT LOAD OR STORE INST");
                rd_en_dm <= 1'b1;
                wr_en_dm <= 1'b1;
                addr_dm <= {DM_DEPTH_BITS{1'b0}} ;
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b0}};
	end
endcase
end
else begin
		rd_en_dm <= 1'b1;	
                wr_en_dm <= 1'b1;
		addr_dm <= {DM_DEPTH_BITS{1'b0}} ;
                data_in_dm <= {DATA_WIDTH{1'b0}};
		bit_en <= {DATA_WIDTH{1'b0}};
end
end
endmodule
