module peArray();

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
	#( 
		.ENABLE(ENABLE), 
		.ROUTER_CONFIG(ROUTER_CONFIG), 
		.OPCODE_CONFIG(OPCODE_CONFIG),
		.LOCAL_CONFIG(LOCAL_CONFIG),
		.CONST_CONFIG(CONST_CONFIG),
		.ALU_ADD(ALU_ADD),	
		.ALU_CMP(ALU_CMP),
		.ALU_LOG(ALU_LOG),
		.ALU_MUL(ALU_MUL),
		.ALU_MEM(ALU_MEM))	
            tile(
                .clk                                (clk_shifted),
                .reset                              (r__reset__pff[NUM_RESET_SYNC_STAGES-1]),
                .i__flit_in                         (w__flit_in[0][0]),
                .o__flit_out                        (w__flit_out[0][0]),
		.control_mem_data_in                (cm_data),
                .control_mem_en                     (cm_en[0]),
                .control_mem_bit_en                 (cm_bit_en0),
                .control_mem_addr                   (addr_shifted),
                .control_mem_wr_en                  (wr_en_shifted),
		.is_dm_tile			    (1'b1),
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


 tile
	#( 
		.ENABLE(ENABLE_00), 
		.ROUTER_CONFIG(ROUTER_CONFIG_00), 
		.OPCODE_CONFIG(OPCODE_CONFIG_00),
		.LOCAL_CONFIG(LOCAL_CONFIG_00),
		.CONST_CONFIG(CONST_CONFIG_00),
		.ALU_ADD(ALU_ADD_00),	
		.ALU_CMP(ALU_CMP_00),
		.ALU_LOG(ALU_LOG_00),
		.ALU_MUL(ALU_MUL_00),
		.ALU_MEM(ALU_MEM_00))	
            tile(
                .clk                                (clk),
                .reset                              (reset),
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



endmodule
