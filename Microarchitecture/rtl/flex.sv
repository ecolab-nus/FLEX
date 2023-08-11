module flex(
    clk,
    clkn,
    rstn,
    chip_en,
    data_in,
    address_in,
    data_out,
    read_write,
    data_out_valid,
    data_addr_valid,
    scan_start_exec,
    exec_end,
    trigger_op
);

//-----------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------
import TopPkg::*;
import SMARTPkg::*;

parameter DATA_WIDTH                    = 16;
parameter DM_DEPTH                      = 256;
parameter DM_DEPTH_BITS                 = $clog2(DM_DEPTH);
parameter NUM_DMEM_BANKS		= 4;
parameter DM_WIDTH             		= 16; 
parameter DM_WIDTH_BITS         	= $clog2(DM_WIDTH);

parameter CM_WIDTH_OPERATION		= 24;
parameter CM_WIDTH_ROUTING		= 16;
parameter CM_WIDTH_ROUTING_MEM		= 28;

parameter CM_DEPTH_OPERATION		= 8;
parameter CM_DEPTH_ROUTING		= 8;

parameter CM_WIDTH                      = 64;
parameter CM_DEPTH                      = 8;
parameter CM_DEPTH_BITS                 = $clog2(CM_DEPTH);

parameter ADDR_LEN			= 19;
parameter IN_WIDTH			= 16;

parameter MEM_PORTS_BITS		= $clog2(NUM_DMEM_BANKS*2);
parameter NUM_TILES_X                  = 4;
parameter NUM_TILES_Y                  = 4;

parameter STRIDE_BITS		        = 2;
parameter VEC_WIDTH			= 8;
localparam VEC_WIDTH_BITS		= $clog2(VEC_WIDTH);

parameter LOCAL_REG_DEPTH		= 8;
parameter LOCAL_REG_DEPTH_BITS         	= $clog2(LOCAL_REG_DEPTH);

parameter NOP_BITS			=  2;
parameter TRIGGER_SIZE			= 12;

localparam TILE_NUM_INPUT_PORTS         = 6;
localparam TILE_NUM_OUTPUT_PORTS        = 7;
localparam NUM_CLUSTERS                 = 4;

localparam NO_PES_BITS			= $clog2(NUM_TILES_Y*NUM_TILES_X);
localparam CONFIG_WIDTH_ITER 		= STRIDE_BITS + DATA_WIDTH ;

localparam CM_DEPTH_OPERATION_BITS      = $clog2(CM_DEPTH_OPERATION);
localparam CM_DEPTH_ROUTING_BITS	= $clog2(CM_DEPTH_ROUTING);

//parameter [3:0] ALU_MEM [35:0]= {4'b1100,4'b0000,4'b0000,4'b0000,4'b0000,4'b0110,4'b1011,4'b0000,4'b0000,4'b0000,4'b0000,4'b0101,4'b1010,4'b0000,4'b0000,4'b0000,4'b0000,4'b0100,4'b1001,4'b0000,4'b0000,4'b0000,4'b0000,4'b0011,4'b1000,4'b0000,4'b0000,4'b0000,4'b0000,4'b0010,4'b0111,4'b0000,4'b0000,4'b0000,4'b0000,4'b0001};
parameter [3:0] ALU_MEM [15:0]= {4'b1000,4'b0000,4'b0000,4'b0100,4'b0111,4'b0000,4'b0000,4'b0011,4'b0110,4'b0000,4'b0000,4'b0010,4'b0101,4'b0000,4'b0000,4'b0001};

parameter [3:0] ALU_MEM_PES [7:0]= {4'b1111,4'b1011,4'b0111,4'b0011,4'b1100,4'b1000,4'b0100,4'b0000};


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// IO
//------------------------------------------------------------------------------
input logic                             clk;
input logic                             clkn;

input logic                             rstn;
input logic                             chip_en;

input  logic                            scan_start_exec;

output  logic                           data_out_valid;
input  [1:0]                            data_addr_valid;

input [IN_WIDTH-1:0]                            data_in;
input [ADDR_LEN-1:0]                            address_in;
output logic [DATA_WIDTH-1:0]           data_out;
input  logic                            read_write;
input [NUM_TILES_Y-1:0] trigger_op [NUM_TILES_X-1:0]; 
output logic                            exec_end;

logic start_exec;
logic [1:0] data_addr_valid_reg;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Data Memory IO
//------------------------------------------------------------------------------
logic [15:0] dm_en;
logic [NUM_DMEM_BANKS*2-1:0] chip_en_dm; //cos dual port memories
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Config Memory IO
//------------------------------------------------------------------------------
logic [CM_WIDTH-1:0] cm_data;
logic [CM_WIDTH-1:0] cm_bit_en [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Internal signals
//------------------------------------------------------------------------------
logic [`RT_FLIT_SIZE-1:0] w__flit_in  [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_INPUT_PORTS-1-2:0];
logic [`RT_FLIT_SIZE-1:0] w__flit_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_OUTPUT_PORTS-1-3:0];
logic [`RT_FLIT_SIZE-1:0] w__flit_l_in  [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_INPUT_PORTS-1-2:0];
logic [`RT_FLIT_SIZE-1:0] w__flit_l_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_OUTPUT_PORTS-1-3:0];

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Submodule
//------------------------------------------------------------------------------
//logic rd_en_shifted_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic [DM_DEPTH_BITS-1:0] addr_dm_out [NUM_TILES_Y*NUM_TILES_X-1:0];
logic [DM_WIDTH-1:0] bit_en_out [NUM_TILES_Y*NUM_TILES_X-1:0];
logic [DM_WIDTH-1:0] data_in_dm_out [NUM_TILES_Y*NUM_TILES_X-1:0];
logic rd_en_dm_out [NUM_TILES_Y*NUM_TILES_X-1:0];
logic wr_en_dm_out [NUM_TILES_Y*NUM_TILES_X-1:0];
logic [3:0] addr;

logic [DM_DEPTH_BITS-1:0] addr_dm_shifted [NUM_DMEM_BANKS*2-1:0];
logic [DM_WIDTH-1:0] bit_en_shifted [NUM_DMEM_BANKS*2-1:0];
logic [DM_WIDTH-1:0] data_in_dm_shifted [NUM_DMEM_BANKS*2-1:0];
logic chip_en_dm_shifted [NUM_DMEM_BANKS*2-1:0];
logic wr_en_dm_shifted [NUM_DMEM_BANKS*2-1:0];
logic [DM_WIDTH-1:0] data_out_dm [NUM_DMEM_BANKS*2-1:0];

logic [NUM_CLUSTERS-1:0] cluster_exec_enabled; // active low
logic [NUM_CLUSTERS-1:0] dm_exec_end;
logic start_exec_shifted,start_exec_shifted_shifted;

logic read_write_iteration;
logic [CONFIG_WIDTH_ITER-1:0] iter_config_data; 
logic [DATA_WIDTH:0] iterValue;
logic [DATA_WIDTH:0] iterValue_reg;


logic [TRIGGER_SIZE-1:0] trigger_data_in  [NUM_TILES_Y*NUM_TILES_X-1:0];

logic [VEC_WIDTH_BITS-1:0 ] vec_counter_r;
logic clken_vec_r,clkn_g_r,clk_g_r,clk_g_r1;
logic [CM_DEPTH_ROUTING_BITS:0] addr_shifted_r;


//----------------------------------------------------------------------
//-------------------------CLK GATING-----------------------------------
//----------------------------------------------------------------------
logic [IN_WIDTH-1:0] clken;
logic [IN_WIDTH-1:0] clken_cmem_op_tot;
logic [CM_DEPTH-1:0] clken_cmem_op_trans [IN_WIDTH-1:0];
logic [IN_WIDTH-1:0] clken_cmem_op [CM_DEPTH-1:0];
logic [IN_WIDTH-1:0] clken_cmem_r [CM_DEPTH-1:0];

logic clk_g,clkn_g;
clkgate #(1) clkgate0 (clk, clkn, !start_exec_shifted, rstn, clk_g);
clkgate #(1) clkgate1 (clkn, clk, !start_exec_shifted, rstn, clkn_g);

always_ff @(posedge clk_g or negedge rstn) begin
	if (!rstn) begin
		clken <= {IN_WIDTH{1'b1}};
	end
	else begin
		if(data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1101 && !scan_start_exec) begin
				clken[IN_WIDTH-1:0] <= data_in;
		end
	end
end

always_ff @(posedge clk_g or negedge rstn) begin
	if (!rstn) begin
	for (int jj_cm = 0; jj_cm < CM_DEPTH; jj_cm = jj_cm + 1) begin
		clken_cmem_r[jj_cm] <= {IN_WIDTH{1'b1}};
	end
	end
	else begin
		if(data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1001 && !scan_start_exec) begin
			clken_cmem_r[address_in[2+CM_DEPTH_BITS-1:2]][IN_WIDTH-1:0] <= data_in;
		end
	end
end

always_ff @(posedge clk_g or negedge rstn) begin
	if (!rstn) begin
	for (int jj_cm = 0; jj_cm < CM_DEPTH; jj_cm = jj_cm + 1) begin
		clken_cmem_op[jj_cm] <= {IN_WIDTH{1'b1}};
	end
	end
	else begin
		if(data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b0101 && !scan_start_exec) begin
				clken_cmem_op[address_in[2+CM_DEPTH_BITS-1:2]][IN_WIDTH-1:0] <= data_in;
		end
	end
end

always_comb begin
    for (int ii_cm = 0; ii_cm < CM_DEPTH; ii_cm = ii_cm + 1) begin
        for (int jj_cm = 0; jj_cm < IN_WIDTH; jj_cm = jj_cm + 1) begin
	  if(!rstn)
            clken_cmem_op_trans[jj_cm][ii_cm] = 1'b1;
	  else begin	
	    clken_cmem_op_trans[jj_cm][ii_cm] = clken_cmem_op[ii_cm][jj_cm];
 	  end
        end
    end
end


always_ff @(posedge clk_g or negedge rstn) begin
	if (!rstn) begin
		clken_cmem_op_tot<= {IN_WIDTH{1'b1}};
	end
	else begin
		if(data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b0111 && !scan_start_exec) begin
			clken_cmem_op_tot[IN_WIDTH-1:0] <= data_in;
		end
	end
end

//----------------------------------------------------------------------
//-------------------------OLOAD DATA/ADDR------------------------------
//----------------------------------------------------------------------
logic [DATA_WIDTH-1:0] oload_data [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
logic [LOCAL_REG_DEPTH_BITS-1:0] oload_addr;
logic oload_valid  [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];

always_comb begin
    if(!rstn) begin
        //oload_data = {DATA_WIDTH{1'b0}};
	oload_addr = {LOCAL_REG_DEPTH_BITS{1'b0}};
    end
    else begin
    	if (data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1110 && !scan_start_exec) begin
           oload_addr = address_in[LOCAL_REG_DEPTH_BITS-1:0];
	   //oload_data = data_in[DATA_WIDTH-1:0];
    	end
    end
end

always_comb begin
    for (int ii_cm = 0; ii_cm < NUM_TILES_Y; ii_cm = ii_cm + 1) begin
        for (int jj_cm = 0; jj_cm < NUM_TILES_X; jj_cm = jj_cm + 1) begin
	  if(!rstn)
            oload_data[ii_cm][jj_cm] = {DATA_WIDTH{1'b0}};
	  else begin	
            if (address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1110) begin
                if (address_in[8:3]  == ii_cm * NUM_TILES_X + jj_cm) begin
		    if (data_addr_valid == 2'b11) begin
                        oload_data[ii_cm][jj_cm] = data_in[DATA_WIDTH-1:0];
			oload_valid[ii_cm][jj_cm] = 1'b1;
                    end else begin
                        oload_data[ii_cm][jj_cm] = {DATA_WIDTH{1'b0}};
			oload_valid[ii_cm][jj_cm] = 1'b0;
                    end
                end
		else begin
		        oload_valid[ii_cm][jj_cm] = 1'b0;
			oload_data[ii_cm][jj_cm] = {DATA_WIDTH{1'b0}};
		end
            end
	  end
        end
    end
end

//----------------------------------------------------------------------
//-------------------------VEC SIZE------------------------------
//----------------------------------------------------------------------
logic [VEC_WIDTH_BITS-1:0] vec_size;

always_comb begin
    if(!rstn) begin
	vec_size = {VEC_WIDTH_BITS{1'b0}};
    end
    else begin
    	if (data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b0110 && !scan_start_exec) begin
           vec_size = data_in[VEC_WIDTH_BITS-1:0];
    	end
    end
end

//----------------------------------------------------------------------
//-------------------------TRIGGER DATA   ------------------------------
//----------------------------------------------------------------------
/*always_comb begin
    if(!rstn) begin
    	for (int ii_cm = 0; ii_cm < NUM_TILES_Y; ii_cm = ii_cm + 1) begin
        	for (int jj_cm = 0; jj_cm < NUM_TILES_X; jj_cm = jj_cm + 1) begin
			trigger_data_in [ii_cm*NUM_TILES_X + jj_cm] = {TRIGGER_SIZE{1'b0}};
    		end
	end
    end
    else begin
    	if (data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b0100 && !scan_start_exec) begin
		trigger_data_in [address_in[NO_PES_BITS-1:0]] = 	data_in[TRIGGER_SIZE-1:0];
    	end
   end
end*/

//----------------------------------------------------------------------
//-------------------------LOOP START/LOOP_END--------------------------
//----------------------------------------------------------------------
logic [2*CM_DEPTH_BITS-1:0] loop_var;

logic [CM_DEPTH_BITS-1:0] loop_start;
logic [CM_DEPTH_BITS-1:0] loop_end;

always_ff @(posedge clk_g or negedge rstn) begin
	if (!rstn) begin
		loop_var <= {2*CM_DEPTH_BITS{1'b0}};
	end
	else begin
		if(data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1100 && !scan_start_exec) begin
			loop_var <= data_in[2*CM_DEPTH_BITS-1:0];
		end
	end
end

assign loop_start = loop_var[CM_DEPTH_BITS-1:0];
assign loop_end = loop_var[2*CM_DEPTH_BITS-1:CM_DEPTH_BITS];


always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
       start_exec_shifted <= 1'b0;
    end
    else begin
        if (chip_en == 1'b1) begin
            start_exec_shifted <= scan_start_exec;
        end else begin
            start_exec_shifted <= 1'b0;
        end
    end
end
always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
       start_exec_shifted_shifted <= 1'b0;
    end
    else begin
        if (chip_en == 1'b1) begin
            start_exec_shifted_shifted <= start_exec_shifted;
        end else begin
            start_exec_shifted_shifted <= 1'b0;
        end
    end
end

//----------------------------------------------------------------------
//-------------------------ITERATION COUNTER----------------------------
//----------------------------------------------------------------------

assign read_write_iteration = (data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1111 && !scan_start_exec)? 1'b1:1'b0;

always_comb begin
    if(!rstn)
    	iter_config_data = {CONFIG_WIDTH_ITER{1'b0}};
    else begin
    	if (data_addr_valid == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b1111 && !scan_start_exec) begin
            if (address_in[0] == 'b0) begin
            	iter_config_data = {iter_config_data[CONFIG_WIDTH_ITER-1:DATA_WIDTH],data_in[DATA_WIDTH-1:0]};
            end begin
           	iter_config_data = {data_in[STRIDE_BITS-1:0],iter_config_data[DATA_WIDTH-1:0]};
            end
    	end
    end
end

	iterCounter
	#(
		.CONFIG_MEM_BITS(CM_DEPTH_BITS),
		.ITER_RANGE_BITS(DATA_WIDTH),
		.STRIDE_BITS(STRIDE_BITS),
		.VEC_WIDTH(VEC_WIDTH)
	)
	counter
	(
		.clk(clk),
		.rstn(rstn),
		.start_exec_shifted(start_exec_shifted),
		.start_exec(scan_start_exec),
		.wr_en(read_write_iteration),
		.chip_en(chip_en),
		.loop_end(loop_end),
		.configuration(iter_config_data),
		.iterValue(iterValue),
		.iterValue_reg(iterValue_reg),
		.exec_end(exec_end),
		.vec_size(vec_size),
		.addr_cmem(addr_shifted_r),
		.vec_counter(vec_counter_r),
		.clk_g(clk_g_r1)
	);


always_ff @(posedge clk_g or negedge rstn) begin
    if (!rstn) begin
        data_addr_valid_reg <=2'b00;
    end
    else begin
        if (chip_en) begin
            data_addr_valid_reg <=data_addr_valid;
        end
    end
end


always_comb begin
    if(!rstn)
    	cm_data = {CM_WIDTH{1'b0}};
    else begin
    if (address_in[ADDR_LEN-1:ADDR_LEN-4]=='b0000) begin
        if (data_addr_valid == 2'b00) begin
            cm_data[address_in[2:1]*16 +: 16] = {data_in[15:0]};
        end else if (data_addr_valid == 2'b01) begin
            cm_data[address_in[2:1]*16 +: 16] = {{8{1'b0}},data_in[7:0]};
        end else if (data_addr_valid == 2'b10) begin
            cm_data[address_in[2:1]*16 +: 16] = {data_in[15:8],{8{1'b0}}};
        end else begin
            cm_data[address_in[2:1]*16 +: 16] = {data_in[15:0]};
        end
    end
    end
end

always_comb begin
    for (int ii_cm = 0; ii_cm < NUM_TILES_Y; ii_cm = ii_cm + 1) begin
        for (int jj_cm = 0; jj_cm < NUM_TILES_X; jj_cm = jj_cm + 1) begin
	  if(!rstn)
            cm_bit_en[ii_cm][jj_cm] = {CM_WIDTH{1'b0}};
	  else begin	
            if (address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b0000) begin
                if (address_in[13:8]  == ii_cm * NUM_TILES_X + jj_cm) begin
                    if (data_addr_valid == 2'b00) begin
                        cm_bit_en[ii_cm][jj_cm][address_in[2:1]*16 +: 16] = 16'h0000;
                    end else if (data_addr_valid == 2'b01) begin
                        cm_bit_en[ii_cm][jj_cm][address_in[2:1]*16 +: 16] = 16'h00FF;
                    end else if (data_addr_valid == 2'b10) begin
                        cm_bit_en[ii_cm][jj_cm][address_in[2:1]*16 +: 16] = 16'hFF00;
                    end else begin
                        cm_bit_en[ii_cm][jj_cm][address_in[2:1]*16 +: 16] = 16'hFFFF;
                    end
                end
		else begin
			cm_bit_en[ii_cm][jj_cm] = {CM_WIDTH{1'b0}};
		end
            end
	  end
        end
    end
end

//------------------------------------------------------------------------------
// Data SRAMS
//------------------------------------------------------------------------------
//
genvar l;
generate
	for(l =0 ; l < NUM_DMEM_BANKS ; l++ ) begin
		always_ff @(posedge clk_g or negedge rstn) begin
			if (!rstn) begin
				chip_en_dm[2*l] <= 1'b1;
				chip_en_dm[2*l+1] <= 1'b1;
			end
			else begin
				if(data_addr_valid == 2'b00 || address_in[ADDR_LEN-1:ADDR_LEN-4] != 'b0001) begin
					chip_en_dm[2*l] <= 1'b1;
					chip_en_dm[2*l+1] <= 1'b1;
				end
				else begin
					chip_en_dm[2*l] <= ((2*l) == address_in[ADDR_LEN-5:ADDR_LEN-5-MEM_PORTS_BITS+1])? 1'b0 : 1'b1;	
					chip_en_dm[2*l+1] <= ((2*l+1) == address_in[ADDR_LEN-5:ADDR_LEN-5-MEM_PORTS_BITS+1])? 1'b0 : 1'b1;
				end
			end
		end


			IN22FDX_SDPV_NFVG_W00256B016M08C064 data_mem (
				.CLK_A(clkn),
				.CLK_B(clkn),
				.CEN_A(chip_en_dm_shifted[2*l]),
				.CEN_B(chip_en_dm_shifted[2*l+1]),
				.RDWEN_A(wr_en_dm_shifted[2*l]),
				.RDWEN_B(wr_en_dm_shifted[2*l+1]),
				.DEEPSLEEP(1'b0),
				.POWERGATE(1'b0),
				.AS_A(addr_dm_shifted[2*l][7]),
				.AW_A(addr_dm_shifted[2*l][3:0]),
				.AC_A(addr_dm_shifted[2*l][6:4]),
				.AS_B(addr_dm_shifted[2*l+1][7]),
				.AW_B(addr_dm_shifted[2*l+1][3:0]),
				.AC_B(addr_dm_shifted[2*l+1][6:4]),
				.D_A(data_in_dm_shifted[2*l]),
				.D_B(data_in_dm_shifted[2*l+1]),
				.BW_A(bit_en_shifted[2*l]),
				.BW_B(bit_en_shifted[2*l+1]),
				.T_BIST(1'b0),
				.T_LOGIC(1'b0),
				.T_CEN_A(1'b1),
				.T_CEN_B(1'b1),
				.T_RDWEN_A(1'b0),
				.T_RDWEN_B(1'b0),
				.T_DEEPSLEEP(1'b0),
				.T_POWERGATE(1'b0),
				.T_AS_A('0),
				.T_AS_B('0),
				.T_AW_A('0),
				.T_AW_B('0),
				.T_AC_A('0),
				.T_AC_B('0),
				.T_D_A({16{1'b0}}),
				.T_D_B({16{1'b0}}),
				.T_BW_A({16{1'b0}}),
				.T_BW_B({16{1'b0}}),
				.T_WBT(1'b0),
				.T_STAB(1'b0),
				.MA_SAWL(2'b01),
				.MA_WL(2'b00),
				.MA_WRAS(2'b00),
				.MA_WRASD(1'b0),
				.Q_A(data_out_dm[2*l]),
				.Q_B(data_out_dm[2*l+1]),
				.OBSV_CTL_A(),
				.OBSV_CTL_B()
			);
		/*	IN22FDX_SDPV_NFVG_W00256B032M04C064 data_mem (
				.CLK_A(clkn),
				.CLK_B(clkn),
				.CEN_A(chip_en_dm_shifted[2*l]),
				.CEN_B(chip_en_dm_shifted[2*l+1]),
				.RDWEN_A(wr_en_dm_shifted[2*l]),
				.RDWEN_B(wr_en_dm_shifted[2*l+1]),
				.DEEPSLEEP(1'b0),
				.POWERGATE(1'b0),
				.AS_A(addr_dm_shifted[2*l][7]),
				.AW_A(addr_dm_shifted[2*l][4:0]),
				.AC_A(addr_dm_shifted[2*l][6:5]),
				.AS_B(addr_dm_shifted[2*l+1][7]),
				.AW_B(addr_dm_shifted[2*l+1][4:0]),
				.AC_B(addr_dm_shifted[2*l+1][6:5]),
				.D_A(data_in_dm_shifted[2*l]),
				.D_B(data_in_dm_shifted[2*l+1]),
				.BW_A(bit_en_shifted[2*l]),
				.BW_B(bit_en_shifted[2*l+1]),
				.T_BIST(1'b0),
				.T_LOGIC(1'b0),
				.T_CEN_A(1'b1),
				.T_CEN_B(1'b1),
				.T_RDWEN_A(1'b0),
				.T_RDWEN_B(1'b0),
				.T_DEEPSLEEP(1'b0),
				.T_POWERGATE(1'b0),
				.T_AS_A('0),
				.T_AS_B('0),
				.T_AW_A('0),
				.T_AW_B('0),
				.T_AC_A('0),
				.T_AC_B('0),
				.T_D_A({64{1'b0}}),
				.T_D_B({64{1'b0}}),
				.T_BW_A({64{1'b0}}),
				.T_BW_B({64{1'b0}}),
				.T_WBT(1'b0),
				.T_STAB(1'b0),
				.MA_SAWL(2'b01),
				.MA_WL(2'b00),
				.MA_WRAS(2'b00),
				.MA_WRASD(1'b0),
				.Q_A(data_out_dm[2*l]),
				.Q_B(data_out_dm[2*l+1]),
				.OBSV_CTL_A(),
				.OBSV_CTL_B()
			);*/
	end
endgenerate

generate
genvar i, j, k;

for(i = 0; i < NUM_TILES_Y; i++)
begin: y
    for(j = 0; j < NUM_TILES_X; j++)
    begin: x
        //TileXYId                  my_xy_id    [NUM_TILES_Y-1:0][NUM_TILES_X-1:0];
        logic [`RT_FLIT_SIZE-1:0] my_flit_in  [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_INPUT_PORTS-1-2:0];
        logic [`RT_FLIT_SIZE-1:0] my_flit_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_OUTPUT_PORTS-1-3:0];
        logic [`RT_FLIT_SIZE-1:0] my_flit_l_in  [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_INPUT_PORTS-1-2:0];
        logic [`RT_FLIT_SIZE-1:0] my_flit_l_out [NUM_TILES_Y-1:0][NUM_TILES_X-1:0][TILE_NUM_OUTPUT_PORTS-1-3:0];
 

//------------------------------------------------------------------------------
// Data memory
//------------------------------------------------------------------------------
logic [CM_WIDTH-1:0] data_in;
logic wr_en;
logic [4:0] addr;
logic [DM_DEPTH_BITS-1:0] addr_dm;
logic [DATA_WIDTH-1:0] bit_en;
logic [DM_WIDTH-1:0] data_in_dm;
logic rd_en_dm;
logic wr_en_dm;
logic is_dm_tile;

logic wr_en_shifted;
logic rd_en_shifted;
logic [DM_WIDTH-1:0] data_out_dm_x;
logic [CM_WIDTH-1:0] cm_bit_en_shifted;
logic [CM_WIDTH-1:0] cm_data_shifted;
logic [CM_WIDTH-1:0] cm_data_out;

logic [CM_DEPTH_ROUTING_BITS:0] addr_shifted_r_reg;
logic [CM_DEPTH_ROUTING_BITS:0] addr_shifted_r_to_mem;

logic clk_g_r_gated,clkn_gg;

        //----------------------------------------------------------------------
        // Flit, Credit, SSR
        //----------------------------------------------------------------------
        assign my_flit_in[i][j][EAST]     = w__flit_in[i][j][EAST];
        assign my_flit_in[i][j][SOUTH]    = w__flit_in[i][j][SOUTH];
        assign my_flit_in[i][j][WEST]     = w__flit_in[i][j][WEST];
        assign my_flit_in[i][j][NORTH]    = w__flit_in[i][j][NORTH];
        assign my_flit_l_in[i][j][EAST]     = w__flit_l_in[i][j][EAST];
        assign my_flit_l_in[i][j][SOUTH]    = w__flit_l_in[i][j][SOUTH];
        assign my_flit_l_in[i][j][WEST]     = w__flit_l_in[i][j][WEST];
        assign my_flit_l_in[i][j][NORTH]    = w__flit_l_in[i][j][NORTH];

        assign w__flit_out[i][j][EAST]    = my_flit_out[i][j][EAST];
        assign w__flit_out[i][j][SOUTH]   = my_flit_out[i][j][SOUTH];
        assign w__flit_out[i][j][WEST]    = my_flit_out[i][j][WEST];
        assign w__flit_out[i][j][NORTH]   = my_flit_out[i][j][NORTH];
        assign w__flit_l_out[i][j][EAST]    = my_flit_l_out[i][j][EAST];
        assign w__flit_l_out[i][j][SOUTH]   = my_flit_l_out[i][j][SOUTH];
        assign w__flit_l_out[i][j][WEST]    = my_flit_l_out[i][j][WEST];
        assign w__flit_l_out[i][j][NORTH]   = my_flit_l_out[i][j][NORTH];

        //----------------------------------------------------------------------
	always_comb
	begin
		if (ALU_MEM[i*NUM_TILES_X+j] == 0) begin
			data_out_dm_x = {DM_WIDTH{1'b0}} ;
			is_dm_tile = 1'b0;
		end
		else begin
			data_out_dm_x = data_out_dm[(ALU_MEM[i*NUM_TILES_X+j]-1) & {MEM_PORTS_BITS{1'b1}}][DATA_WIDTH-1:0];
			is_dm_tile = 1'b1;
		end
	end

always_ff @(posedge clkn_g or negedge rstn) begin
    if (!rstn) begin
        wr_en_shifted <= 1'b1;
        rd_en_shifted <= 1'b1;
    end
    else begin
        if (chip_en) begin
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
end

//wangbo
assign addr_dm_out[i*NUM_TILES_X+j] = addr_dm;
assign bit_en_out[i*NUM_TILES_X+j] = bit_en;
assign data_in_dm_out[i*NUM_TILES_X+j] = data_in_dm;
assign rd_en_dm_out[i*NUM_TILES_X+j] = rd_en_dm;
assign wr_en_dm_out[i*NUM_TILES_X+j] = wr_en_dm;

always_ff @(posedge clkn_g or negedge rstn) begin
    if (!rstn) begin
        cm_bit_en_shifted <= {CM_WIDTH{1'b0}};
    end
    else begin
        if (chip_en == 1'b1) begin
            cm_bit_en_shifted <= cm_bit_en[i][j];
        end else begin
            cm_bit_en_shifted <= {CM_WIDTH{1'b0}};
        end
    end
end

always_ff @(posedge clkn_g or negedge rstn) begin
    if (!rstn) begin
        cm_data_shifted <= {CM_WIDTH{1'b0}};
    end
    else begin
        if (chip_en == 1'b1) begin
            cm_data_shifted <= cm_data;
        end else begin
            cm_data_shifted <= {CM_WIDTH{1'b0}};
        end
    end
end

assign clk_g_r_gated = clk_g_r  & clken[i*NUM_TILES_X+j] & clken_cmem_r[addr_shifted_r[CM_DEPTH_ROUTING_BITS-1:0]][i*NUM_TILES_X+j];

always_ff @(posedge clk_g_r or negedge rstn) begin
    if (!rstn) begin
        addr_shifted_r_reg <= {CM_DEPTH_ROUTING_BITS{1'b0}};
    end
    else begin
            addr_shifted_r_reg <= addr_shifted_r;
    end
end
assign addr_shifted_r_to_mem = (clken_cmem_r[addr_shifted_r[CM_DEPTH_ROUTING_BITS-1:0]][i*NUM_TILES_X+j]) ? addr_shifted_r : addr_shifted_r_reg;
clkgate #(1) clkgate0 (clk, clkn, clken[i*NUM_TILES_X+j], rstn, clk_gg);
clkgate #(1) clkgate00 (clkn, clk, clken[i*NUM_TILES_X+j], rstn, clkn_gg);

	if(ALU_MEM[i*NUM_TILES_X+j]>0) begin
        	tile_ldst
		#(
			.DATA_WIDTH			    (DATA_WIDTH),
			.DM_DEPTH			    (DM_DEPTH),
			.DM_WIDTH			    (DM_WIDTH),
			.CM_WIDTH_OPERATION		    (CM_WIDTH_OPERATION),
			.CM_WIDTH_ROUTING		    (CM_WIDTH_ROUTING),
			.CM_DEPTH_OPERATION		    (CM_DEPTH),
			.CM_DEPTH_ROUTING		    (CM_DEPTH),
			.LOCAL_REG_DEPTH		    (LOCAL_REG_DEPTH),
			.ADDR_LEN			    (ADDR_LEN),
			.NOP_BITS			    (NOP_BITS),
			.TRIGGER_SIZE			    (TRIGGER_SIZE),
			.VEC_WIDTH			    (VEC_WIDTH))
            	tile(
                	.clk                                (clk_gg),
			.clkn    			    (clkn_gg),
                 	.rstn                               (rstn),
                	.chip_en                            (chip_en),
                	.i__flit_in_0                       (my_flit_in[i][j][0]),
                	.i__flit_in_1                       (my_flit_in[i][j][1]),
                	.i__flit_in_2                       (my_flit_in[i][j][2]),
                	.i__flit_in_3                       (my_flit_in[i][j][3]),
                	.o__flit_out_0                      (my_flit_out[i][j][0]),
                	.o__flit_out_1                      (my_flit_out[i][j][1]),
                	.o__flit_out_2                      (my_flit_out[i][j][2]),
                	.o__flit_out_3                      (my_flit_out[i][j][3]),  
                	.i__flit_in_l_0                       (my_flit_l_in[i][j][0]),
                	.i__flit_in_l_1                       (my_flit_l_in[i][j][1]),
                	.i__flit_in_l_2                       (my_flit_l_in[i][j][2]),
                	.i__flit_in_l_3                       (my_flit_l_in[i][j][3]),
                	.o__flit_out_l_0                      (my_flit_l_out[i][j][0]),
                	.o__flit_out_l_1                      (my_flit_l_out[i][j][1]),
                	.o__flit_out_l_2                      (my_flit_l_out[i][j][2]),
                	.o__flit_out_l_3                      (my_flit_l_out[i][j][3]), 
			.start_exec_shifted                 (start_exec_shifted),
			.start_exec_shifted_shifted	    (start_exec_shifted_shifted),
        		.data_in_dm             	    (data_in_dm),
    			.data_out_dm                	    (data_out_dm_x),
        		.addr_dm                	    (addr_dm),
        		.bit_en                 	    (bit_en),
        		.rd_en_dm               	    (rd_en_dm),
        		.wr_en_dm               	    (wr_en_dm),
        		.wr_en_shifted     		    (wr_en_shifted),
   		        .cm_bit_en_shifted  		    (cm_bit_en_shifted[CM_WIDTH_OPERATION + CM_WIDTH_ROUTING_MEM-1:0]),
        		.cm_data_shifted    		    (cm_data_shifted[CM_WIDTH_OPERATION + CM_WIDTH_ROUTING_MEM-1:0]),
			.data_oload_in	    		    (oload_data[i][j]),
			.data_oload_valid	            (oload_valid[i][j]),
			.addr_oload_in	    		    (oload_addr),
			.address_in	  		    (address_in),
        		.trigger_op          		    (trigger_op[i][j]),
			.start_exec	  		    (scan_start_exec),
			.trigger_data_in		    (trigger_data_in[i*NUM_TILES_X+j]),
			.addr_shifted_r                     (addr_shifted_r_to_mem),
			.vec_counter_r         		    (vec_counter_r),
			.clk_g_r			    (clk_g_r_gated),
			.loop_end_r			    (loop_end),
			.vec_size			    (vec_size),
			.clken_cmem_op			    (clken_cmem_op_tot[i*NUM_TILES_X+j]),
			.clken_cmem_r			    (clken_cmem_r[addr_shifted_r[CM_DEPTH_ROUTING_BITS-1:0]][i*NUM_TILES_X+j]),
	                .clken_cmem_op_cycle		    (clken_cmem_op_trans[i*NUM_TILES_X+j])
            	);
	end
	else
	begin
		tile
		#(
			.DATA_WIDTH(DATA_WIDTH),
			.CM_WIDTH_OPERATION		    (CM_WIDTH_OPERATION),
			.CM_WIDTH_ROUTING		    (CM_WIDTH_ROUTING),
			.CM_DEPTH_OPERATION		    (CM_DEPTH),
			.CM_DEPTH_ROUTING		    (CM_DEPTH),
			.LOCAL_REG_DEPTH		    (LOCAL_REG_DEPTH),
			.ADDR_LEN			    (ADDR_LEN),
			.NOP_BITS			    (NOP_BITS),
			.TRIGGER_SIZE			    (TRIGGER_SIZE),
			.VEC_WIDTH			    (VEC_WIDTH))
            	tile(
                	.clk                                (clk_gg),
			.clkn    			    (clkn_gg),
                	.rstn                               (rstn),
                	.chip_en                            (chip_en),
                	.i__flit_in_0                       (my_flit_in[i][j][0]),
                	.i__flit_in_1                       (my_flit_in[i][j][1]),
                	.i__flit_in_2                       (my_flit_in[i][j][2]),
                	.i__flit_in_3                       (my_flit_in[i][j][3]),
                	.o__flit_out_0                      (my_flit_out[i][j][0]),
                	.o__flit_out_1                      (my_flit_out[i][j][1]),
                	.o__flit_out_2                      (my_flit_out[i][j][2]),
                	.o__flit_out_3                      (my_flit_out[i][j][3]),
                	.i__flit_in_l_0                       (my_flit_l_in[i][j][0]),
                	.i__flit_in_l_1                       (my_flit_l_in[i][j][1]),
                	.i__flit_in_l_2                       (my_flit_l_in[i][j][2]),
                	.i__flit_in_l_3                       (my_flit_l_in[i][j][3]),
                	.o__flit_out_l_0                      (my_flit_l_out[i][j][0]),
                	.o__flit_out_l_1                      (my_flit_l_out[i][j][1]),
                	.o__flit_out_l_2                      (my_flit_l_out[i][j][2]),
                	.o__flit_out_l_3                      (my_flit_l_out[i][j][3]), 
        		.start_exec_shifted            	    (start_exec_shifted),
			.start_exec_shifted_shifted	    (start_exec_shifted_shifted),
        		.wr_en_shifted     		    (wr_en_shifted),
   		        .cm_bit_en_shifted                  (cm_bit_en_shifted[CM_WIDTH_OPERATION + CM_WIDTH_ROUTING_MEM-1:0]),
        		.cm_data_shifted   		    (cm_data_shifted[CM_WIDTH_OPERATION + CM_WIDTH_ROUTING_MEM-1:0]),
			.data_oload_in	   		    (oload_data[i][j]),
			.data_oload_valid	            (oload_valid[i][j]),
			.addr_oload_in	    		    (oload_addr),
			.address_in	  		    (address_in),
        		.trigger_op         		    (trigger_op[i][j]),
			.start_exec	  		    (scan_start_exec),
			.trigger_data_in		    (trigger_data_in[i*NUM_TILES_X+j]),
			.addr_shifted_r                     (addr_shifted_r_to_mem),
			.vec_counter_r         		    (vec_counter_r),
			.loop_end_r			    (loop_end),
			.clk_g_r			    (clk_g_r_gated),
			.vec_size			    (vec_size),
			.clken_cmem_op			    (clken_cmem_op_tot[i*NUM_TILES_X+j]),
			.clken_cmem_r			    (clken_cmem_r[addr_shifted_r[CM_DEPTH_ROUTING_BITS-1:0]][i*NUM_TILES_X+j]),
	                .clken_cmem_op_cycle	            (clken_cmem_op_trans[i*NUM_TILES_X+j])
            	);

	end
    end
end

for(i = 0; i < NUM_TILES_Y; i++)
begin: y_connection
    for(j = 0; j < NUM_TILES_X; j++)
    begin: x_connection

        //----------------------------------------------------------------------
        // Flit and credit
        //----------------------------------------------------------------------
        // East
        if(j < (NUM_TILES_X-1))
        begin: flit_east
            assign w__flit_in[i][j][EAST]   = w__flit_out[i][j + 1][WEST];
	    assign w__flit_l_in[i][j][EAST]   = w__flit_l_out[i][j + 1][WEST];
        end
        else
        begin: flit_east_out
		if(ALU_MEM[i*NUM_TILES_X+j]>0) begin
			assign w__flit_in[i][j][EAST]   = iterValue+vec_counter_r;
			assign w__flit_l_in[i][j][EAST]   = iterValue_reg+vec_counter_r;
			//assign w__flit_in[i][j][EAST]   = iterValue;
			//assign w__flit_l_in[i][j][EAST]   = iterValue_reg;


		end
		else
		begin
            		assign w__flit_in[i][j][EAST]   = '0;
			assign w__flit_l_in[i][j][EAST]   = '0;
		end
        end

        // South
        if(i >= 1)
        begin: flit_south
            assign w__flit_in[i][j][NORTH]      = w__flit_out[i - 1][j][SOUTH];
	    assign w__flit_l_in[i][j][NORTH]      = w__flit_l_out[i - 1][j][SOUTH];
        end
        else
        begin: flit_south_out
            assign w__flit_in[i][j][NORTH]      = iterValue+vec_counter_r;
            assign w__flit_l_in[i][j][NORTH]      = iterValue_reg+vec_counter_r;
            //assign w__flit_in[i][j][NORTH]      = iterValue;
            //assign w__flit_l_in[i][j][NORTH]      = iterValue_reg;	
        end

        // West
        if(j >= 1)
        begin: flit_west
            assign w__flit_in[i][j][WEST]   = w__flit_out[i][j - 1][EAST];
	    assign w__flit_l_in[i][j][WEST]   = w__flit_l_out[i][j - 1][EAST];

        end
        else
        begin: flit_west_out
		if(ALU_MEM[i*NUM_TILES_X+j]>0) begin
			assign w__flit_in[i][j][WEST]   = iterValue+vec_counter_r;
			assign w__flit_l_in[i][j][WEST]   = iterValue_reg+vec_counter_r;
			//assign w__flit_in[i][j][WEST]   = iterValue;
			//assign w__flit_l_in[i][j][WEST]   = iterValue_reg;

		end
		else
		begin
            		assign w__flit_in[i][j][WEST]   = '0;
			assign w__flit_l_in[i][j][WEST]   = '0;
		end
        end

        // North
        if(i < (NUM_TILES_Y-1))
        begin: flit_north
            assign w__flit_in[i][j][SOUTH]      = w__flit_out[i + 1][j][NORTH];
            assign w__flit_l_in[i][j][SOUTH]      = w__flit_l_out[i + 1][j][NORTH];
        end
        else
        begin: flit_north_out
            assign w__flit_in[i][j][SOUTH]      = iterValue+vec_counter_r;
	    assign w__flit_l_in[i][j][SOUTH]      = iterValue_reg+vec_counter_r;
	    // assign w__flit_in[i][j][SOUTH]      = iterValue;
	    // assign w__flit_l_in[i][j][SOUTH]      = iterValue_reg;

        end
        //----------------------------------------------------------------------

    end
end

endgenerate

genvar m;
generate
	for (m=0;m<2*NUM_DMEM_BANKS;m++) begin
		always_comb begin
			if(!scan_start_exec) begin
				addr_dm_shifted[m] = address_in[DM_DEPTH_BITS:1];
				bit_en_shifted[m] = {DM_WIDTH{1'b1}};
			//	if(address_in[1] == 1'b0)
					data_in_dm_shifted[m][DATA_WIDTH-1:0] = data_in;
		//		else //if(address_in[2:1] == 2'b01)
		//			data_in_dm_shifted[m][2*DATA_WIDTH-1:DATA_WIDTH] = data_in;
			/*	else if(address_in[2:1] == 2'b10)
					data_in_dm_shifted[m][3*DATA_WIDTH-1:2*DATA_WIDTH] = data_in;
				else
					data_in_dm_shifted[m][4*DATA_WIDTH-1:3*DATA_WIDTH] = data_in;*/
		
				chip_en_dm_shifted[m] = chip_en_dm[m];
				wr_en_dm_shifted[m] = read_write;

			end
			else
			begin
				addr_dm_shifted[m] = addr_dm_out[ALU_MEM_PES[m] & {NO_PES_BITS{1'b1}}];
				bit_en_shifted[m] = bit_en_out[ALU_MEM_PES[m] & {NO_PES_BITS{1'b1}}];
				data_in_dm_shifted[m] = data_in_dm_out[ALU_MEM_PES[m] & {NO_PES_BITS{1'b1}}];
				chip_en_dm_shifted[m] = rd_en_dm_out[ALU_MEM_PES[m] & {NO_PES_BITS{1'b1}}];
				wr_en_dm_shifted[m] = wr_en_dm_out[ALU_MEM_PES[m] & {NO_PES_BITS{1'b1}}];
			end
		end
	end
endgenerate


always_ff @(posedge clk_g or negedge rstn) begin
    if (!rstn) begin
        data_out <= {DATA_WIDTH{1'b0}};
        data_out_valid <= 1'b0;
    end
    else begin
        if (chip_en) begin
            if (read_write && data_addr_valid_reg == 2'b11 && address_in[ADDR_LEN-1:ADDR_LEN-4] == 'b0001) begin
                data_out_valid <= 1'b1;
                data_out <= data_out_dm[address_in[ADDR_LEN-3:ADDR_LEN-3-MEM_PORTS_BITS+1]][DATA_WIDTH-1:0];
    		end else begin
                data_out <= data_out;
                data_out_valid <= 1'b0;
            end
        end else begin
            data_out <= {DATA_WIDTH{1'b0}};
            data_out_valid <= 1'b0;
        end
    end
end



always_ff @(posedge clk or negedge rstn) begin
    if (!rstn)
        vec_counter_r <= {VEC_WIDTH_BITS{1'b1}};
    else begin
        if (chip_en && start_exec_shifted == 1'b1) begin
	   if ((vec_counter_r == vec_size)) begin
              vec_counter_r <= {VEC_WIDTH_BITS{1'b0}};
           end else begin
              vec_counter_r <= vec_counter_r + 1'b1;
           end
        end else begin
	   vec_counter_r <= {VEC_WIDTH_BITS{1'b1}};
        end
    end
end
assign clken_vec_r = (!start_exec_shifted)? 1: ((vec_counter_r == 'b0 ) && (loop_end != 'b0))? 1:0;
logic clken_vec_r1;
assign clken_vec_r1 = (!start_exec_shifted)? 1: ((vec_counter_r == 'b0 ))? 1:0;

clkgate #(1) clkgate5 (clkn, clk, clken_vec_r, rstn, clkn_g_r);
clkgate #(1) clkgate6 (clk, clkn, clken_vec_r, rstn, clk_g_r);
clkgate #(1) clkgate7 (clk, clkn, clken_vec_r1, rstn, clk_g_r1);


always_ff @(posedge clkn_g_r or negedge rstn) begin
    if (!rstn)
        addr_shifted_r <= {CM_DEPTH_ROUTING_BITS+1{1'b0}};
    else begin
        if (chip_en) begin
            if (start_exec_shifted_shifted == 1'b1) begin

		if ((addr_shifted_r >= loop_end)) begin
                    addr_shifted_r <= 'b0;
                end else begin
                    addr_shifted_r <= addr_shifted_r + 1'b1;
                end
            end else if (scan_start_exec == 1'b1 && start_exec_shifted == 1'b0) begin
		if (loop_end == 1'b0) begin
		    addr_shifted_r <= 'b0;
		end else begin	
                    addr_shifted_r <=  'b0;	
		end	
            end else if (start_exec_shifted == 1'b1 && start_exec_shifted_shifted == 1'b0 && loop_end == 1'b0) begin
		    addr_shifted_r <= 'b0;

            end else begin
		if (address_in[18:17] == 2'b00) begin
                	addr_shifted_r <= {1'b0,address_in[CM_DEPTH_ROUTING_BITS+2:3]};
		end
		else
		begin
			addr_shifted_r <= {CM_DEPTH_ROUTING_BITS+1{1'b0}};
		end
            end
        end
    end
end

endmodule

