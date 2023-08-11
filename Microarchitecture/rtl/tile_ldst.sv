module tile_ldst(
    clk,
    clk_g_r,
    clkn,
    rstn,
    chip_en,
    i__flit_in_0,
    i__flit_in_1,
    i__flit_in_2,
    i__flit_in_3,
    i__flit_in_l_0,
    i__flit_in_l_1,
    i__flit_in_l_2,
    i__flit_in_l_3,
    o__flit_out_0,
    o__flit_out_1,
    o__flit_out_2,
    o__flit_out_3,
    o__flit_out_l_0,
    o__flit_out_l_1,
    o__flit_out_l_2,
    o__flit_out_l_3,
    start_exec_shifted,
    start_exec_shifted_shifted,  
    data_in_dm,
    data_out_dm,
    addr_dm,
    bit_en,
    rd_en_dm,
    wr_en_dm,
    wr_en_shifted,
    cm_data_shifted,
    cm_bit_en_shifted,
    addr_oload_in,
    data_oload_in,
    data_oload_valid,
    trigger_data_in,
    trigger_op,
    start_exec,
    loop_end_r,
    address_in,
    addr_shifted_r,
    vec_counter_r,
    vec_size,
    clken_cmem_op,
    clken_cmem_r,
    clken_cmem_op_cycle
);

//------------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------
import TopPkg::*;
import SMARTPkg::*;

parameter DATA_WIDTH            	= 16;
parameter CM_WIDTH_OPERATION		= 24;
parameter CM_WIDTH_ROUTING		= 16;
parameter CM_WIDTH_ROUTING_MEM		= 28;

parameter CM_DEPTH_OPERATION          	= 8;
parameter CM_DEPTH_ROUTING		= 8;
localparam CM_DEPTH_OPERATION_BITS      = $clog2(CM_DEPTH_OPERATION);
localparam CM_DEPTH_ROUTING_BITS	= $clog2(CM_DEPTH_ROUTING);

parameter DM_DEPTH          		= 256; 
parameter DM_DEPTH_BITS         	= $clog2(DM_DEPTH);
parameter DM_WIDTH             		= 16; 
parameter DM_WIDTH_BITS         	= $clog2(DM_WIDTH);
parameter LOCAL_REG_DEPTH		= 8;
parameter LOCAL_REG_DEPTH_BITS         	= $clog2(LOCAL_REG_DEPTH);
parameter VEC_WIDTH			= 2;
localparam VEC_WIDTH_BITS		= $clog2(VEC_WIDTH);
parameter ADDR_LEN			= 19;
parameter NOP_BITS			=  2;
parameter TRIGGER_SIZE			= 15;

localparam NUM_INPUT_PORTS              = 7;
localparam NUM_OUTPUT_PORTS             = 7;
localparam ROUTER_NUM_INPUT_PORTS       = NUM_INPUT_PORTS;
localparam ROUTER_NUM_OUTPUT_PORTS      = NUM_OUTPUT_PORTS;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// IO
//------------------------------------------------------------------------------


input  logic                            clk;
input logic 				clkn;
input  logic                            rstn;
input  logic                            chip_en;

input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_0;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_1;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_2;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_3;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_l_0;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_l_1;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_l_2;
input  logic [`RT_FLIT_SIZE-1:0] i__flit_in_l_3;

output logic [`RT_FLIT_SIZE-1:0] o__flit_out_0;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_1;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_2;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_3;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_l_0;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_l_1;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_l_2;
output logic [`RT_FLIT_SIZE-1:0] o__flit_out_l_3;

input   start_exec_shifted,start_exec_shifted_shifted;

output [DM_DEPTH_BITS-1:0] addr_dm;
output [DATA_WIDTH-1:0] bit_en;
input [DM_WIDTH-1:0] data_out_dm;
output [DM_WIDTH-1:0] data_in_dm;
output rd_en_dm;
output wr_en_dm;
input logic [VEC_WIDTH_BITS-1:0] vec_size;

//control_mem
input wr_en_shifted;
input logic [CM_WIDTH_OPERATION+CM_WIDTH_ROUTING_MEM-1:0] cm_bit_en_shifted;
input logic [CM_WIDTH_OPERATION+CM_WIDTH_ROUTING_MEM-1:0] cm_data_shifted;
input logic [CM_DEPTH_ROUTING_BITS-1:0] loop_end_r;

input [ADDR_LEN-1:0] address_in;
input logic trigger_op;
input logic start_exec;

input logic [CM_DEPTH_ROUTING_BITS:0] addr_shifted_r;
input logic [VEC_WIDTH_BITS-1:0 ] vec_counter_r;
input logic clk_g_r;

//oload store
input [LOCAL_REG_DEPTH_BITS-1:0] addr_oload_in;
input [DATA_WIDTH-1:0] data_oload_in;
input data_oload_valid;

input [TRIGGER_SIZE-1:0] trigger_data_in;
//input logic clken_cmem_r;
input logic clken_cmem_op;
input logic clken_cmem_r;
input [CM_DEPTH_OPERATION-1:0] clken_cmem_op_cycle;
//----------------------------------------------------------------------------
// Internal signals
//------------------------------------------------------------------------------
logic [CM_WIDTH_OPERATION-1:0] control_reg_data_op;
logic [CM_WIDTH_ROUTING-1:0] control_reg_data_r;
logic rstn_s;

logic [ROUTER_NUM_INPUT_PORTS-1:0]  i__sram_xbar_sel    [ROUTER_NUM_OUTPUT_PORTS-1:0];
logic [`RT_FLIT_SIZE-1:0] o__flit_out_wire [NUM_OUTPUT_PORTS-1:0];

logic [DM_WIDTH-1:0] data_out_dm;
logic [DATA_WIDTH-1:0] data_out_dm_reg;
logic [DM_DEPTH_BITS-1:0] addr_dm;
logic [DATA_WIDTH-1:0] bit_en;
logic [DM_WIDTH-1:0] data_in_dm;
logic rd_en_dm;
logic wr_en_dm;

logic [DATA_WIDTH-1:0] lsu_alu_out;
logic not_to_execute_reg;

logic [DATA_WIDTH:0] alu_out;
//logic [DATA_WIDTH-1:0] alu_out_reg;

logic [DATA_WIDTH:0] tile_out;
logic [DATA_WIDTH:0] op_rhs;
logic [DATA_WIDTH:0] op_lhs;
logic [DATA_WIDTH:0] op_predicate_reg;
logic [DATA_WIDTH:0] op_predicate;
//logic [DATA_WIDTH:0] op_shift;
//logic [DATA_WIDTH:0] op_4;
logic [2:0] operation;
logic [3:0] regbypass;
//logic [3:0] regWEN;
//logic [8:0] prev_cycle_p_i2_i1;
logic not_to_execute;
logic not_to_execute_all;
logic [2:0] ldst;
logic [DATA_WIDTH:0] treg;
logic [DATA_WIDTH:0] treg1;

logic [DATA_WIDTH-1:0] op_const_ls;
logic [DATA_WIDTH-1:0] op_const_add;

logic clk_nop,clk_neg_nop;
logic [CM_DEPTH_OPERATION_BITS:0] addr_shifted_op;
logic rd_en_dm_reg;
logic  [CM_DEPTH_OPERATION_BITS:0] addr_shifted_op_reg;
logic [VEC_WIDTH_BITS-1:0 ] vec_counter_op;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// Reset synchronizor
ff_rst #(2) ff_rst0 (clk, rstn, rstn, rstn_s);

// Seperate storage for OLOADS
logic [DATA_WIDTH-1:0] local_reg [LOCAL_REG_DEPTH-1:0];
integer i;
always_ff @(posedge clk_nop or negedge rstn) begin
	if (!rstn) begin
		for (i=0; i<LOCAL_REG_DEPTH; i=i+1) local_reg[i] <= {DATA_WIDTH{1'b0}};
	end
	else begin
		if(!start_exec_shifted & data_oload_valid) begin
			local_reg[addr_oload_in] <= data_oload_in[DATA_WIDTH-1:0];
		end
	end
end
always_comb begin
    if (!rstn_s) begin
        op_rhs <= {DATA_WIDTH+1{1'b0}};
        op_lhs <= {DATA_WIDTH+1{1'b0}};
        op_predicate_reg <= {DATA_WIDTH+1{1'b0}};
    end
    else begin
        if (start_exec_shifted && clken_cmem_op_cycle[addr_shifted_op_reg[CM_DEPTH_OPERATION_BITS-1:0]]) begin
                op_rhs <= o__flit_out_wire[4][DATA_WIDTH:0];
                op_lhs <= o__flit_out_wire[5][DATA_WIDTH:0];
                op_predicate_reg <= o__flit_out_wire[6][DATA_WIDTH:0];
        end
	else begin
        	op_rhs <= {DATA_WIDTH+1{1'b0}};
        	op_lhs <= {DATA_WIDTH+1{1'b0}};
        	op_predicate_reg <= {DATA_WIDTH+1{1'b0}};	
	end
    end
end

/*always_ff @(posedge clk_nop or negedge rstn_s) begin
    if (!rstn_s) begin
        alu_out_reg <= {DATA_WIDTH{1'b0}};
    end
    else begin
	alu_out_reg <= alu_out;
    end
end*/
assign op_predicate = (control_reg_data_op[19]) ? {op_predicate_reg[DATA_WIDTH:1], ~op_predicate_reg[0]} : op_predicate_reg[DATA_WIDTH:0];
assign op_const_add = {local_reg[control_reg_data_op[14:12]]};//control_reg_data[42+DATA_WIDTH-1:42];
assign op_const_ls  = 16'b0000000000000001;// constant 2 for 16 bit data//{{DATA_WIDTH-5{1'b0}},control_reg_data[46+DATA_WIDTH:42+DATA_WIDTH]};
//assign addr_oload  = control_reg_data[46+DATA_WIDTH+LOCAL_REG_DEPTH_BITS-1:46+DATA_WIDTH];
assign operation = {control_reg_data_op[17:15]};
//assign regbypass = {control_reg_data,control_reg_data_r[11],control_reg_data_r[7],control_reg_data_r[3]};
assign regbypass[0] = ((control_reg_data_op[2:0] == 'b000) & control_reg_data_op[3]) || ((control_reg_data_op[6:4] == 'b000) & control_reg_data_op[7]) || ((control_reg_data_op[10:8] == 'b000) & control_reg_data_op[11]) || ((control_reg_data_r[2:0] == 'b000) & control_reg_data_r[3]) || ((control_reg_data_r[6:4] == 'b000) & control_reg_data_r[7]) || ((control_reg_data_r[10:8] == 'b000) & control_reg_data_r[11]) || ((control_reg_data_r[14:12] == 'b000) & control_reg_data_r[15]); 
assign regbypass[3] = ((control_reg_data_op[2:0] == 'b001) & control_reg_data_op[3]) || ((control_reg_data_op[6:4] == 'b001) & control_reg_data_op[7]) || ((control_reg_data_op[10:8] == 'b001) & control_reg_data_op[11]) || ((control_reg_data_r[2:0] == 'b001) & control_reg_data_r[3]) || ((control_reg_data_r[6:4] == 'b001) & control_reg_data_r[7]) || ((control_reg_data_r[10:8] == 'b001) & control_reg_data_r[11]) || ((control_reg_data_r[14:12] == 'b001) & control_reg_data_r[15]); 
assign regbypass[1] = ((control_reg_data_op[2:0] == 'b010) & control_reg_data_op[3]) || ((control_reg_data_op[6:4] == 'b010) & control_reg_data_op[7]) || ((control_reg_data_op[10:8] == 'b010) & control_reg_data_op[11]) || ((control_reg_data_r[2:0] == 'b010) & control_reg_data_r[3]) || ((control_reg_data_r[6:4] == 'b010) & control_reg_data_r[7]) || ((control_reg_data_r[10:8] == 'b010) & control_reg_data_r[11]) || ((control_reg_data_r[14:12] == 'b010) & control_reg_data_r[15]); 
assign regbypass[2] = ((control_reg_data_op[2:0] == 'b011) & control_reg_data_op[3]) || ((control_reg_data_op[6:4] == 'b011) & control_reg_data_op[7]) || ((control_reg_data_op[10:8] == 'b011) & control_reg_data_op[11]) || ((control_reg_data_r[2:0] == 'b011) & control_reg_data_r[3]) || ((control_reg_data_r[6:4] == 'b011) & control_reg_data_r[7]) || ((control_reg_data_r[10:8] == 'b011) & control_reg_data_r[11]) || ((control_reg_data_r[14:12] == 'b011) & control_reg_data_r[15]); 


//assign regWEN = 4'b1111;//control_reg_data[32:29];

//assign not_to_execute_all = ((prev_cycle_p_i2_i1[2:0] != 3'b111 && op_rhs[DATA_WIDTH]==1'b0) || (control_reg_data_op[18] != 1'b1 && prev_cycle_p_i2_i1[5:3] != 3'b111 && op_lhs[DATA_WIDTH]==1'b0) || (prev_cycle_p_i2_i1[8:6] != 3'b111 && (op_predicate[DATA_WIDTH]==1'b0 ||op_predicate[DATA_WIDTH-1:0]=={DATA_WIDTH{1'b0}}) )) ? 1'b1 : 1'b0;
assign not_to_execute_all = (((control_reg_data_op[2:0] != 3'b111 && op_rhs[DATA_WIDTH]==1'b0) || (control_reg_data_op[19] != 1'b1 && control_reg_data_op[6:4] != 3'b111 && op_lhs[DATA_WIDTH]==1'b0) || (control_reg_data_op[10:8] != 3'b111 && (op_predicate[DATA_WIDTH]==1'b0 ||op_predicate[DATA_WIDTH-1:0]=={DATA_WIDTH{1'b0}}) )) || !clken_cmem_op_cycle[addr_shifted_op_reg[CM_DEPTH_OPERATION_BITS-1:0]]) ? 1'b1 : 1'b0;
assign not_to_execute = not_to_execute_all;

assign o__flit_out_0 = o__flit_out_wire[0];
assign o__flit_out_1 = o__flit_out_wire[1];
assign o__flit_out_2 = o__flit_out_wire[2];
assign o__flit_out_3 = o__flit_out_wire[3];


ls_add_unit
    #(
    .DATA_WIDTH     (DATA_WIDTH),
    .VEC_WIDTH	    (VEC_WIDTH)
    ) a25_ls_add_unit (
    .op_LHS   		(op_rhs), // rhs carries data
    .op_const_ls     	(op_const_ls),
    .op_const_add     	(op_const_add),
    .vec_counter	(vec_counter_op),
    .result     	(alu_out)
);

 configurator
	#(
	.DATA_WIDTH(DATA_WIDTH),
	.CM_WIDTH_OPERATION(CM_WIDTH_OPERATION),
	.CM_WIDTH_ROUTING(CM_WIDTH_ROUTING),
	.CM_DEPTH_OPERATION(CM_DEPTH_OPERATION),
	.CM_DEPTH_ROUTING(CM_DEPTH_ROUTING),
	.VEC_WIDTH(VEC_WIDTH),
	.NOP_BITS(NOP_BITS),
	.TRIGGER_SIZE(TRIGGER_SIZE),
	.ADDR_LEN(ADDR_LEN)
	) config_gen(
	.clk(clk),
	.clkn(clkn),
	.rstn(rstn),
	.chip_en(chip_en),
	.start_exec_shifted(start_exec_shifted),
	.start_exec_shifted_shifted(start_exec_shifted_shifted),
	.start_exec(start_exec),
	.wr_en_shifted(wr_en_shifted),
	.cm_bit_en_shifted  (cm_bit_en_shifted),
    	.cm_data_shifted(cm_data_shifted),
	.trigger_data_in(trigger_data_in),	
	.address_in(address_in),
	.control_reg_data_op(control_reg_data_op),
	.control_reg_data_r(control_reg_data_r),
	.trigger_op(trigger_op),
	.clk_gated(clk_nop),
	.clk_neg_nop(clk_neg_nop),
	.addr_shifted_r                     (addr_shifted_r),
	.vec_counter_r         		    (vec_counter_r),
	.clk_g_r			    (clk_g_r),
	.vec_size			    (vec_size),
	.loop_end_r			    (loop_end_r),
	.clken_cmem_op			    (clken_cmem_op),
	.clken_cmem_r			    (clken_cmem_r),
        .clken_cmem_op_cycle	            (clken_cmem_op_cycle),
	.addr_shifted_op		    (addr_shifted_op),
	.vec_counter_op			    (vec_counter_op));



always_ff @(posedge clk_neg_nop or negedge rstn_s) begin
    if (!rstn_s) begin
        not_to_execute_reg <= 1'b0;
    end
    else begin
        if (start_exec_shifted) begin
            not_to_execute_reg <= not_to_execute;
        end
    end
end

assign tile_out [DATA_WIDTH-1:0] = lsu_alu_out;
//assign tile_out [DATA_WIDTH] = ~not_to_execute_reg;
assign tile_out [DATA_WIDTH] = 1'b0;

assign ldst = operation;

assign lsu_alu_out = data_out_dm_reg;

/*logic clk_rd_en;
assign clk_rd_en = (clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]]) ? clk : 1'b0;*/
always_ff @(posedge clk or negedge rstn) begin
	if (!rstn) begin
		addr_shifted_op_reg = {CM_DEPTH_OPERATION_BITS{1'b0}};
	end
	else begin
		addr_shifted_op_reg = addr_shifted_op;
	end
end
assign rd_en_dm = rd_en_dm_reg;

logic clk_en_compute1,clk_en_compute2;
always_ff @(posedge clk_nop or negedge rstn_s) begin
    if (!rstn_s) begin
        clk_en_compute1 <= 1'b0;
    end
    else begin
        if (start_exec_shifted)
                clk_en_compute1 <= clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]];
    end
end
always_ff @(posedge clk_nop or negedge rstn_s) begin
    if (!rstn_s) begin
        clk_en_compute2 <= 1'b0;
    end
    else begin
        if (start_exec_shifted)
                clk_en_compute2 <= clk_en_compute1;
    end
end

logic clk_nop_out,clk_nop_treg,clk_nop_treg1;
clkgate #(1) clkgate00 (clk_nop, clkn, clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]], rstn, clk_nop_out);
clkgate #(1) clkgate01 (clk_nop, clkn, clk_en_compute1, rstn, clk_nop_treg);
clkgate #(1) clkgate02 (clk_nop, clkn, clk_en_compute2, rstn, clk_nop_treg1);


//assign clk_nop_out = clk_nop && clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]];
//assign clk_nop_treg = clk_nop && clk_en_compute1;
//assign clk_nop_treg1 = clk_nop && clk_en_compute2;

always_ff @(posedge clk_nop_treg or negedge rstn_s) begin
    if (!rstn_s) begin
        treg <= {DATA_WIDTH+1{1'b0}};
    end
    else begin
        if (start_exec_shifted)
                treg <= tile_out;
    end
end

always_ff @(posedge clk_nop_treg1 or negedge rstn_s) begin
    if (!rstn_s) begin
        treg1 <= {DATA_WIDTH+1{1'b0}};
    end
    else begin
        if (start_exec_shifted)
                treg1 <= treg;
    end
end

always_ff @(posedge clk_nop_out or negedge rstn_s) begin
    if (!rstn_s) begin
        data_out_dm_reg <= {DATA_WIDTH{1'b0}};
    end
    else begin
        if (start_exec_shifted) begin
            case(ldst)
	   3'b001: begin
 
			data_out_dm_reg <= data_out_dm[DATA_WIDTH-1:0];

	   end
 /*          3'b010: begin
                if (alu_out[0]== 1'b0) begin
                   data_out_dm_reg <= {{DATA_WIDTH-8{1'b0}},data_out_dm[7:0]};
                end
                else begin
                   data_out_dm_reg <= {{DATA_WIDTH-8{1'b0}},data_out_dm[15:8]};
                end
           end*/
              default: begin
 
			data_out_dm_reg <= data_out_dm[DATA_WIDTH-1:0];

		    
                    end
            endcase
        end
	else begin
		data_out_dm_reg <= {DATA_WIDTH{1'b0}};
	end
    end
end

always_comb begin
    //if (clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]]) begin
        case(ldst)
	   3'b001: begin
           	//$display("LOAD INST");
		if(clken_cmem_op_cycle[addr_shifted_op_reg[CM_DEPTH_OPERATION_BITS-1:0]]) begin
                	rd_en_dm_reg = 1'b0;
                	wr_en_dm = 1'b1;
                	addr_dm = alu_out[DATA_WIDTH] ? alu_out[DM_DEPTH_BITS:1]:{DM_DEPTH_BITS{1'b0}};
                	data_in_dm = {DM_WIDTH{1'b0}};
               		bit_en = {DM_WIDTH{1'b1}};
		end else begin
			rd_en_dm_reg = 1'b1;
        		wr_en_dm = 1'b1;
        		addr_dm = {DM_DEPTH_BITS{1'b0}} ;
        		data_in_dm = {DATA_WIDTH{1'b0}};
        		bit_en = {DATA_WIDTH{1'b0}};

		end

                //rd_en_dm = 1'b0;
	   end
 /*          3'b010: begin
                //$display("LOAD 1byte INST");
                rd_en_dm = 1'b0;
                wr_en_dm = 1'b1;
                addr_dm = alu_out[DM_DEPTH_BITS:1];
                data_in_dm = {DM_WIDTH{1'b0}};
                bit_en = {DM_WIDTH{1'b1}};
           end*/
           3'b011: begin
           	//$display("STORE INST");
		if(clken_cmem_op_cycle[addr_shifted_op_reg[CM_DEPTH_OPERATION_BITS-1:0]]) begin
                	rd_en_dm_reg = 1'b0;
                	wr_en_dm = 1'b0;
                	addr_dm = alu_out[DATA_WIDTH] ? alu_out[DM_DEPTH_BITS:1]:{DM_DEPTH_BITS{1'b0}};
			data_in_dm[DATA_WIDTH-1:0] = op_lhs[DATA_WIDTH-1:0];
                	bit_en = {DM_WIDTH{1'b1}};
		end else begin
			rd_en_dm_reg = 1'b1;
      			wr_en_dm = 1'b1;
        		addr_dm = {DM_DEPTH_BITS{1'b0}} ;
        		data_in_dm = {DATA_WIDTH{1'b0}};
        		bit_en = {DATA_WIDTH{1'b0}};
 
		end

           end
             default: begin
                //$display("NOT LOAD OR STORE INST");
                rd_en_dm_reg = 1'b1;
     		wr_en_dm = 1'b1;
        	addr_dm = {DM_DEPTH_BITS{1'b0}} ;
        	data_in_dm = {DATA_WIDTH{1'b0}};
        	bit_en = {DATA_WIDTH{1'b0}};
            end
        endcase
    /*end
    else begin
        rd_en_dm = 1'b1;

    end*/
end

always_comb begin
    case (control_reg_data_r[2:0])
        3'b000: begin
                i__sram_xbar_sel[0] = 6'b000001;
            end
        3'b001: begin
                i__sram_xbar_sel[0] = 6'b000010;
            end
        3'b010: begin
                i__sram_xbar_sel[0] = 6'b000100;
            end
        3'b011: begin
                i__sram_xbar_sel[0] = 6'b001000;
            end
        3'b100: begin
                i__sram_xbar_sel[0] = 6'b010000;
            end
        3'b101: begin
                i__sram_xbar_sel[0] = 6'b100000;
            end
        default: begin
                i__sram_xbar_sel[0] = 6'b000000;
            end
    endcase
end

always_comb begin
    case (control_reg_data_r[6:4])
        3'b000: begin
                i__sram_xbar_sel[1] = 6'b000001;
            end
        3'b001: begin
                i__sram_xbar_sel[1] = 6'b000010;
            end
        3'b010: begin
                i__sram_xbar_sel[1] = 6'b000100;
            end
        3'b011: begin
                i__sram_xbar_sel[1] = 6'b001000;
            end
        3'b100: begin
                i__sram_xbar_sel[1] = 6'b010000;
            end
        3'b101: begin
                i__sram_xbar_sel[1] = 6'b100000;
            end
        default: begin
                i__sram_xbar_sel[1] = 6'b000000;
            end
    endcase
end

always_comb begin
    case (control_reg_data_r[10:8])
        3'b000: begin
                i__sram_xbar_sel[2] = 6'b000001;
            end
        3'b001: begin
                i__sram_xbar_sel[2] = 6'b000010;
            end
        3'b010: begin
                i__sram_xbar_sel[2] = 6'b000100;
            end
        3'b011: begin
                i__sram_xbar_sel[2] = 6'b001000;
            end
        3'b100: begin
                i__sram_xbar_sel[2] = 6'b010000;
            end
        3'b101: begin
                i__sram_xbar_sel[2] = 6'b100000;
            end
        default: begin
                i__sram_xbar_sel[2] = 6'b000000;
            end
    endcase
end

always_comb begin
    case (control_reg_data_r[14:12])
        3'b000: begin
                i__sram_xbar_sel[3] = 6'b000001;
                end
        3'b001: begin
                i__sram_xbar_sel[3] = 6'b000010;
                end
        3'b010: begin
                i__sram_xbar_sel[3] = 6'b000100;
                end
        3'b011: begin
                i__sram_xbar_sel[3] = 6'b001000;
                end
        3'b100: begin
                i__sram_xbar_sel[3] = 6'b010000;
                end
        3'b101: begin
                i__sram_xbar_sel[3] = 6'b100000;
                end
        default: begin
                i__sram_xbar_sel[3] = 6'b000000;
                end
    endcase
end

always_comb begin
    case (control_reg_data_op[2:0])
        3'b000: begin
                i__sram_xbar_sel[4] = 6'b000001;
                end
        3'b001: begin
                i__sram_xbar_sel[4] = 6'b000010;
                end
        3'b010: begin
                i__sram_xbar_sel[4] = 6'b000100;
                end
        3'b011: begin
                i__sram_xbar_sel[4] = 6'b001000;
                end
        3'b100: begin
                i__sram_xbar_sel[4] = 6'b010000;
                end
        3'b101: begin
                i__sram_xbar_sel[4] = 6'b100000;
                end
        default: begin
                i__sram_xbar_sel[4] = 6'b000000;
                end
    endcase
end

always_comb begin
    case (control_reg_data_op[6:4])
        3'b000: begin
                i__sram_xbar_sel[5] = 6'b000001;
                end
        3'b001: begin
                i__sram_xbar_sel[5] = 6'b000010;
                end
        3'b010: begin
                i__sram_xbar_sel[5] = 6'b000100;
                end
        3'b011: begin
                i__sram_xbar_sel[5] = 6'b001000;
                end
        3'b100: begin
                i__sram_xbar_sel[5] = 6'b010000;
                end
        3'b101: begin
                i__sram_xbar_sel[5] = 6'b100000;
                end
        default: begin
                i__sram_xbar_sel[5] = 6'b000000;
                end
    endcase
end

always_comb begin
    case (control_reg_data_op[10:8])
        3'b000: begin
                i__sram_xbar_sel[6] = 6'b000001;
                end
        3'b001: begin
                i__sram_xbar_sel[6] = 6'b000010;
                end
        3'b010: begin
                i__sram_xbar_sel[6] = 6'b000100;
                end
        3'b011: begin
                i__sram_xbar_sel[6] = 6'b001000;
                end
        3'b100: begin
                i__sram_xbar_sel[6] = 6'b010000;
                end
        3'b101: begin
                i__sram_xbar_sel[6] = 6'b100000;
                end
        default: begin
                i__sram_xbar_sel[6] = 6'b000000;
                end
    endcase
end

router
    #(
    .DATA_WIDTH         (DATA_WIDTH)
       )
    router(
        .clk                            (clk_nop),
	.clkn				(clkn),
        .reset                           (!rstn_s),
	.start_exec_shifted		(start_exec_shifted),
        .i__sram_xbar_sel               (i__sram_xbar_sel),
        .i__flit_in_0                   (i__flit_in_0),
        .i__flit_in_1                   (i__flit_in_1),
        .i__flit_in_2                   (i__flit_in_2),
        .i__flit_in_3                   (i__flit_in_3),
        .i__flit_in_l_0                   (i__flit_in_l_0),
        .i__flit_in_l_1                   (i__flit_in_l_1),
        .i__flit_in_l_2                   (i__flit_in_l_2),
        .i__flit_in_l_3                   (i__flit_in_l_3),
	.o__flit_out_l_0		(o__flit_out_l_0),
	.o__flit_out_l_1		(o__flit_out_l_1),
	.o__flit_out_l_2		(o__flit_out_l_2),
	.o__flit_out_l_3		(o__flit_out_l_3),		
        .i__alu_out                     (tile_out),
        .i__treg                        (treg),
	.i__treg1			(treg1),
        .o__flit_out                    (o__flit_out_wire),
        .regbypass          (regbypass)
    );
//------------------------------------------------------------------------------


endmodule

