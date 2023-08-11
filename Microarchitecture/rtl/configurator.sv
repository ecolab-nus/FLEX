module configurator(
	clk,
	clkn,
	clk_g_r,
	rstn,
	chip_en,
	start_exec_shifted,
	start_exec_shifted_shifted,
	start_exec,
	wr_en_shifted,
	cm_bit_en_shifted,
    	cm_data_shifted,
	trigger_data_in,	
	address_in,
	control_reg_data_op,
	control_reg_data_r,
	trigger_op,
	clk_gated,
	clk_neg_nop,
	loop_end_r,
	addr_shifted_r,
	vec_counter_r,
	vec_size,
    	clken_cmem_op,
    	clken_cmem_op_cycle,
    	clken_cmem_r,
	addr_shifted_op,
	vec_counter_op
);

import TopPkg::*;
import SMARTPkg::*;

parameter DATA_WIDTH            	= 16;
parameter CM_WIDTH_OPERATION		= 24;
parameter CM_WIDTH_ROUTING		= 16;
parameter CM_WIDTH_ROUTING_MEM		= 28;
parameter CM_DEPTH_OPERATION          	= 8;
parameter CM_DEPTH_ROUTING		= 8;
parameter VEC_WIDTH			= 2;
parameter NOP_BITS			= 3;
parameter TRIGGER_SIZE			= 15;
parameter ADDR_LEN			= 19;

localparam VEC_WIDTH_BITS		= $clog2(VEC_WIDTH);
localparam CM_DEPTH_OPERATION_BITS      = $clog2(CM_DEPTH_OPERATION);
localparam CM_DEPTH_ROUTING_BITS        = $clog2(CM_DEPTH_ROUTING);

input logic clk;
input logic clkn;
input logic rstn;
input logic chip_en;
input logic start_exec_shifted,start_exec_shifted_shifted;
input logic start_exec;
input logic wr_en_shifted;
input logic [CM_WIDTH_OPERATION+CM_WIDTH_ROUTING_MEM-1:0] cm_data_shifted;
input logic [CM_WIDTH_OPERATION+CM_WIDTH_ROUTING_MEM-1:0] cm_bit_en_shifted;

input logic [TRIGGER_SIZE-1:0] trigger_data_in;
input logic [ADDR_LEN-1:0] address_in;
input logic [VEC_WIDTH_BITS-1:0] vec_size;
output [CM_WIDTH_OPERATION-1:0] control_reg_data_op;
output [CM_WIDTH_ROUTING-1:0] control_reg_data_r;


logic [CM_WIDTH_OPERATION-1:0] data_out_cmem_op;
logic [CM_WIDTH_ROUTING_MEM-1:0] data_out_cmem_r;

output logic [CM_DEPTH_OPERATION_BITS:0] addr_shifted_op;
logic [CM_DEPTH_OPERATION_BITS:0] addr_shifted_op_reg;
logic [CM_DEPTH_OPERATION_BITS:0] addr_shifted_op_to_mem;

input logic [CM_DEPTH_ROUTING_BITS:0] addr_shifted_r;
input logic [VEC_WIDTH_BITS-1:0 ] vec_counter_r;
input logic clk_g_r;
input logic [CM_DEPTH_ROUTING_BITS-1:0] loop_end_r;
logic [CM_DEPTH_OPERATION_BITS-1:0] loop_end_op;
logic clk_loop_end_op;
logic clk_loop_end_op_en;
input logic clken_cmem_r;
input logic clken_cmem_op;
input logic [CM_DEPTH_OPERATION-1:0] clken_cmem_op_cycle;
input logic trigger_op;
logic trigger_E;
logic trigger_W;
logic trigger_N;
logic trigger_S;
logic clk_E;
logic clk_W;
logic clk_N;
logic clk_S;

logic [VEC_WIDTH_BITS-1:0 ] vec_counter;
output logic [VEC_WIDTH_BITS-1:0 ] vec_counter_op;

logic gate_nop;
logic [NOP_BITS-1:0] nop_counter;
output logic clk_gated,clk_neg_nop;
logic clk_n_nop;
logic clk_nop;
logic clken_vec_op;
logic [CM_WIDTH_ROUTING-1:0] control_reg_data_r_reg;
logic [11:0] control_reg_data_op_reg;
logic clkn_g_op,clk_g_op,clk_g_op_gated;
logic clk_en_spatial;
//NOP Gating

clkgate #(1) clkgate0 (clk, clkn, ~(control_reg_data_op[23:21] == {NOP_BITS{1'b0}}), rstn, clk_nop);
assign clk_n_nop = ~clk_nop & clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]];
always_ff @(posedge clk_n_nop or negedge rstn) begin
    if (!rstn)
    begin
        nop_counter <= {NOP_BITS{1'b0}};
	gate_nop <= 1'b0;
    end
    else begin
	if (chip_en && start_exec_shifted == 1'b1 && trigger_op == 1'b1) begin
		if (nop_counter == control_reg_data_op[23:21]) 
		begin
			gate_nop <= 1'b0;
			nop_counter <= {NOP_BITS{1'b0}};
		end
		else
		begin
			gate_nop <= 1'b1;
			nop_counter <= nop_counter + 1'b1;			
		end
	end
    end
end

clkgate #(1) clkgate2 (clk, clkn, ~gate_nop , rstn, clk_op_cmem);
assign clk_gated = clk_op_cmem;
assign clken_vec_op = (!start_exec_shifted_shifted)? 1'b1: ((vec_counter_op == 'b0) && (loop_end_op != 'b0))? 1'b1:1'b0;
logic clken_vec_op1;
assign clken_vec_op1 = (!start_exec_shifted_shifted)? 1'b1: (((vec_counter_op == 'b1) && (loop_end_op != 'b0)) || ((vec_size== 'b0) && (vec_counter_op == 'b0) && (loop_end_op != 'b0)))? 1'b1:1'b0;

assign clk_en_spatial = (!start_exec_shifted_shifted)? 1'b1: ((loop_end_op != 'b0))? 1'b1:1'b0;

always_ff @(posedge clk_op_cmem or negedge rstn) begin
    if (!rstn)
        vec_counter_op <= {VEC_WIDTH_BITS{1'b1}};
    else begin
        if (chip_en && start_exec_shifted_shifted == 1'b1 && trigger_op != 1'b0) begin
	   if ((vec_counter_op == vec_size)) begin
              vec_counter_op <= {VEC_WIDTH_BITS{1'b0}};
           end else begin
              vec_counter_op <= vec_counter_op + 1'b1;
           end
        end else begin
	   vec_counter_op <= {VEC_WIDTH_BITS{1'b1}};
        end
    end
end

clkgate #(1) clkgate3 (clkn, clk, clken_vec_op & clken_cmem_op, rstn, clkn_g_op);
clkgate #(1) clkgate4 (clk, clkn, clken_vec_op1 & clken_cmem_op, rstn, clk_g_op);
//assign clkn_g_op = (clken_vec_op & clken_cmem_op)? clkn:1'b0;
//assign clk_g_op = (clken_vec_op & clken_cmem_op)? clk:1'b0;

/*assign clk_loop_end_op_en = (start_exec_shifted & (addr_shifted_op == 'b0)) ? 1'b1:1'b0;
clkgate #(1) clkgate11 (clk, clkn, clk_loop_end_op_en, rstn, clk_loop_end_op);

always_ff @(posedge clk_loop_end_op or negedge rstn) begin
    if (!rstn)
        loop_end_op <= 'b0;
    else begin
	loop_end_op <= data_out_cmem_op[12+CM_DEPTH_OPERATION_BITS-1:12];
    end
end*/
assign loop_end_op = loop_end_r;

always_ff @(posedge clkn_g_op or negedge rstn) begin
    if (!rstn)
        addr_shifted_op <= {CM_DEPTH_OPERATION_BITS+1{1'b0}};
    else begin
        if (chip_en) begin
            if (start_exec_shifted_shifted == 1'b1 && trigger_op != 1'b0) begin
		if ((addr_shifted_op >= loop_end_op) ) begin
                    addr_shifted_op <= 'b0;
                end else begin
                    addr_shifted_op <= addr_shifted_op + 1'b1;
                end
            end else if ((start_exec == 1'b1 && start_exec_shifted == 1'b0) || trigger_op == 1'b0) begin
                addr_shifted_op <=  {CM_DEPTH_OPERATION_BITS+1{1'b0}};
            end else begin
		if (address_in[18:17] == 2'b00) begin
                	addr_shifted_op <= {1'b0,address_in[CM_DEPTH_OPERATION_BITS+2:3]};
		end
		else
		begin
			addr_shifted_op <= {CM_DEPTH_OPERATION_BITS+1{1'b0}};
		end
            end
        end
    end
end

assign clk_g_op_gated = clk_g_op & clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]];
always_ff @(posedge clk_g_op or negedge rstn) begin
    if (!rstn)
        addr_shifted_op_reg <= 'b0;
    else begin
	addr_shifted_op_reg <= addr_shifted_op;
    end
end

assign addr_shifted_op_to_mem = (clken_cmem_op_cycle[addr_shifted_op[CM_DEPTH_OPERATION_BITS-1:0]]) ?  addr_shifted_op : addr_shifted_op_reg;

logic [VEC_WIDTH_BITS-1 :0] trigger_count_E;
logic [VEC_WIDTH_BITS-1 :0] trigger_count_S;
logic [VEC_WIDTH_BITS-1 :0] trigger_count_W;
logic [VEC_WIDTH_BITS-1 :0] trigger_count_N;

//assign trigger_count_E = ((trigger_data_in[2:0] + {1'b0,data_out_cmem_r[5:4]}) < vec_size ) ? (trigger_data_in[2:0] + {1'b0,data_out_cmem_r[5:4]}): vec_size - (trigger_data_in[2:0] + {1'b0,data_out_cmem_r[5:4]});
//assign trigger_count_S = ((trigger_data_in[5:3] + {1'b0,data_out_cmem_r[11:10]}) < vec_size ) ? (trigger_data_in[5:3] + {1'b0,data_out_cmem_r[11:10]}): vec_size - (trigger_data_in[5:3] + {1'b0,data_out_cmem_r[11:10]});
//assign trigger_count_N = ((trigger_data_in[8:6] + {1'b0,data_out_cmem_r[17:16]}) < vec_size ) ? (trigger_data_in[8:6] + {1'b0,data_out_cmem_r[17:16]}): vec_size - (trigger_data_in[8:6] + {1'b0,data_out_cmem_r[17:16]});
//assign trigger_count_W = ((trigger_data_in[11:9] + {1'b0,data_out_cmem_r[23:22]}) < vec_size ) ? (trigger_data_in[11:9] + {1'b0,data_out_cmem_r[23:22]}): vec_size - (trigger_data_in[11:9] + {1'b0,data_out_cmem_r[23:22]});
assign trigger_count_E = {data_out_cmem_r[6:4]};
assign trigger_count_S = {data_out_cmem_r[13:11]};
assign trigger_count_N = {data_out_cmem_r[20:18]};
assign trigger_count_W = {data_out_cmem_r[27:25]};

assign trigger_N = ((vec_counter_r == trigger_count_N) || (!start_exec_shifted_shifted)) ? 1'b1:1'b0;
assign trigger_W = ((vec_counter_r == trigger_count_W) || (!start_exec_shifted_shifted)) ? 1'b1:1'b0;
assign trigger_S = ((vec_counter_r == trigger_count_S) || (!start_exec_shifted_shifted)) ? 1'b1:1'b0;
assign trigger_E = ((vec_counter_r == trigger_count_E) || (!start_exec_shifted_shifted)) ? 1'b1:1'b0;

assign clk_E = clk & clken_cmem_r & clk_en_spatial & trigger_E ;
assign clk_S = clk & clken_cmem_r & clk_en_spatial & trigger_S ;
assign clk_W = clk & clken_cmem_r & clk_en_spatial & trigger_W ;
assign clk_N = clk & clken_cmem_r & clk_en_spatial & trigger_N ;

always_ff @(posedge clk_E or negedge rstn) begin
    if (!rstn)
        control_reg_data_r_reg[3:0] <= {4{1'b0}};
    else begin
	control_reg_data_r_reg[3:0] <= data_out_cmem_r[3:0];
    end
end

always_ff @(posedge clk_S or negedge rstn) begin
    if (!rstn)
        control_reg_data_r_reg[7:4] <= {4{1'b0}};
    else begin
	control_reg_data_r_reg[7:4] <= data_out_cmem_r[10:7];
    end
end

always_ff @(posedge clk_W or negedge rstn) begin
    if (!rstn)
        control_reg_data_r_reg[11:8] <= {4{1'b0}};
    else begin
	control_reg_data_r_reg[11:8] <= data_out_cmem_r[17:14];
    end
end

always_ff @(posedge clk_N or negedge rstn) begin
    if (!rstn)
        control_reg_data_r_reg[15:12] <= {4{1'b0}};
    else begin
	control_reg_data_r_reg[15:12] <= data_out_cmem_r[24:21];
    end
end

    

IN22FDX_R1PV_NFVG_W00008B024M02C256 control_mem_op(
.CLK(clk_g_op_gated),
.CEN(1'b0),
.RDWEN(wr_en_shifted),
.DEEPSLEEP(1'b0),
.POWERGATE(1'b0),
.AW(addr_shifted_op_to_mem[1:0]),
.AC(addr_shifted_op_to_mem[2]),
.D(cm_data_shifted[CM_WIDTH_ROUTING_MEM+CM_WIDTH_OPERATION-1:CM_WIDTH_ROUTING_MEM]),
.BW(cm_bit_en_shifted[CM_WIDTH_ROUTING_MEM+CM_WIDTH_OPERATION-1:CM_WIDTH_ROUTING_MEM]),
.T_LOGIC(1'b0),
.MA_SAWL(1'b0),
.MA_WL(1'b0),
.MA_WRAS(1'b0),
.MA_WRASD(1'b0),
.Q(data_out_cmem_op),
.OBSV_CTL()
);

IN22FDX_R1PV_NFVG_W00008B028M02C256 control_mem_r(
.CLK(clk_g_r),
.CEN(1'b0),
.RDWEN(wr_en_shifted),
.DEEPSLEEP(1'b0),
.POWERGATE(1'b0),
.AW(addr_shifted_r[1:0]),
.AC(addr_shifted_r[2]),
.D(cm_data_shifted[CM_WIDTH_ROUTING_MEM-1:0]),
.BW(cm_bit_en_shifted[CM_WIDTH_ROUTING_MEM-1:0]),
.T_LOGIC(1'b0),
.MA_SAWL(1'b0),
.MA_WL(1'b0),
.MA_WRAS(1'b0),
.MA_WRASD(1'b0),
.Q(data_out_cmem_r),
.OBSV_CTL()
);

//assign control_reg_data_op = (chip_en&&start_exec_shifted) ? {data_out_cmem_op[CM_WIDTH_OPERATION-1:12],control_reg_data_op_reg} :'b000000000000111111111111 ;
assign control_reg_data_op = (chip_en&&start_exec_shifted&&clken_cmem_op_cycle[addr_shifted_op_reg[CM_DEPTH_OPERATION_BITS-1:0]]) ? data_out_cmem_op[CM_WIDTH_OPERATION-1:0] :'b000000000000111111111111 ;
assign control_reg_data_r = (chip_en&&start_exec_shifted) ? control_reg_data_r_reg :'b1111111111111111 ;
assign clk_neg_nop = clk_g_op;
endmodule
