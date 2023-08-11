module ls_add_unit(
	op_LHS,
       	op_const_ls,
       	op_const_add,
	vec_counter,
       	result
);

parameter DATA_WIDTH = 8;
parameter VEC_WIDTH = 8;
localparam VEC_WIDTH_BITS		= $clog2(VEC_WIDTH);

input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH-1:0] op_const_ls;
input [DATA_WIDTH-1:0] op_const_add;
input logic [VEC_WIDTH_BITS-1:0 ] vec_counter; 

output logic [DATA_WIDTH:0] result;

logic [DATA_WIDTH-1:0] op_ls_out;
logic [DATA_WIDTH-1:0] temp;

assign op_ls_out = op_LHS[DATA_WIDTH-1:0] << op_const_ls;
//assign temp =(op_LHS[DATA_WIDTH]) ? (op_ls_out + op_const_add + {{DATA_WIDTH-VEC_WIDTH_BITS-1{1'b0}},vec_counter<<1}): 'b0;
assign temp =(op_LHS[DATA_WIDTH]) ? (op_ls_out + op_const_add): 'b0;
assign result = {op_LHS[DATA_WIDTH],temp};
endmodule
