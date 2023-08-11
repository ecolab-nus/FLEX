//******************************
// SIMPLE ALU
//******************************

//`define MUX_ALU

module simple_alu(
    operation,
    op_RHS,
    op_LHS,
    op_SHIFT,
    op_predicate,
    result
);
    parameter DATA_WIDTH = 16;

    input logic [5:0] operation;
    input logic [DATA_WIDTH:0] op_RHS;
    input logic [DATA_WIDTH:0] op_LHS;
    input logic [DATA_WIDTH:0] op_SHIFT;
    input logic [DATA_WIDTH-1:0] op_predicate;

    output logic [DATA_WIDTH-1:0] result;

`ifdef MUX_ALU
    mux_alu #(
        .DATA_WIDTH(DATA_WIDTH)
    ) mux_alu_inst (
        .operation(operation),
        .op_RHS(op_RHS),
        .op_LHS(op_LHS),
        .op_SHIFT(op_SHIFT),
        .op_predicate(op_predicate),
        .result(result)
    );
`else
    generic_alu #(
        .DATA_WIDTH(DATA_WIDTH)
    ) generic_alu_inst (
        .operation(operation),
        .op_RHS(op_RHS),
        .op_LHS(op_LHS),
        .op_SHIFT(op_SHIFT),
        .op_predicate(op_predicate),
        .result(result)
    );
`endif

endmodule

//******************************
// GENERIC ALU
//******************************
module generic_alu(
    operation,
    op_RHS,
    op_LHS,
    op_SHIFT,
    op_predicate,
    result
);
/*
    parameter DATA_WIDTH = 16;
    parameter OP_MAX = 32;

    input logic [5:0] operation;
    input logic [DATA_WIDTH:0] op_RHS;
    input logic [DATA_WIDTH:0] op_LHS;
    input logic [DATA_WIDTH:0] op_SHIFT;
    input logic [DATA_WIDTH-1:0] op_predicate;

    output logic [DATA_WIDTH-1:0] result;

    reg overflow;

    wire [DATA_WIDTH:0] op_2;
    assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

    always_comb begin: alu
        case(operation[4:0])
            5'b00000: result = {DATA_WIDTH{1'b0}};//nop
            5'b00001: {overflow,result} = (op_RHS[DATA_WIDTH-1:0] + op_2[DATA_WIDTH-1:0]); //add
            5'b00010: {overflow,result} = (op_RHS[DATA_WIDTH-1:0] - op_2[DATA_WIDTH-1:0]) ; //sub
            5'b00011: result = ($signed(op_RHS[DATA_WIDTH-1:0]) * $signed(op_2[DATA_WIDTH-1:0])) ; //mult
            //5'b00101: result = ($signed(op_RHS[DATA_WIDTH-1:0]) / $signed(op_2[DATA_WIDTH-1:0])); //div
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
                end else begin
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
*/

/*
`ifndef SIMULATION
    always @(posedge resetn or clock)
    if rst
    else begin
        if (result != a+b)
            $error("ERROR: Incorrect calc");
    end
`endif
*/

parameter DATA_WIDTH = 16;

input [DATA_WIDTH:0] op_RHS;
input [DATA_WIDTH:0] op_LHS;
input [DATA_WIDTH:0] op_SHIFT;
input [DATA_WIDTH-1:0] op_predicate;

input [5:0] operation;

output [DATA_WIDTH-1:0] result;

reg [DATA_WIDTH-1:0] result;
reg overflow;

wire [DATA_WIDTH:0] op_2;
assign op_2 = (operation[5]) ? op_SHIFT : op_LHS;

always_comb
begin: alu
case(operation[4:0])
	5'b00000: result = {DATA_WIDTH{1'b0}};//nop
	5'b00001: {overflow,result} = (op_RHS[DATA_WIDTH-1:0] + op_2[DATA_WIDTH-1:0]); //add
	5'b00010: {overflow,result} = (op_RHS[DATA_WIDTH-1:0] - op_2[DATA_WIDTH-1:0]) ; //sub
	5'b00011: result = ($signed(op_RHS[DATA_WIDTH-1:0]) * $signed(op_2[DATA_WIDTH-1:0])) ; //mult
	5'b00101: result = ($signed(op_RHS[DATA_WIDTH-1:0]) / $signed(op_2[DATA_WIDTH-1:0])); //div
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
        5'b11000: result = op_SHIFT[DATA_WIDTH-1:0] + op_RHS[DATA_WIDTH-1:0]; //accum
	default: result = {DATA_WIDTH{1'b0}};
endcase
end

/*
`ifndef SIMULATION
	always @(posedge resetn or clock)
	if rst
	else begin
		if (result != a+b)
			$error("ERROR: Incorrect calc");
	end
`endif
*/

endmodule

//******************************
// MUX ALU
//******************************
module mux_alu(
    operation,
    op_RHS,
    op_LHS,
    op_SHIFT,
    op_predicate,
    result
);
    parameter DATA_WIDTH = 16;
    parameter OP_MAX = 32;

    input logic [5:0] operation;
    input logic [DATA_WIDTH:0] op_RHS;
    input logic [DATA_WIDTH:0] op_LHS;
    input logic [DATA_WIDTH:0] op_SHIFT;
    input logic [DATA_WIDTH-1:0] op_predicate;

    output logic [DATA_WIDTH-1:0] result;
    logic overflow;

    // Data Mux
    logic [DATA_WIDTH:0]   op_tmp           [0:OP_MAX-1];
    logic [DATA_WIDTH:0]   op_RHS_tmp       [0:OP_MAX-1];
    logic [DATA_WIDTH:0]   op_LHS_tmp       [0:OP_MAX-1];
    logic [DATA_WIDTH:0]   op_SHIFT_tmp     [0:OP_MAX-1];
    logic [DATA_WIDTH-1:0] op_predicate_tmp [0:OP_MAX-1];

    always_comb begin
        for (int ii = 0; ii < OP_MAX; ii = ii + 1) begin
            if (operation[4:0] == ii) begin
                op_tmp[ii]           = (operation[5]) ? op_SHIFT : op_LHS;
                op_RHS_tmp[ii]       = op_RHS;
                op_LHS_tmp[ii]       = op_LHS;
                op_SHIFT_tmp[ii]     = op_SHIFT;
                op_predicate_tmp[ii] = op_predicate;
            end else begin
                op_tmp[ii]           = {(DATA_WIDTH+1){1'b0}};
                op_RHS_tmp[ii]       = {(DATA_WIDTH+1){1'b0}};
                op_LHS_tmp[ii]       = {(DATA_WIDTH+1){1'b0}};
                op_SHIFT_tmp[ii]     = {(DATA_WIDTH+1){1'b0}};
                op_predicate_tmp[ii] = {(DATA_WIDTH){1'b0}};
            end
        end
    end

    // Result Mux
    logic [DATA_WIDTH-1:0] result_tmp   [0:OP_MAX-1];
    logic                  overflow_tmp [0:OP_MAX-1];

    always_comb begin
        result   = result_tmp[operation[4:0]];
        overflow = overflow_tmp[operation[4:0]];
    end

    // ALU core
    always_comb begin
        for (int ii = 0; ii < OP_MAX; ii = ii + 1) begin
            {overflow_tmp[ii], result_tmp[ii]} = {1'b0, {DATA_WIDTH{1'b0}}};
        end

        {overflow_tmp[0], result_tmp[0]} = {1'b0, {DATA_WIDTH{1'b0}}};
        {overflow_tmp[1], result_tmp[1]} = (op_RHS_tmp[1][DATA_WIDTH-1:0] + op_tmp[1][DATA_WIDTH-1:0]);
        {overflow_tmp[2], result_tmp[2]} = (op_RHS_tmp[2][DATA_WIDTH-1:0] - op_tmp[2][DATA_WIDTH-1:0]);
        {overflow_tmp[3], result_tmp[3]} = {1'b0, ($signed(op_RHS_tmp[3][DATA_WIDTH-1:0]) * $signed(op_tmp[3][DATA_WIDTH-1:0]))};
	{overflow_tmp[5], result_tmp[5]} = {1'b0, ($signed(op_RHS_tmp[3][DATA_WIDTH-1:0]) / $signed(op_tmp[3][DATA_WIDTH-1:0]))};
        {overflow_tmp[8], result_tmp[8]} = {1'b0, (op_RHS_tmp[8][DATA_WIDTH-1:0] << op_tmp[8][DATA_WIDTH-1:0])};
        {overflow_tmp[9], result_tmp[9]} = {1'b0, (op_RHS_tmp[9][DATA_WIDTH-1:0] >> op_tmp[9][DATA_WIDTH-1:0])};
        {overflow_tmp[10], result_tmp[10]} = {1'b0, (op_RHS_tmp[10][DATA_WIDTH-1:0] >>> op_tmp[10][DATA_WIDTH-1:0])};
        {overflow_tmp[11], result_tmp[11]} = {1'b0, (op_RHS_tmp[11][DATA_WIDTH-1:0] & op_tmp[11][DATA_WIDTH-1:0])};
        {overflow_tmp[12], result_tmp[12]} = {1'b0, (op_RHS_tmp[12][DATA_WIDTH-1:0] | op_tmp[12][DATA_WIDTH-1:0])};
        {overflow_tmp[13], result_tmp[13]} = {1'b0, (op_RHS_tmp[13][DATA_WIDTH-1:0] ^ op_tmp[13][DATA_WIDTH-1:0])};

        overflow_tmp[16] = 1'b0;
        if (operation[5]==1'b0) begin
            if (op_LHS_tmp[16][DATA_WIDTH] == 1'b1)
                result_tmp[16] = op_LHS_tmp[16][DATA_WIDTH-1:0]; //select
            else if (op_RHS_tmp[16][DATA_WIDTH] == 1'b1)
                result_tmp[16] = op_RHS_tmp[16][DATA_WIDTH-1:0]; //select
            else
                result_tmp[16] = {DATA_WIDTH{1'b0}};
        end else begin
            result_tmp[16] = op_SHIFT_tmp[16][DATA_WIDTH-1:0];
        end

        overflow_tmp[17] = 1'b0;
        if (operation[5]==1'b0)
            result_tmp[17] = op_RHS_tmp[17][DATA_WIDTH-1:0]; //cmerge
        else
            result_tmp[17] = op_SHIFT_tmp[17][DATA_WIDTH-1:0]; //cmerge

        {overflow_tmp[18], result_tmp[18]} = {1'b0,
            {{DATA_WIDTH-1{1'b0}}, (op_RHS_tmp[18][DATA_WIDTH-1:0] == op_tmp[18][DATA_WIDTH-1:0])}};
        {overflow_tmp[19], result_tmp[19]} = {1'b0,
            {{DATA_WIDTH-1{1'b0}}, ($signed(op_RHS_tmp[19][DATA_WIDTH-1:0]) < $signed(op_tmp[19][DATA_WIDTH-1:0]))}};
        {overflow_tmp[20], result_tmp[20]} = {1'b0,
            (op_predicate_tmp[20][DATA_WIDTH-1:0] | op_RHS_tmp[20][DATA_WIDTH-1:0] | op_tmp[20][DATA_WIDTH-1:0])};
        {overflow_tmp[21], result_tmp[21]} = {1'b0,
            {{DATA_WIDTH-1{1'b0}}, ($signed(op_tmp[21][DATA_WIDTH-1:0]) < $signed(op_RHS_tmp[21][DATA_WIDTH-1:0]))}};
        {overflow_tmp[31], result_tmp[31]} = {1'b0, op_tmp[31][DATA_WIDTH-1:0]};
	{overflow_tmp[24], result_tmp[1]} = (op_RHS_tmp[1][DATA_WIDTH-1:0] + op_SHIFT_tmp[1][DATA_WIDTH-1:0]);

    end

endmodule
