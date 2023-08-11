`include "globals_top.vh"
//`include "globals_network.vh"

//------------------------------------------------------------------------------
// SMART Router
//------------------------------------------------------------------------------
module router(
    clk,
    clkn,
    reset,

    i__sram_xbar_sel,
    start_exec_shifted,

    i__flit_in_0,
    i__flit_in_1,
    i__flit_in_2,
    i__flit_in_3,
    i__flit_in_l_0,
    i__flit_in_l_1,
    i__flit_in_l_2,
    i__flit_in_l_3,
    o__flit_out_l_0,
    o__flit_out_l_1,
    o__flit_out_l_2,
    o__flit_out_l_3,
    i__alu_out,
    i__treg,
    i__treg1,
    o__flit_out,
    regbypass
);

//------------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------
import SMARTPkg::*;

parameter DATA_WIDTH			= 16;
parameter NUM_VCS                       = 4;

localparam NUM_INPUT_PORTS              = 7;
localparam NUM_OUTPUT_PORTS             = 7;
localparam LOG_NUM_VCS                  = $clog2(NUM_VCS);
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// IO
//------------------------------------------------------------------------------

// General
input  logic                            clk,clkn;
input  logic                            reset;

input  logic [NUM_INPUT_PORTS-1:0]	i__sram_xbar_sel	        [NUM_OUTPUT_PORTS-1:0];

// Incoming flit and outgoing credit

input  FlitFixed         i__flit_in_0                      ;
input  FlitFixed         i__flit_in_1                      ;
input  FlitFixed         i__flit_in_2                      ;
input  FlitFixed         i__flit_in_3                      ;
input  FlitFixed         i__flit_in_l_0                      ;
input  FlitFixed         i__flit_in_l_1                      ;
input  FlitFixed         i__flit_in_l_2                      ;
input  FlitFixed         i__flit_in_l_3                      ;
output FlitFixed     o__flit_out_l_0;
output FlitFixed     o__flit_out_l_1;
output FlitFixed     o__flit_out_l_2;
output FlitFixed     o__flit_out_l_3;

input  FlitFixed         i__alu_out;
input  FlitFixed         i__treg;
input  FlitFixed         i__treg1;


// Outgoing flit and incoming credit
output FlitFixed         o__flit_out                     [NUM_OUTPUT_PORTS-1:0];

input  [3:0] 				regbypass;
input start_exec_shifted;
//input  [3:0] 				regWEN;

//------------------------------------------------------------------------------
// Internal signals
//------------------------------------------------------------------------------

FlitFixed                w__flit_xbar_flit_out                   [NUM_OUTPUT_PORTS-1:0];

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Submodules
//------------------------------------------------------------------------------
    logic [NUM_INPUT_PORTS-1:0]         this_flit_xbar_sel              [NUM_OUTPUT_PORTS-1:0];
    FlitFixed                           this_flit_out_local             [NUM_INPUT_PORTS-1-2:0];
    FlitFixed                           this_flit_in                    [NUM_INPUT_PORTS-1:0];
    FlitFixed                           this_flit_xbar_flit_out         [NUM_OUTPUT_PORTS-1:0];

//Debug
logic [DATA_WIDTH:0] this_flit_in_E;
logic [DATA_WIDTH:0] this_flit_in_S;
logic [DATA_WIDTH:0] this_flit_in_W;
logic [DATA_WIDTH:0] this_flit_in_N;
logic [DATA_WIDTH:0] this_flit_in_ALU;
logic [DATA_WIDTH:0] this_flit_in_TREG;
assign this_flit_in_E = this_flit_in[0];
assign this_flit_in_S = this_flit_in[1];
assign this_flit_in_W = this_flit_in[2];
assign this_flit_in_N = this_flit_in[3];
assign this_flit_in_ALU = this_flit_in[4];
assign this_flit_in_TREG = this_flit_in[5];
logic [DATA_WIDTH:0] this_flit_out_R0;
logic [DATA_WIDTH:0] this_flit_out_R1;
logic [DATA_WIDTH:0] this_flit_out_R2;
logic [DATA_WIDTH:0] this_flit_out_R3;
assign this_flit_out_R0 = this_flit_out_local[0];
assign this_flit_out_R3 = this_flit_out_local[1];
assign this_flit_out_R1 = this_flit_out_local[2];
assign this_flit_out_R2 = this_flit_out_local[3];

    always_comb
    begin
	/*if(~start_exec_shifted)
	begin
		for(int j = 0; j < NUM_INPUT_PORTS; j++)
       	 	begin
            		this_flit_in[j] = {DATA_WIDTH+1{1'b0}};
        	end

	end
	else begin*/
        //for(int j = 0; j < NUM_INPUT_PORTS-2; j++)
        //begin
	    this_flit_in[0] 		= i__flit_in_0;
	    this_flit_in[1] 		= i__flit_in_1;
	    this_flit_in[2] 		= i__flit_in_2;
	    this_flit_in[3] 		= i__flit_in_3;
        //end
	    this_flit_in[ALU_T] 	= i__alu_out;
	    this_flit_in[TREG]  	= i__treg;
	    this_flit_in[LOCAL]  	= i__treg1;


        for(int j = 0; j < NUM_OUTPUT_PORTS; j++)
        begin
            for(int k = 0; k < NUM_INPUT_PORTS; k++)
            begin
                this_flit_xbar_sel[j][k] = i__sram_xbar_sel[j][k];
            end
        end
	//end
    end

    always_comb
    begin
  	    this_flit_out_local[0] 		= i__flit_in_l_0;
	    this_flit_out_local[1] 		= i__flit_in_l_1;
	    this_flit_out_local[2] 		= i__flit_in_l_2;
	    this_flit_out_local[3] 		= i__flit_in_l_3;
    end

logic clk_en_0,clk_en_1,clk_en_2,clk_en_3;
logic clk_g_0, clk_g_1,clk_g_2,clk_g_3;
assign clk_en_0 = ((this_flit_xbar_sel[0] == 'b000000)&&(start_exec_shifted)) ? 1'b0:1'b1;
assign clk_en_1 = ((this_flit_xbar_sel[1] == 'b000000)&&(start_exec_shifted)) ? 1'b0:1'b1;
assign clk_en_2 = ((this_flit_xbar_sel[2] == 'b000000)&&(start_exec_shifted)) ? 1'b0:1'b1;
assign clk_en_3 = ((this_flit_xbar_sel[3] == 'b000000)&&(start_exec_shifted)) ? 1'b0:1'b1;

clkgate #(1) clkgate0 (clk, clkn,clk_en_0 , rstn, clk_g_0);
clkgate #(1) clkgate1 (clk, clkn,clk_en_1 , rstn, clk_g_1);
clkgate #(1) clkgate2 (clk, clkn,clk_en_2 , rstn, clk_g_2);
clkgate #(1) clkgate3 (clk, clkn,clk_en_3 , rstn, clk_g_3);

always @(posedge clk_g_0)
if (reset)
begin
	o__flit_out_l_0 <= {DATA_WIDTH+1{1'b0}};
end
else 
begin
	o__flit_out_l_0 <= w__flit_xbar_flit_out[0];
end

always @(posedge clk_g_1)
if (reset)
begin
	o__flit_out_l_1 <= {DATA_WIDTH+1{1'b0}};
end
else 
begin
	o__flit_out_l_1 <= w__flit_xbar_flit_out[1];
end

always @(posedge clk_g_2)
if (reset)
begin
	o__flit_out_l_2 <= {DATA_WIDTH+1{1'b0}};
end
else 
begin
	o__flit_out_l_2 <= w__flit_xbar_flit_out[2];
end

always @(posedge clk_g_3)
if (reset)
begin
	o__flit_out_l_3 <= {DATA_WIDTH+1{1'b0}};
end
else 
begin
	o__flit_out_l_3 <= w__flit_xbar_flit_out[3];
end


always_comb
    begin
        for(int j = 0; j < NUM_OUTPUT_PORTS; j++)
        begin
            w__flit_xbar_flit_out[j] = this_flit_xbar_flit_out[j];
        end
    end

    xbar_bypass  
        #(
            .DATA_WIDTH                 ($bits(FlitFixed))
        )
        xbar_bypass(
            .i__sel                     (this_flit_xbar_sel),
            .i__data_in_local           (this_flit_out_local),
            .i__data_in_remote          (this_flit_in),
	    .regbypass			(regbypass),
            .o__data_out                (this_flit_xbar_flit_out)
        );

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Flit to neighbor router
//------------------------------------------------------------------------------
always_comb
begin
    for(int i = 0; i < (NUM_OUTPUT_PORTS); i++)
    begin
	/*if (~start_exec_shifted)
	begin
		o__flit_out[i] = {DATA_WIDTH+1{1'b0}};
	end
	else begin*/
        	o__flit_out[i] = w__flit_xbar_flit_out[i];
	//end
    end
end
//------------------------------------------------------------------------------


endmodule


