module iterCounter(
	clk,
	rstn,
	start_exec_shifted,
	start_exec,
	wr_en,
	chip_en,
	loop_end,
	configuration,
	iterValue,
	iterValue_reg,
	vec_size,
	exec_end,
	addr_cmem,
	vec_counter,
	clk_g
);

parameter CONFIG_MEM_BITS 	= 3;
parameter ITER_RANGE_BITS 	= 32;
parameter STRIDE_BITS		= 2;
parameter CONFIG_WIDTH 		= STRIDE_BITS + ITER_RANGE_BITS ;
parameter VEC_WIDTH			= 4;
localparam VEC_WIDTH_BITS		= $clog2(VEC_WIDTH);

//----------------------------------------------------------------------------
//--------------------------- INPUTS -----------------------------------------
//----------------------------------------------------------------------------

input clk;
input rstn;
input start_exec_shifted,start_exec;
input wr_en;
input chip_en;
input [CONFIG_WIDTH-1:0] configuration;
input [CONFIG_MEM_BITS-1:0] loop_end;
input [VEC_WIDTH_BITS-1:0] vec_size;

//----------------------------------------------------------------------------
//--------------------------- OUTPUTS ----------------------------------------
//----------------------------------------------------------------------------

output [ITER_RANGE_BITS:0] iterValue;
output logic [ITER_RANGE_BITS:0] iterValue_reg;
output exec_end;

//----------------------------------------------------------------------------
//--------------------------- OTHER ------------------------------------------
//----------------------------------------------------------------------------

logic [STRIDE_BITS-1:0] stride;
logic [ITER_RANGE_BITS-1:0] maxCount;
logic [CONFIG_WIDTH-1:0] configuration_reg;
logic [ITER_RANGE_BITS-1:0] iterValue_regx;

//----------------------------------------------------------------------------
//--------------------------- ITERATE CONFIGS ----------------------------------
//----------------------------------------------------------------------------
input logic [CONFIG_MEM_BITS:0] addr_cmem;
//logic clken_vec;
input logic [VEC_WIDTH_BITS-1:0 ] vec_counter;
input clk_g;
//logic clk_g,clkn_g;

/*clkgate #(1) clkgate3 (clk, clkn, clken_vec, rstn, clk_g);
clkgate #(1) clkgate4 (clkn, clk, clken_vec, rstn, clkn_g);


assign clken_vec = (!start_exec_shifted)? 1'b1: (vec_counter == vec_size)? 1'b1:1'b0;

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn)
        vec_counter <= {VEC_WIDTH_BITS{1'b0}};
    else begin
        if (chip_en && start_exec_shifted == 1'b1) begin
	   if ((vec_counter == vec_size)) begin
              vec_counter <= {VEC_WIDTH_BITS{1'b0}};
           end else begin
              vec_counter <= vec_counter + 1'b1;
           end
        end else begin
	   vec_counter <= {VEC_WIDTH_BITS{1'b0}};
        end
    end
end

always_ff @(posedge clk_g or negedge rstn) begin
    if (!rstn)
        addr_cmem <= {CONFIG_MEM_BITS+1{1'b0}};
    else begin
        if (chip_en) begin
            if (start_exec_shifted == 1'b1) begin
		if ((addr_cmem >= loop_end) || (addr_cmem < loop_start)) begin
                    addr_cmem <= loop_start;
                end else begin
                    addr_cmem <= addr_cmem + 1'b1;
                end
            end 	   
	    else
	        addr_cmem <= {CONFIG_MEM_BITS+1{1'b0}};		
        end
    end
end*/

//----------------------------------------------------------------------------
//--------------------------- CONFIGURATION ----------------------------------
//----------------------------------------------------------------------------

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn)
            configuration_reg <= {CONFIG_WIDTH{1'b0}};
    else begin
        if (chip_en && !start_exec_shifted && wr_en)
            configuration_reg <= configuration;
    end
end

assign maxCount	= configuration_reg[ITER_RANGE_BITS-1:0];
assign stride = configuration_reg[CONFIG_WIDTH-1:ITER_RANGE_BITS];

//----------------------------------------------------------------------------
//--------------------------- COUNTER ----------------------------------------
//----------------------------------------------------------------------------

always_ff @(posedge clk_g or negedge rstn) begin
    if (!rstn)
        iterValue_regx <= {ITER_RANGE_BITS{1'b1}};
    else begin
        if (chip_en && start_exec_shifted) begin
		if((iterValue_regx == {ITER_RANGE_BITS{1'b1}})) begin
			iterValue_regx <= {ITER_RANGE_BITS{1'b0}};
		end
		else if ((addr_cmem == loop_end) && (iterValue_regx < maxCount)) begin // check addr_cmem && vec_count
                    iterValue_regx <= iterValue_regx + stride*(vec_size+1);
                end 
		else begin
                    iterValue_regx <= iterValue_regx;
                end
	end else if (chip_en && start_exec) begin
			iterValue_regx <= {ITER_RANGE_BITS{1'b0}};
	end else
	begin
		iterValue_regx <= {ITER_RANGE_BITS{1'b1}};
	end
    end
end

always_ff @(posedge clk_g or negedge rstn) begin
    if (!rstn)
        iterValue_reg <= {ITER_RANGE_BITS+1{1'b1}};
    else begin
        iterValue_reg <= {{iterValue_regx != {ITER_RANGE_BITS{1'b1}}},iterValue_regx};
    end
end
//assign iterValue_reg = {ITER_RANGE_BITS+1{1'b0}};


assign exec_end = (chip_en && start_exec_shifted && (iterValue_regx >= maxCount)) ? 1'b1 : 1'b0;
assign iterValue = (chip_en && start_exec_shifted && !exec_end) ?  {{iterValue_regx != {ITER_RANGE_BITS{1'b1}}},iterValue_regx} : {ITER_RANGE_BITS{1'b0}};
//assign iterValue = {ITER_RANGE_BITS+1{1'b0}};

endmodule
