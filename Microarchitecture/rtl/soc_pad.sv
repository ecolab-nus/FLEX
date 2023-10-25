`include "globals_top.vh"

module soc_pad (
    input                        clk,
    input                        rst,
    input [15:0]		data_in,
    output [15:0]		data_out,
    input [19:0]		address_in,
    input 			read_write,
    input [1:0]			data_addr_valid,
    input			scan_start_exec,
    input [3:0]			trigger [3:0],
    input			chip_en,
    output			data_out_valid,
    output			exec_end
);

    logic rstn;
    logic rstn_clk;
    logic rstn_lock;
    logic rstn_sync;
    assign rstn = ~rst;
    
    ff_rst #(2) ff_rst0 (clk, rstn, rstn, rstn_clk); 
    ff_rst #(4) ff_rst1 (clk, rstn, rstn_lock, rstn_sync); 

    flex flex0 (
        .clk(clk),
        .clkn(~clk),
        .rstn(rstn_sync),
        .chip_en(chip_en),
        .data_in(data_in),
        .address_in(address_in[18:0]),
        .data_out(data_out),
        .read_write(read_write),
        .data_out_valid(data_out_valid),
        .data_addr_valid(data_addr_valid),
        .scan_start_exec(scan_start_exec),
        .exec_end(exec_end),
	.trigger_op(trigger)
    );
  
 
    assign rstn_lock = rstn;


endmodule

