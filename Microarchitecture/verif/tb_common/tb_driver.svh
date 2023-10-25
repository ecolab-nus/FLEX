`define INSTANTIATE_DUT \
    soc_pad soc_pad ( \
        .clk(tb_driver.clk), \
        .rst(~tb_driver.rstn), \
	.data_in(data_in), \
	.address_in(address_in), \
	.data_out(data_out), \
	.read_write(read_write), \
	.data_addr_valid(data_addr_valid), \
	.scan_start_exec(scan_start_exec), \
	.trigger(trigger), \
	.chip_en(chip_en), \
	.data_out_valid(data_out_valid), \
	.exec_end(exec_end));


module tb_driver ();
    logic                        clk;
    logic                        rstn; 
    /*logic [5:0]                  cfg_sel; 
    logic                        spi_ss;
    logic                        spi_sclk;
    wire  [3:0]                  spi_data;*/

    //Reset sequence completion signal
    logic rst_seq_done;

    //Assertion related parameters
    localparam ON=3, OFF=4;
    localparam UNIQUE=32, UNIQUE0=64, PRIORITY=128;

    //Primary clock driver
    initial begin
        clk = 1'bX;
        #((100+$urandom%100) * 1ns)
        clk = 1'b0;
        forever begin
            #(`SYS_CLK_PERIOD * 0.5 * 1ns) clk = ~clk;
        end
    end

    //Primary reset: rstn
    initial begin
        $assertcontrol( OFF , UNIQUE | UNIQUE0 | PRIORITY );
        rstn = 1'b0;
        #((($urandom%100)+`SYS_CLK_PERIOD * 50) * 1ns)
        rstn = 1'b1;
        $assertcontrol( ON , UNIQUE | UNIQUE0 | PRIORITY );
    end

    //Reset sequence completion calculation
    initial begin
        rst_seq_done = 1'b0;
        wait (`SOC_TOP.rstn_lock === 1'b1); 
        #(`SYS_CLK_PERIOD * 50 * 1ns)
        rst_seq_done = 1'b1;
    end

endmodule

