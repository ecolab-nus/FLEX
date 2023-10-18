`ifndef _pace_rw_task_svh_
`define _pace_rw_task_svh_


logic chip_en;
logic scan_data_or_addr;
logic read_write;
logic data_out_valid;
logic [1:0] data_addr_valid;
logic scan_start_exec;
logic exec_end;
logic cm_en;

logic [`DATA_WIDTH-1:0] data_in;
logic [2:0] addr_cmem;
logic [`ADDR_WIDTH-1:0] address_in;
logic [`DATA_WIDTH-1:0] data_out;
logic [3:0] trigger [3:0];

int num_inst = 0;
bit success = 1;
bit test_word_success = 1;
logic [`DATA_WIDTH-1:0] memory_dataSRAM [`TB_NUM_INST-1:0];
logic [`ADDR_WIDTH-1:0] memory_addrSRAM [`TB_NUM_INST-1:0];
logic [`DATA_WIDTH-1:0] memory_dataSRAM_expected [`TB_NUM_DMEM_INST-1:0];

task load_SRAM;
    // addr SRAM
    @(posedge `HYC_TOP.clk);
    #0.1
    address_in <= memory_addrSRAM[num_inst];
    data_in <= memory_dataSRAM[num_inst];
    scan_start_exec <= 1'b0;
    chip_en <= 1'b1;
    read_write <= 1'b0;
    data_addr_valid <= 2'b11;

    @(posedge `HYC_TOP.clk);
    #0.1
    data_addr_valid <= 2'b00;
    @(posedge `HYC_TOP.clk);
    #0.1;
endtask

task read_CMEM(input [3:0] addr);
    // addr SRAM
    @(posedge `HYC_TOP.clk);
    #0.1
    address_in <= {{15{1'b0}},addr} ;
    cm_en <= 1'b0;

    @(posedge `HYC_TOP.clk);
    #0.1
    cm_en <= 1'b1;
    @(posedge `HYC_TOP.clk);
    #0.1;
endtask


task check_dataSRAM;
    // Specify read address
    @(posedge `HYC_TOP.clk);
    #0.1
    address_in <= memory_addrSRAM[`TB_NUM_CMEM_INST+num_inst];
    scan_start_exec <= 1'b0;
    chip_en <= 1'b1;
    read_write <= 1'b1;
    data_addr_valid <= 2'b11;

    @(posedge `HYC_TOP.data_out_valid);
    @(posedge `HYC_TOP.clk); 
    #0.1
    test_word_success = (data_out == memory_dataSRAM[`TB_NUM_CMEM_INST+num_inst]);
    if(!test_word_success) begin
        $error("Mismatch: address %b\nExpected: %b, Actual: %b\n",
        memory_addrSRAM[`TB_NUM_CMEM_INST+num_inst],
        memory_dataSRAM[`TB_NUM_CMEM_INST+num_inst],
        data_out);
    end
    success = success & test_word_success;

    @(posedge `HYC_TOP.clk);
    #0.1
    data_addr_valid <= 2'b00;
    @(posedge `HYC_TOP.clk);
    #0.1
    @(posedge `HYC_TOP.clk);
endtask


task verify_dataSRAM;
    // Specify read address
    @(posedge `HYC_TOP.clk);
    #0.1
    address_in <= memory_addrSRAM[`TB_NUM_CMEM_INST+num_inst];
    scan_start_exec <= 1'b0;
    chip_en <= 1'b1;
    read_write <= 1'b1;
    data_addr_valid <= 2'b11;

    @(posedge `HYC_TOP.data_out_valid);
    @(posedge `HYC_TOP.clk); 
    #0.1
    test_word_success = (data_out == memory_dataSRAM_expected[num_inst]);
    if(!test_word_success) begin
        $error("Mismatch: address %b\nExpected: %b, Actual: %b\n",
        memory_addrSRAM[`TB_NUM_CMEM_INST+num_inst],
        memory_dataSRAM_expected[num_inst],
        data_out);
    end
    success = success & test_word_success;

    @(posedge `HYC_TOP.clk);
    #0.1
    data_addr_valid <= 2'b00;
    @(posedge `HYC_TOP.clk);
    #0.1
    @(posedge `HYC_TOP.clk);
endtask

task initialize_testbench;
    input string tracedata;
    input string traceaddr;
    input string traceresults;
    integer tracefile_addr;
    integer tracefile_data;
    integer tracefile_results;

    $display("Initializing testbench...\n");
    
    tracefile_addr <= $fopen(traceaddr, "r");
    if(!tracefile_addr)
        $error("Trace file (%s) could not be opened.",traceaddr);
    else begin
        $readmemb(traceaddr, memory_addrSRAM);
    end

    tracefile_data <= $fopen(tracedata, "r");
    if(!tracefile_data)
        $error("Trace file (%s) could not be opened.",tracedata);
    else begin
        $readmemb(tracedata, memory_dataSRAM);
    end

    tracefile_results <= $fopen(traceresults, "r");
    if(!tracefile_results)
        $error("Trace file (%s) could not be opened.",traceresults);
    else begin
        $readmemb(traceresults, memory_dataSRAM_expected);
    end
endtask

`endif //_pace_rw_task_svh_

