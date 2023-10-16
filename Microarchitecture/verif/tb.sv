// Simulation precision
`timescale 1 ns / 1 ps

`define TB_NUM_INST 1306
`define TB_NUM_DMEM_INST 1024
`define TB_NUM_CMEM_INST 128

`include "tb_defines.vh"
//`include "qspi_rw_task.svh"
`include "pace_rw_task.svh"
`include "tb_driver.svh"

module tb_top();

//------------------------------------------------------------------------------
// Module instantiation
//------------------------------------------------------------------------------

//Common testbench driver module
tb_driver tb_driver();

//SoC DUT
`INSTANTIATE_DUT

//------------------------------------------------------------------------------
// Simulation signals / registers
//------------------------------------------------------------------------------
integer num_sim_cycles = `TB_NUM_SIM_CYCLES;
event initial_signals;
event start_stimulus;
bit marker_reg = 0;

// Used to view expected results (results_expected.trc) in waveform
wire [15:0] dmem_expected;
assign dmem_expected = memory_dataSRAM_expected[num_inst];
//------------------------------------------------------------------------------
// Initial values
//------------------------------------------------------------------------------
initial begin: initial_signals_block
    @ (initial_signals);
        scan_data_or_addr = 1'b0;
        read_write = 1'b0;
        scan_start_exec = 1'b0;
end

//------------------------------------------------------------------------------
// Simulation control flow
//------------------------------------------------------------------------------
initial begin: simulation_control_flow
    initialize_testbench("$REPO_ROOT/verif/flex_fir_4x4_2/totaldata.trc",
                         "$REPO_ROOT/verif/flex_fir_4x4_2/totaladdr.trc",
                         "$REPO_ROOT/verif/flex_fir_4x4_2/results_expected.trc");


    /*force `SOC_TOP.hycube0.data_in = data_in;
    force `SOC_TOP.hycube0.address_in = address_in;
    force `SOC_TOP.hycube0.data_addr_valid = data_addr_valid;
    force `SOC_TOP.hycube0.scan_start_exec = scan_start_exec;
    force `SOC_TOP.hycube0.read_write = read_write;
    force `SOC_TOP.hycube0.trigger_op = trigger;
    force data_out_valid = `SOC_TOP.hycube0.data_out_valid;
    force data_out = `SOC_TOP.hycube0.data_out;
    force exec_end = `SOC_TOP.hycube0.exec_end;*/


    -> initial_signals;

    wait (tb_driver.rst_seq_done); //Wait for reset sequence to complete

    // Print instance
    #(`SYS_CLK_PERIOD*10);
    $display("TB_NUM_INST: %d\n", `TB_NUM_INST);

    // LOAD SRAM (CMEM and DMEM)
    #(`SYS_CLK_PERIOD*10);
    for(num_inst=0; num_inst < `TB_NUM_INST; num_inst++) begin
        load_SRAM;
    end
    $display("[%16d] Stage : MEM load completed\n", $realtime);

    #(`SYS_CLK_PERIOD*10);
    scan_start_exec = 1'b1;
    #(`SYS_CLK_PERIOD) 
    trigger[0]=4'b0100;
    trigger[1]=4'b0111;
    trigger[2]=4'b1001;
    trigger[3]=4'b1111;

    #(`SYS_CLK_PERIOD) 
    trigger[0]=4'b1111;
    trigger[1]=4'b1111;
    trigger[2]=4'b1111;
    trigger[3]=4'b1111;


    $display("[%16d] ASK : START EXEC\n", $realtime);

    @(posedge exec_end);
    $display("[%16d] ASK : END EXEC\n", $realtime);
    #(`SYS_CLK_PERIOD*24);
    scan_start_exec = 1'b0;

    $finish();
end

//TB timeout section
initial begin
    #0.1s
    $error("Error: Test timeout\n");
    $fatal;
end

endmodule
