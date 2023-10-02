<<<<<<< HEAD
## Copyright (C) A*STAR - All Rights Reserved
## Unauthorized copying of this file, via any medium is strictly prohibited
## Proprietary and confidential
## Owner(s): Lan Jingjing <Lan_Jingjing@ime.a-star.edu.sg>, Mar 2021
## Description: Constraints file for tile module
=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

## Timing unit
set_units -capacitance 1fF
set_units -time 1000.0ps

<<<<<<< HEAD
#TODO by LJJ, update constrain when specs are ready
=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
## Clock parameters
set clk_period                 10 
set clk_uncertainty            0.25

## Primary clock pin (clk)
if {[llength [get_pins clk -quiet]]} {
    puts "Primary clock pin detected (clk)"
    create_clock -period $clk_period -name clk [get_pins clk]
## Primary clock port (clk)
} elseif {[llength [get_port clk -quiet]]} {
    puts "Primary clock port detected (clk)"
    create_clock -period $clk_period -name clk [get_port clk]
}

<<<<<<< HEAD
### control_mem clock pin (clk_shifted1)
#if {[llength [get_pins clk_shifted1 -quiet]]} {
#    puts "control_mem generated clock pin detected (clk_shifted1)"
#    create_generated_clock -add -divide_by 1 -invert -name clk_pemem -master_clock clk -source [get_port clk] [get_pins clk_shifted1]
### control_mem clock port (clk_shifted1)
#} elseif {[llength [get_port clk_shifted1 -quiet]]} {
#    puts "control_mem generated clock port detected (clk_shifted1)"
#    create_generated_clock -add -divide_by 1 -invert -name clk_pemem -master_clock clk -source [get_port clk] [get_port clk_shifted1]
#}

=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
## control_mem clock pin (clkn)
if {[llength [get_pins clkn -quiet]]} {
    puts "control_mem generated clock pin detected (clkn)"
    create_generated_clock -add -divide_by 1 -invert -name clk_pemem -master_clock clk -source [get_port clk] [get_pins clkn]
## control_mem clock port (clkn)
} elseif {[llength [get_port clkn -quiet]]} {
    puts "control_mem generated clock port detected (clkn)"
    create_generated_clock -add -divide_by 1 -invert -name clk_pemem -master_clock clk -source [get_port clk] [get_port clkn]
}

if {[llength [get_clocks clk -quiet]]} {
    set_dont_touch_network [get_clocks clk]
    set_clock_uncertainty -setup $clk_uncertainty clk
    set_clock_uncertainty -hold  $clk_uncertainty clk
}

## IO delays - General
set_input_delay  -clock clk [expr $clk_period * 0.2] -max [all_inputs]
set_input_delay  -clock clk [expr $clk_period * 0.1] -min [all_inputs]
set_output_delay -clock clk [expr $clk_period * 0.2] -max [all_outputs]
set_output_delay -clock clk [expr $clk_period * 0.1] -min [all_outputs]

## IO delays - Crossbar
set_input_delay  -clock clk [expr $clk_period * 0.45] -max [get_ports [list data_in*]]
<<<<<<< HEAD
#set_input_delay  -clock clk [expr $clk_period * 0.40] -min [get_ports [list i__flit_in*]]
set_output_delay -clock clk [expr $clk_period * 0.45] -max [get_ports [list data_out*]]
#set_output_delay -clock clk [expr $clk_period * 0.40] -min [get_ports [list o__flit_out*]]
#set_max_delay 1 -from [get_ports [list i__flit_in*]] -to [get_ports [list o__flit_out*]]

## Harcoded SRAM configuration settings for timing analysis
#set_case_analysis 0 [get_ports creg_rf_cfg[10]]
#set_case_analysis 1 [get_ports creg_rf_cfg[9]]
#set_case_analysis 1 [get_ports creg_rf_cfg[8]]
#set_case_analysis 0 [get_ports creg_rf_cfg[7]]
#set_case_analysis 1 [get_ports creg_rf_cfg[6]]
#set_case_analysis 0 [get_ports creg_rf_cfg[5]]
#set_case_analysis 1 [get_ports creg_rf_cfg[4]]
#set_case_analysis 0 [get_ports creg_rf_cfg[3]]
#set_case_analysis 0 [get_ports creg_rf_cfg[2]]
#set_case_analysis 0 [get_ports creg_rf_cfg[1]]
#set_case_analysis 0 [get_ports creg_rf_cfg[0]]

## IO delays - Data Memory
#set_input_delay  -clock clk_pemem [expr $clk_period * 0.2] -max [get_ports [list data_out_dm*]]
#set_input_delay  -clock clk_pemem [expr $clk_period * 0.1] -min [get_ports [list data_out_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.2] -max [get_ports [list data_in_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.1] -min [get_ports [list data_in_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.2] -max [get_ports [list addr_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.1] -min [get_ports [list addr_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.2] -max [get_ports [list bit_en*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.1] -min [get_ports [list bit_en*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.2] -max [get_ports [list rd_en_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.1] -min [get_ports [list rd_en_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.2] -max [get_ports [list wr_en_dm*]]
#set_output_delay -clock clk_pemem [expr $clk_period * 0.1] -min [get_ports [list wr_en_dm*]]

## Select driving cells
### 7T_eLVT_C40
#set_driving_cell -lib_cell BUF_X4M_A12TUL50 [all_inputs]
set_driving_cell -lib_cell SC7P5T_CKBUFX4_CSC28L [all_inputs]
## 9Track_c50_Rvt
#set_driving_cell -lib_cell BUF_X4M_A9TRULP_C50_W3 [all_inputs]
=======
set_output_delay -clock clk [expr $clk_period * 0.45] -max [get_ports [list data_out*]]
## Select driving cells
set_driving_cell -lib_cell SC7P5T_CKBUFX4_CSC28L [all_inputs]
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

## Set load for all outputs
set_load 50 [all_outputs]
set_load 10 [get_ports [list data_out*]]

## Select max fanout and wireload
set_max_fanout 35 $DESIGN
<<<<<<< HEAD
#set_wire_load_model -name Medium

## Genus built-in script to auto add buffers to IO ports
#include load_etc.tcl
#insert_io_buffers -isolate_top ins $DESIGN
=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

