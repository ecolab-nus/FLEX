
## Timing unit
set_units -capacitance 1fF
set_units -time 1000.0ps

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
set_output_delay -clock clk [expr $clk_period * 0.45] -max [get_ports [list data_out*]]
## Select driving cells
set_driving_cell -lib_cell SC7P5T_CKBUFX4_CSC28L [all_inputs]

## Set load for all outputs
set_load 50 [all_outputs]
set_load 10 [get_ports [list data_out*]]

## Select max fanout and wireload
set_max_fanout 35 $DESIGN

