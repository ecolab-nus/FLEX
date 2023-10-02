<<<<<<< HEAD
## Copyright (C) A*STAR - All Rights Reserved
## Unauthorized copying of this file, via any medium is strictly prohibited
## Proprietary and confidential
## Written by Lan Jingjing <Lan_Jingjing@ime.a-star.edu.sg>, March 2021
## Description: Genus synth script for tile module

=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
## Top level design name
set DESIGN $::env(UNIT)

## Include common setup file (root attributes & setup variables).
include $::env(REPO_ROOT)/syn/common_syn.tcl

## RTL search path
set_db init_hdl_search_path "$::env(RTLFILES)"

## Parse RTL list
<<<<<<< HEAD
read_hdl -sv   "flex_final_4x4_optimized_14/globals_top.vh pace_clkgate.sv flex_final_4x4_optimized_14/flex.sv flex_final_4x4_optimized_14/simple_alu.sv encoder_onehot.sv \
                flex_final_4x4_optimized_14/xbar_bypass.sv flex_final_4x4_optimized_14/router.sv flex_final_4x4_optimized_14/tile_no_ldst.sv flex_final_4x4_optimized_14/tile_ldst.sv flex_final_4x4_optimized_14/configurator.sv flex_final_4x4_optimized_14/iterCounter.sv flex_final_4x4_optimized_14/ls_add_unit.sv pace_ff_rst.sv "
=======
read_hdl -sv   "globals_top.vh pace_clkgate.sv flex.sv simple_alu.sv encoder_onehot.sv \
                xbar_bypass.sv router.sv tile_no_ldst.sv tile_ldst.sv configurator.sv iterCounter.sv ls_add_unit.sv pace_ff_rst.sv "
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
##latch error
set_db hdl_error_on_latch false
## Clock gating
set_db lp_insert_clock_gating true

<<<<<<< HEAD
### Multibit FF options
#set_db use_multibit_cells true
#set_db use_multibit_combo_cells true
#set_db use_multibit_seq_and_tristate_cells true
#set_db bank_based_multibit_inferencing true

=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

set_db auto_ungroup none
set_db hdl_preserve_unused_registers true
set_db optimize_constant_0_flops false
set_db optimize_constant_1_flops false
set_db delete_unloaded_seqs false
set_db delete_unloaded_insts false
set_db optimize_constant_latches false

## Top level RTL to synthesize
elaborate $DESIGN
uniquify $DESIGN

## Load constraints file
read_sdc $::env(REPO_ROOT)/syn/flex.sdc

## Synthesize and map design
syn_generic
syn_map
syn_opt
#syn_opt -incremental

## Write out netlist file
write_design -innovus -basename $_OUTPUTS_PATH/$DESIGN

## Dump reports
report area > $_REPORTS_PATH/area_report.log
report timing -nworst 50 > $_REPORTS_PATH/timing_report.log
report timing -lint -verbose > $_REPORTS_PATH/timing_lint.log
report power > $_REPORTS_PATH/power_report.log
report datapath > $_REPORTS_PATH/datapath.log
report messages > $_REPORTS_PATH/messages.log
report gates > $_REPORTS_PATH/gates.log
report qor > $_REPORTS_PATH/qor.log
report nets > $_REPORTS_PATH/nets.log
report sequential > $_REPORTS_PATH/sequential.log
check_design -unresolved > $_REPORTS_PATH/check_design.log
generate_reports -directory $_REPORTS_PATH/${DESIGN}_rpt -tag generic
summary_table -directory $_REPORTS_PATH/${DESIGN}_rpt

quit

