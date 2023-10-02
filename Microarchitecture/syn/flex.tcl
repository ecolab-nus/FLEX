## Top level design name
set DESIGN $::env(UNIT)

## Include common setup file (root attributes & setup variables).
include $::env(REPO_ROOT)/syn/common_syn.tcl

## RTL search path
set_db init_hdl_search_path "$::env(RTLFILES)"

## Parse RTL list
read_hdl -sv   "globals_top.vh pace_clkgate.sv flex.sv simple_alu.sv encoder_onehot.sv \
                xbar_bypass.sv router.sv tile_no_ldst.sv tile_ldst.sv configurator.sv iterCounter.sv ls_add_unit.sv pace_ff_rst.sv "
##latch error
set_db hdl_error_on_latch false
## Clock gating
set_db lp_insert_clock_gating true

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

