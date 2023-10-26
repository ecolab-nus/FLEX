#-------------------------------------------------------------------------------
# Configure parameters
#-------------------------------------------------------------------------------
set pwra_mode "time_based" ;# average| time_based
set estimate_data_buf 16 ;# estimate data_buf max_fanout (default=16)
#set_attribute information_level 9
set_db information_level 9

#-------------------------------------------------------------------------------
# Configure module/instance parameters
#-------------------------------------------------------------------------------

set DESIGN flex
set INSTANCE /tb_top/soc_pad/flex0

set rtl_dir  "$::env(REPO_ROOT)/rtl"
set rtl_list "globals_top.vh flex.sv tile_no_ldst.sv tile_ldst.sv iterCounter.sv ls_add_unit.sv simple_alu.sv  router.sv xbar_bypass.sv encoder_onehot.sv clkgate.sv ff_rst.sv configurator.sv"
set f_sdc    "$::env(REPO_ROOT)/syn/flex.sdc"


set upf_list "$::env(REPO_ROOT)/rtl/upf/generic.upf"
set f_netlist "$::env(REPO_ROOT)/workdir/syn/${DESIGN}/fv/${DESIGN}/fv_map.v.gz"

set root_buffers   BUF_X4M_A7TELULP_C40_W3
set branch_buffers BUF_X4M_A7TELULP_C40_W3
set leaf_buffers   BUF_X4M_A7TELULP_C40_W3

#Waveform file
set STIM $::env(REPO_ROOT)/workdir/xrun/$::env(WAVE).shm

#-------------------------------------------------------------------------------
# Read Library and Create Library Domains (copy from syn/common_syn.tcl)
#-------------------------------------------------------------------------------
include $::env(REPO_ROOT)/syn/common_syn.tcl

#Reports
set f_sdb       $joulesWorkDir/$_REPORTS_PATH/${DESIGN}.sdb
set f_jdb       $joulesWorkDir/$_REPORTS_PATH/${DESIGN}.jdb
set f_ppa_rpt   $joulesWorkDir/$_REPORTS_PATH/${DESIGN}_ppa.rpt
set f_power_rpt $joulesWorkDir/$_REPORTS_PATH/${DESIGN}_power.rpt

#-------------------------------------------------------------------------------
# Generate Elabrate Database
#-------------------------------------------------------------------------------
set_db hdl_track_filename_row_col 1
## Parse RTL list
set_db hdl_error_on_latch false

set_db hdl_search_path $rtl_dir
read_hdl -sv $rtl_list
read_power_intent -module $DESIGN -1801 $upf_list
elaborate $DESIGN
check_design -unresolved
apply_power_intent
commit_power_intent

# save elab DB
write_db $DESIGN -to_file $f_jdb
puts stdout "Joules DB Created: $f_jdb"

#-------------------------------------------------------------------------------
# Generate Stimulus Database
#-------------------------------------------------------------------------------

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 39581 ns          \
                       -end 39911 ns          \
                       -append


# save SDB
write_sdb -out $f_sdb
puts stdout "Joules SDB Created: $f_sdb"

#-------------------------------------------------------------------------------
# Initialize with rtlstim2gate - Netlist Flow
#-------------------------------------------------------------------------------
rtlstim2gate -init $f_jdb

#-------------------------------------------------------------------------------
# If synthesis was done with naming settings, apply that as rules
#-------------------------------------------------------------------------------
rtlstim2gate -rule reg_ext {%s_reg%s}
rtlstim2gate -rule bit_slice {[%s]}
rtlstim2gate -rule array_slice {[%s]}
rtlstim2gate -rule hier_slice {[%s]}
rtlstim2gate -rule generate {%s.%s}

#-------------------------------------------------------------------------------
# Load Netlist
#-------------------------------------------------------------------------------
include $::env(REPO_ROOT)/syn/common_syn.tcl
read_netlist $f_netlist

#-------------------------------------------------------------------------------
# Clock Tree Synthesis Netlist
#-------------------------------------------------------------------------------
if {$f_sdc != ""} {
    read_sdc $f_sdc
    gen_clock_tree -ideal_clock 
}

#-------------------------------------------------------------------------------
# Load SDB
#-------------------------------------------------------------------------------
read_stimulus -file $f_sdb


#-------------------------------------------------------------------------------
# Compute power
#-------------------------------------------------------------------------------
compute_power -auto_tune clock -mode $pwra_mode -out $f_power_rpt

#-------------------------------------------------------------------------------
# Report PPA
#-------------------------------------------------------------------------------
report_ppa -out $f_ppa_rpt

#-------------------------------------------------------------------------------
# Report Power
#-------------------------------------------------------------------------------
    # By_category

    report_power -stims {/stim_run} \
                 -by_category             \
                 -out $f_ppa_rpt -append  \
                 -unit uW


  
    # By_hierarchy

    report_power -stims {/stim_run}                   \
                 -by_hierarchy                              \
                 -levels 4                   \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW



# Print Power report
cdn_cat $f_ppa_rpt
#-------------------------------------------------------------------------------
# Joules Done
#-------------------------------------------------------------------------------
puts stdout "Done Reporting PPA"
exit

