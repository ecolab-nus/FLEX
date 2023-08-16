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
set INSTANCE /tb_top/soc_pad/hycube0

set rtl_dir  "$::env(REPO_ROOT)/rtl"
set rtl_list "globals_top.vh flex.sv tile_no_ldst.sv tile_ldst.sv iterCounter.sv ls_add_unit.sv simple_alu.sv  router.sv xbar_bypass.sv encoder_onehot.sv pace_clkgate.sv pace_ff_rst.sv configurator.sv"
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
# register naming convention: <signal>_reg[<bit_width>] format
# set_attribute hdl_array_naming_style %s\[%d\] / ;# format forced in Joules

# track filename/line number through the flow
#set_attribute hdl_track_filename_row_col 1
set_db hdl_track_filename_row_col 1
## Parse RTL list
#set_attribute hdl_search_path $rtl_dir
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
if {"$::env(TBTOP)" == "flex_stockham_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77351 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 45187 ns          \
                       -end 45407 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 77651 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_stockham_4x4_4"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77427 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46093 ns          \
                       -end 46333 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 77860 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_aes_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 80392 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 49656 ns          \
                       -end 49996 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 80800 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_aes_4x4_4"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 80392 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 50061 ns          \
                       -end 50361 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 80800 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_convolution2d_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78911 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48139 ns          \
                       -end 50079 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 80900 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_convolution2d_4x4_4"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 80397 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48108 ns          \
                       -end 50008 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 83700 ns          \
                       -append


} elseif {"$::env(TBTOP)" == "flex_stencil3d_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77408 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46725 ns          \
                       -end 47065 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 77900 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_stencil3d_4x4_4"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77385 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 77433 ns          \
                       -end 77733 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 77800 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_stencil3d_4x4_8"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77385 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48106 ns          \
                       -end 48526 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 77800 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_idwt_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77373 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46649 ns          \
                       -end 46829 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 77700 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_idwt_4x4_4"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77373 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48104 ns          \
                       -end 48324 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 77700 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_fir_4x4_1"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77420 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46176 ns          \
                       -end 46506 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 77900 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_fir_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77420 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46137 ns          \
                       -end 46457 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 77900 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_fir_4x4_3"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77420 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46136 ns          \
                       -end 46486 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 77900 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_fir_4x4_4"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78890 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 46150 ns          \
                       -end 46450 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 79305 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_fir_4x4_5"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78890 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48098 ns          \
                       -end 48668 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 79305 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_fir_4x4_6"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78890 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48094 ns          \
                       -end 48594 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 79305 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_fir_4x4_7"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78890 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 50117 ns          \
                       -end 50907 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 79305 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_fir_4x4_8"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78890 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 50135 ns          \
                       -end 50715 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 79305 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_nw_4x4_1"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78879 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 48131 ns          \
                       -end 51981 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 82800 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_fdtd_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78935 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 78938 ns          \
                       -end 86608 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 86700 ns          \
                       -append
} elseif {"$::env(TBTOP)" == "flex_jpeg_4x4_2"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 78935 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 57965 ns          \
                       -end 58605 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 86700 ns          \
                       -append

} elseif {"$::env(TBTOP)" == "flex_gemm4_4x4_1"} {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 75891 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 44138 ns          \
                       -end 44797 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0 ns        \
                       -end 76600 ns          \
                       -append


} else {
    ## read load config ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_load_control \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns    \
                       -start 0ns            \
                       -end 77204 ns

    ## load data ram stage
    read_stimulus -file $STIM                   \
                       -alias stim_run    \
                       -top_instance $INSTANCE  \
                       -interval_size 5ns  \
                       -start 78654 ns          \
                       -end 78914 ns          \
                       -append

    ## read run stage
    read_stimulus -file $STIM                   \
                       -alias stim_total          \
                       -top_instance $INSTANCE  \
                       -interval_size 50ns      \
                       -start 0ns        \
                       -end 77600 ns          \
                       -append

}


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
    #gen_clock_tree -name CT1 -root_buffers $root_buffers -branch_buffers $branch_buffers -leaf_buffers $leaf_buffers
    gen_clock_tree -ideal_clock 
    #report_clock_tree -ctg_name CT1
}

#-------------------------------------------------------------------------------
# Load SDB
#-------------------------------------------------------------------------------
read_stimulus -file $f_sdb

#-------------------------------------------------------------------------------
# Estimate Data Buffers
#-------------------------------------------------------------------------------
# plot_net_distribution -xkey load_cap
#estimate_data_buffer -detailed -max_fanout $estimate_data_buf

#-------------------------------------------------------------------------------
# Compute power
#-------------------------------------------------------------------------------
compute_power -auto_tune clock -mode $pwra_mode -out $f_power_rpt

#-------------------------------------------------------------------------------
# Report PPA
#-------------------------------------------------------------------------------
report_ppa -out $f_ppa_rpt
#report_icgc_efficiency -out $f_ppa_rpt -append

#-------------------------------------------------------------------------------
# Report Power
#-------------------------------------------------------------------------------
    # By_category
    report_power -stims {/stim_load_control} \
                 -by_category                \
                 -out $f_ppa_rpt -append     \
                 -unit uW

    report_power -stims {/stim_run} \
                 -by_category             \
                 -out $f_ppa_rpt -append  \
                 -unit uW

    report_power -stims {/stim_total}      \
                 -by_category            \
                 -out $f_ppa_rpt -append \
                 -unit uW

  
    # By_hierarchy
    report_power -stims {/stim_load_control}                \
                 -by_hierarchy                              \
                 -levels 4                   \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW

    report_power -stims {/stim_run}                   \
                 -by_hierarchy                              \
                 -levels 4                   \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW

    report_power -stims {/stim_total}                         \
                 -by_hierarchy                              \
                 -levels 4                   \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW

   
    # By_instance memory
    report_power -stims {/stim_load_control}                \
                 -by_leaf_instance -rtl_type {memory}       \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW

    report_power -stims {/stim_run}                   \
                 -by_leaf_instance -rtl_type {memory}       \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW

    report_power -stims {/stim_total}                         \
                 -by_leaf_instance -rtl_type {memory}       \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW

   report_power -stims {/stim_run}                   \
                 -by_leaf_instance -rtl_type {register}       \
                 -cols {hier                                \
                        total leakage internal switching    \
                        cells area}                         \
                 -out $f_ppa_rpt -append                    \
                 -unit uW



# Print Power report
cdn_cat $f_ppa_rpt
#
## Plot Power graph
#if {"$::env(TBTOP)" == "alu_bench"} {
#    plot_power_profile -stims {/stim_bench}                     \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_bench}                     \
#                       -category memory register logic clock    \
#                       -unit uW
#} elseif {"$::env(TBTOP)" == "tile_bench"} {
#    plot_power_profile -stims {/stim_total}                     \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_total}                     \
#                       -category memory register logic clock    \
#                       -unit uW
#}  else {
#    plot_power_profile -stims {/stim_load_control}              \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_load_control}              \
#                       -category memory register logic clock    \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_run}                 \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_run}                 \
#                       -category memory register logic clock    \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_total}                       \
#                       -unit uW
#
#    plot_power_profile -stims {/stim_total}                       \
#                       -category memory register logic clock    \
#                       -unit uW
#
# }

#-------------------------------------------------------------------------------
# Joules Done
#-------------------------------------------------------------------------------
puts stdout "Done Reporting PPA"
exit

