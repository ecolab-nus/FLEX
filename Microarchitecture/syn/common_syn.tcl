## Print CPU info on screen
if {[file exists /proc/cpuinfo]} {
  sh grep "model name" /proc/cpuinfo
  sh grep "cpu MHz"    /proc/cpuinfo
}
puts "Hostname : [info hostname]"

## Preset global variables and attributes
set SYN_EFF high
set MAP_EFF high
set DATE [clock format [clock seconds] -format "%b%d-%T"]
set _OUTPUTS_PATH outputs_${DESIGN}_${DATE}
set _REPORTS_PATH reports_${DESIGN}_${DATE}
set _LOG_PATH logs_${DESIGN}_${DATE}

## Genus settings
set_db max_cpus_per_server 16
set_db information_level 7
set_db hdl_track_filename_row_col true

## Set project level RTL defines
set_db hdl_verilog_defines "SYNTHESIS UMC40NM SVA_OFF"


# Issue an error when latches are inferred
set_db hdl_error_on_latch true

###############################################################
## Library setup
###############################################################

set_db design_process_node 22

## Library search paths
set_db lib_search_path "\
    $::env(TECHLIB)/model/timing/lib \
    $::env(TECHLIB)/lef \
    $::env(TARGET_GEN)/genviews-output/model/timing/lib \
    $::env(TARGET_GEN)/genviews-output/lef"

## Lib files
set hip_lib_list "GF22FDX_SC7P5T_116CPP_BASE_CSC28L_TT_0P80V_0P00V_0P00V_0P00V_25C.lib IN22FDX_R1PV_NFVG_W00008B024M02C256_116cpp_TT_0P800V_0P800V_0P000V_0P000V_025C.lib IN22FDX_R1PV_NFVG_W00008B028M02C256_116cpp_TT_0P800V_0P800V_0P000V_0P000V_025C.lib IN22FDX_SDPV_NFVG_W00256B016M08C064_116cpp_TT_0P800V_0P800V_0P000V_0P000V_025C.lib"
set_db library "$hip_lib_list"

## LEF files
set hip_lef_list "22FDSOI_7M_2Mx_4Cx_1Ix_LB_116cpp_tech.lef GF22FDX_SC7P5T_116CPP_BASE_CSC28L.lef IN22FDX_R1PV_NFVG_W00008B024M02C256.lef IN22FDX_R1PV_NFVG_W00008B028M02C256.lef IN22FDX_SDPV_NFVG_W00256B016M08C064.lef"
set_db lef_library "$hip_lef_list"


