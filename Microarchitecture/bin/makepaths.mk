<<<<<<< HEAD
## Copyright (C) A*STAR - All Rights Reserved
## Unauthorized copying of this file, via any medium is strictly prohibited
## Proprietary and confidential
## Written by Vishnu Paramasivam <vishnu_paramasivam@ime.a-star.edu.sg>, January 2018
## Description: Top level Makefile variables and paths

######################################
## Project build variables/directories
######################################
=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

## Select shell
export SHELL = /bin/bash

## RTL/TB/IP/libs/tools/etc directories
export REPO_ROOT = $(shell pwd)
<<<<<<< HEAD
export TECHLEF   = /opt/22FDX/V1.2_0.0/PlaceRoute/Innovus/Techfiles/7M_2Mx_4Cx_1Ix_LB
#export TECHLEF   = /opt/pdks/arm/umc/l40lp/arm_tech/r0p0/lef/1P7M1T0H0A
#export TECHQRC   = /opt/pdks/arm/umc/UMC40LP/RCmax/
export TECHQRC   = /opt/22FDX/V1.2_0.0/PEX/QRC/7M_2Mx_4Cx_1Ix_LBthick/nominal/
#export TECHLIB   = /opt/pdks/arm/umc/l40lp/sc12mc_base_ulvt_c50/r1p0
export TECHLIB   = /opt/pdks/FOUNDATION_IP/STDLIB/7P5T/BASE/LVT/FE/GF22FDX_SC7P5T_116CPP_BASE_CSC28L_FE_RELV02R00/GF22FDX_SC7P5T_116CPP_BASE_CSC28L_FDK_RELV02R00
#export HDLPKLIB  = /cad/tech/UMC40ULP/LIBRARIES/SC_Arm/7T-C40-HDLPK-eLvt/arm/umc/l40ulp/sc7mc_hdlpk_elvtulp_c40_w3/r0p0
#export HPKLIB    = /cad/tech/UMC40ULP/LIBRARIES/SC_Arm/7T-C40-HPK-eLvt/arm/umc/l40ulp/sc7mc_hpk_elvtulp_c40_w3/r0p0
export IOPADLIB  = /opt/pdks/GPIO/foh0l_prs25/2017Q3v1.1/TMVH33L18_GENERIC_IO
export CWPATH    = /opt/cadence/GENUS211/share/synth/lib/chipware/syn/CW
export RTLFILES  = $(REPO_ROOT)/rtl
export SUBIP     = $(REPO_ROOT)/subip
export TBFILES   = $(REPO_ROOT)/verif
export TBCOMMON  = $(REPO_ROOT)/verif/tb_common
export SWFILES   = $(REPO_ROOT)/sw
export SRSPECS   = $(REPO_ROOT)/tools
#Issue tracker repo
#export ISSUE_TRK = ??
=======
export TECHLIB   = /opt/pdks/FOUNDATION_IP/STDLIB/7P5T/BASE/LVT/FE/GF22FDX_SC7P5T_116CPP_BASE_CSC28L_FE_RELV02R00/GF22FDX_SC7P5T_116CPP_BASE_CSC28L_FDK_RELV02R00
export RTLFILES  = $(REPO_ROOT)/rtl
export TBFILES   = $(REPO_ROOT)/verif
export TBCOMMON  = $(REPO_ROOT)/verif/tb_common
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

## Build directories
export TARGET_ROOT = $(REPO_ROOT)/workdir
export TARGET_SYN  = $(TARGET_ROOT)/syn
<<<<<<< HEAD
export TARGET_FPGA = $(TARGET_ROOT)/vivado
export TARGET_RTL  = $(TARGET_ROOT)/xrun
export TARGET_GEN  = $(TARGET_ROOT)/gen
export TARGET_FEV  = $(TARGET_ROOT)/fev
export TARGET_LINT = $(TARGET_ROOT)/lint
export TARGET_CDC  = $(TARGET_ROOT)/cdc
export TARGET_PWR  = $(TARGET_ROOT)/joules

## Build variables/switches
export UNIT    = tile
export TBTOP   = pace_orig_test
#export TBTOP   = array_add
#export TBTOP   = pedometer_step_only
#export TBTOP   = gemm2x2x2_diff_data
export TEST    = $(TBTOP)
export SEED    = random
export WAVE    = waves
export FPGADBG = 0
=======
export TARGET_RTL  = $(TARGET_ROOT)/xrun
export TARGET_GEN  = $(TARGET_ROOT)/gen
export TARGET_PWR  = $(TARGET_ROOT)/joules

## Build variables/switches
export UNIT    = flex
export TBTOP   = pace_orig_test
export TEST    = $(TBTOP)
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

## Logging variables
export MAKELOG = $(TARGET_ROOT)/make.log
export LOG     = 2>&1 | tee -i -a $(MAKELOG)

## Genus synthesis script path (.tcl)
#### Use generic if no existing script found
ifeq ("$(wildcard $(REPO_ROOT)/syn/$(UNIT).tcl)","")
export GNTOP = $(REPO_ROOT)/syn/generic_syn.tcl
#### Otherwise use script with name from $UNIT
else
export GNTOP = $(REPO_ROOT)/syn/$(UNIT).tcl
endif
<<<<<<< HEAD

## FEV check script path (.do)
#### Use generic if no existing script found
ifeq ("$(wildcard $(REPO_ROOT)/tools/fev_$(UNIT).do)","")
export FEVDO = $(REPO_ROOT)/tools/fev_generic.do
#### Otherwise use script with name from $UNIT
else
export FEVDO = $(REPO_ROOT)/tools/fev_$(UNIT).do
endif

## Lint check script path (.tcl)
#### Use generic if no existing script found
ifeq ("$(wildcard $(REPO_ROOT)/tools/lint_$(UNIT).tcl)","")
export LINT_TCL = $(REPO_ROOT)/tools/lint_generic.tcl
#### Otherwise use script with name from $UNIT
else
export LINT_TCL = $(REPO_ROOT)/tools/lint_$(UNIT).tcl
endif

## CDC check script path (.tcl)
#### Use generic if no existing script found
ifeq ("$(wildcard $(REPO_ROOT)/tools/cdc_$(UNIT).tcl)","")
export CDC_TCL = $(REPO_ROOT)/tools/cdc_generic.tcl
#### Otherwise use script with name from $UNIT
else
export CDC_TCL = $(REPO_ROOT)/tools/cdc_$(UNIT).tcl
endif

## Xprop switch for Xcelium
ifeq ($(XPROP),1)
export XPROP_SW = -xprop F
endif

## Enable/disable GUI switches, add tools as needed
ifneq ($(GUI),1)
export FEVGUI_SW = -nogui
export JASPGUI_SW = -batch
#export GNGUI_SW = -no_gui
endif

## Specify simulation coverage database name to enable coverage analysis
ifneq ($(COVDB),)
export COVOPTS = -covtest $(COVDB) -covfile $(REPO_ROOT)/tools/covfile.ccf -covoverwrite
endif

=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
