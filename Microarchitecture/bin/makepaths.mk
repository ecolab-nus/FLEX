## Select shell
export SHELL = /bin/bash

## RTL/TB/IP/libs/tools/etc directories
export REPO_ROOT = $(shell pwd)
export TECHLEF   = /opt/22FDX/V1.2_0.0/PlaceRoute/Innovus/Techfiles/7M_2Mx_4Cx_1Ix_LB
export TECHQRC   = /opt/22FDX/V1.2_0.0/PEX/QRC/7M_2Mx_4Cx_1Ix_LBthick/nominal/
export TECHLIB   = /opt/pdks/FOUNDATION_IP/STDLIB/7P5T/BASE/LVT/FE/GF22FDX_SC7P5T_116CPP_BASE_CSC28L_FE_RELV02R00/GF22FDX_SC7P5T_116CPP_BASE_CSC28L_FDK_RELV02R00
export RTLFILES  = $(REPO_ROOT)/rtl
export TBFILES   = $(REPO_ROOT)/verif
export TBCOMMON  = $(REPO_ROOT)/verif/tb_common
export SRSPECS   = $(REPO_ROOT)/tools

## Build directories
export TARGET_ROOT = $(REPO_ROOT)/workdir
export TARGET_SYN  = $(TARGET_ROOT)/syn
export TARGET_RTL  = $(TARGET_ROOT)/xrun
export TARGET_GEN  = $(TARGET_ROOT)/gen
export TARGET_PWR  = $(TARGET_ROOT)/joules

## Build variables/switches
export UNIT    = flex
export TBTOP   = orig_test
export TEST    = $(TBTOP)
export SEED    = random
export WAVE    = waves
export FPGADBG = 0

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
