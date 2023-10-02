<<<<<<< HEAD
## Copyright (C) A*STAR - All Rights Reserved
## Unauthorized copying of this file, via any medium is strictly prohibited
## Proprietary and confidential
## Written by Vishnu Paramasivam <vishnu_paramasivam@ime.a-star.edu.sg>, December 2018
## Description: Project env setup script (csh). Verbose switch '-v' prints additonal details/paths/etc.
## Warning: Do not change tool source sequence unless absolutely necessary!

## Common
#source /cad/setup/.cshrc_basic

## Repo root env variable
#SRC_SETPROJ_CMD=($_)
#export REPO_ROOT=`readlink -f $SRC_SETPROJ_CMD | sed 's/\/bin\/setproj.sh//g'`
=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
export REPO_ROOT=`pwd`

## Cadence Analog/Digital/Mixed design software
### Due to bin conflicts, the sequence must be: Joules, Virtuoso(ic), Incisive(ius), Genus, Tempus(ssv), Innovus
export LM_LICENSE_FILE=$LM_LICENSE_FILE:5280@everest1.d2.comp.nus.edu.sg
export CDS_LIC_FILE=5280@everest1.d2.comp.nus.edu.sg
export CDS_LIC_ONLY=t
export LANG=en_US
export CDS_AUTO_64BIT=all
export CDS_ROOT=/opt/cadence

#XCELIUM
export XCELIUM_HOME=/opt/cadence/XCELIUM2103

<<<<<<< HEAD
#SPECTRE
export SPECTREHOME=/opt/cadence/SPECTRE171

#Genus
export GENUS_HOME=/opt/cadence/GENUS211

#Innovus
#export INNOVUS_HOME=/opt/cadence/INNOVUS211
export INNOVUS_HOME=/opt/cadence/INNOVUS181

#virtuoso
export VIRTUOSO_HOME=/opt/cadence/IC618

#PVS
export PVS_HOME=/opt/cadence/PVS201

#ICADVM/VIRTUOSO
export ICADVM_HOME=/opt/cadence/ICADVM201
#export Virtuoso_Photonics_Option=t
#export Virtuoso_MultiTech=t

#LIBERATE
export LIBERATE_HOME=/opt/cadence/LIBERATE211

#JOULES
export JOULES_HOME=/opt/cadence/JLS201

export JASPER_HOME=/opt/cadence/jasper_2021.06p002

export cdsPath=$XCELIUM_HOME/tools/bin:$SPECTREHOME/tools:$SPECTREHOME/tools/bin:$GENUS_HOME/tools.lnx86/bin:$INNOVUS_HOME/tools.lnx86/bin:$PVS_HOME/tools.lnx86/bin:$VIRTUOSO_HOME/bin:$LIBERATE_HOME/bin:$JOULES_HOME/bin:$JASPER_HOME/bin:$ICADVM_HOME/bin
export  sysPath=/usr/local/bin:/usr/bin:/usr/local/sbin:/usr/sbin
export  PATH=$PATH:$cdsPath:$sysPath
export PVS_AUTOINIT_VIRTUOSO_MENUS=t

### SRAM/RF compiler
#SRAM_COMPILER=/opt/pdks/arm/umc/l40lp/sram_sp_hse_rvt_mvt/r1p0/bin
export SRAM_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/S1PV/FE/IN22FDX_MEMS1PV_COMPILER_FE_RELV02R60/bin
export SRAM_DP_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/SDPV/FE/IN22FDX_MEMSDPV_COMPILER_FE_RELV02R60
export RF_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/R1PV/FE/IN22FDX_MEMR1PV_COMPILER_FE_RELV02R60
export RF2_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/R2PV/FE/IN22FDX_MEMR2PV_COMPILER_FE_RELV02R60

#SRAM_DP_COMPILER=/opt/pdks/arm/umc/l40lp/sram_dp_hde_hvt_rvt/r1p0/bin
#RF_COMPILER=/opt/pdks/arm/umc/l40lp/rf_sp_hdf_hvt_rvt/r0p0/bin
export PATH=$PATH:$SRAM_COMPILER:$SRAM_DP_COMPILER:$RF_COMPILER

### Vivado
##source /cad/setup/vivado2020.start

### Tools setup
PACE_TOOLS=/proj/pace/paceVNP/tools
export PATH=$PACE_TOOLS/bin:$PATH
=======
#Genus
export GENUS_HOME=/opt/cadence/GENUS211

#JOULES
export JOULES_HOME=/opt/cadence/JLS201

export cdsPath=$XCELIUM_HOME/tools/bin:$GENUS_HOME/tools.lnx86/bin:$JOULES_HOME/bin
export  sysPath=/usr/local/bin:/usr/bin:/usr/local/sbin:/usr/sbin
export  PATH=$PATH:$cdsPath:$sysPath

### SRAM/RF compiler
#SRAM_COMPILER=/opt/pdks/arm/umc/l40lp/sram_sp_hse_rvt_mvt/r1p0/bin
export SRAM_DP_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/SDPV/FE/IN22FDX_MEMSDPV_COMPILER_FE_RELV02R60
export RF_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/R1PV/FE/IN22FDX_MEMR1PV_COMPILER_FE_RELV02R60

export PATH=$PATH:$SRAM_DP_COMPILER:$RF_COMPILER
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

### Repo make command shortcut (can execute from any dir)
alias mk="make -C $REPO_ROOT"

<<<<<<< HEAD
### Git-Issue (Issues tracking tool)
#alias iss $REPO_ROOT/bin/git-issue
=======
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832

### General tool setup
set filec
set autolist
<<<<<<< HEAD
#bindkey -k up history-search-backward
#bindkey -k down history-search-forward
alias realpath="readlink -f"

###### Verbose report switch
###if ($1 == '-v') then
###    echo "\nCAD tool paths report:"
###    which pureview
###    which liberate
###    which virtuoso
###    which xrun
###    which simvision
###    which genus
###    which innovus
###    which tempus
###    which voltus
###    which valus
###    which lec
###    which pvs
###    which jg
###    which joules
###    which vivado
###    which sram_sp_ulplve_mvt
###    which sram_dp_hde_hvt_rvt
###endif

=======
alias realpath="readlink -f"
>>>>>>> 94e0f4ee83c3fe6b68f6a5e9baa2e1168e898832
