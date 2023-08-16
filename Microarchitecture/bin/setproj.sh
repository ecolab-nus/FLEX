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

### Repo make command shortcut (can execute from any dir)
alias mk="make -C $REPO_ROOT"


### General tool setup
set filec
set autolist
alias realpath="readlink -f"
