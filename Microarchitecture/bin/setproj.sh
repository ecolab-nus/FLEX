export REPO_ROOT=`pwd`

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
export sysPath=/usr/local/bin:/usr/bin:/usr/local/sbin:/usr/sbin
export PATH=$PATH:$cdsPath:$sysPath
export PVS_AUTOINIT_VIRTUOSO_MENUS=t

### SRAM/RF compiler
export SRAM_DP_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/SDPV/FE/IN22FDX_MEMSDPV_COMPILER_FE_RELV02R60
export RF_COMPILER=/opt/pdks/FOUNDATION_IP/MEMORY/BASE/R1PV/FE/IN22FDX_MEMR1PV_COMPILER_FE_RELV02R60

export PATH=$PATH:$SRAM_COMPILER:$SRAM_DP_COMPILER:$RF_COMPILER

