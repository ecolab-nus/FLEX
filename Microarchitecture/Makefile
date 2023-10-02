

## Primary include file(s)
include bin/makepaths.mk

#####################################
## Project build stages (targets)
#####################################

## Must make all stages phony to ensure proper execution
.PHONY: all clean temp list rtl pre gen syn power sim

## Primary build stage (build all)
all: pre gen rtl

## Prebuild phase and set up logging
pre:
	#Create target dirs
	@mkdir -p $(TARGET_ROOT)
	@mkdir -p $(TARGET_RTL)
	@mkdir -p $(TARGET_GEN)
	#Dump date and env vars into logfile
	date $(LOG)
	env $(LOG)
	@bin/checklog $(MAKELOG) bin/error.lst bin/pass.lst $(LOG)

## Generate collaterals stage
gen: pre
	@bin/printc "\n* STAGE: Generating RTL collaterals" CYAN $(LOG)
	#Generate SRAM collaterals
	export IDDK_ROOT="$(RF_COMPILER)/iddk"
	cd $(TARGET_GEN) && $(RF_COMPILER)/bin/IN22FDX_MEM_genviews -corner 'TT 0.800 0.800 0.0 0.0 25' -macro IN22FDX_R1PV_NFVG_W00008B024M02C256 -log genviews.log
	cd $(TARGET_GEN) && $(RF_COMPILER)/bin/IN22FDX_MEM_genviews -corner 'TT 0.800 0.800 0.0 0.0 25' -macro IN22FDX_R1PV_NFVG_W00008B028M02C256 -log genviews.log
	cd $(TARGET_GEN) && $(SRAM_DP_COMPILER)/bin/IN22FDX_MEM_genviews -corner 'TT 0.800 0.800 0.0 0.0 25' -macro IN22FDX_SDPV_NFVG_W00256B016M08C064 -log genviews.log 

## RTL compile stage
rtl: pre
	@bin/printc "\n* STAGE: RTL compilation" CYAN $(LOG)
	cd $(TARGET_RTL) && xrun -compile -f $(RTLFILES)/hdl.lst -work soc_lib $(LOG)
	@bin/checklog $(MAKELOG) bin/error.lst bin/pass.lst $(LOG)

## RTL simulation stage
sim: pre rtl
	@bin/printc "\n* STAGE: Simulation run = $(TBTOP):$(TEST) seed:$(SEED)" CYAN $(LOG)
	#Elaborate test_lib with other libs and run test
	cd $(TARGET_RTL) && xrun -elaborate -f $(TBFILES)/tb_hdl.lst -reflib soc_lib -pkgsearch test_lib -work test_lib $(LOG)
	cd $(TARGET_RTL) && xrun -R -seed $(SEED) -input $(REPO_ROOT)/tools/wavedump.tcl +UVM_TESTNAME=$(TEST) -f $(TBFILES)/tb_hdl.lst $(LOG)
	@bin/checklog $(MAKELOG) bin/error.lst bin/pass.lst $(LOG)

## RTL synthesis stage
syn: pre
	@bin/printc "\n* STAGE: RTL synthesis = $(UNIT)" CYAN $(LOG)
	mkdir -p $(TARGET_SYN)/$(UNIT) $(LOG)
	cd $(TARGET_SYN)/$(UNIT) && genus -no_gui -64 -f $(GNTOP) -log run_rc.log $(LOG)
	@bin/checklog $(MAKELOG) bin/error.lst bin/pass.lst $(LOG)

## Compute power stage
power: pre
	@bin/printc "\n* STAGE: Compute power" CYAN $(LOG)
	mkdir -p $(TARGET_PWR) $(LOG)
	cd $(TARGET_PWR) && joules -batch -common_ui -f $(REPO_ROOT)/tools/run_joules_flex_generic_16_3_28.tcl $(LOG)
	@bin/checklog $(MAKELOG) bin/error.lst bin/pass.lst $(LOG)


