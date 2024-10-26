#=======================================================================
# 6.375 Makefile for dc-synth
#-----------------------------------------------------------------------
#
# This makefile will use Cadence RTL Compiler to synthesize
# your RTL into a gate-level verilog netlist.
#

default : all

#basedir = $(PROJ_DIR)
#--------------------------------------------------------------------
# Sources
#--------------------------------------------------------------------

#include Makefile.common

# get source verilog filenames and toplevel module name
#include Makefile_dc.include

# default clock frequency of 1 GHz
CLOCK_PERIOD 		= 2000 
IS_SET_FROM_CMD		= 0
NUM_HOPS_PER_CYCLE	= 4
STDCELL_LATCH_NAME	= PREICG_X1B_A9TR

#--------------------------------------------------------------------
# Build rules
#--------------------------------------------------------------------

build_suffix   := $(shell date +%Y-%m-%d_%H-%M-%S)
build_dir      := build-$(build_suffix)
curr_build_dir := current
curr_build_dir_tstamp := current/timestamp.txt

synth_tcl_4x4      := dc_4x4.tcl
libs_tcl_4x4       := dc_setup_4x4.tcl
synth_tcl_6x6      := dc_6x6.tcl
libs_tcl_6x6       := dc_setup_6x6.tcl
synth_sdc      := cgra.sdc
#synth_saif     := cgra.saif
output_tcl     := dc_setup_filenames.tcl
common_tcl     := common_setup.tcl
define_tcl     := define_setup.tcl
#phy_tcl        := cgra.physical_constraints.tcl
scripts_4x4        := $(synth_tcl_4x4) $(libs_tcl_4x4) $(synth_sdc) $(output_tcl) $(common_tcl) $(define_tcl)
scripts_6x6        := $(synth_tcl_6x6) $(libs_tcl_6x6) $(synth_sdc) $(output_tcl) $(common_tcl) $(define_tcl)
makegen_tcl    := make_generated_vars.tcl

synth_vars = \
	set DESIGN_RTL_DIR  		{ $(RTL_FILES_ROOT) $(SEARCH_PATHS) }; \
	set DESIGN_RTL      		{ $(RTL_FILES) $(RTL_FILES_CORE) }; \
	set DESIGN_TOPLEVEL 		$(TOPLEVEL); \
	set SYNTH_LIB       		{ $(SYNTH_LIB) }; \
	set CLOCK_PERIOD			$(CLOCK_PERIOD); \
	set IS_SET_FROM_CMD 		$(IS_SET_FROM_CMD); \
	set NUM_HOPS_PER_CYCLE		$(NUM_HOPS_PER_CYCLE); \
	set STDCELL_LATCH_NAME		$(STDCELL_LATCH_NAME); \

define new-build-dir-cmds-4x4
	mkdir $(build_dir)
	rm -f $(curr_build_dir)
	ln -s $(build_dir) $(curr_build_dir)
	cp $(scripts_4x4) $(curr_build_dir)
	echo '$(synth_vars)' > $(curr_build_dir)/$(makegen_tcl)
endef

define new-build-dir-cmds-6x6
	mkdir $(build_dir)
	rm -f $(curr_build_dir)
	ln -s $(build_dir) $(curr_build_dir)
	cp $(scripts_6x6) $(curr_build_dir)
	echo '$(synth_vars)' > $(curr_build_dir)/$(makegen_tcl)
endef


new-build-dir-4x4 :
	$(new-build-dir-cmds-4x4)

new-build-dir-6x6 :
	$(new-build-dir-cmds-6x6)

%.v : 
	true

synth_4x4 :  
	$(vsrcs) $(scripts) 
	$(new-build-dir-cmds-4x4)
	cd $(curr_build_dir); \
	dc_shell -64bit $(OPTS) -f dc_4x4.tcl | tee dc.log; \

synth_6x6 : 
	$(vsrcs) $(scripts) 
	$(new-build-dir-cmds-6x6)
	cd $(curr_build_dir); \
	dc_shell -64bit $(OPTS) -f dc_6x6.tcl | tee dc.log; \

junk += 

.PHONY : synth_4x4 synth_6x6 new-build-dir-4x4 new-build-dir-cmds-6x6

#--------------------------------------------------------------------
# Default make target
#--------------------------------------------------------------------

all : synth_4x4 synth_6x6

#--------------------------------------------------------------------
# Clean up
#--------------------------------------------------------------------

clean :
	rm -rf build-[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]_[0-9][0-9]-[0-9][0-9] \
	       current $(junk) *~ \#*
