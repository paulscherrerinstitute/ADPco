TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================


# Note, the following files were manually copied from the poc.sdk distribution.
ifeq ($(OS_CLASS), WIN32)
LIB_INSTALLS    += os/windows-x64/pco_cdlg.lib
LIB_INSTALLS    += os/windows-x64/pco_conv.lib
LIB_INSTALLS    += os/windows-x64/pco_edge_gs_calc.lib
LIB_INSTALLS    += os/windows-x64/sc2_cam.lib
LIB_INSTALLS    += os/windows-x64/sc2_dlg.lib

else ifeq ($(OS_CLASS), Linux)
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
