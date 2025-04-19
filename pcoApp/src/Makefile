TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += ADPco

# External PCO Linux SDK
ifeq ($(WITH_PCO), YES)
  LIBRARY_IOC_Linux += ADPco

  USR_CPPFLAGS_Linux += -DPCO_LINUX
  USR_CPPFLAGS_Linux += -I $(PCO_INCLUDE)
  USR_LDFLAGS_Linux += -L $(PCO_LIB) -Wl,-rpath,$(PCO_LIB)
  LIB_SYS_LIBS_Linux  += sc2_cam
endif

# PCO Windows SDK
USR_INCLUDES_WIN32 += -I ../../pcoSupport/include

ifeq ($(SHARED_LIBRARIES), YES)
  USR_CPPFLAGS_WIN32 += -D_AFXDLL
endif

LIB_LIBS_WIN32  += pco_cdlg pco_conv sc2_cam sc2_dlg

LIB_SRCS += ADPco.cpp

DBD += ADPcoSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

