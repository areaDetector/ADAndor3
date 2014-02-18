TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = andor3App
PROD_IOC_Linux += $(PROD_NAME)
PROD_IOC_WIN32 += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += andor3Support.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS           += andor3
PROD_LIBS_WIN32     += atcorem
PROD_SYS_LIBS_Linux += atcore

ifeq (windows-x64-debug, $(findstring windows-x64-debug, $(T_A)))
USR_LDFLAGS_WIN32 += /NODEFAULTLIB:LIBCMT.lib
endif

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

