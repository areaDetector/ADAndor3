TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_Linux += andor3
LIBRARY_IOC_WIN32 += andor3
LIB_SRCS += andor3.cpp

PROD_NAME = andor3App
PROD_IOC_Linux += $(PROD_NAME)
PROD_IOC_Linux += binTest
PROD_IOC_Linux += rateTest
PROD_IOC_WIN32 += $(PROD_NAME)
PROD_IOC_WIN32 += binTest
PROD_IOC_WIN32 += rateTest

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += andor3Support.dbd
DBD += andor3Support.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
andor3App_SRCS += andor3App_registerRecordDeviceDriver.cpp andor3AppMain.cpp
binTestSrcs += binTest.cpp
binTestSrcs += rateTest.cpp

# Add locally compiled object code
PROD_LIBS += andor3
PROD_LIBS_WIN32 += atcorem
PROD_SYS_LIBS_Linux += atcore

ifeq (windows-x64-debug, $(findstring windows-x64-debug, $(T_A)))
USR_LDFLAGS_WIN32 += /NODEFAULTLIB:LIBCMT.lib
endif

# Use this line when building on Windows with debugging
include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

