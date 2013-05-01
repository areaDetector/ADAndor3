TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

LIBRARY_IOC_Linux += andor3
LIBRARY_IOC_WIN32 += andor3
LIB_SRCS += andor3.cpp

DBD += andor3Support.dbd

#====== DETECTOR APPLICATION ============

ifeq ($(BUILD_APPS), YES)
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

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp
binTestSrcs += binTest.cpp
rateTestSrcs += rateTest.cpp

# Add locally compiled object code
PROD_LIBS += andor3
PROD_LIBS_WIN32 += atcorem
PROD_SYS_LIBS_Linux += atcore

ifeq (windows-x64-debug, $(findstring windows-x64-debug, $(T_A)))
USR_LDFLAGS_WIN32 += /NODEFAULTLIB:LIBCMT.lib
endif

# Use this line when building on Windows with debugging
include $(TOP)/ADApp/commonDriverMakefile
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

