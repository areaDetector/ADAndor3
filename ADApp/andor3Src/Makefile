TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_Linux += andor3
LIBRARY_IOC_WIN32 += andor3
LIB_SRCS += andor3.cpp

PROD_IOC_Linux += andor3App
PROD_IOC_Linux += serialNumber
PROD_IOC_WIN32 += andor3App
PROD_IOC_WIN32 += serialNumber

# <name>.dbd will be created from <name>Include.dbd
DBD += andor3App.dbd
DBD += andor3Support.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
andor3App_SRCS += andor3App_registerRecordDeviceDriver.cpp andor3AppMain.cpp
serialNumberSrcs += serialNumber.cpp

# Add locally compiled object code
PROD_LIBS += andor3

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
PROD_LIBS_WIN32 += atcorem
PROD_LIBS_WIN32 += atmcd32m

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
PROD_LIBS_WIN32 += atcorem
PROD_LIBS_WIN32 += atmcd64m

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
PROD_LIBS_Linux += atcore
STATIC_BUILD=NO

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
PROD_LIBS_Linux += atcore
STATIC_BUILD=NO
endif


include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

