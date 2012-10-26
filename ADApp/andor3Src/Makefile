TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#LIBRARY_IOC_WIN32 += andor3
LIBRARY_IOC_Linux += andor3
LIB_SRCS += andor3.cpp

#PROD_IOC_WIN32 += andor3App
PROD_IOC_Linux += andor3App

# <name>.dbd will be created from <name>Include.dbd
DBD += andor3App.dbd
DBD += andor3Support.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += andor3App_registerRecordDeviceDriver.cpp andor3AppMain.cpp

# Add locally compiled object code
PROD_LIBS += andor3

# Add system libraries
PROD_SYS_LIBS += atcore

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

