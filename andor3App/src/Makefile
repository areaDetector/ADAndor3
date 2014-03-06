TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE


# Need to figure this out; library is only shareable, so we say LIB_SYS_LIBS and it works
ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
SHARED_LIBRARIES=NO
endif

LIBRARY_IOC_Linux += andor3
LIBRARY_IOC_WIN32 += andor3
LIB_SRCS += andor3.cpp
LIB_LIBS_WIN32 += atcorem
LIB_SYS_LIBS_Linux += atcore

DBD += andor3Support.dbd

PROD_IOC_Linux += binTest
PROD_IOC_Linux += rateTest
PROD_IOC_WIN32 += binTest
PROD_IOC_WIN32 += rateTest

binTestSrcs  += binTest.cpp
rateTestSrcs += rateTest.cpp

# Add locally compiled object code
PROD_LIBS += andor3
PROD_LIBS_WIN32 += atcorem
PROD_SYS_LIBS_Linux += atcore

ifeq (windows-x64-debug, $(findstring windows-x64-debug, $(T_A)))
USR_LDFLAGS_WIN32 += /NODEFAULTLIB:LIBCMT.lib
endif

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

