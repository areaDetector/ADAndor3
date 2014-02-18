TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from the Andor SDK3 directories 
# for Linux and Windows.  The soft links for Linux were manually created.

INC += atcore.h

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/win32-x86/atcorem.lib

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/atcorem.lib

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcore.so.3.5.30014.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcore.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcore.so

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libatcore.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatcore.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatcore.so

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

