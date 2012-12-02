TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from the Andor SDK directories 
# for Linux and Windows.  The soft links for Linux were manually created.

# The extension on ATMCD32D.h was changed from .H to .h because the EPICS build system does
# not recognize .H
#INC += ATMCD32D.h
INC += atcore.h

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
BIN_INSTALLS_WIN32    += ../os/win32-x86/atblkbx.dll
BIN_INSTALLS_WIN32    += ../os/win32-x86/atcl_bitflow.dll
BIN_INSTALLS_WIN32    += ../os/win32-x86/atcore.dll
LIB_INSTALLS_WIN32    += ../os/win32-x86/atcorem.lib
BIN_INSTALLS_WIN32    += ../os/win32-x86/atdevregcam.dll
BIN_INSTALLS_WIN32    += ../os/win32-x86/atdevsimcam.dll
BIN_INSTALLS_WIN32    += ../os/win32-x86/atmcd32d.dll
LIB_INSTALLS_WIN32    += ../os/win32-x86/atmcd32m.lib

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
BIN_INSTALLS_WIN32    += ../os/windows-x64/atblkbx.dll
BIN_INSTALLS_WIN32    += ../os/windows-x64/atcl_bitflow.dll
BIN_INSTALLS_WIN32    += ../os/windows-x64/atcore.dll
LIB_INSTALLS_WIN32    += ../os/windows-x64/atcorem.lib
BIN_INSTALLS_WIN32    += ../os/windows-x64/atdevregcam.dll
BIN_INSTALLS_WIN32    += ../os/windows-x64/atdevsimcam.dll
BIN_INSTALLS_WIN32    += ../os/windows-x64/atmcd64d.dll
LIB_INSTALLS_WIN32    += ../os/windows-x64/atmcd64m.lib

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatblkbx.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatblkbx.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatblkbx.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcl_bitflow.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcl_bitflow.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcl_bitflow.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcore.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcore.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatcore.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatdevregcam.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatdevregcam.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatdevregcam.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatdevsimcam.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatdevsimcam.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatdevsimcam.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatmcd.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatmcd.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatmcd.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatusb_libusb.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatusb_libusb.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libatusb_libusb.so

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libatblkbx.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatblkbx.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatblkbx.so
LIB_INSTALLS_Linux += ../os/linux-x86/libatcl_bitflow.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatcl_bitflow.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatcl_bitflow.so
LIB_INSTALLS_Linux += ../os/linux-x86/libatcore.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatcore.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatcore.so
LIB_INSTALLS_Linux += ../os/linux-x86/libatdevregcam.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatdevregcam.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatdevregcam.so
LIB_INSTALLS_Linux += ../os/linux-x86/libatdevsimcam.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatdevsimcam.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatdevsimcam.so
LIB_INSTALLS_Linux += ../os/linux-x86/libatmcd.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatmcd.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatmcd.so
LIB_INSTALLS_Linux += ../os/linux-x86/libatusb_libusb.so.3.5.30007.0
LIB_INSTALLS_Linux += ../os/linux-x86/libatusb_libusb.so.3
LIB_INSTALLS_Linux += ../os/linux-x86/libatusb_libusb.so

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

