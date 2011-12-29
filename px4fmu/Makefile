
SRCROOT		:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILDROOT	:= $(abspath $(SRCROOT)/../build/$(lastword $(subst /, ,$(SRCROOT))))
NUTTXROOT	:= $(SRCROOT)/../dist
LOCAL_NUTTX	:= $(SRCROOT)/nuttx
LOCAL_APPS	:= $(SRCROOT)/apps
CONFIG		 = px4fmu/ostest

include ../etc/nuttx-build.mk
