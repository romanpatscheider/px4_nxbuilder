
SRCROOT		:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CONFIG		 = px4fmu/nsh

include ../etc/nuttx-build.mk

upload-mac:
	sh upload-mac.sh
	