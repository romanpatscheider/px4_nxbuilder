
SRCROOT		:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CONFIG		 = px4fmu/nsh

include ../etc/nuttx-build.mk

upload-mac:
	sh upload-mac.sh
	
upload-linux:
	sh upload-linux.sh
	
upload-usb: upload-usb-px4fmu

