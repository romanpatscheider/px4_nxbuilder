
SRCROOT		:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CONFIG		 = px4fmu/nsh

include ../etc/nuttx-build.mk

upload-usb:
	@../../px4_bootloader/px_mkfw.py --board_id 5 > px4flow_prototype.px4
	@../../px4_bootloader/px_mkfw.py --prototype px4flow_prototype.px4 --image ../build/px4fmu/nuttx/nuttx.bin > px4flow.px4
	@if test -e /dev/ttyACM0; then ../../px4_bootloader/px_uploader.py px4flow.px4 --port /dev/ttyACM0; else echo Tried /dev/ttyACM0, no device..; fi
	@if test -e /dev/ttyACM1; then ../../px4_bootloader/px_uploader.py px4flow.px4 --port /dev/ttyACM1; else echo Tried /dev/ttyACM1, no device..; fi
	@if test -e /dev/ttyACM2; then ../../px4_bootloader/px_uploader.py px4flow.px4 --port /dev/ttyACM2; else echo Tried /dev/ttyACM2, no device..; fi
	@if test -e /dev/ttyACM3; then ../../px4_bootloader/px_uploader.py px4flow.px4 --port /dev/ttyACM3; else echo Tried /dev/ttyACM3, no device..; fi
	../../px4_bootloader/px_uploader.py px4flow.px4 --port /dev/tty.usbmodemDEM1
	#@if test -e /dev/tty.usbmodemDEM1; then echo Found port: /dev/tty.usbmodemDEM1; ../../px4_bootloader/px_uploader.py px4flow.px4 --port /dev/tty.usbmodemDEM1; else echo Tried /dev/tty.usbmodemDEM1, no device..; fi

