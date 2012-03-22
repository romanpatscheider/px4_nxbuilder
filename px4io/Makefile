#
# STM33F100 build and flash (ARM-USB-Tiny)
#

BINARY     = ../build/px4io/nuttx/nuttx.elf

LIBOPENCM3 ?= ../../libopencm3
OPENOCD    ?= ../../../sat/bin/openocd

JTAGCONFIG ?= ../etc/olimex-jtag-tiny.cfg
CHIPCONFIG ?= ../etc/stm32f100.cfg 


SRCROOT	  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CONFIG	   = px4io/nsh

include ../etc/nuttx-build.mk

upload: flash-firmware

flash-firmware:
	$(OPENOCD) --search ../../ -f $(JTAGCONFIG) -f $(CHIPCONFIG) -c init -c "reset halt" -c "flash write_image erase $(BINARY) " -c "reset run" -c shutdown
