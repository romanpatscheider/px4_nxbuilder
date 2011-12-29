#
# NXbuilder skeleton that builds the generic NuttX configurations only.
#

ifeq ($(CONFIG),)
$(error You must specify CONFIG=<valid NuttX target> to build the generic target.)
endif

SRCROOT		:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILDROOT	:= $(abspath $(SRCROOT)/../build/$(lastword $(subst /, ,$(SRCROOT))))
NUTTXROOT	:= $(SRCROOT)/../dist

$(info Building in $(BUILDROOT))

include ../etc/nuttx-build.mk
