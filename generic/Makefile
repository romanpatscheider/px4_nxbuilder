#
# NXbuilder skeleton that builds the generic NuttX configurations only.
#

ifeq ($(CONFIG),)
$(error You must specify CONFIG=<valid NuttX target> to build the generic target.)
endif

SRCROOT		:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

include ../etc/nuttx-build.mk
