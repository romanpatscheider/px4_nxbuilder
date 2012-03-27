#
# Makefile fragment that knows how to combine a working tree with
# the NuttX source tree and build it.
#
# TODO:
#	As configuration is always performed, this cannot be used for
#	incremental builds (build directly in $(BUILDROOT) to achieve that).
#	To fix this, configure.sh needs to be tweaked to only overwrite the
#	configuration files if they have materially changed.
#

#
# The calling Makefile must have set SRCROOT to the root of the working tree,
# and CONFIG must be set to the NuttX configuration that will be built.
#
# BUILDROOT can be set to locate the work area, otherwise it will be
# located in $(SRCROOT)/../build
#
# LOCAL_NUTTX can be set to a directory to be merged with the NuttX source directory.
# LOCAL_APPS can be set to a directory to be merged with the NuttX apps directory.
# If neither are set, they default to $(SRCROOT)/nuttx and $(SRCROOT)/apps respectively
# if either is present.
#
ifeq ($(SRCROOT),)
$(error Must set SRCROOT to the working tree root)
endif

NUTTXROOT	?= $(SRCROOT)/../dist
BUILDROOT	?= $(abspath $(SRCROOT)/../build/$(lastword $(subst /, ,$(SRCROOT))))
LOCAL_NUTTX	:= $(wildcard $(SRCROOT)/nuttx)
LOCAL_APPS	:= $(wildcard $(SRCROOT)/apps)

$(info working in $(BUILDROOT))

METATARGETS	 = installsrc cleanbuild upload-usb-px4fmu

ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS	 = all
endif

ifeq ($(filter $(METATARGETS),$(MAKECMDGOALS)),)
$(MAKECMDGOALS): installsrc
	@echo Building...
	@(cd $(BUILDROOT)/nuttx && make $(MAKECMDGOALS))
endif

installsrc:
	@echo Copying NuttX and stock applications...
	@mkdir -p $(BUILDROOT)
	@rsync -aL --exclude=.svn --exclude=.git $(NUTTXROOT)/* $(BUILDROOT)
ifneq ($(LOCAL_NUTTX),)
	@echo Merging local NuttX tree...
	@echo "rsync -aL --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_NUTTX)/* $(BUILDROOT)/nuttx"
	@rsync -a --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_NUTTX)/* $(BUILDROOT)/nuttx
endif
ifneq ($(LOCAL_APPS),)
	@echo Merging local applications tree...
	@rsync -aL --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_APPS)/* $(BUILDROOT)/apps
endif
	@echo Configuring...
	@(cd $(BUILDROOT)/nuttx/tools && ./configure.sh $(CONFIG))

# Upload targets for all boards
upload-jtag-px4io:
	@echo Needs implementation

upload-usb-px4fmu:
	@echo Attempting to flash PX4FMU board via USB
	@../../px4_bootloader/px_mkfw.py --board_id 5 > px4fmu_prototype.px4
	@../../px4_bootloader/px_mkfw.py --prototype px4fmu_prototype.px4 --image ../build/px4fmu/nuttx/nuttx.bin > px4fmu.px4
	../../px4_bootloader/px_uploader.py px4fmu.px4 --port "/dev/ttyACM5,/dev/ttyACM4,/dev/ttyACM3,/dev/ttyACM2,/dev/ttyACM1,/dev/ttyACM0,\\.\COM14,\\.\COM13,\\.\COM12,\\.\COM11,\\.\COM10,COM9,COM8,COM7,COM6,COM5,COM4,COM3,COM2,COM1,COM0,/dev/tty.usbmodemDEM4,/dev/tty.usbmodemDEM3,/dev/tty.usbmodemDEM2,/dev/tty.usbmodemDEM1"
upload-usb:
	@echo Finished upload attempt.

cleanbuild:
	@echo Cleaning build area...
	@rm -rf $(BUILDROOT)

.PHONY:	$(METATARGETS)
