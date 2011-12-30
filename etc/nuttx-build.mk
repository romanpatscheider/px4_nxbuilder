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

METATARGETS	 = installsrc cleanbuild

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
	@rsync -a --exclude=.svn --exclude=.git $(NUTTXROOT)/* $(BUILDROOT)
ifneq ($(LOCAL_NUTTX),)
	@echo Merging local NuttX tree...
	@rsync -a --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_NUTTX)/* $(BUILDROOT)/nuttx
endif
ifneq ($(LOCAL_APPS),)
	@echo Merging local applications tree...
	@rsync -a --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_APPS)/* $(BUILDROOT)/apps
endif
	@echo Configuring...
	@(cd $(BUILDROOT)/nuttx/tools && ./configure.sh $(CONFIG))

cleanbuild:
	@echo Cleaning build area...
	@rm -rf $(BUILDROOT)

.PHONY:	$(METATARGETS)
