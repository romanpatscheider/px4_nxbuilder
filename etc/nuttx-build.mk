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
# The calling Makefile must have set SRCROOT and NUTTXROOT to the
# root of the working tree and the top-level NuttX tree.
# Both should contain 'nuttx' and 'apps' directories as required.
#
# CONFIG must be set to the NuttX configuration that will be built.
#
# BUILDROOT can be set to locate the work area, otherwise it will be
# located in /tmp/nxbuild
#
# LOCAL_NUTTX can be set to a directory to be merged with the NuttX source directory.
# LOCAL_APPS can be set to a directory to be merged with the NuttX apps directory.
#
ifeq ($(SRCROOT),)
$(error Must set SRCROOT to the working tree root)
endif
ifeq ($(NUTTXROOT),)
$(error Must set NUTTXROOT to the toplevel NuttX checkout/distribution directory)
endif

#BUILDROOT	?= $(shell mktemp -d -t nxbuild)
BUILDROOT	?= /tmp/nxbuild
$(info working in $(BUILDROOT))

ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS	 = all
endif

$(MAKECMDGOALS): installsrc
	@echo Building...
	@(cd $(BUILDROOT)/nuttx && make $(MAKECMDGOALS))

installsrc:
	@echo Copying NuttX and stock applications...
	@rsync -av --exclude=.svn --exclude=.git $(NUTTXROOT)/* $(BUILDROOT)
ifneq ($(LOCAL_NUTTX),)
	@echo Merging local NuttX tree...
	@rsync -av --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_NUTTX)/* $(BUILDROOT)/nuttx
endif
ifneq ($(LOCAL_APPS),)
	@echo Merging local applications tree...
	@rsync -av --exclude=.svn --exclude=.git --exclude=$(BUILDROOT) $(LOCAL_APPS)/* $(BUILDROOT)/apps
endif
	@echo Configuring...
	@(cd $(BUILDROOT)/nuttx/tools && ./configure.sh $(CONFIG))

cleanbuild:
	@echo Cleaning build area...
	@rm -rf $(BUILDROOT)
	@echo Remaking build area...
	@mkdir -p $(BUILDROOT)

.PHONY:	installsrc cleanbuild
