############################################################################
# apps/nshlib/Makefile
#
#   Copyright (C) 2012 Michael Smith. All rights reserved.
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  o Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  o Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# ARM CMSIS DSP library

ASRCS		=
CSRCS		= $(wildcard CMSIS/DSP_Lib/Source/*/*.c)

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ifeq ($(WINTOOL),y)
  BIN		= "${shell cygpath -w  $(APPDIR)/libapps$(LIBEXT)}"
else
  BIN		= "$(APPDIR)/libapps$(LIBEXT)"
endif

ROOTDEPPATH	= --dep-path .
VPATH		= 
CFLAGS		+= -I CMSIS/Include \
		   -I Device/ARM/ARMCM0/Include \
		   -I Device/ARM/ARMCM3/Include \
		   -I Device/ARM/ARMCM4/Include

# Pick the right CPU options - it would be nice to infer these.
#
ifeq ($(CONFIG_CMSIS_DSPLIB_TARGET),CortexM0)
  CFLAGS +=	-DARM_MATH_CM0
else ifeq ($(CONFIG_CMSIS_DSPLIB_TARGET),CortexM3)
  CFLAGS +=	-DARM_MATH_CM3
else ifeq ($(CONFIG_CMSIS_DSPLIB_TARGET),CortexM4)
  ifeq ($(CONFIG_ARCH_FPU),y)
    CFLAGS +=	-DARM_MATH_CM4
  else
    CFLAGS +=	-DARM_MATH_CM3
  endif
else
  $(error You must set CONFIG_CMSIS_DSPLIB_TARGET before using CMSISlib)
endif

# Build targets

all:	.built
.PHONY: context .depend depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $(BIN), $${obj}); \
	done ; )
	@touch .built

context:

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) \
	  $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@rm -f *.o *~ .*.swp .built
	$(call CLEAN)

distclean: clean
	@rm -f Make.dep .depend

-include Make.dep

