############################################################################
# libs/libzmq/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
#
###########################################################################

include $(TOPDIR)/Make.defs

ifeq ($(CONFIG_ZEROMQ),y)
include libzmq.defs
endif

# # Object Files

# AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))
# CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))
CPPOBJS = $(CPPSRCS:.cpp=$(OBJEXT))

# SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS) $(CPPSRCS)
SRCS = $(CSRCS) $(CPPSRCS)
# OBJS = $(AOBJS) $(COBJS) $(CXXOBJS) $(CPPOBJS)
OBJS = $(COBJS) $(CPPOBJS)

BIN = libzmq$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean context

# $(AOBJS): %$(OBJEXT): %.S
# 	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# $(CXXOBJS): %$(OBJEXT): %.cxx
# 	$(call COMPILEXX, $<, $@)

$(CPPOBJS): %$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

context::

# makedepfile: $(CXXSRCS:.cxx=.ddx) $(CPPSRCS:.cpp=.ddp)
makedepfile: $(CPPSRCS:.cpp=.ddp)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
