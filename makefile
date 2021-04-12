CC=gcc
CFLAGS=-g -O -Wall -Wextra

# deps holds list of object and static lib files prduced in submodules
DEPS = $(CURDIR)/Threaded-Logger/liblogger.a\
$(CURDIR)/Threaded-TCP/libtcphandler.a\
$(CURDIR)/scanning-mirror/scanmirror.o\
$(CURDIR)/scanning-mirror/pinpoller.o\
$(CURDIR)/MLD-019/MLD019.o\
$(CURDIR)/Data-Processor/libdatproc.a

INC=$(dir $(DEPS)) $(CURDIR)
INCS = $(addprefix -I ,$(INC))
CLEANDEPS = $(addsuffix .clean, $(DEPS))

LIBFLAGS = -lpigpio -pthread -lm

.PHONY: clean subclean $(CLEANDEPS) suball $(DEPS) $(SUBOBJ)

# This rule makes the object/library files in all submodules
suball: $(DEPS)
$(DEPS):
	$(MAKE) -C $(@D) $(@F)

# First cleans submodule directoryies then cleans the current directory
subclean: $(CLEANDEPS)
	rm -f *.o *.a
$(CLEANDEPS): %.clean:
	$(MAKE) -C $(*D) clean 


tdc_util.o: tdc_util.c tdc_util.h
	$(CC) $(CFLAGS) -c $< -o $@ -I$(INCDIR) -I. $(LIBFLAGS)

# Pattern rule for compiling any program using submodules
%.out: $(DEPS) tdc_util.o
	$(CC) $(CFLAGS) $(subst .out,.c,$@) $^ -o $@ $(addprefix -I,$(INC)) $(LIBFLAGS)
