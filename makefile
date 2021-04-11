CC=gcc
CFLAGS=-g -O -Wall -Wextra

DEPS = $(CURDIR)/Threaded-Logger/liblogger.a\
$(CURDIR)/Threaded-TCP/libtcphandler.a\
$(CURDIR)/scanning-mirror/scanmirror.o\
$(CURDIR)/scanning-mirror/pinpoller.o\
$(CURDIR)/MLD-019/MLD019.o\
$(CURDIR)/Data-Processor/libdatproc.a

INC=$(dir $(DEPS)) $(CURDIR)
INCS = $(addprefix -I ,$(INC))
CLEANDEPS = $(addsuffix .clean, $(DEPS))

LIBFLAGS = -lpigpio -pthread

.PHONY: clean subclean $(CLEANDEPS) suball $(DEPS) $(SUBOBJ)

suball: $(DEPS)
$(DEPS):
	$(MAKE) -C $(@D) $(@F)

clean:
	rm -f *.o *.a && make subclean

subclean: $(CLEANDEPS)
	rm -f *.o *.a
$(CLEANDEPS): %.clean:
	$(MAKE) -C $(*D) clean

tdc_util.o: tdc_util.c tdc_util.h
	$(CC) $(CFLAGS) -c $< -o $@ -I$(INCDIR) -I. $(LIBFLAGS)

%.out: $(DEPS) tdc_util.o
	$(CC) $(CFLAGS) $(subst .out,.c,$@) $^ -o $@ $(addprefix -I,$(INC)) $(LIBFLAGS)
