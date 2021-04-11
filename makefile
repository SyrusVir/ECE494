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

%.out: $(DEPS) tdc.o
	
	$(CC) $(CFLAGS) $(subst .out,.c,$@) $^ -o $@ $(addprefix -I,$(INC)) $(LIBFLAGS)

tdc.o: tdc.c tdc.h
	$(CC) $(CFLAGS) -c $< $(INCS)  $(LIBFLAGS)

tdc_util.o: tdc_util.c logger.o
	$(CC) $(CFLAGS) -c tdc_util.c -o $(OBJDIR)/$@ -I$(INCDIR) -I. $(LIBFLAGS)
 
 .PHONY: clean
 clean:
	rm $(OBJDIR)/*.o
