CC=gcc
CFLAGS=-g -Wall -Wextra
SRCDIR = $(CURDIR)/src
INCDIR = $(CURDIR)/headers
OBJDIR = $(CURDIR)/obj
LIBFLAGS = -lpigpio -pthread -lm
OBJS = fifo.o tcp_handler.o logger.o data_processor.o tdc.o

.PHONY: all
all: $(OBJS)

tdc_test.out: $(OBJS)
	$(CC) $(CFLAGS) tdc_test.c -o $@ $(addprefix $(OBJDIR)/,$(OBJS)) -I$(INCDIR) -I. $(LIBFLAGS)

fifo.o:
	$(CC) $(CFLAGS) -c $(SRCDIR)/fifo.c -o $(OBJDIR)/$@ -I$(INCDIR) -pthread

tcp_handler.o: fifo.o
	$(CC) $(CFLAGS) -c $(SRCDIR)/tcp_handler.c -o $(OBJDIR)/$@ -I$(INCDIR) -pthread

logger.o: fifo.o
	$(CC) $(CFLAGS) -c $(SRCDIR)/logger.c -o $(OBJDIR)/$@ -I$(INCDIR) $(LIBFLAGS)

data_processor.o: logger.o
	$(CC) $(CFLAGS) -c $(SRCDIR)/data_processor.c -o $(OBJDIR)/$@ -I$(INCDIR) -I. -pthread

tdc.o: logger.o
	$(CC) $(CFLAGS) -c tdc.c -o $(OBJDIR)/$@ -I$(INCDIR) -I. $(LIBFLAGS)
 
 .PHONY: clean
 clean:
	rm $(OBJDIR)/*.o