CC=gcc
CFLAGS=-g -Wall -Wextra
SRCDIR = $(CURDIR)/src
INCDIR = $(CURDIR)/headers
OBJDIR = $(CURDIR)/obj
LIBFLAGS = -lpigpio -pthread

.PHONY: dataproc_test.out
dataproc_test.out: $(OBJDIR)/fifo.o 
	$(CC) $(CFLAGS) dataproc_test.c $< -o $@ $(LIBFLAGS) -I. -I$(INCDIR)

%/fifo.o:
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