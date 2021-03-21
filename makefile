CC=gcc
DEPS=code.h
CFLAGS=-g -Wall
OBJ=code.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

code: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -lwiringPi

new_code: 