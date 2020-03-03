#ifndef CODE_H
#define CODE_H

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>

const char *device = "/dev/spidev0.0";
const double CLOCK_SPEED = 9600000.0;
const int PIN_TRIG = 2;
const int PIN_EN = 3;
const int PIN_START = 4;
const int PIN_STOP = 5;
const int PIN_INTB = 6;
const int PIN_CLK = 7;

uint32_t* getValue(int, uint8_t*, int);
uint32_t* convertToLongArray(uint8_t*);
int initTDC(int);
int startMeas(int);
double getToF(int);
void readDataToFile(int, FILE*);
void deconfigurePins(uint32_t*);
uint32_t* configurePins(int);
void trigInt(void);
void intTrigger(void);

#endif