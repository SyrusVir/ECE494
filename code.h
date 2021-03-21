#ifndef CODE_H
#define CODE_H
#define _GNU_SOURCE

//Libraries to be used
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
// #include <xmlrpc-c/base.h>
// #include <xmlrpc-c/client.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>

//Defines pin locations and TDC clock speed
const double CLOCK_SPEED = 9600000.0;
const int PIN_TRIG = 0;
const int PIN_EN = 2;
const int PIN_START = 9;
const int PIN_STOP = 4;
const int PIN_INTB = 3;
const int PIN_CLK = 7;

//Function prototypes
uint32_t* getValue(int, uint8_t*, int);
uint32_t* convertToLongArray(uint8_t*);
int initTDC(int);
int startMeas(int);
double getToF(int);
void readDataToFile(int, FILE*);
void deconfigurePins(uint32_t*);
void configurePins(int);
uint32_t* setClockParams(int);

#endif
