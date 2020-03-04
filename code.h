#ifndef CODE_H
#define CODE_H

//Libraries to be used
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>

//Defines pin locations and TDC clock speed
const double CLOCK_SPEED = 9600000.0;
const int PIN_TRIG = 2;
const int PIN_EN = 3;
const int PIN_START = 4;
const int PIN_STOP = 5;
const int PIN_INTB = 6;
const int PIN_CLK = 7;

//Function prototypes
uint32_t* getValue(int, uint8_t*, int);
uint32_t* convertToLongArray(uint8_t*);
int initTDC(int);
int startMeas(int);
double getToF(int);
void readDataToFile(int, FILE*);
void deconfigurePins(uint32_t*);
uint32_t* configurePins(int);

#endif
