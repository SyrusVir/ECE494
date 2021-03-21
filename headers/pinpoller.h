/**File: sos_poller.c
 * Author: Jude Alnas
 * date: March 16, 2021
 * 
 * Description: The purpose of these functions is to implement a thread-safe poller that uses busy waiting
 *              to detect as quicly as possible when an input IO pin reads a specified level. This is done
 *              by executing the polling loop in a thread that (ideally) executes on a dedicated CPU core.
 *              When the specified event is detected, a spin lock is locked and is unlocked after a specified 
 *              number of microseconds has elapsed. Spin locks are used instead of mutexes for decreased overhead.
 *              
 *              Usage steps:
 *              1)  call pinPollerInit() - this configures the pin poller
 *              2)  call pthread_create() with pinPollerMain() as the function parameter - this starts the main polling loop
 *                                                                                     in the specified thread
 *              3)  Raise exit_flag - using the pointer returned by pinPollerInit(), raise the exit_flag to terminate the
 *                                    polling loop
 *              4)  call pinPollerDestroy() - this simply frees the memory space allocated to hold the pin poller structure
 * 
 *              GPIO IO interfacing is done using the pigpio library
 * 
 **/

#ifndef _PINPOLLER_H_
#define _PINPOLLER_H_
#include <pigpio.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>

typedef struct
{
    pthread_spinlock_t* spin_lock;    //mutex to lock to indicate [trigger_level] was detected at pin [sos_detector_pin]
    uint8_t exit_flag;  //raise flag to exit poller and poller thread
    uint8_t trigger_level;  //1 or 0; GPIO level poller looks for
    uint8_t poller_pin;   //RPi pin to poll for [trigger_level]
    int return_status;      //final status of poller upon exit
    uint32_t delay_usec;    // after delay_usec, spin_lock is unlocked 
} pin_poller_t;

//Obtain pointer to configured pin poller. spin_lock must be a pointer to a configured pthread spinlock. If NULL returned, critical failure has occured
pin_poller_t* pinPollerInit(pthread_spinlock_t* spin_lock, uint8_t poller_pin, uint8_t trigger_level, uint32_t delay_usec);

//Utility function that signals the poller to exit
void pinPollerExit(pin_poller_t* poller);

//Free poller memory. The spin lock is NOT freed here and must be free'd elsewhere.
int pinPollerDestroy(pin_poller_t* poller);

//Returns 0 if no event has occurred. Returns 1 if an event has occurred and -1 if EINVAL error occurred.
int pinPollerCheckIn(pin_poller_t* poller);

//Main polling loop. Expects sos_poller_arg to be of type (pin_poller_t*). Pass as argument to pthread_create().
void* pinPollerMain(void* sos_poller_arg);
#endif