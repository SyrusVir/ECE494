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

#include <pinpoller.h>
/* #include <pigpio.h>
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
} pin_poller_t; */

//Obtain pointer to configured pin poller. spin_lock must be a pointer to a configured pthread spinlock. If NULL returned, critical failure has occured
pin_poller_t* pinPollerInit(pthread_spinlock_t* spin_lock, uint8_t poller_pin, uint8_t trigger_level, uint32_t delay_usec)
{
    /**Function: pinPollerInit
     * Parameters:  pthread_spinock_t* spin_lock - pointer to a configured pthread spinlock. Be sure to free 
     *                                              this once pinPollerMain terminates
     *              uint8_t poller_pin - the GPIO pin of the RPI to poll.
     *              uint8_t trigger_level - If poller_pin is sampled at this logic level, the poller locks spin_lock
     *              uint32_t delay_usec - how long to keep spin_lock locked before releasing
     * 
     * Return: pin_poller_t* poller - pointer to allocated space containing the poller structure
     * 
     * Description: Instantiates a poller structure in allocated memory with the user parameters, returning a pointer.
     *              The spin_lock must be pre-configured as it is expected that this lock will be shared with other
     *              threads/processes. It is user's responsiblity to deallocate this spin_lock. The previous 
     *              configuration of poller_pin is cleared and it is set as an input.
     */

    if (gpioInitialise() < 0) return NULL;
    pin_poller_t* poller = malloc(sizeof(pin_poller_t));
    poller->poller_pin = poller_pin;
    poller->exit_flag = 0;
    poller->delay_usec = delay_usec;
    poller->trigger_level = (trigger_level ? 1 : 0); //return 1 if trigger_level non_zero, 0 otherwise
    poller->spin_lock = spin_lock;
    poller->return_status = 0;

    gpioSetMode(poller_pin, PI_INPUT);  //configured provided pin as input

    return poller;
} //end pinPollerInit()

void pinPollerExit(pin_poller_t* poller)
{
    poller->exit_flag = 1;
} //end pinPollerExit

//Free poller memory. The spin lock is NOT freed here and must be free'd elsewhere.
int pinPollerDestroy(pin_poller_t* poller)
{   
    free(poller);
    return 0;
} //end pinPollerDestroy

//Returns 0 if no event has occurred. Returns 1 if an event has occurred and -1 if EINVAL error occurred.
inline int pinPollerCheckIn(pin_poller_t* poller)
{
    /**Function: pinPollerCheckin
     * Parameter: pin_poller_t* poller - a configured poller object with a valid spinlock reference
     * Return: 0, 1,-1 - 0 = Poller has not detected an event; 1 = poller has detected event; -1 = invalid spin lock
     * 
     * Description: Use to see if the poller has detected an event. A check for an event consists of trying
     *              to lock the spin lock shared with the poller.
     * 
     *              pthread_spin_trylock has 3 return values:
     *              1) 0 = The spin lock has been acquired. This indicates the poller has NOT detected an event.
     *                      In this situation, 0 is retrned by pinPollerCheckIn()
     *              2) EBUSY = indicates that the spin lock is held by another thread, i.e. the polling thread.
     *                         This indicates an event has occured. In this scenario, 1 is returned by pin PollerCheckIn()
     *              3) EINVAL = the spin lock reference is invalid. A critical error. -1 is returned by pinPollerCheckIn().
     *                          The actual error can be retrieved from errno.
     */
    int status = pthread_spin_trylock(poller->spin_lock);
    if (status == 0) 
    {
        pthread_spin_unlock(poller->spin_lock);
        return 0;
    }
    else return (status == EINVAL ? -1 : 1);
}

//Main polling loop. Expects sos_poller_arg to be of type (pin_poller_t*). Pass as argument to pthread_create().
void* pinPollerMain(void* sos_poller_arg)
{
    /**Function: pinPollerMain
     * Parameters: void* sos_poller_arg - Pass a pointer to a configured pin poller, i.e. (pin_poller_t*)
     * Return:  None
     * 
     * Description: Intended to be function argument to pthread_create(). First, local copies of the poller
     *              parameters are saved for quicker access. Then the main loop is entered.
     * 
     *              If the logic level trigger_level is observed on pin, the spin lock pointed to by lock
     *              is locked. After delay_usec microseconds, the lock is released. Repeat the process until
     *              exit_flag is raised.
     */
    pin_poller_t* poller = (pin_poller_t*) sos_poller_arg;

    //local copies of variables for quicker access within main loop
    uint8_t local_pin = poller->poller_pin;
    uint8_t local_trigger_level = poller->trigger_level; 
    uint32_t local_delay_usec = poller->delay_usec;
    pthread_spinlock_t* local_lock = poller->spin_lock; 

    //main polling loop
    while(!poller->exit_flag)
    {
        //poll until trigger_level or exit_flag. 
        //gpioRead first to take advantage of short-circuit conditional evaluation
        while (gpioRead(local_pin) != local_trigger_level && !poller->exit_flag); 
        
        if (poller->exit_flag) break;   //exit loop if exit_flag set
        
        //Lock the spin lock to signal an event. If lock invalid, exit with EINVAL error
        if(pthread_spin_lock(local_lock) == EINVAL)
        {
            printf("lock error\n\r");
            poller->return_status = EINVAL;
            return NULL;
        } 
        
        //to reduce jitter caused by extended events, repeat delay until the target pin is no longer 
        //at the specifed level
        do
        {
            gpioDelay(local_delay_usec);            //delay for specified microseconds
        } while (gpioRead(local_pin) == local_trigger_level && !poller->exit_flag);
        
        pthread_spin_unlock(local_lock);        //unlock mutex, allowing emission     
    } //end while(!poller->exit_flag)

    return NULL;
}