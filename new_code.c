#define _GNU_SOURCE
#include <pigpio.h>
#include "fifo.h"
#include "logger.h"
#include "MLD019.h"
#include "pinpoller.h"
#include "scanmirror.h"

//CPU core assignments
#define MAIN_CPU 1      // isolated CPU core on which to execute the main control thread
#define POLLER_CPU 2    // isolated CPU core on which to execute the pin polling loop

//Mirror definitions
#define MIRROR_FREQ_PIN 18      // physical pin 12; must be a pin that suppports PWM; controls mirror RPM
#define MIRROR_SPEED_PIN 23     // physical pin 16; input for the mirror ATSPEED output; TTL LO when at speed
#define MIRROR_ENABLE_PIN 24    // physical pin 18; drive TTL HI to enable
#define MIRROR_RPM 1000         // desired rotational speed of scanning mirror  

//Start-of-scan definitions
#define SOS_PIN 25              // physical pin 22; input for the start-of-scan detector output; 
#define SOS_POLARITY 0          // SOS detector outputs LO when triggered
#define SOS_DELAY_USEC 10       // How long to pause laser emission when SOS detector is triggered

//Laser definitions
#define LASER_ENABLE_PIN 5      // physical pin 29; must be TTL HI to allow emission
#define LASER_SHUTTER_PIN 6     // physical pin 31; must be TTL HI to allow emission; wait 500 ms after raising
#define LASER_PULSE_PIN 13      // physical pin 33; outputs trigger pulses to laser driver

//Laser driver serial definitions
#define MLD_SERIAL_SERTTY "/dev/ttyAMA0"

//TDC definitions
#define TDC_CLK_PIN 4           // physical pin 7; must be a GPCLK0 pin
#define TDC_ENABLE_PIN 17       // physical pin 11; active LO
#define TDC_INT_PIN 27          // physical pin 13
#define TDC_START_PIN 22        // physical pin 15
#define TDC_STOP_PIN 0          // physical pin 27
#define TDC_CLK_FREQ 9600000    // reference clock frequency provided to TDC; allowed vals: 0 (off) or 4689 to 250M
#define TDC_BAUD 15625000       // SPI baud rate;; divides 250MHz base SPI clock by a power of two (16 = 2^4) 

int main(int argc, char** argv)
{
    cpu_set_t cpu_mask;         // cpu set mask for setting thread affinities
    pthread_spinlock_t lock;    // spin lock to synchronize between polling thread and main thread
    pthread_attr_t attr;        // pthread attributes used to assign cpu affinities
    int status;                 // int to hold return value of some function calls

    /**Configure 1 usec gpio sample speed timed using PCM module
    *  This allows detection of the short SOS trigger signal and
    *  frees the PWM module to control the scanning mirror.
    */
    status = gpioCfgClock(1,PI_CLOCK_PCM,0);   
    
    status = gpioInitialise();
    if (status < 0)
    {
        perror("CRITICAL ERROR IN gpioInitialise()");
        return status;
    }
    else
    {
        printf("Pigpio version: %d\n", status);
    }

    /********** Configuring SOS pin poller **********/
    pin_poller_t* sos_poller;   // pointer to pin poller struct
    pthread_t sos_poller_tid;   // thread ID of SOS pin polling thread
    
    //configuring poller
    pthread_spin_init(&lock,0); //lock will be shared between threads, NOT processes
    sos_poller = pinPollerInit(&lock, SOS_PIN, SOS_POLARITY, SOS_DELAY_USEC);
    
    //configuring polling thread
    CPU_ZERO(&cpu_mask);
    CPU_SET(POLLER_CPU, &cpu_mask);
    pthread_attr_setaffinity_np(&attr,sizeof(cpu_mask), &cpu_mask);     // assign polling thread to isolated cpu
    pthread_create(&sos_poller_tid, &attr,&pinPollerMain,sos_poller);   // start polling thread
    /************************************************/
    
    /********** Configuring Scanning Mirror **********/
    mirror_t mirror = {
        .ATSPEED_PIN = MIRROR_SPEED_PIN,
        .FREQ_PIN = MIRROR_FREQ_PIN,
        .ENABLE_PIN = MIRROR_ENABLE_PIN
    };

    status = mirrorConfig(mirror);
    if (status != 0)
    {
        perror("ERROR in mirrorConfig()");
        return -1;
    }
    /*************************************************/

    /********** TDC Configuration **********/
    struct TDC {
        uint8_t CLK_PIN;
        uint8_t ENABLE_PIN;
        uint8_t INT_PIN;
        uint8_t START_PIN;
        uint8_t STOP_PIN;
        unsigned int CLK_FREQ;
        int spi_handle;
    };

    struct TDC tdc = {
        .CLK_PIN = TDC_CLK_PIN,
        .ENABLE_PIN = TDC_ENABLE_PIN,
        .INT_PIN = TDC_INT_PIN,
        .START_PIN = TDC_START_PIN,
        .STOP_PIN = TDC_STOP_PIN,
        .CLK_FREQ = TDC_CLK_FREQ
    };
    
    gpioSetMode(tdc.ENABLE_PIN, PI_OUTPUT);     // active LOW
    gpioSetMode(tdc.INT_PIN, PI_INPUT);         // active LOW        
    gpioSetPullUpDown(tdc.INT_PIN, PI_PUD_UP);  // INT pin is open drain, active LOW
    gpioSetMode(tdc.START_PIN, PI_OUTPUT);
    gpioSetMode(tdc.STOP_PIN, PI_OUTPUT);
    gpioSetMode(tdc.CLK_PIN, PI_ALT0);
    
    gpioHardwareClock(tdc.CLK_PIN,tdc.CLK_FREQ);
    
    gpioWrite(tdc.ENABLE_PIN, 0);

    tdc.spi_handle = spiOpen(0, TDC_BAUD,0);

    char tx_buff[] = {0x41, 0x00};
    spiWrite(tdc.spi_handle, tx_buff, sizeof(tx_buff));
    /****************************************/
    
    /********** Exit Routines **********/
    //exit and destroyo pin poller thread
    pinPollerExit(sos_poller);
    pthread_join(sos_poller_tid, NULL);
    pinPollerDestroy(sos_poller);

    //stop and disable mirror
    mirrorSetRPM(mirror, 0);
    mirrorDisable(mirror);

    //Disable Laser
    gpioWrite(LASER_SHUTTER_PIN,0);
    gpioWrite(LASER_ENABLE_PIN,0);

    //stop TDC clock
    gpioHardwareClock(TDC_CLK_PIN, 0);

    gpioTerminate();
    /***********************************/
    
    return 0;
}