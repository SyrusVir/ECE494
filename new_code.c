#define _GNU_SOURCE
#include <pigpio.h>
#include <sys/sysinfo.h>
#include <signal.h>
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
#define SOS_PIN 7               // physical pin 26; input for the start-of-scan detector output; 
#define SOS_POLARITY 0          // SOS detector outputs LO when triggered
#define SOS_DELAY_USEC 10       // How long to pause laser emission when SOS detector is triggered

//Laser definitions
#define LASER_ENABLE_PIN 26     // physical pin 37; must be TTL HI to allow emission
#define LASER_SHUTTER_PIN 6     // physical pin 31; must be TTL HI to allow emission; wait 500 ms after raising
#define LASER_PULSE_PIN 23      // physical pin 16; outputs trigger pulses to laser driver

//Laser driver serial definitions
#define MLD_SERIAL_SERTTY "/dev/ttyAMA0"

//TDC definitions
#define TDC_CLK_PIN 4           // physical pin 7; must be a GPCLK0 pin
#define TDC_ENABLE_PIN 27       // physical pin 13; active LO
#define TDC_INT_PIN 22          // physical pin 15
//#define TDC_START_PIN 23        // physical pin 16
#define TDC_STOP_PIN 0          // physical pin 27
#define TDC_CLK_FREQ 9600000    // reference clock frequency provided to TDC; allowed vals: 0 (off) or 4689 to 250M
#define TDC_BAUD 15625000       // SPI baud rate;; divides 250MHz base SPI clock by a power of two (250M/TDC_BAUD = 16 = 2^4) 
#define TDC_TIMEOUT_USEC 50     // How long to wait for the TDC_INT_PIN to go lo before abandoning measurement

int main_stop_flag = 0;
void catchSigInt()
{
	main_stop_flag = 1;
}

void reverse8bitArr(uint8_t* arr, int arr_size)
{
    uint8_t temp;
    for (int i = 0; i < arr_size/2; i++)
    {
        temp = arr[i];
        arr[i] = arr[arr_size - 1 -i];
        arr[arr_size - 1 - i] = temp;
    }
}

int main(int argc, char** argv)
{
    gpioSetSignalFunc(SIGINT, catchSigInt);    // reassign interrupt signal

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
    /****************** Configuring CPU masks ******************/
    // cpu for main thread
    cpu_set_t main_cpu;
    CPU_ZERO(&main_cpu);
    CPU_SET(MAIN_CPU, &main_cpu);
    
    // cpu for polling thread
    cpu_set_t poller_cpu;
    CPU_ZERO(&poller_cpu);
    CPU_SET(POLLER_CPU, &poller_cpu);

    cpu_set_t non_isol_cpu;
    // initialize non_isol_cpu as set of all available cores
    for (int i = 0; i < get_nprocs(); i++) CPU_SET(i, &non_isol_cpu);
    
    // use XOR to remove cpus assigned to main and poller
    CPU_XOR(&non_isol_cpu, &non_isol_cpu, &main_cpu);
    CPU_XOR(&non_isol_cpu, &non_isol_cpu, &poller_cpu);

    /********************`***************************************/
    /********** Configuring SOS pin poller **********/
    pin_poller_t* sos_poller;   // pointer to pin poller struct
    pthread_t sos_poller_tid;   // thread ID of SOS pin polling thread
    
    //configuring poller
    pthread_spin_init(&lock,0); //lock will be shared between threads, NOT processes
    sos_poller = pinPollerInit(&lock, SOS_PIN, SOS_POLARITY, SOS_DELAY_USEC);
    
    //configuring polling thread
    pthread_attr_setaffinity_np(&attr,sizeof(poller_cpu), &poller_cpu);     // assign polling thread to isolated cpu
    pthread_create(&sos_poller_tid, &attr,&pinPollerMain,sos_poller);   // start polling thread
    /************************************************/
    
    /*************** Logger Configuration ***************/
    pthread_t logger_tid;
    logger_t* logger = loggerCreate(100);
    pthread_attr_setaffinity_np(&attr, sizeof(non_isol_cpu), &non_isol_cpu);
    pthread_create(logger_tid, &attr, &loggerMain, logger);
    /****************************************************/
    
    /**************** TCP Handler thread configuration ****************/
    // TODO: finish tcp handler header file
    /******************************************************************/

    /******************** MLD019 Serial Comm Config ********************/
    mld_t* mld = mldInit(MLD_SERIAL_SERTTY);
    
    /*******************************************************************/
    
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
    
    // configuring tdc pins
    gpioSetMode(tdc.ENABLE_PIN, PI_OUTPUT);     // active LOW
    gpioSetMode(tdc.INT_PIN, PI_INPUT);         // active LOW        
    gpioSetPullUpDown(tdc.INT_PIN, PI_PUD_UP);  // INT pin is open drain, active LOW
    gpioSetMode(tdc.START_PIN, PI_OUTPUT);
    gpioSetMode(tdc.STOP_PIN, PI_OUTPUT);
    gpioSetMode(tdc.CLK_PIN, PI_ALT0);
    
    gpioHardwareClock(tdc.CLK_PIN,tdc.CLK_FREQ);
    
    gpioWrite(tdc.ENABLE_PIN, 0);

    // Configure TDC 
    tdc.spi_handle = spiOpen(0, TDC_BAUD,
            0b00 |          // Positive (MSb 0) clock edge centered (LSb 0) on data bit
            (0b000 << 2) |  // CEx is active low
            (0b000 << 5) |  // CEx pins reserved for SPI
            (0 << 8) |      // 0 = Main SPI; 1 = Aux SPI 
            (0 << 9) |      // If 1, 3-wire mode
            ((0 & 0xF) << 10) | // bytes to write before switching to read (N/A if not in 3-wire mode)
            (0 << 14) |         // If 1, tx LSb first (Aux SPI only)
            (0 << 15) |         // If 1, recv LSb first (Aux SPI only)
            ((8 & 0x3F) << 16)  // bits per word; default 8

    );

    //Non-incrementing write to CONFIG2 reg (address 0x01)
    //Clear CONFIG2 to configure 2 calibration clock periods,
    // no averaging, and single stop signal operation
    uint8_t tx_buff[] = {0x41, 0x00};
    spiWrite(tdc.spi_handle, tx_buff, sizeof(tx_buff));
    /****************************************/
    
    /************* Placing main thread on isolated cpu ************/
    pthread_setaffinity_np(pthread_self(), sizeof(main_cpu), &main_cpu);
    /**************************************************************/

    /******************** Data Acquisition Loop ********************/
    int timestamp_sec;  // seconds from the Epoch
    int timestamp_usec; // additional microseconds after timestamp_sec
    
    static const uint8_t meas_commands[2] = {
        0x40,   //Write to CONFIG1
        0x43    //Start measurement in mode 2 with parity and rising edge start, stop, trigger signals 
    };

    static const uint8_t tof_commands[5] = {    // TDC commands for retrieving TOF
        0x10,   //Read TIME1
        0x11,   //Read CLOCK_COUNT1
        0x12,   //Read TIME2
        0x1B,   //Read CALIBRATION
        0x1C    //Read CALIBRATION2
    };

    //starting mirror
    mirrorSetRPM(mirror, MIRROR_RPM);   
    gpioSleep(PI_TIME_RELATIVE,5,0); //give mirror time to get to speed
    
    while (pinPollerCheckIn(sos_poller) == 0);  // wait for first sos detector to trigger to get main laser in known location
    while (pinPollerCheckIn(sos_poller) == 1);  // wait for poller to release lock

    while (1)
    {
        // wait while an event has occured and the stop flag is low 
        while (pinPollerCheckIn(sos_poller) == 1 && !main_stop_flag);

        if (main_stop_flag) break;  // exit if the stop flag was raised

        //start new measurement on TDC
        spiWrite(tdc.spi_handle, meas_commands, sizeof(meas_commands));
        gpioDelay(10);  // small delay to allow TDC to process data

        //Pulse the laser/TDC pin
        gpioTrigger(LASER_PULSE_PIN, 3, 1);

        //Poll TDC INT pin to signal available data
        uint32_t curr_tick = gpioTick();
        uint32_t stop_tick = curr_tick + TDC_TIMEOUT_USEC;
        while (gpioRead(tdc.INT_PIN) && curr_tick < stop_tick) curr_tick = gpioTick();
        
        if (curr_tick < stop_tick) //if TDC returned in time
        {
            gpioTime(PI_TIME_ABSOLUTE, &timestamp_sec, &timestamp_usec);

            //Extracting ToF from TDC///////////
            uint8_t tx_buff[13] = {0x90};    // start an auto incrementing read to read TIME1, CLOCK_COUTN1, TIME2, CLOCK_COUNT2 in a single command
            uint8_t rx_buff[sizeof(tx_buff)];

            uint16_t bytes_xfer = spiXfer(tdc.spi_handle, tx_buff, rx_buff, sizeof(tx_buff));
            if (bytes_xfer < sizeof(tx_buff)) {/**TODO: Handle case of SPI error**/}

            reverse8bitArr(rx_buff,sizeof(tx_buff));
            

        } // end if (curr_tick < stop_tick), i.e. no timeout waiting for TDC

    }
    /***************************************************************/ 

    /********** Exit Routines **********/
    //Disable Laser
    gpioWrite(LASER_SHUTTER_PIN,0);
    gpioWrite(LASER_ENABLE_PIN,0);
    
    //stop and disable mirror
    mirrorSetRPM(mirror, 0);
    mirrorDisable(mirror);

    //MLD019 serial interface
    mldClose(mld);

    //exit and destroy pin poller thread
    pinPollerExit(sos_poller);
    pthread_join(sos_poller_tid, NULL);
    pinPollerDestroy(sos_poller);
    
    //stop logger
    void** logger_leftover;
    loggerSendCloseMsg(logger, 0, true);
    pthread_join(logger_tid, logger_leftover);
    //TODO: (Optional) log any leftover elements here//
    
    //stop TDC clock
    gpioHardwareClock(TDC_CLK_PIN, 0);

    gpioTerminate();
    /***********************************/
    
    return 0;
}