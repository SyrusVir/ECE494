#define _GNU_SOURCE
#include <pigpio.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <sched.h>
#include "tdc_util.h"
#include "logger.h"
#include "data_processor.h"
#include "tcp_handler.h"
#include "scanmirror.h"
#include "pinpoller.h"
#include "MLD019.h"
#include <stdint.h>

// user interface libraries
#include <string.h>
#include <ctype.h>

// TCP Port definition
#define TCP_PORT 49417 

// MLD-019 definitions
#define MLD_TTY "/dev/ttyAMA0"
#define MLD_TIMEOUT_MSEC 10

//TDC definitions
#define TDC_CLK_PIN 4     // physical pin 7; GPIOCLK0 for TDC reference
#define TDC_ENABLE_PIN 27 // physical pin 13; TDC Enable
#define TDC_INT_PIN 22    // physical pin 15; TDC interrupt pin
#define TDC_BAUD (uint32_t)250E6/64
#define TDC_START_PIN 24 // physical pin 18; provides TDC start signal for debugging
#define TDC_STOP_PIN 18  // physical pin 12; provides TDC stop signal for debugging
#define TDC_TIMEOUT_USEC (uint32_t)5E6 // time to wait for TDC INT pin to go LO
#define TDC_CLK_FREQ (uint32_t)19.2e6/2 // TDC ref clock frequency from Pi
#define TDC_MEAS_MODE 1   //0 = mode 1; 1 = mode 2
#define TDC_DELAY_USEC 10 // delay between TDC_START and TDC_STOP

//Laser pin defintions
#define DETECTOR_GATE_PIN 5 // physical pin 29; controls photon detector gate
#define LASER_ENABLE_PIN 26 // physical pin 37; must be TTL HI to allow emission
#define LASER_SHUTTER_PIN 6 // physical pin 31; must be TTL HI to allow emission; wait 500 ms after raising
#define LASER_PULSE_PIN 23  // physical pin 16; outputs trigger pulses to laser driver
#define LASER_PULSE_COUNT 2
#define LASER_PULSE_FREQ 10e3
#define LASER_PULSE_PERIOD 1 / (LASER_PULSE_FREQ)*1E6
#define LASER_PULSE_POL 1 // Determines laser pulse polarity; 1 means pulse line is normally LO and pulsed HI
#define LASER_ACQ_USEC (uint32_t)45e6
#define OUT_FILE "./all_vals.txt"

// Core definitinos
#define MAIN_CORE 3 // isolated core for DAQ and instrument control, i.e. main
#define POLL_CORE 2 // isolated core for pin polling

//Mirror pin definitions
#define MIRROR_FREQ_PIN 18      //phyiscal pin 12; PWM mirror control signal
#define MIRROR_ATSPEED_PIN -1   // -1 for unused
#define MIRROR_ENABLE_PIN -1   // -1 for unused

//Start-of-scan definitions
#define SOS_PIN 7           // physical pin 26; start-of-scan detector input
#define SOS_LEVEL 0         // detector is active LO
#define SOS_DELAY_USEC 10   // usecs to hold spinlock when SOS goes LO

// Module selectors 
#define USE_AUTOINC_METHOD  // comment out this line to use 5 separate SPI transactions to retrieve data (SLOWER)
#define USE_DEBUG           // comment out this line to use LASER pin definitions instead of TDC_START_PIN and TDC_STOP_PIN
#define USE_LOGGER          // comment out this line to not use the threaded logger
#define USE_TCP             // comment out this line to not use the threaded tcp handler
// #define USE_POLLER          // comment out this line to not use pin polling
// #define USE_MLD019          // comment out this line to not use serial commands to MLD-019 driver
#define USE_MIRROR          // comment out this line to not use the GECKO scanning mirror

// structure defining argument to dataprocFunc
struct DataProcArg
{
    logger_t *logger;
    tcp_handler_t *tcp_handler;
    tdc_t *tdc;
    char *raw_tdc_data;
    int raw_tdc_size;
    bool data_break;
};

double getEpochTime()
{
    static struct timeval tv;
    gettimeofday(&tv, NULL);

    return tv.tv_sec + tv.tv_usec * 1E-6;
}

// This funciton will be executed in the data processor's thread
void *dataprocFunc(void *arg)
{
    struct DataProcArg *tdc_arg = (struct DataProcArg *)arg;

    // printf("dataprocFunc: %p\n", tdc_arg->raw_tdc_data);
    bool valid_data_flag = true;
    double ToF;
    uint32_t tdc_data[5];

    // static array of rx_buff indices pointing to TDC register data within array of raw bytes
    static uint8_t const rx_data_idx[] = {
        1,  // TIME1 data start index
        4,  // CLOCK_COUNT1 data start index
        7,  // TIME2 data start index
        11, // CALIBRATION1 data start index
        14  // CLAIBRATION2 data start index
    };

    /******** Converting Data into 32-bit Numbers ********/
    /**Iterate over the indices in rx_data_idx, converting the 
     * subset of 3 bytes starting at rx_buff[rx_data_idx[i]]
     */
    for (uint8_t i = 0; i < 5; i++)
    {
        #ifdef USE_AUTOINC_METHOD
        uint32_t conv = convertSubsetToLong(tdc_arg->raw_tdc_data + rx_data_idx[i], 3, true);
        #else
        uint32_t conv = convertSubsetToLong(tdc_arg->raw_tdc_data + i * 4, 4, true);
        #endif

        if (checkOddParity(conv))
        {
            valid_data_flag = false;
            break;
        }

        tdc_data[i] = conv & ~TDC_PARITY_MASK; // clear the parity bit from data
    }
    /*****************************************************/

    valid_data_flag = true;
    // printf("valid=%d\n", valid_data_flag);

    if (valid_data_flag)
    {
        uint8_t cal_periods;
        switch (tdc_arg->tdc->cal_periods)
        {
        default:
        case TDC_CAL_2:
            cal_periods = 2;
            break;
        case TDC_CAL_10:
            cal_periods = 10;
            break;
        case TDC_CAL_20:
            cal_periods = 20;
            break;
        case TDC_CAL_40:
            cal_periods = 40;
            break;
        }

        if(TDC_MEAS_MODE)
        {
            ToF = calcToF(tdc_data, cal_periods, tdc_arg->tdc->clk_freq);
        }
        else
        {
            ToF = tdc_data[0]*(cal_periods - 1) / ((double)tdc_data[4] - tdc_data[3]) / (double)tdc_arg->tdc->clk_freq;
        }

    } // end if (valid_data_flag)
    else
    {
        // if any invalid data retrieved from TDC, set negative ToF
        printf("Invalid data\n");
        ToF = -1;
    } //end else linked to if (valid_data_flag)

    double dist = calcDist(ToF);
    double time = getEpochTime();

    char data_str[70];

    // holds number of characters in data_str excluding NULL
    int data_str_len = sprintf(data_str, "%lf,%lf,%lf,%u,%u,%u,%u,%u\n",
                        time, dist, ToF*1e6, tdc_data[0], tdc_data[1], tdc_data[2], tdc_data[3], tdc_data[4]);
    if (tdc_arg->data_break)
    {
        data_str[data_str_len] = '\n';
        data_str[++data_str_len] = '\0';
    }

    /********** pass data to logger and tcp consumers if available **********/ 
    // printf("logger state = %d\n", tdc_arg->logger->status);
    if (tdc_arg->logger != NULL)
    {
        loggerSendLogMsg(tdc_arg->logger, data_str, data_str_len, OUT_FILE, 0, true);
    }
    // printf("TCP state = %d\n", tdc_arg->tcp_handler->tcp_state);
    if (tdc_arg->tcp_handler != NULL && tdc_arg->tcp_handler->tcp_state == TCPH_STATE_CONNECTED)
    {
        tcpHandlerWrite(tdc_arg->tcp_handler, data_str, data_str_len, 0, true);
    }
    /*************************************************************************/

    free(tdc_arg->raw_tdc_data);
    free(tdc_arg);
    return NULL;
}

int main()
{
    /***** GPIO clock configuration and GPIO library initialisation *****/
    gpioCfgClock(1, PI_CLOCK_PCM, 0); //1us sample rate, PCM clock
    if (gpioInitialise() == PI_INIT_FAILED)
    {
        perror("CRITICAL ERROR in gpioInitialise()");
        return -1;
    } 
    /********************************************************************/
    
    /********** Building CPU masks and thread attr's **********/
    cpu_set_t main_cpu;
    CPU_ZERO(&main_cpu);
    CPU_SET(MAIN_CORE, &main_cpu);

    cpu_set_t poller_cpu;
    CPU_ZERO(&poller_cpu);
    CPU_SET(POLL_CORE, &poller_cpu);

    cpu_set_t nonisol_cpu;
    CPU_ZERO(&nonisol_cpu);
    for (uint8_t i = 0; i < get_nprocs_conf(); i++) // iterate over available cpu cores
    {
        // add core to non isolated set if not main or poller core
        if (i != MAIN_CORE && i != POLL_CORE)
        {
            CPU_SET(i, &nonisol_cpu);
        }
    }
    /********************************************/
    
    /********** Threaded Logger Configuration *********/
    logger_t* logger = NULL;
    pthread_t logger_tid = 0;
    #ifdef USE_LOGGER
    pthread_attr_t logger_attr;
    logger = loggerCreate(100);

    // configure logger thread attribute to assign cpu cores
    pthread_attr_init(&logger_attr);
    pthread_attr_setaffinity_np(&logger_attr, sizeof(nonisol_cpu), &nonisol_cpu);

    pthread_create(&logger_tid, &logger_attr, &loggerMain, logger); // start loggerMain, passing the configured logger struct as argument
    pthread_attr_destroy(&logger_attr); // destroy attr; no effect on already created threads
    #endif
    /************************************************/

    /********** TCP Handler Configuration **********/
    tcp_handler_t* tcp_handler = NULL;
    pthread_t tcp_tid = 0;
    #ifdef USE_TCP
    pthread_attr_t tcp_attr;
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,        //TCP
        .sin_port = htons(TCP_PORT),  //define port
        .sin_addr.s_addr = INADDR_ANY //accept connection at any address available
    };
    tcp_handler = tcpHandlerInit(server_addr, 100);

    // config tcp thread attribute to assign non-isolated cores
    pthread_attr_init(&tcp_attr);
    pthread_attr_setaffinity_np(&tcp_attr, sizeof(nonisol_cpu), &nonisol_cpu);

    pthread_create(&tcp_tid, &tcp_attr, &tcpHandlerMain, tcp_handler); // start tcp thread
    pthread_attr_destroy(&tcp_attr); // destroy attr; no effect on already created threads
    #endif
    /*************************************************/

    /********* Data Processor Configuraiton *********/
    pthread_t data_proc_tid = 0;
    pthread_attr_t data_proc_attr;
    dataproc_t *data_proc = dataprocCreate(100);

    // configure data processor core affinities to non-isolated cores
    pthread_attr_init(&data_proc_attr);
    pthread_attr_setaffinity_np(&data_proc_attr, sizeof(nonisol_cpu), &nonisol_cpu);

    pthread_create(&data_proc_tid, &data_proc_attr, &dataprocMain, data_proc); // start data processor thread
    pthread_attr_destroy(&data_proc_attr); // destroy attr; no effect on already created threads
    /************************************************/
    
    /********* Pin Poller Configuration *********/
    pin_poller_t* poller = NULL;
    pthread_t poller_tid = 0;
    #ifdef USE_POLLER
    pthread_spinlock_t lock;
    pthread_attr_t poller_attr;
    poller = pinPollerInit(&lock, SOS_PIN, SOS_LEVEL, SOS_DELAY_USEC);

    // configure spin lock; unshared between processes (only between threads)
    pthread_spin_init(&lock, 0);

    // configure polling thread core affinities to non-isolated cores 
    pthread_attr_init(&poller_attr);
    pthread_attr_setaffinity_np(&poller_attr, sizeof(nonisol_cpu), &nonisol_cpu);

    pthread_create(&poller_tid, &poller_attr, &pinPollerMain, poller); // start polling thread
    pthread_attr_destroy(&poller_attr); // destroy attr; no effect on already created threads
    #endif
    /*******************************************/
    
    pthread_setaffinity_np(pthread_self(), sizeof(main_cpu), &main_cpu); // place DAQ thread on isolated cpu 3

    /********* MLD-019 Serial Comms Initialization *********/
    mld_t* mld = NULL;
    #ifdef USE_MLD019
    mld = mldInit(MLD_TTY, MLD_TIMEOUT_MSEC);
    #endif
    /*******************************************************/

    /******** TDC Initialization *********/
    // opens SPI connection and configures assigned pins
    tdc_t tdc = {
        .enable_pin = TDC_ENABLE_PIN,
        .int_pin = TDC_INT_PIN,
        .clk_pin = TDC_CLK_PIN,
        .clk_freq = TDC_CLK_FREQ,
        .cal_periods = TDC_CAL_2};
    tdcInit(&tdc, TDC_BAUD);

    // enable additional pins for debugging
    gpioSetMode(TDC_START_PIN, PI_OUTPUT); // active HI
    gpioSetMode(TDC_STOP_PIN, PI_OUTPUT);  // active HI

    // TDC must see rising edge of ENABLE while powered for proper internal initializaiton
    gpioWrite(tdc.enable_pin, 0);
    gpioDelay(3); // Short delay to make sure TDC sees LOW before rising edge
    gpioWrite(tdc.enable_pin, 1);
    
    //Non-incrementing write to CONFIG2 reg (address 0x01)
    //Clear CONFIG2 to configure 2 calibration clock periods,
    // no averaging, and single stop signal operation
    char config2_cmds[] = {
        TDC_CMD(0, 1, TDC_CONFIG2),
        TDC_CONFIG2_BITS(tdc.cal_periods, TDC_AVG_1CYC, 1)};
    char config2_rx[sizeof(config2_cmds)];

    spiXfer(tdc.spi_handle, config2_cmds, config2_rx, sizeof(config2_cmds));
    /****************************************/

    /********* Initializing laser control pins *********/
    gpioSetMode(DETECTOR_GATE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_ENABLE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_PULSE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_SHUTTER_PIN, PI_OUTPUT);

    gpioWrite(DETECTOR_GATE_PIN, 0); // initialise to low/open
    gpioWrite(LASER_SHUTTER_PIN, 0); // initialise to low/shuttered
    gpioWrite(LASER_ENABLE_PIN, 0);  // initialise to low/disabled
    /**************************************************/

    #ifdef USE_MIRROR
    /********* Mirror configuration *********/
    mirror_t mirror = {
        .FREQ_PIN = MIRROR_FREQ_PIN,
        .ENABLE_PIN = MIRROR_ENABLE_PIN,
        .ATSPEED_PIN = MIRROR_ATSPEED_PIN
    };
    mirrorConfig(mirror);    // configure pins
    mirrorSetRPM(mirror, 0); // start with mirror off
    #endif
    /****************************************/

    while (1) // begin main loop
    {
        static bool shutter_state = 0;
        static bool enable_state = 0;
        static bool gate_state = 0;

        char str_in[10];
        char c;
        // printf("Enter S to toggle shutter.\n");
        // printf("Enter E to toggle enable.\n");
        printf("Enter P to begin a TDC measurement.\n");
        printf("Enter G to toggle detector gate.\n");
        printf("Enter a number to set mirror RPM.\n");
        printf("Enter 'q' or 'Q' to quit.\n");
        scanf(" %s", &str_in);
        
        if (isdigit(*str_in)) //if first char is number assume whole string number
        {
             
            int freq = atoi(str_in);    // set mirror rpm
            mirrorSetRPM(mirror, freq); // jump to next loop iteration
            continue;
        }
        else if (strlen(str_in) == 1)
        {
            c = *str_in; // if single char entered, save to variable and do NOT jump to next iteration
        }
        else continue; // if a word string is entered, ignore by jumping to next iteration

        /******** This point only reached if user enteres single character *********/

        if (c == 'q' || c == 'Q') 
        {
            mirrorSetRPM(mirror,0);
            break;
        }
        else if (c == 'L') // emit a 10khz 50% duty signal for 30 s
        {
            gpioSetPWMfrequency(LASER_PULSE_PIN, (unsigned int)10E3);
            gpioPWM(LASER_PULSE_PIN, 255/2);

            gpioSleep(PI_TIME_RELATIVE, 30, 0);

            gpioPWM(LASER_PULSE_PIN, 0);
        }
        else if (c == 'G') // toggle photon detector gate
        {
            gate_state = !gate_state;
            gpioWrite(DETECTOR_GATE_PIN, gate_state);
            continue;
        }
        else if (c == 'S') // toggle laser shutter
        {
            shutter_state = !shutter_state;
            gpioWrite(LASER_SHUTTER_PIN, shutter_state);
            continue;
        }
        else if (c == 'E') // toggle laser enable
        {
            enable_state = !enable_state;
            gpioWrite(LASER_ENABLE_PIN, enable_state);
            continue;
        }
        else if (c == 'P') // start measurement
        {
            printf("Acquiring data...\n");
            mirrorSetRPM(mirror, 0); // disable mirror
            gpioDelay(3); // short delay

            char hdr_strs[] = 
                "TIMESTAMP (s),DIST (m),TOF (usec),TIME1,CLOCK_COUNT1,TIME2,CAL1,CAL2\n";

            loggerSendLogMsg(logger, hdr_strs, sizeof(hdr_strs), OUT_FILE, 0, true);       
            
            //start new measurement on TDC
            static char meas_cmds[2] = {
                0x40, //Write to CONFIG1
                TDC_CONFIG1_BITS(0,1,0,0,0,TDC_MEAS_MODE,1)  //Start measurement in mode 1 with parity and rising edge start, stop, trigger signals
            };
            char meas_cmds_rx[sizeof(meas_cmds)];

            uint32_t start_tick = gpioTick();
            while ((gpioTick() - start_tick) < LASER_ACQ_USEC) // main data acquisition loop
            {
                gpioDelay(10); 
                spiXfer(tdc.spi_handle, meas_cmds, meas_cmds_rx, sizeof(meas_cmds));
                gpioDelay(1); // small delay to allow TDC to process data
            
                #ifdef USE_DEBUG
                //DEBUGGING: wait a know period of time and send a stop pulse
                gpioWrite(TDC_START_PIN, 1);
                gpioDelay(TDC_DELAY_USEC);
                gpioWrite(TDC_STOP_PIN, 1);

                gpioWrite(TDC_STOP_PIN, 0);
                gpioWrite(TDC_START_PIN, 0);
                #else
                // send train of laser pulses
                for (int i = 0; i < LASER_PULSE_COUNT; i++)
                {
                    if (i == 1) 
                        gpioWrite_Bits_0_31_Set((LASER_PULSE_POL << LASER_PULSE_PIN) | (1 << TDC_START_PIN));
                    else
                        gpioWrite(LASER_PULSE_PIN, LASER_PULSE_POL);
                    gpioDelay(LASER_PULSE_PERIOD / 2);
                    gpioWrite(LASER_PULSE_PIN, !LASER_PULSE_POL);
                    gpioDelay(LASER_PULSE_PERIOD / 2);
                }

                // gpioDelay(20);
                // gpioTrigger(TDC_STOP_PIN, 3, 1);
                gpioWrite(TDC_START_PIN, 0);
                #endif

                //Poll TDC INT pin to signal available data
                uint32_t curr_tick = gpioTick();
                while (gpioRead(tdc.int_pin) && (gpioTick() - curr_tick) < tdc.timeout_us);

                if (!gpioRead(tdc.int_pin)) //if TDC returned in time
                {
                    #ifdef USE_AUTOINC_METHOD
                    /******** Transaction 1 *********/
                    /**Transaction 1 starts an auto-incrementing read at register TIME1,
                    * reading 9 bytes to obtain the 3-byte long TIME1, CLOCK_COUNT1, 
                    * and TIME2 registers.
                    */

                    /**array to holds return bytes from both SPI transactions retrieving Measurement registers
                    * TIME1, CLOCK_COUNT1, TIME2, CALIBRATION1, CALIBRATION2 in that order.
                    * These registers are 24-bits long where the MSb is a parity bit.
                    * Hence rx_buff holds 5 3-byte data chars and 2 1-byte command chars (17 bytes total)
                    * rx_buff[0] = 0 (junk data from Transaction 1 command byte)
                    * rx_buff[1-3] = TIME1 bytes in big-endian order
                    * rx_buff[4-6] = CLOCK_COUNT1 bytes in big-endian order
                    * rx_buff[7-9] = TIME2 bytes in big-endian order
                    * rx_buff[10] = 0 (junk data from Transaction 2 command byte)
                    * rx_buff[11-13] = CALIBRATION1 in big-endian order
                    * rx_buff[14-16] = CALIBRATION2 in big-endian order
                    */
                    char* rx_buff = (char*) calloc(17,sizeof(char));

                    char tx_buff1[10] = {0x90}; // start an auto incrementing read to read TIME1, CLOCK_COUNT1, TIME2 in a single command
                    spiXfer(tdc.spi_handle, tx_buff1, rx_buff, sizeof(tx_buff1));

                    //print returned data
                    // printf("rx_buff after transaction 1=");
                    // printArray(rx_buff, 17);
                    // printf("\n");
                    /*********************************/

                    /********* Transaction 2 *********/
                    /**Transaciton 2 starts an auto-incrementing read at register CALIBRATION1
                    * and reads the 24-bit CALIBRATION1 and CALIBRATION2 registers. Hence, 
                    * the transaction sends 7 bytes (1 command, 6 reading bytes). The first
                    * byte of the return buffer will always be 0
                    */
                    char tx_buff2[7] = {TDC_CMD(1, 0, TDC_CALIBRATION1)}; //auto incrementing read of CALIBRATION1 and CALIBRATION2
                    spiXfer(tdc.spi_handle, tx_buff2, rx_buff + sizeof(tx_buff1), sizeof(tx_buff2));

                    // printf("rx_buff after txaction 2=");
                    // printArray(rx_buff, 17);
                    // printf("\n");
                    /*********************************/
                    #else
                    static char tof_cmds[5] = {
                        // TDC commands for retrieving TOF
                        0x10, //Read TIME1
                        0x11, //Read CLOCK_COUNT1
                        0x12, //Read TIME2
                        0x1B, //Read CALIBRATION
                        0x1C  //Read CALIBRATION2
                    };
                    static char rx_buff1[sizeof(tof_cmds) * 4];

                    for (int i = 0; i < sizeof(tof_cmds); i++)
                    {
                        char tx_temp[4] = {tof_cmds[i]};

                        spiXfer(tdc.spi_handle, tx_temp, rx_buff1 + i * 4, sizeof(tx_temp));
                        // printf("Command %02X return=", tof_cmds[i]);
                        // printArray(rx_buff1 + i * 4, 4);
                        // printf("\n");
                    }
                    #endif

                    // allocate argument struct for data processor. Free inside data processor function
                    struct DataProcArg *data = (struct DataProcArg *)malloc(sizeof(struct DataProcArg));
                    data->data_break = false;
                    data->logger = logger;
                    data->raw_tdc_data = rx_buff;
                    data->raw_tdc_size = sizeof(rx_buff);
                    data->tcp_handler = tcp_handler;
                    data->tdc = &tdc;

                    // printf("queuing dataproc\n");
                    dataprocSendData(data_proc, &dataprocFunc, (void *)data, 0, true);
                    // printf("queued dataproc\n");
                }    // end if (!gpioRead(tdc.int_pin)), i.e. no timeout waiting for TDC
                else //else timeout occured
                {
                    // printf("TDC timeout occured\n");
                } // end else linked to if (!gpioRead(tdc.int_pin))
            } // end main data acquisitio loop; while((gpioTick() - start_tick) < ...)
            
            gpioPWM(LASER_PULSE_PIN, 0); // stop laser pulse train
            printf("done Acq\n");
        } // end else if (c == 'P')
        else continue;
    } // end while(1); main loop

    tcpHandlerClose(tcp_handler, 0, true);
    loggerSendCloseMsg(logger, 0, true);
    dataprocSendStop(data_proc, 0, true);
    pinPollerExit(poller);
    mldClose(mld);
    
    tdcClose(&tdc);
    gpioWrite(LASER_SHUTTER_PIN, 0);
    gpioWrite(LASER_ENABLE_PIN, 0);

    pthread_join(tcp_tid, NULL);
    pthread_join(logger_tid, NULL);
    pthread_join(data_proc_tid, NULL);
    pthread_join(poller_tid, NULL);

    loggerDestroy(logger);
    tcpHandlerDestroy(tcp_handler);
    dataprocDestroy(data_proc);

    gpioTerminate();
} // end main()