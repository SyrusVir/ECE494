#define _GNU_SOURCE
#include <sys/time.h>
#include <sched.h>
#include "tdc_util.h"
#include "logger.h"
#include "data_processor.h"
#include "tcp_handler.h"

#define AUTOINC_METHOD
#define DEBUG

//TEST definitions
#define TDC_CLK_PIN 4     // physical pin 7; GPIOCLK0 for TDC reference
#define TDC_ENABLE_PIN 27 // physical pin 13; TDC Enable
#define TDC_INT_PIN 22    // physical pin 15; TDC interrupt pin
#define TDC_BAUD (uint32_t)250E6/64
#define TDC_START_PIN 24 // physical pin 18; provides TDC start signal for debugging
#define TDC_STOP_PIN 18  // physical pin 12; provides TDC stop signal for debugging
#define TDC_TIMEOUT_USEC (uint32_t)5E6
#define TDC_CLK_FREQ (uint32_t) 8e6
#define TDC_MEAS_MODE 1   //0 = mode 1; 1 = mode 2

//Laser pin defintions
#define LASER_ENABLE_PIN 26 // physical pin 37; must be TTL HI to allow emission
#define LASER_SHUTTER_PIN 6 // physical pin 31; must be TTL HI to allow emission; wait 500 ms after raising
#define LASER_PULSE_PIN 23  // physical pin 16; outputs trigger pulses to laser driver
#define LASER_PULSE_COUNT 2
#define LASER_PULSE_FREQ 10e3
#define LASER_PULSE_PERIOD 1 / (LASER_PULSE_FREQ)*1E6
#define LASER_PULSE_POL 1 // Determines laser pulse polarity; 1 means pulse line is normally LO and pulsed HI
#define LASER_ACQ_USEC (uint32_t)180e6
#define OUT_FILE "./all_vals.txt"

#define TCP_PORT 49417

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
        #ifdef AUTOINC_METHOD
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
            ToF = tdc_data[0]*(cal_periods - 1) / ((double)tdc_data[4] - tdc_data[3]) / (double)tdc_arg->tdc->clk_freq;
        }
        else
        {
            ToF = calcToF(tdc_data, cal_periods, tdc_arg->tdc->clk_freq);
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
    printf("logger state = %d\n", tdc_arg->logger->status);
    if (tdc_arg->logger != NULL)
    {
        loggerSendLogMsg(tdc_arg->logger, data_str, data_str_len, OUT_FILE, 0, true);
    }
    printf("TCP state = %d\n", tdc_arg->tcp_handler->tcp_state);
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
    /********** Building CPU masks **********/
    cpu_set_t main_cpu;
    CPU_ZERO(&main_cpu);
    CPU_SET(1, &main_cpu);
    pthread_setaffinity_np(pthread_self(), sizeof(main_cpu), &main_cpu); // place DAQ thread on isolated cpu
    /****************************************/

    /********** Threaded Logger Configuration *********/
    pthread_t logger_tid;
    logger_t *logger = loggerCreate(100);

    // start loggerMain, passing the configured logger struct as argument
    pthread_create(&logger_tid, NULL, &loggerMain, logger);
    /************************************************/

    /********** TCP Handler Configuration **********/
    pthread_t tcp_tid;
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,        //TCP
        .sin_port = htons(TCP_PORT),  //define port
        .sin_addr.s_addr = INADDR_ANY //accept connection at any address available
    };
    tcp_handler_t *tcp_handler = tcpHandlerInit(server_addr, 100);

    pthread_create(&tcp_tid, NULL, &tcpHandlerMain, tcp_handler);
    /*************************************************/

    /********* Data Processor Configuraiton *********/
    pthread_t data_proc_tid;
    dataproc_t *data_proc = dataprocCreate(100);
    pthread_create(&data_proc_tid, NULL, &dataprocMain, data_proc);
    /************************************************/
    

    printf("LASER_PULSE_PERIOD= %f\n", LASER_PULSE_PERIOD);

    gpioCfgClock(1, PI_CLOCK_PCM, 0); //1us sample rate, PCM clock
    gpioInitialise();

    /******** TDC Initialization *********/
    // opens SPI connection and configures assigned pins
    tdc_t tdc = {
        .enable_pin = TDC_ENABLE_PIN,
        .int_pin = TDC_INT_PIN,
        .clk_pin = TDC_CLK_PIN,
        .clk_freq = TDC_CLK_FREQ,
        .cal_periods = TDC_CAL_2};
    tdcInit(&tdc, TDC_BAUD);

    // printf("tdc.spi_handle=%d\n", tdc.spi_handle);

    // enable additional pins for debugging
    gpioSetMode(TDC_START_PIN, PI_OUTPUT); // active LOW
    gpioSetMode(TDC_STOP_PIN, PI_OUTPUT);  // active LOW

    // TDC must see rising edge of ENABLE while powered for proper internal initializaiton
    gpioWrite(tdc.enable_pin, 0);
    gpioDelay(3); // Short delay to make sure TDC sees LOW before rising edge
    gpioWrite(tdc.enable_pin, 1);
    /***************************************/

    // Configure laser control pins
    gpioSetMode(LASER_ENABLE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_PULSE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_SHUTTER_PIN, PI_OUTPUT);
    gpioWrite(LASER_SHUTTER_PIN, 0); // initialise to low
    gpioWrite(LASER_ENABLE_PIN, 0);  // initialise to low

    //Non-incrementing write to CONFIG2 reg (address 0x01)
    //Clear CONFIG2 to configure 2 calibration clock periods,
    // no averaging, and single stop signal operation
    char config2_cmds[] = {
        TDC_CMD(0, 1, TDC_CONFIG2),
        TDC_CONFIG2_BITS(tdc.cal_periods, TDC_AVG_1CYC, 1)};
    char config2_rx[sizeof(config2_cmds)];

    spiXfer(tdc.spi_handle, config2_cmds, config2_rx, sizeof(config2_cmds));
    /****************************************/

    while (1) // begin main loop
    {
        static bool shutter_state = 0;
        static bool enable_state = 0;

        char c;
        printf("Enter S to toggle shutter.\n");
        printf("Enter E to toggle enable.\n");
        printf("Enter P to begin a TDC measurement.\n");
        printf("Enter 'q' or 'Q' to quit.\n");
        scanf(" %c", &c);
        if (c == 'q' || c == 'Q')
            break;
        else if (c == 'S')
        {
            shutter_state = !shutter_state;
            gpioWrite(LASER_SHUTTER_PIN, shutter_state);
            continue;
        }
        else if (c == 'E')
        {
            enable_state = !enable_state;
            gpioWrite(LASER_ENABLE_PIN, enable_state);
            continue;
        }
        else if (c == 'P')
        {
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
                gpioDelay(10000);
                spiXfer(tdc.spi_handle, meas_cmds, meas_cmds_rx, sizeof(meas_cmds));
                gpioDelay(1); // small delay to allow TDC to process data
            
                #ifdef DEBUG
                //DEBUGGING: wait a know period of time and send a stop pulse
                gpioWrite(TDC_START_PIN, 1);
                gpioDelay(5);
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
                    #ifdef AUTOINC_METHOD
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

                    printf("queuing dataproc\n");
                    dataprocSendData(data_proc, &dataprocFunc, (void *)data, 0, false);
                    printf("queued dataproc\n");
                }    // end if (!gpioRead(tdc.int_pin)), i.e. no timeout waiting for TDC
                else //else timeout occured
                {
                    // printf("TDC timeout occured\n");
                } // end else linked to if (!gpioRead(tdc.int_pin))
            } // end main data acquisitio loop; while((gpioTick() - start_tick) < ...)
            printf("done Acq\n");
        } // end else if (c == 'P')
        else
            continue;

    } // end while(1)

    spiClose(tdc.spi_handle);

    tcpHandlerClose(tcp_handler, 0, true);
    loggerSendCloseMsg(logger, 0, true);
    dataprocSendStop(data_proc, 0, true);

    gpioHardwareClock(tdc.clk_pin, 0);
    gpioWrite(LASER_SHUTTER_PIN, 0);
    gpioWrite(LASER_ENABLE_PIN, 0);
    gpioTerminate();

    pthread_join(tcp_tid, NULL);
    pthread_join(logger_tid, NULL);
    pthread_join(data_proc_tid, NULL);

    loggerDestroy(logger);
    tcpHandlerDestroy(tcp_handler);
    dataprocDestroy(data_proc);

} // end main()