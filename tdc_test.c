
#include "tdc.h"
#include "logger.h"

// #define AUTOINC_METHOD
#define PIGPIO
// #define DEBUG

#define TDC_CAL_PERIODS 2

//TEST definitions
#define TDC_CLK_PIN 4     // physical pin 7; GPIOCLK0 for TDC reference
#define TDC_ENABLE_PIN 27 // physical pin 13; TDC Enable
#define TDC_INT_PIN 22    // physical pin 15; TDC interrupt pin
#define TDC_BAUD (uint32_t)20E6
#define TDC_START_PIN 23 // physical pin 16; provides TDC start signal for debugging
#define TDC_STOP_PIN 18  // physical pin 12; provides TDC stop signal for debugging
#define TDC_TIMEOUT_USEC (uint32_t)5E6
#define TDC_CLK_FREQ (uint32_t) 16e6//19.2E6 / 2
// #define TDC_CHECK_PARITY 1

//Laser pin defintions
#define LASER_ENABLE_PIN 26 // physical pin 37; must be TTL HI to allow emission
#define LASER_SHUTTER_PIN 6 // physical pin 31; must be TTL HI to allow emission; wait 500 ms after raising
#define LASER_PULSE_PIN 23  // physical pin 16; outputs trigger pulses to laser driver
#define LASER_PULSE_COUNT 2
#define LASER_PULSE_FREQ 10e3
#define LASER_PULSE_PERIOD 1 / (LASER_PULSE_FREQ)*1E6
#define LASER_PULSE_POL 1 // Determines laser pulse polarity; 1 means pulse line is normally LO and pulsed HI
#define LASER_ACQ_TIME_USEC (uint32_t)60E6 
#define OUT_FILE "./all_vals.txt"

int main()
{
    /********* Threaded Logger Configuration ********/
    pthread_t logger_tid;
    logger_t *logger = loggerCreate(100);

    // start loggerMain, passing the configured logger struct as argument
    pthread_create(&logger_tid, NULL, &loggerMain, logger);
    /************************************************/
    
    char hdr_strs[] = 
        "TIMESTAMP (s),TOF (usec),DIST (m),TIME1,CLOCK_COUNT1,TIME2,CAL1,CAL2\n";

    loggerSendLogMsg(logger, hdr_strs, sizeof(hdr_strs), OUT_FILE, 0, true);

    printf("LASER_PULSE_PERIOD= %f\n", LASER_PULSE_PERIOD);

    gpioCfgClock(1, PI_CLOCK_PCM, 0); //1us sample rate, PCM clock
    gpioInitialise();

    /******** TDC Initialization *********/
    // opens SPI connection and configures assigned pins
    tdc_t tdc = {
        .enable_pin = TDC_ENABLE_PIN,
        .int_pin = TDC_INT_PIN,
        .clk_pin = TDC_CLK_PIN,
        .clk_freq = TDC_CLK_FREQ};
    tdcInit(&tdc, TDC_BAUD);

    printf("tdc.spi_handle=%d\n", tdc.spi_handle);

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
    char config2_cmds[] = {0x41, 0x00};
    char config2_rx[sizeof(config2_cmds)];

    // char* config2_rx = spiTransact(tdc.spi_handle, config2_cmds, sizeof(config2_cmds));
    printf("config2 spiWrite=%d\n", spiTransact(tdc.spi_handle, config2_cmds, config2_rx, sizeof(config2_cmds)));
    printf("config2_rx=");
    printArray(config2_rx, sizeof(config2_rx));
    /****************************************/

    while (1)
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
            uint32_t start_tick = gpioTick();
            while ((gpioTick() - start_tick) < LASER_ACQ_TIME_USEC)
            {
                //start new measurement on TDC
                static char meas_cmds[2] = {
                    0x40, //Write to CONFIG1
                    0x43  //Start measurement in mode 2 with parity and rising edge start, stop, trigger signals
                };
                char meas_cmds_rx[sizeof(meas_cmds)];

                printf("meas_command spiTransact=%d\n", spiTransact(tdc.spi_handle, meas_cmds, meas_cmds_rx, sizeof(meas_cmds)));
                // printf("meas_command write response = ");
                // printArray(meas_cmds_rx, sizeof(meas_cmds_rx));
                gpioDelay(10); // small delay to allow TDC to process data

                #ifdef DEBUG
                //DEBUGGING: wait a know period of time and send a stop pulse
                gpioWrite(TDC_START_PIN, 1);
                gpioDelay(5);
                gpioWrite(TDC_STOP_PIN, 1);

                gpioWrite(TDC_START_PIN, 0);
                gpioWrite(TDC_STOP_PIN, 0);
                #else
                // send train of laser pulses
                for (int i = 0; i < LASER_PULSE_COUNT; i++)
                {
                    gpioWrite(LASER_PULSE_PIN, LASER_PULSE_POL);
                    gpioDelay(LASER_PULSE_PERIOD / 2);
                    gpioWrite(LASER_PULSE_PIN, !LASER_PULSE_POL);
                    gpioDelay(LASER_PULSE_PERIOD / 2);
                }
                #endif

                //Poll TDC INT pin to signal available data
                uint32_t curr_tick = gpioTick();
                while (gpioRead(tdc.int_pin) && (gpioTick() - curr_tick) < TDC_TIMEOUT_USEC);

                if (!gpioRead(tdc.int_pin)) //if TDC returned in time
                {
                    /********* Variable Declarations *********/
                    uint32_t time1;        // internal clock counts from START edge to next external clock edge
                    uint32_t clock_count1; // external clock counts from TIME1 to STOP edge
                    uint32_t time2;        // internal clock counts from CLOCK_COUNT1 to next external clock edge
                    uint32_t calibration1;
                    uint32_t calibration2;
                    double ToF;

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
                    char rx_buff[17];

                    // static array of rx_buff indices pointing to TDC register data
                    static uint8_t const rx_data_idx[] = {
                        1,  // TIME1 data start index
                        4,  // CLOCK_COUNT1 data start index
                        7,  // TIME2 data start index
                        11, // CALIBRATION1 data start index
                        14  // CLAIBRATION2 data start index
                    };

                    // array to hold converted Measurement Register values in same order as rx_buff
                    uint32_t tdc_data[sizeof(rx_data_idx) / sizeof(*rx_data_idx)];

                    // false if parity of any received data is not even
                    bool valid_data_flag = true;
                    /*******************************************/

                    #ifdef AUTOINC_METHOD
                    /******** Transaction 1 *********/
                    /**Transaction 1 starts an auto-incrementing read at register TIME1,
                    * reading 9 bytes to obtain the 3-byte long TIME1, CLOCK_COUNT1, 
                    * and TIME2 registers.
                    */
                    char tx_buff1[10] = {0x90}; // start an auto incrementing read to read TIME1, CLOCK_COUNT1, TIME2 in a single command
                    printf("Transaction 1 spiXfer=%d\n", spiTransact(tdc.spi_handle, tx_buff1, rx_buff, sizeof(tx_buff1)));

                    //print returned data
                    printf("rx_buff after transaction 1=");
                    printArray(rx_buff, sizeof(rx_buff));

                    /*********************************/

                    /********* Transaction 2 *********/
                    /**Transaciton 2 starts an auto-incrementing read at register CALIBRATION1
                    * and reads the 24-bit CALIBRATION1 and CALIBRATION2 registers. Hence, 
                    * the transaction sends 7 bytes (1 command, 6 reading bytes). The first
                    * byte of the return buffer will always be 0
                    */
                    char tx_buff2[7] = {TDC_CMD(1, 0, TDC_CALIBRATION1)}; //auto incrementing read of CALIBRATION1 and CALIBRATION2
                    printf("Transaction 2 spiXfer=%d\n", spiTransact(tdc.spi_handle, tx_buff2, rx_buff + sizeof(tx_buff1), sizeof(tx_buff2)));

                    // print returne data
                    printf("rx_buff after txaction 2=");
                    printArray(rx_buff, sizeof(rx_buff));
                    /*********************************/

                    /******** Converting Data into 32-bit Numbers ********/
                    /**Iterate over the indices in rx_data_idx, converting the 
                    * subset of 3 bytes starting at rx_buff[rx_data_idx[i]]
                    */
                    for (uint8_t i = 0; i < sizeof(rx_data_idx) / sizeof(*rx_data_idx); i++)
                    {
                        uint32_t conv = convertSubsetToLong(rx_buff + rx_data_idx[i], 3, true);
                        if (checkOddParity(conv))
                        {
                            valid_data_flag = false;
                            break;
                        }

                        tdc_data[i] = conv & ~TDC_PARITY_MASK; // clear the parity bit from data
                    }
                    /*****************************************************/
                    #else
                    static char tof_cmds[5] = {
                        // TDC commands for retrieving TOF
                        0x10, //Read TIME1
                        0x11, //Read CLOCK_COUNT1
                        0x12, //Read TIME2
                        0x1B, //Read CALIBRATION
                        0x1C  //Read CALIBRATION2
                    };

                    for (int i = 0; i < sizeof(tof_cmds); i++)
                    {
                        char tx_temp[4] = {tof_cmds[i]};
                        char rx_temp[4];

                        printf("Command %02X transaction=%d\n", tof_cmds[i], spiTransact(tdc.spi_handle, tx_temp, rx_temp, sizeof(tx_temp)));
                        // printf("Command %02X return=", tof_cmds[i]);
                        // printArray(rx_temp, sizeof(rx_temp));

                        uint32_t conv = convertSubsetToLong(rx_temp, 4, true);
                        printf("conv=%X\n", conv);
                        if (checkOddParity(conv))
                        {
                            valid_data_flag = false;
                            break;
                        }
                        tdc_data[i] = conv & ~TDC_PARITY_MASK;
                        // printf("rx_buff1 converted= %u\n", tdc_data[i]);
                    }
                    #endif

                    #ifndef TDC_PARITY_CHECK
                    valid_data_flag = true;
                    #endif

                    printf("valid=%d\n", valid_data_flag);

                    if (valid_data_flag)
                    {
                        ToF = calcToF(tdc_data, TDC_CAL_PERIODS, tdc.clk_freq);
                    } // end if (valid_data_flag)
                    else
                    {
                        // if any invalid data retrieved from TDC, set negative ToF
                        printf("Invalid data\n");
                        ToF = -1;
                    } //end else linked to if (valid_data_flag)

                    double dist = calcDist(ToF);
                    double time = getEpochTime();

                    char data_str[100];
                    // int data_str_len = buildDataStr(data_str, time, dist, ToF, true);

                    int data_str_len = sprintf(data_str, "%lf,%lE,%lE,%u,%u,%u,%u,%u\n",
                            time, ToF*1e6,dist, tdc_data[0], tdc_data[1], tdc_data[2], tdc_data[3], tdc_data[4]);

                    printf("logger state = %d\n", logger->status);
                    loggerSendLogMsg(logger, data_str, data_str_len, OUT_FILE, 0, true);
                }    // end if (!gpioRead(tdc.int_pin)), i.e. no timeout waiting for TDC
                else //else timeout occured
                {
                    printf("TDC timeout occured\n");
                } // end else linked to if (!gpioRead(tdc.int_pin))
            } // end while (gpioTick() - start_tick < LASER_ACQ_TIME_USEC)
        } // end if (c == 'P')
        else
            continue;
    } // end while(1)

    #ifdef PIGPIO
    spiClose(tdc.spi_handle);
    #else
    close(tdc.spi_handle);
    #endif

    loggerSendCloseMsg(logger, 0, true);
    pthread_join(logger_tid, NULL);
    loggerDestroy(logger);
    gpioHardwareClock(tdc.clk_pin, 0);
    gpioWrite(LASER_SHUTTER_PIN, 0);
    gpioWrite(LASER_ENABLE_PIN, 0);
    gpioTerminate();
} // end main()