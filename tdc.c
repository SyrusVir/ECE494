#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

// #define AUTOINC_METHOD
#define PIGPIO

//TDC register addresses; easier to define using enum
enum TDC_REG_ADDR {
    TDC_CONFIG1 = 0x0, 
    TDC_CONFIG2, 
    TDC_INT_STATUS, 
    TDC_INT_MASK, 
    TDC_COARSE_CNTR_OVF_H,
    TDC_COARSE_CNTR_OVF_L,
    TDC_CLOCK_CNTR_OVF_H,
    TDC_CLOCK_CNTR_OVF_L,
    TDC_CLOCK_CNTR_STOP_MASK_H,
    TDC_CLOCK_CNTR_STOP_MASK_L,
    TDC_TIME1 = 0x10,
    TDC_CLOCK_COUNT1,
    TDC_TIME2,
    TDC_CLOCK_COUNT2,    
    TDC_TIME3,
    TDC_CLOCK_COUNT3,    
    TDC_TIME4,
    TDC_CLOCK_COUNT4,    
    TDC_TIME5,
    TDC_CLOCK_COUNT5,    
    TDC_TIME6,
    TDC_CALIBRATION1,
    TDC_CALIBRATION2
};

#define TDC_CMD(auto_inc, write, tdc_addr) (auto_inc << 7) | (write << 6) | (tdc_addr)
#define TDC_PARITY_MASK 0x00800000 // a bit mask for isolating parity bit of measurement registers

//TEST definitions
#define TDC_CLK_PIN 4   // physical pin 7; GPIOCLK0 for TDC reference
#define TDC_ENABLE_PIN 27   // physical pin 13; TDC Enable
#define TDC_INT_PIN 22  // physical pin 15; TDC interrupt pin
#define TDC_BAUD (uint32_t) 20E6
#define TDC_START_PIN 23         // physical pin 16; provides TDC start signal for debugging 
#define TDC_STOP_PIN 18         // physical pin 12; provides TDC stop signal for debugging 
#define TDC_TIMEOUT_USEC (uint32_t)5E6
#define TDC_CLK_FREQ (uint32_t)19.2E6/2

//Laser pin defintions
#define LASER_ENABLE_PIN 26     // physical pin 37; must be TTL HI to allow emission
#define LASER_SHUTTER_PIN 6     // physical pin 31; must be TTL HI to allow emission; wait 500 ms after raising
#define LASER_PULSE_PIN 23      // physical pin 16; outputs trigger pulses to laser driver
#define LASER_PULSE_COUNT 2
#define LASER_PULSE_FREQ 10e3
#define LASER_PULSE_PERIOD 1/(LASER_PULSE_FREQ) * 1E6

typedef struct TDC {
    uint8_t clk_pin;    // Proivdes TDC reference clock
    uint8_t enable_pin; 
    uint8_t int_pin;    // Pin at which to read the TDC interrupt pin
    uint32_t clk_freq;  // frequency of reference clock provided to TDC
    int spi_handle;
} tdc_t;

void printArray(char* arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++)
    {
        printf("%X ", arr[i]);
    }
    printf("\n");
}

char* spiTransact(int fd, char* tx_buf, int count)
{
    char* rx_buf = (char*) malloc(count*sizeof(tx_buf));

    #ifdef PIGPIO
    printf("spiXfer=%d\n", spiXfer(fd, tx_buf, rx_buf, count));
    #else
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long) tx_buf,
        .rx_buf = (unsigned long) rx_buf,
        .len = count,
        .delay_usecs = 0,
        .speed_hz = TDC_BAUD,
        .bits_per_word = 8
    };

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    #endif

    return rx_buf;
}

//convert a subset of a byte array into a 32-bit number
uint32_t convertSubsetToLong(char* start, int len, bool big_endian)
{
    /**Parameters: char* start - pointer to first element in byte array subset
     *             int len - number of elements, max 4, in the subset
     *             bool big_endian - if true, the first element of start is considerd the MSB
     * Returns: 
     */

    len = (len > 4 ? 4 : len); //if len > 4, assign 4. OTW assign user-provided length
    uint32_t out = 0;
    for (int i = 0; i < len; i++)
    {
        out |= (start[(big_endian ? len - 1 - i : i)] << 8*i);
    }

    return out; 
}

int main () 
{
    tdc_t tdc = {
        .enable_pin = TDC_ENABLE_PIN,
        .int_pin = TDC_INT_PIN,
        .clk_pin = TDC_CLK_PIN,
        .clk_freq = TDC_CLK_FREQ
    };
    printf("LASER_PULSE_PERIOD= %f\n", LASER_PULSE_PERIOD);

    gpioCfgClock(1, 1,0); //1us sample rate, PCM clock
    gpioInitialise();

    #ifndef PIGPIO
    /******** spidev init ********/
    tdc.spi_handle = open("/dev/spidev0.0", O_RDWR);
    /*****************************/
    #else
    /******** Pigpio SPI init ********/
    tdc.spi_handle = spiOpen(0, TDC_BAUD, 0
            /* 0b00 |          // Positive (MSb 0) clock edge centered (LSb 0) on data bit
            (0b000 << 2) |  // all 3 CE pins are active low
            (0b000 << 5) |  // all 3 CE pins reserved for SPI
            (0 << 8) |      // 0 = Main SPI; 1 = Aux SPI 
            (0 << 9) |      // If 1, 3-wire mode
            ((0 & 0xF) << 10) | // bytes to write before switching to read (N/A if not in 3-wire mode)
            (0 << 14) |         // If 1, tx LSb first (Aux SPI only)
            (0 << 15) |         // If 1, recv LSb first (Aux SPI only)
            ((8 & 0x3F) << 16)  // bits per word; default 8 */

    );
    /******************************/
    #endif

    printf("tdc.spi_handle=%d\n",tdc.spi_handle);
    gpioHardwareClock(tdc.clk_pin,tdc.clk_freq);
    gpioSetMode(tdc.enable_pin, PI_OUTPUT);     // active LOW
    gpioSetMode(tdc.int_pin, PI_INPUT);         // active LOW        
    // gpioSetPullUpDown(tdc.int_pin, PI_PUD_UP);  // INT pin is open drain, active LOW
    gpioSetMode(TDC_START_PIN, PI_OUTPUT);     // active LOW
    gpioSetMode(TDC_STOP_PIN, PI_OUTPUT);     // active LOW
    gpioSetMode(LASER_ENABLE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_PULSE_PIN, PI_OUTPUT);
    gpioSetMode(LASER_SHUTTER_PIN, PI_OUTPUT);
    
    gpioWrite(LASER_SHUTTER_PIN,0);
    gpioWrite(LASER_ENABLE_PIN,0);

    gpioWrite(tdc.enable_pin, 0);
    gpioDelay(5000);
    gpioWrite(tdc.enable_pin,1);
    // gpioDelay(5000);
    // gpioWrite(tdc.enable_pin, 0);

    //Non-incrementing write to CONFIG2 reg (address 0x01)
    //Clear CONFIG2 to configure 2 calibration clock periods,
    // no averaging, and single stop signal operation
    uint8_t cal_periods = 2;
    char tx_buff[] = {0x41, 0x00};
    printArray(tx_buff, sizeof(tx_buff));

    char* rx_buff = spiTransact(tdc.spi_handle, tx_buff, sizeof(tx_buff));
    printf("rx_buff from config transact=");
    printArray(rx_buff, sizeof(tx_buff));
    free(rx_buff);
    
    /****************************************/
    
    static char meas_commands[2] = {
        0x40,   //Write to CONFIG1
        0x03    //Start measurement in mode 2 withOUT parity and rising edge start, stop, trigger signals 
    };

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
        if (c == 'q' || c == 'Q') break;
        else if (c == 'S')
        {
            shutter_state = !shutter_state;
            gpioWrite(LASER_SHUTTER_PIN,shutter_state);
            continue;
        }
        else if (c == 'E')
        {
            enable_state = !enable_state;
            gpioWrite(LASER_ENABLE_PIN,enable_state);
            continue;
        }
        else if (c == 'P')
        {
            //start new measurement on TDC
            char* rx_buff = spiTransact(tdc.spi_handle, meas_commands, sizeof(meas_commands));
            printf("meas_command write response = ");
            printArray(rx_buff, sizeof(meas_commands));
            free(rx_buff);
            gpioDelay(10); // small delay to allow TDC to process data
            for (int i = 0; i < LASER_PULSE_COUNT; i++)
            {
                gpioWrite(LASER_PULSE_PIN,1);
                gpioDelay(LASER_PULSE_PERIOD/2);
                gpioWrite(LASER_PULSE_PIN,0);
                gpioDelay(LASER_PULSE_PERIOD/2);
            }
        }
        else continue;

        
        //DEBUGGING: wait a know period of time and send a stop pulse
        // gpioTrigger(TDC_START_PIN, 1, 1);
        // gpioDelay(5);
        // gpioTrigger(TDC_STOP_PIN, 1, 1);

        //Poll TDC INT pin to signal available data
        uint32_t curr_tick = gpioTick();
        uint32_t stop_tick = curr_tick + TDC_TIMEOUT_USEC;
        while (gpioRead(tdc.int_pin) && (gpioTick() - curr_tick)  < TDC_TIMEOUT_USEC);

        if (!gpioRead(tdc.int_pin)) //if TDC returned in time
        {
            //Extracting ToF from TDC///////////
            uint32_t time1;
            uint32_t clock_count1;
            uint32_t time2;
            uint32_t calibration1;
            uint32_t calibration2;

            #ifdef AUTOINC_METHOD
            //Transaction 1
            char tx_buff1[10] = {0x90}; // start an auto incrementing read to read TIME1, CLOCK_COUNT1, TIME2 in a single command
            // char rx_buff1[sizeof(tx_buff1)];
            char* rx_buff1 = spiTransact(tdc.spi_handle, tx_buff1, sizeof(tx_buff1));
            // int bytes_xfer = spiXfer(tdc.spi_handle, tx_buff1, rx_buff1, sizeof(tx_buff1));
            // if (bytes_xfer < sizeof(tx_buff1))
            // {
            //     printf("Transaction 2: Only %d bytes received out of %d\n", bytes_xfer, sizeof(tx_buff1));
            // }
            printf("rx_buff1 =");
            printArray(rx_buff1, sizeof(tx_buff1));

            //Transaction 2
            char tx_buff2[7] = {TDC_CMD(1, 0, TDC_CALIBRATION1)}; //auto incrementing read of CALIBRATION1 and CALIBRATION2
            //char rx_buff2[sizeof(tx_buff2)];
            char* rx_buff2 = spiTransact(tdc.spi_handle, tx_buff2, sizeof(tx_buff2));
            // bytes_xfer = spiXfer(tdc.spi_handle, tx_buff2, rx_buff2, sizeof(tx_buff2));
            // if (bytes_xfer < sizeof(tx_buff2))
            // {
            //     printf("Transaction 2: Only %d bytes received out of %d\n", bytes_xfer, sizeof(tx_buff2));
            // }
            printf("rx_buff2 =");
            printArray(rx_buff2, sizeof(tx_buff2));

            time1 = convertSubsetToLong(rx_buff1 + 1, 3, 1);
            clock_count1 = convertSubsetToLong(rx_buff1 + 4, 3, 1);
            time2 = convertSubsetToLong(rx_buff1 + 7, 3, 1);
            calibration1 = convertSubsetToLong(rx_buff2 + 1, 3, 1);
            calibration2 = convertSubsetToLong(rx_buff2 + 4, 3, 1);
            #else
            static char tof_commands[5] = {
                // TDC commands for retrieving TOF
                0x10, //Read TIME1
                0x11, //Read CLOCK_COUNT1
                0x12, //Read TIME2
                0x1B, //Read CALIBRATION
                0x1C  //Read CALIBRATION2
            };
            uint32_t tdc_data[sizeof(tof_commands)]; // array to hold TIME1, CLOCK_COUNT1, TIME2, CLOCK_COUNT2, CALIBRATION1, CALIBRATION2 in that order
            for (int i = 0; i < sizeof(tof_commands); i++)
            {
                char tx_buff1[4] = {tof_commands[i]};
                // char rx_buff1[sizeof(tx_buff1)];
                char* rx_buff1 = spiTransact(tdc.spi_handle, tx_buff1, sizeof(tx_buff1));
                printf("%d) rx_buff1 =", i);
                printArray(rx_buff1, sizeof(tx_buff1));
                tdc_data[i] = convertSubsetToLong(rx_buff1, sizeof(tx_buff1),true);
                printf("rx_buff1 converted= %u\n", tdc_data[i]);
            }

            time1 = tdc_data[0];
            clock_count1 = tdc_data[1];
            time2 = tdc_data[2];
            calibration1 = tdc_data[3];
            calibration2 = tdc_data[4];
            #endif

            printf("time1=%u\n", time1);
            printf("clock_count1=%u\n", clock_count1);
            printf("time2=%u\n", time2);
            printf("calibration1=%u\n", calibration1);
            printf("calibration2=%u\n", calibration2);

            double ToF;
            double calCount = ((double)calibration2 - calibration1) / (double)(cal_periods - 1);
            if (!calCount)
                ToF = 0; // catch divide-by-zero error
            else
            {
                ToF = (((double)time1 - time2) / calCount + clock_count1) / tdc.clk_freq * 1E6;
                printf("ToF = %f usec\n", ToF);
            }
        } // end if (curr_tick < stop_tick), i.e. no timeout waiting for TDC
        else //else timeout occured
        {
            printf("TDC timeout occured\n");
        }
    } // end while(1)

    #ifdef PIGPIO
    spiClose(tdc.spi_handle);
    #else
    close(tdc.spi_handle);
    #endif

    gpioWrite(LASER_SHUTTER_PIN, 0);
    gpioWrite(LASER_ENABLE_PIN, 0);
    gpioTerminate();
} // end main()