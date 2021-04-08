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
#include <math.h>

#include "data_processor.h"

#define LIGHT_SPEED 299792458.0
#define DATA_SEPARATOR ','


// Constructs a TDC command byte from the given parameters
#define TDC_CMD(auto_inc, write, tdc_addr) (auto_inc << 7) | (write << 6) | (tdc_addr)
#define TDC_PARITY_MASK 0x00800000  // bit mask for extracting parity bit from 24-bit data registers (TIMEn, CLOCK_COUNTN, etc.)

// constructs a byte to write to the TDC CONFIG1 regsiter
#define TDC_CONFIG1_BITS(force_cal, parity, trigg_edge, stop_edge, start_edge, mode, start_meas) \
    (force_cal << 7)  | \
    (parity << 6)     | \
    (trigg_edge << 5) | \
    (stop_edge << 4)  | \
    (start_edge << 3) | \
    (mode << 1)       | \
    (start_meas)

// constructs a byte to write to the TDC CONFIG2 regsiter
#define TDC_CONFIG2_BITS(cal_periods, avg_cycles, num_stop) \
    (cal_periods << 6) |    \
    (avg_cycles << 3)  |    \
    ((num_stop - 1))  

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

enum TDC_CAL2_PERIODS
{
    TDC_CAL2_2,
    TDC_CAL2_10,
    TDC_CAL2_20,
    TDC_CAL2_40
};

enum TDC_AVG_CYCLES
{
    TDC_AVG_1CYC,
    TDC_AVG_2CYC,
    TDC_AVG_4CYC,
    TDC_AVG_8CYC,
    TDC_AVG_16CYC,
    TDC_AVG_32CYC,
    TDC_AVG_64CYC,
    TDC_AVG_128CYC
};
typedef struct TDC {
    int spi_handle;
    uint32_t clk_freq;  // frequency of reference clock provided to TDC
    uint32_t timeout_us;// microseconds to wait for INT pin to go LO
    uint8_t clk_pin;    // Proivdes TDC reference clock
    uint8_t enable_pin; // active HIGH
    uint8_t int_pin;    // Pin at which to read the TDC interrupt pin; active LO until next measurement
} tdc_t;

void printArray(char* arr, int arr_size);

// Returns true if n has odd parity
bool checkOddParity(uint32_t n);

// Calculate time of flight in microseconds
double calcToF(uint32_t* tdc_data, uint32_t cal_periods, uint32_t clk_freq);

// Calculate distance in meters
double calcDist(double ToF);

/**This function encapsulates the routines for an SPI transaction
 * using either pigio or spidev routines depending on if the macro
 *  PIGPIO is defined.
 */
int spiTransact(int fd, char* tx_buf, char* rx_buf, int count);

/**Convert the bytes at start to start + (len-1) into a 32-bit number.
 * If big_endian = true, the byte *start is considered the MSB.
 * len is restricted to 4 max.
 */
uint32_t convertSubsetToLong(char* start, int len, bool big_endian);

/**Opens an SPI connection using the desired SPI library (spidev or pigpio; #define PIGPIO to use pigpio)
 * Also initializes pins specified in the passed tdc struct. Assign desired pins and reference clock frequency
 * prior to passing to this function
 */
int tdcInit(tdc_t* tdc, int baud);

/**Sends the necessary commands to start a TDC measurement; See tdc.c for
 * exact measurement configuration
 */ 
int tdcStartMeas(tdc_t* tdc);

// Retrieve the current seconds from the epoch with microsecond resolution
double getEpochTime();

/**Build a data string for logging/transmission to GUI.
 */
int buildDataStr(char* out_str, double timestamp, double distance, double ToF, bool add_break);
