
#include "tdc_util.h"

void printArray(char* arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++)
    {
        printf("%X ", arr[i]);
    }
    printf("\n");
} // end printArray()

//Returns true if n has odd parity
bool checkOddParity(uint32_t n)
{
    /**Calculate parity by repeatedly dividing the bits of n into halves
     * and XORing them together. 1 = Odd parity
     * 
     * 8-bit example:
     * n = b7 b6 b5 b4 b3 b2 b1 b0
     * n ^= (n >> 4) --> n = b7 b6 b5 b4 (b7^b3) (b6^b2) (b5^b1) (b4^b0)
     * n ^= (n >> 2) --> n = b7 b6 b5 b4 (b7^b3) (b6^b2) (b7^b5^b3^b1) (b6^b4^b2^b0)
     * n ^= (n >> 1) --> n = b7 b6 b5 b4 (b7^b3) (b6^b2) (b7^b5^b3^b1) (b7^b6^b5^b4^b3^b2^b1^b0)
     * return n & 1 = return (b7^b6^b5^b4^b3^b2^b1^b0)
     */        
    for (uint8_t i = (sizeof(n)*8 >> 1); i > 0; i >>= 1)
    {
        n ^= (n >> i);
    }

    return n & 1;
} // end checkOddParity

/**Assumes tdc_data to be a 5-element array of uint32_t of the following form:
 * [TIME1],[CLOCK_COUNT1],[TIME2],[CALIBRATION1],[CALIBRATION2] 
 */
double calcToF(uint32_t* tdc_data, uint32_t cal_periods, uint32_t clk_freq)
{
    double ToF;
    double time1 = tdc_data[0];
    uint32_t clock_count1 = tdc_data[1];
    uint32_t time2 = tdc_data[2];
    uint32_t calibration1 = tdc_data[3];
    double calibration2 = tdc_data[4];

    double calCount = fabs(calibration2 - calibration1) / (double)(cal_periods - 1);
    if (calCount == 0)
        return 0; // catch divide-by-zero error
    else
    {
        ToF = (fabs(time1 - time2) / calCount + clock_count1) / clk_freq;
        return ToF;
    }
}

inline double calcDist(double ToF)
{
    return ToF*LIGHT_SPEED/2;
}

//convert a subset of a byte array into a 32-bit number
uint32_t convertSubsetToLong(char* start, int len, bool big_endian)
{
    /**Parameters:  char* start - pointer to first element in byte array subset
     *              int len - number of elements, max 4, in the subset
     *              bool big_endian - if true, the first element of start is considerd the MSB
     * Returns:     uint32_t out - the final result of conversion
     */

    len = (len > 4 ? 4 : len); //if len > 4, assign 4. OTW assign user-provided length
    uint32_t out = 0;
    for (int i = 0; i < len; i++)
    {
        // shift the bytes pointed to by start and OR to get output
        out |= (start[(big_endian ? len - 1 - i : i)] << 8*i);
    }

    return out; 
} // end convertSubsetToLong

/**Function: tdcInit
 * Parameters: tdc_t* tdc - pointer to a configured TDC struct, i.e. pin numbers, clock 
 *                          frequency, and SPI timeout time already assigned.
 *             int baud - SPI baud rate; Max 20MHz for TDC7200
 * Return: int spi_handle - returns a copy of the spi_handle assigned to tdc->spi_handle
 * 
 * Description: This function initialises the pigpio library, configured the pins specified within
 *              the provided struct, starting the TDC reference clock, and opening an SPI connection
 */  
int tdcInit(tdc_t* tdc, int baud)
{
    int status = gpioInitialise();
    if (status < 0) 
    {
        tdc->spi_handle = -1;
        return status;
    }
        
    /******** Pigpio SPI init ********/
    tdc->spi_handle = spiOpen(0, baud, 0
            /* 0b00 |          // Positive (MSb 0) clock edge centered (LSb 0) on data bit
            (0b000 << 2) |  // all 3 CE pins are active low
            (0b000 << 5) |  // all 3 CE pins reserved for SPI
            (0 << 8) |      // 0 = Main SPI; 1 = Aux SPI 
            (0 << 9) |      // If 1, 3-wire mode
            ((0 & 0xF) << 10) | // bytes to write before switching to read (N/A if not in 3-wire mode)
            (0 << 14) |         // If 1, tx LSb first (Aux SPI only)
            (0 << 15) |         // If 1, recv LSb first (Aux SPI only)
            ((8 & 0x3F) << 16)  // bits per word; 0 defaults to 8 */

    );
    /******************************/
    
    // checking for non-zero, in-range reference clock frequency
    if (tdc->clk_freq == 0 || tdc->clk_freq < 1000000 || 16000000 < tdc->clk_freq)
    {
        printf("WARNING: TDC requires an 1-16 MHz external reference clock\n");
    }
    gpioHardwareClock(tdc->clk_pin,tdc->clk_freq);// start TDC reference clock
    gpioSetMode(tdc->enable_pin, PI_OUTPUT);     // active HI
    gpioSetMode(tdc->int_pin, PI_INPUT);         // active LOW 
    return tdc->spi_handle;
} // end tdcInit()

/**Closes SPI configuration, stops reference clock, and disables TDC
 */
void tdcClose(tdc_t* tdc)
{
    spiClose(tdc->spi_handle);
    gpioHardwareClock(tdc->clk_pin,0);
    gpioWrite(tdc->enable_pin,0); // place TDC in low power state
} // end tdcClose()

