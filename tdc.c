#include <pigpio.h>
#include <stdio.h>
#include <stdbool.h>

#define AUTOINC_METHOD

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

//TEST definitions
#define TDC_ENABLE_PIN 0
#define TDC_INT_PIN 0
#define TDC_BAUD (uint32_t)20E6
#define TDC_START_PIN 0         // physical pin #; provides TDC start signal for debugging 
#define TDC_STOP_PIN 0         // physical pin #; provides TDC stop signal for debugging 
#define TDC_TIMEOUT_USEC 500

typedef struct TDC {
    uint8_t clk_pin;    // Proivdes TDC reference clock
    uint8_t enable_pin; 
    uint8_t int_pin;    // Pin at which to read the TDC interrupt pin
    uint32_t clk_freq;  // frequency of reference clock provided to TDC
    int spi_handle;
} tdc_t;

//convert a subset of a byte array into a 32-bit number
uint32_t convertSubsetToLong(uint8_t* start, int len, bool big_endian)
{
    /**Parameters: uint8_t* start - pointer to first element in byte array subset
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
        .int_pin = TDC_INT_PIN
    };
    
    gpioSetMode(tdc.enable_pin, PI_OUTPUT);     // active LOW
    gpioSetMode(tdc.int_pin, PI_INPUT);         // active LOW        
    gpioSetPullUpDown(tdc.int_pin, PI_PUD_UP);  // INT pin is open drain, active LOW
    gpioSetMode(TDC_START_PIN, PI_OUTPUT);     // active LOW
    gpioSetMode(TDC_STOP_PIN, PI_OUTPUT);     // active LOW
    
    tdc.spi_handle = spiOpen(0, TDC_BAUD,
            0b00 |          // Positive (MSb 0) clock edge centered (LSb 0) on data bit
            (0b000 << 2) |  // all 3 CE pins are active low
            (0b000 << 5) |  // all 3 CE pins reserved for SPI
            (0 << 8) |      // 0 = Main SPI; 1 = Aux SPI 
            (0 << 9) |      // If 1, 3-wire mode
            ((0 & 0xF) << 10) | // bytes to write before switching to read (N/A if not in 3-wire mode)
            (0 << 14) |         // If 1, tx LSb first (Aux SPI only)
            (0 << 15) |         // If 1, recv LSb first (Aux SPI only)
            ((8 & 0x3F) << 16)  // bits per word; default 8

    );

    gpioHardwareClock(tdc.clk_pin,tdc.clk_freq);
    
    gpioWrite(tdc.enable_pin, 0);

    //Non-incrementing write to CONFIG2 reg (address 0x01)
    //Clear CONFIG2 to configure 2 calibration clock periods,
    // no averaging, and single stop signal operation
    uint8_t cal_periods = 2;
    uint8_t tx_buff[] = {0x41, 0x00};
    spiWrite(tdc.spi_handle, tx_buff, sizeof(tx_buff));
    /****************************************/
    
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

    //start new measurement on TDC
    spiWrite(tdc.spi_handle, meas_commands, sizeof(meas_commands));
    gpioDelay(10);  // small delay to allow TDC to process data
    gpioTrigger(TDC_START_PIN,1,1);

    //DEBUGGING: wait a know period of time and send a stop pulse
    gpioDelay(500);
    gpioTrigger(TDC_STOP_PIN,1,1);

    //Poll TDC INT pin to signal available data
    uint32_t curr_tick = gpioTick();
    uint32_t stop_tick = curr_tick + TDC_TIMEOUT_USEC;
    while (gpioRead(tdc.int_pin) && curr_tick < stop_tick) curr_tick = gpioTick();
    
    if (curr_tick < stop_tick) //if TDC returned in time
    {
        //Extracting ToF from TDC///////////
        uint32_t time1;
        uint32_t clock_count1;
        uint32_t time2;
        uint32_t clock_count2;
        uint32_t calibration1;
        uint32_t calibration2;  
        
        #ifdef AUTOINC_METHOD
        //Transaction 1
        uint8_t tx_buff1[13] = {0x90};    // start an auto incrementing read to read TIME1, CLOCK_COUTN1, TIME2, CLOCK_COUNT2 in a single command
        uint8_t rx_buff1[sizeof(tx_buff1)];
        int bytes_xfer = spiXfer(tdc.spi_handle, tx_buff1, rx_buff1, sizeof(tx_buff1));
        if (bytes_xfer < sizeof(tx_buff1))
        {
            printf("Transaction 2: Only %d bytes received out of %d\n", bytes_xfer, sizeof(tx_buff1));
        } 
        printf("rx_buff1=%s\n");

        //Transaction 2
        uint8_t tx_buff2[7] = {TDC_CMD(1,0,TDC_CALIBRATION1)};  //auto incrementing read of CALIBRATION1 and CALIBRATION2
        uint8_t rx_buff2[sizeof(tx_buff2)]; 
        uint16_t bytes_xfer = spiXfer(tdc.spi_handle, tx_buff2, rx_buff2, sizeof(tx_buff2));
        if (bytes_xfer < sizeof(tx_buff2))
        {
            printf("Transaction 2: Only %d bytes received out of %d\n", bytes_xfer, sizeof(tx_buff2));
        } 
        printf("rx_buff2=%s\n");

        time1 = convertSubsetToLong(rx_buff1+1,3,1);
        clock_count1 = convertSubsetToLong(rx_buff1+4,3,1);
        time2 = convertSubsetToLong(rx_buff1+7,3,1);
        clock_count2 = convertSubsetToLong(rx_buff1+10,3,1);
        calibration1 = convertSubsetToLong(rx_buff2+1,3,1);
        calibration2 = convertSubsetToLong(rx_buff2+4,3,1);
        #else
        uint32_t tdc_data[6]; // array to hold TIME1, CLOCK_COUNT1, TIME2, CLOCK_COUNT2, CALIBRATION1, CALIBRATION2 in that order
        for(int i = 0; i < sizeof(tof_commands); i++)
        {
            uint8_t tx_buff1[4] = {tof_commands[i]}
            uint8_t rx_buff1[sizeof(tx_buff1)];
            int bytes_xfer = spiXfer(tdc.spi_handle, tx_buff1, rx_buff1, sizeof(tx_buff1));
            print("rx_buff1 = %s\n", rx_buff1);
            tdc_data[i] = convertSubsetToLong(rx_buff1)
            print("rx_buff1 converted= %u\n", tdc_data[i]);
        }

        time1          = tdc_data[0];
        clock_count1   = tdc_data[1];
        time2          = tdc_data[2];
        clock_count2   = tdc_data[3];
        calibration1   = tdc_data[4];
        calibration2   = tdc_data[5];
        #endif
        
        printf("time1=%u\n", time1);
        printf("clock_count1=%u\n", clock_count1);
        printf("time2=%u\n", time2);
        printf("clock_count2=%u\n", clock_count2);
        printf("calibration1=%u\n", calibration1);
        printf("calibration2=%u\n", calibration2);

        double ToF;
        double calCount = (calibration2 - calibration1) / (double)(cal_periods - 1);
        double normLSB = 1 / (calCount * tdc.clk_freq);
        if (!calCount) ToF = 0; // catch divide-by-zero error
        else
        {
            ToF
        }
    } // end if (curr_tick < stop_tick), i.e. no timeout waiting for TDC
    else //else timeout occured
    {
        printf("TDC timeout occured\n");
    }
}