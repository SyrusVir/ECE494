#include <pigpio.h>
#include <stdio.h>

#define TDC_MAX_FREQ (int)250E6

//bit positions for TDC SPI command byte
#define TDC_CMD_AUTOINC 7
#define TDC_CMD_RW 6
#define TDC_CMD_ADDR 0

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

typedef struct TDC {
    uint8_t clk_pin;    // Proivdes TDC reference clock
    uint8_t enable_pin; 
    uint8_t int_pin;    // Pin at which to read the TDC interrupt pin
    uint8_t start_pin;  
    uint8_t stop_pin;
    unsigned int clk_freq;  // frequency of reference clock provided to TDC
    int spi_handle;
} tdc_t;

int tdcInit(tdc_t* tdc, uint8_t spi_chan, unsigned int baud, uint32_t spi_flags)
{
    
    tdc->spi_handle = spiOpen((spi_chan ? 1 : 0), baud, spi_flags);
    if (tdc->spi_handle < 0)
    {
        printf("ERROR opening SPI handle in tdcInit\n");
        return tdc->spi_handle;
    }

    char tx[2] = {}
    spiWrite(tdc->spi_handle,);
}