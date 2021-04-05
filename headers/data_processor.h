#ifndef _DATA_PROCESSOR_H_
#define _DATA_PROCESSOR_H_
#include <pigpio.h>
#include "fifo.h"
#include "logger.h"
#include "tcp_handler.h"

typedef enum DataProcStatus
{
    DATAPROC_UNINIT,
    DATAPROC_IDLE,
    DATAPROC_WORKING,
    DATAPROC_STOPPED
} dataproc_stat_t;

typedef enum DataProcCMD
{
    DATAPROC_DATA,
    DATAPROC_STOP
} dataproc_cmd_t;

typedef struct DataProcMSG
{
    struct timespec ts;         // timestamp of measurement
    
    uint8_t* tdc_data_idx;      // array of indices pointing to first byte of each TDC data quantity (e.g. TIME1)
    char* tdc_raw_data;         // bytes of raw TDC data
    size_t tdc_raw_data_len;    // bytes contained in string holding raw TDC data
    size_t tdc_num_idx;         // number of indices in tdc_data_idx
    uint32_t clk_freq;          // TDC reference clock frequency
    uint8_t cal_periods;        // TDC calibration periods
    
    dataproc_cmd_t CMD;         // specified command
    
    bool add_break;             // add extra line break to signify SOS detection
} dataproc_msg_t;

typedef struct DataProc
{
    logger_t* logger;
    tcp_handler_t* tcp_handler;
    fifo_buffer_t* buffer;
    dataproc_stat_t status;
} dataproc_t;
dataproc_msg_t* dataprocMsgCreate(char* data, size_t data_len);
void dataprocMsgDestroy(dataproc_msg_t* msg);
int dataprocSendData(dataproc_t* data_processor, char* data, size_t data_len, int priority, bool blocking);
void* dataprocMain(void* arg_dataproc);
#endif