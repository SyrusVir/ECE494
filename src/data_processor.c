#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "fifo.h"
#include "logger.h"
#include "tcp_handler.h"
#include "tdc.c"

#define DATA_SEPARATOR ','

typedef enum DataProcStatus
{
    UNINIT,
    IDLE,
    WORKING,
    CLOSED
} dataproc_stat_t;

typedef enum DataProcCMD
{
    DATA,
    CLOSE
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

dataproc_t* dataprocInit(uint16_t buffer_size, )

dataproc_msg_t* dataprocMsgCreate(char* data, size_t data_len)
{
    dataproc_msg_t* out_msg = (dataproc_msg_t*) malloc(sizeof(dataproc_msg_t));
    out_msg->tdc_raw_data = strndup(data, data_len);
    out_msg->tdc_raw_data_len = data_len;

    return out_msg;
}

void dataprocMsgDestroy(dataproc_msg_t* msg)
{
    free(msg->tdc_raw_data);
    free(msg);
}

int dataprocSendData(dataproc_t* data_processor, char* data, size_t data_len, int priority, bool blocking)
{
    return fifoPush(data_processor->buffer, (void*)dataprocMsgCreate(data, data_len), priority, blocking);
}

/**Construct data string for logging and TCP transmission. out_str will point to a 
 * newly allocated string. The string length (including NULL terminator) is returned
 */
int buildDataStr(char* out_str, double timestamp, double distance, double ToF, bool add_break)
{
    if (out_str == NULL) return 0;
    else
    {
        int out_str_size = 0;
        char time_str[20];
        char dist_str[20];
        char tof_str[20];

        out_str_size += sprintf(time_str,"%.6f%c", timestamp, DATA_SEPARATOR);
        out_str_size += sprintf(dist_str, "%.6f%c,", distance, DATA_SEPARATOR);
        out_str_size += sprintf(tof_str, "%.6f%c\n", ToF, DATA_SEPARATOR);

        out_str = (char*) calloc(sizeof(char)*(out_str_size + 2 /* +2 for additional \n and \0 */));
        strcat(out_str, time_str);
        strcat(out_str, dist_str);
        strcat(out_str, tof_str);

        // add additional new line if requested
        if (add_break)
        {
            strcat(out_str, "\n");
            out_str_size++;
        }

        return out_str_size;
    }
}

void* dataprocMain(void* arg_dataproc)
{
    dataproc_t* data_processor = (dataproc_t*) arg_dataproc;
    logger_t* logger = data_processor->logger;
    tcp_handler_t* tcp = data_processor->tcp_handler;

    while (1)
    {
        // Obtain command from buffer
        data_processor->status = IDLE;
        dataproc_msg_t* recv_msg = (dataproc_msg_t*) fifoPull(data_processor->buffer, true);

        // do work
        data_processor->status = WORKING;
        if (recv_msg != NULL)
        {
            if (recv_msg->CMD == CLOSE) break; // exit if received NULL data pointer
            else
            {
                double ToF;
                double distance;
                bool valid_data_flag = true;
                
                uint32_t tdc_data[recv_msg->tdc_num_idx];
                for (uint8_t i = 0; i < recv_msg->tdc_num_idx; i++)
                {
                    uint8_t idx = recv_msg->tdc_data_idx[i];
                    uint32_t conv = convertSubsetToLong(recv_msg->tdc_raw_data+idx, 3, true);

                    // if parity check failed, clear valid flag and terminate data conversion 
                    if (checkOddParity(conv)) 
                    {
                        valid_data_flag;
                        break;
                    }

                    tdc_data[i] = conv & ~TDC_PARITY_MASK; // clear the parity bit from data
                }

                if (valid_data_flag)
                {
                    ToF = calcToF(tdc_data, 2, recv_msg->clk_freq);
                    distance = ToF/LIGHT_SPEED/2;
                }
                else
                {
                    ToF = -1;
                    distance = -1;
                }

                struct timeval ts;
                gettimeofday(&ts, NULL); // faster than clock_gettime but usec resolution instead of nanosec

                double timestamp = tv.tv_sec + tv.tv_usec * 1E-6;

                char* data_str;
                int data_str_size = buildDataStr(data_str, timestamp, distance, ToF, recv_msg->add_break);

                loggerSendLogMsg(logger, data_str, data_str_size, "./values.txt",0,true);

                if (tcp->tcp_state == CONNECTED) tcpHandlerWrite(tcp, data_str, data_str_size, 0, true);
            } 

            dataprocMsgDestroy(recv_msg);
        }
        else
        {
            printf("NULL msg received in data processor\n");
        }
    }
    
    data_processor->status = CLOSED;

    return;



    

}