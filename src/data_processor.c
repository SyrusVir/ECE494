#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "fifo.h"
#include "logger.h"
#include "tcp_handler.h"
#include "tdc.h"
#include "data_processor.h"

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

void* dataprocMain(void* arg_dataproc)
{
    pthread_t logger_tid;
    pthread_t tcph_tid;
    bool valid_tcp = false;
    bool valid_logger = false;

    dataproc_t* data_processor = (dataproc_t*) arg_dataproc;
    logger_t* logger = data_processor->logger;
    tcp_handler_t* tcp = data_processor->tcp_handler;

    if (logger != NULL)
    {
        valid_logger = true;
        pthread_create(&logger_tid, NULL, &loggerMain, logger);
    }
    
    if (tcp != NULL)
    {
        valid_tcp = true;
        pthread_create(&tcph_tid, NULL, &tcpHandlerMain, tcp);
    }
    

    while (1)
    {
        // Obtain command from buffer
        data_processor->status = DATAPROC_IDLE;
        dataproc_msg_t* recv_msg = (dataproc_msg_t*) fifoPull(data_processor->buffer, true);

        // do work
        data_processor->status = DATAPROC_WORKING;
        if (recv_msg != NULL)
        {
            if (recv_msg->CMD == DATAPROC_STOP) break; // exit if received NULL data pointer
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
                        valid_data_flag = false;
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

                double timestamp = getEpochTime();

                char* data_str;
                int data_str_size = buildDataStr(data_str, timestamp, distance, ToF, recv_msg->add_break);

                if (valid_logger)
                    loggerSendLogMsg(logger, data_str, data_str_size, "./values.txt",0,true);

                if (valid_tcp && tcp->tcp_state == TCPH_STATE_CONNECTED) 
                    tcpHandlerWrite(tcp, data_str, data_str_size, 0, true);
            } 

            dataprocMsgDestroy(recv_msg);
        }
        else
        {
            printf("NULL msg received in data processor\n");
        }
    }
    
    loggerSendCloseMsg(logger, 0, true);
    tcpHandlerClose(tcp, 0, true);

    pthread_join(logger_tid, NULL);
    pthread_join(tcph_tid, NULL);
    
    data_processor->status = DATAPROC_STOPPED;

    return;    

}