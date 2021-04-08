/**Author: Jude Alnas
 * Date: April 7, 2021
 * Email: jalnas@crimson.ua.edu
 * 
 * Description: The role of the data processor Consumer is to occupy a thread and execute
 *              functions passed to it. The original purpose for writing this module is
 *              to offload conversion and formatting of raw TDC data away from the main
 *              data acquisition thread.
 * 
 *              The dataproc_t struct wraps a pointer to a FIFO buffer (fifo_buffer_t) and
 *              an enum of the data processors current status (dataproc_stat_t)
 * 
 *              A message to the data processor consists of a command enum (dataproc_cmd_t),
 *              a pointer to function that takes a single void* argument and returns void*,
 *              and lastly a void* to the arguments of the aforementioned function. When a 
 *              DATAPROC_CMD_DATA command is received, dataprocMain() will call the provided
 *              function, passing the provided arguments. 
 * 
 *              Usage:
 *                  1) call dataprocCreate(), passing desired FIFO buffer length
 *                  2) call pthread_create(), passing dataprocMain as the 3rd argument
 *                     and the return of dataprocCreate() as the 4th argument
 *                  3) call dataprocSendStop() and dataprocSendData() to control the now running 
 *                     data processor.
 */
#ifndef _DATA_PROCESSOR_H_
#define _DATA_PROCESSOR_H_
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "fifo.h"

typedef enum DataProcStatus
{
    DATAPROC_STAT_UNINIT,
    DATAPROC_STAT_IDLE,
    DATAPROC_STAT_WORKING,
    DATAPROC_STAT_STOPPED
} dataproc_stat_t;

typedef enum DataProcCMD
{
    DATAPROC_CMD_DATA,
    DATAPROC_CMD_STOP
} dataproc_cmd_t;

typedef struct DataProcMSG
{
    dataproc_cmd_t CMD;     // specified command
    void* (*p_func)(void*); // provided function pointer
    void* p_args;           // arguments to function pointer
} dataproc_msg_t;

// Struct encapsulating the data processor's command buffer and optional 
// logger and tcp_handler pointers
typedef struct DataProc
{
    fifo_buffer_t* buffer;
    dataproc_stat_t status;
} dataproc_t;

// Initialize a data processor struct
dataproc_t* dataprocCreate(int buffer_size);

// Destroys/deallocates an initialized data processor struct
dataproc_msg_t** dataprocDestroy(dataproc_t* data_proc);

// Returns pointer to allocated and populated dataproc_msg_t
dataproc_msg_t* dataprocMsgCreate(dataproc_cmd_t CMD, void* (*p_func)(void*), void* p_args, size_t arg_size);

// frees all memory allocated for a dataproc_msg_t
void dataprocMsgDestroy(dataproc_msg_t* msg);

// Constructs and sends a command to data_processor to execute the provided function with the provided arguments
int dataprocSendData(dataproc_t* data_processor, void* (*p_func)(void*), void* p_args, size_t arg_size, int priority, bool blocking);

// Constructs and sends the stop command to the data processor. dataprocMain() will return once received
int dataprocSendStop(dataproc_t* data_processor, int priority, bool blocking);

// The main data processing loop
void* dataprocMain(void* arg_dataproc);
#endif