#include "data_processor.h"

/**Function: dataprocCreate
 * 
 * Parameters: int buffer_size - capacity of the FIFO buffer 
 * 
 * Returns: dataproc_t* out - pointer to allocated memory holding
 *                                  the data proc structure ready for use
 * 
 * Description: Allocates memory for the data processor structure and initializes
 *              a fifo buffer on which to place commands
 */
dataproc_t* dataprocCreate(int buffer_size)
{
    dataproc_t* out = (dataproc_t*) malloc(sizeof(dataproc_t));
    out->buffer = fifoBufferInit(buffer_size);
    return out;
}

/**Function: dataprocDestroy
 * 
 * Parameters: dataproc_t* data_proc - pointer to a data processor structure initialized via
 *                                     dataprocCreate()
 * 
 * Returns dataproc_msg_t** - returns a NULL terminated array of pointers to data_msg_t structs still in buffer
 */
dataproc_msg_t** dataprocDestroy(dataproc_t* data_proc)
{
    fifo_buffer_t* buffer = data_proc->buffer;
    free(data_proc);
    return (dataproc_msg_t**) fifoBufferClose(buffer);
} // end dataprocDestroy

/**Function: dataProcMsgCreate
 * 
 * Parameters: dataproc_cmd_t CMD - desired supported command for data processor
 *             void* (*p_func)(void*) - a pointer to a function that takes a single void* parameter and returns a void*
 *             void* p_args - pointer to arguments to pass to p_func
 *             size_t arg_ size - size in bytes of the argument to p_func
 * 
 * Return: dataproc_msg_t* out_msg - pointer to allocated message ready to send to a data processing Consumer
 * 
 * Description: Memory space is allocated to duplicate the provided function arguments and for the output message 
 *              structure
 */
dataproc_msg_t* dataprocMsgCreate(dataproc_cmd_t CMD, void* (*p_func)(void*), void* p_args, size_t arg_size)
{
    dataproc_msg_t* out_msg = (dataproc_msg_t*) malloc(sizeof(dataproc_msg_t));
    out_msg->p_args = malloc(arg_size);

    out_msg->CMD = CMD;
    out_msg->p_func = p_func;
    
    if (p_args == NULL) out_msg->p_args = NULL;
    else memcpy(out_msg->p_args, p_args, arg_size);

    return out_msg;
}

/**Function dataprocMsgDestroy
 * Parameters - dataproc_msg_t* msg - pointer to message struct to deallocate
 * Returns: None
 * 
 * Description: frees the allocated pointer to function arguments as well as allocated
 *              pointer to the message struct
 */
void dataprocMsgDestroy(dataproc_msg_t* msg)
{
    free(msg->p_args);
    free(msg);
}

int dataprocSendData(dataproc_t* data_processor, void* (*p_func)(void*), void* p_args, size_t arg_size, int priority, bool blocking)
{
    return fifoPush(data_processor->buffer, (void*)dataprocMsgCreate(DATAPROC_CMD_DATA, p_func, p_args, arg_size), priority, blocking);
}

int dataprocSendStop(dataproc_t* data_processor, int priority, bool blocking)
{
    return fifoPush(data_processor->buffer, (void*)dataprocMsgCreate(DATAPROC_CMD_STOP, NULL, NULL, 0), priority, blocking);
}

void* dataprocMain(void* arg_dataproc)
{
    dataproc_t* data_processor = (dataproc_t*) arg_dataproc;
    
    // if received data processor is NULL, immediately raise stop flag for early exit
    bool loop_stop = (data_processor == NULL); 

    while (!loop_stop)
    {
        // Obtain command from buffer
        data_processor->status = DATAPROC_STAT_IDLE;
        dataproc_msg_t* recv_msg = (dataproc_msg_t*) fifoPull(data_processor->buffer, true);

        // do work
        data_processor->status = DATAPROC_STAT_WORKING;
        if (recv_msg != NULL)
        {
            if (recv_msg->CMD == DATAPROC_CMD_STOP) break; // exit if received NULL data pointer
            else
            {
                recv_msg->p_func(recv_msg->p_args);
            }

            dataprocMsgDestroy(recv_msg);
        }
        else
        {
            printf("NULL msg received in data processor\n");
        }
    } // end while (1), the main loop
        
    data_processor->status = DATAPROC_STAT_STOPPED;

    return NULL;    
} // end dataprocMain()