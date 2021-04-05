#include "logger.h"

void* loggerMain(void* arg_logger) 
{
    /** CORE FUNCTIONALITY OF LOGGER IMPLEMENTED HERE.
     * Pass as argument to pthreads_create for multithreading.
     * Pass a logger constructed using loggerInit(). **/
    logger_t* logger = (logger_t*) arg_logger;

    bool loop_stop = false;
    while (!loop_stop) 
    {
        logger->status = LOGGER_IDLE;
        logger_msg_t* rec_msg = (logger_msg_t*)fifoPull(logger->buffer,true);
        logger->status = LOGGER_WORKING;
        
        // if NULL msg received, log event and go to next loop iteration
        if (rec_msg == NULL) 
        {
            logStatus(logger,"loggerMain::NULL pointer received\n");
            continue;
        }
        //sleep(1);
        char rec_msg_str[200];  //string to hold string representation of received message; may be source of future memory overflows
        snprintf(rec_msg_str,200,"loggerMain:: CMD: %d\t PATH: %s\t MSG: %s\n", rec_msg->cmd, rec_msg->path, rec_msg->data);
        logStatus(logger, rec_msg_str);
        
        switch (rec_msg->cmd)
        {
        case LOGGER_LOG:;  //semicolon allows variable declaration following case label
            FILE* f = fopen(rec_msg->path, "a");
            if (f == NULL) 
            {
                char msg[] = "loggerMain: Error opening provided path"; 
                printf("%s\n",msg);
                logStatus(logger, msg); 
            }
            else 
            {
                printf("Writing \"%s\"\n", rec_msg->data);
                if (fprintf(f,"%s",rec_msg->data) < 0) 
                {
                    char msg[] = "loggerMain: Error writing to provided path";
                    printf("%s\n", msg);
                    logStatus(logger,msg);
                }
            }
            fclose(f);
            break;
        case LOGGER_STOP:;
            loop_stop = true;
            break;
        default:
            break;
        
        loggerMsgDestroy(rec_msg); //destroy message after processing
        } // end switch(rec_ms->cmd)

        /** NOTE: I believe it is not the responsiblity of the Consumer to destroy itself.
         * Stopping a consumer here is "freezing" its state. The consumer can be restarted 
         * by passing the struct back to a pthread_create(). It is the responsibility of the
         * Producer(s) to deallocate memory for the Consumer when all Producers agree that it is 
         * no longer needed 
         */
        logger->status = LOGGER_STOPPED;
        return;
    } // end while(!loop_stop)
} // end loggerMain

//Constructors
logger_t* loggerCreate(uint16_t buffer_size) 
{
    /**Returns a configured logger struct to the buffer pointed to by logger
     * Calls buffer initialization and create a log file for status reports.
     * Use as parameter to pthreads_create for threaded logging **/

    //stat log initialization////////////////////////////
    char *stat_log_stem = "/logger_stat_logs.txt"; //WARNING: ASSUME UNIX PATH SEPARATOER
    char *stat_log_root = "./logs";
    mkdir(stat_log_root, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
    char *stat_log_path = (char*) malloc(strlen(stat_log_root)+strlen(stat_log_stem)+2); //MEMCHECK 4
    if (stat_log_path == NULL) 
    {
        printf("ERROR ALLOCATING MEMORY FOR STAT LOG PATH IN LOGGER INITIALIZATION\n");
    }

    strcpy(stat_log_path,stat_log_root);
    strcat(stat_log_path,stat_log_stem);
    FILE* stat_log_file = fopen(stat_log_path, "a");
    if (stat_log_file == NULL) 
    {
        printf("ERROR OPENING STAT LOG FILE REFERENCE\n");
    }
    
    //write current time and separator to file
    uint8_t timestring_size = 32;
    char sep[] = "-----------------------------------\n";
    char timestring[timestring_size];
    struct tm now;
    time_t rawtime = time(NULL);
    gmtime_r(&rawtime,&now);
    strftime(timestring, timestring_size,"\n%F %T %Z\n",&now);
    fprintf(stat_log_file,"%s", timestring);
    fprintf(stat_log_file,"%s", sep);
    fclose(stat_log_file);
    /////////////////////////////////////////////////////////////////

    //Buffer initialization
    fifo_buffer_t* buffer = fifoBufferInit(buffer_size); //returns pointer to buffer in allocated memory

    //once data members are allocated
    logger_t* logger = (logger_t*) malloc(sizeof(stat_log_file) + sizeof(stat_log_path)+sizeof(*buffer));
    logger->buffer = buffer;
    logger->stat_log_path = stat_log_path;
    logger->status = LOGGER_UNINIT;
    
    return logger; 
}

//Destructors
logger_msg_t** loggerDestroy(logger_t* logger) 
{
    fifo_buffer_t* buffer = logger->buffer; // save buffer pointer
    free(logger->stat_log_path); // destroy status log path string
    free(logger); // destroy logger itself
    return (logger_msg_t**)fifoBufferClose(buffer); // flush its message buffer
}

logger_msg_t* loggerMsgCreate(logger_cmd_t cmd, char* data_str, size_t data_size, char* path) 
{
    logger_msg_t* msg_out = (logger_msg_t*)malloc(sizeof(logger_msg_t));
    msg_out->cmd = cmd;
    msg_out->data = strndup(data_str, data_size);
    msg_out->data_leng = data_size;
    msg_out->path = strdup(path);

    return msg_out;
}

void loggerMsgDestroy(logger_msg_t* msg) 
{
    free(msg->data);
    free(msg->path);
    free(msg);
}

int logStatus(logger_t* buffer, char* msg) 
{
    FILE* f = fopen(buffer->stat_log_path, "a");
    if (f != NULL) 
    {
        fprintf(f,"%s",msg);
        return fclose(f);
    }
    else 
    {
        return -1; 
    }
}

int loggerSendLogMsg(logger_t* logger, char* data_str, size_t data_str_size, char* path, int priority, bool blocking) 
{
    void* data = (void*) loggerMsgCreate(LOGGER_LOG, data_str, data_str_size, path);
    return fifoPush(logger->buffer,data, priority, blocking);
}

int loggerSendCloseMsg(logger_t* logger, int priority, bool blocking) 
{
    void* data = (void*) loggerMsgCreate(LOGGER_STOP, " ", sizeof(" "), " ");
    return fifoPush(logger->buffer,data, priority, blocking);
}
