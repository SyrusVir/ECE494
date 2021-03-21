#include <logger.h>

void* loggerMain(void* arg_logger) {
    /** CORE FUNCTIONALITY OF LOGGER IMPLEMENTED HERE.
     * Pass as argument to pthreads_create for multithreading.
     * Pass a logger constructed using loggerInit(). **/
    logger_t* logger = (logger_t*) arg_logger;
    while (1) {
        logger_msg_t* rec_msg = (logger_msg_t*)fifoPull(logger->buffer,true);
        if (rec_msg == NULL) {
            logStatus(logger,"loggerMain::NULL pointer received\n");
        }

        char rec_msg_str[200];  //string to hold string representation of received message; may be source of future memory overflows
        snprintf(rec_msg_str,200,"loggerMain:: CMD: %d\t PATH: %s\t MSG: %s\n", rec_msg->cmd, rec_msg->path, rec_msg->data);
        logStatus(logger, rec_msg_str);
        
        switch (rec_msg->cmd)
        {
        case LOG:;  //semicolon allows variable declaration following case label
            FILE* f = fopen(rec_msg->path, "a");
            if (f == NULL) {
                char msg[] = "loggerMain: Error opening provided path"; 
                printf("%s\n",msg);
                logStatus(logger, msg); 
            }
            else {
                printf("Writing \"%s\"\n", rec_msg->data);
                if (fprintf(f,"%s",rec_msg->data) < 0) {
                    char msg[] = "loggerMain: Error writing to provided path";
                    printf("%s\n", msg);
                    logStatus(logger,msg);
                }
            }
            fclose(f);
            break;
        case CLOSE:;
            /** FREE MEMORY HERE **/
            return loggerDestroy(logger);
            break;
        default:
            break;
        
        loggerMsgDestroy(rec_msg); //destroy message after processing
        }
    }
}

//Constructors
logger_t* loggerCreate(uint16_t buffer_size) {
    /**Returns a configured logger struct to the buffer pointed to by logger
     * Calls buffer initialization and create a log file for status reports.
     * Use as parameter to pthreads_create for threaded logging **/

    //stat log initialization////////////////////////////
    char *stat_log_stem = "/logger_stat_logs.txt"; //WARNING: ASSUME UNIX PATH SEPARATOER
    char *stat_log_root = "./logs";
    mkdir(stat_log_root, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
    char *stat_log_path = (char*) malloc(strlen(stat_log_root)+strlen(stat_log_stem)+2); //MEMCHECK 4
    if (stat_log_path == NULL) {
        printf("ERROR ALLOCATING MEMORY FOR STAT LOG PATH IN LOGGER INITIALIZATION");
    }

    strcpy(stat_log_path,stat_log_root);
    strcat(stat_log_path,stat_log_stem);
    FILE* stat_log_file = fopen(stat_log_path, "a");
    if (stat_log_file == NULL) {
        printf("ERROR OPENING STAT LOG FILE REFERENCE");
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
    
    return logger; 
}

//Destructors
logger_msg_t** loggerDestroy(logger_t* logger) {
    free(logger->stat_log_path);
    return (logger_msg_t**)fifoBufferClose(logger->buffer);

}

logger_msg_t* loggerMsgCreate(logger_cmd_t cmd, char* data_str, size_t data_size, char* path) {
    logger_msg_t* msg_out = (logger_msg_t*)malloc(sizeof(logger_msg_t));
    msg_out->cmd = cmd;
    msg_out->data = strndup(data_str, data_size);
    msg_out->data_leng = data_size;
    msg_out->path = strdup(path);

    return msg_out;
}

void loggerMsgDestroy(logger_msg_t* msg) {
    free(msg->data);
    free(msg->path);
    free(msg);
}

int logStatus(logger_t* buffer, char* msg) {
    FILE* f = fopen(buffer->stat_log_path, "a");
    if (f != NULL) {
        fprintf(f,"%s",msg);
        return fclose(f);
    }
    else {
        return -1; 
    }
}

int loggerSendLogMsg(logger_t* logger, char* data_str, size_t data_str_size, char* path, int priority, bool blocking) {
    void* data = (void*) loggerMsgCreate(LOG, data_str, data_str_size, path);
    return fifoPush(logger->buffer,data, priority, blocking);
}

int loggerSendCloseMsg(logger_t* logger, int priority, bool blocking) {
    void* data = (void*) loggerMsgCreate(CLOSE, "", sizeof(""), "");
    return fifoPush(logger->buffer,data, priority, blocking);
}
