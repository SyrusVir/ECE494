#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
#include "fifo.h"

typedef enum LoggerStatus {
    LOGGER_UNINIT,
    LOGGER_IDLE,
    LOGGER_WORKING,
    LOGGER_STOPPED
} logger_status_t;

typedef enum LoggerState {
    LOGGER_STATE_UNINIT,
    LOGGER_STATE_IDLE,
    LOGGER_STATE_LOGGING,
    LOGGER_STATE_CLOSED
} logger_state_t;

typedef enum LoggerCommand {
    LOGGER_LOG,
    LOGGER_STOP
} logger_cmd_t;

typedef struct LoggerMessage {
    logger_cmd_t cmd;
    uint16_t data_leng;
    char *path;
    char *data;
} logger_msg_t;

typedef struct Logger {
    fifo_buffer_t *buffer;
    char *stat_log_path;
    logger_status_t status;
} logger_t; 

//CORE FUNCTION
void* loggerMain(void* logger);

//Constructor functions
logger_t* loggerCreate(uint16_t buffer_size);
logger_msg_t* loggerMsgCreate(logger_cmd_t cmd, char* data_str, size_t data_size, char* path);

//Destructor functions
logger_msg_t** loggerDestroy(logger_t* logger);
void loggerMsgDestroy(logger_msg_t* msg);


//Utility functions
int logStatus(logger_t* logger, char* msg); //logs msg to Loggers status log file
int loggerSendLogMsg(logger_t* logger, char* data_str, size_t data_str_size, char* path, int priority, bool blocking);
int loggerSendCloseMsg(logger_t* logger, int priority, bool blocking);
#endif