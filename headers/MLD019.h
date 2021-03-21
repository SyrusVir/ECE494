#ifndef _MLD019_H_
#define _MLD019_H

#include <pigpio.h>
#include <stdint-gcc.h>
#include <stdbool.h>

typedef enum mld_mode {
    MLD_MODE_HW = 0x40,
    MLD_MODE_SW = 0x70
} mld_mode_t;

typedef enum mld_trigger {
    MLD_TRIG_INT = 0x0,
    MLD_TRIG_EXT = 0x02
} mld_trig_t;

typedef enum mld_controls {
    MLD_SET_INTERLOCK,
    MLD_UNSET_INTERLOCK,
    MLD_SET_ENABLE,
    MLD_UNSET_ENABLE
} mld_controls_t;

typedef enum MldError {
    MLD_NOERR = 0,
    MLD_ERR = -1
} mld_err_t;

typedef struct MldDriver {
    mld_mode_t mode;
    mld_trig_t trigger_source;
    int serial_handle;

} mld_t;

typedef struct MldMessage {
    uint8_t checksum;  //LSB
    uint8_t datum3;
    uint8_t datum2;
    uint8_t datum1;
    uint8_t header;    //MSB
} mld_msg_t;

typedef union MldMessageUnion
{
    mld_msg_t msg_struct;
    char msg_arr[5];    //msg_arr[0] = LSB
    uint64_t msg_num_u: 40;
    int64_t msg_num_s: 40;
} mld_msg_u;

void printMsgStruct(mld_msg_u msg);
int mldSendMsg(mld_t* mld, mld_msg_u msg);
mld_msg_u mldRecvMsg(mld_t* mld);

//Calculates checksum as XOR of upper 4-bytes of [msg]
char mldChecksum(mld_msg_u msg);

//Checks msg for any errors. Returns 0 if none occured.
//Otherwise contains either a (negative) pigpio error code 
//or an MLD019 error byte
mld_err_t mldValidateMsg(mld_msg_u msg);

//Converting messages to hexadecimal string
char* mldMsgToString(char* buff, mld_msg_u msg); 

//convert hexadecimal string into a message
mld_msg_u mldStringToMsg(char* str);

//Utility function that sends a message containing [hex_cmd], returning the response.
mld_msg_u mldExecuteCMD(mld_t* mld, uint64_t hex_cmd);

//Open the serial module described by sertty and return a configured MLD struct
mld_t* mldInit(char* sertty);

//Use to verify serial connection and responsive MLD019 driver
int16_t mldLinkControl(mld_t* mld);

//Returns total laser on time
int64_t mldReadRTC(mld_t* mld);

//Returns temperature of laser head in Celsius
float mldCaseTemp (mld_t* mld);

//Returns voltage applied to driver
float mldVLD(mld_t* mld);

//Returns an 8-bit driver status number
int16_t mldStatus(mld_t* mld);

//Returns version of the control firmware
uint32_t mldFirmware(mld_t* mld);

//Returns the driver board temperature in Celsius
float mldBoardTemp(mld_t* mld);

//Updates the mode and trigger_source members of *mld with current driver configuration
uint32_t mldCheckConfig(mld_t* mld);

//Reads the state of the Enable (0th bit) and Interlock (1st bit) digital inputs
uint16_t mldDIMonitor(mld_t* mld); 

//Returns serial number of the laser head
uint32_t mldSerialNum(mld_t* mld);

//Updates *mld with the current driver configuration (control method and trigger source)
uint32_t mldCheckConfig(mld_t* mld);

//Configures the driver for control via TTL signals
uint32_t mldHWConfig(mld_t* mld);

//Configures the driver for control via serial commands
uint32_t mldSWConfig(mld_t* mld);

//Selects the trigger source as either external (TTL signals/serial commands)
//or internal (internal PRR generator)
uint32_t mldTrigConfig(mld_t* mld, mld_trig_t trig_src);


//Sets/unsets the interlock/enable
uint16_t mldLaserControl(mld_t* mld, mld_controls_t cntrl;

//sets the pulse repetition rate of the internal generator
void mldSetPRR(mld_t* mld, uint16_t period_usec);


#endif