/*NOTE: Hex argument to mldExecuteCMD must be at least 5 bytes long*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint-gcc.h>
#include <string.h>
#include "MLD019.h"

#define SERIAL_TTY "/dev/ttyAMA0"
#define MSG_BYTES 6 //5-byte command + carriage return
#define BAUD 9600   //baud rate defined b MLD-019 datasheet
#define SERIAL_TIMEOUT_SEC 3
void printMsgStruct(mld_msg_u msg) {
    printf("%hhX ",msg.msg_struct.header);
    printf("%hhX ",msg.msg_struct.datum1);
    printf("%hhX ",msg.msg_struct.datum2);
    printf("%hhX ",msg.msg_struct.datum3);
    printf("%hhX\n",msg.msg_struct.checksum);
    return;
}

char mldChecksum(mld_msg_u msg){
    /**Parameteres: mld_msg_u msg - an MLD019 message union representation
     * Return:      char checksum - checksum
     * 
     * Description: Using the struct representation, calculate the checksum as the
     *              XOR of the upper 4-bytes (header and data). 
     * **/

    char checksum = 0x0;
    checksum = checksum ^ msg.msg_struct.header;
    checksum = checksum ^ msg.msg_struct.datum1;
    checksum = checksum ^ msg.msg_struct.datum2;
    checksum = checksum ^ msg.msg_struct.datum3;

    return checksum;
}   //end mldChecksum

mld_err_t mldValidateMsg(mld_msg_u msg) {
    /** Parameters: mld_msg_u msg - an MLD019 message union containing a received message
     *  Return:     mld_err_t - returns an enum indicating the present error. 0 if no error occurs.
     *                          Currently only two vals: 0 for no error, -1 for errors
    **/
    //printf("rec_sum=%hhX\tcalc_sum=%hhX\n",msg.msg_struct.checksum, mldChecksum(msg));

    if (msg.msg_struct.checksum != mldChecksum(msg)) { //if msg fails checksum test...
        if (msg.msg_struct.header == 0xFF) { //..check if error occured in pigpio serial routines
            
            return MLD_ERR;
            //return msg.msg_num_s & 0xFFFF;   //return least significant word of signed error code if so
        }
        else {
            return MLD_ERR;
            //return -147; //otherwise, no serial errors or simple checksum test failure, return -147 (unused by pigpio);
        }
    }
    else if (msg.msg_struct.header == 0xE0) {   //if error occurred driver-side, return error code
        return MLD_ERR;
        //return msg.msg_struct.datum1;
    }
    else {  //no errors occured
        return MLD_NOERR;
    }
} //end mldValidateMsg()

char* mldMsgToString(char* buff, mld_msg_u msg) {
    /** Parameters: char* buff - allocated buffer in which to place output
     *              mld_msg_u msg - message structure whose data will be 
     *                              converted into string form
     *  Return: char* buff - returns the same pointer passed in [buff]
     * 
     *  Description: Convert the hex command code contained in [msg] 
     *  into a string, appending a carriage return
     * 
     *  Command hex codes are 5 bytes long, transmitted as a 
     *  string of hex characters followed by a carriage return,
     *  totaling 11 bytes. Only the lower 5 bytes of the numeric
     *  representation of [msg] is used. This avoids excess 'F' hex
     *  characters if [msg] represents a negative number
     * 
     *  Resulting stirng is placed in [buff]. The pointer to [buff]
     *  is also returned.
     *  **/
    
    //0-padded, 10-character hex string + carriage return + NULL = 12 bytes
    snprintf(buff,12,"%0*llX\r",10, msg.msg_num_u & 0xFFFFFFFFFF);   

    return buff;
}   //end mldMsgToString()

mld_msg_u mldStringToMsg(char* str) {
    /** Parameters: char* str - a string of hexadecimal characters
     *  Return:     mld_msg_u msg - an MLD019 message union containing the hexadecimal number 
     *                              represented by [str]
     * Description: Uses the strtoll function to convert a string of hexadecimal characters into 
     *              a 64-bit number. This number is used to create an mld_msg_u union, which
     *              uses the lower 40 bits. 
     **/

    mld_msg_u msg;
    msg.msg_num_u = strtoll(str,NULL,16);
    return msg;
}   //end mldStringToMsg()

int mldSendMsg(mld_t* mld, mld_msg_u msg) {
    /** Parameters: mld_t* mld - a structure describing the current state of the MLD019 driver
     *              mld_msg_u msg - Union representing hexadecimal code to transmit
     * Return:      int serWrite() - status of the serWrite() call, which returns 0 if OK,
     *                               <0 otherwise. 
    **/
    char out_buff[12];  //1 byte header + 3 bytes data + 1 byte checksum + carriage return + NUll
    msg.msg_struct.checksum = mldChecksum(msg);
    mldMsgToString(out_buff, msg);
    //printf("out_buff=%s\n",out_buff);
    return serWrite(mld->serial_handle,out_buff, 11);
}   //end mldSendMsg

mld_msg_u mldRecvMsg(mld_t* mld) {
    /** Parameters: mld_t* mld - struct describing MLD019 driver to read from
     *  Return:     mld_msg_u recv_msg - a message union encpasulating the 5-byte return data or
     *                                   serial error
     * 
     *  Description: Uses the serRead command to read a number of bytes from the serial handle at once
     *               bytes_read tracks total bytes read from the serial buffer. Also acts as an index
     *               to recv_buff to prevent overwriting. num_bytes contains bytes read by a serRead()
     *               call or a serRead pigpio error code (<0). If num_bytes < 0, recv_msg contains 
     *               num_bytes. Note that pigpio error codes are always negative, thus
     *               recv_msg.msg_struct.header == 0xFF. This header is not used by any MLD019 command.     
     **/
    mld_msg_u recv_msg;
    char recv_buff[11]; //1 byte header + 3 bytes data + 1 byte checksum + carriage return
    int bytes_read = 0;
    int num_bytes = 0;
    while (bytes_read < 11) {
        
        //printf("bytes_read=%d\n",bytes_read);
        num_bytes = serRead(mld->serial_handle, recv_buff+bytes_read, 11 - bytes_read);

        if (num_bytes < 0) {
            //if serial read error occurs, return error code in recv_msg
            //printf("num_bytes<0\n");
            recv_msg.msg_num_s = num_bytes;
            return recv_msg;
        }
        else {
            bytes_read += num_bytes;
        }
    } //end while(bytes_read < 11)
    //printf("in_buff=%s\n", recv_buff);
    //if no serial errors encountered, parse received string into a message 
    recv_msg = mldStringToMsg(recv_buff);
    
    return recv_msg;
}

mld_msg_u mldExecuteCMD(mld_t* mld, uint64_t hex_cmd) {
    /** Parameters: mld_t* mld - struct representing MLD-019 driver
     *              uint64_t - 5-byte number matching a hexadecimal MLD019 command code
     *  Return:     mld_msg_u recv_msg - return message from MLD019
     * 
     *  Description: Utility function for sending commands to and receiving responses from an MLD019 
     *               laser driver. From [hex_cmd] construct and send a command message. Then read 11 
     *               bytes from serial and construct the returne message recv_msg. If an error 
     *               occurs in serial reading or writing, the resulting error code is contained in 
     *               recv_msg. 
     **/
    mld_msg_u send_msg;
    mld_msg_u recv_msg;

    //build command message
    send_msg.msg_num_u = hex_cmd;
    send_msg.msg_struct.checksum = mldChecksum(send_msg);
    //printf("outgoing= ");
    //printMsgStruct(send_msg);

    //clear Serial RX buffer
    while (serDataAvailable(mld->serial_handle) > 0) {
        //if no data error returned, break from loop early
        if(serReadByte(mld->serial_handle) == PI_SER_READ_NO_DATA) break;
    }

    //send message
    int send_status = mldSendMsg(mld, send_msg);
    //printf("send_status=%d\n",send_status);

    if (send_status < 0) {
        //if error occurs on write, recv_msg contains resulting unsigned error code
        recv_msg.msg_num_u = send_status;
    }
    else {
        //otherwise msg successflly transmitted; wait for return data
        int curr_sec;
        int stop_sec;
        int usec;
        
        //wait at most SERIAL_TIMEOUT_SEC for at least 11 bytes to be received
        gpioTime(PI_TIME_RELATIVE, &curr_sec, &usec);
        stop_sec = curr_sec + SERIAL_TIMEOUT_SEC;
        while(serDataAvailable(mld->serial_handle) < 11) 
        {
            gpioTime(PI_TIME_RELATIVE,&curr_sec, &usec);
            if (curr_sec >= stop_sec) break;
        }

        if (curr_sec < stop_sec) {
            //receive message; if error occurs on read, recv_msg contains resulting error code
            recv_msg = mldRecvMsg(mld);
        }
        else {
            printf("SERIAL TIMEOUT\n");
            recv_msg = mldStringToMsg("0xffffffffff");
        }
    }
    //printf("incoming= ");
    //printMsgStruct(recv_msg);

    return recv_msg;
}

mld_t* mldInit(char* sertty) {
    /** Parameters: char* sertty - path to the unix file representing the serial module
     *  Return:     mld_t* mld - a pointer to a  struct encapsulating a serial handle to the MLD019 driver
     *                          and its current operating mode
     * 
     * Description: Serial communications with an MLD019 is fixed at 9600 baud, 8n1 configuration.
     *              After a valid serial handle is obtained, the Link Control command is sent to
     *              confirm that the driver is responsive. If so, send commands querying the
     *              laser's current configuration
    **/
    mld_t* mld = malloc(sizeof(mld_t));
    mld_msg_u recv;
    //obtain serial handle
    mld->serial_handle = serOpen(sertty,9600,0);
    if (mld->serial_handle < 0) {
        //exit with error if serial connection could not be opened
        printf("CRITICAL Error:\n\t Failed to open serial connection at %s", sertty);
        exit(mld->serial_handle);
    } 
    else {
        mldCheckConfig(mld);
        return mld;
    }
}   //end mldInit

int mldClose(mld_t* mld){
    int close_status = serClose(mld->serial_handle);
    free(mld);
    return close_status;
}

////////////////////////////////////////MLD Available Commands//////////////////////////////////////////////////

int16_t mldLinkControl(mld_t* mld) {
    /** Parameters: mld_t* mld - struct encpasulating state of MLD019 driver
     *  Return:     int16_t - If true, MLD019 driver described by [mld] is responding
     *                     to serial communications **/

    mld_msg_u recv_msg = mldExecuteCMD(mld,0x04FFFF0000);

    return mldValidateMsg(recv_msg);
    
}

int64_t mldReadRTC(mld_t* mld){
    /** Returns the internal laser aging counter. A 24-bit number equal to seconds since 
     *  the laser has been armed.
     *  
     *  Return: uint32_t rtc = [Datum3][Datum2][Datum1][0x00] seconds if no error has occured
     *          if error occurs, return error code in recv_msg
    **/
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x0C00010000);

    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldReadRTC ERROR\n");
        return -1;
    }
    //extracting RTC
    uint32_t rtc = 0;
    uint32_t msk = 0xFF;
    rtc |= ((recv_msg.msg_struct.datum1 & msk) << 8); 
    rtc |= ((recv_msg.msg_struct.datum2 & msk) << 16); 
    rtc |= ((recv_msg.msg_struct.datum3 & msk) << 24); 

    return rtc; 
}

float mldCaseTemp(mld_t* mld) {
    /** Returns the temperature of the laser head.
     *  Case temperature = [Datum1][Datum2]/100 C **/

    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x1005000000);
    uint16_t temp = 0;  //unsure if return value is always unsigned

   
    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldCaseTemp ERROR\n");
        return -1;
    }


    temp = (recv_msg.msg_struct.datum1 << 8) | recv_msg.msg_struct.datum2;
    return temp/100.0;
}

float mldVLD(mld_t* mld){
    uint16_t voltage;
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x10A0000000);
    
    
    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldVLD ERROR\n");
        return -1;
    }

    voltage = (recv_msg.msg_struct.datum1 << 8) | recv_msg.msg_struct.datum2;
    return voltage/100.0;
    
}

int16_t mldStatus(mld_t* mld) {
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x100C000000);

    
    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldStatus ERROR\n");
        return -1;
    }

    return recv_msg.msg_struct.datum2;
}

uint32_t mldFirmware(mld_t* mld) {
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x101A000000);

    
    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldFirmware ERROR\n");
        return -1;
    }

    //TODO: Fix return; firmware version returned as hex characters, not decimal
    //      Fix by converting the data bytes into a string and then into a decimal number
    return (recv_msg.msg_num_u & 0x00FFFFFF00) >> 8;
}

float mldBoardTemp(mld_t* mld){
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x1018000000);
    uint16_t temp = 0;  //unsure if return value is always unsigned

    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldBoardTemp ERROR\n");
        return -1;
    }

    temp = (recv_msg.msg_struct.datum1 << 8) | recv_msg.msg_struct.datum2;
    
    return temp/100.0;
}

uint16_t mldDIMonitor(mld_t* mld) {
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x1010000000);
    
    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldDIMonitor ERROR\n");
        return -1;
    }
    return (recv_msg.msg_struct.datum1 << 8) | recv_msg.msg_struct.datum2;

}

uint32_t mldSerialNum(mld_t* mld) {
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x10A1000000);
    
    if(mldValidateMsg(recv_msg) != 0) {
        printf("mldSerialNum ERROR\n");
        return -1;
    }
    
    return (recv_msg.msg_num_u & 0x00FFFFFF00) >> 8;
}

uint32_t mldCheckConfig(mld_t* mld) {
    mld_msg_u recv_msg = mldExecuteCMD(mld, 0x300C10032F);

    if(mldValidateMsg(recv_msg) != 0) 
    {
        printf("mldReadRTC ERROR\n");
        return -1;
    }
    
    mld->mode = recv_msg.msg_struct.datum3 & 0xF0;
    mld->trigger_source = recv_msg.msg_struct.datum3 & 0x0F;
    return recv_msg.msg_num_u;
}

uint32_t mldHWConfig(mld_t* mld) {
    static const uint64_t commands[] = {
        0x310C10022F,
        0x3200004270,
        0x310C14022B,
        0x320000182A
    };

    mld_msg_u send_msg;
    for (int i = 0; i < 4; i++) {
        send_msg.msg_num_u = commands[i];
        send_msg.msg_struct.checksum = mldChecksum(send_msg);
        mldSendMsg(mld, send_msg);
    }

    //verify configuration
    return mldCheckConfig(mld);
}

uint32_t mldSWConfig(mld_t* mld) {
    static const uint64_t commands[] = {
        0x310C14022B,
        0x3200000032,
        0x310C10022F,
        0x3200007240
    };

    mld_msg_u send_msg;
    for (int i = 0; i < 4; i++) {
        send_msg.msg_num_u = commands[i];
        send_msg.msg_struct.checksum = mldChecksum(send_msg); //checksum calculation optional
        mldSendMsg(mld, send_msg);
    }

    //verify configuration
    return mldCheckConfig(mld);
}

uint32_t mldTrigConfig(mld_t* mld, mld_trig_t trig_src) {
    static const uint64_t commands[2] = {
        0x310c10022F,
        0x3200000000
    };
    
    //if already configured for desired trigger source, return
    if (mld->trigger_source == trig_src) return 0;

    //Sending first command
    mld_msg_u send_msg;
    send_msg.msg_num_u = commands[0];
    send_msg.msg_struct.checksum = mldChecksum(send_msg);
    mldSend(mld, send_msg);

    uint8_t target_config = (mld->mode | trig_src);
    send_msg.msg_num_u = commands[1];
    send_msg.msg_struct.datum3 = target_config;
    send_msg.msg_struct.checksum = mldChecksum(send_msg);
    mldSendMsg(mld, send_msg);

    return mldCheckConfig(mld);
}

uint16_t mldLaserControl(mld_t* mld, mld_controls_t cntrl) {
    static const uint64_t commands [4][2] = {
        {0x3101000131, 0x3200000133},   //set interlock
        {0x3101000131, 0x3200000032},   //unset interlock
        {0x3101000030, 0x3200000133},   //set enable
        {0x3101000030, 0x3200000032}    //unset enable
    };

    mld_msg_u send_msg;
    for (int i = 0; i < 2; i++) {
        send_msg.msg_num_u = commands[cntrl][i];
        send_msg.msg_struct.checksum = mldChecksum(send_msg);
        mldSendMsg(mld, send_msg);
    } 
    return mldDIMonitor(mld);
}

void mldSetPRR(mld_t* mld, uint16_t period_usec) {
    mld_msg_u send_msg;
    send_msg.msg_num_u = 0x310C0E0231;
    send_msg.msg_struct.checksum = mldChecksum(send_msg);
    mldSendMsg(mld, send_msg);

    send_msg.msg_num_u = 0x3200000000;
    send_msg.msg_struct.datum2 = (period_usec & 0xFF00) >> 8;
    send_msg.msg_struct.datum3 = period_usec & 0x00FF;
    send_msg.msg_struct.checksum = mldChecksum(send_msg);
    mldSendMsg(mld, send_msg);
}