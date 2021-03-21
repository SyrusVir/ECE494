#include "fifo.h"
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>

#include <netinet/in.h>
#include <netinet/tcp.h>


typedef enum TcpHandlerCMD {
    WRITE,
    DISCONNECT,
    CLOSE
} tcp_cmd_t;

typedef enum TcpHandlerState {
    UNCONNECTED,
    CONNECTED,
    ERROR
} tcp_state_t;

typedef struct TcpHandlerMsg {
    tcp_cmd_t CMD;
    char* data;
    size_t data_len;
} tcp_msg_t;

typedef struct TcpHandler {
    struct sockaddr_in server_address;  //address to place socket that listens for client connections
    tcp_state_t tcp_state;              //current state of the tcpHandler; check from producer to see if ready for writes
    fifo_buffer_t* write_buffer;        //fifo buffer holding data to write
} tcp_handler_t;

tcp_msg_t* tcpMsgCreate(tcp_cmd_t CMD, char* data, size_t data_len)
{
    tcp_msg_t* tcp_msg = (tcp_msg_t*) malloc(sizeof(tcp_msg_t));
    tcp_msg->CMD = CMD;
    tcp_msg->data_len = data_len;
    
    if (data != NULL)
    {
        //if valid data provided, duplicate into message struct (tcp_msg->data MUST BE FREED USING DESTRUCTOR)
        tcp_msg->data = strndup(data, data_len);
    }
    else
    {
        //OTW assign NULL pointer
        tcp_msg->data = NULL;
    }
    return tcp_msg;
} //end tcpMsgCreate()

void tcpMsgDestroy(tcp_msg_t* tcp_msg)
{
    free(tcp_msg->data);
    free(tcp_msg);

    return;
} //end tcpMsgDestroy()

int tcpConfigKeepalive(int socket, int idle_time_sec, int num_probes, int probe_intvl_sec)
{
    uint8_t val = 1;
    setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE,&val, sizeof(val));
    
    //set idle time in seconds before sending keepalive packets
    setsockopt(socket, SOL_TCP, TCP_KEEPIDLE, &idle_time_sec, sizeof(idle_time_sec));
    
    //Set # of keepalive packets to send before declaring connection dead
    setsockopt(socket, SOL_TCP, TCP_KEEPCNT, &num_probes, sizeof(num_probes));

    //Set time interval in seconds between keepalive packets
    setsockopt(socket, SOL_TCP, TCP_KEEPCNT, &probe_intvl_sec, sizeof(probe_intvl_sec));
}

tcp_handler_t* tcpHandlerInit(struct sockaddr_in server_address, int max_buffer_size)
{   
    /*  Assumes [server_address] is already configured with the desired port, settings, etc.
        Instantiates the tcpHandler structure for access from main thread and handling thread */

    tcp_handler_t* tcpHandler = (tcp_handler_t*) malloc(sizeof(struct sockaddr_in) + sizeof(tcp_state_t) + sizeof(fifo_buffer_t*));
    tcpHandler->server_address = server_address;
    tcpHandler->tcp_state = UNCONNECTED;
    tcpHandler->write_buffer = fifoBufferInit(max_buffer_size);
    
    return tcpHandler;
} //end tcpHandlerInit()

tcp_msg_t** tcpHandlerDestroy(tcp_handler_t* tcp_handler) 
{   
    tcp_msg_t** leftover = (tcp_msg_t**)fifoBufferClose(tcp_handler->write_buffer);
    free(tcp_handler);
    return leftover; 
} //end tcpHandlerDestroy()

int tcpHandlerWrite(tcp_handler_t* tcp_handler, char* data, size_t data_len, int priority, bool blocking)
{
    void* msg = (void*)tcpMsgCreate(WRITE, data, data_len);
    return fifoPush(tcp_handler->write_buffer, msg, priority, blocking);
} //end tcpHandlerWrite()

int tcpHandlerClose(tcp_handler_t* tcp_handler, int priority, bool blocking)
{
    void* msg = (void*)tcpMsgCreate(CLOSE, NULL, 0);
    return fifoPush(tcp_handler->write_buffer, msg, priority, blocking);
} //end tcpHandlerWrite()

int tcpHandlerDisconnect(tcp_handler_t* tcp_handler, int priority, bool blocking)
{
    void* msg = (void*)tcpMsgCreate(DISCONNECT, NULL, 0);
    return fifoPush(tcp_handler->write_buffer, msg, priority, blocking);
} //end tcpHandlerDisconnect()

void* tcpHandlerMain(void* tcpHandler_void)
{
    //cast void argument to appropriate pointer type
    tcp_handler_t* tcpHandler = (tcp_handler_t*) tcpHandler_void;
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket  < 0) 
	{
		perror("socket() call failed!");
        return NULL;
	}
    printf("tcp_handler: server socket created\n");

    // bind server socket to the address
	if (bind(server_socket, (struct sockaddr*) &tcpHandler->server_address, sizeof(tcpHandler->server_address)) < 0) 
	{
		perror("bind() call failed!");
		return NULL;
	}
    printf("tcp_handler: binding\n");

    //listen for at most 1 connection
    if (listen(server_socket, 1) < 0) 
	{
		perror("listen() failed!");
		return NULL;
	}
    printf("tcp_handler: listneing\n");

    tcp_msg_t* recv_msg;
    while (1) 
    {
        switch (tcpHandler->tcp_state)
        {
            case UNCONNECTED:;
                //wait for an incoming connection; create new client socket upon connection
                int client_socket;
                printf("tcp_handler: accepting\n");

                client_socket = accept(server_socket, NULL, NULL);
                if (client_socket < 0) 
                {
                    perror("accept() failed");
                    return NULL;
                }

                //config TCP Keepalive
                tcpConfigKeepalive(client_socket, 15, 5, 3);

                //state transition
                tcpHandler->tcp_state = CONNECTED;
                break; //end case UNCONNECTED

            case CONNECTED:
                while (tcpHandler->tcp_state == CONNECTED)
                {
                    printf("tcpHandlerMain: waiting for CMD\n");
                    recv_msg = (tcp_msg_t*)fifoPull(tcpHandler->write_buffer,1);
                    printf("tcp_handler: received %d\n", recv_msg->CMD);
                    switch(recv_msg->CMD)
                    {
                        case WRITE:;
                            printf("tcpHandlerMain: about to send\n");
                            ssize_t send_status = send(client_socket,recv_msg->data, recv_msg->data_len, MSG_NOSIGNAL);
                            printf("tcpHandlerMain: send_status=%ld\n", send_status);
                            if (send_status >= 0) 
                            {
                                if(send_status < recv_msg->data_len) 
                                {
                                    printf("Not all data sent\n");
                                }
                            }
                            else if (errno == EPIPE) 
                            {
                                printf("sending on dead socket. Breaking from loop.\n");
                                tcpHandler->tcp_state = UNCONNECTED;
                                break;
                            }
                            else 
                            {
                                perror("Error in send(): ");
                                tcpHandler->tcp_state = UNCONNECTED;
                                break;
                            }
                            break;
                        case DISCONNECT:
                            close(client_socket);
                            printf("tcp_handler: DISCONNECT executed\n");
                            tcpHandler->tcp_state = UNCONNECTED;
                            break;
                        case CLOSE:
                            close(client_socket);
                            printf("tcp_handler: CLOSE executed\n");
                            return tcpHandlerDestroy(tcpHandler);
                            break;
                        default:
                            break;
                    } //end switch(recv_msg->CMD) in CONNECTED case
                    
                    tcpMsgDestroy(recv_msg);

                } //end while (tcpHandler->tcp_state == CONNECTED)
                break; // end case CONNECTEDs
        } //end switch(tcp_state)
    } //end while (1) [main loop]
} //end tcpHandlerMain()