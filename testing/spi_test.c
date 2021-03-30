#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
// #include <stdint.h>
// #include <sys/mman.h>
// #include <sys/time.h>
// #include <sys/ioctl.h>
// #include <sched.h>
// #include <time.h>
// #include <fcntl.h>
// #include <unistd.h>
// //#include <linux/spi/spidev.h>

#define PIGPIO
#define BAUD (unsigned int)20e6

void printCharArray(char* arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++)
    {
        printf("%X ", arr[i]);
    }

    printf("\n");
}

#ifndef PIGPIO
char* spiTransact(int fd, char* tx_buf, int count)
{
    char* rx_buf = (char*) malloc(count*sizeof(*tx_buf));

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long) tx_buf,
        .rx_buf = (unsigned long) rx_buf,
        .len = count,
        .delay_usecs = 0,
        .speed_hz = BAUD,
        .bits_per_word = 8
    };

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    return rx_buf;
}
#endif
int main()
{

    #ifdef PIGPIO
    gpioInitialise();

    int handle = spiOpen(0,BAUD,0);
    printf("handle=%d\n", handle);
    #else
    int spi_driver = open("/dev/spidev0.0", O_RDWR);
    printf("spi_driver=%d\n",spi_driver);
    #endif

    char c;
    do 
    {
        scanf(" %c", &c);
        if (c == 'q') break;

        char tx_buff[] = {0x41, 0x0};
        
        #ifdef PIGPIO
        char rx_buff[sizeof(tx_buff)];
        printf("spiXfer=%d\n",spiXfer(handle, tx_buff,rx_buff, sizeof(tx_buff)));
        // printf("spiWrite=%d\n", spiWrite(handle, tx_buff, sizeof(tx_buff)));
        // printf("spiRead=%d\n", spiRead(handle, rx_buff, sizeof(tx_buff)));
        printf("rx_buff=");
        printCharArray(rx_buff,sizeof(rx_buff)); 
        
        #else
        char* rx_buff = spiTransact(spi_driver, tx_buff, sizeof(tx_buff));
        printf("rx_buff=");
        printCharArray(rx_buff, sizeof(tx_buff));
        free(rx_buff);
        #endif

    } while (c != 'q');

    #ifdef PIGPIO
    spiClose(handle);
    gpioTerminate();
    #else
    close(spi_driver);
    #endif
    return 0;
}