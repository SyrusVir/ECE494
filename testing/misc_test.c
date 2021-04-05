#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include <string.h>

#define LIGHT_SPEED 299792458.0

#define DATA_SEPARATOR ','

double calcDist(double ToF)
{
    return ToF*LIGHT_SPEED/2.0;
}


double getEpochTime()
{
    static struct timeval tv;
    gettimeofday(&tv, NULL);

    return tv.tv_sec + tv.tv_usec *1E-6; 
}


int buildDataStr(char* out_str, double timestamp, double distance, double ToF, bool add_break)
{
    
    return sprintf(out_str, "%2$.6lf%1$c%3$.6lE%1$c%4$.6lE%5$s\n", 
            DATA_SEPARATOR,
            timestamp,
            distance,
            ToF,
            (add_break ? "\n" : "")
            );
}

int main (int argc, char** argv)
{
    /* // 8-bit array to 32-bit number test
    char data[13] = {0x00, 0x04, 0x04, 0x30, 0x22, 0xFF,
                        0x34, 0x34, 0x9f, 0xEE, 0xA2, 0x5C, 0x87};

    printf("%X\n",convertSubsetToLong(data+1,3,1));
    printf("\n");
    printf("%X\n",convertSubsetToLong(data+1,3,0));
 */

    // getParity test code
/* 
    uint32_t data = 1;
    if (argc > 1)
    {
        data = atoi(argv[1]);
    }

    uint32_t data_bits = sizeof(data)*8;
    printf("data_bits = %u\n", data_bits);
    printf("data = 0b");
    for (size_t i = 0; i < data_bits; i++)
    {
        printf("%hu", (data & (0x80000000 >> i)) != 0);
    }
    printf("\n");

    printf("parity of %u = %u\n", data, getParity(data));
 */

    // buildDataStr test code
    double time = getEpochTime();
    double ToF = 10E-6;
    double dist = calcDist(ToF);

    printf("time=%lf\tToF=%lf\tdist=%lf\n", time, ToF, dist);

    char data_str[60];
    int data_str_len = buildDataStr(data_str, time, dist, ToF, true);
    printf("data_str=%s\n", data_str);
    printf("data_str_size=%d\n", data_str_len);
    return 0;
} 