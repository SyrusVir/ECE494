#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef union __attribute__((__packed__))
{
    uint32_t data : 24;
} uint24_t;

typedef struct __attribute__((__packed__))
{
    uint32_t time1          : 24;
    uint32_t clock_count1   : 24;
    uint32_t time2          : 24;
    uint32_t calibration1   : 24;
    uint32_t calibration2   : 24;
} tdc_raw_t;

typedef union
{
    tdc_raw_t raw_struct;
    char raw_bytes[15];
    uint24_t raw_long[5];
} tdc_u;

int main ()
{
    printf("%d\n", sizeof(tdc_raw_t));
    printf("%d\n", sizeof(tdc_u));
    printf("%d\n", sizeof(uint24_t));
    tdc_u uni;

    char raw[] = {
        0, 1, 2,
        3, 4, 5,
        6, 7, 8,
        9, 10, 11,
        12, 13, 14
    };

    memcpy(uni.raw_bytes, raw, 15);

    for (int i = 0; i < sizeof(raw); i++)
    {
        printf("%d ", uni.raw_bytes[i]);
    }
    printf("\n");

    for (int i = 0; i < 5; i++)
    {
        printf("%X ", uni.raw_long[i].data);
    }
    printf("\n");

    printf("%X\n", uni.raw_struct.time2);



}