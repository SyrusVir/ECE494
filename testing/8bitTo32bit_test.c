#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

uint32_t convertSubsetToLong(char* start, int len, bool big_endian)
{
    /**Parameters: uint8_t* start - pointer to first element in byte array subset
     *             int len - number of elements, max 4, in the subset
     *             bool big_endian - if true, the first element of start is considerd the MSB
     * Returns: 
     */

    len = (len > 4 ? 4 : len); //if len > 4, assign 4. OTW assign user-provided length
    uint32_t out = 0;
    for (int i = 0; i < len; i++)
    {
        out |= (start[(big_endian ? len - 1 - i : i)] << 8*i);
    }

    return out;
}

int main ()
{
    char data[13] = {0x00, 0x04, 0x04, 0x30, 0x22, 0xFF,
                        0x34, 0x34, 0x9f, 0xEE, 0xA2, 0x5C, 0x87};

    printf("%X\n",convertSubsetToLong(data+1,3,1));
    printf("\n");
    printf("%X\n",convertSubsetToLong(data+1,3,0));

} 