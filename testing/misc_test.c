#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

uint8_t getParity(uint32_t n)
{

    /**Calculate parity by repeatedly dividing the bits of n into halves
     * and XORing them together
     * 
     * 8-bit example:
     * n = b7b6b5b4b3b2b1b0
     * n ^= (n >> 4) --> n = b7 b6 b5 b4 (b7^b3) (b6^b2) (b5^b1) (b4^b0)
     * n ^= (n >> 2) --> n = b7 b6 b5 b4 (b7^b3) (b6^b2) (b7^b5^b3^b1) (b6^b4^b2^b0)
     * n ^= (n >> 1) --> n = b7 b6 b5 b4 (b7^b3) (b6^b2) (b7^b5^b3^b1) (b7^b6^b5^b4^b3^b2^b1^b0)
     * return n & 1 = return (b7^b6^b5^b4^b3^b2^b1^b0)
     */        
    for (size_t i = (sizeof(n)*8 >> 1); i > 0; i >>= 1)
    {
        n ^= (n >> i);
    }

    return n & 1;
}

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

    uint32_t x[5];
    printf("%d\n", sizeof(x));
    return 0;
} 