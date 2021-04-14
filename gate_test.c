#include <pigpio.h>
#include <stdio.h>
#include <stdbool.h>

#define GATE_PIN 5 // physical pin 29

int main()
{
    bool gate_state = false;
    gpioInitialise();

    gpioSetMode(GATE_PIN, PI_OUTPUT);
    gpioWrite(GATE_PIN, gate_state);

    char c = '0';
    while(1)
    {
        printf("Q or q to quit\n");
        printf("Else change output\n");
        scanf(" %c", &c);

        if (c == 'q' || c == 'Q') break;
        else
        {
            gate_state = !gate_state;
            printf("gate_state=%d\n",gate_state);
            gpioWrite(GATE_PIN, gate_state);
        }
    }    

    gpioWrite(GATE_PIN, 0);
    gpioTerminate();
    return 0;
}