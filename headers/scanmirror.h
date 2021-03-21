#ifndef _SCANMIRROR_H_
#define _SCANMIRROR_H_
#include <pigpio.h>
#include <stdint.h>

typedef struct Mirror {
    uint8_t FREQ_PIN;
    uint8_t ENABLE_PIN;
    uint8_t ATSPEED_PIN;
} mirror_t;

//Pass a configured mirror_t struct. 
//Configures the necessary pins. Returns 0 if successful.
int mirrorConfig(mirror_t m);

//Writes a logic low to the configured enable pin
int mirrorEnable(mirror_t m);

//Writes a logic hi to the configured enable pin
int mirrorDisable(mirror_t m);

//Configures the HW PWM on the configured frequency pin at the 
//appropriate frequency for the specified RPM. (ensure selected pin supports HW PWM)
int mirrorSetRPM(mirror_t m, uint16_t rpm);

//Reads the configured ATSPEED pin. If true, mirror is spinning at commanded speed.
int mirrorCheckAtSpeed(mirror_t m);
#endif
