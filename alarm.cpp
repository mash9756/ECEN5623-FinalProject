/**
 * 
 * 
*/

#include <pigpio.h>
#include <stdio.h>

#include "alarm.h"
#include "sensor.h"

/* LED is connected to GPIO26 (physical pin 37) */ 
    static const unsigned int LED_PIN   = 26;

int configAlarm(void) {

    printf("Configuring LED Pin %d to Output Mode...", LED_PIN);
    if(check_gpio_error(gpioSetMode(LED_PIN,  PI_OUTPUT), LED_PIN)) {
        return -1;
    }
    printf("Done!\n");

    gpioWrite(LED_PIN, PI_ON);

    return 0;
}
