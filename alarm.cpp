/**
 * 
 * 
*/

#include <pigpio.h>
#include <stdio.h>

#include "alarm.h"
#include "sensor.h"

/* LED is connected to GPIO26 (physical pin 37) */ 
static const unsigned int LED_PIN = 26;

/* delays are in microseconds */
static const unsigned int DELAY_2_SEC   = 2000000;
static const unsigned int DELAY_1_SEC   = 1000000;
static const unsigned int DELAY_500MS   = 500000;
static const unsigned int DELAY_250MS   = 250000;
static const unsigned int DELAY_100MS   = 100000;

/* ranges in cm */
static const unsigned int MAX_RANGE     = 300;
static const unsigned int MID_RANGE     = 150;
static const unsigned int CLOSE_RANGE   = 100;
static const unsigned int DANGER_RANGE  = 50;

void *alarm_func(void *threadp) {
    uint32_t delay  = 0;
    double range    = 0;

    while(1) {
    /* skip over current iteration if we can't lock the semaphore */
        if(lockRangeSem()) {
            continue;
        }
        range = getRange();
        unlockRangeSem();

        switch (range)
        {
        case sensorData > MAX_RANGE:
            delay = DELAY_1_SEC;
            break;
        case sensorData > MID_RANGE:
            delay = DELAY_500MS; 
            break;
        case sensorData > CLOSE_RANGE:
            delay = DELAY_250MS;
            break;
        case sensorData > DANGER_RANGE:
            delay = DELAY_100MS;
            break;
        default:
            delay = DELAY_2_SEC;
            break;
        }

        printf("\nObject Detected! Range: %.02f", range);
        gpioWrite(LED_PIN, PI_ON);
        gpioDelay(delay);
        gpioWrite(LED_PIN, PI_OFF);
        gpioDelay(delay);
    }
    return NULL;
}

int configAlarm(void) {
    printf("Configuring LED Pin %d to Output Mode...", LED_PIN);
    if(check_gpio_error(gpioSetMode(LED_PIN,  PI_OUTPUT), LED_PIN)) {
        return -1;
    }
    printf("Done!\n");
    return 0;
}
