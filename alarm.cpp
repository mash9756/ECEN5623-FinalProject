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
static const double MAX_RANGE     = 300;
static const double MID_RANGE     = 150;
static const double CLOSE_RANGE   = 100;
static const double DANGER_RANGE  = 50;

void *alarm_func(void *threadp) {
    uint32_t delay  = 0;
    double range    = 0;

    while(1) {
    /* skip over current iteration if we can't lock the semaphore */
        if(lockRangeSem()) {
            printf("\nCouldn't lock rangeSem!");
            continue;
        }
        range = getRange();
        unlockRangeSem();

        if(range > MAX_RANGE) {
            delay = DELAY_2_SEC;
        }
        else if(range > MID_RANGE) {
            delay = DELAY_1_SEC; 
        }
        else if(range > CLOSE_RANGE) {
            delay = DELAY_500MS;
        }
        else if(range > DANGER_RANGE) {
            delay = DELAY_250MS;
        }
        else{
            delay = DELAY_100MS;
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
