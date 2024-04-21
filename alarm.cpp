/**
 * 
 * 
*/

#include <pigpio.h>
#include <stdio.h>

#include "alarm.h"
#include "sensor.h"
#include "misc.h"

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
static const double MID_RANGE     = 200;
static const double CLOSE_RANGE   = 100;
static const double DANGER_RANGE  = 50;

/* WCET timing */
struct timespec start   = {0, 0};
struct timespec finish  = {0, 0};
struct timespec WCET    = {0, 0};

/**
 *  > 300 we do nothing
 *  200 - 300   led blinks
 *  200 - 100   led blinks faster, buzzer starts
 *  100 - 50    led blinks faster, buzzer faster
 *  < 50        max blink / buzzer
*/

void *alarm_func(void *threadp) {
    uint32_t delay  = 0;
    double range    = 0;

    while(1) {
        clock_gettime(CLOCK_REALTIME, &start);

    /* skip over current iteration if we can't lock the semaphore */
        if(lockRangeSem()) {
            printf("Couldn't lock rangeSem!\n");
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
/* probably need external timer or counter instead of delay */
        printf("Object Detected! Range: %.02f\n", range);
        gpioWrite(LED_PIN, PI_ON);
        gpioDelay(delay);
        gpioWrite(LED_PIN, PI_OFF);
        gpioDelay(delay);

        delta_t(&finish, &start, &WCET);
        //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        printf("\talarm WCET %lf msec\n", timestamp(&WCET));
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
