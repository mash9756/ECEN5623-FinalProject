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
static const unsigned int LED_PIN       = 26;

/* delays are in ms */
static const unsigned int DELAY_2_SEC   = 2000;
static const unsigned int DELAY_1_SEC   = 1000;
static const unsigned int DELAY_500MS   = 500;
static const unsigned int DELAY_250MS   = 250;
static const unsigned int DELAY_100MS   = 100;
static const unsigned int DELAY_50MS    = 50;

/**
 *  > 300 we do nothing
 *  200 - 300   led blinks
 *  200 - 100   led blinks faster, buzzer starts
 *  100 - 50    led blinks faster, buzzer faster
 *  < 50        max blink / buzzer
*/
/* ranges in cm */
static const double MAX_RANGE     = 300.00;
static const double MID_RANGE     = 200.00;
static const double CLOSE_RANGE   = 100.00;
static const double DANGER_RANGE  = 50.00;

/* WCET timing */
static struct timespec alarmWCET    = {0, 0};
struct timespec alarmStart          = {0, 0};
struct timespec alarmFinish         = {0, 0};
struct timespec alarmDelta          = {0, 0};

void toggleLED(void) {
    int level = gpioRead(LED_PIN);
    gpioWrite(LED_PIN, !level);
}

void *alarm_func(void *threadp) {
    uint32_t delay  = 0;
    objectData_t *objData;

    while(1) {
        clock_gettime(CLOCK_REALTIME, &alarmStart);

    /* lock objectData for alarm determinations */
        lockObjectData();
        objData = getObjectData();
        unlockObjectData();

        if(objData->range_cm > MAX_RANGE) {
        /* disable the timer if object detected is outside max range */
            gpioSetTimerFunc(ALARM_TIMER, 0, NULL);
            continue;
        }
        else if(objData->range_cm > MID_RANGE) {
            delay = DELAY_500MS; 
        }
        else if(objData->range_cm > CLOSE_RANGE) {
            delay = DELAY_250MS;
        }
        else if(objData->range_cm > DANGER_RANGE) {
            delay = DELAY_100MS;
        }
        else{
            delay = DELAY_50MS;
        }
        //printf("Object Detected!\n");
        //printf("PrevRange: %.02fcm | Range: %.02fcm\n", objData->prevRange_cm, objData->range_cm);
        //printf("Velocity: %.02fcm/s\n", objData->velocity);
        gpioSetTimerFunc(ALARM_TIMER, delay, toggleLED);

        clock_gettime(CLOCK_REALTIME, &alarmFinish);
        delta_t(&alarmFinish, &alarmStart, &alarmDelta);
        if(timestamp(&alarmDelta) > timestamp(&alarmWCET)) {
            alarmWCET.tv_sec    = alarmDelta.tv_sec;
            alarmWCET.tv_nsec   = alarmDelta.tv_nsec;
            printf("\talarm WCET %lfms\n", timestamp(&alarmWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
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
