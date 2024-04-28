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

/* scale detected range by 10 for proof of concept  */
constexpr double RANGE_SCALE    = 10;
/* m/s to km/hr */
constexpr double VELOCITY_SCALE = 3.6; 

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

static bool stopAlarmFlag = false;

void stopAlarm(void) {
    stopAlarmFlag = true;
    printf("\n\tStopping alarm service...");
    gpioDelay(1000000);
}

void toggleLED(void) {
    int level = gpioRead(LED_PIN);
    gpioWrite(LED_PIN, !level);
}

void *alarm_func(void *threadp) {
    uint32_t delay  = 0;
    objectData_t *objData;

    while(!stopAlarmFlag) {
        clock_gettime(CLOCK_REALTIME, &alarmStart);
        lockObjectData();
        objData = getObjectData();

        if(objData->range_cm > MAX_RANGE) {
        /* do nothing if no object detected in our given range */
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
        gpioSetTimerFunc(ALARM_TIMER, delay, toggleLED);
        printf("\033c");
        printf("*** Object Detected! ***\n");
        printf("\tPrevRange: %.02fm | Range: %.02fm | Velocity: %.02fkm/hr\n",
                    (objData->prevRange_cm), (objData->range_cm), 
                    (objData->velocity * VELOCITY_SCALE));
        printf("\tTime to Collision: %.02fs\n", objData->timeToCollision);
        
        clock_gettime(CLOCK_REALTIME, &alarmFinish);
        delta_t(&alarmFinish, &alarmStart, &alarmDelta);
        if(timestamp(&alarmDelta) > timestamp(&alarmWCET)) {
            alarmWCET.tv_sec    = alarmDelta.tv_sec;
            alarmWCET.tv_nsec   = alarmDelta.tv_nsec;
            //printf("\talarm WCET %lfms\n", timestamp(&alarmWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
    }
    gpioSetTimerFunc(ALARM_TIMER, 0, NULL);
    //destroyObjectDataSem();
    printf("\n\t\tFinal alarm WCET %lfms", timestamp(&alarmWCET));
    pthread_exit(NULL);
}

int configAlarm(void) {
    printf("Configuring LED Pin %d to Output Mode...", LED_PIN);
    if(check_gpio_error(gpioSetMode(LED_PIN,  PI_OUTPUT), LED_PIN)) {
        return -1;
    }
    printf("Done!\n");
    return 0;
}
