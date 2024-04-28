/**
 * 
 * 
*/

#include <pigpio.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>

#include "alarm.h"
#include "sensor.h"
#include "misc.h"

/* LED is connected to GPIO26 (physical pin 37) */ 
constexpr unsigned int LED_PIN       = 26;

/* delays are in ms */
constexpr unsigned int DELAY_2_SEC   = 2000;
constexpr unsigned int DELAY_1_SEC   = 1000;
constexpr unsigned int DELAY_500MS   = 500;
constexpr unsigned int DELAY_250MS   = 250;
constexpr unsigned int DELAY_100MS   = 100;
constexpr unsigned int DELAY_50MS    = 50;

/**
 *  > 300 we do nothing
 *  200 - 300   led blinks
 *  200 - 100   led blinks faster, buzzer starts
 *  100 - 50    led blinks faster, buzzer faster
 *  < 50        max blink / buzzer
*/
/* ranges in cm */
constexpr double MAX_ALARM_RANGE    = 300.00;
constexpr double MID_ALARM_RANGE    = 200.00;
constexpr double CLOSE_ALARM_RANGE  = 100.00;
constexpr double DANGER_ALARM_RANGE = 50.00;

/* WCET timing */
static struct timespec alarmWCET    = {0, 0};
struct timespec alarmStart          = {0, 0};
struct timespec alarmFinish         = {0, 0};
struct timespec alarmDelta          = {0, 0};

static bool stopAlarmFlag = false;

void stopAlarm(void) {
    stopAlarmFlag = true;
    unlockObjectData();
    printf("\tStopping alarm service...\n");
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
        lockObjectData();
        printf("sensorProcess Complete / alarm requested: %d\n", gpioTick());
        clock_gettime(CLOCK_REALTIME, &alarmStart);
        objData = getObjectData();

        if(objData->range_cm > MAX_ALARM_RANGE) {
        /* do nothing if no object detected in our given range */
            continue;
        }
        else if(objData->range_cm > MID_ALARM_RANGE) {
            delay = DELAY_500MS; 
        }
        else if(objData->range_cm > CLOSE_ALARM_RANGE) {
            delay = DELAY_250MS;
        }
        else if(objData->range_cm > DANGER_ALARM_RANGE) {
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
                    (objData->velocity));
        if(objData->timeToCollision <= 0) {
            printf("\tNo collision incoming, object relative velocity is zero or negative!\n");
        }
        else {
            printf("\tTime to Collision: %.02fs\n", objData->timeToCollision);
        }
        
        printf("alarm Complete: %d\n", gpioTick());
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
    destroyObjectDataSem();
    printf("\t\tFinal alarm WCET %lfms\n", timestamp(&alarmWCET));
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
