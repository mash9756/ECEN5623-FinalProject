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
constexpr unsigned int LED_PIN      = 26;

/* delays are in ms */
constexpr unsigned int DELAY_1_SEC  = 1000;
constexpr unsigned int DELAY_500MS  = 500;
constexpr unsigned int DELAY_250MS  = 250;
constexpr unsigned int DELAY_100MS  = 100;
constexpr unsigned int DELAY_50MS   = 50;

/* Time To Collision alarm ranges */
constexpr double MIN_TTC    = 2.8;
constexpr double MID_TTC_1  = 2 * MIN_TTC;
constexpr double MID_TTC_2  = 3 * MIN_TTC;
constexpr double MID_TTC_3  = 4 * MIN_TTC;
constexpr double MAX_TTC    = 5 * MIN_TTC;

/* WCET timing */
static struct timespec alarmWCET    = {0, 0};
struct timespec alarmStart          = {0, 0};
struct timespec alarmFinish         = {0, 0};
struct timespec alarmDelta          = {0, 0};

/* thread exit flag, set by SIGINT handler */
static bool stopAlarmFlag = false;

/**
 * 
*/
void stopAlarm(void) {
    stopAlarmFlag = true;
    unlockObjectData();
    printf("\tStopping alarm service...\n");
    gpioDelay(1000000);
}

/**
 * 
*/
void toggleLED(void) {
    int level = gpioRead(LED_PIN);
    gpioWrite(LED_PIN, !level);
}

/**
 * 
*/
void *alarm_func(void *threadp) {
    uint32_t delay  = 0;
    objectData_t *objData;

    while(!stopAlarmFlag) {
        lockObjectData();
        clock_gettime(CLOCK_REALTIME, &alarmStart);
        objData = getObjectData();

        if((objData->timeToCollision > MAX_TTC) || (objData->timeToCollision < 0)) {
        /* disable LED alarm if outside max detection */
            gpioSetTimerFunc(ALARM_TIMER, 0, NULL);
            gpioWrite(LED_PIN, PI_OFF);
            continue;
        }
        else if(objData->timeToCollision > MID_TTC_1) {
            delay = DELAY_1_SEC; 
        }
        else if(objData->timeToCollision > MID_TTC_2) {
            delay = DELAY_500MS;
        }
        else if(objData->timeToCollision > MID_TTC_3) {
            delay = DELAY_250MS;
        }
        else if(objData->timeToCollision > MIN_TTC) {
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
                    (objData->velocity_kmPerHr));
        printf("\tTime to Collision: %.02fs\n", objData->timeToCollision);
        
        clock_gettime(CLOCK_REALTIME, &alarmFinish);
        delta_t(&alarmFinish, &alarmStart, &alarmDelta);
        if(timestamp(&alarmDelta) > timestamp(&alarmWCET)) {
            alarmWCET.tv_sec    = alarmDelta.tv_sec;
            alarmWCET.tv_nsec   = alarmDelta.tv_nsec;
        }
    }
/* thread cleanup */
    gpioSetTimerFunc(ALARM_TIMER, 0, NULL);
    destroyObjectDataSem();
    printf("\t\tFinal alarm WCET %lfms\n", timestamp(&alarmWCET));
    pthread_exit(NULL);
}

/**
 * 
*/
int configAlarm(void) {
    printf("Configuring LED Pin %d to Output Mode...", LED_PIN);
    if(check_gpio_error(gpioSetMode(LED_PIN,  PI_OUTPUT), LED_PIN)) {
        return -1;
    }
    printf("Done!\n");
    return 0;
}
