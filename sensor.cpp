/**
 * 
 * https://github.com/buildrobotsbetter/rpi4b_gpio-example/blob/main/pigpio/src/flash-led/flash-led.cpp
 * 
*/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <semaphore.h>
#include <stdint.h>
#include <unistd.h>

#include <pigpio.h>

#include "misc.h"
#include "sensor.h"

static sensorData_t sensorData = {};
static objectData_t objectData = {};

/* HC-SR04 TRIG is connected to GPIO23 (physical pin 16) */
static const unsigned int TRIG_PIN  = 23;  
/* HC-SR04 TRIG is connected to GPIO24 (physical pin 18) */  
static const unsigned int ECHO_PIN  = 24; 
/* Speed of sound at sea level, m/s*/
static const double SPEED_OF_SOUND  = 343.00;
/* meters to cm */
static const double M_TO_CM  = 100;
/* seconds to microseconds */
static const double SEC_TO_US  = 1000000;

/* locking for sensor data and processed object data */
pthread_cond_t sensorDataReady  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t sensorDataMutex = PTHREAD_MUTEX_INITIALIZER;

sem_t objectDataSem;

/* sensorRx WCET timing */
static struct timespec sensorRxWCET     = {0, 0};
struct timespec sensorRxStart           = {0, 0};
struct timespec sensorRxFinish          = {0, 0};
struct timespec sensorRxDelta           = {0, 0};

/* sensorProcess WCET timing */
static struct timespec sensorProcessWCET    = {0, 0};
struct timespec sensorProcessStart          = {0, 0};
struct timespec sensorProcessFinish         = {0, 0};
struct timespec sensorProcessDelta          = {0, 0};

static bool sensorDataFlag = false;

void lockSensorData(void) {
    int ret = pthread_mutex_lock(&sensorDataMutex);
    if(ret) {
        perror("sensorDataMutex");
        pthread_exit(NULL);
    }
}

void unlockSensorData(void) {
    pthread_mutex_unlock(&sensorDataMutex);
}

void waitSensorData(void) {
    int ret = pthread_cond_wait(&sensorDataReady, &sensorDataMutex);
    if(ret) {
        perror("sensorDataReady");
        pthread_exit(NULL);
    }
}

void lockObjectData(void) {
    sem_wait(&objectDataSem);
}

void unlockObjectData(void) {
    sem_post(&objectDataSem);
}


objectData_t *getObjectData(void) {
    return &objectData;
}

/* error check for individual gpio init */
int check_gpio_error(int ret, int pin) {
    if (ret != 0) {
        switch (ret) {
            case PI_BAD_GPIO:
                printf("GPIO %d is bad.\n", pin);
                return -1;
                break;
            case PI_BAD_MODE:
                printf("GPIO %d bad mode.\n", pin);
                return -1; 
                break;
            default:
                printf("GPIO %d unexpected result, error %d.\n", pin, ret);
                return -1;
                break;
        }
    }
    else {
        return 0;
    }
}

/* HC-SR04 sensor requires 10us pulse to trigger a measurement */
void trigger(void) {
    gpioWrite(TRIG_PIN, PI_ON);
    gpioDelay(10);
    gpioWrite(TRIG_PIN, PI_OFF);
}

/**
 * 
*/
void sensorRx(int pin, int level, uint32_t time_us) {
    int ret  = 0;
    static double start_us = 0;
    double read_us = (double)time_us;

    clock_gettime(CLOCK_REALTIME, &sensorRxStart);
/* get start tick when high detected */
    if(level == PI_ON) {
        start_us = read_us;
    }

    if(level == PI_OFF) {
        lockSensorData();
        sensorData.prevReadTime = sensorData.readTime;
        sensorData.readTime = read_us / SEC_TO_US;
        sensorData.echoTime = read_us - start_us;
        sensorDataFlag      = true;
        //printf("sensorData: Prev %.02f | Curr %.02f | Echo %.02f\n", sensorData.prevReadTime, sensorData.readTime, sensorData.echoTime);
        unlockSensorData();
        pthread_cond_signal(&sensorDataReady);
    }

    clock_gettime(CLOCK_REALTIME, &sensorRxFinish);
    delta_t(&sensorRxFinish, &sensorRxStart, &sensorRxDelta);
    if(timestamp(&sensorRxDelta) > timestamp(&sensorRxWCET)) {
        sensorRxWCET.tv_sec    = sensorRxDelta.tv_sec;
        sensorRxWCET.tv_nsec   = sensorRxDelta.tv_nsec;
        printf("\nsensorRx WCET %lfms\n", timestamp(&sensorRxWCET));
        //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
    }
}

/**
 *  distance = rate * time, rate = speed of sound, time is in microseconds
 *  convert m/s to cm/us to get distance in cm
 *  divide by 2 as the sonar pulse is travelling to the object and back
*/
void *sensorProcess_func(void *threadp) {
    int ret = 0;
    double time_us = 0;

    while(1) {
    /* lock sensor data while we process the data, but we need to wait for valid sensor data to be available */
        lockSensorData();
        while(!sensorDataFlag) {
            waitSensorData();
        }
        clock_gettime(CLOCK_REALTIME, &sensorProcessStart);
    /* save previous distance for velocity calc */
        objectData.prevRange_cm = objectData.range_cm;
    /* calculate latest detected range */
        objectData.range_cm     = ((sensorData.echoTime * SPEED_OF_SOUND * (M_TO_CM / SEC_TO_US)) / 2);
    /* calculate velocity based on time between detections and range difference */
        objectData.velocity     = (objectData.prevRange_cm - objectData.range_cm) / (sensorData.readTime - sensorData.prevReadTime);
    /* calculate time to collision based on current range and object speed */
        objectData.timeToCollision = objectData.range_cm / objectData.velocity;
    /* set data flags */
        sensorDataFlag = false;
        unlockSensorData();
        unlockObjectData();
        //printf("sensorData: Prev %.02f | Curr %.02f | Echo %.02f\n", sensorData.prevReadTime, sensorData.readTime, sensorData.echoTime);
        //printf("prevRange: %.02f | Range: %.02f |  velocity: %.02fcm/s\n", objectData.prevRange_cm, objectData.range_cm, objectData.velocity);

        clock_gettime(CLOCK_REALTIME, &sensorProcessFinish);
        delta_t(&sensorProcessFinish, &sensorProcessStart, &sensorProcessDelta);
        if(timestamp(&sensorProcessDelta) > timestamp(&sensorProcessWCET)) {
            sensorProcessWCET.tv_sec    = sensorProcessDelta.tv_sec;
            sensorProcessWCET.tv_nsec   = sensorProcessDelta.tv_nsec;
            printf("\n\tsensorProcess WCET %lfms\n", timestamp(&sensorProcessWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
    }
    pthread_exit(NULL);
}

int configHCSR04(void) {
    printf("Configuring TRIG Pin %d to Output Mode...", TRIG_PIN);
    if(check_gpio_error(gpioSetMode(TRIG_PIN,  PI_OUTPUT), TRIG_PIN)) {
        return -1;
    }
    printf("Done!\n");

    printf("Configuring ECHO Pin %d to Input Mode...", ECHO_PIN);
    if(check_gpio_error(gpioSetMode(ECHO_PIN,  PI_INPUT), ECHO_PIN)) {
        return -1;
    }
    printf("Done!\n");

/* turn off TRIG_PIN to start */
    gpioWrite(TRIG_PIN, PI_OFF);
/* setup timer to trigger the sonar sensor every 50ms */
    gpioSetTimerFunc(TRIGGER_TIMER, 50, trigger);
/*  */    
    gpioSetAlertFunc(ECHO_PIN, sensorRx);

    sem_init(&objectDataSem, 0, 1);

    return 0;    
}
