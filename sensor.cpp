/**
 * 
 * https://github.com/buildrobotsbetter/rpi4b_gpio-example/blob/main/pigpio/src/flash-led/flash-led.cpp
 * 
*/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>

#include <pigpio.h>

#include "misc.h"
#include "sensor.h"

static sensorData_t sensorData = {};
static objectData_t objectData = {};

/* HC-SR04 TRIG is connected to GPIO23 (physical pin 16) */
constexpr unsigned int TRIG_PIN  = 23;  
/* HC-SR04 TRIG is connected to GPIO24 (physical pin 18) */  
constexpr unsigned int ECHO_PIN  = 24; 
/* Speed of sound at sea level, m/s*/
constexpr double SPEED_OF_SOUND  = 343.00;
/* meters to cm */
constexpr double M_TO_CM  = 100;
/* seconds to microseconds */
constexpr double SEC_TO_US  = 1000000;
/* delay for 1 sensor read period before data processing begins */
constexpr int SENSOR_STARTUP_DELAY = 50000;
/* min/max allowable echo time, discard anything above as a misread */
constexpr double MAX_ECHO_TIME_US   = (MAX_RANGE_CM * 2 * SEC_TO_US) / (M_TO_CM * SPEED_OF_SOUND);
constexpr double MIN_ECHO_TIME_US   = (MIN_RANGE_CM * 2 * SEC_TO_US) / (M_TO_CM * SPEED_OF_SOUND);
/* max allowable speed detection, see  references */
constexpr double MAX_VELOCITY       =  150.00;
constexpr double MIN_VELOCITY       = -150.00;

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
static bool sensorStopFlag = false;

void stopSensor(void) {
    sensorStopFlag = true;
    printf("\n\tStopping sensor services...\n");
    gpioDelay(1000000);
}

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
    int ret = sem_wait(&objectDataSem);
    if(ret) {
        perror("objectDataSem");
        pthread_exit(NULL);
    }
}

void unlockObjectData(void) {
    sem_post(&objectDataSem);
}

void destroyObjectDataSem(void) {
    sem_destroy(&objectDataSem);
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
    clock_gettime(CLOCK_REALTIME, &sensorRxStart);
    gpioWrite(TRIG_PIN, PI_ON);
    gpioDelay(10);
    gpioWrite(TRIG_PIN, PI_OFF);
}

/**
 * 
*/
void sensorRx(int pin, int level, uint32_t time_us) {
    static double start_us = 0;
    int ret  = 0;
    double read_us = 0;
    double echo_us = 0;

    if(sensorStopFlag) {
    /* disable trigger timer, program ending */
        gpioSetTimerFunc(TRIGGER_TIMER, 0, NULL);
        sensorDataFlag = true;
        unlockSensorData();
        pthread_cond_signal(&sensorDataReady);
        printf("\t\tFinal sensorRx WCET %lfms\n", timestamp(&sensorRxWCET));
    }
    else {
        read_us = (double)time_us;
    /* get start tick when high detected */
        if(level == PI_ON) {
            start_us = read_us;
        }

        if(level == PI_OFF) {
        /* discard any reads that are out of bounds */
            echo_us = read_us - start_us;
            if(echo_us > MAX_ECHO_TIME_US || echo_us < MIN_ECHO_TIME_US) {
                return;
            }
        /* lock data for update */
            lockSensorData();
            sensorData.readCnt++;
            sensorData.prevReadTime = sensorData.readTime;
            sensorData.readTime = read_us / SEC_TO_US;
            sensorData.echoTime = read_us - start_us;
            sensorDataFlag      = true;
            unlockSensorData();
        /* signal processing thread that data is ready */
            pthread_cond_signal(&sensorDataReady);
        }

        clock_gettime(CLOCK_REALTIME, &sensorRxFinish);
        delta_t(&sensorRxFinish, &sensorRxStart, &sensorRxDelta);
        if(timestamp(&sensorRxDelta) > timestamp(&sensorRxWCET)) {
            sensorRxWCET.tv_sec    = sensorRxDelta.tv_sec;
            sensorRxWCET.tv_nsec   = sensorRxDelta.tv_nsec;
            //printf("sensorRx WCET %lfms\n\n", timestamp(&sensorRxWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
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

    while(!sensorStopFlag) {
    /* lock sensor data while we process the data, but we need to wait for valid sensor data to be available */
        lockSensorData();
        while(!sensorDataFlag && !sensorStopFlag) {
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
    /* ignore detected velocities higher than max possible speed */
        // if( ((objectData.velocity * VELOCITY_SCALE) > MAX_VELOCITY) || 
        //     ((objectData.velocity * VELOCITY_SCALE) < MIN_VELOCITY)) {
        //     continue;
        // }
    /* post objectData ready semaphore for Alarm thread */
        unlockObjectData();

        clock_gettime(CLOCK_REALTIME, &sensorProcessFinish);
        delta_t(&sensorProcessFinish, &sensorProcessStart, &sensorProcessDelta);
        if(timestamp(&sensorProcessDelta) > timestamp(&sensorProcessWCET)) {
            sensorProcessWCET.tv_sec    = sensorProcessDelta.tv_sec;
            sensorProcessWCET.tv_nsec   = sensorProcessDelta.tv_nsec;
            // printf("readCnt: %ld | sensorData: Prev %.02f | Curr %.02f | Echo %.02f\n", 
            //         sensorData.readCnt, sensorData.prevReadTime, 
            //         sensorData.readTime, sensorData.echoTime);
            // printf("prevRange: %.02f | Range: %.02f |  velocity: %.02fcm/s\n", 
            //         objectData.prevRange_cm, objectData.range_cm, objectData.velocity);
            // printf("sensorProcess WCET %lfms\n\n", timestamp(&sensorProcessWCET));
            //syslog(LOG_NOTICE, "\talarm WCET %lf msec\n", timestamp(&WCET));
        }
    }
    pthread_mutex_destroy(&sensorDataMutex);
    pthread_cond_destroy(&sensorDataReady);
    printf("\t\tFinal sensorProcess WCET %lfms\n", timestamp(&sensorProcessWCET));
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

    objectData.prevRange_cm     = 0;
    objectData.range_cm         = 0;
    objectData.timeToCollision  = 0;
    objectData.velocity         = 0;

    sensorData.echoTime         = 0;
    sensorData.prevReadTime     = 0;
    sensorData.readTime         = 0;
    sensorData.readCnt          = 0;

/* turn off TRIG_PIN to start */
    gpioWrite(TRIG_PIN, PI_OFF);

/* setup timer to trigger the sonar sensor every 75ms */
    gpioSetTimerFunc(TRIGGER_TIMER, 75, trigger);

/* edge-detection callback for capturing sensor data */    
    gpioSetAlertFunc(ECHO_PIN, sensorRx);

/* objectData semaphore for processing/alarm sync */
    sem_init(&objectDataSem, 0, 0);
    
/* delay for first sensor read */
    gpioDelay(SENSOR_STARTUP_DELAY);

    return 0;    
}
