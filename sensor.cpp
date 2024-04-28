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
/* cm/s to km/hr conversion */
constexpr double CMPERS_TO_KMPERHR = 27.778;
/* delay for 1 sensor read period before data processing begins */
constexpr int TRIGGER_PERIOD = 50;

/* min/max detectable range for HC-SR04 */
constexpr double MAX_RANGE_CM = 400;
constexpr double MIN_RANGE_CM = 1;

/* min/max allowable echo time, discard anything outside as a misread */
constexpr double MAX_ECHO_TIME_US   = (MAX_RANGE_CM * 2 * SEC_TO_US) / (M_TO_CM * SPEED_OF_SOUND);
constexpr double MIN_ECHO_TIME_US   = (MIN_RANGE_CM * 2 * SEC_TO_US) / (M_TO_CM * SPEED_OF_SOUND);

/* number of samples used for averaging sensor data */
constexpr uint8_t AVG_READING_CNT = 5;

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

/* static instances of read and processed data */
static sensorData_t sensorData[5]   = {};
static sensorData_t sensorAvg       = {};
static objectData_t objectData      = {};

/* read counter for averaging sensor reads */
static uint8_t readCnt = 0;

/* data ready flag used in conjunction with cond signal */
static bool sensorDataFlag = false;

/* thread exit flag, set by SIGINT handler */
static bool sensorStopFlag = false;

/**
 * 
*/
void stopSensor(void) {
    sensorStopFlag = true;
    printf("\n\tStopping sensor services...\n");
    gpioDelay(1000000);
}

/**
 * 
*/
void lockSensorData(void) {
    int ret = pthread_mutex_lock(&sensorDataMutex);
    if(ret) {
        perror("sensorDataMutex");
        pthread_exit(NULL);
    }
}

/**
 * 
*/
void unlockSensorData(void) {
    pthread_mutex_unlock(&sensorDataMutex);
}

/**
 * 
*/
void waitSensorData(void) {
    int ret = pthread_cond_wait(&sensorDataReady, &sensorDataMutex);
    if(ret) {
        perror("sensorDataReady");
        pthread_exit(NULL);
    }
}

/**
 * 
*/
void lockObjectData(void) {
    int ret = sem_wait(&objectDataSem);
    if(ret) {
        perror("objectDataSem");
        pthread_exit(NULL);
    }
}

/**
 * 
*/
void unlockObjectData(void) {
    sem_post(&objectDataSem);
}

/**
 * 
*/
void destroyObjectDataSem(void) {
    sem_destroy(&objectDataSem);
}

/**
 * 
*/
objectData_t *getObjectData(void) {
    return &objectData;
}

/**
 *
 * error check for individual gpio init
*/
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

/**
 * 
 * HC-SR04 sensor requires 10us pulse to trigger a measurement
*/
void trigger(void) {
/* sensor triggered, start of data receive service */
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
    /* unlock and signal sensor data available so data processing thread can exit */
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

        /* lock sensor data while we ready 5 values for averaging */
            if(readCnt == 0) {
                lockSensorData();
            }

        /* store the sensor data and associated timing for processing */
            sensorData[readCnt].prevReadTime = sensorData[readCnt].readTime;
            sensorData[readCnt].readTime = read_us / SEC_TO_US;
            sensorData[readCnt].echoTime = read_us - start_us;
            readCnt++;

        /* all data for averaging captured, signal the data processing thread */
            if(readCnt >= AVG_READING_CNT) {
                readCnt = 0;
                sensorDataFlag = true;
                unlockSensorData();
                pthread_cond_signal(&sensorDataReady);
            }
        }

        clock_gettime(CLOCK_REALTIME, &sensorRxFinish);
        delta_t(&sensorRxFinish, &sensorRxStart, &sensorRxDelta);
        if(timestamp(&sensorRxDelta) > timestamp(&sensorRxWCET)) {
            sensorRxWCET.tv_sec    = sensorRxDelta.tv_sec;
            sensorRxWCET.tv_nsec   = sensorRxDelta.tv_nsec;
        }
    }
}

/**
 *  distance = rate * time, rate = speed of sound, time is in microseconds
 *  convert m/s to cm/us to get distance in cm
 *  divide by 2 as the sonar pulse is travelling to the object and back
*/
/**
 * 
 * 
*/
void *sensorProcess_func(void *threadp) {
    int ret = 0;
    double time_us = 0;

    while(!sensorStopFlag) {
    /** 
     *  lock sensor data while we process the data
     *      need to wait for valid sensor data to be available
     *      done using pthread_cond, see manual pages and references for examples
    */
        lockSensorData();
        while(!sensorDataFlag && !sensorStopFlag) {
            waitSensorData();
        }
    /* data read, start of data processing service */
        clock_gettime(CLOCK_REALTIME, &sensorProcessStart);

    /* average 5 sensor readings for stability */
        for (size_t i = 0; i < AVG_READING_CNT; i++)
        {
            sensorAvg.echoTime      += sensorData[i].echoTime;
            sensorAvg.prevReadTime  += sensorData[i].prevReadTime;
            sensorAvg.readTime      += sensorData[i].readTime;
        }
        sensorAvg.echoTime      = sensorAvg.echoTime / AVG_READING_CNT;
        sensorAvg.prevReadTime  = sensorAvg.prevReadTime / AVG_READING_CNT;
        sensorAvg.readTime      = sensorAvg.readTime / AVG_READING_CNT;

    /* save previous distance for velocity calc */
        objectData.prevRange_cm     = objectData.range_cm;
    /* calculate latest detected range */
        objectData.range_cm         = ((sensorAvg.echoTime * SPEED_OF_SOUND * (M_TO_CM / SEC_TO_US)) / 2);
    /* calculate velocity based on time between detections and range difference */
        objectData.velocity_cmPerS  = (objectData.prevRange_cm - objectData.range_cm) / (sensorAvg.readTime - sensorAvg.prevReadTime);
    /* calculate time to collision based on current range and object speed */
        objectData.timeToCollision  = objectData.range_cm / objectData.velocity_cmPerS;
    /* convert cm/s to km/hr */
        objectData.velocity_kmPerHr = objectData.velocity_cmPerS / CMPERS_TO_KMPERHR;
    /* set sensor data flags indicating processing is done */
        sensorDataFlag = false;
        unlockSensorData();
    /* post objectData ready semaphore for Alarm thread */
        unlockObjectData();

        clock_gettime(CLOCK_REALTIME, &sensorProcessFinish);
        delta_t(&sensorProcessFinish, &sensorProcessStart, &sensorProcessDelta);
        if(timestamp(&sensorProcessDelta) > timestamp(&sensorProcessWCET)) {
            sensorProcessWCET.tv_sec    = sensorProcessDelta.tv_sec;
            sensorProcessWCET.tv_nsec   = sensorProcessDelta.tv_nsec;
        }
    }
/* thread exit cleanup */
    pthread_mutex_destroy(&sensorDataMutex);
    pthread_cond_destroy(&sensorDataReady);
    printf("\t\tFinal sensorProcess WCET %lfms\n", timestamp(&sensorProcessWCET));
    pthread_exit(NULL);
}

/**
 * 
 * 
*/
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
    gpioSetTimerFunc(TRIGGER_TIMER, TRIGGER_PERIOD, trigger);

/* edge-detection callback for capturing sensor data */    
    gpioSetAlertFunc(ECHO_PIN, sensorRx);

/* objectData semaphore for data processing / alarm sync */
    sem_init(&objectDataSem, 0, 0);
    
/* delay for first sensor data processing */
    gpioDelay(TRIGGER_PERIOD * AVG_READING_CNT);

    return 0;    
}
