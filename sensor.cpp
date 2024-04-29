/**
 *  @file       capture.cpp
 *  @author     Mark Sherman
 *  @date       4/28/2024
 * 
 *  @brief      reference links below
 *              [1] https://stackoverflow.com/questions/16522858/understanding-of-pthread-cond-wait-and-pthread-cond-signal
 *              [2] https://github.com/buildrobotsbetter/rpi4b_gpio-example/blob/main/pigpio/src/flash-led/flash-led.cpp
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <syslog.h>

#include <pigpio.h>

#include "misc.h"
#include "sensor.h"

/* HC-SR04 TRIG is connected to GPIO23 (physical pin 16) */
constexpr unsigned int TRIG_PIN  = 23;  
/* HC-SR04 TRIG is connected to GPIO24 (physical pin 18) */  
constexpr unsigned int ECHO_PIN  = 24; 

/* Speed of sound at sea level, m/s for range calc */
constexpr double SPEED_OF_SOUND = 343.00;
/* meters to cm conversion for velocity calc */
constexpr double M_TO_CM        = 100;
/* seconds to microseconds conversion for velocity calc */
constexpr double SEC_TO_US      = 1000000;
/* cm/s to km/hr conversion for TTC calc */
constexpr double CMPS_TO_KMPHR  = 27.778;

/* sensor read period */
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

/* sensor latency timing */
static struct timespec sensorLatencyWCET    = {0, 0};
struct timespec sensorLatencyStart      = {0, 0};
struct timespec sensorLatencyFinish     = {0, 0};
struct timespec sensorLatencyDelta      = {0, 0};

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
 *  @name   stopSensor
 *  @brief  signals program exit was triggered, break sensor service loop and delay for final execution
 * 
 *  @param  NONE
 *  @return VOID
*/
void stopSensor(void) {
    sensorStopFlag = true;
    printf("\n\tStopping sensor services...\n");
    gpioDelay(EXIT_DELAY);
}

/**
 *  @name   lockSensorData
 *  @brief  wrapper for sensor data mutex lock
 * 
 *  @param  VOID
 *  @return VOID, exit current running thread on failure
*/
void lockSensorData(void) {
    int ret = pthread_mutex_lock(&sensorDataMutex);
    if(ret) {
        perror("sensorDataMutex");
        pthread_exit(NULL);
    }
}

/**
 *  @name   unlockSensorData
 *  @brief  wrapper for sensor data mutex unlock
 * 
 *  @param  VOID
 *  @return VOID
*/
void unlockSensorData(void) {
    pthread_mutex_unlock(&sensorDataMutex);
}

/**
 *  @name   waitSensorData
 *  @brief  wrapper for sensor data signal conditional wait
 * 
 *  @param  VOID
 *  @return VOID, exit current running thread on failure
*/
void waitSensorData(void) {
    int ret = pthread_cond_wait(&sensorDataReady, &sensorDataMutex);
    if(ret) {
        perror("sensorDataReady");
        pthread_exit(NULL);
    }
}

/**
 *  @name   lockObjectData
 *  @brief  wrapper for processed data ready semaphore wait
 * 
 *  @param  VOID
 *  @return VOID, exit current running thread on failure
*/
void lockObjectData(void) {
    int ret = sem_wait(&objectDataSem);
    if(ret) {
        perror("objectDataSem");
        pthread_exit(NULL);
    }
}

/**
 *  @name   unlockObjectData
 *  @brief  wrapper for processed data ready semaphore post
 * 
 *  @param  VOID
 *  @return VOID, exit current running thread on failure
*/
void unlockObjectData(void) {
    sem_post(&objectDataSem);
}

/**
 *  @name   destroyObjectData
 *  @brief  wrapper for deinit of the processed data ready semaphore
 * 
 *  @param  VOID
 *  @return VOID
*/
void destroyObjectDataSem(void) {
    sem_destroy(&objectDataSem);
}

/**
 *  @name   getObjectData
 *  @brief  getter for static global instance of the current processed sensor data
 * 
 *  @param  VOID
 *  @return pointer to processed data instance
*/
objectData_t *getObjectData(void) {
    return &objectData;
}

/**
 *  @name   triggerSensor
 *  @brief  initiate a sensor read by toggling the TRIG pin
 *          also take a timestamp for WCET calc
 *          HC-SR04 sensor requires 10us pulse on the TRIG pin to trigger a read
 * 
 *  @param  NONE
 *  @return VOID
*/
void triggerSensor(void) {
/* sensor triggered, start of sensor data receive latency */
    clock_gettime(CLOCK_REALTIME, &sensorLatencyStart);
    delta_t(&sensorLatencyStart, getSystemStartTime(), &sensorLatencyStart);
    syslog(LOG_NOTICE, "\tsensorLatency start %.02fms\n", timestamp(&sensorLatencyStart));

    gpioWrite(TRIG_PIN, PI_ON);
    gpioDelay(10);
    gpioWrite(TRIG_PIN, PI_OFF);
}

/**
 *  @name   sensorRx
 *  @brief  sensor data receive service function
 *          registered as a callback to the level detection function for the ECHO pin
 *          
 *  @param  pin     pin registered for level detection
 *  @param  level   measured level, PI_HI or PI_LOW
 *  @param  time_us system tick when level detection occurred
 * 
 *  @return VOID
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
        printf("\t\tFinal sensorRx WCET %.02fms\n", timestamp(&sensorRxWCET));
        syslog(LOG_NOTICE, "\t\tFinal sensorRx WCET %.02fms\n", timestamp(&sensorRxWCET));
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

        /* WCET of the actual sensor receive service is just reading the data */
            clock_gettime(CLOCK_REALTIME, &sensorRxStart);
            delta_t(&sensorRxStart, getSystemStartTime(), &sensorRxStart);
            syslog(LOG_NOTICE, "\tsensorRx service start %.02fms\n", timestamp(&sensorRxStart));

        /* end sensor read latency, data is ready to read */ 
            sensorLatencyFinish = sensorRxStart;
            delta_t(&sensorLatencyFinish, &sensorLatencyStart, &sensorLatencyDelta);
            syslog(LOG_NOTICE, "\tsensorLatency end %.02fms | ET: %.02fms\n", timestamp(&sensorLatencyFinish), timestamp(&sensorLatencyDelta));
            if(updateWCET(&sensorLatencyDelta, &sensorLatencyWCET)) {
                syslog(LOG_NOTICE, "\tsensorLatency WCET Updated: %.02fms\n", timestamp(&sensorLatencyWCET));
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
            clock_gettime(CLOCK_REALTIME, &sensorRxFinish);
            delta_t(&sensorRxFinish, getSystemStartTime(), &sensorRxFinish);
            delta_t(&sensorRxFinish, &sensorRxStart, &sensorRxDelta);
            syslog(LOG_NOTICE, "\tsensorRx service end: %.02fms | ET: %.02fms\n", timestamp(&sensorRxFinish), timestamp(&sensorRxDelta));
            if(updateWCET(&sensorRxDelta, &sensorRxWCET)) {
                syslog(LOG_NOTICE, "\tsensorRx WCET Updated: %.02fms\n", timestamp(&sensorRxWCET));
            }
        }
    }
}

/**
 *  @name   sensorProcess_func
 *  @brief  sensor data processing service function
 *          waits for 5 sensor reads to be completed, then averages the raw data
 *          then calculates object detection range, velocity, and TTC
 *          once complete, signals alarm service to run via the objectDataSem
 * 
 *          distance = rate * time, rate = speed of sound, time is in microseconds
 *          convert m/s to cm/us to get distance in cm
 *          divide by 2 as the sonar pulse is travelling to the object and back
 * 
 *  @param  threadp     thread parameters, unused
 *  @return VOID
*/
void *sensorProcess_func(void *threadp) {
    int ret = 0;
    double time_us = 0;

    while(!sensorStopFlag) {
    /** 
     *  lock sensor data while we process the data
     *      need to wait for valid sensor data to be available
     *      done using pthread_cond, see manual pages and reference [1] for examples
    */
        lockSensorData();
        while(!sensorDataFlag && !sensorStopFlag) {
            waitSensorData();
        }
    /* data read, start of data processing service */
        clock_gettime(CLOCK_REALTIME, &sensorProcessStart);
        delta_t(&sensorProcessStart, getSystemStartTime(), &sensorProcessStart);
        syslog(LOG_NOTICE, "\tsensorProcess service start %.02fms\n", timestamp(&sensorProcessStart));

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
        objectData.velocity_kmPerHr = objectData.velocity_cmPerS / CMPS_TO_KMPHR;
    /* set sensor data flags indicating processing is done */
        sensorDataFlag = false;
        unlockSensorData();
    /* post objectData ready semaphore for Alarm thread */
        unlockObjectData();

    /* calculate execution time, store WCET if it occurred */
        clock_gettime(CLOCK_REALTIME, &sensorProcessFinish);
        delta_t(&sensorProcessFinish, getSystemStartTime(), &sensorProcessFinish);
        delta_t(&sensorProcessFinish, &sensorProcessStart, &sensorProcessDelta);
        syslog(LOG_NOTICE, "\tsensorProcess service end: %.02fms | ET: %.02fms\n", timestamp(&sensorProcessFinish), timestamp(&sensorProcessDelta));
        if(updateWCET(&sensorProcessDelta, &sensorProcessWCET)) {
                syslog(LOG_NOTICE, "\tsensorProcess WCET Updated: %.02fms\n", timestamp(&sensorProcessWCET));
        }
    }
/* thread exit cleanup */
    pthread_mutex_destroy(&sensorDataMutex);
    pthread_cond_destroy(&sensorDataReady);
    printf("\t\tFinal sensorProcess WCET %.02fms\n", timestamp(&sensorProcessWCET));
    syslog(LOG_NOTICE, "\t\tFinal sensorProcess WCET %.02fms\n", timestamp(&sensorProcessWCET));
    pthread_exit(NULL);
}

/**
 *  @name   configSensor
 *  @brief  configure hardware for HC-SR04 sensor
 *          setup trigger timer to pulse the TRIG pin periodically to initiate a data read
 *          setup the ECHO level detection callback, i.e. the sensorRx service
 *          delay system start for the initial sensor reads
 * 
 *  @param  NONE
 *  @return completion status, -1 indicates gpio init error
*/
int configSensor(void) {
    int ret = 0;

    printf("Configuring TRIG Pin %d to Output Mode...", TRIG_PIN);
    ret = check_gpio_error(gpioSetMode(TRIG_PIN,  PI_OUTPUT), TRIG_PIN);
    if(ret) {
        return ret;
    }
    printf("Done!\n");

    printf("Configuring ECHO Pin %d to Input Mode...", ECHO_PIN);
    ret = check_gpio_error(gpioSetMode(ECHO_PIN,  PI_INPUT), ECHO_PIN);
    if(ret) {
        return ret;
    }
    printf("Done!\n");

/* turn off TRIG_PIN to start */
    gpioWrite(TRIG_PIN, PI_OFF);

/* setup timer to trigger the sonar sensor every 50ms */
    ret = gpioSetTimerFunc(TRIGGER_TIMER, TRIGGER_PERIOD, triggerSensor);
    if(ret) {
        printf("\nSensor Trigger Timer setup failed: %d", ret);
        return ret;
    }

/* edge-detection callback for capturing sensor data */    
    ret = gpioSetAlertFunc(ECHO_PIN, sensorRx);
    if(ret) {
        printf("\nECHO pin level detection setup failed: %d", ret);
        return ret;
    }

/* objectData semaphore for data processing / alarm sync */
    ret = sem_init(&objectDataSem, 0, 0);
    if(ret) {
        perror("objectDataSem");
        return ret;
    }
    
/* delay for first sensor data processing */
    gpioDelay(TRIGGER_PERIOD * AVG_READING_CNT);

    return 0;    
}
