/**
 * 
 * 
 */

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdbool.h>
#include <syslog.h>

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include <pigpio.h>

#include "capture.h"
#include "sensor.h"
#include "alarm.h"
#include "misc.h"

// typedef struct
// {
//   int threadIdx;
// } threadParams_t;

/* webcam stream thread declarations and sched attributes */
pthread_t liveStream_thread;
//threadParams_t liveStream_thread_params;
pthread_attr_t liveStream_attr;
struct sched_param liveStream_param;

/* HC-SR04 Sensor Data Receive thread declarations and sched attributes */
pthread_t sensorRx_thread;
//threadParams_t sensorRx_thread_params;
pthread_attr_t sensorRx_attr;
struct sched_param sensorRx_param;

/* HC-SR04 Sensor Data Process thread declarations and sched attributes */
pthread_t sensorProcess_thread;
//threadParams_t sensorProcess_thread_params;
pthread_attr_t sensorProcess_attr;
struct sched_param sensorProcess_param;

/* Object Detection Alarm thread declarations and sched attributes */
pthread_t alarm_thread;
//threadParams_t alarm_thread_params;
pthread_attr_t alarm_attr;
struct sched_param alarm_param;

int main() {
    /* init pigpio library */
    printf("Initializing pigpio... ");
    int ret = gpioInitialise();
    if (ret == PI_INIT_FAILED) {
        printf("GPIO init failed, error %d\n", ret);
        return -1;    
    }
    printf("Done!\n");

    print_scheduler();

/* setup HC-SR04 Ultrasonic Sensor */
    if(configHCSR04()) {
        printf("HC-SR04 Config failed!\n");
        return -1;
    };
/* setup alarm hardware */
    if(configAlarm()) {
        printf("Alarm Config failed!\n");
        return -1;
    }

/* create threads for each service */
    pthread_create(&liveStream_thread,  &liveStream_attr,   liveStream_func,     NULL);
    pthread_create(&alarm_thread,       &alarm_attr,        alarm_func,          NULL);
    //pthread_create(&sensorRx_thread,        &sensorRx_attr,         sensorRx_func,       NULL);
    //pthread_create(&sensorProcess_thread,   &sensorProcess_attr,    sensorProcess_func,  NULL);

    while(1){
        sleep(1);
    }

    gpioTerminate();
}
