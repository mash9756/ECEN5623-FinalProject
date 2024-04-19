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

void set_liveStream_sched(void) {
    pthread_attr_init(&liveStream_attr);
    pthread_attr_setinheritsched(&liveStream_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&liveStream_attr, SCHED_FIFO);

    liveStream_param.sched_priority = rt_max_prio - LIVE_STREAM_PRIO;
    pthread_attr_setschedparam(&liveStream_attr, &liveStream_param);
}

void set_sensorRx_sched(void) {
    pthread_attr_init(&sensorRx_attr);
    pthread_attr_setinheritsched(&sensorRx_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&sensorRx_attr, SCHED_FIFO);

    sensorRx_param.sched_priority = rt_max_prio - SENSOR_RX_PRIO;
    pthread_attr_setschedparam(&sensorRx_attr, &sensorRx_param);
}

void set_sensorProcess_sched(void) {
    pthread_attr_init(&sensorProcess_attr);
    pthread_attr_setinheritsched(&sensorProcess_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&sensorProcess_attr, SCHED_FIFO);

    sensorProcess_param.sched_priority = rt_max_prio - SENSOR_PROCESS_PRIO;
    pthread_attr_setschedparam(&sensorProcess_attr, &sensorProcess_param);
}

void set_alarm_sched(void) {
    pthread_attr_init(&alarm_attr);
    pthread_attr_setinheritsched(&alarm_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&alarm_attr, SCHED_FIFO);

    alarm_param.sched_priority = rt_max_prio - ALARM_PRIO;
    pthread_attr_setschedparam(&alarm_attr, &alarm_param);
}

int main() {
/* ---------------------- init pigpio library ---------------------- */
    printf("Initializing pigpio... ");
    int ret = gpioInitialise();
    if (ret == PI_INIT_FAILED) {
        printf("GPIO init failed, error %d\n", ret);
        return -1;    
    }
    printf("Done!\n");
/* ------------------------------------------------------------------ */

/* ---------------------- configure scheduling ---------------------- */
    print_scheduler();
    set_alarm_sched();
    set_sensorRx_sched();
    set_sensorProcess_sched();
    set_liveStream_sched();
/** TODO: configure main scheduler to SCHED_FIFO */
    print_scheduler();
/* ------------------------------------------------------------------ */

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
    pthread_create(&liveStream_thread,      &liveStream_attr,       liveStream_func,     NULL);
    pthread_create(&alarm_thread,           &alarm_attr,            alarm_func,          NULL);
    pthread_create(&sensorProcess_thread,   &sensorProcess_attr,    sensorProcess_func,  NULL);
    //pthread_create(&sensorRx_thread,        &sensorRx_attr,         sensorRx_func,       NULL);

    while(1){
        sleep(1);
    }

    gpioTerminate();
}
