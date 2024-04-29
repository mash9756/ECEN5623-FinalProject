/**
 *  @file       main.cpp
 *  @author     Mark Sherman
 *  @date       4/28/2024
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
#include <signal.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include <pigpio.h>

#include "capture.h"
#include "sensor.h"
#include "alarm.h"
#include "misc.h"

/* webcam stream thread declarations and sched attributes */
pthread_t liveStream_thread;
pthread_attr_t liveStream_attr;
struct sched_param liveStream_param;

/* HC-SR04 Sensor Data Process thread declarations and sched attributes */
pthread_t sensorProcess_thread;
pthread_attr_t sensorProcess_attr;
struct sched_param sensorProcess_param;

/* Object Detection Alarm thread declarations and sched attributes */
pthread_t alarm_thread;
pthread_attr_t alarm_attr;
struct sched_param alarm_param;

/* Main thread declarations and sched attributes */
pthread_attr_t main_attr;
struct sched_param main_param;

/* init SCHED_FIFO priorities */
int rt_max_prio = sched_get_priority_max(SCHED_FIFO);
int rt_min_prio = sched_get_priority_min(SCHED_FIFO);

/* thread exit flag, set by SIGINT handler */
static bool stopMainFlag = false;

/**
 *  @name   intHandler
 *  @brief  interrupt handler used to catch SIGINT and gracefully shutdown the program
 * 
 *  @param  NONE
 *  @return VOID
*/
void intHandler(int dummy) {
    printf("\nStopping...\n");
    stopLiveStream();
    stopAlarm();
    stopSensor();
    stopMainFlag = true;
}

/**
 *  @name   set_liveStream_sched
 *  @brief  configure thread parameters for the camera live stream
 * 
 *  @param  NONE
 *  @return VOID
*/
void set_liveStream_sched(void) {
    cpu_set_t threadcpu;

    CPU_ZERO(&threadcpu);
    CPU_SET(LIVESTREAM_CORE_ID, &threadcpu);
    printf("liveStream thread set to run on core %d\n", LIVESTREAM_CORE_ID);

    pthread_attr_init(&liveStream_attr);
    pthread_attr_setinheritsched(&liveStream_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&liveStream_attr, SCHED_FIFO);
    pthread_attr_setaffinity_np(&liveStream_attr, sizeof(cpu_set_t), &threadcpu);

    liveStream_param.sched_priority = rt_max_prio - LIVE_STREAM_PRIO;
    pthread_attr_setschedparam(&liveStream_attr, &liveStream_param);
}

/**
 *  @name   set_sensorProcess_sched
 *  @brief  configure thread parameters for sensor data processing
 * 
 *  @param  NONE
 *  @return VOID
*/
void set_sensorProcess_sched(void) {
    cpu_set_t threadcpu;

    CPU_ZERO(&threadcpu);
    CPU_SET(SENSOR_CORE_ID, &threadcpu);
    printf("sensorProcess thread set to run on core %d\n", SENSOR_CORE_ID);

    pthread_attr_init(&sensorProcess_attr);
    pthread_attr_setinheritsched(&sensorProcess_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&sensorProcess_attr, SCHED_FIFO);
    pthread_attr_setaffinity_np(&sensorProcess_attr, sizeof(cpu_set_t), &threadcpu);

    sensorProcess_param.sched_priority = rt_max_prio - SENSOR_PROCESS_PRIO;
    pthread_attr_setschedparam(&sensorProcess_attr, &sensorProcess_param);
}

/**
 *  @name   set_alarm_sched
 *  @brief  configure thread parameters for the object detection alarm
 * 
 *  @param  NONE
 *  @return VOID
*/
void set_alarm_sched(void) {
    cpu_set_t threadcpu;

    CPU_ZERO(&threadcpu);
    CPU_SET(SENSOR_CORE_ID, &threadcpu);
    printf("alarm thread set to run on core %d\n", SENSOR_CORE_ID);

    pthread_attr_init(&alarm_attr);
    pthread_attr_setinheritsched(&alarm_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&alarm_attr, SCHED_FIFO);
    pthread_attr_setaffinity_np(&alarm_attr, sizeof(cpu_set_t), &threadcpu);

    alarm_param.sched_priority = rt_max_prio - ALARM_PRIO;
    pthread_attr_setschedparam(&alarm_attr, &alarm_param);
}

/**
 *  @name   set_main_sched
 *  @brief  configure thread parameters for the main thread
 *          sets up use of SCHED_FIFO for created threads
 * 
 *  @param  NONE
 *  @return VOID
*/
void set_main_sched(void) {
    int rc          = 0;
    int scope       = 0;
    pid_t main_pid  = getpid();

    rc = sched_getparam(main_pid, &main_param);
    if(rc < 0) {
        perror("sched getparam");
    }
    main_param.sched_priority = rt_max_prio;
    rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) {
        perror("setscheduler");
    }

    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM) {
        printf("PTHREAD SCOPE SYSTEM\n");
    }
    else if (scope == PTHREAD_SCOPE_PROCESS) {
        printf("PTHREAD SCOPE PROCESS\n");
    }
    else {
        printf("PTHREAD SCOPE UNKNOWN\n");
    }

    pthread_attr_setschedparam(&main_attr, &main_param);
}

/**
 *  @name   main
 *  @brief  main program execution
 *          initalizes hardware and threads, then waits for completion
 * 
 *  @param  NONE
 *  @return execution status, -1 on any error
*/
int main() {
/* init pigpio library */
    printf("Initializing pigpio... ");
    int ret = gpioInitialise();
    if (ret == PI_INIT_FAILED) {
        printf("GPIO init failed, error %d\n", ret);
        return -1;    
    }
    printf("Done!\n");

/* configure CTRL-C Handle to stop program */
    struct sigaction act;
    act.sa_handler = intHandler;
    sigaction(SIGINT, &act, NULL);

/* configure scheduling */
    print_scheduler();
    set_main_sched();
    set_alarm_sched();
    set_sensorProcess_sched();
    set_liveStream_sched();
    print_scheduler();

/* setup HC-SR04 Ultrasonic Sensor */
    if(configSensor()) {
        printf("HC-SR04 Config failed!\n");
        return -1;
    };

/* setup alarm hardware */
    if(configAlarm()) {
        printf("Alarm Config failed!\n");
        return -1;
    }

/* setup liveStream release timing */
    if(configLiveStream()) {
        printf("liveStream Config failed!\n");
        return -1;
    }

/* get system start time for relative timestamp calculations */
    setSystemStartTime();

/* create threads for each service */
    pthread_create(&liveStream_thread,      &liveStream_attr,       liveStream_func,     NULL);
    pthread_create(&sensorProcess_thread,   &sensorProcess_attr,    sensorProcess_func,  NULL);
    pthread_create(&alarm_thread,           &alarm_attr,            alarm_func,          NULL);

/* allow threads to run */
    while(!stopMainFlag){
        sleep(1);
    }

/* wait for all threads to complete */
    printf("\nWaiting for threads...");
    pthread_join(liveStream_thread, NULL);  
    pthread_join(alarm_thread, NULL);
    pthread_join(sensorProcess_thread, NULL);
    printf("Done!\n");
    
/* deinit gpio on program exit */
    gpioTerminate();

    return 0;
}
