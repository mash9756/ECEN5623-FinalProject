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

static double sensorData    = 0;
static double range         = 0;

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

sem_t dataSem;
sem_t rangeSem;

int lockRangeSem(void) {
    return sem_wait(&rangeSem);
}

int unlockRangeSem(void) {
    return sem_post(&rangeSem);
}

double *getRange(void) {
    return &range;
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

/* tick is microseconds since boot, wraps every ~72 mins */
void echo(int pin, int level, uint32_t time_us) {
    static double start_us = 0;
    static double first_us = 0;
    double diff_us = 0;
    double range_cm = 0;

    if(!first_us) {
        first_us = (double)time_us;
    }
/* get start tick when high detected */
    if(level == PI_ON) {
        start_us = (double)time_us;
    }
/* get pulse travel time from start tick and current tick when level drops */
    else if(level == PI_OFF) {
        sem_wait(&dataSem);
        sensorData = (double)time_us - start_us;
        sem_post(&dataSem);
    }
}

/**
 *  distance = rate * time, rate = speed of sound, time is in microseconds
 *  convert m/s to cm/us to get distance in cm
 *  divide by 2 as the sonar pulse is travelling to the object and back
*/
void *sensorProcess_func(void *threadp) {
    double prev_time    = 0;
    double time_us      = 0;
    double prev_range   = 0;
    double range_cm     = 0;

    while(1) {
        prev_time   = time_us;
        prev_range  = range_cm;

    /* lock sensor data and range while we update */
        sem_wait(&dataSem);
        lockRangeSem();
        time_us     = sensorData;
        range_cm    = ((time_us * SPEED_OF_SOUND * (M_TO_CM / SEC_TO_US)) / 2);
        range       = range_cm;
        sem_post(&dataSem);
        unlockRangeSem();
    }
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
    gpioSetTimerFunc(0, 50, trigger);
/* setup callback when level change detected on ECHO pin */
    gpioSetAlertFunc(ECHO_PIN, echo);

    return 0;    
}
