/**
 * 
 * 
*/

#ifndef SENSOR_H_
#define SENSOR_H_

typedef struct {
    double echoTime;
    double prevReadTime;
    double readTime;
} sensorData_t;

typedef struct {
    double prevRange_cm;
    double range_cm;
    double velocity;
} objectData_t;

int configHCSR04(void);
void *sensorProcess_func(void *threadp);
void *sensorRx_func(void *threadp);
int check_gpio_error(int ret, int pin);

int lockObjectData(void);
int unlockObjectData(void);
objectData_t *getObjectData(void);

#endif
