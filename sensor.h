/**
 * 
 * 
*/

#ifndef SENSOR_H_
#define SENSOR_H_

typedef struct {
    double echo_time;
    double prev_echo_time;
    double range;
    double prev_range;
} ObjectData_t;

int configHCSR04(void);
void *sensorProcess_func(void *threadp);
int check_gpio_error(int ret, int pin);

int lockObjectData(void);
int unlockObjectData(void);
int lockSensorData(void);
int unlockSensorData(void);

ObjectData_t *getObjectData(void);

#endif
