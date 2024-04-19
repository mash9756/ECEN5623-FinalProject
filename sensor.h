/**
 * 
 * 
*/

#ifndef SENSOR_H_
#define SENSOR_H_

int configHCSR04(void);
void *sensorProcess_func(void *threadp);
int check_gpio_error(int ret, int pin);

int lockSensorSem(void);
int unlockSensorSem(void);

#endif
