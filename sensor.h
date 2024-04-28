/**
 * 
 * 
*/

#ifndef SENSOR_H_
#define SENSOR_H_

typedef struct sensorData
{
    ssize_t readCnt;
    double echoTime;
    double prevReadTime;
    double readTime;
}sensorData_t;

typedef struct objectData
{
    double prevRange_cm;
    double range_cm;
    double velocity;
    double timeToCollision;
}objectData_t;

int check_gpio_error(int ret, int pin);

int configHCSR04(void);
void *sensorProcess_func(void *threadp);

objectData_t *getObjectData(void);

void lockObjectData(void);
void unlockObjectData(void);
void destroyObjectDataSem(void);

void stopSensor(void);

#endif
