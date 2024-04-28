/**
 * 
 * 
*/

#ifndef SENSOR_H_
#define SENSOR_H_

constexpr double MAX_RANGE_CM = 400;
constexpr double MIN_RANGE_CM = 1;
/* m/s to km/hr */
constexpr double VELOCITY_SCALE = 3.6; 

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
