/**
 *  @file       sensor.h
 *  @author     Mark Sherman
 *  @date       4/28/2024
 * 
 *  @brief      sensor config and data processing functions / data structure
 */

#ifndef SENSOR_H_
#define SENSOR_H_

/* raw sensor data struct */
typedef struct sensorData
{
    double echoTime;
    double prevReadTime;
    double readTime;
}sensorData_t;

/* processed data structure */
typedef struct objectData
{
    double prevRange_cm;
    double range_cm;
    double velocity_cmPerS;
    double velocity_kmPerHr;
    double timeToCollision;
}objectData_t;

int configSensor(void);
void *sensorProcess_func(void *threadp);

objectData_t *getObjectData(void);

void lockObjectData(void);
void unlockObjectData(void);
void destroyObjectDataSem(void);

void stopSensor(void);

#endif
