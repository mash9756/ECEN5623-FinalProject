/**
 *  @file       alarm.h
 *  @author     Mark Sherman
 *  @date       4/28/2024
 */

#ifndef ALARM_H_
#define ALARM_H_

int configAlarm(void);
void *alarm_func(void *threadp);
void stopAlarm(void);

#endif
