/**
 * 
 * 
 * 
*/

#ifndef MISC_H_
#define MISC_H_

#include <time.h>

/* first pass at priorities, might need to change? */
#define ALARM_PRIO          (1)
#define SENSOR_RX_PRIO      (2)
#define SENSOR_PROCESS_PRIO (3)
#define LIVE_STREAM_PRIO    (4)

void print_scheduler(void);
double timestamp(struct timespec *duration);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif
