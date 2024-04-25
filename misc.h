/**
 * 
 * 
 * 
*/

#ifndef MISC_H_
#define MISC_H_

#include <time.h>

/* first pass at priorities, might need to change? */
/* camera refresh rate requirement = 30Hz, Deadline 33ms, RM gives shortest highest prio */
#define ALARM_PRIO          (2)
#define SENSOR_RX_PRIO      (3)
#define SENSOR_PROCESS_PRIO (2)
#define LIVE_STREAM_PRIO    (1)

#define ALARM_TIMER         (0)
#define TRIGGER_TIMER       (1)
#define ECHO_TIMER          (2)

/* cores for given services, we might just run cam on 0 and all other on 1 */
#define LIVESTREAM_CORE_ID  (0)
#define SENSOR_CORE_ID      (1)
#define ALARM_CORE_ID       (2)

void print_scheduler(void);
double timestamp(struct timespec *duration);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif
