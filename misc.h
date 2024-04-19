/**
 * 
 * 
 * 
*/

#ifndef MISC_H_
#define MISC_H_

/* first pass at priorities, might need to change? */
#define ALARM_PRIO          (1)
#define SENSOR_RX_PRIO      (2)
#define SENSOR_PROCESS_PRIO (3)
#define LIVE_STREAM_PRIO    (4)

void print_scheduler(void);

#endif
