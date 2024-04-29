/**
 *  @file       misc.h
 *  @author     Mark Sherman
 *  @date       4/28/2024
 * 
 *  @brief      misc functions and definitons
 */

#ifndef MISC_H_
#define MISC_H_

#include <time.h>
#include <stdlib.h>

/* service priorities */
#define ALARM_PRIO          (2)
#define SENSOR_PROCESS_PRIO (2)
#define LIVE_STREAM_PRIO    (1)

/* Timer definitions */
#define ALARM_TIMER         (0)
#define TRIGGER_TIMER       (1)
#define LIVESTREAM_TIMER    (2)

/* cores for given services, we might just run cam on 0 and all other on 1 */
#define LIVESTREAM_CORE_ID  (0)
#define SENSOR_CORE_ID      (1)

/* exit delay */
#define EXIT_DELAY          (1000000)

int check_gpio_error(int ret, int pin);
void print_scheduler(void);

struct timespec *getSystemStartTime(void);
void setSystemStartTime(void);
bool updateWCET(struct timespec *delta, struct timespec *WCET);

double timestamp(struct timespec *duration);
void delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif
