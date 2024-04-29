/**
 *  @file       misc.cpp
 *  @author     Mark Sherman
 *  @date       4/28/2024
 * 
 *  @brief      misc functions and definitons
 */

#include "misc.h"
#include <sched.h>
#include <unistd.h>
#include <stdio.h>
#include <pigpio.h>

/* conversion values for timestamp calc */
constexpr int NSEC_PER_SEC  = 1000000000;
constexpr int NSEC_PER_MSEC = 1000000;
constexpr int MSEC_PER_SEC  = 1000;

static timespec systemStart = {0, 0};

/**
 *  @name   getSystemStartTime
 *  @brief  getter for static system start time
 * 
 *  @param  NONE
 *  @return pointer to static system start time 
*/
struct timespec *getSystemStartTime(void) {
    return &systemStart;
}

/**
 *  @name   setSystemStartTime
 *  @brief  set system start time for timestamp reference
 * 
 *  @param  NONE
 *  @return VOID
*/
void setSystemStartTime(void) {
    clock_gettime(CLOCK_REALTIME, &systemStart);
}

/**
 *  @name   updateWCET
 *  @brief  store latest execution time if it is longer than the current stored WCET
 * 
 *  @param  delta   current execution time
 *  @param  WCET    current WCET
 * 
 *  @return TRUE if updated, FALSE otherwise    
*/
bool updateWCET(struct timespec *delta, struct timespec *WCET) {
    if(timestamp(delta) > timestamp(WCET)) {
      WCET->tv_sec    = delta->tv_sec;
      WCET->tv_nsec   = delta->tv_nsec;
      return true;
    }
    return false;
}

/**
 *  @name   check_gpio_error
 *  @brief  error handling for gpio pin initialization
 *  
 *  @param  ret     return status from the attempted gpio init
 *  @param  pin     pin being initialized
 * 
 *  @return -1 on error indicating init failure
*/
int check_gpio_error(int ret, int pin) {
    if (ret != 0) {
        switch (ret) {
            case PI_BAD_GPIO:
                printf("GPIO %d is bad.\n", pin);
                return -1;
                break;
            case PI_BAD_MODE:
                printf("GPIO %d bad mode.\n", pin);
                return -1; 
                break;
            default:
                printf("GPIO %d unexpected result, error %d.\n", pin, ret);
                return -1;
                break;
        }
    }
    else {
        return 0;
    }
}

/**
 *  @name   print_scheduler
 *  @brief  read and print the current system scheduling configuration
 * 
 *  @param  NONE
 *  @return VOID
*/
void print_scheduler(void)
{
   int schedType;
   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
           printf("Pthread Policy is SCHED_OTHER\n");
           break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }
}

/**
 *  @name   timestampt
 *  @brief  calculates the time in ms from a timespec object
 * 
 *  @param  duration  pointer to a timespec object holding a duration
 *  @return calculated timestamp in ms
*/
double timestamp(struct timespec *duration) {
    return (double)((double)duration->tv_sec * MSEC_PER_SEC) + (double)((double)duration->tv_nsec / NSEC_PER_MSEC);
}

/**
 *  @name   delta_t
 *  @brief  calculate the time difference between two timespec objects
 * 
 *  @param  stop      timespec object with stop time
 *  @param  start     timespec object with start time
 *  @param  delta_t   destination timespec object to store the calculate duration
 * 
 *  @return VOID
*/
void delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
    }
  }
}
