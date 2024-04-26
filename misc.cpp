/**
 * 
 * 
*/

#include "misc.h"
#include <sched.h>
#include <unistd.h>
#include <stdio.h>

#define NSEC_PER_SEC            (1000000000)
#define NSEC_PER_MSEC           (1000000)
#define NSEC_PER_MICROSEC       (1000)
#define MSEC_PER_SEC            (1000)

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

/* calculates a duration in ms with added precision */
double timestamp(struct timespec *duration) {
    return (double)(duration->tv_sec) + (double)((double)duration->tv_nsec / NSEC_PER_MSEC);
}

int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
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

  return(1);
}

/* simple sleep function with ms parameter for ease of use */
int sleep_ms(int ms) {
    int ret = 0;
    struct timespec req_time;
    struct timespec rem_time;

    int sec   = ms / MSEC_PER_SEC;
    int nsec  = (ms % MSEC_PER_SEC) * NSEC_PER_MSEC;

    req_time.tv_sec   = sec;
    req_time.tv_nsec  = nsec;

    if(ret = nanosleep(&req_time , &rem_time) < 0) {
        printf("Nano sleep system call failed \n");
        return -1;
    }
    return ret;
}