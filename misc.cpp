/**
 * 
 * 
*/

#include "misc.h"
#include <sched.h>
#include <unistd.h>
#include <stdio.h>

void print_scheduler(void)
{
   int schedType;
   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("\nPthread Policy is SCHED_FIFO");
           break;
     case SCHED_OTHER:
           printf("\nPthread Policy is SCHED_OTHER");
       break;
     case SCHED_RR:
           printf("\nPthread Policy is SCHED_OTHER");
           break;
     default:
       printf("\nPthread Policy is UNKNOWN");
   }
}
