#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

 
 
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>

#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC 1E9 /* The number of nsecs per sec. */

void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}

int main(int argc, char* argv[])
{
        struct timespec t, t_start, t_stop;
        struct sched_param param;
        int interval = 50000000; /* 0.05s*/
        double accum;
        //int interval_s = 2;

        /* Declare ourself as a real time task */

        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();

        clock_gettime(CLOCK_MONOTONIC ,&t);
		
		
        /* start after one second */
        t.tv_sec++;

        while(1) {
			// Time it
			clock_gettime(CLOCK_MONOTONIC, &t_start);
			
			/* wait until next shot */			
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
			

			/* do the stuff */
			//printf("PREEMT_RT timer of %f seconds\n", (float)interval);

			/* calculate next shot */
			t.tv_nsec += interval;
			//t.tv_sec += interval_s;

			while (t.tv_nsec >= NSEC_PER_SEC) {
				   t.tv_nsec -= NSEC_PER_SEC;
					t.tv_sec++;
			}
			
			clock_gettime(CLOCK_MONOTONIC, &t_stop);
			accum=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
			printf("%lf\n",accum);
   }
}
