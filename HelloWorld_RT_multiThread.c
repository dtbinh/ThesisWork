#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

 
 
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <pthread.h>

#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */

static void *threadTask1(void*);
static void *threadTask2(void*);

void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}

int main(int argc, char* argv[])
{
		pthread_t threadT1, threadT2;
		int res1, res2;
		
		res1=pthread_create(&threadT1, NULL, &threadTask1, NULL);
		res2=pthread_create(&threadT2, NULL, &threadTask2, NULL);

		
		// If threads created successful, start them
		if (!res1) pthread_join( threadT1, NULL);
		if (!res2) pthread_join( threadT2, NULL);
		
		return 1;
}

void *threadTask1(void *arg){
		struct timespec t;
        struct sched_param param;
        int interval_s = 2;
	
        /* Declare ourself as a real time task */
        param.sched_priority = 39;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();

        clock_gettime(CLOCK_MONOTONIC ,&t);
        /* start after one second */
        t.tv_sec++;

        while(1) {
                /* wait until next shot */
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

                /* do the stuff */
                clock_gettime(CLOCK_MONOTONIC ,&t);
                printf("PREEMT_RT timer of Task1: %f seconds\n", (float)t.tv_nsec);

                /* calculate next shot */
                //t.tv_nsec += interval;
                t.tv_sec += interval_s;

                while (t.tv_nsec >= NSEC_PER_SEC) {
                       t.tv_nsec -= NSEC_PER_SEC;
                        t.tv_sec++;
                }
		}
}


void *threadTask2(void *arg){
		struct timespec t;
        struct sched_param param;
        int interval_s = 1;
	
        /* Declare ourself as a real time task */
        param.sched_priority = 40;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();

        clock_gettime(CLOCK_MONOTONIC ,&t);
        /* start after one second */
        t.tv_sec++;

        while(1) {
                /* wait until next shot */
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

                /* do the stuff */
                printf("PREEMT_RT timer of Task2: %f seconds\n", (float)interval_s);

                /* calculate next shot */
                //t.tv_nsec += interval;
                t.tv_sec += interval_s;

                while (t.tv_nsec >= NSEC_PER_SEC) {
                       t.tv_nsec -= NSEC_PER_SEC;
                        t.tv_sec++;
                }
		}
}
