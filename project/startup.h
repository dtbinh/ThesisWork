#ifndef _STARTUP_
#define _STARTUP_

#define _BSD_SOURCE // for usleep();
#define _POSIX_C_SOURCE 199309L // for time.h functionality
#define _GNU_SOURCE // for realtime nanospeep()

// PREEMPT_RT
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)

#define tsController 100000000
#define tsWatchdog 100000000
#define tsUDP 100000000
#define tsSensors 100000000

typedef struct _structPipe{
	int parent[2];
	int child[2];
} structPipe;

typedef struct _pipeArray{
	structPipe *pipe1;
	structPipe *pipe2;
}pipeArray;


#endif
