#ifndef _STARTUP_
#define _STARTUP_

#define _BSD_SOURCE // for usleep();


typedef struct _structPipe{
	int parent[2];
	int child[2];
} structPipe;


#endif
