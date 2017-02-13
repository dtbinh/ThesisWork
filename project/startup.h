#ifndef _STARTUP_
#define _STARTUP_

typedef struct _structPipe{
	int parent[2];
	int child[2];
} structPipe;

typedef enum _initState{ STARTUP, WAITING, READY }initState; 

#endif
