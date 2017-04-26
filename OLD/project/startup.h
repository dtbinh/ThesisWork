#ifndef _STARTUP_
#define _STARTUP_

#define _BSD_SOURCE // for usleep();

typedef struct _structPipe{
	int parent[2];
	int child[2];
} structPipe;

typedef struct _pipeArray{
	structPipe *pipe1;
	structPipe *pipe2;
}pipeArray;


#endif
