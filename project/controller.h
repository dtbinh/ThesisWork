#ifndef _CONTROLLER_
#define _CONTROLLER_

#define _BSD_SOURCE // for usleep();

extern void startController(void*, void*);
extern void *threadPipeSensorToController(void*);
extern void *threadPipeCommToController(void*);
extern void *intController(void*);
#endif
