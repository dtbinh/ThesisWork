#ifndef _SENSOR_
#define _SENSOR_

#define PIPE_BUFFER_SIZE 1
#define _BSD_SOURCE // for usleep();

typedef struct _structAcc structAcc;
typedef struct _structGyr structGyr;
typedef struct _structMag structMag;
typedef struct _structBmp structBmp;

extern void startSensors(void*, void*);
void *threadSensorFusionPosition (void*);
void *threadSensorFusionAngles (void*);
void *threadPipeSensorToControllerAndComm (void*);
//static void *threadPipeSensorToComm (void*);
#endif
