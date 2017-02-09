#ifndef _SENSOR_
#define _SENSOR_

typedef struct _structAcc structAcc;
typedef struct _structGyr structGyr;
typedef struct _structMag structMag;
typedef struct _structBmp structBmp;

typedef struct _structPipeSensor{
	int parent[2];
	int child[2];
} structPipeSensor;

extern int startProcessCommunication(structPipeSensor*);
extern int startSensorFusionAngles(void);
extern int startSensorFusionPosition(void);

//const int dummy=0;

/*
void readIMU(float*, float*, float*, float*);
void readBeacon(int*);
void getAngles(void);
void getPosition(void);
*/

#endif
