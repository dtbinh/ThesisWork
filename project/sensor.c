// SENSOR FUSION CODE

#include "sensor.h"
#include "startup.h"

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>


/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadPipeSensorToController (void*);
static void *threadPipeSensorToComm (void*);
static void *threadSensorFusionPosition (void*);
static void *threadSensorFusionAngles (void*);


// Static variables for threads
static float position[3]={1.0f,1.0f,1.0f};
static float angles[3]={2.0f, 2.0f, 2.0f};

static pthread_mutex_t mutexPositionData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexAngleData = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *arg1, void *arg2){
	// Create thread
	pthread_t threadPipeSensToCtrl, threadPipeSensToComm, threadAngles, threadPosition;
	int res1, res2, res3 ,res4;
	
	res1=pthread_create(&threadPipeSensToCtrl, NULL, &threadPipeSensorToController, arg1);
	res2=pthread_create(&threadPipeSensToComm, NULL, &threadPipeSensorToComm, arg2);
	res3=pthread_create(&threadAngles, NULL, &threadSensorFusionAngles, NULL);
	res4=pthread_create(&threadPosition, NULL, &threadSensorFusionPosition, NULL);	
	
	// If threads created successful, start them
	if (!res1) pthread_join( threadPipeSensToCtrl, NULL);
	if (!res2) pthread_join( threadPipeSensToComm, NULL);
	if (!res3) pthread_join( threadAngles, NULL);
	if (!res4) pthread_join( threadPosition, NULL);
}


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/


// Thread - Pipe Sensor to Controller process write
void *threadPipeSensorToController (void *arg){
	// Get pipe and define local variables
	structPipe *ptrPipeToCtrl = arg;
	float sensorDataBuffer[6];
	
	// Loop forever sending data to controller and communication processes
	while(1){
		// Get angle and position data from global variables in sensor.c
		pthread_mutex_lock(&mutexAngleData);
		memcpy(sensorDataBuffer, angles, sizeof(angles));
		pthread_mutex_unlock(&mutexAngleData);
		pthread_mutex_lock(&mutexPositionData);
		memcpy(sensorDataBuffer+sizeof(angles), position, sizeof(position));	
		pthread_mutex_unlock(&mutexPositionData);
		
		// Write to Controller process
		if (write(ptrPipeToCtrl->child[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor to Controller\n");
		else printf("Sensor ID: %d, Sent: %f to Controller\n", (int)getpid(), sensorDataBuffer[0]);
		sleep(5);
	}
	
	return NULL;
}


// Thread - Pipe Sensor to Communication process write
void *threadPipeSensorToComm (void *arg){
	// Get pipe and define local variables
	structPipe *ptrPipeToComm = arg;
	float sensorDataBuffer[6];
	
	// Loop forever sending data to controller and communication processes
	while(1){
		// Get angle and position data from global variables in sensor.c
		pthread_mutex_lock(&mutexAngleData);
		memcpy(sensorDataBuffer, angles, sizeof(angles));
		pthread_mutex_unlock(&mutexAngleData);
		pthread_mutex_lock(&mutexPositionData);
		memcpy(sensorDataBuffer+sizeof(angles), position, sizeof(position));	
		pthread_mutex_unlock(&mutexPositionData);
		
		// Write to Communication process
		if (write(ptrPipeToComm->parent[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error to communicaiont in sensors\n");
		else printf("Sensor ID: %d, Sent: %f to Communication\n", (int)getpid(), sensorDataBuffer[0]);
		sleep(5);
	}
	
	return NULL;
}


// Thread - Sensor Fusion Position
void *threadSensorFusionPosition (void *arg){
	// Loop for ever
	while(1){				 
		// Write data to local variables using mutex
		pthread_mutex_lock(&mutexPositionData);
		usleep(100);
		float newPosition[3]={1,2,3};
		memcpy(position, newPosition, sizeof(newPosition));	
		pthread_mutex_unlock(&mutexPositionData);	
	}
	return NULL;
}


// Thread - Sensor Fusion Angles
void *threadSensorFusionAngles (void *arg){
	// Loop for ever
	while(1){				 
		// Write data to local variables using mutex
		pthread_mutex_lock(&mutexAngleData);
		usleep(100);
		float newAngles[3]={1,2,3};
		memcpy(angles, newAngles, sizeof(newAngles));	
		pthread_mutex_unlock(&mutexAngleData);	
	}
	return NULL;
}




/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/




