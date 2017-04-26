// CONTROLLER CODE

#include "controller.h"
#include "startup.h"

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <errno.h>
#include <semaphore.h>
#include <math.h>
#include <arpa/inet.h>
#include <string.h>

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadPipeSensorToController(void*);
static void *threadPipeCommToController(void*);
static void *threadPipeControllerToComm(void*);
static void *threadController(void*);

// Static variables for threads
static float sensorData[6]={0,0,0,0,0,0};
static float constraintData[6]={0,0,0,0,0,0};

static float position[3];
static float angle[3];
static float constraints[6];
static float controller[9];
static int flagController=1;

static pthread_mutex_t mutexSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexConstraintData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexControllerData = PTHREAD_MUTEX_INITIALIZER;


/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startController(void *arg1, void *arg2){
	// Create threads
	pthread_t threadPipeSensToCtrl, threadPipeCommToCtrl, threadPipeCtrlToComm, threadCtrl;
	int res1, res2, res3, res4;
	
	res1=pthread_create(&threadPipeSensToCtrl, NULL, &threadPipeSensorToController, arg1);
	res2=pthread_create(&threadPipeCommToCtrl, NULL, &threadPipeCommToController, arg2);
	res3=pthread_create(&threadPipeCtrlToComm, NULL, &threadPipeControllerToComm, arg2);
	res4=pthread_create(&threadCtrl, NULL, &threadController, arg1);
	
	// If threads created successful, start them
	if (!res1) pthread_join( threadPipeSensToCtrl, NULL);
	if (!res2) pthread_join( threadPipeCommToCtrl, NULL);
	if (!res3) pthread_join( threadPipeCtrlToComm, NULL);
	if (!res4) pthread_join( threadCtrl, NULL);
}



/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Pipe Sensor to Controller read
void *threadPipeSensorToController(void *arg)
{
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float sensorDataBuffer[6]={0,0,0,0,0,0};
	
	// Loop forever reading/waiting for data
	while(1){
		// Read data from sensor process
		if(read(ptrPipe->child[0], sensorDataBuffer, sizeof(sensorDataBuffer)) == -1) printf("read error in Controller from Sensor\n");
		//else printf("Controller ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Put new data in to global variable in controller.c
		pthread_mutex_lock(&mutexSensorData);
		memcpy(sensorData, sensorDataBuffer, sizeof(sensorDataBuffer));
		pthread_mutex_unlock(&mutexSensorData);
	}
	return NULL;
}

// Thread - Pipe Communication to Controller read
void *threadPipeCommToController(void *arg)
{	
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float constraintDataBuffer[6] = {0,0,0,0,0,0};
	
	// Loop forever streaming data
	while(1){
		// Read data from communication process
		if(read(ptrPipe->child[0], constraintDataBuffer, sizeof(constraintDataBuffer)) == -1) printf("read error in controller from communication\n");
		//else printf("Controller ID: %d, Recieved Communication data: %f\n", (int)getpid(), constraintDataBuffer[0]);
		// Put new data in to global variable in controller.c
		pthread_mutex_lock(&mutexConstraintData);
		memcpy(constraintData, constraintDataBuffer, sizeof(constraintDataBuffer));
		pthread_mutex_unlock(&mutexConstraintData);
	}
	
	return NULL;
}

// Thread - Pipe Controller to Communication write
void *threadPipeControllerToComm(void *arg)
{	
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float controllerDataBuffer[9];
	
	// Loop forever streaming data
	while(1){
		// Check flag to see if new controller data available
		if (flagController==1){
			// Get new computed controller data
			pthread_mutex_lock(&mutexControllerData);
			memcpy(controllerDataBuffer, controller, sizeof(controller));
			pthread_mutex_unlock(&mutexControllerData);
			
			// Write data to communication process
			if (write(ptrPipe->parent[1], controllerDataBuffer, sizeof(controllerDataBuffer)) != sizeof(controllerDataBuffer)) printf("write error in controller to communication\n");
			//else printf("Controller ID: %d, Sent: %f to Communication\n", (int)getpid(), controllerDataBuffer[0]);
		}
		sleep(5);
	}

	return NULL;
}


// Thread - Controller algorithm
void *threadController(void *arg)
{
	structPipe *ptrPipe = arg;
	sleep(15);
	char input[10];
	float value[4];

	while(1){
		// Read in PWM value
		printf("Enter PWM value:\n");
		fgets(input, 10, stdin);
		value[0] = atof(input);
		printf("Value: %f\n", value[0]);
		
		for (int i=1;i<4;i++){
			value[i]=value[0];
		}
		
		// Write data to sensor process
		if (write(ptrPipe->parent[1], value, sizeof(value)) != sizeof(value)) printf("write error in controller to sensor\n");
	}
	
	return NULL;
}



/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/









	// Update sensor measurements and constraints
	/*pthread_mutex_lock(&mutexSensorData);
	memcpy(position, sensorData, 3);
	memcpy(angle, sensorData+3,3);
	pthread_mutex_unlock(&mutexSensorData);
	pthread_mutex_lock(&mutexConstraintData);
	memcpy(constraints, constraintData, sizeof(constraintData));
	pthread_mutex_unlock(&mutexConstraintData);
	*/
	
	// Compute control signal
	/*
	sleep(10);
	float controllerData[9] = {1,2,3,4,5,6,7,8,9};
	*/
	
	// 
	
	// Flag pipe write to communication process
	/*
	flagController=1;
	pthread_mutex_lock(&mutexControllerData);
	memcpy(controller, controllerData, sizeof(controllerData));
	pthread_mutex_unlock(&mutexControllerData);
	*/
	
