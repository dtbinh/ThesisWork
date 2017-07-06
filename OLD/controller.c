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

// PREEMPT_RT
//#include <time.h>
#include <sched.h>
#include <sys/mman.h>

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadUpdateMeasurements(void*);
static void *threadUpdateConstraints(void*);
static void *threadController(void*);
static void *threadControllerWatchdog(void*);

void stack_prefault(void);

// Static variables for threads
static float globalSensorData[6]={0,0,0,0,0,0};
static float globalConstraintsData[6]={0,0,0,0,0,0};
static int globalWatchdog=0;

// Controller variables
static int Ts=1; // 0.5s


static pthread_mutex_t mutexSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexConstraintsData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexWatchdog = PTHREAD_MUTEX_INITIALIZER;


/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/


// Function to start the sensor process threads
void startController(void *arg1, void *arg2){
	// Create pipe array
	pipeArray pipeArrayStruct = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create threads
	pthread_t threadUpdateMeas, threadUpdateConstr, threadCtrl, threadCtrlWD;
	int res1, res2, res3, res4;
	
	//res1=pthread_create(&threadUpdateMeas, NULL, &threadUpdateMeasurements, arg1);
	//res2=pthread_create(&threadUpdateConstr, NULL, &threadUpdateConstraints, arg2);
	res3=pthread_create(&threadCtrl, NULL, &threadController, &pipeArrayStruct);
	res4=pthread_create(&threadCtrlWD, NULL, &threadControllerWatchdog, NULL);
	
	// If threads created successful, start them
	//if (!res1) pthread_join( threadUpdateMeas, NULL);
	//if (!res2) pthread_join( threadUpdateConstr, NULL);
	if (!res3) pthread_join( threadCtrl, NULL);
	if (!res4) pthread_join( threadCtrlWD, NULL);
}


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Update constriants from other drones (pipe from communication process).
// Includes any updates on setpoints or MPC settings from computer
void *threadUpdateConstraints(void *arg)
{	
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float constraintsDataBuffer[6] = {0,0,0,0,0,0};
	
	// Loop forever streaming data
	while(1){
		// Read data from communication process
		if(read(ptrPipe->child[0], constraintsDataBuffer, sizeof(constraintsDataBuffer)) == -1) printf("read error in controller from communication\n");
		//else printf("Controller ID: %d, Recieved Communication data: %f\n", (int)getpid(), constraintDataBuffer[0]);
		
		// Put new constraints data in to global data in controller.c such that controller thread can access and use it.
		pthread_mutex_lock(&mutexConstraintsData);
			memcpy(globalConstraintsData, constraintsDataBuffer, sizeof(constraintsDataBuffer));
		pthread_mutex_unlock(&mutexConstraintsData);
	}
	
	return NULL;
}


// Thread - Update local variables with any new sensor measurements (pipe from sensor process)
void *threadUpdateMeasurements(void *arg)
{
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	
	float sensorDataBuffer[6] = {0,0,0,0,0,0};
	
	while(1){
		// Read data from sensor process. Data should always be available for controller.
		if(read(ptrPipe->child[0], sensorDataBuffer, sizeof(sensorDataBuffer)) == -1) printf("read error in Controller from Sensor\n");
		//else printf("Controller ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0])
		
		// Put new sensor data in to global data in controller.c such that controller thread can access and use it.
		pthread_mutex_lock(&mutexSensorData);
			memcpy(globalSensorData, sensorDataBuffer, sizeof(sensorDataBuffer));
		pthread_mutex_unlock(&mutexSensorData);
	}
	
	return NULL;
}


// Thread - Controller algorithm running Fast MPC and setting motor signals (with pipe to sensor (PWM) and communication process)
void *threadController(void *arg)
{
	// Get pipe array and define local variables
	//pipeArray *pipeArrayStruct = arg;
	//structPipe *ptrPipe1 = pipeArrayStruct->pipe1;
	//structPipe *ptrPipe2 = pipeArrayStruct->pipe2;
	
	// Get pipe and define local variables
	struct timespec t;
	struct sched_param param;

	// Declare ourself as a real time task
	param.sched_priority = 39;
	if(sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1){
		perror("sched_setscheduler failed");
	}
	

	// Lock memory
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
		perror("mlockall failed");
	}
	
	// Pre-fault our stack
	//stack_prefault();
	
	// Start after 1 second
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	
	// Loop forever at specific sampling rate
	while(1){
		// Wait until next shot
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
			
		// Run controller
			
			
		// Set motor PWM signals by writing to the sensor.c process which applies the changes over I2C.
		//if (write(ptrPipe1->parent[1], value, sizeof(value)) != sizeof(value)) printf("write error in controller to sensor\n");
		//else printf("Controller ID: %d, Sent: %f to Communication\n", (int)getpid(), controllerDataBuffer[0]);
	
		// Set update of constraints and controller results by writing to the communication.c process which applies the changes over UDP.
		//if (write(ptrPipe2->parent[1], controllerDataBuffer, sizeof(controllerDataBuffer)) != sizeof(controllerDataBuffer)) printf("write error in controller to communication\n");
		//else printf("Controller ID: %d, Sent: %f to Communication\n", (int)getpid(), controllerDataBuffer[0]);

		// Update watchdog
		pthread_mutex_lock(&mutexWatchdog);
			globalWatchdog++;
		pthread_mutex_unlock(&mutexWatchdog);
		
		// Calculate next shot
		t.tv_sec += Ts;
		while (t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}
	}
	
	return NULL;
}

// Thread - Watchdog for controller to flag if sampling time is not satisfied.
void *threadControllerWatchdog(void *arg)
{	
	// Get pipe and define local variables
	struct timespec t;
	struct sched_param param;
	int watchdog_current, watchdog_prev=0;

	// Declare ourself as a real time task
	param.sched_priority = 40;
	if(sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1){
		perror("sched_setscheduler failed");
	}
	
	// Lock memory
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
		perror("mlockall failed");
	}
	
	// Pre-fault our stack
	//stack_prefault();
	
	// Start after 1 second
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	
	// Run controller algorithm
	while(1){
		// Wait until next shot
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
		
		// Get watchdog status
		pthread_mutex_lock(&mutexWatchdog);
			watchdog_current=globalWatchdog; // current
			globalWatchdog=watchdog_prev+1; // update to new
		pthread_mutex_unlock(&mutexWatchdog);
		
		// Check if deadline was met
		if (watchdog_current==watchdog_prev){
			printf("MPC did NOT meet deadline2\n");
		}
		
		// Update previous watchdog to current
		watchdog_prev++;
		
		// Calculate next shot
		t.tv_sec += Ts;
		while (t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}
	}
	
	return NULL;
}
/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

void stack_prefault(void){
	unsigned char dummy[MAX_SAFE_STACK];
	
	memset(dummy, 0, MAX_SAFE_STACK);
	return;
}



				/*
		// Read in PWM value
		printf("Enter PWM value:\n");
		fgets(input, 10, stdin);
		value[0] = atof(input);
		printf("Value: %f\n", value[0]);
		
		for (int i=1;i<4;i++){
			value[i]=value[0];
		}
		*/	
