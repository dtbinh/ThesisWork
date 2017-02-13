// SENSOR FUSION CODE

#include "sensor.h"
#include "startup.h"

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>

typedef struct _structPipeArray{
	structPipe *pipeToCtrl;
	structPipe *pipeToComm;
} structPipeArray;

/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *pipeSensToCtrl, void *pipeSensToComm){
	// Put pipes in to array for passing on as argument to pthread_create
	structPipeArray pipeArray = {.pipeToCtrl=pipeSensToCtrl, .pipeToComm=pipeSensToComm};	
	
	// Create thread
	pthread_t threadPipeSensToCtrlAndComm, threadAngles, threadPosition;
	int res1, res2, res3;
	
	res1=pthread_create(&threadPipeSensToCtrlAndComm, NULL, &threadPipeSensorToControllerAndComm, &pipeArray);
	res2=pthread_create(&threadAngles, NULL, &threadSensorFusionAngles, NULL);
	res3=pthread_create(&threadPosition, NULL, &threadSensorFusionPosition, NULL);	
	
	if (!res1) pthread_join( threadPipeSensToCtrlAndComm, NULL);
	if (!res2) pthread_join( threadAngles, NULL);
	if (!res3) pthread_join( threadPosition, NULL);
}



/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Static variables for threads
static initState initStateSensor=STARTUP;
static int flagAngles=0;
static int flagPosition=0;
static float position[3]={1.0f,1.0f,1.0f};
static float angles[3]={2.0f, 2.0f, 2.0f};
static pthread_mutex_t mutexPosition = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexAngle = PTHREAD_MUTEX_INITIALIZER;

// Thread - Sensor Fusion Position
void *threadSensorFusionPosition (void *arg){
	// Loop for ever
	while(1){
		switch (initStateSensor)
		{
			case STARTUP:
				// Wait for process communication to get initialized
				printf("threadSensorFusionPosition waiting for process communication to get ready...\n");
				sleep(2);
			break;
			
			default:
				/* 
				 * Call Sensor fusion function for position
				 */
				 
				// Write data to local variables using mutex
				pthread_mutex_lock(&mutexPosition);
				for(int i=0;i<3;i++){
					position[i]+=0.1;
				}
				usleep(200);
				pthread_mutex_unlock(&mutexPosition);
				
				 // After running the sensor fusion first time, set initStateSensor ready
				flagPosition=1;
				if (initStateSensor==WAITING && flagAngles==1 && flagPosition==1) initStateSensor=READY;
				//sleep(1);
			break;
		}
	}
	return NULL;
}

// Thread - Sensor Fusion Angles
void *threadSensorFusionAngles (void *arg){
	// Loop for ever
	while(1){
		switch (initStateSensor)
		{
			case STARTUP:
				// Wait for process communication to get initialized
				printf("threadSensorFusionPosition waiting for process communication to get ready...\n");
				sleep(2);
			break;
			
			default:
				/* 
				 * Call Sensor fusion function for angles
				 */
				 
				// Write data to local variables using mutex
				pthread_mutex_lock(&mutexAngle);
				for(int i=0;i<3;i++){
					angles[i]+=0.1;
				}
				usleep(200);
				pthread_mutex_unlock(&mutexAngle);
				
				 // After running the sensor fusion first time, set initStateSensor ready
				 flagAngles=1;
				 if (initStateSensor==WAITING && flagAngles==1 && flagPosition==1) initStateSensor=READY;
				 //sleep(1);
			break;
		}
			
	}
	return NULL;
}

// Thread - Pipe Sensor to Controller write
void *threadPipeSensorToControllerAndComm (void *pipeArray){
	structPipeArray *ptrPipeArray = pipeArray;
	structPipe *ptrPipeToCtrl = ptrPipeArray->pipeToCtrl;
	structPipe *ptrPipeToComm = ptrPipeArray->pipeToComm;
	
	float dataBuffer[6];
	//uint8_t initBuffer[PIPE_BUFFER_SIZE];
	
	printf("Communication process ID %d starting sending data\n", (int)getpid()); 
	
	while(1){
		// Put sensor data in to dataBuffer
		pthread_mutex_lock(&mutexAngle);
		for(int i=0;i<3;i++){
		dataBuffer[i]=angles[i];
		}
		pthread_mutex_unlock(&mutexAngle);
		pthread_mutex_lock(&mutexPosition);
		for(int i=0;i<3;i++){
		dataBuffer[i+3]=position[i];
		}
		pthread_mutex_unlock(&mutexPosition);
		
		// Write to Controller process
		if (write(ptrPipeToCtrl->child[1], dataBuffer, sizeof(dataBuffer)) != sizeof(dataBuffer)) printf("write error in child\n");
		else printf("In child ID: %d, Sent: %f\n", (int)getpid(), dataBuffer[0]);
		// Write to Communication process
		if (write(ptrPipeToComm->child[1], dataBuffer, sizeof(dataBuffer)) != sizeof(dataBuffer)) printf("write error in child\n");
		else printf("In child ID: %d, Sent: %f\n", (int)getpid(), dataBuffer[0]);

		sleep(1);
	}
	
	/*
	initBuffer[0]=0;
	while(1){
		switch (initStateSensor)
		{
			// STARTUP procedure to make sure all child processes are communicating
			case STARTUP:
			if(read(newPipe->parent[0], initBuffer, sizeof(initBuffer)) == -1) printf("read error in child\n");
			else printf("In child ID: %d, Recieved: %d\n", (int)getpid(), initBuffer[0]);
			// If recieved value 1 the child process sends 1 back to indicate communication working
			if (initBuffer[0]==1){
				if (write(newPipe->child[1], initBuffer, sizeof(initBuffer)) != sizeof(initBuffer)) printf("write error in child\n");
				else printf("In child ID: %d, Sent: %d\n", (int)getpid(), initBuffer[0]);
			}
			// If recieved value 2 the parent process has got all child processes running and the tasks can start
			else if (initBuffer[0]==2){
				initStateSensor=WAITING;
			}
			break;
			
			// WAITING procedure to make sure that the sensor fusion has started
			case WAITING:
				printf("threadProcessCommunication waiting for first sensor fusion computations\n");
				usleep(100);
			break;
			
			// READY procedure for sending sensor fusion data to controller
			case READY:
				// Put sensor data in to dataBuffer
				pthread_mutex_lock(&mutexAngle);
				for(int i=0;i<3;i++){
					dataBuffer[i]=angles[i];
				}
				pthread_mutex_unlock(&mutexAngle);
				pthread_mutex_lock(&mutexPosition);
				for(int i=0;i<3;i++){
					dataBuffer[i+3]=position[i];
				}
				pthread_mutex_unlock(&mutexPosition);
				
				if (write(newPipe->child[1], &dataBuffer, sizeof(dataBuffer)) != sizeof(dataBuffer)) printf("write error in child\n");
				else printf("In child ID: %d, Sent: %.1f - %.1f - %.1f - %.1f - %.1f - %.1f\n", (int)getpid(), dataBuffer[0], dataBuffer[1], dataBuffer[2], dataBuffer[3], dataBuffer[4], dataBuffer[5]);
			break;
		}
		sleep(1);
	}
	*/
	return NULL;
}


/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/




