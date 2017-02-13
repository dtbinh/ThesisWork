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
/****************************VARIABLES*****************************/
/******************************************************************/

// Static variables for threads



/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startController(void *pipeSensToCtrl, void *pipeComCtrl){
	// Create threads
	pthread_t threadPipeSensToCtrl, threadPipeCommToCtrl, intCtrl;
	int res1, res2, res3;
	
	res1=pthread_create(&threadPipeSensToCtrl, NULL, &threadPipeSensorToController, pipeSensToCtrl);
	res2=pthread_create(&threadPipeCommToCtrl, NULL, &threadPipeCommToController, pipeComCtrl);
	res3=pthread_create(&intCtrl, NULL, &intController, NULL);
	
	if (!res1) pthread_join( threadPipeSensToCtrl, NULL);
	if (!res2) pthread_join( threadPipeCommToCtrl, NULL);
	if (!res3) pthread_join( intCtrl, NULL);
}



/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Pipe Sensor to Controller read
void *threadPipeSensorToController(void *arg)
{
	// Loop forever reading/waiting for data
	while(1){
		
		usleep(100);
	}
	return NULL;
}

// Thread - Pipe Communication to Controller read
void *threadPipeCommToController(void *arg)
{		
	// Loop forever streaming data
	while(1){
		sleep(1);
	}
	
	return NULL;
}

// Interrupt - Controller algorithm
void *intController(void *arg)
{		
	// Loop forever streaming data (will be ticked by interrupt with given sampling time)
	while(1){
		usleep(100);
		
		
		
	}
	
	return NULL;
}



/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/


