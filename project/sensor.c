// SENSOR FUSION CODE

#include "sensor.h"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>

/*
 * 		Structures declared for use within functions
 **********************************************************
 */
/*
 * threadSensorFusionPosition:
 * 		Thread that handles the sensor fusion of data
 * 		from beacon, pressure sensor and accelerometer
 **********************************************************
 */
/*
struct _acc{
	float x;
	float y;
	float z;
};
struct _gyr{
	float x;
	float y;
	float z;
};
struct _mag{
	float x;
	float y;
	float z;
};
struct _bmp{
	float pres;
	float alt;
	float temp;
};*/

//static struct structPipeSensor pipeSensors;

/*
static void *threadSensorFusionPosition (void *arg){
	// Loop for ever
	while(1){
	sleep(1);
	}
	return NULL;
}

static void *threadSensorFusionAngles (void *arg){
	// Loop for ever
	while(1){
	sleep(1);
	}
	return NULL;
}*/

static void *threadProcessCommunication (void *pipe){
	structPipeSensor *newPipe = (structPipeSensor*)pipe;
	// close write and read ends insuring no old data present
	if (close((*newPipe).parent[1])==-1) printf("close error - parent to child write\n");
	if (close((*newPipe).child[0])==-1) printf("close error - parent to child write\n");
	
	sleep(5);
	
	printf("Communication process ID %d starting sending data", (int)getpid()); 
	
	uint8_t buff[1]={0};
	
	if(read((*newPipe).parent[0], buff, 1) == -1) printf("read error in child\n");
	printf("ID: %d, Recieved: %d", (int)getpid(), buff[0]); 
	if(write((*newPipe).child[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error in child\n");
	printf("ID: %d, Sent: %d", (int)getpid(), buff[0]); 
	return NULL;
}

/*
int startSensorFusionPosition(void){
	// Create thread
	pthread_t threadPosition;
	int res;
	res=pthread_create(&threadPosition, NULL, &threadSensorFusionPosition, NULL);
	return res;
}

int startSensorFusionAngles(void){
	// Create thread
	pthread_t threadAngles;
	int res;
	res=pthread_create(&threadAngles, NULL, &threadSensorFusionAngles, NULL);
	return res;
}
*/


int startProcessCommunication(structPipeSensor *pipe){
	// Create thread
	pthread_t threadProcessCom;
	int res;
	res=pthread_create(&threadProcessCom, NULL, &threadProcessCommunication, (void*) &pipe);
	return res;
}






