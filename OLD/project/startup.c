/* Multiprocessing scenario where the 
 * main (parent) process has 3 sub (child)
 * processes.
 * 
 *
 * Parent (Startup and Controller)
 * 						-> Sensor fusion - Child1(threadReadIMU, threadAngles, threadReadBeacon, threadPosition)
 * 						-> Data communication - Child2(threadWriteUdp, threadReadUdp)
 * 
 *
 * 
 * 
 * The Program uses the following files:
 * controller.c
 * communication.c
 * sensorfusion.c
 * 
 * 
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <inttypes.h>

#include "sensor.h"
#include "communication.h"
#include "startup.h"
#include "controller.h"
#include "bmp180.h"

void startupProcedure(void*, void*);

/******************************************************************/
/******************************MAIN********************************/
/******************************************************************/

int main(int argc, char **argv)
{
	// Create pipes for passing data between processes
	structPipe pipeSensorController;
	structPipe pipeSensorCommunication;
	structPipe pipeCommunicationController;
	
	if (pipe(pipeSensorController.parent)==-1) printf("pipe error sensor parent");
	if (pipe(pipeSensorController.child)==-1) printf("pipe error sensor child");
	if (pipe(pipeSensorCommunication.parent)==-1) printf("pipe error sensor parent");
	if (pipe(pipeSensorCommunication.child)==-1) printf("pipe error sensor child");
	if (pipe(pipeCommunicationController.parent)==-1) printf("pipe error comm parent");
	if (pipe(pipeCommunicationController.child)==-1) printf("pipe error comm child");

	
	// Create first child process (sensors)
	switch(fork())
	{
		// If the fork() failed
		case -1:
			printf("#1 Fork() error\n");
		break;
		
		case 0:
			printf("Sensor Fusion process ID: %d\n", (int)getpid());
			
			// Close write and read ends insuring no old data present
			if (close(pipeSensorController.parent[1])==-1) printf("close error - parent to child write\n");
			if (close(pipeSensorController.child[0])==-1) printf("close error - child to parent read\n");
			if (close(pipeSensorCommunication.parent[0])==-1) printf("close error - parent to child write\n");
			if (close(pipeSensorCommunication.child[1])==-1) printf("close error - child to parent read\n");
			
			//printf("Sensor child pipe ready...\n");
			
			// Call Sensor Fusion process
			startSensors(&pipeSensorController, &pipeSensorCommunication);
		break;
		
		default:
			printf("Controller process ID: %d\n", (int)getpid());
		break;
	}
	
	
	// Create second child process (communication)
	switch(fork())
	{
		// If the fork() failed
		case -1:
			printf("#2 Fork() error\n");
		break;
		
		case 0:	
			printf("Communication process ID: %d\n", (int)getpid());
			
			// Close write and read ends insuring no old data present
			if (close(pipeCommunicationController.parent[1])==-1) printf("close error - parent to child write\n");
			if (close(pipeCommunicationController.child[0])==-1) printf("close error - child to parent read\n");
			if (close(pipeSensorCommunication.parent[1])==-1) printf("close error - parent to child write\n");
			if (close(pipeSensorCommunication.child[0])==-1) printf("close error - child to parent read\n");
	
			// Call Sensor Fusion process within child process
			startCommunication(&pipeCommunicationController, &pipeSensorCommunication);
		break;
		
		default:
			// Call controller threads within main process
			if (close(pipeSensorController.parent[0])==-1) printf("close error - parent to child write\n");
			if (close(pipeSensorController.child[1])==-1) printf("close error - child to parent read\n");
			if (close(pipeCommunicationController.parent[0])==-1) printf("close error - parent to child write\n");
			if (close(pipeCommunicationController.child[1])==-1) printf("close error - child to parent read\n");
			
			startController(&pipeSensorController, &pipeCommunicationController);
			printf("Controller started...\n");
		break;
	}
	
	return 0;
}



/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/
/*
void startupProcedure(void *pipe1, void *pipe2){
	// Create local pointers to the pipes
	structPipe *ptrPipeSensor = pipe1;
	structPipe *ptrPipeComm = pipe2;
	
	// Close write and read ends insuring no old data present
	if (close(ptrPipeSensor->child[1])==-1) printf("close error - sensor child to parent write\n");
	if (close(ptrPipeSensor->parent[0])==-1) printf("close error - sensor parent to child read\n");
	if (close(ptrPipeComm->child[1])==-1) printf("close error - comm child to parent write\n");
	if (close(ptrPipeComm->parent[0])==-1) printf("close error - comm parent to child read\n");

	sleep(1);
		
	uint8_t buff[1]={1};
			
	// Write startup value=1 to sensor process, wait for result and then write ready signal == 2
	if (write(ptrPipeSensor->parent[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error to sensor process in parent\n");
	else printf("In parent ID: %d, Sent: %d to sensor process\n", (int)getpid(), buff[0]);
	if (read(ptrPipeSensor->child[0], buff, 1) == -1) printf("1 read error in parent\n");
	else printf("In parent ID: %d, Recieved: %d\n", (int)getpid(), buff[0]);
	if (buff[0]==1){
		buff[0]=2;
		if (write(ptrPipeSensor->parent[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error in parent\n");
		else printf("In parent ID: %d, Sent: %d to sensor process\n", (int)getpid(), buff[0]);
		printf("Sensor process startup successful\n");
	}
	else{
		printf("Error pipe sensor process startup\n");
		exit(0);
	}
	
	buff[0]=1;
	
	// Write startup value=1 to communication process, wait for result and then write ready signal == 2
	if (write(ptrPipeComm->parent[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error to communication process in parent\n");
	else printf("In parent ID: %d, Sent: %d to communication process\n", (int)getpid(), buff[0]);
	if (read(ptrPipeComm->child[0], buff, 1) == -1) printf("1 read error in parent\n");
	else printf("In parent ID: %d, Recieved: %d\n", (int)getpid(), buff[0]);
	if (buff[0]==1){
		buff[0]=2;
		if (write(ptrPipeComm->parent[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error in parent\n");
		else printf("In parent ID: %d, Sent: %d to communication process\n", (int)getpid(), buff[0]);
		printf("Communication process startup successful\n");
	}
	else{
		printf("Error pipe sensor process startup");
		exit(0);
	}
	
	sleep(1);	
}
*/

