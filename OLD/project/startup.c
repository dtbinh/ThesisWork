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
			
			// Call Sensor Fusion process
			startSensors(&pipeSensorController, &pipeSensorCommunication);
		break;
		
		default:
		
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
					printf("Communication started...\n");
				break;
				
				default:
					printf("Controller process ID: %d\n", (int)getpid());
					
					// Call controller threads within main process
					if (close(pipeSensorController.parent[0])==-1) printf("close error - parent to child write\n");
					if (close(pipeSensorController.child[1])==-1) printf("close error - child to parent read\n");
					if (close(pipeCommunicationController.parent[0])==-1) printf("close error - parent to child write\n");
					if (close(pipeCommunicationController.child[1])==-1) printf("close error - child to parent read\n");
					
					startController(&pipeSensorController, &pipeCommunicationController);
					printf("Controller started...\n");
				break;
			}
		
		break;
	}

	return 0;
}



/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

// Matrix print function
void printmat(double *A, int m, int n){
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%3.18f\t", *(dptr+m*i+j));
        }
        printf("\n");
    }
    //printf("\n");
    return;
}
