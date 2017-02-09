/* Multiprocessing scenario where the 
 * main (parent) process has 3 sub (child)
 * processes.
 * 
 *
 * Parent	-> Controller - Child1(threadMPC)
 * 			-> Sensor fusion - Child2(threadReadIMU, threadAngles, threadReadBeacon, threadPosition)
 * 			-> Data communication - Child3(threadWriteUdp, threadReadUdp)
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
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <inttypes.h>

int main(int argc, char **argv)
{
	// Pipe array
	int pipeSensorFusionToController[2];
	int pipeSensorFusionToCommunication[2];
	int pipeCommunicationToController[2];
	
	// Create pipes
	if(pipe(pipeSensorFusionToController)==-1) printf("pipe error");
	if(pipe(pipeSensorFusionToCommunication)==-1) printf("pipe error");
	if(pipe(pipeCommunicationToController)==-1) printf("pipe error");
	
	// Loop through creating three child processes from parent
	printf("Creating first child from main process ID: %d\n", (int)getpid());
	switch(fork())
	{
		// If the fork() failed
		case -1:
		printf("#1 Fork() error\n");
		break;

		// Child process
		case 0:	
		/*
		* 
		* Call Sensor Fusion threads
		* 
		*/
		printf("Sensor Fusion child process ID: %d\n", (int)getpid());
		while(1)
			{
				sleep(1);
				printf("Sensor Fusion child process ID: %d\n", (int)getpid());
			}
		break;

		// Parent process - Create second child process
		default:
		printf("Creating second child from main process ID: %d\n", (int)getpid());
		switch(fork())
		{
			// If the fork() failed
			case -1:
			printf("#1 Fork() error\n");
			break;
			
				// Child process
				case 0:	
				/*
				* 
				* Call Controller threads
				* 
				* 
				*/
				printf("Controller child process ID: %d\n", (int)getpid());
				while(1)
				{
					sleep(1);
					printf("Controller child process ID: %d\n", (int)getpid());
				}
				break;
				
				// Parent process - Create third child process
				default:
				printf("Creating third child from main process ID: %d\n", (int)getpid());
				switch(fork())
				{
					// If the fork() failed
					case -1:
					printf("#1 Fork() error\n");
					break;

					// Child process
					case 0:	
					/*
					* 
					* Call Sensor Fusion threads
					* 
					*/
					printf("Communication child process ID: %d\n", (int)getpid());
					while(1)
					{
						sleep(1);
						printf("Communication child process ID: %d\n", (int)getpid());
					}
					break;

					// Parent process - Create second child process
					default:
					printf("Starting initialization... from process ID: %d\n", (int)getpid());
				}
		}
	}
	return 0;
}

