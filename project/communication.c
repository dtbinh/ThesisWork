// COMMUNICATION CODE

#include "communication.h"
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

// Static variables for threads
static initState initStateComm=STARTUP;
static int stream=1;/*
static float setpoint[] = {0.0,0.0,0.0}; // coordinates {x,y,z}
static float constraints[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // coordinates {x1,y1,z1,x2,y2,z2,x3,y3,z3}
static float tuning[] = {0.0,0.0,0.0}; // temporary tuning parameters
*/
static int fdsocket_read, fdsocket_write;
static struct sockaddr_in addr_read, addr_write;
static socklen_t fromlen;
static int broadcast=1;
static char readBuff[BUFFER_LENGTH];
static char writeBuff[BUFFER_LENGTH];

pthread_mutex_t mutexPrint = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexData = PTHREAD_MUTEX_INITIALIZER;

//static void messageDecode(char*);


/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startCommunication(void *pipeSensToCom, void *pipeComToCtrl){
	// Create thread
	pthread_t threadProcessCom, threadUdpR, threadUdpW;
	int res1, res2, res3;
	
	res1=pthread_create(&threadProcessCom, NULL, &threadProcessCommunication, (void*) pipe);
	if (!res1) pthread_join( threadProcessCom, NULL);
	
	// Activate socket communication before creating UDP threads
	openSocketCommunication();
	
	res2=pthread_create(&threadUdpR, NULL, &threadUdpRead, NULL);
	res3=pthread_create(&threadUdpW, NULL, &threadUdpWrite, NULL);
	if (!res2) pthread_join( threadUdpR, NULL);
	if (!res3) pthread_join( threadUdpW, NULL);
}







/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// UDP read thread
static void *threadUdpRead()
{
	fromlen = sizeof(addr_read);
	// Loop forever reading/waiting for data
	while(1){
		if (recvfrom(fdsocket_read, readBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &addr_read, &fromlen) == -1){
			perror("read");
		}
		else{
			//printf("Data read: %s\n\n", readBuff);
			//messageDecode(readBuff);
			memset(&readBuff[0], 0, sizeof(readBuff));
			//pthread_mutex_lock(&mutexData);
			//sprintf(writeBuff,"%f",data);
			//printf("Data read: %s\n\n", readBuff);
			//pthread_mutex_unlock(&mutexData);
		}
	sleep(1);
	}
	
	return NULL;
}

// UDP write thread
static void *threadUdpWrite()
{		
	//float data2=1.0;
	// Loop forever streaming data
	while(1){
		//strncpy(writeBuff,"A1A1S201.00,055.10,10.99",BUFFER_LENGTH);
		//strncpy(writeBuff,"A0A1ST1",BUFFER_LENGTH);
		//strncpy(writeBuff,"A0A1TU10000,10000,10000",BUFFER_LENGTH);
		//strncpy(writeBuff,"A0A1TU10000,10000,10000",BUFFER_LENGTH);
		strncpy(writeBuff,"A1A6DA01.00,01.00,01.00",BUFFER_LENGTH);
		
		sleep(10);
		//pthread_mutex_lock(&mutexData);
		//sprintf(writeBuff,"%f",data2++);
		//pthread_mutex_unlock(&mutexData);
		
		if (stream){
			if (sendto(fdsocket_write, writeBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &addr_write, sizeof(addr_write)) == -1){
				perror("write");
			}
			else{
				//pthread_mutex_lock(&mutexData);
				//printf("Data sent: %s\n", writeBuff);
				//pthread_mutex_unlock(&mutexData);
				memset(&writeBuff[0], 0, sizeof(writeBuff));
			}
		}
	}
	
	return NULL;
}

// Thread for process communication
static void *threadProcessCommunication (void *pipe){
	structPipe *ptrPipeComm = pipe;
	float dataBuffer[6]={0,0,0,0,0,0};
	uint8_t initBuffer[PIPE_BUFFER_SIZE];
	
	
	printf("Communication process ID %d starting sending data\n", (int)getpid()); 
	initBuffer[0]=0;
	while(1){
		switch (initStateComm)
		{
			// STARTUP procedure to make sure all child processes are communicating
			case STARTUP:
			if(read(ptrPipeComm->parent[0], initBuffer, sizeof(initBuffer)) == -1) printf("read error in child\n");
			else printf("In child ID: %d, Recieved: %d\n", (int)getpid(), initBuffer[0]);
			// If recieved value 1 the child process sends 1 back to indicate communication working
			if (initBuffer[0]==1){
				if (write(ptrPipeComm->child[1], initBuffer, sizeof(initBuffer)) != sizeof(initBuffer)) printf("write error in child\n");
				else printf("In child ID: %d, Sent: %d\n", (int)getpid(), initBuffer[0]);
			}
			// If recieved value 2 the parent process has got all child processes running and the tasks can start
			else if (initBuffer[0]==2){
				initStateComm=WAITING;
			}
			break;
			
			// WAITING procedure to make sure that the sensor fusion has started
			case WAITING:
				printf("threadProcessCommunication waiting for first communication data\n");
				initStateComm=READY;
				usleep(100);
			break;
			
			// READY procedure for sending sensor fusion data to controller
			case READY:
				// Put sensor data in to dataBuffer
				/*pthread_mutex_lock(&mutexAngle);
				for(int i=0;i<3;i++){
					dataBuffer[i]=angles[i];
				}
				pthread_mutex_unlock(&mutexAngle);
				pthread_mutex_lock(&mutexPosition);
				for(int i=0;i<3;i++){
					dataBuffer[i+3]=position[i];
				}
				pthread_mutex_unlock(&mutexPosition);*/
				// Write dataBuffer to pipe
				if (write(ptrPipeComm->child[1], &dataBuffer, sizeof(dataBuffer)) != sizeof(dataBuffer)) printf("write error in child\n");
				else printf("In child ID: %d, Sent: %.1f - %.1f - %.1f - %.1f - %.1f - %.1f\n", (int)getpid(), dataBuffer[0], dataBuffer[1], dataBuffer[2], dataBuffer[3], dataBuffer[4], dataBuffer[5]);
			break;
		}
		sleep(1);
	}

	return NULL;
}




/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

// Create UDP sockets and bind.
static void openSocketCommunication(){
// Create sockets. SOCK_STREAM=TPC. SOCK_DGRAM=UDP. 
	fdsocket_read = socket(AF_INET, SOCK_DGRAM, 0);
	fdsocket_write = socket(AF_INET, SOCK_DGRAM, 0);	
	if (fdsocket_read == -1){
		perror("socket read");
		exit(1);
	}
	if (fdsocket_write == -1){
	perror("socket write");
	exit(1);
	}
	
	// Activate UDP broadcasting
	if (setsockopt(fdsocket_read, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1){
		perror("setup read");
		exit(1);
	}
	if (setsockopt(fdsocket_write, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1){
	perror("setup write");
	exit(1);
	}
	
	// Socket settings
	addr_read.sin_family = AF_INET;
	addr_read.sin_port = htons(SERVERPORT);
	addr_read.sin_addr.s_addr = htonl(INADDR_ANY); // INADDR_ANY = 0.0.0.0 (allows the socket to bind to port)
	memset(addr_read.sin_zero, '\0',sizeof(addr_read.sin_zero));
	addr_write.sin_family = AF_INET;
	addr_write.sin_port = htons(SERVERPORT);
	addr_write.sin_addr.s_addr = htonl(INADDR_BROADCAST); // INADDR_BROADCAST = 255.255.255.255 (cannot be binded to)
	memset(addr_write.sin_zero, '\0',sizeof(addr_write.sin_zero));
	
	// Bind socket to port
	if (bind(fdsocket_read, (struct sockaddr*)&addr_read, sizeof(addr_read)) == -1){
		perror("bind read");
	}
}

/*

// Data messages decoded with respect to the documentation i sharelatex
static void messageDecode(char *input)
{
	if (!strncmp(input,MYSELF,2)){
		printf("Message from myself. No actions\n");
	}
	else if (!strncmp(input,SUPERVISOR,2)){
		printf("Message from SUPERVISOR\n");
		// Check if supervisor message concerns me
		if (!strncmp(input+2,MYSELF,2)){	
			// Activate/deactivate data stream
			if (!strncmp(input+4,STREAM,2)){
				char value[1];
				strncpy(value,input+6,1);
				if (!strcmp(value,"1")){
					stream=1;
				}
				else if(!strcmp(value,"0")){
					stream=0;
				}
				else{
					printf("Error. Bad stream activation format\n");
				}
			}
			// Setpoint change
			else if (!strncmp(input+4,SETPOINT,2)){
				char value[5];
				if (!strncmp(input+8,".",1) && !strncmp(input+11,",",1) && !strncmp(input+14,".",1) && !strncmp(input+17,",",1) && !strncmp(input+20,".",1)){
					strncpy(value,input+6,5); // setpoint x coordinate
					setpoint[0]=strtof(value,NULL);
					strncpy(value,input+12,5); // setpoint y coordinate
					setpoint[1]=strtof(value,NULL);
					strncpy(value,input+18,5); // setpoint z coordinate
					setpoint[2]=strtof(value,NULL);
					printf("New setpoints: %.2f %.2f %.2f\n", setpoint[0], setpoint[1], setpoint[2]);
				}
				else{
					printf("Error. Bad setpoint format\n");
				}
			}
			// Tuning change
			else if (!strncmp(input+4,TUNING,2)){
				char value[5];
				if (!strncmp(input+11,",",1) && !strncmp(input+17,",",1)){
					strncpy(value,input+6,5); // tuning[0]
					tuning[0]=strtof(value,NULL);
					strncpy(value,input+12,5); // tuning[1]
					tuning[1]=strtof(value,NULL);
					strncpy(value,input+18,5); // tuning[2]
					tuning[2]=strtof(value,NULL);
					printf("New tuning parameters: %.2f %.2f %.2f\n", tuning[0], tuning[1], tuning[2]);
				}
				else{
					printf("Error. Bad tuning format\n");
				}
			}
			else{
				printf("Error. Bad type code\n");
			}
		}
		else{
			printf("Error. Bad supervisor to agent address or the message does not concern me.\n");
		}
	}
	else if (!strncmp(input,AGENT1,2)){
		printf("Message from AGENT1\n");
		// Agent1 constraint updates
		if (!strncmp(input+4,DATA,2)){
			char value[5];
			if (!strncmp(input+8,".",1) && !strncmp(input+11,",",1) && !strncmp(input+14,".",1) && !strncmp(input+17,",",1) && !strncmp(input+20,".",1)){
				strncpy(value,input+6,5); // constraints[0]
				constraints[0]=strtof(value,NULL);
				strncpy(value,input+12,5); // constraints[1]
				constraints[1]=strtof(value,NULL);
				strncpy(value,input+18,5); // constraints[2]
				constraints[2]=strtof(value,NULL);
				printf("New constrained positions: %.2f %.2f %.2f\n", constraints[0], constraints[1], constraints[2]);
			}
			else{
				printf("Error. Bad constraints format from AGENT1\n");
			}
		}
		else{
			printf("Error. Bad type code\n");
		}
	}
	else if (!strncmp(input,AGENT2,2)){
		printf("Message from AGENT2\n");
		// Agent2 constraint updates
		if (!strncmp(input+4,DATA,2)){
			char value[5];
			if (!strncmp(input+8,".",1) && !strncmp(input+11,",",1) && !strncmp(input+14,".",1) && !strncmp(input+17,",",1) && !strncmp(input+20,".",1)){
				strncpy(value,input+6,5); // constraints[3]
				constraints[3]=strtof(value,NULL);
				strncpy(value,input+12,5); // constraints[4]
				constraints[4]=strtof(value,NULL);
				strncpy(value,input+18,5); // constraints[5]
				constraints[5]=strtof(value,NULL);
				printf("New constrained positions: %.2f %.2f %.2f\n", constraints[3], constraints[4], constraints[5]);
			}
			else{
				printf("Error. Bad constraints format from AGENT2\n");
			}
		}
		else{
			printf("Error. Bad type code\n");
		}
	}
	else if (!strncmp(input,AGENT3,2)){
		printf("Message from AGENT3\n");
		// Agent3 constraint updates
		if (!strncmp(input+4,DATA,2)){
			char value[5];
			if (!strncmp(input+8,".",1) && !strncmp(input+11,",",1) && !strncmp(input+14,".",1) && !strncmp(input+17,",",1) && !strncmp(input+20,".",1)){
				strncpy(value,input+6,5); // constraints[6]
				constraints[6]=strtof(value,NULL);
				strncpy(value,input+12,5); // constraints[7]
				constraints[7]=strtof(value,NULL);
				strncpy(value,input+18,5); // constraints[8]
				constraints[8]=strtof(value,NULL);
				printf("New constrained positions: %.2f %.2f %.2f\n", constraints[6], constraints[7], constraints[8]);
			}
			else{
				printf("Error. Bad constraints format from AGENT3\n");
			}
		}
		else{
			printf("Error. Bad type code\n");
		}
	}
	else{
		printf("Error. Bad device code\n");
	}
}

*/

