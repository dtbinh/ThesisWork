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

// PREEMPT_RT
//#include <time.h>
#include <sched.h>
#include <sys/mman.h>

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadPipeControllerToComm(void*);
static void *threadPipeSensorToCommunication(void*);
static void *threadPipeCommunicationtoController(void*);
static void *threadUdpRead(void*);
static void *threadUdpWrite();
static void openSocketCommunication(void);
static void *threadKeyReading( void* );
// Functions
static void keyReading( int );


// Static variables for threads
static double controllerData[9]={0,0,0,0,0,0,0,0,0};
static double sensorData[19]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static double keyboardData[4]={0,0,0,0};

static int socketReady=0;

//static float setpoint[] = {0.0,0.0,0.0}; // coordinates {x,y,z}
//static double constraints[6] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // coordinates {x1,y1,z1,x2,y2,z2,x3,y3,z3}
//static float tuning[] = {0.0,0.0,0.0}; // temporary tuning parameters

static int fdsocket_read, fdsocket_write;
static struct sockaddr_in addr_read, addr_write;
static socklen_t fromlen = sizeof(addr_read);
static int broadcast=1;
static char readBuff[BUFFER_LENGTH];
static char writeBuff[BUFFER_LENGTH];

static pthread_mutex_t mutexControllerData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexSensorData = PTHREAD_MUTEX_INITIALIZER;

//static void messageDecode(char*);


/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startCommunication(void *arg1, void *arg2)
{
	pipeArray pipeArray1 = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create thread
	pthread_t threadPipeCtrlToComm, threadPipeSensorToComm, threadUdpR, threadUdpW, threadkeyRead;
	int res1, res2, res3, res4, res5;
	
	// Activate socket communication before creating UDP threads
	openSocketCommunication();

	res1=pthread_create(&threadPipeCtrlToComm, NULL, &threadPipeControllerToComm, arg1);
	res2=pthread_create(&threadPipeSensorToComm, NULL, &threadPipeSensorToCommunication, arg2);
	res3=pthread_create(&threadUdpR, NULL, &threadUdpRead, &pipeArray1);
	res4=pthread_create(&threadUdpW, NULL, &threadUdpWrite, NULL);
	res5=pthread_create(&threadkeyRead, NULL, &threadKeyReading, NULL);
	
	// If threads created successful, start them
	if (!res1) pthread_join( threadPipeCtrlToComm, NULL);
	if (!res2) pthread_join( threadPipeSensorToComm, NULL);
	if (!res3) pthread_join( threadUdpR, NULL);
	if (!res4) pthread_join( threadUdpW, NULL);
	if (!res5) pthread_join( threadkeyRead, NULL);
}


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Pipe Communication from Controller read
static void *threadPipeControllerToComm(void *arg)
{
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float controllerDataBuffer[9];
	
	// Loop forever reading/waiting for data
	while(1){
		// Read data from controller process
		//if(read(ptrPipe->parent[0], controllerDataBuffer, sizeof(controllerDataBuffer)) == -1) printf("read error in communication from controller\n");
		//else printf("Communication ID: %d, Recieved Controller data: %f\n", (int)getpid(), controllerDataBuffer[0]);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexControllerData);
			memcpy(controllerData, controllerDataBuffer, sizeof(controllerDataBuffer));
		pthread_mutex_unlock(&mutexControllerData);
		
		sleep(1);
	}
	return NULL;
}


// Thread - Pipe Communication from Sensor read
static void *threadPipeSensorToCommunication(void *arg)
{
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	double sensorDataBuffer[19];
	
	// Loop forever reading/waiting for data
	while(1){
		// Read data from sensor process
		if(read(ptrPipe->parent[0], sensorDataBuffer, sizeof(sensorDataBuffer)) == -1) printf("read error in communication from sensor\n");
		//else printf("Communication ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexSensorData);
			memcpy(sensorData, sensorDataBuffer, sizeof(sensorDataBuffer));
		pthread_mutex_unlock(&mutexSensorData);
	}
	return NULL;
}


// Thread - Pipe Communication to Controller write
static void *threadPipeCommunicationtoController(void *arg)
{
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	double sensorDataBuffer[19];
	
	// Loop forever reading/waiting for data
	while(1){
		// Read data from sensor process
		if(read(ptrPipe->parent[0], sensorDataBuffer, sizeof(sensorDataBuffer)) == -1) printf("read error in communication from sensor\n");
		//else printf("Communication ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexSensorData);
			memcpy(sensorData, sensorDataBuffer, sizeof(sensorDataBuffer));
		pthread_mutex_unlock(&mutexSensorData);
	}
	return NULL;
}



// UDP read thread
static void *threadUdpRead(void *arg)
{
	// Get pipe array and define local variables
	//pipeArray *pipeArray1 = arg;
	//structPipe *ptrPipe1 = pipeArray1->pipe1;
	//structPipe *ptrPipe2 = pipeArray1->pipe2;
	float udpDataBuffer[6]={2,2,2,2,2,2};
	
	// Loop forever reading/waiting for UDP data, calling message decoder and sending data to controller
	while(1){
		/*
		if (recvfrom(fdsocket_read, readBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &addr_read, &fromlen) == -1){
			perror("read");
		}
		else{*/
			// Call messageDecode
			//messageDecode(readBuff);
			
			// Write data to Controller process
			//if (write(ptrPipe1->child[1], udpDataBuffer, sizeof(udpDataBuffer)) != sizeof(udpDataBuffer)) printf("write error in parent\n");
			//else printf("Communication ID: %d, Sent: %f to Controller\n", (int)getpid(), udpDataBuffer[0]);
			
			// Clear readBuffer
			//memset(&readBuff[0], 0, sizeof(readBuff));
			
			// Write data to Sensor process
			//float sensorTuning[3]={9,9,9};
			//if (write(ptrPipe2->child[1], sensorTuning, sizeof(sensorTuning)) != sizeof(sensorTuning)) printf("write error in parent\n");
			//else printf("Communication ID: %d, Sent: %f to Sensor\n", (int)getpid(), sensorTuning[0]);
			sleep(5);
				
		//}
	}
	
	return NULL;
}


// UDP write thread
static void *threadUdpWrite()
{
	// Local variables
	double agentData[19]={0,0,0,0,0,0,0,0,0,0,0,0};
	
	// Loop forever streaming data
	while(1){
		if(socketReady==1){
			// Get sensor and controller data from global variables in communication.c
			pthread_mutex_lock(&mutexSensorData);
				memcpy(agentData, sensorData, sizeof(sensorData));
			pthread_mutex_unlock(&mutexSensorData);
			//printf("threadUdpWrite: %f\n", agentData[18]);
			//printf("Communication 2 ID: %d, Recieved Sensor data: %f\n", (int)getpid(), agentData[0]);
			
			//pthread_mutex_lock(&mutexControllerData);
			//memcpy(agentData+sizeof(sensorData), controllerData, sizeof(controllerData));
			//pthread_mutex_unlock(&mutexControllerData);
			
			
				
			sprintf(writeBuff,"A1A6DA%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f,%08.3f",agentData[0] ,agentData[1] ,agentData[2], agentData[3] ,agentData[4] ,agentData[5], agentData[6] ,agentData[7] ,agentData[8], agentData[9] ,agentData[10] ,agentData[11] ,agentData[12] ,agentData[13] ,agentData[14] ,agentData[15],agentData[16] ,agentData[17] ,agentData[18]);
			//printf("%s\n", writeBuff);
			// Send data over UDP
			usleep(20000);
			if (sendto(fdsocket_write, writeBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &addr_write, sizeof(addr_write)) == -1){
				perror("write");
			}
		}
		else{
			//printf("Socket not ready\n");
			usleep(20000);
		}
			
	}
	return NULL;
}

// Thread - reading from the keyboard from the stand-alone computer
static void *threadKeyReading( void *arg ) {
	
	while(1) {
		printf("threadKeyReading \n");
		
		keyReading(NULL);
		
	}
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
	printf("Socket ready\n");
	socketReady=0;
}

/* Read in PWM value */
void keyReading( int arg ) {
	char input_char[10];
	double keyboardDataBuffer[4]={0,0,0,0}; // {ref_x,ref_y,ref_z,sk}
	
	printf("I read %f \n", atof(input_char));
	
	
	
	
	
	
	pthread_mutex_lock(&mutexSensorData);
		memcpy(keyboardData, keyboardDataBuffer, sizeof(keyboardDataBuffer));
	pthread_mutex_unlock(&mutexSensorData);
	
	
	
	/*
	double input, value[4];
	printf("Enter PWM value:\n");
	fgets(input, 10, stdin);
	value[0] = atof(input);
	printf("Value: %f\n", value[0]);
	
	for (int i=1;i<4;i++){
		value[i]=value[0];
	}
	*/
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

