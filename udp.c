#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <errno.h>
#include <math.h>
#include <arpa/inet.h>
#include <string.h>

#define SERVERPORT 5000
#define BUFFER_LENGTH 25

#define SUPERVISOR "A0"
#define AGENT1 "A1"
#define AGENT2 "A2"
#define AGENT3 "A3"
#define MYSELF "A1"
#define DATA "DA"
#define STREAM "ST"
#define SETPOINT "SP"
#define TUNING "TU"

int stream=1;
float setpoint[] = {0.0,0.0,0.0}; // coordinates {x,y,z}
float constraints[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // coordinates {x1,y1,z1,x2,y2,z2,x3,y3,z3}
float tuning[] = {0.0,0.0,0.0}; // temporary tuning parameters

int fd_socket;
struct sockaddr_in their_addr;
socklen_t fromlen;
int broadcast=1;
char readBuff[BUFFER_LENGTH];
char writeBuff[BUFFER_LENGTH];

pthread_mutex_t mutexPrint = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexData = PTHREAD_MUTEX_INITIALIZER;

void *threadUdpRead();
void *threadUdpWrite();
void messageDecode(char *input);

// *******************MAIN FUNCTION*******************
int main(int argc, char **argv)
{
	// Create socket. SOCK_STREAM=TPC. SOCK_DGRAM=UDP. 
	fd_socket = socket(AF_INET, SOCK_DGRAM, 0);
	
	if (fd_socket == -1){
		perror("socket");
		exit(1);
	}
	
	// Activate UDP broadcasting
	if (setsockopt(fd_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1){
		perror("setup");
		exit(1);
	}
	
	// Socket settings
	their_addr.sin_family = AF_INET;
	their_addr.sin_port = htons(SERVERPORT);
	their_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	memset(their_addr.sin_zero, '\0',sizeof(their_addr.sin_zero));
	
	// Bind socket to port
	if (bind(fd_socket, (struct sockaddr*)&their_addr, sizeof(their_addr)) == -1){
		perror("bind");
	}
	
	// Create threads
	pthread_t thread1, thread2;
	if (pthread_create(&thread1, NULL, &threadUdpRead, NULL)!=0){
		perror("thread1");
	} 
	if (pthread_create(&thread2, NULL, &threadUdpWrite, NULL)!=0){
		perror("thread2");
	}
	
	// Activate threads
	pthread_join( thread1, NULL);
	pthread_join( thread2, NULL);
	
	exit(EXIT_SUCCESS);
	return 0;
}

// UDP read thread
void *threadUdpRead()
{
	fromlen = sizeof(their_addr);
	// Loop forever reading/waiting for data
	while(1){
		if (recvfrom(fd_socket, readBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &their_addr, &fromlen) == -1){
			perror("read");
		}
		else{
			//printf("Data read: %s\n\n", readBuff);
			messageDecode(readBuff);
			memset(&readBuff[0], 0, sizeof(readBuff));
			//pthread_mutex_lock(&mutexData);
			//sprintf(writeBuff,"%f",data);
			//printf("Data read: %s\n\n", readBuff);
			//pthread_mutex_unlock(&mutexData);
		}
	delay(1000);
	}
}

// UDP write thread
void *threadUdpWrite()
{		
	//float data2=1.0;
	// Loop forever streaming data
	while(1){
		//strncpy(writeBuff,"A1A1S201.00,055.10,10.99",BUFFER_LENGTH);
		//strncpy(writeBuff,"A0A1ST1",BUFFER_LENGTH);
		//strncpy(writeBuff,"A0A1TU10000,10000,10000",BUFFER_LENGTH);
		//strncpy(writeBuff,"A0A1TU10000,10000,10000",BUFFER_LENGTH);
		strncpy(writeBuff,"A3A6DA21.00,55.10,10.99",BUFFER_LENGTH);
		
		delay(2000);
		//pthread_mutex_lock(&mutexData);
		//sprintf(writeBuff,"%f",data2++);
		//pthread_mutex_unlock(&mutexData);
		if (stream){
			if (sendto(fd_socket, writeBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &their_addr, sizeof(their_addr)) == -1){
				perror("write");
			}
			else{
				pthread_mutex_lock(&mutexData);
				printf("Data sent: %s\n", writeBuff);
				pthread_mutex_unlock(&mutexData);
				memset(&writeBuff[0], 0, sizeof(writeBuff));
			}
		}
	}
}

// Data messages decoded with respect to the documentation i sharelatex
void messageDecode(char *input)
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

