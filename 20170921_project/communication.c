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
//#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <errno.h>
#include <semaphore.h>
#include <math.h>
#include <arpa/inet.h>
#include <string.h>

// PREEMPT_RT
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadPipeControllerToComm(void*);
static void *threadPipeSensorToCommunication(void*);
//static void *threadPipeCommunicationtoController(void*);
static void *threadUdpRead(void*);
static void *threadUdpWrite();
static void openSocketCommunication(void);
static void *threadKeyReading( void* );

// Functions
static void keyReading( void );

// Static variables for threads
static double controllerData[9]={0,0,0,0,0,0,0,0,0};
static double sensorData[19]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static double keyboardData[18]={0,0,0,0,0,0,0,0,0,0,0,0.01,0.05,0,0,0,0,0}; // {ref_x,ref_y,ref_z, switch[0=STOP, 1=FLY], pwm_print, timer_print,ekf_print,reset ekf/mpc, EKF print 6 states, reset calibration sensor.c, ramp ref, alpha, beta, enable/disable position control, ff attmpc toggle, save data, pid trigger,toggle motor pwm range tuning}
static double tuningMpcData[14]={mpcPos_Q_1,mpcPos_Q_2,mpcPos_Q_3,mpcPos_Q_4,mpcPos_Q_5,mpcPos_Q_6,mpcAtt_Q_1,mpcAtt_Q_2,mpcAtt_Q_3,mpcAtt_Q_4,mpcAtt_Q_5,mpcAtt_Q_6,mpcAlt_Q_1,mpcAlt_Q_2}; // Q and Qf mpc {x,xdot,y,ydot,xform,yform,phi,phidot,theta,thetadot,psi,psidot,z,zdot}
static double tuningMpcQfData[9]={mpcAtt_Qf_1,mpcAtt_Qf_2,mpcAtt_Qf_3,mpcAtt_Qf_4,mpcAtt_Qf_5,mpcAtt_Qf_6,mpcAtt_Qf_1_2,mpcAtt_Qf_3_4,mpcAtt_Qf_5_6};
static double tuningMpcDataControl[6]={mpcPos_R_1,mpcPos_R_2,mpcAtt_R_1,mpcAtt_R_2,mpcAtt_R_3,mpcAlt_R_1}; // R mpc {pos,pos,taux,tauy,tauz,alt}
static double tuningEkfData[18]={ekf_Q_1,ekf_Q_2,ekf_Q_3,ekf_Q_4,ekf_Q_5,ekf_Q_6,ekf_Q_7,ekf_Q_8,ekf_Q_9,ekf_Q_10,ekf_Q_11,ekf_Q_12,ekf_Q_13,ekf_Q_14,ekf_Q_15,ekf_Q_16,ekf_Q_17,ekf_Q_18};
static double tuningPidData[6]={pid_gyro_kp,pid_gyro_ki,pid_gyro_kd,pid_angle_kp,pid_angle_ki,pid_angle_kd}; // PID gains
static double manualThrustData[1]={manualThrust};



static int socketReady=0;

//static float setpoint[] = {0.0,0.0,0.0}; // coordinates {x,y,z}
//static double constraints[6] = {0.manualThrust0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // coordinates {x1,y1,z1,x2,y2,z2,x3,y3,z3}
//static float tuning[] = {0.0,0.0,0.0}; // temporary tuning parameters

static int fdsocket_read, fdsocket_write;
static struct sockaddr_in addr_read, addr_write;
//static socklen_t fromlen = sizeof(addr_read);
static int broadcast=1;
//static char readBuff[BUFFER_LENGTH];
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
	pthread_t threadPipeCtrlToComm, threadPipeSensorToComm, threadUdpW, threadkeyRead; // threadUdpR
	int threadPID1, threadPID2, threadPID4, threadPID5; // threadPID3
	
	// Activate socket communication before creating UDP threads
	openSocketCommunication();

	threadPID1=pthread_create(&threadPipeCtrlToComm, NULL, &threadPipeControllerToComm, arg1);
	threadPID2=pthread_create(&threadPipeSensorToComm, NULL, &threadPipeSensorToCommunication, arg2);
	//threadPID3=pthread_create(&threadUdpR, NULL, &threadUdpRead, &pipeArray1);
	threadPID4=pthread_create(&threadUdpW, NULL, &threadUdpWrite, NULL);
	threadPID5=pthread_create(&threadkeyRead, NULL, &threadKeyReading, &pipeArray1);
	
	// Set up thread scheduler priority for real time tasks
	struct sched_param paramThread1, paramThread2, paramThread4,paramThread5; // paramThread3
	paramThread1.sched_priority = PRIORITY_COMMUNICATION_PIPE_CONTROLLER; // set priorities
	paramThread2.sched_priority = PRIORITY_COMMUNICATION_PIPE_SENSOR;
	//paramThread3.sched_priority = PRIORITY_COMMUNICATION_UDP_READ;
	paramThread4.sched_priority = PRIORITY_COMMUNICATION_UDP_WRITE;
	paramThread5.sched_priority = PRIORITY_COMMUNICATION_KEYBOARD;
	
	if(sched_setscheduler(threadPID1, SCHED_FIFO, &paramThread1)==-1) {perror("sched_setscheduler failed for threadPID1");exit(-1);}
	if(sched_setscheduler(threadPID2, SCHED_FIFO, &paramThread2)==-1) {perror("sched_setscheduler failed for threadPID2");exit(-1);}
	//if(sched_setscheduler(threadPID3, SCHED_FIFO, &paramThread3)==-1) {perror("sched_setscheduler failed for threadPID3");exit(-1);}
	if(sched_setscheduler(threadPID4, SCHED_FIFO, &paramThread4)==-1) {perror("sched_setscheduler failed for threadPID3");exit(-1);}
	if(sched_setscheduler(threadPID5, SCHED_FIFO, &paramThread5)==-1) {perror("sched_setscheduler failed for threadPID3");exit(-1);}
	
	// If threads created successful, start them
	if (!threadPID1) pthread_join( threadPipeCtrlToComm, NULL);
	if (!threadPID2) pthread_join( threadPipeSensorToComm, NULL);
	//if (!threadPID3) pthread_join( threadUdpR, NULL);
	if (!threadPID4) pthread_join( threadUdpW, NULL);
	if (!threadPID5) pthread_join( threadkeyRead, NULL);
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
	
	/// Setup timer variables for real time performance check
	struct timespec t_start,t_stop;
	
	/// Average sampling
	int tsAverageCounter=0;
	double tsAverageAccum=0;
	double tsAverage=tsController, tsTrue;
	int timerPrint=0;
	
	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadPWMControl");
	}
	
	// Loop forever reading/waiting for data
	while(1){
		/// Time it
		clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock
		
		// Read data from controller process
		if(read(ptrPipe->parent[0], controllerDataBuffer, sizeof(controllerDataBuffer)) == -1) printf("read error in communication from controller\n");
		//else printf("Communication ID: %d, Recieved Controller data: %f\n", (int)getpid(), controllerDataBuffer[0]);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexControllerData);
			memcpy(controllerData, controllerDataBuffer, sizeof(controllerDataBuffer));
		pthread_mutex_unlock(&mutexControllerData);
		
		/// Print true sampling rate
		clock_gettime(CLOCK_MONOTONIC, &t_stop);
		tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
		//printf("Sampling time [s] PWM received: %lf\n",tsTrue);
		
		/// Get average sampling time
		if(tsAverageCounter<50){
			tsAverageAccum+=tsTrue;
			tsAverageCounter++;
		}
		else{
			tsAverageAccum/=50;
			tsAverage=tsAverageAccum;
			if(timerPrint){
				printf("Communication pipe from Controller Read: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
			}
			tsAverageCounter=0;
			tsAverageAccum=0;
			
		}
	}
	return NULL;
}


// Thread - Pipe Communication from Sensor read
static void *threadPipeSensorToCommunication(void *arg){
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	double sensorDataBuffer[19];
	
	/// Setup timer variables for real time performance check
	struct timespec t_start,t_stop;
	
	/// Average sampling
	int tsAverageCounter=0;
	double tsAverageAccum=0;
	double tsAverage=tsController, tsTrue;
	int timerPrint=0;
	
	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadPWMControl");
	}
	
	// Loop forever reading/waiting for data
	while(1){
		/// Time it
		clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock
		
		// Read data from sensor process
		if(read(ptrPipe->parent[0], sensorDataBuffer, sizeof(sensorDataBuffer)) == -1) printf("read error in communication from sensor\n");
		//else printf("Communication ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0]);
		
		//printf("% 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f % 1.3f\n", sensorDataBuffer[0],sensorDataBuffer[1], sensorDataBuffer[2], sensorDataBuffer[3], sensorDataBuffer[4], sensorDataBuffer[5], sensorDataBuffer[6], sensorDataBuffer[7], sensorDataBuffer[8], sensorDataBuffer[9], sensorDataBuffer[10], sensorDataBuffer[11], sensorDataBuffer[12], sensorDataBuffer[13], sensorDataBuffer[14], sensorDataBuffer[15], sensorDataBuffer[16], sensorDataBuffer[17], sensorDataBuffer[18]);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexSensorData);
			memcpy(sensorData, sensorDataBuffer, sizeof(sensorDataBuffer));
		pthread_mutex_unlock(&mutexSensorData);
		
		/// Print true sampling rate
		clock_gettime(CLOCK_MONOTONIC, &t_stop);
		tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
		//printf("Sampling time [s] PWM received: %lf\n",tsTrue);
		
		/// Get average sampling time
		if(tsAverageCounter<50){
			tsAverageAccum+=tsTrue;
			tsAverageCounter++;
		}
		else{
			tsAverageAccum/=50;
			tsAverage=tsAverageAccum;
			if(timerPrint){
				printf("Communication pipe from Sensor Read: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
			}
			tsAverageCounter=0;
			tsAverageAccum=0;
			
		}
	}
	return NULL;
}

/*
//// Thread - Pipe Communication to Controller write
//static void *threadPipeCommunicationtoController(void *arg)
//{
	//// Get pipe and define local variables
	//structPipe *ptrPipe = arg;
	//double sensorDataBuffer[19];
	
	//// Loop forever reading/waiting for data
	//while(1){
		//// Read data from sensor process
		////if(write(ptrPipe->parent[1], keyboardData, sizeof(keyboardData)) == -1) printf("Write error in keyboardData communication to controller\n");
		////else printf("Communication ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0]);

		
		//// Put new data in to global variable in communication.c
		//pthread_mutex_lock(&mutexSensorData);
			//memcpy(sensorData, sensorDataBuffer, sizeof(sensorDataBuffer));
		//pthread_mutex_unlock(&mutexSensorData);
		
		//sleep(1);
	//}
	//return NULL;
//}
*/


// UDP read thread
static void *threadUdpRead(void *arg){
	// Get pipe array and define local variables
	//pipeArray *pipeArray1 = arg;
	//structPipe *ptrPipe1 = pipeArray1->pipe1;
	//structPipe *ptrPipe2 = pipeArray1->pipe2;
	//float udpDataBuffer[6]={2,2,2,2,2,2};
	
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
static void *threadUdpWrite(){
	// Local variables
	double agentData[19];
		
	/// Setup timer variables for real time performance check
	struct timespec t, t_start,t_stop;
	
	/// Average sampling
	int tsAverageCounter=0;
	double tsAverageAccum=0;
	double tsAverage=tsUdpWrite, tsTrue;
	int timerPrint=0;
	
	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadPWMControl");
	}
	
	/// Wait 10 seconds before starting before starting
	t.tv_sec+=10;
	
	// Loop forever streaming data
	while(1){
		/// Time it
		clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // sleep for necessary time to reach desired sampling time
		
		if(socketReady){
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
			if (sendto(fdsocket_write, writeBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &addr_write, sizeof(addr_write)) == -1){
				perror("write");
			}
		}
		
		/// Calculate next shot
		t.tv_nsec += (int)tsUdpWrite;
		while (t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}	
		
		/// Print true sampling rate
		clock_gettime(CLOCK_MONOTONIC, &t_stop);
		tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
		//printf("Sampling time [s] UDP : %lf\n",tsTrue);
		
		/// Get average sampling time
		if(tsAverageCounter<50){
			tsAverageAccum+=tsTrue;
			tsAverageCounter++;
		}
		else{
			tsAverageAccum/=50;
			tsAverage=tsAverageAccum;
			if(timerPrint){
				printf("UDP Write: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
			}
			tsAverageCounter=0;
			tsAverageAccum=0;
			
		}

			
	}
	return NULL;
}


// Thread - reading from the keyboard from the stand-alone computer
static void *threadKeyReading( void *arg ) {
	// Get pipe and define local variables
	pipeArray *pipeArray1 = arg;
	structPipe *ptrPipe1 = pipeArray1->pipe1;
	structPipe *ptrPipe2 = pipeArray1->pipe2;
	
	/// Setup timer variables for real time performance check
	//struct timespec t_start,t_stop;
	
	/// Average sampling
	//int tsAverageCounter=0;
	//double tsAverageAccum=0;
	//double tsTrue; // tsAverage=tsController
	double keyboardDataController[72];
	//int timerPrint=0;
	
	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadPWMControl");
	}
	
	while(1) {
		///// Time it
		//clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock
		
		keyReading();
		
		memcpy(keyboardDataController, keyboardData, sizeof(keyboardData));
		memcpy(keyboardDataController+18, tuningMpcData, sizeof(tuningMpcData));
		memcpy(keyboardDataController+32, tuningMpcDataControl, sizeof(tuningMpcDataControl));
		memcpy(keyboardDataController+38, tuningEkfData, sizeof(tuningEkfData));
		memcpy(keyboardDataController+56, tuningPidData, sizeof(tuningPidData));
		memcpy(keyboardDataController+62, tuningMpcQfData, sizeof(tuningMpcQfData));
		memcpy(keyboardDataController+71, manualThrustData, sizeof(manualThrustData));
		
		//printf("%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f \n\n%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n", keyboardDataController[0], keyboardDataController[1], keyboardDataController[2], keyboardDataController[3], keyboardDataController[4], keyboardDataController[5], keyboardDataController[6], keyboardDataController[7], keyboardDataController[8],keyboardDataController[9], keyboardDataController[10], keyboardDataController[11], keyboardDataController[12], keyboardDataController[13], keyboardDataController[14], keyboardDataController[15], keyboardDataController[16], keyboardDataController[17],keyboardDataController[18], keyboardDataController[19], keyboardDataController[20], keyboardDataController[21], keyboardDataController[22], keyboardDataController[23], keyboardDataController[24], keyboardDataController[25], keyboardDataController[26], keyboardDataController[27], keyboardDataController[28], keyboardDataController[29], keyboardDataController[30], keyboardDataController[31], keyboardDataController[32], keyboardDataController[33], keyboardDataController[34], keyboardDataController[35], keyboardDataController[36], keyboardDataController[37], keyboardDataController[38], keyboardDataController[39], keyboardDataController[40], keyboardDataController[41], keyboardDataController[42], keyboardDataController[43], keyboardDataController[44], keyboardDataController[45], keyboardDataController[46], keyboardDataController[47], keyboardDataController[48], keyboardDataController[49]);
		
		// Write data to Controller process
		if (write(ptrPipe1->child[1], keyboardDataController, sizeof(keyboardDataController)) != sizeof(keyboardDataController) ) printf("Error in writing keyboardData from Communication to Controller\n");
		//else printf("Communication ID: %d, Sent: %f to Controller\n", (int)getpid(), keyboardData[0]);
	
		// Write data to Sensor process
		if (write(ptrPipe2->child[1], keyboardDataController, sizeof(keyboardDataController)) != sizeof(keyboardDataController)) printf("Error in writing keyboardData from Communication to Sensor\n");
		//else printf("Communication ID: %d, Sent: %f to Sensor\n", (int)getpid(), keyboardData[0]);
		
		///// Print true sampling rate
		//clock_gettime(CLOCK_MONOTONIC, &t_stop);
		//tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
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
	printf("Socket ready\n");
	socketReady=0;
}

/* Read in PWM value */
void keyReading( void ) {
	char input_char[50] = { '\0' };
	char selection[2] = { '\0' };
	double keyboardDataBuffer[4] = {0,0,0,0}; // {ref_x,ref_y,ref_z,switch}
	double tuningMpcBuffer[14];
	double tuningMpcQfBuffer[9];
	double tuningMpcBufferControl[6];
	double tuningEkfBuffer[18];
	double tuningPidBuffer[6];	
	//double manualThrustBuffer[1];
	
	memcpy(tuningMpcBuffer, tuningMpcData, sizeof(tuningMpcData));
	memcpy(tuningMpcQfBuffer, tuningMpcQfData, sizeof(tuningMpcQfData));
	memcpy(tuningMpcBufferControl, tuningMpcDataControl, sizeof(tuningMpcDataControl));
	memcpy(tuningEkfBuffer, tuningEkfData, sizeof(tuningEkfData));
	memcpy(tuningPidBuffer, tuningPidData, sizeof(tuningPidData));	
	//memcpy(manualThrustBuffer, manualThrustData, sizeof(manualThrustData));
	
	char *pt;
	int counter=0;
	int tuningFlag=1;
	
	printf("Keyboard listening... \n");
	scanf("%s", selection);
	//printf("I read-> %s \n", selection);
	
	switch( selection[0] ) {
		case 'a' :
			printf("Tell me your alpha:\n");
			
			scanf("%s", &input_char[0]);
			if ( strcmp(&input_char[0], "x" ) == 0 ) { printf("Aborting\n"); break; }
			keyboardData[11] = atof(&input_char[0]);
			printf("alpha ->  %f\n", keyboardData[11]);
			break;	
			
		case 'b' :
			printf("Tell me your beta:\n");
			
			scanf("%s", &input_char[0]);
			if ( strcmp(&input_char[0], "x" ) == 0 ) { printf("Aborting\n"); break; }
			keyboardData[12] = atof(&input_char[0]);
			printf("beta ->  %f\n", keyboardData[12]);
			break;	
			
		
		case 'r' :
			printf("Tell me your references:\n");
			
			scanf("%s", &input_char[0]);
			if ( strcmp(&input_char[0], "x" ) == 0 ) { printf("Aborting\n"); break; }
			keyboardDataBuffer[0] = atof(&input_char[0]);
			//printf("ref X  ->  %f\n", keyboardData[0]);
			
			scanf("%s", &input_char[1]);
			if ( strcmp(&input_char[1], "x" ) == 0 ) { printf("Aborting\n"); break; }
			keyboardDataBuffer[1] = atof(&input_char[1]);
			//printf("ref Y  ->  %f\n", keyboardData[1]);
			
			scanf("%s", &input_char[2]);
			if ( strcmp(&input_char[2], "x" ) == 0 ) { printf("Aborting\n"); break; }
			keyboardDataBuffer[2] = atof(&input_char[2]);
			//printf("ref Z  ->  %f\n", keyboardData[2]);
			
			printf("X = %f, Y = %f and Z = %f,  [y]es or [n]o?\n", keyboardData[0], keyboardData[1], keyboardData[2]);
			scanf("%s", selection);
			if ( strcmp(selection, "x" ) == 0 ) { printf("Aborting\n"); break; }
			if ( strcmp(selection, "y" ) == 0 ) {
				//pthread_mutex_lock(&mutexSensorData);
					memcpy(keyboardData, keyboardDataBuffer, sizeof(keyboardDataBuffer)*3/4);
				//pthread_mutex_unlock(&mutexSensorData);
				printf("Updated! X = %f, Y = %f, Z = %f and switch is %f\n", keyboardData[0], keyboardData[1], keyboardData[2], keyboardData[3]);
				printf("Do you also wanna ramp them?\n");
				scanf("%s", selection);
				if ( strcmp(selection, "y" ) == 0 ) { 
					keyboardData[10] = 1; 
					printf("Ramped\n");
				}
				else {
					keyboardData[10] = 0; 
					printf("NOT Ramped\n");
				}
			}
			else {
				printf("Discarded! X = %f, Y = %f, Z = %f and switch is %f\n", keyboardData[0], keyboardData[1], keyboardData[2], keyboardData[3]);
			}
			
			break;
			
		case 's' :
			if (keyboardData[3] == 0) {
				printf("It is already STOP you idiot!\n");
			}
			else if ( keyboardData[3] == 1 ) {
				//pthread_mutex_lock(&mutexSensorData);
					keyboardData[3] = 0;
					//keyboardDataBuffer[3] = 0;
				//pthread_mutex_unlock(&mutexSensorData);
				printf("Set to STOP now!\n");
				keyboardData[14] = 0;
				printf(" Feed forward attitude MPC not active any more: %i\n", (int)keyboardData[14]);
			}
			break;
			
		case 'f' :
			if (keyboardData[3] == 0) {
				printf("It is STOP, wanna FLY it, [y]es or [n]o?\n");
				scanf("%s", selection);
				if ( strcmp(selection, "x" ) == 0 ) { printf("Aborting\n"); break; }
				if ( strcmp(selection, "y" ) == 0 ) {
				//pthread_mutex_lock(&mutexSensorData);
					keyboardData[3] = 1;
					//keyboardDataBuffer[3] = 1;
				//pthread_mutex_unlock(&mutexSensorData);
				printf("Set to FLY now!\n");
				}
				else {
					printf("Kept STOP!\n");
				}
			}
			else if ( keyboardData[3] == 1 ) {
				//printf("It is already FLY idiot!\n");
				printf("Feed forward attitude MPC active, [y]es or [n]o?\n");
				scanf("%s", selection);
				if ( strcmp(selection, "x" ) == 0 ){
					keyboardData[14] = 0;
					printf(" Feed forward attitude MPC not active: %i\n", (int)keyboardData[14]);
					break;
				}
				if ( strcmp(selection, "y" ) == 0 ){
					keyboardData[14] = 1;
					printf(" Feed forward attitude MPC active: %i\n", (int)keyboardData[14]);
				}
				else {
					printf("No selection. Feed forward status: %i. Aborting\n", (int)keyboardData[14]);
					break;
				}
			}
			break;
		
		// Use this for manual thrust setting instead, for now!	
		//case 'i' :
				//printf("X = %f, Y = %f, Z = %f and switch is %f\n", keyboardData[0], keyboardData[1], keyboardData[2], keyboardData[3]);
			//break;

		case 'p' :
			if (keyboardData[4]==0){
				keyboardData[4]=1;
				printf("PWM print toggle: %i\n", (int)keyboardData[4]);
			}
			else if(keyboardData[4]==1){
				keyboardData[4]=0;
				printf("PWM print toggle: %i\n", (int)keyboardData[4]);
			}
		break;
		
		case 'd' :
			if (keyboardData[15]==0){
				keyboardData[15]=1;
				printf("Data save toggle: %i\n", (int)keyboardData[15]);
			}
			else if(keyboardData[15]==1){
				keyboardData[15]=0;
				printf("Data save toggle: %i\n", (int)keyboardData[15]);
			}
		break;
		
		case 't' :
			if (keyboardData[5]==0){
				keyboardData[5]=1;
				printf("Timer print toggle: %i\n", (int)keyboardData[5]);
			}
			else if(keyboardData[5]==1){
				keyboardData[5]=0;
				printf("Timer print toggle: %i\n", (int)keyboardData[5]);
			}
		break;
		
		case 'e' :
			if (keyboardData[6]==0){
				keyboardData[6]=1;
				printf("EKF print toggle: %i\n", (int)keyboardData[6]);
			}
			else if(keyboardData[6]==1){
				keyboardData[6]=0;
				printf("EKF print toggle: %i\n", (int)keyboardData[6]);
			}
		break;
				
		case 'n' :
			if (keyboardData[7]==0){
				keyboardData[7]=1;
				printf("Reset EKF/MPC toggle: %i. Set EKF/MPC toggle back to restart\n", (int)keyboardData[7]);
				if ( keyboardData[3] == 1 ) {
					keyboardData[3] = 0;
					printf("Set to STOP now!\n");
				}
			}
			else if(keyboardData[7]==1){
				keyboardData[7]=0;
				printf("Reset EKF/MPC toggle: %i. EKF restarted. MPC ready when fly is commanded\n", (int)keyboardData[7]);
			}
		break;
		
		case 'w' :
			if (keyboardData[8]==0){
				keyboardData[8]=1;
				printf("EKF print 6 states toggle: %i\n", (int)keyboardData[8]);
			}
			else if(keyboardData[8]==1){
				keyboardData[8]=0;
				printf("EKF print 6 states toggle: %i\n", (int)keyboardData[8]);
			}
		break;

		
		case 'c' : // sensor.c redo calibration function
			if (keyboardData[9] == 0) {
				printf("Do you want to recalibrate sensor fusion and EKF? [y]es or [n]o?\n");
				scanf("%s", selection);
				if ( strcmp(selection, "x" ) == 0 ) { printf("Aborting\n"); break; }
				if ( strcmp(selection, "y" ) == 0 ) {
				//pthread_mutex_lock(&mutexSensorData);
					keyboardData[9] = 1;
					//keyboardDataBuffer[3] = 1;
				//pthread_mutex_unlock(&mutexSensorData);
				printf("Trying to restart calibration! Remember to deactivate this keyboard command when calibration has been confirmed started.\n");
				}
				else {
					printf("Keeping old calibration data\n");
				}
			}
			else if ( keyboardData[9] == 1 ) {
				keyboardData[9] = 0;
				printf("Restart calibration command deactivated. Calibration will finish if it actually started!\n");
			}
		break;
		
		// MPC tuning from keyboard		
		case 'm' :
			printf("\n [p]osition mpc\n [a]ttitude mpc\n [v]altitude mpc\n [c]urrent tuning values set\n [x]exit\n");
			scanf("%s", selection);
			// position tuning
			if ( strcmp(selection, "p" ) == 0 ) {
				while (tuningFlag){
					printf(" [q] state weights\n [r] control weights\n [t]oggle MPC on/off");
					scanf("%s", selection);
					// State weights
					if( strcmp(selection, "q" ) == 0 ){
						printf("Position MPC Q {x,xdot,y,ydot,xform,yform}\n Old: {%f,%f,%f,%f,%f,%f}\n New: ", tuningMpcData[0], tuningMpcData[1], tuningMpcData[2], tuningMpcData[3], tuningMpcData[4], tuningMpcData[5]);
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcBuffer[counter]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=6){
								printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<6;i++){
									tuningMpcData[i]=tuningMpcBuffer[i];
								}
								printf("Updated: {%f,%f,%f,%f,%f,%f}\n", tuningMpcData[0], tuningMpcData[1], tuningMpcData[2], tuningMpcData[3], tuningMpcData[4], tuningMpcData[5]);
							}
							tuningFlag=0;
						}
					}
					
					// Control weights
					else if( strcmp(selection, "r" ) == 0 ){
						printf("Position MPC R {theta_ref, phi_ref}\n Old: {%f,%f}\n New: ", tuningMpcDataControl[0], tuningMpcDataControl[1]); // {pos,pos,taux,tauy,tauz,alt}
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcBufferControl[counter]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=2){
							printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<2;i++){
									tuningMpcDataControl[i]=tuningMpcBufferControl[i];
								}
								printf("Updated: {%f,%f}\n", tuningMpcDataControl[0], tuningMpcDataControl[1]);
							}
							tuningFlag=0;
						}
					}
					
					// Toggle MPC on/off
					else if( strcmp(selection, "t" ) == 0 ){
						if (keyboardData[13]==0){
							keyboardData[13]=1;
							printf("MPC position toggle: %i\n", (int)keyboardData[13]);
						}
						else if(keyboardData[13]==1){
							keyboardData[13]=0;
							printf("MPC position toggle: %i\n", (int)keyboardData[13]);
						}
					}
					else{
						break;
					}
					
				}
				break;
			}
			
			// attitude tuning
			else if ( strcmp(selection, "a" ) == 0 ) {
				while (tuningFlag){
					printf(" [q] state weights\n [r] control weights\n [f] qf weights\n [p]id control gains\n [t] pid control toggle\n");
					scanf("%s", selection);
					// State weights
					if( strcmp(selection, "q" ) == 0 ){
						printf("Attitude MPC Q {phi,phidot,theta,thetadot,psi,psidot}\n Old: {%f,%f,%f,%f,%f,%f}\n New: ", tuningMpcData[6], tuningMpcData[7], tuningMpcData[8], tuningMpcData[9], tuningMpcData[10], tuningMpcData[11]);
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcBuffer[counter+6]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=6){
								printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<6;i++){
									tuningMpcData[i+6]=tuningMpcBuffer[i+6];
								}
								printf("Updated: {%f,%f,%f,%f,%f,%f}\n", tuningMpcData[6], tuningMpcData[7], tuningMpcData[8], tuningMpcData[9], tuningMpcData[10], tuningMpcData[11]);
							}
							tuningFlag=0;
						}
					} // "Updated: {%7.15f,%7.15f,%7.15f,%7.15f,%7.15f,%7.15f}\n"
					
					// Control terminal weights
					else if( strcmp(selection, "f" ) == 0 ){
						printf("Attitude MPC Qf {phi,phidot,theta,thetadot,psi,psidot,phi-phidot,theta-thetadot,psi-psidot}\n Old: {%f,%f,%f,%f,%f,%f,%f,%f,%f}\n New: ", tuningMpcQfData[0], tuningMpcQfData[1], tuningMpcQfData[2], tuningMpcQfData[3], tuningMpcQfData[4], tuningMpcQfData[5], tuningMpcQfData[6], tuningMpcQfData[7], tuningMpcQfData[8]);
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcQfBuffer[counter+9]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=9){
								printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<9;i++){
									tuningMpcQfData[i+9]=tuningMpcQfBuffer[i+9];
								}
								printf("Updated: {%f,%f,%f,%f,%f,%f,%f,%f,%f}\n", tuningMpcQfData[0], tuningMpcQfData[1], tuningMpcQfData[2], tuningMpcQfData[3], tuningMpcQfData[4], tuningMpcQfData[5], tuningMpcQfData[6], tuningMpcQfData[7], tuningMpcQfData[8]);
							}
							tuningFlag=0;
						}
					} // "Updated: {%7.15f,%7.15f,%7.15f,%7.15f,%7.15f,%7.15f}\n"
					
					// Control weights
					else if( strcmp(selection, "r" ) == 0 ){
						printf("Attitude MPC R {taux,tauy,tauz}\n Old: {%f,%f,%f}\n New: ", tuningMpcDataControl[2], tuningMpcDataControl[3], tuningMpcDataControl[4]); // {pos,pos,taux,tauy,tauz,alt}
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcBufferControl[counter+2]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=3){
								printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<3;i++){
									tuningMpcDataControl[i+2]=tuningMpcBufferControl[i+2];
								}
								printf("Updated: {%f,%f,%f}\n", tuningMpcDataControl[2], tuningMpcDataControl[3], tuningMpcDataControl[4]);
							}
							tuningFlag=0;
						}
					}
					
					// Toggle PID(MPC) on/off
					else if( strcmp(selection, "t" ) == 0 ){
						if (keyboardData[16]==0){
							keyboardData[16]=1;
							printf("PID attitude control active. MPC not active. Toggle: %i\n", (int)keyboardData[16]);
							printf("Manual thrust, [y]es or [n]o?\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								manualThrustData[0]=0;
								printf("Manual thrust is ON\n");
							}
							else{
								manualThrustData[0]=-1;
								printf("Manual thrust is OFF\n");
							}
						}
						else if(keyboardData[16]==1){
							keyboardData[16]=0;
							printf("MPC attitude control active. PID not active. Toggle: %i\n", (int)keyboardData[16]);
						}
						break;
					}
					
					// PID tuning gains
					else if ( strcmp(selection, "p" ) == 0 ) {
						while (tuningFlag){
							printf("PID gains attitude {gyro_kp,gyro_ki,gyro_kd,angle_kp,angle_ki,angle_kd}\n Old: {%f,%f,%f,%f,%f,%f}\n New: ", tuningPidData[0], tuningPidData[1], tuningPidData[2], tuningPidData[3], tuningPidData[4], tuningPidData[5]);
							scanf("%s", input_char);
							pt = strtok(input_char, ",");
							while (pt != NULL){
								tuningPidBuffer[counter]=atof(pt);
								pt = strtok(NULL, ",");
								counter++;
							}
							if (counter!=6){
								printf("Bad format. Retry [y]?  Else press any button to cancel\n");
								scanf("%s", selection);
								if ( strcmp(selection, "y" ) == 0 ){
									counter=0;
									tuningFlag=1;
								}
								else{
									tuningFlag=0;
								}
							}
							else {
								printf("\nAccept [y]? Else press any button to cancel\n");
								scanf("%s", selection);
								if ( strcmp(selection, "y" ) == 0 ){
									for (int i=0;i<6;i++){
										tuningPidData[i]=tuningPidBuffer[i];
									}
									printf("Updated: {%f,%f,%f,%f,%f,%f}\n", tuningPidData[0], tuningPidData[1], tuningPidData[2], tuningPidData[3], tuningPidData[4], tuningPidData[5]);
								}
								tuningFlag=0;
							}			
						}
						break;
					}
					
					else{
						break;
					}
				}
			}
			
			// altitude tuning
			else if ( strcmp(selection, "v" ) == 0 ) {
				while (tuningFlag){
					printf(" [q]state weights\n [r]control weights\n");
					scanf("%s", selection);
					// State weights
					if( strcmp(selection, "q" ) == 0 ){		
						printf("Altitude MPC Q {z,zdot}\n Old: {%f,%f}\n New: ", tuningMpcData[12], tuningMpcData[13]);
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcBuffer[counter+12]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=2){
								printf("Bad format. Retry [y]?  Else press any button to cancel\n");
								scanf("%s", selection);
								if ( strcmp(selection, "y" ) == 0 ){
									counter=0;
									tuningFlag=1;
								}
								else{
									tuningFlag=0;
								}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<2;i++){
									tuningMpcData[i+12]=tuningMpcBuffer[i+12];
								}
								printf("Updated: {%f,%f}\n", tuningMpcData[12], tuningMpcData[13]);
							}
							tuningFlag=0;
						}
					}
					
					// Control weights
					else if( strcmp(selection, "r" ) == 0 ){
						printf("Altitude MPC R {thrust}\n Old: {%f}\n New: ", tuningMpcDataControl[5]); // {pos,pos,taux,tauy,tauz,alt}
						scanf("%s", input_char);
						pt = strtok(input_char, ",");
						while (pt != NULL){
							tuningMpcBufferControl[counter+5]=atof(pt);
							pt = strtok(NULL, ",");
							counter++;
						}
						if (counter!=1){
							printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
						}
						else {
							printf("\nAccept [y]? Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								for (int i=0;i<1;i++){
									tuningMpcDataControl[i+5]=tuningMpcBufferControl[i+5];
								}
								printf("Updated: {%f}\n", tuningMpcDataControl[5]);
							}
							tuningFlag=0;
						}
					}
					else{
						break;
					}
				}
			}
			
			// show current tuning values
			else if ( strcmp(selection, "c" ) == 0 ) {			
				printf("Current Q {x,xdot,y,ydot,xform,yform,phi,phidot,theta,thetadot,psi,psidot,z,zdot}\n{%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f}\n", tuningMpcData[0], tuningMpcData[1], tuningMpcData[2], tuningMpcData[3], tuningMpcData[4], tuningMpcData[5], tuningMpcData[6], tuningMpcData[7], tuningMpcData[8], tuningMpcData[9], tuningMpcData[10], tuningMpcData[11], tuningMpcData[12], tuningMpcData[13]);
				printf("Current R {theta_ref,phi_ref,taux,tauy,tauz,thrust}\n{%f,%f,%f,%f,%f,%f}\n", tuningMpcDataControl[0], tuningMpcDataControl[1], tuningMpcDataControl[2], tuningMpcDataControl[3], tuningMpcDataControl[4], tuningMpcDataControl[5]);
				
				break;
			}
			
			else{ printf("Aborting\n"); break; }
		
		break;
		
		// EKF tuning (Q)
		case 'q' :
			printf("\n [p]osition ekf\n [a]ttitude ekf\n [d]isturbance ekf\n [c]urrent ekf Q values set\n [x]exit\n");
			scanf("%s", selection);
			// position tuning
			if ( strcmp(selection, "p" ) == 0 ) {
				while (tuningFlag){
					printf("Position ekf Q {x,y,z,xdot,ydot,zdot}\n Old: {%f,%f,%f,%f,%f,%f}\n New: ", tuningEkfData[0], tuningEkfData[1], tuningEkfData[2], tuningEkfData[3], tuningEkfData[4], tuningEkfData[5]);
					scanf("%s", input_char);
					pt = strtok(input_char, ",");
					while (pt != NULL){
						tuningEkfBuffer[counter]=atof(pt);
						pt = strtok(NULL, ",");
						counter++;
					}
					if (counter!=6){
						printf("Bad format. Retry [y]?  Else press any button to cancel\n");
						scanf("%s", selection);
						if ( strcmp(selection, "y" ) == 0 ){
							counter=0;
							tuningFlag=1;
						}
						else{
							tuningFlag=0;
						}
					}
					else {
						printf("\nAccept [y]? Else press any button to cancel\n");
						scanf("%s", selection);
						if ( strcmp(selection, "y" ) == 0 ){
							for (int i=0;i<6;i++){
								tuningEkfData[i]=tuningEkfBuffer[i];
							}
							printf("Updated: {%f,%f,%f,%f,%f,%f}\n", tuningEkfData[0], tuningEkfData[1], tuningEkfData[2], tuningEkfData[3], tuningEkfData[4], tuningEkfData[5]);
						}
						tuningFlag=0;
					}			
				}
				break;
			}
			
			// attitude tuning
			else if ( strcmp(selection, "a" ) == 0 ) {
				while (tuningFlag){
					printf("Attitude ekf Q {phi,theta,psi,phidot,thetadot,psidot}\n Old: {%f,%f,%f,%f,%f,%f}\n New: ", tuningEkfData[6], tuningEkfData[7], tuningEkfData[8], tuningEkfData[9], tuningEkfData[10], tuningEkfData[11]);
					scanf("%s", input_char);
					pt = strtok(input_char, ",");
					while (pt != NULL){
						tuningEkfBuffer[counter+6]=atof(pt);
						pt = strtok(NULL, ",");
						counter++;
					}
					if (counter!=6){
						printf("Bad format. Retry [y]?  Else press any button to cancel\n");
						scanf("%s", selection);
						if ( strcmp(selection, "y" ) == 0 ){
							counter=0;
							tuningFlag=1;
						}
						else{
							tuningFlag=0;
						}
					}
					else {
						printf("\nAccept [y]? Else press any button to cancel\n");
						scanf("%s", selection);
						if ( strcmp(selection, "y" ) == 0 ){
							for (int i=0;i<6;i++){
								tuningEkfData[i+6]=tuningEkfBuffer[i+6];
							}
							printf("Updated: {%f,%f,%f,%f,%f,%f}\n", tuningEkfData[6], tuningEkfData[7], tuningEkfData[8], tuningEkfData[9], tuningEkfData[10], tuningEkfData[11]);
						}
						tuningFlag=0;
					}
				}
				break;
			}
			
			// disturbance tuning
			else if ( strcmp(selection, "d" ) == 0 ) {
				while (tuningFlag){
					printf("Disturbance ekf Q {dist_x,dist_y,dist_z,dist_taux,dist_tauy,dist_tauz}\n Old: {%f,%f,%f,%f,%f,%f}\n New: ", tuningEkfData[12], tuningEkfData[13], tuningEkfData[14], tuningEkfData[15], tuningEkfData[16], tuningEkfData[17]);
					scanf("%s", input_char);
					pt = strtok(input_char, ",");
					while (pt != NULL){
						tuningEkfBuffer[counter+12]=atof(pt);
						pt = strtok(NULL, ",");
						counter++;
					}
					if (counter!=6){
							printf("Bad format. Retry [y]?  Else press any button to cancel\n");
							scanf("%s", selection);
							if ( strcmp(selection, "y" ) == 0 ){
								counter=0;
								tuningFlag=1;
							}
							else{
								tuningFlag=0;
							}
					}
					else {
						printf("\nAccept [y]? Else press any button to cancel\n");
						scanf("%s", selection);
						if ( strcmp(selection, "y" ) == 0 ){
							for (int i=0;i<6;i++){
								tuningEkfData[i+12]=tuningEkfBuffer[i+12];
							}
							printf("Updated: {%f,%f,%f,%f,%f,%f}\n", tuningEkfData[12], tuningEkfData[13], tuningEkfData[14], tuningEkfData[15], tuningEkfData[16], tuningEkfData[17]);
						}
						tuningFlag=0;
					}
				}
				break;
			}
		
			// show current tuning values
			else if ( strcmp(selection, "c" ) == 0 ) {			
				printf("Current ekf Q {x,y,z,xdot,ydot,zdot,phi,theta,psi,phidot,thetadot,psidot,dist_x,dist_y,dist_z,dist_taux,dist_tauy,dist_tauz}\n{%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f}\n", tuningEkfData[0], tuningEkfData[1], tuningEkfData[2], tuningEkfData[3], tuningEkfData[4], tuningEkfData[5], tuningEkfData[6], tuningEkfData[7], tuningEkfData[8], tuningEkfData[9], tuningEkfData[10], tuningEkfData[11], tuningEkfData[12], tuningEkfData[13], tuningEkfData[14], tuningEkfData[15], tuningEkfData[16], tuningEkfData[17]);			
				break;
			}
			
			else{ printf("Aborting\n"); break; }

		case 'v' :
			if (keyboardData[17]==0){
				printf("Motor PWM range settings status: %i. Start by [y] or cancel by [x]\n", (int)keyboardData[17]);
				scanf("%s", selection);
				if ( strcmp(selection, "x" ) == 0 ) { printf("Aborting\n"); break; }
				if ( strcmp(selection, "y" ) == 0 ) {
					printf("Power off speed controllers. Continue [y] or [x]?\n");
					scanf("%s", selection);
					if ( strcmp(selection, "x" ) == 0 ) { printf("Aborting\n"); break; }
					if ( strcmp(selection, "y" ) == 0 ) {
						printf("PWM is set to 100. Power on speed controllers. Confirming sound should be made.\n");
						
						keyboardData[17]=1;
						printf("Motor PWM range setting status: %i. Toggle [v] again to deactive and finish adjustments.\n", (int)keyboardData[17]);
						break;
					}
									
					
				}
			}
			
			else if(keyboardData[17]==1){
				printf("Motor PWM range settings status: %i. Stop by [y] or cancel by [x]\n", (int)keyboardData[17]);
				scanf("%s", selection);
				if ( strcmp(selection, "x" ) == 0 ) { printf("Aborting. Motor PWM range stil active\n"); break; }
				if ( strcmp(selection, "y" ) == 0 ) {
					keyboardData[17]=0;
					printf("PWM is set to 0. Confirming sound should be made.\n");
					printf("Motor PWM range setting status: %i. Adjustment finish.\n", (int)keyboardData[17]);
					break;
				}
			}
			break;
			
		case 'o' :
			if (manualThrustData[0] >= 0) {
				manualThrustData[0] += 1;
				printf("UP 1 - thrust = %f\n", manualThrustData[0]);
			}
			else {
				printf("Manual thrust is switched OFF!\n");
			}
		break;
			
		case 'l' :
			if (manualThrustData[0] >= 0) {
				manualThrustData[0] -= 1;
				printf("DOWN 1 - thrust = %f\n", manualThrustData[0]);
			}
			else {
				printf("Manual thrust is switched OFF!\n");
			}
		break;

		case 'i' :
			if (manualThrustData[0] >= 0) {
				manualThrustData[0] += 0.1;
				printf("UP 01 - thrust = %f\n", manualThrustData[0]);
			}
			else {
				printf("Manual thrust is switched OFF!\n");
			}
		break;
				
		case 'k' :
			if (manualThrustData[0] >= 0) {
				manualThrustData[0] -= 0.1;
				printf("DOWN 01 - thrust = %f\n", manualThrustData[0]);
			}
			else {
				printf("Manual thrust is switched OFF!\n");
			}
		break;
						
		case 'u' :
			if (manualThrustData[0] >= 0) {
				manualThrustData[0] += 0.01;
				printf("UP 001 - thrust = %f\n", manualThrustData[0]);
			}
			else {
				printf("Manual thrust is switched OFF!\n");
			}
		break;
				
		case 'j' :
			if (manualThrustData[0] >= 0) {
				manualThrustData[0] -= 0.01;
				printf("DOWN 001 - thrust = %f\n", manualThrustData[0]);
			}
			else {
				printf("Manual thrust is switched OFF!\n");
			}
		break;
						
					
		case 'h' :
			printf("\n [r]eferences - Sets the references\n [s]top - Sets the switch to 0 and stops it hopefully!\n [f]ly - Set the switch to 1!\t [f]eed forward - attitude mpc\n [i]nfo - Shows all the references and the switch\n [h]elp - Shows this again!\n [x] Aborts at every reading!\n [p]wm - Print PWM in terminal by toggle on/off\n [t]timers - Print average real time by toggle on/off\n [e]kf - Print EKF xhat (states, inertias and disturbances) by toggle on/off\n [w]ekf 6 states - Print EKF xhat (reference states) by toggle on/off\n [n]ew try - Reset EKF and MPC by toggle on/off\n [c]alibrate sensor fusion and EKF - Redo calibration\n [a]lpha magnetometer outlier forgetting factor\n [b]eta Madgwick Filter gain\n [m]pc settings\n [q]ekf settings\n [d]ata save to file\n [v] motor pwm range settings\n");
			break;
				
		default :
			printf("Invalid try again\n");
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
