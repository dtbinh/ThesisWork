// SENSOR FUSION CODE

#include "sensor.h"
#include "startup.h"
#include "PWM.h"
#include "lapack.h"
#include "blas.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include "mInv.h"

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <errno.h>
#include <math.h>
#include <errno.h>

// PREEMPT_RT
// #include <time.h>
#include <sched.h>
#include <sys/mman.h>

#define PI 3.141592653589793
#define CALIBRATION 1000
#define BUFFER 100

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadReadBeacon (void*);
static void *threadSensorFusion (void*);
static void *threadPWMControl (void*);
static void *threadPipeCommunicationToSensor(void*);

void qNormalize(double*);
void q2euler(double*, double*);
void q2euler_zyx(double *, double *);

void ekfCalibration6x6(double*, double*, double*, double*, int);
void ekfCalibration9x9_bias(double*, double*, double*, double*, int);
void ekfCalibration9x9(double*, double*, double*, double*, int);

void lowPassFilter(double *, double*, double *, double* , double* , double* );

int loadSettings(double*, char*, int);
//void saveSettings(double*, char*, int, FILE**);
void saveSettings(double*, char*, int);
//void saveData(double*, char* , int, FILE**, int);
void saveData(double*, char* , int);
void printBits(size_t const, void const * const);


void EKF_6x6(double*, double*, double*, double*, double*, double*, double);
void EKF_9x9(double*, double*, double*, double*, double*, double*, double, int, double*);
void EKF_9x9_bias(double*, double*, double*, double*, double*, double*, double);


void fx_6x1(double*, double*, double*, double);
void Jfx_6x6(double*, double*, double*, double);
void fx_9x1(double*, double*, double*, double, double*);
void Jfx_9x9(double*, double*, double*, double, double*);
void fx_9x1_bias(double*, double*, double*, double);
void Jfx_9x9_bias(double*, double*, double*, double);
void saturation(double*, int, double, double);

// Static variables for threads
static double sensorRawDataPosition[3]={0,0,0}; // Global variable in sensor.c to communicate between IMU read and angle fusion threads
static double controlData[8]={.1,.1,.1,.1,0,0,0,0}; // Global variable in sensor.c to pass control signal u from controller.c to EKF in sensor fusion {pwm0,pwm1,pwm2,pwm3,thrust,taux,tauy,tauz};
static double keyboardData[18]= { 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0.01, 0.05, 0, 0, 0, 0, 0}; // {ref_x,ref_y,ref_z, switch [0=STOP, 1=FLY], PWM print, Timer print, EKF print, reset ekf/mpc, EKF print 6 states, restart calibration, ramp ref, alpha, beta, mpc position toggle, ff toggle mpAtt, save data, PID trigger, PWM range setting}
static double tuningEkfData[18]={ekf_Q_1,ekf_Q_2,ekf_Q_3,ekf_Q_4,ekf_Q_5,ekf_Q_6,ekf_Q_7,ekf_Q_8,ekf_Q_9,ekf_Q_10,ekf_Q_11,ekf_Q_12,ekf_Q_13,ekf_Q_14,ekf_Q_15,ekf_Q_16,ekf_Q_17,ekf_Q_18};

// Variables
static int beaconConnected=0;

// Mutexes
static pthread_mutex_t mutexPositionSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexI2CBusy = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexControlData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexKeyboardData = PTHREAD_MUTEX_INITIALIZER;


/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *arg1, void *arg2){
	// Create pipe array
	pipeArray pipeArrayStruct = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create thread
	pthread_t threadSenFus, threadPWMCtrl, threadCommToSens; // threadReadPos,
	int threadPID2, threadPID3, threadPID4; //t hreadPID1, 
	
	//threadPID1=pthread_create(&threadReadPos, NULL, &threadReadBeacon, NULL);
	threadPID2=pthread_create(&threadSenFus, NULL, &threadSensorFusion, &pipeArrayStruct);
	threadPID3=pthread_create(&threadPWMCtrl, NULL, &threadPWMControl, arg1);
	threadPID4=pthread_create(&threadCommToSens, NULL, &threadPipeCommunicationToSensor, arg2);
	
	// Set up thread scheduler priority for real time tasks
	struct sched_param paramThread2, paramThread3, paramThread4; // paramThread1, 
	//paramThread1.sched_priority = PRIORITY_SENSOR_BEACON; // set priorities
	paramThread2.sched_priority = PRIORITY_SENSOR_FUSION;
	paramThread3.sched_priority = PRIORITY_SENSOR_PWM;
	paramThread4.sched_priority = PRIORITY_SENSOR_PIPE_COMMUNICATION;
	//if(sched_setscheduler(threadPID1, SCHED_FIFO, &paramThread1)==-1) {perror("sched_setscheduler failed for threadPID1");exit(-1);}
	if(sched_setscheduler(threadPID2, SCHED_FIFO, &paramThread2)==-1) {perror("sched_setscheduler failed for threadPID2");exit(-1);}
	if(sched_setscheduler(threadPID3, SCHED_FIFO, &paramThread3)==-1) {perror("sched_setscheduler failed for threadPID3");exit(-1);}
	if(sched_setscheduler(threadPID4, SCHED_FIFO, &paramThread4)==-1) {perror("sched_setscheduler failed for threadPID3");exit(-1);}
	
	// If threads created successful, start them
	//if (!threadPID1) pthread_join( threadReadPos, NULL);
	if (!threadPID2) pthread_join( threadSenFus, NULL);
	if (!threadPID3) pthread_join( threadPWMCtrl, NULL);
	if (!threadPID4) pthread_join( threadCommToSens, NULL);
}


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Commnication.c to Sensor.c with keyobard inputs
static void *threadPipeCommunicationToSensor(void *arg){
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	double communicationDataBuffer[72];
	double keyboardDataBuffer[18];
	double tuningEkfBuffer[18];
	
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
		if(read(ptrPipe->child[0], communicationDataBuffer, sizeof(communicationDataBuffer)) == -1) printf("read error in sensor from communication\n");
		//else printf("Sensor ID: %d, Recieved Communication data: %f\n", (int)getpid(), keyboardDataBuffer[0]);
				
		memcpy(keyboardDataBuffer, communicationDataBuffer, sizeof(communicationDataBuffer)*18/71);
		memcpy(tuningEkfBuffer, communicationDataBuffer+38, sizeof(communicationDataBuffer)*18/71);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexKeyboardData);
			memcpy(keyboardData, keyboardDataBuffer, sizeof(keyboardDataBuffer));
			memcpy(tuningEkfData, tuningEkfBuffer, sizeof(tuningEkfBuffer));
			timerPrint=(int)keyboardData[5];
		pthread_mutex_unlock(&mutexKeyboardData);
		
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
				printf("Sensor pipe from Communication Read: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
			}
			tsAverageCounter=0;
			tsAverageAccum=0;
			
		}
	}
	return NULL;
}

// Thread - Read position values
void *threadReadBeacon (void *arg){
	// Define local variables
	double posRaw[3], tsTrue=tsReadBeacon;
	//double beaconTimestamp = 0.0;
	//double beaconTimestampPrev = 0.0;
	uint8_t data8[100];	//this must be number of beacons*14+6+2 at least!
	uint16_t data16;
	uint32_t data32;
	//double beacons[16];	//each beacon needs 4 doubles, address and XYZ
	int n;
	//double hedgehogReading[1] = {0};
	int beaconFlag = 0;
	//int beaconsCoordinates = 0;
	//double dummy = 0.0;
	int fdBeacon;
	
	/// Setup timer variables for real time
	struct timespec t,t_start,t_stop;

	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadSensorFusion");
	}
	
	/// Start after 1 second
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	
	// Loop forever trying to connect to Beacon sensor via USB
	while(1){
		// Open serial communication
		if ((fdBeacon=serialOpen("/dev/ttyACM0", 115200)) < 0){
			//fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		}
		else{
			// Activate wiringPiSetup
			if (wiringPiSetup() == -1){
				fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
			}
			else{
				beaconConnected=1; // set flag true for the calibration to start when ready
				// Loop for ever reading data
				while(1){
					/// Time it and wait until next shot
					clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock
					clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // sleep for necessary time to reach desired sampling time
								
					//counter++;			 
					// Read serial data when available
					if (serialDataAvail(fdBeacon)){
						if ((int)(data8[0]=serialGetchar(fdBeacon)) == -1){
							fprintf(stderr, "serialGetchar, block for 10s: %s\n", strerror (errno));
						}
						else if (data8[0] == 0xff){
							data8[1] = serialGetchar(fdBeacon);	// Yype of packet: 0x47 for both hedgehog coordinates and every 10s about beacons coordinates
							data8[2] = serialGetchar(fdBeacon); // this and 3 are both code of data in packet
							data8[3] = serialGetchar(fdBeacon);
							data8[4] = serialGetchar(fdBeacon);	// number of bytes of data transmitting
							
							if ((data16=data8[3] << 8| data8[2]) == 0x0011 && data8[1] == 0x47 && data8[4] == 0x16){
								//hedgehogReading[0] = 1.0; //reading mm				
								n = (int)(data8[4]);
								//printf("%d", n);
								for (int i=0;i<n;i++){
									data8[5+i] = serialGetchar(fdBeacon);
								}
								data8[5+n] = serialGetchar(fdBeacon); // This and (6+n) are CRC-16
								data8[6+n] = serialGetchar(fdBeacon);
								
								//beaconTimestamp = (double)(data32 = data8[8] << 24| data8[7] << 16| data8[6] << 8| data8[5]);			
								// Raw position uint data to float
								posRaw[0] = (double)(int32_t)(data32 = data8[12] << 24| data8[11] << 16| data8[10] << 8| data8[9])*0.001;
								posRaw[1] = (double)(int32_t)(data32 = data8[16] << 24| data8[15] << 16| data8[14] << 8| data8[13])*0.001;	
								posRaw[2] = (double)(int32_t)(data32 = data8[20] << 24| data8[19] << 16| data8[18] << 8| data8[17])*0.001;
								
								beaconFlag = (data8[21] >> 0) & 1; //takes the bit number 1 of it!
								//printf("beaconFlag %i\n", beaconFlag);						
								if ( beaconFlag == 0 ) {
									// Copy raw position to global variable for use in sensor fusion thread
									pthread_mutex_lock(&mutexPositionSensorData);
										memcpy(sensorRawDataPosition, posRaw, sizeof(posRaw));
										//memcpy(sensorRawDataPosition+3, hedgehogReading, sizeof(hedgehogReading));
										//memcpy(sensorRawDataPosition+4, &beaconTimestamp, sizeof(beaconTimestamp));
										//memcpy(sensorRawDataPosition+5, &beaconFlag, sizeof(beaconFlag));
									pthread_mutex_unlock(&mutexPositionSensorData);
								}
								else {
									printf("beaconFlag ERROR, flag is %i\n", beaconFlag); //keep this on RPi console for troubleshooting
								}
								
								//printf("INSIDE %i    ", counter);
								//printmat(sensorRawDataPosition, 1, 4);
								
								// beaconFlag=(int)data8[21];
								//printf("%.0f => X=%.3f, Y=%.3f, Z=%.3f at %f with all flags (mm)\n", hedgehogReading[0], posRaw[0], posRaw[1], posRaw[2], 1/(beaconTimestamp-beaconTimestampPrev)*1000);
								// printBits(1, &beaconFlag);
								//beaconTimestampPrev = beaconTimestamp;
								
								//usleep(15000);
							}
							else if ((data16=data8[3] << 8| data8[2]) == 0x0001 && data8[1] == 0x47 && data8[4] == 0x10){
								//hedgehogReading[0] = 2.0; //reading cm				
								n = (int)(data8[4]);
								//printf("%d", n);
								for (int i=0;i<n;i++){
									data8[5+i] = serialGetchar(fdBeacon);
								}
								data8[5+n] = serialGetchar(fdBeacon); // This and (6+n) are CRC-16
								data8[6+n] = serialGetchar(fdBeacon);
								
								//beaconTimestamp = (double)(data32 = data8[8] << 24| data8[7] << 16| data8[6] << 8| data8[5]);			
								// Raw position uint data to float
								posRaw[0] = (double)(int16_t)(data16 = data8[10] << 8| data8[9])*0.01;
								posRaw[1] = (double)(int16_t)(data16 = data8[12] << 8| data8[11])*0.01;	
								posRaw[2] = (double)(int16_t)(data16 = data8[14] << 8| data8[13])*0.01;
								
								beaconFlag = (data8[15] >> 0) & 1; //takes the bit number 1 of it!
								//printf("beaconFlag %i\n", beaconFlag);					
								if ( beaconFlag == 1 ) {
									// Copy raw position to global variable for use in sensor fusion thread
									pthread_mutex_lock(&mutexPositionSensorData);
										memcpy(sensorRawDataPosition, posRaw, sizeof(posRaw));
										//memcpy(sensorRawDataPosition+3, hedgehogReading, sizeof(hedgehogReading));
										//memcpy(sensorRawDataPosition+4, &beaconTimestamp, sizeof(beaconTimestamp));
										//memcpy(sensorRawDataPosition+5, &beaconFlag, sizeof(beaconFlag));
									pthread_mutex_unlock(&mutexPositionSensorData);	
								}
								else {
									printf("beaconFlag ERROR, flag is %i\n", beaconFlag); //keep this on RPi console for troubleshooting
								}
								
								//beaconFlag=(int)data8[15];	
								//printf("%.0f => X=%.3f, Y=%.3f, Z=%.3f at %f with all flags (cm)\n", hedgehogReading, posRaw[0], posRaw[1], posRaw[2], 1/(beaconTimestamp-beaconTimestampPrev)*1000);
								//printBits(1, &beaconFlag);
								//beaconTimestampPrev = beaconTimestamp;
								
								//usleep(15000);
							}
							else if ((data16=data8[3] << 8| data8[2]) == 0x0002 && data8[1] == 0x47){//cm
								data8[5] = serialGetchar(fdBeacon); // number of beacons
								n = (int)(data8[4]);
								//printf("%d", n);
								for (int i=0;i<n;i++){
									data8[6+i] = serialGetchar(fdBeacon);
								}
								data8[6+n] = serialGetchar(fdBeacon); // This and (7+n) are CRC-16
								data8[7+n] = serialGetchar(fdBeacon);
								/*
								beacons[0] = (double)(data8[6]);//address
								beacons[1] = (double)(data16 = data8[8] << 8| data8[7])*0.01;
								beacons[2] = (double)(data16 = data8[10] << 8| data8[9])*0.01;
								beacons[3] = (double)(data16 = data8[12] << 8| data8[11])*0.01;
								beacons[4] = (double)(data8[14]);//address
								beacons[5] = (double)(data16 = data8[16] << 8| data8[15])*0.01;
								beacons[6] = (double)(data16 = data8[18] << 8| data8[17])*0.01;
								beacons[7] = (double)(data16 = data8[20] << 8| data8[19])*0.01;
								beacons[8] = (double)(data8[22]);//address
								beacons[9] = (double)(data16 = data8[24] << 8| data8[23])*0.01;
								beacons[10] = (double)(data16 = data8[26] << 8| data8[25])*0.01;
								beacons[11] = (double)(data16 = data8[28] << 8| data8[27])*0.01;
								beacons[12] = (double)(data8[30]);//address
								beacons[13] = (double)(data16 = data8[32] << 8| data8[31])*0.01;
								beacons[14] = (double)(data16 = data8[34] << 8| data8[33])*0.01;
								beacons[15] = (double)(data16 = data8[36] << 8| data8[35])*0.01;
								*/
								//printf("%i beacons:\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f,\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f\n", data8[5], (int)beacons[0] ,beacons[1] ,beacons[2], beacons[3], (int)beacons[4], beacons[5], beacons[6], beacons[7], (int)beacons[8], beacons[9], beacons[10], beacons[11], (int)beacons[12], beacons[13], beacons[14], beacons[15]);
								
								//usleep(15000);
							}
							else if ((data16=data8[3] << 8| data8[2]) == 0x0012 && data8[1] == 0x47){//mm
								data8[5] = serialGetchar(fdBeacon); // number of beacons
								n = (int)(data8[4]);
								//printf("%d", n);
								for (int i=0;i<n;i++){
									data8[6+i] = serialGetchar(fdBeacon);
								}
								data8[6+n] = serialGetchar(fdBeacon); // This and (7+n) are CRC-16
								data8[7+n] = serialGetchar(fdBeacon);
								/*
								beacons[0] = (double)(data8[6]);//address
								beacons[1] = (double)(data32 = data8[10] << 24| data8[9] << 16| data8[8] << 8| data8[7])*0.001;
								beacons[2] = (double)(data32 = data8[14] << 24| data8[13] << 16| data8[12] << 8| data8[11])*0.001; //?????????
								beacons[3] = (double)(data32 = data8[18] << 24| data8[17] << 16| data8[16] << 8| data8[15])*0.001;
								beacons[4] = (double)(data8[20]);//address
								beacons[5] = (double)(data32 = data8[24] << 24| data8[23] << 16| data8[22] << 8| data8[21])*0.001;
								beacons[6] = (double)(data32 = data8[28] << 24| data8[27] << 16| data8[26] << 8| data8[25])*0.001;
								beacons[7] = (double)(data32 = data8[32] << 24| data8[31] << 16| data8[30] << 8| data8[29])*0.001;
								beacons[8] = (double)(data8[34]);//address
								beacons[9] = (double)(data32 = data8[38] << 24| data8[37] << 16| data8[36] << 8| data8[35])*0.001;
								beacons[10] = (double)(data32 = data8[42] << 24| data8[41] << 16| data8[40] << 8| data8[39])*0.001;
								beacons[11] = (double)(data32 = data8[46] << 24| data8[45] << 16| data8[44] << 8| data8[43])*0.001;
								beacons[12] = (double)(data8[48]);//address
								beacons[13] = (double)(data32 = data8[52] << 24| data8[51] << 16| data8[50] << 8| data8[49])*0.001;
								beacons[14] = (double)(data32 = data8[56] << 24| data8[55] << 16| data8[54] << 8| data8[53])*0.001;
								beacons[15] = (double)(data32 = data8[60] << 24| data8[59] << 16| data8[58] << 8| data8[57])*0.001;
								*/
								//printf("%i beacons:\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f,\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f\n", data8[5], (int)beacons[0] ,beacons[1] ,beacons[2], beacons[3], (int)beacons[4], beacons[5], beacons[6], beacons[7], (int)beacons[8], beacons[9], beacons[10], beacons[11], (int)beacons[12], beacons[13], beacons[14], beacons[15]);
								
								//usleep(15000);
							}
							else{
								//printf("data8[1] -> %04x\n", data8[1]);
								printf("Unrecognized code of data in packet -> %04x  and  %02x\n", data16, data8[1]);
								//usleep(20000);
							}
						}
						else{
							//usleep(20000);	// if it is not broadcasting 0xff
						}
					}
					else{// else not avail
						//printf("else %i\n", counter);
						/*hedgehogReading[0] = 0.0; //reading is done!
						posRaw[0] = NAN;
						posRaw[1] = NAN;	
						posRaw[2] = NAN;
						pthread_mutex_lock(&mutexPositionSensorData);
							memcpy(sensorRawDataPosition, posRaw, sizeof(posRaw));
							//memcpy(sensorRawDataPosition+3, hedgehogReading, sizeof(hedgehogReading));
							//memcpy(sensorRawDataPosition+4, &beaconTimestamp, sizeof(beaconTimestamp));
							//memcpy(sensorRawDataPosition+5, &beaconFlag, sizeof(beaconFlag));
						pthread_mutex_unlock(&mutexPositionSensorData);*/
						
						//usleep(15000);	// if Data not avail
					}
					
					/// Calculate next shot
					t.tv_nsec += tsReadBeacon;
					while (t.tv_nsec >= NSEC_PER_SEC) {
						t.tv_nsec -= NSEC_PER_SEC;
						t.tv_sec++;
					}	
					
					/// Print true sampling rate
					clock_gettime(CLOCK_MONOTONIC, &t_stop);
					tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
					//printf("Sampling time [s] read beacon: %lf\n",tsTrue);
				}
			}
			t.tv_sec+=2; // wiringpi not activated, sleep for 2 and retry
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

		}
		t.tv_sec+=2; // usb not open, sleep for 2 and retry
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

	}
	return NULL;
}

// Thread - Sensor fusion Orientation and Position
static void *threadSensorFusion (void *arg){
	// Get pipe array and define local variables
	pipeArray *pipeArrayStruct = arg;
	structPipe *ptrPipe1 = pipeArrayStruct->pipe1;
	structPipe *ptrPipe2 = pipeArrayStruct->pipe2;
	
	// Define local variables
	double accRaw[3]={0,0,0}, gyrRaw[3]={0,0,0}, magRaw[3]={0,0,0}, magRawRot[3], tempRaw=0, euler[3]={0,0,0}; // acc0[3]={0,0,0}, gyr0[3]={0,0,0}, mag0[3]={0,0,0}, accCal[3*CALIBRATION], gyrCal[3*CALIBRATION], magCal[3*CALIBRATION], 
	double L=1, normMag=0, sensorDataBuffer[19]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Racc[9]={0,0,0,0,0,0,0,0,0}, Rgyr[9]={0,0,0,0,0,0,0,0,0}, Rmag[9]={0,0,0,0,0,0,0,0,0}, Patt[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}, q[4]={1,0,0,0},
	double posRaw[3]={0,0,0}, posRawPrev[3]={0,0,0}, stateDataBuffer[19]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double tsTrue=tsSensorsFusion; // true sampling time measured using clock_gettime() ,ts_save_buffer
	int  calibrationCounterEKF=0, posRawOldFlag=0, enableMPU9250Flag=-1, enableAK8963Flag=-1; // calibrationCounter=0, calibrationLoaded=0,

	
	// Save to file buffer variable
	double buffer_u1[BUFFER];
	double buffer_u2[BUFFER];
	double buffer_u3[BUFFER];
	double buffer_u4[BUFFER];
	double buffer_omega_x[BUFFER];
	double buffer_omega_y[BUFFER];
	double buffer_omega_z[BUFFER];
	//double buffer_angle_x[BUFFER];
	//double buffer_angle_y[BUFFER];
	//double buffer_angle_z[BUFFER];
	double buffer_thrust[BUFFER];
	double buffer_tau_x[BUFFER];
	double buffer_tau_y[BUFFER];
	double buffer_tau_z[BUFFER];
	double buffer_ts[BUFFER];
	
	double buffer_acc_x[BUFFER];
	double buffer_acc_y[BUFFER];
	double buffer_acc_z[BUFFER];
	double buffer_gyr_x[BUFFER];
	double buffer_gyr_y[BUFFER];
	double buffer_gyr_z[BUFFER];
	double buffer_mag_x[BUFFER];
	double buffer_mag_y[BUFFER];
	double buffer_mag_z[BUFFER];
	double buffer_acc_x_filt[BUFFER];
	double buffer_acc_y_filt[BUFFER];
	double buffer_acc_z_filt[BUFFER];
	double buffer_gyr_x_filt[BUFFER];
	double buffer_gyr_y_filt[BUFFER];
	double buffer_gyr_z_filt[BUFFER];
	
	
	
	int buffer_counter=0;
	//FILE *fpWrite;
	
	// EKF variables
	//double Pekf9x9_bias[81]={1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
	double Pekf6x6[36]={1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1};
	double Pekf9x9[81]={1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
	//double xhat9x9_bias[9]={0,0,0,0,0,0,0,0,0};
	double xhat6x6[6]={0,0,0,0,0,0};
	double xhat9x9[9]={0,0,0,0,0,0,0,0,-par_g};
	double uControl[4]={.1,.1,.1,.1};
	
	//double Pekf9x9_biasInit[81]={1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
	double Pekf6x6Init[36]={1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1};
	double Pekf9x9Init[81]={1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
	//double xhat9x9_biasInit[9]={0,0,0,0,0,0,0,0,0};
	double xhat6x6Init[6]={0,0,0,0,0,0};
	double xhat9x9Init[9]={0,0,0,0,0,0,0,0,-par_g};
	double uControlInit[4]={.1,.1,.1,.1};
	double uControlThrustTorques[4]={0,0,0,0};
	
	//double Rekf9x9_bias[36]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double Rekf6x6[36]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double Rekf9x9[9]={0,0,0,0,0,0,0,0,0};
	//double ymeas9x9_bias[6]; // vector for 9x9 EKF attitude - measurement: angles and gyro
	double ymeas6x6[6];
	double ymeas9x9[3]; // vector for 9x9 EKF position - measurement: position

	//double ekf09x9_bias[6]={0,0,0,0,0,0}, ekfCal9x9_bias[6*CALIBRATION];
	double ekf06x6[6]={0,0,0,0,0,0}, ekfCal6x6[6*CALIBRATION];
	double ekf09x9[3]={0,0,0}, ekfCal9x9[3*CALIBRATION];
	
	//double tuningEkfBuffer9x9_bias[9]={ekf_Q_7,ekf_Q_8,ekf_Q_9,ekf_Q_10,ekf_Q_11,ekf_Q_12,ekf_Q_16,ekf_Q_17,ekf_Q_18}; //{phi, theta, psi, omega_x, omega_y, omega_z,bias_taux, bias_tauy,bias_tauz}
	double tuningEkfBuffer6x6[6]={ekf_Q_7,ekf_Q_8,ekf_Q_9,ekf_Q_10,ekf_Q_11,ekf_Q_12}; //{phi, theta, psi, omega_x, omega_y, omega_z,bias_taux, bias_tauy,bias_tauz}
	double tuningEkfBuffer9x9[9]={ekf_Q_1,ekf_Q_2,ekf_Q_3,ekf_Q_4,ekf_Q_5,ekf_Q_6,ekf_Q_13,ekf_Q_14,ekf_Q_15}; //{x, y, z, xdot, ydot, zdot, distx, disty, distz}
	
	// Low Pass filter variables(20Hz and 30 for gyr)
	//double b_acc[25]={1.54791392878543e-06,2.07185488006612e-05,-5.19703972529415e-20,-0.000462086226027071,-0.00215334176418159,-0.00523098435850049,-0.00693485312903388,2.82650148861993e-18,0.0249007529841827,0.0717988692685668,0.131922751556644,0.183873196192904,0.204526858025432,0.183873196192904,0.131922751556644,0.0717988692685668,0.0249007529841827,2.82650148861993e-18,-0.00693485312903388,-0.0052309843585005,-0.00215334176418159,-0.000462086226027072,-5.1970397252942e-20,2.07185488006612e-05,1.54791392878543e-06};
	//double b_gyr[25]={-1.51379393190905e-06,-2.78880561210566e-05,7.62372544145295e-20,0.000621987897327971,0.00210587658352694,0.00166218520266596,-0.00678199116255076,-0.0225713907176518,-0.0243518764618903,0.0228146386774076,0.129014835433318,0.247501215646055,0.300027841503688,0.247501215646055,0.129014835433318,0.0228146386774076,-0.0243518764618903,-0.0225713907176518,-0.00678199116255077,0.00166218520266596,0.00210587658352694,0.000621987897327972,7.62372544145302e-20,-2.78880561210566e-05,-1.51379393190905e-06};

	//5hz b values
	double b_acc[25]={3.67701752843937e-06,8.27007534721863e-05,0.000504038930132126,0.0018444766295764,0.00511519100925013,0.0116414444963577,0.0226738421555306,0.0387678986038862,0.0591509020569521,0.0814154703145087,0.101822705283757,0.116246817848376,0.121461669801344,0.116246817848376,0.101822705283757,0.0814154703145088,0.0591509020569522,0.0387678986038862,0.0226738421555306,0.0116414444963577,0.00511519100925013,0.0018444766295764,0.00050403893013213,8.27007534721863e-05,3.67701752843937e-06};
	double b_gyr[25]={3.67701752843937e-06,8.27007534721863e-05,0.000504038930132126,0.0018444766295764,0.00511519100925013,0.0116414444963577,0.0226738421555306,0.0387678986038862,0.0591509020569521,0.0814154703145087,0.101822705283757,0.116246817848376,0.121461669801344,0.116246817848376,0.101822705283757,0.0814154703145088,0.0591509020569522,0.0387678986038862,0.0226738421555306,0.0116414444963577,0.00511519100925013,0.0018444766295764,0.00050403893013213,8.27007534721863e-05,3.67701752843937e-06};
	
	double accRawMem[75]={0}; // memory buffer where elements 0-24=x-axis, 25-49=y-axis and 50-74=z-axis
	double gyrRawMem[75]={0};
	
	
	// Random variables
	double L_temp;
	double a=0.01;
	int counterCalEuler=0;
	double q_comp[4], q_init[4]; 
	int k=0;
	double euler_comp[3];
	double euler_mean[3];
	int eulerCalFlag=0;
	float beta_keyboard;
	int isnan_flag=0, outofbounds_flag=0;

	// Keyboard control variables
	int timerPrint=0, ekfPrint=0, ekfReset=0, ekfPrint6States=0, sensorCalibrationRestart=0,saveDataTrigger=0;
	int outlierFlag, outlierFlagPercentage, outlierFlagMem[1000];
	int ekfPrint6StatesCounter=0, ekfPrintCounter=0;
	
	/// Setup timer variables for real time
	struct timespec t,t_start,t_stop; // ,t_start_buffer,t_stop_buffer

	/// Average sampling
	int tsAverageCounter=0, tsAverageReadyEKF=0; // tsAverageReadyEKF is used for to give orientation filter som time to converge before calibration of EKF starts collecting data
	double tsAverageAccum=0;
	double tsAverage=tsSensorsFusion;

	/// Lock memory
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
		perror("mlockall failed in threadSensorFusion");
	}
	
	/// Start after 1 second
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;

	printf("Enabling sensors...\n");
	while(1){
		// Try to enable acc, gyr, mag  and bmp sensors
		pthread_mutex_lock(&mutexI2CBusy);
			enableMPU9250Flag=enableMPU9250();
			//enableAK8963Flag=enableAK8963();
		pthread_mutex_unlock(&mutexI2CBusy);
		
		// Check that I2C sensors have been enabled
		if(enableMPU9250Flag==-1){
			printf("MPU9250 failed to be enabled\n");
		}
		//else if(enableAK8963Flag==-1){
			//printf("AK8963 failed to be enabled\n");
		//}
		else{
			// Loop for ever
			while(1){
				/// Wait until next shot
				 clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // sleep for necessary time to reach desired sampling time
				
				// Read raw sensor data from I2C bus to local variable
				pthread_mutex_lock(&mutexI2CBusy);	
					readAllSensorData(accRaw, gyrRaw, magRaw, &tempRaw);	
				pthread_mutex_unlock(&mutexI2CBusy);
				
				// Read raw position data from global to local variable
				pthread_mutex_lock(&mutexPositionSensorData);	
					memcpy(posRaw, sensorRawDataPosition, sizeof(sensorRawDataPosition));		
				pthread_mutex_unlock(&mutexPositionSensorData);
				
				// Read latest control signal from globale variable to local variable
				pthread_mutex_lock(&mutexControlData);	
					memcpy(uControl, controlData, sizeof(uControl));		
					memcpy(uControlThrustTorques, controlData+4, sizeof(uControlThrustTorques));	
				pthread_mutex_unlock(&mutexControlData);
				
				// Get keyboard input data
				pthread_mutex_lock(&mutexKeyboardData);
					timerPrint=(int)keyboardData[5];
					ekfPrint=(int)keyboardData[6];
					ekfReset=(int)keyboardData[7];
					ekfPrint6States=(int)keyboardData[8];
					sensorCalibrationRestart=(int)keyboardData[9];
					a=keyboardData[11];
					beta_keyboard=keyboardData[12];
					saveDataTrigger=(int)keyboardData[15];
					 //memcpy(tuningEkfBuffer9x9_bias, tuningEkfData+6, sizeof(tuningEkfData)*6/18); // ekf states 7-12
					 //memcpy(tuningEkfBuffer9x9_bias+6, tuningEkfData+15, sizeof(tuningEkfData)*3/18); // ekf states 16-18
					memcpy(tuningEkfBuffer6x6, tuningEkfData+6, sizeof(tuningEkfData)*6/18); // ekf states 7-12
					memcpy(tuningEkfBuffer9x9, tuningEkfData, sizeof(tuningEkfData)*6/18); // ekf states 1-6
					memcpy(tuningEkfBuffer9x9+6, tuningEkfData+12, sizeof(tuningEkfData)*3/18); // ekf states 13-15
				pthread_mutex_unlock(&mutexKeyboardData);
				
				// Convert sensor data to correct (filter) units:
				// Acc: g -> m/s² Factor: 9.81
				// Gyr: degrees/s -> radians/s Factor: (PI/180)
				// Mag: milli gauss -> micro tesla Factor: 10^-1
				//accRaw[0]*=9.81;
				//accRaw[1]*=9.81;
				//accRaw[2]*=9.81;
				gyrRaw[0]*=(PI/180);
				gyrRaw[1]*=(PI/180);
				gyrRaw[2]*=(PI/180);
				//magRaw[0]/=10;
				//magRaw[1]/=10;
				//magRaw[2]/=10;
				//magRaw[0]*=1000;
				//magRaw[1]*=1000;
				//magRaw[2]*=1000;
				
				// Rotate magnetometer data such that the sensor coordinate frames match.
				// Note: For more info check the MPU9250 Product Specification (Chapter 9)
				magRawRot[0]=magRaw[1];
				magRawRot[1]=magRaw[0];
				magRawRot[2]=magRaw[2];
				
				/// Time it and print true sampling rate
				clock_gettime(CLOCK_MONOTONIC, &t_stop); /// stop elapsed time clock
				tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
				sampleFreq=(float) 1.0/tsTrue; /// set Madgwick filter sampling frequency equal to true sampling time
				//printf("SampleFre: %f\n", sampleFreq);
				clock_gettime(CLOCK_MONOTONIC ,&t_start); /// start elapsed time clock
								
				/// Get average sampling time
				if(tsAverageCounter<50){
					tsAverageAccum+=tsTrue;
					tsAverageCounter++;
				}
				else{
					//printf("tsAverageAccum: %lf\n", tsAverageAccum);
					tsAverageAccum/=50;
					tsAverage=tsAverageAccum;
					if(timerPrint){
						printf("EKF: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
					}
					
					/// Activate EKF calibration after tsAverage has been within limit for a number of times. (allows orientation filter to converge)
					if(tsAverage>(tsSensorsFusion/NSEC_PER_SEC)*0.9 && tsAverage<(tsSensorsFusion/NSEC_PER_SEC)*1.1 && tsAverageReadyEKF<2){
						tsAverageReadyEKF++;
						printf("tsAverage within limit for EKF to start %i times\n",tsAverageReadyEKF);
					}
				
					//printmat(Patt,4,4);
					tsAverageCounter=0;
					tsAverageAccum=0;
				}
				
				//tsAverageReadyEKF=2;
				
				// Set gain of orientation estimation Madgwick beta and activate Low Pass filtering of raw accelerometer and gyroscope after sampling frequency has stabilized
				if(tsAverageReadyEKF==2){
					
					if(saveDataTrigger){ // only save data when activated from keyboard
						//clock_gettime(CLOCK_MONOTONIC ,&t_start_buffer); /// start elapsed time clock for buffering procedure
						// Saving data before low-pass filtering
						if(buffer_counter==BUFFER){ // if buffer is full, save to file
							saveData(buffer_acc_x,"acc_x",sizeof(buffer_acc_x)/sizeof(double));
							saveData(buffer_acc_y,"acc_y",sizeof(buffer_acc_y)/sizeof(double));
							saveData(buffer_acc_z,"acc_z",sizeof(buffer_acc_z)/sizeof(double));
							saveData(buffer_gyr_x,"gyr_x",sizeof(buffer_gyr_x)/sizeof(double));
							saveData(buffer_gyr_y,"gyr_y",sizeof(buffer_gyr_y)/sizeof(double));
							saveData(buffer_gyr_z,"gyr_z",sizeof(buffer_gyr_z)/sizeof(double));
						}
						else{ // else keep saving data to buffer
							buffer_acc_x[buffer_counter]=accRaw[0];
							buffer_acc_y[buffer_counter]=accRaw[1];
							buffer_acc_z[buffer_counter]=accRaw[2];
							buffer_gyr_x[buffer_counter]=gyrRaw[0];
							buffer_gyr_y[buffer_counter]=gyrRaw[1];
							buffer_gyr_z[buffer_counter]=gyrRaw[2];

						}
					}
					// Low Pass Filter using Blackman Harris window
					// Order of 24 = 12 sample delay = 0.012s
					// Cut-off frequencies: accelerometer = 20Hz and gyroscope = 30Hz 
					// The delay is approx half of controller frequency
					// Leaving enough time for Madgwick and EKF to converge after Low Pass filtering and before the controller need new fresh measurements
					
					//lowPassFilter(accRaw, gyrRaw, accRawMem, gyrRawMem, b_acc, b_gyr);
					
					// Set gain of orientation estimation Madgwick beta after initial filter learn
					if(k==1000){
						beta=0.05;
						//eulerCalFlag=1;
					}
					else{
						printf("SampleFre: %f Sample: %i\n", sampleFreq, k++);
					}
				
					//sensorDataBuffer[6]=magRawRot[0];
					//sensorDataBuffer[7]=magRawRot[1];
					//sensorDataBuffer[8]=magRawRot[2];
						
					if(eulerCalFlag==1){
						beta=beta_keyboard;
					}
						
					// Magnetometer outlier detection
					//outlierFlag=0;
					//normMag=sqrt(pow(magRawRot[0],2) + pow(magRawRot[1],2) + pow(magRawRot[2],2));
					//L_temp=(1-a)*L+a*normMag; // recursive magnetometer compensator
					//L=L_temp;
					//if ((normMag > L*1.05 || normMag < L*0.95) && eulerCalFlag==1){
						//magRawRot[0]=0.0f;
						//magRawRot[1]=0.0f;
						//magRawRot[2]=0.0f;
						//outlierFlag=1;
						//beta*=0.8;
						////printf("Mag outlier\n");
					//}
					
					//// outlier percentage
					//outlierFlagPercentage = 0;
					//for (int i=1; i<1000; i++) {
						//outlierFlagMem[i-1] = outlierFlagMem[i];
						//outlierFlagPercentage += outlierFlagMem[i-1];
					//}
					//outlierFlagMem[999] = outlierFlag;
					//outlierFlagPercentage += outlierFlagMem[999];
									
					// Orientation estimation with Madgwick filter
					//MadgwickAHRSupdate((float)gyrRaw[0], (float)gyrRaw[1], (float)gyrRaw[2], (float)accRaw[0], (float)accRaw[1], (float)accRaw[2], (float)magRawRot[0], (float)magRawRot[1], (float)magRawRot[2]);
					MadgwickAHRSupdateIMU((float)gyrRaw[0], (float)gyrRaw[1], (float)gyrRaw[2], (float)accRaw[0], (float)accRaw[1], (float)accRaw[2]);
					
					// Copy out the returned quaternions from the filter
					q_comp[0]=q0;
					q_comp[1]=-q1;
					q_comp[2]=-q2;
					q_comp[3]=-q3;
					
					// Quaternions to eulers (rad)
					q2euler_zyx(euler,q_comp);
				
					 //Allignment compensation for initial point of orientation angles
					if(k==1000){
						if(counterCalEuler<1000){
							// Mean (bias) accelerometer, gyroscope and magnetometer
							euler_mean[0]+=euler[0];
							euler_mean[1]+=euler[1];
							euler_mean[2]+=euler[2];
							counterCalEuler++;
							printf("euler_sum: %1.4f %1.4f %1.4f counter: %i\n", euler_mean[0], euler_mean[1], euler_mean[2], counterCalEuler);
						}
						else if(counterCalEuler==1000){
							euler_mean[0]/=1000.0f;
							euler_mean[1]/=1000.0f;
							euler_mean[2]/=1000.0f;
							counterCalEuler++;
							eulerCalFlag=1;
							printf("euler_mean: %1.4f %1.4f %1.4f counter: %i\n", euler_mean[0], euler_mean[1], euler_mean[2], counterCalEuler);
						}
						else{
							euler_comp[0]=euler[0]-euler_mean[0];
							euler_comp[1]=euler[1]-euler_mean[1];
							euler_comp[2]=euler[2]-euler_mean[2];
						}
					}
				
					}
					else{
						printf("SampleFre: %f\n", sampleFreq);
					}
								
					// Move over data to communication.c via pipe
					sensorDataBuffer[0]=gyrRaw[0];
					sensorDataBuffer[1]=gyrRaw[1];
					sensorDataBuffer[2]=gyrRaw[2];
					sensorDataBuffer[3]=accRaw[0];
					sensorDataBuffer[4]=accRaw[1];
					sensorDataBuffer[5]=accRaw[2];
					sensorDataBuffer[6]=magRawRot[0];
					sensorDataBuffer[7]=magRawRot[1];
					sensorDataBuffer[8]=magRawRot[2];
					sensorDataBuffer[9]=euler_comp[0];
					sensorDataBuffer[10]=euler_comp[1];
					sensorDataBuffer[11]=euler_comp[2];
					sensorDataBuffer[12]=(double)q_comp[0];
					sensorDataBuffer[13]=(double)q_comp[1];
					sensorDataBuffer[14]=(double)q_comp[2];
					sensorDataBuffer[15]=(double)q_comp[3];
					sensorDataBuffer[16]=posRaw[0];
					sensorDataBuffer[17]=posRaw[1];
					sensorDataBuffer[18]=posRaw[2];
					
					// Write to Communication process
					if (write(ptrPipe2->parent[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor ot Communicaiont\n");
					//else printf("Sensor ID: %d, Sent: %f to Communication\n", (int)getpid(), sensorDataBuffer[0]);
							
					beaconConnected=1;
							
					if(beaconConnected==1 && tsAverageReadyEKF==2 && eulerCalFlag==1){
						// Check if raw position data is new or old
						if(posRaw[0]==posRawPrev[0] && posRaw[1]==posRawPrev[1] && posRaw[2]==posRawPrev[2]){
							posRawOldFlag=1;
						}
						else{
							posRawOldFlag=0;
							memcpy(posRawPrev, posRaw, sizeof(posRaw));
						}
						
						// Move data from (euler and posRaw) array to ymeas
						//ymeas[0]=posRaw[0];
						//ymeas[1]=posRaw[1];
						//ymeas[2]=posRaw[2];
						ymeas9x9[0]=0; // position x
						ymeas9x9[1]=0; // position y
						ymeas9x9[2]=0; // position z
						//ymeas9x9_bias[0]=euler_comp[2]; // phi (x-axis)
						//ymeas9x9_bias[1]=euler_comp[1]; // theta (y-axis)
						//ymeas9x9_bias[2]=euler_comp[0]; // psi (z-axis)
						//ymeas9x9_bias[3]=gyrRaw[0]; // gyro x
						//ymeas9x9_bias[4]=gyrRaw[1]; // gyro y
						//ymeas9x9_bias[5]=gyrRaw[2]; // gyro z
						
						ymeas6x6[0]=euler_comp[2]; // phi (x-axis)
						ymeas6x6[1]=euler_comp[1]; // theta (y-axis)
						ymeas6x6[2]=euler_comp[0]; // psi (z-axis)
						ymeas6x6[3]=gyrRaw[0]; // gyro x
						ymeas6x6[4]=gyrRaw[1]; // gyro y
						ymeas6x6[5]=gyrRaw[2]; // gyro z
						
						// Flip direction of rotation and gyro around y-axis and x-axis to match model
						//ymeas9x9_bias[0]*=-1; // flip phi (x-axis)						
						//ymeas9x9_bias[1]*=-1; // flip theta (y-axis)	
						//ymeas9x9_bias[3]*=-1; // flip gyro (x-axis)						
						////ymeas9x9_bias[4]*=-1; // flip gyro (y-axis)
						//ymeas9x9_bias[5]*=-1; // flip gyro (z-axis)
						
						ymeas6x6[0]*=-1; // flip phi (x-axis)						
						ymeas6x6[1]*=-1; // flip theta (y-axis)	
						ymeas6x6[3]*=-1; // flip gyro (x-axis)						
						//ymeas6x6[4]*=-1; // flip gyro (y-axis)
						ymeas6x6[5]*=-1; // flip gyro (z-axis)

						// Calibration routine for EKF
						if (calibrationCounterEKF==0){
							printf("EKF Calibration started\n");
							//ekfCalibration9x9_bias(Rekf9x9_bias, ekf09x9_bias, ekfCal9x9_bias, ymeas9x9_bias, calibrationCounterEKF);
							ekfCalibration6x6(Rekf6x6, ekf06x6, ekfCal6x6, ymeas6x6, calibrationCounterEKF);
							ekfCalibration9x9(Rekf9x9, ekf09x9, ekfCal9x9, ymeas9x9, calibrationCounterEKF);
							//printf("calibrationCounterEKF\n: %i", calibrationCounterEKF);
							calibrationCounterEKF++;
						}
						else if(calibrationCounterEKF<CALIBRATION){
							 //ekfCalibration9x9_bias(Rekf9x9_bias, ekf09x9_bias, ekfCal9x9_bias, ymeas9x9_bias, calibrationCounterEKF);
							ekfCalibration6x6(Rekf6x6, ekf06x6, ekfCal6x6, ymeas6x6, calibrationCounterEKF);
							ekfCalibration9x9(Rekf9x9, ekf09x9, ekfCal9x9, ymeas9x9, calibrationCounterEKF);
							//printf("calibrationCounterEKF\n: %i", calibrationCounterEKF);
							calibrationCounterEKF++;
							
						}
						else if(calibrationCounterEKF==CALIBRATION){
							//ekfCalibration9x9_bias(Rekf9x9_bias, ekf09x9_bias, ekfCal9x9_bias, ymeas9x9_bias, calibrationCounterEKF);
							ekfCalibration6x6(Rekf6x6, ekf06x6, ekfCal6x6, ymeas6x6, calibrationCounterEKF);
							ekfCalibration9x9(Rekf9x9, ekf09x9, ekfCal9x9, ymeas9x9, calibrationCounterEKF);
								
							// Save calibration in 'settings.txt' if it does not exist
							//saveSettings(Rekf,"Rekf",sizeof(Rekf)/sizeof(double), &fpWrite);
							//saveSettings(ekf0,"ekf0",sizeof(ekf0)/sizeof(double), &fpWrite);
							 //saveSettings(Rekf9x9_bias,"Rekf9x9_bias",sizeof(Rekf9x9_bias)/sizeof(double));
							saveSettings(Rekf6x6,"Rekf6x6",sizeof(Rekf6x6)/sizeof(double));
							saveSettings(Rekf9x9,"Rekf9x9",sizeof(Rekf9x9)/sizeof(double));
							//saveSettings(ekf09x9_bias,"ekf09x9_bias",sizeof(ekf09x9_bias)/sizeof(double));
							saveSettings(ekf06x6,"ekf06x6",sizeof(ekf06x6)/sizeof(double));
							saveSettings(ekf09x9,"ekf09x9",sizeof(ekf09x9)/sizeof(double));
								
							//printf("calibrationCounterEKF: %i\n", calibrationCounterEKF);
							printf("EKF Calibrations finish\n");
							calibrationCounterEKF++;
							
							// Initialize EKF with current available measurement
							//printf("Initialize EKF xhat with current measurments for position and orientation");
							
							// Attitude states initial measurments
							xhat6x6[0]=0;
							xhat6x6[1]=0;
							xhat6x6[2]=0;
							xhat6x6[3]=0;
							xhat6x6[4]=0;
							xhat6x6[5]=0;
							
							// Position states initial measurments
							xhat9x9[0]=ymeas9x9[0];
							xhat9x9[1]=ymeas9x9[1];
							xhat9x9[2]=ymeas9x9[2];
							
							// Quaternions initial measurments
							q_init[0]=q0;
							q_init[1]=q1;
							q_init[2]=q2;
							q_init[3]=q3;
						}
						// State Estimator
						else{
							// Run EKF as long as ekfReset keyboard input is false
							if(!ekfReset){
								// Run Extended Kalman Filter (state estimator) using position and orientation data
								//EKF_no_inertia(Pekf,xhat,uControl,ymeas,Qekf,Rekf,tsAverage,posRawOldFlag);
								//EKF_9x9_bias(Pekf9x9_bias,xhat9x9_bias,uControl,ymeas9x9_bias,tuningEkfBuffer9x9_bias,Rekf9x9_bias,tsTrue);
								
								
								EKF_6x6(Pekf6x6,xhat6x6,uControl,ymeas6x6,tuningEkfBuffer6x6,Rekf6x6,tsTrue);
								EKF_9x9(Pekf9x9,xhat9x9,uControl,ymeas9x9,tuningEkfBuffer9x9,Rekf9x9,tsTrue,posRawOldFlag,xhat6x6);
									
								stateDataBuffer[15]=1; // ready flag for MPC to start using the initial conditions given by EKF.
							}
							// Reset EKF with initial Phat, xhat and uControl as long as ekfReset keyboard input is true
							else{
								//memcpy(Pekf9x9_bias, Pekf9x9_biasInit, sizeof(Pekf9x9_biasInit));
								memcpy(Pekf6x6, Pekf6x6, sizeof(Pekf6x6Init));	
								memcpy(Pekf9x9, Pekf9x9Init, sizeof(Pekf9x9Init));	
								//memcpy(xhat9x9_bias, xhat9x9_biasInit, sizeof(xhat9x9_biasInit));
								memcpy(xhat6x6, xhat6x6Init, sizeof(xhat6x6Init));
								memcpy(xhat9x9, xhat9x9Init, sizeof(xhat9x9Init));
								memcpy(uControl, uControlInit, sizeof(uControlInit));
								
								q0=q_init[0];
								q1=q_init[1];
								q2=q_init[2];
								q3=q_init[3];
								
								stateDataBuffer[15]=0; // set ready flag for MPC false during reset
								isnan_flag=0;
								outofbounds_flag=0;
							}
							
							// Override disturbance estimation in x and y direction
							xhat9x9[6]=0;
							xhat9x9[7]=0;
							
							// Torque disturbance saturation
							//saturation(xhat9x9_bias,6,-0.01,0.01);
							//saturation(xhat9x9_bias,7,-0.01,0.01);
							// saturation(xhat9x9_bias,6,0.0,0.0);
							// saturation(xhat9x9_bias,7,0.0,0.0);
							// saturation(xhat9x9_bias,8,0.0,0.0);
							
							////Check for EKF9x9_bias failure (isnan)
							 //for (int j=0;j<9;j++){
								 //if (isnan(xhat9x9_bias[j])!=0){
									 //isnan_flag=1;
									 //break;
								 //}						
							 //}
							 
							//Check for EKF6x6 failure (isnan)
							 for (int j=0;j<6;j++){
								 if (isnan(xhat6x6[j])!=0){
									 isnan_flag=1;
									 break;
								 }						
							 }
							
							// Check for EKF9x9 failure (isnan)
							for (int j=0;j<9;j++){
								if (isnan(xhat9x9[j])!=0){
									isnan_flag=1;
									break;
								}						
							}
							
							//// Check for EKF9x9_bias failure (out of bounds)
							 //for (int j=0;j<9;j++){
								 //if (xhat9x9_bias[j]>1e6){
									 //outofbounds_flag=1;
									 //break;
								 //}						
							 //}				
							
							// Check for EKF6x6 failure (out of bounds)
							for (int j=0;j<6;j++){
								if (xhat6x6[j]>1e6){
									 outofbounds_flag=1;
									 break;
								}						
							}
							
							// Check for EKF9x9 failure (out of bounds)
							for (int j=0;j<9;j++){
								if (xhat9x9[j]>1e6){
									outofbounds_flag=1;
									break;
								}						
							}
						
						if(isnan_flag){
							stateDataBuffer[15]=0;
							printf("EKF xhat=nan\n");
						}
						else if(outofbounds_flag){
							stateDataBuffer[15]=0;
							printf("EKF xhat=out of bounds\n");
						}
\
						// Move over data to controller.c via pipe
						stateDataBuffer[0]=xhat9x9[0]; // position x
						stateDataBuffer[1]=xhat9x9[1]; // position y
						stateDataBuffer[2]=xhat9x9[2]; // position z
						stateDataBuffer[3]=xhat9x9[3]; // velocity x
						stateDataBuffer[4]=xhat9x9[4]; // velocity y
						stateDataBuffer[5]=xhat9x9[5]; // velocity z
						
						//stateDataBuffer[6]=ymeas9x9_bias[0]; // phi (x-axis)
						//stateDataBuffer[7]=ymeas9x9_bias[1]; // theta (y-axis)
						//stateDataBuffer[8]=ymeas9x9_bias[2]; // psi (z-axis)
						//stateDataBuffer[9]=ymeas9x9_bias[3]; // omega x (gyro)
						//stateDataBuffer[10]=ymeas9x9_bias[4]; // omega y (gyro)
						//stateDataBuffer[11]=ymeas9x9_bias[5]; // omega z (gyro)
						stateDataBuffer[6]=xhat6x6[0]; // phi (x-axis)
						stateDataBuffer[7]=xhat6x6[1]; // theta (y-axis)
						stateDataBuffer[8]=xhat6x6[2]; // psi (z-axis)
						stateDataBuffer[9]=xhat6x6[3]; // omega x (gyro)
						stateDataBuffer[10]=xhat6x6[4]; // omega y (gyro)
						stateDataBuffer[11]=xhat6x6[5]; // omega z (gyro)
						
						stateDataBuffer[12]=xhat9x9[6]; // disturbance x
						stateDataBuffer[13]=xhat9x9[7]; // disturbance y
						stateDataBuffer[14]=xhat9x9[8]; // disturbance z
						
						 //stateDataBuffer[16]=xhat9x9_bias[6]; // bias taux
						 //stateDataBuffer[17]=xhat9x9_bias[7]; // bias tauy
						 //stateDataBuffer[18]=xhat9x9_bias[8]; // bias tauz
						 stateDataBuffer[16]=0; // bias taux
						 stateDataBuffer[17]=0; // bias tauy
						 stateDataBuffer[18]=0; // bias tauz
						 
						//stateDataBuffer[15]=1; // ready flag for MPC to start using the initial conditions given by EKF.

						if(ekfPrint && ekfPrintCounter % 10 == 0){
							printf("xhat: (pos) % 1.4f % 1.4f % 1.4f (vel) % 1.4f % 1.4f % 1.4f (dist pos) % 1.4f % 1.4f % 1.4f (ang°) % 2.4f % 2.4f % 2.4f (angVel°) % 2.4f % 2.4f % 2.4f (freq) % 3.1f\n",xhat9x9[0],xhat9x9[1],xhat9x9[2],xhat9x9[3],xhat9x9[4],xhat9x9[5],xhat9x9[6],xhat9x9[7],xhat9x9[8],xhat6x6[0]*(180/PI),xhat6x6[1]*(180/PI),xhat6x6[2]*(180/PI),xhat6x6[3]*(180/PI),xhat6x6[4]*(180/PI),xhat6x6[5]*(180/PI), sampleFreq);
						}
						ekfPrintCounter++;
						
						if(ekfPrint6States && ekfPrint6StatesCounter % 10 == 0){
							//printf("xhat: % 1.4f % 1.4f % 1.4f % 2.4f % 2.4f % 2.4f (euler_meas) % 2.4f % 2.4f % 2.4f (gyr_meas) % 2.4f % 2.4f % 2.4f (outlier) %i %i (freq) %3.5f u: %3.4f %3.4f %3.4f %3.4f\n",xhat9x9[0],xhat9x9[1],xhat9x9[2],xhat9x9_bias[0]*(180/PI),xhat9x9_bias[1]*(180/PI),xhat9x9_bias[2]*(180/PI), ymeas9x9_bias[0]*(180/PI),ymeas9x9_bias[1]*(180/PI),ymeas9x9_bias[2]*(180/PI), gyrRaw[0], gyrRaw[1], gyrRaw[2], outlierFlag, outlierFlagPercentage, sampleFreq, uControl[0], uControl[1], uControl[2], uControl[3]);
							printf("(ang(meas)) % 2.4f % 2.4f % 2.4f (ang(xhat)) % 2.4f % 2.4f % 2.4f (pwm) % 3.4f % 3.4f % 3.4f % 3.4f (thrust) % 1.3f (torque) % 1.4f % 1.4f % 1.4f (acc) % 1.4f % 1.4f % 1.4f \n",ymeas6x6[0]*(180/PI),ymeas6x6[1]*(180/PI),ymeas6x6[2]*(180/PI), xhat6x6[0]*(180/PI),xhat6x6[1]*(180/PI),xhat6x6[2]*(180/PI), uControl[0], uControl[1], uControl[2], uControl[3], uControlThrustTorques[0], uControlThrustTorques[1], uControlThrustTorques[2], uControlThrustTorques[3], accRaw[0], accRaw[1], accRaw[2]);
						}
						ekfPrint6StatesCounter++;
	
						// Write to Controller process
						if (write(ptrPipe1->child[1], stateDataBuffer, sizeof(stateDataBuffer)) != sizeof(stateDataBuffer)) printf("pipe write error in Sensor to Controller\n");
						//else printf("Sensor ID: %d, Sent: %f to Controller\n", (int)getpid(), sensorDataBuffer[0]);
						
						// Restart sensor fusion and EKF calibration
						if(sensorCalibrationRestart){
							// calibrationCounter=0; // forces sensor fusion to restart calibration
							calibrationCounterEKF=0; // forces ekf to restart calibration
						}
						
						//// Save buffered data to file
						if(saveDataTrigger){ // only save data when activated from keyboard
							//clock_gettime(CLOCK_MONOTONIC ,&t_start_buffer); /// start elapsed time clock for buffering procedure
							if(buffer_counter==BUFFER){ // if buffer is full, save to file
								//saveData(buffer_u1,"u1",sizeof(buffer_u1)/sizeof(double));
								//saveData(buffer_u2,"u2",sizeof(buffer_u2)/sizeof(double));
								//saveData(buffer_u3,"u3",sizeof(buffer_u3)/sizeof(double));
								//saveData(buffer_u4,"u4",sizeof(buffer_u4)/sizeof(double));
								//saveData(buffer_omega_x,"omega_x",sizeof(buffer_omega_x)/sizeof(double));
								//saveData(buffer_omega_y,"omega_y",sizeof(buffer_omega_y)/sizeof(double));
								//saveData(buffer_omega_z,"omega_z",sizeof(buffer_omega_z)/sizeof(double));
								////saveData(buffer_angle_x,"angle_x",sizeof(buffer_angle_x)/sizeof(double));
								////saveData(buffer_angle_y,"angle_y",sizeof(buffer_angle_y)/sizeof(double));
								////saveData(buffer_angle_z,"angle_z",sizeof(buffer_angle_z)/sizeof(double));
								//saveData(buffer_thrust,"thrust",sizeof(buffer_thrust)/sizeof(double));
								//saveData(buffer_tau_x,"tau_x",sizeof(buffer_tau_x)/sizeof(double));
								//saveData(buffer_tau_y,"tau_y",sizeof(buffer_tau_y)/sizeof(double));
								//saveData(buffer_tau_z,"tau_z",sizeof(buffer_tau_z)/sizeof(double));
								saveData(buffer_acc_x_filt,"acc_x_filt",sizeof(buffer_acc_x)/sizeof(double));
								saveData(buffer_acc_y_filt,"acc_y_filt",sizeof(buffer_acc_y)/sizeof(double));
								saveData(buffer_acc_z_filt,"acc_z_filt",sizeof(buffer_acc_z)/sizeof(double));
								saveData(buffer_gyr_x_filt,"gyr_x_filt",sizeof(buffer_gyr_x)/sizeof(double));
								saveData(buffer_gyr_y_filt,"gyr_y_filt",sizeof(buffer_gyr_y)/sizeof(double));
								saveData(buffer_gyr_z_filt,"gyr_z_filt",sizeof(buffer_gyr_z)/sizeof(double));
								saveData(buffer_mag_x,"mag_x",sizeof(buffer_mag_x)/sizeof(double));
								saveData(buffer_mag_y,"mag_y",sizeof(buffer_mag_y)/sizeof(double));
								saveData(buffer_mag_z,"mag_z",sizeof(buffer_mag_z)/sizeof(double));	
								saveData(buffer_ts,"ts",sizeof(buffer_ts)/sizeof(double));
								buffer_counter=0;
							}
							else{ // else keep saving data to buffer
								//buffer_u1[buffer_counter]=uControl[0];
								//buffer_u2[buffer_counter]=uControl[1];
								//buffer_u3[buffer_counter]=uControl[2];
								//buffer_u4[buffer_counter]=uControl[3];
								////buffer_omega_x[buffer_counter]=ymeas9x9_bias[3];
								////buffer_omega_y[buffer_counter]=ymeas9x9_bias[4];
								////buffer_omega_z[buffer_counter]=ymeas9x9_bias[5];
								//buffer_omega_x[buffer_counter]=ymeas6x6[3];
								//buffer_omega_y[buffer_counter]=ymeas6x6[4];
								//buffer_omega_z[buffer_counter]=ymeas6x6[5];
								////buffer_angle_x[buffer_counter]=xhat9x9_bias[0];
								////buffer_angle_y[buffer_counter]=xhat9x9_bias[1];
								////buffer_angle_z[buffer_counter]=xhat9x9_bias[2];
								//buffer_thrust[buffer_counter]=uControlThrustTorques[0];
								//buffer_tau_x[buffer_counter]=uControlThrustTorques[1];
								//buffer_tau_y[buffer_counter]=uControlThrustTorques[2];
								//buffer_tau_z[buffer_counter]=uControlThrustTorques[3];
								buffer_acc_x_filt[buffer_counter]=accRaw[0];
								buffer_acc_y_filt[buffer_counter]=accRaw[1];
								buffer_acc_z_filt[buffer_counter]=accRaw[2];
								buffer_gyr_x_filt[buffer_counter]=gyrRaw[0];
								buffer_gyr_y_filt[buffer_counter]=gyrRaw[1];
								buffer_gyr_z_filt[buffer_counter]=gyrRaw[2];
								buffer_mag_x[buffer_counter]=magRaw[0];
								buffer_mag_y[buffer_counter]=magRaw[1];
								buffer_mag_z[buffer_counter]=magRaw[2];
								
								buffer_ts[buffer_counter]=tsTrue;
								buffer_counter++;
							}
						}
					}
				}
							
				/// Calculate next shot
				 t.tv_nsec += (int)tsSensorsFusion;
				 while (t.tv_nsec >= NSEC_PER_SEC) {
					 t.tv_nsec -= NSEC_PER_SEC;
					 t.tv_sec++;
				 }
			}
		}
		t.tv_sec+=2; // I2C sensors not enabled, sleep for 2 and retry
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

	}
	return NULL;
}

// Thread - PWM Control
static void *threadPWMControl(void *arg){
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	double pwmValueBuffer[8], tsTrue;

	// Initialize I2C connection to the PWM board and define PWM frequency
	pthread_mutex_lock(&mutexI2CBusy);
		int fdPWM=wiringPiI2CSetup(PWM_ADDRESS);
	pthread_mutex_unlock(&mutexI2CBusy);

	if(fdPWM==-1){
	 printf("Error setup the I2C PWM connection\n");
	}
	
	/// Setup timer variables for real time performance check
	struct timespec t_start,t_stop;
	
	/// Average sampling
	int tsAverageCounter=0;
	double tsAverageAccum=0;
	double tsAverage=tsController;
	int timerPrint=0;
	int killPWM=0; // switch [0=STOP, 1=FLY]
	int pwmRangeSetting=0; // switch for setting the PWM for tuning range of speed controllers
	
	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadPWMControl");
	}
	
	else{

		// Initialize PWM board
		pthread_mutex_lock(&mutexI2CBusy);
			enablePWM(fdPWM,500);
		pthread_mutex_unlock(&mutexI2CBusy);
		printf("PWM initialization complete\n");
		
		// Run forever and set PWM when controller computes new values
		while(1){
			/// Time it
			clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock

			// Get keyboard input data
			pthread_mutex_lock(&mutexKeyboardData);
				timerPrint=(int)keyboardData[5];
				killPWM=(int)keyboardData[3];
				pwmRangeSetting=(int)keyboardData[17];
			pthread_mutex_unlock(&mutexKeyboardData);
			
			// Read data from controller process
			if(read(ptrPipe->parent[0], pwmValueBuffer, sizeof(pwmValueBuffer)) == -1) printf("read error in sensor from controller\n");
			//printf("Data received: %f\n", pwmValueBuffer[0]);
			
			//printf("(pwm)  % 3.4f % 3.4f % 3.4f % 3.4f (thrust) % 1.4f (torque) % 1.4f % 1.4f % 1.4f\n", pwmValueBuffer[0], pwmValueBuffer[1], pwmValueBuffer[2], pwmValueBuffer[3], pwmValueBuffer[4], pwmValueBuffer[5], pwmValueBuffer[6], pwmValueBuffer[7]);
			
			// killPWM is linked to keyboard start flying switch. Forces PWM to zero if stop signal is given
			if(!killPWM){
				pwmValueBuffer[0]=0.0f;
				pwmValueBuffer[1]=0.0f;
				pwmValueBuffer[2]=0.0f;
				pwmValueBuffer[3]=0.0f;
			}
			
			if(!killPWM && pwmRangeSetting){
				pwmValueBuffer[0]=100.0f;
				pwmValueBuffer[1]=100.0f;
				pwmValueBuffer[2]=100.0f;
				pwmValueBuffer[3]=100.0f;
			}
				
			//else{
				//printf("(pwm)  % 3.4f % 3.4f % 3.4f % 3.4f\n", pwmValueBuffer[0], pwmValueBuffer[1], pwmValueBuffer[2], pwmValueBuffer[3]);
			//}
			

			
			// Saturation pwm 0-100%
			for(int i=0;i<4;i++){
				if(pwmValueBuffer[i]>100){
					pwmValueBuffer[i]=100;
				}
				else if(pwmValueBuffer[i]<0){
					pwmValueBuffer[i]=0;
				}
			}
			
			//printf("(pwm)  % 3.4f % 3.4f % 3.4f % 3.4f (thrust) % 1.4f (torque) % 1.4f % 1.4f % 1.4f\n", pwmValueBuffer[0], pwmValueBuffer[1], pwmValueBuffer[2], pwmValueBuffer[3], pwmValueBuffer[4], pwmValueBuffer[5], pwmValueBuffer[6], pwmValueBuffer[7]);
			
			
			// Copy control signal over to global memory for EKF to use during next state estimation
			pthread_mutex_lock(&mutexControlData);	
				memcpy(controlData, pwmValueBuffer, sizeof(controlData));		
			pthread_mutex_unlock(&mutexControlData);
			
			// Set PWM
			pthread_mutex_lock(&mutexI2CBusy);
				setPWM(fdPWM, pwmValueBuffer);
			pthread_mutex_unlock(&mutexI2CBusy);
			
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
					printf("PWM: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
					//printf("PWM received: %3.4f %3.4f %3.4f %3.4f\n", pwmValueBuffer[0], pwmValueBuffer[1], pwmValueBuffer[2], pwmValueBuffer[3]);
					//printf("(pwm)  % 3.4f % 3.4f % 3.4f % 3.4f (thrust) % 1.4f (torque) % 1.4f % 1.4f % 1.4f\n", pwmValueBuffer[0], pwmValueBuffer[1], pwmValueBuffer[2], pwmValueBuffer[3], pwmValueBuffer[4], pwmValueBuffer[5], pwmValueBuffer[6], pwmValueBuffer[7]);
				}
				tsAverageCounter=0;
				tsAverageAccum=0;
				
			}
			
		}
	}
	return NULL;
}


/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

// EKF calibration
 void ekfCalibration6x6(double *Rekf, double *ekf0, double *ekfCal, double *ymeas, int counterCal){
	 // Calibration routine to get mean, variance and std_deviation
	 if(counterCal==CALIBRATION){
		 // Mean (bias) accelerometer, gyroscope and magnetometer
		 for (int i=0;i<CALIBRATION;i++){
			 ekf0[0]+=ekfCal[i*6];
			 ekf0[1]+=ekfCal[i*6+1];
			 ekf0[2]+=ekfCal[i*6+2];
			 ekf0[3]+=ekfCal[i*6+3];
			 ekf0[4]+=ekfCal[i*6+4];
			 ekf0[5]+=ekfCal[i*6+5];
		 }
		 ekf0[0]/=CALIBRATION;
		 ekf0[1]/=CALIBRATION;
		 ekf0[2]/=CALIBRATION;
		 ekf0[3]/=CALIBRATION;
		 ekf0[4]/=CALIBRATION;
		 ekf0[5]/=CALIBRATION;
		
		 // Sum up for variance calculation
		 for (int i=0;i<CALIBRATION;i++){
			 Rekf[0]+=pow((ekfCal[i*6] - ekf0[0]), 2);
			 Rekf[7]+=pow((ekfCal[i*6+1] - ekf0[1]), 2);
			 Rekf[14]+=pow((ekfCal[i*6+2] - ekf0[2]), 2);
			 Rekf[21]+=pow((ekfCal[i*6+3] - ekf0[3]), 2);
			 Rekf[28]+=pow((ekfCal[i*6+4] - ekf0[4]), 2);
			 Rekf[35]+=pow((ekfCal[i*6+5] - ekf0[5]), 2);
		 }
		 // Variance (sigma)
		 Rekf[0]/=CALIBRATION;
		 Rekf[7]/=CALIBRATION;
		 Rekf[14]/=CALIBRATION;
		 Rekf[21]/=CALIBRATION;
		 Rekf[28]/=CALIBRATION;
		 Rekf[35]/=CALIBRATION;
	
		 // Print results
		 printf("Mean (bias) EKF 6x6\n");
		 printmat(ekf0,6,1);
		 printf("Covariance matrix (sigma) EKF 6x6\n");
		 printmat(Rekf,6,6);
	 }
	 // Default i save calibrartion data
	 else{
		 ekfCal[counterCal*6]=ymeas[0];
		 ekfCal[counterCal*6+1]=ymeas[1];
		 ekfCal[counterCal*6+2]=ymeas[2];
		 ekfCal[counterCal*6+3]=ymeas[3];
		 ekfCal[counterCal*6+4]=ymeas[4];
		 ekfCal[counterCal*6+5]=ymeas[5];
	 }		
 }

// EKF calibration bias
 void ekfCalibration9x9_bias(double *Rekf, double *ekf0, double *ekfCal, double *ymeas, int counterCal){
	 // Calibration routine to get mean, variance and std_deviation
	 if(counterCal==CALIBRATION){
		 // Mean (bias) accelerometer, gyroscope and magnetometer
		 for (int i=0;i<CALIBRATION;i++){
			 ekf0[0]+=ekfCal[i*6];
			 ekf0[1]+=ekfCal[i*6+1];
			 ekf0[2]+=ekfCal[i*6+2];
			 ekf0[3]+=ekfCal[i*6+3];
			 ekf0[4]+=ekfCal[i*6+4];
			 ekf0[5]+=ekfCal[i*6+5];
		 }
		 ekf0[0]/=CALIBRATION;
		 ekf0[1]/=CALIBRATION;
		 ekf0[2]/=CALIBRATION;
		 ekf0[3]/=CALIBRATION;
		 ekf0[4]/=CALIBRATION;
		 ekf0[5]/=CALIBRATION;
		
		 // Sum up for variance calculation
		 for (int i=0;i<CALIBRATION;i++){
			 Rekf[0]+=pow((ekfCal[i*6] - ekf0[0]), 2);
			 Rekf[7]+=pow((ekfCal[i*6+1] - ekf0[1]), 2);
			 Rekf[14]+=pow((ekfCal[i*6+2] - ekf0[2]), 2);
			 Rekf[21]+=pow((ekfCal[i*6+3] - ekf0[3]), 2);
			 Rekf[28]+=pow((ekfCal[i*6+4] - ekf0[4]), 2);
			 Rekf[35]+=pow((ekfCal[i*6+5] - ekf0[5]), 2);
		 }
		 // Variance (sigma)
		 Rekf[0]/=CALIBRATION;
		 Rekf[7]/=CALIBRATION;
		 Rekf[14]/=CALIBRATION;
		 Rekf[21]/=CALIBRATION;
		 Rekf[28]/=CALIBRATION;
		 Rekf[35]/=CALIBRATION;
	
		 // Print results
		 printf("Mean (bias) EKF 9x9 bias\n");
		 printmat(ekf0,6,1);
		 printf("Covariance matrix (sigma) EKF 9x9 bias\n");
		 printmat(Rekf,6,6);
	 }
	 // Default i save calibrartion data
	 else{
		 ekfCal[counterCal*6]=ymeas[0];
		 ekfCal[counterCal*6+1]=ymeas[1];
		 ekfCal[counterCal*6+2]=ymeas[2];
		 ekfCal[counterCal*6+3]=ymeas[3];
		 ekfCal[counterCal*6+4]=ymeas[4];
		 ekfCal[counterCal*6+5]=ymeas[5];
	 }		
 }

// EKF calibration
void ekfCalibration9x9(double *Rekf, double *ekf0, double *ekfCal, double *ymeas, int counterCal){
	// Calibration routine to get mean, variance and std_deviation
	if(counterCal==CALIBRATION){
		// Mean (bias) accelerometer, gyroscope and magnetometer
		for (int i=0;i<CALIBRATION;i++){
			ekf0[0]+=ekfCal[i*3];
			ekf0[1]+=ekfCal[i*3+1];
			ekf0[2]+=ekfCal[i*3+2];
		}
		ekf0[0]/=CALIBRATION;
		ekf0[1]/=CALIBRATION;
		ekf0[2]/=CALIBRATION;
		
		// Sum up for variance calculation
		for (int i=0;i<CALIBRATION;i++){
			Rekf[0]+=pow((ekfCal[i*3] - ekf0[0]), 2);
			Rekf[4]+=pow((ekfCal[i*3+1] - ekf0[1]), 2);
			Rekf[8]+=pow((ekfCal[i*3+2] - ekf0[2]), 2);
		}
		// Variance (sigma)
		Rekf[0]/=CALIBRATION;
		Rekf[4]/=CALIBRATION;
		Rekf[8]/=CALIBRATION;
		
		// Overide calibration when position measurements are gone
		Rekf[0]=1;
		Rekf[4]=1;
		Rekf[8]=1;
	
		// Print results
		printf("Mean (bias) EKF 9x9\n");
		printmat(ekf0,3,1);
		printf("Covariance matrix (sigma) EKF 9x9\n");
		printmat(Rekf,3,3);
	}
	// Default i save calibrartion data
	else{
		ekfCal[counterCal*3]=ymeas[0];
		ekfCal[counterCal*3+1]=ymeas[1];
		ekfCal[counterCal*3+2]=ymeas[2];
	}		
}

// Quaternions normalization
void qNormalize(double *q){
	double qnorm=sqrt(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2));
	q[0]/=qnorm;
	q[1]/=qnorm;
	q[2]/=qnorm;
	q[3]/=qnorm;
	if(q[0]<0){
		q[0]*=-1;
		q[1]*=-1;
		q[2]*=-1;
		q[3]*=-1;
	}
}

// Quaternions to Eulers (avoids gimbal lock)
void q2euler(double *result, double *q){
	// Handle north pole case
	if (q[1]*q[3]+q[0]*q[2] > 0.5){
		result[0]=2*atan2(q[1],q[0]);
		result[2]=0;
	}
	else{
		result[0]=atan2(-2*(q[0]*q[2]-q[0]*q[3]),1-2*(pow(q[2],2)+pow(q[3],2))); //heading
		result[2]=atan2(2*(q[2]*q[3]-q[0]*q[1]),1-2*(pow(q[1],2)+pow(q[2],2))); // bank
	}
	
	// Handle south pole case
	if (q[1]*q[3]+q[0]*q[2] < -0.5){
		result[0]=-2*atan2(q[1],q[0]);
		result[2]=0;
	}
	else{
		result[0]=atan2(-2*(q[0]*q[2]-q[0]*q[3]),1-2*(pow(q[2],2)+pow(q[3],2))); //heading
		result[2]=atan2(2*(q[2]*q[3]-q[0]*q[1]),1-2*(pow(q[1],2)+pow(q[2],2))); // bank
	}
	
	result[1]=asin(2*(q[1]*q[3]+q[0]*q[2])); // attitude
}

// Quaternions to Eulers according to ZYX rotation
void q2euler_zyx(double *result, double *q){
	double R[5];
	R[0] = 2.*pow(q[0],2)-1+2.*pow(q[1],2);
    R[1] = 2.*(q[1]*q[2]-q[0]*q[3]);
    R[2] = 2.*(q[1]*q[3]+q[0]*q[2]);
    R[3] = 2.*(q[2]*q[3]-q[0]*q[1]);
    R[4] = 2.*pow(q[0],2)-1+2.*pow(q[3],2);

	//R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    //R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    //R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    //R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    //R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

    result[2] = atan2(R[3],R[4]); // phi
    result[1] = -atan(R[2]/(sqrt(1-pow(R[2],2)))); // theta
    result[0] = atan2(R[1],R[0]); // psi
}

// Load settings file
int loadSettings(double *data, char* name, int size){
	// Create file pointer
	FILE *fp;
	
	float value;
	char string[30];
	int finish=0;
	//int lines=0;
	int i;
	
	
	// Open file and prepare for read
	fp=fopen("settings.txt", "r");
	// Check to see that file has opened
	if(fp==NULL){
		printf("File could not be opened for read\n");
	}
	else{
		while((fgets(string,30,fp)) != NULL && !finish){
			// Search for variable
			if(strstr(string,name) != NULL){
				//printf("%s\n",name);
				// Get variable data
				for(i=0;i<size;i++){
					if((fgets(string,30,fp)) != NULL && string[0]!='\n'){
						sscanf(string, "%f\n", &value);
						data[i]=(double)value;
						//printf("%f\n",value);
					}
					else{
						printf("Bad format or missing data on reading settings file\n");
					}
				}
				finish=1;
				printf("%s settings loaded\n",name);
			}
		}
		if(!finish){
			printf("%s not loaded (does not exist in settings.txt)\n",name);
		}
	}
	
	// Close file
	fclose(fp);
	return finish;
}

// Save settings file
//void saveSettings(double *data, char* name, int size, FILE **fp){
void saveSettings(double *data, char* name, int size){
	// Create file pointer
	FILE *fpWrite;
	int i;
	int finish=0;

	// Open file and prepare for write
	fpWrite=fopen("settings.txt", "a"); // "a" means append
	if(fpWrite==NULL){
		printf("File could not be opened for write\n");
	}
	else{
		if(!finish){	// if variable does not exist
			fprintf(fpWrite, "%s\n", name); // append variable name
			for(i=0;i<size;i++){
				fprintf(fpWrite, "%3.18f\n", data[i]); // append content
			}
			fprintf(fpWrite, "\n"); // newline
			printf("%s settings saved\n",name);
		}
		else{	// if variable does exist
			printf("%s not saved (lready exists in settings.txt)\n", name);
		}
	}
	
	// Close file
	fclose(fpWrite);
}

// Save data to file
//void saveData(double *data, char* name, int size, FILE **fp, int action){
void saveData(double *data, char* name, int size){
	// int action: 0=close, 1=open, NULL=nothing
	// Create file pointer
	FILE *fpWrite;
	int i;

	// Open file and prepare for write (append)
		fpWrite=fopen("data.txt", "a"); // "a" means append
	
	// Write to file
	if(fpWrite==NULL){
		printf("File could not be opened for write\n");
	}
	else{
		fprintf(fpWrite, "%s\n", name); // append variable name
		for(i=0;i<size;i++){
			fprintf(fpWrite, "%5.18f\n", data[i]); // append content
		}
		fprintf(fpWrite, "\n"); // newline
		//printf("%s settings saved\n",name);
	}
	
	// Close file
		fclose(fpWrite);
}

// Used to print the bits in a data type
void printBits(size_t const size, void const * const ptr){
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}

// State Observer - Extended Kalman Filter for 6 attitude states
void EKF_6x6(double *Phat, double *xhat, double *u, double *ymeas, double *Q, double *R, double Ts){
	// Local variables
	double xhat_pred[6]={0,0,0,0,0,0};
	double C[36]={1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1};
	double eye6[36]={1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1};
	double S_inv[36];
	double A[36], S[36], C_temp[36], Jfx_temp[36], Phat_pred[36], K_temp[36], K[36], V[6], xhat_temp[6], x_temp[6], fone=1, fzero=0;
	int n=6, k=6, m=6, ione=1;
	
	// Prediction step
	fx_6x1(xhat_pred, xhat, u, Ts); // state 
	Jfx_6x6(xhat, A, u, Ts); // update Jacobian A matrix
	
	// A*Phat_prev*A' + Q
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,A,&m,Phat,&k,&fzero,Jfx_temp,&m);
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Jfx_temp,&m,A,&n,&fzero,Phat_pred,&m);
	Phat_pred[0]+=Q[0];
	Phat_pred[7]+=Q[1];
	Phat_pred[14]+=Q[2];
	Phat_pred[21]+=Q[3];
	Phat_pred[28]+=Q[4];
	Phat_pred[35]+=Q[5];

	// Update step
	// S=C*P*C'+R; Innovation covariance
	n=6, k=6, m=6;
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,C,&m,Phat_pred,&k,&fzero,C_temp,&m);
	n=6, k=6, m=6;
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,C_temp,&m,C,&n,&fzero,S,&m);
	S[0]+=R[0];
	S[7]+=R[7];
	S[14]+=R[14];
	S[21]+=R[21];
	S[28]+=R[28];
	S[35]+=R[35];

	// K=P*C'*S^-1; Kalman gain
	n=6, k=6, m=6; // 6x6 * 6x6 = 6x6
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Phat_pred,&m,C,&n,&fzero,K_temp,&m);
	mInverse6x6(S,S_inv);
	n=6, k=6, m=6; // 6x6 * 6*6 = 6x6
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K_temp,&m,S_inv,&k,&fzero,K,&m);

	// V=y_meas-C*x_hat; Innovation
	n=6, m=6; 
	F77_CALL(dgemv)("n",&m,&n,&fone,C,&m,xhat_pred,&ione,&fzero,xhat_temp,&ione);
	V[0]=ymeas[0]-xhat_temp[0];
	V[1]=ymeas[1]-xhat_temp[1];
	V[2]=ymeas[2]-xhat_temp[2];
	V[3]=ymeas[3]-xhat_temp[3];
	V[4]=ymeas[4]-xhat_temp[4];
	V[5]=ymeas[5]-xhat_temp[5];
	
	//printf("\nymeas:\n");
	//printmat(ymeas,1,6);
	
	//printf("\nxhat_temp:\n");
	//printmat(xhat_temp,1,6);
		
	//printf("\nV:\n");
	//printmat(V,1,6);
	
	//printf("\nK:\n");
	//printmat(K,15,6);

	// x=x+K*v; State update
	n=6, m=6;
	F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,V,&ione,&fzero,x_temp,&ione);
	xhat[0]=xhat_pred[0]+x_temp[0];
	xhat[1]=xhat_pred[1]+x_temp[1];
	xhat[2]=xhat_pred[2]+x_temp[2];
	xhat[3]=xhat_pred[3]+x_temp[3];
	xhat[4]=xhat_pred[4]+x_temp[4];
	xhat[5]=xhat_pred[5]+x_temp[5];
	
	//printf("\nxhat\n");
	//printmat(xhat,15,1);
	
	// P=P-K*S*K'; Covariance update
	n=6, k=6, m=6; // 6x6 * 6x6 = 6x6
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K,&m,S,&k,&fzero,K_temp,&m); // K*S
	n=6, k=6, m=6; // 6x6 * 6x6 = 6x6
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_temp,&m,K,&n,&fzero,Phat,&m); // K_temp*K'
	n=6, k=6, m=6; // 6x6 * 6x6 = 6x6
	fzero=-1;
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,eye6,&m,Phat_pred,&k,&fzero,Phat,&m); // P=P-K*S*K'
	
	//printf("\nPhat\n");
	//printmat(Phat,15,15);
}

// State Observer - Extended Kalman Filter for 9 attitude states (including bias estimation)
 void EKF_9x9_bias(double *Phat, double *xhat, double *u, double *ymeas, double *Q, double *R, double Ts){
	 // Local variables
	 double xhat_pred[9]={0,0,0,0,0,0};
	 double C[54]={1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	 double eye9[81]={1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
	 double S_inv[36];
	 double A[81], S[36], C_temp[81], Jfx_temp[81], Phat_pred[81], K_temp[54], K[54], V[6], xhat_temp[6], x_temp[6], fone=1, fzero=0;
	 int n=9, k=9, m=9, ione=1;
	
	 // Prediction step
	 fx_9x1_bias(xhat_pred, xhat, u, Ts); // state 
	 Jfx_9x9_bias(xhat, A, u, Ts); // update Jacobian A matrix
	
	 // A*Phat_prev*A' + Q
	 F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,A,&m,Phat,&k,&fzero,Jfx_temp,&m);
	 F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Jfx_temp,&m,A,&n,&fzero,Phat_pred,&m);
	 Phat_pred[0]+=Q[0];
	 Phat_pred[10]+=Q[1];
	 Phat_pred[20]+=Q[2];
	 Phat_pred[30]+=Q[3];
	 Phat_pred[40]+=Q[4];
	 Phat_pred[50]+=Q[5];
	 Phat_pred[60]+=Q[6];
	 Phat_pred[70]+=Q[7];
	 Phat_pred[80]+=Q[8];

	 // Update step
	 // S=C*P*C'+R; Innovation covariance
	 n=9, k=9, m=6;
	 F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,C,&m,Phat_pred,&k,&fzero,C_temp,&m);
	 n=6, k=9, m=6;
	 F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,C_temp,&m,C,&n,&fzero,S,&m);
	 S[0]+=R[0];
	 S[7]+=R[7];
	 S[14]+=R[14];
	 S[21]+=R[21];
	 S[28]+=R[28];
	 S[35]+=R[35];

	 // K=P*C'*S^-1; Kalman gain
	 n=6, k=9, m=9; // 9x9 * 9x6 = 9x6
	 F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Phat_pred,&m,C,&n,&fzero,K_temp,&m);
	 mInverse6x6(S,S_inv);
	 n=6, k=6, m=9; // 9x6 * 6*6 = 9x6
	 F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K_temp,&m,S_inv,&k,&fzero,K,&m);

	 // V=y_meas-C*x_hat; Innovation
	 n=9, m=6; 
	 F77_CALL(dgemv)("n",&m,&n,&fone,C,&m,xhat_pred,&ione,&fzero,xhat_temp,&ione);
	 V[0]=ymeas[0]-xhat_temp[0];
	 V[1]=ymeas[1]-xhat_temp[1];
	 V[2]=ymeas[2]-xhat_temp[2];
	 V[3]=ymeas[3]-xhat_temp[3];
	 V[4]=ymeas[4]-xhat_temp[4];
	 V[5]=ymeas[5]-xhat_temp[5];
	 
	 //V[0]=0;
	 //V[1]=0;
	 //V[2]=0;
	 //V[3]=0;
	 //V[4]=0;
	 //V[5]=0;
	
	 //printf("\nymeas:\n");
	 //printmat(ymeas,1,6);
	
	 //printf("\nxhat_temp:\n");
	 //printmat(xhat_temp,1,6);
		
	 //printf("\nV:\n");
	 //printmat(V,1,6);
	
	 //printf("\nK:\n");
	 //printmat(K,15,6);

	 // x=x+K*v; State update
	 n=6, m=9;
	 F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,V,&ione,&fzero,x_temp,&ione);
	 xhat[0]=xhat_pred[0]+x_temp[0];
	 xhat[1]=xhat_pred[1]+x_temp[1];
	 xhat[2]=xhat_pred[2]+x_temp[2];
	 xhat[3]=xhat_pred[3]+x_temp[3];
	 xhat[4]=xhat_pred[4]+x_temp[4];
	 xhat[5]=xhat_pred[5]+x_temp[5];
	 xhat[6]=xhat_pred[6]+x_temp[6];
	 xhat[7]=xhat_pred[7]+x_temp[7];
	 xhat[8]=xhat_pred[8]+x_temp[8];
	
	 //printf("\nxhat\n");
	 //printmat(xhat,15,1);
	
	 // P=P-K*S*K'; Covariance update
	 n=6, k=6, m=9; // 9x6 * 6x6 = 9x6
	 F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K,&m,S,&k,&fzero,K_temp,&m); // K*S
	 n=9, k=6, m=9; // 9x6 * 6x9 = 9x9
	 F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_temp,&m,K,&n,&fzero,Phat,&m); // K_temp*K'
	 n=9, k=9, m=9; // 9x9 * 9x9 = 9x9
	 fzero=-1;
	 F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,eye9,&m,Phat_pred,&k,&fzero,Phat,&m); // P=P-K*S*K'
	
	 //printf("\nPhat\n");
	 //printmat(Phat,15,15);
 }

// State Observer - Extended Kalman Filter for 9 position states
void EKF_9x9(double *Phat, double *xhat, double *u, double *ymeas, double *Q, double *R, double Ts, int flag, double *par_att){
	// Local variables
	double xhat_pred[9]={0,0,0,0,0,0,0,0,0};
	double C[27]={1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double eye9[81]={1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
	double S_inv[9];
	double A[81], S[9], C_temp[27], Jfx_temp[81], Phat_pred[81], K_temp[27], K[27], V[3], xhat_temp[3], x_temp[9], fone=1, fzero=0;
	int n=9, k=9, m=9, ione=1;
	
	// Prediction step
	fx_9x1(xhat_pred, xhat, u, Ts, par_att); // state 
	Jfx_9x9(xhat, A, u, Ts, par_att); // update Jacobian A matrix
	
	// A*Phat_prev*A' + Q
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,A,&m,Phat,&k,&fzero,Jfx_temp,&m);
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Jfx_temp,&m,A,&n,&fzero,Phat_pred,&m);
	Phat_pred[0]+=Q[0];
	Phat_pred[10]+=Q[1];
	Phat_pred[20]+=Q[2];
	Phat_pred[30]+=Q[3];
	Phat_pred[40]+=Q[4];
	Phat_pred[50]+=Q[5];
	Phat_pred[60]+=Q[6];
	Phat_pred[70]+=Q[7];
	Phat_pred[80]+=Q[8];

	// Update step
	// S=C*P*C'+R; Innovation covariance
	n=9, k=9, m=3;
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,C,&m,Phat_pred,&k,&fzero,C_temp,&m);
	n=3, k=9, m=3;
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,C_temp,&m,C,&n,&fzero,S,&m);
	S[0]+=R[0];
	S[4]+=R[4];
	S[8]+=R[8];

	// K=P*C'*S^-1; Kalman gain
	n=3, k=9, m=9; // 9x9 * 9x3 = 9x3
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Phat_pred,&m,C,&n,&fzero,K_temp,&m);
	mInverse(S,S_inv);
	n=3, k=3, m=9; // 9x3 * 3*3 = 9x3
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K_temp,&m,S_inv,&k,&fzero,K,&m);

	// V=y_meas-C*x_hat; Innovation
	n=9, m=3; 
	F77_CALL(dgemv)("n",&m,&n,&fone,C,&m,xhat_pred,&ione,&fzero,xhat_temp,&ione);
	// Check if ymeas is new data
	if(flag==1){
		V[0]=0; // Old data, kill innovation
		V[1]=0; // Old data, kill innovation
		V[2]=0; // Old data, kill innovation
		//printf("Old data\n");
	}
	else{
		V[0]=ymeas[0]-xhat_temp[0];
		V[1]=ymeas[1]-xhat_temp[1];
		V[2]=ymeas[2]-xhat_temp[2];
	}
	
	//printf("\nymeas:\n");
	//printmat(ymeas,1,6);
	
	//printf("\nxhat_temp:\n");
	//printmat(xhat_temp,1,6);
		
	//printf("\nV:\n");
	//printmat(V,1,6);
	
	//printf("\nK:\n");
	//printmat(K,15,6);

	// x=x+K*v; State update
	n=3, m=9;
	F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,V,&ione,&fzero,x_temp,&ione);
	xhat[0]=xhat_pred[0]+x_temp[0];
	xhat[1]=xhat_pred[1]+x_temp[1];
	xhat[2]=xhat_pred[2]+x_temp[2];
	xhat[3]=xhat_pred[3]+x_temp[3];
	xhat[4]=xhat_pred[4]+x_temp[4];
	xhat[5]=xhat_pred[5]+x_temp[5];
	xhat[6]=xhat_pred[6]+x_temp[6];
	xhat[7]=xhat_pred[7]+x_temp[7];
	xhat[8]=xhat_pred[8]+x_temp[8];
	
	//printf("\nxhat\n");
	//printmat(xhat,15,1);
	
	// P=P-K*S*K'; Covariance update
	n=3, k=3, m=9; // 9x3 * 3x3 = 9x3
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K,&m,S,&k,&fzero,K_temp,&m); // K*S
	n=9, k=3, m=9; // 9x3 * 3x9 = 9x9
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_temp,&m,K,&n,&fzero,Phat,&m); // K_temp*K'
	n=9, k=9, m=9; // 9x9 * 9x9 = 9x9
	fzero=-1;
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,eye9,&m,Phat_pred,&k,&fzero,Phat,&m); // P=P-K*S*K'
	
	//printf("\nPhat\n");
	//printmat(Phat,15,15);
}

// Nonlinear Model for attitude states (6x1)
 void fx_6x1(double *xhat, double *xhat_prev, double *u, double Ts){
	 xhat[0]=xhat_prev[0] + Ts*(xhat_prev[3] + xhat_prev[5]*cos(xhat_prev[0])*tan(xhat_prev[1]) + xhat_prev[4]*sin(xhat_prev[0])*tan(xhat_prev[1]));
	 xhat[1]=xhat_prev[1] + Ts*(xhat_prev[4]*cos(xhat_prev[0]) - xhat_prev[5]*sin(xhat_prev[0]));
	 xhat[2]=xhat_prev[2] + Ts*((xhat_prev[5]*cos(xhat_prev[0]))/cos(xhat_prev[1]) + (xhat_prev[4]*sin(xhat_prev[0]))/cos(xhat_prev[1]));
	 xhat[3]=xhat_prev[3] - Ts*((xhat_prev[4]*xhat_prev[5]*(par_i_yy - par_i_zz))/par_i_xx - (par_L*par_c_m*par_k*(pow(u[0],2) - pow(u[2],2)))/par_i_xx);
	 xhat[4]=xhat_prev[4] + Ts*((xhat_prev[3]*xhat_prev[5]*(par_i_xx - par_i_zz))/par_i_yy + (par_L*par_c_m*par_k*(pow(u[1],2) - pow(u[3],2)))/par_i_yy);
	 xhat[5]=xhat_prev[5] + Ts*((par_b*par_c_m*(pow(u[0],2) - pow(u[1],2) + pow(u[2],2) - pow(u[3],2)))/par_i_zz - (xhat_prev[3]*xhat_prev[4]*(par_i_xx - par_i_yy))/par_i_zz);
 }

// Jacobian of model for attitude states (6x6)
 void Jfx_6x6(double *xhat, double *A, double *u, double Ts){
	 A[0]=Ts*(xhat[4]*cos(xhat[0])*tan(xhat[1]) - xhat[5]*sin(xhat[0])*tan(xhat[1])) + 1;A[1]=-Ts*(xhat[5]*cos(xhat[0]) + xhat[4]*sin(xhat[0]));A[2]=Ts*((xhat[4]*cos(xhat[0]))/cos(xhat[1]) - (xhat[5]*sin(xhat[0]))/cos(xhat[1]));A[3]=0;A[4]=0;A[5]=0;A[6]=Ts*(xhat[5]*cos(xhat[0])*(pow(tan(xhat[1]),2) + 1) + xhat[4]*sin(xhat[0])*(pow(tan(xhat[1]),2) + 1));A[7]=1;A[8]=Ts*((xhat[5]*cos(xhat[0])*sin(xhat[1]))/pow(cos(xhat[1]),2) + (xhat[4]*sin(xhat[0])*sin(xhat[1]))/pow(cos(xhat[1]),2));A[9]=0;A[10]=0;A[11]=0;A[12]=0;A[13]=0;A[14]=1;A[15]=0;A[16]=0;A[17]=0;A[18]=Ts;A[19]=0;A[20]=0;A[21]=1;A[22]=(Ts*xhat[5]*(par_i_xx - par_i_zz))/par_i_yy;A[23]=-(Ts*xhat[4]*(par_i_xx - par_i_yy))/par_i_zz;A[24]=Ts*sin(xhat[0])*tan(xhat[1]);A[25]=Ts*cos(xhat[0]);A[26]=(Ts*sin(xhat[0]))/cos(xhat[1]);A[27]=-(Ts*xhat[5]*(par_i_yy - par_i_zz))/par_i_xx;A[28]=1;A[29]=-(Ts*xhat[3]*(par_i_xx - par_i_yy))/par_i_zz;A[30]=Ts*cos(xhat[0])*tan(xhat[1]);A[31]=-Ts*sin(xhat[0]);A[32]=(Ts*cos(xhat[0]))/cos(xhat[1]);A[33]=-(Ts*xhat[4]*(par_i_yy - par_i_zz))/par_i_xx;A[34]=(Ts*xhat[3]*(par_i_xx - par_i_zz))/par_i_yy;A[35]=1;
 }

// Nonlinear Model for position states (9x1)
void fx_9x1(double *xhat, double *xhat_prev, double *u, double Ts, double *par_att){
	xhat[0]=xhat_prev[0] + Ts*xhat_prev[3];
	xhat[1]=xhat_prev[1] + Ts*xhat_prev[4];
	xhat[2]=xhat_prev[2] + Ts*xhat_prev[5];
	xhat[3]=xhat_prev[3] + Ts*(xhat_prev[6] - (par_k_d*xhat_prev[3])/par_mass + (par_c_m*par_k*(sin(par_att[0])*sin(par_att[2]) + cos(par_att[0])*cos(par_att[2])*sin(par_att[1]))*(pow(u[0],2) + pow(u[1],2) + pow(u[2],2) + pow(u[3],2)))/par_mass);
	xhat[4]=xhat_prev[4] - Ts*((par_k_d*xhat_prev[4])/par_mass - xhat_prev[7] + (par_c_m*par_k*(cos(par_att[2])*sin(par_att[0]) - cos(par_att[0])*sin(par_att[1])*sin(par_att[2]))*(pow(u[0],2) + pow(u[1],2) + pow(u[2],2) + pow(u[3],2)))/par_mass);
	xhat[5]=xhat_prev[5] + Ts*(xhat_prev[8] - (par_k_d*xhat_prev[5])/par_mass + (par_c_m*par_k*cos(par_att[0])*cos(par_att[1])*(pow(u[0],2) + pow(u[1],2) + pow(u[2],2) + pow(u[3],2)))/par_mass);
	xhat[6]=xhat_prev[6];
	xhat[7]=xhat_prev[7];
	xhat[8]=xhat_prev[8];
}

// Jacobian of model for position states (9x9)
void Jfx_9x9(double *xhat, double *A, double *u, double Ts, double *par_att){
	A[0]=1;A[1]=0;A[2]=0;A[3]=0;A[4]=0;A[5]=0;A[6]=0;A[7]=0;A[8]=0;A[9]=0;A[10]=1;A[11]=0;A[12]=0;A[13]=0;A[14]=0;A[15]=0;A[16]=0;A[17]=0;A[18]=0;A[19]=0;A[20]=1;A[21]=0;A[22]=0;A[23]=0;A[24]=0;A[25]=0;A[26]=0;A[27]=Ts;A[28]=0;A[29]=0;A[30]=1 - (Ts*par_k_d)/par_mass;A[31]=0;A[32]=0;A[33]=0;A[34]=0;A[35]=0;A[36]=0;A[37]=Ts;A[38]=0;A[39]=0;A[40]=1 - (Ts*par_k_d)/par_mass;A[41]=0;A[42]=0;A[43]=0;A[44]=0;A[45]=0;A[46]=0;A[47]=Ts;A[48]=0;A[49]=0;A[50]=1 - (Ts*par_k_d)/par_mass;A[51]=0;A[52]=0;A[53]=0;A[54]=0;A[55]=0;A[56]=0;A[57]=Ts;A[58]=0;A[59]=0;A[60]=1;A[61]=0;A[62]=0;A[63]=0;A[64]=0;A[65]=0;A[66]=0;A[67]=Ts;A[68]=0;A[69]=0;A[70]=1;A[71]=0;A[72]=0;A[73]=0;A[74]=0;A[75]=0;A[76]=0;A[77]=Ts;A[78]=0;A[79]=0;A[80]=1;
}

// Nonlinear Model for attitude states including bias estimation (9x1)
 void fx_9x1_bias(double *xhat, double *xhat_prev, double *u, double Ts){
	 xhat[0]=xhat_prev[0] + Ts*(xhat_prev[3] + xhat_prev[5]*cos(xhat_prev[0])*tan(xhat_prev[1]) + xhat_prev[4]*sin(xhat_prev[0])*tan(xhat_prev[1]));
	 xhat[1]=xhat_prev[1] + Ts*(xhat_prev[4]*cos(xhat_prev[0]) - xhat_prev[5]*sin(xhat_prev[0]));
	 xhat[2]=xhat_prev[2] + Ts*((xhat_prev[5]*cos(xhat_prev[0]))/cos(xhat_prev[1]) + (xhat_prev[4]*sin(xhat_prev[0]))/cos(xhat_prev[1]));
	 xhat[3]=xhat_prev[3] + Ts*(xhat_prev[6]/par_i_xx - (xhat_prev[4]*xhat_prev[5]*(par_i_yy - par_i_zz))/par_i_xx + (par_L*par_c_m*par_k*(pow(u[0],2) - pow(u[2],2)))/par_i_xx);
	 xhat[4]=xhat_prev[4] + Ts*(xhat_prev[7]/par_i_yy + (xhat_prev[3]*xhat_prev[5]*(par_i_xx - par_i_zz))/par_i_yy + (par_L*par_c_m*par_k*(pow(u[1],2) - pow(u[3],2)))/par_i_yy);
	 xhat[5]=xhat_prev[5] + Ts*(xhat_prev[8]/par_i_zz + (par_b*par_c_m*(pow(u[0],2) - pow(u[1],2) + pow(u[2],2) - pow(u[3],2)))/par_i_zz - (xhat_prev[3]*xhat_prev[4]*(par_i_xx - par_i_yy))/par_i_zz);
	 xhat[6]=xhat_prev[6];
	 xhat[7]=xhat_prev[7];
	 xhat[8]=xhat_prev[8];
 }

// Jacobian of model for attitude states including bias estimation (9x9)
 void Jfx_9x9_bias(double *xhat, double *A, double *u, double Ts){
	 A[0]=Ts*(xhat[4]*cos(xhat[0])*tan(xhat[1]) - xhat[5]*sin(xhat[0])*tan(xhat[1])) + 1;A[1]=-Ts*(xhat[5]*cos(xhat[0]) + xhat[4]*sin(xhat[0]));A[2]=Ts*((xhat[4]*cos(xhat[0]))/cos(xhat[1]) - (xhat[5]*sin(xhat[0]))/cos(xhat[1]));A[3]=0;A[4]=0;A[5]=0;A[6]=0;A[7]=0;A[8]=0;A[9]=Ts*(xhat[5]*cos(xhat[0])*(pow(tan(xhat[1]),2) + 1) + xhat[4]*sin(xhat[0])*(pow(tan(xhat[1]),2) + 1));A[10]=1;A[11]=Ts*((xhat[5]*cos(xhat[0])*sin(xhat[1]))/pow(cos(xhat[1]),2) + (xhat[4]*sin(xhat[0])*sin(xhat[1]))/pow(cos(xhat[1]),2));A[12]=0;A[13]=0;A[14]=0;A[15]=0;A[16]=0;A[17]=0;A[18]=0;A[19]=0;A[20]=1;A[21]=0;A[22]=0;A[23]=0;A[24]=0;A[25]=0;A[26]=0;A[27]=Ts;A[28]=0;A[29]=0;A[30]=1;A[31]=(Ts*xhat[5]*(par_i_xx - par_i_zz))/par_i_yy;A[32]=-(Ts*xhat[4]*(par_i_xx - par_i_yy))/par_i_zz;A[33]=0;A[34]=0;A[35]=0;A[36]=Ts*sin(xhat[0])*tan(xhat[1]);A[37]=Ts*cos(xhat[0]);A[38]=(Ts*sin(xhat[0]))/cos(xhat[1]);A[39]=-(Ts*xhat[5]*(par_i_yy - par_i_zz))/par_i_xx;A[40]=1;A[41]=-(Ts*xhat[3]*(par_i_xx - par_i_yy))/par_i_zz;A[42]=0;A[43]=0;A[44]=0;A[45]=Ts*cos(xhat[0])*tan(xhat[1]);A[46]=-Ts*sin(xhat[0]);A[47]=(Ts*cos(xhat[0]))/cos(xhat[1]);A[48]=-(Ts*xhat[4]*(par_i_yy - par_i_zz))/par_i_xx;A[49]=(Ts*xhat[3]*(par_i_xx - par_i_zz))/par_i_yy;A[50]=1;A[51]=0;A[52]=0;A[53]=0;A[54]=0;A[55]=0;A[56]=0;A[57]=Ts/par_i_xx;A[58]=0;A[59]=0;A[60]=1;A[61]=0;A[62]=0;A[63]=0;A[64]=0;A[65]=0;A[66]=0;A[67]=Ts/par_i_yy;A[68]=0;A[69]=0;A[70]=1;A[71]=0;A[72]=0;A[73]=0;A[74]=0;A[75]=0;A[76]=0;A[77]=Ts/par_i_zz;A[78]=0;A[79]=0;A[80]=1;
 }

// Saturation function
void saturation(double *var, int index, double limMin, double limMax){
	if(var[index]<limMin){
		var[index]=limMin;
	}
	else if(var[index]>limMax){
		var[index]=limMax;
	}
}

// Low Pass Filter 24 order
void lowPassFilter(double *accRaw, double* gyrRaw, double *accRawMem, double* gyrRawMem, double* b_acc, double* b_gyr){
	// Shift all old data in measurement memory by one element: new -> [,,,,] -> old
	for (int k = 24; k > 0; k--){        
		accRawMem[k]=accRawMem[k-1];		// x-axis
		accRawMem[k+25]=accRawMem[k-1+25];	// y-axis
		accRawMem[k+50]=accRawMem[k-1+50];	// z-axis
		gyrRawMem[k]=gyrRawMem[k-1];		// x-axis
		gyrRawMem[k+25]=gyrRawMem[k-1+25];	// y-axis
		gyrRawMem[k+50]=gyrRawMem[k-1+50];	// z-axis
	}
	
	// Assign fresh non filtered measurement to memory
	accRawMem[0]=accRaw[0];		// x-axis
	accRawMem[0+25]=accRaw[1];	// y-axis
	accRawMem[0+50]=accRaw[2];	// z-axis
	gyrRawMem[0]=gyrRaw[0];		// x-axis
	gyrRawMem[0+25]=gyrRaw[1];	// y-axis
	gyrRawMem[0+50]=gyrRaw[2];	// z-axis
	
	// Zero out measurement before adding filtered data to
	accRaw[0]=0;
	accRaw[1]=0;
	accRaw[2]=0;
	gyrRaw[0]=0;
	gyrRaw[1]=0;
	gyrRaw[2]=0;
	
	// Filter the data
	for(int i=0;i<25;i++){
		accRaw[0]+=b_acc[i]*accRawMem[i];		// x-axis
		accRaw[1]+=b_acc[i]*accRawMem[i+25];	// y-axis
		accRaw[2]+=b_acc[i]*accRawMem[i+50];	// z-axis
		gyrRaw[0]+=b_gyr[i]*gyrRawMem[i];		// x-axis
		gyrRaw[1]+=b_gyr[i]*gyrRawMem[i+25];	// y-axis
		gyrRaw[2]+=b_gyr[i]*gyrRawMem[i+50];	// z-axis

		// ORIGINAL y(k) = y(k) + b(i)*acc_x(k-i);
	}
}
