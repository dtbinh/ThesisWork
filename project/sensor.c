// SENSOR FUSION CODE

#include "sensor.h"
#include "startup.h"
#include "PWM.h"
#include "lapack.h"
#include "blas.h"
#include "MPU9250.h"

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
#include <time.h>
#include <sched.h>
#include <sys/mman.h>




#define PI 3.141592653589793
#define CALIBRATION 100

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadReadBeacon (void*);
static void *threadSensorFusion (void*);
static void *threadPWMControl (void*);
void Qq(double*, double*);
void dQqdq(double*, double*, double*, double*, double*, double*, double*);
void printmat(double*, int, int);
void getOrientationEulers(double*, double*, double*);
void qNormalize(double*);
void Sq(double*, double*, double);
void Somega(double*, double*);
void q2euler(double*, double*);
void mInverse(double*, double*);
void accelerometerUpdate(double*, double*, double*, double*, double*);
void gyroscopeUpdate(double*, double*, double*, double*, double);
void magnetometerUpdate(double*, double*, double*, double*, double*, double);
void sensorCalibration(double*, double*, double*, double*, double*, double*, double*, double*, double*, double*, double*, double*, int);
void loadSettings();
void saveSettings();
void kalmanFilterCV(double*, double*, double, double, double, double, int);
void kalmanFilterCA(double*, double*, double, double, double, double);
void kalmanFilterRW(double*, double*, double*, double*, double*, double);
void printBits(size_t const, void const * const);
int outlierGpsVelocity(double, double, double, double);

// Static variables for threads
static double sensorRawDataPosition[3]; // Global variable in sensor.c to communicate between IMU read and angle fusion threads

// Mutexes
static pthread_mutex_t mutexPositionSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexI2CBusy = PTHREAD_MUTEX_INITIALIZER;


int counter=0;


/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *arg1, void *arg2){
	// Create pipe array
	pipeArray pipeArrayStruct = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create thread
	pthread_t threadSenFus, threadPWMCtrl,threadReadPos;
	int res1, res2, res3;
	
	res1=pthread_create(&threadReadPos, NULL, &threadReadBeacon, NULL);
	res2=pthread_create(&threadSenFus, NULL, &threadSensorFusion, &pipeArrayStruct);
	res3=pthread_create(&threadPWMCtrl, NULL, &threadPWMControl, arg1);
	
	// If threads created successful, start them
	if (!res1) pthread_join( threadReadPos, NULL);
	if (!res2) pthread_join( threadSenFus, NULL);
	if (!res3) pthread_join( threadPWMCtrl, NULL);
}


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/


// Thread - Read position values
void *threadReadBeacon (void *arg){
	// Define local variables
	double posRaw[3];
	double beaconTimestamp = 0.0;
	double beaconTimestampPrev = 0.0;
	uint8_t data8[100];	//this must be number of beacons*14+6+2 at least!
	uint16_t data16;
	uint32_t data32;
	double beacons[16];	//each beacon needs 4 doubles, address and XYZ
	int n;
	double hedgehogReading[1] = {0};
	int beaconFlag = 0;
	int beaconsCoordinates = 0;
	//double dummy = 0.0;
	int fdBeacon;
	
	// Loop forever trying to connect to Beacon sensor via USB
	while(1){
		// Open serial communication
		if ((fdBeacon=serialOpen("/dev/ttyACM0", 115200)) < 0){
			fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		}
		else{
			// Activate wiringPiSetup
			if (wiringPiSetup() == -1){
				fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
			}
			
			// Loop for ever reading data
			while(1){	
				counter++;			 
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
							hedgehogReading[0] = 1.0; //reading mm				
							n = (int)(data8[4]);
							//printf("%d", n);
							for (int i=0;i<n;i++){
								data8[5+i] = serialGetchar(fdBeacon);
							}
							data8[5+n] = serialGetchar(fdBeacon); // This and (6+n) are CRC-16
							data8[6+n] = serialGetchar(fdBeacon);
							
							beaconTimestamp = (double)(data32 = data8[8] << 24| data8[7] << 16| data8[6] << 8| data8[5]);			
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
							beaconTimestampPrev = beaconTimestamp;
							
							//usleep(15000);
						}
						else if ((data16=data8[3] << 8| data8[2]) == 0x0001 && data8[1] == 0x47 && data8[4] == 0x10){
							hedgehogReading[0] = 2.0; //reading cm				
							n = (int)(data8[4]);
							//printf("%d", n);
							for (int i=0;i<n;i++){
								data8[5+i] = serialGetchar(fdBeacon);
							}
							data8[5+n] = serialGetchar(fdBeacon); // This and (6+n) are CRC-16
							data8[6+n] = serialGetchar(fdBeacon);
							
							beaconTimestamp = (double)(data32 = data8[8] << 24| data8[7] << 16| data8[6] << 8| data8[5]);			
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
							beaconTimestampPrev = beaconTimestamp;
							
							usleep(15000);
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
							//printf("%i beacons:\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f,\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f\n", data8[5], (int)beacons[0] ,beacons[1] ,beacons[2], beacons[3], (int)beacons[4], beacons[5], beacons[6], beacons[7], (int)beacons[8], beacons[9], beacons[10], beacons[11], (int)beacons[12], beacons[13], beacons[14], beacons[15]);
							
							usleep(15000);
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
							//printf("%i beacons:\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f,\nBeacon%i> %.3f %.3f %.3f, Beacon%i> %.3f %.3f %.3f\n", data8[5], (int)beacons[0] ,beacons[1] ,beacons[2], beacons[3], (int)beacons[4], beacons[5], beacons[6], beacons[7], (int)beacons[8], beacons[9], beacons[10], beacons[11], (int)beacons[12], beacons[13], beacons[14], beacons[15]);
							
							usleep(15000);
						}
						else{
							//printf("data8[1] -> %04x\n", data8[1]);
							//printf("Unrecognized code of data in packet -> %04x  and  %02x\n", data16, data8[1]);
							
							usleep(20000);
						}
					}
					else{
						usleep(20000);	// if it is not broadcasting 0xff
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
					
					usleep(15000);	// if Data not avail
				}
			}
			
		}
		sleep(5); // file not open sleep for 5
	}
	return NULL;
}

// Thread - Sensor fusion Orientation and Position
static void *threadSensorFusion (void *arg){
	// Get pipe array and define local variables
	pipeArray *pipeArrayStruct = arg;
	//structPipe *ptrPipe1 = pipeArray1->pipe1;
	structPipe *ptrPipe2 = pipeArrayStruct->pipe2;
	
	// Define local variables
	double accRaw[3]={0,0,0}, gyrRaw[3]={0,0,0}, magRaw[3]={0,0,0}, magRawRot[3], tempRaw=0, acc0[3]={0,0,0}, gyr0[3]={0,0,0}, mag0[3]={0,0,0}, accCal[3*CALIBRATION], gyrCal[3*CALIBRATION], magCal[3*CALIBRATION], euler[3]={0,0,0};
	double Racc[9]={0,0,0,0,0,0,0,0,0}, Rgyr[9]={0,0,0,0,0,0,0,0,0}, Rmag[9]={0,0,0,0,0,0,0,0,0}, P[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}, L=1, q[4]={1,0,0,0}, sensorDataBuffer[19]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	posRaw[3]={0,0,0}, xx[2]={0,0}, xy[2]={0,0}, xz[2]={0,0}, Px[4]={1,0,0,1}, Py[4]={1,0,0,1}, Pz[4]={1,0,0,1}, posRawPrev[3]={0,0,0};
	uint32_t desiredPeriod = 50, start;
	int calibrationCounter=0, posRawOldFlag=0;	
			
	// Enable acc, gyr, mag  and bmp sensors
	printf("Enabling sensors...\n");
	pthread_mutex_lock(&mutexI2CBusy);
		enableMPU9250();
		enableAK8963();
	pthread_mutex_unlock(&mutexI2CBusy);
	
	// Loop for ever
	while(1){
		// Timing
		//printf("Ts: %i\n", millis()-start2);
		start=millis();
		
		// Read raw sensor data from I2C bus to local variable
		pthread_mutex_lock(&mutexI2CBusy);	
			readAllSensorData(accRaw, gyrRaw, magRaw, &tempRaw);	
		pthread_mutex_unlock(&mutexI2CBusy);
		
		// Read raw position data from global to local variable
		pthread_mutex_lock(&mutexPositionSensorData);	
			memcpy(posRaw, sensorRawDataPosition, sizeof(sensorRawDataPosition));		
		pthread_mutex_unlock(&mutexPositionSensorData);
		
		// Convert sensor data to correct (filter) units:
		// Acc: g -> m/s² Factor: 9.81
		// Gyr: degrees/s -> radians/s Factor: (PI/180)
		// Mag: milli gauss -> micro tesla Factor: 10^-1
		accRaw[0]*=9.81;
		accRaw[1]*=9.81;
		accRaw[2]*=9.81;
		gyrRaw[0]*=(PI/180);
		gyrRaw[1]*=(PI/180);
		gyrRaw[2]*=(PI/180);
		magRaw[0]/=10;
		magRaw[1]/=10;
		magRaw[2]/=10;
		
		// Rotate magnetometer data such that the sensor coordinate frames match.
		// Rz(90), magX*(-1), magZ*(-1)
		// Note: For more info check the MPU9250 Product Specification (Chapter 9)
		magRawRot[0]=-magRaw[1]*(-1);
		magRawRot[1]=magRaw[0];
		magRawRot[2]=magRaw[3]*(-1);
		
		// Calibration routine
		if (calibrationCounter==0){
			printf("Sensor Calibration started\n");
			sensorCalibration(Racc, Rgyr, Rmag, acc0, gyr0, mag0, accCal, gyrCal, magCal, accRaw, gyrRaw, magRawRot, calibrationCounter);
			calibrationCounter++;
		}
		else if(calibrationCounter<CALIBRATION){
			sensorCalibration(Racc, Rgyr, Rmag, acc0, gyr0, mag0, accCal, gyrCal, magCal, accRaw, gyrRaw, magRawRot, calibrationCounter);
			calibrationCounter++;
			
		}
		else if(calibrationCounter==CALIBRATION){
			sensorCalibration(Racc, Rgyr, Rmag, acc0, gyr0, mag0, accCal, gyrCal, magCal, accRaw, gyrRaw, magRawRot, calibrationCounter);
			printf("Sensor Calibration finish\n");
			calibrationCounter++;
			xx[0]=posRaw[0];
			xy[0]=posRaw[1];
			xz[0]=posRaw[2];
		}
		// Sensor fusion
		else{
			// Orientation estimation
			accelerometerUpdate(q, P, accRaw, acc0, Racc);
			qNormalize(q);	
			gyroscopeUpdate(q, P, gyrRaw, Rgyr, (double)desiredPeriod/1000);
			qNormalize(q);
			magnetometerUpdate(q, P, magRawRot, mag0, Rmag, L);
			qNormalize(q);
			q2euler(euler,q);
			
			// Check if raw position data is new or old
			if(posRaw[0]==posRawPrev[0] && posRaw[1]==posRawPrev[1] && posRaw[2]==posRawPrev[2]){
				posRawOldFlag=1;
			}
			else{
				posRawOldFlag=0;
				memcpy(posRawPrev, posRaw, sizeof(posRaw));
			}
			
			// Check if raw poisition data has moved to much
			
			// Position estimation
			kalmanFilterCV(xx, Px, 0.0001, 0.00003212, posRaw[0], (double)desiredPeriod/1000, posRawOldFlag);
			kalmanFilterCV(xy, Py, 0.0001, 0.00004067, posRaw[1], (double)desiredPeriod/1000, posRawOldFlag);
			kalmanFilterCV(xz, Pz, 0.0001, 0.00005055, posRaw[2], (double)desiredPeriod/1000, posRawOldFlag);
			
			printf("X=%.3f, X=%.3f, Y=%.3f, Y=%.3f, Z=%.3f, Z=%.3f\n", posRaw[0], posRaw[1], posRaw[2], xx[0], xy[0], xz[0]);

			// Move over data to global variables for sending to controller process
			sensorDataBuffer[0]=gyrRaw[0];
			sensorDataBuffer[1]=gyrRaw[1];
			sensorDataBuffer[2]=gyrRaw[2];
			sensorDataBuffer[3]=accRaw[0];
			sensorDataBuffer[4]=accRaw[1];
			sensorDataBuffer[5]=accRaw[2];
			sensorDataBuffer[6]=magRaw[0];
			sensorDataBuffer[7]=magRaw[1];
			sensorDataBuffer[8]=magRaw[2];
			sensorDataBuffer[9]=euler[0];
			sensorDataBuffer[10]=euler[1];
			sensorDataBuffer[11]=euler[2];
			sensorDataBuffer[12]=q[0];
			sensorDataBuffer[13]=q[1];
			sensorDataBuffer[14]=q[2];
			sensorDataBuffer[15]=q[3];
			sensorDataBuffer[16]=xx[0];
			sensorDataBuffer[17]=xy[0];
			sensorDataBuffer[18]=xz[0];
			
			// Write to Controller process
			//if (write(ptrPipe1->child[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor to Controller\n");
			//else printf("Sensor ID: %d, Sent: %f to Controller\n", (int)getpid(), sensorDataBuffer[0]);
			
			// Write to Communication process
			if (write(ptrPipe2->parent[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor ot Communicaiont\n");
			//else printf("Sensor ID: %d, Sent: %f to Communication\n", (int)getpid(), sensorDataBuffer[0]);
		}
		
		// Sleep for desired sampling frequency
		if((millis()-start)<desiredPeriod)
			usleep(1000*(desiredPeriod-(millis()-start)));
	}
	return NULL;
}

// Thread - PWM Control
static void *threadPWMControl(void *arg){
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float pwmValueBuffer[4];
	
	// Initialize I2C connection to the PWM board and define PWM frequency
	pthread_mutex_lock(&mutexI2CBusy);
		int fdPWM=wiringPiI2CSetup(PWM_ADDRESS);
	pthread_mutex_unlock(&mutexI2CBusy);

	
	if(fdPWM==-1){
	 printf("Error setup the I2C PWM connection\n");
	}
	else{
		// Initialize PWM board
		pthread_mutex_lock(&mutexI2CBusy);
			enablePWM(fdPWM,500);
		pthread_mutex_unlock(&mutexI2CBusy);
		printf("PWM initialization complete\n");
		
		// Run forever and set PWM when controller computes new values
		while(1){
			// Read data from controller process
			if(read(ptrPipe->parent[0], pwmValueBuffer, sizeof(pwmValueBuffer)) == -1) printf("read error in sensor from controller\n");
			printf("Data received: %f\n", pwmValueBuffer[0]);
			
			// Set PWM
			pthread_mutex_lock(&mutexI2CBusy);
				setPWM(fdPWM, pwmValueBuffer);
			pthread_mutex_unlock(&mutexI2CBusy);
		}	
	}
	return NULL;
}


/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/
// Outlier GPS Velocity
int outlierGpsVelocity(double ymeas, double x_pred, double velMax, double Ts){
	if(fabs(ymeas-x_pred)>velMax*Ts){
		printf("Velocity outlier active\n");
		return 1;
	}
	else{
		return 0;
	}
}

// Kalman filter (Constant Velocity model)
void kalmanFilterCV(double *x, double *P, double q, double r, double ymeas, double Ts, int flag){
	// Declare filter variables
	double A_cv[4]={1,0,Ts,1}; // model augmented with bias in position
	double Q_cv[4]={pow(Ts,3)*q/3,pow(Ts,2)*q/2,pow(Ts,2)*q/2,Ts*q}; // Covariance
	double x_pred[2]={0,0}, P_pred[4]={0,0,0,0}, P_temp[4]={0,0,0,0}, S_temp[4], S[2], fone=1, fzero=0, C[2]={1,0}, K[2]={0,0}, V=0;
	
	// x[k|k-1]=A*x[k-1|k-1]; State prediction
	int ione=1, n=2, k=2, m=2;
	F77_CALL(dgemv)("n",&m,&n,&fone,A_cv,&m,x,&ione,&fzero,x_pred,&ione);
	
	// P[k|k-1]=A*P[k-1|k-1]*A'+Q; Covariance prediction
	n=2; k=2; m=2;
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,A_cv,&m,P,&k,&fzero,P_temp,&m);
	n=2; k=2; m=2;
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,P_temp,&m,A_cv,&n,&fzero,P_pred,&m);
	P_pred[0]+=Q_cv[0];
	P_pred[1]+=Q_cv[1];
	P_pred[2]+=Q_cv[2];
	P_pred[3]+=Q_cv[3];	
	
	// S=C*P*C'+R; Innovation covariance
	double C_temp[4]={1,0,0,0};
	n=2; k=2; m=2;
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,C_temp,&m,P_pred,&k,&fzero,S_temp,&m);
	F77_CALL(dgemv)("t",&m,&n,&fone,S_temp,&m,C,&ione,&fzero,S,&ione);
	S[0]+=r;
	
	// K=P*C'*S^-1; Kalman gain
	F77_CALL(dgemv)("t",&m,&n,&fone,P_pred,&m,C,&ione,&fzero,K,&ione);
	K[0]/=S[0];
	K[1]/=S[0];
	
	// V=yk-C*x;
	// Check if ymeas is new data
	if(flag==1 || outlierGpsVelocity(ymeas, x_pred[0], 0.5, Ts)==1){
		V=0;
		//printf("Old data\n");
	}
	else{
		V=ymeas-x_pred[0];
	}
	
	// x=x+K*v;
	x[0]=x_pred[0]+K[0]*V;
	x[1]=x_pred[1]+K[1]*V;
	
	// P=P-K*S*K';
	double K_tempS[4]={K[0]*S[0],K[1]*S[0],0,0};
	double K_temp[4]={K[0],K[1],0,0};
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_tempS,&m,K_temp,&k,&fzero,P_temp,&m);	
	P[0]=P_pred[0]-P_temp[0];
	P[1]=P_pred[1]-P_temp[1];
	P[2]=P_pred[2]-P_temp[2];
	P[3]=P_pred[3]-P_temp[3];
}

// Kalman filter (Constant Acceleration model)
void kalmanFilterCA(double *x, double *P, double q, double r, double ymeas, double Ts){
// Declare filter variables
	double A_ca[9]={1,0,0,Ts,1,0,pow(Ts,2)/2,Ts,1}; // model augmented with bias in position
	double Q_ca[9]={pow(Ts,5)*q/20,pow(Ts,4)*q/8,pow(Ts,3)*q/6,pow(Ts,4)*q/8,pow(Ts,3)*q/3,pow(Ts,2)*q/2,pow(Ts,3)*q/6,pow(Ts,2)*q/2,Ts*q}; // Covariance
	double x_pred[3]={0,0,0}, P_pred[9]={0,0,0,0,0,0,0,0,0}, P_temp[9]={0,0,0,0,0,0,0,0,0}, S_temp[9], S[3], fone=1, fzero=0, C[3]={1,0,0}, K[3]={0,0,0}, V=0;
	
	// x[k|k-1]=A*x[k-1|k-1]; State prediction
	int ione=1, n=3, k=3, m=3;
	F77_CALL(dgemv)("n",&m,&n,&fone,A_ca,&m,x,&ione,&fzero,x_pred,&ione);
	
	// P[k|k-1]=A*P[k-1|k-1]*A'+Q; Covariance prediction
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,A_ca,&m,P,&k,&fzero,P_temp,&m);
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,P_temp,&m,A_ca,&n,&fzero,P_pred,&m);
	P_pred[0]+=Q_ca[0];
	P_pred[1]+=Q_ca[1];
	P_pred[2]+=Q_ca[2];
	P_pred[3]+=Q_ca[3];
	P_pred[4]+=Q_ca[4];
	P_pred[5]+=Q_ca[5];
	P_pred[6]+=Q_ca[6];
	P_pred[7]+=Q_ca[7];
	P_pred[8]+=Q_ca[8];	
	
	// S=C*P*C'+R; Innovation covariance
	double C_temp[9]={1,0,0,0,0,0,0,0,0};
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,C_temp,&m,P_pred,&k,&fzero,S_temp,&m);
	F77_CALL(dgemv)("t",&m,&n,&fone,S_temp,&m,C,&ione,&fzero,S,&ione);
	S[0]+=r;
	
	// K=P*C'*S^-1; Kalman gain
	F77_CALL(dgemv)("t",&m,&n,&fone,P_pred,&m,C,&ione,&fzero,K,&ione);
	K[0]/=S[0];
	K[1]/=S[0];
	K[2]/=S[0];
	
	// V=yk-C*x; 
	V=ymeas-x_pred[0];
	
	// x=x+K*v;
	x[0]=x_pred[0]+K[0]*V;
	x[1]=x_pred[1]+K[1]*V;
	x[2]=x_pred[2]+K[2]*V;
	
	// P=P-K*S*K';
	double K_tempS[9]={K[0]*S[0],K[1]*S[0],K[2]*S[0],0,0,0,0,0,0};
	double K_temp[9]={K[0],K[1],K[2],0,0,0,0,0,0};
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_tempS,&m,K_temp,&k,&fzero,P_temp,&m);	
	P[0]=P_pred[0]-P_temp[0];
	P[1]=P_pred[1]-P_temp[1];
	P[2]=P_pred[2]-P_temp[2];
	P[3]=P_pred[3]-P_temp[3];
	P[4]=P_pred[4]-P_temp[4];
	P[5]=P_pred[5]-P_temp[5];
	P[6]=P_pred[6]-P_temp[6];
	P[7]=P_pred[7]-P_temp[7];
	P[8]=P_pred[8]-P_temp[8];
}

// Kalman filter (Random Walk model)
void kalmanFilterRW(double *x, double *P, double *q, double *r, double *ymeas, double Ts){
// Declare filter variables
	double A_ca[9]={1,0,0,0,1,0,0,0,1}; // random walk model (identity)
	double Q_ca[9]={q[0],0,0,0,q[1],0,0,0,q[2]}; // Covariance
	double x_pred[3]={0,0,0}, P_pred[9]={0,0,0,0,0,0,0,0,0}, P_temp[9]={0,0,0,0,0,0,0,0,0}, S_temp[9]={0,0,0,0,0,0,0,0,0}, S[9]={0,0,0,0,0,0,0,0,0}, fone=1, fzero=0, C[9]={1,0,0,0,1,0,0,0,1}, K[9]={0,0,0,0,0,0,0,0,0}, V[3];
	
	// x[k|k-1]=A*x[k-1|k-1]; State prediction
	int ione=1, n=3, k=3, m=3;
	F77_CALL(dgemv)("n",&m,&n,&fone,A_ca,&m,x,&ione,&fzero,x_pred,&ione);
	
	// P[k|k-1]=A*P[k-1|k-1]*A'+Q; Covariance prediction
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,A_ca,&m,P,&k,&fzero,P_temp,&m);
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,P_temp,&m,A_ca,&n,&fzero,P_pred,&m);
	P_pred[0]+=Q_ca[0];
	P_pred[1]+=Q_ca[1];
	P_pred[2]+=Q_ca[2];
	P_pred[3]+=Q_ca[3];
	P_pred[4]+=Q_ca[4];
	P_pred[5]+=Q_ca[5];
	P_pred[6]+=Q_ca[6];
	P_pred[7]+=Q_ca[7];
	P_pred[8]+=Q_ca[8];	
	
	// S=C*P*C'+R; Innovation covariance
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,C,&m,P_pred,&k,&fzero,S_temp,&m);
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,S_temp,&m,C,&k,&fzero,S,&m);
	S[0]+=r[0];
	S[4]+=r[1];
	S[8]+=r[2];
	
	
	// K=P*C'*S^-1; Kalman gain
	double K_temp[9];
	double S_inv[9];
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,P_pred,&m,C,&k,&fzero,K_temp,&m);
	mInverse(S,S_inv);
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K_temp,&m,S_inv,&k,&fzero,K,&m);
	
	// V=yk-C*x; 
	V[0]=ymeas[0]-x_pred[0];
	V[1]=ymeas[1]-x_pred[1];
	V[2]=ymeas[2]-x_pred[2];
	
	// x=x+K*v;
	double x_temp[3];
	F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,V,&ione,&fzero,x_temp,&ione);
	x[0]=x_pred[0]+x_temp[0];
	x[1]=x_pred[1]+x_temp[1];
	x[2]=x_pred[2]+x_temp[2];
	
	// P=P-K*S*K';
	//double K_tempS[9]={K[0]*S[0],K[1]*S[0],K[2]*S[0],0,0,0,0,0,0};
	//double K_temp[9]={K[0],K[1],K[2],0,0,0,0,0,0};
	F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K,&m,S,&k,&fzero,K_temp,&m);	
	F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_temp,&m,K,&k,&fzero,P_temp,&m);	
	P[0]=P_pred[0]-P_temp[0];
	P[1]=P_pred[1]-P_temp[1];
	P[2]=P_pred[2]-P_temp[2];
	P[3]=P_pred[3]-P_temp[3];
	P[4]=P_pred[4]-P_temp[4];
	P[5]=P_pred[5]-P_temp[5];
	P[6]=P_pred[6]-P_temp[6];
	P[7]=P_pred[7]-P_temp[7];
	P[8]=P_pred[8]-P_temp[8];
}

// Accelerometer part: mu_g
void accelerometerUpdate(double *q, double *P, double *yacc, double *g0, double *Ra){
	// local variables
	int ione=1, n=3, k=3, m=3;
	double fone=1, fzero=0, yka[3], yka2[9], Q[9], h1[9], h2[9], h3[9], h4[9], hd[12], Sacc[9], P_temp[16], S_temp[12], K_temp[12], Sacc_inv[9], yacc_diff[3], state_temp[4];
	double fka[3]={0,0,0}, K[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
	// check if acc is valid (isnan and all!=0)
	// outlier detection
	if (sqrt(pow(yacc[0],2) + pow(yacc[1],2) + pow(yacc[2],2)) > 1.2* sqrtf(pow(g0[0],2) + pow(g0[1],2) + pow(g0[2],2))){
		// dont use measurement
		//printf("Accelerometer Outlier\n");
	}
	else{
		// continue measurement
		// mu_g
		// yka=Qq(x)'*yka2=Qq(x)'*(g0+fka); accelerometer and quaternion model relation
		Qq(Q, q);
		yka2[0]=g0[0]+fka[0];
		yka2[1]=g0[1]+fka[1];
		yka2[2]=g0[2]+fka[2];

		F77_CALL(dgemv)("t",&m,&n,&fone,Q,&m,yka2,&ione,&fzero,yka,&ione);
		
		// [h1 h2 h3 h4]=dQqdq(x); jacobian
		// hd=[h1'*g0 h2'*g0 h3'*g0 h4'*g0];
		dQqdq(h1, h2, h3, h4, hd, q, g0);	
		
		// S=hd*P*hd'+Ra; innovation covariance
		n=4; k=4; m=3;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,hd,&m,P,&k,&fzero,S_temp,&m);
		n=3; k=4; m=3;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,S_temp,&m,hd,&n,&fzero,Sacc,&m);
		Sacc[0]+=Ra[0];
		Sacc[1]+=Ra[1];
		Sacc[2]+=Ra[2];
		Sacc[3]+=Ra[3];
		Sacc[4]+=Ra[4];
		Sacc[5]+=Ra[5];
		Sacc[6]+=Ra[6];
		Sacc[7]+=Ra[7];
		Sacc[8]+=Ra[8];
		
		// K=P*hd'/S; kalman gain
		n=3; k=4; m=4;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,P,&m,hd,&n,&fzero,K_temp,&m);
		mInverse(Sacc, Sacc_inv);
		n=3; k=3; m=4;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K_temp,&m,Sacc_inv,&k,&fzero,K,&m);
				
		// x=x+K*yacc_diff=x+K*(yacc-yka); state update
		yacc_diff[0]=yacc[0]-yka[0];
		yacc_diff[1]=yacc[1]-yka[1];
		yacc_diff[2]=yacc[2]-yka[2];
		n=3, k=4, m=4;
		F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,yacc_diff,&ione,&fzero,state_temp,&ione);
		q[0]+=state_temp[0];
		q[1]+=state_temp[1];
		q[2]+=state_temp[2];
		q[3]+=state_temp[3];

		// P=P-K*S*K'; covariance update
		n=3; k=3; m=4;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K,&m,Sacc,&k,&fzero,K_temp,&m);
		n=4; k=3; m=4;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_temp,&m,K,&n,&fzero,P_temp,&m);
		P[0]-=P_temp[0];
		P[1]-=P_temp[1];
		P[2]-=P_temp[2];
		P[3]-=P_temp[3];
		P[4]-=P_temp[4];
		P[5]-=P_temp[5];
		P[6]-=P_temp[6];
		P[7]-=P_temp[7];
		P[8]-=P_temp[8];
		P[9]-=P_temp[9];
		P[10]-=P_temp[10];
		P[11]-=P_temp[11];
		P[12]-=P_temp[12];
		P[13]-=P_temp[13];
		P[14]-=P_temp[14];
		P[15]-=P_temp[15];
	}
}

// Gyroscope part: tu_qw
void gyroscopeUpdate(double *q, double *P, double *ygyr, double *Rw, double Ts){
	// local variables
	int ione=1, n=4, k=4, m=4;
	double fone=1, fzero=0, Gm[12], Sm[16], F[16], q_temp[4], F_temp[16], P_temp[16], Gm_temp12[12], Gm_temp16[16];
	
	// check if gyr is valid (isnan and all!=0)
	// outlier detection
	if (ygyr[0]==0 && ygyr[1]==0 && ygyr[2]==0){
		// dont use measurement
		//printf("Gyroscope Outlier\n");
	}
	else{
		// continue measurement
		// tu_qw
		// Gm=Sq(x)*0.5*T, ;
		Sq(Gm, q, Ts);
		
		// F=eye(4)+Somega(omega)*0.5*T;
		Somega(Sm,ygyr);
		F[0]=1+Sm[0]*0.5*Ts;
		F[1]=0+Sm[1]*0.5*Ts;
		F[2]=0+Sm[2]*0.5*Ts;
		F[3]=0+Sm[3]*0.5*Ts;
		F[4]=0+Sm[4]*0.5*Ts;
		F[5]=1+Sm[5]*0.5*Ts;
		F[6]=0+Sm[6]*0.5*Ts;
		F[7]=0+Sm[7]*0.5*Ts;
		F[8]=0+Sm[8]*0.5*Ts;
		F[9]=0+Sm[9]*0.5*Ts;
		F[10]=1+Sm[10]*0.5*Ts;
		F[11]=0+Sm[11]*0.5*Ts;
		F[12]=0+Sm[12]*0.5*Ts;
		F[13]=0+Sm[13]*0.5*Ts;
		F[14]=0+Sm[14]*0.5*Ts;
		F[15]=1+Sm[15]*0.5*Ts;
		
		// x=F*x; predicted state estimate
		F77_CALL(dgemv)("n",&m,&n,&fone,F,&m,q,&ione,&fzero,q_temp,&ione);	
		memcpy(q, q_temp, sizeof(q_temp));
		
		// P=F_temp*F'+G_temp*G'=F*P*F'+G*Rw*G'=P_temp+G_temp2; predicted covariance estimate
		n=4; k=4; m=4;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,F,&m,P,&k,&fzero,F_temp,&m);
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,F_temp,&m,F,&n,&fzero,P_temp,&m);
		n=3; k=3; m=4;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,Gm,&m,Rw,&k,&fzero,Gm_temp12,&m);
		n=4; k=3; m=4;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,Gm_temp12,&m,Gm,&n,&fzero,Gm_temp16,&m);	
		P[0]=P_temp[0]+Gm_temp16[0];
		P[1]=P_temp[1]+Gm_temp16[1];
		P[2]=P_temp[2]+Gm_temp16[2];
		P[3]=P_temp[3]+Gm_temp16[3];
		P[4]=P_temp[4]+Gm_temp16[4];
		P[5]=P_temp[5]+Gm_temp16[5];
		P[6]=P_temp[6]+Gm_temp16[6];
		P[7]=P_temp[7]+Gm_temp16[7];
		P[8]=P_temp[8]+Gm_temp16[8];
		P[9]=P_temp[9]+Gm_temp16[9];
		P[10]=P_temp[10]+Gm_temp16[10];
		P[11]=P_temp[11]+Gm_temp16[11];
		P[12]=P_temp[12]+Gm_temp16[12];
		P[13]=P_temp[13]+Gm_temp16[13];
		P[14]=P_temp[14]+Gm_temp16[14];
		P[15]=P_temp[15]+Gm_temp16[15];
	}
}

// Magnetometer part: mu_m
void magnetometerUpdate(double *q, double *P, double *ymag, double *m0, double *Rm, double L){
	// local variables
	int ione=1, n=3, k=3, m=3;
	double fone=1, fzero=0, ykm[3], ykm2[9], Q[9], h1[9], h2[9], h3[9], h4[9], hd[12], Smag[9], P_temp[16], S_temp[12], K_temp[12], Smag_inv[9], ymag_diff[3], state_temp[4];
	double fkm[3]={0,0,0}, K[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}, a=0.01;
	
	// check if acc is valid (isnan and all!=0)
	// outlier detection
	L=(1-a)*L+a*sqrt(pow(ymag[0],2) + pow(ymag[1],2) + pow(ymag[2],2)); // recursive magnetometer compensator
	if (sqrt(pow(ymag[0],2) + pow(ymag[1],2) + pow(ymag[2],2)) > L){
		// dont use measurement
		//printf("Magnetometer Outlier\n");
	}
	else{
		// continue measurement
		// mu_m
		// ykm=Qq(x)'*ykm2=Qq(x)'*(m0+fkm); magnetometer and quaternion model relation
		Qq(Q, q);
		ykm2[0]=m0[0]+fkm[0];
		ykm2[1]=m0[1]+fkm[1];
		ykm2[2]=m0[2]+fkm[2];
		F77_CALL(dgemv)("t",&m,&n,&fone,Q,&m,ykm2,&ione,&fzero,ykm,&ione);
		
		// [h1 h2 h3 h4]=dQqdq(x); jacobian
		// hd=[h1'*m0 h2'*m0 h3'*m0 h4'*m0];
		dQqdq(h1, h2, h3, h4, hd, q, m0);	
		
		// Smag=hd*P*hd'+Rm; innovation covariance
		n=4; k=4; m=3;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,hd,&m,P,&k,&fzero,S_temp,&m);
		n=3; k=4; m=3;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,S_temp,&m,hd,&n,&fzero,Smag,&m);
		Smag[0]+=Rm[0];
		Smag[1]+=Rm[1];
		Smag[2]+=Rm[2];
		Smag[3]+=Rm[3];
		Smag[4]+=Rm[4];
		Smag[5]+=Rm[5];
		Smag[6]+=Rm[6];
		Smag[7]+=Rm[7];
		Smag[8]+=Rm[8];

		// K=P*hd'/Smag; kalman gain
		n=3; k=4; m=4;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,P,&m,hd,&n,&fzero,K_temp,&m);	
		mInverse(Smag, Smag_inv);
		n=3; k=3; m=4;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K_temp,&m,Smag_inv,&k,&fzero,K,&m);		
				
		// x=x+K*ymag_diff=x+K*(ymag-ykm); state update
		ymag_diff[0]=ymag[0]-ykm[0];
		ymag_diff[1]=ymag[1]-ykm[1];
		ymag_diff[2]=ymag[2]-ykm[2];
		n=3, k=4, m=4;
		F77_CALL(dgemv)("n",&m,&n,&fone,K,&m,ymag_diff,&ione,&fzero,state_temp,&ione);
		q[0]+=state_temp[0];
		q[1]+=state_temp[1];
		q[2]+=state_temp[2];
		q[3]+=state_temp[3];
		
		// P=P-K*S*K'; covariance update
		n=3; k=3; m=4;
		F77_CALL(dgemm)("n","n",&m,&n,&k,&fone,K,&m,Smag,&k,&fzero,K_temp,&m);
		n=4; k=3; m=4;
		F77_CALL(dgemm)("n","t",&m,&n,&k,&fone,K_temp,&m,K,&n,&fzero,P_temp,&m);
		P[0]-=P_temp[0];
		P[1]-=P_temp[1];
		P[2]-=P_temp[2];
		P[3]-=P_temp[3];
		P[4]-=P_temp[4];
		P[5]-=P_temp[5];
		P[6]-=P_temp[6];
		P[7]-=P_temp[7];
		P[8]-=P_temp[8];
		P[9]-=P_temp[9];
		P[10]-=P_temp[10];
		P[11]-=P_temp[11];
		P[12]-=P_temp[12];
		P[13]-=P_temp[13];
		P[14]-=P_temp[14];
		P[15]-=P_temp[15];
	}

}

// Sensor calibration
void sensorCalibration(double *Racc, double *Rgyr, double *Rmag, double *acc0, double *gyr0, double *mag0, double *accCal, double *gyrCal, double *magCal, double *yacc, double *ygyr, double *ymag, int counterCal){
	// Calibration routine to get mean, variance and std_deviation
	if(counterCal==CALIBRATION){
		// Mean (bias) accelerometer, gyroscope and magnetometer
		for (int i=0;i<CALIBRATION;i++){
			acc0[0]+=accCal[i*3];
			acc0[1]+=accCal[i*3+1];
			acc0[2]+=accCal[i*3+2];
			gyr0[0]+=gyrCal[i*3];
			gyr0[1]+=gyrCal[i*3+1];
			gyr0[2]+=gyrCal[i*3+2];
			mag0[0]+=magCal[i*3];
			mag0[1]+=magCal[i*3+1];
			mag0[2]+=magCal[i*3+2];
		}
		acc0[0]/=CALIBRATION;
		acc0[1]/=CALIBRATION;
		acc0[2]/=CALIBRATION;
		gyr0[0]/=CALIBRATION;
		gyr0[1]/=CALIBRATION;
		gyr0[2]/=CALIBRATION;
		mag0[0]/=CALIBRATION;
		mag0[1]/=CALIBRATION;
		mag0[2]/=CALIBRATION;
		
		// Sum up for variance calculation
		for (int i=0;i<CALIBRATION;i++){
			Racc[0]+=pow((accCal[i*3] - acc0[0]), 2);
			Racc[4]+=pow((accCal[i*3+1] - acc0[1]), 2);
			Racc[8]+=pow((accCal[i*3+2] - acc0[2]), 2);
			Rgyr[0]+=pow((gyrCal[i*3] - gyr0[0]), 2);
			Rgyr[4]+=pow((gyrCal[i*3+1] - gyr0[1]), 2);
			Rgyr[8]+=pow((gyrCal[i*3+2] - gyr0[2]), 2);
			Rmag[0]+=pow((magCal[i*3] - mag0[0]), 2);
			Rmag[4]+=pow((magCal[i*3+1] - mag0[1]), 2);
			Rmag[8]+=pow((magCal[i*3+2] - mag0[2]), 2);
		}
		// Variance (sigma)
		Racc[0]/=CALIBRATION;
		Racc[4]/=CALIBRATION;
		Racc[8]/=CALIBRATION;
		Rgyr[0]/=CALIBRATION;
		Rgyr[4]/=CALIBRATION;
		Rgyr[8]/=CALIBRATION;
		Rmag[0]/=CALIBRATION;
		Rmag[4]/=CALIBRATION;
		Rmag[8]/=CALIBRATION;
		
		// Print results
		printf("Mean (bias) accelerometer\n");
		printmat(acc0,3,1);
		printf("Mean (bias) gyroscope\n");
		printmat(gyr0,3,1);
		printf("Mean (bias) magnetometer\n");
		printmat(mag0,3,1);
		printf("Covariance matrix (sigma) accelerometer\n");
		printmat(Racc,3,3);
		printf("Covariance (sigma) gyroscope\n");
		printmat(Rgyr,3,3);
		printf("Covariance (sigma) magnetometer\n");
		printmat(Rmag,3,3);
	}
	// Default i save calibrartion data
	else{
		accCal[counterCal*3]=yacc[0];
		accCal[counterCal*3+1]=yacc[1];
		accCal[counterCal*3+2]=yacc[2];
		gyrCal[counterCal*3]=ygyr[0];
		gyrCal[counterCal*3+1]=ygyr[1];
		gyrCal[counterCal*3+2]=ygyr[2];
		magCal[counterCal*3]=ymag[0];
		magCal[counterCal*3+1]=ymag[1];
		magCal[counterCal*3+2]=ymag[2];
	}		
}

// S(q) matrix
void Sq(double *Gm, double *q, double T){
	Gm[0]=-q[1]*0.5*T;
	Gm[1]=q[0]*0.5*T;
	Gm[2]=q[3]*0.5*T;
	Gm[3]=-q[2]*0.5*T;
	Gm[4]=-q[2]*0.5*T;
	Gm[5]=-q[3]*0.5*T;
	Gm[6]=q[0]*0.5*T;
	Gm[7]=q[1]*0.5*T;
	Gm[8]=-q[3]*0.5*T;
	Gm[9]=q[2]*0.5*T;
	Gm[10]=-q[1]*0.5*T;
	Gm[11]=q[0]*0.5*T;	
}

// S(omega) matrix
void Somega(double *Sm, double *omega){
	Sm[0]=0;
	Sm[1]=omega[0];
	Sm[2]=omega[1];
	Sm[3]=omega[2];
	Sm[4]=-omega[0];
	Sm[5]=0;
	Sm[6]=-omega[2];
	Sm[7]=omega[1];
	Sm[8]=-omega[1];
	Sm[9]=omega[2];
	Sm[10]=0;
	Sm[11]=-omega[0];
	Sm[12]=-omega[2];
	Sm[13]=-omega[1];
	Sm[14]=omega[0];
	Sm[15]=0;
}

// Quaternions matrix
void Qq(double *Q, double *q){
	// input q0->q3
	//float q0, q1, q2, q3;
	Q[0] = 2*(pow(q[0],2)+pow(q[1],2))-1;
	Q[3] = 2*(q[1]*q[2]-q[0]*q[3]);
	Q[6] = 2*(q[1]*q[3]+q[0]*q[2]);
	Q[1] = 2*(q[1]*q[2]+q[0]*q[3]);
	Q[4] = 2*(pow(q[0],2)+pow(q[2],2))-1;
	Q[7] = 2*(q[2]*q[3]-q[0]*q[1]);
	Q[2] = 2*(q[1]*q[3]-q[0]*q[2]);
	Q[5] = 2*(q[2]*q[3]+q[0]*q[1]);
	Q[8] = 2*(pow(q[0],2)+pow(q[3],2))-1;
}

// Quaternions Jacobian
void dQqdq(double *h1, double *h2, double *h3, double *h4, double *hd, double *q, double *biasvec){
	// Q=dQqdq(q) Jacobian
	// input q0->q3
	// [h1 h2 h3 h4]=dQqdq(x);
	h1[0] = 4*q[0];
	h1[1] = 2*q[3];
	h1[2] = -2*q[2];
	h1[3] = -2*q[3];
	h1[4] = 4*q[0];
	h1[5] = 2*q[1];
	h1[6] = 2*q[2];
	h1[7] = -2*q[1];
	h1[8] = 4*q[0];
	
	//printmat(h1, 3, 3);
	
	h2[0] = 4*q[1];
	h2[1] = 2*q[2];
	h2[2] = 2*q[3];
	h2[3] = 2*q[2];
	h2[4] = 0;
	h2[5] = 2*q[0]; 
	h2[6] = 2*q[3];
	h2[7] = -2*q[0];
	h2[8] = 0;
	
	//printmat(h2, 3, 3);
	
	h3[0] = 0;
	h3[1] = 2*q[1];
	h3[2] = -2*q[0];
	h3[3] = 2*q[1];
	h3[4] = 4*q[2];
	h3[5] = 2*q[3];
	h3[6] = 2*q[0];
	h3[7] = 2*q[3];
	h3[8] = 0;
	
	//printmat(h3, 3, 3);
	
	h4[0] = 0;
	h4[1] = 2*q[0];
	h4[2] = 2*q[1];
	h4[3] = -2*q[0];
	h4[4] = 0;
	h4[5] = 2*q[2];
	h4[6] = 2*q[1];
	h4[7] = 2*q[2];
	h4[8] = 4*q[3];
	
	//printmat(h4, 3, 3);
	
	// hd=[h1'*biasvec h2'*biasvec h3'*biasvec h4'*biasvec];
	double hd_temp[3];
	int n=3, m=3, ione=1;
	double fone=1, fzero=0;
	F77_CALL(dgemv)("t",&m,&n,&fone,h1,&m,biasvec,&ione,&fzero,hd_temp,&ione);
	memcpy(hd, hd_temp, sizeof(hd_temp));
	F77_CALL(dgemv)("t",&m,&n,&fone,h2,&m,biasvec,&ione,&fzero,hd_temp,&ione);
	memcpy(hd+3, hd_temp, sizeof(hd_temp));
	F77_CALL(dgemv)("t",&m,&n,&fone,h3,&m,biasvec,&ione,&fzero,hd_temp,&ione);
	memcpy(hd+6, hd_temp, sizeof(hd_temp));
	F77_CALL(dgemv)("t",&m,&n,&fone,h4,&m,biasvec,&ione,&fzero,hd_temp,&ione);
	memcpy(hd+9, hd_temp, sizeof(hd_temp));
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

// Matrix inverse 3x3
void mInverse(double *m, double *mInv){
	mInv[0] = (m[4] * m[8] - m[5] * m[7]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[3] = -(m[3] * m[8] - m[5] * m[6]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[6] = (m[3] * m[7] - m[4] * m[6]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[1] = -(m[1] * m[8] - m[2] * m[7]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[4] = (m[0] * m[8] - m[2] * m[6]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[7] = -(m[0] * m[7] - m[1] * m[6]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[2] = (m[1] * m[5] - m[2] * m[4]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[5] = -(m[0] * m[5] - m[2] * m[3]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
	mInv[8] = (m[0] * m[4] - m[1] * m[3]) / (m[0] * m[4] * m[8] - m[0] * m[5] * m[7] - m[1] * m[3] * m[8] + m[1] * m[6] * m[5] + m[2] * m[3] * m[7] - m[2] * m[6] * m[4]);
}

// Matrix print function
void printmat(double *A, int m, int n){
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%6.4f\t", *(dptr+m*i+j));
        }
        printf("\n");
    }
    //printf("\n");
    return;
}

// Load settings file
void loadSettings(){
	// Create file pointer
	FILE *fp;
	float value;
	// Open file and prepare for read
	fp=fopen("settings.txt", "r");
	// Check to see that file has opened
	if(fp==NULL){
		printf("File could not be opened for read\n");
	}
	else{
		fscanf(fp, "%f\n", &value);
		printf("Value read: %f\n", value);
	}
	
	// Close file
	fclose(fp);
}

// Save settings file
void saveSettings(){
	// Create file pointer
	FILE *fp;
	
	// Open file and prepare for write
	fp=fopen("settings.txt", "w");
	// Check to see that file has opened
	if(fp==NULL){
		printf("File could not be opened for write\n");
	}
	else{
		fprintf(fp, "THIS IS A TEST WRITE %f\n", 99220.98);
	}
	
	// Close file
	fclose(fp);
}

// Used to print the bits in a data type
void printBits(size_t const size, void const * const ptr)
{
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
