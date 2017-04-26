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

#define PI 3.141592653589793
#define CALIBRATION 2000

/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadPipeSensorToControllerAndComm (void*);
//static void *threadPipeSensorToComm (void*);
static void *threadReadBeacon (void*);
//static void *threadSensorFusionAngles (void*);
//static void *threadPipeCommToSensor (void*);
static void *threadSensorFusion (void*);
//static void *threadReadBeacon (void*);
//static void *threadSensorFusion (void*);
static void *threadPWMControl (void*);
//float kalmanFilter(float, float, int);
//void mTranspose(float**, float**, int, int);
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
void loadSettings(double*, double*, double*, double*, double*, double*);
void saveSettings(double*, double*, double*, double*, double*, double*);

// Static variables for threads
//static float position[3]={1.0f,1.0f,1.0f};
//static float angles[3]={0,0,0};
//static float tuning[3];
static double sensorRawDataPosition[3]; // Global variable in sensor.c to communicate between IMU read and angle fusion threads
//static double sensorRawData[16]={0,0,0,0,0,0,0,0,0,0,0,0}; // Global variable in sensor.c to communicate between imu read and position fusion threads
static double sensorRawDataAngles[16]={0,0,0,0,0,0,0,0,0,0,0,0}; 

//static pthread_mutex_t mutexPositionData = PTHREAD_MUTEX_INITIALIZER;
//static pthread_mutex_t mutexAngleData = PTHREAD_MUTEX_INITIALIZER;
//static pthread_mutex_t mutexTuningData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexAngleSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexPositionSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexI2CBusy = PTHREAD_MUTEX_INITIALIZER;





/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *arg1, void *arg2){
	// Create pipe array
	pipeArray pipeArray1 = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create thread
	pthread_t threadPipeSensToCtrlAndComm, threadSenFus, threadPWMCtrl, threadReadPos;
	int res1, res3, res5, res6;
	
	res1=pthread_create(&threadPipeSensToCtrlAndComm, NULL, &threadPipeSensorToControllerAndComm, &pipeArray1);
	//res2=pthread_create(&threadAngles, NULL, &threadSensorFusionAngles, NULL);
	//res3=pthread_create(&threadReadPos, NULL, &threadReadBeacon, NULL);
	//res4=pthread_create(&threadPipeCommToSens, NULL, &threadPipeCommToSensor, arg2);
	res5=pthread_create(&threadSenFus, NULL, &threadSensorFusion, arg1);
	res6=pthread_create(&threadPWMCtrl, NULL, &threadPWMControl, arg1);
	
	// If threads created successful, start them
	if (!res1) pthread_join( threadPipeSensToCtrlAndComm, NULL);
	//if (!res2) pthread_join( threadAngles, NULL);
	//if (!res3) pthread_join( threadReadPos, NULL);
	//if (!res4) pthread_join( threadPipeCommToSens, NULL);
	if (!res5) pthread_join( threadSenFus, NULL);
	if (!res6) pthread_join( threadPWMCtrl, NULL);
}





/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Sensor fusion and pipe writing to Controller and Communication processes
/*static void *threadSensorFusion (void *arg){
		// Get pipe array and define local variables
	pipeArray *pipeArray1 = arg;
	structPipe *ptrPipe1 = pipeArray1->pipe1;
	structPipe *ptrPipe2 = pipeArray1->pipe2;
	float sensorDataBuffer[12]={0,0,0,0,0,0,0,0,0,0,0,0};
	
	// Loop forever sending data to controller and communication processes
	while(1){
	*/
		// Get angle and position data from global variables in sensor.c
		/*
		pthread_mutex_lock(&mutexAngleData);
		memcpy(sensorDataBuffer, angles, sizeof(angles));
		pthread_mutex_unlock(&mutexAngleData);
		pthread_mutex_lock(&mutexPositionData);
		memcpy(sensorDataBuffer+sizeof(angles), position, sizeof(position));	
		pthread_mutex_unlock(&mutexPositionData);
		*/
		/*
		pthread_mutex_lock(&mutexAngleSensorData);
		memcpy(sensorDataBuffer, sensorRawDataAngles, sizeof(sensorRawDataAngles));
		pthread_mutex_unlock(&mutexAngleSensorData);
		pthread_mutex_lock(&mutexPositionSensorData);
		memcpy(sensorDataBuffer+sizeof(sensorRawDataAngles), sensorRawDataPosition+3, sizeof(position)-3);	
		pthread_mutex_unlock(&mutexPositionSensorData);
		*/
		/*
		// Write to Controller process
		if (write(ptrPipe1->child[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor to Controller\n");
		//else printf("Sensor ID: %d, Sent: %f to Controller\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Write to Communication process
		if (write(ptrPipe2->parent[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error to communicaiont in sensors\n");
		//else printf("Sensor ID: %d, Sent: %f to Communication\n", (int)getpid(), sensorDataBuffer[0]);
		usleep(100000);
		*/
		/*sleep(5);
	}
	
	return NULL;
}
*/


// Thread - Pipe Sensor to Controller and Communication process write
static void *threadPipeSensorToControllerAndComm (void *arg){
	// Get pipe array and define local variables
	pipeArray *pipeArray1 = arg;
	//structPipe *ptrPipe1 = pipeArray1->pipe1;
	structPipe *ptrPipe2 = pipeArray1->pipe2;
	//float sensorDataBuffer[3]={0,0,0};
	double sensorDataBuffer[19]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	// Timers for sampling frequency
	uint32_t desiredPeriod = 100;
	uint32_t start=millis();
	
	// Loop forever sending data to controller and communication processes
	while(1){
		start=millis();
		
		// Get angle and position data from global variables in sensor.c
		//pthread_mutex_lock(&mutexAngleData);
		//memcpy(sensorDataBuffer, sensorRawData, sizeof(sensorRawData));
		//memcpy(sensorDataBuffer, angles, sizeof(angles));
		//pthread_mutex_unlock(&mutexAngleData);
		/*pthread_mutex_lock(&mutexPositionData);
		memcpy(sensorDataBuffer+sizeof(angles), position, sizeof(position));	
		pthread_mutex_unlock(&mutexPositionData);
		*/
		
		printf("pipe mutexAngleSensorData start\n");
		pthread_mutex_lock(&mutexAngleSensorData);
			memcpy(sensorDataBuffer, sensorRawDataAngles, sizeof(sensorRawDataAngles));
		pthread_mutex_unlock(&mutexAngleSensorData);
		printf("pipe mutexAngleSensorData stop\n");
		//usleep(20000);
		//pthread_mutex_lock(&mutexPositionSensorData);
			//memcpy(sensorDataBuffer+sizeof(sensorRawDataPosition), sensorRawDataPosition, sizeof(sensorRawDataPosition));	
		//pthread_mutex_unlock(&mutexPositionSensorData);
		
		// Write to Controller process
		//if (write(ptrPipe1->child[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor to Controller\n");
		//else printf("Sensor ID: %d, Sent: %f to Controller\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Write to Communication process
		if (write(ptrPipe2->parent[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor ot Communicaiont\n");
		//else printf("Sensor ID: %d, Sent: %f to Communication\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Sleep for desired sampling frequency
		if((millis()-start)<desiredPeriod)
			usleep(1000*(desiredPeriod-(millis()-start)));
	}
	
	return NULL;
}


// Thread - Pipe Communication to Sensor process read
/*
static void *threadPipeCommToSensor (void *arg)
{
	// Get pipe and define local variables
	structPipe *ptrPipe = arg;
	float tuningDataBuffer[3];
	
	// Loop forever reading/waiting for data
	while(1){
		// Read data from controller process
		if(read(ptrPipe->child[0], tuningDataBuffer, sizeof(tuningDataBuffer)) == -1) printf("read error in sensor from communication\n");
		//else printf("Sensor ID: %d, Recieved Communication data: %f\n", (int)getpid(), tuningDataBuffer[0]);
		
		// Put new data in to global variable in communication.c
		pthread_mutex_lock(&mutexTuningData);
		memcpy(tuning, tuningDataBuffer, sizeof(tuningDataBuffer));
		pthread_mutex_unlock(&mutexTuningData);
		
		sleep(10);
	}
	return NULL;
}
*/


// Thread - Read position values
void *threadReadBeacon (void *arg){
	// Define local variables
	double posRaw[3];
	uint8_t data8[30];
	uint16_t data16[2];
	uint32_t data32[4];
	int n;
	
	int fdBeacon;
	
	// Loop for ever trying to connect to Beacon sensor via USB
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
				// Read serial data when available
				if (serialDataAvail(fdBeacon)){
					if ((int)(data8[0]=serialGetchar(fdBeacon)) == -1){
						fprintf(stderr, "serialGetchar, block for 10s: %s\n", strerror (errno));
					}
					else  if (data8[0] == 0xff){
						data8[1] = serialGetchar(fdBeacon);	// Data type: 0x47
						data8[2] = serialGetchar(fdBeacon);
						data8[3] = serialGetchar(fdBeacon);
						data8[4] = serialGetchar(fdBeacon);
						n = (int)(data8[4]);
						//printf("%d", n);
						for (int i=0;i<n;i++){
							data8[5+i] = serialGetchar(fdBeacon);
						}
						data8[5+n] = serialGetchar(fdBeacon);
						data8[6+n] = serialGetchar(fdBeacon);		
						
						if ((data16[0]=data8[3] << 8| data8[2]) != 0x0011){
							//printf("Unrecognized code of data in packet -> %04x\n", data16[0]);
							usleep(20000);
						}
						else{
						
						// Raw position uint data to float
						posRaw[0] = (double)((data32[1] = data8[12] << 24| data8[11] << 16| data8[10] << 8| data8[9]));
						posRaw[1] = (double)((data32[2] = data8[16] << 24| data8[15] << 16| data8[14] << 8| data8[13]));	
						posRaw[2] = (double)((data32[3] = data8[20] << 24| data8[19] << 16| data8[18] << 8| data8[17]));
						
						// Copy raw position to global variable for use in sensor fusion thread
						printf("beacon mutexPositionSensorData start\n");
						pthread_mutex_lock(&mutexPositionSensorData);
						memcpy(sensorRawDataPosition, posRaw, sizeof(posRaw));
						pthread_mutex_unlock(&mutexPositionSensorData);
						printf("beacon mutexPositionSensorData stopp\n");	
							
						printf("X=%.3f, Y=%.3f, Z=%.3f\n\n", posRaw[0]/1000, posRaw[1]/1000, posRaw[2]/1000);
						}
					}
					else{
						//printf("No beacon data. Sleep for 1 second\n");
						usleep(50000);

					}
				}
				usleep(50000);
			}
		}
		sleep(5);
	}
	return NULL;
}


// Thread - Sensor fusion
static void *threadSensorFusion (void *arg){
	// Define local variables
	double accRaw[3]={0,0,0}, gyrRaw[3]={0,0,0}, magRaw[3]={0,0,0}, magRawRot[3], tempRaw=0, acc0[3]={0,0,0}, gyr0[3]={0,0,0}, mag0[3]={0,0,0}, accCal[3*CALIBRATION], gyrCal[3*CALIBRATION], magCal[3*CALIBRATION], euler[3]={0,0,0};
	double Racc[9]={0,0,0,0,0,0,0,0,0}, Rgyr[9]={0,0,0,0,0,0,0,0,0}, Rmag[9]={0,0,0,0,0,0,0,0,0}, P[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}, L=4, q[4]={1,0,0,0};
	uint32_t desiredPeriod = 20, start;
	int calibrationCounter=0;
	
	// Enable acc, gyr, mag  and bmp sensors
	printf("Enabling sensors...\n");
	printf("sf1 mutexI2CBusy start\n");
	pthread_mutex_lock(&mutexI2CBusy);
		enableMPU9250();
		enableAK8963();
	pthread_mutex_unlock(&mutexI2CBusy);
	printf("sf1 mutexI2CBusy stopp\n");
	
	// Try and load covariance and bias from file "Settings.txt" and check if they are valid, else calibrate.
	loadSettings(Racc, Rgyr, Rmag, acc0, gyr0, mag0);
	if(acc0[0]==0 && acc0[1]==0 && acc0[2]==0){
		printf("Loaded settings not valid\n");
		sleep(2);
	}
	else{
		printf("Loaded settings successfully");
		calibrationCounter=CALIBRATION+1; // Avoids the calibration routine inside while(1)
		sleep(2);
	}
	
	
	// Loop for ever
	while(1){
		// Timing
		//printf("Ts: %i\n", millis()-start2);
		start=millis();
		//start2=start;
		
		// Read sensor data to local variable
		printf("sf2 mutexI2CBusy start\n");
		pthread_mutex_lock(&mutexI2CBusy);	
			readAllSensorData(accRaw, gyrRaw, magRaw, &tempRaw);	
		pthread_mutex_unlock(&mutexI2CBusy);
		printf("sf2 mutexI2CBusy stopp\n");
		
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
		magRawRot[0]=magRaw[1]*(-1);
		magRawRot[1]=-magRaw[0];
		magRawRot[2]=magRaw[3]*(-1);
		
		// Print sensor data
		//printf("Acc [m/s²] X: %7.4f Y: %7.4f %7.4f \t Gyr [rad/s] X: %7.4f Y: %7.4f Z: %7.4f \t Mag [microT] X: %7.4f Y: %7.4f Z: %7.4f \t Temp [C]: %7.4f\n", accRaw[0], accRaw[1], accRaw[2], gyrRaw[0], gyrRaw[1], gyrRaw[2], magRaw[0], magRaw[1], magRaw[2], tempRaw);
	
		
		// Calibration routine.
		// If covariance and bias has been successfully loaded, calibrationCounter is set equal to CALIBRATION+1 such that calibration is skipped.
		if(calibrationCounter==0){
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
			saveSettings(Racc, Rgyr, Rmag, acc0, gyr0, mag0); // Save covariance and bias to "Settings.txt"
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
			
			// Move over data to global variables for sending to controller process
			printf("sf3 mutexAngleSensorData start\n");
			pthread_mutex_lock(&mutexAngleSensorData);
				sensorRawDataAngles[0]=gyrRaw[0];
				sensorRawDataAngles[1]=gyrRaw[1];
				sensorRawDataAngles[2]=gyrRaw[2];
				sensorRawDataAngles[3]=accRaw[0];
				sensorRawDataAngles[4]=accRaw[1];
				sensorRawDataAngles[5]=accRaw[2];
				sensorRawDataAngles[6]=magRaw[0];
				sensorRawDataAngles[7]=magRaw[1];
				sensorRawDataAngles[8]=magRaw[2];
				sensorRawDataAngles[9]=euler[0];
				sensorRawDataAngles[10]=euler[1];
				sensorRawDataAngles[11]=euler[2];
				sensorRawDataAngles[12]=q[0];
				sensorRawDataAngles[13]=q[1];
				sensorRawDataAngles[14]=q[2];
				sensorRawDataAngles[15]=q[3];
			pthread_mutex_unlock(&mutexAngleSensorData);
			printf("sf3 mutexAngleSensorData stopp\n");
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
		sleep(10000);
	// Initialize I2C connection to the PWM board and define PWM frequency
	printf("pwm1 mutexI2CBusy start\n");
	pthread_mutex_lock(&mutexI2CBusy);
		int fdPWM=wiringPiI2CSetup(PWM_ADDRESS);
	pthread_mutex_unlock(&mutexI2CBusy);
	printf("pwm1 mutexI2CBusy stopp\n");


	if(fdPWM==-1){
	 printf("Error setup the I2C PWM connection\n");
	}
	else{
		// Initialize PWM board
		printf("pwm2 mutexI2CBusy start\n");
		pthread_mutex_lock(&mutexI2CBusy);
			enablePWM(fdPWM,500);
		pthread_mutex_unlock(&mutexI2CBusy);
		printf("pwm2 mutexI2CBusy stopp\n");
		printf("PWM initialization complete\n");
		
		// Run forever and set PWM when controller computes new values
		while(1){
			// Read data from controller process
			if(read(ptrPipe->parent[0], pwmValueBuffer, sizeof(pwmValueBuffer)) == -1) printf("read error in sensor from controller\n");
			printf("Data received: %f\n", pwmValueBuffer[0]);
			
			// Set PWM
			printf("pwm3 mutexI2CBusy start\n");
			pthread_mutex_lock(&mutexI2CBusy);
				setPWM(fdPWM, pwmValueBuffer);
			pthread_mutex_unlock(&mutexI2CBusy);
			printf("pwm3 mutexI2CBusy stopp\n");
		}	
	}
	return NULL;
}



/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

// Kalman filter (random walk)
/*
float kalmanFilter(float yk, float Ts, int index){
	float Ak=1+A*(Ts/1000);
	float Qk=Q[index]*(Ts/1000);
	//printf("yk=%f Ak=%f Qk=%f\n", yk, Ak, Qk);
	// Prediction step
	xkhat[index]=Ak*xkhat[index];
	Pk[index]=Ak*Pk[index]*Ak+Qk;
	//printf("Prediction: xkhat=%f Pk=%f\n", xkhat, Pk);
	
	// Update step
	Sk[index]=Hk*Pk[index]*Hk+Rk[index];
	Kk[index]=Pk[index]*Hk*(1/Sk[index]);
	vk[index]=yk-Hk*xkhat[index];
	//printf("Sk=%f Kk=%f vk=%f\n", Sk, Kk, vk);

	xkhat[index]=xkhat[index]+Kk[index]*vk[index];
	Pk[index]=Pk[index]-Kk[index]*Sk[index]*Kk[index];
	//printf("Update: xkhat=%f Pk=%f\n", xkhat, Pk);
	
	return xkhat[index];
}
*/

// Low pass filter
/*
static void lowPassFilter(float* data, int Ts, int freq, int r){
	c=1.0f/tanf(PI*freq / Ts);
	a1=1.0f/(1.0f+r*c+c*c);
	a2=2.0f*a1;
	a3=a1;
	b1=2.0f*(1.0f-c*c)*a1;
	b2=(1.0f-r*c+c*c)*a1;
}

// High pass filter
static void highPassFilter(float* data, int Ts, int freq, int r){
	c=tanf(PI*freq / Ts);
	a1=1.0f/(1.0f+r*c+c*c);
	a2=-2.0f*a1;
	a3=a1;
	b1=2.0f*(c*c-1.0f)*a1;
	b2=(1.0f-r*c+c*c)*a1;
}
*/

// Accelerometer part: mu_g
void accelerometerUpdate(double *q, double *P, double *yacc, double *g0, double *Ra){
	// local variables
	int ione=1, n=3, k=3, m=3;
	double fone=1, fzero=0, yka[3], yka2[9], Q[9], h1[9], h2[9], h3[9], h4[9], hd[12], Sacc[9], P_temp[16], S_temp[12], K_temp[12], Sacc_inv[9], yacc_diff[3], state_temp[4];
	double fka[3]={0,0,0}, K[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
	// check if acc is valid (isnan and all!=0)
	// outlier detection
	if (sqrt(pow(yacc[0],2) + pow(yacc[1],2) + pow(yacc[2],2)) > 1.0* sqrtf(pow(g0[0],2) + pow(g0[1],2) + pow(g0[2],2))){
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
void loadSettings(double *Racc, double *Rgyr, double *Rmag, double *acc0, double *gyr0, double *mag0){
	// Create file pointer
	FILE *fp;
	
	// Open file and prepare for read
	fp=fopen("settings.txt", "r");
	// Check to see that file has opened
	if(fp==NULL){
		printf("File could not be opened for read\n");
	}
	else{
		// Get covariance matrices
		fscanf(fp, "Racc:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &Racc[0], &Racc[1], &Racc[2], &Racc[3], &Racc[4], &Racc[5], &Racc[6], &Racc[7], &Racc[8]);
		fscanf(fp, "Rgyr:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &Rgyr[0], &Rgyr[1], &Rgyr[2], &Rgyr[3], &Rgyr[4], &Rgyr[5], &Rgyr[6], &Rgyr[7], &Rgyr[8]);
		fscanf(fp, "Rmag:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &Rmag[0], &Rmag[1], &Rmag[2], &Rmag[3], &Rmag[4], &Rmag[5], &Rmag[6], &Rmag[7], &Rmag[8]);
		fscanf(fp, "acc0:%lf,%lf,%lf\n", &acc0[0], &acc0[1], &acc0[2]);
		fscanf(fp, "gyr0:%lf,%lf,%lf\n", &gyr0[0], &gyr0[1], &gyr0[2]);
		fscanf(fp, "mag0:%lf,%lf,%lf\n", &mag0[0], &mag0[1], &mag0[2]);
	}
	// Close file
	fclose(fp);
}

// Save settings file
void saveSettings(double *Racc, double *Rgyr, double *Rmag, double *acc0, double *gyr0, double *mag0){
	// Create file pointer
	FILE *fp;
	
	// Open file and prepare for write
	fp=fopen("settings.txt", "w");
	// Check to see that file has opened
	if(fp==NULL){
		printf("File could not be opened for write\n");
	}
	else{
		fprintf(fp, "Racc:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", Racc[0], Racc[1], Racc[2], Racc[3], Racc[4], Racc[5], Racc[6], Racc[7], Racc[8]);
		fprintf(fp, "Rgyr:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", Rgyr[0], Rgyr[1], Rgyr[2], Rgyr[3], Rgyr[4], Rgyr[5], Rgyr[6], Rgyr[7], Rgyr[8]);
		fprintf(fp, "Rmag:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", Rmag[0], Rmag[1], Rmag[2], Rmag[3], Rmag[4], Rmag[5], Rmag[6], Rmag[7], Rmag[8]);
		fprintf(fp, "acc0:%lf,%lf,%lf\n", acc0[0], acc0[1], acc0[2]);
		fprintf(fp, "gyr0:%lf,%lf,%lf\n", gyr0[0], gyr0[1], gyr0[2]);
		fprintf(fp, "mag0:%lf,%lf,%lf\n", mag0[0], mag0[1], mag0[2]);
	}
	
	// Close file
	fclose(fp);
}
