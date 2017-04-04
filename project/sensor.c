// SENSOR FUSION CODE

#include "sensor.h"
#include "startup.h"
#include "L3G.h"
#include "LSM303.h"
#include "bmp180.h"
#include "PWM.h"
#include "MadgwickAHRS.h"


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
#define CALIBRATION 10000


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
float kalmanFilter(float, float, int);


// Static variables for threads
//static float position[3]={1.0f,1.0f,1.0f};
//static float angles[3]={0,0,0};
//static float tuning[3];
static float sensorRawDataPosition[3]; // Global variable in sensor.c to communicate between imu read and angle fusion threads
static float sensorRawData[12]={0,0,0,0,0,0,0,0,0,0,0,0}; // Global variable in sensor.c to communicate between imu read and position fusion threads

//static pthread_mutex_t mutexPositionData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexAngleData = PTHREAD_MUTEX_INITIALIZER;
//static pthread_mutex_t mutexTuningData = PTHREAD_MUTEX_INITIALIZER;
//static pthread_mutex_t mutexAngleSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexPositionSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexI2CBusy = PTHREAD_MUTEX_INITIALIZER;



float A=1;
float Hk=1;
float xkhat[9]={0,0,0,0,0,0,0,0,0};
float Pk[9]={0,0,0,0,0,0,0,0,0};
float Q[9]={.8,.8,.8,.8,.8,.8,.8,.8,.8}; // motion noise
float Rk[9]={0.0691,0.0778,0.1417,0,0,0,0.1035,0.1207,0.1986}; // [acc, gyr, mag] measurement noise
float Kk[9], Sk[9], vk[9];




/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *arg1, void *arg2){
	// Create pipe array
	pipeArray pipeArray1 = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create thread
	pthread_t threadPipeSensToCtrlAndComm, threadSenFus, threadPWMCtrl;
	int res1, res5, res6;
	
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
	structPipe *ptrPipe1 = pipeArray1->pipe1;
	structPipe *ptrPipe2 = pipeArray1->pipe2;
	//float sensorDataBuffer[3]={0,0,0};
	float sensorDataBuffer[12]={0,0,0,0,0,0,0,0,0,0,0,0};
	
	// Timers for sampling frequency
	uint32_t desiredPeriod = 1000;
	uint32_t start=millis();
	
	// Loop forever sending data to controller and communication processes
	while(1){
		start=millis();
		
		// Get angle and position data from global variables in sensor.c
		pthread_mutex_lock(&mutexAngleData);
		memcpy(sensorDataBuffer, sensorRawData, sizeof(sensorRawData));
		//memcpy(sensorDataBuffer, angles, sizeof(angles));
		pthread_mutex_unlock(&mutexAngleData);
		/*pthread_mutex_lock(&mutexPositionData);
		memcpy(sensorDataBuffer+sizeof(angles), position, sizeof(position));	
		pthread_mutex_unlock(&mutexPositionData);
		
		
		pthread_mutex_lock(&mutexAngleSensorData);
		memcpy(sensorDataBuffer, sensorRawDataAngles, sizeof(sensorRawDataAngles));
		pthread_mutex_unlock(&mutexAngleSensorData);
		pthread_mutex_lock(&mutexPositionSensorData);
		memcpy(sensorDataBuffer+sizeof(sensorRawDataAngles), sensorRawDataPosition+3, sizeof(position)-3);	
		pthread_mutex_unlock(&mutexPositionSensorData);
		*/
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


/*
// Thread - Pipe Communication to Sensor process read
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
	float posRaw[3];
	uint8_t data8[30];
	uint16_t data16[2];
	uint32_t data32[4];
	int n;
	
	int fdBeacon;
	
	// Loop for ever trying to connect to Beacon sensor via USB
	while(1){
		// Open serial communication
		if ((fdBeacon=serialOpen("/dev/ttyACM1", 115200)) < 0){
			//fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
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
						posRaw[0] = (float)((data32[1] = data8[12] << 24| data8[11] << 16| data8[10] << 8| data8[9]));
						posRaw[1] = (float)((data32[2] = data8[16] << 24| data8[15] << 16| data8[14] << 8| data8[13]));	
						posRaw[2] = (float)((data32[3] = data8[20] << 24| data8[19] << 16| data8[18] << 8| data8[17]));
						
						// Copy raw position to global variable for use in sensor fusion thread
						pthread_mutex_lock(&mutexPositionSensorData);
						memcpy(sensorRawDataPosition, posRaw, sizeof(posRaw));
						pthread_mutex_unlock(&mutexPositionSensorData);
							
						printf("X=%.3f, Y=%.3f, Z=%.3f\n\n", posRaw[0]/1000, posRaw[1]/1000, posRaw[2]/1000);
						}
					}
					else{
						sleep(10);
					}
				}
			}
		}
		sleep(5);
	}
	return NULL;
}



// Thread - Sensor fusion
static void *threadSensorFusion (void *arg){
	beta = 10;
	printf("Beta: %.2f\n", beta);
	sleep(2);
	// Define local variables
	float accRaw[3];
	float gyrRaw[3];
	float magRaw[3];
	//float bmpRaw[3];
	
	float roll, pitch, yaw;
	uint32_t desiredPeriod = 500;
	uint32_t start=millis();
	uint32_t start2=start;
	
	float mean[3], variance[3], std_deviation[3], accRawCal[3][CALIBRATION], sum[3]={0.0,0.0,0.0}, sum1[3]={0.0,0.0,0.0};
	int counterCal=0;
	
	// Setup I2C communication
	pthread_mutex_lock(&mutexI2CBusy);
		int fdAcc=wiringPiI2CSetup(ACC_ADDRESS);
		int fdMag=wiringPiI2CSetup(MAG_ADDRESS);
		int fdGyr=wiringPiI2CSetup(GYR_ADDRESS);
		//int fdBmp=wiringPiI2CSetup(BMP180_ADDRESS);
	pthread_mutex_unlock(&mutexI2CBusy);
	
	// Check that the I2C setup was successful
	//if(fdAcc==-1 || fdMag==-1 || fdGyr==-1 || fdBmp==-1)
	if(fdAcc==-1 || fdMag==-1 || fdGyr==-1)
	{
	 printf("Error setup the I2C devices\n");
	}
	else
	{
		printf("Enabling sensors...\n");
		// Enable acc, gyr, mag  and bmp sensors
		pthread_mutex_lock(&mutexI2CBusy);
			enableAccelerometer(fdAcc);
			enableMagnetometer(fdMag);
			enableGyroscope(fdGyr);
			//enableBMP(fdBmp);
		pthread_mutex_unlock(&mutexI2CBusy);

		printf("Start sensor Calibration...\n");
		int calibrationFlag=0;
		
		sleep(10);
		int k=0;
		// Loop for ever
		while(1){
			// Timing
			//printf("Ts: %i\n", millis()-start2);
			start=millis();
			start2=start;
			
			// Read sensor data to local variable
			pthread_mutex_lock(&mutexI2CBusy);
				readAccelerometer(accRaw, fdAcc);
				readMagnetometer(magRaw, fdMag);
				readGyroscope(gyrRaw, fdGyr);
			pthread_mutex_unlock(&mutexI2CBusy);
			
			
			if(k<100)
				k++;
			else
				beta=1;
				
			
			
		
		/*
			// Kalman Filter the raw acc data
			for (int i=0;i<3;i++){
				 accRaw[i]=kalmanFilter(accRaw[i], (float)desiredPeriod, i);
			}
			// Kalman Filter the raw gyr data
			for (int i=3;i<6;i++){
				gyrRaw[i]=kalmanFilter(gyrRaw[i], (float)desiredPeriod, i);
			}
			// Kalman Filter the raw mag data
			for (int i=6;i<9;i++){
				magRaw[i]=kalmanFilter(magRaw[i], (float)desiredPeriod, i);
			}
		*/
			//printf("Raw Acc: %6.3f %6.3f %6.3f Filter acc: %6.3f %6.3f %6.3f P acc: %6.3f %6.3f %6.3f\n", accRaw[0], accRaw[1], accRaw[2], xkhat[0], xkhat[1], xkhat[2], Pk[0], Pk[1], Pk[2]);
		
			// Run Sebastian Madgwick AHRS algorithm
			//MadgwickAHRSupdate(gyrRaw[0]*(PI/180), gyrRaw[1]*(PI/180), gyrRaw[2]*(PI/180), accRaw[0], accRaw[1], accRaw[2], magRaw[0], magRaw[1], magRaw[2]);
			//MadgwickAHRSupdate(xkhat[3], xkhat[4], xkhat[5], xkhat[0], xkhat[1], xkhat[2], xkhat[6], xkhat[7], xkhat[8]);
			//MadgwickAHRSupdateIMU(gyrRaw[0], gyrRaw[1], gyrRaw[2], accRaw[0], accRaw[1], accRaw[2]);
			

			
			
	
	
			// Calibration routine to get mean, variance and std_deviation
			/*if(counterCal==CALIBRATION-1){
				// Mean
				for (int i=0;i<CALIBRATION;i++){
					sum[0]+=accRawCal[0][i];
					sum[1]+=accRawCal[1][i];
					sum[2]+=accRawCal[2][i];
				}
				mean[0]=sum[0]/CALIBRATION;
				mean[1]=sum[1]/CALIBRATION;
				mean[2]=sum[2]/CALIBRATION;
				
				// Variance and std_deviation
				for (int i=0;i<CALIBRATION;i++){
					sum1[0]+=pow((accRawCal[0][i] - mean[0]), 2);
					sum1[1]+=pow((accRawCal[1][i] - mean[1]), 2);
					sum1[2]+=pow((accRawCal[2][i] - mean[2]), 2);
				}
				std_deviation[0]=sum1[0]/CALIBRATION;
				std_deviation[1]=sum1[1]/CALIBRATION;
				std_deviation[2]=sum1[2]/CALIBRATION;
				variance[0]=sqrt(std_deviation[0]);
				variance[1]=sqrt(std_deviation[1]);
				variance[2]=sqrt(std_deviation[2]);
				printf("Mean: %.4f %.4f %.4f Variance: %.4f %.4f %.4f Std Deviation: %.4f %.4f %.4f\n", mean[0], mean[1], mean[2], variance[0], variance[1], variance[2], std_deviation[0], std_deviation[1], std_deviation[2]);
				counterCal=CALIBRATION;
			}
			else{
				accRawCal[0][counterCal]=accRaw[0];
				accRawCal[1][counterCal]=accRaw[1];
				accRawCal[2][counterCal]=accRaw[2];
				counterCal++;
			}
			
			*/
			
			
			
			
			//float accFiltered=kalmanFilter(accRaw[0], (float)desiredPeriod);
			
			//printf("Acc1: %6.3f %6.3f %6.3f Acc2: %6.3f %6.3f %6.3f\n", accRaw[0], accRaw[1], accRaw[2], accRaw[3], accRaw[4], accRaw[5]);
			printf("Acc: %6.3f %6.3f %6.3f Mag: %6.3f %6.3f %6.3f Gyr: %6.3f %6.3f %6.3f\n", accRaw[0], accRaw[1], accRaw[2], magRaw[0], magRaw[1], magRaw[2], gyrRaw[0], gyrRaw[1], gyrRaw[2]);
			//printf("Acc: %f %f\n", accRaw[0], accFiltered);
		
			// Quaternion to euler angles [rad]
			/*
			yaw=atan2f(2*q1*q2-2*q0*q3,2*pow(q0,2)+2*pow(q1,2)-1)*(180.0/PI);
			pitch=-asinf(2*q1*q3+2*q0*q2)*(180.0/PI);
			roll=atan2f(2*q2*q3-2*q0*q1,2*pow(q0,2)+2*pow(q3,2)-1)*(180.0/PI);
			*/
			
			/*
			if(millis()-start2>=10000 && calibrationFlag==0){
				printf("Finished sensor Calibration...\n");
				calibrationFlag=1;
				beta = 0.033;
				printf("Beta: %.2f\n", beta);
			}
			*/
			
			//if(calibrationFlag==1)
				
				//printf("roll: %.2f pitch: %.2f yaw: %.2f\n", roll, pitch, yaw);
				fflush(stdout);
				printf("\033c");
				
				
			// Sleep for desired sampling frequency
			if((millis()-start)<desiredPeriod)
				usleep(1000*(desiredPeriod-(millis()-start)));
		}
	}
	return NULL;
}





// Thread - PWM Control
static void *threadPWMControl (void *arg){
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

// Kalman filter (random walk)
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






/*
// Low pass filter
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
