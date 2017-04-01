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
#define CALIBRATION 100


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
	
	float eulers[3]; // roll, pitch, yaw;
	float roll, pitch, yaw;
	uint32_t desiredPeriod = 20;
	uint32_t start=millis();
	uint32_t start2=start;
	uint32_t start3;
	
	//float mean[3], variance[3], std_deviation[3], accRawCal[3][100], sum[3]={0.0,0.0,0.0}, sum1[3]={0.0,0.0,0.0};
	
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
		// Loop for ever
		while(1){
			// Timing
			start=millis();
			start3=start;
			printf("Ts: %i\n", start3-millis());
			
			// Read sensor data to local variable
			pthread_mutex_lock(&mutexI2CBusy);
				readAccelerometer(accRaw, fdAcc);
				readMagnetometer(magRaw, fdMag);
				readGyroscope(gyrRaw, fdGyr);
			pthread_mutex_unlock(&mutexI2CBusy);
		
			// Run Sebastian Madgwick AHRS algorithm
			MadgwickAHRSupdate(gyrRaw[0], gyrRaw[1], gyrRaw[2], accRaw[0], accRaw[1], accRaw[2], magRaw[0], magRaw[1], magRaw[2]);
			//MadgwickAHRSupdateIMU(gyrRaw[0], gyrRaw[1], gyrRaw[2], accRaw[0], accRaw[1], accRaw[2]);
			
			// Quaternion to euler angles [rad]
			yaw=atan2f(2*q1*q2-2*q0*q3,2*pow(q0,2)+2*pow(q1,2)-1)*(180.0/PI);
			pitch=-asinf(2*q1*q3+2*q0*q2)*(180.0/PI);
			roll=atan2f(2*q2*q3-2*q0*q1,2*pow(q0,2)+2*pow(q3,2)-1)*(180.0/PI);
			
			if(millis()-start2>=10000 && calibrationFlag==0){
				printf("Finished sensor Calibration...\n");
				calibrationFlag=1;
				beta = 0.033;
				printf("Beta: %.2f\n", beta);
			}
			
			if(calibrationFlag==1)
				//printf("roll: %.1f pitch: %.1f yaw: %.1f\n", roll, pitch, yaw);
			
			//printf("Acc: %.3f, %.3f, %.3f\n", accRaw[0], accRaw[1], accRaw[2]); 
			//printf("Mag: %.3f, %.3f, %.3f\n", magRaw[0], magRaw[1], magRaw[2]); 
			
			/*
			// Share data for WIFI communication
			pthread_mutex_lock(&mutexAngleData);
			//memcpy(angles, eulers, sizeof(eulers));
			sensorRawData[0]=gyrRaw[0];
			sensorRawData[1]=gyrRaw[1];
			sensorRawData[2]=gyrRaw[2];
			sensorRawData[3]=accRaw[0];
			sensorRawData[4]=accRaw[1];
			sensorRawData[5]=accRaw[2];
			sensorRawData[6]=magRaw[0];
			sensorRawData[7]=magRaw[1];
			sensorRawData[8]=magRaw[2];
			sensorRawData[9]=eulers[0]*(180.0/PI);
			sensorRawData[10]=eulers[1]*(180.0/PI);
			sensorRawData[11]=eulers[2]*(180.0/PI);
			pthread_mutex_unlock(&mutexAngleData);
			*/
			
			// Sleep for desired sampling frequency
			start2=millis()-start;
			if(start2<desiredPeriod)
				usleep(1000*(desiredPeriod-start2));
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



