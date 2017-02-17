// SENSOR FUSION CODE

#include "sensor.h"
#include "startup.h"
#include "L3G.h"
#include "LSM303.h"
#include "bmp180.h"


#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <errno.h>
#include <math.h>





/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static void *threadPipeSensorToControllerAndComm (void*);
//static void *threadPipeSensorToComm (void*);
static void *threadSensorFusionPosition (void*);
static void *threadSensorFusionAngles (void*);
static void *threadPipeCommToSensor (void*);
static void *threadReadIMU (void*);
static void readAccRaw(float*, float*, int);
static void readGyrRaw(float*, float*, int);
static void readMagRaw(float*, float*, int);


// Static variables for threads
static float position[3]={1.0f,1.0f,1.0f};
static float angles[3]={2.0f, 2.0f, 2.0f};
static float tuning[3];
static float sensorRaw[12]={0,0,0,0,0,0,0,0,0,0,0,0};

static pthread_mutex_t mutexPositionData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexAngleData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexTuningData = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads
void startSensors(void *arg1, void *arg2){
	// Create pipe array
	pipeArray pipeArray1 = {.pipe1 = arg1, .pipe2 = arg2 };
	
	// Create thread
	pthread_t threadPipeSensToCtrlAndComm, threadPipeCommToSens, threadAngles, threadPosition, threadReadImu;
	int res1, res2 ,res3, res4, res5;
	
	res1=pthread_create(&threadPipeSensToCtrlAndComm, NULL, &threadPipeSensorToControllerAndComm, &pipeArray1);
	res2=pthread_create(&threadAngles, NULL, &threadSensorFusionAngles, NULL);
	res3=pthread_create(&threadPosition, NULL, &threadSensorFusionPosition, NULL);
	res4=pthread_create(&threadPipeCommToSens, NULL, &threadPipeCommToSensor, arg2);
	res5=pthread_create(&threadReadImu, NULL, &threadReadIMU, NULL);
	
	// If threads created successful, start them
	if (!res1) pthread_join( threadPipeSensToCtrlAndComm, NULL);
	if (!res2) pthread_join( threadAngles, NULL);
	if (!res3) pthread_join( threadPosition, NULL);
	if (!res4) pthread_join( threadPipeCommToSens, NULL);
	if (!res5) pthread_join( threadReadImu, NULL);
}


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

// Thread - Pipe Sensor to Controller and Communication process write
static void *threadPipeSensorToControllerAndComm (void *arg){
	// Get pipe array and define local variables
	pipeArray *pipeArray1 = arg;
	structPipe *ptrPipe1 = pipeArray1->pipe1;
	structPipe *ptrPipe2 = pipeArray1->pipe2;
	float sensorDataBuffer[6];
	
	// Loop forever sending data to controller and communication processes
	while(1){
		// Get angle and position data from global variables in sensor.c
		pthread_mutex_lock(&mutexAngleData);
		memcpy(sensorDataBuffer, angles, sizeof(angles));
		pthread_mutex_unlock(&mutexAngleData);
		pthread_mutex_lock(&mutexPositionData);
		memcpy(sensorDataBuffer+sizeof(angles), position, sizeof(position));	
		pthread_mutex_unlock(&mutexPositionData);
		
		// Write to Controller process
		if (write(ptrPipe1->child[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error in Sensor to Controller\n");
		//else printf("Sensor ID: %d, Sent: %f to Controller\n", (int)getpid(), sensorDataBuffer[0]);
		
		// Write to Communication process
		if (write(ptrPipe2->parent[1], sensorDataBuffer, sizeof(sensorDataBuffer)) != sizeof(sensorDataBuffer)) printf("pipe write error to communicaiont in sensors\n");
		//else printf("Sensor ID: %d, Sent: %f to Communication\n", (int)getpid(), sensorDataBuffer[0]);
		sleep(5);
	}
	
	return NULL;
}


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
		
		sleep(1);
	}
	return NULL;
}


// Thread - Sensor Fusion Position
void *threadSensorFusionPosition (void *arg){
	// Loop for ever
	while(1){				 
		// Write data to local variables using mutex
		pthread_mutex_lock(&mutexPositionData);
		usleep(100);
		float newPosition[3]={1,2,3};
		memcpy(position, newPosition, sizeof(newPosition));	
		pthread_mutex_unlock(&mutexPositionData);	
	}
	return NULL;
}


// Thread - Sensor Fusion Angles
void *threadSensorFusionAngles (void *arg){
	// Loop for ever
	while(1){				 
		// Write data to local variables using mutex
		pthread_mutex_lock(&mutexAngleData);
		usleep(100);
		float newAngles[3]={1,2,3};
		memcpy(angles, newAngles, sizeof(newAngles));	
		pthread_mutex_unlock(&mutexAngleData);	
	}
	return NULL;
}

// Thread - Read IMU values
static void *threadReadIMU (void *arg){
	sleep(5);
	// Define local variables

	float accRaw[3];
	float gyrRaw[3];
	float magRaw[3];
	
	// Setup I2C communication
	int fdAcc=wiringPiI2CSetup(ACC_ADDRESS);
	int fdMag=wiringPiI2CSetup(MAG_ADDRESS);
	int fdGyr=wiringPiI2CSetup(GYR_ADDRESS);
	int fdBmp=wiringPiI2CSetup(BMP180_ADDRESS);
	
	if(fdAcc==-1 || fdMag==-1 || fdGyr==-1 /*|| fdBmp==-1*/)
	{
	 printf("Error setup the I2C devices\n");
	}
	else
	{
		 // Enabling Accelerometer
		 wiringPiI2CWriteReg8(fdAcc,LSM303_CTRL_REG1_A,0b01010111); //  z,y,x axis enabled , 100Hz data rate
		 wiringPiI2CWriteReg8(fdAcc,LSM303_CTRL_REG4_A,0b00101000); // +/- 8G full scale: FS = 10 on DLHC, high resolution output mode

		 // Enabling Magnometer
		 wiringPiI2CWriteReg8(fdMag,LSM303_MR_REG_M,0x00); // enable magnometer
		 
		 // Enabling Gyroscope
		 wiringPiI2CWriteReg8(fdGyr,L3G_CTRL_REG1, 0b00001111); // Normal power mode, all axes enabled
         wiringPiI2CWriteReg8(fdGyr,L3G_CTRL_REG4, 0b00110000); // Continuos update, 2000 dps full scale

		// Download BMP180 factory calibration data
		loadCalibration(fdBmp);
		
		// Loop for ever
		while(1){
			
			magRaw[0] = (float)(((int16_t)(mag[1] << 8| mag[0]))/pow(2,8)*10);
			magRaw[1] = (float)(((int16_t)(mag[3] << 8| mag[2]))/pow(2,8)*10);
			magRaw[2] = (float)(((int16_t)(mag[5] << 8| mag[4]))/pow(2,8)*10);
			gyrRaw[0] = (float)(((int16_t)(gyr[1] << 8| gyr[0]))/pow(2,8));
			gyrRaw[1] = (float)(((int16_t)(gyr[3] << 8| gyr[2]))/pow(2,8));
			gyrRaw[2] = (float)(((int16_t)(gyr[5] << 8| gyr[4]))/pow(2,8));

			// Print results
			//printf("Temp: %.2f C Pressure: %.2f Pa Altitude: %.2f h\n",readTemperature(fdBmp), readPressure(fdBmp)/100.0, readAltitude(fdBmp));
			printf("Acc: %.2f %.2f %.2f\tGyr: %.2f %.2f %.2f\tMag: %.2f %.2f %.2f\n", accRaw[0], accRaw[1], accRaw[2], gyrRaw[0], gyrRaw[1], gyrRaw[2], magRaw[0], magRaw[1], magRaw[2]);
			usleep(100000);
		}
	}
	return NULL;
}


/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

static readAccRaw(float *acc, float *accRaw, int fd){
	// Read Accelerometer
	acc[0]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_L_A);
	acc[1]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_H_A);
	acc[2]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_L_A);
	acc[3]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_H_A);
	acc[4]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_L_A);
	acc[5]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_H_A);
	
	// Convert 16 bit 2's complement to floating point value
	accRaw[0] = (float)(((int16_t)(acc[1] << 8| acc[0]) >> 4)/pow(2,8)*10);
	accRaw[1] = (float)(((int16_t)(acc[3] << 8| acc[2]) >> 4)/pow(2,8)*10);
	accRaw[2] = (float)(((int16_t)(acc[5] << 8| acc[4]) >> 4)/pow(2,8)*10);
}

static void readGyrRaw(float *gyr, int fd){
	// Read Gyroscope
	gyr[0]=wiringPiI2CReadReg8(fd,L3G_OUT_X_L);
	gyr[1]=wiringPiI2CReadReg8(fd,L3G_OUT_X_H);
	gyr[2]=wiringPiI2CReadReg8(fd,L3G_OUT_Y_L);
	gyr[3]=wiringPiI2CReadReg8(fd,L3G_OUT_Y_H);
	gyr[4]=wiringPiI2CReadReg8(fd,L3G_OUT_Z_L);
	gyr[5]=wiringPiI2CReadReg8(fd,L3G_OUT_Z_H);
}

static void readMagRaw(float *mag, int fd){
	// Read Magnometer
	mag[0]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_L_M);
	mag[1]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_H_M);
	mag[2]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_L_M);
	mag[3]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_H_M);
	mag[4]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_L_M);
	mag[5]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_H_M);
}



