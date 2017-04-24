#include "LSM303.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>        
#include <inttypes.h>

static uint8_t acc[6];
static uint8_t mag[6];

// Function for enabling Accelerometer sensor
void enableAccelerometer(int fd){
	wiringPiI2CWriteReg8(fd,LSM303_CTRL_REG1_A,0b01010111); //  z,y,x axis enabled , 100Hz data rate
	wiringPiI2CWriteReg8(fd,LSM303_CTRL_REG4_A,0b00101000); // +/- 8G full scale: FS = 10 on DLHC, high resolution output mode
}

// Function for enabling Magnetometer sensor
void enableMagnetometer(int fd){
	wiringPiI2CWriteReg8(fd,LSM303_MR_REG_M,0x00); // enable magnometer
}	


// Function for reading Accelerometer data
void readAccelerometer(double *accRaw, int fd){
	acc[0]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_L_A);
	acc[1]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_H_A);
	acc[2]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_L_A);
	acc[3]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_H_A);
	acc[4]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_L_A);
	acc[5]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_H_A);
	
	// Convert 16 bit 2's complement to floating point value
	accRaw[0] = (double)((int16_t)( acc[0] | acc[1] << 8 ) >> 4)*0.004;
	accRaw[1] = (double)((int16_t)( acc[2] | acc[3] << 8 ) >> 4)*0.004;
	accRaw[2] = (double)((int16_t)( acc[4] | acc[5] << 8 ) >> 4)*0.004;
}
	



// Function for reading Magnetometer data
void readMagnetometer(double *magRaw, int fd){
	mag[0]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_L_M);
	mag[1]=wiringPiI2CReadReg8(fd,LSM303_OUT_X_H_M);
	mag[2]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_L_M);
	mag[3]=wiringPiI2CReadReg8(fd,LSM303_OUT_Y_H_M);
	mag[4]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_L_M);
	mag[5]=wiringPiI2CReadReg8(fd,LSM303_OUT_Z_H_M);
	
	// Convert 16 bit 2's complement to floating point value
	magRaw[0] = (double)(int16_t)(mag[0] | mag[1] << 8)/1100;
	magRaw[1] = (double)(int16_t)(mag[2] | mag[3] << 8)/1100;
	magRaw[2] = (double)(int16_t)(mag[4] | mag[5] << 8)/980;
}
