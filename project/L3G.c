#include "L3G.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <inttypes.h>

static uint8_t gyr[6];

// Function for enabling Gyroscope sensor
void enableGyroscope(int fd){
	wiringPiI2CWriteReg8(fd,L3G_CTRL_REG1, 0b00001111); // Normal power mode, all axes enabled
	wiringPiI2CWriteReg8(fd,L3G_CTRL_REG4, 0b00110000); // Continuos update, 2000 dps full scale
}


// Function for reading Gyroscope data
void readGyroscope(float *gyrRaw, int fd){
	gyr[0]=wiringPiI2CReadReg8(fd,L3G_OUT_X_L);
	gyr[1]=wiringPiI2CReadReg8(fd,L3G_OUT_X_H);
	gyr[2]=wiringPiI2CReadReg8(fd,L3G_OUT_Y_L);
	gyr[3]=wiringPiI2CReadReg8(fd,L3G_OUT_Y_H);
	gyr[4]=wiringPiI2CReadReg8(fd,L3G_OUT_Z_L);
	gyr[5]=wiringPiI2CReadReg8(fd,L3G_OUT_Z_H);
	
	gyrRaw[0] = (float)(int16_t)(gyr[1] << 8| gyr[0])*0.07*(3.141592653589793/180);
	gyrRaw[1] = (float)(int16_t)(gyr[3] << 8| gyr[2])*0.07*(3.141592653589793/180);
	gyrRaw[2] = (float)(int16_t)(gyr[5] << 8| gyr[4])*0.07*(3.141592653589793/180);

}
