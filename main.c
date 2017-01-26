/* IMU sensor board addresses are the following:
 * Accelerometer:	0x19
 * Magnometer:		0x1e
 * Gyroscope:		0x69
 * Barometer:		0x77
 */
	 	 

#include <stdio.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <errno.h>
#include "L3G.h"
#include "LSM303.h"
#include <math.h>

int main(int argc, char **argv)
{
	// Setting up I2C communication to each sensor
	int fd_acc;
	int fd_mag;
	int fd_gyr;

	fd_acc=wiringPiI2CSetup(ACC_ADDRESS);
	fd_mag=wiringPiI2CSetup(MAG_ADDRESS);
	fd_gyr=wiringPiI2CSetup(GYR_ADDRESS);

	 if(fd_acc==-1 || fd_mag==-1 || fd_gyr==-1)
	 {
		 printf("Can't setup the I2C devices\n");
		 return -1;
	 }
	 else
	 {
		 // Enabling Accelerometer
		 wiringPiI2CWriteReg8(fd_acc,LSM303_CTRL_REG1_A,0b01010111); //  z,y,x axis enabled , 100Hz data rate
		 wiringPiI2CWriteReg8(fd_acc,LSM303_CTRL_REG4_A,0b00101000); // +/- 8G full scale: FS = 10 on DLHC, high resolution output mode

		 // Enabling Magnometer
		 wiringPiI2CWriteReg8(fd_mag,LSM303_MR_REG_M,0x00); // enable magnometer
		 
		 // Enabling Gyroscope
		 wiringPiI2CWriteReg8(fd_gyr,L3G_CTRL_REG1, 0b00001111); // Normal power mode, all axes enabled
         wiringPiI2CWriteReg8(fd_gyr,L3G_CTRL_REG4, 0b00110000); // Continuos update, 2000 dps full scale

		uint8_t acc[6];
		uint8_t gyr[6];
		uint8_t mag[6];

		float acc_raw[3];
		float gyr_raw[3];
		float mag_raw[3]; 
		
		 
		 while(1)
		 {
			 //printf("%d\n",counter++);
			 
			 // Read Accelerometer
			acc[0]=wiringPiI2CReadReg8(fd_acc,LSM303_OUT_X_L_A);
			acc[1]=wiringPiI2CReadReg8(fd_acc,LSM303_OUT_X_H_A);
			acc[2]=wiringPiI2CReadReg8(fd_acc,LSM303_OUT_Y_L_A);
			acc[3]=wiringPiI2CReadReg8(fd_acc,LSM303_OUT_Y_H_A);
			acc[4]=wiringPiI2CReadReg8(fd_acc,LSM303_OUT_Z_L_A);
			acc[5]=wiringPiI2CReadReg8(fd_acc,LSM303_OUT_Z_H_A);
			
			// Read Magnometer
			mag[0]=wiringPiI2CReadReg8(fd_mag,LSM303_OUT_X_L_M);
			mag[1]=wiringPiI2CReadReg8(fd_mag,LSM303_OUT_X_H_M);
			mag[2]=wiringPiI2CReadReg8(fd_mag,LSM303_OUT_Y_L_M);
			mag[3]=wiringPiI2CReadReg8(fd_mag,LSM303_OUT_Y_H_M);
			mag[4]=wiringPiI2CReadReg8(fd_mag,LSM303_OUT_Z_L_M);
			mag[5]=wiringPiI2CReadReg8(fd_mag,LSM303_OUT_Z_H_M);
			
			// Read Gyroscope
			gyr[0]=wiringPiI2CReadReg8(fd_gyr,L3G_OUT_X_L);
			gyr[1]=wiringPiI2CReadReg8(fd_gyr,L3G_OUT_X_H);
			gyr[2]=wiringPiI2CReadReg8(fd_gyr,L3G_OUT_Y_L);
			gyr[3]=wiringPiI2CReadReg8(fd_gyr,L3G_OUT_Y_H);
			gyr[4]=wiringPiI2CReadReg8(fd_gyr,L3G_OUT_Z_L);
			gyr[5]=wiringPiI2CReadReg8(fd_gyr,L3G_OUT_Z_H);


			// Convert 16 bit 2's complement to floating point value
			acc_raw[0] = (float)(((int16_t)(acc[1] << 8| acc[0]) >> 4)/pow(2,8)*10);
			acc_raw[1] = (float)(((int16_t)(acc[3] << 8| acc[2]) >> 4)/pow(2,8)*10);
			acc_raw[2] = (float)(((int16_t)(acc[5] << 8| acc[4]) >> 4)/pow(2,8)*10);
			mag_raw[0] = (float)(((int16_t)(mag[1] << 8| mag[0]))/pow(2,8)*10);
			mag_raw[1] = (float)(((int16_t)(mag[3] << 8| mag[2]))/pow(2,8)*10);
			mag_raw[2] = (float)(((int16_t)(mag[5] << 8| mag[4]))/pow(2,8)*10);
			gyr_raw[0] = (float)(((int16_t)(gyr[1] << 8| gyr[0]))/pow(2,8)*10);
			gyr_raw[1] = (float)(((int16_t)(gyr[3] << 8| gyr[2]))/pow(2,8)*10);
			gyr_raw[2] = (float)(((int16_t)(gyr[5] << 8| gyr[4]))/pow(2,8)*10);
			
			 printf("Acc_x: %.2f\t Acc_y: %.2f\t Acc_z: %.2f\t\t Gyr_x: %.2f\t Gyr_y: %.2f\t Gyr_z: %.2f\t\t Mag_x: %.2f\t Mag_y: %.2f\t Mag_z: %.2f\t\t\n", acc_raw[0], acc_raw[1], acc_raw[2], gyr_raw[0], gyr_raw[1], gyr_raw[2], mag_raw[0], mag_raw[1], mag_raw[2]);
			 //printf("Acc_x: %.2f\t Acc_y: %.2f\t Acc_z: %.2f\n", acc_raw[0], acc_raw[1], acc_raw[2]);
			
			 delay(100);
		 }
	 }
	return 0;
}



