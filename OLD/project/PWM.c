#include "PWM.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <inttypes.h>
#include <unistd.h>


static void scalePWM(float*, int*);

// Function for enabling Gyroscope sensor
void enablePWM(int fd, int freq){
	// Initializing the PWM board
	wiringPiI2CWriteReg8(fd, ALL_LED_ON_L, 0 & 0xFF);
	wiringPiI2CWriteReg8(fd, ALL_LED_ON_H, 0 >> 8);
	wiringPiI2CWriteReg8(fd, ALL_LED_OFF_L, 2047 & 0xFF);
	wiringPiI2CWriteReg8(fd, ALL_LED_OFF_H, 2047 >> 8);
	wiringPiI2CWriteReg8(fd, MODE2, OUTDRV);
	wiringPiI2CWriteReg8(fd, MODE1, ALLCALL);
	
	sleep(1); // wait for oscillator
	
	int mode1 = wiringPiI2CReadReg8(fd, MODE1);
	mode1 = mode1 & SLEEP; // wake up (reset sleep)
	wiringPiI2CWriteReg8(fd, MODE1, mode1);
	
	sleep(1);
	
	// Set PWM frequency
	float prescaleval = 25000000.0; // 25MHz
	prescaleval /= 4096.0; // 12-bit
	prescaleval /= (float)freq;
	prescaleval -= 1.0;
	float prescale = floor(prescaleval + 0.5);
	int oldmode = wiringPiI2CReadReg8(fd, MODE1);
	int newmode = (oldmode & 0x7F) | 0x10; // sleep
	wiringPiI2CWriteReg8(fd, MODE1, newmode); // go to sleep
	wiringPiI2CWriteReg8(fd, PRESCALE, (int)floor(prescale)); // go to sleep
	wiringPiI2CWriteReg8(fd, MODE1, oldmode); // go to sleep
	sleep(1);
	wiringPiI2CWriteReg8(fd, MODE1, oldmode | 0x80); // go to sleep
	
	
}


// Function for setting PWM output
void setPWM(int fd, float* value){
	int pwm[4];
	scalePWM(value, pwm); // 0-100% = 2047-4095
	
	// Set PWM values
	for (int i=0;i<4;i++){
		wiringPiI2CWriteReg8(fd, LED0_ON_L+4*i, 0 & 0xFF);
		wiringPiI2CWriteReg8(fd, LED0_ON_H+4*i, 0 >> 8);
		wiringPiI2CWriteReg8(fd, LED0_OFF_L+4*i, pwm[i] & 0xFF);
		wiringPiI2CWriteReg8(fd, LED0_OFF_H+4*i, pwm[i] >> 8);
	}
}


// Function for scaling 0-100% throttel to 12 bit PWM value
static void scalePWM(float* input, int* pwm){
	for(int i=0;i<4;i++){
		pwm[i]=(int)((4095-2047)/(100-0))*input[i]+2047; // ((pmax-pmin)/(%max-%min))*input+pmin
	}
}

