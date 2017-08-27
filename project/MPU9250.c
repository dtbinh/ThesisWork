#include "MPU9250.h"
#include "startup.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <unistd.h>

static uint8_t c;
static uint8_t Gscale = GFS_250DPS; // Specify sensor full scale
static uint8_t Ascale = AFS_2G; // Specify sensor full scale
static uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
static uint8_t Mmode = M_100HZ; // M_8HZ; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
static int fdMPU9250, fdAK8963;
static float fSelfTest[6], aRes, gRes, mRes, gyroBias[3]  = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3]   = {0, 0, 0}, magScale[3]  = {0, 0, 0}, factoryMagCalibration[3] = {0,0,0};      

static void MPU9250SelfTest();
static void initMPU9250();
static void initAK8963();
static void calibrateMPU9250();
static void magCalMPU9250();
static void getAres();
static void getGres();
static void getMres();
static void readMagDataI(int16_t*);

// Function for getting resolution factor accelerometer
void getAres(){
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}

// Function for getting resolution factors gyroscope
void getGres(){
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

// Function for getting resolution factors magnetometer
void getMres(){
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

// Function for enabling MPU9250 sensor
int enableMPU9250(){
	fdMPU9250=wiringPiI2CSetup(MPU9250_ADDRESS);
	if(fdAK8963==-1){ 
	 printf("Error setup the I2C device MPU9250\n");
	 return -1;
	}
	else{
		// Read the WHO_AM_I register of the accelerometer and gyroscope
		c = wiringPiI2CReadReg8(fdMPU9250,WHO_AM_I_MPU9250);
		printf("MPU9250 I AM %i\n", c);
		printf("I should be %i\n", 0x71);	
		if (c==0x71){
			printf("MPU9250 is online...\n");
			MPU9250SelfTest();
			calibrateMPU9250();
			initMPU9250();
			return 0;
		}
		
		else{
			printf("MPU9250 Communication failed, abort!\n\n");
			return -1;
		}
	}
}

// Function for enabling AK8963 sensor (within MPU9250, hence enableMPU9250() must execute first)
int enableAK8963(){
	fdAK8963=wiringPiI2CSetup(AK8963_ADDRESS);
	if(fdAK8963==-1){ 
	 printf("Error setup the I2C device AK8963\n");
	 return -1;
	}
	else{
		printf("AK8963 is online...\n");
		initAK8963();
		magCalMPU9250();
		return 0;
	}
}

// Start by performing self test and reporting values
void MPU9250SelfTest(){
	// local variables
	uint8_t rawData[6]={0,0,0,0,0,0};
	uint8_t selfTest[6];
	int32_t gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
	float factoryTrim[6];
	uint8_t FS=0;
	
	// Set gyro sample rate to 1 kHz
	wiringPiI2CWriteReg8(fdMPU9250,SMPLRT_DIV, 0x00);
	// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	wiringPiI2CWriteReg8(fdMPU9250, CONFIG, 0x02);
	// Set full scale range for the gyro to 250 dps
	wiringPiI2CWriteReg8(fdMPU9250, GYRO_CONFIG, 1<<FS);
	// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG2, 0x02);
	// Set full scale range for the accelerometer to 2 g
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG, 1<<FS);

	// Get average current values of gyro and acclerometer
	for (int ii = 0; ii < 200; ii++){
		//printf("BHW::ii = %i\n",ii);
		// Read the six raw data registers into data array
		rawData[0]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_XOUT_H);
		rawData[1]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_XOUT_L);
		rawData[2]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_YOUT_H);
		rawData[3]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_YOUT_L);
		rawData[4]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_ZOUT_H);
		rawData[5]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_ZOUT_L);
		
		// Turn the MSB and LSB into a signed 16-bit value
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		// Read the six raw data registers sequentially into data array
		rawData[0]=wiringPiI2CReadReg8(fdMPU9250, GYRO_XOUT_H);
		rawData[1]=wiringPiI2CReadReg8(fdMPU9250, GYRO_XOUT_L);
		rawData[2]=wiringPiI2CReadReg8(fdMPU9250, GYRO_YOUT_H);
		rawData[3]=wiringPiI2CReadReg8(fdMPU9250, GYRO_YOUT_L);
		rawData[4]=wiringPiI2CReadReg8(fdMPU9250, GYRO_ZOUT_H);
		rawData[5]=wiringPiI2CReadReg8(fdMPU9250, GYRO_ZOUT_L);

		// Turn the MSB and LSB into a signed 16-bit value
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	// Get average of 200 values and store as average current readings
	for (int ii =0; ii < 3; ii++){
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	// Enable self test on all three axes and set accelerometer range to +/- 2 g
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG, 0xE0);
	// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	wiringPiI2CWriteReg8(fdMPU9250, GYRO_CONFIG,  0xE0);
	usleep(25000);  // Delay a while to let the device stabilize

	// Get average self-test values of gyro and acclerometer
	for (int ii = 0; ii < 200; ii++){
		// Read the six raw data registers into data array
		rawData[0]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_XOUT_H);
		rawData[1]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_XOUT_L);
		rawData[2]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_YOUT_H);
		rawData[3]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_YOUT_L);
		rawData[4]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_ZOUT_H);
		rawData[5]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_ZOUT_L);
		
		// Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		// Read the six raw data registers sequentially into data array
		rawData[0]=wiringPiI2CReadReg8(fdMPU9250, GYRO_XOUT_H);
		rawData[1]=wiringPiI2CReadReg8(fdMPU9250, GYRO_XOUT_L);
		rawData[2]=wiringPiI2CReadReg8(fdMPU9250, GYRO_YOUT_H);
		rawData[3]=wiringPiI2CReadReg8(fdMPU9250, GYRO_YOUT_L);
		rawData[4]=wiringPiI2CReadReg8(fdMPU9250, GYRO_ZOUT_H);
		rawData[5]=wiringPiI2CReadReg8(fdMPU9250, GYRO_ZOUT_L);
		
		// Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	// Get average of 200 values and store as average self-test readings
	for (int ii =0; ii < 3; ii++){
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG, 0x00);
	wiringPiI2CWriteReg8(fdMPU9250, GYRO_CONFIG,  0x00);
	usleep(25000);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	// X-axis accel self-test results
	selfTest[0] = wiringPiI2CReadReg8(fdMPU9250, SELF_TEST_X_ACCEL);
	// Y-axis accel self-test results
	selfTest[1] = wiringPiI2CReadReg8(fdMPU9250, SELF_TEST_Y_ACCEL);
	// Z-axis accel self-test results
	selfTest[2] = wiringPiI2CReadReg8(fdMPU9250, SELF_TEST_Z_ACCEL);
	// X-axis gyro self-test results
	selfTest[3] = wiringPiI2CReadReg8(fdMPU9250, SELF_TEST_X_GYRO);
	// Y-axis gyro self-test results
	selfTest[4] = wiringPiI2CReadReg8(fdMPU9250, SELF_TEST_Y_GYRO);
	// Z-axis gyro self-test results
	selfTest[5] = wiringPiI2CReadReg8(fdMPU9250, SELF_TEST_Z_GYRO);

	// Retrieve factory self-test value from self-test code reads
	// FT[Xa] factory trim calculation
	factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
	// FT[Ya] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
	// FT[Za] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
	// FT[Xg] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
	// FT[Yg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
	// FT[Zg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
	// of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++){
		// Report percent differences
		fSelfTest[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
		- 100.;
		// Report percent differences
		fSelfTest[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
		- 100.;
	}

	printf("x-axis self test: acceleration trim within : %f %% of factory value\n", fSelfTest[0]);
	printf("y-axis self test: acceleration trim within : %f %% of factory value\n", fSelfTest[1]);
	printf("z-axis self test: acceleration trim within : %f %% of factory value\n", fSelfTest[2]);
	printf("x-axis self test: gyration trim within : %f %% of factory value\n", fSelfTest[3]);
	printf("y-axis self test: gyration trim within : %f %% of factory value\n", fSelfTest[4]);
	printf("z-axis self test: gyration trim within : %f %% of factory value\n", fSelfTest[5]);
	
	printf("MPU9250 self test finished\n");
}

// Calibration routine for accelerometer and gyroscope
static void calibrateMPU9250(){
	// Calibrate gyro and accelerometers, load biases in bias registers
	printf("Acc and Gyr Calibration start!\n");
	
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, jj, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	// Write a one to bit 7 reset bit; toggle reset device
	wiringPiI2CWriteReg8(fdMPU9250, PWR_MGMT_1, READ_FLAG);
	usleep(100000);

	// get stable time source; Auto select clock source to be PLL gyroscope
	// reference if ready else use the internal oscillator, bits 2:0 = 001
	wiringPiI2CWriteReg8(fdMPU9250, PWR_MGMT_1, 0x01);
	wiringPiI2CWriteReg8(fdMPU9250, PWR_MGMT_2, 0x00);
	usleep(200000);

	// Configure device for bias calculation
	// Disable all interrupts
	wiringPiI2CWriteReg8(fdMPU9250, INT_ENABLE, 0x00);
	// Disable FIFO
	wiringPiI2CWriteReg8(fdMPU9250, FIFO_EN, 0x00);
	// Turn on internal clock source
	wiringPiI2CWriteReg8(fdMPU9250, PWR_MGMT_1, 0x00);
	// Disable I2C master
	wiringPiI2CWriteReg8(fdMPU9250, I2C_MST_CTRL, 0x00);
	// Disable FIFO and I2C master modes
	wiringPiI2CWriteReg8(fdMPU9250, USER_CTRL, 0x00);
	// Reset FIFO and DMP
	wiringPiI2CWriteReg8(fdMPU9250, USER_CTRL, 0x0C);
	usleep(15000);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	// Set low-pass filter to 188 Hz
	wiringPiI2CWriteReg8(fdMPU9250, CONFIG, 0x01);
	// Set sample rate to 1 kHz
	wiringPiI2CWriteReg8(fdMPU9250, SMPLRT_DIV, 0x00);
	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	wiringPiI2CWriteReg8(fdMPU9250, GYRO_CONFIG, 0x00);
	// Set accelerometer full-scale to 2 g, maximum sensitivity
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG, 0x00);

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	wiringPiI2CWriteReg8(fdMPU9250, USER_CTRL, 0x40);  // Enable FIFO
	// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
	// MPU-9150)
	wiringPiI2CWriteReg8(fdMPU9250, FIFO_EN, 0x78);
	usleep(40000);  // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	// Disable gyro and accelerometer sensors for FIFO
	wiringPiI2CWriteReg8(fdMPU9250, FIFO_EN, 0x00);
	// Read FIFO sample count
	data[0]=wiringPiI2CReadReg8(fdMPU9250, FIFO_COUNTH);
	data[1]=wiringPiI2CReadReg8(fdMPU9250, FIFO_COUNTL);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	// How many sets of full gyro and accelerometer data for averaging
	packet_count = fifo_count/12;

	for (ii = 0; ii < packet_count; ii++){
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		// Read data from FIFO for averaging.
		// Loop through FIFO list and read number of bytes equal to data[12]
		for (jj = 0; jj < 12; jj++){ 
			data[jj]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W);
		}
		/*
		data[1]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+1);
		data[2]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+2);
		data[3]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+3);
		data[4]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+4);
		data[5]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+5);
		data[6]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+6);
		data[7]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+7);
		data[8]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+8);
		data[9]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+9);
		data[10]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+10);
		data[11]=wiringPiI2CReadReg8(fdMPU9250, FIFO_R_W+11);
		*/
		// Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases.
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}
	
	// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	if (accel_bias[2] > 0L){
		accel_bias[2] -= (int32_t) accelsensitivity;
	}
	else{
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
	// format.
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
	// Biases are additive, so change sign on calculated average gyro biases
	data[1] = (-gyro_bias[0]/4)       & 0xFF;
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	wiringPiI2CWriteReg8(fdMPU9250, XG_OFFSET_H, data[0]);
	wiringPiI2CWriteReg8(fdMPU9250, XG_OFFSET_L, data[1]);
	wiringPiI2CWriteReg8(fdMPU9250, YG_OFFSET_H, data[2]);
	wiringPiI2CWriteReg8(fdMPU9250, YG_OFFSET_L, data[3]);
	wiringPiI2CWriteReg8(fdMPU9250, ZG_OFFSET_H, data[4]);
	wiringPiI2CWriteReg8(fdMPU9250, ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer
	// bias registers. These registers contain factory trim values which must be
	// added to the calculated accelerometer biases; on boot up these registers
	// will hold non-zero values. In addition, bit 0 of the lower byte must be
	// preserved since it is used for temperature compensation calculations.
	// Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	// A place to hold the factory accelerometer trim biases
	int32_t accel_bias_reg[3] = {0, 0, 0};
	// Read factory accelerometer trim values
	data[0]=wiringPiI2CReadReg8(fdMPU9250, XA_OFFSET_H);
	
	
	
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	data[0]=wiringPiI2CReadReg8(fdMPU9250, YA_OFFSET_H);
	data[1]=wiringPiI2CReadReg8(fdMPU9250, YA_OFFSET_L);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	data[0]=wiringPiI2CReadReg8(fdMPU9250, ZA_OFFSET_H);
	data[1]=wiringPiI2CReadReg8(fdMPU9250, ZA_OFFSET_H);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	// Define mask for temperature compensation bit 0 of lower byte of
	// accelerometer bias registers
	uint32_t mask = 1uL;
	// Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = {0, 0, 0};

	for (ii = 0; ii < 3; ii++){
		// If temperature compensation bit is set, record that fact in mask_bit
		if ((accel_bias_reg[ii] & mask)){
		  mask_bit[ii] = 0x01;
		}
	}

	// Construct total accelerometer bias, including calculated average
	// accelerometer bias from above
	// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
	// (16 g full scale)
	accel_bias_reg[0] -= (accel_bias[0]/8);
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	// preserve temperature compensation bit when writing back to accelerometer
	// bias registers
	data[1] = data[1] | mask_bit[0];
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	// Preserve temperature compensation bit when writing back to accelerometer
	// bias registers
	data[3] = data[3] | mask_bit[1];
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	// Preserve temperature compensation bit when writing back to accelerometer
	// bias registers
	data[5] = data[5] | mask_bit[2];

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	wiringPiI2CWriteReg8(fdMPU9250, XA_OFFSET_H, data[0]);
	wiringPiI2CWriteReg8(fdMPU9250, XA_OFFSET_L, data[1]);
	wiringPiI2CWriteReg8(fdMPU9250, YA_OFFSET_H, data[2]);
	wiringPiI2CWriteReg8(fdMPU9250, YA_OFFSET_L, data[3]);
	wiringPiI2CWriteReg8(fdMPU9250, ZA_OFFSET_H, data[4]);
	wiringPiI2CWriteReg8(fdMPU9250, ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
	accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
	accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
	
	printf("Acc and Gyr Calibration done!\n");
	printf("MPU9250 bias\n");
	printf("Acc [g] X: %6.4f Y: %6.4f Z: %6.4f\n", accelBias[0], accelBias[1], accelBias[2]);
	printf("Gyr [o/s] X: %6.4f Y: %6.4f Z: %6.4f\n", gyroBias[0], gyroBias[1], gyroBias[2]);

	// Get resolution data
	getAres();
	getGres();
}

// Initialize accelerometer and gyroscope
void initMPU9250(){
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
	// wake up device
	// Clear sleep mode bit (6), enable all sensors
	wiringPiI2CWriteReg8(fdMPU9250, PWR_MGMT_1, 0x00);
	usleep(100000); // Wait for all registers to reset

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else
	wiringPiI2CWriteReg8(fdMPU9250, PWR_MGMT_1, 0x01);
	usleep(200000);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
	// respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion
	// update rates cannot be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
	// 8 kHz, or 1 kHz
	wiringPiI2CWriteReg8(fdMPU9250, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above.
	wiringPiI2CWriteReg8(fdMPU9250, SMPLRT_DIV, 0x04);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
	// left-shifted into positions 4:3

	// get current GYRO_CONFIG register value
	uint8_t c = wiringPiI2CReadReg8(fdMPU9250, GYRO_CONFIG);
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
	// GYRO_CONFIG
	// c =| 0x00;
	// Write new GYRO_CONFIG value to register
	wiringPiI2CWriteReg8(fdMPU9250, GYRO_CONFIG, c );

	// Set accelerometer full-scale range configuration
	// Get current ACCEL_CONFIG register value
	c = wiringPiI2CReadReg8(fdMPU9250, ACCEL_CONFIG);
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	// Write new ACCEL_CONFIG register value
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG, c);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by
	// choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
	// 1.13 kHz
	// Get current ACCEL_CONFIG2 register value
	c = wiringPiI2CReadReg8(fdMPU9250, ACCEL_CONFIG2);
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	// Write new ACCEL_CONFIG2 register value
	wiringPiI2CWriteReg8(fdMPU9250, ACCEL_CONFIG2, c);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because
	// of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	wiringPiI2CWriteReg8(fdMPU9250, INT_PIN_CFG, 0x22);
	// Enable data ready (bit 0) interrupt
	wiringPiI2CWriteReg8(fdMPU9250, INT_ENABLE, 0x01);
	usleep(100000);
	
	printf("MPU9250 initialized for active data mode....\n");
}

// Initialize magnetometer
void initAK8963(){
    // Get magnetometer calibration from AK8963 ROM
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	// TODO: Test this!! Likely doesn't work
	wiringPiI2CWriteReg8(fdAK8963, AK8963_CNTL, 0x00); // Power down magnetometer
	usleep(10000);
	wiringPiI2CWriteReg8(fdAK8963, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	usleep(10000);

	// Read the x-, y-, and z-axis calibration values
	rawData[0]=wiringPiI2CReadReg8(fdAK8963, AK8963_ASAX);
	rawData[1]=wiringPiI2CReadReg8(fdAK8963, AK8963_ASAY);
	rawData[2]=wiringPiI2CReadReg8(fdAK8963, AK8963_ASAZ);

	// Return x-axis sensitivity adjustment values, etc.
	factoryMagCalibration[0] =  (float)(rawData[0] - 128)/256. + 1.;
	factoryMagCalibration[1] =  (float)(rawData[1] - 128)/256. + 1.;
	factoryMagCalibration[2] =  (float)(rawData[2] - 128)/256. + 1.;
	wiringPiI2CWriteReg8(fdAK8963, AK8963_CNTL, 0x00); // Power down magnetometer
	usleep(10000);

	// Configure the magnetometer for continuous read and highest resolution.
	// Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	// register, and enable continuous mode data acquisition Mmode (bits [3:0]),
	// 0010 for 8 Hz and 0110 for 100 Hz sample rates.

	// Set magnetometer data resolution and sample ODR
	wiringPiI2CWriteReg8(fdAK8963, AK8963_CNTL, Mscale << 4 | Mmode);
	usleep(10000);
    // Initialize device for active mode read of magnetometer
    printf("AK8963 initialized for active data mode....\n");
    
	//printf("Mag Calibration values: \n");
	//printf("Mag factory sensitivity adjustment X: %6.4f Y: %6.4f Z: %6.4f\n", factoryMagCalibration[0], factoryMagCalibration[1], factoryMagCalibration[2]);
}

// Calibration routine for magnetometer
void magCalMPU9250(){
	printf("Mag Calibration start!\n");  

	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3]  = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000}, mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

	// Make sure resolution has been calculated
	getMres();
	printf("4 seconds to get ready followed by 15 seconds of sampling\n");
	usleep(4000000);

	// shoot for ~fifteen seconds of mag data
	// at 8 Hz ODR, new mag data is available every 125 ms
	if (Mmode == M_8HZ){
		sample_count = 128;
	}
	// at 100 Hz ODR, new mag data is available every 10 ms
	if (Mmode == M_100HZ){
		sample_count = 1500;
	}

	for (ii = 0; ii < sample_count; ii++){
		readMagDataI(mag_temp);  // Read the mag data

		for (int jj = 0; jj < 3; jj++)
		{
			if (mag_temp[jj] > mag_max[jj]){
				mag_max[jj] = mag_temp[jj];
			}
			if (mag_temp[jj] < mag_min[jj]){
			mag_min[jj] = mag_temp[jj];
			}
		}

		if (Mmode == M_8HZ){
		  usleep(135000); // At 8 Hz ODR, new mag data is available every 125 ms
		}
		if (Mmode == M_100HZ){
		  usleep(12000);  // At 100 Hz ODR, new mag data is available every 10 ms
		}
	}
	
	printf("Mag data collected. 4 seconds put quadrotor back down\n");
	usleep(4000000);

	// Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	// Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	// Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	// Get 'average' x mag bias in counts
	mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
	// Get 'average' y mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
	// Get 'average' z mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

	// Save mag biases in G for main program
	magBias[0] = (float)mag_bias[0] * mRes * factoryMagCalibration[0];
	magBias[1] = (float)mag_bias[1] * mRes * factoryMagCalibration[1];
	magBias[2] = (float)mag_bias[2] * mRes * factoryMagCalibration[2];

	// Get soft iron correction estimate
	// Get average x axis max chord length in counts
	mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
	// Get average y axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
	// Get average z axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	magScale[0] = avg_rad / ((float)mag_scale[0]);
	magScale[1] = avg_rad / ((float)mag_scale[1]);
	magScale[2] = avg_rad / ((float)mag_scale[2]);

	printf("AK8963 Mag Calibration done!\n");   
	printf("Bias [mG] X: %6.4f Y: %6.4f Z: %6.4f\n", magBias[0], magBias[1], magBias[2]);
	printf("Scale [mG] X: %6.4f Y: %6.4f Z: %6.4f\n", magScale[0], magScale[1], magScale[2]);	   
	printf("Mag factory sensitivity adjustment X: %6.4f Y: %6.4f Z: %6.4f\n", factoryMagCalibration[0], factoryMagCalibration[1], factoryMagCalibration[2]);
}

// Read magnetometer data 16 bit integer (used during calibration only)
void readMagDataI(int16_t *magRawInt16){
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end
	// of data acquisition
	uint8_t rawData[7];
	// Wait for magnetometer data ready bit to be set
	if (wiringPiI2CReadReg8(fdAK8963, AK8963_ST1) & 0x01){
		// Read the six raw data and ST2 registers sequentially into data array
		rawData[0]=wiringPiI2CReadReg8(fdAK8963, AK8963_XOUT_L);
		rawData[1]=wiringPiI2CReadReg8(fdAK8963, AK8963_XOUT_H);
		rawData[2]=wiringPiI2CReadReg8(fdAK8963, AK8963_YOUT_L);
		rawData[3]=wiringPiI2CReadReg8(fdAK8963, AK8963_YOUT_H);
		rawData[4]=wiringPiI2CReadReg8(fdAK8963, AK8963_ZOUT_L);
		rawData[5]=wiringPiI2CReadReg8(fdAK8963, AK8963_ZOUT_H);
		rawData[6]=wiringPiI2CReadReg8(fdAK8963, AK8963_ST2);
		
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		// Check if magnetic sensor overflow set, if not then report data
		if (!(c & 0x08)){
			// Turn the MSB and LSB into a signed 16-bit value
			magRawInt16[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			// Data stored as little Endian
			magRawInt16[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			magRawInt16[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}		
	}
}

// Read magnetometer data 16 bit integer (used during calibration only)
void readMagData(double *magRaw){
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end
	// of data acquisition
	uint8_t rawData[7];
	int16_t magRawInt16[3];
	// Wait for magnetometer data ready bit to be set
	if (wiringPiI2CReadReg8(fdAK8963, AK8963_ST1) & 0x01){
		// Read the six raw data and ST2 registers sequentially into data array
		rawData[0]=wiringPiI2CReadReg8(fdAK8963, AK8963_XOUT_L);
		rawData[1]=wiringPiI2CReadReg8(fdAK8963, AK8963_XOUT_H);
		rawData[2]=wiringPiI2CReadReg8(fdAK8963, AK8963_YOUT_L);
		rawData[3]=wiringPiI2CReadReg8(fdAK8963, AK8963_YOUT_H);
		rawData[4]=wiringPiI2CReadReg8(fdAK8963, AK8963_ZOUT_L);
		rawData[5]=wiringPiI2CReadReg8(fdAK8963, AK8963_ZOUT_H);
		rawData[6]=wiringPiI2CReadReg8(fdAK8963, AK8963_ST2);
		
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		// Check if magnetic sensor overflow set, if not then report data
		if (!(c & 0x08)){
			// Turn the MSB and LSB into a signed 16-bit value

			magRawInt16[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			// Data stored as little Endian
			magRawInt16[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			magRawInt16[2] = ((int16_t)rawData[5] << 8) | rawData[4];
			// Data stored as little Endian
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental
			// corrections
			
			// Get actual magnetometer value, this depends on scale being set
			magRaw[0] = (double)magRawInt16[0] * mRes * factoryMagCalibration[0] - magBias[0];
			magRaw[1] = (double)magRawInt16[1] * mRes * factoryMagCalibration[1] - magBias[1];
			magRaw[2] = (double)magRawInt16[2] * mRes * factoryMagCalibration[2] - magBias[2];
			
		}		
	}
}

// Read accelerometer data
void readAccelData(double *accRaw){
	uint8_t rawData[6];  // x/y/z accel register data stored here
	int16_t accRawInt16[3];
	
	// Read the six raw data registers into data array
	rawData[0]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_XOUT_H);
	rawData[1]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_XOUT_L);
	rawData[2]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_YOUT_H);
	rawData[3]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_YOUT_L);
	rawData[4]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_ZOUT_H);
	rawData[5]=wiringPiI2CReadReg8(fdMPU9250, ACCEL_ZOUT_L);

	// Turn the MSB and LSB into a signed 16-bit value
	accRawInt16[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
	accRawInt16[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	accRawInt16[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	
	// Calculate the accleration value into actual g's
	// This depends on scale being set
	accRaw[0]=(double)accRawInt16[0] * aRes; //-accelBias[0];
	accRaw[1]=(double)accRawInt16[1] * aRes; //-accelBias[1];
	accRaw[2]=(double)accRawInt16[2] * aRes; //-accelBias[2];
}

// Read gyroscope data
void readGyroData(double *gyrRaw){
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	int16_t gyrRawInt16[3];
	
	// Read the six raw data registers sequentially into data array
	rawData[0]=wiringPiI2CReadReg8(fdMPU9250, GYRO_XOUT_H);
	rawData[1]=wiringPiI2CReadReg8(fdMPU9250, GYRO_XOUT_L);
	rawData[2]=wiringPiI2CReadReg8(fdMPU9250, GYRO_YOUT_H);
	rawData[3]=wiringPiI2CReadReg8(fdMPU9250, GYRO_YOUT_L);
	rawData[4]=wiringPiI2CReadReg8(fdMPU9250, GYRO_ZOUT_H);
	rawData[5]=wiringPiI2CReadReg8(fdMPU9250, GYRO_ZOUT_L);
	
	// Turn the MSB and LSB into a signed 16-bit value
	gyrRawInt16[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
	gyrRawInt16[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	gyrRawInt16[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;

	// Calculate the gyro value into actual degrees per second
	// This depends on scale being set
	gyrRaw[0]=(double)gyrRawInt16[0] * gRes; // - myIMU.accelBias[0];
	gyrRaw[1]=(double)gyrRawInt16[1] * gRes; // - myIMU.accelBias[1];
	gyrRaw[2]=(double)gyrRawInt16[2] * gRes; // - myIMU.accelBias[2];
	
}

// Read temperature data
void readTempData(double *tempRaw){
	uint8_t rawData[2]; // x/y/z gyro register data stored here
	int16_t tempRawInt16;
	
	// Read the two raw data registers sequentially into data array
	rawData[0]=wiringPiI2CReadReg8(fdMPU9250, TEMP_OUT_H);
	rawData[1]=wiringPiI2CReadReg8(fdMPU9250, TEMP_OUT_L);
	
	// Turn the MSB and LSB into a 16-bit value
	tempRawInt16=((int16_t)rawData[0] << 8) | rawData[1];
	
	// Temperature in degrees Centigrade
	*tempRaw = ((double)tempRawInt16) / 333.87 + 21.0;	
		
}

// Read all sensor data
int readAllSensorData(double *accRaw, double *gyrRaw, double *magRaw, double *tempRaw){
	// If intPin goes high, all data registers have new data
	// On interrupt, check if data ready interrupt
	if (wiringPiI2CReadReg8(fdMPU9250, INT_STATUS) & 0x01){
		readAccelData(accRaw);
		readGyroData(gyrRaw);
		readMagData(magRaw);
		//readTempData(tempRaw);
		return 1;
	}
	else{
		return 0;
	}
}


/*

void loop()
{

  } 

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {

      }

#ifdef LCD
      display.clearDisplay();
      display.setCursor(0, 0); display.print("MPU9250/AK8963");
      display.setCursor(0, 8); display.print(" x   y   z  ");

      display.setCursor(0,  16); display.print((int)(1000 * myIMU.ax));
      display.setCursor(24, 16); display.print((int)(1000 * myIMU.ay));
      display.setCursor(48, 16); display.print((int)(1000 * myIMU.az));
      display.setCursor(72, 16); display.print("mg");

      display.setCursor(0,  24); display.print((int)(myIMU.gx));
      display.setCursor(24, 24); display.print((int)(myIMU.gy));
      display.setCursor(48, 24); display.print((int)(myIMU.gz));
      display.setCursor(66, 24); display.print("o/s");

      display.setCursor(0,  32); display.print((int)(myIMU.mx));
      display.setCursor(24, 32); display.print((int)(myIMU.my));
      display.setCursor(48, 32); display.print((int)(myIMU.mz));
      display.setCursor(72, 32); display.print("mG");

      display.setCursor(0,  40); display.print("Gyro T ");
      display.setCursor(50,  40); display.print(myIMU.temperature, 1);
      display.print(" C");
      display.display();
#endif // LCD

      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }

#ifdef LCD
      display.clearDisplay();

      display.setCursor(0, 0); display.print(" x   y   z  ");

      display.setCursor(0,  8); display.print((int)(1000 * myIMU.ax));
      display.setCursor(24, 8); display.print((int)(1000 * myIMU.ay));
      display.setCursor(48, 8); display.print((int)(1000 * myIMU.az));
      display.setCursor(72, 8); display.print("mg");

      display.setCursor(0,  16); display.print((int)(myIMU.gx));
      display.setCursor(24, 16); display.print((int)(myIMU.gy));
      display.setCursor(48, 16); display.print((int)(myIMU.gz));
      display.setCursor(66, 16); display.print("o/s");

      display.setCursor(0,  24); display.print((int)(myIMU.mx));
      display.setCursor(24, 24); display.print((int)(myIMU.my));
      display.setCursor(48, 24); display.print((int)(myIMU.mz));
      display.setCursor(72, 24); display.print("mG");

      display.setCursor(0,  32); display.print((int)(myIMU.yaw));
      display.setCursor(24, 32); display.print((int)(myIMU.pitch));
      display.setCursor(48, 32); display.print((int)(myIMU.roll));
      display.setCursor(66, 32); display.print("ypr");

    // With these settings the filter is updating at a ~145 Hz rate using the
    // Madgwick scheme and >200 Hz using the Mahony scheme even though the
    // display refreshes at only 2 Hz. The filter update rate is determined
    // mostly by the mathematical steps in the respective algorithms, the
    // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
    // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
    // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
    // presumably because the magnetometer read takes longer than the gyro or
    // accelerometer reads. This filter update rate should be fast enough to
    // maintain accurate platform orientation for stabilization control of a
    // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050
    // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
    // well!
      display.setCursor(0, 40); display.print("rt: ");
      display.print((float) myIMU.sumCount / myIMU.sum, 2);
      display.print(" Hz");
      display.display();
#endif // LCD

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}
*/
