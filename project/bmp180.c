#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <math.h>
#include "bmp180.h"


/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/

// Predeclarations
static char I2CReadByte(int, int);
static unsigned short I2CReadU16(int, int);
static short I2CReadS16(int, int);
static void I2CWriteByte(int, int, int);
static int readRawTemp(int);
static int readRawPressure(int);

// Static variables
static short AC1,AC2,AC3,B1,B2,MB,MC,MD;
static unsigned short AC4,AC5,AC6;


/******************************************************************/
/****************************FUNCTIONS*****************************/
/******************************************************************/

// Read single byte from specified register
static char I2CReadByte(int reg, int fd)
{
    return (char)wiringPiI2CReadReg8(fd,reg);
}

// Read two unsigned bytes starting from specified register using I2C_readByte
static unsigned short I2CReadU16(int reg, int fd)
{
    int MSB,LSB;
    MSB = I2CReadByte(reg, fd);
    LSB = I2CReadByte(reg + 1, fd);
    int value = (MSB << 8) +LSB;
    return (unsigned short)value;
}

// Read two signed bytes starting from specified register
static short I2CReadS16(int reg, int fd)
{
    int result;
    result = I2CReadU16(reg, fd);
    if (result > 32767)result -= 65536;
    return (short)result;
}

// Write byte to specified register
static void I2CWriteByte(int reg,int val, int fd)
{
    wiringPiI2CWriteReg8(fd,reg,val);
}

// Read raw temperature
static int readRawTemp(int fd)
{
    int raw;
    I2CWriteByte(BMP180_CONTROL,BMP180_READTEMPCMD, fd);
    delay(5);  //5ms;
    raw = I2CReadByte(BMP180_TEMPDATA, fd) << 8;
    raw += I2CReadByte(BMP180_TEMPDATA+1, fd);
    return raw;
}

// Read raw pressure
static int readRawPressure(int fd)
{
    int MSB,LSB,XLSB,raw;
    I2CWriteByte(BMP180_CONTROL,BMP180_READPRESSURECMD +(OSS << 6), fd);
    switch(OSS)
    {
        case BMP180_ULTRALOWPOWER:
            delay(5);break;
        case BMP180_HIGHRES:
            delay(14);break;
        case BMP180_ULTRAHIGHRES:
            delay(26);break;
        default :
            delay(8);
    }
    MSB  = I2CReadByte(BMP180_PRESSUREDATA, fd);
    LSB  = I2CReadByte(BMP180_PRESSUREDATA + 1, fd);
    XLSB = I2CReadByte(BMP180_PRESSUREDATA + 2, fd);
    raw = ((MSB << 16) + (LSB << 8) + XLSB) >> (8 - OSS);
    return raw;
}



// Global function for loading factory calibration data from bmp sensor
void enableBMP(int fd)
{
    AC1 = I2CReadS16(BMP180_CAL_AC1, fd);
    AC2 = I2CReadS16(BMP180_CAL_AC2, fd);
    AC3 = I2CReadS16(BMP180_CAL_AC3, fd);
    AC4 = I2CReadU16(BMP180_CAL_AC4, fd);
    AC5 = I2CReadU16(BMP180_CAL_AC5, fd);
    AC6 = I2CReadU16(BMP180_CAL_AC6, fd);
    B1  = I2CReadS16(BMP180_CAL_B1, fd);
    B2  = I2CReadS16(BMP180_CAL_B2, fd);
    MB  = I2CReadS16(BMP180_CAL_MB, fd);
    MC  = I2CReadS16(BMP180_CAL_MC, fd);
    MD  = I2CReadS16(BMP180_CAL_MD, fd);
}

// Global function for reading temperature
static float readTemperature(int fd)
{
    float T;
    int UT,X1,X2,B5;
    UT = readRawTemp(fd);
    X1 = ((UT - AC6)*AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    T = ((B5 + 8) >> 4) /10.0;
    return T;
}

// Global function for reading pressure
static float readPressure(int fd)
{
    int P;
    int UT,UP,X1,X2,X3,B3,B5,B6;
    unsigned int B4;
    int B7;
    UT = readRawTemp(fd);
    UP = readRawPressure(fd);

    X1 = ((UT - AC6)*AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    //Pressure Calculations
    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6) >> 12) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (X3 + 32768)) >> 15;
    B7 = (UP - B3) * (50000 >> OSS);
    if (B7 < 0x80000000){P = (B7 * 2) / B4;}
    else {P = (B7 / B4) * 2;}
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);
    return P;

}

// Global function for reading altitude
static float readAltitude(int fd)
{
    float pressure,altitude;
    float sealevelPa = 101325.0;
    pressure = (float)readPressure(fd);
    altitude = 44330.0 * (1.0 - pow(pressure / sealevelPa,(1.0/5.255)));
    return altitude;
}

// Global function for reading sea lever pressure
static float readSealevelPressure(int fd)
{
    float altitude_m = 0.0;
    float pressure,p0;
    pressure =(float)readPressure(fd);
    p0 = pressure / pow(1.0 - altitude_m/44330.0,5.255);
    return p0;
}

// Global function for returning array of data
void readBMP(float *bmpRaw, int fd){
	bmpRaw[0]=readTemperature(fd);
	bmpRaw[1]=readPressure(fd)/100.0;
	bmpRaw[2]=readAltitude(fd);
}
