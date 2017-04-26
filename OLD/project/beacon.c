#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main(int argc, char **argv)
{
	int fd, count, i, n;
	int test=-500;
	uint8_t  data8[30];
	uint16_t data16[2];
	uint32_t data32[4];
	//uint8_t data[30];
	float pos_raw[3];
		
	if ((fd = serialOpen("/dev/ttyACM1",115200)) < 0){
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
		}
			
	if (wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
		
	while(test < 0){
		if (serialDataAvail(fd)){
		/*if ((fd = serialDataAvail(fd)) == -1){
			fprintf(stderr, "Data not available: %s\n", strerror (errno));
			return -1;
		}
		else{
			count = serialDataAvail(fd);
			printf("bytes avail: %d\n", count);
		}*/
		//serialPrintf((int)fd, (const char)serialGetchar(fd));

		if ((int)(data8[0]=serialGetchar(fd)) == -1){
			fprintf(stderr, "serialGetchar, block for 10s: %s\n", strerror (errno));
			return 1;
		}
		else  if (data8[0] == 0xff){
		data8[1] = serialGetchar(fd);	// Data type: 0x47
		data8[2] = serialGetchar(fd);
		data8[3] = serialGetchar(fd);
		data8[4] = serialGetchar(fd);
		n = (int)(data8[4]);
		//printf("%d", n);
		for (i=0;i<n;i++){
			data8[5+i] = serialGetchar(fd);
		};
		data8[5+n] = serialGetchar(fd);
		data8[6+n] = serialGetchar(fd);		
		
		if ((data16[0]=data8[3] << 8| data8[2]) != 0x0011){
			printf("Unrecognized code of data in packet -> %04x\n", data16[0]);
		};
		
		pos_raw[0] = (float)((data32[1] = data8[12] << 24| data8[11] << 16| data8[10] << 8| data8[9]));
		pos_raw[1] = (float)((data32[2] = data8[16] << 24| data8[15] << 16| data8[14] << 8| data8[13]));	
		pos_raw[2] = (float)((data32[3] = data8[20] << 24| data8[19] << 16| data8[18] << 8| data8[17]));
		
		printf("X=%.3f, Y=%.3f, Z=%.3f\n\n", pos_raw[0]/1000, pos_raw[1]/1000, pos_raw[2]/1000);
	}
			/*
			printf("data8[0] is %04x\n", data8[0]);
			printf("data8[1] is %04x\n", data8[1]);
			printf("data16[0] is %04x\n", data16[0]);
			printf("data8[2] is %04x\n", data8[2]);
			printf("data8[3] is %04x\n", data8[3]);
			printf("data8[4] is %04x\n", data8[4]);
			printf("data8[5] is %04x\n", data8[5]);
			printf("data8[6] is %04x\n", data8[6]);
			printf("data8[7] is %04x\n", data8[7]);
			printf("data8[8] is %04x\n", data8[8]);

			printf("data16[1] is %04x\n", data16[1]);
		
			pos_raw[0] = (float)data32[1]/pow(2,8)*10;
			pos_raw[1] = (float)data32[2]/pow(2,8)*10;
			pos_raw[2] = (float)data32[3]/pow(2,8)*10;

			
			//test++;
			//return 2;
			*/
		}
		//printf("data8[8] is %04x\n", data16[0]);
		//test++;
	}   
	return 0;
}
