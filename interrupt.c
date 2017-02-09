#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>

#define INTERRUPT_PIN 0
int start, diff;
volatile int eventCounter = 0;
uint8_t value=100;
int fd;

void myInterrupt(void){
	diff = millis() - start;
	start = millis();
	
	// CONTROLLER GOES HERE

	serialPutchar(fd, value);
	//serialFlush(fd);
		
	printf("Interrupt: %d. Sampling time: %dms. Data sent: %d\n", eventCounter++, diff, value);

}

int main(int argc, char **argv)
{
	if (wiringPiSetup() < 0){
		printf("wiringPiSetup");
	}
	
	if ((fd = serialOpen("/dev/ttyS0", 115200)) < 0){
		printf("serial");
	}

	if (wiringPiISR(0, INT_EDGE_RISING, &myInterrupt) < 0){
		printf("wiringPiISR");
	}
	

	
	while(1){
		//printf("%d", digitalRead(INTERRUPT_PIN));
		//printf("Test\n");
		sleep(2);
	}
	
	return 0;
}

