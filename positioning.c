#include <stdio.h>
#include "marvelmind.h"

int main(int argc, char **argv)
{
	createMarvelmindHedge();
	
struct MarvelmindHedge
{
// serial port device name (physical or USB/virtual). It should be provided as
// an argument:
// /dev/ttyACM0 - typical for Linux / Raspberry Pi
// /dev/tty.usbmodem1451 - typical for Mac OS X
    const char * ttyFileName;

// Baud rate. Should be match to baudrate of hedgehog-beacon
// default: 9600
    uint32_t baudRate;

// maximum count of measurements of coordinates stored in buffer
// default: 3
    uint8_t maxBufferedPositions;

// buffer of measurements
    struct PositionValue * positionBuffer;

    struct StationaryBeaconsPositions positionsBeacons;

// verbose flag which activate console output
//		default: False
    bool verbose;

//	pause flag. If True, class would not read serial data
    bool pause;

//  If True, thread would exit from main loop and stop
    bool terminationRequired;

//  receiveDataCallback is callback function to recieve data
    void (*receiveDataCallback)(struct PositionValue position);

// private variables
    uint8_t lastValuesCount_;
    bool haveNewValues_;
#ifdef WIN32
    HANDLE thread_;
    CRITICAL_SECTION lock_;
#else
    pthread_t thread_;
    pthread_mutex_t lock_;
#endif

};
	
	return 0;
}

