#ifndef _COMMUNICATION_
#define _COMMUNICATION_

#define PIPE_BUFFER_SIZE 1
#define SERVERPORT 4950
#define BUFFER_LENGTH 200

#define SUPERVISOR "A0"
#define AGENT1 "A1"
#define AGENT2 "A2"
#define AGENT3 "A3"
#define MYSELF "A1"
#define DATA "DA"
#define STREAM "ST"
#define SETPOINT "SP"
#define TUNING "TU"

extern void startCommunication(void*, void*);

#endif
