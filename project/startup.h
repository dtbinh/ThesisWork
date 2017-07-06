#ifndef _STARTUP_
#define _STARTUP_

#define _BSD_SOURCE // for usleep();
#define _POSIX_C_SOURCE 199309L // for time.h functionality
#define _GNU_SOURCE // for realtime nanospeep()

/***** REAL TIME SETTINGS *****/

// PREEMPT_RT
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC 1E9

// PRIORITY
#define PRIORITY_CONTROLLER_STATE_UPDATE 4
#define PRIORITY_CONTROLLER_CONSTRAINTS_UPDATE 42
#define PRIORITY_CONTROLLER_MPC 43
#define PRIORITY_CONTROLLER_WATCHDOG 44

#define PRIORITY_SENSOR_BEACON 38
#define PRIORITY_SENSOR_PWM 40
#define PRIORITY_SENSOR_FUSION 39

// Sampling Time
#define tsController 500000000 // 0.5s
#define tsWatchdog 500000000 // 0.5s
//#define tsUDP 100000000
#define tsSensorsFusion 500000000 // 0.5s
#define tsReadBeacon 500000000 // 0.5s

/******************************/



// Model Parameters
#define par_g 9.81 // gravity
#define	par_mass 0.472 // total mass
#define	par_L 0.125 // length from center to motor
#define	par_k 0.000010107 // lift coeff
#define	par_b 0.00000033691 // drag coeff
#define	par_k_d 0.25 // air friction
#define par_i_xx 0.0012 // quad inertia about xb
#define par_i_yy 0.0012 // quad inertia about yb
#define par_i_zz 0.0023 // quad inertia about zb
#define	par_c_m 23.0907 // motor constant

typedef struct _structPipe{
	int parent[2];
	int child[2];
} structPipe;

typedef struct _pipeArray{
	structPipe *pipe1;
	structPipe *pipe2;
} pipeArray;


void printmat(double*, int, int);

#endif
