#ifndef _STARTUP_
#define _STARTUP_

#define _BSD_SOURCE // for usleep();
#define _POSIX_C_SOURCE 199309L // for time.h functionality
#define _GNU_SOURCE // for realtime nanospeep()

// PREEMPT_RT
#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC (1000000000)

#define tsController 100000000
#define tsWatchdog 100000000
#define tsUDP 100000000
#define tsSensors 100000000

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
}pipeArray;


#endif
