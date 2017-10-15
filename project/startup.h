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
#define PRIORITY_CONTROLLER_STATE_UPDATE 42
#define PRIORITY_CONTROLLER_CONSTRAINTS_UPDATE 41
#define PRIORITY_CONTROLLER_MPC 60
#define PRIORITY_CONTROLLER_WATCHDOG 44

#define PRIORITY_SENSOR_BEACON 38
#define PRIORITY_SENSOR_PWM 40
#define PRIORITY_SENSOR_FUSION 50
#define PRIORITY_SENSOR_PIPE_COMMUNICATION 30

#define PRIORITY_COMMUNICATION_UDP_WRITE 33
#define PRIORITY_COMMUNICATION_UDP_READ 32
#define PRIORITY_COMMUNICATION_KEYBOARD 31
#define PRIORITY_COMMUNICATION_PIPE_CONTROLLER 29
#define PRIORITY_COMMUNICATION_PIPE_SENSOR 28

// Sampling Time
#define tsController 25e+6 // 0.025s
#define tsWatchdog 500000000 // 0.5s
//#define tsUDP 100000000
#define tsSensorsFusion 10e+6 // 0.025s
#define tsReadBeacon 50000000 // 0.05s
#define tsUdpWrite 10e+6 // 0.1s

/******************************/
// Model Parameters
#define par_g 9.81f // gravity
#define	par_mass 0.4234f // total mass
#define	par_L 0.125f // length from center to motor
#define	par_k 0.000010107f// lift coeff
#define	par_b 0.00000033691f // drag coeff
#define	par_k_d 0.25f // air friction
#define par_i_xx 0.001692037f // quad inertia about xb
#define par_i_yy 0.001463176f // quad inertia about yb
#define par_i_zz 0.0023f // quad inertia about zb
//#define par_i_xx 0.022f // quad inertia about xb
//#define par_i_yy 0.022f // quad inertia about yb
//#define par_i_zz 0.043f // quad inertia about zb
//#define	par_c_m 23.0907f // motor constant
#define	par_c_m 31.95f // motor constant

/*
 * 
 * Current Q {x,xdot,y,ydot,xform,yform,phi,phidot,theta,thetadot,psi,psidot,z,zdot}
{10000,1,1,1,6000,6000,1000,10,1000,10,0.0000001,0.0000001,100,100}
Current R {theta_ref,phi_ref,taux,tauy,tauz,thrust}
{1000,1000,1,1,10000000,1
Keyboard listening... 

 **/ 
 
#define manualThrust -1.0f


// Default MPC POSITION weights Q (Qf) and R
#define mpcPos_Q_1 10.0f
#define mpcPos_Q_2 1.0f
#define mpcPos_Q_3 10.0f
#define mpcPos_Q_4 1.0f
#define mpcPos_Q_5 1.0f
#define mpcPos_Q_6 1.0f

#define mpcPos_R_1 10.0f
#define mpcPos_R_2 10.0f

// Default MPC ATTITUDE weights Q (Qf) and R
#define mpcAtt_Q_1 1500.0f
#define mpcAtt_Q_2 100.0f
#define mpcAtt_Q_3 1500.0f
#define mpcAtt_Q_4 100.0f
#define mpcAtt_Q_5 100.0f
#define mpcAtt_Q_6 300.0f

//Qf for 1500,100,150,10,1,1 and 1,5000,1e10
//{{17760.1, 406.515, 0., 0., 0., 0.}, 
//{406.515, 110.167, 0., 0., 0., 0.}, 
//{0., 0., 1898.2, 76.5355, 0., 0.}, 
//{0., 0., 76.5355, 22.2999, 0., 0.}, 
//{0., 0., 0., 0., 859.837, 9210.74}, 
//{0., 0., 0., 0., 9210.74, 197763.}}

#define mpcAtt_Qf_1 17760.1
#define mpcAtt_Qf_2 110.167
#define mpcAtt_Qf_3 17760.1
#define mpcAtt_Qf_4 110.167
#define mpcAtt_Qf_5 859.837
#define mpcAtt_Qf_6 197763
#define mpcAtt_Qf_1_2 406.515
#define mpcAtt_Qf_3_4 406.515
#define mpcAtt_Qf_5_6 9210.74

#define mpcPos_Qf_1 4.531983e+03
#define mpcPos_Qf_2 1.194034e+03
#define mpcPos_Qf_3 4.531983e+03
#define mpcPos_Qf_4 1.194034e+03
#define mpcPos_Qf_5 1.0
#define mpcPos_Qf_6 1.0
#define mpcPos_Qf_1_2 1.634923e+03
#define mpcPos_Qf_3_4 1.634923e+03
#define mpcPos_Qf_5_6 1.0

#define mpcAlt_Qf_1 3.524190e+04
#define mpcAlt_Qf_2 1.120085e+04
#define mpcAlt_Qf_1_2 1.308437e+04

//2000.000000,2000.000000,10000000000000.000000
#define mpcAtt_R_1 1.0f
#define mpcAtt_R_2 1.0f
#define mpcAtt_R_3 100.0f

// Default MPC ALTITUDE weights Q (Qf) and R
#define mpcAlt_Q_1 1000.0f
#define mpcAlt_Q_2 100.0f

#define mpcAlt_R_1 100.0f

// Default EKF weights Q
#define ekf_Q_1 1.0e-2f
#define ekf_Q_2 1.0e-2f
#define ekf_Q_3 1.0e-5f
#define ekf_Q_4 1.0e-2f
#define ekf_Q_5 1.0e-2f
#define ekf_Q_6 1.0e-5f

#define ekf_Q_7 1.0e-4f
#define ekf_Q_8 1.0e-4f
#define ekf_Q_9 1.0e-6f
#define ekf_Q_10 1.0e-4f
#define ekf_Q_11 1.0e-4f
#define ekf_Q_12 1.0e-6f

#define ekf_Q_13 1.0e-10f // dist x
#define ekf_Q_14 1.0e-10f // dist y
#define ekf_Q_15 1.0e-10f // dist z

#define ekf_Q_16 1.0f
#define ekf_Q_17 1.0f
#define ekf_Q_18 1.0f

// Default PID gains
//#define pid_gyro_kp 2e-2f
//#define pid_gyro_ki 0.0f
//#define pid_gyro_kd 0.0f
//#define pid_angle_kp 4.0f
//#define pid_angle_ki 3.0e-1f
//#define pid_angle_kd 0.0f
#define pid_pos_x_kp_def 5e-2f
#define pid_pos_x_ki_def 1e-3f
#define pid_pos_y_kp_def 5e-2f
#define pid_pos_y_ki_def 1e-3f

// Default MPC integral gains
#define mpcAtt_ki_def 3.0e-1f
#define mpcPos_ki_def 0.0f

// pid_angle_ki 6.0e-1f for PID controller!
//{0.045000,0.000000,0.000000,1.500000,1.000000,0.000000} for c_m=23... and without batteries

// Agent ID's
//#define SUPERVISOR "A0"
#define AGENT1 1017
#define AGENT2 1016
#define AGENT3 1015
#define MYSELF AGENT1

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
