/* CONTROLLER CODE
 * Abstract: 
 * 		takes states and disturbances estimates and references and updates PWM
 * Updates:
 * 		July 4th - position controller is augmented
 * 		July 4th - *threadController() is added to use three controller functions to run them all with the same Ts
 * 
 */
 
/* SIMULINK changes:
 * 0. Add printmat() function!
 * 1. TIMER_ABSTIME is not delclared and commented out
 * 2. sensorReady and triggerFly are hardcoded to always true 
 * 3. threadUpdateMeasurements() and threadUpdateConstraintsSettingsReferences() are commented out
 * 4. all the mutexes are commented out
 * 5. comment all the RT stuff their libraries (they are blue with ///)
 * 6. comment all the multicore pipe communication (they are blue with ///)
 * 7. change while(1) to if(1) in the controller 
 */

#include "controller.h"
#include "startup.h"

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <errno.h>
#include <semaphore.h>
#include <math.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>

#include "blas.h"
#include "lapack.h"

// PREEMPT_RT
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

#define PI 3.141592653589793


/******************************************************************/
/*******************VARIABLES & PREDECLARATIONS********************/
/******************************************************************/
// Static variables for threads
// static float globalSensorData[6]={0,0,0,0,0,0};
// static float globalConstraintsData[6]={0,0,0,0,0,0};
static double keyboardData[18]={0,0,0,0,0,0,0,0,0,0,1,0.01,0.05,1,0,0,0,0}; // {ref_x,ref_y,ref_z, switch[0=STOP, 1=FLY], pwm_print, timer_print,ekf_print,reset ekf/mpc, EKF print 6 states, reset calibration sensor.c, ramp ref, alpha, beta, mpc position toggle, ff,save data,pid_trigger, pwm range setting}
static double tuningMpcData[14]={mpcPos_Q_1,mpcPos_Q_2,mpcPos_Q_3,mpcPos_Q_4,mpcPos_Q_5,mpcPos_Q_6,mpcAtt_Q_1,mpcAtt_Q_2,mpcAtt_Q_3,mpcAtt_Q_4,mpcAtt_Q_5,mpcAtt_Q_6,mpcAlt_Q_1,mpcAlt_Q_2}; // Q and Qf mpc {x,xdot,y,ydot,xform,yform,phi,phidot,theta,thetadot,psi,psidot,z,zdot}
static double tuningMpcQfData[9]={mpcAtt_Qf_1,mpcAtt_Qf_2,mpcAtt_Qf_3,mpcAtt_Qf_4,mpcAtt_Qf_5,mpcAtt_Qf_6,mpcAtt_Qf_1_2,mpcAtt_Qf_3_4,mpcAtt_Qf_5_6};
static double tuningMpcDataControl[6]={mpcPos_R_1,mpcPos_R_2,mpcAtt_R_1,mpcAtt_R_2,mpcAtt_R_3,mpcAlt_R_1}; // R mpc {pos,pos,taux,tauy,tauz,alt}
static double tuningPidData[6]={pid_pos_x_kp_def,pid_pos_x_ki_def,mpcAtt_ki_def,pid_pos_y_kp_def,pid_pos_y_ki_def,mpcPos_ki_def}; // PID gains
static double manualThrustData[1]={manualThrust};
static double positionsData[9]={0};
static double positions_timeoutData[3]={1};
static double theta_ref_comp, phi_ref_comp; // dx_comp, dy_comp;

static double PWM[4] = { 0, 0, 0, 0 };
//static int globalWatchdog=0;
static const int ione = 1;
static const int itwo = 2;
static const int ithree = 3;
static const int iseven = 7;
static const double fone = 1;
static const double ftwo = 2;
static const double fzero = 0;
static const double fmone = -1;
static int quiet = 0;

// Outputs
static double *attX_all, *attU_all;
static double *posX_all, *posU_all;
static double *altX_all, *altU_all;
static double thrust = 0.0;
static double phi_dist = 0.0;
static double theta_dist = 0.0;
static double tau_x = 0.0;
static double tau_y = 0.0;
static double tau_z = 0.0;

struct Parameters mdl_param = {
	.g = par_g,
	.mass = par_mass,
	.L = par_L,
	.k = par_k,
	.b = par_b,
	.k_d = par_k_d,
	.i_xx = par_i_xx,
	.i_yy = par_i_yy,
	.i_zz = par_i_zz,
	.c_m = par_c_m
};
	
struct AltInputs altInputs = { 
	.xmax = { 50, 50 },
	.xmin = { -50, -50 },
};

struct AttInputs attInputs = { 
	.xmax = { 50, 50, 50, 50, 50, 50 },
	.xmin = { -50, -50, -50, -50, -50, -50 },
};

struct PosInputs posInputs = { 
	.xmax = {  50,  50,  50,  50,  50,  50 },
	.xmin = { -50, -50, -50, -50, -50, -50 },
};

// These will later come from standalone computer and estimator
double references[12] = { 0,0,0,	0,0,0,	NAN,NAN,0,	0,0,0 };
double measurements[12] = { 0,0,0,	0,0,0,	0,0,0,	0,0,0 };
double disturbances [3] = { 0,0,-par_g };	// x , y and z disturbances
double disturbances_tau[3] = {0,0,0}; // torque disturbance attitude x,y,z
double inertias[3] = {par_i_xx,par_i_yy,par_i_zz};
int sensorInitReady=0; // ekfReady[0=not, 1=mpc can start]
 
// Controller variables
static double PosTsSec = 0.05;
static double AttTsSec = 0.05;
static double AltTsSec = 0.05;

// Predeclarations
static void *threadUpdateMeasurements(void*);
static void *threadController(void*);
static void *threadUpdateConstraintsSettingsReferences(void*);
static void controllerPos( struct PosParams *posParams, struct PosInputs *posInputs, double *posX_all, double *posU_all, double *meas, double *ref, double *ref_form, double *error_integral, int mpcPos_ff, double ki, double ts );
static void controllerAtt( struct AttParams *, struct AttInputs *, double *attX_all, double *attU_all, double *meas, double *ref, double* error_integral, int mpcAtt_ff, double ki, double ts);
static void controllerAlt( struct AltParams *, struct AltInputs *, double *altX_all, double *altU_all, double *attU_all, double *meas, double *ref, double *dist);
static void posFmpc( struct PosParams *, struct PosInputs *, double *posX_all, double *posU_all );
static void attFmpc( struct AttParams *, struct AttInputs *, double *attX_all, double *attU_all );
static void altFmpc( struct AltParams *, struct AltInputs *, double *altX_all, double *altU_all );
static void refGen_formation( double *, double *, double * );

static double controllerPID(double, double*, double*, double, double, double, double);

// FMPC functions
static void fmpcsolve(double *A, double *B, double *At, double *Bt, double *eyen, 
        double *eyem, double *Q, double *R, double *Qf, double *zmax, double *zmin, 
        double *x, double *z, int T, int n, int m, int nz, int niters, double kappa);

static void gfgphp(double *Q, double *R, double *Qf, double *zmax, double *zmin, double *z,
        int T, int n, int m, int nz, double *gf, double *gp, double *hp);

static void rdrp(double *A, double *B, double *Q, double *R, double *Qf, double *z, double *nu, 
        double *gf, double *gp, double *b, int T, int n, int m, int nz, 
        double kappa, double *rd, double *rp, double *Ctnu);

static void resdresp(double *rd, double *rp, int T, int n, int nz, double *resd, 
        double *resp, double *res);
        
static void dnudz(double *A, double *B, double *At, double *Bt, double *eyen, 
        double *eyem, double *Q, double *R, double *Qf, double *hp, double *rd, 
        double *rp, int T, int n, int m, int nz, double kappa, double *dnu, double *dz);

// Predeclare thread mutexes
static pthread_mutex_t mutexSensorData = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutexConstraintsData = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************/
/*************************START PROCESS****************************/
/******************************************************************/

// Function to start the sensor process threads

 void startController(void *arg1, void *arg2) {
 	// Create pipe array
 	pipeArray pipeArrayStruct = {.pipe1 = arg1, .pipe2 = arg2 }; //arg1 to sensor, arg2 to comm
 	
 	// Create threads
 	pthread_t threadUpdateMeas, threadCtrl, threadUpdateConstr; //hreadCtrlPos, threadCtrlAtt, threadCtrlAlt, threadCtrlWDPos, threadCtrlWDAtt, threadCtrlWDAlt;
 	int threadPID1, threadPID3, threadPID2; //, res5, res6, res7, res8;
 	
 	threadPID1=pthread_create(&threadUpdateMeas, NULL, &threadUpdateMeasurements, arg1);
 	threadPID2=pthread_create(&threadUpdateConstr, NULL, &threadUpdateConstraintsSettingsReferences, arg2);
	threadPID3=pthread_create(&threadCtrl, NULL, &threadController, &pipeArrayStruct);
	
 	//threadPID4=pthread_create(&threadCtrlWD, NULL, &threadControllerWatchdog, arg2);
 	// res3=pthread_create(&threadCtrlPos, NULL, &threadControllerPos, &pipeArrayStruct);
 	// res4=pthread_create(&threadCtrlWDPos, NULL, &threadControllerWatchdogPos, NULL);
 	// res5=pthread_create(&threadCtrlAtt, NULL, &threadControllerAtt, &pipeArrayStruct);
 	// res6=pthread_create(&threadCtrlWDAtt, NULL, &threadControllerWatchdogAtt, NULL);
 	// res7=pthread_create(&threadCtrlAlt, NULL, &threadControllerAlt, &pipeArrayStruct);
 	// res8=pthread_create(&threadCtrlWDAlt, NULL, &threadControllerWatchdogAlt, NULL);
 	
	/// Set up thread scheduler priority for real time tasks
	struct sched_param paramThread1, paramThread3, paramThread2;
	
	paramThread1.sched_priority = PRIORITY_CONTROLLER_STATE_UPDATE; // set priorities
	paramThread2.sched_priority = PRIORITY_CONTROLLER_CONSTRAINTS_UPDATE;
	paramThread3.sched_priority = PRIORITY_CONTROLLER_MPC;
	
	//paramThread4.sched_priority = PRIORITY_CONTROLLER_WATCHDOG;
	if(sched_setscheduler(threadPID1, SCHED_FIFO, &paramThread1)==-1) {perror("sched_setscheduler failed for threadPID1");exit(-1);}
	if(sched_setscheduler(threadPID2, SCHED_FIFO, &paramThread2)==-1) {perror("sched_setscheduler failed for threadPID2");exit(-1);}
	if(sched_setscheduler(threadPID3, SCHED_FIFO, &paramThread3)==-1) {perror("sched_setscheduler failed for threadPID3");exit(-1);}
	//if(sched_setscheduler(threadPID4, SCHED_FIFO, &paramThread4)==-1) {perror("sched_setscheduler failed for threadPID4");exit(-1);}
 	
 	// If threads created successful, start them
 	if (!threadPID1) pthread_join( threadUpdateMeas, NULL);
	if (!threadPID2) pthread_join( threadUpdateConstr, NULL);
 	if (!threadPID3) pthread_join( threadCtrl, NULL);
 	//if (!threadPID4) pthread_join( threadCtrlWD, NULL);
	// if (!threadPID3) pthread_join( threadCtrlPos, NULL);
 	// if (!threadPID4) pthread_join( threadCtrlWDPos, NULL);
 	// if (!res5) pthread_join( threadCtrlAtt, NULL);
 	// if (!res6) pthread_join( threadCtrlWDAtt, NULL);
 	// if (!res7) pthread_join( threadCtrlAlt, NULL);
 	// if (!res8) pthread_join( threadCtrlWDAlt, NULL);
 }


/******************************************************************/
/*****************************THREADS******************************/
/******************************************************************/

//Thread - Update constriants from other drones (pipe from communication process).
//Includes any updates on setpoints or MPC settings from computer
void *threadUpdateConstraintsSettingsReferences(void *arg) {	
	// Get pipe and define local variables
	//pipeArray *pipeArrayStruct = arg;
	//structPipe *ptrPipe2 = pipeArrayStruct->pipe2;	// to comm
	structPipe *ptrPipe = arg; // to comm

	double communicationDataBuffer[84];
	double keyboardDataBuffer[18];
	double tuningMpcBuffer[14];
	double tuningMpcQfBuffer[9];
	double tuningMpcBufferControl[6];
	double tuningPidBuffer[6];
	double manualThrustBuffer[1];
	double positionsBuffer[9];
	double positions_timeoutBuffer[3];
	
	//// Loop forever streaming data
	while(1){
		// Read data from Communication process
		if (read(ptrPipe->child[0], communicationDataBuffer, sizeof(communicationDataBuffer)) != sizeof(communicationDataBuffer) ) printf("Error in reading 'keyboardData' from Communication to Controller\n");
		//else printf("Controller ID: %d, Read: %f from Communication\n", (int)getpid(), keyboardDataBuffer[0]);
		
		// Put new data in to local variables
		memcpy(keyboardDataBuffer, communicationDataBuffer, sizeof(communicationDataBuffer)*18/84);
		memcpy(tuningMpcBuffer, communicationDataBuffer+18, sizeof(communicationDataBuffer)*14/84);
		memcpy(tuningMpcBufferControl, communicationDataBuffer+32, sizeof(communicationDataBuffer)*6/84);
		memcpy(tuningPidBuffer, communicationDataBuffer+56, sizeof(communicationDataBuffer)*6/84);
		memcpy(tuningMpcQfBuffer, communicationDataBuffer+62, sizeof(communicationDataBuffer)*9/84);	
		memcpy(manualThrustBuffer, communicationDataBuffer+71, sizeof(communicationDataBuffer)*1/84);
		memcpy(tuningMpcQfBuffer, communicationDataBuffer+62, sizeof(communicationDataBuffer)*9/84);	
		memcpy(manualThrustBuffer, communicationDataBuffer+71, sizeof(communicationDataBuffer)*1/84);
		memcpy(positionsBuffer, communicationDataBuffer+72, sizeof(communicationDataBuffer)*9/84);	
		memcpy(positions_timeoutBuffer, communicationDataBuffer+81, sizeof(communicationDataBuffer)*3/84);
	
		// Put new constraints data in to global data in controller.c such that controller thread can access and use it.
		pthread_mutex_lock(&mutexConstraintsData);
			memcpy(references, keyboardDataBuffer, sizeof(keyboardDataBuffer)*3/18); // {ref_x,ref_y,ref_z}
			memcpy(keyboardData, keyboardDataBuffer, sizeof(keyboardDataBuffer)); // {ref_x,ref_y,ref_z, switch[0=STOP, 1=FLY], pwm_print, timer_print,ekf_print,reset ekf/mpc, EKF print 6 states, reset calibration sensor.c, ramp ref}
			memcpy(tuningMpcData, tuningMpcBuffer, sizeof(tuningMpcBuffer));
			memcpy(tuningMpcQfData, tuningMpcQfBuffer, sizeof(tuningMpcQfBuffer));
			memcpy(tuningMpcDataControl, tuningMpcBufferControl, sizeof(tuningMpcBufferControl));
			memcpy(tuningPidData, tuningPidBuffer, sizeof(tuningPidBuffer));
			memcpy(manualThrustData, manualThrustBuffer, sizeof(manualThrustBuffer));
			memcpy(positionsData, positionsBuffer, sizeof(positionsBuffer));
			memcpy(positions_timeoutData, positions_timeoutBuffer, sizeof(positions_timeoutBuffer));
			//printf("positions_timeout: %f, %f, %f\n",positions_timeoutData[0], positions_timeoutData[1], positions_timeoutData[2]);
		pthread_mutex_unlock(&mutexConstraintsData);
		
		

	}

return NULL;
}

//Thread - Update local variables with any new sensor measurements (pipe from sensor process)
void *threadUpdateMeasurements(void *arg) {
 	// Get pipe and define local variables
 	structPipe *ptrPipe = arg;
 	
 	double sensorDataBuffer[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // {x,y,z,xdot,ydot,zdot,phi,theta,psi,omegax,omegay,omegaz,dx,dy,dz,ekfReady[0=not,1=mpc can start],dx_tau,dy_tau,dz_tau}
 	
 	while(1){
 		// Read data from sensor process. Data should always be available for controller.
 		if(read(ptrPipe->child[0], sensorDataBuffer, sizeof(sensorDataBuffer)) == -1) printf("read error in Controller from Sensor\n");
 		//else printf("Controller ID: %d, Recieved Sensor data: %f\n", (int)getpid(), sensorDataBuffer[0])
 		
 		// Put new sensor data in to global data in controller.c such that controller thread can access and use it.
 		pthread_mutex_lock(&mutexSensorData);
 			memcpy(measurements, sensorDataBuffer, sizeof(sensorDataBuffer)*12/16);
 			disturbances[0]=sensorDataBuffer[12];
 			disturbances[1]=sensorDataBuffer[13];
 			disturbances[2]=sensorDataBuffer[14];
 			disturbances_tau[0]=sensorDataBuffer[16];
			disturbances_tau[1]=sensorDataBuffer[17];
 			disturbances_tau[2]=sensorDataBuffer[18];
 			sensorInitReady=(int)sensorDataBuffer[15]; // ekfReady[0=not, 1=mpc can start]
 		pthread_mutex_unlock(&mutexSensorData);
 	}
 	
 	return NULL;
 }

/* Thread - Controller algorithm for all three (with pipe to sensor (PWM) and communication process) */
void *threadController( void *arg ) {
	// Initialize local structures
	struct PosParams posParams = { 
		.A = { 1,0,0,0,0,0,		PosTsSec,1,0,0,PosTsSec,0,		0,0,1,0,0,0,		0,0,PosTsSec,1,0,PosTsSec,		0,0,0,0,1,0,		0,0,0,0,0,1 },
		.B = { 0,mdl_param.g*PosTsSec,0,0,0,0,		0,0,0,-mdl_param.g*PosTsSec,0,0 },
		.Q =  { mpcPos_Q_1,0,0,0,0,0,	0,mpcPos_Q_2,0,0,0,0,	0,0,mpcPos_Q_3,0,0,0,	0,0,0,mpcPos_Q_4,0,0,	0,0,0,0,mpcPos_Q_5,0,	0,0,0,0,0,mpcPos_Q_6 },
		//.Qf = { mpcPos_Qf_1,mpcPos_Qf_1_2,0,0,0,0,	mpcPos_Qf_1_2,mpcPos_Qf_2,0,0,0,0,	0,0,mpcPos_Qf_3,mpcPos_Qf_3_4,0,0,	0,0,mpcPos_Qf_3_4,mpcPos_Qf_4,0,0,	0,0,0,0,mpcPos_Qf_5,mpcPos_Qf_5_6,	0,0,0,0,mpcPos_Qf_5_6,mpcPos_Qf_6 },
		.Qf = { mpcPos_Q_1,0,0,0,0,0,	0,mpcPos_Q_2,0,0,0,0,	0,0,mpcPos_Q_3,0,0,0,	0,0,0,mpcPos_Q_4,0,0,	0,0,0,0,mpcPos_Q_5,0,	0,0,0,0,0,mpcPos_Q_6 },
		.R = { mpcPos_R_1,0,		0,mpcPos_R_2 },
		.umax = { 15*PI/180, 15*PI/180 },
		.umin = { -15*PI/180, -15*PI/180 },
		.n = 6, .m = 2, .T = 20, .niters = 5, .kappa = 1e-3
	};	
	
	posX_all = calloc(posParams.n*posParams.T, sizeof(double));
	posU_all = calloc(posParams.m*posParams.T, sizeof(double));
	
	struct AttParams attParams = { 
		.A = { 1,0,0,0,0,0,		AttTsSec,1,0,0,0,0,	0,0,1,0,0,0,	0,0,AttTsSec,1,0,0,	0,0,0,0,1,0,	0,0,0,0,AttTsSec,1 },
		.B = { 0,AttTsSec/mdl_param.i_xx,0,0,0,0,	0,0,0,AttTsSec/mdl_param.i_yy,0,0,	0,0,0,0,0,AttTsSec/mdl_param.i_zz },
		.Q =  { mpcAtt_Q_1,0,0,0,0,0,		0,mpcAtt_Q_2,0,0,0,0,	0,0,mpcAtt_Q_3,0,0,0,		0,0,0,mpcAtt_Q_4,0,0,	0,0,0,0,mpcAtt_Q_5,0,		0,0,0,0,0,mpcAtt_Q_6 },
		//.Qf = { mpcAtt_Q_1,0,0,0,0,0,		0,mpcAtt_Q_2,0,0,0,0,	0,0,mpcAtt_Q_3,0,0,0,		0,0,0,mpcAtt_Q_4,0,0,	0,0,0,0,mpcAtt_Q_5,0,		0,0,0,0,0,mpcAtt_Q_6 },
		.Qf = { mpcAtt_Qf_1,mpcAtt_Qf_1_2,0,0,0,0,		mpcAtt_Qf_1_2,mpcAtt_Qf_2,0,0,0,0,	0,0,mpcAtt_Qf_3,mpcAtt_Qf_3_4,0,0,		0,0,mpcAtt_Qf_3_4,mpcAtt_Qf_4,0,0,	0,0,0,0,mpcAtt_Qf_5,mpcAtt_Qf_5_6,		0,0,0,0,mpcAtt_Qf_5_6,mpcAtt_Qf_6 },
		// Qf with DARE with Q=2e4,1e2,2e4,1e2,1,1 and R = 1000,1000,1e10
		//.Qf = {8.798002e+04,1.738053e+03,-2.789917e-11,-2.684214e-12,2.249382e-11,4.186520e-10,1.738053e+03,1.476911e+02,-1.032293e-12,-9.948871e-14,8.105915e-13,1.202670e-11,-2.789917e-11,-1.032293e-12,8.798002e+04,1.738053e+03,-6.083022e-09,-1.319085e-07,-2.684214e-12,-9.948871e-14,1.738053e+03,1.476911e+02,-1.600276e-10,-3.472108e-09,2.249382e-11,8.105915e-13,-6.083022e-09,-1.600276e-10,8.598367e+02,9.210742e+03,4.186520e-10,1.202670e-11,-1.319085e-07,-3.472108e-09,9.210742e+03,1.977631e+05},
		.R = { mpcAtt_R_1,0,0,	0,mpcAtt_R_2,0,	0,0,mpcAtt_R_3 },
		.umax = {  1e1, 1e1, 1e1 },
		.umin = { -1e1,-1e1,-1e1 },
		//.n = 6, .m = 3, .T = 40, .niters = 5, .kappa = 1e-3 // niters iteration, larger better. kappa smaller better
		.n = 6, .m = 3, .T = 10, .niters = 5, .kappa = 1e-3
	};
	
	attX_all = calloc(attParams.n*attParams.T, sizeof(double));
	attU_all = calloc(attParams.m*attParams.T, sizeof(double));
	
	struct AltParams altParams = { 
		.A = { 1, 0, AltTsSec, 1 },
		.B = { 0, AltTsSec },
		.Q =  { mpcAlt_Q_1, 0, 0, mpcAlt_Q_2 },
		//.Qf = { mpcAlt_Qf_1, mpcAlt_Qf_1_2, mpcAlt_Qf_1_2, mpcAlt_Qf_2 },
		.Qf = { mpcAlt_Q_1, 0, 0, mpcAlt_Q_2 },
		.R = { mpcAlt_R_1 },
		.umax = { -mdl_param.g+100*100*(4*mdl_param.c_m*mdl_param.k)/mdl_param.mass },
		.umin = { .8*(-mdl_param.g+(0)/mdl_param.mass) },
		.n = 2, .m = 1, .T = 10, .niters = 5, .kappa = 1e-2
	};
	
	altX_all = calloc(altParams.n*altParams.T, sizeof(double));
	altU_all = calloc(altParams.m*altParams.T, sizeof(double));
		    
	/// Get pipe array and define local variables
	pipeArray *pipeArrayStruct = arg;
	structPipe *ptrPipe1 = pipeArrayStruct->pipe1;
	//structPipe *ptrPipe2 = pipeArrayStruct->pipe2;

	// Local variables to store global data in to using mutexes
	double measBuffer[12], refBuffer[12], ref_formBuffer[2]={0}, distBuffer[3], distBufferTau[3], point_1[3], point_2[3], p3[3], distance = -1, step_size=0.05, alpha = -1;	
	int triggerFly, sensorReady, pwmPrint, keyRampRef, triggerMpcPos,timerPrint, mpcAtt_ff, mpcPos_ff; // feed forward activation trigger for attitude mpc;
	const double PWM0[4] = { 0.1,0.1,0.1,0.1 };
	double Lbc_mk4 = 4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k;	// common denominator for all lines of forces2PWM calc
	int i;
	
	double tuningMpcBuffer[14];		//Q - 14 states
	double tuningMpcQfBuffer[9];		//Qf
	double tuningMpcBufferControl[6];	//R - 1 for alt, 2 for pos,  3 for att
	double controllerBuffer[19]; // {PWM, thrust, torques} to be sent over to sensor.c
	double tuningPidBuffer[6];
	double manualThrustBuffer[1]={manualThrust};
	double positionsBuffer[9]; // positions of all directly from 'gps'
	double positions_timeoutBuffer[3]; // timeout data
	//double positions_agents[6]; // position of other agents used for formation flying
	//double positions_self_prev[3]; // previous self position used in case positioning system has time out
	
	// PID variables
	//double pid_gyro_error_integral[1]={0};
	//double pid_gyro_error_prev[1]={0};
	//double pid_gyro_kp_local=pid_gyro_kp;
	//double pid_gyro_ki_local=pid_gyro_ki;
	//double pid_gyro_kd_local=pid_gyro_kd;
	
	double mpcAtt_integral_error[2]={0};
	double mpcPos_integral_error[2]={0};
	
	
	
	double pid_pos_x_integral_error[1]={0};
	double pid_pos_x_prev_error[1]={0};
	double pid_pos_x_kp=pid_pos_x_kp_def;
	double pid_pos_x_ki=pid_pos_x_ki_def;
	//double pid_angle_kd_local=pid_angle_kd;
	
	//double pid_gyro_theta_error_integral[1]={0};
	//double pid_gyro_theta_error_prev[1]={0};
	double pid_pos_y_integral_error[1]={0};
	double pid_pos_y_prev_error[1]={0};
	double pid_pos_y_kp=pid_pos_y_kp_def;
	double pid_pos_y_ki=pid_pos_y_ki_def;
	//double pid_gyro_theta_kd_local=pid_gyro_kd;
	
	//double pid_angle_theta_error_integral[1]={0};
	//double pid_angle_theta_error_prev[1]={0};
	
	
	//double pid_angle_theta_kp_local=pid_angle_kp;
	
	
	double mpcAtt_ki=mpcAtt_ki_def;
	double mpcPos_ki=mpcPos_ki_def;
	
	//double pid_angle_theta_kd_local=pid_angle_kd;
	
	//double pid_gyro_psi_error_integral[1]={0};
	//double pid_gyro_psi_error_prev[1]={0};
	//double pid_gyro_psi_kp_local=0.02;
	//double pid_gyro_psi_ki_local=0.0;
	//double pid_gyro_psi_kd_local=0.0;
	
	int pid_trigger;
	int controllerCounter=0;
	
	// Activate random number generator seed
	srand((unsigned int)time(NULL));
	
	/// Setup timer variables for real time
	struct timespec t,t_start,t_stop;
	double tsTrue=tsController,tsAverageAccum=0,tsAverage;
	int tsAverageCounter=0;
	//int mpcMissedDeadlines=0;

	/// Lock memory
	if(mlockall(MCL_CURRENT) == -1){
		perror("mlockall failed in threadSensorFusion");
	}
	
	/// Start after 1 second
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	
	 //Loop forever at specific sampling rate
	while(1){
		/// Time it and wait until next shot
		//clock_gettime(CLOCK_MONOTONIC ,&t_start); // start elapsed time clock
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); // sleep for necessary time to reach desired sampling time
		
		// Get measurements, disturbance and inertias (xhat)
		pthread_mutex_lock(&mutexSensorData);
			memcpy(measBuffer, measurements, sizeof(measurements));
			memcpy(distBuffer, disturbances, sizeof(disturbances));
			memcpy(distBufferTau, disturbances_tau, sizeof(disturbances_tau));
			//memcpy(inertBuffer, inertias, sizeof(inertias));
			sensorReady=sensorInitReady;
		pthread_mutex_unlock(&mutexSensorData);
		
		// Get references, positions (other agents) and tuning values(keyboard for now)
		pthread_mutex_lock(&mutexConstraintsData);
			memcpy(refBuffer, references, sizeof(references));
			triggerFly=(int)keyboardData[3];
			triggerMpcPos=(int)keyboardData[13];
			pwmPrint=(int)keyboardData[4];
			timerPrint=(int)keyboardData[5];
			keyRampRef=(int)keyboardData[10];
			mpcAtt_ff=(int)keyboardData[14];
			mpcPos_ff=(int)keyboardData[14];
			pid_trigger=(int)keyboardData[16];
			memcpy(tuningMpcBuffer, tuningMpcData, sizeof(tuningMpcData));
			memcpy(tuningMpcQfBuffer, tuningMpcQfData, sizeof(tuningMpcQfData));
			memcpy(tuningMpcBufferControl, tuningMpcDataControl, sizeof(tuningMpcDataControl));
			memcpy(tuningPidBuffer, tuningPidData, sizeof(tuningPidData));
			memcpy(manualThrustBuffer, manualThrustData, sizeof(manualThrustData));
			memcpy(positionsBuffer, positionsData, sizeof(positionsData));
			memcpy(positions_timeoutBuffer, positions_timeoutData, sizeof(positions_timeoutData));
		pthread_mutex_unlock(&mutexConstraintsData);
			
		//printf("triggerFly: %i pwmPrint: %i sensorReady: %i\n", triggerFly, pwmPrint, sensorInitReady);
			
		/// Time it and print true sampling rate
		clock_gettime(CLOCK_MONOTONIC, &t_stop); /// stop elapsed time clock
		tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
		clock_gettime(CLOCK_MONOTONIC ,&t_start); /// start elapsed time clock
		
		/// Get average sampling time
		if(tsAverageCounter<50){
			tsAverageAccum+=tsTrue;
			tsAverageCounter++;
		}
		else{
			tsAverageAccum/=50;
			tsAverage=tsAverageAccum;
			if(timerPrint){
				printf("MPC: tsAverage %lf tsTrue %lf\n", tsAverage, tsTrue);
			}
			tsAverageCounter=0;
			tsAverageAccum=0;
		}
		
		/// Assign true sampling time to mpc models
		PosTsSec = tsTrue;
		AttTsSec = tsTrue;
		AltTsSec = tsTrue;
		
		// MPC position toggle
		if(!triggerMpcPos){
			measBuffer[0]=0; // override position x control
 			measBuffer[1]=0; // override position y control
 			//measBuffer[2]=0; // override position z control
			measBuffer[3]=0; // override position xdot control
 			measBuffer[4]=0; // override position ydot control
 			//measBuffer[5]=0; // override position zdot control
		}
		
		// Update controller parameters Q
		for (i=0;i<6;i++){
			posParams.Q[i*7]=tuningMpcBuffer[i];
			posParams.Qf[i*7]=tuningMpcBuffer[i];
			attParams.Q[i*7]=tuningMpcBuffer[i+6];
			//attParams.Qf[i*7]=tuningMpcBuffer[i+6];
		}
		for (i=0;i<2;i++){
			altParams.Q[i*3]=tuningMpcBuffer[i+12];
			altParams.Qf[i*3]=tuningMpcBuffer[i+12];
		}
		
		// Update controller parameters Qf
		attParams.Qf[0]=tuningMpcQfData[0];
		attParams.Qf[7]=tuningMpcQfData[1];
		attParams.Qf[14]=tuningMpcQfData[2];
		attParams.Qf[21]=tuningMpcQfData[3];
		attParams.Qf[28]=tuningMpcQfData[4];
		attParams.Qf[35]=tuningMpcQfData[5];
		
		attParams.Qf[1]=tuningMpcQfData[6];
		attParams.Qf[6]=tuningMpcQfData[6];
		attParams.Qf[15]=tuningMpcQfData[7];
		attParams.Qf[20]=tuningMpcQfData[7];
		attParams.Qf[29]=tuningMpcQfData[8];
		attParams.Qf[34]=tuningMpcQfData[8];

		// Update controller parameters R
		posParams.R[0]=tuningMpcBufferControl[0];
		posParams.R[3]=tuningMpcBufferControl[1];
		attParams.R[0]=tuningMpcBufferControl[2];
		attParams.R[4]=tuningMpcBufferControl[3];
		attParams.R[8]=tuningMpcBufferControl[4];
		altParams.R[0]=tuningMpcBufferControl[5];
		
		// Update controller PID gains
		//pid_gyro_theta_kp_local=tuningPidBuffer[0];
		//pid_gyro_theta_ki_local=tuningPidBuffer[1];
		mpcAtt_ki=tuningPidBuffer[2];
		//pid_angle_theta_kp_local=tuningPidBuffer[3];
		//pid_angle_theta_ki_local=tuningPidBuffer[4];
		mpcPos_ki=tuningPidBuffer[5];
		
		pid_pos_x_kp=tuningPidBuffer[0]; // x
		pid_pos_x_ki=tuningPidBuffer[1]; // x
		//mpcAtt_ki=tuningPidBuffer[2];
		pid_pos_y_kp=tuningPidBuffer[3]; // y
		pid_pos_y_ki=tuningPidBuffer[4]; // y
		//mpcPos_ki=tuningPidBuffer[5];
	
	
		//// Formation flying *********************
		// Copy over other 2 agents to buffer
		//switch (MYSELF){
			//case AGENT1:
				//memcpy(positions_agents, positionsBuffer+3, sizeof(positionsBuffer)*6/9); // agents 2 and 3
				//break;
			//case AGENT2:
				//memcpy(positions_agents, positionsBuffer, sizeof(positionsBuffer)*3/9); // agent 1
				//memcpy(positions_agents+3, positionsBuffer+6, sizeof(positionsBuffer)*3/9); // agent 3
				//break;
			//case AGENT3:
				//memcpy(positions_agents, positionsBuffer, sizeof(positionsBuffer)*6/9); // agents 1 and 2
				//break;
		//}
	
		//// Check for timeouts. If not timeouts, compute the formation reference
		//if (!positions_timeoutBuffer[0] && !positions_timeoutBuffer[1] && !positions_timeoutBuffer[2]){
			//refGen_formation(positions_agents,measBuffer,ref_formBuffer);
			
			//// save current position
			//positions_self_prev[0]=measBuffer[0]; // x
			//positions_self_prev[1]=measBuffer[1]; // y
			//positions_self_prev[2]=measBuffer[2]; // z
			
			////printf("no timeout, positions_self_prev: %f, %f, %f\n",positions_self_prev[0], positions_self_prev[1], positions_self_prev[2]);
		//}
		//// If any timeout present, deactivate formation flying and "stop" drone by setting final reference to previous position.
		//else{
			//// deactivate formation flying by setting error=0
			ref_formBuffer[0]=measBuffer[0];
			ref_formBuffer[1]=measBuffer[1];
			
			////printf("timeout, ref_formBuffer: %f, %f\n", ref_formBuffer[0], ref_formBuffer[1]);
			
			//// setting final reference to previous position
			////refBuffer[0]=positions_self_prev[0];
			////refBuffer[1]=positions_self_prev[1];
			////refBuffer[2]=positions_self_prev[2];
		//}
		
	
		// To ramp the references in x, y and z
		if ( keyRampRef ) {
			memcpy(point_1, measBuffer, sizeof(measBuffer)*3/12);
			memcpy(point_2, refBuffer, sizeof(refBuffer)*3/12);
			
			distance = sqrt( pow(point_2[0]-point_1[0],2) + pow(point_2[1]-point_1[1],2) + pow(point_2[2]-point_1[2],2) );
			
			if ( distance >= step_size) {
				alpha = step_size/distance;
				p3[0] = point_1[0] + ((point_2[0]-point_1[0])*alpha);
				p3[1] = point_1[1] + ((point_2[1]-point_1[1])*alpha);
				p3[2] = point_1[2] + ((point_2[2]-point_1[2])*alpha);

				refBuffer[0] = p3[0];
				refBuffer[1] = p3[1];
				refBuffer[2] = p3[2];
			}
		}
		
		// Only run controller if EKF (sensor.c) is actually ready and finished calibrated
		if(sensorReady){	
			// Check keyboard fly trigger is true	
			if(triggerFly){
				//printf("references-> ");
				//printmat(refBuffer, 1, 12);
				
				//printf("Ref pos: %1.2f %1.2f %1.2f\n", refBuffer[0], refBuffer[1], refBuffer[2]); 	
				//printf("(xyz) %f %f %f\n", measBuffer[0], measBuffer[1], measBuffer[2]);
				
				//printf("mpcAtt_ki % 1.4f mpcPos_ki % 1.4f\n", mpcAtt_ki, mpcPos_ki);
				
				// Run controllers 
				
				controllerPos( &posParams, &posInputs, posX_all, posU_all, measBuffer, refBuffer, ref_formBuffer, mpcPos_integral_error, mpcPos_ff, mpcPos_ki, tsTrue);
				
				//theta_dist = controllerPID((measBuffer[0]-refBuffer[0]),pid_pos_x_integral_error,pid_pos_x_prev_error,pid_pos_x_kp,pid_pos_x_ki,0, tsTrue);
				//phi_dist = controllerPID((measBuffer[1]-refBuffer[1]),pid_pos_y_integral_error,pid_pos_y_prev_error,pid_pos_y_kp,pid_pos_y_ki,0, tsTrue);
				//phi_dist *= -1;
				
				
				
				
				controllerAtt( &attParams, &attInputs, attX_all, attU_all, measBuffer, refBuffer, mpcAtt_integral_error, mpcAtt_ff, mpcAtt_ki, tsTrue);
				//tau_x=0; tau_y=0; tau_z=0;
				 if (manualThrustBuffer[0] >= 0) {
					thrust = manualThrustBuffer[0];
					//printf("thrust manual = %f\n", thrust);
				 }
				 else {
					controllerAlt( &altParams, &altInputs, altX_all, altU_all, attU_all, measBuffer, refBuffer, distBuffer );
					//printf("thrust mpc = %f\n", thrust);
				 }

				 if (pid_trigger) {
					 ////printf("PID\n");
					 //// angle controller
					 //pid_angle_u = controllerPID((measBuffer[6]-phi_dist),pid_angle_error_integral,pid_angle_error_prev,pid_angle_kp_local,pid_angle_ki_local,pid_angle_kd_local, tsTrue);
					 //// gyro controller
					 //tau_x = controllerPID((measBuffer[9]-pid_angle_u),pid_gyro_error_integral,pid_gyro_error_prev,pid_gyro_kp_local,pid_gyro_ki_local,pid_gyro_kd_local, tsTrue);
					 
					 //// angle controller
					 //pid_angle_u = controllerPID((measBuffer[7] - theta_dist),pid_angle_theta_error_integral,pid_angle_theta_error_prev,pid_angle_theta_kp_local,pid_angle_theta_ki_local,pid_angle_theta_kd_local, tsTrue);
					 //pid_angle_u *= -1;
					 ////pid_angle_u = 0;
					 //// gyro controller
					 //tau_y = controllerPID((measBuffer[10] - pid_angle_u),pid_gyro_theta_error_integral,pid_gyro_theta_error_prev,pid_gyro_theta_kp_local,pid_gyro_theta_ki_local,pid_gyro_theta_kd_local, tsTrue);
					 ////tau_y = pid_angle_u;
					 //tau_y *= -1;
					 
					 //tau_z = controllerPID((measBuffer[11] - refBuffer[11]),pid_gyro_psi_error_integral,pid_gyro_psi_error_prev,pid_gyro_psi_kp_local,pid_gyro_psi_ki_local,pid_gyro_psi_kd_local, tsTrue);
					 //tau_z *= -1;
					 
					 ////tau_x=0;
					 ////tau_y=0;
					 ////tau_z=0;
					
					 //if (tau_x > .1) {tau_x = .1;}
					 //else if (tau_x < -.1) {tau_x = -.1;}	
					 //if (tau_y > .1) {tau_y = .1;}
					 //else if (tau_y < -.1) {tau_y = -.1;}	
					 //if (tau_z > .1) {tau_z = .1;}
					 //else if (tau_z < -.1) {tau_z = -.1;}	 				
				 }
				 
				 //// Inertia identification using sin wave with white noise
				 //tsTrue_accumulated+=tsTrue;
				 //tau_x=amp*sin(2*PI*tsTrue_accumulated/L)+(-a)+2*((double)rand()/(double)(RAND_MAX)) * a;
				 ////tau_y=amp*sin(2*PI*tsTrue_accumulated/L);
				 ////tau_y=(-a)+2*((double)rand()/(double)(RAND_MAX)) * a;
				 
				if (thrust<0){
					//printf("thrust less than zero = %f\n", thrust);
					thrust=0;
				}
				
				if ( thrust <= 2 ) {
					tau_x=0; tau_y=0; tau_z=0;
					if ( controllerCounter % 40 == 0 ) {
						//printf("thrust less than %f\n", thrust);
					}
				}
	
				//tau_x = 0.000;
				//tau_y = 0.000;
				//tau_z = 0.000;
				
				//// PWM from tau and thrust using inverse of M matrix using our own frames!
				PWM[0] = sqrt( ( -2*mdl_param.b*tau_x + thrust*mdl_param.L*mdl_param.b + mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
				PWM[1] = sqrt( ( -2*mdl_param.b*tau_y + thrust*mdl_param.L*mdl_param.b - mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
				PWM[2] = sqrt( (  2*mdl_param.b*tau_x + thrust*mdl_param.L*mdl_param.b + mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
				PWM[3] = sqrt( (  2*mdl_param.b*tau_y + thrust*mdl_param.L*mdl_param.b - mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );

				controllerCounter++;
			}

			// If false, force PWM outputs to zero.
			else{
				if(sensorReady){
					//printf("MPC not running: key start = %i\n", triggerFly);
				}
				
				// Update initial MPC conditions Xall state such that they are ready for when triggerFly is true again
				for ( i = 0; i < posParams.T-1; i++ ) {
					posInputs.X0_all[i*posParams.n+0] = measBuffer[0];	// x
					posInputs.X0_all[i*posParams.n+1] = measBuffer[3];	// xdot
					posInputs.X0_all[i*posParams.n+2] = measBuffer[1];	// y
					posInputs.X0_all[i*posParams.n+3] = measBuffer[4];	// ydot
					posInputs.X0_all[i*posParams.n+4] = measBuffer[0];	// x_formation
					posInputs.X0_all[i*posParams.n+5] = measBuffer[1];	// y_formation
				}
				
				for ( i = 0; i < altParams.T-1; i++ ) {
					altInputs.X0_all[i*altParams.n+0] = measBuffer[2];	// z
					altInputs.X0_all[i*altParams.n+1] = measBuffer[5];	// zdot
				}
				
				for ( i = 0; i < attParams.T-1; i++ ) {
					attInputs.X0_all[i*attParams.n+0] = measBuffer[6];	//phi
					attInputs.X0_all[i*attParams.n+1] = measBuffer[9];	//phidot
					attInputs.X0_all[i*attParams.n+2] = measBuffer[7];	//theta 
					attInputs.X0_all[i*attParams.n+3] = measBuffer[10]; //thetadot
					attInputs.X0_all[i*attParams.n+4] = measBuffer[8];	//psi
					attInputs.X0_all[i*attParams.n+5] = measBuffer[11];	//psidot
				}
								
				memcpy(PWM, PWM0, sizeof(PWM));
				mpcAtt_integral_error[0]=0;
				mpcAtt_integral_error[1]=0;
				mpcPos_integral_error[0]=0;
				mpcPos_integral_error[1]=0;
				
				pid_pos_x_integral_error[0]=0;
				pid_pos_y_integral_error[0]=0;
				pid_pos_x_prev_error[0]=0;
				pid_pos_y_prev_error[0]=0;
				
				//tau_x = 0;
				//pid_gyro_error_integral[0]=0;
				//pid_gyro_error_prev[0]=0;
				//pid_angle_error_integral[0]=0.0;
				//pid_angle_error_integral[1]=0.0;
				//pid_angle_error_prev[0]=0.0;		
			}
		
			// Print PWM signal sent to motors
			if(pwmPrint){
				printf("PWM: %3.4f %3.4f %3.4f %3.4f (u_mpcPos) % 2.5f % 2.5f (u_mpcPos_comp) % 2.5f % 2.5f (u_mpcAtt) % 2.4f % 2.4f % 2.4f (err_pos) % 2.4f % 2.4f\n", PWM[0], PWM[1], PWM[2], PWM[3], theta_dist*180/PI, phi_dist*180/PI, theta_ref_comp*180/PI, phi_ref_comp*180/PI, tau_x, tau_y, tau_z, measBuffer[0]-refBuffer[0], measBuffer[1]-refBuffer[1]);
			}
			
			// Copy data over to common controller buffer before sending it to sensor.c
			controllerBuffer[0]=PWM[0];
			controllerBuffer[1]=PWM[1];
			controllerBuffer[2]=PWM[2];
			controllerBuffer[3]=PWM[3];
			controllerBuffer[4]=thrust;
			controllerBuffer[5]=tau_x;
			controllerBuffer[6]=tau_y;
			controllerBuffer[7]=tau_z;
			controllerBuffer[8]=altU_all[0]; // G
			controllerBuffer[9]=theta_dist;
			controllerBuffer[10]=phi_dist;
			controllerBuffer[11]=theta_ref_comp;
			controllerBuffer[12]=phi_ref_comp;
			controllerBuffer[13]=attU_all[0]; // taux
			controllerBuffer[14]=attU_all[1]; // tauy
			controllerBuffer[15]=tau_x; // taux with integrator
			controllerBuffer[16]=tau_y; // tauy with integrator
			controllerBuffer[17]=tau_z;
			controllerBuffer[18]=tsTrue;
			
			// Set motor PWM signals by writing to the sensor.c process which applies the changes over I2C.
			if (write(ptrPipe1->parent[1], controllerBuffer, sizeof(controllerBuffer)) != sizeof(controllerBuffer)) printf("write error in controller to sensor\n");
			//else printf("Controller ID: %d, Sent controllerBuffer: %3.5f to Communication\n", (int)getpid(), controllerBuffer[0]);
		}
		// Set update of constraints and controller results by writing to the communication.c process which applies the changes over UDP to other agents
		//if (write(ptrPipe2->parent[1], posX_all, sizeof(posX_all)) != sizeof(posX_all)) printf("write error in controller to communication\n");
		//else printf("Controller ID: %d, Sent: %f to Communication\n", (int)getpid(), controllerDataBuffer[0]);

		/// Calculate next shot
		t.tv_nsec += tsController;
		while (t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}	
		
		///// Print true sampling rate
		//clock_gettime(CLOCK_MONOTONIC, &t_stop);
		//tsTrue=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
		////printf("Sampling time [s] mpc: %lf\n",tsTrue);
		//if(tsTrue-tsController/NSEC_PER_SEC>0.0001){
			////printf("MPC did not meet deadline by %lf [s]. Nr %i\n",tsTrue-tsController/NSEC_PER_SEC,++mpcMissedDeadlines);
		//}
		
	}
	
	return NULL;
}

/******************************************************************/
/*************************   FUNCTIONS   **************************/
/******************************************************************/

/* Generates x and y refs for formation states for pos controller */
void refGen_formation( double *pos_agents, double *pos_self, double *ref_formation ){
	double points[4]={0};	// intersection points of the agents, first ind is the point number
	double r = 2;	// the radius around other agents for the formation
	double x1, x2, y1, y2, d1, d2;
	
	//% re-arrange variables
	x1=pos_agents[0];
	y1=pos_agents[1];
	x2=pos_agents[3];
	y2=pos_agents[4];


	// If circles DO intersect
	// Reference points becomes the closest intersection points
	if ( sqrt(pow(x1 - x2, 2) + pow((y1 - y2), 2) ) < 2*r ) {
		//% generate circle intersections gfor desired distance between agents
		points[1]=(x1*sqrt((- pow(r,2) + 2*r*r - pow(r,2) + pow(x1,2) - 2*x1*x2 + pow(x2,2) + pow(y1,2) - 2*y1*y2 + pow(y2,2))*(pow(r,2) + 2*r*r + pow(r,2) - pow(x1,2) + 2*x1*x2 - pow(x2,2) - pow(y1,2) + 2*y1*y2 - pow(y2,2))) - x2*sqrt((- pow(r,2) + 2*r*r - pow(r,2) + pow(x1,2) - 2*x1*x2 + pow(x2,2) + pow(y1,2) - 2*y1*y2 + pow(y2,2))*(pow(r,2) + 2*r*r + pow(r,2) - pow(x1,2) + 2*x1*x2 - pow(x2,2) - pow(y1,2) + 2*y1*y2 - pow(y2,2))) - pow(r,2)*y1 + pow(r,2)*y2 + pow(r,2)*y1 - pow(r,2)*y2 + pow(x1,2)*y1 + pow(x1,2)*y2 + pow(x2,2)*y1 + pow(x2,2)*y2 - y1*pow(y2,2) - pow(y1,2)*y2 + pow(y1,3) + pow(y1,3) - 2*x1*x2*y1 - 2*x1*x2*y2)/(2*(pow(x1,2) - 2*x1*x2 + pow(x2,2) + pow(y1,2) - 2*y1*y2 + pow(y2,2)));
		points[0]=1/(2*x1-2*x2)*(pow(r,2)-pow(r,2)-(pow(x2,2)-pow(x1,2))-points[1]*(2*y1-2*y2)-(pow(y2,2)-pow(y1,2)));
				
		points[3]=(x2*sqrt((- pow(r,2) + 2*r*r - pow(r,2) + pow(x1,2) - 2*x1*x2 + pow(x2,2) + pow(y1,2) - 2*y1*y2 + pow(y2,2))*(pow(r,2) + 2*r*r + pow(r,2) - pow(x1,2) + 2*x1*x2 - pow(x2,2) - pow(y1,2) + 2*y1*y2 - pow(y2,2))) - x1*sqrt((- pow(r,2) + 2*r*r - pow(r,2) + pow(x1,2) - 2*x1*x2 + pow(x2,2) + pow(y1,2) - 2*y1*y2 + pow(y2,2))*(pow(r,2) + 2*r*r + pow(r,2) - pow(x1,2) + 2*x1*x2 - pow(x2,2) - pow(y1,2) + 2*y1*y2 - pow(y2,2))) - pow(r,2)*y1 + pow(r,2)*y2 + pow(r,2)*y1 - pow(r,2)*y2 + pow(x1,2)*y1 + pow(x1,2)*y2 + pow(x2,2)*y1 + pow(x2,2)*y2 - y1*pow(y2,2) - pow(y1,2)*y2 + pow(y1,3) + pow(y1,3) - 2*x1*x2*y1 - 2*x1*x2*y2)/(2*(pow(x1,2) - 2*x1*x2 + pow(x2,2) + pow(y1,2) - 2*y1*y2 + pow(y2,2)));
		points[2]=1/(2*x1-2*x2)*(pow(r,2)-pow(r,2)-(pow(x2,2)-pow(x1,2))-points[3]*(2*y1-2*y2)-(pow(y2,2)-pow(y1,2)));
	}
	
	// generate distance between self position and two new reference options
	 d1=sqrt(pow(points[0] - pos_self[0], 2) + pow((points[1] - pos_self[1]), 2) );
	 d2=sqrt(pow(points[2] - pos_self[0], 2) + pow((points[3] - pos_self[1]), 2) );
	
	// select closest intersection point as new formation reference
	if(d1<d2){
		ref_formation[0]=points[0];
		ref_formation[1]=points[1];
	}
	else{
		ref_formation[0]=points[2];
		ref_formation[1]=points[3];
	}
}

/* PID controller - Gains: kp = proportional, ki = integral, kd = derivative*/
static double controllerPID(double error_current, double *error_integral, double *error_prev, double kp, double ki, double kd, double ts){
	// Integral error
	double u;
	error_integral[0] += (error_current*ts);
	
	//printf("error_current> %f - kp> %f - ki> %f - kd> %f\n", error_current, kp, ki, kd);

	// Calculate control action
	u = (kp*error_current) + (ki*error_integral[0]) + (kd*(error_current-error_prev[0])/ts);
	
	// Flip controller signal (torque directions)
	u*=-1;
	
	// Save current error for next iteration
	error_prev[0]=error_current;
	
	return u;
}

/* Saturation calculation for altitude MPC control input constraints [umin/umax] */
static void getAltitudeInputConstraints( double *dist , struct AltParams *altParams, double *attU_all ) {
	// Saturation function to compensate for disturbance z-axis + necessary
	// torques in order to keep stable. As a result, accuracy of altitude
	// control may suffer.
	
	// Local variables
	double G[8];
	double umin, umax;
	
	//// Create PWM signal from calculated thrust and torques
	//PWM[0] = sqrt( (  2*mdl_param.b*tau_x + thrust*mdl_param.L*mdl_param.b + mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//PWM[1] = sqrt( (  2*mdl_param.b*tau_y + thrust*mdl_param.L*mdl_param.b - mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//PWM[2] = sqrt( ( -2*mdl_param.b*tau_x + thrust*mdl_param.L*mdl_param.b + mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//PWM[3] = sqrt( ( -2*mdl_param.b*tau_y + thrust*mdl_param.L*mdl_param.b - mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//// PWM from tau and thrust using inverse of M matrix using our own frames!
	//PWM[0] = sqrt( ( -2*mdl_param.b*tau_x + thrust*mdl_param.L*mdl_param.b + mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//PWM[1] = sqrt( ( -2*mdl_param.b*tau_y + thrust*mdl_param.L*mdl_param.b - mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//PWM[2] = sqrt( (  2*mdl_param.b*tau_x + thrust*mdl_param.L*mdl_param.b + mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	//PWM[3] = sqrt( (  2*mdl_param.b*tau_y + thrust*mdl_param.L*mdl_param.b - mdl_param.L*mdl_param.k*tau_z )/Lbc_mk4 );
	
	// equations to find minimum G representing 0% PWM
	//G[0]=-(2*mdl_param.b*attU_all[0] + mdl_param.L*mdl_param.k*attU_all[2] - mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	//G[1]=(mdl_param.L*mdl_param.k*attU_all[2] - 2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	//G[2]=(2*mdl_param.b*attU_all[0] - mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	//G[3]=(2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	G[0]=( 2*mdl_param.b*attU_all[0] - mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	G[1]=( 2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	G[2]=(-2*mdl_param.b*attU_all[0] - mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);
	G[3]=(-2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(mdl_param.L*mdl_param.b*mdl_param.mass);

	// equations to find maximum G representing 100% PWM
	//G[4]=-(4*mdl_param.c_m*mdl_param.k*((2*mdl_param.b*attU_all[0] + mdl_param.L*mdl_param.k*attU_all[2] - mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) - 10000))/mdl_param.mass;
	//G[5]=(4*mdl_param.c_m*mdl_param.k*((mdl_param.L*mdl_param.k*attU_all[2] - 2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) + 10000))/mdl_param.mass;
	//G[6]=(4*mdl_param.c_m*mdl_param.k*((2*mdl_param.b*attU_all[0] - mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) + 10000))/mdl_param.mass;
	//G[7]=(4*mdl_param.c_m*mdl_param.k*((2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) + 10000))/mdl_param.mass;
	G[4]=-(4*mdl_param.c_m*mdl_param.k*((-2*mdl_param.b*attU_all[0] + mdl_param.L*mdl_param.k*attU_all[2] - mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) - 10000))/mdl_param.mass;
	G[5]=(4*mdl_param.c_m*mdl_param.k*((mdl_param.L*mdl_param.k*attU_all[2] + 2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) + 10000))/mdl_param.mass;
	G[6]=(4*mdl_param.c_m*mdl_param.k*((-2*mdl_param.b*attU_all[0] - mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) + 10000))/mdl_param.mass;
	G[7]=(4*mdl_param.c_m*mdl_param.k*((-2*mdl_param.b*attU_all[1] + mdl_param.L*mdl_param.k*attU_all[2] + mdl_param.L*mdl_param.b*dist[2]*mdl_param.mass)/(4*mdl_param.L*mdl_param.b*mdl_param.c_m*mdl_param.k) + 10000))/mdl_param.mass;

	// max of G[0:3] becomes altitude control umin
	umin=G[0];
	for (int i=1;i<4;i++){
		if(G[i]>umin){
			umin=G[i];
		}
	}

	// min of G[4:7] becomes altitude control umax
	umax=G[4];
	for (int i=5;i<7;i++){
		if(G[i]<umax){
			umax=G[i];
		}
	}
	
	// Update umin and umax of altitude parameters
	altParams->umin[0]=umin;
	altParams->umax[0]=umax;
}

/* The same as threadControllerPos but here as a function and not a thread with sample rate */
static void controllerPos( struct PosParams *posParams, struct PosInputs *posInputs, double *posX_all, double *posU_all, double *meas, double *ref, double *ref_form, double *error_integral, int mpcPos_ff, double ki, double ts ) {
	int i;
	// Warm start before running the controller - MEMCPY IS NOT NEEDED IN SIMULINK
	memcpy(&posInputs->X0_all[0], &posX_all[posParams->n], sizeof(double)*posParams->n*posParams->T-posParams->n); 	
	memcpy(&posInputs->U0_all[0], &posU_all[posParams->m], sizeof(double)*posParams->m*posParams->T-posParams->m); 
	for ( i = posParams->n*posParams->T-posParams->n; i < posParams->n*posParams->T; i++ ) {
		posInputs->X0_all[i] = 0;
		}
	for ( i = posParams->m*posParams->T-posParams->m; i < posParams->m*posParams->T; i++ ) {
		posInputs->U0_all[i] = 0;
		}
		
	// Integrator for position error x and y
	if(mpcPos_ff){
		error_integral[0] += ki*((posInputs->x0[0])*ts);
		error_integral[1] += ki*((posInputs->x0[2])*ts);
		
		// Integrator saturation at equivalent of -2 and +2 degrees
		 if (error_integral[0] > 1*PI/180) {error_integral[0] = 1*PI/180;}
		 else if (error_integral[0] < -1*PI/180) {error_integral[0] = -1*PI/180;}	
		 if (error_integral[1] > 1*PI/180) {error_integral[1] = 1*PI/180;}
		 else if (error_integral[1] < -1*PI/180) {error_integral[1] = -1*PI/180;}		
	}
	else{
		error_integral[0]=0;
		error_integral[1]=0;
	}
		
	// Update controller input contraints [umin/umax] to compensate for disturbances
	//posParams->umax[0]=6*PI/180-(dist[0]*mdl_param.mass)/(dist[2]);
	//posParams->umin[0]=-6*PI/180-(dist[0]*mdl_param.mass)/(dist[2]);
	//posParams->umax[1]=6*PI/180-(dist[1]*mdl_param.mass)/(dist[2]);
	//posParams->umin[1]=-6*PI/180-(dist[1]*mdl_param.mass)/(dist[2]);
	//posParams->umax[0]=6*PI/180+error_integral[0];
	//posParams->umin[0]=-6*PI/180+error_integral[0];
	//posParams->umax[1]=6*PI/180-error_integral[1];
	//posParams->umin[1]=-6*PI/180-error_integral[1];
	
	posParams->umax[0]=15*PI/180+error_integral[0];
	posParams->umin[0]=-15*PI/180+error_integral[0];
	posParams->umax[1]=15*PI/180-error_integral[1];
	posParams->umin[1]=-15*PI/180-error_integral[1];
	
	// Get measurements and references from global data
	posInputs->x0[0] = meas[0] - ref[0];		// x
	posInputs->x0[1] = meas[3] - ref[3];		// xdot
	posInputs->x0[2] = meas[1] - ref[1];		// y
	posInputs->x0[3] = meas[4] - ref[4];		// ydot
	posInputs->x0[4] = meas[0] - ref_form[0];		// x_formation
	posInputs->x0[5] = meas[1] - ref_form[1];		// y_formation

	posFmpc(posParams, posInputs, posX_all, posU_all);
		
	// Get measurements and references from global data
	//theta_dist = posU_all[0]+(dist[0]*mdl_param.mass)/dist[2];		//theta with disturbance compensation
	//phi_dist = posU_all[1]+(dist[1]*mdl_param.mass)/dist[2];			//phi with disturbance compensation
	
	theta_dist = posU_all[0];		//theta with disturbance compensation
	phi_dist = posU_all[1];			//phi with disturbance compensation
	
	// Feed forward disturbance compensation
	if(mpcPos_ff){
		theta_dist -= error_integral[0]; // deducting error_integral since the model of x control has g in B matrix.
		phi_dist += error_integral[1]; // adding error_integral since the model of y control has -g in B matrix.
	}
	
	//printf("mpcPos (x,umax/umin/int) % 1.4f % 1.4f % 1.4f (y,umax/umin/int) % 1.4f % 1.4f % 1.4f (theta/phi) % 1.4f % 1.4f (theta/phi_ff) % 1.4f % 1.4f\n", posParams->umax[0]*180/PI,posParams->umin[0]*180/PI, error_integral[0]*180/PI, posParams->umax[1]*180/PI, posParams->umin[1]*180/PI, error_integral[1]*180/PI, posU_all[0]*180/PI, posU_all[1]*180/PI, theta_dist*180/PI, phi_dist*180/PI);

	
	//dx_comp=(dist[0]*mdl_param.mass)/dist[2];
	//dy_comp=(dist[1]*mdl_param.mass)/dist[2];
}
	
/* The same as threadControllerAtt but here as a function and not a thread with sample rate */
static void controllerAtt( struct AttParams *attParams, struct AttInputs *attInputs, double *attX_all, double *attU_all, double *meas, double *ref, double* error_integral, int mpcAtt_ff, double ki, double ts) {
	int i;
	
	// Warm start before running the controller - MEMCPY IS NOT NEEDED IN SIMULINK
	memcpy(&attInputs->X0_all[0], &attX_all[attParams->n], sizeof(double)*attParams->n*attParams->T-attParams->n); 	
	memcpy(&attInputs->U0_all[0], &attU_all[attParams->m], sizeof(double)*attParams->m*attParams->T-attParams->m); 	
	for ( i = attParams->n*attParams->T-attParams->n; i < attParams->n*attParams->T; i++ ) {
		attInputs->X0_all[i] = 0;
		}
	for ( i = attParams->m*attParams->T-attParams->m; i < attParams->m*attParams->T; i++ ) {
		attInputs->U0_all[i] = 0;
		}
	
	// Update controller input contraints [umin/umax] to compensate for disturbances in torque
	if(mpcAtt_ff){
		//attParams->umax[0]=10+error_integral[0];
		//attParams->umin[0]=-10+error_integral[0];
		// attParams->umax[0]=0.1+dist[0];
		// attParams->umin[0]=-0.1+dist[0];
		// attParams->umax[1]=0.1+dist[1];
		// attParams->umin[1]=-0.1+dist[1];
		// attParams->umax[2]=0.1+dist[2];
		// attParams->umin[2]=-0.1+dist[2];
	}
	
	//printf("umaxumin_0(% 1.4f % 1.4f) umaxumin_1(% 1.4f % 1.4f) umaxumin_2(% 1.4f % 1.4f)\n", attParams->umax[0]-dist[0], attParams->umin[0]-dist[0], attParams->umax[1]-dist[1], attParams->umin[1]-dist[1], attParams->umax[2]-dist[2], attParams->umin[2]-dist[2]);
	
	// Rotate angle references such that yaw angle is taken in to account
	theta_ref_comp=cos(meas[8])*theta_dist - phi_dist*sin(meas[8]);
	phi_ref_comp=cos(meas[8])*phi_dist + sin(meas[8])*theta_dist;
	
	//theta_ref_comp+=dx_comp;
	//phi_ref_comp+=dy_comp;
	
	// Get measurements and references from global data
	//attInputs->x0[0] = meas[6] - phi_dist;			//phi - coming from the pos controller
	attInputs->x0[0] = meas[6] - phi_ref_comp;			//phi - coming from the pos controller
	attInputs->x0[1] = meas[9] - ref[9];		//phidots
	//attInputs->x0[2] = +1*(meas[7] - theta_dist);			//theta - coming from the pos controller
	attInputs->x0[2] = +1*(meas[7] - theta_ref_comp);			//theta - coming from the pos controller
	attInputs->x0[3] = +1*(meas[10] - ref[10]);	//thetadot
	attInputs->x0[4] = meas[8] - ref[8];		//psi
	attInputs->x0[5] = meas[11] - ref[11];	//psidot
	//// Rotate 90 around its Z LH
	//attInputs->x0[0] = -1*(meas[7] - theta_dist);		//phi 
	//attInputs->x0[1] = -1*(meas[10] - ref[10]);			//phidots
	//attInputs->x0[2] = meas[6] - phi_dist;				//theta 
	//attInputs->x0[3] = meas[9] - ref[9];				//thetadot
	//attInputs->x0[4] = meas[8] - ref[8];				//psi
	//attInputs->x0[5] = meas[11] - ref[11];				//psidot
	
	// Integrator action for angle state to get offset free control
	 if(mpcAtt_ff){
		error_integral[0] += ki*((attInputs->x0[0])*ts);
		error_integral[1] += ki*((attInputs->x0[2])*ts);
	 }
	 else{
		 error_integral[0]=0;
		 error_integral[1]=0;
	 }
	
	//printf("(attInputs) % 2.4f % 2.4f % 2.4f % 2.4f % 2.4f % 2.4f\n", attInputs->x0[0], attInputs->x0[1], attInputs->x0[2], attInputs->x0[3], attInputs->x0[4], attInputs->x0[5]);
	
	attFmpc(attParams, attInputs, attX_all, attU_all);
	
	//printf("attU_all{% 1.8f % 1.8f % 1.8f}\n", attU_all[0], attU_all[1], attU_all[2]);
	
	tau_x = attU_all[0];		// phi
	tau_y = attU_all[1];		// theta
	tau_z = attU_all[2];		// psi
	tau_z *= -1;

	// Feed forward disturbance compensation
	if(mpcAtt_ff){
		tau_x -= error_integral[0];
		tau_y -= error_integral[1];
	}

	for ( i = 0; i < attParams->m*attParams->T; i++ ) {
		if(isnan(attU_all[i])!=0){
			printf("MPC attU[%i]=nan\n",i);
		}
	}
	
	//printf("(taux) % 2.4f (tauy) % 2.4f (tauz) % 2.4f\n", tau_x, tau_y, tau_z);
}
	
/* The same as threadControllerAlt but here as a function and not a thread with sample rate */
static void controllerAlt( struct AltParams *altParams, struct AltInputs *altInputs, double *altX_all, double *altU_all, double *attU_all, double *meas, double *ref, double *dist) {
	int i;

	// Warm start before running the controller - MEMCPY IS NOT NEEDED IN SIMULINK
	memcpy(&altInputs->X0_all[0], &altX_all[altParams->n], sizeof(double)*altParams->n*altParams->T-altParams->n); 	
	memcpy(&altInputs->U0_all[0], &altU_all[altParams->m], sizeof(double)*altParams->m*altParams->T-altParams->m); 	
	for ( i = altParams->n*altParams->T-altParams->n; i < altParams->n*altParams->T; i++ ) {
		altInputs->X0_all[i] = 0;
		}
	for ( i = altParams->m*altParams->T-altParams->m; i < altParams->m*altParams->T; i++ ) {
		altInputs->U0_all[i] = 0;
		}

	// Set input contraints according to attitude torques and disturbances in z axis (gravity ++)
	// This will result in a PWM control saturation of 0-100%
	getAltitudeInputConstraints( dist , altParams, attU_all );	
	
	// Get measurements and references from global data
	//printf("MPC_alt error: %f (meas) %f (ref) %f\n", meas[2] - ref[2], meas[2], ref[2]);
	altInputs->x0[0] = meas[2] - ref[2];
	altInputs->x0[1] = meas[5] - ref[5];

	altFmpc(altParams, altInputs, altX_all, altU_all);
	
	thrust = (altU_all[0]-dist[2])*mdl_param.mass;	// gravity compensation
	//thrust = altU_all[0]*mdl_param.mass;	// gravity compensation
}

/* interact with fast MPC POS */
static void posFmpc( struct PosParams *posParams, struct PosInputs *posInputs, double *posX_all, double *posU_all ) {

	/* problem setup */
    int i, j, m, n, nz, T, niters;
    double kappa;
    double *dptr, *dptr1, *dptr2;
    //const double *Cdptr;
    double *At, *Bt, *xmax, *xmin, *x;
    double *A, *B, *Q, *R, *Qf, *umax, *umin;
    double *zmax, *zmin, *zmaxp, *zminp, *z, *eyen, *eyem, *x0;
	//double *X_all, *U_all;
    double *X0_all, *U0_all;
    //int agent_mode = 0;
    //double telapsed;
    //clock_t t1, t2;
    
	/* Parameters */
	n = posParams->n;
	m = posParams->m;
	T = posParams->T;
	kappa = posParams->kappa;
	niters = posParams->niters;
	A = posParams->A;
	B = posParams->B;
	Q = posParams->Q;
	R = posParams->R;
	Qf = posParams->Qf;
	umax = posParams->umax;
	umin = posParams->umin;
	nz = T * (n + m);
	
	/* Inputs */
	X0_all = posInputs->X0_all;
	U0_all = posInputs->U0_all;
	x0 = posInputs->x0;
	xmax = posInputs->xmax;
	xmin = posInputs->xmin;

    /* Outputs */
    //X_all = malloc(sizeof(double)*n*T);
    //U_all = malloc(sizeof(double)*m*T);
    At = malloc(sizeof(double)*n*n);
    Bt = malloc(sizeof(double)*n*m);
    eyen = malloc(sizeof(double)*n*n);
    eyem = malloc(sizeof(double)*m*m);
    z = malloc(sizeof(double)*nz);
    x = malloc(sizeof(double)*n);
    zmax = malloc(sizeof(double)*nz);
    zmin = malloc(sizeof(double)*nz);
    zmaxp = malloc(sizeof(double)*nz);
    zminp = malloc(sizeof(double)*nz);
     
    /* eyen, eyem */
    dptr = eyen;
    for (i = 0; i < n*n; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-n*n;
    for (i = 0; i < n; i++)
    {
        *dptr = 1;
        dptr = dptr+n+1;
    }
 
    dptr = eyem;
    for (i = 0; i < m*m; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-m*m;
    for (i = 0; i < m; i++)
    {
        *(dptr+i*m+i) = 1;
    }
    dptr = x; dptr1 = x0;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr = *(U0_all+i*m+j);
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr = *(X0_all+i*n+j);
            dptr++; 
        }
    }  
    /* At, Bt */
    F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,A,&n,eyen,&n,&fzero,At,&n);
    F77_CALL(dgemm)("n","t",&m,&n,&m,&fone,eyem,&m,B,&n,&fzero,Bt,&m);
 
    /* zmax, zmin */
    dptr1 = zmax;
    dptr2 = zmin;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr1 = *(umax+j);
            *dptr2 = *(umin+j);
            dptr1++; dptr2++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr1 = *(xmax+j);
            *dptr2 = *(xmin+j);
            dptr1++; dptr2++;
        }
    }  
 
    /* zmaxp, zminp */
    for (i = 0; i < nz; i++) zminp[i] = zmin[i] + 0.01*(zmax[i]-zmin[i]);
    for (i = 0; i < nz; i++) zmaxp[i] = zmax[i] - 0.01*(zmax[i]-zmin[i]);
 
    /* project z */
    for (i = 0; i < nz; i++) z[i] = z[i] > zmaxp[i] ? zmaxp[i] : z[i];
    for (i = 0; i < nz; i++) z[i] = z[i] < zminp[i] ? zminp[i] : z[i];
    
    ///* just plotting it before! */
    //printf("====  posFmpc()  ====\n");
	//printf("n = %i | m = %i | T = %i | niters = %i | kappa = %f | quiet = %i | nz = %i\n", n, m, T, niters, kappa, quiet, nz);
	//printf("A\n");
	//printmat(A, n, n);
	//printf("B\n");
	//printmat(B, n, m);
	//printf("At\n");
	//printmat(At, n, n);
	//printf("Bt\n");
	//printmat(Bt, m, n);
	//printf("eyen\n");
	//printmat(eyen, n, n);
	//printf("eyem\n");
	//printmat(eyem, m, m);
	//printf("Q\n");
	//printmat(Q, n, n);
	//printf("R\n");
	//printmat(R, m, m);
	//printf("Qf\n");
	//printmat(Qf, n, n);     
	//printf("zmax\n");
	//printmat(zmax, n+m, T);
	//printf("zmin\n");
	//printmat(zmin, n+m, T); 
	//printf("x\n");
	//printmat(x, n, 1);
	//printf("z\n");
	//printmat(z, n+m, T);
	//*/
	
    //t1 = clock();
    fmpcsolve(A,B,At,Bt,eyen,eyem,Q,R,Qf,zmax,zmin,x,z,T,n,m,nz,niters,kappa);
    //t2 = clock();
    //*telapsed = (double)(t2-t1)/(CLOCKS_PER_SEC);
    
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *(posU_all+i*m+j) = *dptr;
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *(posX_all+i*n+j) = *dptr;
            dptr++;
        }
    }
    
    /* free inputs memories */
    //free(X0_all); free(U0_all); free(x0); free(xmax); free(xmin);
    //free(A); free(B); 
    /* free outputs memories */
    //free(X_all); free(U_all);
    free(At); free(Bt); free(eyen); free(eyem);
    free(z); free(x); free(zmax); free(zmin); free(zmaxp); free(zminp);
    return;
	}

/* interact with fast MPC ATT */
static void attFmpc( struct AttParams *attParams, struct AttInputs *attInputs, double *attX_all, double *attU_all ) {

	/* problem setup */
    int i, j, m, n, nz, T, niters;
    double kappa;
    double *dptr, *dptr1, *dptr2;
    //const double *Cdptr;
    double *A, *B, *At, *Bt, *Q, *R, *Qf, *xmax, *xmin, *umax, *umin, *x;
    double *zmax, *zmin, *zmaxp, *zminp, *z, *eyen, *eyem, *x0;
	//double *X_all, *U_all
    double *X0_all, *U0_all;
    //int agent_mode = 0;
    //double telapsed;
    //clock_t t1, t2;
    
	/* Parameters */
	n = attParams->n;
	m = attParams->m;
	T = attParams->T;
	kappa = attParams->kappa;
	niters = attParams->niters;
	A = attParams->A;
	B = attParams->B;
	Q = attParams->Q;
	R = attParams->R;
	Qf = attParams->Qf;
	umax = attParams->umax;
	umin = attParams->umin;
	nz = T * (n + m);
	
	/* Inputs */
	X0_all = attInputs->X0_all;
	U0_all = attInputs->U0_all;
	x0 = attInputs->x0;
	xmax = attInputs->xmax;
	xmin = attInputs->xmin;

    /* Outputs */
    //X_all = malloc(sizeof(double)*n*T);
    //U_all = malloc(sizeof(double)*m*T);
    At = malloc(sizeof(double)*n*n);
    Bt = malloc(sizeof(double)*n*m);
    eyen = malloc(sizeof(double)*n*n);
    eyem = malloc(sizeof(double)*m*m);
    z = malloc(sizeof(double)*nz);
    x = malloc(sizeof(double)*n);
    zmax = malloc(sizeof(double)*nz);
    zmin = malloc(sizeof(double)*nz);
    zmaxp = malloc(sizeof(double)*nz);
    zminp = malloc(sizeof(double)*nz);
     
    /* eyen, eyem */
    dptr = eyen;
    for (i = 0; i < n*n; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-n*n;
    for (i = 0; i < n; i++)
    {
        *dptr = 1;
        dptr = dptr+n+1;
    }
 
    dptr = eyem;
    for (i = 0; i < m*m; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-m*m;
    for (i = 0; i < m; i++)
    {
        *(dptr+i*m+i) = 1;
    }
    dptr = x; dptr1 = x0;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr = *(U0_all+i*m+j);
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr = *(X0_all+i*n+j);
            dptr++; 
        }
    }  
    /* At, Bt */
    F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,A,&n,eyen,&n,&fzero,At,&n);
    F77_CALL(dgemm)("n","t",&m,&n,&m,&fone,eyem,&m,B,&n,&fzero,Bt,&m);
 
    /* zmax, zmin */
    dptr1 = zmax;
    dptr2 = zmin;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr1 = *(umax+j);
            *dptr2 = *(umin+j);
            dptr1++; dptr2++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr1 = *(xmax+j);
            *dptr2 = *(xmin+j);
            dptr1++; dptr2++;
        }
    }  
 
    /* zmaxp, zminp */
    for (i = 0; i < nz; i++) zminp[i] = zmin[i] + 0.01*(zmax[i]-zmin[i]);
    for (i = 0; i < nz; i++) zmaxp[i] = zmax[i] - 0.01*(zmax[i]-zmin[i]);
 
    /* project z */
    for (i = 0; i < nz; i++) z[i] = z[i] > zmaxp[i] ? zmaxp[i] : z[i];
    for (i = 0; i < nz; i++) z[i] = z[i] < zminp[i] ? zminp[i] : z[i];
    
    /* just plotting it! */
	///*
	//printf("====  attFmpc()  ====\n");
	//printf("n = %i | m = %i | T = %i | niters = %i | kappa = %f | quiet = %i | nz = %i\n", n, m, T, niters, kappa, quiet, nz);
	//printf("A\n");
	//printmat(A, n, n);
	//printf("B\n");
	//printmat(B, n, m);
	//printf("At\n");
	//printmat(At, n, n);
	//printf("Bt\n");
	//printmat(Bt, m, n);
	//printf("eyen\n");
	//printmat(eyen, n, n);
	//printf("eyem\n");
	//printmat(eyem, m, m);
	//printf("Q\n");
	//printmat(Q, n, n);
	//printf("R\n");
	//printmat(R, m, m);
	//printf("Qf\n");
	//printmat(Qf, n, n);     
	//printf("zmax\n");
	//printmat(zmax, n+m, T);
	//printf("zmin\n");
	//printmat(zmin, n+m, T); 
	//printf("x\n");
	//printmat(x, n, 1);
	//printf("z\n");
	//printmat(z, n+m, T);
	//*/
	
    //t1 = clock();
    fmpcsolve(A,B,At,Bt,eyen,eyem,Q,R,Qf,zmax,zmin,x,z,T,n,m,nz,niters,kappa);
    //t2 = clock();
    //telapsed = (double)(t2-t1)/(CLOCKS_PER_SEC);
    
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *(attU_all+i*m+j) = *dptr;
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *(attX_all+i*n+j) = *dptr;
            dptr++;
        }
    }
    
    /* free inputs memories */
    //free(X0_all); free(U0_all); free(x0); free(xmax); free(xmin);
    //free(A); free(B); 
    /* free outputs memories */
    //free(X_all); free(U_all);
    free(At); free(Bt); free(eyen); free(eyem);
    free(z); free(x); free(zmax); free(zmin); free(zmaxp); free(zminp);
    return;
}

/* interact with fast MPC ALT */
static void altFmpc( struct AltParams *altParams, struct AltInputs *altInputs, double *altX_all, double *altU_all ) {

	/* problem setup */
    int i, j, m, n, nz, T, niters;
    double kappa;
    double *dptr, *dptr1, *dptr2;
    //const double *Cdptr;
    double *A, *B, *At, *Bt, *Q, *R, *Qf, *xmax, *xmin, *umax, *umin, *x;
    double *zmax, *zmin, *zmaxp, *zminp, *z, *eyen, *eyem, *x0;
    //double *X_all, *U_all
    double *X0_all, *U0_all;
    //int agent_mode = 0;
    //double telapsed;
    //clock_t t1, t2;
    
	/* Parameters */
	n = altParams->n;
	m = altParams->m;
	T = altParams->T;
	kappa = altParams->kappa;
	niters = altParams->niters;
	A = altParams->A;
	B = altParams->B;
	Q = altParams->Q;
	R = altParams->R;
	Qf = altParams->Qf;
	umax = altParams->umax;
	umin = altParams->umin;
	nz = altParams->T * (altParams->n + altParams->m);
	
	/* Inputs */
	X0_all = altInputs->X0_all;
	U0_all = altInputs->U0_all;
	x0 = altInputs->x0;
	xmax = altInputs->xmax;
	xmin = altInputs->xmin;

    /* Outputs */
    //X_all = malloc(sizeof(double)*n*T);
    //U_all = malloc(sizeof(double)*m*T);
    At = malloc(sizeof(double)*n*n);
    Bt = malloc(sizeof(double)*n*m);
    eyen = malloc(sizeof(double)*n*n);
    eyem = malloc(sizeof(double)*m*m);
    z = malloc(sizeof(double)*nz);
    x = malloc(sizeof(double)*n);
    zmax = malloc(sizeof(double)*nz);
    zmin = malloc(sizeof(double)*nz);
    zmaxp = malloc(sizeof(double)*nz);
    zminp = malloc(sizeof(double)*nz);
     
    /* eyen, eyem */
    dptr = eyen;
    for (i = 0; i < n*n; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-n*n;
    for (i = 0; i < n; i++)
    {
        *dptr = 1;
        dptr = dptr+n+1;
    }
 
    dptr = eyem;
    for (i = 0; i < m*m; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-m*m;
    for (i = 0; i < m; i++)
    {
        *(dptr+i*m+i) = 1;
    }
    dptr = x; dptr1 = x0;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr = *(U0_all+i*m+j);
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr = *(X0_all+i*n+j);
            dptr++; 
        }
    }
    /* At, Bt */
    F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,A,&n,eyen,&n,&fzero,At,&n);
    F77_CALL(dgemm)("n","t",&m,&n,&m,&fone,eyem,&m,B,&n,&fzero,Bt,&m);
 
    /* zmax, zmin */
    dptr1 = zmax;
    dptr2 = zmin;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr1 = *(umax+j);
            *dptr2 = *(umin+j);
            dptr1++; dptr2++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr1 = *(xmax+j);
            *dptr2 = *(xmin+j);
            dptr1++; dptr2++;
        }
    }  
 
    /* zmaxp, zminp */
    for (i = 0; i < nz; i++) zminp[i] = zmin[i] + 0.01*(zmax[i]-zmin[i]);
    for (i = 0; i < nz; i++) zmaxp[i] = zmax[i] - 0.01*(zmax[i]-zmin[i]);
 
    /* project z */
    for (i = 0; i < nz; i++) z[i] = z[i] > zmaxp[i] ? zmaxp[i] : z[i];
    for (i = 0; i < nz; i++) z[i] = z[i] < zminp[i] ? zminp[i] : z[i];
    
    ///* just plotting it! */
	//printf("====  altFmpc()  ====\n");
	//printf("n = %i | m = %i | T = %i | niters = %i | kappa = %f | quiet = %i | nz = %i\n", n, m, T, niters, kappa, quiet, nz);
	//printf("A\n");
	//printmat(A, n, n);
	//printf("B\n");
	//printmat(B, n, m);
	//printf("At\n");
	//printmat(At, n, n);
	//printf("Bt\n");
	//printmat(Bt, m, n);
	//printf("eyen\n");
	//printmat(eyen, n, n);
	//printf("eyem\n");
	//printmat(eyem, m, m);
	//printf("Q\n");
	//printmat(Q, n, n);
	//printf("R\n");
	//printmat(R, m, m);
	//printf("Qf\n");
	//printmat(Qf, n, n);     
	//printf("zmax\n");
	//printmat(zmax, n+m, T);
	//printf("zmin\n");
	//printmat(zmin, n+m, T); 
	//printf("x\n");
	//printmat(x, n, 1);
	//printf("z\n");
	//printmat(z, n+m, T);
	
    //t1 = clock();
    fmpcsolve(A,B,At,Bt,eyen,eyem,Q,R,Qf,zmax,zmin,x,z,T,n,m,nz,niters,kappa);
    //t2 = clock();
    //telapsed = (double)(t2-t1)/(CLOCKS_PER_SEC);
    
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *(altU_all+i*m+j) = *dptr;
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *(altX_all+i*n+j) = *dptr;
            dptr++;
        }
    }
    
    /* free inputs memories */
    //free(X0_all); free(U0_all); free(x0); free(xmax); free(xmin);
    //free(A); free(B); 
    /* free outputs memories */
    //free(X_all); free(U_all);
    free(At); free(Bt); free(eyen); free(eyem);
    free(z); free(x); free(zmax); free(zmin); free(zmaxp); free(zminp);
    return;
}

void fmpcsolve(double *A, double *B, double *At, double *Bt, double *eyen,
         double *eyem, double *Q, double *R, double *Qf, double *zmax, double *zmin, 
         double *x, double *z0, int T, int n, int m, int nz, int niters, double kappa) {
    int maxiter = niters;
    int iiter, i, cont;
    double alpha = 0.01;
    double beta = 0.9;
    double tol = 0.1;
    double s;
    double resd, resp, res, newresd, newresp, newres;
    double *b, *z, *nu, *Ctnu;
    double *dnu, *dz;
    double *gf, *gp, *hp, *newgf, *newgp, *newhp;
    double *rd, *rp, *newrd, *newrp;
    double *dptr, *dptr1, *dptr2, *dptr3;
    double *newnu, *newz, *newCtnu;

    /* memory allocation */
    b = malloc(sizeof(double)*T*n);
    dnu = malloc(sizeof(double)*T*n);
    dz = malloc(sizeof(double)*nz);
    nu = malloc(sizeof(double)*T*n);
    Ctnu = malloc(sizeof(double)*nz);
    z = malloc(sizeof(double)*nz);
    gp = malloc(sizeof(double)*nz);
    hp = malloc(sizeof(double)*nz);
    gf = malloc(sizeof(double)*nz);
    rp = malloc(sizeof(double)*T*n);
    rd = malloc(sizeof(double)*nz);
    newnu = malloc(sizeof(double)*T*n);
    newz = malloc(sizeof(double)*nz);
    newCtnu = malloc(sizeof(double)*nz);
    newgf = malloc(sizeof(double)*nz);
    newhp = malloc(sizeof(double)*nz);
    newgp = malloc(sizeof(double)*nz);
    newrp = malloc(sizeof(double)*T*n);
    newrd = malloc(sizeof(double)*nz);

    for (i = 0; i < n*T; i++) nu[i] = 0;
    for (i = 0; i < n*T; i++) b[i] = 0;

    F77_CALL(dgemv)("n",&n,&n,&fone,A,&n,x,&ione,&fzero,b,&ione);
    dptr = z; dptr1 = z0;
    for (i = 0; i < nz; i++) 
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    if (quiet == 0 && n == 6 && m == 3)
    {   
        //printf("Controller with n = %i and m = %i\n iteration \t step \t\t rd \t\t\t rp\n", n, m);
    }
    for (iiter = 0; iiter < maxiter; iiter++)
    {
        gfgphp(Q,R,Qf,zmax,zmin,z,T,n,m,nz,gf,gp,hp);
        rdrp(A,B,Q,R,Qf,z,nu,gf,gp,b,T,n,m,nz,kappa,rd,rp,Ctnu);
        resdresp(rd,rp,T,n,nz,&resd,&resp,&res);

        if (res < tol) break;

        dnudz(A,B,At,Bt,eyen,eyem,Q,R,Qf,hp,rd,rp,T,n,m,nz,kappa,dnu,dz); 

        s = 1; 
        /* feasibility search */
        while (1)
        {
            cont = 0;
            dptr = z; dptr1 = dz; dptr2 = zmax; dptr3 = zmin;
            for (i = 0; i < nz; i++)
            {
                if (*dptr+s*(*dptr1) >= *dptr2) cont = 1;
                if (*dptr+s*(*dptr1) <= *dptr3) cont = 1;
                dptr++; dptr1++; dptr2++; dptr3++;
            }
            if (cont == 1)
            {
                s = beta*s;
                continue;
            }
            else
                break;
        }

        dptr = newnu; dptr1 = nu; dptr2 = dnu;
        for (i = 0; i < T*n; i++)
        {
            *dptr = *dptr1+s*(*dptr2);
            dptr++; dptr1++; dptr2++;
        }
        dptr = newz; dptr1 = z; dptr2 = dz;
        for (i = 0; i < nz; i++)
        {
            *dptr = *dptr1+s*(*dptr2);
            dptr++; dptr1++; dptr2++;
        }

        /* insert backtracking line search */
        while (1)
        {
            gfgphp(Q,R,Qf,zmax,zmin,newz,T,n,m,nz,newgf,newgp,newhp);
            rdrp(A,B,Q,R,Qf,newz,newnu,newgf,newgp,b,T,n,m,nz,kappa,newrd,newrp,newCtnu);
            resdresp(newrd,newrp,T,n,nz,&newresd,&newresp,&newres);
            if (newres <= (1-alpha*s)*res) break;
            s = beta*s;
            dptr = newnu; dptr1 = nu; dptr2 = dnu;
            for (i = 0; i < T*n; i++)
            {
                *dptr = *dptr1+s*(*dptr2);
                dptr++; dptr1++; dptr2++;
            }
            dptr = newz; dptr1 = z; dptr2 = dz;
            for (i = 0; i < nz; i++)
            {
                *dptr = *dptr1+s*(*dptr2);
                dptr++; dptr1++; dptr2++;
            }
        }
        
        dptr = nu; dptr1 = newnu; 
        for (i = 0; i < T*n; i++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = z; dptr1 = newz;
        for (i = 0; i < nz; i++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        if (quiet == 0 && n==6 && m==3)
        {
            //printf("    %d \t\t %5.4f \t %0.5e \t\t %0.5e\n",iiter,s,newresd,newresp);
        }
    }
    dptr = z0; dptr1 = z;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }

    free(b); free(dnu); free(dz); free(nu); free(Ctnu);
    free(z); free(gp); free(hp); free(gf); free(rp); free(rd);
    free(newnu); free(newz); free(newCtnu); free(newgf); free(newhp);
    free(newgp); free(newrp); free(newrd);
    return;
}

/* computes the search directions dz and dnu */
void dnudz(double *A, double *B, double *At, double *Bt, double *eyen,
        double *eyem, double *Q, double *R, double *Qf, double *hp, double *rd, 
        double *rp, int T, int n, int m, int nz, double kappa, double *dnu, double *dz)
{
    int i,j,info; // nT;
    double *dptr, *dptr1, *dptr2, *dptr3, *temp, *tempmatn, *tempmatm;
    double *PhiQ, *PhiR, *Yd, *Yud, *Ld, *Lld, *Ctdnu, *gam, *v, *be, *rdmCtdnu;
    double *PhiinvQAt, *PhiinvRBt, *PhiinvQeye, *PhiinvReye, *CPhiinvrd;
    //nT = n*T;

    /* allocate memory */
    PhiQ = malloc(sizeof(double)*n*n*T);
    PhiR = malloc(sizeof(double)*m*m*T);
    PhiinvQAt = malloc(sizeof(double)*n*n*T);
    PhiinvRBt = malloc(sizeof(double)*m*n*T);
    PhiinvQeye = malloc(sizeof(double)*n*n*T);
    PhiinvReye = malloc(sizeof(double)*m*m*T);
    CPhiinvrd = malloc(sizeof(double)*n*T);
    Yd = malloc(sizeof(double)*n*n*T);
    Yud = malloc(sizeof(double)*n*n*(T-1));
    Ld = malloc(sizeof(double)*n*n*T);
    Lld = malloc(sizeof(double)*n*n*(T-1));
    gam = malloc(sizeof(double)*n*T);
    v = malloc(sizeof(double)*n*T);
    be = malloc(sizeof(double)*n*T);
    temp = malloc(sizeof(double)*n);
    tempmatn = malloc(sizeof(double)*n*n);
    tempmatm = malloc(sizeof(double)*m*m);
    Ctdnu = malloc(sizeof(double)*nz);
    rdmCtdnu = malloc(sizeof(double)*nz);

    /* form PhiQ and PhiR */
    for (i = 0; i < T-1; i++)
    {
        dptr = PhiQ+n*n*i; dptr1 = Q;
        for (j = 0; j < n*n; j++)
        {
            *dptr = 2*(*dptr1);
            dptr++; dptr1++;
        }
        dptr = PhiQ+n*n*i; dptr1 = hp+m*(i+1)+n*i;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr+kappa*(*dptr1);
            dptr = dptr+n+1; dptr1++;
        }
        dptr = PhiR+m*m*i; dptr1 = R;
        for (j = 0; j < m*m; j++)
        {
            *dptr = 2*(*dptr1);
            dptr++; dptr1++;
        }
        dptr = PhiR+m*m*i; dptr1 = hp+i*(n+m);
        for (j = 0; j < m; j++)
        {
            *dptr = *dptr+kappa*(*dptr1);
            dptr = dptr+m+1; dptr1++;
        }
    }
    
    dptr = PhiR+m*m*(T-1); dptr1 = R;
    for (j = 0; j < m*m; j++)
    {
        *dptr = 2*(*dptr1);
        dptr++; dptr1++;
    }
    dptr = PhiR+m*m*(T-1); dptr1 = hp+(T-1)*(n+m);
    for (j = 0; j < m; j++)
    {
        *dptr = *dptr+kappa*(*dptr1);
        dptr = dptr+m+1; dptr1++;
    }
    dptr = PhiQ+n*n*(T-1); dptr1 = Qf;
    for (j = 0; j < n*n; j++)
    {
        *dptr = 2*(*dptr1);
        dptr++; dptr1++;
    }
    dptr = PhiQ+n*n*(T-1); dptr1 = hp+m*T+n*(T-1);
    for (j = 0; j < n; j++)
    {
        *dptr = *dptr+kappa*(*dptr1);
        dptr = dptr+n+1; dptr1++;
    }

    /* compute PhiinvQAt, PhiinvRBt, PhiinvQeye, PhiinvReye */
    for (i = 0; i < T; i++)
    {
        dptr = PhiinvQAt+n*n*i; dptr1 = At;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n*n; dptr1 = PhiQ+n*n*i;
        F77_CALL(dposv)("l",&n,&n,dptr1,&n,dptr,&n,&info);
        dptr = PhiinvQeye+n*n*i; dptr1 = eyen;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n*n; dptr1 = PhiQ+n*n*i;
        F77_CALL(dtrtrs)("l","n","n",&n,&n,dptr1,&n,dptr,&n,&info);
        F77_CALL(dtrtrs)("l","t","n",&n,&n,dptr1,&n,dptr,&n,&info);
    }
    for (i = 0; i < T; i++)
    {
        dptr = PhiinvRBt+m*n*i; dptr1 = Bt;
        for (j = 0; j < n*m; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-m*n; dptr1 = PhiR+m*m*i;
        F77_CALL(dposv)("l",&m,&n,dptr1,&m,dptr,&m,&info);
        dptr = PhiinvReye+m*m*i; dptr1 = eyem;
        for (j = 0; j < m*m; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-m*m; dptr1 = PhiR+m*m*i;
        F77_CALL(dtrtrs)("l","n","n",&m,&m,dptr1,&m,dptr,&m,&info);
        F77_CALL(dtrtrs)("l","t","n",&m,&m,dptr1,&m,dptr,&m,&info);
    }
    
    /* form Yd and Yud */
    dptr = Yud; dptr1 = PhiinvQAt; 
    for (i = 0; i < n*n*(T-1); i++)
    {
        *dptr = -(*dptr1);
        dptr++; dptr1++;
    }
    dptr2 = Yd; dptr3 = PhiinvQeye;
    for (i = 0; i < n*n; i++)
    {
        *dptr2 = *dptr3;
        dptr2++; dptr3++;
    }
    dptr2 = dptr2-n*n;
    F77_CALL(dgemm)("n","n",&n,&n,&m,&fone,B,&n,PhiinvRBt,&m,&fone,dptr2,&n);
    for (i = 1; i < T; i++)
    {
        dptr = Yd+n*n*i; dptr1 = PhiinvQeye+n*n*i;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr1 = PhiinvRBt+m*n*i; dptr = dptr-n*n;
        F77_CALL(dgemm)("n","n",&n,&n,&m,&fone,B,&n,dptr1,&m,&fone,dptr,&n); 
        dptr1 = PhiinvQAt+n*n*(i-1);
        F77_CALL(dgemm)("n","n",&n,&n,&n,&fone,A,&n,dptr1,&n,&fone,dptr,&n);
    }

    /* compute Lii */
    dptr = Ld; dptr1 = Yd; 
    for (i = 0; i < n*n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++; 
    }
    dptr = dptr-n*n; 
    F77_CALL(dposv)("l",&n,&ione,dptr,&n,temp,&n,&info);
    for (i = 1; i < T; i++)
    {
        dptr = Ld+n*n*(i-1); dptr1 = Yud+n*n*(i-1); dptr2 = Lld+n*n*(i-1);
        for (j = 0; j < n*n; j++)
        {
            *dptr2 = *dptr1;
            dptr2++; dptr1++;
        }
        dptr2 = dptr2-n*n;
        F77_CALL(dtrtrs)("l","n","n",&n,&n,dptr,&n,dptr2,&n,&info);
        dptr1 = tempmatn;
        for (j = 0; j < n*n; j++)
        {
            *dptr1 = *dptr2;
            dptr1++; dptr2++;
        }
        dptr1 = dptr1-n*n; dptr2 = dptr2-n*n;
        F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,dptr1,&n,eyen,&n,&fzero,dptr2,&n);
        dptr = Ld+n*n*i; dptr1 = Yd+n*n*i;
        for (j = 0; j < n*n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n*n;
        F77_CALL(dgemm)("n","t",&n,&n,&n,&fmone,dptr2,&n,dptr2,&n,&fone,dptr,&n);
        F77_CALL(dposv)("l",&n,&ione,dptr,&n,temp,&n,&info);
    }

    /* compute CPhiinvrd */
    dptr = CPhiinvrd; dptr1 = rd+m;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = dptr-n;
    F77_CALL(dtrsv)("l","n","n",&n,PhiQ,&n,dptr,&ione);
    F77_CALL(dtrsv)("l","t","n",&n,PhiQ,&n,dptr,&ione);
    dptr2 = temp; dptr1 = rd;
    for (i = 0; i < m; i++)
    {
        *dptr2 = *dptr1;
        dptr2++; dptr1++;
    }
    dptr2 = dptr2-m;
    F77_CALL(dtrsv)("l","n","n",&m,PhiR,&m,dptr2,&ione);
    F77_CALL(dtrsv)("l","t","n",&m,PhiR,&m,dptr2,&ione);
    F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,temp,&ione,&fone,dptr,&ione);
    
    for (i = 1; i < T; i++)
    {
        dptr = CPhiinvrd+n*i; dptr1 = rd+m+i*(n+m);
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n; dptr3 = PhiQ+n*n*i;
        F77_CALL(dtrsv)("l","n","n",&n,dptr3,&n,dptr,&ione);
        F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr,&ione);
        dptr2 = temp; dptr1 = rd+i*(m+n);
        for (j = 0; j < m; j++)
        {
            *dptr2 = *dptr1;
            dptr2++; dptr1++;
        }
        dptr3 = PhiR+m*m*i; dptr2 = dptr2-m;
        F77_CALL(dtrsv)("l","n","n",&m,dptr3,&m,dptr2,&ione);
        F77_CALL(dtrsv)("l","t","n",&m,dptr3,&m,dptr2,&ione);
        F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,temp,&ione,&fone,dptr,&ione);
        dptr2 = temp; dptr1 = rd+(i-1)*(n+m)+m;
        for (j = 0; j < n; j++)
        {
            *dptr2 = *dptr1;
            dptr2++; dptr1++;
        }
        dptr3 = PhiQ+n*n*(i-1); dptr2 = dptr2-n;
        F77_CALL(dtrsv)("l","n","n",&n,dptr3,&n,dptr2,&ione);
        F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr2,&ione);
        F77_CALL(dgemv)("n",&n,&n,&fmone,A,&n,temp,&ione,&fone,dptr,&ione);
    }

    /* form be */
    dptr = be; dptr1 = rp; dptr2 = CPhiinvrd;
    for (i = 0; i < n*T; i++)
    {
        *dptr = (*dptr2)-(*dptr1);
        dptr++; dptr1++; dptr2++;
    }

    /* solve for dnu */
    dptr = v; dptr1 = be;
    for (i = 0; i < n; i++)
    {
        *dptr = -(*dptr1);
        dptr++; dptr1++;
    }
    dptr = dptr-n;
    F77_CALL(dtrsv)("l","n","n",&n,Ld,&n,dptr,&ione);
    for (i = 1; i < T; i++)
    {
        dptr = v+i*n; dptr1 = v+(i-1)*n; dptr2 = be+i*n; 
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr2;
            dptr++; dptr2++;
        }
        dptr = dptr-n; dptr3 = Lld+n*n*(i-1);
        F77_CALL(dgemv)("n",&n,&n,&fmone,dptr3,&n,dptr1,&ione,&fmone,dptr,&ione);
        dptr3 = Ld+n*n*i;
        F77_CALL(dtrsv)("l","n","n",&n,dptr3,&n,dptr,&ione);
    }
    dptr = dnu+n*(T-1); dptr1 = v+n*(T-1);
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = dptr-n; dptr3 = Ld+n*n*(T-1);
    F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr,&ione);
    for (i = T-1; i > 0; i--)
    {
        dptr = dnu+n*(i-1); dptr1 = dnu+n*i; dptr2 = v+n*(i-1); 
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr2;
            dptr++; dptr2++;
        }
        dptr = dptr-n; dptr3 = Lld+n*n*(i-1);
        F77_CALL(dgemv)("t",&n,&n,&fmone,dptr3,&n,dptr1,&ione,&fone,dptr,&ione);
        dptr3 = Ld+n*n*(i-1);
        F77_CALL(dtrsv)("l","t","n",&n,dptr3,&n,dptr,&ione);
    }

    /* form Ctdnu */
    for (i = 0; i < T-1; i++)
    {
        dptr = Ctdnu+i*(n+m); dptr1 = dnu+i*n;
        F77_CALL(dgemv)("n",&m,&n,&fmone,Bt,&m,dptr1,&ione,&fzero,dptr,&ione);
        dptr = Ctdnu+i*(n+m)+m; dptr2 = dnu+(i+1)*n;
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr1;
            dptr++; dptr1++;
        }
        dptr = dptr-n;
        F77_CALL(dgemv)("n",&n,&n,&fmone,At,&n,dptr2,&ione,&fone,dptr,&ione);
    }
    
    dptr = Ctdnu+(T-1)*(n+m); dptr1 = dnu+(T-1)*n;
    F77_CALL(dgemv)("n",&m,&n,&fmone,Bt,&m,dptr1,&ione,&fzero,dptr,&ione);
    dptr = dptr+m; 
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = rdmCtdnu; dptr1 = Ctdnu; dptr2 = rd;
    for (i = 0; i < nz; i++)
    {
        *dptr = -(*dptr1)-(*dptr2);
        dptr++; dptr1++; dptr2++;
    }

    /* solve for dz */
    for (i = 0; i < T; i++)
    {
        dptr = dz+(i+1)*m+i*n; dptr1 = rdmCtdnu+(i+1)*m+i*n;
        dptr2 = PhiinvQeye+n*n*i;
        F77_CALL(dgemv)("n",&n,&n,&fone,dptr2,&n,dptr1,&ione,&fzero,dptr,&ione);
    }
    for (i = 0; i < T; i++)
    {
        dptr = dz+i*(m+n); dptr1 = rdmCtdnu+i*(m+n);
        dptr2 = PhiinvReye+m*m*i;
        F77_CALL(dgemv)("n",&m,&m,&fone,dptr2,&m,dptr1,&ione,&fzero,dptr,&ione);
    }
    free(PhiQ); free(PhiR); free(PhiinvQAt); free(PhiinvRBt); free(PhiinvQeye);
    free(PhiinvReye); free(CPhiinvrd); free(Yd); free(Yud); free(Ld); free(Lld);
    free(gam); free(v); free(be); free(temp); free(tempmatn); free(tempmatm); 
    free(Ctdnu); free(rdmCtdnu);
    return;
}

/* computes rd and rp */
void rdrp(double *A, double *B, double *Q, double *R, double *Qf, double *z, double *nu, 
        double *gf, double *gp, double *b, int T, int n, int m, int nz, 
        double kappa, double *rd, double *rp, double *Ctnu)
{
    int i, j;
    double *Cz;
    double *dptr, *dptr1, *dptr2;

    Cz = malloc(sizeof(double)*T*n);
    
    /* compute Cz */
    dptr = Cz; dptr1 = z+m;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,z,&ione,&fone,Cz,&ione);
    for (i = 2; i <= T; i++)
    {
        dptr = Cz+(i-1)*n; dptr1 = z+m+(i-2)*(n+m); 
        dptr2 = z+m+(i-1)*(m+n);
        for (j = 0; j < n; j++)
        {
            *dptr = *dptr2;
            dptr++; dptr2++;
        }
        dptr = dptr-n; 
        F77_CALL(dgemv)("n",&n,&n,&fmone,A,&n,dptr1,&ione,&fone,dptr,&ione);
        dptr1 = dptr1+n;
        F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,dptr1,&ione,&fone,dptr,&ione);
    }
    /*
    dptr = Cz+(T-1)*n; dptr1 = z+m+(T-2)*(n+m);
    F77_CALL(dgemv)("n",&n,&n,&fmone,A,&n,dptr1,&ione,&fzero,dptr,&ione);
    dptr1 = dptr1+n;
    F77_CALL(dgemv)("n",&n,&m,&fmone,B,&n,dptr1,&ione,&fone,dptr,&ione);
    dptr1 = z+nz-n;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr+*dptr1;
        dptr++; dptr1++;
    }
    */

    /* compute Ctnu */
    dptr = Ctnu; dptr1 = Ctnu+m; dptr2 = nu;
    for (i = 1; i <= T-1; i++)
    {
        F77_CALL(dgemv)("t",&n,&m,&fmone,B,&n,dptr2,&ione,&fzero,dptr,&ione);
        dptr = dptr+n+m;
        for (j = 0; j < n; j++)
        {
            *dptr1 = *dptr2;
            dptr1++; dptr2++;
        }
        dptr1 = Ctnu+m+(i-1)*(n+m);
        F77_CALL(dgemv)("t",&n,&n,&fmone,A,&n,dptr2,&ione,&fone,dptr1,&ione);
        dptr1 = dptr1+n+m;
    }
    F77_CALL(dgemv)("t",&n,&m,&fmone,B,&n,dptr2,&ione,&fzero,dptr,&ione);
    dptr = Ctnu+nz-n; dptr1 = nu+(T-1)*n;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }

    dptr = rp; dptr1 = Cz; dptr2 = b;
    for (i = 0; i < n*T; i++)
    {
        *dptr = *dptr1-*dptr2;
        dptr++; dptr1++; dptr2++;
    }
    dptr = rd; dptr1 = Ctnu; dptr2 = gf;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1+*dptr2;
        dptr++; dptr1++; dptr2++;
    }
    F77_CALL(daxpy)(&nz,&kappa,gp,&ione,rd,&ione);
    free(Cz);
    return;
}

/* computes gf, gp and hp */
void gfgphp(double *Q, double *R, double *Qf, double *zmax, double *zmin, double *z,
        int T, int n, int m, int nz, double *gf, double *gp, double *hp)
{
    int i;
    double *dptr, *dptr1, *dptr2;
    double *gp1, *gp2;

    gp1 = malloc(sizeof(double)*nz);
    gp2 = malloc(sizeof(double)*nz);

    dptr = gp1; dptr1 = zmax; dptr2 = z;
    for (i = 0; i < nz; i++)
    {
        *dptr = 1.0/(*dptr1-*dptr2);
        dptr++; dptr1++; dptr2++;
    }
    dptr = gp2; dptr1 = zmin; dptr2 = z;
    for (i = 0; i < nz; i++)
    {
        *dptr = 1.0/(*dptr2-*dptr1);
        dptr++; dptr1++; dptr2++;
    }
    dptr = hp; dptr1 = gp1; dptr2 = gp2;
    for (i = 0; i < nz; i++)
    {
        *dptr = (*dptr1)*(*dptr1) + (*dptr2)*(*dptr2);
        dptr++; dptr1++; dptr2++;
    }
    dptr = gp; dptr1 = gp1; dptr2 = gp2;
    for (i = 0; i < nz; i++)
    {
        *dptr = *dptr1-*dptr2;
        dptr++; dptr1++; dptr2++;
    }
    
    dptr = gf; dptr1 = z; 
    for (i = 0; i < T-1; i++)
    {
        F77_CALL(dgemv)("n",&m,&m,&ftwo,R,&m,dptr1,&ione,&fzero,dptr,&ione);
        dptr = dptr+m; dptr1 = dptr1+m;
        F77_CALL(dgemv)("n",&n,&n,&ftwo,Q,&n,dptr1,&ione,&fzero,dptr,&ione);
        dptr = dptr+n; dptr1 = dptr1+n;
    }
    F77_CALL(dgemv)("n",&m,&m,&ftwo,R,&m,dptr1,&ione,&fzero,dptr,&ione);
    dptr = dptr+m; dptr1 = dptr1+m;
    F77_CALL(dgemv)("n",&n,&n,&ftwo,Qf,&n,dptr1,&ione,&fzero,dptr,&ione);

    free(gp1); free(gp2);
    return;
}

/* computes resd, resp, and res */
void resdresp(double *rd, double *rp, int T, int n, int nz, double *resd, 
        double *resp, double *res)
{
    int nnu = T*n;
    *resp = F77_CALL(dnrm2)(&nnu,rp,&ione);
    *resd = F77_CALL(dnrm2)(&nz,rd,&ione);
    *res = sqrt((*resp)*(*resp)+(*resd)*(*resd));
    return;
}

