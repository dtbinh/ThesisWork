#ifndef _CONTROLLER_
#define _CONTROLLER_

#define T_max 50


struct Parameters {
	double g, mass, L, k, b, k_d, i_xx, i_yy, i_zz, c_m;
	};
	
struct PosParams {
	double A[36], B[12], Q[36], R[4], Qf[36], umax[2], umin[2];
	double kappa;
	int n, m, T, niters;
	};
	
struct PosInputs {
	double X0_all[6*T_max], U0_all[2*T_max], x0[6], xmax[6], xmin[6];
	}posInputs;
	
struct AttParams {
	double A[36], B[18], Q[36], R[9], Qf[36], umax[3], umin[3];
	double kappa;
	int n, m, T, niters;
	};
	
struct AttInputs {
	double X0_all[6*T_max], U0_all[3*T_max], x0[6], xmax[6], xmin[6];
	}attInputs;

struct AltParams {
	double A[4], B[2], Q[4], R[1], Qf[4], umax[1], umin[1];
	double kappa;
	int n, m, T, niters;
	};

struct AltInputs {
	double X0_all[2*T_max], U0_all[1*T_max], x0[2], xmax[2], xmin[2];
	}altInputs;






extern void startController(void*, void*);

#endif
