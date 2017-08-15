/*
 * VarianceMean.c
 * 
 * Copyright 2017  <pi@raspberrypi>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include <stdio.h>

#define CALIBRATION 1000

void ekfCalibration(double *Rekf, double *ekf0, double *ekfCal, double *ymeas, int counterCal){
	// Calibration routine to get mean, variance and std_deviation
	if(counterCal==CALIBRATION){
		// Mean (bias) accelerometer, gyroscope and magnetometer
		for (int i=0;i<CALIBRATION;i++){
			ekf0[0]+=ekfCal[i*3];
			ekf0[1]+=ekfCal[i*3+1];
			ekf0[2]+=ekfCal[i*3+2];
			ekf0[3]+=ekfCal[i*3+3];
			ekf0[4]+=ekfCal[i*3+4];
			ekf0[5]+=ekfCal[i*3+5];
		}
		ekf0[0]/=CALIBRATION;
		ekf0[1]/=CALIBRATION;
		ekf0[2]/=CALIBRATION;
		ekf0[3]/=CALIBRATION;
		ekf0[4]/=CALIBRATION;
		ekf0[5]/=CALIBRATION;
		
		// Sum up for variance calculation
		for (int i=0;i<CALIBRATION;i++){
			Rekf[0]+=pow((ekfCal[i*3] - ekf0[0]), 2);
			Rekf[7]+=pow((ekfCal[i*3+1] - ekf0[1]), 2);
			Rekf[14]+=pow((ekfCal[i*3+2] - ekf0[2]), 2);
			Rekf[21]+=pow((ekfCal[i*3+3] - ekf0[3]), 2);
			Rekf[28]+=pow((ekfCal[i*3+4] - ekf0[4]), 2);
			Rekf[35]+=pow((ekfCal[i*3+5] - ekf0[5]), 2);
		}
		// Variance (sigma)
		Rekf[0]/=CALIBRATION;
		Rekf[7]/=CALIBRATION;
		Rekf[14]/=CALIBRATION;
		Rekf[21]/=CALIBRATION;
		Rekf[28]/=CALIBRATION;
		Rekf[35]/=CALIBRATION;
		
		// Overide calibration when position measurements are gone
		Rekf[0]=1;
		Rekf[7]=1;
		Rekf[14]=1;
	
		// Print results
		printf("Mean (bias) EKF\n");
		printmat(ekf0,6,1);
		printf("Covariance matrix (sigma) EKF\n");
		printmat(Rekf,6,6);
	}
	// Default i save calibrartion data
	else{
		ekfCal[counterCal*3]=ymeas[0];
		ekfCal[counterCal*3+1]=ymeas[1];
		ekfCal[counterCal*3+2]=ymeas[2];
		ekfCal[counterCal*3+3]=ymeas[3];
		ekfCal[counterCal*3+4]=ymeas[4];
		ekfCal[counterCal*3+5]=ymeas[5];
	}		
}

// Matrix print function
void printmat(double *A, int m, int n){
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%3.18f\t", *(dptr+m*i+j));
        }
        printf("\n");
    }
    //printf("\n");
    return;
}


int main(int argc, char **argv)
{
	double Rekf[36]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double ekf0[6]={0,0,0,0,0,0}, ekfCal[6*CALIBRATION];
	
	
	//ekfCalibration(double *Rekf, double *ekf0, double *ekfCal, double *ymeas, int counterCal)
	
	return 0;
}

