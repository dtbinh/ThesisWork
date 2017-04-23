/*
 * MatrixInverse3x3.c
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


// Matrix print function
void printmat(double *A, int m, int n){
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%6.4f\t", *(dptr+m*i+j));
        }
        printf("\n");
    }
    printf("\n");
    return;
}


int main(int argc, char **argv)
{
	double Sacc_inv[9]={0,0,0,0,0,0,0,0,0};
	double Sacc[9]={4,2,2,2,2,2,3,6,9};
	
	// inverse test
	Sacc_inv[0] = (Sacc[4] * Sacc[8] - Sacc[5] * Sacc[7]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[3] = -(Sacc[3] * Sacc[8] - Sacc[5] * Sacc[6]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[6] = (Sacc[3] * Sacc[7] - Sacc[4] * Sacc[6]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[1] = -(Sacc[1] * Sacc[8] - Sacc[2] * Sacc[7]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[4] = (Sacc[0] * Sacc[8] - Sacc[2] * Sacc[6]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[7] = -(Sacc[0] * Sacc[7] - Sacc[1] * Sacc[6]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[2] = (Sacc[1] * Sacc[5] - Sacc[2] * Sacc[4]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[5] = -(Sacc[0] * Sacc[5] - Sacc[2] * Sacc[3]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);
	Sacc_inv[8] = (Sacc[0] * Sacc[4] - Sacc[1] * Sacc[3]) / (Sacc[0] * Sacc[4] * Sacc[8] - Sacc[0] * Sacc[5] * Sacc[7] - Sacc[1] * Sacc[3] * Sacc[8] + Sacc[1] * Sacc[6] * Sacc[5] + Sacc[2] * Sacc[3] * Sacc[7] - Sacc[2] * Sacc[6] * Sacc[4]);

	printf("Matrix S:\n");
	printmat(Sacc,3,3);
	
	printf("Inverse Matrix S:\n");
	printmat(Sacc_inv,3,3);
	return 0;
}


