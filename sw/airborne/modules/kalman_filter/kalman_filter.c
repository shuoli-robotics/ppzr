/*
 * Copyright (C) Shuo Li
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/kalman_filter/kalman_filter.c"
 * @author Shuo Li
 * 
 */

#include "modules/kalman_filter/kalman_filter.h"
#include "math.h"
#include <stdlib.h>
#include <stdio.h>
#include "pprz_algebra_float.h"



double determinant(double a[3][3],int k);
void transpose(double num[3][3],double fac[3][3],int r,double c[][3]);
/*double inverse[3][3];*/
void cofactor(double num[3][3],int f,double c[][3]);
void matrix_copy(double matrix_a[][3],double matrix_b[][3]);
void matrix_mult(double matrix_a[][3],double matrix_b[][3],double c[][3]);
void matrix_transpose(double matrix_a[][3],double matrix_b[][3]);
void matrix_add(double matrix_a[][3],double matrix_b[][3],double matrix_c[][3]);
void matrix_subs(double matrix_a[][3],double matrix_b[][3],double matrix_c[][3]);
void matrix_mult_vector(double a[3],double b[][3],double c[3]);
void vector_mult_cons(double a[3],double constant);
void vector_add(double a[3],double b[3],double c[3]);
void vector_subs(double a[3],double b[3],double c[3]);
void vector_copy(double a[3],double b[3]);


double phi;
double theta;
double psi;
double phi_previous_step;
double theta_previous_step;
double psi_previous_step;
double P_k_k[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
double P_k_1[3][3];
double P_k_k_1[3][3]; 
double Q_guess[3][3]={};
double R_guess[3][3]={};
double K_k[3][3];
double H[3][3]={{1,0,0},{0,1,0},{0,0,1}};
double I[3][3]={{1,0,0},{0,1,0},{0,0,1}};
double v_last_step[3];
double estimated_v[3]; 
double predicted_v[3]; 
double R_B_E[3][3];
double estimated_delta_x[3];
int pointer=1;



void extended_kalman_filter() {
if (pointer == 1)
{
		theta = stateGetNedToBodyEulers_f()->theta;
		phi = stateGetNedToBodyEulers_f()->phi;
		psi = stateGetNedToBodyEulers_f()->psi;
		estimated_v[0]= stateGetSpeedNed_f()->x;
		estimated_v[1]= stateGetSpeedNed_f()->y;
		estimated_v[2]= stateGetSpeedNed_f()->z;
}	
else
{
		theta = theta_previous_step;
		phi = phi_previous_step;
		psi = psi_previous_step;
}

double temp_matrix1[3][3];
double temp_matrix2[3][3];
double temp_matrix3[3][3];
double temp_inv[3][3];


// predict velocity
double temp_rotation_matrix[9] = {cosf(psi)*cosf(theta),cosf(psi)*sinf(theta)*sinf(phi)-sinf(psi)*cosf(phi),
    cosf(psi)*sinf(theta)*cosf(phi)+sinf(psi)*sinf(phi),
		sinf(psi)*cosf(theta),sinf(psi)*sinf(theta)*sinf(phi)+cosf(psi)*cosf(phi),
      sinf(psi)*sinf(theta)*cosf(phi)-cosf(psi)*sinf(phi),
		-sinf(theta),cosf(theta)*sinf(phi),cosf(theta)*cosf(phi)};  
int k = 0;
for(int i = 0;i<3;i++)
{
		for (int j=0;j<3;j++)
		{
				R_B_E[i][j] = temp_rotation_matrix[k];
				k++;
		}
}

double thrust[3]= {0,0,-9.8};
double va[3];
matrix_mult_vector(va,R_B_E,thrust);
vector_mult_cons(va,STEP);
va[2]= va[2]+9.8; 
vector_copy(v_last_step,estimated_v);
vector_add(predicted_v,v_last_step,va);


// calculate P_k_k_1
double Phi_k_k_1[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
matrix_copy(P_k_1,P_k_k);
matrix_mult(temp_matrix1,Phi_k_k_1,P_k_1);  // temp1 = phi_k_k_1*P_k_1
matrix_transpose(temp_matrix2,Phi_k_k_1);   // temp2 = phi_k_k_1'
matrix_mult(temp_matrix3,temp_matrix1,temp_matrix2); // temp3 = phi_k_k_1*P_k_1*phi_k_k_1'
matrix_add(P_k_k_1,temp_matrix3,Q_guess);           //P_k_k_1 = phi_k_k_1*P_k_1*phi_k_k_1'+Q

// calculate K_k
double trans_H[3][3];
matrix_transpose(trans_H,H);
matrix_mult(temp_matrix1,H,P_k_k_1);  // temp1 = H*P_k_k_1
matrix_mult(temp_matrix2,temp_matrix1,trans_H); // temp2 = H*P_k_k_1*H'
matrix_add(temp_matrix3,temp_matrix2,R_guess);  // temp3 = H*P_k_k_1*H'+R
cofactor(temp_matrix3,3,temp_inv);            // temp_inv = inv(temp3 = H*P_k_k_1*H'+R)
matrix_mult(temp_matrix1,P_k_k_1,trans_H);   // temp1 = P_k_k_1*H'
matrix_mult(K_k,temp_matrix1,temp_inv);    // K_k =P_k_k_1*H' +inv(H*P_k_k_1*H'+R)


// calculate estimated_delta_x and estimated current velocity 
double delta_states[3];
double delta_velocity[3];
double current_velocity[3]={stateGetSpeedNed_f()->x,stateGetSpeedNed_f()->y,stateGetSpeedNed_f()->z};
vector_subs(delta_velocity,current_velocity,predicted_v);
matrix_mult_vector(delta_states,K_k,delta_velocity);
vector_add(estimated_v,predicted_v,delta_states);

// update P_k_k
double trans_matrix[3][3];
matrix_mult(temp_matrix1,K_k,H); // temp_matrix1 = K_k*H
matrix_subs(temp_matrix2,I,temp_matrix1);  // temp_matrix2 = I - K_k*H
matrix_transpose(trans_matrix,temp_matrix2);  // trans_matrix = (I-K_k*H)'
matrix_mult(temp_matrix1,temp_matrix2,P_k_k_1); //temp_matrix1 = (I-K_k*H)*P_k_k_1
matrix_mult(temp_matrix2,temp_matrix1,trans_matrix);  // temp_matrix2 = (I-K_k*H)*P_k_k_1*(I-K_k*H)'
matrix_mult(temp_matrix3,K_k,R_guess);        // temp_matrix3 = K_k*R_k
matrix_transpose(trans_matrix,K_k);    // trans_matrix = K_k'
matrix_mult(temp_matrix1,temp_matrix3,trans_matrix); //temp_matrix1 = K_k*R_k*K_k'
matrix_add(P_k_k,temp_matrix2,temp_matrix1);

// log attitude at this step. They will be used at next step to predict velocity
theta_previous_step = stateGetNedToBodyEulers_f()->theta;
phi_previous_step = stateGetNedToBodyEulers_f()->phi;
psi_previous_step = stateGetNedToBodyEulers_f()->psi;

}


/*For calculating Determinant of the Matrix */
double determinant(double a[3][3],int k)
{
		double s=1,det=0,b[3][3];
		int i,j,m,n,c;
		if (k==1)
		{
				return (a[0][0]);
		}
		else
		{
				det=0;
				for (c=0;c<k;c++)
				{
						m=0;
						n=0;
						for (i=0;i<k;i++)
						{
								for (j=0;j<k;j++)
								{
										b[i][j]=0;
										if (i != 0 && j != c)
										{
												b[m][n]=a[i][j];
												if (n<(k-2))
														n++;
												else
												{
														n=0;
														m++;
												}
										}
								}
						}
						det=det + s * (a[0][c] * determinant(b,k-1));
						s=-1 * s;
				}
		}
		return (det);
}



void cofactor(double num[3][3],int f,double c[][3])
{
		double b[3][3],fac[3][3];
		int p,q,m,n,i,j;
		for (q=0;q<f;q++)
		{
				for (p=0;p<f;p++)
				{
						m=0;
						n=0;
						for (i=0;i<f;i++)
						{
								for (j=0;j<f;j++)
								{
										if (i != q && j != p)
										{
												b[m][n]=num[i][j];
												if (n<(f-2))
														n++;
												else
												{
														n=0;
														m++;
												}
										}
								}
						}
						fac[q][p]=pow(-1,q + p) * determinant(b,f-1);
				}
		}
		transpose(num,fac,f,c);
}
/*Finding transpose of matrix*/ 
void transpose(double num[3][3],double fac[3][3],int r,double c[][3])
{
		// num: original matrix fac:co matrix c:inversed matrix
		int i,j;
		double b[3][3],d;

		for (i=0;i<r;i++)
		{
				for (j=0;j<r;j++)
				{
						b[i][j]=fac[j][i];
				}
		}
		d=determinant(num,r);
		for (i=0;i<r;i++)
		{
				for (j=0;j<r;j++)
				{
						c[i][j]=b[i][j] / d;
				}
		}
   /*     printf("\n\n\nThe inverse of matrix is : \n");*/

		/*for (i=0;i<r;i++)*/
		/*{*/
				/*for (j=0;j<r;j++)*/
				/*{*/
						/*printf("\t%f",c[i][j]);*/
				/*}*/
				/*printf("\n");*/
	  /*  }*/
}

void matrix_copy(double matrix_a[][3],double matrix_b[][3])
{
		for (int i;i<3;i++)
		{
				for (int j;j<3;j++)
				{
						matrix_a[i][j] = matrix_b[i][j];
				}
		}
}


void matrix_mult(double matrix_a[][3],double matrix_b[][3],double matrix_c[][3])
{
//Multiplication Logic
int i,j,k;
double sum;
		for (i = 0; i <= 2; i++) {
				for (j = 0; j <= 2; j++) {
						sum = 0;
						for (k = 0; k <= 2; k++) {
								sum = sum +matrix_b[i][k]*matrix_c[k][j];
						}
						matrix_a[i][j] = sum;
				}
		}
}


void matrix_transpose(double matrix_a[][3],double matrix_b[][3])
{
		int i,j;
		for(i=0; i<3; ++i)
				for(j=0; j<3; ++j)
				{
						matrix_a[j][i] = matrix_b[i][j];
				}
}

void matrix_add(double matrix_a[][3],double matrix_b[][3],double matrix_c[][3])
{
		int i,j;
		for(i=0; i<3; ++i)
				for(j=0; j<3; ++j)
				{
						matrix_a[i][j] = matrix_b[i][j]+matrix_c[i][j];
				}
}

void matrix_subs(double matrix_a[][3],double matrix_b[][3],double matrix_c[][3])
{
		int i,j;
		for(i=0; i<3; ++i)
				for(j=0; j<3; ++j)
				{
						matrix_a[i][j] = matrix_b[i][j]-matrix_c[i][j];
				}
}

void matrix_mult_vector(double a[3],double b[][3],double c[3])
{
	for (int i=0;i<3;i++)
	{
		double sum = 0;
		for (int j = 0;j<3;j++)
		{
				sum = sum+b[i][j]*c[j];
		}
		a[i]=sum;
	}
}

void vector_mult_cons(double a[3],double constant)
{
		for(int i=0;i<3;i++)
		a[i] = a[i]*constant;
}

void vector_add(double a[3],double b[3],double c[3])
{
		for(int i=0;i<3;i++)
				a[i]=c[i]+b[i];
}

void vector_subs(double a[3],double b[3],double c[3])
{
		for(int i=0;i<3;i++)
				a[i]=b[i]-c[i];
}

void vector_copy(double a[3],double b[3])
{
		for(int i = 0;i<3;i++)
				a[i]=b[i];
}
