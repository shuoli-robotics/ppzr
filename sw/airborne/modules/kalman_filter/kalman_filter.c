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



double determinant(double a[3][3],double k);
void transpose(double num[3][3],double fac[3][3],double r,double c[][3]);
double inverse[3][3];
void cofactor(double num[3][3],double f,double c[][3]);




 void extended_kalman_filter() {
 
 }

void inv_matrix(double a[][3], double b[][3],int n)
{
		double det = determinant(a,n);
		printf("!!!!!!!!!!!!!%f\n",det);
		cofactor(a,3,b);
		//b = inverse;
}



/*
 *    Recursive definition of determinate using expansion by minors.
 *    */

/*For calculating Determinant of the Matrix */
double determinant(double a[3][3],double k)
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



void cofactor(double num[3][3],double f,double c[][3])
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
void transpose(double num[3][3],double fac[3][3],double r,double c[][3])
{
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
		printf("\n\n\nThe inverse of matrix is : \n");

		for (i=0;i<r;i++)
		{
				for (j=0;j<r;j++)
				{
						printf("\t%f",c[i][j]);
				}
				printf("\n");
		}
}

