/*
 * Copyright (C) 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/snake_gate_detection.h
 */

#ifndef SNAKE_GATE_DETECTION_CV_PLUGIN_H
#define SNAKE_GATE_DETECTION_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

/* Gate structure */
struct gate_img {
  int x;             ///< The image x coordinate of the gate center
  int y;             ///< The image y coordinate of the gate center
  int x_corners[4];///< Array of corner x coordinates
  int y_corners[4];///< Array of corner y coordinates
  int sz;            ///< Half the image size of the gate 
  float gate_q;      ///< gate quality
  int n_sides;       ///< How many sides are orange (to prevent detecting a small gate in the corner of a big one partially out of view).
  float sz_left;     ///< Half the image size of the left side
  float sz_right;    ///< Half the image size of the right side
};

// Module functions
extern void snake_gate_detection_init(void);
extern int check_color(struct image_t *im, int x, int y);
extern void snake_up_and_down(struct image_t *im, int x, int y, int* y_low, int* y_high);
extern void snake_left_and_right(struct image_t *im, int x, int y, int* x_low, int* x_high);
extern void draw_gate(struct image_t *im, struct gate_img gate);
extern void draw_gate_polygon(struct image_t *im, int *x_points, int *y_points, uint8_t* color);
extern void draw_gate_color(struct image_t *im, struct gate_img gate, uint8_t* color);
extern void check_gate(struct image_t *im, struct gate_img gate, float* quality, int* sides);
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int* n_points, int* n_colored_points);
void check_gate_free(struct image_t *im, struct gate_img gate, float *quality, int *n_sides);

void vec_from_point(float point_x, float point_y, int f, struct FloatVect3 *vec);
void vec_from_point_2(float point_x, float point_y, int f, struct FloatVect3 *vec);
void vec_from_point_ned(float point_x, float point_y, int f, struct FloatVect3 *vec);
void undistort_fisheye_point(int point_x, int point_y, float *undistorted_x, float *undistorted_y, int f, float k, float x_img_center, float y_img_center);
void back_proj_points(struct FloatVect3 *gate_point, struct FloatVect3 *cam_pos, struct FloatMat33 *R_mat, float *x_res, float *y_res);
float euclidean_distance(float x_i, float x_bp, float y_i, float y_bp);
int find_minimum(float *error);

void smooth_hist(int *smooth, int *raw_hist, int window);
int find_max_hist(int *hist);
int find_hist_peeks(float *hist,int *peeks);
int find_hist_peeks_flat(int *hist,int *peeks);
void print_hist(struct image_t *img,int *hist);
void print_sides(struct image_t *im, int side_1, int side_2);
float detect_gate_sides(int *hist_raw, int *side_1, int *side_2);

void print_matrix(struct FloatMat33 mat);
void print_vector(struct FloatVect3 vec);
void draw_cross(struct image_t *im,int x, int y, uint8_t* color);

void check_color_center(struct image_t *im, uint8_t *y_c, uint8_t *cb_c, uint8_t *cr_c);
uint16_t image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y);

extern void snake_gate_periodic(void);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern int color_count;

//static void snake_gate_send(struct transport_tx *trans, struct link_device *dev);

//uint16_t image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y);
//void calculate_gate_position(int x_pix,int y_pix, int sz_pix, struct image_t *img,struct gate_img gate);
//void snake_gate_periodic(void);


extern struct video_listener *listener;

extern float measured_x_gate;
extern float measured_y_gate;
extern float measured_z_gate;
extern float delta_z_gate;

extern float x_dist;

extern float current_x_gate;
extern float current_y_gate;
extern float current_z_gate;
extern char fitness;
extern int gate_detected;
extern int init_pos_filter;

extern float p3p_result_x;
extern float p3p_result_y;
extern float p3p_result_z;
extern float p3p_result_phi;
extern float p3p_result_theta;
extern float p3p_result_psi;
extern float snake_res_x;
extern float snake_res_y;
extern float snake_res_z;

//logging corner points in image plane
extern float gate_img_point_x_1;
extern float gate_img_point_y_1;
extern float gate_img_point_x_2;
extern float gate_img_point_y_2;
extern float gate_img_point_x_3;
extern float gate_img_point_y_3;
extern float gate_img_point_x_4;
extern float gate_img_point_y_4;

//least squares final results
extern float ls_pos_x;
extern float ls_pos_y;
extern float ls_pos_z;

//hist final approach
extern float x_pos_hist;
extern float y_pos_hist;

//final KF results
extern float kf_pos_x;
extern float kf_pos_y;
extern float kf_vel_x;
extern float kf_vel_y;

//quality check of best gate
extern float gate_quality;

//Special vector vector operation
/* multiply _vin by _mat, store in _vout */
#define VECT3_VECT3_TRANS_MUL(_mat, _v_a,_v_b) {    \
    MAT33_ELMT((_mat),0,0) = (_v_a).x*(_v_b).x;  \
    MAT33_ELMT((_mat),0,1) = (_v_a).x*(_v_b).y;   \
    MAT33_ELMT((_mat),0,2) = (_v_a).x*(_v_b).z;   \
    MAT33_ELMT((_mat),1,0) = (_v_a).y*(_v_b).x;   \
    MAT33_ELMT((_mat),1,1) = (_v_a).y*(_v_b).y;   \
    MAT33_ELMT((_mat),1,2) = (_v_a).y*(_v_b).z;   \
    MAT33_ELMT((_mat),2,0) = (_v_a).z*(_v_b).x;   \
    MAT33_ELMT((_mat),2,1) = (_v_a).z*(_v_b).y;  \
    MAT33_ELMT((_mat),2,2) = (_v_a).z*(_v_b).z;   \
  }

#define MAT33_MAT33_DIFF(_mat1,_mat2,_mat3) {     \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat2),0,0)-MAT33_ELMT((_mat3),0,0);  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat2),0,1)-MAT33_ELMT((_mat3),0,1);  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat2),0,2)-MAT33_ELMT((_mat3),0,2);  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat2),1,0)-MAT33_ELMT((_mat3),1,0);  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat2),1,1)-MAT33_ELMT((_mat3),1,1);  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat2),1,2)-MAT33_ELMT((_mat3),1,2);  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat2),2,0)-MAT33_ELMT((_mat3),2,0);  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat2),2,1)-MAT33_ELMT((_mat3),2,1);  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat2),2,2)-MAT33_ELMT((_mat3),2,2);  \
  }
  
#define MAT33_MAT33_SUM(_mat1,_mat2,_mat3) {     \
    MAT33_ELMT((_mat1),0,0) = MAT33_ELMT((_mat2),0,0)+MAT33_ELMT((_mat3),0,0);  \
    MAT33_ELMT((_mat1),0,1) = MAT33_ELMT((_mat2),0,1)+MAT33_ELMT((_mat3),0,1);  \
    MAT33_ELMT((_mat1),0,2) = MAT33_ELMT((_mat2),0,2)+MAT33_ELMT((_mat3),0,2);  \
    MAT33_ELMT((_mat1),1,0) = MAT33_ELMT((_mat2),1,0)+MAT33_ELMT((_mat3),1,0);  \
    MAT33_ELMT((_mat1),1,1) = MAT33_ELMT((_mat2),1,1)+MAT33_ELMT((_mat3),1,1);  \
    MAT33_ELMT((_mat1),1,2) = MAT33_ELMT((_mat2),1,2)+MAT33_ELMT((_mat3),1,2);  \
    MAT33_ELMT((_mat1),2,0) = MAT33_ELMT((_mat2),2,0)+MAT33_ELMT((_mat3),2,0);  \
    MAT33_ELMT((_mat1),2,1) = MAT33_ELMT((_mat2),2,1)+MAT33_ELMT((_mat3),2,1);  \
    MAT33_ELMT((_mat1),2,2) = MAT33_ELMT((_mat2),2,2)+MAT33_ELMT((_mat3),2,2);  \
  }

//
// C = A+B
//
#define MAT_SUM(_i, _j, C, A, B) {              \
    int l,c;                                    \
    for (l=0; l<_i; l++)                        \
      for (c=0; c<_j; c++)                      \
        C[l][c] = A[l][c] + B[l][c];            \
  }

  
//
// C = A+(B*k)
//
#define MAT_SUM_c(_i, _j, C, A, B,k_) {              \
    int l,c;                                    \
    for (l=0; l<_i; l++)                        \
      for (c=0; c<_j; c++)                      \
        C[l][c] = A[l][c] + B[l][c]*k_;            \
  }
  
//
// C = c*A*B   A:(i,k) B:(k,j) C:(i,j)
//
#define MAT_MUL_c(_i, _k, _j, C, A, B,c_) {          \
    int l,c,m;                                  \
    for (l=0; l<_i; l++)                        \
      for (c=0; c<_j; c++) {                    \
        C[l][c] = 0.;                           \
        for (m=0; m<_k; m++)                    \
          C[l][c] += c_*A[l][m]*B[m][c];           \
      }                                         \
  }
  
  
#define MAT_PRINT(_i, _j,A) {           \
    int l,c;                            \
    printf(#A);				\
    printf("\n");			\
    for (l=0; l<_i; l++){               \
      for (c=0; c<_j; c++){             \
	  printf("%f,",A[l][c]);        \
        }				\
        printf("\n"); 			\
     }					\
  }

#endif /* SNAKE_GATE_DETECTION_CV_PLUGIN_H */
