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


#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

struct opengate_img {
	int x;            //central bar x location
	int y_l;          //lower y pixel
	int y_h;          //upper y pixel
	int size_bar;
	//int x_inner[4];
	//int y_inner[4];
	int x_corners[4]; // left bottom, left top, right bottom, right top
	int y_corners[4];
	int found[4];
	int n_bars;       // number of detected bars
	float opengate_q; // quality of open gate
};

// Module functions
int check_color(struct image_t *im, int x, int y);
void snake_up_and_down(struct image_t *im, int x, int y, int* y_low, int* y_high);
void snake_left_and_right(struct image_t *im, int x, int y, int* x_low, int* x_high);
void snake_left_and_right_new(struct image_t *im, int x, int y, int *x_low, int *x_high, int y_result[]);
void check_opengate(struct image_t *im, struct opengate_img opengate, float *quality, int *n_bars);
int check_back_side_QR_code(struct image_t* im, struct gate_img best_gate);
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int* n_points, int* n_colored_points);

void vec_from_point(float point_x, float point_y, int f, struct FloatVect3 *vec);
void vec_from_point_2(float point_x, float point_y, int f, struct FloatVect3 *vec);
void vec_from_point_ned(float point_x, float point_y, int f, struct FloatVect3 *vec);
void undistort_fisheye_point(int point_x, int point_y, float *undistorted_x, float *undistorted_y, int f, float k, float x_img_center, float y_img_center);
void back_proj_points(struct FloatVect3 *gate_point, struct FloatVect3 *cam_pos, struct FloatMat33 *R_mat, float *x_res, float *y_res);
float euclidean_distance(float x_i, float x_bp, float y_i, float y_bp);
int find_minimum(float *error);

extern int open_gate_processing(struct image_t *img,float *o_pos_x, float *o_pos_y, float *o_pos_z);


uint8_t color_lum_min;
uint8_t color_lum_max;

uint8_t color_cb_min;
uint8_t color_cb_max;

uint8_t color_cr_min;
uint8_t color_cr_max;

int color_count;

//first stretch histogram testing
extern int center_p;
extern int peek_height_o;
extern float side_angle_o;

//static void snake_gate_send(struct transport_tx *trans, struct link_device *dev);

//uint16_t image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y);
//void calculate_gate_position(int x_pix,int y_pix, int sz_pix, struct image_t *img,struct gate_img gate);
//void snake_gate_periodic(void);



