/*
 * gate_detection.h
 *
 *  Created on: Sep 5, 2016
 *      Author: Guido de Croon
 */

#ifndef GATE_DETECTION_H_
#define GATE_DETECTION_H_

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/snake_gate_detection.h"

#define GOOD_FIT 0.04
#define BAD_FIT 0.12
#define MAX_POINTS 250

// floating point points are used in the fit.
struct point_f {
  float x;
  float y;
};

// main gate detection function:
extern void gate_detection_free(struct image_t* color_image, int *x_points, int *y_points, float* x_center, float* y_center, float* radius, float* fitness, float* x0, float* y0, float* size0,
                    uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, int clock_arms, float* angle_1, float* angle_2, float* psi, float* size_left, float* size_right);

// "private" functions:
void convert_image_to_points(struct image_t* color_image, uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y);
void fit_window_to_points(float *points_x, float *points_y, float* x0, float* y0, float* size0, float* x_center, float* y_center, float* radius, float* fitness, float* s_left, float* s_right);
float fit_clock_arms(float x_center, float y_center, float radius, float* angle_1, float* angle_2);
float mean_distance_to_circle(float* genome);
float mean_distance_to_square(float* genome);
float mean_distance_to_polygon(float* genome);
float mean_distance_to_polygon_free(float* genome);
float mean_distance_to_arms(float* genome, float x, float y);
float distance_to_line(struct point_f Q1, struct point_f Q2, struct point_f P);
float distance_to_segment(struct point_f Q1, struct point_f Q2, struct point_f P);
float distance_to_vertical_segment(struct point_f Q1, struct point_f Q2, struct point_f P);
float distance_to_horizontal_segment(struct point_f Q1, struct point_f Q2, struct point_f P);
float get_angle_from_polygon(float s_left, float s_right, struct image_t* color_image);

// utility functions: should probably be placed in some other file:
float get_random_number(void);
float get_minimum(float* nums, int n_elements, int *index);
float get_sum(float* nums, int n_elements);


// drawing functions:
void draw_circle(struct image_t* Im, float x_center, float y_center, float radius, uint8_t* color);
void draw_stick(struct image_t* Im, float x_center, float y_center, float radius, uint8_t* color);
void draw_line_segment(struct image_t* Im, struct point_f Q1, struct point_f Q2, uint8_t* color);

// calculating the color fit cannot be done with the current stereo output:
// float check_color_fit();

#endif /* GATE_DETECTION_H_ */


