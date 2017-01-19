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
extern int check_back_side_QR_code(struct image_t* im, struct gate_img best_gate);
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int* n_points, int* n_colored_points);

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

extern float current_x_gate;
extern float current_y_gate;
extern float current_z_gate;
extern char fitness;
extern int gate_detected;
extern int init_pos_filter;

#endif /* SNAKE_GATE_DETECTION_CV_PLUGIN_H */
