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
 * @file modules/computer_vision/snake_gate_detection.c
 */

// Own header
#include "modules/computer_vision/snake_gate_detection.h"
#include <stdio.h>
#include "modules/computer_vision/lib/vision/image.h"
#include <stdlib.h>
#include "subsystems/datalink/telemetry.h"
//#include "modules/computer_vision/lib/vision/gate_detection.h"
#include "modules/computer_vision/lib/vision/gate_detection_free.h"
#include "state.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/lib/vision/qr_code_recognition.h"
#include "modules/computer_vision/lib/vision/calc_p3p.h"

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

#define PI 3.1415926

//initial position after gate pass
#define INITIAL_X 0
#define INITIAL_Y 2
#define INITIAL_Z -1.5

//initial position and speed safety margins

#define X_POS_MARGIN 0.20//m
#define Y_POS_MARGIN 0.5//m
#define Z_POS_MARGIN 0.2//m
#define X_SPEED_MARGIN 0.15//m/s
#define Y_SPEED_MARGIN 0.2//m/s

#define GOOD_FIT 1.0


struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 60;//105;
uint8_t color_lum_max = 228;//205;
uint8_t color_cb_min  = 66;//52;
uint8_t color_cb_max  = 194;//140;

uint8_t color_cr_min  = 139;//138;//146;//was 180

uint8_t color_cr_max  = 230;//255;

// Gate detection settings:
int n_samples = 1000;//1000;//500;
int min_pixel_size = 20;//40;//100;
float min_gate_quality = 0.20;//0.2;
float gate_thickness = 0;//0.05;//0.10;//
float gate_size = 34;

//color I dont know
uint8_t green_color[4] = {255,128,255,128}; //{0,250,0,250};
uint8_t blue_color[4] = {0,128,0,128};//{250,250,0,250};

int y_low = 0;
int y_high = 0;
int x_low1 = 0;
int x_high1 = 0;
int x_low2 = 0;
int x_high2 = 0;
int sz = 0;
int szx1 = 0;
int szx2 = 0;


// Result
int color_count = 0;
#define MAX_GATES 50
struct gate_img gates[MAX_GATES];
struct gate_img best_gate;
struct gate_img temp_check_gate;
struct image_t img_result;
int n_gates = 0;
float best_quality = 0;
float current_quality = 0;
float best_fitness = 100000;
float psi_gate = 0;
float size_left = 0;
float size_right = 0;
//color picker
uint8_t y_center_picker  = 0;
uint8_t cb_center  = 0;
uint8_t cr_center  = 0;

//camera parameters
#define radians_per_pix_w 0.006666667//2.1 rad(60deg)/315
#define radians_per_pix_h 0.0065625 //1.05rad / 160

//pixel distance conversion
int pix_x = 0;
int pix_y = 0;
int pix_sz = 0;
float hor_angle = 0;
float vert_angle = 0;
float x_dist = 0;
float y_dist = 0;
float z_dist = 0;

//state filter
float body_v_x = 0;
float body_v_y = 0;

float body_filter_x = 0;
float body_filter_y = 0;

float predicted_x_gate = 0;
float predicted_y_gate = 0;
float predicted_z_gate = 0;

//float current_x_gate = 0;
//float current_y_gate = 0;
//float current_z_gate = 0;
float delta_z_gate   = 0;

float previous_x_gate = 0;
float previous_y_gate = 0;
float previous_z_gate = 0;

// previous best gate:
struct gate_img previous_best_gate;

//SAFETY AND RESET FLAGS
int uncertainty_gate = 0;
//int gate_detected = 0;
//init_pos_filter = 0;
int safe_pass_counter = 0;
int gate_gen = 0;

float gate_quality = 0;

float fps_filter = 0;

struct timeval stop, start;

//QR code classification
int QR_class = 0;
float QR_uncertainty;

// back-side of a gate:
int back_side = 0;

//Debug messages

//free polygon points
int points_x[4];
int points_y[4];

int gates_sz = 0;

float x_center, y_center, radius;//,fitness, angle_1, angle_2, s_left, s_right;


//vectors for p3p etc
struct FloatVect3 vec_point_1, vec_point_2, vec_point_3, vec_point_4, vec_temp1, vec_temp2, vec_temp3, vec_temp4,
vec_ver_1, vec_ver_2, vec_ver_3, vec_ver_4;

struct FloatVect3 gate_vectors[4];
struct FloatVect3 gate_points[4];
struct FloatVect3 p3p_pos_solution;

//debugging
float debug_1 = 1.1;
float debug_2 = 2.2;
float debug_3 = 3.3;


//optic flow dummy
float opt_body_v_x = 0;
float opt_body_v_y = 0;

static void snake_gate_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SNAKE_GATE_INFO(trans, dev, AC_ID, &pix_x, &pix_y, &pix_sz, &hor_angle, &vert_angle, &x_dist, &y_dist,
                                &z_dist,
                                &current_x_gate, &current_y_gate, &current_z_gate, &debug_1, &debug_2,
                                &y_center_picker, &cb_center, &QR_class, &sz, &states_race.ready_pass_through, &temp_check_gate.x_corners[1],
                                &debug_3);//psi_gate); //
}


// Checks for a single pixel if it is the right color
// 1 means that it passes the filter
int check_color(struct image_t *im, int x, int y)
{
  // if (x % 2 == 1) { x--; }
  if (y % 2 == 1) { y--; }
  
  // if (x < 0 || x >= im->w || y < 0 || y >= im->h) {
  if (x < 0 || x >= im->h || y < 0 || y >= im->w) {
    return 0;
  }

  uint8_t *buf = im->buf;
  // buf += 2 * (y * (im->w) + x); // each pixel has two bytes
  buf += 2 * (x * (im->w) + y); // each pixel has two bytes
  // odd ones are uy
  // even ones are vy


  if (
    (buf[1] >= color_lum_min)
    && (buf[1] <= color_lum_max)
    && (buf[0] >= color_cb_min)
    && (buf[0] <= color_cb_max)
    && (buf[2] >= color_cr_min)
    && (buf[2] <= color_cr_max)
  ) {
    // the pixel passes:
    return 1;
  } else {
    // the pixel does not:
    return 0;
  }
}

void check_color_center(struct image_t *im, uint8_t *y_c, uint8_t *cb_c, uint8_t *cr_c)
{
  uint8_t *buf = im->buf;
  // int x = (im->w) / 2;
  // int y = (im->h) / 2;
  int x = (im->h) / 2;
  int y = (im->w) / 2;
  // buf += y * (im->w) * 2 + x * 2;
  buf += x * (im->w) * 2 + y * 2;

  *y_c = buf[1];
  *cb_c = buf[0];
  *cr_c = buf[2];
}


//set color pixel
uint16_t image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;
  // if (x % 2 == 1) { x--; }
  if (y % 2 == 1) { y--; }

  if (x < 0 || x >= input->w || y < 0 || y >= input->h) {
    return;
  }


  /*
  source += y * (input->w) * 2 + x * 2;
  dest += y * (output->w) * 2 + x * 2;
  */
  source += x * (input->w) * 2 + y * 2;
  dest += x * (output->w) * 2 + y * 2;
  // UYVY
  dest[0] = 65;//211;        // U//was 65
  dest[1] = source[1];  // Y
  dest[2] = 255;//60;        // V//was 255
  dest[3] = source[3];  // Y
}

void calculate_gate_position(int x_pix, int y_pix, int sz_pix, struct image_t *img, struct gate_img gate)
{
  float hor_calib = 0.075;
  //calculate angles here  
  /*
  vert_angle = (-(((float)x_pix * 1.0) - ((float)(img->w) / 2.0)) * radians_per_pix_w) -
               (stateGetNedToBodyEulers_f()->theta);
  hor_angle = ((((float)y_pix * 1.0) - ((float)(img->h) / 2.0)) * radians_per_pix_h) + hor_calib;
  */
  vert_angle = (-(((float)y_pix * 1.0) - ((float)(img->w) / 2.0)) * radians_per_pix_h) -
               (stateGetNedToBodyEulers_f()->theta);
  hor_angle = ((((float)x_pix * 1.0) - ((float)(img->h) / 2.0)) * radians_per_pix_w) + hor_calib;

  pix_x = x_pix;
  pix_y = y_pix;
  pix_sz = gate.sz;
  current_quality = gate.gate_q;

  if (gate_size == 0) {
    gate_size = 1;
  }

  float gate_size_m = tan(((float)gate_size / 2.0) * radians_per_pix_w) * 3.0;
  y_dist = gate_size_m / tan((pix_sz / 2) * radians_per_pix_w);
  x_dist = y_dist * sin(hor_angle);
  z_dist = y_dist * sin(vert_angle);

}

//state filter in periodic loop
void snake_gate_periodic(void)
{
  //SAFETY  gate_detected
  if (y_dist > 0.6 && y_dist < 4) { // && gate_gen == 1)
    states_race.gate_detected = 1;
    counter_gate_detected = 0;
    time_gate_detected = 0;
  } else {
    states_race.gate_detected = 0;
    counter_gate_detected = 0;
    time_gate_detected = 0;
  }

  //SAFETY ready_pass_trough
  if (states_race.gate_detected == 1 && fabs(x_dist - INITIAL_X) < X_POS_MARGIN && fabs(y_dist - INITIAL_Y) < Y_POS_MARGIN
      && fabs(z_dist - 0) < Z_POS_MARGIN && fabs(opt_body_v_x) < Y_SPEED_MARGIN
      && fabs(opt_body_v_x) < Y_SPEED_MARGIN) {
    safe_pass_counter += 1;
  } else {
    safe_pass_counter = 0;
    states_race.ready_pass_through = 0;
  }

  if (safe_pass_counter > 2) {
    safe_pass_counter = 0;
    states_race.ready_pass_through = 1;
  }

  // Reinitialization after gate is cleared and turn is made(called from velocity guidance module)
  if (init_pos_filter == 1) {
    init_pos_filter = 0;
    //assumed initial position at other end of the gate
    //predicted_x_gate = INITIAL_X;//0;
    //predicted_y_gate = INITIAL_Y;//1.5;
      current_x_gate = INITIAL_X;//0;
      current_y_gate = INITIAL_Y;
      current_z_gate = INITIAL_Z;

  }

  //State filter


  //convert earth velocity to body x y velocity
  float v_x_earth = stateGetSpeedNed_f()->x;
  float v_y_earth = stateGetSpeedNed_f()->y;
  float psi = stateGetNedToBodyEulers_f()->psi;
  //When using optitrack
  //body_v_x = cosf(psi)*v_x_earth + sinf(psi)*v_y_earth;
  //body_v_y = -sinf(psi)*v_x_earth+cosf(psi)*v_y_earth;

  body_v_x = opt_body_v_x;
  body_v_y = opt_body_v_y;

  //body velocity in filter frame
  body_filter_x = -body_v_y;
  body_filter_y = -body_v_x;

  gettimeofday(&stop, 0);
  double curr_time = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
  double elapsed = curr_time - (double)(start.tv_sec + start.tv_usec / 1000000.0);
  gettimeofday(&start, 0);
  float dt = elapsed;

  fps_filter = (float)1.0 / dt;

  // predict the new location:
  float dx_gate = dt * body_filter_x;//(cos(current_angle_gate) * gate_turn_rate * current_distance);
  float dy_gate = dt * body_filter_y; //(velocity_gate - sin(current_angle_gate) * gate_turn_rate * current_distance);
  predicted_x_gate = previous_x_gate + dx_gate;
  predicted_y_gate = previous_y_gate + dy_gate;
  predicted_z_gate = previous_z_gate;

  float sonar_alt = stateGetPositionNed_f()->z;

  if (states_race.gate_detected == 1) {

    // Mix the measurement with the prediction:
    float weight_measurement;
    if (uncertainty_gate > 150) {
      weight_measurement = 1.0f;
      uncertainty_gate = 151;//max: does not matter - is replaced below
    } else {
      weight_measurement = 0.7;  //(GOOD_FIT-best_quality)/GOOD_FIT;//check constant weight
    }

     float z_weight = 0.2;

    current_x_gate = weight_measurement * x_dist + (1.0f - weight_measurement) * predicted_x_gate;
    current_y_gate = weight_measurement * y_dist + (1.0f - weight_measurement) * predicted_y_gate;
    current_z_gate = z_weight * (z_dist + sonar_alt) + (1.0f - z_weight) * predicted_z_gate;

    //psi_bias
    //if state is adjust position then slowly add bias using the fitness as weight
    //keep always updating bias based on current angle and limit
    //psi_filter_weight = GOOD_POLY_FIT - best_fitness;
    //psi_increment = psi_filter_weight * psi_gate;

    // reset uncertainty:
    uncertainty_gate = 0;
  } else {
    // just the prediction
    current_x_gate = predicted_x_gate;
    current_y_gate = predicted_y_gate;
    current_z_gate = predicted_z_gate;

    // increase uncertainty
    uncertainty_gate++;
  }
  // set the previous state for the next time:
  previous_x_gate = current_x_gate;
  previous_y_gate = current_y_gate;
  previous_z_gate = current_z_gate;
  delta_z_gate = current_z_gate - sonar_alt;
}

// Function
// Samples from the image and checks if the pixel is the right color.
// If yes, it "snakes" up and down to see if it is the side of a gate.
// If this stretch is long enough, it "snakes" also left and right.
// If the left/right stretch is also long enough, add the coords as a
// candidate square, optionally drawing it on the image.
struct image_t *snake_gate_detection_func(struct image_t *img);
struct image_t *snake_gate_detection_func(struct image_t *img)
{
  int filter = 0;
  int gen_alg = 1;
  uint16_t i;
  int x, y;//, y_low, y_high, x_low1, x_high1, x_low2, x_high2, sz, szx1, szx2;
  float quality;
  struct point_t from, to;
  best_quality = 0;
  best_gate.gate_q = 0;
  //test
  //pix_x = img->w;
  //pix_y = img->h;



  n_gates = 0;

  //color picker
  //check_color_center(img,&y_center_picker,&cb_center,&cr_center);

  for (i = 0; i < n_samples; i++) {
    // get a random coordinate:
    x = rand() % img->h;
    y = rand() % img->w;

    //check_color(img, 1, 1);
    // check if it has the right color
    if (check_color(img, x, y)) {
      // snake up and down:
      snake_up_and_down(img, x, y, &y_low, &y_high);
      sz = y_high - y_low;

      y_low = y_low + (sz * gate_thickness);
      y_high = y_high - (sz * gate_thickness);

      y = (y_high + y_low) / 2;

      // if the stretch is long enough
      if (sz > min_pixel_size) {
        // snake left and right:
        snake_left_and_right(img, x, y_low, &x_low1, &x_high1);
        snake_left_and_right(img, x, y_high, &x_low2, &x_high2);

        x_low1 = x_low1 + (sz * gate_thickness);
        x_high1 = x_high1 - (sz * gate_thickness);
        x_low2 = x_low2 + (sz * gate_thickness);
        x_high2 = x_high2 - (sz * gate_thickness);

        // sizes of the left-right stretches: in y pixel coordinates
        szx1 = (x_high1 - x_low1);
        szx2 = (x_high2 - x_low2);

        // if the size is big enough:
        if (szx1 > min_pixel_size) {
          // draw four lines on the image:
          x = (x_high1 + x_low1) / 2;//was+
          // set the size to the largest line found:
          sz = (sz > szx1) ? sz : szx1;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &quality, &gates[n_gates].n_sides);
          gates[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (quality > best_quality) { //min_gate_quality)
          //draw_gate(img, gates[n_gates]);
          best_quality = quality;
          n_gates++;
        }
      } else if (szx2 > min_pixel_size) {
          x = (x_high2 + x_low2) / 2;//was +
          // set the size to the largest line found:
          sz = (sz > szx2) ? sz : szx2;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &quality, &gates[n_gates].n_sides);
          gates[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (quality > best_quality) { //min_gate_quality)
            //draw_gate(img, gates[n_gates]);
            best_quality = quality;
            n_gates++;
          }
        }
        if (n_gates >= MAX_GATES) {
          break;
        }

      }
      //
    }

  }

  // variables used for fitting:
  //float x_center, y_center, radius,
  float fitness, angle_1, angle_2, s_left, s_right;
  int clock_arms = 1;
  // prepare the Region of Interest (ROI), which is larger than the gate:
  float size_factor = 1.5;//2;//1.25;

  // do an additional fit to improve the gate detection:
  if (best_quality > min_gate_quality && n_gates > 0) {
    // temporary variables:


    //if (gen_alg) {

      int max_candidate_gates = 1;//10;

      best_fitness = 100;
      if (n_gates > 0 && n_gates < max_candidate_gates) {
        for (int gate_nr = 0; gate_nr < n_gates; gate_nr += 1) {
          int16_t ROI_size = (int16_t)(((float) gates[gate_nr].sz) * size_factor);
          int16_t min_x = gates[gate_nr].x - ROI_size;
          min_x = (min_x < 0) ? 0 : min_x;
          int16_t max_x = gates[gate_nr].x + ROI_size;
          max_x = (max_x < img->h) ? max_x : img->h;
          int16_t min_y = gates[gate_nr].y - ROI_size;
          min_y = (min_y < 0) ? 0 : min_y;
          int16_t max_y = gates[gate_nr].y + ROI_size;
          max_y = (max_y < img->w) ? max_y : img->w;

          //draw_gate(img, gates[gate_nr]);

          int gates_x = gates[gate_nr].x;
          int gates_y = gates[gate_nr].y;
          gates_sz = gates[gate_nr].sz;
	  
	  x_center = gates_x;
	  y_center = gates_y;
	  radius   = gates_sz;
          // detect the gate:
          gate_detection_free(img, points_x, points_y, &x_center, &y_center, &radius, &fitness, &gates_x, &gates_y, &gates_sz,
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi_gate,
                         &s_left, &s_right);
//draw_gate_polygon(img,points_x,points_y,blue_color);
          //if (fitness < best_fitness) {
            //best_fitness = fitness;
            // store the information in the gate:
            temp_check_gate.x = (int) x_center;
            temp_check_gate.y = (int) y_center;
            temp_check_gate.sz = (int) radius;
            temp_check_gate.sz_left = (int) s_left;
            temp_check_gate.sz_right = (int) s_right;
	    memcpy(&(temp_check_gate.x_corners[0]),points_x,sizeof(int)*4);
	    memcpy(&(temp_check_gate.y_corners[0]),points_y,sizeof(int)*4);
	    
            // also get the color fitness
            check_gate_free(img, temp_check_gate, &temp_check_gate.gate_q, &temp_check_gate.n_sides);
	    
	    if(temp_check_gate.n_sides > 2 && temp_check_gate.gate_q > best_gate.gate_q)
	    {
	    best_fitness = fitness;
	    //best_quality = .gate_q(first maybe zero
            // store the information in the gate:
            best_gate.x = temp_check_gate.x;
            best_gate.y = temp_check_gate.y;
            best_gate.sz = temp_check_gate.sz;
            best_gate.sz_left = temp_check_gate.sz_left;
            best_gate.sz_right = temp_check_gate.sz_right;
	    best_gate.gate_q = temp_check_gate.gate_q;
	    best_gate.n_sides = temp_check_gate.n_sides;
	    memcpy(&(best_gate.x_corners[0]),&(temp_check_gate.x_corners[0]),sizeof(int)*4);
	    memcpy(&(best_gate.y_corners[0]),&(temp_check_gate.y_corners[0]),sizeof(int)*4);
	    }
          //}

        }
        /*for (int gate_nr = 0; gate_nr < n_gates; gate_nr += 1) {
          draw_gate(img, gates[gate_nr]);
        }*/
      } else if (n_gates >= max_candidate_gates) {
        for (int gate_nr = n_gates - max_candidate_gates; gate_nr < n_gates; gate_nr += 1) {
          int16_t ROI_size = (int16_t)(((float) gates[gate_nr].sz) * size_factor);
          int16_t min_x = gates[gate_nr].x - ROI_size;
          min_x = (min_x < 0) ? 0 : min_x;
          int16_t max_x = gates[gate_nr].x + ROI_size;
          max_x = (max_x < img->h) ? max_x : img->h;
          int16_t min_y = gates[gate_nr].y - ROI_size;
          min_y = (min_y < 0) ? 0 : min_y;
          int16_t max_y = gates[gate_nr].y + ROI_size;
          max_y = (max_y < img->w) ? max_y : img->w;
          //draw_gate(img, gates[gate_nr]);
	  gates_sz = gates[gate_nr].sz;
	  x_center = gates[gate_nr].x;
	  y_center = gates[gate_nr].y;
	  radius   = gates_sz;
          // detect the gate:
          gate_detection_free(img, points_x, points_y, &x_center, &y_center, &radius, &fitness, &(gates[gate_nr].x), &(gates[gate_nr].y),
                         &(gates[gate_nr].sz),
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi_gate,
                         &s_left, &s_right);
	  //draw_gate_polygon(img,points_x,points_y,blue_color);
          //if (fitness < best_fitness) {
            //best_fitness = fitness;
            // store the information in the gate:
            temp_check_gate.x = (int) x_center;
            temp_check_gate.y = (int) y_center;
            temp_check_gate.sz = (int) radius;
            temp_check_gate.sz_left = (int) s_left;
            temp_check_gate.sz_right = (int) s_right;
	    
	    memcpy(&(temp_check_gate.x_corners[0]),points_x,sizeof(int)*4);
	    memcpy(&(temp_check_gate.y_corners[0]),points_y,sizeof(int)*4);
	    
	    
            // also get the color fitness
            check_gate_free(img, temp_check_gate, &temp_check_gate.gate_q, &temp_check_gate.n_sides);
	    
	    if(temp_check_gate.n_sides > 2 && temp_check_gate.gate_q > best_gate.gate_q)
	    {
	    //best_fitness = fitness;
            // store the information in the gate:
            best_gate.x = temp_check_gate.x;
            best_gate.y = temp_check_gate.y;
            best_gate.sz = temp_check_gate.sz;
            best_gate.sz_left = temp_check_gate.sz_left;
            best_gate.sz_right = temp_check_gate.sz_right;
	    best_gate.gate_q = temp_check_gate.gate_q;
	    best_gate.n_sides = temp_check_gate.n_sides;
	    memcpy(&(best_gate.x_corners[0]),&(temp_check_gate.x_corners[0]),sizeof(int)*4);
	    memcpy(&(best_gate.y_corners[0]),&(temp_check_gate.y_corners[0]),sizeof(int)*4);
	    }
         // }
        }

        /*for (int gate_nr = n_gates - max_candidate_gates; gate_nr < n_gates; gate_nr += 1) {
          draw_gate(img, gates[gate_nr]);
        }*/

      }
   
  } else {
    //random position guesses here and then genetic algorithm
    //use random sizes and positions bounded by minimum size

  }

  /*
  * What is better? The current or previous gate?
  */
  if(previous_best_gate.sz != 0 && best_quality > min_gate_quality && n_gates > 0)
  {
    // refit the previous best gate in the current image and compare the quality:
    int16_t ROI_size = (int16_t)(((float) previous_best_gate.sz) * size_factor);
    int16_t min_x = previous_best_gate.x - ROI_size;
    min_x = (min_x < 0) ? 0 : min_x;
    int16_t max_x = previous_best_gate.x + ROI_size;
    max_x = (max_x < img->h) ? max_x : img->h;
    int16_t min_y = previous_best_gate.y - ROI_size;
    min_y = (min_y < 0) ? 0 : min_y;
    int16_t max_y = previous_best_gate.y + ROI_size;
    max_y = (max_y < img->w) ? max_y : img->w;

    // detect the gate:
    gate_detection_free(img, points_x, points_y, &x_center, &y_center, &radius, &fitness, &(previous_best_gate.x), &(previous_best_gate.y),
                   &(previous_best_gate.sz),
                   (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi_gate,
                   &s_left, &s_right);
//draw_gate_polygon(img,points_x,points_y,blue_color);
    // store the information in the gate:
    previous_best_gate.x = (int) x_center;
    previous_best_gate.y = (int) y_center;
    previous_best_gate.sz = (int) radius;
    previous_best_gate.sz_left = (int) s_left;
    previous_best_gate.sz_right = (int) s_right;

    // also get the color fitness
    check_gate_free(img, previous_best_gate, &previous_best_gate.gate_q, &previous_best_gate.n_sides);

    // if the quality of the "old" gate is better, keep the old gate:
    if(previous_best_gate.gate_q > best_gate.gate_q &&  previous_best_gate.n_sides > 2)//n_sides
    {
      best_gate.x = previous_best_gate.x;
      best_gate.y = previous_best_gate.y;
      best_gate.sz = previous_best_gate.sz;
      best_gate.sz_left = previous_best_gate.sz_left;
      best_gate.sz_right = previous_best_gate.sz_right;
      best_gate.gate_q = previous_best_gate.gate_q;
      best_gate.n_sides = previous_best_gate.n_sides;
    }
  }

  // prepare for the next time:
  previous_best_gate.x = best_gate.x;
  previous_best_gate.y = best_gate.y;
  previous_best_gate.sz = best_gate.sz;
  previous_best_gate.sz_left = best_gate.sz_left;
  previous_best_gate.sz_right = best_gate.sz_right;
  previous_best_gate.gate_q = best_gate.gate_q;
  previous_best_gate.n_sides = best_gate.n_sides;
  
    //color filtered version of image for overlay and debugging
  if (filter) {
    int color_count = image_yuv422_colorfilt(img, img,
                      color_lum_min, color_lum_max,
                      color_cb_min, color_cb_max,
                      color_cr_min, color_cr_max
                                            );
  }

  /**************************************
  * BEST GATE -> TRANSFORM TO COORDINATES
  ***************************************/

  if (best_gate.gate_q > (min_gate_quality*2) && best_gate.n_sides > 3) {//n_sides was > 2

    current_quality = best_quality;
    size_left = best_gate.sz_left;
    size_right = best_gate.sz_right;
    

    gate_quality = best_gate.gate_q;
    

    //image_yuv422_set_color(img,img,gates[n_gates-1].x,gates[n_gates-1].y);

    //calculate_gate_position(gates[n_gates-1].x,gates[n_gates-1].y,gates[n_gates-1].sz,img,gates[n_gates-1]);
    calculate_gate_position(best_gate.x, best_gate.y, best_gate.sz, img, best_gate);

        gate_gen = 1;//0;
        states_race.gate_detected = 1;
        //draw_gate_color(img, best_gate, blue_color);
	//draw_gate_polygon(img,points_x,points_y,blue_color);
//draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,blue_color);
	
	//vector and matrix declarations
	  struct FloatVect3 gate_point_0,gate_point_1,gate_point_2,
	  p3p_pos_sol_0,p3p_pos_sol_1,p3p_pos_sol_2,p3p_pos_sol_3;
	  struct FloatMat33 R_mat_0,R_mat_1,R_mat_2,R_mat_3;
	  struct FloatEulers R_eulers;
	
	//DEBUG----------------------------------------------------------------
	
	  VECT3_ASSIGN(p3p_pos_sol_0, 2.295645,0.877159, -1.031926);
	  
// 	  MAT33_ELMT(R_mat_0, 0, 0) = 0.999867;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 0, 1) =  -0.015745;
// 	  MAT33_ELMT(R_mat_0, 0, 2) = -0.004201;
// 
// 	  MAT33_ELMT(R_mat_0, 1, 0) = 0.015464;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 1, 1) = 0.998070;
// 	  MAT33_ELMT(R_mat_0, 1, 2) = -0.060151;
// 
// 	  MAT33_ELMT(R_mat_0, 2, 0) = 0.005140;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 2, 1) = 0.060078;
// 	  MAT33_ELMT(R_mat_0, 2, 2) = 0.998180;
// 	
	  MAT33_ELMT(R_mat_0, 0, 0) = 1;//(row,column)
	  MAT33_ELMT(R_mat_0, 0, 1) = 0;
	  MAT33_ELMT(R_mat_0, 0, 2) = 0;

	  MAT33_ELMT(R_mat_0, 1, 0) = 0;//(row,column)
	  MAT33_ELMT(R_mat_0, 1, 1) = 1;
	  MAT33_ELMT(R_mat_0, 1, 2) = 0;

	  MAT33_ELMT(R_mat_0, 2, 0) = 0;//(row,column)
	  MAT33_ELMT(R_mat_0, 2, 1) = 0;
	  MAT33_ELMT(R_mat_0, 2, 2) = 1;
	  
	int x_bp_corners[4];
	int y_bp_corners[4];
	float x_f_corners[4];
	float y_f_corners[4];
	
	for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol_0,&R_mat_0,&x_bp,&y_bp);
	    x_f_corners[i] = x_bp;
	    y_f_corners[i] = y_bp;
// 	    debug_1 = x_bp;
// 	    debug_2 = y_bp;
	  }
	
	//DEBUG---------------------------------------------------------------
	
	//Undistort fisheye points
	int f_fisheye = 168;
	float k_fisheye = 1.085;
	for(int i = 0;i<4;i++)
	{
	  float undist_x, undist_y;
	  //debug_1 = (float)best_gate.x_corners[i];// best_gate.y_corners[i]
	  //debug_2 = (float)best_gate.y_corners[i];//
	  //undist_y = 5;//(float)best_gate.y_corners[i];
	  float princ_x = 157.0;
	  float princ_y = 32.0;
	  undistort_fisheye_point(best_gate.x_corners[i] ,best_gate.y_corners[i],&undist_x,&undist_y,f_fisheye,k_fisheye,princ_x,princ_y);
	  
// 	  draw_cross(img,((int)undist_x)+157,((int)undist_y)+32,green_color);
// 	  //debug_1 = undist_x;
// 	  //debug_2 = undist_y;
// 	  
// 	  vec_from_point_ned(undist_x, undist_y, f_fisheye,&gate_vectors[i]);
	  
	  draw_cross(img,(int)x_f_corners[i],(int)y_f_corners[i],green_color);
	  vec_from_point(x_f_corners[i],y_f_corners[i], f_fisheye,&gate_vectors[i]);
	  
	  
// 	  debug_1 = gate_vectors[i].x;
// 	  debug_2 = gate_vectors[i].y;
// 	  debug_3 = gate_vectors[i].z;
	  
	}
	
	 //p3p algorithm
	
	
	  VECT3_ASSIGN(gate_points[0], 4.2000,0.3000, -1.9000);
	  VECT3_ASSIGN(gate_points[1], 4.2000,1.3000, -1.9000);
	  VECT3_ASSIGN(gate_points[2], 4.2000,1.3000, -0.9000);
	  VECT3_ASSIGN(gate_points[3], 4.2000,0.3000, -0.9000);
	  
	  P3p_computePoses(&gate_points[0],&gate_points[1],&gate_points[2],
			   &gate_vectors[0],&gate_vectors[1],&gate_vectors[2],
			   &p3p_pos_sol_0,&p3p_pos_sol_1,&p3p_pos_sol_2,&p3p_pos_sol_3,
			   &R_mat_0,&R_mat_1,&R_mat_2,&R_mat_3);
	  
	  float_eulers_of_rmat(&R_eulers,&R_mat_0);
	  
	  //for all 4 solutions
	  //{
	  //proj_point(p3p_pos_sol_0,p3p_pos_sol_1,p3p_pos_sol_2,R_mat) //project 4 points based on position and rotation
	  //}
	  //best_solution_p3p()//sort error list and chose respective solution pos and R_mat
	  //R_mat_to_angle()//from paparazzi
	  
// 	  VECT3_ASSIGN(p3p_pos_sol_0, 1.5000,0.8000, -1.4000);
//	  VECT3_ASSIGN(p3p_pos_sol_0, 2.295645,0.877159, -1.031926);
	  
// 	  MAT33_ELMT(R_mat_0, 0, 0) = 1;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 0, 1) = 0;
// 	  MAT33_ELMT(R_mat_0, 0, 2) = 0;
// 
// 	  MAT33_ELMT(R_mat_0, 1, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 1, 1) = 1;
// 	  MAT33_ELMT(R_mat_0, 1, 2) = 0;
// 
// 	  MAT33_ELMT(R_mat_0, 2, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 2, 1) = 0;
// 	  MAT33_ELMT(R_mat_0, 2, 2) = 1;
	  
// 	  MAT33_ELMT(R_mat_0, 0, 0) = 0.999867;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 0, 1) =  -0.015745;
// 	  MAT33_ELMT(R_mat_0, 0, 2) = -0.004201;
// 
// 	  MAT33_ELMT(R_mat_0, 1, 0) = 0.015464;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 1, 1) = 0.998070;
// 	  MAT33_ELMT(R_mat_0, 1, 2) = -0.060151;
// 
// 	  MAT33_ELMT(R_mat_0, 2, 0) = 0.005140;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 2, 1) = 0.060078;
// 	  MAT33_ELMT(R_mat_0, 2, 2) = 0.998180;
	  
	  
	  /*0.999867, -0.015745, -0.004201
0.015464, 0.998070, -0.060151
0.005140, 0.060078, 0.998180
Position solution nr:1 /// x:2.295645 y:0.877159 z-1.031926*/
	  
	  	printf("R_mat_0:\n");
 		print_mat(R_mat_0);
		printf("Position solution nr:%d /// x:%f y:%f z%f\n",0,p3p_pos_sol_0.x,p3p_pos_sol_0.y,p3p_pos_sol_0.z);
		float_eulers_of_rmat(&R_eulers,&R_mat_0);
		printf("Eulers_0 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
		
		printf("R_mat_1:\n");
 		print_mat(R_mat_1);
		printf("Position solution nr:%d /// x:%f y:%f z%f\n",1,p3p_pos_sol_1.x,p3p_pos_sol_1.y,p3p_pos_sol_1.z);
		float_eulers_of_rmat(&R_eulers,&R_mat_1);
		printf("Eulers_1 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
		
		printf("R_mat_2:\n");
 		print_mat(R_mat_2);
		printf("Position solution nr:%d /// x:%f y:%f z%f\n",2,p3p_pos_sol_2.x,p3p_pos_sol_2.y,p3p_pos_sol_2.z);
		float_eulers_of_rmat(&R_eulers,&R_mat_2);
		printf("Eulers_2 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
		
		
		printf("R_mat_3:\n");
 		print_mat(R_mat_3);
		printf("Position solution nr:%d /// x:%f y:%f z%f\n",3,p3p_pos_sol_3.x,p3p_pos_sol_3.y,p3p_pos_sol_3.z);
		float_eulers_of_rmat(&R_eulers,&R_mat_3);
		printf("Eulers_3 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
		
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol_0,&R_mat_0,&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
// 	    debug_1 = x_bp;
// 	    debug_2 = y_bp;
	  }
	  draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol_1,&R_mat_1,&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	  }
	  draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol_2,&R_mat_2,&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	  }
	  draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol_3,&R_mat_3,&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	  }
	  draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  
// 	  debug_1 = p3p_pos_sol_0.x;
// 	  debug_2 = p3p_pos_sol_0.y;
// 	  debug_3 = p3p_pos_sol_0.z;
	  
// 	  debug_1 = p3p_pos_sol_1.x;
// 	  debug_2 = p3p_pos_sol_1.y;
// 	  debug_3 = p3p_pos_sol_1.z;
	
	    //error tests
	// reference vectors
	
// 	vec_ver_1.x = 200;
// 	vec_ver_1.y = -100;
// 	vec_ver_1.z = 72;
// 	
// 	vec_ver_2.x = 200;
// 	vec_ver_2.y = 0;
// 	vec_ver_2.z = 72;
// 	
// 	vec_ver_3.x = 200;
// 	vec_ver_3.y = 0;
// 	vec_ver_3.z = -28;
// 	
// 	vec_ver_4.x = 200;
// 	vec_ver_4.y = -100;
// 	vec_ver_4.z = -28;
// 	
// 	double dot = VECT3_DOT_PRODUCT(vec_ver_1, gate_vectors[0]);
// 	float norm_a = sqrtf(VECT3_NORM2(vec_ver_1));
// 	float norm_b = sqrtf(VECT3_NORM2(gate_vectors[0]));
// 	
// 	float angle_error_dot = acosf((float)dot/(norm_a*norm_b));
// 	debug_1 = angle_error_dot*57;
// 	
// 	 dot = VECT3_DOT_PRODUCT(vec_ver_2, gate_vectors[1]);
// 	 norm_a = sqrtf(VECT3_NORM2(vec_ver_2));
// 	 norm_b = sqrtf(VECT3_NORM2(gate_vectors[1]));
// 	
// 	 angle_error_dot = acosf((float)dot/(norm_a*norm_b));
// 	 debug_2 = angle_error_dot*57;
// 	 
// 	 dot = VECT3_DOT_PRODUCT(vec_ver_3, gate_vectors[2]);
// 	 norm_a = sqrtf(VECT3_NORM2(vec_ver_3));
// 	 norm_b = sqrtf(VECT3_NORM2(gate_vectors[2]));
// 	
// 	 angle_error_dot = acosf((float)dot/(norm_a*norm_b));
// 	 debug_3 = angle_error_dot*57;
	
	
	
    
  } else {

     
      states_race.gate_detected = 0;
      current_quality = 0;
      gate_gen = 1;
    
  }
  	//principal point
	//draw_cross(img,158,12,blue_color);
	
	//better principal point?
	draw_cross(img,158,32,green_color);
	
	
	
  return img; // snake_gate_detection did not make a new image
}

void undistort_fisheye_point(int point_x, int point_y, float *undistorted_x, float *undistorted_y, int f, float k, float x_img_center, float y_img_center)
{
  /*
f = 168;
k = 1.085;
%k = 1.051;
%k = 1.118;

*/
  
  float x_mid = (float)(point_x) - 157.0f;//-(float)(x_princip);
  float y_mid = (float)(point_y) - 32.0f;//-(float)(y_princip);
  
  //debug_1 = x_mid;
  //debug_2 = y_mid;
  
  //to polar coordinates
  float r = sqrtf((pow(x_mid,2))+(pow(y_mid,2)));
  float theta = atan2f(y_mid,x_mid);//atanf?
  
  //debug_1 = r;
  //debug_2 = theta;
  
  //debug_1 = k;
  //debug_2 = (float)f;
  //k = 1.085;
k = 1.051;
 // k = 1.118;
  
  //radial distortion correction
  float R = (float)f*tan(asin(sin( atan(r/(float)f))*k));
  
                                                  // +y
                                                  // ^
                                                  // |
                                                  // |
  *undistorted_x =  R * cos(theta);//+x_princip; in (0,0)--->+x
  *undistorted_y =  R * sin(theta);//+y_princip;
  
  
}

//calculate unit vector from undistorted image point.
void vec_from_point(float point_x, float point_y, int f, struct FloatVect3 *vec)
{
  
  vec->x = (float)f;
  vec->y = (float)point_x;
  vec->z = (float)point_y;
  
  double norm = sqrt(VECT3_NORM2(*vec));
  VECT3_SDIV(*vec, *vec, norm);
  
}

void vec_from_point_ned(float point_x, float point_y, int f, struct FloatVect3 *vec)
{
  
  vec->x = (float)f;
  vec->y = (float)point_x;
  vec->z = -(float)point_y;
  
  double norm = sqrt(VECT3_NORM2(*vec));
  VECT3_SDIV(*vec, *vec, norm);
  
}


void back_proj_points(struct FloatVect3 *gate_point, struct FloatVect3 *cam_pos, struct FloatMat33 *R_mat, float *x_res, float *y_res)
{
  
//   point_3d = R*(gate_point-cam_pos');
// 
//     hom_coord = [point_3d(2)/point_3d(1);
//                  point_3d(3)/point_3d(1);
//                  1];
// 
//     res = intr*hom_coord;
  
  struct FloatVect3 temp1, point_3d, hom_coord, res;
  struct FloatMat33 intr, R_t;
  
  VECT3_DIFF(temp1,*gate_point,*cam_pos);
  MAT33_TRANS(R_t,*R_mat);
  MAT33_VECT3_MUL(point_3d,R_t,temp1);
  hom_coord.x = point_3d.y/point_3d.x;
  hom_coord.y = -point_3d.z/point_3d.x;
  hom_coord.z = 1;
  
//   debug_1 = hom_coord.x;
//   debug_2 = hom_coord.y;
  
  MAT33_ELMT(intr, 0, 0) = 168;//(row,column)
  MAT33_ELMT(intr, 0, 1) = 0;
  MAT33_ELMT(intr, 0, 2) = 157;

  MAT33_ELMT(intr, 1, 0) = 0;//(row,column)
  MAT33_ELMT(intr, 1, 1) = 168;
  MAT33_ELMT(intr, 1, 2) = 32;

  MAT33_ELMT(intr, 2, 0) = 0;//(row,column)
  MAT33_ELMT(intr, 2, 1) = 0;
  MAT33_ELMT(intr, 2, 2) = 0;
  
  MAT33_VECT3_MUL(res,intr,hom_coord);
  
  debug_1 = res.x;
  debug_2 = res.y;
  
  *x_res = res.x;
  *y_res = res.y;
  
  
  //Rmat
  
  //image_error()//compare with undist_x and undist_y
}


void draw_gate(struct image_t *im, struct gate_img gate)
{
  // draw four lines on the image:
  struct point_t from, to;
  if (gate.sz_left == gate.sz_right) {
    // square
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line(im, &from, &to);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line(im, &from, &to);
  } else {
    // polygon
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz_left;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz_left;
    image_draw_line(im, &from, &to);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz_left;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz_right;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz_right;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz_right;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz_right;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz_left;
    image_draw_line(im, &from, &to);
  }
}

void draw_gate_color(struct image_t *im, struct gate_img gate, uint8_t* color)
{
  
  
  // draw four lines on the image:
  struct point_t from, to;
  if (gate.sz_left == gate.sz_right) {
    // square
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line_color(im, &from, &to, color);
    // draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
  } else {
    // polygon
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz_left;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz_left;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz_left;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz_right;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz_right;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz_right;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz_right;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz_left;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
  }
}

//AXIS system
//(0,0)   160
//       Y
// -|----->
//  |
//  |
//  |
//  |
//  \/
// X
//320

void draw_cross(struct image_t *im,int x, int y, uint8_t* color)
{
  struct point_t from, to;
  
    //polygon
    from.x = x-10;
    from.y = y;
    to.x = x+10;
    to.y = y;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = x;
    from.y = y-10;
    to.x = x;
    to.y = y+10;
    image_draw_line_color(im, &from, &to, color);
}

void draw_gate_polygon(struct image_t *im, int *x_points, int *y_points, uint8_t* color)
{
  
  
  // draw four lines on the image:
  struct point_t from, to;
  
    //polygon
    from.x = x_points[0];
    from.y = y_points[0];
    to.x = x_points[1];
    to.y = y_points[1];
    image_draw_line_color(im, &from, &to, color);
   //draw_cross(im,x_points[0],y_points[0],green_color);
    //draw_line_segment(im, from, to, color);
    from.x = x_points[1];
    from.y = y_points[1];
    to.x = x_points[2];
    to.y = y_points[2];
    image_draw_line_color(im, &from, &to, color);
   //draw_cross(im,x_points[1],y_points[1],green_color);
    from.x = x_points[2];
    from.y = y_points[2];
    to.x = x_points[3];
    to.y = y_points[3];
    image_draw_line_color(im, &from, &to, color);
   //draw_cross(im,x_points[2],y_points[2],green_color);
    // draw_line_segment(im, from, to, color);
    from.x = x_points[3];
    from.y = y_points[3];
    to.x = x_points[0];
    to.y = y_points[0];
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
  
}

void check_gate_free(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;
    int bottom;
  // how much of the side should be visible to count as a detected side?
  float min_ratio_side = 0.30;
  (*n_sides) = 0;

  // check the four lines of which the gate consists:
  struct point_t from, to;
  
    from.x = gate.x_corners[0];
    from.y = gate.y_corners[0];
    to.x = gate.x_corners[1];
    to.y = gate.y_corners[1];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x_corners[1];
    from.y = gate.y_corners[1];
    to.x = gate.x_corners[2];
    to.y = gate.y_corners[2];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x_corners[2];
    from.y = gate.y_corners[2];
    to.x = gate.x_corners[3];
    to.y = gate.y_corners[3];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x_corners[3];
    from.y = gate.y_corners[3];
    to.x = gate.x_corners[0];
    to.y = gate.y_corners[0];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    /*else{
        (*n_sides) = 0;
    }*/
    n_points += np;
    n_colored_points += nc;
  

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
}

extern void check_gate(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;
    int bottom;
  // how much of the side should be visible to count as a detected side?
  float min_ratio_side = 0.30;
  (*n_sides) = 0;

  // check the four lines of which the gate consists:
  struct point_t from, to;
  if (gate.sz_left == gate.sz_right) {

    from.x = gate.x - gate.sz;
    from.y = gate.y - gate.sz;
    to.x = gate.x - gate.sz;
    to.y = gate.y + gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x - gate.sz;
    from.y = gate.y + gate.sz;
    to.x = gate.x + gate.sz;
    to.y = gate.y + gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y + gate.sz;
    to.x = gate.x + gate.sz;
    to.y = gate.y - gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y - gate.sz;
    to.x = gate.x - gate.sz;
    to.y = gate.y - gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    /*else{
        (*n_sides) = 0;
    }*/
    n_points += np;
    n_colored_points += nc;
  } else {
    from.x = gate.x - gate.sz;
    from.y = gate.y - gate.sz_left;
    to.x = gate.x - gate.sz;
    to.y = gate.y + gate.sz_left;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x - gate.sz;
    from.y = gate.y + gate.sz_left;
    to.x = gate.x + gate.sz;
    to.y = gate.y + gate.sz_right;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y + gate.sz_right;
    to.x = gate.x + gate.sz;
    to.y = gate.y - gate.sz_right;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y - gate.sz_right;
    to.x = gate.x - gate.sz;
    to.y = gate.y - gate.sz_left;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    /*else{
        (*n_sides) = 0;
    }*/
    n_points += np;
    n_colored_points += nc;
  }

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
}


extern int check_back_side_QR_code(struct image_t* im, struct gate_img gate)
{
  // check a square at the top left of the gate:
  int n_points, n_colored_points;
  int min_x, max_x, min_y, max_y;
  n_points = 0;
  n_colored_points = 0;
  float size_square = 0.5; // quarter of a gate  
  struct gate_img bs_square;  
  float threshold_color_ratio = 0.5;

  if (gate.sz_left == gate.sz_right) {

    // square gate:
    min_x = gate.x - (1.0f + size_square) * gate.sz;
    max_x = gate.x - gate.sz;
    min_y = gate.y + gate.sz;
    max_y = gate.y + (1.0f - size_square) * gate.sz;
    
    // draw it:
    bs_square.x = (min_x + max_x) / 2;
    bs_square.y = (min_y + max_y) / 2;
    bs_square.sz = (max_x - min_x) / 2;
    bs_square.sz_left = bs_square.sz;
    bs_square.sz_right = bs_square.sz;
    //draw_gate(im, bs_square);

    // go over the back side square and see if it is orange enough:
    for(y = min_y; y < max_y; y++)
    {
      for(x = min_x; x < max_x; x++)
      {
        n_points++;
        n_colored_points += check_color(im, x, y); 
      }
    }
    
    if((float) n_colored_points / (float) n_points > threshold_color_ratio)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    // polygon gate:
    
    min_x = gate.x - (1.0f + size_square) * gate.sz;
    max_x = gate.x - gate.sz;
    min_y = gate.y + gate.sz_left;
    max_y = gate.y + (1.0f - size_square) * gate.sz_left;
    
     // draw it:
    bs_square.x = (min_x + max_x) / 2;
    bs_square.y = (min_y + max_y) / 2;
    bs_square.sz = (max_x - min_x) / 2;
    bs_square.sz_left = bs_square.sz_left;
    bs_square.sz_right = bs_square.sz_right;
    //draw_gate(im, bs_square);
    
    for(y = min_y; y < max_y; y++)
    {
      for(x = min_x; x < max_x; x++)
      {
        n_points++;
        n_colored_points += check_color(im, x, y); 
      }
    }
    
    if((float) n_colored_points / (float) n_points > threshold_color_ratio)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }  
    
}

void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points)
{
  (*n_points) = 0;
  (*n_colored_points) = 0;

  float t_step = 0.05;
  int x, y;
  float t;
  // go from Q1 to Q2 in 1/t_step steps:
  for (t = 0.0f; t < 1.0f; t += t_step) {
    // determine integer coordinate on the line:
    x = (int)(t * Q1.x + (1.0f - t) * Q2.x);
    y = (int)(t * Q1.y + (1.0f - t) * Q2.y);

    // if (x >= 0 && x < im->w && y >= 0 && y < im->h) {
    if (x >= 0 && x < im->h && y >= 0 && y < im->w) {
      // augment number of checked points:
      (*n_points)++;

      if (check_color(im, x, y)) {
        // the point is of the right color:
        (*n_colored_points)++;
      }
    }
  }
}

void snake_up_and_down(struct image_t *im, int x, int y, int *y_low, int *y_high)
{
  int done = 0;
  int x_initial = x;
  (*y_low) = y;

  // snake towards negative y (down?)
  while ((*y_low) > 0 && !done) {
    if (check_color(im, x, (*y_low) - 1)) {
      (*y_low)--;

    } else if (x+1 < im->h && check_color(im, x + 1, (*y_low) - 1)) {
      x++;
      (*y_low)--;
    } else if (x-1 >= 0 && check_color(im, x - 1, (*y_low) - 1)) {
      x--;
      (*y_low)--;
    } else {
      done = 1;
    }
  }

  x = x_initial;
  (*y_high) = y;
  done = 0;
  // snake towards positive y (up?)
  // while ((*y_high) < im->h - 1 && !done) {
  while ((*y_high) < im->w - 1 && !done) {

    if (check_color(im, x, (*y_high) + 1)) {
      (*y_high)++;
    //    } else if (x < im->w - 1 && check_color(im, x + 1, (*y_high) + 1)) {
    } else if (x < im->h - 1 && check_color(im, x + 1, (*y_high) + 1)) {
      x++;
      (*y_high)++;
    } else if (x > 0 && check_color(im, x - 1, (*y_high) + 1)) {
      x--;
      (*y_high)++;
    } else {
      done = 1;
    }
  }
}

void snake_left_and_right(struct image_t *im, int x, int y, int *x_low, int *x_high)
{
  int done = 0;
  int y_initial = y;
  (*x_low) = x;

  // snake towards negative x (left)
  while ((*x_low) > 0 && !done) {
    if (check_color(im, (*x_low) - 1, y)) {
      (*x_low)--;  
    // } else if (y < im->h - 1 && check_color(im, (*x_low) - 1, y + 1)) {
    } else if (y < im->w - 1 && check_color(im, (*x_low) - 1, y + 1)) {
      y++;
      (*x_low)--;
    } else if (y > 0 && check_color(im, (*x_low) - 1, y - 1)) {
      y--;
      (*x_low)--;
    } else {
      done = 1;
    }
  }

  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  // while ((*x_high) < im->w - 1 && !done) {
  while ((*x_high) < im->h - 1 && !done) {

    if (check_color(im, (*x_high) + 1, y)) {
      (*x_high)++;
    // } else if (y < im->h - 1 && check_color(im, (*x_high) + 1, y++)) {
    } else if (y < im->w - 1 && check_color(im, (*x_high) + 1, y++)) {
      y++;
      (*x_high)++;
    } else if (y > 0 && check_color(im, (*x_high) + 1, y - 1)) {
      y--;
      (*x_high)++;
    } else {
      done = 1;
    }
  }
}


void snake_gate_detection_init(void)
{
  previous_best_gate.sz = 0;
  listener = cv_add_to_device(&SGD_CAMERA, snake_gate_detection_func);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SNAKE_GATE_INFO, snake_gate_send);
  gettimeofday(&start, NULL);
}
