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
#include "subsystems/imu.h"
//#include "modules/computer_vision/lib/vision/gate_detection.h"
#include "modules/computer_vision/lib/vision/gate_detection_free.h"
#include "modules/computer_vision/lib/vision/gate_corner_refine.h"
#include "state.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/lib/vision/qr_code_recognition.h"
#include "modules/computer_vision/lib/vision/calc_p3p.h"

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"

#include "filters/low_pass_filter.h"

#include "modules/computer_vision/lib/vision/ekf_race.h"

#include "modules/computer_vision/lib/vision/closed_gate_processing.h"

//#define PI 3.1415926

#define AHRS_PROPAGATE_FREQUENCY 512

#define HFF_LOWPASS_CUTOFF_FREQUENCY 20 //maybe also try 15


struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 20;//105;
uint8_t color_lum_max = 228;//205;
uint8_t color_cb_min  = 66;//52;
uint8_t color_cb_max  = 194;//140;

uint8_t color_cr_min  = 134;//138;//146;//was 180

uint8_t color_cr_max  = 230;//255;

// //color I dont know
uint8_t green_color[4] = {255,128,255,128}; //{0,250,0,250};
uint8_t blue_color[4] = {0,128,0,128};//{250,250,0,250};
// 

// //debugging
float debug_1 = 1.1;
float debug_2 = 2.2;
float debug_3 = 3.3;
float debug_4 = 4.4;
float debug_5 = 5.5;


//least squares final results
float ls_pos_x = 0;
float ls_pos_y = 0;
float ls_pos_z = 0;

float prev_ls_pos_x = 0;
float prev_ls_pos_y = 0;

//KF 

float X_int[7][1] = {{0}};


float u_k[8][1] = {{0}};

/* low pass filter variables */
Butterworth2LowPass_int filter_x;
Butterworth2LowPass_int filter_y;
Butterworth2LowPass_int filter_z;

int vision_sample = 0;

//KF timing

float EKF_dt = 0;
float EKF_m_dt = 0;
double time_prev = 0;
double time_prev_m = 0;

//final KF results
float kf_pos_x = 0;
float kf_pos_y = 0;
float kf_vel_x = 0;
float kf_vel_y = 0;

int ekf_debug_cnt = 0;

//variables declared extern which actually shouldn't
int hist_sample = 0;

float x_pos_hist = 0;
float y_pos_hist = 0;

float gate_heading = 0;
float gate_distance = 3.5;
////////////////extern 


int run_ekf = 0;
int run_ekf_m = 0;
int ekf_sonar_update = 0;

double last_open_loop_time = 0;

struct timeval stop, start;

static void snake_gate_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SNAKE_GATE_INFO(trans, dev, AC_ID,&debug_1, &debug_2, &debug_3, &debug_4,&debug_5);
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
  
  //if (y % 2 == 1) { y--; }

  if (x < 0 || x >= input->h || y < 0 || y >= input->w) {
    return;
  }


  /*
  source += y * (input->w) * 2 + x * 2;
  dest += y * (output->w) * 2 + x * 2;
  */
//   source += x * (input->w) * 2 + y * 2;
//   dest += x * (output->w) * 2 + y * 2;

  source += x * (input->w) * 2 + y * 2;
  dest += x * (output->w) * 2 + y * 2;
  // UYVY
  dest[0] = 65;//211;        // U//was 65
  dest[1] = source[1];  // Y
  dest[2] = 255;//60;        // V//was 255
  dest[3] = source[3];  // Y
  return 1;
}

void initialize_EKF(){
    X_int[0][0] = 0;
    X_int[1][0] = 0;
    X_int[2][0] = stateGetPositionNed_f()->z;
    //also reset gate position
    gate_heading = gate_initial_heading[race_state.gate_counter];
    gate_distance = gate_initial_position_y[race_state.gate_counter];
    if(race_state.gate_counter < -1){
	  gate_size_m = 1.4;
        printf("LARGE GATE\n");
      /*gate_size_m = 1.0;*/
    }else{
      gate_size_m = 1.0; //after second gate, switch to smaller gates
        printf("SMALL GATE\n");
    }
    run_ekf_m = 1;
    run_ekf = 1;
    printf("init EKF !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printf("gate distance:%f\n",gate_distance);
    printf("gate heading:%f\n",gate_heading);
    MAT_PRINT(7, 7,P_k_1_k_1_d);
    debug_5 = gate_dist_x;
}

void init_next_open_gate(){
  gate_distance = X_int[0][0] + gate_initial_position_y[race_state.gate_counter] - ls_pos_x;//
  X_int[1][0] = ls_pos_y;//init y estimate with first new ls detection
}

//state filter in periodic loop
void snake_gate_periodic(void)
{
  gettimeofday(&stop, 0);
  double time_now = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
  EKF_dt = time_now - time_prev;
  time_prev = time_now;
  
  
//   struct Int32Vect3 acc_meas_body;
//   struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
//   int32_rmat_transp_vmult(&acc_meas_body, body_to_imu_rmat, &imu.accel);
  
  struct Int32Vect3 acc_body_filtered;
  acc_body_filtered.x = update_butterworth_2_low_pass_int(&filter_x, imu.accel.x);
  acc_body_filtered.y = update_butterworth_2_low_pass_int(&filter_y, imu.accel.y);
  acc_body_filtered.z = update_butterworth_2_low_pass_int(&filter_z, imu.accel.z);
  
//   struct Int32Vect3 filtered_accel_ltp;
//   struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
//   int32_rmat_transp_vmult(&filtered_accel_ltp, ltp_to_body_rmat, &acc_meas_body);
  
  //    U_k = [acc_x_filter(n) acc_y_filter(n) acc_z(n) omega phi(n) theta(n) psi(n)];
  
  u_k[0][0] = ACCEL_FLOAT_OF_BFP(acc_body_filtered.x);
  u_k[1][0] = ACCEL_FLOAT_OF_BFP(acc_body_filtered.y);
  u_k[2][0] = ACCEL_FLOAT_OF_BFP(acc_body_filtered.z);
  u_k[3][0] = RATE_FLOAT_OF_BFP(imu.gyro.p);
  u_k[4][0] = RATE_FLOAT_OF_BFP(imu.gyro.q);
  u_k[5][0] = stateGetNedToBodyEulers_f()->phi;
  u_k[6][0] = stateGetNedToBodyEulers_f()->theta;
  u_k[7][0] = 0;//stateGetNedToBodyEulers_f()->psi - gate_heading; Quick fix !!!!!!!!!!!!!!!!!!!!!!
  
  
  if(race_state.flag_in_open_loop == TRUE){
    last_open_loop_time = time_now;
    run_ekf = 0;
    //printf("open loop true %f !!!!!!!!!!!!!!!!!!!!!!!!\n",time_now);
  }  
   
  if(run_ekf){
      EKF_propagate_state(X_int,X_int,EKF_dt,u_k);
    //  printf("propagate psi:%f gate_heading:%f -------------------\n",stateGetNedToBodyEulers_f()->psi*57,gate_heading*57);
  }

  //bounding pos, speed and biases
  if(X_int[0][0] > 10)X_int[0][0] = 10;//xmax
  if(X_int[0][0] < -4)X_int[0][0] = -4;//xmin
  if(X_int[1][0] > 7)X_int[1][0] = 7;//ymax
  if(X_int[1][0] < -7)X_int[1][0] = -7;//ymin
  if(X_int[2][0] > 0)X_int[2][0] = 0;//xmax
  if(X_int[2][0] < -5)X_int[2][0] = -5;//xmin
  
  //w speed
  if(X_int[3][0] > 1.5)X_int[3][0] = 1.5;//ymax
  if(X_int[3][0] < -1.5)X_int[3][0] = -1.5;//ymin
  
  //acc biases
  if(X_int[4][0] > 1)X_int[4][0] = 1;//ymax
  if(X_int[4][0] < -1)X_int[4][0] = -1;//ymin
  if(X_int[5][0] > 1)X_int[5][0] = 1;//ymax
  if(X_int[5][0] < -1)X_int[5][0] = -1;//ymin
  if(X_int[6][0] > 2)X_int[6][0] = 2;//ymax
  if(X_int[6][0] < -2)X_int[6][0] = -2;//ymin
  
  kf_pos_x = X_int[0][0];
  kf_pos_y = X_int[1][0];
  kf_vel_x = X_int[2][0];
  kf_vel_y = X_int[3][0];
  
  
 //  debug_1 = X_int[2][0];
//   debug_2 = X_int[1][0];
  
//   debug_3 = kf_pos_x;
//   debug_4 = kf_pos_y;
  
//   debug_1 = u_k[0][0];
//   debug_2 = u_k[3][0];
//   debug_3 = u_k[4][0];
  
   debug_1 = X_int[0][0];
   debug_2 = X_int[1][0];
   
   /*debug_3 = X_int[2][0];*/
    //debug_5 = X_int[2][0];
   //debug_5 = u_k[2][0];
   //debug_5 = X_int[6][0];//bias z
  
  
  //if measurement available -> do EKF measurement update 

    if(X_int[0][0] < 1.8){
      hist_sample = 0;
    }
    
    //-1.6 for open gate?
    if(X_int[0][0] > (gate_dist_x - 0.4)){//block after to close to the target gate //////////////0.8, was 0.4
      //ekf_sonar_update = 1;
      run_ekf_m = 0;
    }else{
      run_ekf_m = 1;
    }
    
    if(X_int[0][0] > gate_dist_x && run_ekf_m == 0){
      run_ekf_m = 1;
      printf("run_ekf_m = 1;--------------------------------------------------\n");
    }
    
    
      ekf_sonar_update = 0;
    
    if(primitive_in_use == ZIGZAG_2){
	  run_ekf_m = 0;
    }
    
    //First stretch gate switching logic
    float switch_threshold_x = 1.0;
    float switch_threshold_y = 0.5;
    if(vision_sample && run_ekf_m == 1  && fabs(X_int[0][0]-ls_pos_x)>switch_threshold_x && fabs(X_int[1][0]-ls_pos_y)>switch_threshold_y){
      //init_next_open_gate();
      //printf("init_next_open_gate()--------------------------------------------------\n");
    }

    //only update when we trust the vision such to be good enough for a meausrement update 
//     if(vision_sample && run_ekf_m == 1){
//     prev_ls_pos_x = ls_pos_x;
//     prev_ls_pos_y = ls_pos_y;
//     }


      //If in first stretch limit detection distance///////////////////////////
      if(ls_pos_x < 0)run_ekf_m = 0;


    hist_sample = 0;
//     printf("run ekf:%d run_ekf_m:%d vision_sample:%d \n",run_ekf,run_ekf_m,vision_sample);
    debug_5 = 0;
  if(( vision_sample || hist_sample || ekf_sonar_update) && run_ekf && run_ekf_m && !isnan(ls_pos_x) && !isnan(ls_pos_y))
  {
    printf("run ekf:%d run_ekf_m:%d vision_sample:%d \n",run_ekf,run_ekf_m,vision_sample);
    debug_5 = 1;
    //printf("x pos=%f y pos=%f\n",debug_1,debug_2);
   // printf("primitivr in use = %d; --------------------------------------\n",primitive_in_use);
    //printf("AAAAAAAAA    pos x = %f and pos_y = %f\n",debug_1,debug_2);
    gettimeofday(&stop, 0);
    double time_m = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
    EKF_m_dt = time_m - time_prev_m;
    time_prev_m = time_m;
    
    //MAT_PRINT(2, 2,temp_2_2_b);
    
    if(EKF_m_dt>5.0)EKF_m_dt=5.0;
    //update dt 
   
      //xy-position in local gate frame(ls, or histogram)
      float local_x = 0;
      float local_y = 0;
      
      //if histogram detection measures close proximity to the gate, switch to histogram method
      if(hist_sample){
	local_x = gate_dist_x - x_pos_hist;
	local_y = y_pos_hist;
	hist_sample = 0;
      }else{
	local_x = ls_pos_x;
	local_y = ls_pos_y;
      }
      
	   debug_3 = local_x;
	   debug_4 = local_y;
      
      float z_k_d[3];
      z_k_d[0] = local_x;
      z_k_d[1] = local_y;
      z_k_d[2] = stateGetPositionNed_f()->z;
      
      EKF_update_state(X_int,X_int,z_k_d,EKF_m_dt,ekf_sonar_update);
      
    vision_sample = 0;
  }
  
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

  int open_gate = 0;
  
  if(state_upper_level  == SECOND_PART){

    //OPENGATE 
    float o_x_p = 0;
    float o_y_p = 0;
    float o_z_p = 0;
    
    if(open_gate_processing(img,&o_x_p,&o_y_p,&o_z_p)){
      ls_pos_x = o_x_p;
      ls_pos_y = o_y_p;
//       debug_1 = ls_pos_x;
//       debug_2 = ls_pos_y;
      vision_sample = 1;
      //printf("position x=%.2f, y=%.2f, z=%.2f\n", o_x_p, o_y_p, o_z_p);
    }
    
    
  }else{
  closed_gate_processing(img);
  }

   //printf("position x=%.2f, y=%.2f, z=%.2f\n", ls_pos_x, ls_pos_y, ls_pos_x);

  return img;
}



void snake_gate_detection_init(void)
{
  //previous_best_gate.sz = 0;
  listener = cv_add_to_device(&SGD_CAMERA, snake_gate_detection_func);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SNAKE_GATE_INFO, snake_gate_send);
  gettimeofday(&start, NULL);
  
  init_butterworth_2_low_pass_int(&filter_x, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_y, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_z, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  
  EKF_init();
  
}
