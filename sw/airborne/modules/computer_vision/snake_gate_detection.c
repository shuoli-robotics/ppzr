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

#define AHRS_PROPAGATE_FREQUENCY 512

#define HFF_LOWPASS_CUTOFF_FREQUENCY 14


struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 20;//105;
uint8_t color_lum_max = 228;//205;
uint8_t color_cb_min  = 66;//52;
uint8_t color_cb_max  = 194;//140;

uint8_t color_cr_min  = 134;//138;//146;//was 180

uint8_t color_cr_max  = 230;//255;

// Gate detection settings:
int n_samples = 2000;//1000;//500;
int min_pixel_size = 20;//40;//100;
float min_gate_quality = 0.15;//0.2;
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

int x = 0;
int y = 0;


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

float current_x_gate = 0;
float current_y_gate = 0;
float current_z_gate = 0;

float delta_z_gate   = 0;

float previous_x_gate = 0;
float previous_y_gate = 0;
float previous_z_gate = 0;

int last_frame_detection = 0;
int repeat_gate = 0;

// previous best gate:
struct gate_img previous_best_gate;
struct gate_img last_gate;

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
float debug_4 = 4.4;


//optic flow dummy
float opt_body_v_x = 0;
float opt_body_v_y = 0;

//p3p final results
float p3p_result_x = 0;
float p3p_result_y = 0;
float p3p_result_z = 0;
float p3p_result_phi = 0;
float p3p_result_theta = 0;
float p3p_result_psi = 0;
float snake_res_x = 0;
float snake_res_y = 0;
float snake_res_z = 0;

//logging corner points in image plane
float gate_img_point_x_1 = 0;
float gate_img_point_y_1 = 0;
float gate_img_point_x_2 = 0;
float gate_img_point_y_2 = 0;
float gate_img_point_x_3 = 0;
float gate_img_point_y_3 = 0;
float gate_img_point_x_4 = 0;
float gate_img_point_y_4 = 0;

//least squares final results
float ls_pos_x = 0;
float ls_pos_y = 0;
float ls_pos_z = 0;

//KF 

#define dt_  0.002

float Gamma[4][2] = {  
   {0, 0} ,   /*  initializers for row indexed by 0 */
   {0, 0} ,   /*  initializers for row indexed by 1 */
   {1, 0} ,  /*  initializers for row indexed by 2 */
   {0, 1}
};

float Phi[4][4] = {  
   {1, 0, dt_, 0} ,   /*  initializers for row indexed by 0 */
   {0, 1, 0, dt_} ,   /*  initializers for row indexed by 1 */
   {0, 0, 1,   0} ,   /*  initializers for row indexed by 2 */
   {0, 0, 0,   1}
};

float Psi[4][2] = {  
   {0,   0} ,   /*  initializers for row indexed by 0 */
   {0,   0} ,   /*  initializers for row indexed by 1 */
   {dt_, 0} ,  /*  initializers for row indexed by 2 */
   {0,   dt_}
};

float X_int[4][1] = {  
   {0} ,   /*  initializers for row indexed by 0 */
   {0} ,   /*  initializers for row indexed by 1 */
   {0} ,  /*  initializers for row indexed by 2 */
   {0}
};

float X_opt[4][1] = {  
   {0} ,   /*  initializers for row indexed by 0 */
   {0} ,   /*  initializers for row indexed by 1 */
   {0} ,  /*  initializers for row indexed by 2 */
   {0}
};

float X_prev[4][1] = {  
   {0} ,   /*  initializers for row indexed by 0 */
   {0} ,   /*  initializers for row indexed by 1 */
   {0} ,  /*  initializers for row indexed by 2 */
   {0}
};

float X_temp_1[4][1];
float X_temp_2[4][1];
float X_temp_3[4][1];

float P_k_1[4][4] = {  
   {0, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
   {0, 0, 0, 0} ,   /*  initializers for row indexed by 1 */
   {0, 0, 0, 0} ,   /*  initializers for row indexed by 2 */
   {0, 0, 0, 0}
};

#define init_P 1.0
float P_k_1_k_1[4][4] = {  
   {init_P, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
   {0, init_P, 0, 0} ,   /*  initializers for row indexed by 1 */
   {0, 0, init_P, 0} ,   /*  initializers for row indexed by 2 */
   {0, 0, 0, init_P}
};

float Q_mat[2][2] = {  //try 0.5?
   {1, 0} ,  
   {0, 1}  
};

float R_k[2][2] = {  //try other?
   {0.2, 0} ,  
   {0, 0.2}  
};

//output mat
float H_k[2][4] = {  
   {1, 0, 0, 0} ,  
   {0, 1, 0, 0}  
};

float u_k[2][1] = {  
   {0} ,  
   {0}  
};

/* low pass filter variables */
Butterworth2LowPass_int filter_x;
Butterworth2LowPass_int filter_y;
Butterworth2LowPass_int filter_z;

int vision_sample = 0;

//KF timing

float KF_dt = 0;
float KF_m_dt = 0;
double time_prev = 0;
double time_prev_m = 0;

float temp_2_4[2][4];
float temp_4_4_a[4][4];
float temp_4_4_b[4][4];
float temp_4_4_c[4][4];
float temp_4_2_a[4][2];
float temp_4_2_b[4][2];
float temp_2_2_a[2][2];
float temp_2_2_b[2][2];

float inv_2_2[2][2];

float K_k[4][2];

float inn_vec[2][1];

float temp_4_1[4][1];

float eye_4[4][4] = {  
   {1, 0, 0, 0} ,   /*  initializers for row indexed by 0 */
   {0, 1, 0, 0} ,   /*  initializers for row indexed by 1 */
   {0, 0, 1, 0} ,   /*  initializers for row indexed by 2 */
   {0, 0, 0, 1}
};

//final KF results
float kf_pos_x = 0;
float kf_pos_y = 0;

int ekf_debug_cnt = 0;

static void snake_gate_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SNAKE_GATE_INFO(trans, dev, AC_ID, &pix_x, &pix_y, &pix_sz, &hor_angle, &vert_angle, &x_dist, &y_dist,
                                &z_dist,
                                &current_x_gate, &current_y_gate, &debug_1, &debug_2, &debug_3,
                                &y_center_picker, &cb_center, &QR_class, &sz, &states_race.ready_pass_through, &temp_check_gate.x_corners[1],
                                &debug_4);//psi_gate); //
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
  gettimeofday(&stop, 0);
  double time_now = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
  KF_dt = time_now - time_prev;
  time_prev = time_now;
  
  struct Int32Vect3 acc_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&acc_meas_body, body_to_imu_rmat, &imu.accel);
  
//   struct Int32Vect3 acc_body_filtered;
//   acc_body_filtered.x = update_butterworth_2_low_pass_int(&filter_x, acc_meas_body.x);
//   acc_body_filtered.y = update_butterworth_2_low_pass_int(&filter_y, acc_meas_body.y);
//   acc_body_filtered.z = update_butterworth_2_low_pass_int(&filter_z, acc_meas_body.z);
  
  struct Int32Vect3 filtered_accel_ltp;
  struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
  int32_rmat_transp_vmult(&filtered_accel_ltp, ltp_to_body_rmat, &acc_meas_body);
  u_k[0][0] = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.x);
  u_k[1][0] = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.y);
  
  //MAT_PRINT(2,1,u_k);
  
  
  //update dt 
  Phi[0][2] = KF_dt;
  Phi[1][3] = KF_dt;
  
  Psi[2][0] = KF_dt;
  Psi[3][1] = KF_dt;
  
  //X_int = Phi*X_int_prev' + Psi*u_k(1:2)'
  
  // C = A*B   A:(i,k) B:(k,j) C:(i,j)
  //MAT_MUL(_i, _k, _j, C, A, B)
  MAT_MUL(4, 4, 1, X_temp_1, Phi, X_int);
  MAT_MUL(4, 2, 1, X_temp_2, Psi, u_k);
  
  // C = A+B
  //MAT_SUM(_i, _j, C, A, B)
  MAT_SUM(4, 1, X_int, X_temp_1, X_temp_2);
  
  //bounding pos and speed
  if(X_int[0][0] > 4)X_int[0][0] = 4;//xmax
  if(X_int[0][0] < -1)X_int[0][0] = -1;//xmin
  if(X_int[1][0] > 3)X_int[1][0] = 3;//ymax
  if(X_int[1][0] < -3)X_int[1][0] = -3;//ymin
  
  kf_pos_x = X_int[0][0];
  kf_pos_y = X_int[1][0];
    
  debug_1 = X_int[0][0];
  debug_2 = X_int[1][0];
  
  if(arc_status.flag_in_arc == TRUE){
    debug_1 = arc_status.x;
    debug_2 = arc_status.y;
  }
  
//   debug_1 = u_k[0][0];
//   debug_2 = u_k[1][0];
  
  //if measurement available -> do KF measurement update 
  if(vision_sample == 1 || arc_status.flag_in_arc == TRUE)// && ekf_debug_cnt < 3)
  {
    ekf_debug_cnt+=1;
//      if(0)%in_turn == 1)
//            X_int(n,1) = arc_pred(n,1);
//            X_int(n,2) = arc_pred(n,2);
//         end
//         
//         Phi = [1 0 EKF_dt 0;
//        0 1 0  EKF_dt;
//        0 0 1  0;
//        0 0 0  1];
//         
//         x_kk_1 = X_int(n,:);
//         
//         % P(k+1|k) (prediction covariance matrix)
//         P_k_1 = Phi*P_k_1_k_1*Phi' + Gamma*Q*Gamma';
// 
//         K = P_k_1 * H_k' / (H_k*P_k_1 * H_k' + R_k);
// 
//         %z_k = [pos_x(n) pos_y(n) pos_z(n)];
//         z_k = [pos_x_ls(n) pos_y_ls(n)];
//        
//         X_opt = x_kk_1' + K * (z_k - X_int(n,1:2))';
//         X_int(n,:) = X_opt';
// 
//         P_k_1_k_1 = (eye(4) - K*H_k) * P_k_1 * (eye(4) - K*H_k)' + K*R_k*K';
    
    gettimeofday(&stop, 0);
    double time_m = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
    KF_m_dt = time_m - time_prev_m;
    time_prev_m = time_m;
    
    //MAT_PRINT(2, 2,temp_2_2_b);
    
    if(KF_m_dt>5.0)KF_m_dt=5.0;
    //update dt 
    Phi[0][2] = KF_m_dt;
    Phi[1][3] = KF_m_dt;
    
//     MAT_PRINT(4, 4,P_k_1_k_1);
//     MAT_PRINT(4, 4,Phi);
//     
//     MAT_PRINT(4, 2,Gamma);
//     MAT_PRINT(2, 2,Q_mat);
    
    
    //P_k_1 = Phi*P_k_1_k_1*Phi' + Gamma*Q*Gamma';
    
    // C = A*B'   A:(i,k) B:(j,k) C:(i,j)
    //MAT_MUL_T(_i, _k, _j, C, A, B) 
    MAT_MUL_T(2, 2, 4, temp_2_4, Q_mat, Gamma);
    
    // C = A*B   A:(i,k) B:(k,j) C:(i,j)
    //MAT_MUL(_i, _k, _j, C, A, B)
    MAT_MUL(4, 2, 4, temp_4_4_a, Gamma, temp_2_4);
    //MAT_PRINT(4, 4,temp_4_4_a);
    
    MAT_MUL_T(4, 4, 4, temp_4_4_b, P_k_1_k_1, Phi);
    //MAT_PRINT(4, 4,temp_4_4_b);
    MAT_MUL(4, 4, 4, temp_4_4_c, Phi, temp_4_4_b);
    //MAT_PRINT(4, 4,temp_4_4_c);
    MAT_SUM(4, 4, P_k_1, temp_4_4_a, temp_4_4_c);
    
    //MAT_PRINT(4, 4,P_k_1);
    
    //MAT_PRINT(2, 4,H_k);
    //K = P_k_1 * H_k' / (H_k*P_k_1 * H_k' + R_k);
    MAT_MUL_T(4, 4, 2, temp_4_2_a, P_k_1, H_k);
   // MAT_PRINT(4, 2,temp_4_2_a);
    MAT_MUL(2, 4, 2, temp_2_2_a, H_k, temp_4_2_a);
   // MAT_PRINT(2, 2,temp_2_2_a);
    MAT_SUM(2, 2, temp_2_2_b, temp_2_2_a, R_k);
    
    //MAT_PRINT(2, 2,temp_2_2_b);
    
    //calc 2x2 inverse
    float det_2_2 = 1/(temp_2_2_b[0][0]*temp_2_2_b[1][1]-temp_2_2_b[1][0]*temp_2_2_b[0][1]);
    printf("det_2_2%f\n",det_2_2);//check for nan!
    
    inv_2_2[0][0] = temp_2_2_b[1][1]*det_2_2;
    inv_2_2[0][1] = -temp_2_2_b[0][1]*det_2_2;
    inv_2_2[1][0] = -temp_2_2_b[1][0]*det_2_2;
    inv_2_2[1][1] = temp_2_2_b[0][0]*det_2_2;
    
    //MAT_PRINT(2, 2,inv_2_2);
    
    MAT_MUL(4, 2, 2, K_k, temp_4_2_a, inv_2_2);
    
    //MAT_PRINT(4, 2,K_k);
    
    //X_opt = x_kk_1' + K * (z_k - X_int(n,1:2))';
    if(arc_status.flag_in_arc == TRUE){
      inn_vec[0][0] = arc_status.x-X_int[0][0];
      inn_vec[1][0] = arc_status.y-X_int[1][0];
      //debug_3 = arc_status.y;
    }else{
      inn_vec[0][0] = ls_pos_x-X_int[0][0];
      inn_vec[1][0] = ls_pos_y-X_int[1][0];
      debug_3 = ls_pos_y;
    }
    
    printf("ls_pos_x:%f\n",ls_pos_x);
    printf("X_int[0][0]:%f\n",X_int[0][0]);
   
    printf("ls_pos_y:%f\n",ls_pos_y);
    printf("X_int[1][0]:%f\n",X_int[1][0]);
    
    
    MAT_MUL(4, 2, 1, temp_4_1, K_k, inn_vec);
    MAT_SUM(4, 1, X_int, X_int, temp_4_1);
    
    
//     debug_1 = kf_pos_x;
//     debug_2 = kf_pos_y;
    
    
    //P_k_1_k_1 = (eye(4) - K*H_k) * P_k_1 * (eye(4) - K*H_k)' + K*R_k*K';
    MAT_MUL(4, 2, 4, temp_4_4_a, K_k,H_k);
    //MAT_PRINT(4, 4,temp_4_4_a);
    
    MAT_SUB(4, 4, temp_4_4_b, eye_4, temp_4_4_a);
    //MAT_PRINT(4, 4,temp_4_4_b);
    //error free until here?
    MAT_MUL_T(4, 4, 4, temp_4_4_c, P_k_1, temp_4_4_b);
    // MAT_PRINT(4, 4,temp_4_4_c);
    //temp_4_4_a reuse
    MAT_MUL(4, 4, 4, temp_4_4_a, temp_4_4_b,temp_4_4_c);
    // MAT_PRINT(4, 4,temp_4_4_a);
    MAT_MUL_T(2, 2, 4, temp_2_4, R_k, K_k);
   //  MAT_PRINT(2, 4,temp_2_4);
    //temp_4_4_b reuse
    MAT_MUL(4, 2, 4, temp_4_4_b, K_k,temp_2_4);
    // MAT_PRINT(4, 4,temp_4_4_b);
    MAT_SUM(4, 4, P_k_1_k_1, temp_4_4_a, temp_4_4_b);
    // MAT_PRINT(4, 4,P_k_1_k_1);
    vision_sample = 0;
  }
  
  
}/*
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

 
}*/

//qsort comp function for sorting 
int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
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
  int filter = 1;
  int gate_graphics = 1;
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

  p3p_result_x = 0;
  p3p_result_y = 0;
  p3p_result_z = 0;
  p3p_result_phi = 0;
  p3p_result_theta = 0;
  p3p_result_psi = 0;
  snake_res_x = 0;
  snake_res_y = 0;
  snake_res_z = 0;


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
  if ((best_quality > min_gate_quality && n_gates > 0)||last_frame_detection) {
    // temporary variables:


    //if (gen_alg) {

      int max_candidate_gates = 10;//10;
      
      repeat_gate = 0;

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
//           gate_detection_free(img, points_x, points_y, &x_center, &y_center, &radius, &fitness, &gates_x, &gates_y, &gates_sz,
//                          (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi_gate,
//                          &s_left, &s_right);
	  int x_center_p = x_center;
	  int y_center_p = y_center;
	  int radius_p   = radius;
	  gate_corner_ref(img, points_x, points_y, &x_center_p, &y_center_p, &radius_p,
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y);
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
//           gate_detection_free(img, points_x, points_y, &x_center, &y_center, &radius, &fitness, &(gates[gate_nr].x), &(gates[gate_nr].y),
//                          &(gates[gate_nr].sz),
//                          (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi_gate,
//                          &s_left, &s_right);
	  int x_center_p = x_center;
	  int y_center_p = y_center;
	  int radius_p   = radius;
	  gate_corner_ref(img, points_x, points_y, &x_center_p, &y_center_p, &radius_p,
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y);
	  
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
///////////////////////////////////////////////Use previous best estimate
      }
      
      if((best_gate.gate_q > (min_gate_quality*2) && best_gate.n_sides > 3) && last_frame_detection == 1){
	
	repeat_gate = 1;
	
	int x_values[4];
	int y_values[4];
	
	memcpy(x_values,&(last_gate.x_corners[0]),sizeof(int)*4);
	memcpy(y_values,&(last_gate.y_corners[0]),sizeof(int)*4);
	
	//sort small to large 
	qsort(x_values, 4, sizeof(int), cmpfunc);
	qsort(y_values, 4, sizeof(int), cmpfunc);
// 	printf("in repeat_gate ########################################################\n ");
// 	printf("repeat_gate x1:%d x2:%d x3:%d x4:%d\n",last_gate.x_corners[0],last_gate.x_corners[1],last_gate.x_corners[2],last_gate.x_corners[3]);
// 	printf("repeat_gate y1:%d y2:%d y3:%d y4:%d\n",last_gate.y_corners[0],last_gate.y_corners[1],last_gate.y_corners[2],last_gate.y_corners[3]);
// 	
	
// 	    //   int16_t ROI_size = (int16_t)(((float) last_gate.sz) * size_factor);
//           int16_t min_x = last_gate.x - ROI_size;
//           min_x = (min_x < 0) ? 0 : min_x;
//           int16_t max_x = last_gate.x + ROI_size;
//           max_x = (max_x < img->h) ? max_x : img->h;
//           int16_t min_y = last_gate.y - ROI_size;
//           min_y = (min_y < 0) ? 0 : min_y;
//           int16_t max_y = last_gate.y + ROI_size;
//           max_y = (max_y < img->w) ? max_y : img->w;
//     //draw_gate(img, gates[gate_nr]);
// 	  gates_sz = last_gate.sz;
// 	  x_center = last_gate.x;
// 	  y_center = last_gate.y;
// 	  radius   = gates_sz;
//           // detect the gate:
// //           gate_detection_free(img, points_x, points_y, &x_center, &y_center, &radius, &fitness, &(gates[gate_nr].x), &(gates[gate_nr].y),
// //                          &(gates[gate_nr].sz),
// //                          (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi_gate,
// //                          &s_left, &s_right);
// 	  int x_center_p = x_center;
// 	  int y_center_p = y_center;
	//check x size, maybe use y also later?
 	  int radius_p   = x_values[3]-x_values[0];
	  gate_corner_ref_2(img, last_gate.x_corners, last_gate.y_corners,&radius_p);
	  
	  //draw_gate_polygon(img,points_x,points_y,blue_color);
          //if (fitness < best_fitness) {
            //best_fitness = fitness;
            // store the information in the gate: ,do we need to?
	  /*
            temp_check_gate.x = (int) x_center;
            temp_check_gate.y = (int) y_center;
            temp_check_gate.sz = (int) radius;
            temp_check_gate.sz_left = (int) s_left;
            temp_check_gate.sz_right = (int) s_right;*/
	    
	    memcpy(&(temp_check_gate.x_corners[0]),points_x,sizeof(int)*4);
	    memcpy(&(temp_check_gate.y_corners[0]),points_y,sizeof(int)*4);
	    
	    
            // also get the color fitness
            check_gate_free(img, temp_check_gate, &temp_check_gate.gate_q, &temp_check_gate.n_sides);
	    
	    if(temp_check_gate.n_sides > 2 && temp_check_gate.gate_q > best_gate.gate_q)
	    {
	    //best_fitness = fitness;
//             // store the information in the gate:
//             best_gate.x = temp_check_gate.x;
//             best_gate.y = temp_check_gate.y;
//             best_gate.sz = temp_check_gate.sz;
//             best_gate.sz_left = temp_check_gate.sz_left;
//             best_gate.sz_right = temp_check_gate.sz_right;
	    best_gate.gate_q = temp_check_gate.gate_q;
	    best_gate.n_sides = temp_check_gate.n_sides;
	    memcpy(&(best_gate.x_corners[0]),&(temp_check_gate.x_corners[0]),sizeof(int)*4);
	    memcpy(&(best_gate.y_corners[0]),&(temp_check_gate.y_corners[0]),sizeof(int)*4);
	    }
      }
   
  } else {
    //random position guesses here and then genetic algorithm
    //use random sizes and positions bounded by minimum size

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
  //xyz_dist zero unless sufficient quality
  x_dist = 0;
  y_dist = 0;
  z_dist = 0;
  if(best_gate.gate_q > (min_gate_quality*2)){
  calculate_gate_position(best_gate.x, best_gate.y, best_gate.sz, img, best_gate);
  //draw_gate_color(img, best_gate, blue_color);
  snake_res_x = x_dist;
  snake_res_y = y_dist;
  snake_res_z = z_dist;
  }
  
  //set gate point coordinate logging values to zero, in case no detection happens
  gate_img_point_x_1 = 0;
  gate_img_point_y_1 = 0;
  gate_img_point_x_2 = 0;
  gate_img_point_y_2 = 0;
  gate_img_point_x_3 = 0;
  gate_img_point_y_3 = 0;
  gate_img_point_x_4 = 0;
  gate_img_point_y_4 = 0;
  
 // draw_gate_color(img, best_gate, blue_color);
  
//   printf("repeat_gate:%d   -------------------------------------\n",repeat_gate);
 // printf("last_frame_detection:%d   -------------------------------------\n",last_frame_detection);
  
  if (best_gate.gate_q > (min_gate_quality*2) && best_gate.n_sides > 3) {//n_sides was > 2

    
    //sucessfull detection
    last_frame_detection = 1;
    vision_sample = 1;
    
    //draw_gate_color(img, best_gate, blue_color);
	if(repeat_gate == 0){
	  draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,blue_color);
	}
	else if(repeat_gate == 1){
	  draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,green_color);
	  for(int i = 0;i < 3;i++){
	    draw_cross(img,last_gate.x_corners[i],last_gate.y_corners[i],blue_color);
	  }
	}
    
    //save for next iteration
    memcpy(&(last_gate.x_corners[0]),&(best_gate.x_corners[0]),sizeof(int)*4);
    memcpy(&(last_gate.y_corners[0]),&(best_gate.y_corners[0]),sizeof(int)*4);
    //previous best snake gate
    last_gate.x = best_gate.x;
    last_gate.y = best_gate.y;
    last_gate.sz = best_gate.sz;
    
    
    current_quality = best_quality;
    size_left = best_gate.sz_left;
    size_right = best_gate.sz_right;
    

    gate_quality = best_gate.gate_q;
    
    //image_yuv422_set_color(img,img,gates[n_gates-1].x,gates[n_gates-1].y);

    //calculate_gate_position(gates[n_gates-1].x,gates[n_gates-1].y,gates[n_gates-1].sz,img,gates[n_gates-1]);
    //was here
    //calculate_gate_position(best_gate.x, best_gate.y, best_gate.sz, img, best_gate);
    

        gate_gen = 1;//0;
        states_race.gate_detected = 1;
        
//draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,blue_color);
	
	//vector and matrix declarations
	  struct FloatVect3 gate_point_0,gate_point_1,gate_point_2,
	  p3p_pos_sol_0,p3p_pos_sol_1,p3p_pos_sol_2,p3p_pos_sol_3,p3p_pos_test;
	  struct FloatMat33 R_mat_0,R_mat_1,R_mat_2,R_mat_3, R_test;
	  struct FloatEulers R_eulers;
	  
	  struct FloatVect3 p3p_pos_sol[4];
	  struct FloatVect3 gate_shift_points[4];
	  struct FloatVect3 gate_shift_vec[4];
	  struct FloatMat33 R_mat[4];
	  //ransac 
	  int ransac_error[4];
	  float ransac_rep_error[4];
	  struct FloatVect3 ransac_pos[4];
	  struct FloatMat33 ransac_R_mat[4];
	  
	  float gate_dist_x = 3.5;//was4.2
	  
	    
	  VECT3_ASSIGN(gate_points[0], gate_dist_x,-0.5000, -1.9000);
	  VECT3_ASSIGN(gate_points[1], gate_dist_x,0.5000, -1.9000);
	  VECT3_ASSIGN(gate_points[2], gate_dist_x,0.5000, -0.9000);
	  VECT3_ASSIGN(gate_points[3], gate_dist_x,-0.5000, -0.9000);
	  
	/*  
	  VECT3_ASSIGN(gate_points[0], 4.2000,0.3000, -1.9000);
	  VECT3_ASSIGN(gate_points[1], 4.2000,1.3000, -1.9000);
	  VECT3_ASSIGN(gate_points[2], 4.2000,1.3000, -0.9000);
	  VECT3_ASSIGN(gate_points[3], 4.2000,0.3000, -0.9000);*/
	
	
	//DEBUG----------------------------------------------------------------
	
	  VECT3_ASSIGN(p3p_pos_test, 2.295645,0.877159, -1.031926);
	  
	  MAT33_ELMT(R_test, 0, 0) = 0.999867;//(row,column)
	  MAT33_ELMT(R_test, 0, 1) =  -0.015745;
	  MAT33_ELMT(R_test, 0, 2) = -0.004201;

	  MAT33_ELMT(R_test, 1, 0) = 0.015464;//(row,column)
	  MAT33_ELMT(R_test, 1, 1) = 0.998070;
	  MAT33_ELMT(R_test, 1, 2) = -0.060151;

	  MAT33_ELMT(R_test, 2, 0) = 0.005140;//(row,column)
	  MAT33_ELMT(R_test, 2, 1) = 0.060078;
	  MAT33_ELMT(R_test, 2, 2) = 0.998180;
// 	
// 	  MAT33_ELMT(R_test, 0, 0) = 1;//(row,column)
// 	  MAT33_ELMT(R_test, 0, 1) = 0;
// 	  MAT33_ELMT(R_test, 0, 2) = 0;
// 
// 	  MAT33_ELMT(R_test, 1, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_test, 1, 1) = 1;
// 	  MAT33_ELMT(R_test, 1, 2) = 0;
// 
// 	  MAT33_ELMT(R_test, 2, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_test, 2, 1) = 0;
// 	  MAT33_ELMT(R_test, 2, 2) = 1;
	  
	int x_bp_corners[4];
	int y_bp_corners[4];
	float x_f_corners[4];
	float y_f_corners[4];
	float x_gate_corners[4];
	float y_gate_corners[4];
	
	for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_test,&R_test,&x_bp,&y_bp);
	    x_f_corners[i] = x_bp;
	    y_f_corners[i] = y_bp;
// 	    debug_1 = x_bp;
// 	    debug_2 = y_bp;
	  }
	
	//DEBUG---------------------------------------------------------------
	
	//Undistort fisheye points
	int f_fisheye = 168;
	float k_fisheye = 1.085;
	float reprojection_error[4];
		
	struct FloatRMat R,R_20,R_trans,Q_mat,I_mat, temp_mat, temp_mat_2;
	struct FloatEulers attitude, cam_body;
	struct FloatVect3 vec_20, p_vec,pos_vec,temp_vec,n_vec;
	
	p_vec.x = 0;
	p_vec.y = 0;
	p_vec.z = 0;
	
	MAT33_ELMT(I_mat, 0, 0) = 1;//(row,column)
	MAT33_ELMT(I_mat, 0, 1) = 0;
	MAT33_ELMT(I_mat, 0, 2) = 0;

	MAT33_ELMT(I_mat, 1, 0) = 0;//(row,column)
	MAT33_ELMT(I_mat, 1, 1) = 1;
	MAT33_ELMT(I_mat, 1, 2) = 0;

	MAT33_ELMT(I_mat, 2, 0) = 0;//(row,column)
	MAT33_ELMT(I_mat, 2, 1) = 0;
	MAT33_ELMT(I_mat, 2, 2) = 1;
	
	MAT33_ELMT(Q_mat, 0, 0) = 0;//(row,column)
	MAT33_ELMT(Q_mat, 0, 1) = 0;
	MAT33_ELMT(Q_mat, 0, 2) = 0;

	MAT33_ELMT(Q_mat, 1, 0) = 0;//(row,column)
	MAT33_ELMT(Q_mat, 1, 1) = 0;
	MAT33_ELMT(Q_mat, 1, 2) = 0;

	MAT33_ELMT(Q_mat, 2, 0) = 0;//(row,column)
	MAT33_ELMT(Q_mat, 2, 1) = 0;
	MAT33_ELMT(Q_mat, 2, 2) = 0;
	
	p_vec.x = 0;
	p_vec.y = 0;
	p_vec.z = 0;
	
	
// 	attitude.phi =    0.0873;//stateGetNedToBodyEulers_f()->phi;//positive ccw
// 	attitude.theta = 0.3491;//-(stateGetNedToBodyEulers_f()->theta-(20*(3.14/180)));//negative downward
// 	attitude.psi = 0.1745;//stateGetNedToBodyEulers_f()->psi;
	attitude.phi =    stateGetNedToBodyEulers_f()->phi;//positive ccw
	attitude.theta = stateGetNedToBodyEulers_f()->theta;//negative downward
	
	float temp_psi = stateGetNedToBodyEulers_f()->psi;
	if(stateGetPositionNed_f()->y>1.5){
	  if(temp_psi<0){
		  attitude.psi = temp_psi+3.14;
	  }
	  else{
		  attitude.psi = temp_psi-3.14;
	  }
	}
	else{
	  attitude.psi = stateGetNedToBodyEulers_f()->psi;
	}
	
	
	float_rmat_of_eulers_321(&R,&attitude);
	
	
	//debug_3 = (180/3.14)*attitude.theta;
	
	float x_trail[4] = {-55.0531, 0.4769, 5.4429, -56.9734};
	float y_trail[4] = {48.1444, 44.2227, 105.7145, 113.9445};
	
	for(int i = 0;i<4;i++)
	{
	  float undist_x, undist_y;
	  //debug_1 = (float)best_gate.x_corners[i];// best_gate.y_corners[i]
	  //debug_2 = (float)best_gate.y_corners[i];//
	  //undist_y = 5;//(float)best_gate.y_corners[i];
	  float princ_x = 157.0;
	  float princ_y = 32.0;
	  undistort_fisheye_point(best_gate.x_corners[i] ,best_gate.y_corners[i],&undist_x,&undist_y,f_fisheye,k_fisheye,princ_x,princ_y);
	  
	  x_gate_corners[i] = undist_x+157.0;
	  y_gate_corners[i] = undist_y+32.0;
	  
	  if(gate_graphics)draw_cross(img,((int)x_gate_corners[i]),((int)y_gate_corners[i]),green_color);
	  //if(gate_graphics)draw_cross(img,((int)x_trail[i])+157,((int)y_trail[i])+32,green_color);
	  vec_from_point_ned(undist_x, undist_y, f_fisheye,&gate_vectors[i]);
	  
	  //Least squares stuff here:
	  vec_from_point_2(undist_x,undist_y,168,&temp_vec);
	  //vec_from_point_2(x_trail[i],y_trail[i],168,&temp_vec);
	  
	  //camera to body rotation
	  cam_body.phi = 0;
	  cam_body.theta = -25*(3.14/180);
	  cam_body.psi = 0;
	  float_rmat_of_eulers_321(&R_20,&cam_body);
	  MAT33_VECT3_MUL(vec_20, R_20,temp_vec);
	  
	  
	  MAT33_TRANS(R_trans,R);
	  MAT33_VECT3_MUL(n_vec, R_trans, vec_20);
	  //print_vector(n_vec);
	  
	  //to ned
	  n_vec.z = -n_vec.z;
	 
	  double vec_norm = sqrt(VECT3_NORM2(n_vec));
	  VECT3_SDIV(n_vec, n_vec, vec_norm);
// 	  printf("n_vec norm:\n");
// 	  print_vector(n_vec);
	  VECT3_VECT3_TRANS_MUL(temp_mat, n_vec,n_vec);
// 	  printf("n_vec_squared:\n");
// 	  print_matrix(temp_mat);
	  MAT33_MAT33_DIFF(temp_mat,I_mat,temp_mat); 
// 	  printf("n_vec_squared - I:\n");
// 	  print_matrix(temp_mat);
	  MAT33_COPY(temp_mat_2,Q_mat);
// 	  printf("temp_mat_2:\n");
// 	  print_matrix(temp_mat_2);
	  MAT33_MAT33_SUM(Q_mat,temp_mat_2,temp_mat);
// 	  printf("Q_mat_sum:%d:\n",i);
// 	  print_matrix(Q_mat);
	  MAT33_VECT3_MUL(temp_vec, temp_mat, gate_points[i]);
	  VECT3_SUM(p_vec,p_vec,temp_vec);
	  
// 	  for i = 1:4
// 	  R = R + (eye(3,3)-n(:,i)*n(:,i)');
// 	  q = q + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
// 
// 	  p = R\q;
//	  or q = Rp
//        hence p = R_inv*q
	  
	  //Q_mat = Q_mat + I_mat - n_vec*n_vec'
	  //MAT33_VECT3_TRANSP_MUL(temp1,N,C);
	  
	  
	  //gate_points[i];
	  
	   //debug_1 = undist_x;
 	  //debug_2 = undist_y;
	  
// 	  draw_cross(img,(int)x_f_corners[i],(int)y_f_corners[i],green_color);
// 	  vec_from_point_ned(x_f_corners[i]-157,y_f_corners[i]-32, f_fisheye,&gate_vectors[i]);
	  
	 // //vec_from_point_ned(x_f_corners[i]-157,y_f_corners[i]-32, f_fisheye,&vec_temp1);
// 	  
// 	  printf("vec_from_point:\n");
// 	  print_vec(vec_temp1);
// 	  
// 	  VECT3_DIFF(vec_temp1,gate_points[i],p3p_pos_sol_0);
// 	  double norm = sqrt(VECT3_NORM2(vec_temp1));
// 	  VECT3_SDIV(gate_vectors[i], vec_temp1, norm);
// 	  
// 	  printf("gate_vectors[%d]:\n",i);
// 	  print_vec(gate_vectors[i]);
	  
	  
// 	  debug_1 = gate_vectors[i].x;
// 	  debug_2 = gate_vectors[i].y;
// 	  debug_3 = gate_vectors[i].z;
	  
	}
	
// 	  printf("Q_mat:\n");
// 	  print_matrix(Q_mat);
	
	MAT33_INV(temp_mat,Q_mat);
	
	MAT33_VECT3_MUL(pos_vec, temp_mat,p_vec);
	
	ls_pos_x = pos_vec.x;
	
	//bound y to remove outliers 
	float y_threshold = 2.5;
	if(pos_vec.y > y_threshold)pos_vec.y = y_threshold;
	if(pos_vec.y < -y_threshold)pos_vec.y = -y_threshold;
	
	
	ls_pos_y = pos_vec.y-0.25;//visual bias??
	ls_pos_z = pos_vec.z;
	//debug_3 = ls_pos_y;
// 		printf("R_mat_trans:\n");
//   		print_matrix(R_trans);
	  
	gate_img_point_x_1 = best_gate.x_corners[0];
	gate_img_point_y_1 = best_gate.y_corners[0];
	gate_img_point_x_2 = best_gate.x_corners[1];
	gate_img_point_y_2 = best_gate.y_corners[1];
	gate_img_point_x_3 = best_gate.x_corners[2];
	gate_img_point_y_3 = best_gate.y_corners[2];
	gate_img_point_x_4 = best_gate.x_corners[3];
	gate_img_point_y_4 = best_gate.y_corners[3];
	
    
  } else {

      //no detection
    
//       ls_pos_x = 0;
//       ls_pos_y = 0;
//       ls_pos_z = 0;
    
      last_frame_detection = 0;
     
      states_race.gate_detected = 0;
      current_quality = 0;
      gate_gen = 1;
    
  }
  
//   debug_1 = ls_pos_x;
//   debug_2 = ls_pos_y;
 // debug_3 = ls_pos_z;
  
  	//principal point
	//draw_cross(img,158,12,blue_color);
	
	//better principal point?
	draw_cross(img,158,32,green_color);
	
	
	
  return img; // snake_gate_detection did not make a new image
}

void print_matrix(struct FloatMat33 mat)
{
  printf("mat:\n");
  printf("%f, %f, %f\n",MAT33_ELMT(mat, 0, 0),MAT33_ELMT(mat, 0, 1),MAT33_ELMT(mat, 0, 2));
  printf("%f, %f, %f\n",MAT33_ELMT(mat, 1, 0),MAT33_ELMT(mat, 1, 1),MAT33_ELMT(mat, 1, 2));
  printf("%f, %f, %f\n",MAT33_ELMT(mat, 2, 0),MAT33_ELMT(mat, 2, 1),MAT33_ELMT(mat, 2, 2));
}

void print_vector(struct FloatVect3 vec)
{
  printf("Vec: %f, %f, %f\n",vec.x,vec.y,vec.z);
}


int find_minimum(float *error)
{
    float minimum = error[0];
    int location = 0;
    
	for (int c = 1 ; c < 4 ; c++ ) 
	{
	    if ( error[c] < minimum ) 
	    {
	      minimum = error[c];
	      location = c;
	    }
	}
    return location;
}


float euclidean_distance(float x_i, float x_bp, float y_i, float y_bp)
{
  float dist = sqrt(pow((x_i - x_bp),2)+pow((y_i - y_bp),2));
  return dist;
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
//k = 1.051;

//k = 1.080;//last k
 //  k = 1.118;
  k = 1.500;
  //k = 1.218;
  
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

void vec_from_point_2(float point_x, float point_y, int f, struct FloatVect3 *vec)
{
  
  f = 168;
  float focus = (float)f;
  vec->x = 1.0;
  //printf("Print in point 1\n");
  vec->y = point_x/focus;
  //printf("Print in point 2\n");
  vec->z = point_y/focus;
  //printf("Print in point 3\n");
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
//   MAT33_VECT3_MUL(point_3d,*R_mat,temp1);
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
  
//   debug_1 = res.x;
//   debug_2 = res.y;
  
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
  
  init_butterworth_2_low_pass_int(&filter_x, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_y, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_z, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  
}
