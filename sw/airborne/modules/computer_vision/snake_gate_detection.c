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
#include "modules/computer_vision/lib/vision/gate_detection.h"
#include "state.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/lib/vision/qr_code_recognition.h"

#define PI 3.1415926

//initial position after gate pass
#define INITIAL_X 0
#define INITIAL_Y 2
#define INITIAL_Z 0

//initial position and speed safety margins

#define X_POS_MARGIN 0.15//m
#define Y_POS_MARGIN 0.5//m
#define Z_POS_MARGIN 0.15//m
#define X_SPEED_MARGIN 0.15//m/s
#define Y_SPEED_MARGIN 0.2//m/s

#define GOOD_FIT 1.0


struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 60;//105;
uint8_t color_lum_max = 228;//205;
uint8_t color_cb_min  = 66;//52;
uint8_t color_cb_max  = 194;//140;
uint8_t color_cr_min  = 140;//was 180
uint8_t color_cr_max  = 230;//255;

// Gate detection settings:
int n_samples = 1000;//1000;//500;
int min_pixel_size = 40;//100;
float min_gate_quality = 0.2;
float gate_thickness = 0;//0.05;//0.10;//
float gate_size = 34;


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
 float x_dist =0;
 float y_dist =0;
 float z_dist =0;
 
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

//SAFETY AND RESET FLAGS
int uncertainty_gate = 0;
//int gate_detected = 0;
int init_pos_filter = 0;
int safe_pass_counter = 0;

float gate_quality = 0;

float fps_filter = 0;

struct timeval stop, start;

//Debug messages

static void snake_gate_send(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_SNAKE_GATE_INFO(trans, dev, AC_ID,&pix_x, &pix_y, &pix_sz, &size_left, &size_right, &x_dist, &y_dist, &z_dist,
				  &current_x_gate,&current_y_gate,&current_z_gate,&best_fitness,&current_quality,
				  &y_center_picker,&cb_center,&cr_center,&sz,&n_gates,&states_race.ready_pass_through,
				  &psi_gate); //
}


// Checks for a single pixel if it is the right color
// 1 means that it passes the filter
int check_color(struct image_t *im, int x, int y)
{
  if(x%2 == 1) x--;
  
  if(x < 0 || x >= im->w || y < 0 || y >= im->h)
    return 0;
   
  uint8_t *buf = im->buf; 
  buf += 2 * (y * (im->w) + x); // each pixel has two bytes
				// odd ones are uy
				// even ones are vy
  

  if (
      (buf[1] >= color_lum_min)
      && (buf[1] <= color_lum_max)
      && (buf[0] >= color_cb_min)
      && (buf[0] <= color_cb_max)
      && (buf[2] >= color_cr_min)
      && (buf[2] <= color_cr_max)
    )
  {
    // the pixel passes:
    return 1;
  }
  else  
  {
    // the pixel does not:
    return 0;
  }
}

void check_color_center(struct image_t *im, uint8_t *y_c, uint8_t *cb_c, uint8_t *cr_c)
{
  uint8_t *buf = im->buf; 
  int x = (im->w)/2;
  int y = (im->h)/2;
  buf += y * (im->w) * 2 + x * 2;
  
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
  if(x%2 == 1) x--;
  if(x < 0 || x >= input->w || y < 0 || y >= input->h)
    return;
  
  
 source += y * (input->w) * 2 + x * 2;
 dest += y * (output->w) * 2 + x * 2;
        // UYVY
        dest[0] = 65;//211;        // U//was 65
        dest[1] = source[1];  // Y
        dest[2] = 255;//60;        // V//was 255
        dest[3] = source[3];  // Y
}

void calculate_gate_position(int x_pix,int y_pix, int sz_pix, struct image_t *img,struct gate_img gate)
{
  //calculate angles here
  vert_angle = (-(((float)x_pix*1.0)-((float)(img->w)/2.0))*radians_per_pix_w)-(stateGetNedToBodyEulers_f()->theta);
  hor_angle = (((float)y_pix*1.0)-((float)(img->h)/2.0))*radians_per_pix_h;
  
  pix_x = x_pix;
  pix_y = y_pix;
  pix_sz = gate.sz;
    current_quality = gate.gate_q;


  if(gate_size == 0)
  {
    gate_size = 1;
  }
  
  float gate_size_m = tan(((float)gate_size/2.0)*radians_per_pix_w)*3.0;
  y_dist = gate_size_m/tan((pix_sz/2)*radians_per_pix_w);
  x_dist = y_dist * sin(hor_angle);
  z_dist = y_dist * sin(vert_angle);
  
}

//state filter in periodic loop
void snake_gate_periodic(void)
{
  	//SAFETY  gate_detected
	if(y_dist > 0.6 && y_dist < 5) {
        states_race.gate_detected = 1;
        counter_gate_detected = 0;
        time_gate_detected = 0;
	}
	else{
        states_race.gate_detected = 0;
        counter_gate_detected = 0;
        time_gate_detected = 0;
	}
	
	//SAFETY ready_pass_trough
	if(states_race.gate_detected == 1 && fabs(x_dist-INITIAL_X) < X_POS_MARGIN && fabs(y_dist-INITIAL_Y) < Y_POS_MARGIN
	  && fabs(z_dist-INITIAL_Z) < Z_POS_MARGIN && fabs(opt_body_v_x)<Y_SPEED_MARGIN && fabs(opt_body_v_x)<Y_SPEED_MARGIN ){
        safe_pass_counter += 1;
	}
	else{
        safe_pass_counter = 0;
	 states_race.ready_pass_through = 0;
	}

    if(safe_pass_counter > 5)
    {
        safe_pass_counter = 0;
        states_race.ready_pass_through = 1;
    }

	// Reinitialization after gate is cleared and turn is made(called from velocity guidance module)
	if(init_pos_filter == 1)
	{
	  init_pos_filter = 0;
	  //assumed initial position at other end of the gate
	  predicted_x_gate = INITIAL_X;//0;
	  predicted_y_gate = INITIAL_Y;//1.5;
	  
	}
	
        //State filter 
	

	//convert earth velocity to body x y velocity
	float v_x_earth =stateGetSpeedNed_f()->x;
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
	
	fps_filter = (float)1.0/dt;
	
        // predict the new location:
	float dx_gate = dt * body_filter_x;//(cos(current_angle_gate) * gate_turn_rate * current_distance);
	float dy_gate = dt * body_filter_y; //(velocity_gate - sin(current_angle_gate) * gate_turn_rate * current_distance);
	predicted_x_gate = previous_x_gate + dx_gate;
	predicted_y_gate = previous_y_gate + dy_gate;
	predicted_z_gate = previous_z_gate;
	
	float sonar_alt = stateGetPositionNed_f()->z;
	
        if (states_race.gate_detected == 1)
	{
	
		// Mix the measurement with the prediction:
		float weight_measurement;
		if (uncertainty_gate > 150)
		{
			weight_measurement = 1.0f;
			uncertainty_gate = 151;//max
		}
		else
			weight_measurement = 0.7;//(GOOD_FIT-best_quality)/GOOD_FIT;//check constant weight

		current_x_gate = weight_measurement * x_dist + (1.0f - weight_measurement) * predicted_x_gate;
		current_y_gate = weight_measurement * y_dist + (1.0f - weight_measurement) * predicted_y_gate;
		current_z_gate = weight_measurement * (z_dist + sonar_alt) + (1.0f - weight_measurement) * predicted_z_gate;
	
		// reset uncertainty:
		uncertainty_gate = 0;
	}
	else
	{
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
  int filter = 1;
  int gen_alg = 1;
  uint16_t i;
  int x, y;//, y_low, y_high, x_low1, x_high1, x_low2, x_high2, sz, szx1, szx2;  
  float quality;
  struct point_t from, to;
  best_quality = 0;
  //test
  //pix_x = img->w;
  //pix_y = img->h;
  
  

  n_gates = 0;  
  
  //color picker
  //check_color_center(img,&y_center_picker,&cb_center,&cr_center);

  for(i = 0; i < n_samples; i++)
  {
    // get a random coordinate:
    x = rand() % img->w;
    y = rand() % img->h;

    //check_color(img, 1, 1);
    // check if it has the right color
    if(check_color(img, x, y))
    {
      // snake up and down:
      snake_up_and_down(img, x, y, &y_low, &y_high);
      sz = y_high - y_low;
      
      y_low = y_low + (sz*gate_thickness);
      y_high = y_high - (sz*gate_thickness);
      
      y = (y_high + y_low) / 2;      
      
      // if the stretch is long enough
      if(sz > min_pixel_size)
      {
        // snake left and right:
        snake_left_and_right(img, x, y_low, &x_low1, &x_high1);
        snake_left_and_right(img, x, y_high, &x_low2, &x_high2); 

        x_low1 = x_low1 + (sz*gate_thickness);
        x_high1 = x_high1 - (sz*gate_thickness);
        x_low2 = x_low2 + (sz*gate_thickness);
        x_high2 = x_high2 - (sz*gate_thickness);
	
        // sizes of the left-right stretches: in y pixel coordinates
        szx1 = (x_high1-x_low1);
        szx2 = (x_high2-x_low2);
            
        // if the size is big enough:
        if(szx1 > min_pixel_size)
        {
	  // draw four lines on the image:	  
          x = (x_high1 + x_low1) / 2;//was+
          // set the size to the largest line found:
          sz = (sz > szx1) ? sz : szx1; 
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz/2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &quality);
	  gates[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if(quality > best_quality)//min_gate_quality)
          {
	    //draw_gate(img, gates[n_gates]);
            best_quality = quality;
            n_gates++;
          }
        }
        else if(szx2 > min_pixel_size)
        {
          x = (x_high2 + x_low2) / 2;//was +
          // set the size to the largest line found:
          sz = (sz > szx2) ? sz : szx2;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz/2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &quality);
	  gates[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if(quality > best_quality)//min_gate_quality)
          {
	    //draw_gate(img, gates[n_gates]);
            best_quality = quality;
            n_gates++;
          }
        }
        if(n_gates >= MAX_GATES)
        {
          break;  
        } 
        
      }
      //
    }
    
  }

  // do an additional fit to improve the gate detection:
  if(best_quality > min_gate_quality && n_gates>0)
  {
        // temporary variables:
    float x_center, y_center, radius, fitness, angle_1, angle_2,s_left,s_right;
    int clock_arms = 1;

    // prepare the Region of Interest (ROI), which is larger than the gate:
    float size_factor = 1.5;//2;//1.25;
    
    if(gen_alg)
    {
      int max_candidate_gates = 5;//5;
      best_fitness = 100;
      if(n_gates > 0 && n_gates < max_candidate_gates)
      {
	for(int gate_nr = 0; gate_nr < n_gates; gate_nr +=1)
	{
	    int16_t ROI_size = (int16_t) (((float) gates[gate_nr].sz) * size_factor);
	    int16_t min_x = gates[gate_nr].x - ROI_size;
	    min_x = (min_x < 0) ? 0 : min_x;
	    int16_t max_x = gates[gate_nr].x + ROI_size;
	    max_x = (max_x < img->w) ? max_x : img->w;
	    int16_t min_y = gates[gate_nr].y - ROI_size;
	    min_y = (min_y < 0) ? 0 : min_y;
	    int16_t max_y = gates[gate_nr].y + ROI_size;
	    max_y = (max_y < img->h) ? max_y : img->h;
	  
	  //draw_gate(img, gates[gate_nr]);
	    
	    int gates_x = gates[gate_nr].x;
	    int gates_y = gates[gate_nr].y;
	    int gates_sz = gates[gate_nr].sz;
	    
	  // detect the gate:
	  gate_detection(img, &x_center, &y_center, &radius, &fitness, &gates_x, &gates_y, &gates_sz,
                    (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2,&psi_gate,&s_left,&s_right);
	  if(fitness < best_fitness)
	  {
	    best_fitness = fitness;
	    // store the information in the gate:
	    best_gate.x = (int) x_center;
	    best_gate.y = (int) y_center;
	    best_gate.sz = (int) radius;
	    best_gate.sz_left = (int) s_left;
            best_gate.sz_right = (int) s_right;
	  }
	  
	}
	for(int gate_nr = 0; gate_nr < n_gates; gate_nr +=1)
	{
	  draw_gate(img, gates[gate_nr]);
	}
      }
      else if(n_gates >= max_candidate_gates)
      { 
	for(int gate_nr = n_gates-max_candidate_gates; gate_nr < n_gates; gate_nr +=1)
	{
	    int16_t ROI_size = (int16_t) (((float) gates[gate_nr].sz) * size_factor);
	    int16_t min_x = gates[gate_nr].x - ROI_size;
	    min_x = (min_x < 0) ? 0 : min_x;
	    int16_t max_x = gates[gate_nr].x + ROI_size;
	    max_x = (max_x < img->w) ? max_x : img->w;
	    int16_t min_y = gates[gate_nr].y - ROI_size;
	    min_y = (min_y < 0) ? 0 : min_y;
	    int16_t max_y = gates[gate_nr].y + ROI_size;
	    max_y = (max_y < img->h) ? max_y : img->h;
	  //draw_gate(img, gates[gate_nr]);
	  // detect the gate:
	  gate_detection(img, &x_center, &y_center, &radius, &fitness, &(gates[gate_nr].x), &(gates[gate_nr].y), &(gates[gate_nr].sz),
                    (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2,&psi_gate,&s_left,&s_right);
	  if(fitness < best_fitness)
	  {
	    best_fitness = fitness;
	    // store the information in the gate:
	    best_gate.x = (int) x_center;
	    best_gate.y = (int) y_center;
	    best_gate.sz = (int) radius;
	    best_gate.sz_left = (int) s_left;
            best_gate.sz_right = (int) s_right;
	  }
	}
	
	for(int gate_nr = n_gates-max_candidate_gates; gate_nr < n_gates; gate_nr +=1)
	{
	  draw_gate(img, gates[gate_nr]);
	}
	
      }
      //draw_gate(img, best_gate);
    }

    // read the QR code:
    float uncertainty;
    // get the class of the QR, given a region of interest (TODO: find out the right parameters for the region next to the gate that has the QR code)
    int QR_class = get_QR_class_ROI(img, (uint32_t) (x_center + radius), (uint32_t) (y_center-radius), (uint32_t) (x_center + 1.25 * radius), (uint32_t) (y_center-0.75*radius), &uncertainty);
  }
          
  //color filtered version of image for overlay and debugging
  if(filter)
  {
  int color_count = image_yuv422_colorfilt(img,img,
                                       color_lum_min, color_lum_max,
                                       color_cb_min, color_cb_max,
                                       color_cr_min, color_cr_max
                                      );}
          
  //DRAW gate
  if(best_quality > min_gate_quality && n_gates>0)
  {
      current_quality = best_quality;
      size_left = best_gate.sz_left;
      size_right = best_gate.sz_right;
  //draw_gate(img, gates[n_gates-1]);
  draw_gate(img, best_gate);
  gate_quality = gates[n_gates-1].gate_q;
  //image_yuv422_set_color(img,img,gates[n_gates-1].x,gates[n_gates-1].y);  
  
  //calculate_gate_position(gates[n_gates-1].x,gates[n_gates-1].y,gates[n_gates-1].sz,img,gates[n_gates-1]);
  calculate_gate_position(best_gate.x,best_gate.y,best_gate.sz,img,best_gate);
  
  }
    else{
      states_race.gate_detected = 0;
      current_quality = 0;}
  return img; // snake_gate_detection did not make a new image
}


void draw_gate(struct image_t *im, struct gate_img gate)
{
  // draw four lines on the image:
  struct point_t from, to;
  if(gate.sz_left == gate.sz_right)
  {
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
  }
  else
  {
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



extern void check_gate(struct image_t *im, struct gate_img gate, float* quality)
{
  int n_points, n_colored_points;
  n_points = 0; 
  n_colored_points = 0;
  int np, nc;
  
  // check the four lines of which the gate consists:
  struct point_t from, to;
  if(gate.sz_left == gate.sz_right)
  {
    from.x = gate.x - gate.sz;
    from.y = gate.y - gate.sz;
    to.x = gate.x - gate.sz;
    to.y = gate.y + gate.sz;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;

    from.x = gate.x - gate.sz;
    from.y = gate.y + gate.sz;
    to.x = gate.x + gate.sz;
    to.y = gate.y + gate.sz;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y + gate.sz;
    to.x = gate.x + gate.sz;
    to.y = gate.y - gate.sz;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y - gate.sz;
    to.x = gate.x - gate.sz;
    to.y = gate.y - gate.sz;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;
  }
  else
  {
    from.x = gate.x - gate.sz;
    from.y = gate.y - gate.sz_left;
    to.x = gate.x - gate.sz;
    to.y = gate.y + gate.sz_left;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;

    from.x = gate.x - gate.sz;
    from.y = gate.y + gate.sz_left;
    to.x = gate.x + gate.sz;
    to.y = gate.y + gate.sz_right;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y + gate.sz_right;
    to.x = gate.x + gate.sz;
    to.y = gate.y - gate.sz_right;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y - gate.sz_right;
    to.x = gate.x - gate.sz;
    to.y = gate.y - gate.sz_left;
    check_line(im, from, to, &np, &nc);
    n_points += np; 
    n_colored_points += nc;    
  }

  // the quality is the ratio of colored points / number of points:
  if(n_points == 0) 
  {
    (*quality) = 0;
  }
  else
  {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
}

void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int* n_points, int* n_colored_points)
{
  (*n_points) = 0;
  (*n_colored_points) = 0;

  float t_step = 0.05; 
	int x, y;
  float t;
  // go from Q1 to Q2 in 1/t_step steps:
	for (t = 0.0f; t < 1.0f; t += t_step)
	{
    // determine integer coordinate on the line:
		x = (int)(t * Q1.x + (1.0f - t) * Q2.x);
		y = (int)(t * Q1.y + (1.0f - t) * Q2.y);

		if (x >= 0 && x < im->w && y >= 0 && y < im->h)
		{
      // augment number of checked points:
      (*n_points)++;

      if(check_color(im, x, y))
      {
        // the point is of the right color:
        (*n_colored_points)++;
      }
		}
	}
}

void snake_up_and_down(struct image_t *im, int x, int y, int* y_low, int* y_high)
{
  int done = 0;
  int x_initial = x;
  (*y_low) = y;
  
  // snake towards negative y (down?)
  while((*y_low) > 0 && !done)
  {
    if(check_color(im, x, (*y_low)-1))
    {
      (*y_low)--;
    }
    else if(check_color(im, x+1, (*y_low)-1))
    {
      x++;
      (*y_low)--;
    }
    else if(check_color(im, x-1, (*y_low)-1))
    {
      x--;
      (*y_low)--;
    }
    else
    {
      done = 1;    
    }
  }

  x = x_initial;
  (*y_high) = y;
  done = 0;
  // snake towards positive y (up?)
  while((*y_high) < im->h - 1 && !done)
  {
    
    if(check_color(im, x, (*y_high)+1))
    {
      (*y_high)++;
    }
    else if(x < im->w-1 && check_color(im, x+1, (*y_high)+1))
    {
      x++;
      (*y_high)++;
    }
    else if(x > 0 && check_color(im, x-1, (*y_high)+1))
    {
      x--;
      (*y_high)++;
    }
    else
    {
      done = 1;
    }
  }
}

void snake_left_and_right(struct image_t *im, int x, int y, int* x_low, int* x_high)
{
  int done = 0;
  int y_initial = y;
  (*x_low) = x;
  
  // snake towards negative x (left)
  while((*x_low) > 0 && !done)
  {
    if(check_color(im, (*x_low)-1, y))
    {
      (*x_low)--;
    }
    else if(y < im->h-1 && check_color(im, (*x_low)-1, y+1))
    {
      y++;
      (*x_low)--;
    }
    else if(y > 0 && check_color(im, (*x_low)-1, y-1))
    {
      y--;
      (*x_low)--;
    }
    else
    {
      done = 1;    
    }
  }

  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  while((*x_high) < im->w - 1 && !done)
  {
    
    if(check_color(im, (*x_high)+1, y))
    {
      (*x_high)++;
    }
    else if(y < im->h - 1 && check_color(im, (*x_high)+1, y++))
    {
      y++;
      (*x_high)++;
    }
    else if(y > 0 && check_color(im, (*x_high)+1, y-1))
    {
      y--;
      (*x_high)++;
    }
    else
    {
      done = 1;
    }
  }
}


void snake_gate_detection_init(void)
{
  listener = cv_add_to_device(&SGD_CAMERA, snake_gate_detection_func);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SNAKE_GATE_INFO, snake_gate_send);
  gettimeofday(&start, NULL);
}
