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

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 60;//105;
uint8_t color_lum_max = 228;//205;
uint8_t color_cb_min  = 66;//52;
uint8_t color_cb_max  = 194;//140;
uint8_t color_cr_min  = 140;//was 180
uint8_t color_cr_max  = 230;//255;

// Gate detection settings:
int n_samples = 500;//1000;//500;
int min_pixel_size = 40;//100;
float min_gate_quality = 0.4;
float gate_thickness = 0;//0.05;//0.10;//

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
struct image_t img_result;
int n_gates = 0;

//color picker
uint8_t y_center_picker  = 0;
uint8_t cb_center  = 0;
uint8_t cr_center  = 0;

//camera parameters
#define radians_per_pix_w 0.006666667//2.1 rad(60deg)/315
#define radians_per_pix_h 0.0065625 //1.05rad / 160


//Debug messages

static void snake_gate_send(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_SNAKE_GATE_INFO(trans, dev, AC_ID,&y_center_picker,&cb_center,&cr_center,&sz,&szx1,&szx2); //  
}


// Checks for a single pixel if it is the right color
// 1 means that it passes the filter
int check_color(struct image_t *im, int x, int y)
{
  uint8_t *buf = im->buf; 
  buf += y * (im->w) * 2 + x * 4;

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
  int x = (im->w)/4;
  int y = (im->h)/2;
  buf += y * (im->w) * 2 + x * 4;
  
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

 source += y * (input->w) * 2 + x * 4;
 dest += y * (output->w) * 2 + x * 4;
        // UYVY
        dest[0] = 65;//211;        // U//was 65
        dest[1] = source[1];  // Y
        dest[2] = 255;//60;        // V//was 255
        dest[3] = source[3];  // Y
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
  uint16_t i;
  int x, y;//, y_low, y_high, x_low1, x_high1, x_low2, x_high2, sz, szx1, szx2;  
  float quality;
  float best_quality = 0;
  struct point_t from, to;
  

  n_gates = 0;  
  
  //color picker
  check_color_center(img,&y_center_picker,&cb_center,&cr_center);

  for(i = 0; i < n_samples; i++)
  {
    // get a random coordinate:
    x = rand() % (img->w/2);
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
        szx1 = (x_high1-x_low1)*2;
        szx2 = (x_high2-x_low2)*2;
            
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
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if(quality > best_quality)//min_gate_quality)
          {
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
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if(quality > best_quality)//min_gate_quality)
          {
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
          
            //color filtered versison of image for overlay and debugging
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
  draw_gate(img, gates[n_gates-1]);
  //image_yuv422_set_color(img,img,gates[n_gates-1].x,gates[n_gates-1].y);  
  
  calculate_gate_position(gates[n_gates-1].x,gates[n_gates-1].y,gates[n_gates-1].sz);
  
  }
  return img; // snake_gate_detection did not make a new image
}

void calculate_gate_position(int x_pix,int y_pix, int sz_pix)
{
  //calculate angles here
  
}

void draw_gate(struct image_t *im, struct gate_img gate)
{
  // draw four lines on the image:
  struct point_t from, to;
  from.x = (gate.x - gate.sz/2)*2;
  from.y = gate.y - gate.sz;
  to.x = (gate.x - gate.sz/2)*2;
  to.y = gate.y + gate.sz;
  image_draw_line(im, &from, &to);
  from.x = (gate.x - gate.sz/2)*2;
  from.y = gate.y + gate.sz;
  to.x = (gate.x + gate.sz/2)*2;
  to.y = gate.y + gate.sz;
  image_draw_line(im, &from, &to);
  from.x = (gate.x + gate.sz/2)*2;
  from.y = gate.y + gate.sz;
  to.x = (gate.x + gate.sz/2)*2;
  to.y = gate.y - gate.sz;
  image_draw_line(im, &from, &to);
  from.x = (gate.x + gate.sz/2)*2;
  from.y = gate.y - gate.sz;
  to.x = (gate.x - gate.sz/2)*2;
  to.y = gate.y - gate.sz;
  image_draw_line(im, &from, &to);
}

extern void check_gate(struct image_t *im, struct gate_img gate, float* quality)
{
  int n_points, n_colored_points;
  n_points = 0; 
  n_colored_points = 0;
  int np, nc;
  
  // check the four lines of which the gate consists:
  struct point_t from, to;
  from.x = gate.x - gate.sz/2;
  from.y = gate.y - gate.sz;
  to.x = gate.x - gate.sz/2;
  to.y = gate.y + gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np; 
  n_colored_points += nc;

  from.x = gate.x - gate.sz/2;
  from.y = gate.y + gate.sz;
  to.x = gate.x + gate.sz/2;
  to.y = gate.y + gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np; 
  n_colored_points += nc;

  from.x = gate.x + gate.sz/2;
  from.y = gate.y + gate.sz;
  to.x = gate.x + gate.sz/2;
  to.y = gate.y - gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np; 
  n_colored_points += nc;

  from.x = gate.x + gate.sz/2;
  from.y = gate.y - gate.sz;
  to.x = gate.x - gate.sz/2;
  to.y = gate.y - gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np; 
  n_colored_points += nc;

  // the quality is the ratio of colored points / number of points:
  (*quality) = ((float) n_colored_points) / ((float) n_points);
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

		if (x >= 0 && x < (im->w/2) && y >= 0 && y < im->h)
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
    else if(x < (im->w/2)-1 && check_color(im, x+1, (*y_high)+1))
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
  while((*x_high) < (im->w/2) - 1 && !done)
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
}
