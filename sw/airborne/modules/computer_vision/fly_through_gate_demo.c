/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/fly_through_gate_demo.h"
#include "modules/computer_vision/opencv_detect_gate.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "navigation.h"
#include "generated/flight_plan.h"
#include "std.h"
#include <stdlib.h>
#include <string.h>
#include "subsystems/datalink/telemetry.h"

// Variables for my state diagram
int eyes_closed_go = 0;
int wait_here_time=0;
enum DRONE_STATE dronerace_drone_state=GO_THROUGH_WINDOW;

// Global settings
float ratio_wanted = 0.44; // ratio between left and right to be enough in front of the window
int distance_pixels_between_just_go=125; // distance in pixels between the left and right pole of the window
float viewingAngle=0.45;//radians

//debugging
int16_t direction_pix = 0;

/* Sets the guided mode goal to a waypoint */
uint8_t guided_stay_wp(uint8_t wp){
	struct EnuCoor_f enu_i_wp = waypoints[wp].enu_f;
	struct NedCoor_f actually_ned;
	ENU_OF_TO_NED(actually_ned, enu_i_wp);
	guidance_h_set_guided_pos(actually_ned.x, actually_ned.y);
	return 0;
}

/* Get the position error to a waypoint */
float getPosErrorMeters(uint8_t wp){
	struct EnuCoor_f enu_i_wp = waypoints[wp].enu_f;
	struct EnuCoor_f my_place =*stateGetPositionEnu_f();
	double squared = fabs(my_place.x-enu_i_wp.x)*fabs(my_place.x-enu_i_wp.x)+\
			fabs(my_place.y-enu_i_wp.y)*fabs(my_place.y-enu_i_wp.y);
	return sqrt(squared);
}

/* Sets the state to detect window */
uint8_t start_fly_through(){
	dronerace_drone_state=DETECT_WINDOW;
	return 0;
}


uint8_t navigate_through_red_window(){
	set_red_window();
	return FALSE;
}
uint8_t make_turn_right_radians(float radians){
	 float yaw = stateGetNedToBodyEulers_f()->psi;
	 printf("Yaw now %f\n",yaw);
		 yaw+=radians;
		 printf("Yaw set %f\n",yaw);
		 guidance_h_set_guided_heading(yaw);

		return FALSE;

}
uint8_t navigate_through_blue_window(){
	set_blue_window();

	return FALSE;
}
/* gets whether the drone is in state GO_SAFETY */
uint8_t should_go_safety(){
	if(dronerace_drone_state==GO_SAFETY){
		return 1;
	}
	return 0;
}

static void race_debug_send(struct transport_tx *trans, struct link_device *dev)
    {
    pprz_msg_send_OBSTACLE_RACE_INFO(trans, dev, AC_ID,&direction_pix, &distance_pixels,&center_pixels,&left_height,&right_height);
    }  

// Function that calls the vision part of the drone race
// Then does a little bit of control.
struct image_t* gate_control_func(struct image_t* img);
struct image_t* gate_control_func(struct image_t* img)
{
  if (img->type == IMAGE_YUV422)
  {
    // Call OpenCV (C++ from paparazzi C function)
    opencv_gate_detect((char*) img->buf, img->w, img->h);   // call function to detect window, maybe return states of result
  }

  //DOWNLINK_SEND_OBSTACLE_RACE_INFO(DefaultChannel, DefaultDevice, &loc_y, &distance_pixels,&center_pixels,&left_height,&right_height);

  float yaw = stateGetNedToBodyEulers_f()->psi;
  float diff = loc_y-(img->h/2);
                                                               /* this part don't understand*/
  float unexplainedOffset=50.0;
  diff+=unexplainedOffset;
  direction_pix = (int16_t)diff;
  double pixelsPerDegree = viewingAngle/img->h;
  yaw += pixelsPerDegree * diff;
  float totalHeight = left_height + right_height;
  float ratio = left_height/totalHeight;

  totalHeight/=100.0;

  // Some quickly made control sequence that works with optitrack in the cyberzoo
  if(dronerace_drone_state==GO_THROUGH_WINDOW){
	  // Check is we want to go forward for a certain amount of time
	  if(eyes_closed_go>0){
		  eyes_closed_go--;
		  if(eyes_closed_go<=0){
			  eyes_closed_go=0;
			  wait_here_time=40;
		  }
		  guidance_h_set_guided_body_vel(1.0,0);
	  }
	  // Check if we want to wait here for a certain amount of time
	  else if(wait_here_time>0){
		  wait_here_time--;
		  if(wait_here_time<=0){
			  dronerace_drone_state=GO_SAFETY;
		  }
	  }
  }
  else if(dronerace_drone_state == DETECT_WINDOW){
	  // Always look at the obstacle
	  guidance_h_set_guided_heading(yaw);

	  // Too much red might mean that we are approaching the side of the obstacle
	  if(too_close){
		  guidance_h_set_guided_body_vel(-1.0,0);
	  }
	  else{
		  // Make sure we are exactly in front of the obstacle by rolling
		  if(ratio < ratio_wanted || ratio > (1.0-ratio_wanted)){
			  if(ratio < ratio_wanted){
				  guidance_h_set_guided_body_vel(0.0,-ratio/totalHeight);
			  }
			  else{
				  guidance_h_set_guided_body_vel(0.0,ratio/totalHeight);
			  }
		  }
		  else{
			  // Here we know we are in front of the obstacle.
			  // Check if we are so close to the obstacle that we would go through
			  if(distance_pixels>distance_pixels_between_just_go){
				  eyes_closed_go=60;
				  dronerace_drone_state=GO_THROUGH_WINDOW;
				  guidance_h_set_guided_body_vel(0.5,0);
			  }
			  else{
				  // Go forward slowly, roll a bit more to the center
				  guidance_h_set_guided_body_vel(0.5,diff/img->h);
			  }
		  }
	  }
  }

  return img;
}

void fly_through_gate_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OBSTACLE_RACE_INFO, race_debug_send);
  cv_add_to_device(&OPENCVDEMO_CAMERA, gate_control_func);
  opencv_init_rects();
}

