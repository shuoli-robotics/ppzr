/*
 * Copyright (C) Shuo Li
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
 * @file "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.c"
 * @author Shuo Li
 * This module is used to generate flight plan in guided mode
 */

#include <stdio.h>
#include <stdlib.h>
#include "flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include <math.h>
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "subsystems/ins.h"
#include "modules/kalman_filter/kalman_filter.h"
#include "subsystems/imu.h"



#define KP_Y 0.6//raw vision:0.2 
#define KI_Y 0.0
#define KD_Y 0.25//raw vision:0.0
#define MAX_PHI  25.0/180*3.14


float psi0;//
float v_x_f;
float psi1;
float psi_startup;
float z0;
float x_start;
float y_start;
float heading;
float body_velocity_x;
float vx_earth;
float vy_earth;
float psi;
float velocity_body_x;
float velocity_body_y;
float velocity_earth_x;
float velocity_earth_y;
float init_heading;
float previous_error_y = 0;
float sum_y_error = 0;

int primitive_in_use; // This variable is used for showing which primitive is used now;

void flight_plan_in_guided_mode_init() {
    primitive_in_use = NO_PRIMITIVE;
}


void display_information()
{
}

bool prepare_before_take_off(double prepare_time)
{
    if(primitive_in_use != PREPARE_BEFORE_TAKE_OFF){
        primitive_in_use = PREPARE_BEFORE_TAKE_OFF;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
    }
	if (time_primitive > prepare_time)
	{
			return 1;
	}
	else
	{
			return 0;
	}
}


void hover()
{
    if(primitive_in_use != HOVER){
        primitive_in_use = HOVER;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
    }
}

bool go_straight(float theta,float distance,double ref_y){
    if(primitive_in_use != GO_STRAIGHT){
        primitive_in_use = GO_STRAIGHT;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
        x_start = stateGetPositionNed_f()->x;
        y_start = stateGetPositionNed_f()->y;
		sum_y_error = 0;
		states_race.attitude_control = TRUE;
		if (arc_counter == 0)
		{

				guidance_loop_set_heading(0.0);
		}
		else
		{
				guidance_loop_set_heading(3.14);
		}
    }

	float current_y;
	float sign = 1;
	if(ref_y > 1.5){
	 sign = -1;
	 current_y = stateGetPositionNed_f()->y;
	}
	else{
	  current_y = stateGetPositionNed_f()->y;//x_dist;//raw vision
	}
	float error_y = (ref_y - current_y)*sign;
	sum_y_error += error_y/20.0;
	float phi = KP_Y * error_y+ KD_Y *(error_y-previous_error_y)*20 + KI_Y*sum_y_error;
	if(phi > MAX_PHI)phi = MAX_PHI;
	if(phi < -MAX_PHI)phi = -MAX_PHI;
	guidance_loop_set_theta(theta);
	guidance_loop_set_phi(phi); 
	/*guidance_loop_set_heading(psi0);*/
	previous_error_y = error_y;
	//if (time_primitive > 1.0)
	
	//update body speed for use in ahrs_int_cmpl_quat.c
	float v_x_e = stateGetSpeedNed_f()->x;
	float v_y_e = stateGetSpeedNed_f()->y;
	v_x_f = cos(psi)*v_x_e +sin(psi)*v_y_e;
	
	if(ref_y < 1.5 && stateGetPositionNed_f()->x > 3)   
	{
			return TRUE;
	}
	else if(ref_y > 1.5 && stateGetPositionNed_f()->x < 0)   
	{
			return TRUE;
	}
	else
	{
			return FALSE;
	}
}


void circle(float radius, float planned_time){
}

void arc(float radius, float planned_time, float desired_angle_change){
}

void set_velocity_test(float vx_earth_t,float vy_earth_t){
}

void go_left_right(float velocity){
}

void go_up_down(float altitude){
}

void adjust_position(float derta_altitude){
}


void search_gate()
{
}

void take_off(float desired_altitude)
{
		if (primitive_in_use != TAKE_OFF)
		{
				psi_startup = stateGetNedToBodyEulers_f()->psi;
				primitive_in_use = TAKE_OFF;
				counter_primitive = 0;
				time_primitive = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				guidance_v_mode_changed(GUIDANCE_V_MODE_MODULE);  // vertical module should be called!
				states_race.attitude_control = FALSE;
				guidance_loop_set_velocity(0,0);
				states_race.altitude_is_achieved = 0;
		}

		if (time_primitive>2)
		{
				states_race.altitude_is_achieved = 1;
		}
}

void land()
{
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
}

void adjust_heading(float delta_heading) {
}

void left_right_back(float velocity_in_body_x,float velocity_in_body_y)
{
}



void hold_altitude(float desired_altitude)
{
    if (primitive_in_use != HOLD_ALTITUDE)
    {
        primitive_in_use = HOLD_ALTITUDE;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        counter_primitive = 0;
        time_primitive = 0;
        states_race.altitude_is_achieved = 0;
        guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
		states_race.attitude_control = FALSE;
		/*guidance_loop_set_velocity(0.0, 0.0);*/
        return;
    }
    if (fabs(stateGetPositionNed_f()->z - desired_altitude)<0.1 && time_primitive > 2)
	{
        // psi1 = stateGetNedToBodyEulers_f()->psi;
        states_race.altitude_is_achieved = 1;
        return;
    }

}

void change_heading_absolute(float psi)
{
}


void set_theta(float desired_theta)
{
}



void set_phi(float desired_phi)
{
}



void set_attitude(float desired_theta,float desired_phi)
{
    if(primitive_in_use != SET_ATTITUDE)
    { 
		primitive_in_use = SET_ATTITUDE;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		states_race.attitude_control = TRUE;
    }
	guidance_loop_set_theta(desired_theta);
	guidance_loop_set_phi(desired_phi); 
	guidance_loop_set_heading(psi0);
}


void calculate_attitude_average(double * p_theta,double *p_phi,struct acceleration* p_accel)
{
}

bool arc_open_loop(double radius,double theta,float delta_psi)
{
    if(primitive_in_use != ARC_OPEN_LOOP)
    { 
		primitive_in_use = ARC_OPEN_LOOP;
        psi = stateGetNedToBodyEulers_f()->psi;
		psi0 = psi;
        z0 = stateGetPositionNed_f()->z;
		float v_x_e = stateGetSpeedNed_f()->x;
		float v_y_e = stateGetSpeedNed_f()->y;
		v_x_f = cos(psi)*v_x_e +sin(psi)*v_y_e;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		states_race.attitude_control = TRUE;
    }
   /* float current_vel_x = stateGetSpeedNed_f()->x;*/
    /*float current_vel_y = stateGetSpeedNed_f()->y;*/
	/*float psi_a = stateGetNedToBodyEulers_f()->psi;*/
	float v_x_b = cos(theta)*v_x_f;
	float phi = stateGetNedToBodyEulers_f()->phi;
	float v_z_b = cos(phi)*sin(theta)*v_x_f;
	float drag_x_b = -0.5 * v_x_b;// - 0.102*v_x_b*v_x_b-0.2276;//was 0.27*v_x_b
	float drag_z_b = -0.3356 * v_z_b + 0.2789*v_z_b*v_z_b-0.0137;
	float drag_x_f = cos(theta)*drag_x_b+cos(phi)*sin(theta)*drag_z_b;
	float drag_z_f = -sin(theta)*drag_x_b+cos(phi)*cos(theta)*drag_z_b;
	float v_x_f_dot = tan(theta)*(-9.81)+drag_x_f;
	/*v_x_f = cos(psi_a)*current_vel_x + sin(psi_a)*current_vel_y;*/

	v_x_f = v_x_f + v_x_f_dot*1/20;//20.0;
	float psi_dot =  v_x_f/radius;
	psi = psi + psi_dot *1/20;//20;	
	double phi_desired = -atan(-v_x_f*psi_dot*cos(theta)/9.81);
	guidance_loop_set_theta(theta);
	guidance_loop_set_phi(phi_desired); 
	guidance_loop_set_heading(psi);
	guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
	//if (stateGetNedToBodyEulers_f()->psi>(psi0+delta_psi))
	if (psi>(psi0+delta_psi))
	{
			return 1;}
	else
	{
			return 0;
	}
}


bool hover_at_origin()
{
    if(primitive_in_use != HOVER_AT_ORIGIN)
    { 
		primitive_in_use = HOVER_AT_ORIGIN;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		guidance_h_set_guided_pos(0.0,0.0);
		guidance_h_set_guided_heading(0.0);
		guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
    }
 float current_x = stateGetPositionNed_f()->x;
 float current_y = stateGetPositionNed_f()->y;
 if (sqrt(current_x*current_x + current_y*current_y)<0.2&&time_primitive > 5)
 {
		 return 1;
 }
 else
 {
		 return 0;
 }
}
