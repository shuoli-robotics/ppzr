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




#define p_x_position 0.3
#define p_y_position 0.3

#define Y_ADJUST_POSITION 2.5

# define Z_SETPOINT -1.5   //was -1.5

float psi0;//
float psi1;
float psi_startup;
float z0;
float omega_psi;
float omega_gamma;
float omega_arc;
float heading;
float omega_circle;
float body_velocity_x;
float vx_earth;
float vy_earth;
float psi;
float velocity_body_x;
float velocity_body_y;
float velocity_earth_x;
float velocity_earth_y;
float init_heading;
float previous_desired_theta = 9999;
float previous_desired_phi = 9999;

int sample_pointer;
float sample_time;
bool arc_is_finished = 0;
double sum_theta;
double sum_phi;
struct acceleration sum_accel;
bool flag_calcualte_ave_atti = FALSE;

int primitive_in_use; // This variable is used for showing which primitive is used now;


//temp fix not loading vision stuff
float current_x_gate = 0;
float current_y_gate = 0;
float current_z_gate = 0;


void flight_plan_in_guided_mode_init() {
    primitive_in_use = NO_PRIMITIVE;
}


void display_information()
{
    if (autopilot_mode == AP_MODE_MODULE) {
        printf("\n");
        printf("\n");
        printf("\n");
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

void go_straight(float velocity){
    if(primitive_in_use != GO_STRAIGHT){
        primitive_in_use = GO_STRAIGHT;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        vx_earth = cosf(psi0)*velocity;
        vy_earth = sinf(psi0)*velocity;
        guidance_loop_set_velocity(vx_earth,vy_earth);   // earth coordinate
        z0 = stateGetPositionNed_f()->z;
       
// 	if(approach_first_part == FALSE)
// 	{
// 	  guidance_loop_set_heading(psi_startup);
// 	}
	
    }
   // guidance_v_set_guided_z(z0);
}

void change_heading_hover(float derta_psi){
    if(primitive_in_use != CHANGE_HEADING_HOVER)
    {
        primitive_in_use = CHANGE_HEADING_HOVER;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
	    guidance_loop_set_heading(psi0+derta_psi);
        states_race.turning = TRUE;
	z0 = stateGetPositionNed_f()->z;
    }
    //guidance_v_set_guided_z(z0);

    if (time_primitive > 1)   // was fabs(stateGetNedToBodyEulers_f()->psi - psi0-derta_psi)<0.05
    {
        states_race.turning = FALSE;
    }
}

void circle(float radius, float planned_time){
    if(primitive_in_use != CIRCLE  )
    {
        primitive_in_use = CIRCLE;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        omega_circle = 2*3.14/planned_time;
        body_velocity_x = omega_circle*radius;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_heading(psi0);
        vx_earth = cosf(psi0)*body_velocity_x;
        vy_earth = sinf(psi0)*body_velocity_x;
        guidance_loop_set_velocity(vx_earth,vy_earth);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0);

    }
    else
    {
        psi = psi0+omega_circle*time_primitive;
        guidance_loop_set_heading(psi);
        vx_earth = cosf(psi)*body_velocity_x;
        vy_earth = sinf(psi)*body_velocity_x;
        guidance_loop_set_velocity(vx_earth,vy_earth);
        guidance_v_set_guided_z(z0);
    }
    if(time_primitive > planned_time)
    {
        return;
    }
}

void arc(float radius, float planned_time, float desired_angle_change){
    if(primitive_in_use != ARC  )
    {
        primitive_in_use = ARC;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        omega_circle = desired_angle_change/planned_time;
        body_velocity_x = omega_circle*radius;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_heading(psi0);
        vx_earth = cosf(psi0)*body_velocity_x;
        vy_earth = sinf(psi0)*body_velocity_x;
        guidance_loop_set_velocity(vx_earth,vy_earth);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0);

    }
    else
    {   psi = psi0+omega_circle*time_primitive;
        guidance_loop_set_heading(psi);
        vx_earth = cosf(psi)*body_velocity_x;
        vy_earth = sinf(psi)*body_velocity_x;
        guidance_loop_set_velocity(vx_earth,vy_earth);
        guidance_v_set_guided_z(z0);
        }

    if(time_primitive > planned_time)      //(time_primitive>planned_time)
    {
        arc_is_finished = 1;
        return;
    }
}

void set_velocity_test(float vx_earth_t,float vy_earth_t){
    if(primitive_in_use != SET_VELOCITY_TEST){
        primitive_in_use = SET_VELOCITY_TEST;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
	    guidance_loop_set_heading(0);
        guidance_loop_set_velocity(vx_earth_t,vy_earth_t);   // earth coordinate
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0);
    }
}

void go_left_right(float velocity){
    if(1){
        primitive_in_use = GO_LEFT_RIGHT;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        vx_earth = -sinf(psi0)*velocity;
        vy_earth = cos(psi0)*velocity;
        guidance_loop_set_velocity(vx_earth,vy_earth);   // earth coordinate
        //z0 = stateGetPositionNed_f()->z;
        //guidance_v_set_guided_z(z0);
    }
}

void go_up_down(float altitude){
    if(primitive_in_use != GO_UP_DOWN){
        primitive_in_use = GO_UP_DOWN;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_velocity(0,0);   // earth coordinate
        //z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(altitude);
	    psi0 = stateGetNedToBodyEulers_f()->psi;
	    guidance_loop_set_heading(psi0);
        states_race.altitude_is_achieved = FALSE;
    }
    if (fabs(stateGetPositionNed_f()->z-altitude)<0.2){
        states_race.altitude_is_achieved = TRUE;
    }

}


void adjust_position(float derta_altitude){
}


void search_gate()
{
    if (primitive_in_use != SEARCH_GATE)
    {
        primitive_in_use = SEARCH_GATE;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_heading(psi0);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0);
        return;
    }
    if (time_primitive < 2)
    {
        velocity_body_x = 0;
        velocity_body_y = 0.2;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        velocity_earth_x = cosf(psi0)*velocity_body_x - sinf(psi0)*velocity_body_y;
        velocity_earth_y = sinf(psi0)*velocity_body_x + cosf(psi0)*velocity_body_y;
        guidance_loop_set_velocity(velocity_earth_x,velocity_earth_y);
    }
    else if(time_primitive < 6)
    {
        velocity_body_x = 0;
        velocity_body_y = -0.2;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        velocity_earth_x = cosf(psi0)*velocity_body_x - sinf(psi0)*velocity_body_y;
        velocity_earth_y = sinf(psi0)*velocity_body_x + cosf(psi0)*velocity_body_y;
        guidance_loop_set_velocity(velocity_earth_x,velocity_earth_y);
    }
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
    if (primitive_in_use != LAND)
    {
        primitive_in_use = LAND;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_heading(psi0);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(0);
    }

    if (fabs(stateGetPositionNed_f()->z)<0.1)
    {
        states_race.land_is_finished = 1;
    }
}

void adjust_heading(float delta_heading) {

    // set z
    if (primitive_in_use != ADJUST_HEADING) {
        primitive_in_use = ADJUST_HEADING;
    }

    psi0 = stateGetNedToBodyEulers_f()->psi;
    //todo: guidance_loop_set_heading(psi0+delta_heading);
}

void left_right_back(float velocity_in_body_x,float velocity_in_body_y)
{
    if (primitive_in_use != LEFT_RIGHT_BACK)
    {
        primitive_in_use = LEFT_RIGHT_BACK;
        z0 = stateGetPositionNed_f()->z;
	 counter_primitive = 0;
        time_primitive = 0;
    }
    //printf("!!!!!!!!!!!!!!!!!!!!!!!!\n");
    guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
    guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
    psi0 = stateGetNedToBodyEulers_f()->psi;
    //guidance_v_set_guided_z(z0);
    //guidance_loop_set_heading(psi1);
    // set vx and vy

        velocity_body_y = velocity_in_body_y;
        velocity_body_x = velocity_in_body_x;

    velocity_earth_x = cosf(psi0)*velocity_body_x - sinf(psi0)*velocity_body_y;
    velocity_earth_y = sinf(psi0)*velocity_body_x + cosf(psi0)*velocity_body_y;

    guidance_loop_set_velocity(velocity_earth_x,velocity_earth_y);
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
        guidance_v_set_guided_z(desired_altitude);
        return;
    }
    if (fabs(stateGetPositionNed_f()->z - desired_altitude)<0.1 && time_primitive > 5)
	{
        // psi1 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_heading(psi_startup);
        states_race.altitude_is_achieved = 1;
        return;
    }

}

void change_heading_absolute(float psi)
{
    if(primitive_in_use != CHANGE_HEADING_ABSOLUTE)
    {
        primitive_in_use = CHANGE_HEADING_ABSOLUTE;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_heading(psi);
        states_race.turning = TRUE;
        /*z0 = stateGetPositionNed_f()->z;*/
    }
    if (time_primitive > 2)   // was fabs(stateGetNedToBodyEulers_f()->psi - psi0-derta_psi)<0.05
    {
        states_race.turning = FALSE;
    }
}


void set_theta(float desired_theta)
{
		if (previous_desired_theta != desired_theta)
		{
				primitive_in_use = NO_PRIMITIVE;
				previous_desired_theta = desired_theta;

		}
    if(primitive_in_use != SET_THETA)
    { 
		primitive_in_use = SET_THETA; 
		counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
//		guidance_loop_set_theta(1.6/180*3.14);
//		guidance_loop_set_phi(0.55/180*3.14); //!!!!!!!!!!!
		guidance_loop_set_theta(0);
		guidance_loop_set_phi(0); //!!!!!!!!!!!
        z0 = stateGetPositionNed_f()->z;
    }
}



void set_phi(float desired_phi)
{
		if (previous_desired_phi!= desired_phi)
		{
				primitive_in_use = NO_PRIMITIVE;
				previous_desired_phi= desired_phi;

		}
    if(primitive_in_use != SET_PHI)
    { 
		primitive_in_use = SET_PHI; 
		counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		guidance_loop_set_phi(desired_phi);
        z0 = stateGetPositionNed_f()->z;
    }
}



void set_attidude(float desired_theta,float desired_phi)
{

    /*if(primitive_in_use != SET_ATTITUDE)*/
    /*{ */
		primitive_in_use = SET_ATTITUDE; 
		counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		guidance_loop_set_theta(desired_theta);
		guidance_loop_set_phi(desired_phi); //!!!!!!!!!!!
        z0 = stateGetPositionNed_f()->z;
    /*}*/
}


void calculate_attitude_average(double * p_theta,double *p_phi,struct acceleration* p_accel)
{
    if(flag_calcualte_ave_atti == FALSE)
    { 
		flag_calcualte_ave_atti = TRUE; 
		sample_pointer = 0;
		sample_time = PREPARE_TIME/SAMPLE_NUM;
		sum_phi = 0;
		sum_theta = 0;
		sum_accel.ax = 0;
		sum_accel.ay = 0;
		sum_accel.az = 0;
    }
	/*srand((unsigned)time(NULL));*/
	if(rand()/(double)(RAND_MAX) < 0.2) 
	{
			sum_theta += stateGetNedToBodyEulers_f()->theta;
			sum_phi += stateGetNedToBodyEulers_f()->phi;
			sum_accel.ax += imu.accel.x;//right hand rule (times -1 for gravity vector)
			sum_accel.ay +=imu.accel.y;
			sum_accel.az +=imu.accel.z;
			sample_pointer++;
	}

	if (sample_pointer == SAMPLE_NUM)
	{
			*p_theta= sum_theta/SAMPLE_NUM;
			*p_phi= sum_phi/SAMPLE_NUM;
			p_accel->ax = sum_accel.ax/SAMPLE_NUM;
			p_accel->ay = sum_accel.ay/SAMPLE_NUM;
			p_accel->az = sum_accel.az/SAMPLE_NUM;
			flag_calcualte_ave_atti = FALSE;
   	}
}


