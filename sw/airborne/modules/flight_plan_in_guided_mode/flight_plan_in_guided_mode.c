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
#include "flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include <math.h>
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.h"
//#include "modules/stereocam/stereo_gate_position/stereo_gate_position.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "subsystems/ins.h"



#define p_x_position 0.12
#define p_y_position 0.12

#define Y_ADJUST_POSITION 2.5

# define Z_SETPOINT -1.5   //was -1.5

float psi0;//
float psi1;
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

bool arc_is_finished = 0;


int primitive_in_use; // This variable is used for showing which primitive is used now;

#define Z_BIAS 0.2//was .2

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
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
        guidance_loop_set_velocity(0,0);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0);
        psi1 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_heading(psi1);
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
    }

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
    }

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
    }
}

void go_up_down(float derta_altitude){
    if(primitive_in_use != GO_UP_DOWN){
        primitive_in_use = GO_UP_DOWN;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_velocity(0,0);   // earth coordinate
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0 - derta_altitude);
        states_race.altitude_is_achieved = FALSE;
    }
    if (fabs(stateGetPositionNed_f()->z-z0-derta_altitude)<0.1){
        states_race.altitude_is_achieved = TRUE;
    }

}


void adjust_position(float derta_altitude){

    // set z
    if (primitive_in_use != ADJUST_POSITION)
    {
        primitive_in_use = ADJUST_POSITION;
    }

        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        z0 = stateGetPositionNed_f()->z;
	float z_setpoint = derta_altitude; //z0 - derta_altitude+Z_BIAS;
	if (z_setpoint>-1)
	  z_setpoint = -1;
	else if (z_setpoint<-3.9)
	  z_setpoint = -3.9;
        guidance_v_set_guided_z(z_setpoint);
        psi0 = stateGetNedToBodyEulers_f()->psi;

    // set vx and vy
    if (fabs(current_x_gate)<0.1)
        velocity_body_y = 0;
    else if (current_x_gate>0.1)
        velocity_body_y = p_x_position * current_x_gate;
    else if (current_x_gate<-0.1)
        velocity_body_y = p_x_position * current_x_gate;

    if (fabs(current_y_gate-Y_ADJUST_POSITION)<0.2)
        velocity_body_x = 0;
    else if (current_y_gate-Y_ADJUST_POSITION>0.2)
        velocity_body_x = 0.1;
    else if (current_y_gate-Y_ADJUST_POSITION<-0.2)
        velocity_body_x =  -0.1;

    velocity_earth_x = cosf(psi0)*velocity_body_x - sinf(psi0)*velocity_body_y;
    velocity_earth_y = sinf(psi0)*velocity_body_x + cosf(psi0)*velocity_body_y;

    guidance_loop_set_velocity(velocity_earth_x,velocity_earth_y);

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
        primitive_in_use = TAKE_OFF;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_velocity(0,0);
        //guidance_v_set_guided_vz(-0.1);
        guidance_v_set_guided_z(desired_altitude);
    }

    if (fabs(stateGetPositionNed_f()->z - desired_altitude)<0.1)
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