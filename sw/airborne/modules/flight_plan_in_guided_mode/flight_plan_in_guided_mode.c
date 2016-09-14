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
#include "modules/stereocam/stereo_gate_position/stereo_gate_position.h"


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
float heading;
float velocity_body_x;
float velocity_body_y;
float velocity_earth_x;
float velocity_earth_y;
bool altitude_is_arrived;
bool adjust_position_mask;
int primitive_in_use; // This variable is used for showing which primitive is used now;



void flight_plan_in_guided_mode_init() {
    primitive_in_use = NO_PRIMITIVE;
    bool adjust_position_mask = 0;
}


void display_information()
{
    if (autopilot_mode == AP_MODE_MODULE) {
        printf("z0 is %f\n",z0);
        printf("Altitude is arrived %d\n",altitude_is_arrived);
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
        //psi0 = stateGetNedToBodyEulers_f()->psi;
        vx_earth = cosf(psi0)*velocity;
        vy_earth = sinf(psi0)*velocity;
        guidance_loop_set_velocity(vx_earth,vy_earth);   // earth coordinate
        //z0 = stateGetPositionNed_f()->z;
        //guidance_v_set_guided_z(z0);
    }

}

void change_heading_hover(float derta_psi,float planned_time){
    if(primitive_in_use != CHANGE_HEADING_HOVER)
    {
        primitive_in_use = CHANGE_HEADING_HOVER;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        omega_psi = derta_psi/planned_time;
        guidance_loop_set_velocity(0,0);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0);
        guidance_loop_set_heading(psi0);
    }
    else
    {
        guidance_loop_set_velocity(0,0);
        guidance_v_set_guided_z(z0);
        guidance_loop_set_heading(psi0+omega_psi*time_primitive);
    }
    if(time_primitive>planned_time)
    {
        return;

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
    if(time_primitive > planned_time)      //(time_primitive>planned_time)
    {
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
        //psi0 = stateGetNedToBodyEulers_f()->psi;
        vx_earth = -sinf(psi0)*velocity;
        vy_earth = cos(psi0)*velocity;
        guidance_loop_set_velocity(vx_earth,vy_earth);   // earth coordinate
        //z0 = stateGetPositionNed_f()->z;
        //guidance_v_set_guided_z(z0);
        //guidance_loop_set_heading(psi0);
    }
}

void go_up_down(float derta_altitude){
    if(primitive_in_use != GO_UP_DOWN){
        primitive_in_use = GO_UP_DOWN;
        counter_primitive = 0;
        time_primitive = 0;
        altitude_is_arrived = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_velocity(0,0);   // earth coordinate
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0 - derta_altitude);
    }

    if (fabs(stateGetPositionNed_f()->z-(z0 - derta_altitude))<0.1)
        altitude_is_arrived = 1;
}

void adjust_position(float derta_altitude){

    // set z
    if (adjust_position_mask == 0)
    {
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        z0 = stateGetPositionNed_f()->z;
        guidance_v_set_guided_z(z0 - derta_altitude);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        adjust_position_mask = 1;
    }
    // set vx and vy
    if (fabs(current_x_gate)<0.2)
        velocity_body_y = 0;
    else if (current_x_gate>0.2)
        velocity_body_y = 0.2;
    else if (current_x_gate<-0.2)
        velocity_body_y = -0.2;

    if (fabs(current_y_gate-1.2)<0.2)
        velocity_body_x = 0;
    else if (current_y_gate-1.2>0.2)
        velocity_body_x = 0.2;
    else if (current_y_gate-1.2<-0.2)
        velocity_body_x = -0.2;

    velocity_earth_x = cosf(psi0)*velocity_body_x - sinf(psi0)*velocity_body_y;
    velocity_earth_y = sinf(psi0)*velocity_body_x + cosf(psi0)*velocity_body_y;

    guidance_loop_set_velocity(velocity_earth_x,velocity_earth_y);
}