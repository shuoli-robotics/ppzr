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
#include <std.h>
#include "flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"
#include <math.h>

float psi0;//
uint16_t primitive_mask;
uint8_t previous_mode;
uint8_t current_mode;

void flight_plan_in_guided_mode_init() {
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    primitive_mask = 0;
    SetBit(primitive_mask,1);
}


void display_information()
{
    if (autopilot_mode == AP_MODE_GUIDED) {
        if(bit_is_set(primitive_mask,1))
            printf("It is in take of mode\n");
        else if(bit_is_set(primitive_mask,2))
            printf("It is in hover mode\n");
        else if (bit_is_set(primitive_mask,3))
            printf("It is in go straight mode\n");
        else if (bit_is_set(primitive_mask,4))
            printf("It is in change_heading_hover mode\n");

        if (bit_is_set(clock_mask,3))
            printf("Time in primitive is %f\n",time_primitive);

        printf("Altitude now is %f !\n",stateGetPositionNed_f()->z);
    }
}

void take_off(float altitude);
void hover(float planned_time);
void go_straight(float planned_time, float velocity);
void change_heading_hover(float derta_psi,float planned_time);

void take_off(float altitude){
    if (!bit_is_set(clock_mask,3))
    {
        SetBit(clock_mask,3);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_h_set_guided_body_vel(0,0);
        guidance_v_set_guided_z(altitude);
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
    }

    else if(fabs(altitude-stateGetPositionNed_f()->z)<0.2)
    {
        ClearBit(primitive_mask,1);
        SetBit(primitive_mask,2);
        ClearBit(clock_mask,3);
    }

}

void hover(float planned_time)
{
    if (!bit_is_set(clock_mask,3))
    {
        SetBit(clock_mask,3);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_h_set_guided_body_vel(0,0);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
    }
    else if(time_primitive > planned_time)
    {
        ClearBit(primitive_mask,2);
        SetBit(primitive_mask,3);
        ClearBit(clock_mask,3);
    }
}

void go_straight(float planned_time, float velocity){
    if (!bit_is_set(clock_mask,3)){
        SetBit(clock_mask,3);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
    }
    else if(time_primitive > planned_time)
    {
        ClearBit(primitive_mask,3);
        SetBit(primitive_mask,4);
        ClearBit(clock_mask,3);
    }
}

void change_heading_hover(float derta_psi,float planned_time){
    if (!bit_is_set(clock_mask,3))
    {
        SetBit(clock_mask,3);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_h_set_guided_body_vel(0,0);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
    }
    else if(time_primitive < planned_time)
    {
        if (fabs(time_primitive-planned_time/4)<0.2)
        guidance_h_set_guided_heading(psi0+derta_psi/4);
        else if (fabs(time_primitive-planned_time/2)<0.2)
            guidance_h_set_guided_heading(psi0+derta_psi/2);
        else if (fabs(time_primitive-planned_time/4*3)<0.2)
            guidance_h_set_guided_heading(psi0+derta_psi/4*3);
        else if (fabs(time_primitive-planned_time)<0.2)
            guidance_h_set_guided_heading(psi0+derta_psi);
    }
    else if(time_primitive < planned_time)      //(time_primitive>planned_time)
    {
        ClearBit(primitive_mask,4);
        SetBit(primitive_mask,2);
        ClearBit(clock_mask,3);
    }
}


void flight_plan_run() {        // 10HZ
    current_mode = autopilot_mode;
    if (current_mode != previous_mode)
    {
        counter_autopilot_mode = 0;
    }
    if (autopilot_mode != AP_MODE_GUIDED)
        ClearBit(clock_mask,3);

    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set(primitive_mask,1))
    {
        take_off(-1.5);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set(primitive_mask,2))
    {
        hover(5);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set(primitive_mask,3))
    {
        go_straight(5,0.3);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT &&  bit_is_set(primitive_mask,4))
    {
        change_heading_hover(90/180*3.14,5);
    }
}