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
#include "flight_plan_clock.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"


float psi0;//
uint16_t primitive_mask;

void flight_plan_in_guided_mode_init() {
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    primitive_mask = 0;
    SetBit( primitive_mask,1);
}


void display_information()
{

    if (counter_valid_flag[1] == TRUE )
    {
        printf("Time in autopilot mode is %f\n",time_autopilot_mode);
    }
    if (counter_valid_flag[2] == TRUE )
    {
        printf("Time in guidance mode is %f\n",time_temp1);
    }

    printf("Guidance h mode is %d\n",guidance_h.mode);
    printf("Guidance v mode is %d\n",guidance_v_mode);
    printf("Atuopilot mode is %d\n",autopilot_mode);
    printf("\n");
}

bool hover(float planned_time);
bool go_straight(float planned_time, float velocity);
bool change_heading_hover(float derta_psi,float planned_time);


void flight_plan_run() {        // 10HZ
    current_mode = autopilot_mode;
    if (current_mode != previous_mode)
    {
        ClearBit(clock_mask,2);
        SetBit(clock_mask,2);     // reset counter_autopilot_mode
    }

    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set(primitive_mask,1))
    {
        hover(5);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set(primitive_mask,2))
    {
        go_straight(3,0.3);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT &&  bit_is_set(primitive_mask,3))
    {
        change_heading_hover(90/180*3.14,5);
    }

}

bool hover(float planned_time)
{
    if (!bit_is_set(clock_mask,3))
    {
        SetBit(clock_mask,3);
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_h_set_guided_body_vel(0,0);
        guidance_v_set_guided_z(*stateGetPositionNed_f()->z);
        guidance_h_set_guided_heading(*stateGetNedToBodyEulers()->psi);
        return 1;
    }
    else if(time_primitive < planned_time)
        return 1;
    else if (time_primitive > planned_time)
    {
        ClearBit(clock_mask,3);
        ClearBit(primitive_mask,1);
        SetBit(primitive_mask,2);
        return 0;
    }
}

bool go_straight(float planned_time, float velocity){
    if (!bit_is_set(clock_mask,3))
    {
        SetBit(clock_mask,3);
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_v_set_guided_z(*stateGetPositionNed_f()->z);
        guidance_v_set_guided_z(-1.5);
        return 1;
    }
    else if(time_primitive < planned_time)
        return 1;
    else if (time_temp1>planned_time)
    {
        ClearBit(clock_mask,3);
        ClearBit(primitive_mask,2);
        SetBit(primitive_mask,3);
        return 0;
    }
}

bool change_heading_hover(float derta_psi,float planned_time){
    if (!bit_is_set(clock_mask,3))
    {
        SetBit(clock_mask,3);
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_h_set_guided_body_vel(0,0);
        guidance_v_set_guided_z(*stateGetPositionNed_f()->z);
        guidance_h_set_guided_heading(*stateGetNedToBodyEulers()->psi);
        return 1;
    }
    else if(time_primitive < planned_time)
    {
        guidance_h_set_guided_heading(psi0+derta_psi/planned_time*time_primitive);
        return 1;
    }
    else if (time_temp1>planned_time)
    {
        ClearBit(clock_mask,3);
        ClearBit(primitive_mask,3);
        SetBit(primitive_mask,1);
        return 0;
    }
}