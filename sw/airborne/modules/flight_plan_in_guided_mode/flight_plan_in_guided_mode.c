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
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"


//typedef int BOOL;
//#define TRUE 1
//#define FALSE 0

uint32_t counter_global;  // start when codes are uploaded
uint32_t counter_autopilot_mode; // start when autopilot mode is changed
uint32_t counter_temp1; // counter for temporarily use
uint32_t counter_temp2; // counter for temporarily use
uint32_t counter_temp3; // counter for temporarily use
uint8_t previous_mode,current_mode;

float time_global,time_autopilot_mode,time_temp1,time_temp2,time_temp3;

float psi0;//

bool counter_valid_flag[5] = {TRUE,TRUE,FALSE,FALSE,FALSE}; /* flag for each counter. TRUE means this
                                                            counter's value is valid */
bool primitive_valid_flag[3] = {TRUE,FALSE,FALSE};

void flight_plan_in_guided_mode_init() {
    counter_global = 0;
    counter_autopilot_mode = 0;
    counter_temp1 = 0;
    counter_temp2 = 0;
    counter_temp3 = 0;
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
}
void clock_run() {
    current_mode = autopilot_mode;
    if (current_mode != previous_mode)
    {
        counter_autopilot_mode = 0;
    }
    previous_mode = current_mode;
    counter_global++;
    counter_autopilot_mode++;
    counter_temp1++;
    counter_temp2++;
    counter_temp3++;
    time_global = counter_global/20.0;
    time_autopilot_mode = counter_autopilot_mode/20.0;
    time_temp1 = counter_temp1/20.0;
    time_temp2 = counter_temp2/20.0;
    time_temp3 = counter_temp3/20.0;
}

void display_information()
{
//    if (counter_valid_flag[0] == TRUE )
//    {
//        printf("Global time is %f\n",time_global);
//    }
    if (counter_valid_flag[1] == TRUE )
    {
        printf("Time in autopilot mode is %f\n",time_autopilot_mode);
    }
    if (counter_valid_flag[2] == TRUE )
    {
        printf("Time in guidance mode is %f\n",time_temp1);
    }
//
//    printf("Guidance h mode is %d\n",guidance_h.mode);
//    printf("Guidance v mode is %d\n",guidance_v_mode);
//    printf("Atuopilot mode is %d\n",autopilot_mode);
    printf("\n");
}

bool hover(float planned_time);
bool go_straight(float planned_time, float velocity);
bool change_heading_hover(float derta_psi,float planned_time);


void flight_plan_run() {        // 10HZ
//    if (time_autopilot_mode<5)
//        primitive_valid_flag[0] = TRUE;
//    else if (time_autopilot_mode<10)
//        primitive_valid_flag[1] = TRUE;
//    else if  (time_autopilot_mode<15)
//        primitive_valid_flag[2] = TRUE;
//    else
//        primitive_valid_flag[0] = TRUE;

    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && primitive_valid_flag[0] == TRUE)
    {
        hover(3);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && primitive_valid_flag[1] == TRUE)
    {
        go_straight(3,0.3);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && primitive_valid_flag[2] == TRUE)
    {
        change_heading_hover(90/180*3.14,2);
    }

}

bool hover(float planned_time)
{
    if (counter_valid_flag[2] == FALSE)
    {
        counter_temp1 = 0;
        counter_valid_flag[2] = TRUE;
        if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT)
        {
//            guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
//            guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            guidance_h_set_guided_body_vel(0,0);
            guidance_v_set_guided_z(-1.5);
            printf("In hover primitive\n");
            return 1;
        }
    }
    else if(counter_valid_flag[2] && time_temp1<planned_time)
        return 1;
    if (counter_valid_flag[2] && time_temp1>planned_time)
    {
        counter_valid_flag[2] = FALSE;
        primitive_valid_flag[0] = FALSE;
        primitive_valid_flag[1] = TRUE;
        return 0;
    }
}

bool go_straight(float planned_time, float velocity){
    if (counter_valid_flag[2] == FALSE)
    {
        counter_temp1 = 0;
        counter_valid_flag[2] = TRUE;
        if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT)
        {
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            guidance_h_set_guided_body_vel(velocity,0);
            guidance_v_set_guided_z(-1.5);
            printf("in go straight primitive\n");
            return 1;
        }
    }
    else if(counter_valid_flag[2] && time_temp1<planned_time)
        return 1;
    if (counter_valid_flag[2] && time_temp1>planned_time)
    {
        counter_valid_flag[2] = FALSE;
        primitive_valid_flag[1] = FALSE;
        primitive_valid_flag[2] = TRUE;
        return 0;
    }
}

bool change_heading_hover(float derta_psi,float planned_time){
    if (counter_valid_flag[2] == FALSE)
    {
        counter_temp1 = 0;
        counter_valid_flag[2] = TRUE;
        if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT)
        {
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            psi0 = stateGetNedToBodyEulers_f()->psi;
            guidance_h_set_guided_body_vel(0,0);
            guidance_v_set_guided_z(-1.5);
            printf("In change heading hover primitive\n");
            return 1;
        }
    }
    else if(counter_valid_flag[2] && time_temp1<planned_time)
    {
        guidance_h_set_guided_heading(60/180*3.14);
        guidance_h_set_guided_body_vel(0,0);
        guidance_v_set_guided_z(-1.5);
        return 1;
    }
    if (counter_valid_flag[2] && time_temp1>planned_time)
    {
        counter_valid_flag[2] = FALSE;
        primitive_valid_flag[2] = FALSE;
        primitive_valid_flag[0] = TRUE;
        return 0;
    }
}