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
 * @file "modules/set_velocity_guidance/set_velocity_guidance.c"
 * @author Shuo Li
 * This mudule is used to set velocity command to the drone.
 */

#include "set_velocity_guidance.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include <stdio.h>


int counter_global;
int counter_in_mode;
int counter;
uint8_t previous_mode;
uint8_t current_mode;
bool mode_change_flag;
float time_global;
float time_in_mode;

 void set_counter_init(void) {
     counter_global = 0;
     counter_in_mode = 0;
     previous_mode = autopilot_mode;
     current_mode = autopilot_mode;
     mode_change_flag = 0;
 }

 void counter_auto(void) {      // frequency is 20HZ
     current_mode = autopilot_mode;
     if (current_mode != previous_mode)
     {
         mode_change_flag = 1;
         counter_in_mode = 0;
     }
     else{
         mode_change_flag = 0;
     }
     previous_mode = current_mode;
     counter_global++;
     counter_in_mode++;
     time_global = counter_global/20.0;
     time_in_mode = counter_in_mode/20.0;
 }


void print_state(void){
/*    printf("Velocity in X direction is %f \n",stateGetSpeedNed_f()->x);
    printf("Velocity in Y direction is %f \n",stateGetSpeedNed_f()->y);
    printf("Velocity in Z direction is %f \n",stateGetSpeedNed_f()->z);
    printf("phi is %d\n",stateGetNedToBodyEulers_i()->phi);
    printf("theta is %d\n",stateGetNedToBodyEulers_i()->theta);
    printf("psi is %d\n",stateGetNedToBodyEulers_i()->psi);*/
    printf("global time is %.2fs\n",time_global);
    printf("Autopilot mode is %d\n",autopilot_mode);
    printf("time in mode %d is %.2fs\n",autopilot_mode,time_in_mode);
    printf("\n");
}

void set_command(void){
    //printf("set_command() is called!!!!!! \n");
    if (time_in_mode < 3){
        guidance_h_set_guided_body_vel(0.5,0);
        guidance_h_set_guided_heading(0.5);
        guidance_v_set_guided_z(-1.5);
    }
    else if(time_in_mode < 5)
    {
        guidance_h_set_guided_body_vel(0, 0);
        guidance_h_set_guided_heading(0.5);
        guidance_v_set_guided_z(-1.5);
    }
    else if (time_in_mode < 8){
        guidance_h_set_guided_body_vel(-0.5, 0);
        guidance_h_set_guided_heading(0.5);
        guidance_v_set_guided_z(-1.5);
    }
    else if ((time_in_mode > 8)){
        guidance_h_set_guided_body_vel(0, 0);
        guidance_h_set_guided_heading(0.5);
        guidance_v_set_guided_z(0.5);
    }
}
