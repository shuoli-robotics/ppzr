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
#include "state.h"


uint8_t counter_global;
uint8_t counter_in_mode;
uint8_t counter;
uint8_t previous_mode;
uint8_t current_mode;
bool mode_change_flag;
bool counter_in_use_flag;
//int motion_type_flag[5] = {1,0,0,0,0}; // hover go_straight go_back turn_around fly_along_arc

float time_global;
float time_in_mode;
float time;

//void hover(float time_planned);
//void go_straight(float velocity,float time_planned);
//void go_back(float velocity,float time_planned);
//void turn_around(float psi0,float time_planned);


 void set_counter_init(void) {
     counter_global = 0;
     counter_in_mode = 0;
     counter = 0;
     previous_mode = autopilot_mode;
     current_mode = autopilot_mode;
     mode_change_flag = 0;
     counter_in_use_flag = 0;
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
     counter++;
     time_global = counter_global/20.0;
     time_in_mode = counter_in_mode/20.0;
     time = counter/20.0;
 }


void print_state(void){
//   printf("Velocity in X direction is %f \n",stateGetSpeedNed_f()->x);
//    printf("Velocity in Y direction is %f \n",stateGetSpeedNed_f()->y);
//    printf("Velocity in Z direction is %f \n",stateGetSpeedNed_f()->z);
//    printf("phi is %d\n",stateGetNedToBodyEulers_i()->phi);
//    printf("theta is %d\n",stateGetNedToBodyEulers_i()->theta);
//    printf("psi is %d\n",stateGetNedToBodyEulers_i()->psi);
//    printf("global time is %.2fs\n",time_global);
//    printf("Autopilot mode is %d\n",autopilot_mode);
//    printf("time in mode %d is %.2fs\n",autopilot_mode,time_in_mode);
//    printf("guidance mode is %d\n",guidance_h.mode);
    printf("\n");
}

void set_command(void){   //frequency is 10 HZ
    //printf("set_command() is called!!!!!! \n");
    //int i;
    /*for(i = 0; i<5;i++){
        if (motion_type_flag[0] ==1)
            hover(3);
        else if (motion_type_flag[1] ==1)
            go_straight(0.5,3);
        else if (motion_type_flag[2] ==1)
            go_back(0.5,2);
//        else if (motion_type_flag[3] ==1){
//            if (counter_in_use_flag == 0){
//                psi0 = stateGetNedToBodyEulers_i()->psi;
//            }
//            turn_around(psi0,4);
//        }
        else if (motion_type_flag[4] ==1)
            hover(3);
        else{
            hover(3);
        }
    }*/
}


//void hover(float time_planned){
//    if (counter_in_use_flag == 0){
//        counter = 0;
//        counter_in_use_flag = 1;
//    }
//
//    if (time < time_planned){
//        guidance_h_set_guided_body_vel(0,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(stateGetNedToBodyEulers_i()->psi);
//    }
//    else if (time > time_planned){
//        motion_type_flag[0] = 0;
//        counter_in_use_flag = 0;
//    }
//}
//
//void go_straight(float velocity,float time_planned){
//    if (counter_in_use_flag == 0){
//        counter = 0;
//        counter_in_use_flag = 1;
//    }
//
//    if (time < time_planned){
//        guidance_h_set_guided_body_vel(velocity,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(stateGetNedToBodyEulers_i()->psi);
//    }
//    else if (time > time_planned){
//        motion_type_flag[1] = 0;
//        //motion_type_flag[0] = 1;
//        counter_in_use_flag = 0;
//    }
//}
//
//void go_back(float velocity,float time_planned){
//    if (counter_in_use_flag == 0){
//        counter = 0;
//        counter_in_use_flag = 1;
//    }
//
//    if (time < time_planned){
//        guidance_h_set_guided_body_vel(velocity,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(stateGetNedToBodyEulers_i()->psi);
//    }
//    else if (time > time_planned){
//        motion_type_flag[2] = 0;
//        //motion_type_flag[0] = 1;
//        counter_in_use_flag = 0;
//    }
//}
//
//void turn_around(float psi0,float time_planned){
//    if (counter_in_use_flag == 0){
//        counter = 0;
//        counter_in_use_flag = 1;
//    }
//    if (time < time_planned/4){
//        guidance_h_set_guided_body_vel(0,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(psi0+3.14/2);
//    }
//    else if (time < 2*time_planned/4){
//        guidance_h_set_guided_body_vel(0,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(psi0+3.14);
//    }
//        else if (time < 3*time_planned/4){
//        guidance_h_set_guided_body_vel(0,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(psi0+3.14/2*3);
//    }
//        else if (time < time_planned){
//        guidance_h_set_guided_body_vel(0,0);
//        guidance_v_set_guided_z(stateGetPositionEnu_f()->z);
//        guidance_h_set_guided_heading(psi0+3.14*2);
//    }
//    else{
//        motion_type_flag[3] = 0;
//        //motion_type_flag[0] = 1;
//        counter_in_use_flag = 0;
//    }
//}