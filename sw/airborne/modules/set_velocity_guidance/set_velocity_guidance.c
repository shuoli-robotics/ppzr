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

//#include "modules/set_velocity_guidance/set_velocity_guidance.h"
#include "set_velocity_guidance.h"
# include <stdio.h>
int counter_global;
int counter;

 void set_counter_init() {
     counter_global = 0;
 }

 void counter_auto() {
     counter_global ++;
 }

void print_state(void){
    printf("Velocity in X direction is %f \n",stateGetSpeedNed_f()->x);
    printf("Velocity in Y direction is %f \n",stateGetSpeedNed_f()->y);
    printf("Velocity in Z direction is %f \n",stateGetSpeedNed_f()->z);
    printf("phi is %d\n",stateGetNedToBodyEulers_i()->phi);
    printf("theta is %d\n",stateGetNedToBodyEulers_i()->theta);
    printf("psi is %d\n",stateGetNedToBodyEulers_i()->psi);
    printf("global counter is %d\n",counter_global);
}

void set_command(){
if (counter_global <5) {
    if(autopilot_guided_move_ned(0.5, 0, 0, 0)){
        return;
    }

}
    else{
    if(autopilot_guided_move_ned(-0.5, 0, 0, 0)){
        return;
    }
}
}


