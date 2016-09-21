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
 * @file "modules/command_level_iros/command_level_iros.c"
 * @author Shuo Li
 * This module is the highest level
 */

#include <stdio.h>
#include "modules/command_level_iros/command_level_iros.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/computer_vision/fly_through_gate_demo.h"
#include "modules/stereocam/stereo_gate_position/stereo_gate_position.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"

uint8_t previous_mode;
uint8_t current_mode;

enum states_lower_level state_lower_level = WAIT_FOR_DETECTION;
enum states_upper_level state_upper_level = SQUARE;

void command_init(){
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
}

void command_run() {
  
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        counter_autopilot_mode = 0;
        time_autopilot_mode = 0;
        primitive_in_use = NO_PRIMITIVE;
    }
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }

   switch (state_lower_level){
       case WAIT_FOR_DETECTION:
           if(gate_detected == 0)
               hover();
           else
           {
               state_lower_level = ADJUST_POSITION;
           }

               break;

   }


    
    previous_mode = current_mode;
}

