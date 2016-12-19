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
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "modules/replay_commands/replay_commands.h"
#include "modules/kalman_filter/kalman_filter.h"

#define PI 3.1415926

uint8_t previous_mode;
uint8_t current_mode;
uint8_t previous_lower_level;

void first_part_logic(void);
void second_part_logic(void);
void third_part_logic(void);
void fourth_part_logic(void);
void fifth_part_logic(void);

bool replay_flag;
enum states_lower_level state_lower_level = WAIT_FOR_DETECTION_CM;
enum states_upper_level state_upper_level = SECOND_PART;


double original_matrix[3][3]={{5,3,3},{1,0,5},{5,1,5}};
double inversed_matrix[3][3];

void command_init(){
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    init_heading = stateGetNedToBodyEulers_f()->psi;
}


void command_run() {
  
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        counter_autopilot_mode = 0;
        time_autopilot_mode = 0;
        primitive_in_use = NO_PRIMITIVE;
        state_lower_level = LAND_CM; //PREPARE_CM;
        state_upper_level = FIRST_PART;
        init_heading = stateGetNedToBodyEulers_f()->psi;
    }
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }

    if(state_upper_level  == FIRST_PART)
    {

        first_part_logic();
    }

    if(state_upper_level  == SECOND_PART)
    {
        second_part_logic();
    }

    if(state_upper_level  == THIRD_PART)
    {
        third_part_logic();
    }

    if(state_upper_level  == FOURTH_PART)
    {
        fourth_part_logic();
    }

    if(state_upper_level  == FIFTH_PART)
    {
			fifth_part_logic();
    }
    previous_mode = current_mode;
}








void first_part_logic()
{
    switch(state_lower_level)
    {
        case PREPARE_CM:
            if (time_autopilot_mode > PREPARE_TIME)
            {
                previous_lower_level = PREPARE_CM;
                state_lower_level = TAKE_OFF_CLOSE_LOOP_CM;
            }
            break;
			
        case TAKE_OFF_OPEN_LOOP_CM:
            take_off(TAKE_OFF_ALTITUDE);
            if(states_race.altitude_is_achieved == TRUE)
            {
                previous_lower_level = TAKE_OFF_OPEN_LOOP_CM;
                state_lower_level =  TAKE_OFF_CLOSE_LOOP_CM;
            }
            break;

        case TAKE_OFF_CLOSE_LOOP_CM:
            hold_altitude(TAKE_OFF_ALTITUDE);
            if(states_race.altitude_is_achieved == TRUE)
            {
                previous_lower_level = TAKE_OFF_CLOSE_LOOP_CM;
                state_lower_level = TURN_CM;
            }
            break;

		case TURN_CM:
			change_heading_absolute(0.0);
			if (states_race.turning == FALSE)
            {
                previous_lower_level = TURN_CM;
                state_lower_level =  HOVER_CM;
            }
			break;

        case HOVER_CM:
            hover();
            if (time_primitive > HOVER_TIME)
            {
				if (previous_mode == TURN_CM)
				{
					previous_lower_level = HOVER_CM;
					state_lower_level = FLIGHT_TEST_THETA1_CM; 
				}
				else if(previous_mode == FLIGHT_TEST_THETA2_CM) 		
				{
					previous_lower_level = HOVER_CM;
					state_lower_level = FLIGHT_TEST_PHI1_CM;
				}	
            }
            break;

		case  FLIGHT_TEST_THETA1_CM: 
			set_theta(-3.0/180*PI);		
            if (time_primitive > THETA_TIME)
            {
                    previous_lower_level =  FLIGHT_TEST_THETA1_CM; 
                    state_lower_level = FLIGHT_TEST_THETA2_CM; 
     		}
            break;

		case FLIGHT_TEST_THETA2_CM: 
			set_theta(3.0/180*PI);		
            if (time_primitive > THETA_TIME)
            {
                    previous_lower_level =  FLIGHT_TEST_THETA2_CM; 
                    state_lower_level = HOVER_CM; 
                }
            break;
			
		case FLIGHT_TEST_PHI1_CM: 
			set_phi(3.0/180*PI);		
            if (time_primitive > PHI_TIME)
            {
                    previous_lower_level = FLIGHT_TEST_PHI1_CM; 
                    state_lower_level = FLIGHT_TEST_PHI2_CM; 
			}
			break;
			
		case FLIGHT_TEST_PHI2_CM:	
			set_phi(-3.0/180*PI);		
            if (time_primitive > PHI_TIME)
            {
                    previous_lower_level = FLIGHT_TEST_PHI2_CM; 
                    state_lower_level = LAND_CM; 
			}
			break;

		case LAND_CM:
			land();
			if ( states_race.land_is_finished == 1)
			{			
					int n = 3;
					inv_matrix(original_matrix,inversed_matrix,n);
					previous_lower_level = LAND_CM; 
                    state_lower_level = LAND_CM; 
					state_upper_level = SECOND_PART;
					states_race.land_is_finished = 0;

			}
    }
}



void second_part_logic()
{


}

void third_part_logic()
{

}


void fourth_part_logic() {
}


void fifth_part_logic()
{
}
