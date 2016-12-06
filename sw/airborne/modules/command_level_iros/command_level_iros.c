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

#define PI 3.1415926

uint8_t previous_mode;
uint8_t current_mode;
uint8_t previous_lower_level;
float turn_heading;
int gate_counter;
bool flag_clock3;


void first_part_logic(void);
void second_part_logic(void);
void third_part_logic(void);
void fourth_part_logic(void);
void fifth_part_logic(void);
//float choose_heading_after_passing_through_gate(void);
//float choose_distance_after_gate(void);

int counter_of_step;
bool replay_flag;
bool approach_first_part;
enum states_lower_level state_lower_level = WAIT_FOR_DETECTION_CM;
enum states_upper_level state_upper_level = SECOND_PART;

float v_x_search_gate,v_y_search_gate,adjusting_time_search_gate;

struct parameters_to_be_tuned parameter_to_be_tuned;

void command_init(){
    int i;

    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    states_race.time_to_go_straight = 0;
    states_race.distance_before_gate = 1.5;
    replay_flag = 0;
    init_heading = stateGetNedToBodyEulers_f()->psi;

    float distance_after_zigzag_temp[100] = {           1.5,0.5,0.5,0.2,0.5,    // 1-5
                                                        0.5,0.5,0.5,0.5,0.5,    // 6-10
                                                        0.5,0.5,0.5,0.5,0.5};  // 11-15

    float heading_after_zigzag_temp[100] = {             -45,0,-10,0,-80,    // 1-5
                                                         0.5,0.5,0.5,0.5,0.5,    // 6-10
                                                         0.5,0.5,0.5,0.5,0.5};  // 11-15

    int search_gate_velocity_in_zigzag_temp[100] = {-0.5,0.5,-0.5,0,0};  // -1 is adjusting to right

    float search_time_in_zigzag_temp[100] = {3,3,3,3,3};
    for(i = 0;i <NUMBER_OF_ZIGZAG; i++)
    {
        parameter_to_be_tuned.distance_after_zigzag[i] = distance_after_zigzag_temp[i];
        parameter_to_be_tuned.heading_after_zigzag[i] = heading_after_zigzag_temp[i]/180.0*PI;
        parameter_to_be_tuned.search_gate_velocity_in_zigzag[i] = search_gate_velocity_in_zigzag_temp[i];
        parameter_to_be_tuned.search_time_in_zigzag[i] = search_time_in_zigzag_temp[i];
    }
    // delta heading after passing through each gate (degree!)

    float heading_after_gates_temp[100] = {            90, -90,90,90,45,      // 1-5
                                                       0,0,0,0,0,           // 6-10
                                                       0,0};                // 11-15

    float distance_after_gates_temp[100] = {            3,3,3,3,5.0,    // 1-5
                                                        0.5,0.5,0.5,0.5,0.5,    // 6-10
                                                        0.5,0.5,0.5,0.5,0.5};  // 11-15

    float height_after_gates_temp[100]   ={             0,0,-2.5,-2,0,            // absolute height
                                                        0,0,0,0,0};             // 1-5

    float approach_after_gates_temp[100]   ={             1.3,1.3,0,0,0,            // time for approach
                                                          0,0,0,0,0};             // 1-5


    for(i=0;i<NUMBER_OF_GATES;i++)
    {
        parameter_to_be_tuned.heading_after_gate[i] = heading_after_gates_temp[i]/180.0*PI;
        parameter_to_be_tuned.distance_after_gate[i] = distance_after_gates_temp[i];
        parameter_to_be_tuned.height_after_gate[i] = height_after_gates_temp[i];
        parameter_to_be_tuned.approach_after_gate[i] = approach_after_gates_temp[i];
    }

}


void command_run() {
  
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        counter_autopilot_mode = 0;
        time_autopilot_mode = 0;
        primitive_in_use = NO_PRIMITIVE;
        state_lower_level = PREPARE_CM; //PREPARE_CM;
        state_upper_level = FIRST_PART;
        states_race.gate_counter_in_second_part = 0;
        states_race.gate_counter_in_third_part = 0;
        replay_flag = 0;
        approach_first_part = FALSE;
        init_heading = stateGetNedToBodyEulers_f()->psi;
        gate_counter = 0;
		flag_clock3 = 0;
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
                state_lower_level =  HOVER_CM;
				state_upper_level = FIFTH_PART;
            }
            break;
        case HOVER_CM:
            hover();
            if (time_primitive > HOVER_TIME)
            {
                if (previous_lower_level == TAKE_OFF_CLOSE_LOOP_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = WAIT_FOR_DETECTION_CM;
                    state_upper_level = SECOND_PART;
                }
            }
            break;


        default:
            break;
    }
}



void second_part_logic()
{

    // if we pass through 100 gates, we can change to third part
    if ( states_race.gate_counter_in_second_part == NUMBER_OF_GATES)
    {
        //states_race.gate_counter_in_second_part = 0;
        state_upper_level = THIRD_PART;
        state_lower_level = WAIT_FOR_DETECTION_CM;
        return;
    }

    switch (state_lower_level)
    {
        case WAIT_FOR_DETECTION_CM:
            hover();
            if(time_primitive < HOVER_TIME)
                break;
            if(states_race.gate_detected == FALSE)
            {
                hover();
            }
            else
            {
                previous_lower_level = WAIT_FOR_DETECTION_CM;
                state_lower_level = ADJUST_POSITION_CM;
            }

            if (time_gate_detected > 3 && states_race.gate_detected == FALSE)
            {
                // gate is lost
                previous_lower_level = WAIT_FOR_DETECTION_CM;
                state_lower_level = SEARCH_GATE_CM;
            }
            break;


        case ADJUST_POSITION_CM:
            if (states_race.gate_detected == FALSE && time_gate_detected > 3)
            {
                // lost gate
                previous_lower_level = ADJUST_POSITION_CM;
                state_lower_level = WAIT_FOR_DETECTION_CM;
                break;
            }
            if(states_race.ready_pass_through == FALSE) {
                adjust_position(current_z_gate);
            }
            else
            {
                states_race.distance_before_gate = current_y_gate;
                //state_lower_level = GO_THROUGH_CM;
                previous_lower_level = ADJUST_POSITION_CM;
                state_lower_level = GO_THROUGH_CM;
            }
            break;


        case GO_THROUGH_CM:
            states_race.time_to_go_straight = (states_race.distance_before_gate +
                    parameter_to_be_tuned.distance_after_gate[states_race.gate_counter_in_second_part])
                                              /(CONSTANT_VELOCITY_STRAIGHT*1.2);
            go_straight(CONSTANT_VELOCITY_STRAIGHT);
            if (time_primitive > states_race.time_to_go_straight)
            {
                previous_lower_level = GO_THROUGH_CM;
                state_lower_level = TURN_CM;
            }
            break;


        case HOVER_CM:
            hover();
            if (time_primitive > HOVER_TIME)
            {
                if (previous_lower_level == GO_THROUGH_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = TURN_CM;
                }
                else if (previous_lower_level == TURN_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level =	APPROACH_GATE_CM;
                }
            }
            break;


        case TURN_CM:
            if (previous_lower_level == GO_THROUGH_CM)
            {
                change_heading_absolute(parameter_to_be_tuned.heading_after_gate[gate_counter]);  // !!!!!!!!!!
                if (states_race.turning == FALSE)
                {
                    previous_lower_level = TURN_CM;
                    state_lower_level = APPROACH_GATE_CM;
                }
            }

            if (previous_lower_level == APPROACH_GATE_CM)
            {
                change_heading_absolute(parameter_to_be_tuned.heading_after_gate[gate_counter]+3.14/2);   //!!!!!!!!!!!!!!!!!
                if (states_race.turning == FALSE)
                {
                    previous_lower_level = TURN_CM;
                    state_lower_level = WAIT_FOR_DETECTION_CM;
                    gate_counter++;
                    if (gate_counter == 2)
                    {
                        gate_counter = 0;
                    }
                }
            }

            break;
		case APPROACH_GATE_CM:
			go_straight(CONSTANT_VELOCITY_STRAIGHT);
            if (time_primitive > parameter_to_be_tuned.approach_after_gate[gate_counter])
            {
                previous_lower_level = APPROACH_GATE_CM;
                state_lower_level = TURN_CM;
            }
        default:
            break;
    }
}



void third_part_logic()
{
    // zigzag
    if ( states_race.gate_counter_in_third_part == NUMBER_OF_ZIGZAG)
    {
        state_upper_level = FOURTH_PART;
        return;
    }

    switch (state_lower_level) {

        case WAIT_FOR_DETECTION_CM:
            hover();
			if (time_primitive < HOVER_TIME)
				break;
            if (states_race.gate_detected == FALSE) {
                hover();
            }
            else {
                previous_lower_level = WAIT_FOR_DETECTION_CM;
                state_lower_level = ADJUST_POSITION_CM;
            }
            break;


        case ADJUST_POSITION_CM:
            if (states_race.gate_detected == FALSE && time_gate_detected > 0.5) {
                // lost gate
                previous_lower_level = ADJUST_POSITION_CM;
                state_lower_level = WAIT_FOR_DETECTION_CM;
                break;
            }
            if (states_race.ready_pass_through == FALSE) {
                adjust_position(current_z_gate);
            }
            else {
                states_race.distance_before_gate = current_y_gate;
                //state_lower_level = GO_THROUGH_CM;
                previous_lower_level = ADJUST_POSITION_CM;
                state_lower_level = GO_THROUGH_CM;
            }
            break;


        case GO_THROUGH_CM:
            states_race.time_to_go_straight = (states_race.distance_before_gate +
                    parameter_to_be_tuned.distance_after_zigzag[states_race.gate_counter_in_third_part])
                                              / (CONSTANT_VELOCITY_STRAIGHT * 1.2);
            go_straight(CONSTANT_VELOCITY_STRAIGHT);
            if (time_primitive > states_race.time_to_go_straight) {
                previous_lower_level = GO_THROUGH_CM;
                state_lower_level = HOVER_CM;
            }
            break;


        case HOVER_CM:
            hover();
            if (time_primitive > HOVER_TIME) {
                if (previous_lower_level == GO_THROUGH_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = TURN_CM;

                }
                else if(previous_lower_level == TURN_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = SEARCH_GATE_CM;
                }

            }
            break;

        case TURN_CM:
            if(parameter_to_be_tuned.heading_after_gate[states_race.gate_counter_in_third_part] != 0)
            {
                change_heading_hover(parameter_to_be_tuned.heading_after_zigzag[states_race.gate_counter_in_third_part]);
                if (states_race.turning == FALSE)
                {
                    // turning is finished, go to next gate
                    previous_lower_level = TURN_CM;
                    state_lower_level = HOVER_CM;
                }
            }
            else
            {
                previous_lower_level = TURN_CM;
                state_lower_level = SEARCH_GATE_CM;
            }
            break;

        case SEARCH_GATE_CM:

            if (parameter_to_be_tuned.search_gate_velocity_in_zigzag[states_race.gate_counter_in_third_part] != 0)
            {
                v_x_search_gate = parameter_to_be_tuned.search_gate_velocity_in_zigzag[states_race.gate_counter_in_third_part];
                v_y_search_gate = -0.5;
                adjusting_time_search_gate = parameter_to_be_tuned.search_time_in_zigzag[states_race.gate_counter_in_third_part];
            }
            else
            {
                // don't need to search gate
                previous_lower_level = SEARCH_GATE_CM;
                state_lower_level = WAIT_FOR_DETECTION_CM;
                states_race.gate_counter_in_third_part++;    //!!!!!!!!!!!!!! finally every gate will arrive here
                init_pos_filter = 1;
                break;
            }
            left_right_back(v_x_search_gate,v_y_search_gate);
            if (time_primitive > adjusting_time_search_gate)
            {
                previous_lower_level = SEARCH_GATE_CM;
                state_lower_level = WAIT_FOR_DETECTION_CM;
                states_race.gate_counter_in_third_part++;    //!!!!!!!!!!!!!! finally every gate will arrive here
                init_pos_filter = 1;
            }
            break;
        default:
            break;
    }
}


void fourth_part_logic() {
switch (state_lower_level)
{
		case HOVER_CM:
            hover();
            if (time_primitive > HOVER_TIME) {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = CHANGE_HEADING_ABSOLUTE_CM;
            }
            break;
		case CHANGE_HEADING_ABSOLUTE_CM:
			change_heading_absolute(0.0);
         	if (states_race.turning == FALSE)
                {
                    previous_lower_level = CHANGE_HEADING_ABSOLUTE_CM;
                    state_lower_level = SET_THETA_CM;
                }
		case SET_THETA_CM:
            set_theta(-5.0/180.0*3.14);//this function is not in 'flight plan in guided mode'
            if (time_primitive > 5) {
					
			}
			break;
		default:
			break;
}
}


void fifth_part_logic()
{
	if (flag_clock3 == 0)
	{
       counter_temp3 =0;
	   time_temp3 = 0;
       guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
       guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
	   flag_clock3 = 1;
	}	
	
	if (time_temp3 < 1)
			return;
	else if(time_temp3<200)
	{ guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
		guidance_h_set_guided_heading(0);
	}
	/*else if (time_temp3<7)
	{
       guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
       guidance_h_set_guided_body_vel(1,0);
	}
	else if (time_temp3 < 11)
	{
       guidance_h_set_guided_body_vel(-1,0);
	}
    else 
    {	
       guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
       guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
	}*/

}
