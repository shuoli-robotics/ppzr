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

void first_part_logic(void);
void second_part_logic(void);
void third_part_logic(void);
void fourth_part_logic(void);
//float choose_heading_after_passing_through_gate(void);
//float choose_distance_after_gate(void);

int counter_of_step;
bool replay_flag;
enum states_lower_level state_lower_level = WAIT_FOR_DETECTION_CM;
enum states_upper_level state_upper_level = SECOND_PART;

struct parameters_to_be_tuned parameter_to_be_tuned;

void command_init(){
    int i;

    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    states_race.time_to_go_straight = 0;
    states_race.distance_before_gate = 1.5;
    //gate_counter_in_second_part = 0;
    //record_command = 0;
    replay_flag = 0;

    float distance_after_zigzag_temp[100] = {           0.5,0.5,0.5,0.2,0.5,    // 1-5
                                                        0.5,0.5,0.5,0.5,0.5,    // 6-10
                                                        0.5,0.5,0.5,0.5,0.5};  // 11-15

    float heading_after_zigzag_temp[100] = {             0,70,0,0,0,    // 1-5
                                                         0.5,0.5,0.5,0.5,0.5,    // 6-10
                                                         0.5,0.5,0.5,0.5,0.5};  // 11-15
    for(i = 0;i <NUMBER_OF_ZIGZAG; i++)
    {
        parameter_to_be_tuned.distance_after_zigzag[i] = distance_after_zigzag_temp[i];
        parameter_to_be_tuned.heading_after_zigzag[i] = heading_after_zigzag_temp[i]/180.0*PI;
    }
    // delta heading after passing through each gate (degree!)

    float heading_after_gates_temp[100] = {            -45,-40,0,0,-90,      // 1-5
                                                       0,0,0,0,0,           // 6-10
                                                       0,0};                // 11-15

    float distance_after_gates_temp[100] = {            0.8,0.5,0.2,0.8,0.5,    // 1-5
                                                        0.5,0.5,0.5,0.5,0.5,    // 6-10
                                                        0.5,0.5,0.5,0.5,0.5};  // 11-15

    float height_after_gates_temp[100]   ={             -2,-1.5,0,0,-1,            // absolute height
                                                        0,0,0,0,0};             // 1-5

    float approach_after_gates_temp[100]   ={             0,0,0,0,0,            // time for approach
                                                          0,0,0,0,0};             // 1-5


    for(i=0;i<NUMBER_OF_GATES;i++)
    {
        parameter_to_be_tuned.heading_after_gate[i] = heading_after_gates_temp[i]/180.0*PI;
        parameter_to_be_tuned.distance_after_gate[i] = distance_after_gates_temp[i];
        parameter_to_be_tuned.height_after_gate[i] = height_after_gates_temp[i];
        parameter_to_be_tuned.approach_after_gate[i] = approach_after_gates_temp[i];
    }

   // parameter_to_be_tuned.heading_after_first_part = 90.0/180.0*PI;
}


void command_run() {
  
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        counter_autopilot_mode = 0;
        time_autopilot_mode = 0;
        primitive_in_use = NO_PRIMITIVE;
        state_lower_level = PREPARE_CM;
        state_upper_level = FIRST_PART;
        states_race.gate_counter_in_second_part = 0;
        states_race.gate_counter_in_third_part = 0;
        replay_flag = 0;
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
                state_lower_level = TAKE_OFF_OPEN_LOOP_CM;
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
                if (previous_lower_level == GO_STRAIGHT_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = TURN_CM;
                }
                if (previous_lower_level == TURN_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = WAIT_FOR_DETECTION_CM;
                    state_upper_level = SECOND_PART;
                }
            }
            break;
        case GO_STRAIGHT_CM:
            go_straight(CONSTANT_VELOCITY_STRAIGHT);
            if (time_primitive > STRAIGHT_TIME)
            {
                previous_lower_level = GO_STRAIGHT_CM;
                state_lower_level = HOVER_CM;
            }
            break;
        case TURN_CM:
            change_heading_hover(ANGLE_AFTER_HALF_GATE);
            if (states_race.turning == FALSE)
            {
                // turning is finished, go to next gate
                init_pos_filter = 1;
                previous_lower_level = TURN_CM;
                state_lower_level = HOVER_CM;
            }
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
            if (states_race.gate_detected == FALSE && time_gate_detected > 0.5)
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
                state_lower_level = HOVER_CM;
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
                    state_lower_level = ADJUST_HEIGHT_CM;
                }
                else if (previous_lower_level == ADJUST_HEIGHT_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = APPROACH_GATE_CM;
                }
                else if (previous_lower_level == APPROACH_GATE_CM)
                {
                    previous_lower_level = HOVER_CM;
                    state_lower_level = WAIT_FOR_DETECTION_CM;
                    init_pos_filter = 1;
                    states_race.gate_counter_in_second_part ++;     // every gate ends here
                }
            }
            break;


        case TURN_CM:
            //change_heading_hover(choose_heading_after_passing_through_gate());
	         change_heading_hover(parameter_to_be_tuned.heading_after_gate[states_race.gate_counter_in_second_part]);
            if (states_race.turning == FALSE)
            {
                // turning is finished, go to next gate

                previous_lower_level = TURN_CM;
                state_lower_level = HOVER_CM;

            }
            break;

        case ADJUST_HEIGHT_CM:
            if (parameter_to_be_tuned.height_after_gate[states_race.gate_counter_in_second_part] == 0)
            {
                previous_lower_level = ADJUST_HEIGHT_CM;
                state_lower_level = HOVER_CM;
            }
            else
            {
                go_up_down(parameter_to_be_tuned.height_after_gate[states_race.gate_counter_in_second_part]);
                    if (states_race.altitude_is_achieved == TRUE)
                    {
                        previous_lower_level = ADJUST_HEIGHT_CM;
                        state_lower_level = HOVER_CM;
                    }
            }
            break;

        case APPROACH_GATE_CM:
            if(parameter_to_be_tuned.approach_after_gate[states_race.gate_counter_in_second_part] == 0)
            {
                previous_lower_level = APPROACH_GATE_CM;
                state_lower_level = HOVER_CM;
            }
            else
            {
                go_straight(CONSTANT_VELOCITY_STRAIGHT);
                if(time_primitive > parameter_to_be_tuned.approach_after_gate[states_race.gate_counter_in_second_part])
                {
                    previous_lower_level = APPROACH_GATE_CM;
                    state_lower_level = HOVER_CM;
                }
            }
            break;

        case SEARCH_GATE_CM:
            search_gate();
            if (states_race.gate_detected == 1 && time_gate_detected > 0.5)
            {
                previous_lower_level = SEARCH_GATE_CM;
                state_lower_level = WAIT_FOR_DETECTION_CM;
            }
            break;
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
            left_right_back(-0.5,0.5);
            float adjusting_time;
            switch(states_race.gate_counter_in_third_part)
            {
                case 0: adjusting_time = 3;
                    break;
                case 1: adjusting_time = 3;
                    break;
                case 2: adjusting_time = 3;
                    break;
                case 3: adjusting_time = 3;
                    break;
            }

            if (time_primitive > adjusting_time)
            {
                previous_lower_level = SEARCH_GATE_CM;
                state_lower_level = WAIT_FOR_DETECTION_CM;
                states_race.gate_counter_in_third_part++;    //!!!!!!!!!!!!!! finally every gate will arrive here
                init_pos_filter = 1;
            }
        default:
            break;
    }
}


void fourth_part_logic()
{
    switch (state_lower_level)
    {

        case HOVER_CM:
            hover();
            if (time_primitive > 5) {
                previous_lower_level = HOVER_CM;
                state_lower_level = REPLAY_CM;
            }
            break;

        case REPLAY_CM:
            if( replay_flag != TRUE)
            {
                replay_flag = TRUE;
                replay_commands_start();
            }
            if (replay == FALSE)
            {
                replay_flag = FALSE;
                previous_lower_level = REPLAY_CM;
                state_lower_level = HOVER_CM;
            }
            break;
    }
}


//float choose_heading_after_passing_through_gate()
//{
//    float heading_turn =parameter_to_be_tuned.heading_after_gate[states_race.gate_counter_in_second_part];
//    return heading_turn;
//}

//float choose_distance_after_gate()
//{
//    float distance_after_gate = parameter_to_be_tuned.distance_after_gate[states_race.gate_counter_in_second_part];
//
//
//    return distance_after_gate;
//}