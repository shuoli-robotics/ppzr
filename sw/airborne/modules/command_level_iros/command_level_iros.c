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
#include "modules/kalman_filter/kalman_filter.h"

#define PI 3.1415926

uint8_t previous_mode;
uint8_t current_mode;
uint8_t previous_lower_level;

double theta_bias;
double phi_bias;
double theta_hover;
double phi_hover;
struct acceleration accel_bias;
struct acceleration accel_hover;
double desired_theta;
double desired_phi;

int arc_counter;
int arc_passed;
float reference_y;

void first_part_logic(void);
void second_part_logic(void);
void third_part_logic(void);
void fourth_part_logic(void);
void fifth_part_logic(void);

bool flag_init_geo;
enum states_lower_level state_lower_level ;
enum states_upper_level state_upper_level ;


float gate_initial_position_y[] = {3.5,2,3.5};
float turn_point[] = {6.0,3.8,3.8};
float gate_initial_heading[] = {0, 135.0/180*3.14,0.0};

float gate_altitude[] = {-2.5,-2.5,-1.5};
float open_loop_altitude[] = {-2.5,-2.5,-1.5};



int   flag_arc_right[] = {1,              -1,           -1};
float arc_radius[] =     {2.5,             1.0,           1.0};
float delta_arc_angle[] = {90.0/180*3.14,135.0/180*3.14};


int   flag_2_arc_right[] = {0,             0,            0};
float two_arc_radius[] =     { 1.25,          1.0,           0};


int   flag_zig_zag_right[] = {1,0,0};
int   flag_zig_zag_break[] = {1,0,0};
float zig_zag_desired_y[] = {5.0,0,0};
float zig_zag_break_time[] = {2,0,0};
float zig_zag_break_angle[] = {7.0/180*3.14,0,0};
float zig_zag_desired_theta[] = {-5.0/180*3.14,0,0};
float zig_zag_max_roll[] = {20.0/180*3.14,20.0/180*3.14,0};

struct race_states race_state;

//temp fix for data set recording
int init_pos_filter = 0;

void command_init(){
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    init_heading = stateGetNedToBodyEulers_f()->psi;
	race_state.flag_in_open_loop = TRUE;
	race_state.gate_counter = 0;
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
        init_heading = stateGetNedToBodyEulers_f()->psi;
		flag_init_geo = FALSE;
		arc_passed = 0;
		arc_counter = 0;
		race_state.flag_in_open_loop = TRUE;
		race_state.gate_counter = 0;
		race_state.current_initial_x =  gate_initial_position_y[race_state.gate_counter];
		race_state.current_initial_heading= gate_initial_heading[race_state.gate_counter] ;
    }
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }


	if (race_state.flag_in_open_loop == TRUE)
	{
		race_state.current_initial_x =  gate_initial_position_y[race_state.gate_counter];
		race_state.current_initial_heading= gate_initial_heading[race_state.gate_counter] ;
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
				if(prepare_before_take_off(1.0) == TRUE)
                state_lower_level = TAKE_OFF_OPEN_LOOP_CM;
            break;
			
		case TAKE_OFF_OPEN_LOOP_CM:
			if(take_off())
			{
				previous_lower_level = TAKE_OFF_OPEN_LOOP_CM;
				state_lower_level =  GO_STRAIGHT_CM;
				state_upper_level =  SECOND_PART;
				/*state_upper_level =  FOURTH_PART;*/
			}
			break;
		default:
			break;

    }
}



void second_part_logic()
{
	switch(state_lower_level)
	{
			case GO_STRAIGHT_CM:

					printf("aaaaaaaa\n");
					if (go_through_gate(-5.0/180*PI))
					{
							if(flag_arc_right[race_state.gate_counter] != 0) 
							{
							state_lower_level =  ARC_CM;
							race_state.flag_in_open_loop = TRUE;
							race_state.current_arc_radius = arc_radius[race_state.gate_counter];
							race_state.current_arc_delta_psi= delta_arc_angle[race_state.gate_counter];
							race_state.current_arc_flag_right = flag_arc_right[race_state.gate_counter];
							}
							/*else if(flag_zig_zag_right[race_state.gate_counter]==1 || flag_zig_zag_right[race_state.gate_counter]==-1 )*/
							/*{*/
									/*state_lower_level =  ZIGZAG_CM;*/
									/*race_state.flag_in_open_loop = TRUE;*/
									/*race_state.current_zigzag_break_time = zig_zag_break_time[race_state.gate_counter];*/
									/*race_state.current_zigzag_break_angle= zig_zag_break_angle[race_state.gate_counter];*/
									/*race_state.current_zigzag_flag_right = flag_zig_zag_right[race_state.gate_counter];*/
									/*race_state.current_zigzag_desired_y= zig_zag_desired_y[race_state.gate_counter];*/
									/*race_state.current_zigzag_desired_theta= zig_zag_desired_theta[race_state.gate_counter];*/
									/*race_state.current_zigzag_max_roll= zig_zag_max_roll[race_state.gate_counter];*/
									/*race_state.current_zigzag_flag_break = flag_zig_zag_break[race_state.gate_counter];*/
									/*printf("RRRRRRRRRRRRRRRRRRR\n");*/
							/*}*/
							
							else if(flag_2_arc_right[race_state.gate_counter] != 0)
							{
									state_lower_level =  TWO_ARCS_CM;
									race_state.flag_in_open_loop = TRUE;
									race_state.current_2_arcs_radius= two_arc_radius[race_state.gate_counter];
									race_state.current_2_arcs_flag_right = flag_2_arc_right[race_state.gate_counter];
							}
					}
					break;
			case ARC_CM:
				if(	arc_open_loop(race_state.current_arc_radius,-5.0/180*3.14,race_state.current_arc_delta_psi,race_state.current_arc_flag_right))
				{
							previous_mode = ARC_CM;
							race_state.flag_in_open_loop = FALSE;
							state_lower_level = GO_STRAIGHT_CM;
				}
				break;
			case ZIGZAG_CM:
				/*printf("AAAAAAAAAAAAAAAA\n");*/
				/*if(zigzag_open_loop(race_state.current_zigzag_desired_y,race_state.current_zigzag_desired_theta,race_state.current_zigzag_max_roll,race_state.current_zigzag_break_angle,race_state.current_zigzag_break_time,race_state.current_zigzag_flag_right,race_state.current_zigzag_flag_break)) */
				/*{*/
							/*previous_mode = ZIGZAG_CM;*/
							/*race_state.flag_in_open_loop = FALSE;*/
							/*state_lower_level = GO_STRAIGHT_CM;*/
				/*}*/
				break;
			case TWO_ARCS_CM:
				if(two_arcs_open_loop(race_state.current_2_arcs_radius,-5.0/180*3.14,race_state.current_2_arcs_flag_right))
				{
						previous_mode = TWO_ARCS_CM;
						race_state.flag_in_open_loop = FALSE;
						state_lower_level = GO_STRAIGHT_CM;
				}
				break;
			default:
					break;
	}
}

void third_part_logic()
{
	switch(state_lower_level)
	{

			case GO_STRAIGHT_CM:
					if (go_through_gate(-5.0/180*PI))
					{ printf("3rd part!-----------------------------\n");
							state_lower_level =  ARC_CM;
							race_state.flag_in_open_loop = TRUE;
					}
					break;
			default:
					break;
	}
}


void fourth_part_logic() {
		
		if(go_straight_test(2.0,-5.0/180*3.14))
		{
				state_lower_level = ZIGZAG_CM;
				state_upper_level = SECOND_PART;
				printf("In test go straight part\n");
				state_lower_level =  ZIGZAG_CM;
				race_state.flag_in_open_loop = TRUE;
				race_state.current_zigzag_break_time = zig_zag_break_time[race_state.gate_counter];
				race_state.current_zigzag_break_angle= zig_zag_break_angle[race_state.gate_counter];
				race_state.current_zigzag_flag_right = flag_zig_zag_right[race_state.gate_counter];
				race_state.current_zigzag_desired_y= zig_zag_desired_y[race_state.gate_counter];
				race_state.current_zigzag_desired_theta= zig_zag_desired_theta[race_state.gate_counter];
				race_state.current_zigzag_max_roll= zig_zag_max_roll[race_state.gate_counter];
				race_state.current_zigzag_flag_break = flag_zig_zag_break[race_state.gate_counter];
				printf("RRRRRRRRRRRRRRRRRRR\n");
		}
}


void fifth_part_logic()
{
}
