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
 * @file "modules/command_level_iros/command_level_iros.h"
 * @author Shuo Li
 * This module is the highest level
 */
#include "state.h"

#ifndef COMMAND_LEVEL_IROS_H
#define COMMAND_LEVEL_IROS_H


extern void command_run(void);  // 20HZ
extern void command_init(void);

enum states_lower_level{HOVER_AT_ORIGIN_CM,HOVER_CM,
TAKE_OFF_OPEN_LOOP_CM,TAKE_OFF_CLOSE_LOOP_CM,LAND_CM,GO_STRAIGHT_CM,PREPARE_CM,
     ZIGZAG_CM,ARC_CM}; 
enum states_upper_level{FIRST_PART,SECOND_PART,THIRD_PART,FOURTH_PART,FIFTH_PART};


extern enum states_lower_level state_lower_level;
extern enum states_upper_level state_upper_level;

struct race_states
{

		bool flag_in_open_loop;
		int gate_counter;
		float current_initial_x;
		float current_initial_heading;
		float current_arc_radius;
		float current_delta_psi;
		int current_flag_right;
};


extern double theta_bias;
extern double phi_bias;
extern uint8_t previous_lower_level;
extern double theta_hover;
extern double phi_hover;
extern struct acceleration accel_bias;
extern struct acceleration accel_hover;
extern int arc_counter;
extern struct race_states race_state;
extern float gate_initial_position_y[];
extern float turn_point[];
extern float arc_radius[];
extern float delta_arc_angle[];
extern float gate_initial_heading[];
extern float gate_altitude[] ;
extern float open_loop_altitude[];
#endif

