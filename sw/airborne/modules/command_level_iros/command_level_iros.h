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

#ifndef CONSTANT_VELOCITY_STRAIGHT
#define CONSTANT_VELOCITY_STRAIGHT 2.5
#endif

#ifndef NUMBER_OF_GATES
#define NUMBER_OF_GATES 6               //second part
#endif

#ifndef ANGLE_AFTER_HALF_GATE
#define ANGLE_AFTER_HALF_GATE 105.0     //degree
#endif

#ifndef VELOCITY_IN_FIRST_PART
#define VELOCITY_IN_FIRST_PART 0.5
#endif




#ifndef HOVER_TIME
#define HOVER_TIME 6
#endif

#ifndef NUMBER_OF_ZIGZAG
#define NUMBER_OF_ZIGZAG 5
#endif

#ifndef STRAIGHT_TIME
#define STRAIGHT_TIME 10.6                     //10.6 for battery 10
#endif

#ifndef TAKE_OFF_ALTITUDE
#define TAKE_OFF_ALTITUDE -1.5
#endif

#ifndef APPROACH_TIME
#define APPROACH_TIME 3.5
#endif


#ifndef THETA_TIME 
#define THETA_TIME 10 
#endif


#ifndef PHI_TIME 
#define PHI_TIME 3  
#endif

#ifndef HEIGHT_FIRST_PART
#define HEIGHT_FIRST_PART -2.5 
#endif

extern void command_run(void);  // 20HZ
extern void command_init(void);

enum states_lower_level{HOVER_AT_ORIGIN_CM,HOVER_CM,
TAKE_OFF_OPEN_LOOP_CM,TAKE_OFF_CLOSE_LOOP_CM,LAND_CM,GO_STRAIGHT_CM,PREPARE_CM,
     ZIGZAG_CM,ARC_CM}; 
enum states_upper_level{FIRST_PART,SECOND_PART,THIRD_PART,FOURTH_PART,FIFTH_PART};


extern enum states_lower_level state_lower_level;
extern enum states_upper_level state_upper_level;


extern double theta_bias;
extern double phi_bias;
extern uint8_t previous_lower_level;
extern double theta_hover;
extern double phi_hover;
extern struct acceleration accel_bias;
extern struct acceleration accel_hover;
extern int arc_counter;
#endif

