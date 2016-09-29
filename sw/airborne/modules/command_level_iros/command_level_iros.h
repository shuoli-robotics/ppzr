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

#ifndef COMMAND_LEVEL_IROS_H
#define COMMAND_LEVEL_IROS_H

#ifndef CONSTANT_VELOCITY_STRAIGHT
#define CONSTANT_VELOCITY_STRAIGHT 0.8
#endif

extern void command_run(void);  // 20HZ
extern void command_init(void);

enum states_lower_level{WAIT_FOR_DETECTION_CM,ADJUST_POSITION_CM,GO_THROUGH_CM,HOVER_CM,
TURN_CM,SEARCH_GATE_CM,TAKE_OFF_CM,LAND_CM,GO_STRAIGHT_CM};
enum states_upper_level{FIRST_PART,SECOND_PART,THIRD_PART};

struct parameters_to_be_tuned{
    float distance_first_gate;
    float distance_second_gate;
    float distance_third_gate;
    float distance_fourth_gate;
    float heading_after_first_gate;
    float heading_after_second_gate;
    float heading_after_third_gate;
    float heading_after_fourth_gate;
};

extern enum states_lower_level state_lower_level;
extern enum states_upper_level state_upper_level;

extern struct parameters_to_be_tuned parameter_to_be_tuned;

#endif

