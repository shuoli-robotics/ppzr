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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PUX RPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/set_velocity_guidance/set_velocity_guidance.h"
 * @author Shuo Li
 * This mudule is used to set velocity command to the drone.
 */

#ifndef SET_VELOCITY_GUIDANCE_H
#define SET_VELOCITY_GUIDANCE_H
#include "state.h"
//#include "autopilot.h"

extern uint8_t counter_global;
extern uint8_t counter;
extern uint8_t counter_in_mode;

extern uint8_t previous_mode;
extern uint8_t current_mode;
extern bool mode_change_flag;

extern float time_global;
extern float time_in_mode;

//extern void if_mode_change(void);
extern void set_counter_init(void);
extern void counter_auto(void);
extern void print_state(void);
extern void set_command(void);
#endif

