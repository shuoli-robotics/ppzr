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
 * @file "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
 * @author Shuo Li
 * This module is used to generate flight plan in guided mode
 */

#ifndef FLIGHT_PLAN_IN_GUIDED_MODE_H
#define FLIGHT_PLAN_IN_GUIDED_MODE_H
#include "flight_plan_clock.h"

#define NO_PRIMITIVE             0
#define HOVER                    1
#define GO_STRAIGHT              2
#define CHANGE_HEADING_HOVER     3
#define CIRCLE                   4
#define GO_LEFT_RIGHT            5
#define SET_VELOCITY_TEST        6
#define GO_UP_DOWN               7
#define ADJUST_POSITION          8


 extern int primitive_in_use;
 extern bool adjust_position_mask;
 extern void flight_plan_in_guided_mode_init(void);
 extern void display_information(void);
 extern void hover(void);
 extern void go_straight(float velocity);
 extern void change_heading_hover(float derta_psi,float planned_time);
 extern void circle(float radius, float planned_time);
 extern void go_left_right(float velocity);
 extern void set_velocity_test(float vx_earth_t,float vy_earth_t);
 extern void go_up_down(float derta_altitude);
 extern void adjust_position(float derta_altitude);

//void set_velocity_test(float vx_earth_t,float vy_earth_t,float planned_time);

#endif

