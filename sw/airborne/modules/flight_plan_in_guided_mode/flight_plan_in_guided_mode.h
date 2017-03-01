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
#include "modules/vertical_loop_control_module/vertical_loop_control_module.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#define NO_PRIMITIVE             0
#define HOVER                    1
#define GO_STRAIGHT              2
#define CHANGE_HEADING_HOVER     3
#define CIRCLE                   4
#define GO_LEFT_RIGHT            5
#define SET_VELOCITY_TEST        6
#define GO_UP_DOWN               7
#define ADJUST_POSITION          8
#define ARC                      9
#define SEARCH_GATE              10
#define TAKE_OFF                 11
#define LAND                     12
#define ADJUST_HEADING           13
#define LEFT_RIGHT_BACK          14
#define HOLD_ALTITUDE            15
#define CHANGE_HEADING_ABSOLUTE  16
#define SET_THETA                17
#define SET_PHI                  18
#define SET_ATTITUDE             19
#define CALCULATE_ATTITUDE_BIAS  20 
#define ARC_OPEN_LOOP            21 
#define HOVER_AT_ORIGIN          22 
#define PREPARE_BEFORE_TAKE_OFF  23 


#ifndef PREPARE_TIME
#define PREPARE_TIME 3
#endif

#ifndef SAMPLE_NUM 
#define SAMPLE_NUM  20
#endif

struct acceleration{
		double ax;
		double ay;
		double az;
};

 extern bool arc_is_finished;
 extern int primitive_in_use;
 extern float init_heading;
 extern bool adjust_position_mask;
 extern void flight_plan_in_guided_mode_init(void);
 extern void display_information(void);
 extern void hover(void);
 extern bool go_straight(float theta,float distance,double ref_y);
 extern void change_heading_hover(float derta_psi);
 extern void circle(float radius, float planned_time);
 extern void go_left_right(float velocity);
 extern void set_velocity_test(float vx_earth_t,float vy_earth_t);
 extern void go_up_down(float derta_altitude);
 extern void adjust_position(float derta_altitude);
 extern void arc(float radius, float planned_time, float desired_angle_change);
 extern void search_gate(void);
 extern void take_off(float desired_altitude);
 extern void land(void);
 extern void adjust_heading(float delta_heading);
 extern void left_right_back(float velocity_in_body_x,float velocity_in_body_y);
 extern void hold_altitude(float desired_altitude);
extern  void change_heading_absolute(float psi);
extern void set_theta(float desired_theta);
extern void set_phi(float desired_phi);
extern void set_attitude(float desired_theta,float desired_phi);
extern void calculate_attitude_average(double * p_theta,double *p_phi,struct acceleration* p_accel);
extern bool arc_open_loop(double radius,double theta,float delta_psi);
extern bool hover_at_origin(void);
extern bool prepare_before_take_off(double prepare_time);
extern int sample_pointer;
#endif

