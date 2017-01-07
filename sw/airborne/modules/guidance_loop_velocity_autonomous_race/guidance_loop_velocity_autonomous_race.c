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
 * @file "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.c"
 * @author Shuo Li
 * This module is used to control velocity only in MODULE mode
 */

#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
//#include "modules/replay_commands/replay_commands.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/command_level_iros/command_level_iros.h"
//#include "modules/replay_commands/replay_commands.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"

//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.h"


#include<stdio.h>
#include<stdlib.h>

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

#ifndef PHI_PGAIN
#define PHI_PGAIN 0.2
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef PHI_IGAIN
#define PHI_IGAIN 0.07
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef PHI_DGAIN
#define PHI_DGAIN 0.6
#endif
PRINT_CONFIG_VAR(VISION_PHI_DGAIN)

#ifndef THETA_PGAIN
#define THETA_PGAIN 0.2
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)


#ifndef THETA_IGAIN

#define THETA_IGAIN 0.07
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)

#ifndef THETA_DGAIN
#define THETA_DGAIN 0.6
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef DESIRED_VX
#define DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX)

#ifndef DESIRED_VY
#define DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY)

struct guidance_module_st guidance_module = {

.phi_pgain = PHI_PGAIN,
.phi_igain = PHI_IGAIN,
.phi_dgain = PHI_DGAIN,
.theta_pgain = THETA_PGAIN,
.theta_igain = THETA_IGAIN,
.theta_dgain = THETA_DGAIN,
        .desired_vx = DESIRED_VX,
        .desired_vy = DESIRED_VY,

};

float guidance_h_module_speed_error_x;
float guidance_h_module_speed_error_y;
float guidance_h_module_speed_error_x_previous = 0;
float guidance_h_module_speed_error_y_previous =0;
float phi_desired_f;
float theta_desired_f;
uint8_t previous_mode;
uint8_t current_mode;

void guidance_h_module_init(void) {
    guidance_module.err_vx_int = 0;
    guidance_module.err_vy_int = 0;
    guidance_module.cmd.phi = 0;
    guidance_module.cmd.theta = 0;
    guidance_module.cmd.psi = stateGetNedToBodyEulers_i()->psi;
    previous_mode = autopilot_mode;
    current_mode =  autopilot_mode;
}

void guidance_h_module_enter(void)
{
    /* Reset the integrated errors */
    guidance_module.err_vx_int = 0;
    guidance_module.err_vy_int = 0;

    /* Set rool/pitch to 0 degrees and psi to current heading */
    guidance_module.cmd.phi = 0;
    guidance_module.cmd.theta = 0;
    guidance_module.cmd.psi = stateGetNedToBodyEulers_i()->psi;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
    // TODO: change the desired vx/vy
}

void guidance_h_module_run(bool in_flight)    // this function is called in higher level in guidance_h.c
{
	/*if(replay == 1)*/
	/*{*/
		/*stabilization_attitude_set_rpy_setpoint_i(&guidance_replay);*/
		/*printf("replay setpoints set -------------------------------------------------\n");*/
	/*}*/
	/*else{*/
    stabilization_attitude_set_rpy_setpoint_i(&guidance_module.cmd);
    //printf("My guidance module is running\n");
	/*}*/
    /*stab_att_sp_euler.phi = phi_desired_f;*/
    /*stab_att_sp_euler.theta = theta_desired_f;*/
    /* Run the default attitude stabilization */
    stabilization_attitude_run(in_flight);
}

/** This function is used to calculate needed phi and theta
 * @param vel_x is desired velocity in earth coordinates
 * @param vel_y is desired velocity in earth coordinates
 */
void guidance_loop_pid()
{
    current_mode = autopilot_mode;
    //current_guidance_h_mode = guidance_h.mode;
    if (current_mode != previous_mode)
    {
        guidance_module.err_vx_int = 0;
        guidance_module.err_vy_int = 0;
    }
    //printf("My guidance loop PID is running!\n");
    /* Check if we are in the correct AP_MODE before setting commands */
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }
    float current_vel_x = stateGetSpeedNed_f()->x;
    float current_vel_y = stateGetSpeedNed_f()->y;
    /* Calculate the error */
    guidance_h_module_speed_error_x = guidance_module.desired_vx - current_vel_x;
    guidance_h_module_speed_error_y = guidance_module.desired_vy - current_vel_y;

    if (state_lower_level == TURN_CM || state_lower_level == TAKE_OFF_OPEN_LOOP_CM)
    {
        guidance_h_module_speed_error_x = 0;
        guidance_h_module_speed_error_y = 0;
        guidance_module.err_vx_int = 0;
        guidance_module.err_vy_int = 0;
    }

    /* Calculate the integrated errors (TODO: bound??) */
    guidance_module.err_vx_int += guidance_h_module_speed_error_x / 512;
    guidance_module.err_vy_int += guidance_h_module_speed_error_y / 512;



    guidance_module.err_vx_deri = (guidance_h_module_speed_error_x - guidance_h_module_speed_error_x_previous)*512;
    guidance_module.err_vy_deri = (guidance_h_module_speed_error_y - guidance_h_module_speed_error_y_previous)*512;

    struct FloatVect2 cmd_f;
    /* Calculate the commands */
    cmd_f.y   = guidance_module.phi_pgain * guidance_h_module_speed_error_y
                               + guidance_module.phi_igain * guidance_module.err_vy_int
                               + guidance_module.phi_dgain * guidance_module.err_vy_deri;
    cmd_f.x   = -(guidance_module.theta_pgain * guidance_h_module_speed_error_x
                                 + guidance_module.theta_igain * guidance_module.err_vx_int
                                 +guidance_module.theta_dgain * guidance_module.err_vx_deri);
    float psi = stateGetNedToBodyEulers_f()->psi;
    float s_psi = sinf(psi);
    float c_psi = cosf(psi);
    phi_desired_f = s_psi * cmd_f.x + c_psi * cmd_f.y;
    theta_desired_f = c_psi * cmd_f.x - s_psi * cmd_f.y;
    if (primitive_in_use == SET_ATTITUDE)
			return;
    guidance_module.cmd.phi = BFP_OF_REAL(phi_desired_f, INT32_ANGLE_FRAC);
    guidance_module.cmd.theta = BFP_OF_REAL(theta_desired_f, INT32_ANGLE_FRAC);

    /* Bound the roll and pitch commands */
    BoundAbs(guidance_module.cmd.phi, CMD_OF_SAT);
    BoundAbs(guidance_module.cmd.theta, CMD_OF_SAT);
    guidance_h_module_speed_error_x_previous = guidance_h_module_speed_error_x;
    guidance_h_module_speed_error_y_previous = guidance_h_module_speed_error_y;
    previous_mode = current_mode;
}

void guidance_loop_set_heading(float heading){
    guidance_module.cmd.psi = BFP_OF_REAL(heading, INT32_ANGLE_FRAC);
}

void guidance_loop_set_velocity(float vx_earth, float vy_earth){
    guidance_module.desired_vx = vx_earth;
    guidance_module.desired_vy = vy_earth;
}

void guidance_loop_set_theta(float desired_theta)
{
    guidance_module.cmd.theta = BFP_OF_REAL(desired_theta, INT32_ANGLE_FRAC);
}


void guidance_loop_set_phi(float desired_phi)
{
    guidance_module.cmd.phi= BFP_OF_REAL(desired_phi, INT32_ANGLE_FRAC);
}
