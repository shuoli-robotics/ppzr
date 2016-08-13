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
 * @file "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
 * @author Shuo Li
 * This module is used to control velocity only in MODULE mode
 */

#ifndef GUIDANCE_LOOP_VELOCITY_AUTONOMOUS_RACE_H
#define GUIDANCE_LOOP_VELOCITY_AUTONOMOUS_RACE_H

#include "math/pprz_algebra_int.h"

struct guidance_module_st {
    int32_t phi_pgain;        ///< The roll P gain on the err_vx
    int32_t phi_igain;        ///< The roll I gain on the err_vx_int
    int32_t theta_pgain;      ///< The pitch P gain on the err_vy
    int32_t theta_igain;      ///< The pitch I gain on the err_vy_int
    float desired_vx;         ///< The desired velocity in the x direction (cm/s)
    float desired_vy;         ///< The desired velocity in the y direction (cm/s)

    float err_vx_int;         ///< The integrated velocity error in x direction (m/s)
    float err_vy_int;         ///< The integrated velocity error in y direction (m/s)
    struct Int32Eulers cmd;   ///< The commands that are send to the hover loop    It is INT!!!
};

extern struct guidance_module_st guidance_module;
extern float guidance_h_module_speed_error_x;
extern float guidance_h_module_speed_error_y;

extern void guidance_h_module_init(void);
extern void guidance_loop_pid(void);
extern void guidance_loop_set_heading(float heading);
extern void guidance_loop_set_velocity(float vx, float vy);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_enter(void);
extern float phi_desired_f;
extern float theta_desired_f;
#endif

