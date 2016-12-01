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
 * @file "modules/velocity_control_indi/velocity_control_indi.h"
 * @author Shuo Li
 * guidance loop
 */

#include "firmwares/rotorcraft/autopilot.h"
#ifndef VELOCITY_CONTROL_INDI_H
#define VELOCITY_CONTROL_INDI_H

struct guidance_module_st {
    float phi_pgain;        ///< The roll P gain on the err_vx
    float phi_igain;        ///< The roll I gain on the err_vx_int
    float phi_dgain;
    float theta_pgain;      ///< The pitch P gain on the err_vy
    float theta_igain;      ///< The pitch I gain on the err_vy_int
    float theta_dgain;
    float desired_vx;         ///< The desired velocity in the x direction (cm/s)
    float desired_vy;         ///< The desired velocity in the y direction (cm/s)
    float desired_z;
    float desired_heading;

    float err_vx_int;         ///< The integrated velocity error in x direction (m/s)
    float err_vy_int;         ///< The integrated velocity error in y direction (m/s)
    float err_vx_deri;
    float err_vy_deri;
    struct Int32Eulers cmd;   ///< The commands that are send to the hover loop    It is INT!!!
};

extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_loop_pid(void);

#endif

