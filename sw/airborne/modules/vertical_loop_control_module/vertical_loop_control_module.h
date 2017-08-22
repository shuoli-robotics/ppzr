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
 * @file "modules/vertical_loop_control_module/vertical_loop_control_module.h"
 * @author Shuo Li
 * vertical controller in module mode
 */

#ifndef VERTICAL_LOOP_CONTROL_MODULE_H
#define VERTICAL_LOOP_CONTROL_MODULE_H
#include "state.h"

#define K_P_Z 0.3


#define K_P_V_Z 1

struct vertical_controler_status
{
		float k_p_z;
		float k_p_v_z;
		float take_off_altitude;
};

extern void guidance_v_module_init(void);
extern void guidance_v_module_run(bool in_flight);
extern void guidance_v_module_read_rc(void);
extern void guidance_v_module_enter(void);

#endif

