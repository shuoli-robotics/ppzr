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
 * @file "modules/vertical_loop_control_module/vertical_loop_control_module.c"
 * @author Shuo Li
 * vertical controller in module mode
 */

#include "modules/vertical_loop_control_module/vertical_loop_control_module.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"



struct vertical_controler_status vcs;
void guidance_v_module_init(void)
{

		vcs.k_p_z = K_P_Z;
		vcs.k_p_v_z = K_P_V_Z;
		vcs.take_off_altitude = TAKE_OFF_ALTITUDE;
		
}

void guidance_v_module_read_rc(void)
{

}
void guidance_v_module_enter(void)
{
    //printf("module_enter is called!\n");
}

void guidance_v_module_run(bool in_flight)
{
				stabilization_cmd[COMMAND_THRUST] = 5300; //5200
}



