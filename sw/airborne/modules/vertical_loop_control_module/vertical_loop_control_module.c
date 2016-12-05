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
#include "modules/replay_commands/replay_commands.h"
void guidance_v_module_init(void)
{

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
    //printf("Vertical control module is called!\n");
    if (replay == 1)
    {
        //printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAA %d\n",cmd_thrust);
        stabilization_cmd[COMMAND_THRUST] = cmd_thrust ;
    }
    else
    {
        stabilization_cmd[COMMAND_THRUST] = 5200; //5200
    }

};



