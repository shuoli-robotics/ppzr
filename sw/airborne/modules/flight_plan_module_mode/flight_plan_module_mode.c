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
 * @file "modules/flight_plan_module_mode/flight_plan_module_mode.c"
 * @author Shuo Li
 * 
 */

#include "modules/flight_plan_module_mode/flight_plan_module_mode.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/time/time.h"
#include "modules/velocity_control_indi/velocity_control_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"

uint8_t previous_mode;
uint8_t current_mode;

void flight_plan_run()
{
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        time.autopilot_counter = 0;
        time.autopilot_time = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
    }
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }

    if (time.autopilot_time < 5)
    {
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
    }

    else if (time.autopilot_time < 10)
    {
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
    }
    else
    {
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
    }


    previous_mode = current_mode;
}



