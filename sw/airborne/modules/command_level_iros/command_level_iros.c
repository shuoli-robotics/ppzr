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
 * @file "modules/command_level_iros/command_level_iros.c"
 * @author Shuo Li
 * This module is the highest level
 */

#include <stdio.h>
#include "modules/command_level_iros/command_level_iros.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"

uint8_t previous_mode;
uint8_t current_mode;


void command_init(){
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
}

void command_run() {
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        counter_autopilot_mode = 0;
        time_autopilot_mode = 0;
        primitive_in_use = NO_PRIMITIVE;
    }
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }
    if (time_autopilot_mode<3)
        hover();
    else if (time_autopilot_mode<6)
        go_left_right(-1);
    else if (time_autopilot_mode<9)
        hover();
    else if (time_autopilot_mode<12)
        go_up_down(-1.0);
        else if(time_autopilot_mode<15)
        go_straight(-1);
    else
        hover();
    previous_mode = current_mode;
}


