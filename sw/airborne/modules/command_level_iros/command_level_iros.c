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

#include "modules/command_level_iros/command_level_iros.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"


#define TEMP_HOVER                   1
#define TEMP_GO_STRAIGHT             2
#define TEMP_CHAGE_HEADING_HOVER     3
#define TEMP_CIRCLE                  4

int temp_primive_chosen;

void command_run() {
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }

    temp_primive_chosen = TEMP_HOVER;

    switch (temp_primive_chosen){
        case TEMP_HOVER: hover();
            break;
        case TEMP_GO_STRAIGHT: go_straight(0.5);
            break;
        case TEMP_CHAGE_HEADING_HOVER: change_heading_hover(90.0/180.0*3.14,5);
            break;
        case TEMP_CIRCLE: circle(2,10);
            break;
        default: hover();
    }

}


