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
 * @file "modules/state_autonomous_race/state_autonomous_race.c"
 * @author Shuo Li
 * The module is used to store all states in the competition
 */

#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "firmwares/rotorcraft/autopilot.h"
#include <stdio.h>

void state_autonomous_race_init();



struct state_autonomous_race states_race;

void state_autonomous_race_init() {
    states_race.gate_counter = 0;
}

void display_states()
{
    if (autopilot_mode != AP_MODE_MODULE)
        return;
    printf("gate_counter is %d \n",states_race.gate_counter);
    printf("\n");
    printf("\n");
    printf("\n");

}